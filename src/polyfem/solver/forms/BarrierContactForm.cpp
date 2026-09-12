#include "BarrierContactForm.hpp"

#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/MaybeParallelFor.hpp>
#include <polyfem/utils/Timer.hpp>
#include <polyfem/utils/Types.hpp>

#include <ipc/barrier/adaptive_stiffness.hpp>
#include <ipc/candidates/edge_edge.hpp>
#include <ipc/candidates/edge_vertex.hpp>
#include <ipc/candidates/face_vertex.hpp>
#include <ipc/candidates/vertex_vertex.hpp>
#include <ipc/barrier/barrier.hpp>
#include <ipc/utils/world_bbox_diagonal_length.hpp>

#include <memory>

#include <algorithm>
#include <vector>
#include <cassert>
#include <cmath>
#include <exception>

namespace polyfem::solver
{
	class BarrierContactForm::CoefficientEventScope
	{
	public:
		CoefficientEventScope(BarrierContactForm &form, const Eigen::VectorXd &x, const char *operation)
			: form_(form), x_(x), exceptions_(std::uncaught_exceptions())
		{
			if (!form_.coefficient_observer_ || !form_.uses_semi_implicit_stiffness())
				return;
			active_ = true;
			outer_ = form_.coefficient_event_depth_++ == 0;
			if (!outer_)
				return;
			try
			{
				event_ = {{"event_id", ++form_.coefficient_event_id_}, {"operation", operation}, {"coordinates", std::vector<double>(x.data(), x.data() + x.size())}, {"before", measurement()}};
			}
			catch (const std::exception &e)
			{
				logger().warn("Coefficient event preparation failed: {}", e.what());
			}
		}

		~CoefficientEventScope() noexcept
		{
			if (!active_)
				return;
			--form_.coefficient_event_depth_;
			if (!outer_)
				return;
			try
			{
				event_["operation_threw"] = std::uncaught_exceptions() > exceptions_;
				event_["after"] = measurement();
				const auto &before = event_["before"]["objective"];
				const auto &after = event_["after"]["objective"];
				if (before.is_number() && after.is_number())
				{
					const double delta = after.get<double>() - before.get<double>();
					event_["objective_change_at_fixed_coordinates"] = std::isfinite(delta) ? json(delta) : json(nullptr);
				}
				else
				{
					event_["objective_change_at_fixed_coordinates"] = nullptr;
					event_["unavailable_reason"] = "Before/after objective unavailable; initial snapshot is not an established prior model";
				}
				form_.coefficient_observer_(event_);
			}
			catch (const std::exception &e)
			{
				logger().warn("Coefficient event observation failed: {}", e.what());
			}
			catch (...)
			{
				logger().warn("Coefficient event observation failed with unknown exception");
			}
		}

	private:
		json measurement() const
		{
			json result = {{"state", form_.diagnostic_state()}, {"objective", nullptr}, {"gradient_objective", nullptr}, {"weight", form_.weight()}};
			if (form_.kappa_surface_.size() == 0)
			{
				result["unavailable_reason"] = "No prior coefficient snapshot";
				return result;
			}
			const auto snapshot = form_.diagnostic_snapshot(x_);
			const double energy = snapshot.value(x_);
			Eigen::VectorXd gradient;
			snapshot.first_derivative(x_, gradient);
			if (gradient.allFinite())
				result["gradient_objective"] = std::vector<double>(gradient.data(), gradient.data() + gradient.size());
			if (std::isfinite(energy))
				result["objective"] = energy;
			else
				result["unavailable_reason"] = "Nonfinite objective";
			result["evaluated_state"] = snapshot.diagnostic_state();
			return result;
		}
		BarrierContactForm &form_;
		const Eigen::VectorXd &x_;
		int exceptions_;
		bool active_ = false, outer_ = false;
		json event_;
	};

	namespace
	{
		/// Clamped-log barrier with a C1 linear continuation below a
		/// squared-distance floor: the force saturates at b'(floor) instead
		/// of blowing up as d -> 0. A single pair driven to the
		/// floating-point floor of the coordinates (scissored thin surfaces,
		/// impact at first contact) otherwise injects ~1/d^4 terms into the
		/// gradient/Hessian and poisons the Newton direction for the whole
		/// mesh; CCD already guarantees non-penetration, so a bounded
		/// push-back is safe.
		class FlooredClampedLogBarrier : public ipc::ClampedLogBarrier
		{
		public:
			FlooredClampedLogBarrier(const double floor_sq) : floor_sq_(floor_sq) {}

			double operator()(const double d, const double dhat) const override
			{
				if (d >= floor_sq_)
					return ipc::ClampedLogBarrier::operator()(d, dhat);
				return ipc::ClampedLogBarrier::operator()(floor_sq_, dhat)
					   + ipc::ClampedLogBarrier::first_derivative(floor_sq_, dhat)
							 * (d - floor_sq_);
			}

			double first_derivative(const double d, const double dhat) const override
			{
				return ipc::ClampedLogBarrier::first_derivative(
					std::max(d, floor_sq_), dhat);
			}

			double second_derivative(const double d, const double dhat) const override
			{
				return d < floor_sq_
						   ? 0.0
						   : ipc::ClampedLogBarrier::second_derivative(d, dhat);
			}

		private:
			double floor_sq_;
		};

		ipc::BarrierPotential make_barrier_potential(
			const double dhat,
			const bool use_physical_barrier,
			const BarrierStiffnessMode stiffness_mode,
			const json &semi_implicit_opts)
		{
			if (stiffness_mode == BarrierStiffnessMode::SemiImplicit)
			{
				// Disabled by default: saturating the barrier removes the
				// direction poisoning but lets a tension-driven pair collapse
				// freely to the fp floor (measured: 1e-11 -> 6.6e-15 on the
				// draping wall) where CCD becomes the deadlock instead.
				// This experimental force saturation is not hard contact.
				const double gap_floor =
					semi_implicit_opts.is_object()
						? semi_implicit_opts.value("gap_floor", 0.0)
						: 0.0;
				if (gap_floor > 0)
					return ipc::BarrierPotential(
						std::make_shared<FlooredClampedLogBarrier>(
							std::pow(gap_floor * dhat, 2)),
						dhat, 1.0, use_physical_barrier);
			}
			return ipc::BarrierPotential(dhat, 1.0, use_physical_barrier);
		}
	} // namespace

	BarrierContactForm::BarrierContactForm(const ipc::CollisionMesh &collision_mesh,
										   const double dhat,
										   const double avg_mass,
										   const bool use_area_weighting,
										   const bool use_improved_max_operator,
										   const bool use_physical_barrier,
										   const bool use_adaptive_barrier_stiffness,
										   const bool is_time_dependent,
										   const bool enable_shape_derivatives,
										   const ipc::BroadPhaseMethod broad_phase_method,
										   const double ccd_tolerance,
										   const int ccd_max_iterations,
										   const BarrierStiffnessMode stiffness_mode,
										   const json &semi_implicit_opts,
										   const Eigen::VectorXd &lumped_vertex_masses) : ContactForm(collision_mesh, dhat, avg_mass, use_adaptive_barrier_stiffness, is_time_dependent, enable_shape_derivatives, broad_phase_method, ccd_tolerance, ccd_max_iterations), barrier_potential_(make_barrier_potential(dhat, use_physical_barrier, stiffness_mode, semi_implicit_opts)), stiffness_mode_(stiffness_mode), lumped_vertex_masses_(lumped_vertex_masses)
	{
		// collision_set_.set_use_convergent_formulation(use_convergent_formulation);
		collision_set_.set_use_area_weighting(use_area_weighting);
		collision_set_.set_use_improved_max_approximator(use_improved_max_operator);
		collision_set_.set_enable_shape_derivatives(enable_shape_derivatives);

		if (uses_semi_implicit_stiffness())
		{
			// IPC's to_full_vertex_id reverses surface selection only: its
			// result is a proxy ID, not necessarily a system Hessian node ID.
			// Read S*T once in O(nnz + rows + cols), without a dense map.
			// Exact unit selectors extract their Hessian blocks directly;
			// every other row (interpolated, scaled, empty) keeps its
			// (node, weight) parents for the RB-03 condensed stiffness.
			const auto &map = collision_mesh_.displacement_map();
			stiffness_node_ids_ = Eigen::VectorXi::Constant(map.rows(), -1);
			interpolation_parents_.assign(map.rows(), {});
			Eigen::VectorXi counts = Eigen::VectorXi::Zero(map.rows());
			for (int col = 0; col < map.outerSize(); ++col)
				for (Eigen::SparseMatrix<double>::InnerIterator it(map, col); it; ++it)
					if (it.value() != 0.)
					{
						++counts[it.row()];
						stiffness_node_ids_[it.row()] = it.value() == 1. ? it.col() : -1;
						interpolation_parents_[it.row()].emplace_back(int(it.col()), it.value());
					}
			for (int row = 0; row < counts.size(); ++row)
			{
				if (counts[row] != 1)
					stiffness_node_ids_[row] = -1;
				if (stiffness_node_ids_[row] >= 0)
				{
					interpolation_parents_[row].clear();
					interpolation_parents_[row].shrink_to_fit();
				}
			}

			if (enable_shape_derivatives)
				log_and_throw_error("Semi-implicit barrier stiffness does not support shape derivatives!");
			if (use_physical_barrier)
				log_and_throw_error("Semi-implicit barrier stiffness does not support the physical barrier; set use_physical_barrier=false!");

			if (semi_implicit_opts.is_object())
			{
				refresh_interval_ = semi_implicit_opts.value("refresh_interval", refresh_interval_);
				trim_lower_ = semi_implicit_opts.value("trim_lower", trim_lower_);
				trim_upper_ = semi_implicit_opts.value("trim_upper", trim_upper_);
				trim_factor_ = semi_implicit_opts.value("trim_factor", trim_factor_);
				trim_min_ = semi_implicit_opts.value("trim_min", trim_min_);
				trim_max_ = semi_implicit_opts.value("trim_max", trim_max_);
				kappa_min_ = semi_implicit_opts.value("kappa_min", kappa_min_);
				kappa_spread_ = semi_implicit_opts.value("kappa_spread", kappa_spread_);
				conditioning_cap_ = semi_implicit_opts.value("conditioning_cap", conditioning_cap_);
				controller_interval_ = semi_implicit_opts.value("controller_interval", controller_interval_);
				if (semi_implicit_opts.value("constraint_floor", 0.0) != 0.0)
					logger().warn("solver.contact.semi_implicit.constraint_floor has been retired and is ignored; barrier deletion and floor projection are no longer supported.");
				trial_displacement_cap_ = semi_implicit_opts.value("trial_displacement_cap", trial_displacement_cap_);
				force_continuation_ = semi_implicit_opts.value("force_continuation", force_continuation_);
				continuation_max_ratio_ = semi_implicit_opts.value("continuation_max_ratio", continuation_max_ratio_);
				const std::string identity = semi_implicit_opts.value("coefficient_identity", std::string("parent"));
				if (identity == "parent")
					parent_keyed_ = true;
				else if (identity == "stencil")
					parent_keyed_ = false;
				else
					log_and_throw_error("Semi-implicit barrier stiffness: coefficient_identity must be \"parent\" or \"stencil\" (got \"{}\")", identity);
			}
			if (continuation_max_ratio_ != 0.0 && !(continuation_max_ratio_ > 1.0))
				log_and_throw_error("Semi-implicit barrier stiffness: continuation_max_ratio must be 0 (pure continuation) or > 1!");

			refresh_interval_ = std::max(refresh_interval_, 0);
			kappa_min_ = std::max(kappa_min_, 0.0);
			if (!(trim_lower_ < trim_upper_))
				log_and_throw_error("Semi-implicit barrier stiffness requires trim_lower < trim_upper!");
		}
	}

	double BarrierContactForm::collapse_severity(const double avg_d2, const double min_d2) const
	{
		// A single contact collapsing far below the band is as dangerous as
		// the average collapsing: take the worse of the average gap and the
		// minimum gap relaxed by min_gap_slack (the minimum may sit well
		// below the average in healthy states).
		constexpr double min_gap_slack = 1e2;
		double severity = std::numeric_limits<double>::infinity();
		if (std::isfinite(avg_d2))
			severity = avg_d2;
		if (std::isfinite(min_d2))
			severity = std::min(severity, min_d2 * min_gap_slack);
		return severity;
	}

	double BarrierContactForm::collapse_bump_factor(const double avg_d2) const
	{
		// Force balance gives kappa_eff ~ F / gap, so the trim needed to lift
		// the average gap back to the band scales with sqrt(band / avg_d2).
		return std::min(256.0, std::max(trim_factor_, std::sqrt(trim_lower_ * dhat_ * dhat_ / avg_d2)));
	}

	void BarrierContactForm::bump_trim(const double factor)
	{
		const double new_trim = std::clamp(barrier_stiffness_ * factor, trim_min_, trim_max_);
		if (new_trim != barrier_stiffness_)
		{
			logger().debug("Barrier stiffness trim: {:g} -> {:g}", barrier_stiffness_, new_trim);
			barrier_stiffness_ = new_trim;
			iters_since_trim_ = 0;
		}
	}

	bool BarrierContactForm::retune_on_stall(const Eigen::VectorXd &x, const double factor)
	{
		if (!uses_semi_implicit_stiffness())
			return false;
		CoefficientEventScope event(*this, x, "stall_retune");
		const double trim_before = barrier_stiffness_;
		// A stall with the gap below the band (average OR a single collapsed
		// contact) means the barrier is too soft (the solver is crawling
		// against CCD); otherwise the barrier is likely dominating the
		// elasticity: recalibrate it against the driving forces, falling
		// back to a blind softening when the balance is degenerate.
		refresh_semi_implicit_stiffness(x, /*run_trim_controller=*/false);
		if (collision_set_.empty())
			return false; // stall unrelated to contact; nothing to retune

		const double avg_d2 = collision_set_.compute_avg_distance(
			collision_mesh_, kappa_surface_, dhat_);
		const double min_d2 = collision_set_.compute_minimum_distance(
			collision_mesh_, kappa_surface_);
		const double severity = collapse_severity(avg_d2, min_d2);

		if (std::isfinite(severity) && severity < trim_lower_ * dhat_ * dhat_)
			bump_trim(std::max(factor, collapse_bump_factor(severity)));
		else if (!calibrate_trim(x))
		{
			// Only soften blindly when the gap is pinned above the band
			// (barrier clearly dominating); a mid-band stall with no
			// balance signal has no direction -- restart on the fresh
			// snapshot with the trim untouched.
			if (std::isfinite(avg_d2) && avg_d2 > trim_upper_ * dhat_ * dhat_)
				bump_trim(1.0 / factor);
		}
		// The refresh re-evaluated every active coefficient at x; compare the
		// memoized values against the previous snapshot so a stall at an
		// unchanged iterate with an unmoved trim is reported as "no change".
		return barrier_stiffness_ != trim_before || kappa_cache_ != prev_kappa_cache_;
	}

	void BarrierContactForm::refresh_semi_implicit_stiffness(const Eigen::VectorXd &x, const bool run_trim_controller, const bool published_endpoint)
	{
		if (!uses_semi_implicit_stiffness())
			return;
		CoefficientEventScope event(*this, x, "refresh");
		if (!system_hessian_provider_)
			log_and_throw_error("Semi-implicit barrier stiffness requires a system Hessian provider!");

		kappa_surface_ = compute_displaced_surface(x);
		system_hessian_provider_(x, kappa_hessian_);
		kappa_hessian_max_ = 0.0;
		for (int k = 0; k < kappa_hessian_.outerSize(); k++)
			for (StiffnessMatrix::InnerIterator it(kappa_hessian_, k); it; ++it)
				kappa_hessian_max_ = std::max(kappa_hessian_max_, std::abs(it.value()));
		// RB-20 force continuation: at a PUBLISHED endpoint every active
		// stencil that has a coefficient keeps the value that ACTED there (the
		// resolved effective scale, floor/cap/kappa_min included), so the
		// barrier force at these coordinates is unchanged by the refresh for
		// every persisting contact. Those values are re-seeded at every
		// following refresh until the next endpoint; everything else (stencils
		// born during a solve, trial-state memo entries) is estimated from
		// the fresh Hessian as before -- a mid-solve birth/stall refresh
		// therefore still re-estimates contacts that no endpoint has vetted.
		if (force_continuation_ && published_endpoint)
		{
			endpoint_kappa_.clear();
			for (size_t i = 0; i < collision_set_.size(); i++)
			{
				if (collision_set_.is_plane_vertex(i))
					continue;
				for (const auto &[key, w] : coefficient_keys(collision_set_, i))
				{
					const auto cached = kappa_cache_.find(key);
					if (cached == kappa_cache_.end())
						continue;
					const double k = resolve_stiffness(cached->second, is_continued(key));
					if (k > 0 && std::isfinite(k))
						endpoint_kappa_.emplace(key, k);
				}
			}
		}
		else if (!force_continuation_)
			endpoint_kappa_.clear();
		// The previous snapshot's values back stencils whose fresh curvature
		// is invalid (RB-18 F2); only a non-empty snapshot is worth keeping.
		if (!kappa_cache_.empty())
			prev_kappa_cache_.swap(kappa_cache_);
		kappa_cache_.clear();
		continued_keys_.clear();
		for (const auto &[key, k] : endpoint_kappa_)
		{
			kappa_cache_.emplace(key, k);
			continued_keys_.insert(key);
		}
		kappa_continued_count_ = 0;
		kappa_fresh_count_ = 0;
		iters_since_refresh_ = 0;
		const bool pull_toward_fresh = published_endpoint && continuation_max_ratio_ > 1.0;

		// First pass uncapped to freeze the cap at kappa_spread * median of
		// the batch (guards against exploded Hessian blocks of crushed
		// elements) and the floor at median / kappa_spread (RB-18 F1: a
		// contact can never be more than kappa_spread below its batch, so a
		// nonpositive local curvature cannot delete its barrier); both are
		// part of the snapshot for determinism. The median is taken over the
		// POSITIVE finite values only: a zero median previously zeroed every
		// contact in the batch (RB-02).
		kappa_cap_ = std::numeric_limits<double>::infinity();
		kappa_floor_ = 0.0;
		kappa_median_ = 0.0;
		kappa_fallback_count_ = 0;
		kappa_abs_fallback_count_ = 0;
		kappa_global_fallback_count_ = 0;
		kappa_interpolated_count_ = 0;
		kappa_direction_fallback_count_ = 0;
		const bool first_contact = !kappa_snapshot_had_contacts_ && !collision_set_.empty();
		kappa_snapshot_had_contacts_ = !collision_set_.empty();
		batch_first_pass_ = true;
		pull_toward_fresh_ = pull_toward_fresh;
		try
		{
			assign_collision_stiffness(collision_set_);
		}
		catch (...)
		{
			batch_first_pass_ = false;
			pull_toward_fresh_ = false;
			throw;
		}
		batch_first_pass_ = false;
		pull_toward_fresh_ = false;
		for (size_t i = 0; i < collision_set_.size(); i++)
		{
			if (collision_set_.is_plane_vertex(i))
				continue;
			bool all_continued = true;
			for (const auto &[key, w] : coefficient_keys(collision_set_, i))
				all_continued = all_continued && is_continued(key);
			if (all_continued)
				++kappa_continued_count_;
		}
		if (!collision_set_.empty())
		{
			// RB-20 D4: the batch statistics (median, floor, cap) describe
			// the FRESH estimates; continued values already acted and are
			// neither clamped by them nor allowed to distort them. When the
			// batch has no fresh value the continued ones stand in, so an F2/F7
			// sentinel of a new stencil still has a reference. The batch is
			// the memo after the first pass: exactly the continued seeds plus
			// every coefficient key (parent or stencil) assigned at this x.
			std::vector<double> kappas, continued_kappas;
			kappas.reserve(kappa_cache_.size());
			for (const auto &[key, k] : kappa_cache_)
			{
				if (!(k > 0 && std::isfinite(k)))
					continue;
				(is_continued(key) ? continued_kappas : kappas).push_back(k);
			}
			if (kappas.empty())
				kappas.swap(continued_kappas);
			if (!kappas.empty())
			{
				std::nth_element(kappas.begin(), kappas.begin() + kappas.size() / 2, kappas.end());
				kappa_median_ = kappas[kappas.size() / 2];
			}
			else
			{
				logger().warn(
					"Semi-implicit barrier stiffness: no contact in the refresh batch of {} has positive local curvature; no batch cap/floor available",
					collision_set_.size());
			}

			if (kappa_median_ > 0 && std::isfinite(kappa_spread_) && kappa_spread_ > 0)
			{
				const double cap = kappa_spread_ * kappa_median_;
				if (std::isfinite(cap))
					kappa_cap_ = cap;
				else
					logger().warn(
						"Semi-implicit barrier stiffness: kappa_spread * median overflows ({} * {:g}); batch cap disabled",
						kappa_spread_, kappa_median_);
				kappa_floor_ = kappa_median_ / kappa_spread_;
			}
			// Re-resolve the already-assigned scales with the frozen batch
			// statistics (this also replaces the invalid-curvature sentinels);
			// every key is memoized now, so this is a pure lookup pass.
			assign_collision_stiffness(collision_set_);
			if (kappa_continued_count_ > 0)
				logger().debug(
					"Semi-implicit barrier stiffness: {} of {} contacts continued from the refresh point, {} estimated fresh (continuation_max_ratio={:g})",
					kappa_continued_count_, collision_set_.size(), kappa_fresh_count_, continuation_max_ratio_);
			if (kappa_fallback_count_ + kappa_abs_fallback_count_ + kappa_global_fallback_count_ > 0)
				logger().debug(
					"Semi-implicit barrier stiffness: {} of {} contacts had invalid curvature and no previous value: {} used |w^T H w|, {} used max|H|/dhat^2, {} resolved to floor={:g} / cap={:g}",
					kappa_fallback_count_ + kappa_abs_fallback_count_ + kappa_global_fallback_count_,
					collision_set_.size(), kappa_abs_fallback_count_, kappa_global_fallback_count_,
					kappa_fallback_count_, kappa_floor_, kappa_cap_);
			if (kappa_interpolated_count_ + kappa_direction_fallback_count_ > 0)
				logger().debug(
					"Semi-implicit barrier stiffness: {} of {} contacts have interpolated map rows: {} condensed the parent block, {} used the gap-normalized direction fallback",
					kappa_interpolated_count_ + kappa_direction_fallback_count_, collision_set_.size(),
					kappa_interpolated_count_, kappa_direction_fallback_count_);
		}

		// Trim controller, one step per refresh: below the gap band the
		// barrier is too soft (proportional upward bump); otherwise calibrate
		// the trim by gradient balance against the current driving forces.
		// When the balance is degenerate (unloaded contact, no gradient
		// provider) fall back to a conditioning cap on the effective
		// stiffness plus the band's downward step.
		if (run_trim_controller && !collision_set_.empty())
		{
			const double avg_d2 = collision_set_.compute_avg_distance(
				collision_mesh_, kappa_surface_, dhat_);
			const double min_d2 = collision_set_.compute_minimum_distance(
				collision_mesh_, kappa_surface_);
			const double severity = collapse_severity(avg_d2, min_d2);
			const double dhat_sq = dhat_ * dhat_;

			if (std::isfinite(severity) && severity < trim_lower_ * dhat_sq)
			{
				bump_trim(collapse_bump_factor(severity));
			}
			else if (!calibrate_trim(x))
			{
				// No force-balance signal. On a first-contact event the
				// fresh kappas have never been vetted, so cap the effective
				// stiffness by conditioning; on later refreshes the
				// persistent trim carries the accumulated load history and
				// must NOT be reset (the emergency bumps and the calibration
				// are the only things allowed to move it, plus the band's
				// downward step when the gap is pinned above the band).
				// (Never cap during an active collapse -- a collapsed first
				// contact needs strength, not conditioning.)
				if (first_contact && kappa_median_ > 0 && kappa_hessian_max_ > 0
					&& !(std::isfinite(severity) && severity < trim_lower_ * dhat_sq))
				{
					// RB-18 F3: kappa carries force/length^3 (divided by dhat^2
					// at assignment) while |H| is force/length, so the ratio
					// needs a dhat^2 to be dimensionless: trim * kappa * weight
					// * dhat^2 is the barrier's interface stiffness at gaps
					// ~dhat, which is what the cap compares to max|H|.
					const double cap = std::clamp(
						conditioning_cap_ * kappa_hessian_max_
							/ (weight_ * kappa_median_ * dhat_ * dhat_),
						trim_min_, trim_max_);
					if (barrier_stiffness_ > cap)
					{
						logger().debug(
							"Conditioning cap on first contact: trim {:g} -> {:g}",
							barrier_stiffness_, cap);
						barrier_stiffness_ = cap;
						iters_since_trim_ = 0;
					}
				}
				if (std::isfinite(avg_d2) && avg_d2 > trim_upper_ * dhat_ * dhat_)
					bump_trim(1.0 / trim_factor_);
			}
		}

		++diagnostic_refresh_id_;

		// Re-anchor the in-solve emergency climbing budget.
		trim_solve_anchor_ = barrier_stiffness_;

		if (!collision_set_.empty())
		{
			double min_kappa = std::numeric_limits<double>::infinity();
			double max_kappa = 0, mean_kappa = 0;
			for (size_t i = 0; i < collision_set_.size(); i++)
			{
				const double k = collision_set_[i].stiffness_scale;
				min_kappa = std::min(min_kappa, k);
				max_kappa = std::max(max_kappa, k);
				mean_kappa += k;
			}
			mean_kappa /= collision_set_.size();
			logger().debug(
				"Refreshed semi-implicit barrier stiffness over {} contacts: min={:g} mean={:g} max={:g} (trim={:g})",
				collision_set_.size(), min_kappa, mean_kappa, max_kappa, barrier_stiffness_);
		}
	}

	std::array<long, 5> BarrierContactForm::stencil_key(const ipc::NormalCollisions &collision_set, const size_t i) const
	{
		const auto vids = collision_set[i].vertex_ids(collision_mesh_.edges(), collision_mesh_.faces());
		long type_tag = 3; // face-vertex
		if (collision_set.is_vertex_vertex(i))
			type_tag = 0;
		else if (collision_set.is_edge_vertex(i))
			type_tag = 1;
		else if (collision_set.is_edge_edge(i))
			type_tag = 2;
		// Doubly braced: std::array wraps a C array, and GCC's
		// -Werror=missing-braces (on in CI) rejects the flat form clang accepts.
		return {{type_tag, long(vids[0]), long(vids[1]), long(vids[2]), long(vids[3])}};
	}

	std::vector<std::pair<std::array<long, 5>, double>> BarrierContactForm::coefficient_keys(const ipc::NormalCollisions &collision_set, const size_t i) const
	{
		std::vector<std::pair<std::array<long, 5>, double>> keys;
		if (parent_keyed_)
		{
			// RB-21: one coefficient per candidate primitive pair that built
			// this collision, weighted by its (positive) contribution. The
			// key space is disjoint from stencil keys (tag offset 10).
			// Duplicate-removal corrections carry negative weights and no
			// coefficient of their own; they only adjust the total weight.
			for (const auto &parent : collision_set[i].parents)
				if (parent.weight > 0)
					keys.push_back({{{10 + long(parent.type), long(parent.id0), long(parent.id1), -1, -1}}, parent.weight});
		}
		if (keys.empty())
			keys.push_back({stencil_key(collision_set, i), 1.0});
		return keys;
	}

	double BarrierContactForm::estimate_stiffness(const ipc::CollisionStencil &stencil, const std::array<long, 5> &key) const
	{
		const Eigen::MatrixXi &E = collision_mesh_.edges();
		const Eigen::MatrixXi &F = collision_mesh_.faces();
		const int dim = collision_mesh_.dim();
		const int n_verts = stencil.num_vertices();
		const auto vids = stencil.vertex_ids(E, F);

		// Local positions, masses, and Hessian block come from the FROZEN
		// snapshot, so the value is a deterministic function of the key
		// between refreshes (well-defined objective during the line search).
		const ipc::VectorMax12d positions = stencil.dof(kappa_surface_, E, F);

		// NOTE: Ando's m/d^2 feasibility term is intentionally NOT used: with
		// a frozen snapshot, a collapsed snapshot distance bakes an unbounded
		// stiffness into the objective, and the log barrier already
		// guarantees non-penetration. Inertia enters through the system
		// Hessian provider instead (elastic + inertia curvature of the
		// incremental potential).
		ipc::VectorMax4d local_mass = ipc::VectorMax4d::Zero(n_verts);
		// Exact selector/permutation stencils use system node IDs (RB-03
		// indexing repair). A stencil with an interpolated, scaled or empty
		// row, or two rows on the same node, has no Hessian block of its own
		// in surface coordinates: its curvature is the parent block condensed
		// onto the stencil (RB-03 interpolated stiffness, 2026-09-11).
		std::array<long, 4> node_ids;
		std::array<long, 4> surface_ids;
		bool exact_selection = true;
		for (int a = 0; a < n_verts; ++a)
		{
			surface_ids[a] = vids[a];
			node_ids[a] = stiffness_node_ids_[vids[a]];
			exact_selection = exact_selection && node_ids[a] >= 0;
			for (int b = 0; b < a; ++b)
				exact_selection = exact_selection && node_ids[a] != node_ids[b];
		}

		// kappa = avg_mass / d^2 + w^T H w [Ando 2024]; for a parent
		// candidate the direction is the closest point on the WHOLE
		// primitive (AUTO distance type), for a built stencil its subfeature.
		double kappa;
		if (exact_selection)
		{
			ipc::MatrixMax12d local_hess =
				ipc::MatrixMax12d::Zero(dim * n_verts, dim * n_verts);
			for (int a = 0; a < n_verts; a++)
			{
				const long va = node_ids[a];
				for (int b = 0; b < n_verts; b++)
				{
					const long vb = node_ids[b];
					if (dim * va + dim > kappa_hessian_.rows()
						|| dim * vb + dim > kappa_hessian_.cols())
						continue; // e.g., obstacle DOF not in the Hessian
					for (int k = 0; k < dim; k++)
						for (int l = 0; l < dim; l++)
							local_hess(dim * a + k, dim * b + l) =
								kappa_hessian_.coeff(dim * va + k, dim * vb + l);
				}
			}
			kappa = ipc::semi_implicit_stiffness(
				stencil, positions, local_mass, local_hess, dmin_);
		}
		else
			kappa = interpolated_stiffness(stencil, positions, surface_ids, n_verts);

		// Unit conversion: the Ando stiffness is an interface SPRING
		// stiffness [force/length], while the clamped-log barrier acts on
		// squared distances (units length^4), so its coefficient carries
		// [force/length^3]. Dividing by dhat^2 makes the barrier's interface
		// stiffness at gaps ~ dhat match the local elasticity, independent of
		// the dhat scale (without this the trim controller must bridge a
		// 1/dhat^2 factor and starves for small dhat).
		kappa /= dhat_ * dhat_;

		// RB-18 F4: a NaN Rayleigh quotient means the frozen Hessian block
		// itself is NaN -- the Newton system is already invalid, so fail
		// loudly instead of substituting a literal.
		if (std::isnan(kappa))
			log_and_throw_error(
				"Semi-implicit barrier stiffness: NaN local curvature for contact key (tag {}, ids {}, {}, {}, {})",
				key[0], key[1], key[2], key[3], key[4]);

		// Remove the form weight (acceleration scaling); it is reapplied by
		// ContactForm::weight(). The global barrier_stiffness_ acts as the
		// trim multiplier, so the effective coefficient is trim * kappa. The
		// weight is checked here because set_weight() can change it after
		// construction.
		if (!std::isfinite(weight_) || !(weight_ >= std::numeric_limits<double>::min()))
			log_and_throw_error(
				"Semi-implicit barrier stiffness requires a finite positive (normal) form weight (got {:g})",
				weight_);
		kappa /= weight_;

		// RB-18 F2/F4: w^T H w is nonpositive for an unprojected indefinite
		// or singular local Hessian, and can overflow to +inf for crushed
		// elements. Neither may delete the barrier (kappa = 0 leaves CCD as
		// the only protection) nor install an arbitrary literal. Keep the
		// key's previous value when it has one; otherwise leave a sentinel
		// (0 = needs the batch floor, +inf = needs the batch cap) that
		// resolve_stiffness() maps onto the frozen batch statistics.
		if (!(kappa > 0) || !std::isfinite(kappa))
		{
			const auto prev = prev_kappa_cache_.find(key);
			if (prev != prev_kappa_cache_.end() && prev->second > 0
				&& std::isfinite(prev->second))
			{
				kappa = prev->second;
			}
			else if (kappa < 0 && std::isfinite(kappa))
			{
				// RB-18 F7 (user choice B): an indefinite local block still
				// has a curvature MAGNITUDE along the normal; the barrier
				// borrows it. Heuristic, not a derivation.
				kappa = -kappa;
				++kappa_abs_fallback_count_;
			}
			else if (kappa == 0)
			{
				// RB-18 F7 (fallback E): singular along the normal, so use
				// the global Hessian scale max|H| / dhat^2 (same
				// normalization as the conditioning cap). Zero only when the
				// system Hessian is identically zero.
				kappa = kappa_hessian_max_ / (dhat_ * dhat_ * weight_);
				if (kappa > 0 && std::isfinite(kappa))
					++kappa_global_fallback_count_;
				else
				{
					kappa = 0.0;
					++kappa_fallback_count_;
				}
			}
			else
			{
				kappa = std::numeric_limits<double>::infinity();
				++kappa_fallback_count_;
			}
		}
		return kappa;
	}

	double BarrierContactForm::interpolated_stiffness(
		const ipc::CollisionStencil &stencil, const ipc::VectorMax12d &positions,
		const std::array<long, 4> &vids, const int n_verts) const
	{
		// Definition (RB-03 decision, 2026-09-11): with the map rows B of the
		// stencil's surface vertices over their parent nodes P, the local
		// stiffness is K = (B H_PP^-1 B^T)^-1 -- the frozen parent block
		// statically condensed onto the stencil, i.e. the stiffness felt by a
		// rigid stencil displacement along the contact direction when every
		// DOF outside P is fixed and the parents settle to minimum energy.
		// For exact selectors this IS the direct block extraction (no freedom
		// to settle), so the two paths agree on selector stencils. Parents
		// without a nonzero diagonal block (obstacle / prescribed proxies) or
		// outside the Hessian are fixed and carry no motion; a row left with
		// no movable parent is dropped and contributes zero, as its zero block
		// does in the selector contract. Fallback (i'): if H_PP is not SPD or
		// the kept rows are dependent (duplicate proxies, two rows on one
		// node), use the gap-normalized force direction
		//   |w_K|^4 (w^T B H_PP B^T w) / (w^T B B^T w)^2,
		// which is the same energy read along u = B^T w scaled to produce the
		// stencil's own motion, and equals w^T H w for selectors.
		const int dim = collision_mesh_.dim();
		const int n = dim * n_verts;
		const auto fixed_node = [&](const long j) {
			if (dim * j + dim > kappa_hessian_.rows() || dim * j + dim > kappa_hessian_.cols())
				return true;
			for (int k = 0; k < dim; ++k)
				for (int l = 0; l < dim; ++l)
					if (kappa_hessian_.coeff(dim * j + k, dim * j + l) != 0.)
						return false;
			return true;
		};
		std::vector<long> parents;
		std::vector<std::vector<std::pair<int, double>>> rows(n_verts);
		std::array<int, 4> slot = {-1, -1, -1, -1};
		int n_kept = 0;
		for (int a = 0; a < n_verts; ++a)
		{
			const auto add = [&](const long node, const double weight) {
				if (weight == 0. || fixed_node(node))
					return;
				auto it = std::find(parents.begin(), parents.end(), node);
				if (it == parents.end())
					it = parents.insert(parents.end(), node);
				rows[a].emplace_back(int(it - parents.begin()), weight);
			};
			const long node = stiffness_node_ids_[vids[a]];
			if (node >= 0)
				add(node, 1.);
			else
				for (const auto &[j, weight] : interpolation_parents_[vids[a]])
					add(j, weight);
			if (!rows[a].empty())
				slot[a] = n_kept++;
		}
		if (n_kept == 0)
			return 0.; // nothing movable: sentinel for the RB-18 F2/F7 chain

		const int np = dim * int(parents.size()), nk = dim * n_kept;
		Eigen::MatrixXd hpp(np, np);
		for (int i = 0; i < int(parents.size()); ++i)
			for (int j = 0; j < int(parents.size()); ++j)
				for (int k = 0; k < dim; ++k)
					for (int l = 0; l < dim; ++l)
						hpp(dim * i + k, dim * j + l) =
							kappa_hessian_.coeff(dim * parents[i] + k, dim * parents[j] + l);
		Eigen::MatrixXd b = Eigen::MatrixXd::Zero(nk, np);
		for (int a = 0; a < n_verts; ++a)
			for (const auto &[p, weight] : rows[a])
				for (int k = 0; k < dim; ++k)
					b(dim * slot[a] + k, dim * p + k) += weight;

		// Quadratic forms along the toolkit's own contact direction (zero
		// masses: the mass term is not used by this form).
		const ipc::VectorMax4d zero_mass = ipc::VectorMax4d::Zero(n_verts);
		const auto quad = [&](const Eigen::MatrixXd &k) {
			ipc::MatrixMax12d local = ipc::MatrixMax12d::Zero(n, n);
			for (int a = 0; a < n_verts; ++a)
				for (int c = 0; c < n_verts; ++c)
					if (slot[a] >= 0 && slot[c] >= 0)
						local.block(dim * a, dim * c, dim, dim) =
							k.block(dim * slot[a], dim * slot[c], dim, dim);
			return ipc::semi_implicit_stiffness(stencil, positions, zero_mass, local, dmin_);
		};

		Eigen::LLT<Eigen::MatrixXd> llt(hpp);
		if (llt.info() == Eigen::Success)
		{
			Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(b);
			qr.setThreshold(1e-10);
			if (qr.rank() == nk)
			{
				const Eigen::MatrixXd compliance = b * llt.solve(b.transpose());
				Eigen::LLT<Eigen::MatrixXd> llt_compliance(compliance);
				if (llt_compliance.info() == Eigen::Success)
				{
					const Eigen::MatrixXd k = llt_compliance.solve(Eigen::MatrixXd::Identity(nk, nk));
					if (k.allFinite())
					{
						++kappa_interpolated_count_;
						return quad(k);
					}
				}
			}
		}

		++kappa_direction_fallback_count_;
		const double q1 = quad(b * hpp * b.transpose());
		const double q2 = quad(b * b.transpose());
		const double wk2 = quad(Eigen::MatrixXd::Identity(nk, nk));
		if (!(q2 > 0) || !std::isfinite(q2))
			return 0.;
		return wk2 * wk2 * q1 / (q2 * q2);
	}

	double BarrierContactForm::memoized_stiffness(const ipc::NormalCollisions &collision_set, const size_t i, const std::array<long, 5> &key) const
	{
		const auto cached = kappa_cache_.find(key);
		const bool continued = cached != kappa_cache_.end() && is_continued(key);
		// RB-20 D3: with a max ratio, a continued coefficient is pulled
		// toward the fresh estimate but by at most that factor per published
		// endpoint. Done once, during the refresh's first pass; the memo then
		// holds the clamped value for the rest of the snapshot.
		const bool pull = continued && batch_first_pass_ && pull_toward_fresh_;
		if (cached != kappa_cache_.end() && !pull)
			return resolve_stiffness(cached->second, continued);

		if (!continued)
			++kappa_fresh_count_;

		double kappa;
		if (key[0] >= 10)
		{
			// RB-21 parent candidate: rebuild the candidate (AUTO distance
			// type, closest point on the whole primitive).
			const long id0 = key[1], id1 = key[2];
			switch (ipc::ParentContribution::Type(key[0] - 10))
			{
			case ipc::ParentContribution::Type::VertexVertex:
				kappa = estimate_stiffness(ipc::VertexVertexCandidate(id0, id1), key);
				break;
			case ipc::ParentContribution::Type::EdgeVertex:
				kappa = estimate_stiffness(ipc::EdgeVertexCandidate(id0, id1), key);
				break;
			case ipc::ParentContribution::Type::EdgeEdge:
				kappa = estimate_stiffness(ipc::EdgeEdgeCandidate(id0, id1), key);
				break;
			case ipc::ParentContribution::Type::FaceVertex:
			default:
				kappa = estimate_stiffness(ipc::FaceVertexCandidate(id0, id1), key);
				break;
			}
		}
		else
			kappa = estimate_stiffness(collision_set[i], key);

		if (!batch_first_pass_)
			logger().trace(
				"Semi-implicit barrier stiffness: fresh coefficient mid-solve (tag {}, ids {}, {}, {}, {}) kappa={:g} (batch median {:g})",
				key[0], key[1], key[2], key[3], key[4], kappa, kappa_median_);
		if (pull)
		{
			// An invalid fresh estimate (F2/F7 sentinel) cannot pull; the
			// continued value stands.
			const double base = cached->second;
			kappa = (kappa > 0 && std::isfinite(kappa))
						? std::clamp(kappa, base / continuation_max_ratio_, base * continuation_max_ratio_)
						: base;
			kappa_cache_[key] = kappa;
			endpoint_kappa_[key] = kappa;
		}
		else
			kappa_cache_.emplace(key, kappa);
		return resolve_stiffness(kappa, continued);
	}

	void BarrierContactForm::assign_collision_stiffness(ipc::NormalCollisions &collision_set) const
	{
		if (!uses_semi_implicit_stiffness() || kappa_surface_.size() == 0)
			return;

		for (size_t i = 0; i < collision_set.size(); i++)
		{
			if (collision_set.is_plane_vertex(i))
				continue; // unused by polyfem; keep the default scale

			// The collision's scale is the contribution-weighted mean of its
			// coefficient keys (RB-21: its positive parents; otherwise the
			// stencil itself), so that weight * scale * b(d) is the sum of
			// each parent's own potential. With every coefficient equal to
			// one this is exactly the unkeyed potential.
			double numerator = 0.0, denominator = 0.0;
			for (const auto &[key, w] : coefficient_keys(collision_set, i))
			{
				numerator += w * memoized_stiffness(collision_set, i, key);
				denominator += w;
			}
			collision_set[i].stiffness_scale = numerator / denominator;
		}
	}

	double BarrierContactForm::resolve_stiffness(const double kappa, const bool continued) const
	{
		// Sentinels from assign_collision_stiffness: 0 = nonpositive curvature
		// with no previous value, +inf = overflow with no previous value.
		// During the uncapped first pass of a refresh the batch statistics are
		// not yet known (floor 0, cap inf) and the sentinel passes through;
		// the refresh re-resolves every stencil once they are.
		double k = kappa;
		if (!continued)
		{
			if (k == 0.0)
				k = kappa_floor_; // 0 when no floor is available
			else if (!std::isfinite(k))
				k = kappa_cap_; // still inf when no cap is available
			if (std::isfinite(kappa_cap_))
				k = std::min(k, kappa_cap_);
			if (kappa_floor_ > 0)
				k = std::max(k, kappa_floor_);
		}
		// kappa_min is the user's absolute floor (system units, so divided
		// by the form weight like every other coefficient); applied last.
		if (kappa_min_ > 0)
			k = std::max(k, kappa_min_ / weight_);
		// An overflowing local curvature with no previous value and no batch
		// cap to reference has no finite meaning; an infinite coefficient
		// would only fail the solve later with an infinite objective.
		if (!batch_first_pass_ && !std::isfinite(k))
			log_and_throw_error(
				"Semi-implicit barrier stiffness: overflowing local curvature with no previous value and no batch cap to fall back on");
		return k;
	}

	bool BarrierContactForm::calibrate_trim(const Eigen::VectorXd &x)
	{
		CoefficientEventScope event(*this, x, "calibration");
		if (!system_gradient_provider_ || collision_set_.empty())
			return false;

		Eigen::VectorXd grad_energy;
		system_gradient_provider_(x, grad_energy);

		// The barrier gradient includes the per-contact stiffness_scale, so
		// the least-squares balance ratio against the driving forces is
		// directly the trim (up to the form weight, which multiplies the
		// barrier but not grad_energy).
		Eigen::VectorXd grad_barrier = barrier_potential_.gradient(
			collision_set_, collision_mesh_, compute_displaced_surface(x));
		grad_barrier = collision_mesh_.to_full_dof(grad_barrier);

		const double gb_norm = grad_barrier.norm();
		const double ge_norm = grad_energy.norm();
		if (!(gb_norm > 0) || !(ge_norm > 0))
			return false;

		// The balance is only meaningful when the driving forces actually
		// load the contact: require a minimum opposition between the energy
		// gradient and the barrier force. A barely-touching contact has
		// ||grad B|| ~ 0 and a near-random direction, so the raw quotient
		// -<gB,gE>/||gB||^2 divides noise by noise and can explode to
		// either clamp; the cosine gate rejects exactly those states.
		const double cos_opposition =
			-grad_barrier.dot(grad_energy) / (gb_norm * ge_norm);
		constexpr double min_opposition = 0.1;
		if (!std::isfinite(cos_opposition) || cos_opposition < min_opposition)
			return false;

		// = -<gB,gE> / (weight * ||gB||^2), the least-squares balance.
		const double kappa_gb = cos_opposition * ge_norm / (weight_ * gb_norm);
		if (!std::isfinite(kappa_gb) || kappa_gb <= 0)
			return false;

		// Upward-only: the balance is dominated by the comfortable bulk of
		// the contacts and cannot see a single collapsing hotspot, so
		// letting it LOWER the trim starves exactly the contacts that are
		// about to fail. Downward motion belongs to the band machinery
		// (first-contact conditioning cap + pinned-above-band steps).
		const double new_trim = std::clamp(kappa_gb, trim_min_, trim_max_);
		if (new_trim > barrier_stiffness_)
		{
			logger().debug(
				"Gradient-balance trim calibration: {:g} -> {:g}",
				barrier_stiffness_, new_trim);
			barrier_stiffness_ = new_trim;
			iters_since_trim_ = 0;
		}
		return true;
	}

	void BarrierContactForm::update_barrier_stiffness(const Eigen::VectorXd &x, const Eigen::MatrixXd &grad_energy)
	{
		if (uses_semi_implicit_stiffness())
		{
			// barrier_stiffness_ is the trim multiplier in this mode; it is
			// initialized to 1 by SolveData and persists across (sub)solves.
			// The refresh recalibrates it by gradient balance (via the
			// injected gradient provider), so the grad_energy argument is
			// unused here.
			refresh_semi_implicit_stiffness(x, true, /*published_endpoint=*/true);
			return;
		}

		if (!use_adaptive_barrier_stiffness())
			return;

		const Eigen::MatrixXd displaced_surface = compute_displaced_surface(x);

		// The adative stiffness is designed for the non-convergent formulation,
		// so we need to compute the gradient of the non-convergent barrier.
		// After we can map it to a good value for the convergent formulation.
		ipc::NormalCollisions nonconvergent_constraints;
		// nonconvergent_constraints.set_use_convergent_formulation(false);
		nonconvergent_constraints.set_use_area_weighting(false);
		nonconvergent_constraints.set_use_improved_max_approximator(false);
		nonconvergent_constraints.build(
			collision_mesh_, displaced_surface, dhat_, dmin_, broad_phase_.get());
		Eigen::VectorXd grad_barrier = barrier_potential_.gradient(
			nonconvergent_constraints, collision_mesh_, displaced_surface);
		grad_barrier = collision_mesh_.to_full_dof(grad_barrier);

		barrier_stiffness_ = ipc::initial_barrier_stiffness(
			ipc::world_bbox_diagonal_length(displaced_surface), barrier_potential_.barrier(), dhat_, avg_mass_,
			grad_energy, grad_barrier, max_barrier_stiffness_);

		if (use_convergent_formulation())
		{
			double scaling_factor = 0;
			if (!nonconvergent_constraints.empty())
			{
				const double nonconvergent_potential = barrier_potential_(
					nonconvergent_constraints, collision_mesh_, displaced_surface);

				update_collision_set(displaced_surface);
				const double convergent_potential = barrier_potential_(
					collision_set_, collision_mesh_, displaced_surface);

				scaling_factor = nonconvergent_potential / convergent_potential;
			}
			else
			{
				// Hardcoded difference between the non-convergent and convergent barrier
				scaling_factor = dhat_ * std::pow(dhat_ + 2 * dmin_, 2);
			}
			barrier_stiffness_ *= scaling_factor;
			max_barrier_stiffness_ *= scaling_factor;
		}

		// The barrier stiffness is choosen based on including the acceleration scaling,
		// but the acceleration scaling will be applied later. Therefore, we need to remove it.
		barrier_stiffness_ /= weight_;
		max_barrier_stiffness_ /= weight_;

		logger().debug(
			"Setting adaptive barrier stiffness to {} (max barrier stiffness: {})",
			barrier_stiffness(), max_barrier_stiffness_);
	}

	void BarrierContactForm::update_collision_set(const Eigen::MatrixXd &displaced_surface)
	{
		// Position equality is not a complete cache key: another form can use
		// the same coordinates with different topology/configuration, and this
		// form's candidates or mesh collision filter can change at fixed x.
		// Rebuild on every notification; retain the instance-owned candidate
		// and frozen per-stencil stiffness caches below. Independent forms do
		// not share mutable collision state (same-form mutation is not concurrent).

		if (use_cached_candidates_)
			collision_set_.build(
				candidates_, collision_mesh_, displaced_surface, dhat_);
		else
			collision_set_.build(
				collision_mesh_, displaced_surface, dhat_, dmin_, broad_phase_.get());

		// Every rebuild flows through here (init, solution_changed, and
		// line-search trial states), so the per-collision stiffnesses are
		// always in sync with the current collision set.
		if (uses_semi_implicit_stiffness())
		{
			assign_collision_stiffness(collision_set_);
		}
	}

	BarrierContactForm BarrierContactForm::diagnostic_snapshot(const Eigen::VectorXd &x) const
	{
		BarrierContactForm snapshot(*this);
		// A fresh broad phase avoids touching the shared production broad phase.
		auto broad_phase = ipc::create_broad_phase(broad_phase_method_);
		snapshot.collision_set_.build(collision_mesh_, compute_displaced_surface(x), dhat_, dmin_, broad_phase.get());
		if (uses_semi_implicit_stiffness())
			snapshot.assign_collision_stiffness(snapshot.collision_set_);
		return snapshot;
	}

	json BarrierContactForm::diagnostic_path(const Eigen::VectorXd &start, const Eigen::VectorXd &end) const
	{
		const Eigen::VectorXd dx = end - start;
		json signatures = json::array();
		std::vector<std::string> identities;
		auto evaluate = [&](double t) {
			const Eigen::VectorXd x = start + t * dx;
			const auto snapshot = diagnostic_snapshot(x);
			Eigen::VectorXd g;
			snapshot.first_derivative(x, g);
			const double energy = snapshot.value(x), derivative = g.dot(dx);
			if (!std::isfinite(energy) || !std::isfinite(derivative))
				throw std::runtime_error("Nonfinite contact path sample");
			std::vector<std::array<long, 5>> keys;
			const auto &collisions = snapshot.collision_set();
			for (size_t i = 0; i < collisions.size(); ++i)
			{
				const auto ids = collisions[i].vertex_ids(collision_mesh_.edges(), collision_mesh_.faces());
				const long tag = collisions.is_vertex_vertex(i) ? 0 : collisions.is_edge_vertex(i) ? 1
																  : collisions.is_edge_edge(i)     ? 2
																								   : 3;
				keys.push_back({{tag, long(ids[0]), long(ids[1]), long(ids[2]), long(ids[3])}});
			}
			std::sort(keys.begin(), keys.end());
			const json key_json = keys;
			const std::string identity = key_json.dump();
			auto found = std::find(identities.begin(), identities.end(), identity);
			const size_t index = found - identities.begin();
			if (found == identities.end())
			{
				identities.push_back(identity);
				signatures.push_back(key_json);
			}
			return json{{"t", t}, {"energy_objective", energy}, {"directional_derivative_objective", derivative}, {"signature", index}};
		};
		json samples = json::array(), quadrature = json::array(), transitions = json::array();
		constexpr int panels = 1024;
		for (int i = 0; i <= panels; ++i)
			samples.push_back(evaluate(double(i) / panels));
		const double change = double(samples.back()["energy_objective"]) - double(samples.front()["energy_objective"]);
		for (int n : {16, 64, 256, 1024})
		{
			double integral = 0;
			for (int i = 0; i <= n; ++i)
				integral += (i == 0 || i == n ? .5 : 1.) * double(samples[i * (panels / n)]["directional_derivative_objective"]) / n;
			quadrature.push_back({{"panels", n}, {"gradient_integral_objective", integral}, {"energy_minus_integral_objective", change - integral}});
		}
		for (int i = 1; i <= panels; ++i)
		{
			if (samples[i - 1]["signature"] == samples[i]["signature"])
				continue;
			json left = samples[i - 1], right = samples[i];
			bool multiple = false;
			for (int j = 0; j < 24; ++j)
			{
				const auto mid = evaluate(.5 * (double(left["t"]) + double(right["t"])));
				if (mid["signature"] == left["signature"])
					left = mid;
				else
				{
					multiple = multiple || mid["signature"] != right["signature"];
					right = mid;
				}
			}
			transitions.push_back({{"left", left}, {"right", right}, {"multiple_signatures_in_bracket", multiple}, {"energy_jump_estimate_objective", double(right["energy_objective"]) - double(left["energy_objective"])}});
		}
		return {{"scope", "Straight physical-coordinate segment with this frozen coefficient snapshot; fixed-grid quadrature and detected feature transitions only, not exhaustive event isolation or a collision certificate"},
				{"samples", samples},
				{"quadrature", quadrature},
				{"transitions", transitions},
				{"signatures", signatures}};
	}

	json BarrierContactForm::diagnostic_state() const
	{
		json result = {{"active_count", collision_set_.size()}, {"dhat", dhat_}, {"trim_or_global_stiffness", barrier_stiffness_}, {"semi_implicit", uses_semi_implicit_stiffness()}, {"refresh_id", diagnostic_refresh_id_}, {"iterations_since_refresh", iters_since_refresh_}, {"memoized_stencil_count", kappa_cache_.size()}};
		int zeros = 0, nonfinite = 0;
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		for (size_t i = 0; i < collision_set_.size(); ++i)
		{
			const double k = collision_set_[i].stiffness_scale;
			if (!std::isfinite(k))
				++nonfinite;
			else
			{
				lo = std::min(lo, k);
				hi = std::max(hi, k);
				zeros += k == 0;
			}
		}
		if (!std::isfinite(barrier_stiffness_))
			result["trim_or_global_stiffness_unavailable_reason"] = "Nonfinite production stiffness";
		result["coefficient_zero_count"] = zeros;
		result["coefficient_nonfinite_count"] = nonfinite;
		result["batch_median"] = kappa_median_;
		result["batch_floor"] = kappa_floor_;
		result["batch_cap"] = std::isfinite(kappa_cap_) ? json(kappa_cap_) : json(nullptr);
		result["curvature_fallback_count"] = kappa_fallback_count_;
		result["curvature_abs_fallback_count"] = kappa_abs_fallback_count_;
		result["curvature_global_fallback_count"] = kappa_global_fallback_count_;
		result["interpolated_condensed_count"] = kappa_interpolated_count_;
		result["interpolated_direction_count"] = kappa_direction_fallback_count_;
		result["force_continuation"] = force_continuation_;
		result["continuation_max_ratio"] = continuation_max_ratio_;
		result["continued_count"] = kappa_continued_count_;
		result["fresh_count"] = kappa_fresh_count_;
		result["coefficient_identity"] = parent_keyed_ ? "parent" : "stencil";
		result["coefficient_range"] = std::isfinite(lo) ? json{{"value", {lo, hi}}}
														: json{{"value", nullptr}, {"unavailable_reason", "No finite active coefficients"}};
		// The swept cache is cleared at line_search_end, so an endpoint sees
		// the retained counts of this solve's trial sweeps (RB-04), not a
		// live cache. A form that never built candidates reports the reason.
		const auto &stats = candidate_statistics();
		if (use_cached_candidates_)
			result["candidate_count"] = {{"value", candidates_.size()}, {"scope", "Active swept candidate cache"}};
		else if (stats.builds > 0)
		{
			result["candidate_count"] = {{"value", stats.last}, {"max", stats.max}, {"builds", stats.builds}, {"scope", "Broad-phase candidates of the last trial sweep handed to CCD since the last statistics reset; max/builds over those sweeps"}};
			// RB-05: the broad phase's own intermediates, when it measures them.
			if (stats.intermediates_measured)
				result["candidate_count"]["broad_phase_intermediates"] = {{"cell_items", {{"last", stats.last_cell_items}, {"max", stats.max_cell_items}}}, {"candidate_emissions", {{"last", stats.last_candidate_emissions}, {"max", stats.max_candidate_emissions}}}, {"scope", "Hash-grid (box, cell) items of the sweep's build and pre-filter pair emissions of its detection passes (emissions counted only while a resource limit is enabled, otherwise 0)"}};
			else
				result["candidate_count"]["broad_phase_intermediates"] = {{"value", nullptr}, {"unavailable_reason", "The configured broad phase does not measure its intermediate buffers"}};
		}
		else
			result["candidate_count"] = {{"value", nullptr}, {"unavailable_reason", "No line search built a swept candidate set since the last statistics reset"}};
		return result;
	}

	double BarrierContactForm::value_unweighted(const Eigen::VectorXd &x) const
	{
		return barrier_potential_(collision_set_, collision_mesh_, compute_displaced_surface(x));
	}

	Eigen::VectorXd BarrierContactForm::value_per_element_unweighted(const Eigen::VectorXd &x) const
	{
		const Eigen::MatrixXd V = compute_displaced_surface(x);
		assert(V.rows() == collision_mesh_.num_vertices());

		const size_t num_vertices = collision_mesh_.num_vertices();

		if (collision_set_.empty())
		{
			return Eigen::VectorXd::Zero(collision_mesh_.full_num_vertices());
		}

		const Eigen::MatrixXi &E = collision_mesh_.edges();
		const Eigen::MatrixXi &F = collision_mesh_.faces();

		auto storage = utils::create_thread_storage<Eigen::VectorXd>(Eigen::VectorXd::Zero(num_vertices));

		utils::maybe_parallel_for(collision_set_.size(), [&](int start, int end, int thread_id) {
			Eigen::VectorXd &local_storage = utils::get_local_thread_storage(storage, thread_id);

			for (size_t i = start; i < end; i++)
			{
				// Quadrature weight is premultiplied by compute_potential
				const double potential = barrier_potential_(collision_set_[i], collision_set_[i].dof(V, E, F));

				const int n_v = collision_set_[i].num_vertices();
				const auto vis = collision_set_[i].vertex_ids(E, F);
				for (int j = 0; j < n_v; j++)
				{
					assert(0 <= vis[j] && vis[j] < num_vertices);
					local_storage[vis[j]] += potential / n_v;
				}
			}
		});

		Eigen::VectorXd out = Eigen::VectorXd::Zero(num_vertices);
		for (const auto &local_potential : storage)
		{
			out += local_potential;
		}

		Eigen::VectorXd out_full = Eigen::VectorXd::Zero(collision_mesh_.full_num_vertices());
		for (int i = 0; i < out.size(); i++)
			out_full[collision_mesh_.to_full_vertex_id(i)] = out[i];

		assert(std::abs(value_unweighted(x) - out_full.sum()) < std::max(1e-10 * out_full.sum(), 1e-10));

		return out_full;
	}

	void BarrierContactForm::first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const
	{
		gradv = barrier_potential_.gradient(collision_set_, collision_mesh_, compute_displaced_surface(x));
		gradv = collision_mesh_.to_full_dof(gradv);
	}

	void BarrierContactForm::second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const
	{
		POLYFEM_SCOPED_TIMER("barrier hessian");

		ipc::PSDProjectionMethod psd_projection_method;

		if (project_to_psd_)
		{
			psd_projection_method = ipc::PSDProjectionMethod::CLAMP;
		}
		else
		{
			psd_projection_method = ipc::PSDProjectionMethod::NONE;
		}

		hessian = barrier_potential_.hessian(collision_set_, collision_mesh_, compute_displaced_surface(x), psd_projection_method);
		hessian = collision_mesh_.to_full_dof(hessian);
	}

	void BarrierContactForm::post_step(const polysolve::nonlinear::PostStepData &data)
	{
		const Eigen::MatrixXd displaced_surface = compute_displaced_surface(data.x);

		const double curr_distance = collision_set_.compute_minimum_distance(collision_mesh_, displaced_surface);
		if (!std::isinf(curr_distance))
		{
			const double ratio = sqrt(curr_distance) / dhat();
			const auto log_level = (ratio < 1e-6) ? spdlog::level::err : ((ratio < 1e-4) ? spdlog::level::warn : spdlog::level::debug);
			polyfem::logger().log(log_level, "Minimum distance during solve: {}, dhat: {}", sqrt(curr_distance), dhat());
		}

		if (data.iter_num == 0)
			return;
		CoefficientEventScope event(*this, data.x, "post_step");

		if (uses_semi_implicit_stiffness())
		{
			// Contact born mid-solve: the snapshot predates any contact, so
			// the trim was never initialized against these kappas. Refresh
			// immediately (with the controller, so the conditioning cap
			// softens the barrier while the contact is still unloaded).
			if (!kappa_snapshot_had_contacts_ && !collision_set_.empty())
				refresh_semi_implicit_stiffness(data.x);

			// In-solve trim controller (mirrors classic IPC's emergency
			// doubling, made two-sided): an upward proportional bump when
			// the gap collapses below the band, and a slower-cadenced
			// downward step when the gap stays pinned above it (barrier
			// dominating the elasticity). Only the global trim moves
			// mid-solve; the per-contact snapshot stays frozen.
			const double avg_d2 = collision_set_.compute_avg_distance(
				collision_mesh_, displaced_surface, dhat_);
			const double severity = collapse_severity(avg_d2, curr_distance);
			++iters_since_trim_;
			if (std::isfinite(severity))
			{
				constexpr int emergency_cooldown = 3;
				// Cap the total in-solve upward excursion above the last
				// refresh: climbing further requires a refresh (stall
				// retune), which re-anchors the budget and lets the
				// calibration weigh in. Without this, a persistent pinch
				// rails the trim to trim_max within a few dozen iterations,
				// destroying the Newton system long before the physics can
				// respond.
				constexpr double max_in_solve_climb = 256.0;
				const double dhat_sq = dhat_ * dhat_;
				if (severity < trim_lower_ * dhat_sq
					&& iters_since_trim_ >= emergency_cooldown)
				{
					const double allowed =
						trim_solve_anchor_ * max_in_solve_climb / barrier_stiffness_;
					if (allowed > 1)
						bump_trim(std::min(collapse_bump_factor(severity), allowed));
				}
				else if (
					controller_interval_ > 0
					&& std::isfinite(avg_d2) && avg_d2 > trim_upper_ * dhat_sq
					&& iters_since_trim_ >= controller_interval_)
					bump_trim(1.0 / trim_factor_);

				polyfem::logger().debug(
					"Semi-implicit barrier stiffness: trim={:g}, sqrt(avg d2)/dhat={:g}, sqrt(min d2)/dhat={:g}",
					barrier_stiffness(),
					std::isfinite(avg_d2) ? sqrt(avg_d2) / dhat_ : -1.0,
					std::isfinite(curr_distance) ? sqrt(curr_distance) / dhat_ : -1.0);
			}

			// Optional per-iteration refresh for experimentation (changes
			// the objective mid-solve; default 0 = disabled).
			if (refresh_interval_ > 0 && ++iters_since_refresh_ >= refresh_interval_)
				refresh_semi_implicit_stiffness(data.x);
		}
		else if (use_adaptive_barrier_stiffness_)
		{
			if (is_time_dependent_)
			{
				const double prev_barrier_stiffness = barrier_stiffness();

				barrier_stiffness_ = ipc::update_barrier_stiffness(
					prev_distance_, curr_distance, max_barrier_stiffness_,
					barrier_stiffness(), ipc::world_bbox_diagonal_length(displaced_surface));

				if (barrier_stiffness() != prev_barrier_stiffness)
				{
					polyfem::logger().debug(
						"updated barrier stiffness from {:g} to {:g} (max barrier stiffness: )",
						prev_barrier_stiffness, barrier_stiffness(), max_barrier_stiffness_);
				}
			}
			else
			{
				// TODO: missing feature
				// update_barrier_stiffness(data.x);
			}
		}

		prev_distance_ = curr_distance;
	}
} // namespace polyfem::solver
