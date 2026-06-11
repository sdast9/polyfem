#include "BarrierContactForm.hpp"

#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/MaybeParallelFor.hpp>
#include <polyfem/utils/Timer.hpp>
#include <polyfem/utils/Types.hpp>

#include <ipc/barrier/adaptive_stiffness.hpp>
#include <ipc/utils/world_bbox_diagonal_length.hpp>

#include <algorithm>
#include <vector>
#include <cassert>
#include <cmath>

namespace polyfem::solver
{
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
										   const Eigen::VectorXd &lumped_vertex_masses) : ContactForm(collision_mesh, dhat, avg_mass, use_adaptive_barrier_stiffness, is_time_dependent, enable_shape_derivatives, broad_phase_method, ccd_tolerance, ccd_max_iterations), barrier_potential_(dhat, 1.0, use_physical_barrier), stiffness_mode_(stiffness_mode), lumped_vertex_masses_(lumped_vertex_masses)
	{
		// collision_set_.set_use_convergent_formulation(use_convergent_formulation);
		collision_set_.set_use_area_weighting(use_area_weighting);
		collision_set_.set_use_improved_max_approximator(use_improved_max_operator);
		collision_set_.set_enable_shape_derivatives(enable_shape_derivatives);

		if (uses_semi_implicit_stiffness())
		{
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
			}

			refresh_interval_ = std::max(refresh_interval_, 0);
			kappa_min_ = std::max(kappa_min_, 0.0);
			if (!(trim_lower_ < trim_upper_))
				log_and_throw_error("Semi-implicit barrier stiffness requires trim_lower < trim_upper!");
		}
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

	void BarrierContactForm::retune_on_stall(const Eigen::VectorXd &x, const double factor)
	{
		// A stall with the gap below the band means the barrier is too soft
		// (the solver is crawling against CCD); otherwise the barrier is
		// likely dominating the elasticity and should be softened.
		const double avg_d2 = collision_set_.compute_avg_distance(
			collision_mesh_, compute_displaced_surface(x), dhat_);

		if (std::isfinite(avg_d2) && avg_d2 < trim_lower_ * dhat_ * dhat_)
			bump_trim(std::max(factor, collapse_bump_factor(avg_d2)));
		else
			bump_trim(1.0 / factor);

		refresh_semi_implicit_stiffness(x, /*run_trim_controller=*/false);
	}

	void BarrierContactForm::refresh_semi_implicit_stiffness(const Eigen::VectorXd &x, const bool run_trim_controller)
	{
		if (!uses_semi_implicit_stiffness())
			return;
		if (!system_hessian_provider_)
			log_and_throw_error("Semi-implicit barrier stiffness requires a system Hessian provider!");

		// Two-sided trim controller, one step per refresh (i.e., per subsolve
		// or stall restart, NOT per Newton iteration -- the objective must
		// stay fixed within a solve): keep the average active squared gap in
		// [trim_lower, trim_upper] * dhat^2.
		if (run_trim_controller)
		{
			const double avg_d2 = collision_set_.compute_avg_distance(
				collision_mesh_, compute_displaced_surface(x), dhat_);
			if (std::isfinite(avg_d2))
			{
				const double dhat_sq = dhat_ * dhat_;
				if (avg_d2 < trim_lower_ * dhat_sq)
					bump_trim(collapse_bump_factor(avg_d2));
				else if (avg_d2 > trim_upper_ * dhat_sq)
					bump_trim(1.0 / trim_factor_);
			}
		}

		kappa_surface_ = compute_displaced_surface(x);
		system_hessian_provider_(x, kappa_hessian_);
		kappa_cache_.clear();
		iters_since_refresh_ = 0;

		// First pass uncapped to freeze the cap at kappa_spread * median of
		// the batch (guards against exploded Hessian blocks of crushed
		// elements); the cap is part of the snapshot for determinism.
		kappa_cap_ = std::numeric_limits<double>::infinity();
		assign_collision_stiffness(collision_set_);
		if (!collision_set_.empty() && std::isfinite(kappa_spread_) && kappa_spread_ > 0)
		{
			std::vector<double> kappas(collision_set_.size());
			for (size_t i = 0; i < collision_set_.size(); i++)
				kappas[i] = collision_set_[i].stiffness_scale;
			std::nth_element(kappas.begin(), kappas.begin() + kappas.size() / 2, kappas.end());
			kappa_cap_ = kappa_spread_ * kappas[kappas.size() / 2];

			// Re-clamp the already-assigned scales with the frozen cap.
			for (size_t i = 0; i < collision_set_.size(); i++)
				collision_set_[i].stiffness_scale =
					std::min(collision_set_[i].stiffness_scale, kappa_cap_);
		}

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

	void BarrierContactForm::assign_collision_stiffness(ipc::NormalCollisions &collision_set) const
	{
		if (!uses_semi_implicit_stiffness() || kappa_surface_.size() == 0)
			return;

		const Eigen::MatrixXi &E = collision_mesh_.edges();
		const Eigen::MatrixXi &F = collision_mesh_.faces();
		const int dim = collision_mesh_.dim();

		for (size_t i = 0; i < collision_set.size(); i++)
		{
			if (collision_set.is_plane_vertex(i))
				continue; // unused by polyfem; keep the default scale

			ipc::NormalCollision &collision = collision_set[i];
			const int n_verts = collision.num_vertices();
			const auto vids = collision.vertex_ids(E, F);

			long type_tag = 3; // face-vertex
			if (collision_set.is_vertex_vertex(i))
				type_tag = 0;
			else if (collision_set.is_edge_vertex(i))
				type_tag = 1;
			else if (collision_set.is_edge_edge(i))
				type_tag = 2;

			const std::array<long, 5> key = {
				type_tag, long(vids[0]), long(vids[1]), long(vids[2]), long(vids[3])};

			double kappa;
			const auto cached = kappa_cache_.find(key);
			if (cached != kappa_cache_.end())
			{
				kappa = cached->second;
			}
			else
			{
				// Local positions, masses, and Hessian block come from the
				// FROZEN snapshot, so the value is a deterministic function
				// of the stencil between refreshes (well-defined objective
				// during the line search).
				const ipc::VectorMax12d positions = collision.dof(kappa_surface_, E, F);

				ipc::VectorMax4d local_mass = ipc::VectorMax4d::Zero(n_verts);
				ipc::MatrixMax12d local_hess =
					ipc::MatrixMax12d::Zero(dim * n_verts, dim * n_verts);
				for (int a = 0; a < n_verts; a++)
				{
					const long va = collision_mesh_.to_full_vertex_id(vids[a]);
					if (va < lumped_vertex_masses_.size())
						local_mass[a] = lumped_vertex_masses_[va];
					for (int b = 0; b < n_verts; b++)
					{
						const long vb = collision_mesh_.to_full_vertex_id(vids[b]);
						if (dim * va + dim > kappa_hessian_.rows()
							|| dim * vb + dim > kappa_hessian_.cols())
							continue; // e.g., obstacle DOF not in the Hessian
						for (int k = 0; k < dim; k++)
							for (int l = 0; l < dim; l++)
								local_hess(dim * a + k, dim * b + l) =
									kappa_hessian_.coeff(dim * va + k, dim * vb + l);
					}
				}

				// kappa = avg_mass / d^2 + w^T H w [Ando 2024]
				kappa = ipc::semi_implicit_stiffness(
					collision, positions, local_mass, local_hess, dmin_);

				// w^T H w can be negative for an unprojected indefinite
				// Hessian, and avg_mass / d^2 can overflow at tiny distances.
				if (!std::isfinite(kappa))
					kappa = 1e30;
				kappa = std::max(kappa, kappa_min_);

				// Remove the form weight (acceleration scaling); it is
				// reapplied by ContactForm::weight(). The global
				// barrier_stiffness_ acts as the trim multiplier, so the
				// effective coefficient is trim * kappa.
				kappa /= weight_;

				kappa_cache_.emplace(key, kappa);
			}

			collision.stiffness_scale = std::min(kappa, kappa_cap_);
		}
	}

	void BarrierContactForm::update_barrier_stiffness(const Eigen::VectorXd &x, const Eigen::MatrixXd &grad_energy)
	{
		if (uses_semi_implicit_stiffness())
		{
			// barrier_stiffness_ is the trim multiplier in this mode; it is
			// initialized to 1 by SolveData and persists across (sub)solves.
			refresh_semi_implicit_stiffness(x);
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
		// Store the previous value used to compute the constraint set to avoid duplicate computation.
		static Eigen::MatrixXd cached_displaced_surface;
		if (cached_displaced_surface.size() == displaced_surface.size() && cached_displaced_surface == displaced_surface)
			return;

		if (use_cached_candidates_)
			collision_set_.build(
				candidates_, collision_mesh_, displaced_surface, dhat_);
		else
			collision_set_.build(
				collision_mesh_, displaced_surface, dhat_, dmin_, broad_phase_.get());
		cached_displaced_surface = displaced_surface;

		// Every rebuild flows through here (init, solution_changed, and
		// line-search trial states), so the per-collision stiffnesses are
		// always in sync with the current collision set.
		if (uses_semi_implicit_stiffness())
			assign_collision_stiffness(collision_set_);
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

		if (uses_semi_implicit_stiffness())
		{
			// The two-sided trim controller acts at refresh points so the
			// objective stays fixed within a Newton solve. The single
			// exception (mirroring classic IPC's emergency doubling) is an
			// upward-only bump when the gap is collapsing below the band.
			const double avg_d2 = collision_set_.compute_avg_distance(
				collision_mesh_, displaced_surface, dhat_);
			++iters_since_trim_;
			if (std::isfinite(avg_d2))
			{
				constexpr int emergency_cooldown = 3;
				if (avg_d2 < trim_lower_ * dhat_ * dhat_
					&& iters_since_trim_ >= emergency_cooldown)
					bump_trim(collapse_bump_factor(avg_d2));

				polyfem::logger().debug(
					"Semi-implicit barrier stiffness: trim={:g}, sqrt(avg d2)/dhat={:g}",
					barrier_stiffness(), sqrt(avg_d2) / dhat_);
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
