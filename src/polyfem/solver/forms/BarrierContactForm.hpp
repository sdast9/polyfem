#pragma once

#include "ContactForm.hpp"

#include <polyfem/utils/Types.hpp>
#include <polysolve/nonlinear/PostStepData.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/potentials/barrier_potential.hpp>

#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <functional>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace polyfem::solver
{
	class BarrierContactForm : public ContactForm
	{
		friend class BarrierContactForceDerivative;

	public:
		BarrierContactForm(const ipc::CollisionMesh &collision_mesh,
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
						   const BarrierStiffnessMode stiffness_mode = BarrierStiffnessMode::Adaptive,
						   const json &semi_implicit_opts = json(nullptr),
						   const Eigen::VectorXd &lumped_vertex_masses = Eigen::VectorXd());

		virtual std::string name() const override { return "barrier-contact"; }

		virtual void update_barrier_stiffness(const Eigen::VectorXd &x, const Eigen::MatrixXd &grad_energy) override;

		/// @brief Update fields after a step in the optimization
		/// @param iter_num Optimization iteration number
		/// @param x Current solution
		void post_step(const polysolve::nonlinear::PostStepData &data) override;

		bool use_convergent_formulation() const override { return use_area_weighting() && use_improved_max_operator() && use_physical_barrier(); }

		/// @brief Get use_area_weighting
		bool use_area_weighting() const { return collision_set().use_area_weighting(); }

		/// @brief Get use_improved_max_operator
		bool use_improved_max_operator() const { return collision_set().use_improved_max_approximator(); }

		/// @brief Get use_physical_barrier
		bool use_physical_barrier() const { return barrier_potential_.use_physical_barrier(); }

		const ipc::NormalCollisions &collision_set() const { return collision_set_; }
		/// Independent endpoint reconstruction; never refreshes or mutates this form.
		BarrierContactForm diagnostic_snapshot(const Eigen::VectorXd &x) const;
		/// Bounded observational path quadrature; never refreshes production coefficients.
		json diagnostic_path(const Eigen::VectorXd &start, const Eigen::VectorXd &end) const;
		json diagnostic_state() const;
		/// Observer for outer refresh/calibration/stall/post-step operations.
		/// Callback failures cannot change solver behavior. Direct initialization
		/// setters and coordinate-only feature transitions are outside this stream.
		void set_coefficient_observer(std::function<void(const json &)> observer) { coefficient_observer_ = std::move(observer); }

		const ipc::BarrierPotential &barrier_potential() const { return barrier_potential_; }

		// -- Semi-implicit per-contact barrier stiffness [Ando 2024] ----------

		/// @brief Is the semi-implicit per-contact stiffness mode active?
		bool uses_semi_implicit_stiffness() const { return stiffness_mode_ == BarrierStiffnessMode::SemiImplicit; }

		/// @brief Opt into sequential clamping only in semi-implicit mode, where
		///        the trial steps that make it worthwhile actually occur.
		bool wants_sequential_step_clamping() const override
		{
			return uses_semi_implicit_stiffness();
		}

		/// @brief Cap trial-step surface displacement only in semi-implicit
		///        mode, where Newton trial steps in distorted states can move
		///        vertices by hundreds of barrier supports -- pricing that
		///        sweep is wasted CCD work and a broad-phase memory risk on
		///        thin geometry. Every other mode keeps the base class's
		///        uncapped behaviour, since the cap also bounds the step when
		///        CCD finds no collision.
		double trial_displacement_cap() const override
		{
			return uses_semi_implicit_stiffness()
					   ? trial_displacement_cap_
					   : std::numeric_limits<double>::infinity();
		}

		/// @brief Set the callback used to assemble the (weighted) system
		///        Hessian of the elastic energy at a given solution.
		void set_system_hessian_provider(const std::function<void(const Eigen::VectorXd &, StiffnessMatrix &)> &provider) { system_hessian_provider_ = provider; }

		/// @brief Set the callback used to assemble the (weighted) gradient of
		///        all non-contact energies at a given solution; used to
		///        calibrate the global trim by gradient balance.
		void set_system_gradient_provider(const std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> &provider) { system_gradient_provider_ = provider; }

		/// @brief One-shot trim calibration by gradient balance (classic IPC's
		///        initialization applied to the per-contact-scaled barrier):
		///        trim = -<grad B, grad E> / (weight * ||grad B||^2). Requires
		///        the gradient provider and a *loaded* contact (the balance is
		///        degenerate when the barrier force is negligible or opposes
		///        nothing). @return true if the trim was calibrated.
		bool calibrate_trim(const Eigen::VectorXd &x);

		/// @brief Refresh the frozen snapshot (displaced surface + system
		///        Hessian) used to compute per-contact stiffnesses, and
		///        assign stiffness scales to the current collision set.
		///        Optionally runs one step of the gap-band trim controller.
		/// @param x Current solution (full size)
		/// @param published_endpoint True only for the between-steps refresh
		///        at a published endpoint: RB-20 force continuation captures
		///        the coefficients that acted there. Mid-solve refreshes
		///        (birth, stall retune, interval) keep the captured values and
		///        re-estimate everything else from the fresh Hessian.
		void refresh_semi_implicit_stiffness(const Eigen::VectorXd &x, const bool run_trim_controller = true, const bool published_endpoint = false);

		/// @brief Assign per-collision stiffness scales computed from the
		///        frozen snapshot to the given collision set. Deterministic
		///        between refreshes (memoized per stencil).
		void assign_collision_stiffness(ipc::NormalCollisions &collision_set) const;
		/// @brief Map a memoized per-stencil value (possibly a 0 / +inf
		///        sentinel for invalid curvature) onto the frozen batch floor,
		///        cap and user minimum (RB-18 F1/F2/F4). A continued value
		///        (RB-20) skips the batch floor/cap: it already acted at the
		///        endpoint, and clamping it to a fresh batch would be drift.
		double resolve_stiffness(const double kappa, const bool continued = false) const;
		/// @brief Whether a stencil's coefficient was carried over from the
		///        state that acted at the last refresh point (RB-20).
		bool is_continued(const std::array<long, 5> &key) const { return continued_keys_.count(key) > 0; }
		/// @brief Memoization key of a collision stencil: (type tag, vertex ids)
		std::array<long, 5> stencil_key(const ipc::NormalCollisions &collision_set, const size_t i) const;
		/// @brief The coefficient keys of collision i with their positive
		///        contribution weights: its parent candidates (RB-21,
		///        tag 10 + candidate type, ids) when coefficient_identity is
		///        "parent" and the builder recorded them, else the stencil key
		///        with weight 1.
		std::vector<std::pair<std::array<long, 5>, double>> coefficient_keys(const ipc::NormalCollisions &collision_set, const size_t i) const;
		/// @brief Fresh Hessian estimate of a key's coefficient on the given
		///        stencil at the frozen snapshot, after the RB-18 F2/F4/F7
		///        resolution (may return a 0 / +inf sentinel).
		double estimate_stiffness(const ipc::CollisionStencil &stencil, const std::array<long, 5> &key) const;
		/// @brief Memo lookup (or fresh estimate + insert) of a key's
		///        coefficient, resolved against the frozen batch statistics.
		double memoized_stiffness(const ipc::NormalCollisions &collision_set, const size_t i, const std::array<long, 5> &key) const;

		/// @brief Multiply the global trim factor (barrier_stiffness_) by the
		///        given factor, clamped to [trim_min, trim_max].
		void bump_trim(const double factor);

		/// @brief Trim factor proportional to how far the average gap has
		///        collapsed below the band (capped at 256 per bump).
		double collapse_bump_factor(const double avg_d2) const;

		/// @brief Collapse measure combining the average active gap and the
		///        (slack-relaxed) minimum gap, both squared distances.
		double collapse_severity(const double avg_d2, const double min_d2) const;

		/// @brief Retune the trim after a line-search stall: increase it when
		///        the average active gap is below the band (barrier too soft),
		///        decrease it otherwise (barrier too stiff), then refresh the
		///        per-contact stiffnesses at x.
		///        Returns whether anything changed (per-contact coefficients
		///        were re-evaluated for a non-empty collision set, or the trim
		///        moved); false means a restart would repeat an identical solve.
		bool retune_on_stall(const Eigen::VectorXd &x, const double factor);

	protected:
		class CoefficientEventScope;
		std::function<void(const json &)> coefficient_observer_;
		int coefficient_event_depth_ = 0;
		uint64_t coefficient_event_id_ = 0;
		/// @brief Compute the contact barrier potential value
		/// @param x Current solution
		/// @return Value of the contact barrier potential
		virtual double value_unweighted(const Eigen::VectorXd &x) const override;

		/// @brief Compute the value of the form multiplied per element
		/// @param x Current solution
		/// @return Computed value
		Eigen::VectorXd value_per_element_unweighted(const Eigen::VectorXd &x) const override;

		/// @brief Compute the first derivative of the value wrt x
		/// @param[in] x Current solution
		/// @param[out] gradv Output gradient of the value wrt x
		virtual void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;

		/// @brief Compute the second derivative of the value wrt x
		/// @param x Current solution
		/// @param hessian Output Hessian of the value wrt x
		virtual void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const override;

		void update_collision_set(const Eigen::MatrixXd &displaced_surface) override;

		/// @brief Cached constraint set for the current solution
		ipc::NormalCollisions collision_set_;

		/// @brief Contact potential
		const ipc::BarrierPotential barrier_potential_;

		// -- Semi-implicit per-contact barrier stiffness state -----------------

		/// @brief How the barrier stiffness is chosen and updated
		const BarrierStiffnessMode stiffness_mode_;

		/// @brief Assembles the (weighted) system Hessian of the elastic
		///        energy at a given solution; injected by SolveData.
		std::function<void(const Eigen::VectorXd &, StiffnessMatrix &)> system_hessian_provider_;

		/// @brief Callback assembling the (weighted) gradient of all
		///        non-contact energies at a given solution; injected by
		///        SolveData, used for gradient-balance trim calibration.
		std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> system_gradient_provider_;

		/// @brief Lumped mass per full-mesh vertex (zeros when quasistatic)
		Eigen::VectorXd lumped_vertex_masses_;

		/// @brief Collision vertex to system node for exact unit selector rows;
		///        -1 marks a row whose stiffness definition remains unresolved.
		///        CollisionMesh's displacement map is immutable during form use.
		Eigen::VectorXi stiffness_node_ids_;

		/// @brief Displaced surface frozen at the last stiffness refresh
		Eigen::MatrixXd kappa_surface_;
		/// @brief System Hessian (full DOF) frozen at the last refresh
		StiffnessMatrix kappa_hessian_;
		/// @brief Memoized per-stencil stiffness for the frozen snapshot;
		///        key = (stencil type tag, vertex ids)
		mutable std::map<std::array<long, 5>, double> kappa_cache_;
		/// @brief Memoized stiffness of the PREVIOUS snapshot (RB-18 F2):
		///        a stencil whose fresh curvature is nonpositive or overflows
		///        keeps the value it had rather than losing its barrier
		mutable std::map<std::array<long, 5>, double> prev_kappa_cache_;
		/// @brief Stencils whose coefficient in kappa_cache_ was carried over
		///        from the value that acted at the refresh point instead of
		///        re-estimated from the Hessian (RB-20 force continuation).
		///        Continued values are the resolved effective coefficient and
		///        bypass the batch floor/cap of the new snapshot.
		std::set<std::array<long, 5>> continued_keys_;
		/// @brief The resolved coefficients active at the last published
		///        endpoint (RB-20); re-seeded into kappa_cache_ at every
		///        refresh until the next endpoint replaces them.
		mutable std::map<std::array<long, 5>, double> endpoint_kappa_;
		/// @brief Continued / freshly estimated stencils in the last refresh
		///        batch (diagnostic)
		int kappa_continued_count_ = 0;
		mutable int kappa_fresh_count_ = 0;
		/// @brief Newton iterations since the last stiffness refresh
		int iters_since_refresh_ = 0;
		uint64_t diagnostic_refresh_id_ = 0; ///< Monotonic per-form completed snapshot identity.
		/// @brief Newton iterations since the trim factor last changed
		int iters_since_trim_ = 0;
		/// @brief Frozen per-contact stiffness cap (kappa_spread * median of
		///        the refresh batch); part of the snapshot for determinism
		double kappa_cap_ = std::numeric_limits<double>::infinity();
		/// @brief Frozen relative floor (median / kappa_spread) of the refresh
		///        batch (RB-18 F1); 0 = no floor available
		double kappa_floor_ = 0.0;
		/// @brief Median of the POSITIVE finite per-contact stiffnesses of the
		///        last refresh batch (RB-18 F1); 0 when none is positive
		double kappa_median_ = 0.0;
		/// @brief Stencils in the last refresh batch whose curvature was
		///        invalid, had no previous value, and could only be resolved by
		///        the batch floor/cap (diagnostic)
		mutable int kappa_fallback_count_ = 0;
		/// @brief ... resolved with |w^T H w| (RB-18 F7 choice B)
		mutable int kappa_abs_fallback_count_ = 0;
		/// @brief ... resolved with max|H| / dhat^2 (RB-18 F7 fallback E)
		mutable int kappa_global_fallback_count_ = 0;
		/// @brief True only during the uncapped first assignment pass of a
		///        refresh, when the batch cap/floor are not yet known and
		///        invalid-curvature sentinels must pass through unresolved
		mutable bool batch_first_pass_ = false;
		/// @brief True during the first pass of a published-endpoint refresh
		///        with continuation_max_ratio > 1 (RB-20 D3)
		mutable bool pull_toward_fresh_ = false;
		/// @brief Whether the collision set was non-empty at the last refresh
		///        (detects contact born mid-solve in post_step)
		bool kappa_snapshot_had_contacts_ = false;
		/// @brief Trim value at the end of the last refresh; the in-solve
		///        emergency bumps may climb at most a fixed factor above it
		///        (unbounded in-solve climbing rails the trim to trim_max
		///        before the physics can respond)
		double trim_solve_anchor_ = 1.0;
		/// @brief Max absolute entry of the frozen system Hessian
		double kappa_hessian_max_ = 0.0;
		// Parsed semi-implicit options (see input-spec.json defaults)
		/// @brief 0 = refresh only at solve starts and stall restarts (frozen
		///        objective within a solve); N > 0 = also every N iterations
		int refresh_interval_ = 0;
		double trim_lower_ = 0.5;
		double trim_upper_ = 0.9;
		double trim_factor_ = 2.0;
		double trim_min_ = std::pow(2.0, -32);
		double trim_max_ = std::pow(2.0, 32);
		double kappa_min_ = 0.0;
		double kappa_spread_ = 1e4;
		/// @brief Cap on effective stiffness relative to max|system Hessian|
		///        applied when the gradient balance is degenerate (unloaded
		///        contact): past ~8 orders of dynamic range the linear solver
		///        loses the elastic block, and an unloaded barrier has no
		///        force-balance signal to justify more stiffness.
		double conditioning_cap_ = 1e3;
		/// @brief Newton-iteration cadence of the in-solve downward trim step
		///        (gap pinned above the band); 0 disables it.
		int controller_interval_ = 30;
		/// @brief Trial-step displacement cap, in barrier supports. Only
		///        applied while the semi-implicit stiffness mode is active.
		double trial_displacement_cap_ = 50.0;
		/// @brief RB-20: a stencil active at a refresh point keeps the
		///        coefficient that acted there; the Hessian estimate is used
		///        only for stencils without one. Removes the post-publication
		///        force drift RB-04 measured. The global trim still acts.
		///        Default on since RB-21's parent identity: with the
		///        historical stencil identity a closest-feature switch mixes
		///        a continued and a fresh value and slows Newton badly.
		bool force_continuation_ = true;
		/// @brief RB-20 D3: 0 = pure continuation; r > 1 lets the fresh
		///        Hessian estimate move a continued coefficient within
		///        [kappa/r, kappa*r] per refresh.
		double continuation_max_ratio_ = 0.0;
		/// @brief RB-21: key coefficients on the builder's parent candidates
		///        (true, default) or on the built stencil (false, historical).
		bool parent_keyed_ = true;
	};
} // namespace polyfem::solver
