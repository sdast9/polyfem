#pragma once

#include "ContactForm.hpp"

#include <polyfem/utils/Types.hpp>
#include <polysolve/nonlinear/PostStepData.hpp>

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/potentials/barrier_potential.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <functional>
#include <map>

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
		const ipc::BarrierPotential &barrier_potential() const { return barrier_potential_; }

		// -- Semi-implicit per-contact barrier stiffness [Ando 2024] ----------

		/// @brief Is the semi-implicit per-contact stiffness mode active?
		bool uses_semi_implicit_stiffness() const { return stiffness_mode_ == BarrierStiffnessMode::SemiImplicit; }

		/// @brief Set the callback used to assemble the (weighted) system
		///        Hessian of the elastic energy at a given solution.
		void set_system_hessian_provider(const std::function<void(const Eigen::VectorXd &, StiffnessMatrix &)> &provider) { system_hessian_provider_ = provider; }

		/// @brief Refresh the frozen snapshot (displaced surface + system
		///        Hessian) used to compute per-contact stiffnesses, and
		///        assign stiffness scales to the current collision set.
		///        Optionally runs one step of the gap-band trim controller.
		/// @param x Current solution (full size)
		void refresh_semi_implicit_stiffness(const Eigen::VectorXd &x, const bool run_trim_controller = true);

		/// @brief Assign per-collision stiffness scales computed from the
		///        frozen snapshot to the given collision set. Deterministic
		///        between refreshes (memoized per stencil).
		void assign_collision_stiffness(ipc::NormalCollisions &collision_set) const;

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
		void retune_on_stall(const Eigen::VectorXd &x, const double factor);

	protected:
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

		/// @brief Lumped mass per full-mesh vertex (zeros when quasistatic)
		Eigen::VectorXd lumped_vertex_masses_;

		/// @brief Displaced surface frozen at the last stiffness refresh
		Eigen::MatrixXd kappa_surface_;
		/// @brief System Hessian (full DOF) frozen at the last refresh
		StiffnessMatrix kappa_hessian_;
		/// @brief Memoized per-stencil stiffness for the frozen snapshot;
		///        key = (stencil type tag, vertex ids)
		mutable std::map<std::array<long, 5>, double> kappa_cache_;
		/// @brief Newton iterations since the last stiffness refresh
		int iters_since_refresh_ = 0;
		/// @brief Newton iterations since the trim factor last changed
		int iters_since_trim_ = 0;
		/// @brief Frozen per-contact stiffness cap (kappa_spread * median of
		///        the refresh batch); part of the snapshot for determinism
		double kappa_cap_ = std::numeric_limits<double>::infinity();
		// Parsed semi-implicit options (see input-spec.json defaults)
		/// @brief 0 = refresh only at solve starts and stall restarts (frozen
		///        objective within a solve); N > 0 = also every N iterations
		int refresh_interval_ = 0;
		double trim_lower_ = 1e-4;
		double trim_upper_ = 0.9;
		double trim_factor_ = 2.0;
		double trim_min_ = std::pow(2.0, -16);
		double trim_max_ = std::pow(2.0, 16);
		double kappa_min_ = 0.0;
		double kappa_spread_ = 1e4;
	};
} // namespace polyfem::solver
