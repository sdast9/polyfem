#pragma once

#include "Form.hpp"
#include "ContactForm.hpp"

#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Types.hpp>

#include <ipc/ipc.hpp>
#include <ipc/collision_mesh.hpp>
#include <ipc/collisions/tangential/tangential_collisions.hpp>
#include <ipc/potentials/friction_potential.hpp>
#include <ipc/broad_phase/create_broad_phase.hpp>

#include <memory>

namespace polyfem::solver
{
	/// @brief Form of the lagged friction disapative potential and forces
	class FrictionForm : public Form
	{
		friend class FrictionForceDerivative;

	public:
		/// @brief Construct a new Friction Form object
		/// @param collision_mesh Reference to the collision mesh
		/// @param time_integrator Pointer to the time integrator
		/// @param epsv Smoothing factor between static and dynamic friction
		/// @param mu Global coefficient of friction
		/// @param dhat Barrier activation distance
		/// @param broad_phase_method Broad-phase method used for distance computation and collision detection
		/// @param contact_form Pointer to contact form; necessary to have the barrier stiffnes, maybe clean me
		/// @param n_lagging_iters Number of lagging iterations
		FrictionForm(
			const ipc::CollisionMesh &collision_mesh,
			const std::shared_ptr<time_integrator::ImplicitTimeIntegrator> time_integrator,
			const double epsv,
			const double mu,
			const ipc::BroadPhaseMethod broad_phase_method,
			const ContactForm &contact_form,
			const int n_lagging_iters);

		std::string name() const override { return "friction"; }

	protected:
		/// @brief Compute the value of the form
		/// @param x Current solution
		/// @return Computed value
		double value_unweighted(const Eigen::VectorXd &x) const override;

		/// @brief Compute the first derivative of the value wrt x
		/// @param[in] x Current solution
		/// @param[out] gradv Output gradient of the value wrt x
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;

		/// @brief Compute the second derivative of the value wrt x
		/// @param[in] x Current solution
		/// @param[out] hessian Output Hessian of the value wrt x
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const override;

	public:
		/// @brief Initialize lagged fields
		/// @param x Current solution
		void init_lagging(const Eigen::VectorXd &x) override;

		/// @brief RB-10 realized-force lag (default): between steps the
		///        accepted endpoint is reached before the barrier's
		///        between-steps refresh, so the lag built here carries the
		///        normal force that acted at that endpoint. No-op with
		///        `friction_lag: "follow_stiffness"`.
		void update_quantities(const double t, const Eigen::VectorXd &x) override;

		/// @brief Update lagged fields
		/// @param x Current solution
		void update_lagging(const Eigen::VectorXd &x, const int iter_num) override;

		/// @brief Update lagged fields
		/// @param x Current solution
		void update_lagging(const Eigen::VectorXd &x) { update_lagging(x, -1); };

		/// @brief Get the maximum number of lagging iteration allowable.
		int max_lagging_iterations() const override { return n_lagging_iters_; }

		/// @brief Does this form require lagging?
		/// @return True if the form requires lagging
		bool uses_lagging() const override { return true; }

		/// @brief Compute the displaced positions of the surface nodes
		Eigen::MatrixXd compute_displaced_surface(const Eigen::VectorXd &x) const;
		/// @brief Compute the surface velocities
		Eigen::MatrixXd compute_surface_velocities(const Eigen::VectorXd &x) const;
		/// @brief Compute the derivative of the velocities wrt x
		double dv_dx() const;

		double mu() const { return mu_; }
		double epsv() const { return epsv_; }
		const ipc::TangentialCollisions &friction_collision_set() const { return friction_collision_set_; }
		/// @brief RB-18 F6 (`friction_lag: "follow_stiffness"` only): the
		///        lagged normal-force magnitudes were built with the contact
		///        trim at lag time; when the in-solve controller moves the
		///        trim, value/gradient/Hessian are rescaled by the trim ratio
		///        to stay consistent with the barrier at fixed coordinates.
		///        1 in the default realized-force mode and every other mode
		///        (RB-10: the equilibrium normal force does not follow the
		///        trim, the gap does).
		double trim_scale() const;
		/// @brief RB-10: does this form lag the realized normal force (no trim
		///        following, lag built before the between-steps refresh)?
		bool realized_lag() const;
		const ipc::FrictionPotential &friction_potential() const { return friction_potential_; }

		/// @brief RB-06: the lag -- the tangential collision set with its
		///        lagged normal forces, the trim baked into them and the
		///        coordinates it was built at.
		struct State : public FormState
		{
			ipc::TangentialCollisions friction_collision_set;
			double lagged_trim = 1;
			Eigen::VectorXd lag_x;
		};
		std::unique_ptr<FormState> save_state() const override;
		void restore_state(const FormState &state, const Eigen::VectorXd &x) override;

	private:
		/// Reference to the collision mesh
		const ipc::CollisionMesh &collision_mesh_;

		/// Pointer to the time integrator
		const std::shared_ptr<time_integrator::ImplicitTimeIntegrator> time_integrator_;

		const double epsv_;                              ///< Smoothing factor between static and dynamic friction
		const double mu_;                                ///< Global coefficient of friction
		const ipc::BroadPhaseMethod broad_phase_method_; ///< Broad-phase method used for distance computation and collision detection
		const int n_lagging_iters_;                      ///< Number of lagging iterations

		ipc::TangentialCollisions friction_collision_set_; ///< Lagged friction constraint set
		double lagged_trim_ = 1;                           ///< Contact trim baked into the lagged normal forces
		Eigen::VectorXd lag_x_;                            ///< Coordinates the current lag was built at (realized mode)

		const ContactForm &contact_form_; ///< necessary to have the barrier stiffnes, maybe clean me

		const ipc::FrictionPotential friction_potential_;
	};
} // namespace polyfem::solver
