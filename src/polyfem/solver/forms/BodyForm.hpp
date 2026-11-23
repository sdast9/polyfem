#pragma once

#include "Form.hpp"

#include <polyfem/assembler/RhsAssembler.hpp>
#include <polyfem/mesh/LocalBoundary.hpp>
#include <polyfem/utils/ElasticityUtils.hpp>

#include <polyfem/utils/Types.hpp>

#include <vector>

namespace polyfem::solver
{
	/// @brief Form representing body forces
	class BodyForm : public Form
	{
		friend class BodyForceDerivative;

	public:
		/// @brief Construct a new Body Form object
		/// @param state Reference to the simulation state
		/// @param rhs_assembler Reference to the right hand side assembler
		BodyForm(const int ndof,
				 const int n_pressure_bases,
				 const std::vector<int> &boundary_nodes,
				 const std::vector<mesh::LocalBoundary> &local_boundary,
				 const std::vector<mesh::LocalBoundary> &local_neumann_boundary,
				 const QuadratureOrders &n_boundary_samples,
				 const Eigen::MatrixXd &rhs,
				 const assembler::RhsAssembler &rhs_assembler,
				 const assembler::Density &density,
				 const bool is_formulation_mixed,
				 const bool is_time_dependent);

		std::string name() const override { return "body"; }

	protected:
		/// @brief Compute the value of the body force form
		/// @param x Current solution
		/// @return Value of the body force form
		double value_unweighted(const Eigen::VectorXd &x) const override;

		/// @brief Compute the first derivative of the value wrt x
		/// @param[in] x Vector containing the current solution
		/// @param[out] gradv Output gradient of the value wrt x
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const override;

		/// @brief Compute the second derivative of the value wrt x
		/// @param[in] x Current solution
		/// @param[out] hessian Output Hessian of the value wrt x
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const override;

	public:
		/// @brief Update time dependent quantities
		/// @param t New time
		/// @param x Solution at time t
		void update_quantities(const double t, const Eigen::VectorXd &x) override;
		/// Physical load for an output frame even when the solve cache is at the next step.
		Eigen::VectorXd force_at_output_time(double time, const Eigen::VectorXd &x) const;
		std::unique_ptr<FormState> save_state() const override;
		void restore_state(const FormState &state, const Eigen::VectorXd &x) override;

		void hessian_wrt_u_prev(
			const Eigen::VectorXd &u_prev,
			const double t,
			StiffnessMatrix &hessian) const;

	private:
		struct State : FormState
		{
			double time;
			Eigen::MatrixXd previous_solution;
			Eigen::MatrixXd rhs;
			Eigen::MatrixXd energy_rhs;
			bool energy_rhs_initialized;
		};

		double t_;       ///< Current time
		const int ndof_; ///< Number of degrees of freedom
		const int n_pressure_bases_;

		Eigen::MatrixXd x_prev_; ///< Cached previous solution

		const std::vector<int> &boundary_nodes_;
		const std::vector<mesh::LocalBoundary> &local_boundary_;
		const std::vector<mesh::LocalBoundary> &local_neumann_boundary_;
		const QuadratureOrders n_boundary_samples_;

		const Eigen::MatrixXd &rhs_;                   ///< static RHS for the current time
		const assembler::RhsAssembler &rhs_assembler_; ///< Reference to the RHS assembler
		const assembler::Density &density_;

		bool is_formulation_mixed_; ///< True if the formulation is mixed

		Eigen::MatrixXd current_rhs_; ///< Cached RHS for the current time
		Eigen::MatrixXd energy_rhs_;  ///< Loads for the energy at t_, including lagged boundary normals
		bool energy_rhs_initialized_ = false;

		/// @brief Update current_rhs
		void update_current_rhs(const Eigen::VectorXd &x);
		void assemble_force(double time, const Eigen::VectorXd &x, Eigen::MatrixXd &force) const;
	};
} // namespace polyfem::solver
