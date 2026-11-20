#include "BodyForm.hpp"

#include <vector>

namespace polyfem::solver
{
	BodyForm::BodyForm(const int ndof,
					   const int n_pressure_bases,
					   const std::vector<int> &boundary_nodes,
					   const std::vector<mesh::LocalBoundary> &local_boundary,
					   const std::vector<mesh::LocalBoundary> &local_neumann_boundary,
					   const QuadratureOrders &n_boundary_samples,
					   const Eigen::MatrixXd &rhs,
					   const assembler::RhsAssembler &rhs_assembler,
					   const assembler::Density &density,
					   const bool is_formulation_mixed,
					   const bool is_time_dependent)
		: ndof_(ndof),
		  n_pressure_bases_(n_pressure_bases),
		  boundary_nodes_(boundary_nodes),
		  local_boundary_(local_boundary),
		  local_neumann_boundary_(local_neumann_boundary),
		  n_boundary_samples_(n_boundary_samples),
		  rhs_(rhs),
		  rhs_assembler_(rhs_assembler),
		  density_(density),
		  is_formulation_mixed_(is_formulation_mixed)
	{
		t_ = 0;
		if (!is_time_dependent)
			update_current_rhs(Eigen::VectorXd());
	}

	double BodyForm::value_unweighted(const Eigen::VectorXd &x) const
	{
		if (!energy_rhs_initialized_)
			return rhs_assembler_.compute_energy(x, x_prev_, local_neumann_boundary_, density_, n_boundary_samples_, t_);
		// Loads depend on time and the lagged displacement, not the line-search
		// iterate. update_current_rhs already assembles them with those quantities.
		if (energy_rhs_.size() == 0)
			return 0;
		return rhs_assembler_.compute_energy_from_rhs(x, energy_rhs_);
	}

	void BodyForm::first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &gradv) const
	{
		// REMEMBER -!!!!!
		gradv = -current_rhs_;
	}

	void BodyForm::second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &hessian) const
	{
		hessian.resize(x.size(), x.size());
	}

	void BodyForm::update_quantities(const double t, const Eigen::VectorXd &x)
	{
		this->t_ = t;
		this->x_prev_ = x;
		update_current_rhs(x);
	}

	std::unique_ptr<FormState> BodyForm::save_state() const
	{
		auto state = std::make_unique<State>();
		save_base_state(*state);
		state->time = t_;
		state->previous_solution = x_prev_;
		state->rhs = current_rhs_;
		state->energy_rhs = energy_rhs_;
		state->energy_rhs_initialized = energy_rhs_initialized_;
		return state;
	}

	void BodyForm::restore_state(const FormState &state, const Eigen::VectorXd &x)
	{
		const auto &body = state_as<State>(state, "BodyForm");
		Form::restore_state(body, x);
		t_ = body.time;
		x_prev_ = body.previous_solution;
		current_rhs_ = body.rhs;
		energy_rhs_ = body.energy_rhs;
		energy_rhs_initialized_ = body.energy_rhs_initialized;
	}

	void BodyForm::update_current_rhs(const Eigen::VectorXd &x)
	{
		rhs_assembler_.compute_energy_grad(
			local_boundary_, boundary_nodes_, density_,
			n_boundary_samples_, local_neumann_boundary_,
			rhs_, t_, current_rhs_);

		if (is_formulation_mixed_ && current_rhs_.size() < ndof_)
		{
			current_rhs_.conservativeResize(
				current_rhs_.rows() + n_pressure_bases_, current_rhs_.cols());
			current_rhs_.bottomRows(n_pressure_bases_).setZero();
		}

		// Apply Neumann boundary conditions
		rhs_assembler_.set_bc(
			std::vector<mesh::LocalBoundary>(), std::vector<int>(),
			n_boundary_samples_, local_neumann_boundary_,
			current_rhs_, x, t_);
		if (rhs_assembler_.problem().is_constant_in_time() && rhs_assembler_.is_rhs_time_dependent(density_))
		{
			// The gradient's supplied static RHS may have been assembled at a
			// different time. Match the energy's density/load evaluation at t_.
			rhs_assembler_.assemble(density_, energy_rhs_, t_);
			energy_rhs_ *= -1;
			rhs_assembler_.set_bc(
				std::vector<mesh::LocalBoundary>(), std::vector<int>(),
				n_boundary_samples_, local_neumann_boundary_, energy_rhs_, x, t_);
		}
		else
			energy_rhs_ = current_rhs_;
		// Preserve the constant-time energy evaluation for unloaded problems.
		if (energy_rhs_.isZero(0))
			energy_rhs_.resize(0, 1);
		energy_rhs_initialized_ = true;
	}

	void BodyForm::hessian_wrt_u_prev(const Eigen::VectorXd &u_prev, const double t, StiffnessMatrix &hessian) const
	{
		rhs_assembler_.compute_energy_hess(boundary_nodes_, n_boundary_samples_, local_neumann_boundary_, u_prev, t, false, hessian);
		hessian *= -1;
	}
} // namespace polyfem::solver
