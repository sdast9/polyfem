#include "ALSolver.hpp"

#include <polyfem/utils/Logger.hpp>

namespace polyfem::solver
{
	ALSolver::ALSolver(
		std::shared_ptr<LagrangianForm> lagr_form,
		std::shared_ptr<LagrangianPenaltyForm> pen_form,
		const double initial_al_weight,
		const double scaling,
		const double max_al_weight,
		const double eta_tol,
		const std::function<void(const Eigen::VectorXd &)> &update_barrier_stiffness)
		: lagr_form(lagr_form),
		  pen_form(pen_form),
		  initial_al_weight(initial_al_weight),
		  scaling(scaling),
		  max_al_weight(max_al_weight),
		  eta_tol(eta_tol),
		  update_barrier_stiffness(update_barrier_stiffness)
	{
	}

	void ALSolver::solve_al(std::shared_ptr<NLSolver> nl_solver, NLProblem &nl_problem, Eigen::MatrixXd &sol)
	{
		assert(sol.size() == nl_problem.full_size());

		const Eigen::VectorXd initial_sol = sol;
		Eigen::VectorXd tmp_sol = nl_problem.full_to_reduced(sol);
		assert(tmp_sol.size() == nl_problem.reduced_size());

		// --------------------------------------------------------------------

		double al_weight = initial_al_weight;
		int al_steps = 0;
		const int iters = nl_solver->stop_criteria().iterations;

		const double initial_error = pen_form->compute_error(sol);

		nl_problem.line_search_begin(sol, tmp_sol);

		while (!std::isfinite(nl_problem.value(tmp_sol))
			   || !nl_problem.is_step_valid(sol, tmp_sol)
			   || !nl_problem.is_step_collision_free(sol, tmp_sol))
		{
			nl_problem.line_search_end();

			set_al_weight(nl_problem, sol, al_weight);
			logger().debug("Solving AL Problem with weight {}", al_weight);

			nl_problem.init(sol);
			update_barrier_stiffness(sol);
			tmp_sol = sol;

			try
			{
				nl_solver->minimize(nl_problem, tmp_sol);
			}
			catch (const std::runtime_error &e)
			{
			}

			sol = tmp_sol;
			set_al_weight(nl_problem, sol, -1);

			const double current_error = pen_form->compute_error(sol);
			const double eta = 1 - sqrt(current_error / initial_error);

			logger().debug("Current eta = {}", eta);

			if (eta < 0)
			{
				logger().debug("Higher error than initial, increase weight and revert to previous solution");
				sol = initial_sol;
			}

			tmp_sol = nl_problem.full_to_reduced(sol);
			nl_problem.line_search_begin(sol, tmp_sol);

			if (eta < eta_tol && al_weight < max_al_weight)
				al_weight *= scaling;
			else
				lagr_form->update_lagrangian(sol, al_weight);

			post_subsolve(al_weight);
			++al_steps;
		}
		nl_problem.line_search_end();
		nl_solver->stop_criteria().iterations = iters;
	}

	void ALSolver::solve_reduced(std::shared_ptr<NLSolver> nl_solver, NLProblem &nl_problem, Eigen::MatrixXd &sol)
	{
		assert(sol.size() == nl_problem.full_size());

		Eigen::VectorXd tmp_sol = nl_problem.full_to_reduced(sol);
		nl_problem.line_search_begin(sol, tmp_sol);

		if (!std::isfinite(nl_problem.value(tmp_sol))
			|| !nl_problem.is_step_valid(sol, tmp_sol)
			|| !nl_problem.is_step_collision_free(sol, tmp_sol))
			log_and_throw_error("Failed to apply boundary conditions; solve with augmented lagrangian first!");

		// --------------------------------------------------------------------
		// Perform one final solve with the DBC projected out

		logger().debug("Successfully applied boundary conditions; solving in reduced space");

		nl_problem.init(sol);
		update_barrier_stiffness(sol);
		try
		{
			nl_solver->minimize(nl_problem, tmp_sol);
		}
		catch (const std::runtime_error &e)
		{
			sol = nl_problem.reduced_to_full(tmp_sol);
			throw e;
		}
		sol = nl_problem.reduced_to_full(tmp_sol);

		post_subsolve(0);
	}

	void ALSolver::set_al_weight(NLProblem &nl_problem, const Eigen::VectorXd &x, const double weight)
	{
		if (pen_form == nullptr || lagr_form == nullptr)
			return;
		if (weight > 0)
		{
			pen_form->enable();
			lagr_form->enable();
			pen_form->set_weight(weight);
			nl_problem.use_full_size();
			nl_problem.set_apply_DBC(x, false);
		}
		else
		{
			pen_form->disable();
			lagr_form->disable();
			nl_problem.use_reduced_size();
			nl_problem.set_apply_DBC(x, true);
		}
	}

} // namespace polyfem::solver