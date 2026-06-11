#include "ALSolver.hpp"

#include <polyfem/utils/Logger.hpp>

#include <cmath>

namespace polyfem::solver
{
	ALSolver::ALSolver(
		const std::vector<std::shared_ptr<AugmentedLagrangianForm>> &alagr_form,
		const double initial_al_weight,
		const double scaling,
		const double max_al_weight,
		const double eta_tol,
		const std::function<void(const Eigen::VectorXd &)> &update_barrier_stiffness,
		const StallRestartOptions &stall_opts,
		const std::function<void(const Eigen::VectorXd &)> &on_stall)
		: alagr_forms{alagr_form},
		  initial_al_weight(initial_al_weight),
		  scaling(scaling),
		  max_al_weight(max_al_weight),
		  eta_tol(eta_tol),
		  update_barrier_stiffness(update_barrier_stiffness),
		  stall_opts(stall_opts),
		  on_stall(on_stall)
	{
	}

	void ALSolver::minimize_with_stall_restarts(
		NLProblem &nl_problem,
		Eigen::VectorXd &tmp_sol,
		const json &nl_solver_params,
		const json &linear_solver,
		const double characteristic_length,
		const std::shared_ptr<NLSolver> &nl_solverin)
	{
		const bool detect_stalls = stall_opts.enabled && on_stall != nullptr;

		int restarts = 0;
		while (true)
		{
			bool stalled = false;
			int stall_count = 0;

			const auto scale = nl_problem.normalize_forms();
			auto nl_solver = nl_solverin == nullptr ? polysolve::nonlinear::Solver::create(
														  nl_solver_params, linear_solver, characteristic_length * scale, logger())
													: nl_solverin;

			if (detect_stalls)
			{
				nl_solver->set_iteration_callback([&](const polysolve::nonlinear::Criteria &crit) -> bool {
					if (int(crit.iterations) < stall_opts.min_iterations)
						return false;

					if (std::isfinite(crit.alpha) && crit.alpha < stall_opts.alpha_threshold)
						++stall_count;
					else
						stall_count = 0;

					stalled = stall_count >= stall_opts.patience
							  || (stall_opts.soft_iteration_limit > 0
								  && int(crit.iterations) >= stall_opts.soft_iteration_limit);
					return stalled;
				});
			}

			try
			{
				nl_solver->minimize(nl_problem, tmp_sol);
				nl_problem.finish();
			}
			catch (...)
			{
				// nl_solverin may be shared with later solves
				nl_solver->set_iteration_callback(nullptr);
				throw;
			}
			nl_solver->set_iteration_callback(nullptr);

			if (!stalled)
				return;

			if (restarts >= stall_opts.max_restarts)
			{
				logger().warn(
					"Line-search stall persisted after {} restart(s); continuing with the current solution",
					restarts);
				return;
			}

			++restarts;
			// Identity when the problem is in full size
			const Eigen::VectorXd full_sol = nl_problem.reduced_to_full(tmp_sol);
			logger().warn(
				"Line-search stall detected (alpha < {:g} for {} iterations); retuning barrier stiffness and restarting ({}/{})",
				stall_opts.alpha_threshold, stall_opts.patience, restarts, stall_opts.max_restarts);

			// on_stall is responsible for retuning the barrier stiffness at
			// full_sol (the update_barrier_stiffness callback may capture a
			// stale solution vector, so it is NOT called here).
			on_stall(full_sol);
			nl_problem.init(full_sol);
			tmp_sol = nl_problem.full_to_reduced(full_sol);
		}
	}

	void ALSolver::solve_al(NLProblem &nl_problem, Eigen::MatrixXd &sol,
							const json &nl_solver_params,
							const json &linear_solver,
							const double characteristic_length,
							std::shared_ptr<polysolve::nonlinear::Solver> nl_solverin)
	{
		assert(sol.size() == nl_problem.full_size());

		const Eigen::VectorXd initial_sol = sol;
		Eigen::VectorXd tmp_sol = nl_problem.full_to_reduced(sol);
		assert(tmp_sol.size() == nl_problem.reduced_size());

		// --------------------------------------------------------------------

		double al_weight = initial_al_weight;
		int al_steps = 0;

		double initial_error = 0;
		for (const auto &f : alagr_forms)
			initial_error += f->compute_error(sol);

		nl_problem.use_reduced_size();
		nl_problem.line_search_begin(sol, tmp_sol);

		for (auto &f : alagr_forms)
			f->set_initial_weight(al_weight);

		double current_error = 0;
		for (const auto &f : alagr_forms)
			current_error += f->compute_error(sol);

		logger().debug("Initial error = {}", current_error);

		while (!std::isfinite(nl_problem.value(tmp_sol))
			   || !nl_problem.is_step_valid(sol, tmp_sol)
			   || !nl_problem.is_step_collision_free(sol, tmp_sol))
		{
			nl_problem.line_search_end();

			nl_problem.use_full_size();
			logger().debug("Solving AL Problem with weight {}", al_weight);

			nl_problem.init(sol);
			update_barrier_stiffness(sol);
			tmp_sol = sol;

			try
			{
				minimize_with_stall_restarts(
					nl_problem, tmp_sol, nl_solver_params, linear_solver,
					characteristic_length, nl_solverin);
			}
			catch (const std::runtime_error &e)
			{
				std::string err_msg = e.what();
				// if the nonlinear solve fails due to invalid energy at the current solution, changing the weights would not help
				if (err_msg.find("f(x) is nan or inf; stopping") != std::string::npos)
					log_and_throw_error("Failed to solve with AL; f(x) is nan or inf");
				if (err_msg.find("Reached iteration limit") != std::string::npos)
					log_and_throw_error("Reached iteration limit in AL");
			}

			sol = tmp_sol;

			current_error = 0;
			for (const auto &f : alagr_forms)
				current_error += f->compute_error(sol);
			logger().debug("Current error = {}", current_error);
			const double eta = 1 - sqrt(current_error / initial_error);

			logger().debug("Current eta = {}", eta);

			if (eta < 0)
			{
				logger().debug("Higher error than initial, increase weight and revert to previous solution");
				sol = initial_sol;
			}

			nl_problem.use_reduced_size();
			tmp_sol = nl_problem.full_to_reduced(sol);
			nl_problem.line_search_begin(sol, tmp_sol);

			if (eta < eta_tol && al_weight < max_al_weight)
				al_weight *= scaling;

			for (auto &f : alagr_forms)
				f->update_lagrangian(sol, al_weight);

			post_subsolve(al_weight);
			++al_steps;
		}
		nl_problem.line_search_end();
	}

	void ALSolver::solve_reduced(NLProblem &nl_problem, Eigen::MatrixXd &sol,
								 const json &nl_solver_params,
								 const json &linear_solver,
								 const double characteristic_length,
								 std::shared_ptr<polysolve::nonlinear::Solver> nl_solverin)
	{
		assert(sol.size() == nl_problem.full_size());

		Eigen::VectorXd tmp_sol = nl_problem.full_to_reduced(sol);
		nl_problem.use_reduced_size();
		nl_problem.line_search_begin(sol, tmp_sol);

		if (!std::isfinite(nl_problem.value(tmp_sol))
			|| !nl_problem.is_step_valid(sol, tmp_sol)
			|| !nl_problem.is_step_collision_free(sol, tmp_sol))
			log_and_throw_error("Failed to apply constraints conditions; solve with augmented lagrangian first!");
		nl_problem.line_search_end();
		// --------------------------------------------------------------------
		// Perform one final solve with the DBC projected out

		logger().debug("Successfully applied constraints conditions; solving in reduced space");

		nl_problem.init(sol);
		update_barrier_stiffness(sol);
		try
		{
			minimize_with_stall_restarts(
				nl_problem, tmp_sol, nl_solver_params, linear_solver,
				characteristic_length, nl_solverin);
		}
		catch (const std::runtime_error &e)
		{
			sol = nl_problem.reduced_to_full(tmp_sol);
			throw e;
		}
		sol = nl_problem.reduced_to_full(tmp_sol);

		post_subsolve(0);
	}

} // namespace polyfem::solver
