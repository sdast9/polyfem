#pragma once

#include <polyfem/solver/NLProblem.hpp>
#include <polysolve/nonlinear/Solver.hpp>
#include <polyfem/solver/forms/lagrangian/AugmentedLagrangianForm.hpp>
#include <polyfem/Common.hpp>

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace polyfem::solver
{
	/// @brief Options for detecting line-search stalls and restarting the
	///        nonlinear solve with retuned barrier stiffness.
	struct StallRestartOptions
	{
		bool enabled = false;
		double alpha_threshold = 1e-4; ///< Line-search alphas below this count towards a stall
		int patience = 5;              ///< Consecutive small-alpha iterations before a stall
		int min_iterations = 5;        ///< Do not judge stalls before this many iterations
		int soft_iteration_limit = -1; ///< Restart after this many iterations (-1 to disable)
		int max_restarts = 5;          ///< Maximum number of restarts per solve
	};

	class ALSolver
	{
		using NLSolver = polysolve::nonlinear::Solver;

	public:
		ALSolver(
			const std::vector<std::shared_ptr<AugmentedLagrangianForm>> &alagr_form,
			const double initial_al_weight,
			const double scaling,
			const double max_al_weight,
			const double eta_tol,
			const std::function<void(const Eigen::VectorXd &)> &update_barrier_stiffness,
			const StallRestartOptions &stall_opts = StallRestartOptions(),
			const std::function<void(const Eigen::VectorXd &)> &on_stall = nullptr);
		virtual ~ALSolver() = default;

		void solve_al(NLProblem &nl_problem, Eigen::MatrixXd &sol,
					  std::shared_ptr<polysolve::nonlinear::Solver> nl_solver)
		{
			solve_al(nl_problem, sol, json{}, json{}, 1, nl_solver);
		}

		void solve_al(NLProblem &nl_problem, Eigen::MatrixXd &sol,
					  const json &nl_solver_params,
					  const json &linear_solver,
					  const double characteristic_length,
					  std::shared_ptr<polysolve::nonlinear::Solver> nl_solver = nullptr);

		void solve_reduced(NLProblem &nl_problem, Eigen::MatrixXd &sol,
						   std::shared_ptr<polysolve::nonlinear::Solver> nl_solver)
		{
			solve_al(nl_problem, sol, json{}, json{}, 1, nl_solver);
		}

		void solve_reduced(NLProblem &nl_problem, Eigen::MatrixXd &sol,
						   const json &nl_solver_params,
						   const json &linear_solver,
						   const double characteristic_length,
						   std::shared_ptr<polysolve::nonlinear::Solver> nl_solver = nullptr);

		std::function<void(const double)> post_subsolve = [](const double) {};

	protected:
		/// @brief Run nl_solver->minimize with stall detection; on a stall,
		///        retune via on_stall and restart (bounded by max_restarts).
		/// @return True if the final minimize ended without a stall request.
		void minimize_with_stall_restarts(
			NLProblem &nl_problem,
			Eigen::VectorXd &tmp_sol,
			const json &nl_solver_params,
			const json &linear_solver,
			const double characteristic_length,
			const std::shared_ptr<NLSolver> &nl_solverin);

		std::vector<std::shared_ptr<AugmentedLagrangianForm>> alagr_forms;
		const double initial_al_weight;
		const double scaling;
		const double max_al_weight;
		const double eta_tol;

		// TODO: replace this with a member function
		std::function<void(const Eigen::VectorXd &)> update_barrier_stiffness;

		/// @brief Stall detection and restart options
		const StallRestartOptions stall_opts;
		/// @brief Called with the current (full-size) solution when a stall
		///        is detected, before restarting the solve.
		std::function<void(const Eigen::VectorXd &)> on_stall;
	};
} // namespace polyfem::solver