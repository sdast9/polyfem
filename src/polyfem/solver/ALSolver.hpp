#pragma once

#include <polyfem/solver/NLProblem.hpp>
#include <polysolve/nonlinear/Solver.hpp>
#include <polyfem/solver/forms/lagrangian/AugmentedLagrangianForm.hpp>
#include <polyfem/Common.hpp>

#include <Eigen/Core>

#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace polyfem::solver
{
	/// @brief Options for detecting line-search stalls and restarting the
	///        nonlinear solve with retuned barrier stiffness.
	struct StallRestartOptions
	{
		/// @brief EF-04: what a small alpha is measured against. Absolute (the
		///        historical trigger): every accepted alpha below the threshold
		///        counts. FeasibleBound: a small alpha counts only when the line
		///        search backtracked below the bound the problem set (finite
		///        energy, CCD and the trial-displacement cap), i.e. accepted /
		///        feasible < feasible_ratio_threshold; a step accepted at that
		///        bound resets the count like a large one.
		enum class AlphaBasis
		{
			Absolute,
			FeasibleBound
		};

		bool enabled = false;
		double alpha_threshold = 1e-4; ///< Line-search alphas below this count towards a stall
		int patience = 5;              ///< Consecutive small-alpha iterations before a stall
		int min_iterations = 5;        ///< Do not judge stalls before this many iterations
		int soft_iteration_limit = -1; ///< Restart after this many iterations (-1 to disable)
		int max_restarts = 5;          ///< Maximum number of restarts per solve
		AlphaBasis alpha_basis = AlphaBasis::Absolute;
		double feasible_ratio_threshold = 0.999; ///< FeasibleBound: accepted/feasible alpha below this counts

		/// @brief Reads solver/contact/semi_implicit/restart (spec-completed).
		static StallRestartOptions from_json(const json &restart);
		static std::string alpha_basis_name(AlphaBasis basis);
	};

	/// @brief RB-07: opt-in bounds on the augmented-Lagrangian feasibility
	///        preparation (solver/augmented_lagrangian/budget). Both exits are
	///        off by default, which is the historical unbounded loop; when one
	///        fires the stage ends with ALBudgetExhausted, a named failure that
	///        is never a convergence claim. The pass cap is a plain budget; the
	///        stagnation exit needs a window of passes at the weight ceiling
	///        (no continuation left) over which none of the measured progress
	///        signals moved: the BC residual, the snap's feasibility gates, the
	///        collision-free fraction of the snap and the iterate itself.
	struct ALBudgetOptions
	{
		int max_passes = 0;               ///< AL passes (subsolves) per solve; 0 = unbounded
		int stagnation_window = 0;        ///< Consecutive passes at the weight ceiling without progress that end the stage; 0 = never
		double progress_tolerance = 1e-2; ///< Relative decrease of the BC residual norm over the window that counts as progress (dimensionless)
		double snap_tolerance = 1e-2;     ///< Increase of the collision-free fraction of the snap over the window that counts as progress (fraction of the snap)
		double drift_tolerance = 1e-2;    ///< Drift of the iterate over the window, relative to the largest constrained-DOF residual, that counts as progress (dimensionless)
		bool enabled() const { return max_passes > 0 || stagnation_window > 0; }
		/// @brief Reads al_args["budget"] when present; every key optional.
		static ALBudgetOptions from_json(const json &al_args);
		json to_json() const;
	};

	/// @brief RB-07: the AL stage ended under its configured budget. Carries
	///        the structured reason (pass history, the reference and last pass
	///        records, tolerances) for the failure record and the manifest.
	class ALBudgetExhausted : public std::runtime_error
	{
	public:
		ALBudgetExhausted(const std::string &what, json details)
			: std::runtime_error(what), details_(std::move(details)) {}
		const json &details() const { return details_; }

	private:
		json details_;
	};

	class ALSolver
	{
		using NLSolver = polysolve::nonlinear::Solver;

	public:
		// Converged means the configured PolySolve criteria, not an extra
		// gradient-only test. Interrupted is usable by AL continuation.
		enum class SubsolveOutcome
		{
			Converged,
			Interrupted
		};

		/// Diagnostics for the actual last inner solve, including failures.
		const json &info() const { return solve_info_; }
		ALSolver(
			const std::vector<std::shared_ptr<AugmentedLagrangianForm>> &alagr_form,
			const double initial_al_weight,
			const double scaling,
			const double max_al_weight,
			const double eta_tol,
			const std::function<void(const Eigen::VectorXd &)> &update_barrier_stiffness,
			const StallRestartOptions &stall_opts = StallRestartOptions(),
			const std::function<bool(const Eigen::VectorXd &)> &on_stall = nullptr);
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
			solve_reduced(nl_problem, sol, json{}, json{}, 1, nl_solver);
		}

		void solve_reduced(NLProblem &nl_problem, Eigen::MatrixXd &sol,
						   const json &nl_solver_params,
						   const json &linear_solver,
						   const double characteristic_length,
						   std::shared_ptr<polysolve::nonlinear::Solver> nl_solver = nullptr);

		std::function<void(const double)> post_subsolve = [](const double) {};

		/// @brief RB-07: install the opt-in AL budget (default: none).
		void set_budget(const ALBudgetOptions &budget) { budget_ = budget; }
		const ALBudgetOptions &budget() const { return budget_; }
		/// @brief RB-07: one record per AL pass of the last solve_al call
		///        (pass 0 is the state the stage started from), also carried
		///        by info()["al_history"] when the stage fails.
		const json &al_history() const { return al_history_; }
		/// @brief RBR-03: the full-space states solve_al currently keeps for
		///        its motion measures (`moved`, `drift_over_window`): none
		///        outside the stage or without a budget, one under a pass cap
		///        alone, at most stagnation_window + 1 under a window, whatever
		///        the pass count. Each pass record carries it as
		///        `retained_states`.
		size_t retained_state_count() const { return carried_.size(); }

		/// @brief Optional filter applied to every Newton update direction
		///        (installed on the nonlinear solver for each subsolve).
		///        The objective derivative remains gradient.dot(direction).
		std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> direction_filter = nullptr;

	protected:
		/// @brief Run nl_solver->minimize with stall detection; on a stall,
		///        retune via on_stall and restart (bounded by max_restarts).
		/// @return Interrupted iterates are continuation states only. Hard
		/// failures throw; a final reduced solve requires Converged.
		SubsolveOutcome minimize_with_stall_restarts(
			NLProblem &nl_problem,
			Eigen::VectorXd &tmp_sol,
			const json &nl_solver_params,
			const json &linear_solver,
			const double characteristic_length,
			const std::shared_ptr<NLSolver> &nl_solverin);

		std::vector<std::shared_ptr<AugmentedLagrangianForm>> alagr_forms;
		json solve_info_ = json::object();
		const double initial_al_weight;
		const double scaling;
		const double max_al_weight;
		const double eta_tol;

		// TODO: replace this with a member function
		std::function<void(const Eigen::VectorXd &)> update_barrier_stiffness;

		/// @brief RB-07: the three feasibility gates of the snap from the
		///        current full-space iterate to the prescribed values, the
		///        collision-free fraction of that snap and the gate that
		///        blocked it. Without a budget the gates are evaluated with
		///        the historical short circuit (a later gate is not evaluated
		///        once an earlier one fails, and reads as unknown); with a
		///        budget every gate is evaluated and the fraction probed.
		struct SnapGate
		{
			bool finite = false;
			std::optional<bool> valid, collision_free;
			bool feasible = false;
			double ccd_fraction = std::numeric_limits<double>::quiet_NaN();
			std::string blocked_by; ///< "", "energy", "validity" or "collision"
			json to_json() const;
		};
		SnapGate snap_gate(NLProblem &nl_problem, const Eigen::VectorXd &sol, const Eigen::VectorXd &tmp_sol) const;
		/// @brief RB-07: throws ALBudgetExhausted when the completed passes
		///        exhaust the pass cap or form a stagnant window.
		void check_budget(NLProblem &nl_problem, const int passes) const;

		ALBudgetOptions budget_;
		json al_history_ = json::array();
		/// @brief RBR-03: the carried full-space states of the passes the
		///        motion measures still need -- the previous pass for `moved`
		///        and the pass stagnation_window back for `drift_over_window`
		///        -- bounded by the window and released when solve_al ends.
		///        Pass 0 (the state the stage started from) is a real entry.
		///        The scalar pass records stay complete; only vectors are
		///        bounded.
		std::deque<Eigen::VectorXd> carried_;

		/// @brief Stall detection and restart options
		const StallRestartOptions stall_opts;
		/// @brief Called with the current (full-size) solution when a stall
		///        is detected, before restarting the solve. Returns whether it
		///        changed anything (coefficients or trim). RB-18 F5: a second
		///        consecutive stall from the same iterate with nothing changed
		///        interrupts the subsolve instead of repeating identical restarts.
		std::function<bool(const Eigen::VectorXd &)> on_stall;
	};
} // namespace polyfem::solver
