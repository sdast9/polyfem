#include "ALSolver.hpp"

#include <polyfem/utils/Logger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace polyfem::solver
{
	ALBudgetOptions ALBudgetOptions::from_json(const json &al_args)
	{
		ALBudgetOptions budget;
		if (!al_args.is_object() || !al_args.contains("budget") || !al_args["budget"].is_object())
			return budget;
		const json &b = al_args["budget"];
		budget.max_passes = b.value("max_passes", budget.max_passes);
		budget.stagnation_window = b.value("stagnation_window", budget.stagnation_window);
		budget.progress_tolerance = b.value("progress_tolerance", budget.progress_tolerance);
		budget.snap_tolerance = b.value("snap_tolerance", budget.snap_tolerance);
		budget.drift_tolerance = b.value("drift_tolerance", budget.drift_tolerance);
		if (budget.max_passes < 0 || budget.stagnation_window < 0 || budget.progress_tolerance < 0
			|| budget.snap_tolerance < 0 || budget.drift_tolerance < 0)
			log_and_throw_error("solver/augmented_lagrangian/budget: max_passes, stagnation_window and the tolerances must be nonnegative");
		return budget;
	}

	json ALBudgetOptions::to_json() const
	{
		return {{"max_passes", max_passes}, {"stagnation_window", stagnation_window}, {"progress_tolerance", progress_tolerance}, {"snap_tolerance", snap_tolerance}, {"drift_tolerance", drift_tolerance}, {"enabled", enabled()}};
	}

	json ALSolver::SnapGate::to_json() const
	{
		const auto opt = [](const std::optional<bool> &v) { return v.has_value() ? json(*v) : json(nullptr); };
		return {{"finite_energy", finite}, {"valid", opt(valid)}, {"collision_free", opt(collision_free)}, {"feasible", feasible}, {"blocked_by", blocked_by.empty() ? json(nullptr) : json(blocked_by)}, {"ccd_fraction", std::isfinite(ccd_fraction) ? json(ccd_fraction) : json(nullptr)}};
	}

	ALSolver::SnapGate ALSolver::snap_gate(NLProblem &nl_problem, const Eigen::VectorXd &sol, const Eigen::VectorXd &tmp_sol) const
	{
		// The snap: from the current full-space iterate (sol) straight to the
		// prescribed values (tmp_sol in reduced coordinates). Same three checks
		// as the historical loop condition, in its order.
		SnapGate gate;
		gate.finite = std::isfinite(nl_problem.value(tmp_sol));
		if (budget_.enabled())
		{
			// Every gate for the record, plus the collision-free fraction of
			// the snap when it is blocked (unobserved: not a Newton trial).
			gate.valid = nl_problem.is_step_valid(sol, tmp_sol);
			gate.collision_free = nl_problem.is_step_collision_free(sol, tmp_sol);
			gate.feasible = gate.finite && *gate.valid && *gate.collision_free;
			if (!gate.feasible)
				gate.ccd_fraction = nl_problem.probe_step_bound(sol, tmp_sol);
		}
		else
		{
			// Historical short circuit: nothing more is evaluated once a gate
			// fails, so the later verdicts are unknown.
			if (gate.finite)
				gate.valid = nl_problem.is_step_valid(sol, tmp_sol);
			if (gate.finite && *gate.valid)
				gate.collision_free = nl_problem.is_step_collision_free(sol, tmp_sol);
			gate.feasible = gate.finite && gate.valid.value_or(false) && gate.collision_free.value_or(false);
		}
		if (!gate.finite)
			gate.blocked_by = "energy";
		else if (gate.valid.has_value() && !*gate.valid)
			gate.blocked_by = "validity";
		else if (gate.collision_free.has_value() && !*gate.collision_free)
			gate.blocked_by = "collision";
		return gate;
	}

	void ALSolver::check_budget(NLProblem &nl_problem, const int passes) const
	{
		if (!budget_.enabled() || passes < 1)
			return;
		assert(al_history_.size() == size_t(passes) + 1);
		const json &last = al_history_[passes];
		const auto number = [](const json &record, const char *key) {
			return record.contains(key) && record[key].is_number() ? record[key].get<double>() : std::numeric_limits<double>::quiet_NaN();
		};
		const auto fail = [&](const std::string &reason, const std::string &what, json details) {
			details["reason"] = reason;
			details["budget"] = budget_.to_json();
			details["passes"] = passes;
			details["weight_ceiling"] = max_al_weight;
			details["last_pass"] = last;
			details["history"] = al_history_;
			details["scope"] = "The augmented-Lagrangian stage prepares a geometrically safe snap to the prescribed values; it ended under solver/augmented_lagrangian/budget before the snap became feasible. Not a convergence claim and not a solver error: the last pass's subsolve outcome and the gate that blocked the snap are in last_pass. The failed attempt is rolled back by the step transaction (RB-06) where one is installed; nothing of the step is published.";
			nl_problem.line_search_end();
			logger().error("{}", what);
			throw ALBudgetExhausted(what, details);
		};
		const auto gate_text = [&](const json &record) {
			const json &gate = record["gate"];
			return fmt::format("snap blocked by {} (finite energy {}, valid {}, collision-free {}, CCD fraction {})",
							   gate["blocked_by"].is_string() ? gate["blocked_by"].get<std::string>() : std::string("nothing"),
							   gate["finite_energy"].dump(), gate["valid"].dump(), gate["collision_free"].dump(), gate["ccd_fraction"].dump());
		};
		if (budget_.max_passes > 0 && passes >= budget_.max_passes)
		{
			fail("pass_budget",
				 fmt::format("Augmented-Lagrangian stage exhausted its pass budget: {} passes (solver/augmented_lagrangian/budget/max_passes = {}) without a feasible snap to the prescribed values. Last pass: weight {:g} (ceiling {:g}), subsolve {} after {} iterations, BC residual {:.6g} (started at {:.6g}), {}. Read the pass history in the failure record; a residual still falling with the gates changing means the budget is too small for this step, a flat residual at the ceiling means the prescribed motion cannot be snapped from this state.",
							 passes, budget_.max_passes, number(last, "weight"), max_al_weight,
							 last["subsolve"]["outcome"].is_string() ? last["subsolve"]["outcome"].get<std::string>() : std::string("?"),
							 last["subsolve"]["iterations"].dump(), number(last, "bc_residual_carried"), number(al_history_[0], "bc_residual_carried"), gate_text(last)),
				 json::object());
		}
		const int W = budget_.stagnation_window;
		if (W <= 0 || passes < W)
			return;
		for (int j = passes - W + 1; j <= passes; ++j)
			if (!al_history_[j].value("at_ceiling", false))
				return;
		const json &ref = al_history_[passes - W];
		const double e_ref = number(ref, "bc_residual_carried"), e_now = number(last, "bc_residual_carried");
		const bool bc_progress = std::isfinite(e_ref) && std::isfinite(e_now) && e_ref > 0 && e_now <= (1 - budget_.progress_tolerance) * e_ref;
		bool gate_progress = false;
		for (const char *key : {"finite_energy", "valid", "collision_free"})
		{
			const json &before = ref["gate"][key], &after = last["gate"][key];
			if (before.is_boolean() && after.is_boolean() && !before.get<bool>() && after.get<bool>())
				gate_progress = true;
		}
		const double f_ref = number(ref["gate"], "ccd_fraction"), f_now = number(last["gate"], "ccd_fraction");
		const bool snap_progress = std::isfinite(f_ref) && std::isfinite(f_now) && f_now >= f_ref + budget_.snap_tolerance;
		const double drift = number(last, "drift_over_window"), snap_linf = std::max(number(last, "snap_linf"), number(ref, "snap_linf"));
		const bool drift_progress = std::isfinite(drift) && (snap_linf > 0 ? drift > budget_.drift_tolerance * snap_linf : drift > 0);
		if (bc_progress || gate_progress || snap_progress || drift_progress)
			return;
		fail("stagnation",
			 fmt::format("Augmented-Lagrangian stage stagnated: {} consecutive passes at the weight ceiling {:g} (passes {}-{}) made no measurable progress toward a feasible snap -- BC residual {:.6g} -> {:.6g} (relative decrease {:.3g}, progress needs >= {:g}), {} throughout (CCD fraction {} -> {}, progress needs an increase >= {:g}), iterate drift {:.3g} against the largest constrained residual {:.3g} (progress needs > {:g} of it). The prescribed motion cannot be snapped from the states this continuation reaches; the last subsolve {} after {} iterations.",
						 W, max_al_weight, passes - W + 1, passes, e_ref, e_now, e_ref > 0 ? (e_ref - e_now) / e_ref : 0.0, budget_.progress_tolerance,
						 gate_text(last), ref["gate"]["ccd_fraction"].dump(), last["gate"]["ccd_fraction"].dump(), budget_.snap_tolerance,
						 drift, snap_linf, budget_.drift_tolerance,
						 last["subsolve"]["outcome"].is_string() ? last["subsolve"]["outcome"].get<std::string>() : std::string("?"), last["subsolve"]["iterations"].dump()),
			 {{"window", W}, {"reference_pass", ref}, {"progress", {{"bc_residual", bc_progress}, {"gates", gate_progress}, {"ccd_fraction", snap_progress}, {"drift", drift_progress}}}});
	}

	ALSolver::ALSolver(
		const std::vector<std::shared_ptr<AugmentedLagrangianForm>> &alagr_form,
		const double initial_al_weight,
		const double scaling,
		const double max_al_weight,
		const double eta_tol,
		const std::function<void(const Eigen::VectorXd &)> &update_barrier_stiffness,
		const StallRestartOptions &stall_opts,
		const std::function<bool(const Eigen::VectorXd &)> &on_stall)
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

	ALSolver::SubsolveOutcome ALSolver::minimize_with_stall_restarts(
		NLProblem &nl_problem,
		Eigen::VectorXd &tmp_sol,
		const json &nl_solver_params,
		const json &linear_solver,
		const double characteristic_length,
		const std::shared_ptr<NLSolver> &nl_solverin)
	{
		const bool detect_stalls = stall_opts.enabled && on_stall != nullptr;
		solve_info_ = {{"outcome", "failed"}};

		// A restart from a wedged iterate (a contact hotspot at CCD scale)
		// cannot escape no matter how the stiffness is retuned: revert to
		// where this solve STARTED and re-solve with the accumulated
		// (retuned) stiffness so the collapse path is never walked again.
		const Eigen::VectorXd subsolve_initial_sol = tmp_sol;
		int hard_stalls = 0;

		// RB-18 F5: a restart is only worth taking when something changed. One
		// unchanged restart is still allowed (a fresh solver instance resets
		// the descent-strategy history, which can free a soft stall on its
		// own); a second consecutive unchanged stall from the same iterate
		// would re-run an identical problem, so it interrupts instead.
		int unchanged_restarts = 0, consecutive_unchanged = 0;

		int restarts = 0;
		while (true)
		{
			const Eigen::VectorXd attempt_initial_sol = detect_stalls ? tmp_sol : Eigen::VectorXd();
			bool stalled = false;
			int stall_count = 0;

			const auto scale = nl_problem.normalize_forms();
			auto nl_solver = nl_solverin == nullptr ? polysolve::nonlinear::Solver::create(
														  nl_solver_params, linear_solver, characteristic_length * scale, logger())
													: nl_solverin;

			if (direction_filter)
				nl_solver->set_direction_filter(direction_filter);

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

			bool hard_stall = false;
			try
			{
				nl_solver->minimize(nl_problem, tmp_sol);
				nl_problem.finish();

				// Preserve PolySolve's configured convergence contract. A
				// callback or iteration budget stop is not convergence, but
				// a permitted step/energy criterion must not become a new
				// gradient-only requirement (especially in the AL phase).
				using polysolve::nonlinear::Status;
				const auto status = nl_solver->status();
				// PolySolve also uses NotDescentDirection for its configured
				// negative slope tolerance. A genuinely non-descending Newton
				// direction throws above; distinguish the finite negative case.
				const double slope = nl_solver->current_criteria().xDeltaDotGrad;
				const bool slope_tolerance = status == Status::NotDescentDirection
											 && nl_solver->stop_criteria().xDeltaDotGrad < 0
											 && std::isfinite(slope) && slope < 0;
				const bool converged = slope_tolerance || status == Status::GradNormTolerance
									   || status == Status::RelGradNormTolerance
									   || (nl_solver->allow_non_grad_convergence
										   && polysolve::nonlinear::is_converged_status(status));
				solve_info_ = nl_solver->info();
				solve_info_["outcome"] = converged ? "converged" : "interrupted";
				solve_info_["termination_reason"] = slope_tolerance
														? "Configured directional-derivative tolerance reached"
														: polysolve::nonlinear::status_message(status);
				solve_info_["directional_derivative"] = slope;
				solve_info_["restarts"] = restarts;
				solve_info_["unchanged_restarts"] = unchanged_restarts;
				if (converged)
				{
					nl_solver->set_iteration_callback(nullptr);
					nl_solver->set_direction_filter(nullptr);
					return SubsolveOutcome::Converged;
				}
			}
			catch (const std::runtime_error &e)
			{
				solve_info_ = {{"outcome", "failed"}, {"error", e.what()}, {"restarts", restarts}, {"unchanged_restarts", unchanged_restarts}};
				// nl_solverin may be shared with later solves
				nl_solver->set_iteration_callback(nullptr);
				nl_solver->set_direction_filter(nullptr);

				// A line search that fails on every strategy is the terminal
				// form of a stall: the iterate is wedged (e.g. a contact
				// hotspot at CCD scale). Retuning the barrier stiffness and
				// restarting is exactly the remedy, so treat it like one
				// while restart budget remains instead of crashing.
				if (detect_stalls && restarts < stall_opts.max_restarts
					&& std::string(e.what()).find("Line search failed") != std::string::npos)
				{
					hard_stall = true;
				}
				else
					throw;
			}
			catch (...)
			{
				solve_info_["outcome"] = "failed";
				nl_solver->set_iteration_callback(nullptr);
				nl_solver->set_direction_filter(nullptr);
				throw;
			}
			nl_solver->set_iteration_callback(nullptr);
			nl_solver->set_direction_filter(nullptr);

			if (!stalled && !hard_stall)
				return SubsolveOutcome::Interrupted;

			if (restarts >= stall_opts.max_restarts)
			{
				logger().warn(
					"Line-search stall persisted after {} restart(s); subsolve interrupted (not converged)",
					restarts);
				return SubsolveOutcome::Interrupted;
			}

			++restarts;

			// A hard stall (line search failed on every strategy) means the
			// CURRENT iterate is wedged; after the first retune fails to
			// free it, revert to the subsolve's initial solution and let the
			// accumulated stiffness prevent the collapse from re-forming.
			if (hard_stall && ++hard_stalls > 1)
			{
				logger().warn(
					"Hard stall persists at the current iterate; reverting to the subsolve's initial solution (restart {}/{})",
					restarts, stall_opts.max_restarts);
				tmp_sol = subsolve_initial_sol;
			}

			// Identity when the problem is in full size
			const Eigen::VectorXd full_sol = nl_problem.reduced_to_full(tmp_sol);
			logger().warn(
				"Line-search stall detected (alpha < {:g} for {} iterations); retuning barrier stiffness and restarting ({}/{})",
				stall_opts.alpha_threshold, stall_opts.patience, restarts, stall_opts.max_restarts);

			// on_stall is responsible for retuning the barrier stiffness at
			// full_sol (the update_barrier_stiffness callback may capture a
			// stale solution vector, so it is NOT called here).
			const bool retuned = on_stall(full_sol);
			// Compare the next restart with this attempt's starting point,
			// after any hard-stall rollback. Soft interruptions may have made
			// useful progress without a retune; repeating a failed trajectory
			// that rolls back to the same start has made no restart progress.
			const bool iterate_changed = tmp_sol.size() != attempt_initial_sol.size()
										 || (tmp_sol.array() != attempt_initial_sol.array()).any();
			if (retuned || iterate_changed)
			{
				consecutive_unchanged = 0;
			}
			else
			{
				++unchanged_restarts;
				if (++consecutive_unchanged > 1)
				{
					logger().warn(
						"Line-search stall persists with no retunable contact state (restart {}/{} would repeat an identical solve); subsolve interrupted (not converged)",
						restarts, stall_opts.max_restarts);
					solve_info_["outcome"] = "interrupted";
					solve_info_["termination_reason"] = "stall persisted with no retunable contact state";
					solve_info_["restarts"] = restarts - 1;
					solve_info_["unchanged_restarts"] = unchanged_restarts;
					return SubsolveOutcome::Interrupted;
				}
				logger().warn(
					"Stall retune changed nothing (no active contact or no trim signal); restarting once with fresh solver history");
			}
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
		int consecutive_failures = 0;

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

		// RB-07: the largest constrained-DOF residual of the carried state
		// (the longest jump the snap would make), for the drift measure.
		const auto snap_linf = [&](const Eigen::VectorXd &x) {
			double linf = 0;
			for (const auto &f : alagr_forms)
			{
				const Eigen::VectorXd residual = f->constraint_matrix() * x - f->constraint_value();
				if (residual.size() > 0)
					linf = std::max(linf, residual.lpNorm<Eigen::Infinity>());
			}
			return linf;
		};
		const auto multiplier_norm = [&]() {
			double norm = 0;
			for (const auto &f : alagr_forms)
				norm += f->lagrange_multipliers().norm();
			return norm;
		};
		const auto finite_or_null = [](const double v) { return std::isfinite(v) ? json(v) : json(nullptr); };

		// Pass 0: the state the stage starts from, against which the first
		// window of the stagnation exit is judged.
		SnapGate gate = snap_gate(nl_problem, sol, tmp_sol);
		al_history_ = json::array();
		al_history_.push_back({{"pass", 0}, {"weight", al_weight}, {"at_ceiling", al_weight >= max_al_weight}, {"subsolve", nullptr}, {"bc_residual", std::sqrt(current_error)}, {"bc_residual_carried", std::sqrt(current_error)}, {"relative_progress", nullptr}, {"rolled_back", false}, {"gate", gate.to_json()}, {"snap_linf", snap_linf(sol)}, {"moved", 0.0}, {"drift_over_window", nullptr}, {"multiplier_norm", multiplier_norm()}, {"wall_seconds", 0.0}});
		std::vector<Eigen::VectorXd> carried; ///< the carried full-space state after each pass (pass 0 = start), for the drift measure
		if (budget_.enabled())
			carried.push_back(sol);

		while (!gate.feasible)
		{
			check_budget(nl_problem, al_steps); // RB-07: throws ALBudgetExhausted (opt-in; never fires at the defaults)
			const auto pass_start = std::chrono::steady_clock::now();
			const double pass_weight = al_weight;
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
				// AL only prepares a feasible snap to the prescribed BCs.
				// An interrupted inner solve is a valid continuation iterate,
				// not a failure: adopt it and check snap feasibility below.
				consecutive_failures = 0;
			}
			catch (const std::runtime_error &e)
			{
				std::string err_msg = e.what();
				// if the nonlinear solve fails due to invalid energy at the current solution, changing the weights would not help
				if (err_msg.find("f(x) is nan or inf; stopping") != std::string::npos)
					log_and_throw_error("Failed to solve with AL; f(x) is nan or inf");
				if (err_msg.find("Reached iteration limit") != std::string::npos)
					log_and_throw_error("Reached iteration limit in AL");

				// Otherwise continuing with a scaled weight is a legitimate
				// retry -- but only finitely often: an iterate the solver
				// cannot move from at ANY weight would loop forever here
				// (each retry doubles the weight and grants a fresh restart
				// budget).
				if (++consecutive_failures >= 3)
					log_and_throw_error(
						"AL subsolve failed {} times in a row ({}); giving up",
						consecutive_failures, err_msg);
				logger().warn("{}", err_msg);
			}

			sol = tmp_sol;

			current_error = 0;
			for (const auto &f : alagr_forms)
				current_error += f->compute_error(sol);
			logger().debug("Current error = {}", current_error);
			// Zero initial residual has no relative progress to measure.
			// Preserve rollback if it grows; otherwise allow penalty continuation
			// while the separate geometric snap checks remain unsatisfied.
			const double eta = initial_error > 0
								   ? 1 - std::sqrt(current_error) / std::sqrt(initial_error)
								   : (current_error > 0 ? -1.0 : 0.0);

			logger().debug("Current eta = {}", eta);

			bool rolled_back = false;
			if (eta < 0)
			{
				logger().debug("Higher error than initial, increase weight and revert to previous solution");
				sol = initial_sol;
				rolled_back = true;
			}

			nl_problem.use_reduced_size();
			tmp_sol = nl_problem.full_to_reduced(sol);
			nl_problem.line_search_begin(sol, tmp_sol);

			if (eta < eta_tol && al_weight < max_al_weight)
				al_weight = std::min(al_weight * scaling, max_al_weight);

			for (auto &f : alagr_forms)
				f->update_lagrangian(sol, al_weight);

			// RB-07: the snap's gates after this pass (the loop condition),
			// recorded with the pass. Same evaluations as before in the same
			// place of the sequence: after the multiplier update, before the
			// subsolve callback.
			gate = snap_gate(nl_problem, sol, tmp_sol);
			++al_steps;
			double moved = 0, drift = std::numeric_limits<double>::quiet_NaN();
			if (budget_.enabled())
			{
				moved = (sol - carried.back()).lpNorm<Eigen::Infinity>();
				carried.push_back(sol);
				if (budget_.stagnation_window > 0 && int(carried.size()) > budget_.stagnation_window)
					drift = (sol - carried[carried.size() - 1 - budget_.stagnation_window]).lpNorm<Eigen::Infinity>();
			}
			const double carried_error = rolled_back ? initial_error : current_error;
			json subsolve = {{"outcome", solve_info_.value("outcome", std::string("failed"))}, {"termination_reason", solve_info_.contains("termination_reason") ? solve_info_["termination_reason"] : json(nullptr)}, {"iterations", solve_info_.contains("iterations") ? solve_info_["iterations"] : json(nullptr)}, {"restarts", solve_info_.contains("restarts") ? solve_info_["restarts"] : json(nullptr)}, {"error", solve_info_.contains("error") ? solve_info_["error"] : json(nullptr)}};
			al_history_.push_back({{"pass", al_steps}, {"weight", pass_weight}, {"at_ceiling", pass_weight >= max_al_weight}, {"subsolve", subsolve}, {"bc_residual", std::sqrt(current_error)}, {"bc_residual_carried", std::sqrt(carried_error)}, {"relative_progress", finite_or_null(eta)}, {"rolled_back", rolled_back}, {"gate", gate.to_json()}, {"snap_linf", snap_linf(sol)}, {"moved", moved}, {"drift_over_window", finite_or_null(drift)}, {"multiplier_norm", multiplier_norm()}, {"wall_seconds", std::chrono::duration<double>(std::chrono::steady_clock::now() - pass_start).count()}});

			solve_info_["al_initial_error"] = initial_error;
			solve_info_["al_current_error"] = current_error;
			solve_info_["al_relative_progress"] = eta;
			solve_info_["al_next_weight"] = al_weight;
			solve_info_["al_pass"] = al_steps;
			solve_info_["al_weight"] = pass_weight;
			solve_info_["al_at_ceiling"] = pass_weight >= max_al_weight;
			solve_info_["al_bc_residual"] = std::sqrt(carried_error);
			solve_info_["al_rolled_back"] = rolled_back;
			solve_info_["al_gate"] = gate.to_json();
			solve_info_["al_moved"] = moved;
			post_subsolve(al_weight);
		}
		nl_problem.line_search_end();
		if (al_steps > 0)
			logger().debug("Augmented-Lagrangian stage: feasible snap after {} pass(es)", al_steps);
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
		if (nl_problem.reduced_size() == 0)
		{
			// Every DOF is prescribed: the reduced problem has no unknowns and
			// the snap just verified (finite energy, valid, collision-free) is
			// the step's solution. Stated here rather than handed to the
			// nonlinear solver, whose outcome on an empty problem would depend
			// on the configuration (a zero first_grad_norm_tol sends it into a
			// Newton step on a 0x0 system, an Linf norm type into maxCoeff of
			// an empty gradient). Same observable sequence as a solve that
			// converges at its start: solution_changed, the iteration-0
			// post_step (the forms and the RB-04 attempt stream), finish.
			nl_problem.solution_changed(tmp_sol);
			const double energy = nl_problem.value(tmp_sol);
			solve_info_ = {{"outcome", "converged"}, {"termination_reason", "No free degrees of freedom: every DOF is prescribed, the reduced problem is empty and the solution is the prescribed values"}, {"iterations", 0}, {"energy", energy}, {"gradNorm", 0.0}, {"directional_derivative", nullptr}, {"restarts", 0}, {"unchanged_restarts", 0}};
			nl_problem.post_step(polysolve::nonlinear::PostStepData(0, solve_info_, tmp_sol, Eigen::VectorXd::Zero(0)));
			nl_problem.finish();
			logger().info("No free degrees of freedom: every DOF is prescribed; the step's solution is the prescribed values (energy {:g})", energy);
		}
		else
		{
			// Keep the caller's solution unchanged on interruption or failure.
			const auto outcome = minimize_with_stall_restarts(
				nl_problem, tmp_sol, nl_solver_params, linear_solver,
				characteristic_length, nl_solverin);
			if (outcome != SubsolveOutcome::Converged)
				log_and_throw_error("Final reduced solve did not converge: {}", solve_info_.dump());
		}
		sol = nl_problem.reduced_to_full(tmp_sol);

		post_subsolve(0);
	}

} // namespace polyfem::solver
