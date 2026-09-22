#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polysolve/linear/Solver.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Matchers::ContainsSubstring;

namespace
{
	class Quartic : public Form
	{
	public:
		std::string name() const override { return "quartic"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return 0.25 * std::pow(x[0], 4); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override
		{
			g = Eigen::VectorXd::Constant(1, std::pow(x[0], 3));
		}
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(1, 1);
			h.coeffRef(0, 0) = 3 * x[0] * x[0];
		}
	};

	class QuarticProblem : public NLProblem
	{
	public:
		QuarticProblem() : NLProblem(1, 0, {std::make_shared<Quartic>()}, {}, nullptr, 1, 1, mass(), 1) {}
		bool block_steps = false;
		bool custom_stop = false;
		bool stop(const TVector &) override { return custom_stop; }
		double max_step_size(const TVector &x0, const TVector &x1) override
		{
			return block_steps ? 0 : NLProblem::max_step_size(x0, x1);
		}
		static StiffnessMatrix mass()
		{
			StiffnessMatrix m(1, 1);
			m.setIdentity();
			return m;
		}
	};

	json parameters()
	{
		return {{"solver", "Newton"}, {"max_iterations", 100}, {"grad_norm_tol", 1e-12}, {"rel_grad_norm_tol", 0.0}, {"first_grad_norm_tol", 0.0}, {"x_delta_tol", 0.0}, {"allow_non_grad_convergence", false}, {"line_search", {{"method", "Backtracking"}}}};
	}
	const json linear = {{"solver", "Eigen::SimplicialLDLT"}};
	StallRestartOptions restart_options(int budget)
	{
		StallRestartOptions opts;
		opts.enabled = true;
		opts.min_iterations = 0;
		opts.soft_iteration_limit = 1;
		opts.max_restarts = budget;
		return opts;
	}
} // namespace

TEST_CASE("AL final solve rejects exhausted soft restarts", "[al_solver]")
{
	for (int budget : {0, 2})
	{
		CAPTURE(budget);
		QuarticProblem problem;
		int retunes = 0, successes = 0;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(budget), [&](const auto &) { ++retunes; return true; });
		solver.post_subsolve = [&](double) { ++successes; };
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, parameters(), linear, 1), ContainsSubstring("Final reduced solve did not converge"));
		CHECK(sol(0, 0) == 10);
		CHECK(retunes == budget);
		CHECK(successes == 0);
		CHECK(solver.info()["outcome"] == "interrupted");
		CHECK(solver.info()["termination_reason"].is_string());
	}
}

TEST_CASE("AL final solve accepts actual stationarity", "[al_solver]")
{
	for (int budget : {-1, 40})
	{
		CAPTURE(budget);
		QuarticProblem problem;
		auto opts = restart_options(budget);
		opts.enabled = budget >= 0;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, opts, [](const auto &) { return true; });
		int successes = 0;
		solver.post_subsolve = [&](double) { ++successes; };
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, parameters(), linear, 1));
		CHECK(std::abs(std::pow(sol(0, 0), 3)) < 1e-12);
		CHECK(solver.info()["outcome"] == "converged");
		CHECK(successes == 1);
	}
}

TEST_CASE("AL final solve rejects other nonstationary stops", "[al_solver]")
{
	QuarticProblem problem;
	auto params = parameters();
	SECTION("objective interruption") { problem.custom_stop = true; }
	SECTION("allowed iteration limit")
	{
		params["max_iterations"] = 1;
		params["allow_out_of_iterations"] = true;
	}
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
	REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, params, linear, 1), ContainsSubstring("Final reduced solve did not converge"));
	CHECK(sol(0, 0) == 10);
}

// RB-18 F5: a stall whose retune changes nothing gets ONE restart (fresh
// solver history), and a second consecutive unchanged stall interrupts the
// subsolve instead of burning the whole restart budget on identical solves.
TEST_CASE("AL stops repeating unchanged stall restarts", "[al_solver]")
{
	SECTION("progressing soft stalls converge with nothing to retune")
	{
		QuarticProblem problem;
		int retunes = 0;
		double previous = 10;
		bool made_progress = true;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(40), [&](const auto &x) {
			++retunes;
			made_progress = made_progress && std::abs(x[0]) < previous;
			previous = std::abs(x[0]);
			return false; });
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, parameters(), linear, 1));
		CHECK(std::abs(std::pow(sol(0, 0), 3)) < 1e-12);
		CHECK(retunes > 2);
		CHECK(made_progress);
		CHECK(solver.info()["outcome"] == "converged");
	}

	SECTION("a retune that changes something resets the counter")
	{
		QuarticProblem problem;
		problem.block_steps = true; // keep the iterate fixed to isolate the retune signal
		int retunes = 0;
		// no, yes, no, no -> interrupted at the fourth (second consecutive no)
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(20), [&](const auto &) { return (++retunes) == 2; });
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, parameters(), linear, 1), ContainsSubstring("Final reduced solve did not converge"));
		CHECK(retunes == 4);
		CHECK(solver.info()["unchanged_restarts"] == 3);
		CHECK(solver.info()["termination_reason"] == "stall persisted with no retunable contact state");
	}

	SECTION("hard stalls still revert the iterate before giving up")
	{
		QuarticProblem problem;
		problem.block_steps = true;
		int retunes = 0;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(20), [&](const auto &) { ++retunes; return false; });
		solver.direction_filter = [](const auto &, auto &) {};
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, parameters(), linear, 1), ContainsSubstring("Final reduced solve did not converge"));
		// The blocked line search never moves the iterate, so the revert on
		// the second hard stall changes nothing either: two unchanged stalls.
		CHECK(retunes == 2);
		CHECK(solver.info()["outcome"] == "interrupted");
		CHECK(solver.info()["termination_reason"] == "stall persisted with no retunable contact state");
	}
}

// BFGS audit stage 4: the stall callback has two independent triggers and the
// restart message named only the first, so a solve the iteration budget
// interrupted was reported -- and read -- as an alpha collapse. The trigger is
// now recorded. Reporting only; no restart policy changed.
TEST_CASE("AL records which condition actually triggered a stall restart", "[al_solver][stall_trigger]")
{
	const auto solve = [](const StallRestartOptions &opts, bool block_steps, bool retunes = true) {
		QuarticProblem problem;
		problem.block_steps = block_steps;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, opts, [retunes](const auto &) { return retunes; });
		if (block_steps)
			solver.direction_filter = [](const auto &, auto &) {};
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS(solver.solve_reduced(problem, sol, parameters(), linear, 1));
		return solver.info();
	};

	SECTION("the soft iteration budget is not an alpha collapse")
	{
		// This is the public scenes' failure mode: the budget ends the pass
		// while the line search is still accepting the full step.
		const json info = solve(restart_options(0), false);
		CHECK(info["stall_trigger"] == "soft_iteration_limit");
		CHECK(info["stall_iteration"] == 1);
		CHECK(info["stall_alpha"] == 1.0);
	}

	SECTION("small alpha is reported as alpha")
	{
		auto opts = restart_options(0);
		opts.soft_iteration_limit = -1; // leave only the alpha patience
		opts.alpha_threshold = 2.0;     // a test knob: every accepted alpha counts
		opts.patience = 2;
		const json info = solve(opts, false);
		CHECK(info["stall_trigger"] == "alpha");
		// The callback runs once per accepted iteration and reports the count
		// completed before it, so a patience of 2 is reached at 1.
		CHECK(info["stall_iteration"] == 1);
		CHECK(info["stall_alpha"] == 1.0);
	}

	SECTION("both conditions at once are reported as both")
	{
		auto opts = restart_options(0); // soft_iteration_limit 1
		opts.alpha_threshold = 2.0;
		opts.patience = 2;
		const json info = solve(opts, false);
		CHECK(info["stall_trigger"] == "alpha_and_soft_iteration_limit");
		CHECK(info["stall_iteration"] == 1);
	}

	SECTION("a line search that failed on every strategy is neither")
	{
		// The callback never sees this one: the solver threw instead. It
		// needs restart budget, because a hard stall with none left is an
		// ordinary failure that rethrows before any restart decision.
		auto opts = restart_options(20);
		const json info = solve(opts, true, /*retunes=*/false);
		CHECK(info["stall_trigger"] == "line_search_failed_on_all_strategies");
		CHECK(info["stall_iteration"] == -1);
		CHECK(info["stall_alpha"].is_null());
	}

	SECTION("a hard stall with no restart budget left rethrows before the decision")
	{
		// Documented boundary: the trigger is recorded where a restart is
		// decided, so the plain failure path carries the error instead.
		const json info = solve(restart_options(0), true);
		CHECK(info["outcome"] == "failed");
		CHECK_FALSE(info.contains("stall_trigger"));
	}
}

TEST_CASE("AL hard line search failures remain failures and clean shared solver", "[al_solver]")
{
	QuarticProblem problem;
	problem.block_steps = true;
	int retunes = 0;
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(2), [&](const auto &) { ++retunes; return true; });
	solver.direction_filter = [](const auto &, auto &) {};
	std::shared_ptr<polysolve::nonlinear::Solver> nl_solver = polysolve::nonlinear::Solver::create(parameters(), linear, 1, logger());
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
	REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, nl_solver), ContainsSubstring("Line search failed"));
	CHECK(sol(0, 0) == 10);
	CHECK(retunes == 2);
	CHECK(solver.info()["outcome"] == "failed");
	problem.block_steps = false;
	ALSolver control({}, 1, 2, 1e8, .99, [](const auto &) {});
	REQUIRE_NOTHROW(control.solve_reduced(problem, sol, nl_solver));
	CHECK(std::abs(std::pow(sol(0, 0), 3)) < 1e-12);
}

TEST_CASE("AL interrupted iterate is explicitly available for continuation", "[al_solver]")
{
	struct ExposedSolver : ALSolver
	{
		using ALSolver::ALSolver;
		using ALSolver::minimize_with_stall_restarts;
	};
	QuarticProblem problem;
	ExposedSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
	Eigen::VectorXd x = Eigen::VectorXd::Constant(1, 10);
	CHECK(solver.minimize_with_stall_restarts(problem, x, parameters(), linear, 1, nullptr) == ALSolver::SubsolveOutcome::Interrupted);
	CHECK(x[0] < 10);
	CHECK(std::pow(x[0], 3) > 1e-12);
}

TEST_CASE("AL final solve preserves configured non-gradient convergence", "[al_solver]")
{
	QuarticProblem problem;
	auto params = parameters();
	params["x_delta_tol"] = 100;
	params["allow_non_grad_convergence"] = true;
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
	REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, params, linear, 1));
	CHECK(solver.info()["outcome"] == "converged");
}

TEST_CASE("AL feasibility permits repeated interrupted passes before reduced convergence", "[al_solver]")
{
	struct TwoQuartics : Form
	{
		std::string name() const override { return "two-quartics"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .25 * x.array().pow(4).sum(); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = x.array().cube(); }
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(2, 2);
			h.setZero();
			for (int i = 0; i < 2; ++i)
				h.coeffRef(i, i) = 3 * x[i] * x[i];
		}
	};
	struct SnapProblem : NLProblem
	{
		SnapProblem(std::shared_ptr<AugmentedLagrangianForm> bc, const StiffnessMatrix &mass)
			: NLProblem(2, 0, {std::make_shared<TwoQuartics>()}, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1) {}
		bool is_step_collision_free(const TVector &from, const TVector &to) override
		{
			// Synthetic geometric gate: snapping the prescribed coordinate
			// is safe only after the free coordinate moves below 0.5.
			if (from.size() == 2 && to.size() == 1)
				return from[1] < .5;
			return NLProblem::is_step_collision_free(from, to);
		}
	};
	StiffnessMatrix mass(2, 2);
	mass.setIdentity();
	const std::vector<int> boundary{0};
	auto bc = std::make_shared<BCLagrangianForm>(2, boundary, mass, 0, Eigen::VectorXd::Zero(2));
	SnapProblem problem(bc, mass);
	ALSolver preparation({bc}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
	int interruptions = 0;
	preparation.post_subsolve = [&](double) { if (preparation.info()["outcome"] == "interrupted") ++interruptions; };
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(2, 10);
	REQUIRE_NOTHROW(preparation.solve_al(problem, sol, parameters(), linear, 1));
	CHECK(interruptions >= 3);
	CHECK(sol(1, 0) < .5);
	CHECK(std::pow(sol(1, 0), 3) > 1e-12);
	ALSolver final_solve({bc}, 1, 2, 1e8, .99, [](const auto &) {});
	REQUIRE_NOTHROW(final_solve.solve_reduced(problem, sol, parameters(), linear, 1));
	CHECK(sol(0, 0) == 0);
	CHECK(std::abs(std::pow(sol(1, 0), 3)) < 1e-12);
}

TEST_CASE("AL final solve preserves the descending slope tolerance", "[al_solver]")
{
	QuarticProblem problem;
	auto params = parameters();
	params["advanced"]["derivative_along_delta_x_tol"] = 1e-4;
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
	REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, params, linear, 1));
	CHECK(solver.info()["outcome"] == "converged");
	CHECK(solver.info()["termination_reason"] == "Configured directional-derivative tolerance reached");
}

TEST_CASE("BC metric normalization excludes obstacle placeholders", "[bc_metric]")
{
	for (int nobs : {0, 1, 3, 20})
	{
		CAPTURE(nobs);
		const int n = 2 + nobs;
		StiffnessMatrix mass(n, n);
		mass.coeffRef(0, 0) = 1;
		mass.coeffRef(1, 1) = 3;
		std::vector<int> boundary;
		for (int i = 0; i < n; ++i)
			boundary.push_back(i);
		BCLagrangianForm form(n, boundary, mass, nobs, Eigen::VectorXd::Zero(n));
		form.set_initial_weight(1);
		StiffnessMatrix h;
		form.second_derivative(Eigen::VectorXd::Zero(n), h);
		CHECK(std::abs(h.coeff(0, 0) - 0.5) < 1e-12);
		CHECK(std::abs(h.coeff(1, 1) - 1.5) < 1e-12);
		for (int i = 2; i < n; ++i)
			CHECK(std::abs(h.coeff(i, i) - 1) < 1e-12);
	}
}

TEST_CASE("BC fixed targets retain inhomogeneous values after slicing", "[bc_metric]")
{
	std::vector<int> boundary{0, 2};
	Eigen::VectorXd target(3);
	target << 0.3, 7, -0.4;
	BCLagrangianForm form(3, boundary, StiffnessMatrix(), 0, target);
	form.set_initial_weight(1);
	REQUIRE(form.constraint_value().rows() == 2);
	CHECK(std::abs(form.constraint_value()(0, 0) - target[0]) < 1e-12);
	CHECK(std::abs(form.constraint_value()(1, 0) - target[2]) < 1e-12);
	CHECK(form.compute_error(target) < 1e-24);
	CHECK(form.value(target) < 1e-24);
}

TEST_CASE("BC metric preserves lumping and identity fallbacks", "[bc_metric]")
{
	for (int nobs : {0, 3})
		for (int mode = 0; mode < 4; ++mode)
		{
			CAPTURE(nobs, mode);
			const int n = 2 + nobs;
			std::vector<int> boundary{1}; // Reference includes unconstrained FEM DOFs too.
			if (nobs)
				boundary.push_back(n - 1);
			StiffnessMatrix mass(n, n);
			double expected = 1;
			if (mode == 0) // Positive row sums
			{
				mass.coeffRef(0, 0) = 0.5;
				mass.coeffRef(0, 1) = 0.5;
				mass.coeffRef(1, 0) = 0.5;
				mass.coeffRef(1, 1) = 2.5;
				expected = 1.5;
			}
			else if (mode == 1) // Invalid row sums use HRZ relative weights
			{
				mass.coeffRef(0, 0) = 1;
				mass.coeffRef(0, 1) = -2;
				mass.coeffRef(1, 0) = -2;
				mass.coeffRef(1, 1) = 9;
				expected = 1.8;
			}
			else if (mode == 2) // Nonpositive HRZ uses identity
			{
				mass.coeffRef(0, 0) = -1;
				mass.coeffRef(1, 1) = 3;
			}
			else // No mass uses identity
			{
				mass.resize(0, 0);
			}
			BCLagrangianForm form(n, boundary, mass, nobs, Eigen::VectorXd::Zero(n));
			form.set_initial_weight(1);
			StiffnessMatrix h;
			form.second_derivative(Eigen::VectorXd::Zero(n), h);
			CHECK(std::abs(h.coeff(0, 0)) < 1e-12);
			CHECK(std::abs(h.coeff(1, 1) - expected) < 1e-12);
			if (nobs)
				CHECK(std::abs(h.coeff(n - 1, n - 1) - 1) < 1e-12);
		}
}

TEST_CASE("BC AL derivatives match the objective at nonunit form scales", "[bc_scale]")
{
	class SeededBC : public BCLagrangianForm
	{
	public:
		using BCLagrangianForm::BCLagrangianForm;
		void seed(const Eigen::VectorXd &lambda) { lagr_mults_ = lambda; }
	};
	const std::vector<int> boundary{0, 2};
	StiffnessMatrix mass(3, 3);
	mass.coeffRef(0, 0) = 1;
	mass.coeffRef(1, 1) = 2;
	mass.coeffRef(2, 2) = 3;
	Eigen::VectorXd target(3), x(3), lambda(2);
	target << 0.3, 7, -0.4;
	x << 0.8, 0.1, 0.6;
	lambda << 0.7, -0.4;
	for (double scale : {0.5, 1.0, 2.0})
	{
		CAPTURE(scale);
		SeededBC form(3, boundary, mass, 0, target);
		form.set_initial_weight(3);
		form.set_scale(scale);
		form.seed(lambda);
		// Verify the inhomogeneous fixture before differentiating it.
		REQUIRE(form.constraint_value().rows() == 2);
		REQUIRE(form.constraint_value()(0, 0) == target[0]);
		REQUIRE(form.constraint_value()(1, 0) == target[2]);
		Eigen::VectorXd residual(2), metric(2);
		residual << x[0] - target[0], x[2] - target[2];
		metric << 0.5, 1.5;
		const double expected_value = (-lambda.dot(metric.array().sqrt().matrix().cwiseProduct(residual))
									   + 1.5 * residual.dot(metric.cwiseProduct(residual)))
									  / scale;
		CHECK(std::abs(form.value(x) - expected_value) < 1e-12);
		Eigen::VectorXd g;
		StiffnessMatrix h;
		form.first_derivative(x, g);
		form.second_derivative(x, h);
		Eigen::VectorXd fd_g(3);
		Eigen::MatrixXd fd_h(3, 3);
		const double eps = 1e-5;
		for (int i = 0; i < 3; ++i)
		{
			Eigen::VectorXd plus = x, minus = x, gp, gm;
			plus[i] += eps;
			minus[i] -= eps;
			fd_g[i] = (form.value(plus) - form.value(minus)) / (2 * eps);
			form.first_derivative(plus, gp);
			form.first_derivative(minus, gm);
			fd_h.col(i) = (gp - gm) / (2 * eps);
		}
		CHECK((g - fd_g).norm() < 1e-8);
		CHECK((Eigen::MatrixXd(h) - fd_h).norm() < 1e-8);
		CHECK(g[1] == 0);
		CHECK(h.coeff(1, 1) == 0);
	}
}

TEST_CASE("Normalized BC AL matches mass metric with converted parameters", "[bc_metric]")
{
	// Independent upstream mass-metric equations, at the existing form scale 1.
	// PF-05 covers nonunit form-scale derivatives separately.
	class SeededBC : public BCLagrangianForm
	{
	public:
		using BCLagrangianForm::BCLagrangianForm;
		void seed(const Eigen::VectorXd &lambda) { lagr_mults_ = lambda; }
	};
	for (double mass_scale : {1e-9, 1.0, 1e9})
	{
		CAPTURE(mass_scale);
		std::vector<int> boundary{0, 2};
		StiffnessMatrix mass(3, 3);
		mass.coeffRef(0, 0) = mass_scale;
		mass.coeffRef(1, 1) = 3 * mass_scale;
		Eigen::VectorXd target(3), x(3), lambda(2), raw(2);
		target << 0.3, 7, -0.4;
		x << 0.8, 0.1, 0.6;
		raw << mass_scale, 2 * mass_scale;
		const double mean = 2 * mass_scale;
		lambda << 0.7 / std::sqrt(mean), -0.4 / std::sqrt(mean);
		SeededBC form(3, boundary, mass, 1, target);
		form.seed(std::sqrt(mean) * lambda);
		for (double rho : {2.0, 5.0, 11.0})
		{
			const double upstream_rho = rho / mean;
			form.set_initial_weight(rho);
			Eigen::VectorXd residual(2);
			residual << x[0] - target[0], x[2] - target[2];
			const Eigen::VectorXd sqrt_raw = raw.array().sqrt();
			const double value = -lambda.dot(sqrt_raw.cwiseProduct(residual))
								 + 0.5 * upstream_rho * residual.dot(raw.cwiseProduct(residual));
			const Eigen::VectorXd constrained_g = -sqrt_raw.cwiseProduct(lambda)
												  + upstream_rho * raw.cwiseProduct(residual);
			Eigen::VectorXd expected_g = Eigen::VectorXd::Zero(3), g;
			expected_g[0] = constrained_g[0];
			expected_g[2] = constrained_g[1];
			Eigen::MatrixXd expected_h = Eigen::MatrixXd::Zero(3, 3);
			expected_h(0, 0) = upstream_rho * raw[0];
			expected_h(2, 2) = upstream_rho * raw[1];
			StiffnessMatrix h;
			form.first_derivative(x, g);
			form.second_derivative(x, h);
			CHECK(std::abs(form.value(x) - value) < 1e-11);
			CHECK((g - expected_g).norm() < 1e-11);
			CHECK((Eigen::MatrixXd(h) - expected_h).norm() < 1e-11);
			// Both multiplier updates preserve lambda_f = sqrt(mean) lambda_u.
			form.update_lagrangian(x, rho);
			lambda -= upstream_rho * sqrt_raw.cwiseProduct(residual);
			form.update_quantities(1.0, x); // Does not renormalize during AL.
			x *= 0.7;
		}
	}
}

TEST_CASE("AL clears direction filters on every exit", "[al_solver][direction_filter]")
{
	QuarticProblem problem;
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
	int calls = 0;
	solver.direction_filter = [&](const auto &, auto &) { ++calls; };
	auto options = parameters();
	bool fails = false;
	SECTION("convergence") {}
	SECTION("interruption")
	{
		options["max_iterations"] = 1;
		options["allow_out_of_iterations"] = true;
		fails = true;
	}
	SECTION("exception")
	{
		problem.block_steps = true;
		fails = true;
	}
	std::shared_ptr<polysolve::nonlinear::Solver> shared = polysolve::nonlinear::Solver::create(options, linear, 1, logger());
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
	if (fails)
		CHECK_THROWS(solver.solve_reduced(problem, sol, shared));
	else
		REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, shared));
	REQUIRE(calls > 0);
	const int previous_calls = calls;
	problem.block_steps = false;
	Eigen::VectorXd x = Eigen::VectorXd::Constant(1, 10);
	REQUIRE_NOTHROW(shared->minimize(problem, x));
	CHECK(calls == previous_calls);
}

TEST_CASE("AL continuation handles zero initial error and caps penalty growth", "[al_solver][al_continuation]")
{
	struct TwoQuartics : Form
	{
		std::string name() const override { return "two-quartics"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .25 * x.array().pow(4).sum(); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = x.array().cube(); }
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(2, 2);
			h.setZero();
			for (int i = 0; i < 2; ++i)
				h.coeffRef(i, i) = 3 * x[i] * x[i];
		}
	};
	struct SnapProblem : NLProblem
	{
		SnapProblem(std::shared_ptr<AugmentedLagrangianForm> bc, const StiffnessMatrix &mass)
			: NLProblem(2, 0, {std::make_shared<TwoQuartics>()}, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1) {}
		bool is_step_collision_free(const TVector &from, const TVector &to) override
		{
			// Synthetic geometric gate: snapping the prescribed coordinate
			// is safe only after the free coordinate moves below 0.5.
			if (from.size() == 2 && to.size() == 1)
				return from[1] < .5;
			return NLProblem::is_step_collision_free(from, to);
		}
	};
	StiffnessMatrix mass(2, 2);
	mass.setIdentity();
	const std::vector<int> boundary{0};
	auto bc = std::make_shared<BCLagrangianForm>(2, boundary, mass, 0, Eigen::VectorXd::Zero(2));
	SnapProblem problem(bc, mass);
	ALSolver preparation({bc}, 3, 2, 5, 1.0, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
	int interruptions = 0;
	std::vector<double> weights;
	preparation.post_subsolve = [&](double weight) { if (preparation.info()["outcome"] == "interrupted") ++interruptions; weights.push_back(weight); };
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(2, 10);
	SECTION("nonzero initial BC error") {}
	SECTION("zero initial BC error") { sol(0, 0) = 0; }
	REQUIRE_NOTHROW(preparation.solve_al(problem, sol, parameters(), linear, 1));
	CHECK(interruptions >= 3);
	REQUIRE_FALSE(weights.empty());
	for (double weight : weights)
		CHECK(weight <= 5);
	CHECK(weights.front() == 5);
	CHECK(sol(1, 0) < .5);
	CHECK(std::pow(sol(1, 0), 3) > 1e-12);
	ALSolver final_solve({bc}, 1, 2, 1e8, .99, [](const auto &) {});
	REQUIRE_NOTHROW(final_solve.solve_reduced(problem, sol, parameters(), linear, 1));
	CHECK(sol(0, 0) == 0);
	CHECK(std::abs(std::pow(sol(1, 0), 3)) < 1e-12);
}

TEST_CASE("AL converted units and density preserve reduced solutions", "[al_solver][al_continuation]")
{
	class Quadratic : public Form
	{
	public:
		Eigen::Matrix2d h;
		std::string name() const override { return "scaled-elastic-plus-inertia"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .5 * x.dot(h * x); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = h * x; }
		void second_derivative_unweighted(const Eigen::VectorXd &, StiffnessMatrix &out) const override { out = h.sparseView(); }
	};
	struct SnapProblem : NLProblem
	{
		double length;
		SnapProblem(std::shared_ptr<Form> f, std::shared_ptr<AugmentedLagrangianForm> bc, const StiffnessMatrix &m, double length)
			: NLProblem(2, 0, {f}, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, m, 1), length(length) {}
		bool is_step_collision_free(const TVector &from, const TVector &to) override
		{
			// Synthetic feasibility gate, not a collision model. Exercise
			// the real AL loop until the prescribed-coordinate gap shrinks.
			if (from.size() == 2 && to.size() == 1)
				return std::abs(from[0] / length - .3) < .05;
			return NLProblem::is_step_collision_free(from, to);
		}
	};
	for (double length : {1e-3, 1.0, 1e3})
		for (double energy : {1e-4, 1.0, 1e4})
			for (double density : {0.0, 1.0, 100.0})
			{
				CAPTURE(length, energy, density);
				const double stiffness = energy / (length * length);
				auto f = std::make_shared<Quadratic>();
				// Dimensionless elastic K plus inertial density*diag(1,3).
				f->h << 2 + density, -1, -1, 2 + 3 * density;
				f->h *= stiffness;
				StiffnessMatrix mass(2, 2);
				// Uniform conversion of FEM masses leaves the BC metric fixed.
				mass.coeffRef(0, 0) = energy * std::max(density, 1.0);
				mass.coeffRef(1, 1) = 3 * mass.coeff(0, 0);
				const std::vector<int> boundary{0};
				Eigen::Vector2d target(.3 * length, 0);
				auto bc = std::make_shared<BCLagrangianForm>(2, boundary, mass, 0, target);
				SnapProblem problem(f, bc, mass, length);
				ALSolver solver({bc}, .2 * stiffness, 2, 1000 * stiffness, .99, [](const auto &) {});
				int passes = 0;
				solver.post_subsolve = [&](double weight) {
					if (weight > 0)
					{
						++passes;
						REQUIRE(passes < 200); // Test watchdog, not a production budget.
						CHECK(weight <= 1000 * stiffness);
						CHECK(std::isfinite(solver.info()["al_relative_progress"].get<double>()));
					}
				};
				auto params = parameters();
				params["grad_norm_tol"] = 1e-9 * energy / length;
				params["advanced"]["derivative_along_delta_x_tol"] = 0;
				Eigen::MatrixXd sol = Eigen::Vector2d(0, length);
				REQUIRE_NOTHROW(solver.solve_al(problem, sol, params, linear, length));
				CHECK(passes > 0);
				REQUIRE_NOTHROW(solver.solve_reduced(problem, sol, params, linear, length));
				CHECK(std::abs(sol(0, 0) / length - .3) < 1e-12);
				CHECK(std::abs(sol(1, 0) / length - .3 / (2 + 3 * density)) < 1e-9);
				CHECK(std::abs((f->h * sol)(1, 0)) * length / energy < 1e-9);
				CHECK(solver.info()["outcome"] == "converged");
			}
}

TEST_CASE("Iteration observer reports proposals, bounds and accepted iterates without changing the solve", "[al_solver][iteration_observer]")
{
	using Kind = IterationObservation::Kind;
	struct Event
	{
		Kind kind;
		Eigen::VectorXd x0, x1;
		double step_bound;
		bool valid;
		int iteration;
		double energy;
	};
	// A form that can veto every step through the forms' step bound, so the
	// bound of 0 is observed through the same hook production CCD uses.
	struct Blocker : Form
	{
		bool blocked = false;
		std::string name() const override { return "blocker"; }
		double value_unweighted(const Eigen::VectorXd &) const override { return 0; }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = Eigen::VectorXd::Zero(x.size()); }
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override { h.resize(x.size(), x.size()); }
		double max_step_size(const Eigen::VectorXd &, const Eigen::VectorXd &) const override { return blocked ? 0 : 1; }
	};
	const auto blocker = std::make_shared<Blocker>();
	NLProblem problem(1, 0, {std::make_shared<Quartic>(), blocker}, {}, nullptr, 1, 1, QuarticProblem::mass(), 1);
	const auto observe = [&](std::vector<Event> *events) {
		if (events)
			problem.set_iteration_observer([events](const IterationObservation &o) {
				events->push_back({o.kind, o.x0 ? *o.x0 : Eigen::VectorXd(), o.x1 ? *o.x1 : Eigen::VectorXd(), o.step_bound, o.valid, o.iteration,
								   o.solver_info && o.solver_info->contains("energy") ? (*o.solver_info)["energy"].get<double>() : std::nan("")});
			});
		else
			problem.set_iteration_observer(nullptr);
	};
	const auto solve = [&]() {
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		solver.solve_reduced(problem, sol, parameters(), linear, 1);
		return std::make_pair(sol(0, 0), solver.info()["iterations"].get<int>());
	};

	std::vector<Event> events;
	observe(&events);
	const auto [x_observed, iterations] = solve();
	observe(nullptr);
	const auto [x_control, control_iterations] = solve();
	CHECK(x_observed == x_control);
	CHECK(iterations == control_iterations);
	REQUIRE(iterations > 0);

	// The reduced solve opens with ALSolver's feasibility check (a proposal
	// closed by line_search_end without a step bound), then PolySolve reports
	// the start point; every later accepted iterate follows a proposal and a
	// step bound and lies on the proposed trial segment.
	REQUIRE(events.size() > 4);
	CHECK(events[0].kind == Kind::Proposal);
	CHECK(events[1].kind == Kind::Validity);
	CHECK(events[2].kind == Kind::LineSearchEnd);
	CHECK(events[3].kind == Kind::Accepted);
	CHECK(events[3].iteration == 0);
	CHECK(events[3].x1[0] == 10);
	int updates = 0, proposals = 0, feasibility = 0, validity = 0, ends = 0;
	const Event *proposal = nullptr, *bound = nullptr;
	for (size_t i = 4; i < events.size(); ++i)
	{
		const Event &e = events[i];
		switch (e.kind)
		{
		case Kind::Validity:
			++validity;
			CHECK(e.valid);
			break;
		case Kind::Proposal:
			proposal = &e;
			bound = nullptr;
			break;
		case Kind::StepBound:
			REQUIRE(proposal != nullptr);
			CHECK(e.x0 == proposal->x0);
			CHECK(e.x1 == proposal->x1);
			CHECK(e.step_bound == 1);
			bound = &e;
			++proposals;
			break;
		case Kind::LineSearchEnd:
			++ends;
			if (proposal != nullptr && bound == nullptr)
				++feasibility;
			break;
		case Kind::Accepted:
			REQUIRE(proposal != nullptr);
			REQUIRE(bound != nullptr);
			{
				const double trial = proposal->x1[0] - proposal->x0[0];
				const double fraction = (e.x1[0] - proposal->x0[0]) / trial;
				CHECK(fraction > 0);
				CHECK(fraction <= bound->step_bound + 1e-12);
				CHECK(std::isfinite(e.energy));
				CHECK(e.iteration == updates); // iterations completed before this update
			}
			++updates;
			proposal = nullptr;
			bound = nullptr;
			break;
		}
	}
	CHECK(updates == iterations);
	CHECK(proposals == iterations);
	CHECK(ends == iterations);
	CHECK(feasibility == 0);
	CHECK(validity >= iterations);

	// A vetoed step (bound 0) is observed as a proposal with a zero bound and
	// no accepted iterate after it; the failure itself is unchanged.
	blocker->blocked = true;
	events.clear();
	observe(&events);
	{
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, parameters(), linear, 1), ContainsSubstring("Line search failed"));
	}
	const auto last_bound = std::find_if(events.rbegin(), events.rend(), [](const Event &e) { return e.kind == Kind::StepBound; });
	REQUIRE(last_bound != events.rend());
	CHECK(last_bound->step_bound == 0);
	CHECK(std::count_if(events.begin(), events.end(), [](const Event &e) { return e.kind == Kind::Accepted; }) == 1); // the start point only
	blocker->blocked = false;

	// A throwing observer is disabled after its first call and the solve is unchanged.
	int calls = 0;
	problem.set_iteration_observer([&](const IterationObservation &) {
		++calls;
		throw std::runtime_error("Injected observer failure");
	});
	{
		const auto [x_throwing, throwing_iterations] = solve();
		CHECK(x_throwing == x_control);
		CHECK(throwing_iterations == control_iterations);
	}
	CHECK(calls == 1);

	// Cleared observer: no events.
	events.clear();
	observe(nullptr);
	solve();
	CHECK(events.empty());
}

// ---------------------------------------------------------------------------
// RB-07: bounded AL stagnation handling. The budget is opt-in and off by
// default; a compatible continuation under a budget it does not exhaust is
// unchanged, an infeasible one ends with a named failure carrying the pass
// history instead of looping forever.
namespace
{
	struct TwoQuarticsForm : Form
	{
		std::string name() const override { return "two-quartics"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .25 * x.array().pow(4).sum(); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = x.array().cube(); }
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(2, 2);
			h.setZero();
			for (int i = 0; i < 2; ++i)
				h.coeffRef(i, i) = 3 * x[i] * x[i];
		}
	};

	// Synthetic geometric gate on the prescribed coordinate: the snap is safe
	// once the free coordinate is below `open_below` (never, when negative).
	struct GatedSnapProblem : NLProblem
	{
		double open_below;
		GatedSnapProblem(const std::vector<std::shared_ptr<Form>> &forms, std::shared_ptr<AugmentedLagrangianForm> bc, const StiffnessMatrix &mass, double open_below)
			: NLProblem(2, 0, forms, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1), open_below(open_below) {}
		bool is_step_collision_free(const TVector &from, const TVector &to) override
		{
			if (from.size() == 2 && to.size() == 1)
				return from[1] < open_below;
			return NLProblem::is_step_collision_free(from, to);
		}
	};

	// A synthetic obstacle on the prescribed coordinate at `wall`: a log
	// barrier of the distance to the wall that the AL cannot push through,
	// with the CCD-like step bound and collision check the real contact form
	// provides (the energy stays finite on the far side, like the unsigned
	// distance of a penetrating configuration; only the sweep is blocked).
	// Not a contact model; it reproduces the shape of the incompatible-motion
	// fixture (the constraint target lies behind the wall).
	struct WallForm : Form
	{
		double wall = .5, kappa = .1;
		std::string name() const override { return "wall"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return -kappa * std::log(std::abs(x[0] - wall)); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override
		{
			g = Eigen::VectorXd::Zero(x.size());
			g[0] = -kappa / (x[0] - wall);
		}
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(x.size(), x.size());
			h.setZero();
			h.coeffRef(0, 0) = kappa / std::pow(x[0] - wall, 2);
		}
		bool is_step_collision_free(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1) const override
		{
			return (x0[0] - wall) * (x1[0] - wall) > 0;
		}
		double max_step_size(const Eigen::VectorXd &x0, const Eigen::VectorXd &x1) const override
		{
			if (is_step_collision_free(x0, x1))
				return 1;
			return std::max(0.0, (x0[0] - wall) / (x0[0] - x1[0]) * .9);
		}
	};

	// A strictly convex free coordinate (the quartic's Hessian vanishes at
	// its minimum and sends Newton to gradient descent, which cannot reach a
	// 1e-12 gradient next to the wall's roundoff).
	struct QuadraticFreeForm : Form
	{
		std::string name() const override { return "quadratic-free"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .5 * x[1] * x[1]; }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override
		{
			g = Eigen::VectorXd::Zero(x.size());
			g[1] = x[1];
		}
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(x.size(), x.size());
			h.setZero();
			h.coeffRef(1, 1) = 1;
		}
	};

	StiffnessMatrix identity2()
	{
		StiffnessMatrix mass(2, 2);
		mass.setIdentity();
		return mass;
	}

	json wall_parameters()
	{
		json params = parameters();
		params["grad_norm_tol"] = 1e-8;
		return params;
	}
} // namespace

TEST_CASE("AL budget options are off by default and read from the AL block", "[al_solver][al_budget]")
{
	CHECK_FALSE(ALBudgetOptions().enabled());
	CHECK_FALSE(ALBudgetOptions::from_json(json::object()).enabled());
	CHECK_FALSE(ALBudgetOptions::from_json({{"budget", json::object()}}).enabled());
	const auto budget = ALBudgetOptions::from_json({{"budget", {{"max_passes", 7}, {"stagnation_window", 2}, {"progress_tolerance", .05}}}});
	CHECK(budget.enabled());
	CHECK(budget.max_passes == 7);
	CHECK(budget.stagnation_window == 2);
	CHECK(budget.progress_tolerance == .05);
	CHECK(budget.snap_tolerance == 1e-2);
	CHECK(budget.drift_tolerance == 1e-2);
	CHECK_THROWS(ALBudgetOptions::from_json({{"budget", {{"max_passes", -1}}}}));
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {});
	CHECK_FALSE(solver.budget().enabled());
}

TEST_CASE("AL pass budget ends an infeasible continuation with a named failure and its pass history", "[al_solver][al_budget]")
{
	const auto mass = identity2();
	auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
	GatedSnapProblem problem({std::make_shared<TwoQuarticsForm>()}, bc, mass, /*open_below=*/-1); // never opens
	ALSolver solver({bc}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
	ALBudgetOptions budget;
	budget.max_passes = 4;
	solver.set_budget(budget);
	int passes = 0;
	solver.post_subsolve = [&](double) { ++passes; };
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(2, 10);
	try
	{
		solver.solve_al(problem, sol, parameters(), linear, 1);
		FAIL("the budget must end the stage");
	}
	catch (const ALBudgetExhausted &e)
	{
		CHECK_THAT(e.what(), ContainsSubstring("exhausted its pass budget: 4 passes"));
		const json &d = e.details();
		CHECK(d["reason"] == "pass_budget");
		CHECK(d["passes"] == 4);
		CHECK(d["budget"]["max_passes"] == 4);
		REQUIRE(d["history"].size() == 5); // pass 0 + four passes
		CHECK(d["history"][0]["pass"] == 0);
		CHECK(d["history"][0]["gate"]["blocked_by"] == "collision");
		for (int k = 1; k <= 4; ++k)
		{
			const json &pass = d["history"][k];
			CHECK(pass["pass"] == k);
			CHECK(pass["subsolve"]["outcome"] == "interrupted");
			CHECK(pass["gate"]["feasible"] == false);
			CHECK(pass["gate"]["blocked_by"] == "collision");
			CHECK(pass["gate"]["finite_energy"] == true);
			CHECK(pass["gate"]["valid"] == true);
			CHECK(pass["gate"]["collision_free"] == false);
			// One Newton step per pass overshoots; the residual is finite and
			// below the start, not monotone.
			CHECK(std::isfinite(pass["bc_residual"].get<double>()));
			CHECK(pass["bc_residual"].get<double>() < d["history"][0]["bc_residual"].get<double>());
			CHECK(pass["moved"].get<double>() > 0);
			CHECK(pass["retained_states"] == 1); // RBR-03: a cap alone keeps the previous state only
		}
		CHECK(d["last_pass"] == d["history"][4]);
	}
	CHECK(passes == 4);
	CHECK(solver.al_history().size() == 5);
	// ALSolver leaves the last iterate in the caller's solution, like every
	// other AL failure; the step transaction (RB-06) restores the accepted
	// state around it.
	CHECK(sol(1, 0) < 10);
}

TEST_CASE("AL budget that is not exhausted leaves a compatible continuation unchanged", "[al_solver][al_budget][al_continuation]")
{
	// The PF-07 fixture: initial weight 3, ceiling 5 (reached at the second
	// pass), one Newton step per pass, the gate opens once the free
	// coordinate is below .5. With a nonzero BC residual the residual falls
	// every pass; with a zero one only the free coordinate moves -- progress
	// the BC residual cannot see and the drift measure must.
	for (const bool zero_initial_error : {false, true})
	{
		CAPTURE(zero_initial_error);
		const auto run = [&](const ALBudgetOptions *budget) {
			const auto mass = identity2();
			auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
			GatedSnapProblem problem({std::make_shared<TwoQuarticsForm>()}, bc, mass, .5);
			ALSolver preparation({bc}, 3, 2, 5, 1.0, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
			if (budget)
				preparation.set_budget(*budget);
			std::vector<double> weights;
			preparation.post_subsolve = [&](double weight) { weights.push_back(weight); };
			Eigen::MatrixXd sol = Eigen::VectorXd::Constant(2, 10);
			if (zero_initial_error)
				sol(0, 0) = 0;
			REQUIRE_NOTHROW(preparation.solve_al(problem, sol, parameters(), linear, 1));
			ALSolver final_solve({bc}, 1, 2, 1e8, .99, [](const auto &) {});
			REQUIRE_NOTHROW(final_solve.solve_reduced(problem, sol, parameters(), linear, 1));
			return std::make_tuple(Eigen::VectorXd(sol), weights, preparation.al_history());
		};
		const auto [sol_off, weights_off, history_off] = run(nullptr);
		ALBudgetOptions budget;
		budget.max_passes = 100;
		budget.stagnation_window = 2;
		const auto [sol_on, weights_on, history_on] = run(&budget);
		REQUIRE(weights_off.size() >= 3);
		CHECK(weights_on == weights_off);
		CHECK(sol_on == sol_off);
		CHECK(sol_on[0] == 0);
		CHECK(std::abs(std::pow(sol_on[1], 3)) < 1e-12);
		// Every pass after the first ran at the ceiling: the stagnation exit
		// was armed and did not fire because the iterate kept moving.
		REQUIRE(history_on.size() == weights_on.size() + 1);
		for (size_t k = 2; k < history_on.size(); ++k)
		{
			CHECK(history_on[k]["at_ceiling"] == true);
			CHECK(history_on[k]["moved"].get<double>() > 0);
			CHECK(history_on[k]["retained_states"] == 3); // RBR-03: the window (2) plus the current state
			// The fraction is probed while the snap is blocked (the last
			// record is the feasible snap that ended the stage).
			CHECK(history_on[k]["gate"]["ccd_fraction"].is_number() == (history_on[k]["gate"]["feasible"] == false));
		}
		// Without a budget the record keeps the historical short circuit:
		// the collision gate is not evaluated once an earlier gate fails, and
		// the CCD fraction is not probed.
		for (size_t k = 0; k < history_off.size(); ++k)
		{
			CHECK(history_off[k]["gate"]["ccd_fraction"].is_null());
			CHECK(history_off[k]["retained_states"] == 0);
		}
	}
}

TEST_CASE("AL stagnation window ends a continuation blocked by a wall at the weight ceiling", "[al_solver][al_budget]")
{
	// The prescribed coordinate is driven to 0 behind a wall at .5: every
	// pass ends at the wall (the BC residual plateaus at .5), the snap is
	// blocked by the collision gate throughout, the collision-free fraction
	// of the snap shrinks with the gap and the iterate stops moving once the
	// free coordinate has converged. Nothing the loop can do changes that.
	const auto mass = identity2();
	auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
	auto wall = std::make_shared<WallForm>();
	NLProblem problem(2, 0, {std::make_shared<QuadraticFreeForm>(), wall}, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1);
	ALSolver solver({bc}, 1, 2, 4, .99, [](const auto &) {});
	ALBudgetOptions budget;
	budget.max_passes = 100;
	budget.stagnation_window = 3;
	solver.set_budget(budget);
	int passes = 0;
	solver.post_subsolve = [&](double) { ++passes; };
	Eigen::MatrixXd sol(2, 1);
	sol << 2, 2;
	try
	{
		solver.solve_al(problem, sol, wall_parameters(), linear, 1);
		FAIL("the stagnation window must end the stage");
	}
	catch (const ALBudgetExhausted &e)
	{
		CHECK_THAT(e.what(), ContainsSubstring("stagnated: 3 consecutive passes at the weight ceiling 4"));
		const json &d = e.details();
		CHECK(d["reason"] == "stagnation");
		CHECK(d["window"] == 3);
		CHECK(d["progress"]["bc_residual"] == false);
		CHECK(d["progress"]["gates"] == false);
		CHECK(d["progress"]["ccd_fraction"] == false);
		CHECK(d["progress"]["drift"] == false);
		const int n = d["passes"].get<int>();
		CHECK(n == passes);
		CHECK(n < 100);
		REQUIRE(d["history"].size() == size_t(n) + 1);
		const json &last = d["last_pass"], &ref = d["reference_pass"];
		CHECK(ref["pass"] == n - 3);
		CHECK(last["pass"] == n);
		for (int k = n - 2; k <= n; ++k)
			CHECK(d["history"][k]["at_ceiling"] == true);
		CHECK(last["gate"]["blocked_by"] == "collision");
		CHECK(last["subsolve"]["outcome"] == "converged");
		const double e_ref = ref["bc_residual_carried"], e_now = last["bc_residual_carried"];
		CHECK(e_now > .5);
		CHECK(e_now > (1 - budget.progress_tolerance) * e_ref);
		CHECK(last["gate"]["ccd_fraction"].get<double>() < ref["gate"]["ccd_fraction"].get<double>() + budget.snap_tolerance);
		CHECK(last["drift_over_window"].get<double>() <= budget.drift_tolerance * last["snap_linf"].get<double>());
	}
	CHECK(sol(0, 0) > .5);
	CHECK(std::abs(sol(1, 0)) < 1e-3);

	// The same stage with the exit off keeps looping: bound it by the pass
	// cap alone and confirm the residual plateau the window judged.
	{
		auto bc2 = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
		NLProblem problem2(2, 0, {std::make_shared<QuadraticFreeForm>(), std::make_shared<WallForm>()}, {bc2}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1);
		ALSolver capped({bc2}, 1, 2, 4, .99, [](const auto &) {});
		ALBudgetOptions cap;
		cap.max_passes = 12;
		capped.set_budget(cap);
		Eigen::MatrixXd sol2(2, 1);
		sol2 << 2, 2;
		REQUIRE_THROWS_AS(capped.solve_al(problem2, sol2, wall_parameters(), linear, 1), ALBudgetExhausted);
		const json &h = capped.al_history();
		REQUIRE(h.size() == 13);
		for (int k = 6; k <= 12; ++k)
			CHECK(std::abs(h[k]["bc_residual_carried"].get<double>() - .5) < .05);
	}
}

// ---------------------------------------------------------------------------
// RBR-03: the AL stage keeps only the full-space states its motion measures
// need (the previous pass for `moved`, the pass W back for
// `drift_over_window`), released when the stage ends. The complete scalar
// pass record stays. An oracle in the tests keeps the complete state history
// the solver no longer stores and recomputes every motion value and the
// stagnation verdict from it.
namespace
{
	// Pulls the prescribed coordinate away from its target harder than the
	// first AL weights pull it back: the early passes raise the BC residual
	// above its start and are rolled back (eta < 0), the later ones are kept
	// -- a carried-state sequence with both kinds of pass.
	struct PullAwayForm : Form
	{
		double stiffness = 4, anchor = 1;
		std::string name() const override { return "pull-away"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .5 * stiffness * std::pow(x[0] - anchor, 2) + .5 * x[1] * x[1]; }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override
		{
			g = Eigen::VectorXd::Zero(x.size());
			g[0] = stiffness * (x[0] - anchor);
			g[1] = x[1];
		}
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(x.size(), x.size());
			h.setZero();
			h.coeffRef(0, 0) = stiffness;
			h.coeffRef(1, 1) = 1;
		}
	};

	// The complete carried-state history, rebuilt from the caller's solution
	// at every post_subsolve (states[k] = the carried state after pass k,
	// states[0] = the start), and the motion measures recomputed from it.
	struct FullHistoryOracle
	{
		std::vector<Eigen::VectorXd> states;
		double moved(const int k) const { return (states[k] - states[k - 1]).lpNorm<Eigen::Infinity>(); }
		bool has_drift(const int k, const int W) const { return W > 0 && k >= W; }
		double drift(const int k, const int W) const { return (states[k] - states[k - W]).lpNorm<Eigen::Infinity>(); }
	};

	// An independent evaluation of the stagnation rule on the scalar pass
	// records with the oracle's drift: the first pass at which the W passes
	// before it (inclusive) ran at the ceiling and none of the four progress
	// signals moved against the pass before the window; -1 when never.
	int first_stagnant_pass(const json &history, const FullHistoryOracle &oracle, const ALBudgetOptions &budget)
	{
		const int W = budget.stagnation_window;
		if (W <= 0)
			return -1;
		const auto num = [](const json &v) { return v.is_number() ? v.get<double>() : std::numeric_limits<double>::quiet_NaN(); };
		for (int k = W; k < int(history.size()); ++k)
		{
			bool at_ceiling = true;
			for (int j = k - W + 1; j <= k; ++j)
				at_ceiling = at_ceiling && history[j]["at_ceiling"].get<bool>();
			if (!at_ceiling)
				continue;
			const json &ref = history[k - W], &last = history[k];
			const double e_ref = num(ref["bc_residual_carried"]), e_now = num(last["bc_residual_carried"]);
			bool progress = e_ref > 0 && e_now <= (1 - budget.progress_tolerance) * e_ref;
			for (const char *key : {"finite_energy", "valid", "collision_free"})
				if (ref["gate"][key].is_boolean() && last["gate"][key].is_boolean() && !ref["gate"][key].get<bool>() && last["gate"][key].get<bool>())
					progress = true;
			const double f_ref = num(ref["gate"]["ccd_fraction"]), f_now = num(last["gate"]["ccd_fraction"]);
			if (std::isfinite(f_ref) && std::isfinite(f_now) && f_now >= f_ref + budget.snap_tolerance)
				progress = true;
			const double d = oracle.drift(k, W), linf = std::max(num(last["snap_linf"]), num(ref["snap_linf"]));
			if (linf > 0 ? d > budget.drift_tolerance * linf : d > 0)
				progress = true;
			if (!progress)
				return k;
		}
		return -1;
	}

	// What the budget must have done on a stage whose snap never becomes
	// feasible: the cap is checked before the window, so stagnation ends the
	// stage only when it is measured strictly before the cap.
	std::pair<std::string, int> expected_exit(const json &history, const FullHistoryOracle &oracle, const ALBudgetOptions &budget)
	{
		const int stagnant = first_stagnant_pass(history, oracle, budget);
		if (stagnant >= 0 && (budget.max_passes <= 0 || stagnant < budget.max_passes))
			return {"stagnation", stagnant};
		return {"pass_budget", budget.max_passes};
	}

	struct BoundedStageRun
	{
		json history;
		FullHistoryOracle oracle;
		size_t max_retained_seen = 0;
		size_t retained_after = 0;
		std::string reason;
		int passes = -1;
	};

	// Runs a never-feasible stage under `budget` on `problem`, sampling the
	// retained-state count at every pass and the caller's solution for the
	// oracle; checks the bound at every pass and the release afterwards.
	BoundedStageRun run_bounded_stage(ALSolver &solver, NLProblem &problem, const ALBudgetOptions &budget, Eigen::MatrixXd sol, const json &params)
	{
		solver.set_budget(budget);
		const size_t bound = budget.enabled() ? (budget.stagnation_window > 0 ? size_t(budget.stagnation_window) + 1 : 1) : 0;
		BoundedStageRun run;
		run.oracle.states.push_back(sol);
		solver.post_subsolve = [&](double) {
			run.oracle.states.push_back(sol);
			run.max_retained_seen = std::max(run.max_retained_seen, solver.retained_state_count());
			CHECK(solver.retained_state_count() <= bound);
		};
		try
		{
			solver.solve_al(problem, sol, params, linear, 1);
			FAIL("the budget must end the stage");
		}
		catch (const ALBudgetExhausted &e)
		{
			run.reason = e.details()["reason"].get<std::string>();
			run.passes = e.details()["passes"].get<int>();
		}
		run.history = solver.al_history();
		run.retained_after = solver.retained_state_count();
		return run;
	}

	// Every pass record's motion values equal the oracle's, and the count of
	// retained states after each pass is exactly what the window needs.
	void check_against_oracle(const BoundedStageRun &run, const ALBudgetOptions &budget)
	{
		const int W = budget.stagnation_window;
		const size_t bound = W > 0 ? size_t(W) + 1 : 1;
		REQUIRE(run.history.size() == size_t(run.passes) + 1);
		REQUIRE(run.oracle.states.size() == run.history.size());
		for (int k = 0; k <= run.passes; ++k)
		{
			CAPTURE(k, W);
			const json &row = run.history[k];
			CHECK(row["pass"] == k);
			CHECK(row["retained_states"] == std::min(size_t(k) + 1, bound));
			if (k == 0)
			{
				CHECK(row["moved"] == 0.0);
				CHECK(row["drift_over_window"].is_null());
				continue;
			}
			CHECK(row["moved"].get<double>() == run.oracle.moved(k));
			if (run.oracle.has_drift(k, W))
				CHECK(row["drift_over_window"].get<double>() == run.oracle.drift(k, W));
			else
				CHECK(row["drift_over_window"].is_null());
		}
		CHECK(run.max_retained_seen == std::min(size_t(run.passes) + 1, bound));
		CHECK(run.retained_after == 0);
		const auto [reason, at] = expected_exit(run.history, run.oracle, budget);
		CHECK(run.reason == reason);
		CHECK(run.passes == at);
	}
} // namespace

TEST_CASE("AL budget retains only the carried states its motion measures need, whatever the pass count", "[al_solver][al_budget]")
{
	// A never-feasible continuation (the gate never opens) with one Newton
	// step per pass and a pass cap: before RBR-03 the stage kept every
	// pass's full-space state (passes + 1 vectors); it needs the previous
	// one and the one W passes back.
	const auto run = [](const int max_passes, const int W) {
		const auto mass = identity2();
		auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
		GatedSnapProblem problem({std::make_shared<TwoQuarticsForm>()}, bc, mass, /*open_below=*/-1);
		ALSolver solver({bc}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
		ALBudgetOptions budget;
		budget.max_passes = max_passes;
		budget.stagnation_window = W;
		const auto result = run_bounded_stage(solver, problem, budget, Eigen::VectorXd::Constant(2, 10), parameters());
		check_against_oracle(result, budget);
		return result;
	};
	SECTION("cap only: the previous state alone")
	{
		const auto r = run(6, 0);
		CHECK(r.reason == "pass_budget");
		CHECK(r.passes == 6);
		for (int k = 0; k <= 6; ++k)
			CHECK(r.history[k]["retained_states"] == 1);
	}
	SECTION("window of one: the drift is the motion of the last pass")
	{
		const auto r = run(6, 1);
		for (int k = 1; k <= 6; ++k)
			CHECK(r.history[k]["drift_over_window"] == r.history[k]["moved"]);
	}
	SECTION("window shorter than the stage") { run(6, 2); }
	SECTION("window equal to the pass count: one drift, at the last pass")
	{
		const auto r = run(6, 6);
		for (int k = 1; k < 6; ++k)
			CHECK(r.history[k]["drift_over_window"].is_null());
		CHECK(r.history[6]["drift_over_window"].is_number());
		CHECK(r.history[6]["retained_states"] == 7);
	}
	SECTION("window longer than the stage: no drift, every state still needed")
	{
		for (const int W : {7, 100})
		{
			const auto r = run(6, W);
			for (int k = 1; k <= 6; ++k)
				CHECK(r.history[k]["drift_over_window"].is_null());
			CHECK(r.max_retained_seen == 7);
		}
	}
	SECTION("many passes: the bound does not grow with the pass count")
	{
		const auto r = run(200, 3);
		CHECK(r.passes == 200);
		CHECK(r.max_retained_seen == 4);
		const auto c = run(200, 0);
		CHECK(c.passes == 200);
		CHECK(c.max_retained_seen == 1);
	}
}

TEST_CASE("AL budget motion measures agree with a full-history oracle through rolled-back passes", "[al_solver][al_budget]")
{
	// The prescribed coordinate starts at .1 (target 0) under a form pulling
	// it towards 1: the first converged passes end farther from the target
	// than the start and are rolled back to the initial state (eta < 0), the
	// weight doubling each time, until the penalty dominates and the passes
	// are kept. The carried state therefore returns to pass 0 several times
	// before it moves.
	const auto mass = identity2();
	auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
	GatedSnapProblem problem({std::make_shared<PullAwayForm>()}, bc, mass, /*open_below=*/-1);
	ALSolver solver({bc}, 1, 2, 1e8, .99, [](const auto &) {});
	ALBudgetOptions budget;
	budget.max_passes = 12;
	budget.stagnation_window = 2;
	Eigen::MatrixXd start(2, 1);
	start << .1, 10;
	const auto run = run_bounded_stage(solver, problem, budget, start, parameters());
	check_against_oracle(run, budget);
	int rolled_back = 0, kept = 0;
	for (int k = 1; k <= run.passes; ++k)
	{
		CAPTURE(k);
		if (run.history[k]["rolled_back"].get<bool>())
		{
			++rolled_back;
			CHECK(run.history[k]["subsolve"]["outcome"] == "converged");
			CHECK(run.oracle.states[k] == run.oracle.states[0]);
		}
		else
		{
			++kept;
			CHECK(run.oracle.states[k] != run.oracle.states[0]);
		}
	}
	CHECK(rolled_back >= 2);
	CHECK(kept >= 2);
	CHECK(run.history[1]["rolled_back"] == true);
	CHECK(run.history[run.passes]["rolled_back"] == false);
	CHECK(run.reason == "pass_budget");
}

TEST_CASE("AL stagnation exit is unchanged by the bounded storage: the wall fixture stops where the full history says", "[al_solver][al_budget]")
{
	// The wall fixture of the stagnation test, under a window with and
	// without a cap: the exit pass, the reason and every motion value are
	// what the complete history gives, and the stage retained W + 1 states
	// from pass W on.
	const auto run = [](const int max_passes, const int W) {
		const auto mass = identity2();
		auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
		NLProblem problem(2, 0, {std::make_shared<QuadraticFreeForm>(), std::make_shared<WallForm>()}, {bc}, polysolve::linear::Solver::create(linear, logger()), 1, 1, mass, 1);
		ALSolver solver({bc}, 1, 2, 4, .99, [](const auto &) {});
		ALBudgetOptions budget;
		budget.max_passes = max_passes;
		budget.stagnation_window = W;
		Eigen::MatrixXd start(2, 1);
		start << 2, 2;
		const auto result = run_bounded_stage(solver, problem, budget, start, wall_parameters());
		check_against_oracle(result, budget);
		return result;
	};
	const auto window_only = run(0, 3);
	CHECK(window_only.reason == "stagnation");
	CHECK(window_only.passes >= 3);
	CHECK(window_only.max_retained_seen == 4);
	const auto both = run(100, 3);
	CHECK(both.reason == "stagnation");
	CHECK(both.passes == window_only.passes);
	CHECK(both.history.size() == window_only.history.size());
	for (size_t k = 0; k < both.history.size(); ++k)
	{
		CAPTURE(k);
		CHECK(both.history[k]["moved"] == window_only.history[k]["moved"]);
		CHECK(both.history[k]["drift_over_window"] == window_only.history[k]["drift_over_window"]);
		CHECK(both.history[k]["bc_residual_carried"] == window_only.history[k]["bc_residual_carried"]);
	}
	// A cap inside the window's reach wins (checked first).
	const auto capped = run(std::max(1, window_only.passes - 1), 3);
	CHECK(capped.reason == "pass_budget");
	CHECK(capped.passes == std::max(1, window_only.passes - 1));
}

TEST_CASE("AL stage without a budget retains no carried state", "[al_solver][al_budget][al_continuation]")
{
	// The PF-07 continuation with the budget off: nothing is stored for the
	// motion measures and every pass record says so.
	const auto mass = identity2();
	auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, mass, 0, Eigen::VectorXd::Zero(2));
	GatedSnapProblem problem({std::make_shared<TwoQuarticsForm>()}, bc, mass, .5);
	ALSolver preparation({bc}, 3, 2, 5, 1.0, [](const auto &) {}, restart_options(0), [](const auto &) { return true; });
	size_t max_retained = 0;
	preparation.post_subsolve = [&](double) { max_retained = std::max(max_retained, preparation.retained_state_count()); };
	Eigen::MatrixXd sol = Eigen::VectorXd::Constant(2, 10);
	REQUIRE_NOTHROW(preparation.solve_al(problem, sol, parameters(), linear, 1));
	CHECK(max_retained == 0);
	CHECK(preparation.retained_state_count() == 0);
	REQUIRE(preparation.al_history().size() >= 4);
	for (const json &row : preparation.al_history())
	{
		CHECK(row["retained_states"] == 0);
		CHECK(row["moved"] == 0.0);
		CHECK(row["drift_over_window"].is_null());
	}
}
