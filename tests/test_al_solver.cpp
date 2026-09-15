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
			// The fraction is probed while the snap is blocked (the last
			// record is the feasible snap that ended the stage).
			CHECK(history_on[k]["gate"]["ccd_fraction"].is_number() == (history_on[k]["gate"]["feasible"] == false));
		}
		// Without a budget the record keeps the historical short circuit:
		// the collision gate is not evaluated once an earlier gate fails, and
		// the CCD fraction is not probed.
		for (size_t k = 0; k < history_off.size(); ++k)
			CHECK(history_off[k]["gate"]["ccd_fraction"].is_null());
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
