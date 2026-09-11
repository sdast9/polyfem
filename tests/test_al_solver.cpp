#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polysolve/linear/Solver.hpp>

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
	SECTION("soft stalls with nothing to retune")
	{
		QuarticProblem problem;
		int retunes = 0;
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(20), [&](const auto &) { ++retunes; return false; });
		Eigen::MatrixXd sol = Eigen::VectorXd::Constant(1, 10);
		REQUIRE_THROWS_WITH(solver.solve_reduced(problem, sol, parameters(), linear, 1), ContainsSubstring("Final reduced solve did not converge"));
		CHECK(sol(0, 0) == 10);
		CHECK(retunes == 2);
		CHECK(solver.info()["outcome"] == "interrupted");
		CHECK(solver.info()["termination_reason"] == "stall persisted with no retunable contact state");
		CHECK(solver.info()["unchanged_restarts"] == 2);
	}

	SECTION("a retune that changes something resets the counter")
	{
		QuarticProblem problem;
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
