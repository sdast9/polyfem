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
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(budget), [&](const auto &) { ++retunes; });
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
		ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, opts, [](const auto &) {});
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

TEST_CASE("AL hard line search failures remain failures and clean shared solver", "[al_solver]")
{
	QuarticProblem problem;
	problem.block_steps = true;
	int retunes = 0;
	ALSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(2), [&](const auto &) { ++retunes; });
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
	ExposedSolver solver({}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) {});
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
	ALSolver preparation({bc}, 1, 2, 1e8, .99, [](const auto &) {}, restart_options(0), [](const auto &) {});
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
