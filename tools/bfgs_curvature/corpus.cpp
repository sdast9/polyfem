// Fixed corpus for the BFGS curvature safeguard (audit stage 1).
//
// Runs the real PolySolve strategies and line searches over the objectives the
// audit reproduced, plus valid-pair controls, under each curvature policy and
// a sweep of the relative threshold. Emits one JSON object per row on stdout.
// Exit zero means the corpus completed, not that every solve converged: the
// status and error of each row say what happened.

#include <polysolve/nonlinear/Solver.hpp>
#include <polysolve/nonlinear/descent_strategies/BFGS.hpp>
#include <polysolve/nonlinear/descent_strategies/LBFGS.hpp>

#include <spdlog/sinks/null_sink.h>

#include <cmath>
#include <iostream>
#include <vector>

using namespace polysolve;
using namespace polysolve::nonlinear;
using V = Eigen::VectorXd;

namespace
{
	json scalar(double x) { return std::isfinite(x) ? json(x) : json(std::isnan(x) ? "NaN" : "Inf"); }

	void emit(const json &row) { std::cout << row.dump() << '\n'; }

	/// One-variable polynomial, coefficients lowest order first.
	struct Polynomial : Problem
	{
		std::vector<double> c;
		explicit Polynomial(std::vector<double> c) : c(std::move(c)) {}
		double value(const V &x) override
		{
			double v = 0;
			for (int i = int(c.size()) - 1; i >= 0; --i)
				v = v * x[0] + c[i];
			return v;
		}
		void gradient(const V &x, V &g) override
		{
			g = V::Zero(1);
			for (int i = int(c.size()) - 1; i >= 1; --i)
				g[0] = g[0] * x[0] + i * c[i];
		}
		void hessian(const V &x, THessian &h) override
		{
			double v = 0;
			for (int i = int(c.size()) - 1; i >= 2; --i)
				v = v * x[0] + i * (i - 1) * c[i];
			h.resize(1, 1);
			h.setZero();
			h.coeffRef(0, 0) = v;
		}
	};

	struct Rosenbrock : Problem
	{
		double value(const V &x) override { return 100 * std::pow(x[1] - x[0] * x[0], 2) + std::pow(1 - x[0], 2); }
		void gradient(const V &x, V &g) override
		{
			g.resize(2);
			g << -400 * x[0] * (x[1] - x[0] * x[0]) - 2 * (1 - x[0]), 200 * (x[1] - x[0] * x[0]);
		}
		void hessian(const V &x, THessian &h) override
		{
			Eigen::MatrixXd d(2, 2);
			d << 1200 * x[0] * x[0] - 400 * x[1] + 2, -400 * x[0], -400 * x[0], 200;
			h = d.sparseView();
		}
	};

	struct Quadratic : Problem
	{
		double curvature;
		explicit Quadratic(double curvature) : curvature(curvature) {}
		double value(const V &x) override { return 0.5 * curvature * x.squaredNorm(); }
		void gradient(const V &x, V &g) override { g = curvature * x; }
		void hessian(const V &x, THessian &h) override
		{
			h.resize(x.size(), x.size());
			h.setIdentity();
			h *= curvature;
		}
	};

	struct Case
	{
		std::string name;
		std::shared_ptr<Problem> problem;
		V start;
		double grad_norm_tol;
	};

	V vec(std::initializer_list<double> v) { return Eigen::Map<const V>(v.begin(), v.size()); }

	std::vector<Case> corpus()
	{
		std::vector<Case> cases;
		// Bounded-below polynomials whose first accepted Armijo step carries
		// zero and negative curvature (audit findings 1 and 2).
		cases.push_back({"x+x^2(x+1)^2", std::make_shared<Polynomial>(std::vector<double>{0, 1, 1, 2, 1}), vec({0.}), 1e-10});
		cases.push_back({"x^4/4-x^2/2+0.1x", std::make_shared<Polynomial>(std::vector<double>{0, 0.1, -0.5, 0, 0.25}), vec({0.}), 1e-10});
		cases.push_back({"rosenbrock", std::make_shared<Rosenbrock>(), vec({-1.2, 1.}), 1e-8});
		// Valid-pair controls across scales, in three variables.
		for (const double c : {1e-8, 1e-3, 1., 1e3, 1e8})
			cases.push_back({fmt::format("quadratic-{:g}", c), std::make_shared<Quadratic>(c), V::Ones(3), 1e-12 * c});
		return cases;
	}

	/// The list form runs the strategy alone; the string form is what the
	/// public configurations use, and appends gradient descent as a fallback.
	json parameters(const std::string &type, const std::string &search,
					const std::string &policy, const double tolerance, const int restart,
					const double grad_norm_tol, const bool fallback = false)
	{
		json options = {{"curvature_policy", policy}, {"curvature_tolerance", tolerance}, {"curvature_restart", restart}};
		if (fallback)
		{
			json params = {
				{"solver", type},
				{type, options},
				{"grad_norm_tol", grad_norm_tol},
				{"rel_grad_norm_tol", 0},
				{"max_iterations", 500},
				{"line_search", {{"method", search}}},
				{"advanced", {{"derivative_along_delta_x_tol", 0}}}};
			return params;
		}
		json strategy = options;
		strategy["type"] = type;
		return json{
			{"solver", json::array({strategy})},
			{"grad_norm_tol", grad_norm_tol},
			{"rel_grad_norm_tol", 0},
			{"max_iterations", 500},
			{"line_search", {{"method", search}}},
			{"advanced", {{"derivative_along_delta_x_tol", 0}}}};
	}

	void run(const Case &problem_case, const std::string &type, const std::string &search,
			 const std::string &policy, const double tolerance, const int restart,
			 const bool trace = false, const bool fallback = false)
	{
		spdlog::logger logger("corpus", std::make_shared<spdlog::sinks::null_sink_mt>());
		json row = {{"kind", "solve"}, {"objective", problem_case.name}, {"strategy", type}, {"line_search", search}, {"policy", policy}, {"tolerance", tolerance}, {"restart", restart}, {"fallback", fallback}};
		V x = problem_case.start;
		std::unique_ptr<Solver> solver;
		json steps = json::array();
		try
		{
			solver = Solver::create(
				parameters(type, search, policy, tolerance, restart, problem_case.grad_norm_tol, fallback),
				{{"solver", "Eigen::LDLT"}}, 1, logger);
			if (trace)
				solver->set_iteration_callback([&steps](const Criteria &c) {
					steps.push_back({{"iteration", c.iterations}, {"energy", scalar(c.energy)}, {"grad_norm", scalar(c.gradNorm)}, {"alpha", scalar(c.alpha)}, {"x_delta", scalar(c.xDelta)}, {"x_delta_dot_grad", scalar(c.xDeltaDotGrad)}});
					return false;
				});
			solver->minimize(*problem_case.problem, x);
			row["status"] = fmt::format("{}", status_message(solver->status()));
			row["iterations"] = solver->current_criteria().iterations;
		}
		catch (const std::exception &error)
		{
			row["status"] = "exception";
			row["error"] = error.what();
			if (solver)
				row["iterations"] = solver->current_criteria().iterations;
		}
		// The counters live in the strategy, so they are reported whether or
		// not the solve ended by an exception; the snapshot is the one the
		// last completed iteration wrote.
		if (solver && solver->info().contains("curvature_guard"))
			row["guard"] = solver->info()["curvature_guard"][type];
		if (trace)
			row["trace"] = steps;
		V grad;
		problem_case.problem->gradient(x, grad);
		row["final_grad_norm"] = scalar(grad.norm());
		row["converged"] = grad.allFinite() && grad.norm() < problem_case.grad_norm_tol;
		row["x"] = json::array();
		for (int i = 0; i < x.size(); ++i)
			row["x"].push_back(scalar(x[i]));
		emit(row);
	}
} // namespace

int main()
{
	const std::vector<Case> cases = corpus();
	constexpr int DEFAULT_RESTART = 1;

	// Every policy against every energy line search, alone and with the
	// gradient descent fallback the public string form appends. The audit
	// measured the same chain before the safeguard.
	for (const auto &problem_case : cases)
		for (const std::string &type : {"L-BFGS", "BFGS"})
			for (const std::string &search : {"Armijo", "RobustArmijo", "Backtracking"})
				for (const std::string &policy : {"Skip", "Damp"})
					for (const bool fallback : {false, true})
						run(problem_case, type, search, policy, 1e-8, DEFAULT_RESTART, false, fallback);

	// How many consecutive refused pairs the approximation may outlive. Zero
	// keeps it for the whole solve, which is what the plan's text alone asks
	// for; the others discard it once the refusals run on.
	for (const auto &problem_case : cases)
		for (const std::string &type : {"L-BFGS", "BFGS"})
			for (const std::string &policy : {"Skip", "Damp"})
				for (const int restart : {0, 1, 3, 6})
					for (const bool fallback : {false, true})
						run(problem_case, type, "Armijo", policy, 1e-8, restart, false, fallback);

	// Sensitivity of the relative threshold, holding everything else fixed.
	for (const auto &problem_case : cases)
		for (const std::string &type : {"L-BFGS", "BFGS"})
			for (const double tolerance : {0., 1e-14, 1e-8, 1e-2})
				run(problem_case, type, "Armijo", "Skip", tolerance, DEFAULT_RESTART);

	// Per-iteration traces of the cases where the policies disagreed.
	for (const auto &problem_case : cases)
		if (problem_case.name == "rosenbrock" || problem_case.name == "x^4/4-x^2/2+0.1x")
			for (const std::string &policy : {"Skip", "Damp"})
				for (const int restart : {0, DEFAULT_RESTART})
					run(problem_case, "L-BFGS", "Armijo", policy, 1e-8, restart, true);

	return 0;
}
