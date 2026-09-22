#include <polysolve/nonlinear/Solver.hpp>
#include <polysolve/nonlinear/BoxConstraintSolver.hpp>
#include <polysolve/nonlinear/descent_strategies/BFGS.hpp>
#include <polysolve/nonlinear/descent_strategies/LBFGS.hpp>
#include <polysolve/nonlinear/descent_strategies/box_constraints/LBFGSB.hpp>
#include <spdlog/sinks/null_sink.h>
#include <iostream>
#include <iomanip>

using namespace polysolve;
using namespace polysolve::nonlinear;
using V = Eigen::VectorXd;

struct Polynomial : Problem
{
	std::vector<double> c;
	explicit Polynomial(std::vector<double> c) : c(c) {}
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
		double a = 0;
		for (int i = int(c.size()) - 1; i >= 2; --i)
			a = a * x[0] + i * (i - 1) * c[i];
		h.resize(1, 1);
		h.setZero();
		h.coeffRef(0, 0) = a;
	}
	void hessian(const V &x, TMatrix &h) override
	{
		THessian s;
		hessian(x, s);
		h = s;
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
	void hessian(const V &x, TMatrix &h) override
	{
		h.resize(2, 2);
		h << 1200 * x[0] * x[0] - 400 * x[1] + 2, -400 * x[0], -400 * x[0], 200;
	}
	void hessian(const V &x, THessian &h) override
	{
		TMatrix d;
		hessian(x, d);
		h = d.sparseView();
	}
};

json scalar(double x) { return std::isfinite(x) ? json(x) : json(std::isnan(x) ? "NaN" : "Inf"); }
json vec(const V &v)
{
	json j = json::array();
	for (double x : v)
		j.push_back(scalar(x));
	return j;
}
void emit(json j) { std::cout << j.dump() << '\n'; }

json parameters(const std::string &method, const std::string &ls, bool fallback = false)
{
	json p = {{"solver", method}, {"max_iterations", 1000}, {"grad_norm_tol", 1e-9}, {"first_grad_norm_tol", 1e-12}, {"rel_grad_norm_tol", 0}, {"advanced", {{"derivative_along_delta_x_tol", 0}}}, {"line_search", {{"method", ls}}}};
	if (!fallback && method != "L-BFGS-B")
		p["solver"] = json::array({json{{"type", method}}});
	if (method == "L-BFGS-B")
		p["box_constraints"] = {{"bounds", {-5, 5}}, {"max_change", 10}};
	return p;
}

int main()
{
	auto sink = std::make_shared<spdlog::sinks::null_sink_mt>();
	spdlog::logger logger("audit", sink);
	const json config = {{"L-BFGS", {{"history_size", 6}}}, {"L-BFGS-B", {{"history_size", 6}}}};
	const json linear = {{"solver", "Eigen::LDLT"}};
	Polynomial zero({0, 1, 1, 2, 1}), negative({0, .1, -.5, 0, .25}), quadratic({0, 0, 2});
	for (const std::string method : {"BFGS", "L-BFGS", "L-BFGS-B"})
	{
		for (const std::string problem : {"positive_curvature", "zero_curvature", "negative_curvature", "objective_retune"})
		{
			std::shared_ptr<DescentStrategy> d;
			if (method == "BFGS")
				d = std::make_shared<BFGS>(config, linear, 1, logger);
			else if (method == "L-BFGS")
				d = std::make_shared<LBFGS>(config, 1, logger);
			else
				d = std::make_shared<LBFGSB>(config, 1, logger);
			d->reset(1);
			V x0(1), x1(1), g0(1), g1(1), p0(1), p1(1);
			Polynomial *f = &quadratic;
			if (problem == "positive_curvature")
			{
				x0 << 1;
				x1 << .5;
			}
			if (problem == "zero_curvature")
			{
				f = &zero;
				x0 << 0;
				x1 << -1;
			}
			if (problem == "negative_curvature")
			{
				f = &negative;
				x0 << 0;
				x1 << -.1;
			}
			if (problem == "objective_retune")
			{
				x0 << 1;
				x1 << .75;
			}
			f->gradient(x0, g0);
			f->gradient(x1, g1);
			if (problem == "objective_retune")
			{
				g0 << .25;
				g1 << 3;
			}
			auto call = [&](const V &x, const V &g, V &p) {
				if (method == "L-BFGS-B")
					return std::static_pointer_cast<LBFGSB>(d)->compute_boxed_update_direction(*f, x, g, V::Constant(1, -5), V::Constant(1, 5), p);
				return d->compute_update_direction(*f, x, g, p);
			};
			bool ok0 = call(x0, g0, p0), ok1 = call(x1, g1, p1);
			emit({{"kind", "direction"}, {"method", method}, {"problem", problem}, {"s_dot_y", (x1 - x0).dot(g1 - g0)}, {"p0", vec(p0)}, {"p1", vec(p1)}, {"g1_dot_p1", scalar(g1.dot(p1))}, {"ok0", ok0}, {"ok1", ok1}});
		}
	}
	for (const std::string ls : {"Armijo", "RobustArmijo", "Backtracking"})
	{
		auto solver = Solver::create(parameters("L-BFGS", ls), linear, 1, logger);
		for (auto entry : std::vector<std::pair<std::string, Polynomial *>>{{"zero_curvature", &zero}, {"negative_curvature", &negative}})
		{
			V x = V::Zero(1), g, p;
			entry.second->gradient(x, g);
			p = -g;
			double alpha = solver->line_search()->line_search(x, p, *entry.second);
			V next = x + alpha * p, gn;
			entry.second->gradient(next, gn);
			emit({{"kind", "line_search"}, {"method", ls}, {"problem", entry.first}, {"alpha", scalar(alpha)}, {"old_f", entry.second->value(x)}, {"new_f", entry.second->value(next)}, {"s_dot_y", (next - x).dot(gn - g)}});
		}
	}
	Rosenbrock rosenbrock;
	for (const std::string method : {"BFGS", "L-BFGS", "L-BFGS-B"})
		for (const std::string ls : {"Armijo", "RobustArmijo", "Backtracking"})
			for (const bool fallback : {false, true})
			{
				if (method == "L-BFGS-B" && fallback)
					continue;
				for (const std::string problem : {"zero_curvature", "negative_curvature", "rosenbrock"})
				{
					Problem *f = problem == "zero_curvature" ? static_cast<Problem *>(&zero) : problem == "negative_curvature" ? static_cast<Problem *>(&negative)
																															   : static_cast<Problem *>(&rosenbrock);
					V x = V::Zero(problem == "rosenbrock" ? 2 : 1);
					if (problem == "rosenbrock")
						x << -1.2, 1;
					json p = parameters(method, ls, fallback);
					auto solver = method == "L-BFGS-B" ? BoxConstraintSolver::create(p, linear, 1, logger) : Solver::create(p, linear, 1, logger);
					std::string error;
					try
					{
						solver->minimize(*f, x);
					}
					catch (const std::exception &e)
					{
						error = e.what();
					}
					V g;
					f->gradient(x, g);
					emit({{"kind", "solve"}, {"method", method}, {"line_search", ls}, {"fallback", fallback}, {"problem", problem}, {"status", std::string(status_message(solver->status()))}, {"iterations", solver->current_criteria().iterations}, {"x", vec(x)}, {"grad_norm", scalar(g.norm())}, {"error", error}});
				}
			}
	try
	{
		auto solver = Solver::create(parameters("L-BFGS-B", "RobustArmijo"), linear, 1, logger);
		emit({{"kind", "forward_factory"}, {"result", "created"}});
	}
	catch (const std::exception &e)
	{
		emit({{"kind", "forward_factory"}, {"error", e.what()}});
	}
}
