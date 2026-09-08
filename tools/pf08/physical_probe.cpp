// PF-08 manufactured contact experiment. One configuration per process avoids
// the known function-static collision-position cache across form instances.
#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polysolve/linear/Solver.hpp>
#include <iostream>
#include <cmath>

using namespace polyfem;
using namespace polyfem::solver;
using V = Eigen::VectorXd;

// A translation-invariant, coupled spring connecting the free point to the
// prescribed corner. This is a manufactured discrete model, not a FEM element.
struct SpringLoad : Form
{
	Eigen::Matrix2d K;
	Eigen::Vector2d rest, load = Eigen::Vector2d::Zero();
	std::string name() const override { return "manufactured-spring-load"; }
	double value_unweighted(const V &x) const override
	{
		Eigen::Vector2d d = x.tail<2>() - x.head<2>() - rest;
		return .5 * d.dot(K * d) - load.dot(x.tail<2>());
	}
	void first_derivative_unweighted(const V &x, V &g) const override
	{
		Eigen::Vector2d f = K * (x.tail<2>() - x.head<2>() - rest);
		g = V::Zero(x.size());
		g.head<2>() = -f;
		g.tail<2>() = f - load;
	}
	void second_derivative_unweighted(const V &x, StiffnessMatrix &h) const override
	{
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, x.size());
		A.leftCols<2>() = -Eigen::Matrix2d::Identity();
		A.rightCols<2>() = Eigen::Matrix2d::Identity();
		h = (A.transpose() * K * A).sparseView();
	}
};

int main(int argc, char **argv)
{
	if (argc != 7)
		return 2;
	logger().set_level(spdlog::level::off);
	const double L = std::stod(argv[1]), S = std::stod(argv[2]);
	const int steps = std::stoi(argv[3]);
	const bool moving = std::stoi(argv[4]), coupled = std::stoi(argv[5]);
	const double floor = std::stod(argv[6]);
	Eigen::MatrixXd vertices(4, 2);
	vertices << 0, 0, 2, 0, 0, 2, .2, .3;
	vertices *= L;
	Eigen::MatrixXi edges(coupled ? 2 : 1, 2);
	edges.row(0) << 0, 1;
	if (coupled)
		edges.row(1) << 0, 2;
	ipc::CollisionMesh mesh(vertices, edges);
	// Pin trim to one for a well-defined energy/work comparison. This is an
	// explicitly frozen-objective experiment, not the production controller.
	json opts = {{"constraint_floor", floor}, {"trim_min", 1.}, {"trim_max", 1.}, {"refresh_interval", 0}};
	auto contact = std::make_shared<BarrierContactForm>(mesh, L, 1., false, false,
														false, true, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8 * L,
														1000000, BarrierStiffnessMode::SemiImplicit, opts, V::Ones(4));
	contact->set_system_hessian_provider([=](const V &, StiffnessMatrix &h) {
		h.resize(8, 8);
		h.setIdentity();
		h *= 100 * S / (L * L);
	});
	V zero = V::Zero(8);
	contact->init(zero);
	contact->update_barrier_stiffness(zero, Eigen::MatrixXd());
	contact->set_barrier_stiffness(1.);
	auto spring = std::make_shared<SpringLoad>();
	spring->K << 100, 30, 30, 80;
	spring->K *= S / (L * L);
	spring->rest = Eigen::Vector2d(.1, .1) * L;
	const json linear = {{"solver", "Eigen::SimplicialLDLT"}};
	const json params = {{"solver", "Newton"}, {"norm_type", "Euclidean"}, {"max_iterations", 100}, {"grad_norm_tol", 1e-9 * S / L}, {"rel_grad_norm_tol", 0.}, {"first_grad_norm_tol", 0.}, {"x_delta_tol", 0.}, {"allow_non_grad_convergence", false}, {"line_search", {{"method", "Backtracking"}}}};
	StiffnessMatrix mass(8, 8);
	mass.setIdentity();
	std::vector<int> fixed{0, 1, 2, 3, 4, 5};
	json result = {{"length_scale", L}, {"objective_scale", S}, {"steps", steps}, {"moving", moving}, {"coupled", coupled}, {"floor", floor}, {"inversions", "not applicable: no volume elements"}};
	Eigen::MatrixXd solution = zero;
	bool passed = true;
	for (int step = 0; step <= steps; ++step)
	{
		const double t = double(step) / steps;
		V target = V::Zero(8), exact = V::Zero(8);
		Eigen::Vector2d shift = moving ? Eigen::Vector2d(.05 * t, .04 * t) * L : Eigen::Vector2d::Zero().eval();
		for (int v = 0; v < 4; ++v)
			exact.segment<2>(2 * v) = shift;
		for (int v = 0; v < 3; ++v)
			target.segment<2>(2 * v) = shift;
		exact.tail<2>() += Eigen::Vector2d(.03 * t, -.02 * t) * L;
		// Manufacture external load from a five-point ENERGY difference,
		// independently of the analytic gradient used by Newton.
		spring->load.setZero();
		auto energy = [&](const V &x) {
			contact->solution_changed(x);
			return contact->value(x) + spring->value(x);
		};
		Eigen::Vector2d load;
		const double eps = 1e-5 * L;
		for (int j = 0; j < 2; ++j)
		{
			V d = V::Zero(8);
			d[6 + j] = eps;
			load[j] = (-energy(exact + 2 * d) + 8 * energy(exact + d) - 8 * energy(exact - d) + energy(exact - 2 * d)) / (12 * eps);
		}
		spring->load = load;
		auto bc = std::make_shared<BCLagrangianForm>(8, fixed, mass, 0, target);
		NLProblem problem(8, 0, {spring, contact}, {bc},
						  polysolve::linear::Solver::create(linear, logger()), L, S / L, mass, 2);
		ALSolver solver({bc}, 1e3 * S / (L * L), 2, 1e8 * S / (L * L), .99, [](const auto &) {});
		solver.direction_filter = [&](const V &x, V &p) {
			const V full = problem.reduced_to_full(x);
			V direction = problem.reduced_to_full(x + p) - full;
			if (contact->project_floor_pairs(full, direction) > 0)
				p = problem.full_to_reduced(full + direction) - x;
		};
		V exact_contact_gradient, exact_spring_gradient;
		contact->solution_changed(exact);
		contact->first_derivative(exact, exact_contact_gradient);
		spring->first_derivative(exact, exact_spring_gradient);
		json row = {{"step", step}, {"manufactured_force_fd_error", (exact_contact_gradient + exact_spring_gradient).tail<2>().norm() / (S / L)}};
		try
		{
			solver.solve_al(problem, solution, params, linear, 1);
			solver.solve_reduced(problem, solution, params, linear, 1);
			V x = solution, gc, gs;
			contact->solution_changed(x);
			contact->first_derivative(x, gc);
			spring->first_derivative(x, gs);
			V residual = gc + gs;
			Eigen::Vector2d reactions = Eigen::Vector2d::Zero();
			for (int v = 0; v < 3; ++v)
				reactions += residual.segment<2>(2 * v);
			const double bc_error = (x.head(6) - target.head(6)).lpNorm<Eigen::Infinity>() / L;
			const double free_error = residual.tail<2>().norm() / (S / L);
			const double balance = (reactions + load).norm() / (S / L);
			const double displacement_error = (x - exact).lpNorm<Eigen::Infinity>() / L;
			const double gap = std::sqrt(contact->collision_set().compute_minimum_distance(mesh, contact->compute_displaced_surface(x))) / L;
			row.update({{"bc_error", bc_error}, {"free_residual", free_error}, {"reaction_balance", balance}, {"displacement_error", displacement_error}, {"gap_over_dhat", gap}, {"normalized_energy", energy(x) / S}, {"collision_count", contact->collision_set().size()}, {"outcome", solver.info().value("outcome", "missing")}, {"termination_reason", solver.info().value("termination_reason", "missing")}});
			row["passed"] = bc_error < 1e-10 && free_error < 1e-6 && balance < 1e-6 && displacement_error < 1e-7 && gap > 0;
		}
		catch (const std::exception &e)
		{
			row["error"] = e.what();
			row["solver_info"] = solver.info();
			row["passed"] = false;
		}
		passed = passed && row["passed"].get<bool>();
		result["solves"].push_back(row);
		if (!row["passed"].get<bool>())
			break;
	}
	// Conservative work on a prescribed path; Simpson refinement separates
	// quadrature error from the analytic-force / energy derivative contract.
	V a = V::Zero(8), b = V::Zero(8);
	b.tail<2>() << .03 * L, -.02 * L;
	spring->load.setZero();
	auto internal_energy = [&](const V &x) { contact->solution_changed(x); return contact->value(x)+spring->value(x); };
	const double delta = internal_energy(b) - internal_energy(a);
	for (int n : {4, 8, 16, 32})
	{
		double work = 0;
		for (int i = 0; i <= n; ++i)
		{
			V x = a + (double(i) / n) * (b - a), gc, gs;
			contact->solution_changed(x);
			contact->first_derivative(x, gc);
			spring->first_derivative(x, gs);
			work += (i == 0 || i == n ? 1 : i % 2 ? 4
												  : 2)
					* (gc + gs).dot(b - a) / (3 * n);
		}
		result["conservative_work"].push_back({{"intervals", n}, {"energy_change", delta / S}, {"work", work / S}, {"error", std::abs(work - delta) / S}});
	}
	// Lagged friction: fixed normal geometry, prescribed sliding velocities.
	// The frozen lag is deliberate; no fixed-point convergence is asserted.
	contact->solution_changed(zero);
	FrictionForm friction(mesh, nullptr, 1e-3 * L, .3, ipc::BroadPhaseMethod::HASH_GRID, *contact, 2);
	friction.init_lagging(zero);
	V normal_gradient;
	contact->first_derivative(zero, normal_gradient);
	for (double slip : {-0.1, -0.0001, 0., 0.0001, 0.1})
	{
		V v = V::Zero(8), g;
		v[6] = slip * L;
		friction.first_derivative(v, g);
		Eigen::Vector2d net = Eigen::Vector2d::Zero();
		for (int i = 0; i < 4; ++i)
			net += g.segment<2>(2 * i);
		const double dissipation = g.dot(v) / S;
		V dv = V::Zero(8);
		dv[6] = 1e-6 * L;
		const double fd = (friction.value(v + dv) - friction.value(v - dv)) / (2e-6 * L);
		const double derivative_error = std::abs(fd - g[6]) / (S / L);
		const double plateau_error = std::abs(std::abs(g[6]) - .3 * std::abs(normal_gradient[7])) / (S / L);
		result["friction"].push_back({{"slip", slip}, {"dissipation", dissipation}, {"net_force", net.norm() / (S / L)}, {"lag_budget", friction.max_lagging_iterations()}, {"derivative_error", derivative_error}, {"plateau_error", std::abs(slip) > .001 ? json(plateau_error) : json(nullptr)}});
		passed = passed && dissipation >= -1e-10 && net.norm() / (S / L) < 1e-7 && derivative_error < 1e-3;
		if (std::abs(slip) > .001)
			passed = passed && dissipation > 0 && plateau_error < 1e-7;
	}
	passed = passed && result["conservative_work"].back()["error"].get<double>() < 1e-6;
	result["passed"] = passed;
	std::cout << result.dump(2) << std::endl;
	return passed ? 0 : 1;
}
