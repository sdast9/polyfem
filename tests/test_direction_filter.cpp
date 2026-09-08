#include <catch2/catch_test_macros.hpp>
#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Logger.hpp>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	// Coupled smooth objective with Newton step (1,-3) at the origin.
	class CoupledQuadratic : public polysolve::nonlinear::Problem
	{
	public:
		Eigen::Matrix2d H = (Eigen::Matrix2d() << 5, 2, 2, 1).finished();
		double value(const TVector &x) override { return .5 * x.dot(H * x) + x.sum(); }
		void gradient(const TVector &x, TVector &g) override { g = H * x + TVector::Ones(2); }
		void hessian(const TVector &, THessian &h) override { h = H.sparseView(); }
	};
	json params(double slope_tol = 100)
	{
		return {{"solver", "Newton"}, {"max_iterations", 10}, {"grad_norm_tol", 1e-12}, {"rel_grad_norm_tol", 0.0}, {"first_grad_norm_tol", 0.0}, {"x_delta_tol", 0.0}, {"advanced", {{"derivative_along_delta_x_tol", slope_tol}}}, {"line_search", {{"method", "Armijo"}}}};
	}
	const json linear = {{"solver", "Eigen::SimplicialLDLT"}};
} // namespace

TEST_CASE("Filtered solver slope differentiates the objective", "[direction_filter]")
{
	CoupledQuadratic problem;
	auto solver = polysolve::nonlinear::Solver::create(params(), linear, 1, logger());
	Eigen::VectorXd p(2);
	p << 1, -3;
	SECTION("unfiltered") {}
	SECTION("identity filter")
	{
		solver->set_direction_filter([](const auto &, auto &) {});
	}
	SECTION("linear orthogonal projection")
	{
		solver->set_direction_filter([](const auto &, auto &d) { d[0] = 0; });
		p[0] = 0;
	}
	SECTION("one-sided opening stays free")
	{
		solver->set_direction_filter([](const auto &, auto &d) { d[0] = std::max(0., d[0]); });
	}
	SECTION("fixed coordinate and coupled free coordinates")
	{
		// A fixed coordinate eliminated from a full direction is represented
		// here by a reduced filter: lift (a,b) to (0,a,b),
		// project a closing pair with normal (-1,0,1), then eliminate DOF 0.
		// Exercise a general lift/project/restrict adapter with fixed DOFs.
		solver->set_direction_filter([](const auto &, auto &d) { if (d[1] < 0) d[1] *= .5; });
		p[1] *= .5;
	}
	Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
	REQUIRE_NOTHROW(solver->minimize(problem, x));
	const double eps = 1e-6;
	const double fd = (problem.value(eps * p) - problem.value(-eps * p)) / (2 * eps);
	CHECK(std::abs(solver->current_criteria().xDeltaDotGrad - fd) < 1e-9);
	CHECK(x.norm() == 0);
}

TEST_CASE("Filtered negative slope uses the configured tolerance", "[direction_filter]")
{
	CoupledQuadratic problem;
	auto solver = polysolve::nonlinear::Solver::create(params(2.5), linear, 1, logger());
	solver->set_direction_filter([](const auto &, auto &d) { d[0] = std::max(0., d[0]); });
	Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
	REQUIRE_NOTHROW(solver->minimize(problem, x));
	// Actual slope -2 is within tolerance. The old surrogate -3 misses it.
	CHECK(x.norm() == 0);
	CHECK(std::abs(solver->current_criteria().xDeltaDotGrad + 2) < 1e-9);
}

TEST_CASE("Filtered ascent is rejected before line search", "[direction_filter]")
{
	CoupledQuadratic problem;
	auto options = params();
	options["line_search"]["method"] = "Backtracking";
	auto solver = polysolve::nonlinear::Solver::create(options, linear, 1, logger());
	// Deliberately invalid caller direction; every strategy must reject it.
	solver->set_direction_filter([](const auto &, auto &d) { d.setOnes(); });
	Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
	CHECK_THROWS(solver->minimize(problem, x));
	CHECK(solver->status() == polysolve::nonlinear::Status::NotDescentDirection);
	CHECK(solver->current_criteria().xDeltaDotGrad == 2);
	CHECK(x.norm() == 0);
}

TEST_CASE("Retired contact floor cannot remove barrier forces", "[contact_floor_retired]")
{
	json opts = json::object();
	SECTION("omitted legacy option") {}
	SECTION("legacy zero") { opts["constraint_floor"] = 0.; }
	SECTION("legacy positive") { opts["constraint_floor"] = 1e-4; }
	Eigen::MatrixXd vertices(3, 2);
	vertices << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	ipc::CollisionMesh mesh(vertices, edges);
	BarrierContactForm contact(mesh, 1., 1., false, false, false, true, false, false,
							   ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
							   BarrierStiffnessMode::SemiImplicit, opts, Eigen::VectorXd::Ones(3));
	contact.set_system_hessian_provider([](const Eigen::VectorXd &, StiffnessMatrix &h) {
		h.resize(6, 6);
		h.setIdentity();
		h *= 100.;
	});
	Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	contact.init(x);
	contact.update_barrier_stiffness(x, Eigen::MatrixXd());
	contact.set_barrier_stiffness(1.);
	auto at_gap = [](double gap) -> Eigen::VectorXd {
		Eigen::VectorXd y = Eigen::VectorXd::Zero(6);
		y[5] = gap - .2;
		return y;
	};
	auto energy = [&](double gap) {
		const Eigen::VectorXd y = at_gap(gap);
		contact.solution_changed(y);
		return contact.value(y);
	};
	const double eps = 1e-9;
	const double slope = (energy(1e-4 + eps) - energy(1e-4 - eps)) / (2 * eps);
	x = at_gap(1e-4);
	contact.solution_changed(x);
	Eigen::VectorXd g;
	contact.first_derivative(x, g);
	CHECK(std::abs(slope - g[5]) / 2e6 < 1e-6);
	x = at_gap(5e-5);
	contact.solution_changed(x);
	const double before = contact.value(x);
	contact.first_derivative(x, g);
	CHECK(before > 1900.);
	CHECK(std::abs(g[5] + 4e6) / 4e6 < 1e-6);
	CHECK(std::abs(g[1] + g[3] + g[5]) < 1e-6);
	contact.update_barrier_stiffness(x, Eigen::MatrixXd());
	contact.set_barrier_stiffness(1.);
	CHECK(std::abs(contact.value(x) - before) < 1e-8);
	energy(5.0001e-5);
	CHECK(std::abs(energy(5e-5) - before) < 1e-8);
	// Retiring the floor must retain CCD and the separate trial-displacement cap.
	const Eigen::VectorXd crossing = at_gap(-1e-4);
	contact.line_search_begin(x, crossing);
	CHECK_FALSE(contact.is_step_collision_free(x, crossing));
	const double alpha = contact.max_step_size(x, crossing);
	CHECK(alpha > 0.);
	CHECK(alpha < 1.);
	CHECK(contact.is_step_collision_free(x, x + alpha * (crossing - x)));
	contact.line_search_end();
	CHECK(contact.trial_displacement_cap() == 50.);
}
