// BFGS audit stage 2 (docs/bfgs-convergence-audit-20260922.md, finding 2;
// record docs/bfgs-objective-generation-20260922.md).
//
// A quasi-Newton secant pair is a secant of one function. A form that retunes
// itself during a solve -- a barrier stiffness or trim moved by the in-solve
// controller, a refreshed per-contact snapshot, a refined quadrature -- is a
// different function afterwards, so the next pair subtracts gradients of two
// functions and is a secant of neither. Every form counts the changes to its
// own objective, the problem sums them, and PolySolve discards the history
// before the next pair is formed.
//
// These regressions pin what counts as a change, what does not, and that the
// count reaches the nonlinear solver. They are not a physical-accuracy test,
// and they do not change any retuning law.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>

#include <polysolve/linear/Solver.hpp>
#include <polysolve/nonlinear/Solver.hpp>

#include <memory>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Approx;

namespace
{
	// 2D: floor edge (0,1) from -1 to 1 at y = 0; free point 2 above it.
	ipc::CollisionMesh make_mesh(const double height = .2)
	{
		Eigen::MatrixXd p(3, 2);
		Eigen::MatrixXi e(1, 2);
		p.row(0) << -1, 0;
		p.row(1) << 1, 0;
		p.row(2) << 0, height;
		e.row(0) << 0, 1;
		return ipc::CollisionMesh(p, e);
	}

	// Semi-implicit barrier on that fixture with a synthetic driving Hessian
	// (as test_step_rollback.cpp and test_kappa_continuity.cpp).
	class Probe : public BarrierContactForm
	{
	public:
		Eigen::MatrixXd driving;
		explicit Probe(const ipc::CollisionMesh &m)
			: BarrierContactForm(m, /*dhat=*/1, /*avg_mass=*/1, false, false, false, true, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, json::object(),
								 Eigen::VectorXd::Ones(m.num_vertices()))
		{
			driving = 100 * Eigen::MatrixXd::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const Eigen::VectorXd &, StiffnessMatrix &h) { h = driving.sparseView(); });
		}
		void refresh(const Eigen::VectorXd &x) { refresh_semi_implicit_stiffness(x, false, false); }
	};

	/// A one-variable quadratic whose curvature can be retuned, reporting the
	/// change the way the contact forms do.
	class Retunable : public Form
	{
	public:
		double curvature = 0.25;
		double center = 0;

		std::string name() const override { return "retunable"; }
		double value_unweighted(const Eigen::VectorXd &x) const override
		{
			return 0.5 * curvature * (x.array() - center).matrix().squaredNorm();
		}
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override
		{
			g = curvature * (x.array() - center).matrix();
		}
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(x.size(), x.size());
			h.setIdentity();
			h *= curvature;
		}

		/// A retune the form reports, as a barrier trim bump does.
		void retune(const double new_curvature, const double new_center = 0)
		{
			curvature = new_curvature;
			center = new_center;
			note_objective_change("test retune");
		}

		/// The same retune with the report suppressed: the behavior before
		/// this signal existed.
		void retune_silently(const double new_curvature, const double new_center = 0)
		{
			curvature = new_curvature;
			center = new_center;
		}

		/// Rebuilding a cache for the same function is not a change.
		void solution_changed(const Eigen::VectorXd &) override { ++cache_rebuilds; }
		int cache_rebuilds = 0;
	};

	StiffnessMatrix identity_mass(const int n)
	{
		StiffnessMatrix m(n, n);
		m.setIdentity();
		return m;
	}

	/// One retunable form, retuned once, after a chosen accepted iterate.
	class RetunedProblem : public NLProblem
	{
	public:
		RetunedProblem(const std::shared_ptr<Retunable> &form, const int retune_after, const bool reports)
			: NLProblem(1, 0, {form}, {}, nullptr, 1, 1, identity_mass(1), 1),
			  form_(form), retune_after_(retune_after), reports_(reports) {}

		void post_step(const polysolve::nonlinear::PostStepData &data) override
		{
			NLProblem::post_step(data);
			if (retuned || data.iter_num != retune_after_)
				return;
			// A different curvature and a different minimizer, so the solve
			// has to keep going under the new objective.
			retuned = true;
			if (reports_)
				form_->retune(9., .5);
			else
				form_->retune_silently(9., .5);
		}

		bool retuned = false;

	private:
		std::shared_ptr<Retunable> form_;
		const int retune_after_;
		const bool reports_;
	};

	json solver_parameters(const std::string &type)
	{
		return {
			{"solver", json::array({json{{"type", type}}})},
			{"max_iterations", 200},
			{"grad_norm_tol", 1e-12},
			{"rel_grad_norm_tol", 0.0},
			{"first_grad_norm_tol", 0.0},
			{"x_delta_tol", 0.0},
			{"line_search", {{"method", "Armijo"}}},
			{"advanced", {{"derivative_along_delta_x_tol", 0.0}}}};
	}
	const json linear_solver = {{"solver", "Eigen::SimplicialLDLT"}};
} // namespace

TEST_CASE("a contact form counts only real changes to its objective", "[objective_generation]")
{
	auto mesh = make_mesh();
	Probe f(mesh);
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	CHECK(f.objective_generation() == 0);

	// Building the collision set is not a change: it is a cache of the same
	// function at these coordinates.
	f.init(x);
	CHECK(f.objective_generation() == 0);
	f.solution_changed(x);
	CHECK(f.objective_generation() == 0);

	// Re-estimating the per-contact coefficients is.
	f.refresh(x);
	const uint64_t after_refresh = f.objective_generation();
	CHECK(after_refresh > 0);

	// Evaluating the form is not.
	Eigen::VectorXd grad;
	f.value(x);
	f.first_derivative(x, grad);
	REQUIRE(grad.norm() > 0);
	f.solution_changed(x);
	CHECK(f.objective_generation() == after_refresh);

	// A trim that moves is a change; one that does not move is not.
	const double trim = f.barrier_stiffness();
	f.bump_trim(2);
	REQUIRE(f.barrier_stiffness() != trim);
	CHECK(f.objective_generation() == after_refresh + 1);
	f.bump_trim(1);
	CHECK(f.barrier_stiffness() == Approx(2 * trim));
	CHECK(f.objective_generation() == after_refresh + 1);

	// The start point PolySolve reports before its first iteration must not
	// retune anything: the solve would discard its first pair for nothing.
	const json info = json::object();
	const uint64_t before_start_point = f.objective_generation();
	f.post_step(polysolve::nonlinear::PostStepData(0, info, x, grad));
	CHECK(f.objective_generation() == before_start_point);
}

TEST_CASE("the problem sums the generations of all its forms", "[objective_generation]")
{
	auto first = std::make_shared<Retunable>();
	auto second = std::make_shared<Retunable>();
	FullNLProblem problem({first, second});
	CHECK(problem.objective_generation() == 0);

	first->retune(2.);
	const uint64_t after_first = problem.objective_generation();
	CHECK(after_first == 1);

	second->retune(3.);
	CHECK(problem.objective_generation() == after_first + 1);

	// A silent retune is invisible: that is the defect this signal repairs,
	// stated so that a form added later without the report is caught here.
	second->retune_silently(4.);
	CHECK(problem.objective_generation() == after_first + 1);

	// The penalty (AL) forms are part of the objective the reduced problem
	// minimizes, so their changes count too.
	auto form = std::make_shared<Retunable>();
	auto bc = std::make_shared<BCLagrangianForm>(2, std::vector<int>{0}, identity_mass(2), 0, Eigen::VectorXd::Zero(2));
	NLProblem reduced(2, 0, {form}, {bc},
					  polysolve::linear::Solver::create(linear_solver, logger()), 1, 1, identity_mass(2), 1);
	CHECK(reduced.objective_generation() == 0);
	form->retune(2.);
	CHECK(reduced.objective_generation() == 1);
	bc->set_weight(2 * bc->weight());
	CHECK(reduced.objective_generation() == 1); // the caller's own ramp, between solves
	auto saved = bc->save_state();
	bc->set_weight(4 * bc->weight());
	bc->restore_state(*saved, Eigen::VectorXd::Zero(2));
	CHECK(reduced.objective_generation() == 2); // a rolled-back weight is a change
}

TEST_CASE("a form retuned mid-solve discards the quasi-Newton history", "[objective_generation]")
{
	for (const std::string &type : {"L-BFGS", "BFGS"})
	{
		for (const bool reports : {true, false})
		{
			CAPTURE(type, reports);
			auto form = std::make_shared<Retunable>();
			RetunedProblem problem(form, /*retune_after=*/1, reports);

			auto solver = polysolve::nonlinear::Solver::create(
				solver_parameters(type), type == "BFGS" ? json{{"solver", "Eigen::LDLT"}} : linear_solver,
				1, logger());
			Eigen::VectorXd x = Eigen::VectorXd::Constant(1, 1.);
			REQUIRE_NOTHROW(solver->minimize(problem, x));
			REQUIRE(problem.retuned);
			CHECK(x[0] == Approx(.5));       // the retuned objective's minimizer
			CHECK(form->cache_rebuilds > 0); // the caches were rebuilt throughout

			const json info = solver->info();
			CHECK(info["objective_changes"] == (reports ? 1 : 0));
			const json resets = info["curvature_guard"][type]["resets"];
			CHECK(resets.count("objective_changed") == (reports ? 1u : 0u));
		}
	}
}
