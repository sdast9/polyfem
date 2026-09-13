// RB-10 regressions (docs/rb-10-validation.md): the lagged friction form
// carries the normal force the barrier actually supplies, through every
// retuning path of the semi-implicit barrier, and its constitutive response
// has the Coulomb magnitude, the IPC smoothing, nonnegative dissipation for
// the velocity it is evaluated on and balanced point/edge forces.
// Real BarrierContactForm / FrictionForm on a single 2D point-over-edge
// stencil with a synthetic frozen Hessian (as tools/rb10/friction_probe.cpp).
// Not a physical-accuracy test; the quasistatic slip convention is
// characterized in the probe, not asserted here.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>

#include <ipc/friction/smooth_friction_mollifier.hpp>

#include <cmath>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Approx;

namespace
{
	// Floor edge (0,1) from -1 to 1 at y = 0; point 2 at (0, .2) above it.
	// Node-major DOFs: point x = 4, point y = 5.
	ipc::CollisionMesh make_mesh()
	{
		Eigen::MatrixXd p(3, 2);
		Eigen::MatrixXi e(1, 2);
		p.row(0) << -1, 0;
		p.row(1) << 1, 0;
		p.row(2) << 0, .2;
		e.row(0) << 0, 1;
		return ipc::CollisionMesh(p, e);
	}

	class Probe : public BarrierContactForm
	{
	public:
		Eigen::MatrixXd driving;
		Probe(const ipc::CollisionMesh &m, json opts = json::object())
			: BarrierContactForm(m, /*dhat=*/1, /*avg_mass=*/1, false, false, false, true, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts, Eigen::VectorXd::Ones(m.num_vertices()))
		{
			driving = 100 * Eigen::MatrixXd::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const Eigen::VectorXd &, StiffnessMatrix &h) { h = driving.sparseView(); });
		}
		// Production order: the birth refresh prices the stencil during the
		// solve, the between-steps refresh at the published endpoint captures
		// the coefficient that acted there (RB-20).
		void start(const Eigen::VectorXd &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false, false);
			refresh_semi_implicit_stiffness(x, false, true);
		}
		double kappa0() const { return collision_set_[0].stiffness_scale; }
	};

	// |y-gradient| of the barrier on the point at x: the normal force the
	// barrier supplies there.
	double barrier_normal_force(Probe &f, const Eigen::VectorXd &x)
	{
		f.solution_changed(x);
		Eigen::VectorXd g;
		f.first_derivative(x, g);
		return std::abs(g[5]);
	}
	// The normal force the friction potential uses: weight * N_lag * trim scale.
	double lagged_normal_force(const FrictionForm &fr)
	{
		REQUIRE(fr.friction_collision_set().size() == 1);
		const auto &c = fr.friction_collision_set()[0];
		return c.weight * c.normal_force_magnitude * fr.trim_scale();
	}
	constexpr double mu = .3, eps = 1e-3;
} // namespace

TEST_CASE("lagged friction constitutive response", "[friction_lag][friction_form]")
{
	auto m = make_mesh();
	Probe f(m);
	const Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
	f.start(z);
	FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
	fr.init_lagging(z);
	const double N = lagged_normal_force(fr);
	REQUIRE(N > 0);
	CHECK(N == Approx(barrier_normal_force(f, z)).epsilon(1e-12));

	for (double ratio : {-5., -2., -1., -.5, -.25, -.1, .1, .25, .5, 1., 2., 5.})
	{
		Eigen::VectorXd x = z;
		x[4] = ratio * eps;
		Eigen::VectorXd g;
		fr.first_derivative(x, g);
		INFO("slip / epsv = " << ratio);
		// Coulomb magnitude beyond epsv, the IPC mollifier inside; opposing the slip.
		const double expected = mu * N * ipc::smooth_friction_f1(std::abs(x[4]), eps) * (x[4] > 0 ? 1 : -1);
		CHECK(g[4] == Approx(expected).epsilon(1e-12));
		if (std::abs(ratio) >= 1)
			CHECK(std::abs(g[4]) == Approx(mu * N).epsilon(1e-12));
		// nonnegative dissipation for the velocity the potential is evaluated on
		CHECK(g.dot(x) >= 0);
		// the edge receives the opposite force
		CHECK(std::abs(g[0] + g[2] + g[4]) <= 1e-12 * mu * N);
		CHECK(std::abs(g[1] + g[3] + g[5]) <= 1e-12 * mu * N);
	}
}

TEST_CASE("lagged friction follows every semi-implicit retuning path", "[friction_lag][friction_form]")
{
	auto m = make_mesh();
	const Eigen::VectorXd z = Eigen::VectorXd::Zero(6);

	SECTION("trim bumps and calibration (RB-18 F6)")
	{
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		f.bump_trim(2);
		f.bump_trim(.25);
		CHECK(lagged_normal_force(fr) == Approx(barrier_normal_force(f, z)).epsilon(1e-12));
		f.set_system_gradient_provider([](const Eigen::VectorXd &x, Eigen::VectorXd &g) { g = Eigen::VectorXd::Zero(x.size()); g[5] = 50; });
		REQUIRE(f.calibrate_trim(z));
		CHECK(f.barrier_stiffness() != 1);
		CHECK(lagged_normal_force(fr) == Approx(barrier_normal_force(f, z)).epsilon(1e-12));
	}
	SECTION("mid-solve refresh: continuation keeps the lagged stencil's coefficient")
	{
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		const double before = barrier_normal_force(f, z);
		f.driving *= 4;
		f.refresh_semi_implicit_stiffness(z, false, false);
		CHECK(f.kappa0() == Approx(100));
		CHECK(barrier_normal_force(f, z) == Approx(before).epsilon(1e-12));
		CHECK(lagged_normal_force(fr) == Approx(before).epsilon(1e-12));
	}
	SECTION("mid-solve refresh without continuation leaves the lag stale (documented)")
	{
		Probe f(m, {{"force_continuation", false}});
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		const double before = barrier_normal_force(f, z);
		f.driving *= 4;
		f.refresh_semi_implicit_stiffness(z, false, false);
		CHECK(f.kappa0() == Approx(400));
		CHECK(barrier_normal_force(f, z) == Approx(4 * before).epsilon(1e-10));
		CHECK(lagged_normal_force(fr) == Approx(before).epsilon(1e-12));
	}
	SECTION("stall retune")
	{
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		f.driving *= 4;
		REQUIRE(f.retune_on_stall(z, 2));
		CHECK(f.barrier_stiffness() != 1);
		CHECK(f.kappa0() == Approx(100));
		CHECK(lagged_normal_force(fr) == Approx(barrier_normal_force(f, z)).epsilon(1e-12));
	}
	SECTION("between-steps refresh at a new endpoint, then the next solve's init_lagging")
	{
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		Eigen::VectorXd x1 = z;
		x1[4] = .05;
		x1[5] = -.05;
		f.solution_changed(x1);
		const double kappa_solve = f.kappa0();
		f.driving *= 4;
		f.update_barrier_stiffness(x1, Eigen::MatrixXd());
		CHECK(f.kappa0() == Approx(kappa_solve).epsilon(1e-12));
		fr.init_lagging(x1);
		CHECK(lagged_normal_force(fr) == Approx(barrier_normal_force(f, x1)).epsilon(1e-12));
	}
}

TEST_CASE("transient lagged friction: held body and reversal", "[friction_lag][friction_form]")
{
	auto m = make_mesh();
	Probe f(m);
	const Eigen::VectorXd z = Eigen::VectorXd::Zero(6);
	f.start(z);
	const double s = 5 * eps;
	Eigen::VectorXd x1 = z;
	x1[4] = s; // step 1 slid +s
	auto integrator = time_integrator::ImplicitTimeIntegrator::construct_time_integrator("ImplicitEuler");
	integrator->init(x1, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), /*dt=*/1);
	FrictionForm fr(m, integrator, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
	fr.init_lagging(x1);
	const double N = lagged_normal_force(fr);
	Eigen::VectorXd g;
	// held: no slip in the step, no friction force
	fr.first_derivative(x1, g);
	CHECK(g.norm() <= 1e-14 * mu * N);
	// reversal by s/2: full sliding force against the step's motion, positive dissipation
	Eigen::VectorXd x2 = x1;
	x2[4] = s / 2;
	fr.first_derivative(x2, g);
	CHECK(g[4] == Approx(-mu * N).epsilon(1e-12));
	CHECK(g.dot(x2 - x1) > 0);
	// advance by s/2: the mirror
	x2[4] = 1.5 * s;
	fr.first_derivative(x2, g);
	CHECK(g[4] == Approx(mu * N).epsilon(1e-12));
	CHECK(g.dot(x2 - x1) > 0);
}
