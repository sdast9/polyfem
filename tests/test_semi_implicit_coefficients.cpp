// RB-18 regressions for the semi-implicit coefficient law (docs/rb-18-quick-fixes.md):
//   F1 positive-only batch median and relative floor,
//   F2 nonpositive local curvature keeps the previous value, never zero when a
//      batch reference exists,
//   F3 dhat^2-normalized first-contact conditioning cap (length-unit independent),
//   F4 NaN curvature / overflow without reference / invalid weight are errors,
//   F6 lagged friction follows the contact trim between lag updates,
//   F7 nonpositive curvature with no history uses |w^T H w|, then max|H|/dhat^2.
// Real BarrierContactForm with a synthetic frozen Hessian and identity mapping,
// mirroring tools/rb02/coefficient_probe.cpp. Not a physical-accuracy test.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace
{
	// `count` disjoint point-over-edge pairs, each (4i-1..4i+1)*L wide with the
	// point at height gap*L; pairs cannot collide with one another.
	ipc::CollisionMesh make_mesh(double L = 1, double gap = .2, int count = 1)
	{
		Eigen::MatrixXd p(3 * count, 2);
		Eigen::MatrixXi e(count, 2);
		for (int i = 0; i < count; ++i)
		{
			p.row(3 * i) << (4 * i - 1) * L, 0;
			p.row(3 * i + 1) << (4 * i + 1) * L, 0;
			p.row(3 * i + 2) << 4 * i * L, gap * L;
			e.row(i) << 3 * i, 3 * i + 1;
		}
		ipc::CollisionMesh mesh(p, e);
		mesh.can_collide = [](size_t a, size_t b) { return a / 3 == b / 3; };
		return mesh;
	}

	class Probe : public BarrierContactForm
	{
	public:
		Eigen::MatrixXd driving;

		Probe(const ipc::CollisionMesh &m, double support = 1, json opts = json::object(), double weight = 1)
			: BarrierContactForm(m, support, 1, false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts, Eigen::VectorXd::Ones(m.num_vertices()))
		{
			driving = 100 * Eigen::MatrixXd::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(weight);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h = driving.sparseView();
			});
		}

		void start(const Eigen::VectorXd &x, bool controller = false)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, controller);
		}

		// Sorted: the parallel broad phase does not order the collision set.
		std::vector<double> scales() const
		{
			std::vector<double> result;
			for (size_t i = 0; i < collision_set_.size(); ++i)
				result.push_back(collision_set_[i].stiffness_scale);
			std::sort(result.begin(), result.end());
			return result;
		}
	};
} // namespace

TEST_CASE("Semi-implicit batch median ignores zeros and applies a relative floor", "[semi_implicit_coefficients]")
{
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(18);

	SECTION("tiny outlier is raised to the relative floor")
	{
		// Two healthy blocks and one nearly singular one (1e-12). The median
		// of the positive values is 100, so the floor 100 / 1e4 raises the
		// outlier; a zero block would instead be handled by F7.
		auto mesh = make_mesh(1, .2, 3);
		Probe f(mesh, 1, {{"kappa_spread", 1e4}});
		f.driving.block(12, 12, 6, 6) *= 1e-14;
		f.start(x);
		const auto k = f.scales();
		REQUIRE(k.size() == 3);
		CHECK(k[0] == Approx(100. / 1e4)); // relative floor, not 1e-12
		CHECK(k[1] == Approx(100.));
		CHECK(k[2] == Approx(100.));
		f.solution_changed(x);
		CHECK(f.value(x) > 0);
		CHECK(f.diagnostic_state()["batch_median"] == 100.);
		CHECK(f.diagnostic_state()["batch_floor"] == Approx(100. / 1e4));
	}

	SECTION("positive batch keeps cap and gains floor")
	{
		auto mesh = make_mesh(1, .2, 3);
		Probe f(mesh, 1, {{"kappa_spread", 2}});
		const double values[3] = {1, 10, 1000};
		for (int i = 0; i < 3; ++i)
			f.driving.block(6 * i, 6 * i, 6, 6) = values[i] * Eigen::MatrixXd::Identity(6, 6);
		f.start(x);
		const auto k = f.scales();
		CHECK(k[0] == Approx(5.)); // floor = median / spread
		CHECK(k[1] == Approx(10.));
		CHECK(k[2] == Approx(20.)); // cap = spread * median
	}

	SECTION("disabled spread applies neither cap nor floor")
	{
		auto mesh = make_mesh(1, .2, 3);
		Probe f(mesh, 1, {{"kappa_spread", 0}});
		f.driving.block(12, 12, 6, 6) *= 1e-14;
		f.start(x);
		const auto k = f.scales();
		CHECK(k[0] == Approx(1e-12));
		CHECK(k[2] == Approx(100.));
	}

	SECTION("overflowing spread product disables the cap but keeps a finite coefficient")
	{
		auto mesh = make_mesh();
		Probe f(mesh, 1, {{"kappa_spread", 1e308}});
		f.start(Eigen::VectorXd::Zero(6));
		CHECK(f.scales()[0] == Approx(100.));
		CHECK(f.diagnostic_state()["batch_cap"].is_null());
	}
}

TEST_CASE("Semi-implicit nonpositive curvature keeps the previous coefficient", "[semi_implicit_coefficients]")
{
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	auto mesh = make_mesh();
	Probe f(mesh);
	f.start(x);
	REQUIRE(f.scales()[0] == Approx(100.));

	SECTION("negative definite on the second refresh")
	{
		f.driving *= -1;
		f.refresh_semi_implicit_stiffness(x, false);
		CHECK(f.scales()[0] == Approx(100.));
		CHECK(f.diagnostic_state()["curvature_fallback_count"] == 0);
	}

	SECTION("singular on the second refresh")
	{
		f.driving.setZero();
		f.refresh_semi_implicit_stiffness(x, false);
		CHECK(f.scales()[0] == Approx(100.));
	}

	SECTION("overflow on the second refresh")
	{
		f.driving(5, 5) = std::numeric_limits<double>::infinity();
		f.refresh_semi_implicit_stiffness(x, false);
		CHECK(f.scales()[0] == Approx(100.));
	}

	SECTION("F7/B: negative curvature with no history borrows its magnitude")
	{
		Probe g(mesh);
		g.driving *= -1;
		g.start(x);
		CHECK(g.scales()[0] == Approx(100.));
		CHECK(g.diagnostic_state()["curvature_abs_fallback_count"] == 1);
		CHECK(g.diagnostic_state()["curvature_fallback_count"] == 0);
	}

	SECTION("F7/E: singular along the normal uses the global Hessian scale")
	{
		// Curvature only on the x DOFs: w^T H w = 0 for a horizontal edge
		// (w has only y components), while max|H| = 100.
		Probe g(mesh, 1);
		g.driving.setZero();
		for (int i = 0; i < 6; i += 2)
			g.driving(i, i) = 100;
		g.start(x);
		CHECK(g.scales()[0] == Approx(100.)); // 100 / (dhat^2 = 1) / (weight = 1)
		CHECK(g.diagnostic_state()["curvature_global_fallback_count"] == 1);
	}

	SECTION("F7/E carries the dhat^2 and weight normalization")
	{
		Probe g(mesh, .5, json::object(), 4.);
		g.driving.setZero();
		for (int i = 0; i < 6; i += 2)
			g.driving(i, i) = 100;
		g.start(x);
		CHECK(g.scales()[0] == Approx(100. / (.25 * 4.)));
	}

	SECTION("identically zero Hessian is the only remaining zero, and is counted")
	{
		Probe g(mesh);
		g.driving.setZero();
		g.start(x);
		CHECK(g.scales()[0] == 0.);
		CHECK(g.diagnostic_state()["curvature_fallback_count"] == 1);
	}

	SECTION("B takes precedence over the batch floor")
	{
		auto mesh3 = make_mesh(1, .2, 3);
		Probe g(mesh3, 1, {{"kappa_spread", 2}});
		g.driving.block(12, 12, 6, 6) *= -1;
		g.start(Eigen::VectorXd::Zero(18));
		const auto k = g.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(100.));
		CHECK(k[2] == Approx(100.));
		CHECK(g.diagnostic_state()["curvature_abs_fallback_count"] == 1);
	}

	SECTION("previous value takes precedence over B")
	{
		f.driving *= -3; // |q| would be 300; the previous 100 wins
		f.refresh_semi_implicit_stiffness(x, false);
		CHECK(f.scales()[0] == Approx(100.));
	}

	SECTION("kappa_min is an absolute floor applied last")
	{
		Probe g(mesh, 1, {{"kappa_min", 1}});
		g.driving.setZero();
		g.start(x);
		CHECK(g.scales()[0] == Approx(1.));
	}
}

TEST_CASE("Semi-implicit first-contact conditioning cap is length-unit independent", "[semi_implicit_coefficients]")
{
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	for (double L : {1e-3, 1., 1e3})
	{
		CAPTURE(L);
		auto mesh = make_mesh(L, .8);
		Probe f(mesh, L);
		f.driving /= (L * L);
		f.set_barrier_stiffness(1e5);
		f.start(x, /*controller=*/true);
		// conditioning_cap (default 1e3) * max|H| / (weight * median * dhat^2)
		// = 1e3 * (100/L^2) / ((100/L^4) * L^2) = 1e3 at every scale.
		CHECK(f.barrier_stiffness() == Approx(1e3));
	}
}

TEST_CASE("Semi-implicit invalid curvature and weight are errors", "[semi_implicit_coefficients]")
{
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	auto mesh = make_mesh();

	SECTION("NaN curvature")
	{
		Probe f(mesh);
		f.driving(5, 5) = std::numeric_limits<double>::quiet_NaN();
		REQUIRE_THROWS_WITH(f.start(x), ContainsSubstring("NaN local curvature"));
	}

	SECTION("overflow with no previous value and no batch")
	{
		Probe f(mesh);
		f.driving(5, 5) = std::numeric_limits<double>::infinity();
		REQUIRE_THROWS_WITH(f.start(x), ContainsSubstring("overflowing local curvature"));
	}

	SECTION("overflow with a batch reference resolves to the cap")
	{
		auto mesh3 = make_mesh(1, .2, 3);
		Probe f(mesh3, 1, {{"kappa_spread", 2}});
		f.driving(17, 17) = std::numeric_limits<double>::infinity();
		f.start(Eigen::VectorXd::Zero(18));
		const auto k = f.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(100.));
		CHECK(k[2] == Approx(200.)); // cap = spread * median
	}

	SECTION("zero and subnormal weights")
	{
		for (double w : {0., 1e-310})
		{
			CAPTURE(w);
			Probe f(mesh, 1, json::object(), w);
			REQUIRE_THROWS_WITH(f.start(x), ContainsSubstring("form weight"));
		}
	}
}

TEST_CASE("Semi-implicit lagged friction follows the trim", "[semi_implicit_coefficients]")
{
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	auto mesh = make_mesh();
	Probe contact(mesh);
	contact.start(x);
	FrictionForm friction(mesh, nullptr, .01, .5, ipc::BroadPhaseMethod::HASH_GRID, contact, 2);
	friction.init_lagging(x);

	Eigen::VectorXd slip = x;
	slip[4] = .02; // tangential motion of the point
	const double v0 = friction.value(slip);
	REQUIRE(v0 > 0);
	Eigen::VectorXd g0;
	friction.first_derivative(slip, g0);
	StiffnessMatrix h0;
	friction.second_derivative(slip, h0);

	// The potential is linear in the lagged normal force: value, gradient and
	// Hessian scale exactly by the trim ratio, so the FD-verified derivative
	// consistency of the unscaled form carries over.
	contact.bump_trim(2.0);
	CHECK(friction.trim_scale() == Approx(2.0));
	CHECK(friction.value(slip) == Approx(2 * v0));
	Eigen::VectorXd g1;
	friction.first_derivative(slip, g1);
	CHECK((g1 - 2 * g0).norm() <= 1e-12 * (1 + g0.norm()));
	StiffnessMatrix h1;
	friction.second_derivative(slip, h1);
	CHECK((Eigen::MatrixXd(h1) - 2 * Eigen::MatrixXd(h0)).norm() <= 1e-12 * (1 + Eigen::MatrixXd(h0).norm()));

	// A lag update re-bases the scale at the new trim; the value is unchanged.
	friction.update_lagging(x, 1);
	CHECK(friction.trim_scale() == Approx(1.0));
	CHECK(friction.value(slip) == Approx(2 * v0));

	// Central finite differences of the rescaled gradient at the bumped trim.
	contact.bump_trim(1.5);
	Eigen::VectorXd g;
	friction.first_derivative(slip, g);
	for (int i = 0; i < 6; ++i)
	{
		Eigen::VectorXd p = slip, n = slip;
		p[i] += 1e-6;
		n[i] -= 1e-6;
		const double fd = (friction.value(p) - friction.value(n)) / 2e-6;
		CHECK(fd == Approx(g[i]).margin(1e-6 * (1 + std::abs(g[i]))));
	}
}
