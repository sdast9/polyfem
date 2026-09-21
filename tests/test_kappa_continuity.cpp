// RB-20 / RB-21 regressions (docs/rb-20-force-continuation.md,
// docs/rb-21-parent-keyed-kappa.md):
//   RB-20 force continuation: a coefficient active at a published endpoint is
//         carried across refreshes; the fresh Hessian estimate is used only
//         for new keys; the barrier force at the endpoint is unchanged by the
//         refresh; continued values bypass the batch floor/cap; the optional
//         pull ratio bounds the fresh estimate's influence.
//   RB-21 parent identity: the toolkit records every candidate contribution
//         on the built collision; a coefficient is keyed on the parent and a
//         collision's scale is the contribution-weighted mean, so the energy
//         is continuous across closest-feature switches and continuation
//         survives them.
//   RBR-04 (docs/rb-review-repair-plan-20260920.md): the improved max
//         operator's duplicate-removal corrections are negative contributions,
//         under which the positive-parent mean is no longer a sum of parent
//         potentials (a finite |k1 - k2| / 2 * b(d) jump at a corner); the
//         semi-implicit form refuses that operator by name, the other
//         stiffness modes keep it unchanged.
// Real BarrierContactForm with a synthetic frozen Hessian and identity
// mapping (as tools/rb02/coefficient_probe.cpp). Not a physical-accuracy test.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>

#include <ipc/broad_phase/hash_grid.hpp>
#include <ipc/potentials/barrier_potential.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Approx;

namespace
{
	// `count` disjoint point-over-edge pairs (vertices 3i, 3i+1 = edge,
	// 3i+2 = point at height gap*L); pairs cannot collide with one another.
	ipc::CollisionMesh make_pairs(double L = 1, double gap = .2, int count = 1)
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

	// A corner: edge e0 = (v0, v1) along -x, edge e1 = (v1, v2) down -y (when
	// `closed`), and a free point v3 at (x, gap). For x < 0 the point is in
	// e0's interior region (EV), for x > 0 beyond the corner v1 (VV from both
	// edges); the distance to the corner equals the distance to e0 at x = 0.
	ipc::CollisionMesh make_corner(bool closed, double x, double gap = .2)
	{
		Eigen::MatrixXd p(4, 2);
		p.row(0) << -1, 0;
		p.row(1) << 0, 0;
		p.row(2) << 0, -1;
		p.row(3) << x, gap;
		Eigen::MatrixXi e(closed ? 2 : 1, 2);
		e.row(0) << 0, 1;
		if (closed)
			e.row(1) << 1, 2;
		ipc::CollisionMesh mesh(p, e);
		mesh.can_collide = [](size_t a, size_t b) { return (a == 3) != (b == 3); };
		return mesh;
	}

	class Probe : public BarrierContactForm
	{
	public:
		Eigen::MatrixXd driving;

		Probe(const ipc::CollisionMesh &m, json opts = json::object(), double support = 1)
			: BarrierContactForm(m, support, 1, false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts, Eigen::VectorXd::Ones(m.num_vertices()))
		{
			driving = 100 * Eigen::MatrixXd::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h = driving.sparseView();
			});
		}

		/// Solve start: build the set and the first snapshot (as production
		/// does before any endpoint is published)
		void start(const Eigen::VectorXd &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false, false);
		}
		/// Between-steps refresh at a published endpoint (RB-20 capture)
		void publish(const Eigen::VectorXd &x)
		{
			solution_changed(x);
			refresh_semi_implicit_stiffness(x, false, true);
		}
		/// Mid-solve refresh (birth / stall retune)
		void refresh_mid_solve(const Eigen::VectorXd &x)
		{
			solution_changed(x);
			refresh_semi_implicit_stiffness(x, false, false);
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
		const ipc::NormalCollisions &collisions() const { return collision_set_; }
		Eigen::VectorXd grad(const Eigen::VectorXd &x) const
		{
			Eigen::VectorXd g;
			first_derivative(x, g);
			return g;
		}
	};

	// Displacement that lifts pair `i`'s point by dy (in a `count`-pair mesh)
	Eigen::VectorXd lift(int count, int i, double dy)
	{
		Eigen::VectorXd x = Eigen::VectorXd::Zero(6 * count);
		x[2 * (3 * i + 2) + 1] = dy;
		return x;
	}
} // namespace

TEST_CASE("RB-20 force continuation carries endpoint coefficients", "[kappa_continuity][continuation]")
{
	const json on = {{"force_continuation", true}, {"kappa_spread", 2}};

	SECTION("a published coefficient is kept when the Hessian changes; the control re-estimates")
	{
		for (const bool enabled : {true, false})
		{
			auto mesh = make_pairs();
			Probe f(mesh, {{"force_continuation", enabled}});
			const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
			f.start(x);
			f.publish(x);
			const auto scales = f.scales();
			REQUIRE(scales.size() == 1);
			REQUIRE(scales[0] == Approx(100.).epsilon(4 * std::numeric_limits<double>::epsilon()));
			f.driving *= 4;
			f.publish(x);
			CHECK(f.scales()[0] == Approx(enabled ? 100. : 400.));
			CHECK(f.diagnostic_state()["continued_count"] == (enabled ? 1 : 0));
		}
	}

	SECTION("the barrier force at the endpoint is unchanged by the refresh, and follows only the trim")
	{
		auto mesh = make_pairs();
		Probe f(mesh, on);
		const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
		f.start(x);
		f.publish(x);
		const Eigen::VectorXd g0 = f.grad(x);
		REQUIRE(g0.norm() > 0);
		f.driving *= 4;
		f.publish(x);
		CHECK((f.grad(x) - g0).norm() == 0.0); // bit-identical: same coefficient, same trim
		f.bump_trim(2);
		f.publish(x);
		CHECK((f.grad(x) - 2 * g0).norm() <= 1e-12 * g0.norm());
	}

	SECTION("a contact born during a solve is priced by that solve's snapshot and continued from publication on")
	{
		auto mesh = make_pairs(1, .2, 2);
		Probe f(mesh, on);
		// Pair 1 lifted out of support (gap .2 + 1.5 > dhat 1) at the first endpoint.
		const Eigen::VectorXd apart = lift(2, 1, 1.5), together = Eigen::VectorXd::Zero(12);
		f.start(apart);
		f.publish(apart);
		const auto scales = f.scales();
		REQUIRE(scales.size() == 1);
		REQUIRE(scales[0] == Approx(100.).epsilon(4 * std::numeric_limits<double>::epsilon()));
		CHECK(f.diagnostic_state()["continued_count"] == 1);
		// The provider changes, but the snapshot that prices contacts born in
		// this "solve" is the one frozen at its start: pair 1 appears at 100.
		f.driving *= 4;
		f.solution_changed(together);
		auto k = f.scales();
		REQUIRE(k.size() == 2);
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(100.));
		// At publication both values acted, so both are captured; the 4x
		// Hessian never enters.
		f.publish(together);
		k = f.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(100.));
		CHECK(f.diagnostic_state()["continued_count"] == 2);
		CHECK(f.diagnostic_state()["fresh_count"] == 0);
	}

	SECTION("a mid-solve refresh keeps endpoint keys and re-estimates the rest")
	{
		auto mesh = make_pairs(1, .2, 2);
		Probe f(mesh, on);
		const Eigen::VectorXd apart = lift(2, 1, 1.5), together = Eigen::VectorXd::Zero(12);
		f.start(apart);
		f.publish(apart);
		f.driving *= 4;
		f.refresh_mid_solve(together);
		auto k = f.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(400.));
		// A second mid-solve refresh with a changed Hessian re-estimates the
		// non-endpoint key again (no endpoint has vetted it) but not the other.
		f.driving *= 2;
		f.refresh_mid_solve(together);
		k = f.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(800.));
	}

	SECTION("continued values bypass the fresh batch floor and cap")
	{
		auto mesh = make_pairs(1, .2, 2);
		Probe f(mesh, on); // kappa_spread 2
		const Eigen::VectorXd apart = lift(2, 1, 1.5), together = Eigen::VectorXd::Zero(12);
		f.start(apart);
		f.publish(apart);
		const auto scales = f.scales();
		REQUIRE(scales.size() == 1);
		REQUIRE(scales[0] == Approx(100.).epsilon(4 * std::numeric_limits<double>::epsilon()));
		// The new contact's fresh block is 1e4x stiffer: at a mid-solve
		// refresh (birth) the fresh batch median is 1e6, floor 5e5, cap 2e6
		// -- which must not raise the continued 100 (RB-18 F1 would have).
		f.driving.block(6, 6, 6, 6) *= 1e4;
		f.refresh_mid_solve(together);
		const auto k = f.scales();
		CHECK(k[0] == Approx(100.));
		CHECK(k[1] == Approx(1e6));
		CHECK(f.diagnostic_state()["batch_median"] == Approx(1e6));
	}

	SECTION("continuation_max_ratio bounds the pull toward the fresh estimate")
	{
		for (const double factor : {100., .01})
		{
			auto mesh = make_pairs();
			Probe f(mesh, {{"force_continuation", true}, {"continuation_max_ratio", 2}});
			const Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
			f.start(x);
			f.publish(x);
			f.driving *= factor;
			f.publish(x);
			CHECK(f.scales()[0] == Approx(factor > 1 ? 200. : 50.));
			// The pulled value is the new endpoint value.
			f.driving *= factor;
			f.publish(x);
			CHECK(f.scales()[0] == Approx(factor > 1 ? 400. : 25.));
		}
	}

	SECTION("an invalid max ratio is rejected")
	{
		auto mesh = make_pairs();
		CHECK_THROWS(Probe(mesh, {{"continuation_max_ratio", 0.5}}));
	}
}

TEST_CASE("RB-21 parent contributions are recorded by the builder", "[kappa_continuity][parent]")
{
	SECTION("a corner vertex sees both edges as parents of one vertex-vertex collision")
	{
		auto mesh = make_corner(true, .1);
		Probe f(mesh);
		const Eigen::VectorXd x = Eigen::VectorXd::Zero(8);
		f.start(x);
		const auto &c = f.collisions();
		REQUIRE(c.size() == 1);
		REQUIRE(c.is_vertex_vertex(0));
		CHECK(c[0].weight == 2.0);
		REQUIRE(c[0].parents.size() == 2);
		double sum = 0;
		for (const auto &p : c[0].parents)
		{
			CHECK(p.type == ipc::ParentContribution::Type::EdgeVertex);
			CHECK(p.id1 == 3); // the free vertex
			CHECK(p.weight == 1.0);
			sum += p.weight;
		}
		CHECK(sum == c[0].weight);
		const bool both_edges = (c[0].parents[0].id0 == 0 && c[0].parents[1].id0 == 1) || (c[0].parents[0].id0 == 1 && c[0].parents[1].id0 == 0);
		CHECK(both_edges);
	}

	SECTION("an interior point has its single edge parent; weights always sum to the collision weight")
	{
		auto mesh = make_corner(true, -.3);
		Probe f(mesh);
		f.start(Eigen::VectorXd::Zero(8));
		const auto &c = f.collisions();
		REQUIRE(c.size() == 2); // EV with e0, VV with the corner from e1
		for (size_t i = 0; i < c.size(); ++i)
		{
			double sum = 0;
			for (const auto &p : c[i].parents)
				sum += p.weight;
			CHECK(sum == c[i].weight);
			CHECK(c[i].parents.size() == 1);
		}
	}
}

TEST_CASE("RB-21 parent-keyed coefficients are continuous across closest-feature switches", "[kappa_continuity][parent]")
{
	// The snapshot is frozen with the point interior to e0 (x = -.3); the
	// point then moves across the corner at x = 0 WITHOUT a refresh, as it
	// does inside a Newton solve. Coefficients are evaluated at the snapshot
	// positions: a stencil created by the switch (the VV stencil) gets a
	// value from the corner geometry while the EV stencil's value reflects
	// the snapshot closest point -- the historical jump. A parent key keeps
	// the value of the candidate that built both.
	auto heterogeneous = [](Probe &f) {
		const double d[4] = {30., 300., 3000., 100.};
		for (int v = 0; v < 4; ++v)
			f.driving.block(2 * v, 2 * v, 2, 2) = d[v] * Eigen::MatrixXd::Identity(2, 2);
	};
	auto energy_at = [&](bool closed, bool parent, double x, bool hetero = true) {
		auto mesh = make_corner(closed, -.3);
		Probe f(mesh, {{"coefficient_identity", parent ? "parent" : "stencil"}});
		if (hetero)
			heterogeneous(f);
		const Eigen::VectorXd zero = Eigen::VectorXd::Zero(8);
		f.start(zero);
		f.refresh_mid_solve(zero);
		Eigen::VectorXd moved = zero;
		moved[6] = x + .3;
		f.solution_changed(moved);
		return f.value(moved);
	};
	constexpr double eps = 1e-9;

	SECTION("single-parent seam: exact in parent mode, a jump in stencil mode")
	{
		const double left = energy_at(false, true, -eps), right = energy_at(false, true, eps);
		CHECK(std::abs(right - left) <= 1e-6 * left);
		const double sl = energy_at(false, false, -eps), sr = energy_at(false, false, eps);
		CHECK(std::abs(sr - sl) > 1e-2 * sl); // the historical EV->VV jump
	}

	SECTION("two-parent seam: exact in parent mode (non-convergent formulation)")
	{
		const double left = energy_at(true, true, -eps), right = energy_at(true, true, eps);
		CHECK(std::abs(right - left) <= 1e-6 * left);
		const double sl = energy_at(true, false, -eps), sr = energy_at(true, false, eps);
		CHECK(std::abs(sr - sl) > 1e-2 * sl);
	}

	SECTION("homogeneous stiffness gives the same potential under both identities")
	{
		for (const double x : {-eps, eps, .1})
			CHECK(energy_at(true, true, x, false) == Approx(energy_at(true, false, x, false)));
	}

	SECTION("stiffness_scale is one, and parents sum to the weight, outside the semi-implicit mode")
	{
		auto mesh = make_corner(true, .1);
		BarrierContactForm f(mesh, 1, 1, false, false, false, false, false, false,
							 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000);
		const Eigen::VectorXd zero = Eigen::VectorXd::Zero(8);
		f.init(zero);
		const auto &c = f.collision_set();
		REQUIRE(c.size() == 1);
		CHECK(c[0].stiffness_scale == 1.0);
		double sum = 0;
		for (const auto &p : c[0].parents)
			sum += p.weight;
		CHECK(sum == c[0].weight);
	}

	SECTION("continuation survives the switch under parent identity only")
	{
		for (const bool parent : {true, false})
		{
			// Endpoint published with the point beyond the corner (VV built from
			// both edge parents), then the point moves into e0's interior region
			// and a mid-solve refresh happens with a stiffer Hessian.
			auto mesh = make_corner(true, .1);
			Probe f(mesh, {{"force_continuation", true}, {"coefficient_identity", parent ? "parent" : "stencil"}});
			heterogeneous(f);
			const Eigen::VectorXd zero = Eigen::VectorXd::Zero(8);
			f.start(zero);
			f.publish(zero);
			REQUIRE(f.scales().size() == 1);
			Eigen::VectorXd moved = zero;
			moved[6] = -.4; // x: .1 -> -.3, now interior to e0
			f.driving *= 4;
			f.refresh_mid_solve(moved);
			REQUIRE(f.collisions().size() == 2); // EV with e0, VV with the corner from e1
			if (parent)
			{
				// Both parents (e0, v3) and (e1, v3) were published: the EV
				// collision and the VV collision are continued.
				CHECK(f.diagnostic_state()["continued_count"] == 2);
				CHECK(f.diagnostic_state()["fresh_count"] == 0);
			}
			else
			{
				// The VV stencil was published and persists; the EV stencil
				// is new and estimated from the 4x Hessian.
				CHECK(f.diagnostic_state()["continued_count"] == 1);
				CHECK(f.diagnostic_state()["fresh_count"] == 1);
			}
		}
	}
}

TEST_CASE("RB-21 parent coefficient assignment checks weighted arithmetic", "[kappa_continuity][parent_identity]")
{
	// Both edge candidates reduce to the same VV collision. Each parent's
	// coefficient and its weighted total are representable in these controls.
	for (const double h : {1e307, 8e307})
	{
		CAPTURE(h);
		auto mesh = make_corner(true, .01, .99);
		Probe f(mesh);
		f.driving = h * Eigen::MatrixXd::Identity(8, 8);
		const Eigen::VectorXd x = Eigen::VectorXd::Zero(8);
		REQUIRE_NOTHROW(f.start(x));
		REQUIRE(f.collisions().size() == 1);
		REQUIRE(f.collisions()[0].parents.size() == 2);
		REQUIRE(f.scales().size() == 1);
		CHECK(std::isfinite(f.scales()[0]));
		CHECK(f.scales()[0] / h == Approx(1.));
		const double energy = f.value(x);
		CHECK(std::isfinite(energy));
		CHECK(energy / h == Approx(1.5680538752965535e-05).epsilon(1e-8));
	}

	// The correct mean is 1e308, but IPC first forms weight * scale = 2e308
	// before multiplying by the small potential/derivatives. Reject that
	// unsupported intermediate explicitly even though the exact energy at
	// this gap could be represented; never install a silently infinite scale.
	auto mesh = make_corner(true, .01, .99);
	Probe f(mesh);
	f.driving = 1e308 * Eigen::MatrixXd::Identity(8, 8);
	REQUIRE_THROWS_WITH(f.start(Eigen::VectorXd::Zero(8)),
						Catch::Matchers::ContainsSubstring("overflowing weighted collision coefficient"));
}

TEST_CASE("RBR-04 the improved max operator is refused under semi-implicit stiffness", "[kappa_continuity][parent][rbr04]")
{
	using Catch::Matchers::ContainsSubstring;
	constexpr double gap = .2, eps = 1e-9;
	// Point beyond the corner (x = .1): with improved max the corner v1 is one
	// vertex-vertex collision of weight 2 - 1 (two edge parents, one negative
	// vertex-vertex correction).
	auto mesh = make_corner(true, .1, gap);
	const Eigen::VectorXd zero = Eigen::VectorXd::Zero(8);
	auto construct = [&](bool improved_max, bool area_weighting, BarrierStiffnessMode mode) {
		return std::make_unique<BarrierContactForm>(
			mesh, 1, 1, area_weighting, improved_max, /*use_physical_barrier=*/false,
			/*use_adaptive_barrier_stiffness=*/mode != BarrierStiffnessMode::Fixed, false, false,
			ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, mode,
			mode == BarrierStiffnessMode::SemiImplicit ? json::object() : json(nullptr), Eigen::VectorXd::Ones(4));
	};
	// Free vertex at (x, y) as a displacement of the mesh built at x = .1.
	auto at = [&](double x, double y) {
		Eigen::VectorXd u = zero;
		u[6] = x - .1;
		u[7] = y - gap;
		return u;
	};

	SECTION("the direct constructor refuses the combination by name and states the alternatives")
	{
		for (const bool area_weighting : {false, true})
		{
			CAPTURE(area_weighting);
			REQUIRE_THROWS_WITH(construct(true, area_weighting, BarrierStiffnessMode::SemiImplicit),
								ContainsSubstring("does not support the improved max operator"));
		}
		REQUIRE_THROWS_WITH(construct(true, false, BarrierStiffnessMode::SemiImplicit),
							ContainsSubstring("use_convergent_formulation = false"));
		REQUIRE_THROWS_WITH(construct(true, false, BarrierStiffnessMode::SemiImplicit),
							ContainsSubstring("use_improved_max_operator = false and use_physical_barrier = false"));
		REQUIRE_THROWS_WITH(construct(true, false, BarrierStiffnessMode::SemiImplicit),
							ContainsSubstring("\"adaptive\" or a fixed value"));
	}

	SECTION("the non-convergent default, area weighting alone and the other stiffness modes still construct")
	{
		REQUIRE_NOTHROW(construct(false, false, BarrierStiffnessMode::SemiImplicit));
		REQUIRE_NOTHROW(construct(false, true, BarrierStiffnessMode::SemiImplicit));
		REQUIRE_NOTHROW(construct(true, false, BarrierStiffnessMode::Fixed));
		REQUIRE_NOTHROW(construct(true, true, BarrierStiffnessMode::Fixed));
		REQUIRE_NOTHROW(construct(true, false, BarrierStiffnessMode::Adaptive));
		REQUIRE_NOTHROW(construct(true, true, BarrierStiffnessMode::Adaptive));
	}

	SECTION("fixed and classic adaptive improved-max potentials are unchanged: signed parents, scale one, continuous at the corner")
	{
		for (const auto mode : {BarrierStiffnessMode::Fixed, BarrierStiffnessMode::Adaptive})
		{
			CAPTURE(int(mode));
			auto f = construct(true, false, mode);
			f->set_weight(1);
			f->set_barrier_stiffness(1);
			f->init(zero);
			const auto &c = f->collision_set();
			REQUIRE(c.size() == 1);
			REQUIRE(c.is_vertex_vertex(0));
			CHECK(c[0].weight == 1.0);
			CHECK(c[0].stiffness_scale == 1.0);
			REQUIRE(c[0].parents.size() == 3);
			int positive_edge_parents = 0, negative_vertex_parents = 0;
			double sum = 0;
			for (const auto &p : c[0].parents)
			{
				sum += p.weight;
				if (p.type == ipc::ParentContribution::Type::EdgeVertex && p.weight == 1.0)
					++positive_edge_parents;
				if (p.type == ipc::ParentContribution::Type::VertexVertex && p.weight == -1.0)
					++negative_vertex_parents;
			}
			CHECK(sum == c[0].weight);
			CHECK(positive_edge_parents == 2);
			CHECK(negative_vertex_parents == 1);
			// Seam A (x = 0 at y = gap) and seam B (y = 0 at x = gap): the
			// homogeneous improved-max potential is C0 (variation O(eps)).
			for (const auto &[left, right] : {std::make_pair(at(-eps, gap), at(eps, gap)), std::make_pair(at(gap, eps), at(gap, -eps))})
			{
				f->solution_changed(left);
				const double el = f->value(left);
				f->solution_changed(right);
				const double er = f->value(right);
				REQUIRE(el > 0);
				CHECK(std::abs(er - el) <= 1e-6 * el);
			}
		}
	}

	SECTION("the seam the refusal prevents: the positive-parent mean under improved max is not a sum of parent potentials")
	{
		// The coefficient rule of assign_collision_stiffness applied by hand to
		// the toolkit's improved-max collision set with the plan's example
		// parent coefficients (70 on e0, 40 on e1): stiffness_scale = the
		// contribution-weighted mean over the positive parents.
		auto assign = [](ipc::NormalCollisions &collisions, const std::function<double(const ipc::ParentContribution &)> &coefficient) {
			for (size_t i = 0; i < collisions.size(); ++i)
			{
				double numerator = 0, denominator = 0;
				for (const auto &p : collisions[i].parents)
					if (p.weight > 0)
					{
						numerator += p.weight * coefficient(p);
						denominator += p.weight;
					}
				REQUIRE(denominator > 0);
				collisions[i].stiffness_scale = numerator / denominator;
			}
		};
		auto energy = [&](bool improved_max, const Eigen::VectorXd &u, const std::function<double(const ipc::ParentContribution &)> &coefficient) {
			ipc::NormalCollisions collisions;
			collisions.set_collision_set_type(improved_max ? ipc::NormalCollisions::CollisionSetType::IMPROVED_MAX_APPROX : ipc::NormalCollisions::CollisionSetType::IPC);
			ipc::HashGrid broad_phase;
			const Eigen::MatrixXd vertices = mesh.displace_vertices(Eigen::Map<const Eigen::MatrixXd>(u.data(), 2, 4).transpose());
			collisions.build(mesh, vertices, /*dhat=*/1, 0, &broad_phase);
			assign(collisions, coefficient);
			const ipc::BarrierPotential potential(1, 1, false);
			return potential(collisions, mesh, vertices);
		};
		auto heterogeneous = [](const ipc::ParentContribution &p) {
			REQUIRE(p.type == ipc::ParentContribution::Type::EdgeVertex);
			return p.id0 == 0 ? 70. : 40.;
		};
		auto homogeneous = [](const ipc::ParentContribution &) { return 55.; };
		const double b = ipc::BarrierPotential(1, 1, false).barrier()(gap * gap, 1.0);
		REQUIRE(b > 0);

		// Seam A: e0's interior region (70 b) -> beyond the corner ((70+40)/2 b
		// on the net weight 2 - 1): the jump is (40 - 70) / 2 * b(gap).
		const double left_A = energy(true, at(-eps, gap), heterogeneous), right_A = energy(true, at(eps, gap), heterogeneous);
		CHECK(left_A == Approx(70 * b).epsilon(1e-6));
		CHECK(right_A == Approx(55 * b).epsilon(1e-6));
		CHECK(right_A - left_A == Approx(-15 * b).epsilon(1e-6));
		// Seam B: beyond the corner -> e1's interior region (40 b): -15 b again
		// (the corner's mean drops to e1's own coefficient).
		const double left_B = energy(true, at(gap, eps), heterogeneous), right_B = energy(true, at(gap, -eps), heterogeneous);
		CHECK(left_B == Approx(55 * b).epsilon(1e-6));
		CHECK(right_B == Approx(40 * b).epsilon(1e-6));
		CHECK(right_B - left_B == Approx(-15 * b).epsilon(1e-6));
		// Equal coefficients: the same set is the homogeneous improved-max
		// potential, continuous at both seams.
		CHECK(energy(true, at(eps, gap), homogeneous) == Approx(energy(true, at(-eps, gap), homogeneous)).epsilon(1e-6));
		CHECK(energy(true, at(gap, -eps), homogeneous) == Approx(energy(true, at(gap, eps), homogeneous)).epsilon(1e-6));
		// The non-convergent set (positive contributions only) is exactly the
		// sum of the parents' potentials: 70 b + 40 b on both sides.
		CHECK(energy(false, at(-eps, gap), heterogeneous) == Approx(110 * b).epsilon(1e-6));
		CHECK(energy(false, at(eps, gap), heterogeneous) == Approx(110 * b).epsilon(1e-6));
	}
}
