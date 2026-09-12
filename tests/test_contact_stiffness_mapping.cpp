#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <polyfem/State.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/MatrixUtils.hpp>

#include "VarFormTestAccess.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <utility>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	class MappedForm : public BarrierContactForm
	{
	public:
		MappedForm(const ipc::CollisionMesh &mesh, const Eigen::MatrixXd &h, const json &opts = json::object())
			: BarrierContactForm(mesh, 1., 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts)
		{
			set_weight(1.);
			set_barrier_stiffness(1.);
			set_system_hessian_provider([h](const Eigen::VectorXd &, StiffnessMatrix &out) { out = h.sparseView(); });
		}
		void start(const Eigen::VectorXd &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false);
		}
	};

	// Independent surface-coordinate oracle. The reference form has identity
	// indexing; only the test constructs the small dense P H P^T reference.
	void compare(const ipc::CollisionMesh &mesh, const Eigen::MatrixXd &b,
				 const Eigen::MatrixXd &h, const Eigen::VectorXd &x)
	{
		const int d = mesh.dim();
		Eigen::MatrixXd p = Eigen::MatrixXd::Zero(d * b.rows(), d * b.cols());
		for (int i = 0; i < b.rows(); ++i)
			for (int j = 0; j < b.cols(); ++j)
				p.block(d * i, d * j, d, d) = b(i, j) * Eigen::MatrixXd::Identity(d, d);
		Eigen::MatrixXd padded = Eigen::MatrixXd::Zero(x.size(), x.size());
		padded.topLeftCorner(h.rows(), h.cols()) = h;
		ipc::CollisionMesh surface(mesh.rest_positions(), mesh.edges(), mesh.faces());
		MappedForm actual(mesh, h), expected(surface, p * padded * p.transpose());
		Eigen::VectorXd y = p * x;
		actual.start(x);
		expected.start(y);
		REQUIRE(actual.collision_set().size() == 1);
		REQUIRE(expected.collision_set().size() == 1);
		const double k = expected.collision_set()[0].stiffness_scale;
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
		CHECK(std::abs(actual.value(x) - expected.value(y)) < 1e-10 * (1 + std::abs(expected.value(y))));
		Eigen::VectorXd ag, eg;
		actual.first_derivative(x, ag);
		expected.first_derivative(y, eg);
		CHECK((ag - p.transpose() * eg).norm() < 1e-10 * (1 + eg.norm()));
		StiffnessMatrix ah, eh;
		actual.second_derivative(x, ah);
		expected.second_derivative(y, eh);
		CHECK((Eigen::MatrixXd(ah) - p.transpose() * eh * p).norm() < 1e-10 * (1 + eh.norm()));
		actual.solution_changed(x);
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
		actual.refresh_semi_implicit_stiffness(x, false);
		CHECK(std::abs(actual.collision_set()[0].stiffness_scale - k) < 1e-10 * (1 + std::abs(k)));
	}

	// RB-03 interpolated-stencil curvature (decision 2026-09-11), as an
	// independent dense oracle: local condensation of the frozen parent block
	// onto the stencil's surface vertices, K = (B H_PP^-1 B^T)^-1, i.e. the
	// stiffness felt by a rigid stencil displacement along the contact
	// direction with every DOF outside the parent set P fixed and the parents
	// settling to minimum energy. Rows whose parents are all fixed (zero
	// diagonal block, or outside the Hessian) are dropped and contribute
	// zero, as in the exact-selector contract. When H_PP is not SPD or the
	// kept rows are dependent, the gap-normalized force direction
	// |w_K|^4 (w^T B H B^T w) / (w^T B B^T w)^2 stands in.
	struct Oracle
	{
		double kappa = 0;
		bool condensed = false;
		bool interpolated = false; // some kept row is not a unit selector
		int kept = 0;
	};

	Oracle oracle(const ipc::CollisionMesh &mesh, const ipc::CollisionStencil &stencil,
				  const Eigen::VectorXd &x, const Eigen::MatrixXd &h)
	{
		const int d = mesh.dim();
		const Eigen::MatrixXd map(mesh.displacement_map());
		const auto vids = stencil.vertex_ids(mesh.edges(), mesh.faces());
		const int n = stencil.num_vertices();

		// Contact direction exactly as the toolkit's semi_implicit_stiffness
		// forms it: w_i = c_i * (sum_j c_j x_j), normalized.
		const Eigen::MatrixXd v = mesh.displace_vertices(utils::unflatten(x, d));
		const ipc::VectorMax12d pos = stencil.dof(v, mesh.edges(), mesh.faces());
		const ipc::VectorMax4d c = stencil.compute_coefficients(pos);
		Eigen::VectorXd normal = Eigen::VectorXd::Zero(d);
		for (int i = 0; i < n; ++i)
			normal += c[i] * pos.segment(d * i, d);
		Eigen::VectorXd w(d * n);
		for (int i = 0; i < n; ++i)
			w.segment(d * i, d) = c[i] * normal;
		w.normalize();

		const auto fixed = [&](const int j) {
			return d * j + d > h.rows() || h.block(d * j, d * j, d, d).isZero(0.);
		};
		std::vector<int> parents;
		std::map<int, int> index;
		std::vector<std::vector<std::pair<int, double>>> rows(n);
		Oracle o;
		for (int a = 0; a < n; ++a)
		{
			for (int j = 0; j < map.cols(); ++j)
			{
				if (map(vids[a], j) == 0 || fixed(j))
					continue;
				if (!index.count(j))
				{
					index[j] = parents.size();
					parents.push_back(j);
				}
				rows[a].emplace_back(index[j], map(vids[a], j));
			}
			if (!rows[a].empty())
			{
				++o.kept;
				o.interpolated = o.interpolated || rows[a].size() != 1 || rows[a][0].second != 1.;
			}
		}
		if (o.kept == 0)
			return o;
		const int np = d * parents.size();
		Eigen::MatrixXd hpp(np, np), b = Eigen::MatrixXd::Zero(d * o.kept, np);
		Eigen::VectorXd wk(d * o.kept);
		for (int i = 0; i < int(parents.size()); ++i)
			for (int j = 0; j < int(parents.size()); ++j)
				hpp.block(d * i, d * j, d, d) = h.block(d * parents[i], d * parents[j], d, d);
		for (int a = 0, r = 0; a < n; ++a)
		{
			if (rows[a].empty())
				continue;
			for (const auto &[p, weight] : rows[a])
				b.block(d * r, d * p, d, d) += weight * Eigen::MatrixXd::Identity(d, d);
			wk.segment(d * r, d) = w.segment(d * a, d);
			++r;
		}
		Eigen::LLT<Eigen::MatrixXd> llt(hpp);
		Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(b);
		qr.setThreshold(1e-10);
		if (llt.info() == Eigen::Success && qr.rank() == b.rows())
		{
			const Eigen::MatrixXd k = (b * llt.solve(b.transpose())).inverse();
			o.kappa = wk.dot(k * wk);
			o.condensed = true;
			return o;
		}
		const double q1 = wk.dot(b * hpp * b.transpose() * wk);
		const double q2 = wk.dot(b * b.transpose() * wk);
		o.kappa = q2 > 0 ? std::pow(wk.squaredNorm(), 2) * q1 / (q2 * q2) : 0.;
		return o;
	}
} // namespace

TEST_CASE("Contact stiffness uses FEM node IDs for exact collision maps", "[contact_stiffness_mapping]")
{
	const int d = GENERATE(2, 3);
	const bool fem_only = GENERATE(false, true);
	const bool moving = GENERATE(false, true);
	CAPTURE(d, fem_only, moving);
	Eigen::MatrixXd rest = Eigen::MatrixXd::Zero(4, d);
	rest(0, 0) = -1;
	rest(2, 0) = 1;
	rest(3, 1) = .2;
	rest(1, 0) = 7; // excluded proxy vertex
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 2;
	// Two appended obstacle proxies select system nodes 3 and 4, even
	// though their proxy IDs 0 and 2 are inside a FEM-only Hessian.
	Eigen::MatrixXd t = Eigen::MatrixXd::Zero(4, 5);
	t(0, 3) = t(1, 2) = t(2, 4) = t(3, 1) = 1.;
	ipc::CollisionMesh mesh(std::vector<bool>{true, false, true, true},
							std::vector<bool>(4, false), rest, edges, Eigen::MatrixXi(), t.sparseView());
	Eigen::MatrixXd b(3, 5);
	b.row(0) = t.row(0);
	b.row(1) = t.row(2);
	b.row(2) = t.row(3);
	const int n = d * (fem_only ? 3 : 5);
	// Distinct diagonal and cross-node/cross-component terms catch more
	// than diagonal-block permutations. This matrix is well conditioned SPD.
	Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(n, .1, 1.);
	Eigen::MatrixXd h = v * v.transpose();
	h.diagonal() += Eigen::VectorXd::LinSpaced(n, 10., 100.);
	Eigen::VectorXd x = Eigen::VectorXd::Zero(5 * d);
	if (moving)
		x[3 * d + 1] = x[4 * d + 1] = .05;
	compare(mesh, b, h, x);
}

TEST_CASE("Identity and explicit rectangular selection share stiffness", "[contact_stiffness_mapping]")
{
	const bool explicit_map = GENERATE(false, true);
	Eigen::MatrixXd rest(3, 2);
	rest << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	Eigen::MatrixXd b = Eigen::MatrixXd::Identity(3, explicit_map ? 4 : 3);
	StiffnessMatrix t;
	if (explicit_map)
		t = b.sparseView();
	ipc::CollisionMesh mesh(rest, edges, Eigen::MatrixXi(), t);
	Eigen::MatrixXd h = Eigen::MatrixXd::Identity(2 * b.cols(), 2 * b.cols());
	h.diagonal() = Eigen::VectorXd::LinSpaced(h.rows(), 10., 100.);
	compare(mesh, b, h, Eigen::VectorXd::Zero(h.rows()));
}

TEST_CASE("Interpolated contact maps condense the parent block", "[contact_stiffness_mapping]")
{
	// 0 genuine interpolation and 1 scaled selector condense; 2 duplicate
	// selectors (dependent rows) and 4 an indefinite parent block fall back to
	// the gap-normalized direction; 3 an empty row is dropped (zero
	// contribution, as an obstacle row); 5 the direction fallback is itself
	// negative and goes through the RB-18 F7 |.| chain. Expected values are
	// hand-derived for edge (-1,0)-(1,0) against the point (0,.2): the unit
	// contact direction is [1, -.5, -.5]/sqrt(1.5) on the y components of
	// (point, e0, e1), with node stiffness 10/20/100/400.
	const int variant = GENERATE(0, 1, 2, 3, 4, 5);
	CAPTURE(variant);
	Eigen::MatrixXd rest(3, 2);
	rest << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3, 4);
	b(0, 3) = b(1, 1) = 1.;
	if (variant == 0 || variant >= 4)
		b(2, 0) = b(2, 2) = .5;
	else if (variant == 1)
		b(2, 2) = 2.;
	else if (variant == 2)
		b(2, 3) = 1.;
	ipc::CollisionMesh mesh(rest, edges, Eigen::MatrixXi(), b.sparseView());
	Eigen::VectorXd diagonal(8);
	diagonal << 10, 10, 20, 20, 100, 100, 400, 400;
	if (variant == 4)
		diagonal[4] = diagonal[5] = -100;
	else if (variant == 5)
		diagonal[4] = diagonal[5] = -1000;
	const Eigen::MatrixXd h(diagonal.asDiagonal());
	MappedForm form(mesh, h, {{"coefficient_identity", "stencil"}});
	form.start(Eigen::VectorXd::Zero(8));
	REQUIRE(form.collision_set().size() == 1);
	const double actual = form.collision_set()[0].stiffness_scale;
	const Oracle o = oracle(mesh, form.collision_set()[0], Eigen::VectorXd::Zero(8), h);
	// (ii): (100 + 5 + 1/(.25/10 + .25/100)) / 1.5; (400/4 + 20/4 + 100/4)/1.5;
	// (i'): (100 + 5)/1.5 / (1/3)^2; (100 + 5)/1.5 (row dropped);
	// (100 + 5 + 2.5 - 25)/1.5 / (2/3)^2; |(100 + 5 + 2.5 - 250)/1.5 / (2/3)^2|.
	const double expected[] = {(105. + 1. / .0275) / 1.5, 130. / 1.5, 630., 70., 123.75, 213.75};
	const bool condensed[] = {true, true, false, true, false, false};
	CHECK(o.condensed == condensed[variant]);
	CHECK(o.interpolated == (variant != 2 && variant != 3));
	CHECK(actual == Catch::Approx(expected[variant]).epsilon(1e-10));
	CHECK((variant == 5 ? -actual : actual) == Catch::Approx(o.kappa).epsilon(1e-10));
	const json state = form.diagnostic_state();
	CHECK(state["interpolated_condensed_count"].get<int>() == (condensed[variant] ? 1 : 0));
	CHECK(state["interpolated_direction_count"].get<int>() == (condensed[variant] ? 0 : 1));
	CHECK(state["curvature_abs_fallback_count"].get<int>() == (variant == 5 ? 1 : 0));
	// Deterministic between refreshes and across a solution change.
	form.solution_changed(Eigen::VectorXd::Zero(8));
	CHECK(form.collision_set()[0].stiffness_scale == Catch::Approx(actual).epsilon(1e-12));
}

TEST_CASE("Q1 hex face centroids get condensed stiffness through the production builder", "[contact_stiffness_mapping][hex]")
{
	// The default hex boundary extraction splits each quad face around a
	// centroid vertex interpolated from its four corners (weights .25); such
	// a vertex has no Hessian row of its own. Before RB-03's interpolation
	// stage its proxy ID was read as a node ID and fell outside the FE range,
	// so the barrier at a centroid contact saw no elastic block at all.
	const std::filesystem::path obstacle = std::filesystem::temp_directory_path() / "polyfem_rb03_hex_obstacle.obj";
	{
		std::ofstream out(obstacle);
		REQUIRE(out.good());
		out << "v -10 -10 -0.5\nv 30 -10 -0.5\nv 30 30 -0.5\nv -10 30 -0.5\nf 1 2 3\nf 1 3 4\n";
	}
	json in_args;
	in_args["/geometry/0/mesh"_json_pointer] = std::string(POLYFEM_DATA_DIR) + "/quad_test/hex.HYBRID";
	in_args["/geometry/1/mesh"_json_pointer] = obstacle.string();
	in_args["/geometry/1/is_obstacle"_json_pointer] = true;
	in_args["/materials/type"_json_pointer] = "NeoHookean";
	in_args["/materials/E"_json_pointer] = 1e5;
	in_args["/materials/nu"_json_pointer] = 0.3;
	in_args["/materials/rho"_json_pointer] = 1e3;
	in_args["/contact/enabled"_json_pointer] = true;
	in_args["/time/time_steps"_json_pointer] = 1;
	in_args["/time/tend"_json_pointer] = 1;
	in_args["/output/log/level"_json_pointer] = "warning";

	State state;
	state.init(in_args, true);
	state.set_max_threads(1);
	state.load_mesh();
	test::VarFormTestAccess::prepare(*state.variational_formulation);
	const test::VarFormDebugData debug = test::VarFormTestAccess::debug_data(*state.variational_formulation);
	const io::OutputSpace output_space = state.variational_formulation->output_space();
	REQUIRE(output_space.collision_mesh != nullptr);
	const ipc::CollisionMesh &mesh = *output_space.collision_mesh;
	REQUIRE(mesh.dim() == 3);
	REQUIRE(debug.n_obstacle_vertices == 4);
	const int n_fe = debug.n_bases - debug.n_obstacle_vertices;
	REQUIRE(n_fe == 20);

	// The production map carries four-corner centroid rows.
	const Eigen::MatrixXd map(mesh.displacement_map());
	REQUIRE(map.cols() == debug.n_bases);
	int centroid_rows = 0;
	for (int r = 0; r < map.rows(); ++r)
	{
		int entries = 0;
		bool quarter = true;
		for (int j = 0; j < map.cols(); ++j)
			if (map(r, j) != 0)
			{
				++entries;
				quarter = quarter && map(r, j) == .25;
			}
		centroid_rows += entries == 4 && quarter;
	}
	CHECK(centroid_rows == 18); // 2 caps + 4 x 4 side faces of the column

	// Frozen synthetic Hessian: SPD with cross-node coupling on the FE
	// nodes, zero (fixed) blocks on the obstacle nodes.
	const int n = 3 * debug.n_bases;
	Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
	{
		const int m = 3 * n_fe;
		const Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(m, .1, 1.);
		h.topLeftCorner(m, m) = v * v.transpose();
		h.topLeftCorner(m, m).diagonal() += Eigen::VectorXd::LinSpaced(m, 10., 100.);
	}
	MappedForm form(mesh, h, {{"coefficient_identity", "stencil"}});
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
	form.start(x);
	REQUIRE(form.collision_set().size() > 0);

	int interpolated = 0, condensed = 0;
	for (size_t i = 0; i < form.collision_set().size(); ++i)
	{
		const Oracle o = oracle(mesh, form.collision_set()[i], x, h);
		CAPTURE(i, o.interpolated, o.condensed, o.kept);
		REQUIRE(o.kept > 0);
		CHECK(o.condensed);
		CHECK(form.collision_set()[i].stiffness_scale == Catch::Approx(o.kappa).epsilon(1e-9));
		interpolated += o.interpolated;
		condensed += o.condensed && o.interpolated;
	}
	CHECK(interpolated >= 1); // the base centroid against the obstacle
	const json diagnostic = form.diagnostic_state();
	CHECK(diagnostic["interpolated_condensed_count"].get<int>() == condensed);
	CHECK(diagnostic["interpolated_direction_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_fallback_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_abs_fallback_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_global_fallback_count"].get<int>() == 0);
	std::filesystem::remove(obstacle);
}

TEST_CASE("Q2 hex contacts through the production builder are exact selectors under both RB-22 tessellations", "[contact_stiffness_mapping][hex][rb22]")
{
	// RB-22: a Q2 hexahedral column over an obstacle plane, built through the
	// production collision-mesh builder with the two supported high-order
	// tessellations. Every proxy vertex of the DOF-resolution proxy is a node
	// (exact selector rows); the order-2 lattice reproduces the Q2 nodes, so
	// max_order gives the same 74-vertex surface. The RB-03 oracle must agree
	// with the production stiffness on every contact.
	const std::string type = GENERATE("dof", "max_order");
	CAPTURE(type);
	const std::filesystem::path obstacle = std::filesystem::temp_directory_path() / ("polyfem_rb22_hex_obstacle_" + type + ".obj");
	{
		std::ofstream out(obstacle);
		REQUIRE(out.good());
		out << "v -10 -10 -0.5\nv 30 -10 -0.5\nv 30 30 -0.5\nv -10 30 -0.5\nf 1 2 3\nf 1 3 4\n";
	}
	json in_args;
	in_args["/geometry/0/mesh"_json_pointer] = std::string(POLYFEM_DATA_DIR) + "/quad_test/hex.HYBRID";
	in_args["/geometry/1/mesh"_json_pointer] = obstacle.string();
	in_args["/geometry/1/is_obstacle"_json_pointer] = true;
	in_args["/materials/type"_json_pointer] = "NeoHookean";
	in_args["/materials/E"_json_pointer] = 1e5;
	in_args["/materials/nu"_json_pointer] = 0.3;
	in_args["/materials/rho"_json_pointer] = 1e3;
	in_args["/space/discr_order"_json_pointer] = 2;
	in_args["/contact/enabled"_json_pointer] = true;
	in_args["/contact/collision_mesh/enabled"_json_pointer] = true;
	in_args["/contact/collision_mesh/tessellation_type"_json_pointer] = type;
	in_args["/time/time_steps"_json_pointer] = 1;
	in_args["/time/tend"_json_pointer] = 1;
	in_args["/output/log/level"_json_pointer] = "warning";

	State state;
	state.init(in_args, true);
	state.set_max_threads(1);
	state.load_mesh();
	test::VarFormTestAccess::prepare(*state.variational_formulation);
	const test::VarFormDebugData debug = test::VarFormTestAccess::debug_data(*state.variational_formulation);
	const io::OutputSpace output_space = state.variational_formulation->output_space();
	REQUIRE(output_space.collision_mesh != nullptr);
	const ipc::CollisionMesh &mesh = *output_space.collision_mesh;
	REQUIRE(mesh.dim() == 3);
	REQUIRE(debug.n_obstacle_vertices == 4);
	const int n_fe = debug.n_bases - debug.n_obstacle_vertices;
	REQUIRE(n_fe == 81); // 3 x 3 x 9 Q2 nodes
	CHECK(mesh.num_vertices() == 74 + 4);
	CHECK(mesh.num_faces() == 18 * 8 + 2);

	const Eigen::MatrixXd map(mesh.displacement_map()); // S * T: rows are surface vertices
	REQUIRE(map.rows() == mesh.num_vertices());
	REQUIRE(map.cols() == debug.n_bases);
	int selector_rows = 0;
	for (int r = 0; r < map.rows(); ++r)
	{
		int entries = 0;
		bool unit = true;
		for (int j = 0; j < map.cols(); ++j)
			if (map(r, j) != 0)
			{
				++entries;
				unit = unit && map(r, j) == 1.;
			}
		selector_rows += entries == 1 && unit;
	}
	CHECK(selector_rows == 74 + 4);

	// Frozen synthetic Hessian: SPD with cross-node coupling on the FE
	// nodes, zero (fixed) blocks on the obstacle nodes.
	const int n = 3 * debug.n_bases;
	Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
	{
		const int m = 3 * n_fe;
		const Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(m, .1, 1.);
		h.topLeftCorner(m, m) = v * v.transpose();
		h.topLeftCorner(m, m).diagonal() += Eigen::VectorXd::LinSpaced(m, 10., 100.);
	}
	MappedForm form(mesh, h, {{"coefficient_identity", "stencil"}});
	const Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
	form.start(x);
	REQUIRE(form.collision_set().size() > 0);

	for (size_t i = 0; i < form.collision_set().size(); ++i)
	{
		const Oracle o = oracle(mesh, form.collision_set()[i], x, h);
		CAPTURE(i, o.interpolated, o.condensed, o.kept);
		REQUIRE(o.kept > 0);
		CHECK_FALSE(o.interpolated);
		CHECK(o.condensed);
		CHECK(form.collision_set()[i].stiffness_scale == Catch::Approx(o.kappa).epsilon(1e-9));
	}
	const json diagnostic = form.diagnostic_state();
	CHECK(diagnostic["interpolated_condensed_count"].get<int>() == 0);
	CHECK(diagnostic["interpolated_direction_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_fallback_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_abs_fallback_count"].get<int>() == 0);
	CHECK(diagnostic["curvature_global_fallback_count"].get<int>() == 0);
	std::filesystem::remove(obstacle);
}
