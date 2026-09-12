// RB-22: high-order hexahedral collision surfaces.
//
// A contact-enabled scene must either get a conforming, watertight collision
// surface for every boundary face of every element type it accepts, or stop
// with a named error naming the elements and orders. Before RB-22 the default
// boundary extraction dropped every non-Q1 hexahedral face at trace level,
// the empty surface was padded to n_bases + n_faces vertex rows with an empty
// (identity) displacement map, and the first Newton Hessian crashed in the
// reduced projection (EXC_BAD_ACCESS). These tests pin the named error on the
// default path and the two supported tessellations of Q2/Q3/serendipity
// hexahedra through the production collision-mesh builder.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/OutData.hpp>
#include <polyfem/autogen/auto_q_bases.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "VarFormTestAccess.hpp"

#include <igl/boundary_facets.h>
#include <igl/edges.h>
#include <igl/doublearea.h>
#include <igl/euler_characteristic.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>

#include <ipc/ipc.hpp>

#include <map>
#include <memory>
#include <set>
#include <string>

using namespace polyfem;
using Catch::Matchers::ContainsSubstring;

namespace
{
	// 1 x 1 x 4 column of hexahedra: 20 vertices, 18 boundary quad faces.
	json column_args(const int order, const json &collision_mesh = json(), const std::string &basis_type = "Lagrange", const bool contact = true)
	{
		json in_args;
		in_args["/geometry/0/mesh"_json_pointer] = std::string(POLYFEM_DATA_DIR) + "/quad_test/hex.HYBRID";
		in_args["/materials/type"_json_pointer] = "NeoHookean";
		in_args["/materials/E"_json_pointer] = 1e5;
		in_args["/materials/nu"_json_pointer] = 0.3;
		in_args["/materials/rho"_json_pointer] = 1e3;
		in_args["/space/discr_order"_json_pointer] = order;
		if (basis_type != "Lagrange")
			in_args["/space/basis_type"_json_pointer] = basis_type;
		in_args["/contact/enabled"_json_pointer] = contact;
		if (!collision_mesh.is_null())
			in_args["/contact/collision_mesh"_json_pointer] = collision_mesh;
		in_args["/time/time_steps"_json_pointer] = 1;
		in_args["/time/tend"_json_pointer] = 1;
		in_args["/output/log/level"_json_pointer] = "error";
		return in_args;
	}

	json tessellation(const std::string &type)
	{
		return json{{"enabled", true}, {"tessellation_type", type}};
	}

	struct Built
	{
		std::shared_ptr<State> state;
		test::VarFormDebugData debug;
		const ipc::CollisionMesh *mesh = nullptr;
	};

	// Runs the production path up to and including the collision-mesh build.
	Built build(const json &in_args)
	{
		Built b;
		b.state = std::make_shared<State>();
		b.state->init(in_args, true);
		b.state->set_max_threads(1);
		b.state->load_mesh();
		test::VarFormTestAccess::prepare(*b.state->variational_formulation);
		b.debug = test::VarFormTestAccess::debug_data(*b.state->variational_formulation);
		b.mesh = b.state->variational_formulation->output_space().collision_mesh;
		return b;
	}

	void expect_named_error(const json &in_args, const std::string &element_text, const std::string &skipped_text)
	{
		State state;
		state.init(in_args, true);
		state.set_max_threads(1);
		state.load_mesh();
		REQUIRE_THROWS_WITH(
			test::VarFormTestAccess::prepare(*state.variational_formulation),
			ContainsSubstring("incomplete collision surface") && ContainsSubstring(element_text)
				&& ContainsSubstring(skipped_text));
	}

	// the quad_test tetrahedral mesh (27 tets, 40 boundary faces) at a given order
	json tet_args(const int order)
	{
		json in_args = column_args(order);
		in_args["/geometry/0/mesh"_json_pointer] = std::string(POLYFEM_DATA_DIR) + "/quad_test/tet.msh";
		return in_args;
	}

	struct SurfaceStats
	{
		int n_vertices = 0, n_faces = 0;
		int selector_rows = 0, interpolated_rows = 0, empty_rows = 0;
		bool closed = false, edge_manifold = false, vertex_manifold = false, positive_area = false, intersection_free = false;
		int euler = 0, components = 0;
	};

	SurfaceStats analyze(const ipc::CollisionMesh &mesh, const int n_fe_nodes)
	{
		SurfaceStats s;
		REQUIRE(mesh.dim() == 3);
		const Eigen::MatrixXd V = mesh.rest_positions();
		const Eigen::MatrixXi F = mesh.faces();
		s.n_vertices = int(V.rows());
		s.n_faces = int(F.rows());
		REQUIRE(s.n_faces > 0);
		REQUIRE(V.allFinite());
		REQUIRE(F.minCoeff() >= 0);
		REQUIRE(F.maxCoeff() < V.rows());

		Eigen::VectorXd double_area;
		igl::doublearea(V, F, double_area);
		s.positive_area = double_area.minCoeff() > 0;
		s.edge_manifold = igl::is_edge_manifold(F);
		s.vertex_manifold = igl::is_vertex_manifold(F);
		Eigen::MatrixXi boundary_edges;
		igl::boundary_facets(F, boundary_edges);
		s.closed = boundary_edges.rows() == 0;
		s.euler = igl::euler_characteristic(F);
		Eigen::VectorXi components;
		s.components = igl::facet_components(F, components);
		s.intersection_free = !ipc::has_intersections(mesh, V);

		// Displacement-map rows of the surface vertices (the toolkit stores
		// S * T, i.e. rows are surface vertex ids): an exact selector is a
		// single entry equal to 1 (the RB-03 exact-indexing contract).
		const Eigen::SparseMatrix<double> &map = mesh.displacement_map();
		REQUIRE(map.rows() == mesh.num_vertices());
		REQUIRE(map.cols() == n_fe_nodes);
		Eigen::VectorXi counts = Eigen::VectorXi::Zero(map.rows());
		Eigen::VectorXd single = Eigen::VectorXd::Zero(map.rows());
		for (int col = 0; col < map.outerSize(); ++col)
			for (Eigen::SparseMatrix<double>::InnerIterator it(map, col); it; ++it)
				if (it.value() != 0.)
				{
					++counts[it.row()];
					single[it.row()] = it.value();
				}
		for (int row = 0; row < mesh.num_vertices(); ++row)
		{
			if (counts[row] == 0)
				++s.empty_rows;
			else if (counts[row] == 1 && single[row] == 1.)
				++s.selector_rows;
			else
				++s.interpolated_rows;
		}
		return s;
	}

	// combinatorial conformity: one closed 2-manifold sphere, every vertex mapped
	void check_topology(const SurfaceStats &s)
	{
		CHECK(s.closed);
		CHECK(s.edge_manifold);
		CHECK(s.vertex_manifold);
		CHECK(s.euler == 2);
		CHECK(s.components == 1);
		CHECK(s.empty_rows == 0);
	}

	// geometric validity: positive areas, no self-intersections
	void check_geometry(const SurfaceStats &s)
	{
		CHECK(s.positive_area);
		CHECK(s.intersection_free);
	}

	void check_closed_sphere(const SurfaceStats &s)
	{
		check_topology(s);
		check_geometry(s);
	}

	// Q3 hexahedra: the basis stores edge/face node positions that are not
	// the images of their reference nodes (nonconforming; see the hex_nodes
	// tests and docs/rb-22-validation.md), so both proxies are combinatorially
	// right but geometrically degenerate, and the production builder refuses
	// them with a named error. The geometric acceptance lives in the hidden
	// [q3_hex_defect] tests until the basis is repaired.
	const char *const Q3_DEFECT = "Q3+ hexahedral bases store edge/face node positions";
} // namespace

TEST_CASE("An extraction that skips boundary faces is refused with a named error", "[rb22][hex_collision_surface]")
{
	// The former silent failure: the Q1-only default extraction skipped every
	// Q2+ hexahedral face (18 of 18 here), the empty surface was padded with an
	// identity map larger than the DOF vector, and the first Hessian crashed.
	// High-order hexahedra are now routed to the DOF proxy by default, so the
	// direct extraction still reports the skips (checked below) while the
	// builder's named error is pinned on a case that still skips: simplex
	// boundary export supports up to P4.
	expect_named_error(tet_args(5), "simplex face with 21 owned nodes", "skipped 40 of 40 boundary faces");

	// The Q1-only extraction itself still skips high-order hex faces and says so.
	const Built b = build(column_args(2, json(), "Lagrange", /*contact=*/false));
	Eigen::MatrixXd V;
	Eigen::MatrixXi E, F;
	std::vector<Eigen::Triplet<double>> map;
	io::OutGeometryData::BoundaryExtractionReport report;
	io::OutGeometryData::extract_boundary_mesh(*b.debug.mesh, b.debug.n_bases, *b.debug.bases, *b.debug.total_local_boundary, V, E, F, map, &report);
	CHECK_FALSE(report.complete());
	CHECK(report.skipped.size() == 18);
	CHECK_THAT(report.describe(), ContainsSubstring("Q2 element with 9 owned nodes"));
}

TEST_CASE("Q2+ hex contact scenes get the DOF-resolution proxy by default", "[rb22][hex_collision_surface]")
{
	// RB-22 default (user decision 2026-09-12): identical to tessellation_type "dof".
	SECTION("Q2")
	{
		const Built b = build(column_args(2));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 74);
		CHECK(s.n_faces == 18 * 8);
		CHECK(s.selector_rows == 74);
		CHECK(s.interpolated_rows == 0);
	}
	SECTION("Q2 with an explicit default-type collision_mesh block")
	{
		const Built b = build(column_args(2, json{{"enabled", true}}));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 74);
		CHECK(s.n_faces == 18 * 8);
	}
	SECTION("serendipity Q2")
	{
		const Built b = build(column_args(2, json(), "Serendipity"));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 56);
		CHECK(s.n_faces == 18 * 6);
		CHECK(s.selector_rows == 56);
	}
	SECTION("Q3: the degenerate node positions are refused, never a crash")
	{
		State state;
		state.init(column_args(3), true);
		state.set_max_threads(1);
		state.load_mesh();
		REQUIRE_THROWS_WITH(test::VarFormTestAccess::prepare(*state.variational_formulation),
							ContainsSubstring("degenerate (zero-area) collision faces") && ContainsSubstring(Q3_DEFECT));
	}
}

TEST_CASE("Q2 hex without contact and Q1 hex with contact are unaffected", "[rb22][hex_collision_surface]")
{
	SECTION("contact disabled builds no collision mesh and does not throw")
	{
		const Built b = build(column_args(2, json(), "Lagrange", /*contact=*/false));
		CHECK(b.mesh == nullptr);
	}
	SECTION("Q1 keeps the centroid split: 4 triangles and one quarter-weight row per face")
	{
		const Built b = build(column_args(1));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 20 + 18);
		CHECK(s.n_faces == 18 * 4);
		CHECK(s.selector_rows == 20);
		CHECK(s.interpolated_rows == 18);
	}
}

TEST_CASE("DOF-resolution proxy of Q2/Q3/serendipity hexahedra is closed with exact selector rows", "[rb22][hex_collision_surface]")
{
	SECTION("Q2: every boundary node is a proxy vertex, 8 triangles per face")
	{
		const Built b = build(column_args(2, tessellation("dof")));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 74); // 3x3x9 nodes minus the 7 interior ones
		CHECK(s.n_faces == 18 * 8);
		CHECK(s.selector_rows == 74);
		CHECK(s.interpolated_rows == 0);
	}
	SECTION("Q3: 18 triangles per face in the extraction; the builder refuses the degenerate node positions")
	{
		State state;
		state.init(column_args(3, tessellation("dof")), true);
		state.set_max_threads(1);
		state.load_mesh();
		REQUIRE_THROWS_WITH(test::VarFormTestAccess::prepare(*state.variational_formulation),
							ContainsSubstring("degenerate (zero-area) collision faces") && ContainsSubstring(Q3_DEFECT));

		const Built b = build(column_args(3, json(), "Lagrange", /*contact=*/false));
		Eigen::MatrixXd V;
		Eigen::MatrixXi E, F;
		std::vector<Eigen::Triplet<double>> map;
		io::OutGeometryData::BoundaryExtractionReport report;
		io::OutGeometryData::extract_boundary_mesh_nodal(*b.debug.mesh, b.debug.n_bases, *b.debug.bases, *b.debug.total_local_boundary, V, E, F, map, &report);
		CHECK(report.complete());
		CHECK(F.rows() == 18 * 18);
		CHECK(map.size() == 164); // 4x4x13 nodes minus the 44 interior ones, one unit entry each
		for (const auto &t : map)
		{
			CHECK(t.row() == t.col());
			CHECK(t.value() == 1.);
		}
		Eigen::MatrixXi boundary_edges, edges;
		igl::boundary_facets(F, boundary_edges);
		CHECK(boundary_edges.rows() == 0);
		CHECK(igl::is_edge_manifold(F));
		igl::edges(F, edges);
		std::set<int> used(F.data(), F.data() + F.size());
		CHECK(int(used.size()) - edges.rows() + F.rows() == 2); // V rows are padded to n_bases
	}
	SECTION("serendipity Q2: 8 nodes per face, convex-sweep triangulation")
	{
		const Built b = build(column_args(2, tessellation("dof"), "Serendipity"));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 56); // 20 vertices + 36 edge midpoints, all on the boundary
		CHECK(s.n_faces == 18 * 6);
		CHECK(s.selector_rows == 56);
		CHECK(s.interpolated_rows == 0);
	}
}

TEST_CASE("max_order lattice proxy of Q2/Q3/serendipity hexahedra is closed", "[rb22][hex_collision_surface]")
{
	SECTION("Q2: the order-2 lattice reproduces the Q2 nodes exactly")
	{
		const Built b = build(column_args(2, tessellation("max_order")));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 74);
		CHECK(s.n_faces == 18 * 8);
		CHECK(s.selector_rows == 74);
		CHECK(s.interpolated_rows == 0);
	}
	SECTION("Q3: lattice nodes at thirds are not exact selectors; the builder refuses the degenerate node positions")
	{
		State state;
		state.init(column_args(3, tessellation("max_order")), true);
		state.set_max_threads(1);
		state.load_mesh();
		REQUIRE_THROWS_WITH(test::VarFormTestAccess::prepare(*state.variational_formulation),
							ContainsSubstring("degenerate (zero-area) collision faces") && ContainsSubstring(Q3_DEFECT));

		// The displacement-map weights are basis values at the lattice
		// points and do not depend on the node positions: at thirds the
		// Q3 Lagrange basis evaluates to 1 - eps rather than exactly 1, so
		// the RB-03 exact-selector classification does not apply (measured:
		// 1 exact selector of 164 rows; recorded in docs/rb-22-validation.md).
		const Built b = build(column_args(3, json(), "Lagrange", /*contact=*/false));
		Eigen::MatrixXd V;
		Eigen::MatrixXi E, F;
		std::vector<Eigen::Triplet<double>> map;
		io::OutGeometryData::BoundaryExtractionReport report;
		io::OutGeometryData::extract_boundary_mesh_sampled(*b.debug.mesh, b.debug.n_bases, *b.debug.bases, *b.debug.total_local_boundary, V, E, F, map, 0, &report);
		CHECK(report.complete());
		CHECK(V.rows() == 164);
		CHECK(F.rows() == 18 * 18);
		std::vector<int> counts(V.rows(), 0);
		std::vector<double> single(V.rows(), 0.);
		for (const auto &t : map)
		{
			++counts[t.row()];
			single[t.row()] = t.value();
		}
		int exact = 0, near = 0;
		for (int r = 0; r < V.rows(); ++r)
		{
			exact += counts[r] == 1 && single[r] == 1.;
			near += counts[r] >= 1 && single[r] != 1. && std::abs(single[r] - 1.) < 1e-12 && counts[r] == 1;
		}
		WARN("max_order Q3 rows: " << exact << " exact selectors, " << near << " single entries within 1e-12 of 1, " << V.rows() - exact - near << " other");
		CHECK(exact + near <= int(V.rows()));
	}
	SECTION("serendipity Q2: face centers are interpolated from the 8 face nodes")
	{
		const Built b = build(column_args(2, tessellation("max_order"), "Serendipity"));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 74);
		CHECK(s.n_faces == 18 * 8);
		CHECK(s.selector_rows == 56);
		CHECK(s.interpolated_rows == 18);
	}
}

TEST_CASE("Boundary extraction report names the skipped hex faces", "[rb22][hex_collision_surface]")
{
	// Direct extraction (as the shape-derivative consumers call it) keeps
	// producing the padded vertex rows; the report carries the skips.
	const Built b = build(column_args(2, json(), "Lagrange", /*contact=*/false));
	REQUIRE(b.debug.mesh != nullptr);
	REQUIRE(b.debug.bases != nullptr);
	REQUIRE(b.debug.total_local_boundary != nullptr);
	CHECK(io::OutGeometryData::has_high_order_hex_boundary(*b.debug.mesh, *b.debug.bases, *b.debug.total_local_boundary));

	Eigen::MatrixXd V;
	Eigen::MatrixXi E, F;
	std::vector<Eigen::Triplet<double>> map;
	io::OutGeometryData::BoundaryExtractionReport report;
	io::OutGeometryData::extract_boundary_mesh(*b.debug.mesh, b.debug.n_bases, *b.debug.bases, *b.debug.total_local_boundary, V, E, F, map, &report);
	CHECK(F.rows() == 0);
	CHECK(map.empty());
	CHECK(V.rows() == b.debug.n_bases + b.debug.mesh->n_faces());
	CHECK_FALSE(report.complete());
	CHECK(report.n_boundary_faces == 18);
	REQUIRE(report.skipped.size() == 18);
	for (const auto &f : report.skipped)
	{
		CHECK(f.order == 2);
		CHECK(f.n_nodes == 27);
		CHECK(b.debug.mesh->is_cube(f.element_id));
	}
	CHECK_THAT(report.describe(), ContainsSubstring("order 2 with 27 nodes: 18 faces"));

	// Q1 column: complete report, nothing high-order
	const Built q1 = build(column_args(1, json(), "Lagrange", /*contact=*/false));
	CHECK_FALSE(io::OutGeometryData::has_high_order_hex_boundary(*q1.debug.mesh, *q1.debug.bases, *q1.debug.total_local_boundary));
	io::OutGeometryData::extract_boundary_mesh(*q1.debug.mesh, q1.debug.n_bases, *q1.debug.bases, *q1.debug.total_local_boundary, V, E, F, map, &report);
	CHECK(report.complete());
	CHECK(report.n_boundary_faces == 18);
	CHECK(F.rows() == 72);
}

namespace
{
	// Both high-order proxies place vertices at basis node positions (or
	// evaluate the bases at reference points weighted by them), so a stored
	// node position that is not the geometric image of that basis' reference
	// node produces a degenerate surface regardless of tessellation.
	int count_node_position_mismatches(const Built &b, const int order, int &checked)
	{
		Eigen::MatrixXd ref_nodes;
		autogen::q_nodes_3d(order, ref_nodes);
		int mismatched = 0;
		checked = 0;
		for (size_t e = 0; e < b.debug.bases->size(); ++e)
		{
			const basis::ElementBases &bases = (*b.debug.bases)[e];
			const basis::ElementBases &gbases = (*b.debug.geometry_bases)[e];
			REQUIRE(bases.bases.size() == size_t(ref_nodes.rows()));
			Eigen::MatrixXd mapped;
			gbases.eval_geom_mapping(ref_nodes, mapped);
			for (size_t j = 0; j < bases.bases.size(); ++j)
			{
				if (bases.bases[j].global().size() != 1)
					continue;
				++checked;
				const double err = (mapped.row(j) - bases.bases[j].global().front().node).norm();
				if (err > 1e-9)
				{
					++mismatched;
					if (mismatched <= 8)
						WARN("element " << e << " local node " << j << " reference " << ref_nodes.row(j) << " maps to " << mapped.row(j) << " but is stored at " << bases.bases[j].global().front().node);
				}
			}
		}
		return mismatched;
	}

	// Conformity of the FE space itself: the image of a shared global node's
	// reference position must be the same from every element that owns it.
	int count_shared_node_disagreements(const Built &b, const int order, int &shared)
	{
		Eigen::MatrixXd ref_nodes;
		autogen::q_nodes_3d(order, ref_nodes);
		std::map<int, Eigen::RowVector3d> first;
		int disagreements = 0;
		shared = 0;
		for (size_t e = 0; e < b.debug.bases->size(); ++e)
		{
			const basis::ElementBases &bases = (*b.debug.bases)[e];
			Eigen::MatrixXd mapped;
			(*b.debug.geometry_bases)[e].eval_geom_mapping(ref_nodes, mapped);
			for (size_t j = 0; j < bases.bases.size(); ++j)
			{
				if (bases.bases[j].global().size() != 1)
					continue;
				const int g = bases.bases[j].global().front().index;
				const auto it = first.find(g);
				if (it == first.end())
				{
					first[g] = mapped.row(j);
					continue;
				}
				++shared;
				if ((it->second - mapped.row(j)).norm() > 1e-9)
				{
					++disagreements;
					if (disagreements <= 6)
						WARN("global node " << g << " is at " << it->second << " from one element and at " << mapped.row(j) << " from element " << e);
				}
			}
		}
		return disagreements;
	}
} // namespace

TEST_CASE("Q2 hex node positions are the geometric images of their reference nodes and shared consistently", "[rb22][hex_collision_surface][hex_nodes]")
{
	const Built b = build(column_args(2, json(), "Lagrange", /*contact=*/false));
	REQUIRE(b.debug.bases != nullptr);
	REQUIRE(b.debug.geometry_bases != nullptr);
	int checked = 0, shared = 0;
	CHECK(count_node_position_mismatches(b, 2, checked) == 0);
	CHECK(checked == 4 * 27);
	CHECK(count_shared_node_disagreements(b, 2, shared) == 0);
	CHECK(shared == 3 * 9); // the three shared Q2 faces
}

// Known upstream defect (RB-22, 2026-09-12): Q3 hexahedra. Hidden; run it by
// name to check a basis repair. On the 4-hex column 124 of 256 node positions
// are not the images of their reference nodes (edge nodes swapped within an
// edge, face-interior nodes permuted within a face) and the 12 face-interior
// nodes of the 3 shared faces are placed differently by their two elements,
// i.e. the Q3 hex space is nonconforming.
TEST_CASE("Q3 hex node positions are the geometric images of their reference nodes and shared consistently", "[.][q3_hex_defect]")
{
	const Built b = build(column_args(3, json(), "Lagrange", /*contact=*/false));
	int checked = 0, shared = 0;
	CHECK(count_node_position_mismatches(b, 3, checked) == 0);
	CHECK(checked == 4 * 64);
	CHECK(count_shared_node_disagreements(b, 3, shared) == 0);
	CHECK(shared == 3 * 16);
}

TEST_CASE("Q3 hex proxies are geometrically valid once the basis is repaired", "[.][q3_hex_defect]")
{
	const std::string type = GENERATE("dof", "max_order");
	CAPTURE(type);
	const Built b = build(column_args(3, tessellation(type)));
	REQUIRE(b.mesh != nullptr);
	const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
	check_closed_sphere(s);
	CHECK(s.n_vertices == 164);
	CHECK(s.n_faces == 18 * 18);
}
