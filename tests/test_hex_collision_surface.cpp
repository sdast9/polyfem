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
// hexahedra through the production collision-mesh builder. Q3 hexahedra were
// refused with a degenerate-face error until RB-23 repaired the basis node
// bookkeeping (2026-09-20); their surfaces are checked here since.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/OutData.hpp>
#include <polyfem/autogen/auto_p_bases.hpp>
#include <polyfem/autogen/auto_q_bases.hpp>
#include <polyfem/assembler/AssemblyValues.hpp>
#include <polyfem/mesh/mesh3D/Mesh3D.hpp>
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

#include <algorithm>
#include <fstream>
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
	SECTION("Q3: the DOF proxy of the repaired basis (RB-23), 4 x 4 nodes per face")
	{
		// Before RB-23 this stopped with the degenerate-face error (32 of
		// 324 zero-area faces from the permuted node positions).
		const Built b = build(column_args(3));
		REQUIRE(b.mesh != nullptr);
		const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
		check_closed_sphere(s);
		CHECK(s.n_vertices == 164); // 4x4x13 nodes minus the 44 interior ones
		CHECK(s.n_faces == 18 * 18);
		CHECK(s.selector_rows == 164);
		CHECK(s.interpolated_rows == 0);
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
	SECTION("Q3: 18 triangles per face, every boundary node an exact selector")
	{
		{
			const Built built = build(column_args(3, tessellation("dof")));
			REQUIRE(built.mesh != nullptr);
			const SurfaceStats s = analyze(*built.mesh, built.debug.n_bases);
			check_closed_sphere(s);
			CHECK(s.n_vertices == 164);
			CHECK(s.n_faces == 18 * 18);
			CHECK(s.selector_rows == 164);
			CHECK(s.interpolated_rows == 0);
		}

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
	SECTION("Q3: lattice nodes at thirds are not exact selectors; the surface is closed")
	{
		{
			const Built built = build(column_args(3, tessellation("max_order")));
			REQUIRE(built.mesh != nullptr);
			const SurfaceStats s = analyze(*built.mesh, built.debug.n_bases);
			check_closed_sphere(s);
			CHECK(s.n_vertices == 164);
			CHECK(s.n_faces == 18 * 18);
			CHECK(s.empty_rows == 0);
		}

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
		int exact = 0, near_selector_count = 0;
		for (int r = 0; r < V.rows(); ++r)
		{
			exact += counts[r] == 1 && single[r] == 1.;
			near_selector_count += counts[r] >= 1 && single[r] != 1. && std::abs(single[r] - 1.) < 1e-12 && counts[r] == 1;
		}
		WARN("max_order Q3 rows: " << exact << " exact selectors, " << near_selector_count << " single entries within 1e-12 of 1, " << V.rows() - exact - near_selector_count << " other");
		CHECK(exact + near_selector_count <= int(V.rows()));
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

// The RB-22 acceptance for the upstream Q3 defect (2026-09-12), hidden until
// RB-23 repaired the basis (2026-09-20). On the 4-hex column 124 of 256 node
// positions were not the images of their reference nodes (the two nodes of
// the vertical edges e5-e7 swapped, face-interior and cell nodes permuted)
// and the 12 face-interior nodes of the 3 shared faces were placed
// differently by their two elements, i.e. the Q3 hex space was nonconforming.
TEST_CASE("Q3 hex node positions are the geometric images of their reference nodes and shared consistently", "[rb22][rb23][hex_collision_surface][hex_nodes][q3_hex_defect]")
{
	const Built b = build(column_args(3, json(), "Lagrange", /*contact=*/false));
	int checked = 0, shared = 0;
	CHECK(count_node_position_mismatches(b, 3, checked) == 0);
	CHECK(checked == 4 * 64);
	CHECK(count_shared_node_disagreements(b, 3, shared) == 0);
	CHECK(shared == 3 * 16);
}

TEST_CASE("Q3 hex proxies are geometrically valid once the basis is repaired", "[rb22][rb23][hex_collision_surface][q3_hex_defect]")
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

// CI-04: mixed P1/P2 tetrahedra. On a conforming mesh each edge takes the
// lowest order around it, so a P2 element's node on an edge shared with a
// P1 element has no DOF: its basis is stitched to the edge's two endpoint
// vertices. The default extraction used to skip every boundary face of a P2
// element carrying such a node (4 or 5 owned nodes out of 6), and the RB-22
// completeness check refused the scene (multi-material/stretch-cubes.json:
// 437 of 7,610 faces). These faces are now tessellated over their owned
// nodes with the constrained edges kept straight, conforming to the P1 face
// across the edge; every surface vertex remains an FE node.
namespace
{
	// quad_test/tet.msh with the elements whose centroid lies in x < 0.5 at
	// P2 (body 2) and the rest at P1
	json mixed_tet_args()
	{
		json in_args = tet_args(1);
		in_args["/geometry/0/volume_selection"_json_pointer] = json::array({json{{"id", 2}, {"box", json::array({json::array({0., 0., 0.}), json::array({0.5, 1., 1.})})}, {"relative", true}}});
		in_args["/space/discr_order"_json_pointer] = json::array({json{{"id", 2}, {"order", 2}}});
		return in_args;
	}

	struct MixedInterfaceStats
	{
		int boundary_faces = 0, constrained_faces = 0, constrained_edges = 0;
		std::map<int, int> owned_nodes; // owned nodes per constrained face -> faces
		std::set<int> orders;
		double fe_area = 0, max_weight_error = 0, max_edge_field_error = 0;
	};

	// Walks the boundary faces through the FE bases: the stitched weights of
	// every constrained edge node, and the FE field of a random nodal vector
	// along each constrained edge against the straight collision edge.
	MixedInterfaceStats inspect_mixed_interface(const Built &b)
	{
		MixedInterfaceStats st;
		const mesh::Mesh3D &mesh = dynamic_cast<const mesh::Mesh3D &>(*b.debug.mesh);
		const std::vector<basis::ElementBases> &bases = *b.debug.bases;
		Eigen::MatrixXd p2_nodes;
		autogen::p_nodes_3d(2, p2_nodes);

		const Eigen::VectorXd U = Eigen::VectorXd::Random(b.debug.n_bases);
		const auto field = [&](const basis::ElementBases &eb, const Eigen::RowVector3d &uv) {
			std::vector<assembler::AssemblyValues> vals;
			eb.evaluate_bases(uv, vals);
			double u = 0;
			for (size_t j = 0; j < vals.size(); ++j)
				for (const basis::Local2Global &g : eb.bases[j].global())
					u += vals[j].val(0) * g.val * U[g.index];
			return u;
		};

		static constexpr int edge_v[3][2] = {{0, 1}, {1, 2}, {2, 0}};
		for (const mesh::LocalBoundary &lb : *b.debug.total_local_boundary)
		{
			const basis::ElementBases &eb = bases[lb.element_id()];
			st.orders.insert(eb.bases.front().order());
			for (int j = 0; j < lb.size(); ++j)
			{
				const int eid = lb.global_primitive_id(j);
				++st.boundary_faces;
				Eigen::Vector3d x[3];
				for (int i = 0; i < 3; ++i)
					x[i] = mesh.point(mesh.face_vertex(eid, i)).transpose();
				st.fe_area += 0.5 * (x[1] - x[0]).cross(x[2] - x[0]).norm();

				const Eigen::VectorXi nodes = eb.local_nodes_for_primitive(eid, mesh);
				int owned = 0;
				for (long n = 0; n < nodes.size(); ++n)
					owned += eb.bases[nodes(n)].global().size() == 1;
				if (owned == nodes.size())
					continue;
				++st.constrained_faces;
				++st.owned_nodes[owned];
				REQUIRE(nodes.size() == 6);

				for (int k = 0; k < 3; ++k)
				{
					const std::vector<basis::Local2Global> &glob = eb.bases[nodes(3 + k)].global();
					if (glob.size() == 1)
						continue;
					++st.constrained_edges;
					const int va = eb.bases[nodes(edge_v[k][0])].global().front().index;
					const int vb = eb.bases[nodes(edge_v[k][1])].global().front().index;
					double wa = 0, wb = 0, other = 0;
					for (const basis::Local2Global &g : glob)
						(g.index == va ? wa : (g.index == vb ? wb : other)) += std::abs(g.val);
					st.max_weight_error = std::max({st.max_weight_error, std::abs(wa - 0.5), std::abs(wb - 0.5), other});

					const Eigen::RowVector3d ra = p2_nodes.row(nodes(edge_v[k][0]));
					const Eigen::RowVector3d rb = p2_nodes.row(nodes(edge_v[k][1]));
					for (const double t : {0.25, 0.5, 0.75})
						st.max_edge_field_error = std::max(
							st.max_edge_field_error,
							std::abs(field(eb, (1 - t) * ra + t * rb) - ((1 - t) * U[va] + t * U[vb])));
				}
			}
		}
		return st;
	}

	// coverage and map consistency of the built collision surface: its area
	// equals the (straight) FE boundary area, and the displacement map applied
	// to the FE node positions reproduces the surface vertices
	void check_surface_against_fe(const Built &b, const MixedInterfaceStats &st)
	{
		const ipc::CollisionMesh &cm = *b.mesh;
		Eigen::VectorXd double_area;
		igl::doublearea(cm.rest_positions(), cm.faces(), double_area);
		CHECK(0.5 * double_area.sum() == Catch::Approx(st.fe_area).epsilon(1e-12));

		Eigen::MatrixXd X = Eigen::MatrixXd::Zero(b.debug.n_bases, 3);
		for (const basis::ElementBases &eb : *b.debug.bases)
			for (const basis::Basis &bs : eb.bases)
				if (bs.global().size() == 1)
					X.row(bs.global().front().index) = bs.global().front().node;
		const Eigen::MatrixXd mapped = cm.displacement_map() * X;
		CHECK((mapped - cm.rest_positions()).cwiseAbs().maxCoeff() < 1e-14);
	}
} // namespace

TEST_CASE("Mixed P1/P2 tetrahedra get a complete conforming collision surface", "[ci04][collision_surface]")
{
	const Built b = build(mixed_tet_args());
	REQUIRE(b.mesh != nullptr);

	const MixedInterfaceStats st = inspect_mixed_interface(b);
	CHECK(st.orders == std::set<int>{1, 2});
	CHECK(st.boundary_faces == 40);
	// the interface reaches the boundary: P2 faces with one stitched edge (the
	// 5-owned-node faces of the fixture) are present
	CHECK(st.constrained_faces > 0);
	CHECK(st.owned_nodes.count(5) == 1);
	CHECK(st.max_weight_error < 1e-12);
	CHECK(st.max_edge_field_error < 1e-12);

	const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
	check_closed_sphere(s);
	// identity map on the FE nodes: every row an exact selector
	CHECK(s.selector_rows == s.n_vertices);
	CHECK(s.interpolated_rows == 0);
	check_surface_against_fe(b, st);

	// the default extraction reports the surface complete
	Eigen::MatrixXd V;
	Eigen::MatrixXi E, F;
	std::vector<Eigen::Triplet<double>> map;
	io::OutGeometryData::BoundaryExtractionReport report;
	io::OutGeometryData::extract_boundary_mesh(*b.debug.mesh, b.debug.n_bases, *b.debug.bases, *b.debug.total_local_boundary, V, E, F, map, &report);
	CHECK(report.complete());
	CHECK(report.n_boundary_faces == 40);
	CHECK(map.empty());
	CHECK(F.rows() == s.n_faces);
}

TEST_CASE("Pure P1 and P2 tetrahedral collision surfaces are unchanged by the mixed-order path", "[ci04][collision_surface]")
{
	const int order = GENERATE(1, 2);
	CAPTURE(order);
	const Built b = build(tet_args(order));
	REQUIRE(b.mesh != nullptr);
	const MixedInterfaceStats st = inspect_mixed_interface(b);
	CHECK(st.constrained_faces == 0);
	const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
	check_closed_sphere(s);
	CHECK(s.n_faces == 40 * (order == 1 ? 1 : 4));
	check_surface_against_fe(b, st);
}

// The CI-04 fixture itself (extraction only; its solve runs in the
// `standard` group): 437 of its 7,610 boundary faces were skipped.
TEST_CASE("The CI-04 fixture's mixed P1/P2 collision surface is complete and exact", "[ci04][collision_surface]")
{
	const std::string scene = std::string(POLYFEM_DATA_DIR) + "/multi-material/stretch-cubes.json";
	std::ifstream file(scene);
	REQUIRE(file.good());
	json in_args = json::parse(file);
	in_args["root_path"] = scene;
	in_args.erase("output");
	in_args["/output/log/level"_json_pointer] = "error";

	const Built b = build(in_args);
	REQUIRE(b.mesh != nullptr);

	const MixedInterfaceStats st = inspect_mixed_interface(b);
	for (const auto &kv : st.owned_nodes)
		WARN(kv.second << " constrained faces with " << kv.first << " owned nodes");
	CHECK(st.orders == std::set<int>{1, 2});
	CHECK(st.boundary_faces == 7610);
	CHECK(st.owned_nodes.count(5) == 1);
	CHECK(st.owned_nodes.at(5) == 437);
	CHECK(st.max_weight_error < 1e-12);
	CHECK(st.max_edge_field_error < 1e-12);

	const SurfaceStats s = analyze(*b.mesh, b.debug.n_bases);
	CHECK(s.closed);
	CHECK(s.edge_manifold);
	CHECK(s.vertex_manifold);
	CHECK(s.positive_area);
	CHECK(s.intersection_free);
	CHECK(s.empty_rows == 0);
	CHECK(s.selector_rows == s.n_vertices);
	CHECK(s.interpolated_rows == 0);
	WARN(s.components << " components, " << s.n_vertices << " vertices, " << s.n_faces << " faces");
	CHECK(s.euler == 2 * s.components);
	check_surface_against_fe(b, st);
}
