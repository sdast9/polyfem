// RB-11: geometry, material and input validation.
//
// Every check here is a named early failure for an input the plan calls
// genuinely invalid, next to the valid neighbour that must keep passing. The
// end-to-end matrix behind these (tools/rb11/run_matrix.py) reproduced the
// pre-change behaviour: segfaults on a missing mesh file or a bad vertex
// index, a hang on dt = 0, silently accepted duplicate elements, obstacle
// faces with garbage indices, E = 0, nu >= 1/2, a body without a material,
// per-element files of the wrong length or bound to the wrong rows,
// conflicting prescribed motion and a broad-phase name that aliased to
// another algorithm.
#include <polyfem/State.hpp>
#include <polyfem/Units.hpp>
#include <polyfem/assembler/Assembler.hpp>
#include <polyfem/assembler/AssemblerUtils.hpp>
#include <polyfem/assembler/GenericProblem.hpp>
#include <polyfem/assembler/HGODispersion.hpp>
#include <polyfem/assembler/Mass.hpp>
#include <polyfem/assembler/MatParams.hpp>
#include <polyfem/assembler/MultiModel.hpp>
#include <polyfem/assembler/NeoHookeanElasticity.hpp>
#include <polyfem/mesh/Mesh.hpp>
#include <polyfem/mesh/MeshUtils.hpp>
#include <polyfem/solver/forms/ContactForm.hpp>
#include <polyfem/utils/ExpressionValue.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/MatrixUtils.hpp>

#include <geogram/mesh/mesh.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <map>
#include <filesystem>
#include <fstream>

using namespace polyfem;
using namespace polyfem::assembler;
using namespace polyfem::mesh;
using Catch::Matchers::ContainsSubstring;

namespace
{
	// Two positively oriented tets sharing the face (0,1,2): body-id friendly.
	Eigen::MatrixXd two_tet_vertices()
	{
		Eigen::MatrixXd V(5, 3);
		V << 0, 0, 0,
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			0, 0, -1;
		return V;
	}

	Eigen::MatrixXi two_tets()
	{
		Eigen::MatrixXi T(2, 4);
		T << 0, 1, 2, 3,
			0, 2, 1, 4;
		return T;
	}

	std::unique_ptr<Mesh> two_tet_mesh()
	{
		return Mesh::create(two_tet_vertices(), two_tets(), false);
	}

	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	json minimal_args(const std::string &mesh_path)
	{
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", mesh_path}}});
		args["materials"] = {{"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}, {"rho", 1000.0}};
		args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
		args["/output/directory"_json_pointer] = "";
		args["/output/log/quiet"_json_pointer] = true;
		args["/output/log/level"_json_pointer] = "off";
		args["/output/paraview/file_name"_json_pointer] = "";
		return args;
	}
} // namespace

// ---------------------------------------------------------------------------
// Mesh topology
// ---------------------------------------------------------------------------

TEST_CASE("cell matrices with invalid vertex indices are refused", "[input_validation][mesh]")
{
	const Eigen::MatrixXd V = two_tet_vertices();

	SECTION("valid")
	{
		CHECK(Mesh::create(V, two_tets(), false) != nullptr);
	}
	SECTION("out of range")
	{
		Eigen::MatrixXi T = two_tets();
		T(1, 3) = 99;
		CHECK_THROWS_WITH(Mesh::create(V, T, false), ContainsSubstring("references vertex 99"));
	}
	SECTION("negative")
	{
		Eigen::MatrixXi T = two_tets();
		T(0, 0) = -3;
		CHECK_THROWS_WITH(Mesh::create(V, T, false), ContainsSubstring("references vertex -3"));
	}
	SECTION("repeated vertex")
	{
		Eigen::MatrixXi T = two_tets();
		T(0, 3) = T(0, 2);
		CHECK_THROWS_WITH(Mesh::create(V, T, false), ContainsSubstring("lists vertex 2 twice"));
	}
	SECTION("too few corners")
	{
		Eigen::MatrixXi T(1, 4);
		T << 0, 1, 2, -1;
		CHECK_THROWS_WITH(Mesh::create(V, T, false), ContainsSubstring("lists only 3 vertices"));
	}
}

TEST_CASE("duplicate rest elements are refused", "[input_validation][mesh]")
{
	SECTION("valid two-tet mesh")
	{
		const auto mesh = two_tet_mesh();
		REQUIRE(mesh != nullptr);
		CHECK_NOTHROW(validate_rest_elements(*mesh, "two tets"));
	}
	SECTION("the same tet twice, differently ordered, in the cell matrix")
	{
		Eigen::MatrixXi T(3, 4);
		T << 0, 1, 2, 3,
			0, 2, 1, 4,
			1, 2, 3, 0; // a permutation of element 0 is still element 0 (topological check)
		CHECK_THROWS_WITH(Mesh::create(two_tet_vertices(), T, false), ContainsSubstring("Elements 0 and 2 are the same element"));
	}
	SECTION("a loaded mesh with a duplicated element")
	{
		// geogram-loaded meshes get the same topological check before the
		// connectivity is built (RB-12: geogram's adjacency assert fired
		// first in Debug builds); validate_rest_elements covers geometry
		GEO::Mesh M;
		const Eigen::MatrixXd V = two_tet_vertices();
		M.vertices.create_vertices(V.rows());
		for (int i = 0; i < V.rows(); ++i)
			for (int d = 0; d < 3; ++d)
				M.vertices.point(i)[d] = V(i, d);
		Eigen::MatrixXi T(3, 4);
		T << 0, 1, 2, 3,
			0, 2, 1, 4,
			0, 1, 2, 3;
		for (int c = 0; c < T.rows(); ++c)
			M.cells.create_tet(T(c, 0), T(c, 1), T(c, 2), T(c, 3));
		CHECK_THROWS_WITH(Mesh::create(M, false), ContainsSubstring("Elements 0 and 2 are the same element"));
	}
}

TEST_CASE("obstacle surfaces are validated", "[input_validation][obstacle]")
{
	Eigen::MatrixXd V(4, 3);
	V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
	Eigen::VectorXi codim_vertices;
	Eigen::MatrixXi codim_edges;

	SECTION("an open slab is valid")
	{
		Eigen::MatrixXi F(2, 3);
		F << 0, 1, 2, 0, 2, 3;
		CHECK_NOTHROW(validate_surface_mesh("slab", V, codim_vertices, codim_edges, F));
	}
	SECTION("a T-junction (edge shared by three faces) is a valid nonmanifold obstacle")
	{
		// slab (0,1,2),(0,2,3) plus a fin below and a fin above its edge (0,1)
		Eigen::MatrixXd W(6, 3);
		W << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0.5, 0, -1, 0.5, 0, 1;
		Eigen::MatrixXi F(4, 3);
		F << 0, 1, 2, 0, 2, 3, 0, 4, 1, 0, 1, 5;
		std::map<std::pair<int, int>, int> degree;
		for (int f = 0; f < F.rows(); ++f)
			for (int k = 0; k < 3; ++k)
			{
				const int a = F(f, k), b = F(f, (k + 1) % 3);
				++degree[{std::min(a, b), std::max(a, b)}];
			}
		int max_degree = 0;
		for (const auto &[edge, n] : degree)
			max_degree = std::max(max_degree, n);
		REQUIRE(degree[{0, 1}] == 3); // the fixture really is nonmanifold
		REQUIRE(max_degree == 3);
		CHECK_NOTHROW(validate_surface_mesh("shelf", W, codim_vertices, codim_edges, F));
	}
	SECTION("face index out of range")
	{
		Eigen::MatrixXi F(2, 3);
		F << 0, 1, 2, 0, 2, 98;
		CHECK_THROWS_WITH(validate_surface_mesh("slab", V, codim_vertices, codim_edges, F), ContainsSubstring("face 1 references vertex 98"));
	}
	SECTION("duplicate face")
	{
		Eigen::MatrixXi F(3, 3);
		F << 0, 1, 2, 0, 2, 3, 2, 0, 1;
		CHECK_THROWS_WITH(validate_surface_mesh("slab", V, codim_vertices, codim_edges, F), ContainsSubstring("faces 0 and 2 are the same face"));
	}
	SECTION("zero-area face")
	{
		Eigen::MatrixXd W(5, 3);
		W << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0.5, 0, 0;
		Eigen::MatrixXi F(3, 3);
		F << 0, 1, 2, 0, 2, 3, 0, 4, 1;
		CHECK_THROWS_WITH(validate_surface_mesh("slab", W, codim_vertices, codim_edges, F), ContainsSubstring("face 2 has zero area"));
	}
	SECTION("zero-length and duplicate edges")
	{
		Eigen::MatrixXi F;
		Eigen::MatrixXi E(2, 2);
		E << 0, 1, 1, 0;
		CHECK_THROWS_WITH(validate_surface_mesh("edges", V, codim_vertices, E, F), ContainsSubstring("edges 0 and 1 are the same edge"));
		E << 0, 1, 2, 2;
		CHECK_THROWS_WITH(validate_surface_mesh("edges", V, codim_vertices, E, F), ContainsSubstring("joins vertex 2 to itself"));
	}
	SECTION("codimensional point out of range")
	{
		Eigen::MatrixXi F;
		Eigen::VectorXi P(2);
		P << 0, 7;
		CHECK_THROWS_WITH(validate_surface_mesh("points", V, P, codim_edges, F), ContainsSubstring("point 1 references vertex 7"));
	}
}

// ---------------------------------------------------------------------------
// Per-element material values: global rows or body-local rows, by length
// ---------------------------------------------------------------------------

TEST_CASE("per-element value lists bind to global or body-local rows by length", "[input_validation][material]")
{
	Units units;
	const Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
	// four elements, bodies 7 8 7 8 (two elements each)
	const std::vector<int> body_ids = {7, 8, 7, 8};

	SECTION("one row per element of the whole mesh: global element id (the HDA contract)")
	{
		Mass mass;
		mass.set_size(3);
		const json materials = json::array({{{"id", 7}, {"rho", json::array({10.0, 20.0, 11.0, 21.0})}},
											{{"id", 8}, {"rho", json::array({10.0, 20.0, 11.0, 21.0})}}});
		mass.set_materials(body_ids, materials, units, "");
		CHECK(mass.density()(p, p, 0, 0) == 10.0);
		CHECK(mass.density()(p, p, 0, 1) == 20.0);
		CHECK(mass.density()(p, p, 0, 2) == 11.0);
		CHECK(mass.density()(p, p, 0, 3) == 21.0);
	}
	SECTION("one row per element of the body: body-local index (upstream #333)")
	{
		Mass mass;
		mass.set_size(3);
		const json materials = json::array({{{"id", 7}, {"rho", json::array({10.0, 11.0})}},
											{{"id", 8}, {"rho", json::array({20.0, 21.0})}}});
		mass.set_materials(body_ids, materials, units, "");
		CHECK(mass.density()(p, p, 0, 0) == 10.0);
		CHECK(mass.density()(p, p, 0, 1) == 20.0);
		CHECK(mass.density()(p, p, 0, 2) == 11.0);
		CHECK(mass.density()(p, p, 0, 3) == 21.0);
	}
	SECTION("any other length does not describe the mesh")
	{
		Mass mass;
		mass.set_size(3);
		const json materials = json::array({{{"id", 7}, {"rho", json::array({10.0, 11.0, 12.0})}},
											{{"id", 8}, {"rho", 1.0}}});
		CHECK_THROWS_WITH(mass.set_materials(body_ids, materials, units, ""), ContainsSubstring("has 3 entries"));
	}
	SECTION("a single material object needs one row per element")
	{
		Mass mass;
		mass.set_size(3);
		CHECK_THROWS_WITH(mass.set_materials(body_ids, json{{"rho", json::array({1.0, 2.0, 3.0})}}, units, ""), ContainsSubstring("has 3 entries"));
		Mass ok;
		ok.set_size(3);
		ok.set_materials(body_ids, json{{"rho", json::array({1.0, 2.0, 3.0, 4.0})}}, units, "");
		CHECK(ok.density()(p, p, 0, 3) == 4.0);
	}
	SECTION("a list of expressions is not a per-element value")
	{
		Mass mass;
		mass.set_size(3);
		CHECK_THROWS_WITH(mass.set_materials(body_ids, json{{"rho", json::array({"1", "2", "3", "4"})}}, units, ""), ContainsSubstring("list of expressions"));
	}
}

TEST_CASE("per-element fibre files bind to global or body-local rows by length", "[input_validation][material][fiber]")
{
	const auto dir = scratch_dir("polyfem-rb11-fibers");
	const auto write_vtk = [&](const std::string &name, const std::vector<Eigen::Vector3d> &rows) {
		std::ofstream out(dir / name);
		out << "# vtk DataFile Version 3.0\nfibres\nASCII\nDATASET UNSTRUCTURED_GRID\nCELL_DATA " << rows.size() << "\nVECTORS FIB float\n";
		for (const auto &r : rows)
			out << r(0) << " " << r(1) << " " << r(2) << "\n";
		return (dir / name).string();
	};
	const Eigen::Vector3d ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);
	Units units;
	const std::vector<int> body_ids = {7, 8, 7, 8};
	const Eigen::RowVector3d p = Eigen::RowVector3d::Zero();

	const auto fibre_of = [&](const HGODispersion &hgo, const int e) {
		return hgo.parameters().at("fiber_direction_y")(p, p, 0, e);
	};

	SECTION("global rows")
	{
		const std::string path = write_vtk("global.vtk", {ex, ey, ex, ey});
		HGODispersion hgo;
		hgo.set_size(3);
		const json fibre = {{"type", "per_element_file"}, {"path", path}, {"field", "FIB"}};
		const json materials = json::array({{{"id", 7}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}},
											{{"id", 8}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}}});
		hgo.set_materials(body_ids, materials, units, "");
		CHECK(fibre_of(hgo, 0) == 0.0);
		CHECK(fibre_of(hgo, 1) == 1.0);
		CHECK(fibre_of(hgo, 2) == 0.0);
		CHECK(fibre_of(hgo, 3) == 1.0);
	}
	SECTION("body-local rows")
	{
		const std::string path = write_vtk("local.vtk", {ey, ez});
		HGODispersion hgo;
		hgo.set_size(3);
		const json fibre = {{"type", "per_element_file"}, {"path", path}, {"field", "FIB"}};
		const json materials = json::array({{{"id", 7}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}},
											{{"id", 8}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}}});
		hgo.set_materials(body_ids, materials, units, "");
		// element 0 = body 7 local 0 -> ey; element 1 = body 8 local 0 -> ey; elements 2, 3 -> ez
		CHECK(fibre_of(hgo, 0) == 1.0);
		CHECK(fibre_of(hgo, 1) == 1.0);
		CHECK(fibre_of(hgo, 2) == 0.0);
		CHECK(fibre_of(hgo, 3) == 0.0);
	}
	SECTION("wrong length")
	{
		const std::string path = write_vtk("wrong.vtk", {ex, ey, ez});
		HGODispersion hgo;
		hgo.set_size(3);
		const json fibre = {{"type", "per_element_file"}, {"path", path}, {"field", "FIB"}};
		const json materials = json::array({{{"id", 7}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}},
											{{"id", 8}, {"k1", 1.0}, {"k2", 1.0}, {"kappa", 0.1}, {"fiber_direction", fibre}}});
		CHECK_THROWS_WITH(hgo.set_materials(body_ids, materials, units, ""), ContainsSubstring("has 3 vectors"));
	}
	std::filesystem::remove_all(dir);
}

TEST_CASE("every body needs a material", "[input_validation][material]")
{
	Units units;
	Mass mass;
	mass.set_size(3);
	const json materials = json::array({{{"id", 7}, {"rho", 1.0}}});
	CHECK_THROWS_WITH(mass.set_materials({7, 8, 7, 8}, materials, units, ""), ContainsSubstring("No material for body id [8]"));
	CHECK_NOTHROW(mass.set_materials({7, 7}, materials, units, ""));
}

// ---------------------------------------------------------------------------
// Parameter ranges
// ---------------------------------------------------------------------------

TEST_CASE("elastic parameters are validated at the element barycenters", "[input_validation][material]")
{
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids(mesh->n_elements(), 0);

	const auto neo_hookean = [&](const json &mat) {
		NeoHookeanElasticity nh;
		nh.set_size(3);
		nh.set_materials(body_ids, mat, units, "");
		validate_material_parameters(nh, *mesh, 0.0, false, "test");
	};

	CHECK_NOTHROW(neo_hookean({{"E", 1e6}, {"nu", 0.3}}));
	CHECK_NOTHROW(neo_hookean({{"E", 1e6}, {"nu", -0.2}}));
	CHECK_NOTHROW(neo_hookean({{"E", 1e6}, {"nu", 0.49}}));
	CHECK_NOTHROW(neo_hookean({{"lambda", 1e6}, {"mu", 1e5}}));
	CHECK_THROWS_WITH(neo_hookean({{"E", 0.0}, {"nu", 0.3}}), ContainsSubstring("shear modulus must be positive")); // E = 0 gives mu = 0 (and a nan back-computed E)
	CHECK_THROWS_WITH(neo_hookean({{"E", -1e6}, {"nu", 0.3}}), ContainsSubstring("Young's modulus must be positive"));
	CHECK_THROWS_WITH(neo_hookean({{"E", 1e6}, {"nu", 0.5}}), ContainsSubstring("must be finite"));
	CHECK_THROWS_WITH(neo_hookean({{"E", 1e6}, {"nu", 0.7}}), ContainsSubstring("Poisson's ratio must lie in (-1, 1/2)"));
	CHECK_THROWS_WITH(neo_hookean({{"E", 1e6}, {"nu", -1.5}}), ContainsSubstring("Poisson's ratio must lie in (-1, 1/2)"));
	CHECK_THROWS_WITH(neo_hookean({{"E", "0/0"}, {"nu", 0.3}}), ContainsSubstring("must be finite"));
	CHECK_THROWS_WITH(neo_hookean({{"E", "1/0"}, {"nu", 0.3}}), ContainsSubstring("must be finite"));
	CHECK_THROWS_WITH(neo_hookean({{"lambda", -1e6}, {"mu", 1e5}}), ContainsSubstring("bulk modulus must be positive"));
	CHECK_THROWS_WITH(neo_hookean({{"lambda", 1e6}, {"mu", -1e5}}), ContainsSubstring("shear modulus must be positive"));
	// the element is named
	CHECK_THROWS_WITH(neo_hookean({{"E", "if(x - 0.2, -1, 1e6)"}, {"nu", 0.3}}), ContainsSubstring("on element 0 (body id 0"));
}

TEST_CASE("density, fibre and dispersion parameters are validated", "[input_validation][material]")
{
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids(mesh->n_elements(), 0);

	SECTION("density")
	{
		const auto density = [&](const double rho, const bool transient) {
			Mass mass;
			mass.set_size(3);
			mass.set_materials(body_ids, json{{"rho", rho}}, units, "");
			validate_material_parameters(mass, *mesh, 0.0, transient, "mass");
		};
		CHECK_NOTHROW(density(1000.0, true));
		CHECK_NOTHROW(density(0.0, false));
		CHECK_NOTHROW(density(0.0, true)); // a warning, not an error
		CHECK_THROWS_WITH(density(-1.0, true), ContainsSubstring("density must be nonnegative"));
	}
	SECTION("HGO dispersion: the kappa domain is [0, 1/d] of the law")
	{
		const auto hgo = [&](const json &mat) {
			HGODispersion h;
			h.set_size(3);
			json m = mat;
			m["fiber_direction"] = json::array({0, 0, 1});
			h.set_materials(body_ids, m, units, "");
			validate_material_parameters(h, *mesh, 0.0, false, "hgo");
		};
		CHECK_NOTHROW(hgo({{"k1", 1e4}, {"k2", 5.0}, {"kappa", 0.2}}));
		CHECK_NOTHROW(hgo({{"k1", 0.0}, {"k2", 5.0}, {"kappa", 0.0}}));
		CHECK_NOTHROW(hgo({{"k1", 1e4}, {"k2", 5.0}, {"kappa", 1.0 / 3.0}})); // the 3D endpoint
		CHECK_THROWS_WITH(hgo({{"k1", 1e4}, {"k2", 0.0}, {"kappa", 0.2}}), ContainsSubstring("k2 must be positive"));
		CHECK_THROWS_WITH(hgo({{"k1", -1.0}, {"k2", 5.0}, {"kappa", 0.2}}), ContainsSubstring("k1 must be nonnegative"));
		CHECK_THROWS_WITH(hgo({{"k1", 1e4}, {"k2", 5.0}, {"kappa", 0.6}}), ContainsSubstring("kappa must lie in [0, 1/3]"));
		CHECK_THROWS_WITH(hgo({{"k1", 1e4}, {"k2", 5.0}, {"kappa", -0.1}}), ContainsSubstring("kappa must lie in [0, 1/3]"));

		// 2D: two triangles, the domain extends to 1/2 (review counterexample: 0.4 was refused)
		Eigen::MatrixXd V2(4, 2);
		V2 << 0, 0, 1, 0, 1, 1, 0, 1;
		Eigen::MatrixXi T2(2, 3);
		T2 << 0, 1, 2, 0, 2, 3;
		const auto mesh2 = Mesh::create(V2, T2, false);
		REQUIRE(mesh2 != nullptr);
		const std::vector<int> body2(mesh2->n_elements(), 0);
		const auto hgo2 = [&](const double kappa, const bool composite) {
			json hgo_json = {{"type", "HGODispersion"}, {"k1", 1e4}, {"k2", 5.0}, {"kappa", kappa}, {"fiber_direction", json::array({0, 1})}};
			std::shared_ptr<Assembler> a;
			json m;
			if (composite)
			{
				a = AssemblerUtils::make_assembler("MaterialSum");
				m = {{"models", json::array({{{"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}}, hgo_json})}};
			}
			else
			{
				a = AssemblerUtils::make_assembler("HGODispersion");
				m = hgo_json;
			}
			REQUIRE(a != nullptr);
			a->set_size(2);
			a->set_materials(body2, m, units, "");
			validate_material_parameters(*a, *mesh2, 0.0, false, "hgo2d");
		};
		for (const bool composite : {false, true})
		{
			INFO("composite " << composite);
			CHECK_NOTHROW(hgo2(0.4, composite));
			CHECK_NOTHROW(hgo2(0.5, composite));
			CHECK_THROWS_WITH(hgo2(0.6, composite), ContainsSubstring("kappa must lie in [0, 1/2]"));
		}
	}
}

TEST_CASE("fibre directions: constant, expression and dimension", "[input_validation][material][fiber]")
{
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids(mesh->n_elements(), 0);
	const auto fibre = [&](const json &direction, const int size, const mesh::Mesh &m) {
		const auto a = AssemblerUtils::make_assembler("MaterialSum");
		REQUIRE(a != nullptr);
		a->set_size(size);
		const json mat = {{"models", json::array({{{"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}},
												  {{"type", "HGOFiber"}, {"k1", 1e4}, {"k2", 5.0}, {"fiber_direction", direction}}})}};
		a->set_materials(std::vector<int>(m.n_elements(), 0), mat, units, "");
		validate_material_parameters(*a, m, 0.0, false, "fibre");
	};
	CHECK_NOTHROW(fibre(json::array({0, 0, 1}), 3, *mesh));
	CHECK_NOTHROW(fibre(json::array({0, 0, 2}), 3, *mesh));           // non-unit: normalised by the law
	CHECK_NOTHROW(fibre(json::array({"x + 1", "0", "1"}), 3, *mesh)); // expression, nonzero everywhere
	CHECK_THROWS_WITH(fibre(json::array({0, 0, 0}), 3, *mesh), ContainsSubstring("is a zero vector"));
	CHECK_THROWS_WITH(fibre(json::array({"0*x", "0", "0"}), 3, *mesh), ContainsSubstring("fibre direction is a zero vector"));

	Eigen::MatrixXd V2(4, 2);
	V2 << 0, 0, 1, 0, 1, 1, 0, 1;
	Eigen::MatrixXi T2(2, 3);
	T2 << 0, 1, 2, 0, 2, 3;
	const auto mesh2 = Mesh::create(V2, T2, false);
	REQUIRE(mesh2 != nullptr);
	CHECK_NOTHROW(fibre(json::array({0, 1}), 2, *mesh2));
	CHECK_THROWS_WITH(fibre(json::array({0, 0, 1}), 2, *mesh2), ContainsSubstring("components but the problem is 2D"));
}

TEST_CASE("per-element material files are detected for the remeshing refusal", "[input_validation][material]")
{
	const auto dir = scratch_dir("polyfem-rb11-remesh");
	const auto file = dir / "E.txt";
	std::ofstream(file) << "1e6\n2e6\n";
	std::string where;
	CHECK_FALSE(materials_use_per_element_files(json{{"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}}, "", where));
	CHECK_FALSE(materials_use_per_element_files(json{{"type", "NeoHookean"}, {"E", "1e6 * (1 + x)"}, {"nu", 0.3}}, "", where));
	CHECK(materials_use_per_element_files(json{{"type", "NeoHookean"}, {"E", file.string()}, {"nu", 0.3}}, "", where));
	CHECK(where == "materials/E");
	const json composite = json::array({{{"id", 1}, {"type", "MaterialSum"}, {"models", json::array({{{"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}}, {{"type", "HGOFiber"}, {"k1", 1.0}, {"k2", 1.0}, {"fiber_direction", {{"type", "per_element_file"}, {"path", "f.vtk"}}}}})}}});
	CHECK(materials_use_per_element_files(composite, "", where));
	CHECK(where == "materials[0]/models[1]/fiber_direction");
	std::filesystem::remove_all(dir);
}

TEST_CASE("shear-only laws accept the incompressible limit", "[input_validation][material]")
{
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids(mesh->n_elements(), 0);
	for (const std::string type : {"IsochoricNeoHookean", "IncompressibleLinearElasticity"})
	{
		const auto assembler = AssemblerUtils::make_assembler(type);
		REQUIRE(assembler != nullptr);
		assembler->set_size(3);
		assembler->set_materials(body_ids, json{{"E", 1e6}, {"nu", 0.5}}, units, "");
		INFO(type);
		CHECK_NOTHROW(validate_material_parameters(*assembler, *mesh, 0.0, false, type));
		const auto bad = AssemblerUtils::make_assembler(type);
		bad->set_size(3);
		bad->set_materials(body_ids, json{{"E", -1e6}, {"nu", 0.3}}, units, "");
		CHECK_THROWS_WITH(validate_material_parameters(*bad, *mesh, 0.0, false, type), ContainsSubstring("shear modulus must be positive"));
	}
	// inside a MaterialSum the child type is still recognised
	const auto sum = AssemblerUtils::make_assembler("MaterialSum");
	REQUIRE(sum != nullptr);
	sum->set_size(3);
	sum->set_materials(body_ids, json{{"models", json::array({{{"type", "IsochoricNeoHookean"}, {"E", 1e6}, {"nu", 0.5}}, {{"type", "VolumePenalty"}, {"k", 1e5}}})}}, units, "");
	CHECK_NOTHROW(validate_material_parameters(*sum, *mesh, 0.0, false, "sum"));
}

TEST_CASE("Ogden term lists must pair up", "[input_validation][material]")
{
	// RB-11 envelope stage: the spec accepted only a scalar `alphas` (a list
	// was refused) while the energy loops over the alphas and reads mus[N] for
	// each; a mismatch silently dropped terms or read out of range.
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids(mesh->n_elements(), 0);

	const auto unconstrained = [&](const json &mat) {
		const auto assembler = AssemblerUtils::make_assembler("UnconstrainedOgden");
		assembler->set_size(3);
		assembler->set_materials(body_ids, mat, units, "");
	};
	CHECK_NOTHROW(unconstrained({{"alphas", json::array({2.0, -2.0})}, {"mus", json::array({5e3, 1e3})}, {"Ds", json::array({1e-4})}}));
	CHECK_NOTHROW(unconstrained({{"alphas", 2.0}, {"mus", json::array({5e3})}, {"Ds", json::array({1e-4, 1e-5})}}));
	CHECK_THROWS_WITH(unconstrained({{"alphas", 2.0}, {"mus", json::array({5e3, 1e3})}, {"Ds", json::array({1e-4})}}),
					  ContainsSubstring("'alphas' has 1 term(s) but 'mus' has 2"));
	CHECK_THROWS_WITH(unconstrained({{"alphas", json::array({2.0, -2.0})}, {"mus", json::array({5e3})}, {"Ds", json::array({1e-4})}}),
					  ContainsSubstring("every Ogden term needs one alpha and one mu"));
	CHECK_THROWS_WITH(unconstrained({{"alphas", 2.0}, {"mus", json::array({5e3})}, {"Ds", json::array()}}),
					  ContainsSubstring("'Ds' needs at least one"));

	const auto incompressible = [&](const json &mat) {
		const auto assembler = AssemblerUtils::make_assembler("IncompressibleOgden");
		assembler->set_size(3);
		assembler->set_materials(body_ids, mat, units, "");
	};
	CHECK_NOTHROW(incompressible({{"c", json::array({5e3, 1e3})}, {"m", json::array({2.0, -2.0})}, {"k", 1e5}}));
	CHECK_THROWS_WITH(incompressible({{"c", json::array({5e3, 1e3})}, {"m", 2.0}, {"k", 1e5}}),
					  ContainsSubstring("'c' has 2 term(s) but 'm' has 1"));

	// two bodies must give the same number of terms
	std::vector<int> two_bodies = {0, 1};
	const auto assembler = AssemblerUtils::make_assembler("UnconstrainedOgden");
	assembler->set_size(3);
	CHECK_THROWS_WITH(
		assembler->set_materials(
			two_bodies,
			json::array({{{"id", 0}, {"alphas", json::array({2.0, -2.0})}, {"mus", json::array({5e3, 1e3})}, {"Ds", json::array({1e-4})}},
						 {{"id", 1}, {"alphas", json::array({2.0})}, {"mus", json::array({5e3})}, {"Ds", json::array({1e-4})}}}),
			units, ""),
		ContainsSubstring("has 1 term(s) on element 1 but 2 on the elements before it"));

	// the spec accepts a list of alphas (it refused one before this stage)
	const auto dir = scratch_dir("rb11-ogden");
	const auto mesh_path = dir / "two.mesh";
	{
		Eigen::MatrixXd V = two_tet_vertices();
		Eigen::MatrixXi T = two_tets();
		std::ofstream out(mesh_path);
		out << "MeshVersionFormatted 2\nDimension 3\nVertices\n"
			<< V.rows() << "\n";
		for (int i = 0; i < V.rows(); ++i)
			out << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << " 0\n";
		out << "Tetrahedra\n"
			<< T.rows() << "\n";
		for (int i = 0; i < T.rows(); ++i)
			out << T(i, 0) + 1 << " " << T(i, 1) + 1 << " " << T(i, 2) + 1 << " " << T(i, 3) + 1 << " 0\n";
		out << "End\n";
	}
	json args = minimal_args(mesh_path.string());
	args["materials"] = {{"type", "UnconstrainedOgden"}, {"alphas", json::array({2.0, -2.0})}, {"mus", json::array({5e3, 1e3})}, {"Ds", json::array({1e-4})}, {"rho", 1000.0}};
	State state;
	CHECK_NOTHROW(state.init(args, true));
	std::filesystem::remove_all(dir);
}

TEST_CASE("multi-model materials are validated against their own model only", "[input_validation][material]")
{
	// body 0: NeoHookean; body 1: HGODispersion (whose E/nu are absent, not zero)
	Units units;
	const auto mesh = two_tet_mesh();
	REQUIRE(mesh != nullptr);
	const std::vector<int> body_ids = {0, 1};
	const json materials = json::array({{{"id", 0}, {"type", "NeoHookean"}, {"E", 1e6}, {"nu", 0.3}},
										{{"id", 1}, {"type", "HGODispersion"}, {"k1", 1e4}, {"k2", 5.0}, {"kappa", 0.1}, {"fiber_direction", json::array({0, 0, 1})}}});
	const auto assembler = AssemblerUtils::make_assembler("MultiModels");
	REQUIRE(assembler != nullptr);
	assembler->set_size(3);
	auto *multi = dynamic_cast<MultiModel *>(assembler.get());
	REQUIRE(multi != nullptr);
	multi->init_multimodels({"NeoHookean", "HGODispersion"});
	assembler->set_materials(body_ids, materials, units, "");
	CHECK_NOTHROW(validate_material_parameters(*assembler, *mesh, 0.0, false, "multi"));

	json bad = materials;
	bad[1]["kappa"] = 0.5;
	const auto assembler2 = AssemblerUtils::make_assembler("MultiModels");
	assembler2->set_size(3);
	dynamic_cast<MultiModel *>(assembler2.get())->init_multimodels({"NeoHookean", "HGODispersion"});
	assembler2->set_materials(body_ids, bad, units, "");
	CHECK_THROWS_WITH(validate_material_parameters(*assembler2, *mesh, 0.0, false, "multi"), ContainsSubstring("kappa"));
}

TEST_CASE("a value-file path that does not exist is reported as such", "[input_validation][material]")
{
	utils::ExpressionValue value;
	CHECK_THROWS_WITH(value.init(std::string("materials/E_values.txt"), ""), ContainsSubstring("is not an existing value file"));
	CHECK_THROWS_WITH(value.init(std::string("2 +* x"), ""), ContainsSubstring("Invalid expression"));
	CHECK_NOTHROW(value.init(std::string("2 * x"), ""));
}

// ---------------------------------------------------------------------------
// Lumped mass, boundary conditions, settings
// ---------------------------------------------------------------------------

TEST_CASE("row-sum lumping with nonpositive nodal masses is reported", "[input_validation][mass]")
{
	// row 3 is all zero: an obstacle DOF appended to the space, massless by construction
	Eigen::SparseMatrix<double> M(4, 4);
	std::vector<Eigen::Triplet<double>> t = {{0, 0, 2.0}, {0, 1, -0.5}, {1, 0, -0.5}, {1, 1, 1.0}, {2, 2, 1.0}};
	M.setFromTriplets(t.begin(), t.end());
	CHECK(utils::check_lumped_mass(M) == 0);

	// row 0 sums to -0.5 (a P2-corner-like row), row 2 sums to exactly 0 although it carries mass;
	// upstream lumps quadratic bodies in every contact example, so this is a warning, not an error
	std::vector<Eigen::Triplet<double>> u = {{0, 0, 1.0}, {0, 1, -1.5}, {1, 0, -1.5}, {1, 1, 4.0}, {2, 2, 1.0}, {2, 1, -1.0}, {1, 2, -1.0}};
	M.setFromTriplets(u.begin(), u.end());
	CHECK(utils::check_lumped_mass(M) == 2);
}

TEST_CASE("duplicate or shadowed Dirichlet entries are refused", "[input_validation][bc]")
{
	const auto parse = [](json dirichlet) {
		for (auto &entry : dirichlet) // the spec injects these defaults before set_parameters
			entry["interpolation"] = json::array({{{"type", "none"}}});
		GenericTensorProblem problem("GenericTensor");
		json params = {{"dirichlet_boundary", dirichlet}, {"root_path", ""}};
		problem.set_parameters(params, "");
	};
	CHECK_NOTHROW(parse(json::array({{{"id", 1}, {"value", {0, 0, 0}}}, {{"id", 2}, {"value", {0, 0, "t"}}}})));
	CHECK_NOTHROW(parse(json::array({{{"id", 2}, {"value", {0, 0, "t"}}}, {{"id", "all"}, {"value", {0, 0, 0}}}})));
	CHECK_THROWS_WITH(parse(json::array({{{"id", 2}, {"value", {0, 0, 0}}}, {{"id", 2}, {"value", {0, 0, "t"}}}})), ContainsSubstring("two entries for id 2"));
	CHECK_THROWS_WITH(parse(json::array({{{"id", "all"}, {"value", {0, 0, 0}}}, {{"id", 2}, {"value", {0, 0, "t"}}}})), ContainsSubstring("would never act"));
}

TEST_CASE("broad-phase names are validated instead of aliased", "[input_validation][settings]")
{
	using solver::ContactForm;
	CHECK(ContactForm::is_known_broad_phase_name("hash_grid"));
	CHECK(ContactForm::is_known_broad_phase_name("sweep_and_prune"));
	CHECK(ContactForm::is_known_broad_phase_name("SAP"));
	CHECK(ContactForm::is_known_broad_phase_name("BVH"));
	CHECK_FALSE(ContactForm::is_known_broad_phase_name("hashgrid"));
	CHECK_FALSE(ContactForm::is_known_broad_phase_name(""));
	const ipc::BroadPhaseMethod sap = json("sweep_and_prune");
	CHECK(sap == ipc::BroadPhaseMethod::SWEEP_AND_PRUNE);
	const ipc::BroadPhaseMethod sap2 = json("SAP");
	CHECK(sap2 == ipc::BroadPhaseMethod::SWEEP_AND_PRUNE);
}

TEST_CASE("time schedule and contact settings are validated at init", "[input_validation][settings]")
{
	const auto init = [](const std::function<void(json &)> &edit, const bool strict = true) {
		json args = minimal_args("does-not-need-to-exist.msh");
		edit(args);
		State state;
		state.init(args, strict);
	};
	CHECK_NOTHROW(init([](json &a) { a["time"] = {{"dt", 0.25}, {"time_steps", 2}}; }));
	CHECK_NOTHROW(init([](json &a) { a["time"] = {{"tend", 0.5}, {"time_steps", 2}}; }));
	CHECK_THROWS_WITH(init([](json &a) { a["time"] = {{"dt", 0.0}, {"tend", 0.5}}; }), ContainsSubstring("time.dt must be a positive"));
	CHECK_THROWS_WITH(init([](json &a) { a["time"] = {{"t0", 1.0}, {"tend", 0.5}, {"time_steps", 2}}; }), ContainsSubstring("time.tend must be a finite time after time.t0"));
	CHECK_THROWS_WITH(init([](json &a) { a["time"] = {{"dt", 0.25}, {"time_steps", 0}}; }), ContainsSubstring("time.time_steps must be a positive"));
	CHECK_THROWS_WITH(init([](json &a) { a["contact"] = {{"enabled", true}, {"dhat", 0.0}}; }), ContainsSubstring("contact.dhat must be a positive"));
	// the spec rejects a misspelling (strict or not); the enum map must
	// know every name the spec offers, or the name silently aliases to the
	// map's first entry (the RB-05 sweep_and_prune observation)
	CHECK_THROWS_WITH(init([](json &a) {
						  a["contact"] = {{"enabled", true}, {"dhat", 1e-3}};
						  a["/solver/contact/CCD/broad_phase"_json_pointer] = "hashgrid";
					  }),
					  ContainsSubstring("Invalid input json"));
	{
		std::ifstream spec_file(std::filesystem::path(POLYFEM_TEST_DIR) / ".." / "json-specs" / "input-spec.json");
		REQUIRE(spec_file.is_open());
		json spec;
		spec_file >> spec;
		int checked = 0;
		for (const auto &rule : spec)
		{
			if (rule.value("pointer", "") != "/solver/contact/CCD/broad_phase")
				continue;
			for (const auto &option : rule["options"])
			{
				INFO("broad_phase option " << option.get<std::string>());
				CHECK(solver::ContactForm::is_known_broad_phase_name(option.get<std::string>()));
				++checked;
			}
		}
		CHECK(checked >= 12);
	}
	CHECK_NOTHROW(init([](json &a) {
		a["contact"] = {{"enabled", true}, {"dhat", 1e-3}};
		a["/solver/contact/CCD/broad_phase"_json_pointer] = "sweep_and_prune";
	}));
}
