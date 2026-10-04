// RB-11: geometry, material and input validation.
//
// Every check here is a named early failure for an input the plan calls
// genuinely invalid, next to the valid neighbour that must keep passing. The
// end-to-end matrix behind these (tools/rb11/run_matrix.py) reproduced the
// pre-change behaviour: segfaults on a missing mesh file or a bad vertex
// index, a hang on dt = 0 or on a Gmsh value that is not a number, silently
// accepted duplicate elements, obstacle faces with garbage indices, E = 0,
// nu >= 1/2, a body without a material, per-element files of the wrong
// length or bound to the wrong rows, conflicting prescribed motion and a
// broad-phase name that aliased to another algorithm.
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

// ---------------------------------------------------------------------------
// Gmsh files: every value of an ASCII section must convert completely
// ---------------------------------------------------------------------------

namespace
{
	// The one-tet file of the 2026-10-02 report, valid here; the report had a
	// numpy-2 repr, "np.float64(0.0)", as the x coordinate of node 1 (line 6).
	// mshio left its stream failed on such a token and looped forever looking
	// for $EndNodes, so the run never got past "Loading mesh".
	const std::vector<std::string> one_tet_v22 = {
		"$MeshFormat", "2.2 0 8", "$EndMeshFormat",
		"$Nodes", "4", "1 0 0 0", "2 1 0 0", "3 0 1 0", "4 0 0 1", "$EndNodes",
		"$Elements", "1", "1 4 2 1 1 1 2 3 4", "$EndElements"};

	// The same tet in MSH 4.1 ASCII, with a physical name and its volume entity.
	const std::vector<std::string> one_tet_v41 = {
		"$MeshFormat", "4.1 0 8", "$EndMeshFormat",
		"$PhysicalNames", "1", "3 1 \"body_1\"", "$EndPhysicalNames",
		"$Entities", "0 0 0 1", "1 0 0 0 1 1 1 1 1 0", "$EndEntities",
		"$Nodes", "1 4 1 4", "3 1 0 4", "1", "2", "3", "4", "0 0 0", "1 0 0", "0 1 0", "0 0 1", "$EndNodes",
		"$Elements", "1 1 1 1", "3 1 4 1", "1 1 2 3 4", "$EndElements"};

	/// `lines` with its 1-based line `line` replaced by `text`.
	std::vector<std::string> with_line(std::vector<std::string> lines, const size_t line, const std::string &text)
	{
		lines.at(line - 1) = text;
		return lines;
	}

	std::string write_msh(const std::filesystem::path &path, const std::vector<std::string> &lines, const std::string &eol = "\n")
	{
		std::ofstream out(path, std::ios::binary);
		for (const auto &line : lines)
			out << line << eol;
		return path.string();
	}
} // namespace

TEST_CASE("the reported Gmsh file with a numpy repr stops by name", "[input_validation][mesh][msh]")
{
	// The 2026-10-02 reproduction through the calls PolyFEM_bin makes: the
	// mesh is named relative to params.json.
	const auto dir = scratch_dir("rb11-msh-report");
	write_msh(dir / "one_tet.msh", with_line(one_tet_v22, 6, "1 np.float64(0.0) 0 0"));

	json args = json::parse(R"({"geometry": [{"mesh": "one_tet.msh"}],
		"materials": [{"id": 1, "type": "NeoHookean", "E": 1e5, "nu": 0.3}],
		"time": {"t0": 0, "tend": 1, "time_steps": 1, "quasistatic": true}})");
	args["root_path"] = (dir / "params.json").string();
	args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
	args["/output/directory"_json_pointer] = "";
	args["/output/log/quiet"_json_pointer] = true;
	args["/output/log/level"_json_pointer] = "off";

	State state;
	state.init(args, true);
	CHECK_THROWS_WITH(state.load_mesh(), ContainsSubstring("one_tet.msh: line 6, $Nodes: \"np.float64(0.0)\" is not a number (the x coordinate of node 1)"));
}

TEST_CASE("malformed values in Gmsh files are refused with their line and token", "[input_validation][mesh][msh]")
{
	const auto dir = scratch_dir("rb11-msh");
	int file = 0;
	const auto load = [&](const std::vector<std::string> &lines) {
		return Mesh::create(write_msh(dir / ("case" + std::to_string(file++) + ".msh"), lines), false);
	};
	const auto first = [](const std::vector<std::string> &lines, const size_t n) {
		return std::vector<std::string>(lines.begin(), lines.begin() + n);
	};

	SECTION("valid files load")
	{
		for (const auto &lines : {one_tet_v22, one_tet_v41})
		{
			const auto mesh = load(lines);
			REQUIRE(mesh != nullptr);
			CHECK(mesh->n_vertices() == 4);
			CHECK(mesh->n_cells() == 1);
		}
		CHECK(Mesh::create(write_msh(dir / "crlf.msh", one_tet_v41, "\r\n"), false) != nullptr);
		CHECK(load(with_line(one_tet_v22, 7, "2\t1.0e+00 0.0E0 -0")) != nullptr);
		// MSVC cannot take this as a raw string literal inside CHECK
		const std::string escaped_name = "3 1 \"body \\\"one\\\" (soft)\"";
		CHECK(load(with_line(one_tet_v41, 6, escaped_name)) != nullptr);
	}
	SECTION("MSH 2.2 nodes")
	{
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 7, "2 1.0abc 0 0")), ContainsSubstring("line 7, $Nodes: \"1.0abc\" is not a number (the x coordinate of node 2)"));
		// libc++ read "1.2" and then ".3" as the next value: the file shifted silently
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 7, "2 1.2.3 0 0")), ContainsSubstring("line 7, $Nodes: \"1.2.3\" is not a number"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 7, "2 0,5 0 0")), ContainsSubstring("line 7, $Nodes: \"0,5\" is not a number"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 7, "2 nan 0 0")), ContainsSubstring("line 7, $Nodes: \"nan\" is not a finite number (the x coordinate of node 2)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 9, "4 0 0 1e999")), ContainsSubstring("line 9, $Nodes: \"1e999\" is too large (the z coordinate of node 4)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 6, "a1 0 0 0")), ContainsSubstring("line 6, $Nodes: \"a1\" is not a positive integer (the tag of node entry 1 of 4)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 6, "0 0 0 0")), ContainsSubstring("line 6, $Nodes: \"0\" is not a positive integer (the tag of node entry 1 of 4)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 9, "3000000000 0 0 1")), ContainsSubstring("line 9, $Nodes: \"3000000000\" is too large (the tag of node entry 4 of 4)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 5, "four")), ContainsSubstring("line 5, $Nodes: \"four\" is not a non-negative integer (the number of nodes)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 5, "5")), ContainsSubstring("line 10, $Nodes: $EndNodes appears where the tag of node entry 5 of 5 was expected (the section holds fewer values than its counts declare)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 5, "3")), ContainsSubstring("line 9, $Nodes: \"4\" appears where $EndNodes was expected (the section holds more values than its counts declare)"));
		// a coordinate missing on line 7 shifts every later value
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 7, "2 1 0")), ContainsSubstring("line 8, $Nodes: \"0\" is not a positive integer (the tag of node entry 3 of 4); if line 8 looks right, a value is missing or extra before it"));
		// a count far beyond the data stops at the data's end without allocating for it
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 5, "1000000000000")), ContainsSubstring("$EndNodes appears where the tag of node entry 5 of 1000000000000 was expected"));
		CHECK_THROWS_WITH(load(first(one_tet_v22, 8)), ContainsSubstring("$Nodes: the file ends where the tag of node entry 4 of 4 was expected"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 8, "2 0 1 0")), ContainsSubstring("node tag 2 is used by two nodes"));
	}
	SECTION("MSH 2.2 elements")
	{
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4 2 1 1 1 2 x 4")), ContainsSubstring("line 13, $Elements: \"x\" is not a non-negative integer (node 3 of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4 2 1 1 1 2 3.5 4")), ContainsSubstring("line 13, $Elements: \"3.5\" is not a non-negative integer (node 3 of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4a 2 1 1 1 2 3 4")), ContainsSubstring("line 13, $Elements: \"4a\" is not an integer (the type of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 99 2 1 1 1 2 3 4")), ContainsSubstring("line 13, $Elements: element type 99 is not supported (the type of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4 -1 1 1 1 2 3 4")), ContainsSubstring("line 13, $Elements: \"-1\" is not a non-negative integer (the number of tags of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4 2 1 1 1 2 3")), ContainsSubstring("line 14, $Elements: $EndElements appears where node 4 of element 1 was expected"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 12, "2")), ContainsSubstring("line 14, $Elements: $EndElements appears where the tag of element entry 2 of 2 was expected"));
		CHECK_THROWS_WITH(load(first(one_tet_v22, 12)), ContainsSubstring("$Elements: the file ends where the tag of element entry 1 of 1 was expected"));
		// mshio's 2.2 regrouping indexed its node tables with this tag: out of bounds
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 13, "1 4 2 1 1 1 2 3 99")), ContainsSubstring("element 1 references node tag 99, which is not a node of the file"));
		// an empty element section used to stop as "memory allocation failed"
		CHECK_THROWS_WITH(load({"$MeshFormat", "2.2 0 8", "$EndMeshFormat", "$Nodes", "1", "1 0 0 0", "$EndNodes", "$Elements", "0", "$EndElements"}), ContainsSubstring("the file has no surface or volume elements"));
	}
	SECTION("MSH 4.1 nodes and elements")
	{
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 19, "np.float64(0.0) 0 0")), ContainsSubstring("line 19, $Nodes: \"np.float64(0.0)\" is not a number (the x coordinate of node 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 20, "1.2.3 0 0")), ContainsSubstring("line 20, $Nodes: \"1.2.3\" is not a number (the x coordinate of node 2)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 15, "a1")), ContainsSubstring("line 15, $Nodes: \"a1\" is not a positive integer (the tag of node entry 1 of 4 in node block 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 14, "3 1 x 4")), ContainsSubstring("line 14, $Nodes: \"x\" is not an integer from 0 to 3 (the parametric flag of node block 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 20, "1 0")), ContainsSubstring("line 23, $Nodes: $EndNodes appears where the z coordinate of node 4 was expected"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 13, "1 5 1 5")), ContainsSubstring("$Nodes: the header declares 5 nodes, but its blocks hold 4"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 17, "2")), ContainsSubstring("node tag 2 is used by two nodes"));
		CHECK_THROWS_WITH(load(with_line(with_line(with_line(one_tet_v41, 13, "1 4 1 3000000000"), 18, "3000000000"), 27, "1 1 2 3 3000000000")), ContainsSubstring("node tag 3000000000 is too large"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 27, "1 1 2 x 4")), ContainsSubstring("line 27, $Elements: \"x\" is not a non-negative integer (node 3 of element 1)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 26, "3 1 4 2")), ContainsSubstring("line 28, $Elements: $EndElements appears where the tag of element entry 2 of 2 in element block 1 was expected"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 25, "1 0 1 1")), ContainsSubstring("$Elements: the header declares 0 elements, but its blocks hold 1"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 27, "1 0 2 3 4")), ContainsSubstring("element 1 references node tag 0, which is not a node of the file"));
	}
	SECTION("the sections around the mesh")
	{
		CHECK_THROWS_WITH(load(with_line(one_tet_v22, 2, "2.2 zero 8")), ContainsSubstring("line 2, $MeshFormat: \"zero\" is not an integer from 0 to 1 (the file type)"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 6, "3 1")), ContainsSubstring("line 7, $PhysicalNames: $EndPhysicalNames appears where the name of physical name entry 1 of 1 was expected"));
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 10, "1 np.float64(0.0) 0 0 1 1 1 1 1 0")), ContainsSubstring("line 10, $Entities: \"np.float64(0.0)\" is not a number (the smallest x of volume 1)"));
		// a volume the header does not count would have lost its physical tag (body id 0)
		CHECK_THROWS_WITH(load(with_line(one_tet_v41, 9, "0 0 0 0")), ContainsSubstring("line 10, $Entities: \"1\" appears where $EndEntities was expected"));
		// data sections are not read value by value, but still stop by name
		std::vector<std::string> with_data = one_tet_v22;
		for (const std::string line : {"$NodeData", "1", "\"temp\"", "1", "0.0", "3", "0", "1", "4", "1 np.float64(1.0)", "2 1", "3 1", "4 1", "$EndNodeData"})
			with_data.push_back(line);
		CHECK_THROWS_WITH(load(with_data), ContainsSubstring("line 24, $NodeData: a value could not be read from the line \"1 np.float64(1.0)\""));
	}
}

// ---------------------------------------------------------------------------
// Gmsh files: parametric node blocks
// ---------------------------------------------------------------------------

namespace
{
	/// A one-tet MSH 4.1 binary file whose single node block has entity
	/// dimension `entity_dim` and parametric flag 1; mshio reads 3 +
	/// `entity_dim` of `values` per node.
	std::string write_msh41_binary(const std::filesystem::path &path, const int entity_dim, const std::vector<double> &values)
	{
		std::ofstream out(path, std::ios::binary);
		const auto put = [&](const auto value) { out.write(reinterpret_cast<const char *>(&value), sizeof(value)); };
		out << "$MeshFormat\n4.1 1 8\n";
		put(int(1));
		out << "\n$EndMeshFormat\n$Nodes\n";
		for (const size_t v : {1, 4, 1, 4}) // blocks, nodes, smallest and largest tag
			put(v);
		for (const int v : {entity_dim, 1, 1}) // entity dimension and tag, parametric flag
			put(v);
		for (const size_t v : {4, 1, 2, 3, 4}) // nodes in the block, their tags
			put(v);
		for (const double v : values)
			put(v);
		out << "\n$EndNodes\n$Elements\n";
		for (const size_t v : {1, 1, 1, 1}) // blocks, elements, smallest and largest tag
			put(v);
		for (const int v : {3, 1, 4}) // entity dimension and tag, element type (tet)
			put(v);
		for (const size_t v : {1, 1, 1, 2, 3, 4}) // elements in the block, the tet's tag and nodes
			put(v);
		out << "\n$EndElements\n";
		return path.string();
	}
} // namespace

TEST_CASE("parametric Gmsh node blocks keep their positions", "[input_validation][mesh][msh]")
{
	// A node block with parametric flag 1 (MSH 4.1 written with Gmsh's
	// Mesh.SaveParametric) stores x y z and then the node's coordinates on its
	// entity: u on a curve, u v on a surface, u v w in a volume. MshReader read
	// every block with a stride of 3: the reported file below loaded and solved
	// with the vertices (0,0,0), (.1,.2,.3), (1,0,0), (.4,.5,.6), and
	// Gmsh-written meshes stopped as "element N is flipped".
	const auto dir = scratch_dir("rb11-msh-parametric");
	int file = 0;
	const auto load = [&](const std::vector<std::string> &lines) {
		return Mesh::create(write_msh(dir / ("case" + std::to_string(file++) + ".msh"), lines), false);
	};
	const auto check_vertices = [](const std::unique_ptr<Mesh> &mesh, const Eigen::MatrixXd &expected) {
		REQUIRE(mesh != nullptr);
		REQUIRE(mesh->n_vertices() == expected.rows());
		for (int v = 0; v < expected.rows(); ++v)
		{
			const RowVectorNd p = mesh->point(v);
			INFO("vertex " << v << " at (" << p << ")");
			REQUIRE(p.size() == expected.cols());
			for (int d = 0; d < expected.cols(); ++d)
				CHECK(p(d) == expected(v, d));
		}
	};

	Eigen::MatrixXd unit_tet(4, 3);
	unit_tet << 0, 0, 0,
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	SECTION("a volume block: the 2026-10-03 reproduction")
	{
		check_vertices(load({"$MeshFormat", "4.1 0 8", "$EndMeshFormat",
							 "$Entities", "0 0 0 1", "1 0 0 0 1 1 1 1 1 0", "$EndEntities",
							 "$Nodes", "1 4 1 4", "3 1 1 4", "1", "2", "3", "4",
							 "0 0 0 0.1 0.2 0.3", "1 0 0 0.4 0.5 0.6", "0 1 0 0.7 0.8 0.9", "0 0 1 0.11 0.12 0.13", "$EndNodes",
							 "$Elements", "1 1 1 1", "3 1 4 1", "1 1 2 3 4", "$EndElements"}),
					   unit_tet);
		// its non-parametric twin
		check_vertices(load(one_tet_v41), unit_tet);
	}
	SECTION("a surface mesh: the three kinds of block Gmsh writes")
	{
		// A unit square in six triangles. The boundary loop's start point
		// stores x y z, the loop x y z u (u its arc-length fraction) and the
		// surface x y z u v.
		Eigen::MatrixXd square(6, 2);
		square << 0, 0,
			1, 0,
			1, 1,
			0, 1,
			0.25, 0.5,
			0.75, 0.5;
		check_vertices(load({"$MeshFormat", "4.1 0 8", "$EndMeshFormat",
							 "$Nodes", "3 6 1 6",
							 "0 1 0 1", "1", "0 0 0",
							 "1 1 1 3", "2", "3", "4", "1 0 0 0.25", "1 1 0 0.5", "0 1 0 0.75",
							 "2 1 1 2", "5", "6", "0.25 0.5 0 0.25 0.5", "0.75 0.5 0 0.75 0.5", "$EndNodes",
							 "$Elements", "1 6 1 6", "2 1 2 6",
							 "1 1 2 6", "2 1 6 5", "3 2 3 6", "4 3 4 5", "5 3 5 6", "6 4 1 5", "$EndElements"}),
					   square);
	}
	SECTION("binary files")
	{
		check_vertices(Mesh::create(write_msh41_binary(dir / "binary.msh", 3, {0, 0, 0, 0.1, 0.2, 0.3, 1, 0, 0, 0.4, 0.5, 0.6, 0, 1, 0, 0.7, 0.8, 0.9, 0, 0, 1, 0.11, 0.12, 0.13}), false),
					   unit_tet);
		// mshio does not range-check a binary block header: this block holds
		// 3 + (-1) values per node, and a stride of 3 read past their end
		CHECK_THROWS_WITH(Mesh::create(write_msh41_binary(dir / "binary_dim.msh", -1, {0, 0, 1, 0, 0, 1, 0, 0}), false),
						  ContainsSubstring("parametric node block 1 has entity dimension -1 (0 to 3 expected)"));
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

TEST_CASE("the convergent formulation's improved max operator is refused with semi-implicit stiffness at init", "[input_validation][settings][rbr04]")
{
	// RBR-04: the semi-implicit coefficient law is a sum of parent potentials
	// only for positive contributions; the improved max operator's
	// duplicate-removal corrections are negative. Refused by name before any
	// mesh is read (the form's constructor refuses it too), naming every
	// unsupported convergent option at once; the non-convergent default, area
	// weighting alone, and the other stiffness modes are accepted.
	const auto init = [](const json &contact, const json &barrier_stiffness) {
		json args = minimal_args("does-not-need-to-exist.msh");
		args["contact"] = contact;
		args["contact"]["enabled"] = true;
		args["contact"]["dhat"] = 1e-3;
		args["/solver/contact/barrier_stiffness"_json_pointer] = barrier_stiffness;
		State state;
		state.init(args, true);
	};
	const json fragment = {{"use_convergent_formulation", true}, {"use_area_weighting", false}, {"use_improved_max_operator", true}, {"use_physical_barrier", false}};

	// the plan's public fragment, with and without area weighting
	CHECK_THROWS_WITH(init(fragment, "semi_implicit"), ContainsSubstring("contact.use_improved_max_operator is unsupported in this mode"));
	CHECK_THROWS_WITH(init(fragment, "semi_implicit"), ContainsSubstring("does not support the improved max operator"));
	CHECK_THROWS_WITH(init(fragment, "semi_implicit"), ContainsSubstring("use_convergent_formulation = false"));
	{
		json area = fragment;
		area["use_area_weighting"] = true;
		CHECK_THROWS_WITH(init(area, "semi_implicit"), ContainsSubstring("contact.use_improved_max_operator is unsupported in this mode"));
	}
	// the convergent defaults (improved max and physical barrier both on): one message names both
	CHECK_THROWS_WITH(init({{"use_convergent_formulation", true}}, "semi_implicit"),
					  ContainsSubstring("contact.use_improved_max_operator and contact.use_physical_barrier are unsupported in this mode"));
	// the physical barrier alone is still refused (the constructor's existing rule, now stated at init)
	{
		json physical = fragment;
		physical["use_improved_max_operator"] = false;
		physical["use_physical_barrier"] = true;
		CHECK_THROWS_WITH(init(physical, "semi_implicit"), ContainsSubstring("contact.use_physical_barrier is unsupported in this mode"));
	}
	// area weighting alone keeps every contribution positive: accepted
	{
		json area_only = fragment;
		area_only["use_area_weighting"] = true;
		area_only["use_improved_max_operator"] = false;
		CHECK_NOTHROW(init(area_only, "semi_implicit"));
	}
	// the non-convergent default is the validated configuration
	CHECK_NOTHROW(init({{"use_convergent_formulation", false}}, "semi_implicit"));
	CHECK_NOTHROW(init(json::object(), "semi_implicit"));
	// the improved max operator stays available to the classic adaptive and fixed modes
	CHECK_NOTHROW(init(fragment, "adaptive"));
	CHECK_NOTHROW(init(fragment, 1e5));
	CHECK_NOTHROW(init({{"use_convergent_formulation", true}}, "adaptive"));
}
