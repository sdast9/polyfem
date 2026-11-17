#include "VtuTestUtils.hpp"
////////////////////////////////////////////////////////////////////////////////
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <polyfem/State.hpp>
#include <polyfem/Common.hpp>
#include <polyfem/io/OutStatsData.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/varforms/VarForm.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
////////////////////////////////////////////////////////////////////////////////

using namespace polyfem;
using namespace polyfem::assembler;
using namespace polyfem::utils;

bool load_json(const std::string &json_file, json &out);

TEST_CASE("output statistics are initialized", "[output]")
{
	io::OutRuntimeData timings;
	CHECK(timings.total_time() == 0);
	CHECK(timings.loading_mesh_time == 0);
	CHECK(timings.assigning_rhs_time == 0);
	timings.building_basis_time = 1;
	timings.loading_mesh_time = 2;
	timings.assigning_rhs_time = 3;
	timings.solving_time = 4;
	CHECK(timings.total_time() == 10);

	io::OutStatsData stats;
	CHECK(stats.spectrum.isZero());
	CHECK(stats.mesh_size == 0);
	CHECK(stats.min_edge_length == 0);
	CHECK(stats.nn_zero == 0);
	CHECK(stats.simplex_count == 0);
}

#ifdef NDEBUG
TEST_CASE("full sim", "[full_sim]")
#else
TEST_CASE("full sim", "[.][full_sim]")
#endif
{
	const std::string scene = GENERATE("2D/unit-tests/5-squares", "3D/unit-tests/5-cubes-fast", "3D/unit-tests/edge-edge-parallel");
	const std::string scene_file = fmt::format("{}/contact/examples/{}.json", POLYFEM_DATA_DIR, scene);

	json args;
	if (!load_json(scene_file, args))
	{
		spdlog::error("unable to open {} file", scene_file);
		FAIL();
	}

	args["root_path"] = scene_file;
	const std::filesystem::path outdir = std::filesystem::current_path() / "DELETE_ME_full_sim_test_output";
	args["/output/directory"_json_pointer] = outdir.string();
	args["/output/paraview"_json_pointer] = R"({
		"file_name": "sim.pvd",
		"volume": true,
		"surface": true,
		"wireframe": true,
		"points": true,
		"vismesh_rel_area": 1e-05,
		"options": {
			"material": true,
			"body_ids": true,
			"contact_forces": true,
			"friction_forces": true,
			"velocity": true,
			"acceleration": true
		}
	})"_json;
	args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
	args["/output/log/level"_json_pointer] = "warning";

	State state;

	state.init(args, true);
	state.load_mesh();

	Eigen::MatrixXd sol;

	state.solve(sol);
	state.variational_formulation->compute_errors(sol);

	CHECK(std::filesystem::exists(outdir));
	CHECK(std::filesystem::exists(outdir / "sim.pvd"));

	int n_volume_files = 0;
	for (const auto &entry : std::filesystem::directory_iterator(outdir))
	{
		if (!std::regex_match(entry.path().filename().string(), std::regex(R"(step_\d+\.vtu)")))
			continue;
		std::ifstream file(entry.path());
		const std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		CAPTURE(entry.path().filename().string());
		CHECK(contents.find("Name=\"solution\"") != std::string::npos);
		++n_volume_files;
	}
	CHECK(n_volume_files > 0);

	std::filesystem::remove_all(outdir);
}

TEST_CASE("PF4 2D volume obstacles export triangle VTU cells", "[pf4][output]")
{
	const auto dir = std::filesystem::current_path() / "DELETE_ME_pf4_obstacle_output";
	std::filesystem::create_directories(dir);
	const auto mesh = dir / "square.msh";
	{
		std::ofstream file(mesh);
		file << "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n4\n1 0 0 0\n2 1 0 0\n3 1 1 0\n4 0 1 0\n"
				"$EndNodes\n$Elements\n2\n1 2 2 0 1 1 2 3\n2 2 2 0 1 1 3 4\n$EndElements\n";
	}
	const auto obstacle_mesh = dir / "square.obj";
	{
		std::ofstream file(obstacle_mesh);
		file << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3\nf 1 3 4\n";
	}
	json args = {
		{"geometry", json::array({{{"mesh", mesh.string()}, {"surface_selection", 1}},
								  {{"mesh", obstacle_mesh.string()}, {"is_obstacle", true}, {"transformation", {{"translation", {3, 0}}}}}})},
		{"materials", {{"type", "NeoHookean"}, {"E", 1e6}, {"nu", .3}}},
		{"boundary_conditions", {{"dirichlet_boundary", json::array({{{"id", "all"}, {"value", {0, 0}}}})}}}};
	args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
	args["/output/directory"_json_pointer] = dir.string();
	args["/output/paraview/file_name"_json_pointer] = "result.vtu";
	args["/output/paraview/options/use_hdf5"_json_pointer] = false;
	args["/output/paraview/options/obstacle_volume"_json_pointer] = true;
	State state;
	state.init(args, true);
	state.load_mesh();
	Eigen::MatrixXd solution;
	state.solve(solution);
	state.variational_formulation->export_data(solution);
	std::ifstream file(dir / "result.vtu");
	REQUIRE(file.is_open());
	const std::string vtu((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	const auto name = vtu.find("Name=\"types\"");
	REQUIRE(name != std::string::npos);
	const auto begin = vtu.find('>', name);
	const auto end = vtu.find("</DataArray>", begin);
	const auto bytes = test::base64_decode(vtu.substr(begin + 1, end - begin - 1));
	REQUIRE(bytes.size() > 8);
	std::uint64_t count;
	std::memcpy(&count, bytes.data(), 8);
	REQUIRE(count == bytes.size() - 8);
	REQUIRE(count >= 4); // two simulated and two obstacle triangles
	for (size_t i = 8; i < bytes.size(); ++i)
		CHECK(bytes[i] == 5); // VTK_TRIANGLE, never VTK_TETRA (10)
	std::filesystem::remove_all(dir);
}
