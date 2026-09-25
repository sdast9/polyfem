// RB-12: run identity. The run manifest must identify what ran on what:
// the sources the library was built from (compiled in, with the declared
// pins next to the effective checkouts), the executable's own hash, the
// effective input with its hash and every referenced file's hash, the model
// the contact form implements, the step history and the completion status.
// These tests pin the hash primitive, the compiled-in build identity, the
// canonical input hash, the referenced-file walk, the manifest of a small
// run and the opt-in/default switches.
#include <polyfem/State.hpp>
#include <polyfem/io/BuildInfo.hpp>
#include <polyfem/io/DiagnosticSchemas.hpp>
#include <polyfem/io/RunManifest.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Sha256.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace polyfem;
using Catch::Matchers::ContainsSubstring;

namespace
{
	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	// [0,2]x[0,1]x[0,1] in 2x1x1 cells, six Kuhn tetrahedra per cell (MEDIT)
	std::string write_beam(const std::filesystem::path &dir)
	{
		const int nx = 2, ny = 1, nz = 1;
		std::vector<std::array<double, 3>> V;
		for (int i = 0; i <= nx; ++i)
			for (int j = 0; j <= ny; ++j)
				for (int k = 0; k <= nz; ++k)
					V.push_back({{double(i), double(j), double(k)}});
		const auto vid = [&](int i, int j, int k) { return (i * (ny + 1) + j) * (nz + 1) + k; };
		const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
		std::vector<std::array<int, 4>> T;
		for (int i = 0; i < nx; ++i)
			for (int j = 0; j < ny; ++j)
				for (int k = 0; k < nz; ++k)
					for (int p = 0; p < 6; ++p)
					{
						int cur[3] = {0, 0, 0};
						std::array<int, 4> t{{vid(i, j, k), 0, 0, 0}};
						for (int s = 0; s < 3; ++s)
						{
							cur[perms[p][s]] = 1;
							t[s + 1] = vid(i + cur[0], j + cur[1], k + cur[2]);
						}
						const auto &a = V[t[0]];
						const auto &b = V[t[1]];
						const auto &c = V[t[2]];
						const auto &d = V[t[3]];
						const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
						const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
						const double ad[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
						const double vol = (ab[1] * ac[2] - ab[2] * ac[1]) * ad[0] + (ab[2] * ac[0] - ab[0] * ac[2]) * ad[1] + (ab[0] * ac[1] - ab[1] * ac[0]) * ad[2];
						if (vol < 0)
							std::swap(t[2], t[3]);
						T.push_back(t);
					}
		const auto path = dir / "beam.mesh";
		std::ofstream out(path);
		out << "MeshVersionFormatted 2\nDimension 3\nVertices\n"
			<< V.size() << "\n";
		for (const auto &v : V)
			out << v[0] << " " << v[1] << " " << v[2] << " 0\n";
		out << "Tetrahedra\n"
			<< T.size() << "\n";
		for (const auto &t : T)
			out << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << " " << t[3] + 1 << " 0\n";
		out << "End\n";
		return path.string();
	}

	json beam_args(const std::string &mesh_path, const std::filesystem::path &out_dir)
	{
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", mesh_path},
										 {"surface_selection", json::array({{{"id", 1}, {"axis", "-x"}, {"position", 1e-6}}})}}});
		args["materials"] = {{"type", "NeoHookean"}, {"E", 1e5}, {"nu", 0.3}, {"rho", 1000.0}};
		args["boundary_conditions"] = {
			{"dirichlet_boundary", json::array({{{"id", 1}, {"value", json::array({0, 0, 0})}}})},
			{"rhs", json::array({0.0, 9.81, 0.0})}};
		args["time"] = {{"dt", 0.5}, {"time_steps", 2}};
		args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
		args["/solver/max_threads"_json_pointer] = 1;
		args["/output/directory"_json_pointer] = out_dir.string();
		args["/output/log/level"_json_pointer] = "error";
		args["/output/paraview/file_name"_json_pointer] = "";
		return args;
	}

	json read_json(const std::filesystem::path &path)
	{
		std::ifstream file(path);
		REQUIRE(file.is_open());
		json j;
		file >> j;
		return j;
	}

	bool is_hex(const std::string &s, const size_t length)
	{
		return s.size() == length && s.find_first_not_of("0123456789abcdef") == std::string::npos;
	}
} // namespace

TEST_CASE("SHA-256 known answers and streaming", "[run_manifest][sha256]")
{
	using utils::Sha256;
	CHECK(Sha256::of("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
	CHECK(Sha256::of("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
	CHECK(Sha256::of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
	const std::string million(1000000, 'a');
	CHECK(Sha256::of(million) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");

	// Streaming in odd chunk sizes crosses every block boundary alignment.
	Sha256 streamed;
	for (size_t i = 0; i < million.size(); i += 61)
		streamed.update(million.data() + i, std::min<size_t>(61, million.size() - i));
	CHECK(streamed.hex_digest() == Sha256::of(million));

	// A 55/56/64-byte message exercises the padding branch on both sides.
	for (const size_t n : {55, 56, 63, 64, 65, 119, 120})
	{
		const std::string message(n, 'x');
		Sha256 a;
		a.update(message.data(), n / 2);
		a.update(message.data() + n / 2, n - n / 2);
		CHECK(a.hex_digest() == Sha256::of(message));
	}

	const auto dir = scratch_dir("polyfem-sha256");
	const auto path = dir / "blob.bin";
	{
		std::ofstream out(path, std::ios::binary);
		out << million;
	}
	CHECK(Sha256::of_file(path.string()) == Sha256::of(million));
	CHECK_THROWS_WITH(Sha256::of_file((dir / "missing").string()), ContainsSubstring("missing"));
	std::filesystem::remove_all(dir);
}

TEST_CASE("The compiled-in build identity names the effective sources and the declared pins", "[run_manifest][build_info]")
{
	const json &info = io::build_info();
	CHECK(info["schema"] == io::schemas::BUILD_INFO);
	CHECK(info["version"] == io::schemas::BUILD_INFO_VERSION);
	REQUIRE(info.contains("sources"));
	for (const char *name : {"polyfem", "ipc_toolkit", "polysolve"})
	{
		INFO(name);
		const json &source = info["sources"][name];
		REQUIRE(source.contains("state"));
		REQUIRE(source.contains("path"));
		if (source["state"] == "git")
		{
			CHECK(is_hex(source["commit"].get<std::string>(), 40));
			CHECK(source["dirty"].is_boolean());
			CHECK(source["tracked_changes"].is_number_integer());
			CHECK(source["untracked_count"].is_number_integer());
			// dirty <=> a patch hash exists
			CHECK(source["patch_sha256"].is_null() == !source["dirty"].get<bool>());
			if (!source["patch_sha256"].is_null())
				CHECK(is_hex(source["patch_sha256"].get<std::string>(), 64));
		}
		else
			CHECK(source.contains("unavailable_reason"));
	}
	// The dependencies' declared pins come from the recipes, whatever the
	// effective source is; the comparison is explicit, never inferred.
	for (const char *name : {"ipc_toolkit", "polysolve"})
	{
		INFO(name);
		const json &source = info["sources"][name];
		REQUIRE(source["declared_pin"].is_string());
		CHECK(is_hex(source["declared_pin"].get<std::string>(), 40));
		CHECK(source["declared_repository"].get<std::string>().find("/") != std::string::npos);
		if (source["state"] == "git")
		{
			REQUIRE(source["matches_declared_pin"].is_boolean());
			CHECK(source["matches_declared_pin"].get<bool>() == (source["commit"] == source["declared_pin"]));
		}
	}
	CHECK(info["sources"]["polyfem"]["declared_pin"].is_null());
	const json &build = info["build"];
	CHECK_FALSE(build["compiler"].get<std::string>().empty());
	CHECK_FALSE(build["configuration"].get<std::string>().empty());
	const std::string threading = build["threading"];
	CHECK((threading == "TBB" || threading == "CPP" || threading == "NONE"));
	CHECK(build["options"].is_object());
	CHECK(build["options"].contains("POLYFEM_WITH_OPTIMIZATION"));
}

TEST_CASE("The canonical input hash ignores key order and sees every value", "[run_manifest][hash]")
{
	const json a = json::parse(R"({"b": 2, "a": {"y": [1, 2.5, "s"], "x": null}})");
	const json b = json::parse(R"({"a": {"x": null, "y": [1, 2.5, "s"]}, "b": 2})");
	CHECK(io::RunManifest::canonical_sha256(a) == io::RunManifest::canonical_sha256(b));
	json c = a;
	c["a"]["y"][1] = 2.5000001;
	CHECK(io::RunManifest::canonical_sha256(a) != io::RunManifest::canonical_sha256(c));
	CHECK(is_hex(io::RunManifest::canonical_sha256(a), 64));
}

TEST_CASE("Referenced files are the input's strings that resolve to existing files, outputs excluded", "[run_manifest][referenced_files]")
{
	const auto dir = scratch_dir("polyfem-manifest-files");
	const std::string mesh = write_beam(dir);
	{
		std::ofstream out(dir / "orders.txt");
		out << "1\n";
	}
	{
		std::ofstream out(dir / "sim.pvd"); // an output name that happens to exist
		out << "x\n";
	}
	json args = json::object();
	args["root_path"] = (dir / "params.json").string();
	args["geometry"] = json::array({{{"mesh", "beam.mesh"}}, {{"mesh", "absent.mesh"}}});
	args["space"] = {{"discr_order", "orders.txt"}};
	args["materials"] = {{"type", "NeoHookean"}, {"E", "orders.txt"}}; // a per-element value file
	args["output"] = {{"paraview", {{"file_name", "sim.pvd"}}}, {"directory", dir.string()}};
	args["notes"] = "multi\nline strings are never paths";

	const json files = io::RunManifest::referenced_files(args, args["root_path"]);
	REQUIRE(files.is_array());
	std::vector<std::string> pointers;
	for (const auto &entry : files)
	{
		pointers.push_back(entry["pointer"]);
		CHECK(is_hex(entry["sha256"].get<std::string>(), 64));
		CHECK(entry["size_bytes"].get<size_t>() > 0);
		CHECK(std::filesystem::path(entry["path"].get<std::string>()).is_absolute());
	}
	CHECK(pointers == std::vector<std::string>{"/geometry/0/mesh", "/materials/E", "/space/discr_order"});
	CHECK(files[0]["sha256"] == utils::Sha256::of_file(mesh));
	CHECK(files[0]["value"] == "beam.mesh");
	std::filesystem::remove_all(dir);
}

TEST_CASE("The barrier form describes the model it implements", "[run_manifest][model]")
{
	Eigen::MatrixXd vertices(3, 2);
	vertices << -1, 0, 1, 0, 0, .2;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	const ipc::CollisionMesh mesh(vertices, edges);
	{
		solver::BarrierContactForm fixed(mesh, .1, 1., false, false, false, false, false, false,
										 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, solver::BarrierStiffnessMode::Fixed);
		const json model = fixed.model_description();
		CHECK(model["form"] == "barrier-contact");
		CHECK(model["stiffness_mode"] == "fixed");
		CHECK(model["dhat"] == .1);
		CHECK(model["coefficient_law"].is_string());
		CHECK(model.contains("gap_convention"));
		CHECK(model.contains("model_selection_status"));
	}
	{
		const json opts = {{"coefficient_identity", "stencil"}, {"force_continuation", false}, {"friction_lag", "follow_stiffness"}, {"trim_upper", .8}};
		solver::BarrierContactForm semi(mesh, .1, 1., false, false, false, true, false, false,
										ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, solver::BarrierStiffnessMode::SemiImplicit, opts);
		const json model = semi.model_description();
		CHECK(model["stiffness_mode"] == "semi_implicit");
		const json &law = model["coefficient_law"];
		REQUIRE(law.is_object());
		CHECK(law["coefficient_identity"] == "stencil");
		CHECK(law["force_continuation"] == false);
		CHECK(law["friction_lag"] == "follow_stiffness");
		CHECK(law["controller"]["trim_upper"] == .8);
		CHECK_THAT(law["version"].get<std::string>(), ContainsSubstring("RB-18"));
		CHECK_THAT(law["curvature_fallbacks"].get<std::string>(), ContainsSubstring("curvature_global_fallback_count"));
	}
}

TEST_CASE("A run's manifest records identity, input hashes, steps and completion", "[run_manifest][run]")
{
	const auto dir = scratch_dir("polyfem-manifest-run");
	const std::string mesh = write_beam(dir);
	json args = beam_args(mesh, dir);
	args["/output/manifest"_json_pointer] = "manifest.json";
	args["root_path"] = (dir / "params.json").string();

	State state;
	state.init(args, true);
	REQUIRE(state.run_manifest != nullptr);
	CHECK(std::filesystem::weakly_canonical(state.run_manifest->path()) == std::filesystem::weakly_canonical(dir / "manifest.json"));
	CHECK(io::RunManifest::active() == state.run_manifest);
	REQUIRE(std::filesystem::is_regular_file(dir / "manifest.json"));
	{
		// Written at init, before the mesh is read: identity without steps.
		const json m = read_json(dir / "manifest.json");
		CHECK(m["schema"] == io::schemas::RUN_MANIFEST);
		CHECK(m["version"] == io::schemas::RUN_MANIFEST_VERSION);
		CHECK(m["run_id"] == state.run_manifest->run_id());
		CHECK(m["completion"]["status"] == "running");
		CHECK(m["steps"].empty());
		CHECK(m["solver"]["model"]["value"].is_null());
		CHECK(m["build"] == io::build_info());
		{
			// RB-24: the CCD root finder's version; 1.1.0 made the bucket
			// depth-first search (bounded memory per capped query) the default.
			REQUIRE(m["libraries"]["tight_inclusion"].is_string());
			int major = 0, minor = 0;
			REQUIRE(std::sscanf(m["libraries"]["tight_inclusion"].get<std::string>().c_str(), "%d.%d", &major, &minor) == 2);
			CHECK((major > 1 || (major == 1 && minor >= 1)));
		}
		CHECK(m["input"]["effective_sha256"] == io::RunManifest::canonical_sha256(state.args));
		{
			json portable = state.args;
			portable.erase("root_path");
			portable["output"].erase("directory");
			CHECK(m["input"]["effective_sha256_without_paths"] == io::RunManifest::canonical_sha256(portable));
			CHECK(m["input"]["effective_sha256_without_paths"] != m["input"]["effective_sha256"]);
		}
		CHECK(m["input"]["effective"] == state.args);
		CHECK(m["input"]["effective"]["output"]["manifest"] == "manifest.json");
		REQUIRE(m["input"]["referenced_files"].size() == 1);
		CHECK(m["input"]["referenced_files"][0]["pointer"] == "/geometry/0/mesh");
		CHECK(m["input"]["referenced_files"][0]["sha256"] == utils::Sha256::of_file(mesh));
		CHECK(m["input"]["file"]["value"].is_null()); // params.json was never written
		CHECK(m["input"]["characteristic_force_density"]["effective"] == 10000.);
		CHECK(m["process"]["threads"]["effective"] == 1);
		CHECK(m["process"]["threads"]["backend"].is_string());
		CHECK(m["process"]["executable"]["sha256"].is_string());
		CHECK(is_hex(m["process"]["executable"]["sha256"].get<std::string>(), 64));
		CHECK(m["process"]["platform"].contains("system"));
		CHECK(m["solver"]["linear"]["solver"] == "Eigen::SimplicialLDLT");
		CHECK(m["solver"]["contact"]["mode"] == "disabled");
		CHECK(m["diagnostics"]["schemas"][io::schemas::PHYSICAL_DIAGNOSTICS] == io::schemas::PHYSICAL_DIAGNOSTICS_VERSION);
		CHECK(m["producer"].is_null());
	}

	state.load_mesh();
	Eigen::MatrixXd sol;
	state.solve(sol);
	REQUIRE(sol.size() > 0);
	{
		const json m = read_json(dir / "manifest.json");
		CHECK(m["completion"]["status"] == "running"); // a library user finalizes explicitly
		CHECK(m["solver"]["model"]["contact"] == "disabled");
		REQUIRE(m["steps"].size() == 2);
		for (int i = 0; i < 2; ++i)
		{
			const json &step = m["steps"][i];
			CHECK(step["step"] == i + 1);
			CHECK(step["time"] == 0.5 * (i + 1));
			CHECK(step["outcome"] == "accepted");
			CHECK(step["phase"] == "returned_endpoint");
			CHECK(step["termination"]["outcome"] == "converged");
			CHECK(step["termination"]["iterations"].is_number_integer());
			CHECK(step["wall_seconds"].get<double>() > 0);
			CHECK(step["subsolves"].is_array());
			CHECK(step["error"].is_null());
			CHECK(step["contact"].is_null());
		}
	}

	state.run_manifest->finalize("completed", -1, "");
	CHECK(state.run_manifest->finalized());
	CHECK(io::RunManifest::active() == nullptr);
	state.run_manifest->finalize("failed", 1, "ignored: already finalized");
	{
		const json m = read_json(dir / "manifest.json");
		CHECK(m["completion"]["status"] == "completed");
		CHECK(m["completion"]["exit_status"].is_null());
		CHECK(m["completion"]["steps_recorded"] == 2);
		CHECK(m["completion"]["wall_seconds"].get<double>() > 0);
		CHECK(m["completion"]["finished_at"].is_string());
	}
	std::filesystem::remove_all(dir);
}

TEST_CASE("The manifest is opt-in for the library and defaulted for the executable", "[run_manifest][switch]")
{
	const auto dir = scratch_dir("polyfem-manifest-switch");
	const std::string mesh = write_beam(dir);
	SECTION("library default: none")
	{
		State state;
		state.init(beam_args(mesh, dir), true);
		CHECK(state.run_manifest == nullptr);
		CHECK_FALSE(std::filesystem::exists(dir / "run-manifest.json"));
	}
	SECTION("executable default: run-manifest.json in the output directory")
	{
		State state;
		state.default_manifest = "run-manifest.json";
		state.init(beam_args(mesh, dir), true);
		REQUIRE(state.run_manifest != nullptr);
		CHECK(std::filesystem::is_regular_file(dir / "run-manifest.json"));
		state.run_manifest->finalize("completed", -1, "");
	}
	SECTION("an explicit empty name disables it even for the executable")
	{
		json args = beam_args(mesh, dir);
		args["/output/manifest"_json_pointer] = "";
		State state;
		state.default_manifest = "run-manifest.json";
		state.init(args, true);
		CHECK(state.run_manifest == nullptr);
	}
	SECTION("provenance is copied only when something is filled in")
	{
		json args = beam_args(mesh, dir);
		args["/output/manifest"_json_pointer] = "with-producer.json";
		args["provenance"] = {{"producer", "unit test"}, {"asset_sha256", "abc"}};
		State state;
		state.init(args, true);
		REQUIRE(state.run_manifest != nullptr);
		const json m = read_json(dir / "with-producer.json");
		CHECK(m["producer"]["producer"] == "unit test");
		CHECK(m["producer"]["asset_sha256"] == "abc");
		state.run_manifest->finalize("completed", -1, "");
	}
	std::filesystem::remove_all(dir);
}
