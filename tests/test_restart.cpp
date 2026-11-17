////////////////////////////////////////////////////////////////////////////////
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/Common.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/StringUtils.hpp>
#include <polyfem/io/MatrixIO.hpp>
#include <polyfem/io/Checkpoint.hpp>

#include <h5pp/h5pp.h>

#include <filesystem>
#include <fstream>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////

using namespace polyfem;
using namespace polyfem::utils;

bool load_json(const std::string &json_file, json &out);

json load_sim_json(const std::string filename, const int time_steps)
{
	json in_args;
	if (!load_json(filename, in_args))
	{
		spdlog::error("unable to open {} file", filename);
		FAIL();
	}

	json args = in_args;
	args["output"] = json({});
	args["output"]["advanced"]["save_time_sequence"] = false;
	{
		json t_args = args["time"];
		if (t_args.contains("tend") && t_args.contains("dt"))
		{
			t_args.erase("tend");
			t_args["time_steps"] = time_steps;
		}
		else if (t_args.contains("tend") && t_args.contains("time_steps"))
		{
			t_args["dt"] = t_args["tend"].get<double>() / t_args["time_steps"].get<int>();
			t_args["time_steps"] = time_steps;
			t_args.erase("tend");
		}
		else if (t_args.contains("dt") && t_args.contains("time_steps"))
		{
			t_args["time_steps"] = time_steps;
		}
		else
		{
			// Required to have two of tend, dt, time_steps
			FAIL();
		}
		args["time"] = t_args;
	}
	args["root_path"] = std::filesystem::path(filename).parent_path().string();
	args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
	// args["/output/log/level"_json_pointer] = "error";

	return args;
}

Eigen::MatrixXd run_sim(State &state, const json &args, const io::CheckpointReader *checkpoint = nullptr)
{
	if (checkpoint)
		state.init(args, *checkpoint, true);
	else
		state.init(args, true);
	state.set_max_threads(1);
	logger().set_level(spdlog::level::info);
	state.load_mesh();

	Eigen::MatrixXd sol;

	state.solve(sol);

	return sol;
}

#ifdef NDEBUG
TEST_CASE("restart", "[restart]")
#else
TEST_CASE("restart", "[.][restart]")
#endif
{
	const std::string scene_file = POLYFEM_DATA_DIR "/contact/examples/3D/unit-tests/2-cubes.json";
	constexpr int total_time_steps = 10;
	REQUIRE(total_time_steps % 2 == 0);
	constexpr int restart_time_steps = total_time_steps / 2;
	constexpr double margin = 1e-3;

	const std::filesystem::path outdir = std::filesystem::current_path() / "DELETE_ME_restart_test_output";
	const std::filesystem::path full_outdir = outdir / "full";
	const std::filesystem::path restart_outdir = outdir / "restart";

	json args = load_sim_json(scene_file, total_time_steps);

	State state;

	args["/output/directory"_json_pointer] = full_outdir.string();
	args["/output/checkpoint/path"_json_pointer] = "restart_{:d}.hdf5";
	const auto full_sol = run_sim(state, args);

	io::CheckpointReader checkpoint(full_outdir / fmt::format("restart_{:d}.hdf5", restart_time_steps));
	CHECK(checkpoint.metadata().remaining_steps == restart_time_steps);
	CHECK(checkpoint.metadata().output_index == restart_time_steps);
	CHECK(checkpoint.metadata().time == args["/time/dt"_json_pointer].get<double>() * restart_time_steps);
	json continuation = checkpoint.config();
	continuation["/output/directory"_json_pointer] = restart_outdir.string();
	const auto restart_sol = run_sim(state, continuation, &checkpoint);

	CHECK(full_sol.rows() == restart_sol.rows());
	CHECK(full_sol.cols() == restart_sol.cols());
	CAPTURE((full_sol - restart_sol).lpNorm<Eigen::Infinity>());
	CHECK(full_sol.isApprox(restart_sol, margin));

	// Verify that the file index offset works: restarted output files should
	// be numbered starting from restart_time_steps, not from 0.
	for (int t = restart_time_steps + 1; t <= total_time_steps; ++t)
	{
		const auto state_file = restart_outdir / fmt::format("restart_{:d}.hdf5", t);
		CHECK(std::filesystem::exists(state_file));
	}
	// Files numbered below restart_time_steps should NOT exist in the restart output dir.
	for (int t = 1; t < restart_time_steps; ++t)
	{
		const auto state_file = restart_outdir / fmt::format("restart_{:d}.hdf5", t);
		CHECK_FALSE(std::filesystem::exists(state_file));
	}

	std::filesystem::remove_all(outdir);
}

// Resume from the restart JSON the solver writes, as a user would. A
// tend + time_steps input must keep its dt (not re-derive it from the restart
// time), the JSON must not depend on the launch directory, the PVD must keep
// the original frame times, and the contact stiffness history (classic
// adaptive stiffness; semi-implicit trim, coefficient caches and the realized
// friction lag) must continue, so the resumed run matches to roundoff.
#ifdef NDEBUG
TEST_CASE("checkpoint preserves contact and AL continuation", "[restart]")
#else
TEST_CASE("checkpoint preserves contact and AL continuation", "[.][restart]")
#endif
{
	const std::string scene_file = POLYFEM_DATA_DIR "/contact/examples/3D/unit-tests/2-cubes.json";
	constexpr int total_time_steps = 6;
	constexpr int restart_step = 3;
	// Only the step times differ (t0 + k dt vs the original indexing, one ulp).
	constexpr double margin = 1e-10;
	const bool semi_implicit = GENERATE(false, true);
	CAPTURE(semi_implicit);

	const std::filesystem::path outdir = std::filesystem::current_path() / "DELETE_ME_restart_json_test_output";
	const std::filesystem::path full_outdir = outdir / "full";
	const std::filesystem::path restart_outdir = outdir / "restart";
	std::filesystem::create_directories(outdir);

	json args = load_sim_json(scene_file, total_time_steps);
	apply_common_params(args);
	for (auto &geometry : args["geometry"])
		geometry["mesh"] = resolve_path(geometry["mesh"], scene_file);
	const double dt = args["time"]["dt"];
	args["time"].erase("dt");
	args["time"]["tend"] = dt * total_time_steps;
	args["time"]["time_steps"] = total_time_steps;
	args["/output/directory"_json_pointer] = full_outdir.string();
	args["/output/checkpoint/path"_json_pointer] = "state_{:d}.hdf5";
	args["/solver/contact/barrier_stiffness"_json_pointer] = semi_implicit ? "semi_implicit" : "adaptive";
	// The fixed face jumps 0.2 up in step 1 and again in step 4, more than
	// an element height each, so those steps cannot snap onto it and run
	// AL passes: the multipliers of step 1 are carried to the restart step
	// and enter the first resumed step's AL stage.
	args["/boundary_conditions/dirichlet_boundary/value"_json_pointer] = json::array({0, "if(t - 0.15, 0.2, 2*t) + if(t - 0.35, 0.2, 0)", 0});
	if (semi_implicit)
		args["/contact/friction_coefficient"_json_pointer] = 0.3;

	const std::filesystem::path params_file = outdir / "params.json";
	args["root_path"] = outdir.string();
	{
		std::ofstream file(params_file);
		file << args;
	}

	State full_state;
	const auto full_sol = run_sim(full_state, args);
	const double full_dt = full_state.args["time"]["dt"];

	const auto relocated = outdir / "relocated";
	std::filesystem::create_directories(relocated);
	const auto copied_checkpoint = relocated / "resume.hdf5";
	std::filesystem::copy_file(full_outdir / fmt::format("state_{:d}.hdf5", restart_step), copied_checkpoint);
	io::CheckpointReader checkpoint(copied_checkpoint);
	json restart_args = checkpoint.config();

	CHECK(restart_args["/time/dt"_json_pointer].get<double>() == full_dt);
	CHECK(restart_args["/time/time_steps"_json_pointer].get<int>() == total_time_steps - restart_step);
	CHECK_FALSE(restart_args["time"].contains("tend"));
	{
		const std::string state = copied_checkpoint.string();
		Eigen::MatrixXd saved;
		CHECK(io::read_matrix(state, "/checkpoint/state/fork/contact/contact_scalars", saved));
		CHECK(io::read_matrix(state, "/checkpoint/state/fork/contact/contact_si_scalars", saved) == semi_implicit);
		CHECK(io::read_matrix(state, "/checkpoint/state/fork/friction/friction_scalars", saved) == semi_implicit);
		// The augmented-Lagrangian multipliers carry across steps; nonzero
		// here, or the resume below would not test restoring them.
		REQUIRE(checkpoint.read_long("/checkpoint/state/fork/al/count") >= 1);
		REQUIRE(io::read_matrix(state, "/checkpoint/state/fork/al/multipliers_0", saved));
		CHECK(saved.norm() > 0);
	}

	restart_args["/output/directory"_json_pointer] = restart_outdir.string();
	restart_args["/output/advanced/save_time_sequence"_json_pointer] = true;
	restart_args["/output/paraview/file_name"_json_pointer] = "sim.pvd";
	State restart_state;
	const auto restart_sol = run_sim(restart_state, restart_args, &checkpoint);

	CHECK(restart_state.args["time"]["dt"].get<double>() == full_dt);
	CHECK(restart_state.args["time"]["time_steps"].get<int>() == total_time_steps - restart_step);
	CHECK(std::abs(restart_state.args["time"]["tend"].get<double>() - dt * total_time_steps) < 1e-12);

	CHECK(full_sol.rows() == restart_sol.rows());
	CAPTURE((full_sol - restart_sol).lpNorm<Eigen::Infinity>());
	CHECK(full_sol.isApprox(restart_sol, margin));

	// The first resumed step continues the saved state exactly: its step
	// time is the same number, so every input of that solve is restored
	// history or rebuilt from it (the AL multipliers included).
	{
		const std::string name = fmt::format("state_{:d}.hdf5", restart_step + 1);
		Eigen::MatrixXd full_u, restart_u;
		REQUIRE(io::read_matrix((full_outdir / name).string(), "/checkpoint/state/solution", full_u));
		REQUIRE(io::read_matrix((restart_outdir / name).string(), "/checkpoint/state/solution", restart_u));
		CAPTURE((full_u - restart_u).lpNorm<Eigen::Infinity>());
		CHECK(full_u == restart_u);
	}

	// Frame i of the PVD is at i * dt, including the frames before the restart.
	std::string pvd;
	{
		// Closed before remove_all below: Windows cannot delete an open file.
		std::ifstream pvd_file(restart_outdir / "sim.pvd");
		REQUIRE(pvd_file.is_open());
		pvd.assign(std::istreambuf_iterator<char>(pvd_file), std::istreambuf_iterator<char>());
	}
	CAPTURE(pvd);
	for (int i = 0; i <= total_time_steps; ++i)
		CHECK(pvd.find(fmt::format("timestep=\"{:f}\" group=\"\" part=\"0\" file=\"step_{:d}.vtm\"", i * full_dt, i)) != std::string::npos);

	std::filesystem::remove_all(outdir);
}

// initial_trim_estimate_scope: run accepts the estimate once per run; the
// "already used" flag is restart state (layout version 2). Version 1 files did
// not store it, so a resumed run re-armed the estimate and seeded again
// (default-controller assessment, E5). A version 1 file still resumes, with
// the flag derived from the saved pending flag.
#ifdef NDEBUG
TEST_CASE("restart keeps the run-scope initial trim estimate", "[restart]")
#else
TEST_CASE("restart keeps the run-scope initial trim estimate", "[.][restart]")
#endif
{
	const std::string scene_file = POLYFEM_DATA_DIR "/contact/examples/3D/unit-tests/2-cubes.json";
	constexpr int total_time_steps = 6;
	constexpr int restart_step = 3;
	constexpr int seed_used_index = 23;

	const std::filesystem::path outdir = std::filesystem::current_path() / "DELETE_ME_restart_estimate_test_output";
	const std::filesystem::path full_outdir = outdir / "full";
	std::filesystem::remove_all(outdir);
	std::filesystem::create_directories(outdir);

	json args = load_sim_json(scene_file, total_time_steps);
	apply_common_params(args);
	for (auto &geometry : args["geometry"])
		geometry["mesh"] = resolve_path(geometry["mesh"], scene_file);
	args["/output/directory"_json_pointer] = full_outdir.string();
	args["/output/checkpoint/path"_json_pointer] = "state_{:d}.hdf5";
	args["/solver/contact/barrier_stiffness"_json_pointer] = "semi_implicit";
	args["/solver/contact/semi_implicit/initial_trim_estimate"_json_pointer] = true;
	args["/solver/contact/semi_implicit/initial_trim_estimate_scope"_json_pointer] = "run";
	// Accept the estimate at the first contact (the default 0.8 gate is
	// rarely passed on a Dirichlet-loaded scene), so the seed lies before
	// the restart step and would be taken again if re-armed.
	args["/solver/contact/semi_implicit/initial_trim_cosine"_json_pointer] = 0.01;
	args["/boundary_conditions/dirichlet_boundary/value"_json_pointer] = json::array({0, "if(t - 0.15, 0.2, 2*t) + if(t - 0.35, 0.2, 0)", 0});
	const std::filesystem::path params_file = outdir / "params.json";
	args["root_path"] = outdir.string();
	{
		std::ofstream file(params_file);
		file << args;
	}

	State full_state;
	const auto full_sol = run_sim(full_state, args);

	const std::string saved_state = (full_outdir / fmt::format("state_{:d}.hdf5", restart_step)).string();
	io::CheckpointReader checkpoint(saved_state);
	json restart_args = checkpoint.config();
	Eigen::MatrixXd scalars;
	REQUIRE(io::read_matrix(saved_state, "/checkpoint/state/fork/contact/contact_si_scalars", scalars));
	REQUIRE(scalars.size() == 24);
	CHECK(scalars(0) == 2);
	// The estimate was accepted before the restart step and is not pending.
	REQUIRE(scalars(seed_used_index) == 1);
	CHECK(scalars(13) == 0);

	const std::string step_name = fmt::format("state_{:d}.hdf5", restart_step + 1);
	Eigen::MatrixXd full_u;
	REQUIRE(io::read_matrix((full_outdir / step_name).string(), "/checkpoint/state/solution", full_u));

	// Resume from a copy of the saved state whose contact_si_scalars are
	// replaced by the given row.
	const auto resume = [&](const std::string &label, const Eigen::MatrixXd &row) {
		const std::filesystem::path dir = outdir / label;
		std::filesystem::create_directories(dir);
		const std::filesystem::path state = dir / "input_state.hdf5";
		std::filesystem::copy_file(saved_state, state, std::filesystem::copy_options::overwrite_existing);
		{
			h5pp::File file(state.string(), h5pp::FileAccess::READWRITE);
			REQUIRE(H5Ldelete(file.openFileHandle(), "/checkpoint/state/fork/contact/contact_si_scalars", H5P_DEFAULT) >= 0);
		}
		REQUIRE(io::write_matrix(state.string(), "/checkpoint/state/fork/contact/contact_si_scalars", row, false));
		json resume_args = restart_args;
		io::CheckpointReader copied(state);
		resume_args["/output/directory"_json_pointer] = (dir / "out").string();
		State resumed;
		const auto sol = run_sim(resumed, resume_args, &copied);
		Eigen::MatrixXd u;
		REQUIRE(io::read_matrix((dir / "out" / step_name).string(), "/checkpoint/state/solution", u));
		return std::make_pair(sol, u);
	};

	{
		// Version 2 as written.
		const auto [sol, u] = resume("v2", scalars);
		CAPTURE((full_u - u).lpNorm<Eigen::Infinity>());
		CHECK(full_u == u);
		CHECK(full_sol.isApprox(sol, 1e-10));
	}
	{
		// Version 1: 23 scalars, flag derived from the pending flag.
		Eigen::MatrixXd v1 = scalars.leftCols(23);
		v1(0) = 1;
		const auto [sol, u] = resume("v1", v1);
		CAPTURE((full_u - u).lpNorm<Eigen::Infinity>());
		CHECK(full_u == u);
		CHECK(full_sol.isApprox(sol, 1e-10));
	}
	{
		// The test discriminates: with the flag cleared (what version 1
		// readers did) the resumed run seeds again and leaves the full run.
		Eigen::MatrixXd cleared = scalars;
		cleared(seed_used_index) = 0;
		cleared(13) = 1;
		const auto [sol, u] = resume("cleared", cleared);
		CAPTURE((full_u - u).lpNorm<Eigen::Infinity>());
		CHECK(full_u != u);
	}

	std::filesystem::remove_all(outdir);
}

TEST_CASE("state file size stays near its payload", "[restart][matrix_io]")
{
	// h5pp's default square chunks (256 x 256) stored a 652,440 x 1 restart
	// vector in 1.34 GB; row-block chunks keep the file near the payload.
	const std::string path = (std::filesystem::temp_directory_path() / "polyfem_state_chunks.hdf5").string();
	const int n = 652440;
	const Eigen::MatrixXd u = Eigen::MatrixXd::Random(n, 1);
	const Eigen::MatrixXd v = Eigen::MatrixXd::Random(n, 2);
	const Eigen::MatrixXd small = Eigen::MatrixXd::Random(779, 6);
	REQUIRE(io::write_matrix(path, "u", u, /*replace=*/true));
	REQUIRE(io::write_matrix(path, "v", v, /*replace=*/false));
	REQUIRE(io::write_matrix(path, "small", small, /*replace=*/false));

	const double payload = double(u.size() + v.size() + small.size()) * sizeof(double);
	CHECK(double(std::filesystem::file_size(path)) < 1.1 * payload + (1 << 20));
	{
		// Whole-row chunks of about 1 MiB, no filter (as before the fix).
		h5pp::File file(path, h5pp::FileAccess::READONLY);
		const auto u_info = file.getDatasetInfo("u");
		const auto v_info = file.getDatasetInfo("v");
		REQUIRE(u_info.dsetChunk.has_value());
		CHECK(u_info.dsetChunk.value() == std::vector<hsize_t>{131072, 1});
		REQUIRE(v_info.dsetChunk.has_value());
		CHECK(v_info.dsetChunk.value() == std::vector<hsize_t>{65536, 2});
		CHECK(H5Pget_nfilters(u_info.h5DsetCreate.value()) == 0);
		CHECK(H5Pget_nfilters(v_info.h5DsetCreate.value()) == 0);
	}

	Eigen::MatrixXd u_read, v_read, small_read;
	REQUIRE(io::read_matrix(path, "u", u_read));
	REQUIRE(io::read_matrix(path, "v", v_read));
	REQUIRE(io::read_matrix(path, "small", small_read));
	CHECK(u_read == u);
	CHECK(v_read == v);
	CHECK(small_read == small);
	std::filesystem::remove(path);
}

TEST_CASE("Retired restart JSON directs users to checkpoints", "[restart][input_validation]")
{
	const bool strict = GENERATE(false, true);
	json args;
	args["/output/restart_json"_json_pointer] = "restart_{:d}.json";
	State state;
	CHECK_THROWS_WITH(state.init(args, strict), Catch::Matchers::ContainsSubstring("/output/checkpoint/path"));
}

TEST_CASE("ESP friction checkpoint combination is refused before loading geometry", "[restart][checkpoint][pf4][input_validation]")
{
	const bool strict = GENERATE(false, true);
	const json base = {{"geometry", json::array({{{"mesh", "not-loaded.msh"}}})},
					   {"materials", {{"type", "NeoHookean"}, {"E", 1e6}, {"nu", .3}, {"rho", 1000.0}}}};
	for (const std::string direction : {"input", "output"})
	{
		json args = base;
		args["contact"] = {{"enabled", true}, {"use_esp_formulation", true}, {"friction_coefficient", 0.3}};
		args[direction]["checkpoint"]["path"] = "not-created.h5";
		State state;
		CHECK_THROWS_WITH(state.init(args, strict), Catch::Matchers::ContainsSubstring("quadrature contributions require additional history identities"));
	}
	json args = base;
	args["contact"] = {{"enabled", true}, {"use_esp_formulation", true}, {"friction_coefficient", 0.3}};
	State ordinary;
	CHECK_NOTHROW(ordinary.init(args, strict));
	args["contact"]["friction_coefficient"] = 0.0;
	args["output"]["checkpoint"]["path"] = "not-created.h5";
	State frictionless;
	CHECK_NOTHROW(frictionless.init(args, strict));
}
