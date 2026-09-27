////////////////////////////////////////////////////////////////////////////////
#include <catch2/catch_test_macros.hpp>

#include <polyfem/State.hpp>
#include <polyfem/Common.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/StringUtils.hpp>

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
	args["root_path"] = filename;
	args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
	// args["/output/log/level"_json_pointer] = "error";

	return args;
}

Eigen::MatrixXd run_sim(State &state, const json &args)
{
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
	args["/output/data/state"_json_pointer] = "restart_{:d}.hdf5";
	const auto full_sol = run_sim(state, args);

	args["/output/directory"_json_pointer] = restart_outdir.string();
	args["/input/data/state"_json_pointer] = (full_outdir / fmt::format("restart_{:d}.hdf5", restart_time_steps)).string();
	args["/time/t0"_json_pointer] = args["/time/dt"_json_pointer].get<double>() * restart_time_steps;
	args["time"]["time_steps"] = restart_time_steps;
	args["/output/data/state"_json_pointer] = "restart_{:d}.hdf5";
	args["/output/data/file_index_offset"_json_pointer] = restart_time_steps;
	const auto restart_sol = run_sim(state, args);

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
// time), the JSON must not depend on the launch directory, and the PVD must
// keep the original frame times.
#ifdef NDEBUG
TEST_CASE("restart from restart json", "[restart]")
#else
TEST_CASE("restart from restart json", "[.][restart]")
#endif
{
	const std::string scene_file = POLYFEM_DATA_DIR "/contact/examples/3D/unit-tests/2-cubes.json";
	constexpr int total_time_steps = 6;
	constexpr int restart_step = 3;
	constexpr double margin = 1e-3;

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
	args["/output/data/state"_json_pointer] = "state_{:d}.hdf5";
	args["/output/restart_json"_json_pointer] = "restart_{:d}.json";

	const std::filesystem::path params_file = outdir / "params.json";
	args["root_path"] = params_file.string();
	{
		std::ofstream file(params_file);
		file << args;
	}

	State full_state;
	const auto full_sol = run_sim(full_state, args);
	const double full_dt = full_state.args["time"]["dt"];

	json restart_args;
	REQUIRE(load_json((full_outdir / fmt::format("restart_{:d}.json", restart_step)).string(), restart_args));
	CHECK(std::filesystem::path(restart_args["root_path"].get<std::string>()).is_absolute());
	CHECK(restart_args["/time/dt"_json_pointer].get<double>() == full_dt);
	CHECK(restart_args["/time/time_steps"_json_pointer].get<int>() == total_time_steps - restart_step);
	CHECK(restart_args["/time/tend"_json_pointer].is_null());

	restart_args["/output/directory"_json_pointer] = restart_outdir.string();
	restart_args["/output/advanced/save_time_sequence"_json_pointer] = true;
	restart_args["/output/paraview/file_name"_json_pointer] = "sim.pvd";
	State restart_state;
	const auto restart_sol = run_sim(restart_state, restart_args);

	CHECK(restart_state.args["time"]["dt"].get<double>() == full_dt);
	CHECK(restart_state.args["time"]["time_steps"].get<int>() == total_time_steps - restart_step);
	CHECK(std::abs(restart_state.args["time"]["tend"].get<double>() - dt * total_time_steps) < 1e-12);

	CHECK(full_sol.rows() == restart_sol.rows());
	CAPTURE((full_sol - restart_sol).lpNorm<Eigen::Infinity>());
	CHECK(full_sol.isApprox(restart_sol, margin));

	// Frame i of the PVD is at i * dt, including the frames before the restart.
	std::ifstream pvd_file(restart_outdir / "sim.pvd");
	REQUIRE(pvd_file.is_open());
	const std::string pvd((std::istreambuf_iterator<char>(pvd_file)), std::istreambuf_iterator<char>());
	CAPTURE(pvd);
	for (int i = 0; i <= total_time_steps; ++i)
		CHECK(pvd.find(fmt::format("timestep=\"{:f}\" group=\"\" part=\"0\" file=\"step_{:d}.vtm\"", i * full_dt, i)) != std::string::npos);

	std::filesystem::remove_all(outdir);
}
