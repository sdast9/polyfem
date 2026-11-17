#include <filesystem>

#include <CLI/CLI.hpp>

#include <polyfem/State.hpp>
#ifdef POLYFEM_WITH_OPTIMIZATION
#include <polyfem/optimization/OptState.hpp>
#endif

#include <polyfem/io/BuildInfo.hpp>
#include <polyfem/io/RunManifest.hpp>
#include <polyfem/utils/ExitStatus.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/varforms/VarForm.hpp>
#include <polyfem/io/ResourceIO.hpp>
#include <polyfem/io/InputLoader.hpp>
#include <polyfem/io/Checkpoint.hpp>

#include <ipc/broad_phase/broad_phase.hpp>
#include <ipc/utils/logger.hpp>

#include <iostream>
#include <new>

using namespace polyfem;
using namespace solver;

bool has_arg(const CLI::App &command_line, const std::string &value)
{
	const auto *opt = command_line.get_option_no_throw(value.size() == 1 ? ("-" + value) : ("--" + value));
	if (!opt)
		return false;

	return opt->count() > 0;
}

int forward_simulation(const CLI::App &command_line,
					   const io::ResourceIO &resources,
					   const std::string &source_file,
					   const std::string &output_dir,
					   const unsigned max_threads,
					   const bool is_strict,
					   const bool fallback_solver,
					   const spdlog::level::level_enum &log_level,
					   json &in_args);

int forward_simulation(const CLI::App &command_line,
					   const io::CheckpointReader &checkpoint,
					   const std::string &source_file,
					   const std::string &output_dir,
					   unsigned max_threads,
					   bool is_strict,
					   bool fallback_solver,
					   const spdlog::level::level_enum &log_level,
					   json &in_args);

#ifdef POLYFEM_WITH_OPTIMIZATION
int optimization_simulation(const CLI::App &command_line,
							const io::ResourceIO &resources,
							const unsigned max_threads,
							const bool is_strict,
							const spdlog::level::level_enum &log_level,
							json &opt_args);
#endif

int run_forward_simulation(State &state);

int forward_simulation_with_varform_state(const io::ResourceIO &resources,
										  const std::string &source_file,
										  json &in_args,
										  const bool is_strict)
{
	State state;
	state.default_manifest = "run-manifest.json";
	state.manifest_input_file = source_file;
	state.init(in_args, resources, is_strict);
	return run_forward_simulation(state);
}

int forward_simulation_with_varform_state(const io::CheckpointReader &checkpoint,
										  const std::string &source_file,
										  const json &in_args,
										  const bool is_strict)
{
	State state;
	state.default_manifest = "run-manifest.json";
	state.manifest_input_file = source_file;
	state.init(in_args, checkpoint, is_strict);
	return run_forward_simulation(state);
}

int run_forward_simulation(State &state)
{
	state.load_mesh();

	Eigen::MatrixXd sol;
	state.solve(sol);

	state.variational_formulation->compute_errors(sol);

	io::OutRuntimeData timings = state.variational_formulation->output_timings();
	logger().info("total time: {}s", timings.total_time());

	state.variational_formulation->save_json(sol);
	state.variational_formulation->export_data(sol);

	if (state.run_manifest)
		state.run_manifest->finalize("completed", ExitStatus::Success, "");
	return EXIT_SUCCESS;
}

// Argument parsing and simulation; main below reports named failures.
static int run(int argc, char **argv)

{
	using namespace polyfem;

	CLI::App command_line{"polyfem"};
	io::RunManifest::set_command_line(argc, argv);

	command_line.ignore_case();
	command_line.ignore_underscore();

	// Eigen::setNbThreads(1);
	unsigned max_threads = std::numeric_limits<unsigned>::max();
	command_line.add_option("--max_threads", max_threads, "Maximum number of threads");

	// RB-12: the build identity compiled into this executable (sources,
	// pins, compiler, configuration), as JSON on stdout; nothing is run.
	bool print_build_info = false;
	command_line.add_flag("--build_info", print_build_info, "Print the build identity (polyfem.build-info JSON) and exit");

	auto input = command_line.add_option_group("input");

	std::string json_file = "";
	input->add_option("-j,--json", json_file, "Simulation JSON file")->check(CLI::ExistingFile);

	std::string yaml_file = "";
	input->add_option("-y,--yaml", yaml_file, "Simulation YAML file")->check(CLI::ExistingFile);

	std::string hdf5_file = "";
	input->add_option("--hdf5", hdf5_file, "Simulation HDF5 file")->check(CLI::ExistingFile);

	std::string checkpoint_file = "";
	auto *checkpoint_option = input->add_option("--checkpoint", checkpoint_file, "Versioned PolyFEM checkpoint")->check(CLI::ExistingFile);
	bool checkpoint_reorder = false;
	command_line.add_flag(
					"--checkpoint-reorder", checkpoint_reorder,
					"Reorder checkpoint state from its saved input-node ordering")
		->needs(checkpoint_option);

	input->require_option(0, 1);

	std::string output_dir = "";
	command_line.add_option("-o,--output_dir", output_dir, "Directory for output files")->check(CLI::ExistingDirectory | CLI::NonexistentPath);

	bool is_strict = true;
	command_line.add_flag("-s,--strict_validation,!--ns,!--no_strict_validation", is_strict, "Disables strict validation of input JSON");

	bool fallback_solver = false;
	command_line.add_flag("--enable_overwrite_solver", fallback_solver, "If solver in input is not present, falls back to default.");

	const std::vector<std::pair<std::string, spdlog::level::level_enum>>
		SPDLOG_LEVEL_NAMES_TO_LEVELS = {
			{"trace", spdlog::level::trace},
			{"debug", spdlog::level::debug},
			{"info", spdlog::level::info},
			{"warning", spdlog::level::warn},
			{"error", spdlog::level::err},
			{"critical", spdlog::level::critical},
			{"off", spdlog::level::off}};
	spdlog::level::level_enum log_level = spdlog::level::debug;
	command_line.add_option("--log_level", log_level, "Log level")
		->transform(CLI::CheckedTransformer(SPDLOG_LEVEL_NAMES_TO_LEVELS, CLI::ignore_case));

	CLI11_PARSE(command_line, argc, argv);

	if (print_build_info)
	{
		std::cout << io::build_info().dump(2) << std::endl;
		return EXIT_SUCCESS;
	}

	json in_args = json({});
	if (!json_file.empty() || !yaml_file.empty())
	{
		const io::LoadedInput loaded = !json_file.empty()
										   ? io::load_json_input(json_file)
										   : io::load_yaml_input(yaml_file);
		in_args = loaded.config;

		if (in_args.contains("states"))
		{
#ifndef POLYFEM_WITH_OPTIMIZATION
			log_and_throw_error("PolyFEM was built without optimization support.");
#else
			return optimization_simulation(command_line, *loaded.resources, max_threads, is_strict, log_level, in_args);
#endif
		}
		else
			return forward_simulation(command_line, *loaded.resources, loaded.source_file, output_dir, max_threads,
									  is_strict, fallback_solver, log_level, in_args);
	}
	else if (!hdf5_file.empty())
	{
		const io::LoadedInput loaded = io::load_hdf5_input(hdf5_file);
		in_args = loaded.config;
		return forward_simulation(command_line, *loaded.resources, loaded.source_file, output_dir, max_threads,
								  is_strict, fallback_solver, log_level, in_args);
	}
	else
	{
		io::CheckpointReader checkpoint(checkpoint_file);
		in_args = checkpoint.config();
		return forward_simulation(command_line, checkpoint, std::filesystem::absolute(checkpoint_file).string(), output_dir, max_threads,
								  is_strict, fallback_solver, log_level, in_args);
	}
}

int forward_simulation(const CLI::App &command_line,
					   const io::ResourceIO &resources,
					   const std::string &source_file,
					   const std::string &output_dir,
					   const unsigned max_threads,
					   const bool is_strict,
					   const bool fallback_solver,
					   const spdlog::level::level_enum &log_level,
					   json &in_args)
{
	if (in_args.empty())
	{
		logger().error("No input file specified!");
		return command_line.exit(CLI::RequiredError("--json or --hdf5"));
	}

	json tmp = json::object();
	if (has_arg(command_line, "log_level"))
		tmp["/output/log/level"_json_pointer] = int(log_level);
	if (has_arg(command_line, "max_threads"))
		tmp["/solver/max_threads"_json_pointer] = max_threads;
	if (has_arg(command_line, "output_dir"))
		tmp["/output/directory"_json_pointer] = std::filesystem::absolute(output_dir);
	if (has_arg(command_line, "enable_overwrite_solver"))
		tmp["/solver/linear/enable_overwrite_solver"_json_pointer] = fallback_solver;
	assert(tmp.is_object());
	in_args.merge_patch(tmp);

	const std::string checkpoint_path =
		in_args.contains("/input/checkpoint/path"_json_pointer)
			? in_args["/input/checkpoint/path"_json_pointer].get<std::string>()
			: std::string();
	if (!checkpoint_path.empty())
	{
		const std::filesystem::path path = std::filesystem::path(checkpoint_path).is_absolute()
											   ? std::filesystem::path(checkpoint_path)
											   : resources.host_directory() / checkpoint_path;
		io::CheckpointReader checkpoint(path.lexically_normal());
		json continuation = checkpoint.config();
		continuation["input"]["checkpoint"]["path"] = "";
		continuation["input"]["checkpoint"]["reorder"] =
			in_args.contains("/input/checkpoint/reorder"_json_pointer)
				? in_args["/input/checkpoint/reorder"_json_pointer].get<bool>()
				: false;
		return forward_simulation(
			command_line, checkpoint, source_file, output_dir, max_threads,
			is_strict, fallback_solver, log_level, continuation);
	}

	return forward_simulation_with_varform_state(resources, source_file, in_args, is_strict);
}

int forward_simulation(const CLI::App &command_line,
					   const io::CheckpointReader &checkpoint,
					   const std::string &source_file,
					   const std::string &output_dir,
					   const unsigned max_threads,
					   const bool is_strict,
					   const bool fallback_solver,
					   const spdlog::level::level_enum &log_level,
					   json &in_args)
{
	json tmp = json::object();
	if (has_arg(command_line, "log_level"))
		tmp["/output/log/level"_json_pointer] = int(log_level);
	if (has_arg(command_line, "max_threads"))
		tmp["/solver/max_threads"_json_pointer] = max_threads;
	if (has_arg(command_line, "output_dir"))
		tmp["/output/directory"_json_pointer] = std::filesystem::absolute(output_dir);
	if (has_arg(command_line, "enable_overwrite_solver"))
		tmp["/solver/linear/enable_overwrite_solver"_json_pointer] = fallback_solver;
	if (has_arg(command_line, "checkpoint-reorder"))
		tmp["/input/checkpoint/reorder"_json_pointer] = true;
	in_args.merge_patch(tmp);

	return forward_simulation_with_varform_state(checkpoint, source_file, in_args, is_strict);
}

#ifdef POLYFEM_WITH_OPTIMIZATION
int optimization_simulation(const CLI::App &command_line,
							const io::ResourceIO &resources,
							const unsigned max_threads,
							const bool is_strict,
							const spdlog::level::level_enum &log_level,
							json &opt_args)
{
	json tmp = json::object();
	if (has_arg(command_line, "log_level"))
		tmp["/output/log/level"_json_pointer] = int(log_level);
	if (has_arg(command_line, "max_threads"))
		tmp["/solver/max_threads"_json_pointer] = max_threads;
	opt_args.merge_patch(tmp);

	OptState opt_state;
	return opt_state.run(opt_args, resources, is_strict);
}
#endif

namespace
{
	// RB-05 / RB-12: every named failure ends here instead of in
	// std::terminate, so the last lines of the log say what happened in
	// plain terms, the sinks are flushed, and the exit status tells a
	// refusal from a crash (ExitStatus.hpp).
	int report_failure(const ExitStatus status, const std::string &what, const std::string &advice)
	{
		// The run manifest, if the run got far enough to have one, records
		// the failure before the log does (RB-12).
		io::RunManifest::finalize_active(status == ExitStatus::ResourceLimit ? "resource_failure" : "failed", status, what);
		logger().critical("PolyFEM stopped: {}", what);
		if (!advice.empty())
			logger().critical("{}", advice);
		logger().critical(
			"Exit status {} ({}).", int(status),
			status == ExitStatus::ResourceLimit ? "resource limit; not a crash, the accepted steps on disk are intact" : "named failure; not a crash");
		logger().flush();
		ipc::logger().flush();
		return status;
	}
} // namespace

int main(int argc, char **argv)
{
	try
	{
		return run(argc, argv);
	}
	catch (const ipc::BroadPhaseBudgetExceeded &e)
	{
		return report_failure(
			ExitStatus::ResourceLimit, e.what(),
			"A contact broad-phase resource limit was reached before the memory was allocated: the solver's trial step would have swept the surfaces so far that finding their candidate pairs needed more scratch memory than solver.contact.CCD.resource_limits allows (a safety stop, not a crash). Usually a few nodes moved very far in one Newton trial. Reduce the time step or the load increment, use a BVH broad phase, or raise the limits (0 disables them).");
	}
	catch (const ipc::BroadPhaseUnrepresentable &e)
	{
		// RBR-02: the same containment family as the budget, raised with or
		// without resource limits -- the grid this sweep needs cannot be
		// indexed, so the build is refused before any conversion.
		return report_failure(
			ExitStatus::ResourceLimit, e.what(),
			"The contact broad phase cannot index the hash grid this trial step would need: the solver's trial step swept the surfaces so far, relative to the grid's cell size, that the grid would have more cells than can be addressed (a safety stop, not a crash; it does not depend on solver.contact.CCD.resource_limits). Usually a few nodes moved very far in one Newton trial. Reduce the time step or the load increment, or use a BVH broad phase.");
	}
	catch (const std::bad_alloc &e)
	{
		return report_failure(
			ExitStatus::ResourceLimit, std::string("memory allocation failed (") + e.what() + ")",
			"The system refused a memory allocation. Reduce the mesh or the time step, close other programs, or run on a machine with more memory.");
	}
	catch (const std::exception &e)
	{
		return report_failure(ExitStatus::Failure, e.what(), "");
	}
}
