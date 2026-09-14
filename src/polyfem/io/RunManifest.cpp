#include "RunManifest.hpp"

#include <polyfem/io/BuildInfo.hpp>
#include <polyfem/io/DiagnosticSchemas.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/Sha256.hpp>
#include <polyfem/utils/StringUtils.hpp>
#include <polyfem/utils/getRSS.h>
#include <polyfem/utils/par_for.hpp>

#include <polysolve/linear/Solver.hpp>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <spdlog/version.h>
#if defined(POLYFEM_WITH_TBB) || defined(POLYFEM_WITH_CPP_THREADS)
#include <tbb/version.h>
#endif

#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <process.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <sys/utsname.h>
#include <unistd.h>
#else
#include <sys/utsname.h>
#include <unistd.h>
#endif

namespace polyfem::io
{
	namespace
	{
		std::mutex &registry_mutex()
		{
			static std::mutex mutex;
			return mutex;
		}

		// A strong reference: the State that created the manifest is gone by
		// the time main's failure handler runs, and the manifest must still be
		// there to record the failure. finalize() releases it.
		std::shared_ptr<RunManifest> &active_slot()
		{
			static std::shared_ptr<RunManifest> slot;
			return slot;
		}

		std::vector<std::string> &command_line_slot()
		{
			static std::vector<std::string> slot;
			return slot;
		}

		json unavailable(const std::string &why)
		{
			return json{{"value", nullptr}, {"unavailable_reason", why}};
		}

		std::string utc_now()
		{
			const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::tm tm{};
#if defined(_WIN32)
			gmtime_s(&tm, &now);
#else
			gmtime_r(&now, &tm);
#endif
			char buffer[32];
			std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
			return buffer;
		}

		std::string file_time_utc(const std::filesystem::path &path)
		{
			// file_clock -> system_clock conversion is C++20; the portable
			// route is the last-write time expressed through the C library.
			const auto ftime = std::filesystem::last_write_time(path);
			const auto system = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
				ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
			const std::time_t t = std::chrono::system_clock::to_time_t(system);
			std::tm tm{};
#if defined(_WIN32)
			gmtime_s(&tm, &t);
#else
			gmtime_r(&t, &tm);
#endif
			char buffer[32];
			std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
			return buffer;
		}

		int process_id()
		{
#if defined(_WIN32)
			return _getpid();
#else
			return int(getpid());
#endif
		}

		std::string executable_path()
		{
#if defined(_WIN32)
			char buffer[MAX_PATH];
			const DWORD n = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
			return n > 0 && n < MAX_PATH ? std::string(buffer, n) : std::string();
#elif defined(__APPLE__)
			uint32_t size = 0;
			_NSGetExecutablePath(nullptr, &size);
			std::string buffer(size, '\0');
			if (_NSGetExecutablePath(buffer.data(), &size) != 0)
				return "";
			buffer.resize(std::strlen(buffer.c_str()));
			std::error_code ec;
			const auto canonical = std::filesystem::canonical(buffer, ec);
			return ec ? buffer : canonical.string();
#else
			std::error_code ec;
			const auto path = std::filesystem::read_symlink("/proc/self/exe", ec);
			return ec ? std::string() : path.string();
#endif
		}

		json executable_identity()
		{
			const std::string path = executable_path();
			if (path.empty())
				return unavailable("The executable path could not be determined on this platform");
			try
			{
				return json{
					{"path", path},
					{"sha256", utils::Sha256::of_file(path)},
					{"size_bytes", std::filesystem::file_size(path)},
					{"modified", file_time_utc(path)}};
			}
			catch (const std::exception &e)
			{
				return unavailable(e.what());
			}
		}

		json platform()
		{
			json result;
#if defined(_WIN32)
			result["system"] = "Windows";
#if defined(_M_ARM64)
			result["machine"] = "arm64";
#elif defined(_M_X64)
			result["machine"] = "x86_64";
#else
			result["machine"] = "unknown";
#endif
			result["release"] = unavailable("Not queried on Windows");
			result["version"] = unavailable("Not queried on Windows");
#else
			utsname name{};
			if (uname(&name) == 0)
			{
				result["system"] = name.sysname;
				result["release"] = name.release;
				result["version"] = name.version;
				result["machine"] = name.machine;
			}
			else
				result = unavailable("uname failed");
#endif
			result["hardware_concurrency"] = std::thread::hardware_concurrency();
			return result;
		}

		json libraries()
		{
			json result;
			result["eigen"] = fmt::format("{}.{}.{}", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
			result["nlohmann_json"] = fmt::format("{}.{}.{}", NLOHMANN_JSON_VERSION_MAJOR, NLOHMANN_JSON_VERSION_MINOR, NLOHMANN_JSON_VERSION_PATCH);
			result["spdlog"] = fmt::format("{}.{}.{}", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH);
#if defined(TBB_VERSION_STRING)
			result["tbb"] = TBB_VERSION_STRING;
#elif defined(TBB_VERSION_MAJOR)
			result["tbb"] = fmt::format("{}.{}", TBB_VERSION_MAJOR, TBB_VERSION_MINOR);
#endif
			return result;
		}

		const char *threading_backend()
		{
#if defined(POLYFEM_WITH_TBB)
			return "TBB";
#elif defined(POLYFEM_WITH_CPP_THREADS)
			return "CPP";
#else
			return "NONE";
#endif
		}

		json hashed_file(const std::string &path)
		{
			try
			{
				return json{{"path", path}, {"sha256", utils::Sha256::of_file(path)}, {"size_bytes", std::filesystem::file_size(path)}};
			}
			catch (const std::exception &e)
			{
				return json{{"path", path}, {"sha256", nullptr}, {"unavailable_reason", e.what()}};
			}
		}

		void collect_referenced(const json &node, const std::string &pointer, const std::string &root_path, json &out)
		{
			if (node.is_object())
			{
				for (const auto &[key, value] : node.items())
				{
					// Output file names are outputs, not inputs; root_path is
					// the input file, recorded on its own.
					if (pointer.empty() && (key == "output" || key == "root_path"))
						continue;
					collect_referenced(value, pointer + "/" + key, root_path, out);
				}
			}
			else if (node.is_array())
			{
				for (size_t i = 0; i < node.size(); ++i)
					collect_referenced(node[i], pointer + "/" + std::to_string(i), root_path, out);
			}
			else if (node.is_string())
			{
				const std::string value = node.get<std::string>();
				if (value.empty() || value.find_first_of("\n\r") != std::string::npos)
					return;
				std::error_code ec;
				const std::string resolved = utils::resolve_path(value, root_path, /*only_if_exists=*/true);
				if (!std::filesystem::is_regular_file(resolved, ec))
					return;
				json entry = hashed_file(std::filesystem::weakly_canonical(resolved, ec).string());
				entry["pointer"] = pointer;
				entry["value"] = value;
				out.push_back(entry);
			}
		}

		std::string contact_mode(const json &args)
		{
			if (!args.contains("contact") || !args["contact"].value("enabled", false))
				return "disabled";
			const json &stiffness = args["solver"]["contact"]["barrier_stiffness"];
			if (stiffness.is_number())
				return "fixed";
			if (stiffness.is_string() && stiffness.get<std::string>() == "semi_implicit")
				return "semi_implicit";
			return "adaptive";
		}
	} // namespace

	void RunManifest::set_command_line(int argc, char **argv)
	{
		std::lock_guard<std::mutex> lock(registry_mutex());
		command_line_slot().assign(argv, argv + argc);
	}

	std::shared_ptr<RunManifest> RunManifest::active()
	{
		std::lock_guard<std::mutex> lock(registry_mutex());
		return active_slot();
	}

	void RunManifest::finalize_active(const std::string &status, const int exit_status, const std::string &message)
	{
		if (auto manifest = active())
			manifest->finalize(status, exit_status, message);
	}

	std::string RunManifest::canonical_sha256(const json &value)
	{
		// nlohmann's object keys are sorted, so dump() is canonical for equal values.
		return utils::Sha256::of(value.dump());
	}

	json RunManifest::referenced_files(const json &args, const std::string &root_path)
	{
		json out = json::array();
		collect_referenced(args, "", root_path, out);
		return out;
	}

	std::shared_ptr<RunManifest> RunManifest::create(
		const json &args,
		const std::string &path,
		const std::string &input_file,
		const std::vector<std::string> &common_chain)
	{
		if (path.empty())
			return nullptr;

		std::shared_ptr<RunManifest> manifest(new RunManifest());
		manifest->path_ = path;
		manifest->started_ = std::chrono::steady_clock::now();
		{
			std::random_device device;
			std::uniform_int_distribution<uint32_t> random;
			manifest->run_id_ = fmt::format("{}-{}-{:08x}", utc_now(), process_id(), random(device));
		}

		json &m = manifest->manifest_;
		m["schema"] = schemas::RUN_MANIFEST;
		m["version"] = schemas::RUN_MANIFEST_VERSION;
		m["run_id"] = manifest->run_id_;
		m["started_at"] = utc_now();
		m["completion"] = {{"status", "running"}, {"exit_status", nullptr}, {"message", nullptr}, {"finished_at", nullptr}, {"wall_seconds", nullptr}, {"peak_rss_mb", nullptr}};

		// Process and binary identity.
		json process;
		process["executable"] = executable_identity();
		{
			std::lock_guard<std::mutex> lock(registry_mutex());
			const auto &command_line = command_line_slot();
			process["command_line"] = command_line.empty() ? unavailable("Not a PolyFEM_bin run (library use) or main did not record it") : json(command_line);
		}
		{
			std::error_code ec;
			const auto cwd = std::filesystem::current_path(ec);
			process["working_directory"] = ec ? unavailable(ec.message()) : json(cwd.string());
		}
		process["pid"] = process_id();
		process["platform"] = platform();
		const json &max_threads = args["solver"]["max_threads"];
		process["threads"] = {
			{"backend", threading_backend()},
			{"requested_max_threads", max_threads},
			{"effective", utils::get_n_threads()},
			{"hardware_concurrency", std::thread::hardware_concurrency()},
			{"scope", "The parallel-for limit (tbb::global_control) and Eigen's thread count; the linear solver and the IPC toolkit follow it, an external OMP/TBB environment setting is not read"}};
		m["process"] = process;

		// Source identity (compiled in) and the libraries' compile-time versions.
		m["build"] = build_info();
		m["libraries"] = libraries();

		// Effective input.
		json input;
		const std::string root_path = args.value("root_path", std::string());
		input["root_path"] = root_path;
		{
			std::error_code ec;
			input["file"] = !input_file.empty() && std::filesystem::is_regular_file(input_file, ec)
								? hashed_file(std::filesystem::weakly_canonical(input_file, ec).string())
								: unavailable("No input file (library use, or the input was not a file)");
		}
		input["common_chain"] = json::array();
		for (const auto &common : common_chain)
			input["common_chain"].push_back(hashed_file(common));
		input["effective"] = args;
		input["effective_sha256"] = canonical_sha256(args);
		input["effective_scope"] = "The input after `common` expansion, defaults and the command-line overrides, as the solver used it; hashed as the compact sorted-key serialization";
		input["referenced_files"] = referenced_files(args, root_path);
		if (args.contains("units"))
			input["units"] = args["units"];
		{
			// RB-09 / RB-11: the nonlinear stopping tolerance scales with this
			// force density and the unit system; record both explicitly.
			const double setting = args["solver"]["advanced"].value("characteristic_force_density", 0.);
			input["characteristic_force_density"] = {
				{"setting", setting},
				{"effective", setting > 0 ? setting : 10000.},
				{"scope", "solver/advanced/characteristic_force_density; a nonpositive setting means the SI default 1e4 (NLProblem::grad_norm_rescaling: F0 * L^1.5 in 3D, F0 * L in 2D for the L2 norm)"}};
		}
		m["input"] = input;

		// Producer provenance, when the input carries it (the Houdini asset);
		// the spec injects an all-empty object otherwise.
		m["producer"] = nullptr;
		if (args.contains("provenance") && args["provenance"].is_object())
			for (const auto &[key, value] : args["provenance"].items())
				if (!value.is_string() || !value.get<std::string>().empty())
				{
					m["producer"] = args["provenance"];
					break;
				}

		// Solver settings summary; the complete settings are in input.effective.
		json solver;
		solver["linear"] = {
			{"solver", args["solver"]["linear"].value("solver", std::string())},
			{"precond", args["solver"]["linear"].value("precond", std::string())},
			{"available_solvers", polysolve::linear::Solver::available_solvers()}};
		solver["nonlinear"] = {
			{"solver", args["solver"]["nonlinear"].value("solver", std::string())},
			{"line_search", args["solver"]["nonlinear"].contains("line_search") ? args["solver"]["nonlinear"]["line_search"].value("method", std::string()) : std::string()}};
		json contact = {{"mode", contact_mode(args)}};
		if (contact["mode"] != "disabled")
		{
			contact["dhat"] = args["contact"]["dhat"];
			contact["friction_coefficient"] = args["contact"]["friction_coefficient"];
			contact["friction_iterations"] = args["solver"]["contact"]["friction_iterations"];
			contact["use_convergent_formulation"] = args["contact"]["use_convergent_formulation"];
			contact["use_gcp_formulation"] = args["contact"]["use_gcp_formulation"];
			contact["broad_phase"] = args["solver"]["contact"]["CCD"]["broad_phase"];
			contact["ccd"] = {{"tolerance", args["solver"]["contact"]["CCD"]["tolerance"]}, {"max_iterations", args["solver"]["contact"]["CCD"]["max_iterations"]}};
			contact["resource_limits"] = args["solver"]["contact"]["CCD"].value("resource_limits", json(nullptr));
			if (contact["mode"] == "semi_implicit")
				contact["semi_implicit"] = args["solver"]["contact"]["semi_implicit"];
		}
		solver["contact"] = contact;
		solver["model"] = unavailable("The forms have not been built yet");
		m["solver"] = solver;

		// What this run may write besides the manifest.
		const json &output = args["output"];
		json diagnostics;
		diagnostics["physical_diagnostics"] = output.value("physical_diagnostics", false);
		diagnostics["physical_diagnostics_contact_path"] = output.value("physical_diagnostics_contact_path", false);
		diagnostics["physical_balance_tolerance"] = output.value("physical_balance_tolerance", 1e-6);
		diagnostics["schemas"] = {
			{schemas::RUN_MANIFEST, schemas::RUN_MANIFEST_VERSION},
			{schemas::BUILD_INFO, schemas::BUILD_INFO_VERSION},
			{schemas::PHYSICAL_DIAGNOSTICS, schemas::PHYSICAL_DIAGNOSTICS_VERSION},
			{schemas::COEFFICIENT_EVENT, schemas::COEFFICIENT_EVENT_VERSION},
			{schemas::SOLVER_ATTEMPT, schemas::SOLVER_ATTEMPT_VERSION}};
		diagnostics["files"] = {
			{"manifest", path},
			{"physical_diagnostics", output.value("physical_diagnostics", false) ? json("physical-diagnostics.jsonl, coefficient-events.jsonl, solver-attempts.jsonl in the output directory") : json(nullptr)},
			{"output_json", output.value("json", std::string()).empty() ? json(nullptr) : output["json"]},
			{"log", output["log"].value("path", std::string()).empty() ? json(nullptr) : output["log"]["path"]}};
		m["diagnostics"] = diagnostics;

		m["steps"] = json::array();
		m["steps_scope"] = "One record per solve_tensor_nonlinear call (step 0 is the static or initial solve), written at the RB-04 recording boundary for every outcome; a run that stops before its first solve has no step";

		{
			std::lock_guard<std::mutex> lock(registry_mutex());
			active_slot() = manifest;
		}
		manifest->write();
		logger().info("Run manifest {} ({})", path, manifest->run_id_);
		return manifest;
	}

	void RunManifest::record_model(const json &model)
	{
		manifest_["solver"]["model"] = model;
		write();
	}

	void RunManifest::record_step(const json &step)
	{
		manifest_["steps"].push_back(step);
		write();
	}

	void RunManifest::finalize(const std::string &status, const int exit_status, const std::string &message)
	{
		if (finalized_)
		{
			if (manifest_["completion"]["status"] != status)
				logger().debug("Run manifest already finalized as {}; ignoring {}", manifest_["completion"]["status"].get<std::string>(), status);
			return;
		}
		finalized_ = true;
		json &completion = manifest_["completion"];
		completion["status"] = status;
		completion["exit_status"] = exit_status < 0 ? json(nullptr) : json(exit_status);
		completion["message"] = message.empty() ? json(nullptr) : json(message);
		completion["finished_at"] = utc_now();
		completion["wall_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
		completion["peak_rss_mb"] = double(getPeakRSS()) / (1024. * 1024.);
		completion["steps_recorded"] = manifest_["steps"].size();
		write();
		std::lock_guard<std::mutex> lock(registry_mutex());
		if (active_slot().get() == this)
			active_slot().reset();
	}

	void RunManifest::write() const
	{
		try
		{
			const std::filesystem::path target(path_);
			const std::filesystem::path temporary = target.string() + ".tmp";
			{
				std::ofstream file(temporary, std::ios::trunc);
				if (!file.is_open())
					throw std::runtime_error("Could not open " + temporary.string());
				file << manifest_.dump(2) << std::endl;
				if (!file)
					throw std::runtime_error("Could not write " + temporary.string());
			}
			std::filesystem::rename(temporary, target);
		}
		catch (const std::exception &e)
		{
			logger().warn("Run manifest not written: {}", e.what());
		}
	}
} // namespace polyfem::io
