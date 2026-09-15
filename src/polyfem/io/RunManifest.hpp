#pragma once

#include <polyfem/Common.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace polyfem::io
{
	/// @brief The run manifest (RB-12, schema `polyfem.run-manifest` version 1):
	///        one JSON file per run, written when the effective input is known
	///        and rewritten after every step and at completion, that identifies
	///        what ran on what. It separates *source identity* (the checkouts
	///        the library was built from, with their declared pins next to the
	///        effective sources) from *binary identity* (the executable's own
	///        SHA-256), and records the platform, thread and linear-solver
	///        settings, the effective expanded input with its hash, every file
	///        that input refers to with its hash, the producer's provenance
	///        when the input carries one (`/provenance`, the Houdini asset),
	///        the model description of the contact form, the diagnostics
	///        schemas the run writes, the per-step attempt history and the
	///        completion status. Anything that could not be measured is
	///        `null` with an `unavailable_reason`, never a guess.
	///
	///        The file is rewritten atomically (temporary file, then rename),
	///        so a reader sees either the previous or the new state. A failure
	///        to write the manifest is logged and never stops the solve.
	class RunManifest
	{
	public:
		/// @brief Create and write the manifest of a run.
		/// @param effective_args The expanded input after defaults (State::args).
		/// @param path The manifest file (already resolved against the output
		///             directory); empty disables the manifest.
		/// @param input_file The input JSON/YAML/HDF5 file when known, else "".
		/// @param common_chain Every `common` file the input was expanded from,
		///                     outermost first.
		/// @return The manifest, or nullptr when `path` is empty.
		static std::shared_ptr<RunManifest> create(
			const json &effective_args,
			const std::string &path,
			const std::string &input_file,
			const std::vector<std::string> &common_chain);

		/// @brief The command line PolyFEM_bin was invoked with, recorded by
		///        main before the manifest exists (a library user need not call it).
		static void set_command_line(int argc, char **argv);

		/// @brief The most recently created, not yet finalized manifest, so the
		///        process-level failure handler can close it; nullptr otherwise.
		static std::shared_ptr<RunManifest> active();

		/// @brief Close the active manifest (if any) with the given completion.
		static void finalize_active(const std::string &status, int exit_status, const std::string &message);

		const std::string &run_id() const { return run_id_; }
		const std::string &path() const { return path_; }
		const json &content() const { return manifest_; }

		/// @brief Description of the contact/coefficient model once the forms
		///        exist (law, coefficient identity, gap convention, fallbacks).
		void record_model(const json &model);

		/// @brief Append one step's attempt record (NonlinearElasticVarForm
		///        writes it at the same boundary as the RB-04 endpoint record,
		///        for every outcome).
		void record_step(const json &step);

		/// @brief RB-06: merge fields into the most recently recorded step (the
		///        rollback verification of a failed attempt is known only after
		///        the step's record was written) and rewrite the file. No-op
		///        without a recorded step.
		void amend_last_step(const json &fields);

		/// @brief Record the completion of the run. The first call wins; a
		///        later call with a different status is logged and ignored.
		/// @param status "completed", "failed" or "resource_failure".
		/// @param exit_status The process exit status, or -1 when not a process.
		void finalize(const std::string &status, int exit_status, const std::string &message);

		bool finalized() const { return finalized_; }

		/// @brief SHA-256 of the canonical (sorted-key, compact) serialization
		///        of a JSON value: the hash the manifest stores for the
		///        effective input.
		static std::string canonical_sha256(const json &value);

		/// @brief Strings of `args` (outside `/output` and `/root_path`) that
		///        resolve, as the solver resolves them, to an existing regular
		///        file: their JSON pointer, resolved path, size and SHA-256.
		static json referenced_files(const json &args, const std::string &root_path);

	private:
		RunManifest() = default;

		void write() const;

		std::string run_id_;
		std::string path_;
		json manifest_;
		bool finalized_ = false;
		std::chrono::steady_clock::time_point started_;
	};
} // namespace polyfem::io
