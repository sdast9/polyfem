#pragma once

namespace polyfem::io
{
	/// @brief Names and versions of the JSON records a run can write, in one
	///        place so the writers and the run manifest (RB-12) cannot drift
	///        apart. Bump a version here when a record's fields change meaning.
	namespace schemas
	{
		/// The run manifest itself (RunManifest.hpp).
		constexpr const char *RUN_MANIFEST = "polyfem.run-manifest";
		constexpr int RUN_MANIFEST_VERSION = 1;
		/// Build identity compiled into the library (BuildInfo.hpp).
		constexpr const char *BUILD_INFO = "polyfem.build-info";
		constexpr int BUILD_INFO_VERSION = 1;
		/// RB-04 endpoint records, physical-diagnostics.jsonl (record version 3
		/// since the RB-09 decision of 2026-09-13).
		constexpr const char *PHYSICAL_DIAGNOSTICS = "polyfem.physical-diagnostics";
		constexpr int PHYSICAL_DIAGNOSTICS_VERSION = 3;
		/// RB-04 coefficient operations, coefficient-events.jsonl.
		constexpr const char *COEFFICIENT_EVENT = "polyfem.coefficient-event";
		constexpr int COEFFICIENT_EVENT_VERSION = 1;
		/// RB-04 remainder: per-iteration attempt observation, solver-attempts.jsonl
		/// (version 2 since the BFGS audit's stage 4 added the `solver` field,
		/// PolySolve's opt-in per-iteration nonlinear diagnostics; version 3
		/// since stage 3: a growing line search's extensions lengthen the
		/// trial, which the `extensions*` and `unextended_norm` fields record).
		constexpr const char *SOLVER_ATTEMPT = "polyfem.solver-attempt";
		constexpr int SOLVER_ATTEMPT_VERSION = 3;
	} // namespace schemas
} // namespace polyfem::io
