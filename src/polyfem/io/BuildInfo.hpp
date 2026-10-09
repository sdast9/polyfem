#pragma once

#include <polyfem/Common.hpp>

namespace polyfem::io
{
	/// @brief Identity of the sources this library was built from (RB-12):
	///        schema `polyfem.build-info`, version 1, with the effective
	///        PolyFEM, IPC Toolkit and PolySolve checkouts (commit, branch,
	///        dirty state, patch hash), the pins the recipes declare, the
	///        compiler, configuration, threading backend and options. Generated
	///        by cmake/polyfem/polyfem_generate_build_info.cmake before every
	///        build of the library; a field that could not be determined
	///        carries an `unavailable_reason` instead of a guess.
	const json &build_info();
} // namespace polyfem::io
