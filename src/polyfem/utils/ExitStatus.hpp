#pragma once

namespace polyfem
{
	/// @brief Process exit statuses of PolyFEM_bin. Every named failure is
	///        caught at the top of main, logged, flushed and mapped here, so a
	///        caller (the Houdini asset, a script) can tell a refusal from a
	///        crash: an abort signal (-6 / 134) is a crash or a failed
	///        assertion, never a named failure (RB-05 / RB-12).
	enum ExitStatus : int
	{
		Success = 0,
		/// A named failure: invalid input, a solver that stopped as
		/// configured, an unsupported option, a scene the code refuses.
		Failure = 1,
		/// A resource failure: a contact broad-phase resource limit reached
		/// before allocation (solver/contact/CCD/resource_limits) or an
		/// allocation the system refused (std::bad_alloc). Not a crash: the
		/// accepted steps on disk are intact.
		ResourceLimit = 3,
	};
} // namespace polyfem
