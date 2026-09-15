#pragma once

#include <polyfem/Common.hpp>

#include <Eigen/Core>

#include <exception>
#include <stdexcept>
#include <string>

namespace polyfem::solver
{
	/// @brief RB-06 test hook (`solver/advanced/failure_injection`): throw a
	///        deterministic failure at a chosen point of one step's nonlinear
	///        solve, so the failed-attempt rollback and the publication path
	///        can be exercised end to end through the real solve. Off by
	///        default; never a production setting.
	struct FailureInjection
	{
		enum class Phase
		{
			None,
			AfterAL,           ///< after solve_al returned, before the reduced solve
			AfterStallRetune,  ///< in the stall restart hook, after the retune
			ALSubsolve,        ///< at accepted iterate `iteration` of an AL subsolve
			Reduced,           ///< at accepted iterate `iteration` of the reduced solve
			Lagging,           ///< after the lag update of lagging iteration `iteration`
			BeforePublication, ///< after the solve returned, before the step callback and outputs
			AfterPublication   ///< after the step's outputs, before the history advances
		};

		Phase phase = Phase::None;
		int step = 1;
		int iteration = 0;
		/// A named failure (std::exception, absorbed by no in-step handler)
		/// or a std::runtime_error (absorbed by the AL scaled-weight retry).
		bool named = true;

		static FailureInjection from_args(const json &advanced);
		static const char *phase_name(Phase phase);

		bool enabled() const { return phase != Phase::None; }
		bool matches(const Phase at, const int at_step) const { return phase == at && step == at_step; }
		bool matches(const Phase at, const int at_step, const int at_iteration) const
		{
			return matches(at, at_step) && iteration == at_iteration;
		}

		/// @brief Throw the configured failure; `where` names the point.
		[[noreturn]] void raise(const std::string &where) const;
	};

	/// @brief The named failure an injection throws: a std::exception that is
	///        not a std::runtime_error, so the solver handlers that retry a
	///        runtime_error (smaller line-search step, scaled AL weight, stall
	///        restart) never absorb it -- the same type discipline as the
	///        RB-05 resource failure.
	class InjectedFailure : public std::exception
	{
	public:
		explicit InjectedFailure(std::string what) : what_(std::move(what)) {}
		const char *what() const noexcept override { return what_.c_str(); }

	private:
		std::string what_;
	};
} // namespace polyfem::solver
