#include "FailureInjection.hpp"

#include <polyfem/utils/Logger.hpp>

namespace polyfem::solver
{
	FailureInjection FailureInjection::from_args(const json &advanced)
	{
		FailureInjection injection;
		if (!advanced.is_object() || !advanced.contains("failure_injection") || !advanced["failure_injection"].is_object())
			return injection;
		const json &args = advanced["failure_injection"];
		const std::string phase = args.value("phase", "none");
		if (phase == "none")
			injection.phase = Phase::None;
		else if (phase == "after_al")
			injection.phase = Phase::AfterAL;
		else if (phase == "after_stall_retune")
			injection.phase = Phase::AfterStallRetune;
		else if (phase == "al_subsolve")
			injection.phase = Phase::ALSubsolve;
		else if (phase == "reduced")
			injection.phase = Phase::Reduced;
		else if (phase == "lagging")
			injection.phase = Phase::Lagging;
		else if (phase == "before_publication")
			injection.phase = Phase::BeforePublication;
		else if (phase == "after_publication")
			injection.phase = Phase::AfterPublication;
		else
			log_and_throw_error("Unknown solver/advanced/failure_injection/phase \"{}\"", phase);
		injection.step = args.value("step", 1);
		injection.iteration = args.value("iteration", 0);
		const std::string kind = args.value("kind", "named");
		if (kind == "named")
			injection.named = true;
		else if (kind == "runtime_error")
			injection.named = false;
		else
			log_and_throw_error("Unknown solver/advanced/failure_injection/kind \"{}\"", kind);
		if (injection.enabled())
			logger().warn(
				"solver/advanced/failure_injection is set: a {} failure will be thrown at phase {} of step {} (iteration {}). This is a test hook, not a simulation result.",
				kind, phase, injection.step, injection.iteration);
		return injection;
	}

	const char *FailureInjection::phase_name(const Phase phase)
	{
		switch (phase)
		{
		case Phase::None:
			return "none";
		case Phase::AfterAL:
			return "after_al";
		case Phase::AfterStallRetune:
			return "after_stall_retune";
		case Phase::ALSubsolve:
			return "al_subsolve";
		case Phase::Reduced:
			return "reduced";
		case Phase::Lagging:
			return "lagging";
		case Phase::BeforePublication:
			return "before_publication";
		case Phase::AfterPublication:
			return "after_publication";
		}
		return "unknown";
	}

	void FailureInjection::raise(const std::string &where) const
	{
		const std::string what = fmt::format(
			"Injected failure (solver/advanced/failure_injection: phase {}, step {}, iteration {}) at {}",
			phase_name(phase), step, iteration, where);
		logger().error("{}", what);
		logger().flush();
		if (named)
			throw InjectedFailure(what);
		throw std::runtime_error(what);
	}
} // namespace polyfem::solver
