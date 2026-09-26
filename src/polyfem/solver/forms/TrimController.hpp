#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace polyfem::solver
{
	// EF-02/03: dimensionless control only; never assigns a contact coefficient.
	struct ForceWeightedTrim
	{
		double lower = .35, upper = .5, hysteresis = .025;
		double max_factor = 4., seed_max_factor = 4096., seed_cosine = .8;
		int interval = 10;

		// Signed derivative d/dg [-(g^2-1)^2 log(g^2)], common scales cancel.
		static double force(const double g)
		{
			return -4 * g * (g * g - 1) * std::log(g * g) - 2 * (g * g - 1) * (g * g - 1) / g;
		}
		double factor(const double gap) const
		{
			if (!(gap > 0 && gap < 1) || !std::isfinite(gap))
				return 1.;
			if (gap >= lower - hysteresis && gap <= upper + hysteresis)
				return 1.;
			const double target = gap > upper ? upper : lower;
			return std::clamp(force(gap) / force(target), 1 / max_factor, max_factor);
		}
		// Veto a downward proposal whose scalar force response predicts the
		// existing collapse statistic would cross its lower threshold. This
		// is a guard, not a new equilibrium or per-contact coefficient law.
		static bool safe_band_step(double factor, double severity_gap, double collapse_gap)
		{
			if (factor >= 1.)
				return true;
			if (!(severity_gap > collapse_gap && collapse_gap > 0 && collapse_gap < 1)
				|| !std::isfinite(severity_gap))
				return false;
			if (severity_gap >= 1.)
				return true;
			return factor >= force(severity_gap) / force(collapse_gap);
		}
		double seed_factor(double current, double estimate, double cosine, bool collapse) const
		{
			if (!(current > 0 && estimate > 0) || !std::isfinite(estimate)
				|| !std::isfinite(cosine) || cosine < seed_cosine)
				return 1.;
			if (collapse && estimate < current)
				return 1.;
			return std::clamp(estimate / current, 1 / seed_max_factor, seed_max_factor);
		}
	};

	// Rescaling avoids overflow and makes common positive force scales cancel.
	struct ForceWeightedGap
	{
		double scale = 0, weights = 0, sum = 0;
		bool valid = true;
		void add(double gap, double force)
		{
			if (!std::isfinite(gap) || !std::isfinite(force) || gap < 0 || force < 0)
			{
				valid = false;
				return;
			}
			if (force == 0)
				return;
			if (force > scale)
			{
				const double ratio = scale / force;
				sum *= ratio;
				weights *= ratio;
				scale = force;
			}
			sum += gap * (force / scale);
			weights += force / scale;
		}
		double mean() const
		{
			return valid && weights > 0 ? sum / weights : std::numeric_limits<double>::quiet_NaN();
		}
	};
} // namespace polyfem::solver
