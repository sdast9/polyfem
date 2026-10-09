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
		// EF-07: the smallest factor safe_band_step accepts for one collapse
		// term: 1 when the term is already at or below its threshold (or
		// invalid), 0 when the gap is at or beyond the support.
		static double min_safe_factor(double severity_gap, double collapse_gap)
		{
			if (!(severity_gap > collapse_gap && collapse_gap > 0 && collapse_gap < 1)
				|| !std::isfinite(severity_gap))
				return 1.;
			if (severity_gap >= 1.)
				return 0.;
			return force(severity_gap) / force(collapse_gap);
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

	// EF-07: opt-in guards against the trim limit cycle of the EF-02/03
	// controller (docs/ef-07-trim-loop.md). Dimensionless control on the
	// global trim only; every option off reproduces EF-02/03 exactly. The
	// per-step memory resets at each time step (new_step) and is part of the
	// form's rollback state.
	struct TrimLoopGuard
	{
		enum Source
		{
			Collapse,
			Band,
			Estimate,
			Other
		};
		static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

		// Options.
		bool pair_guard = false;          ///< band veto on each collapse term's own gap
		bool partial_guard = false;       ///< a vetoed softening is clamped to the smallest safe factor instead of cancelled
		bool exclude_born = false;        ///< min term ignores collisions born since the previous accepted iterate
		bool responsiveness_veto = false; ///< no collapse bump after one that did not lift the collapse gap
		double responsive_rise = 1.1;     ///< peak / reference gap that counts as a response
		double worsening = .5;            ///< current / reference gap that re-allows a vetoed bump
		double step_excursion = 0;        ///< > 1: trim kept within [anchor / B, anchor * B] for the whole step
		int reversal_limit = 0;           ///< > 0: after K reversals in a step, non-collapse moves may not reverse
		bool estimate_once = false;       ///< initial estimate accepted at most once per run

		bool any() const
		{
			return pair_guard || partial_guard || exclude_born || responsiveness_veto || step_excursion > 1 || reversal_limit > 0 || estimate_once;
		}

		// Per-step memory.
		double step_anchor = nan;
		int up = 0, down = 0, reversals = 0, last_direction = 0, blocked = 0, clamped = 0, vetoed = 0;
		double collapse_ref = nan, collapse_peak = nan;

		void new_step(const double trim)
		{
			step_anchor = trim;
			up = down = reversals = last_direction = blocked = clamped = vetoed = 0;
			collapse_ref = collapse_peak = nan;
		}

		// The collapse gap (distance / dhat of the binding collapse term) seen
		// at an accepted iterate or refresh; tracks the response to the last
		// collapse bump.
		void observe_collapse_gap(const double gap)
		{
			if (std::isfinite(collapse_ref) && std::isfinite(gap))
				collapse_peak = std::isfinite(collapse_peak) ? std::max(collapse_peak, gap) : gap;
		}

		// Whether a collapse bump at the given collapse gap may proceed. A bump
		// is vetoed only when the previous one this step did not lift the gap
		// by responsive_rise and the gap is not materially worse since
		// (worsening): a contact that keeps collapsing is still protected.
		bool allow_collapse(const double gap)
		{
			if (!responsiveness_veto || !std::isfinite(collapse_ref) || !std::isfinite(gap))
				return true;
			if (std::isfinite(collapse_peak) && collapse_peak > collapse_ref * responsive_rise)
				return true;
			if (gap < collapse_ref * worsening)
				return true;
			++vetoed;
			return false;
		}
		void collapse_bumped(const double gap)
		{
			collapse_ref = gap;
			collapse_peak = gap;
		}

		// The trim a proposed move may reach: the step excursion bound, then
		// the reversal lockout (collapse bumps are never locked out).
		double limit(const double current, const double proposed, const Source source)
		{
			if (!std::isfinite(step_anchor) || !(step_anchor > 0))
				step_anchor = current;
			double t = proposed;
			if (step_excursion > 1 && std::isfinite(step_excursion))
			{
				const double bounded = std::clamp(t, step_anchor / step_excursion, step_anchor * step_excursion);
				if (bounded != t)
					++clamped;
				t = bounded;
			}
			const int direction = t > current ? 1 : (t < current ? -1 : 0);
			if (direction != 0 && reversal_limit > 0 && source != Collapse && last_direction != 0
				&& direction != last_direction && reversals >= reversal_limit)
			{
				++blocked;
				return current;
			}
			return t;
		}
		void moved(const double before, const double after)
		{
			const int direction = after > before ? 1 : (after < before ? -1 : 0);
			if (direction == 0)
				return;
			if (last_direction != 0 && direction != last_direction)
				++reversals;
			last_direction = direction;
			(direction > 0 ? up : down)++;
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
