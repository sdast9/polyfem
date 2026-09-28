// EF-07: opt-in guards against the EF-02/03 trim limit cycle
// (docs/ef-07-trim-loop.md). Pure controller logic; the form wiring is
// exercised by the ball-burst and EF-01 matrix runs recorded there.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/TrimController.hpp>
#include <polysolve/nonlinear/PostStepData.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	// One vertex over one edge (2D), dhat 1: x[5] moves the vertex, gap = .2 + x[5].
	ipc::CollisionMesh one_pair_mesh()
	{
		Eigen::MatrixXd vertices(3, 2);
		vertices << -1, 0, 1, 0, 0, .2;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		return ipc::CollisionMesh(vertices, edges);
	}

	class GuardForm : public BarrierContactForm
	{
	public:
		GuardForm(const ipc::CollisionMesh &mesh, const json &options)
			: BarrierContactForm(mesh, 1., 1., false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, BarrierStiffnessMode::SemiImplicit,
								 options, Eigen::VectorXd::Ones(3))
		{
			set_system_hessian_provider([](const Eigen::VectorXd &, StiffnessMatrix &h) {
				h.resize(6, 6);
				h.setIdentity();
				h *= 100.;
			});
		}
	};

	// Trim after n accepted iterations at a fixed, pinched iterate (the
	// collapse gap cannot respond because x never moves).
	std::vector<double> pinned_trims(const json &options, const int n)
	{
		const auto mesh = one_pair_mesh();
		GuardForm f(mesh, options);
		Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
		x[5] = -.19; // gap .01 dhat: the collapse proxy fires
		f.init(x);
		f.refresh_semi_implicit_stiffness(x, false);
		std::vector<double> trims{f.barrier_stiffness()};
		const json info = json::object();
		for (int i = 1; i <= n; ++i)
		{
			f.post_step(polysolve::nonlinear::PostStepData(i, info, x, Eigen::VectorXd::Zero(6)));
			trims.push_back(f.barrier_stiffness());
		}
		return trims;
	}
} // namespace

TEST_CASE("Loop guard defaults change nothing", "[trim_controller][ef07]")
{
	TrimLoopGuard g;
	CHECK_FALSE(g.any());
	g.new_step(1.);
	for (const double proposed : {1e-6, .5, 2., 1e6})
		for (const auto source : {TrimLoopGuard::Collapse, TrimLoopGuard::Band, TrimLoopGuard::Estimate})
			CHECK(g.limit(1., proposed, source) == proposed);
	// Counters only observe.
	g.moved(1., 2.);
	g.moved(2., 1.);
	g.moved(1., 2.);
	CHECK(g.reversals == 2);
	CHECK(g.up == 2);
	CHECK(g.down == 1);
	CHECK(g.limit(2., .5, TrimLoopGuard::Band) == .5);
	g.collapse_bumped(.5);
	g.observe_collapse_gap(.5);
	CHECK(g.allow_collapse(.5));
}

TEST_CASE("Step excursion bound is anchored at the step start", "[trim_controller][ef07]")
{
	TrimLoopGuard g;
	g.step_excursion = 256;
	g.new_step(1.);
	CHECK(g.limit(1., 1e6, TrimLoopGuard::Collapse) == 256.);
	CHECK(g.limit(1., 1e-6, TrimLoopGuard::Band) == 1. / 256.);
	CHECK(g.clamped == 2);
	// Unlike the in-solve anchor, a refresh does not move it.
	CHECK(g.limit(200., 400., TrimLoopGuard::Collapse) == 256.);
	g.new_step(4.);
	CHECK(g.limit(4., 2000., TrimLoopGuard::Collapse) == 1024.);
	// A guard constructed without a step start anchors at its first use.
	TrimLoopGuard lazy;
	lazy.step_excursion = 4;
	CHECK(lazy.limit(2., 100., TrimLoopGuard::Band) == 8.);
}

TEST_CASE("Reversal lockout exempts collapse bumps", "[trim_controller][ef07]")
{
	TrimLoopGuard g;
	g.reversal_limit = 1;
	g.new_step(1.);
	g.moved(1., 2.);                                       // up
	CHECK(g.limit(2., 1., TrimLoopGuard::Band) == 1.);     // first reversal allowed
	g.moved(2., 1.);                                       // down, reversals = 1
	CHECK(g.limit(1., 2., TrimLoopGuard::Band) == 1.);     // would reverse again: blocked
	CHECK(g.blocked == 1);
	CHECK(g.limit(1., .5, TrimLoopGuard::Band) == .5);     // same direction is free
	CHECK(g.limit(1., 2., TrimLoopGuard::Collapse) == 2.); // protection kept
	g.new_step(1.);
	CHECK(g.reversals == 0);
}

TEST_CASE("Responsiveness veto keeps protection against a worsening collapse", "[trim_controller][ef07]")
{
	TrimLoopGuard g;
	g.responsiveness_veto = true;
	g.new_step(1.);
	CHECK(g.allow_collapse(.6)); // first bump of the step
	g.collapse_bumped(.6);
	g.observe_collapse_gap(.62); // < 10 % rise: unresponsive
	CHECK_FALSE(g.allow_collapse(.61));
	CHECK(g.vetoed == 1);
	CHECK(g.allow_collapse(.29)); // materially worse: bump again
	g.collapse_bumped(.29);
	g.observe_collapse_gap(.4); // responded
	CHECK(g.allow_collapse(.3));
	g.new_step(1.);
	CHECK(g.allow_collapse(.6));
	CHECK(g.allow_collapse(std::numeric_limits<double>::quiet_NaN()));
}

TEST_CASE("Pair-basis band guard sees the pair the proxy relaxes", "[trim_controller][ef07]")
{
	// The ball-burst step-42 cycle: minimum pair gap .08 (proxy 10x = .8),
	// band proposal .796. The proxy basis lets it through; on the pair's own
	// gap against sqrt(.5 / 100) the scalar force law needs >= .891.
	const double collapse_pair = std::sqrt(.5 / 100.);
	CHECK(ForceWeightedTrim::safe_band_step(.796, 10 * .08, std::sqrt(.5)));
	CHECK_FALSE(ForceWeightedTrim::safe_band_step(.796, .08, collapse_pair));
	CHECK(ForceWeightedTrim::safe_band_step(.9, .08, collapse_pair));
	CHECK(ForceWeightedTrim::force(.08) / ForceWeightedTrim::force(collapse_pair) == Catch::Approx(.8913).epsilon(1e-3));
	// Min gap .0991: the proxy allows any factor, the pair basis >= .732.
	CHECK(ForceWeightedTrim::safe_band_step(.25, .991, std::sqrt(.5)));
	CHECK_FALSE(ForceWeightedTrim::safe_band_step(.6, .0991, collapse_pair));
	CHECK(ForceWeightedTrim::safe_band_step(.74, .0991, collapse_pair));
}

TEST_CASE("Loop guard options are named errors when invalid", "[trim_controller][ef07]")
{
	const auto mesh = one_pair_mesh();
	for (const json &opts : {json{{"collapse_guard_basis", "min"}}, json{{"trim_step_excursion", .5}},
							 json{{"trim_step_excursion", 1.}}, json{{"trim_reversal_limit", -1}},
							 json{{"initial_trim_estimate_scope", "solve"}}})
		CHECK_THROWS(GuardForm(mesh, opts));
	CHECK_NOTHROW(GuardForm(mesh, json{{"collapse_guard_basis", "pair"}, {"trim_step_excursion", 256.}, {"trim_reversal_limit", 3}}));
}

TEST_CASE("An unresponsive collapse is bumped once with the veto, repeatedly without", "[trim_controller][ef07]")
{
	const auto plain = pinned_trims(json{{"band_statistic", "force_weighted"}}, 12);
	const auto vetoed = pinned_trims(json{{"band_statistic", "force_weighted"}, {"collapse_responsiveness_veto", true}}, 12);
	// Without the veto the in-solve climb bumps every cooldown (3) until the
	// 256x budget above the refresh anchor is spent.
	CHECK(plain.back() > 8 * plain.front());
	// With it the first bump happens; the pinned gap did not rise, so the
	// following ones are skipped.
	CHECK(vetoed.back() > vetoed.front());
	CHECK(vetoed.back() == vetoed[4]);
	CHECK(vetoed.back() < plain.back());
}

TEST_CASE("Step excursion bound caps the pinned climb", "[trim_controller][ef07]")
{
	const auto bounded = pinned_trims(json{{"band_statistic", "force_weighted"}, {"trim_step_excursion", 4.}}, 30);
	for (const double t : bounded)
		CHECK(t <= 4 * bounded.front() * (1 + 1e-12));
}

TEST_CASE("Minimum safe factor matches the band guard", "[trim_controller][ef07]")
{
	const double pair = std::sqrt(.5 / 100.);
	for (const double g : {.05, .0707, .08, .0991, .2, .5, .99})
		for (const double f : {.25, .5, .75, .9, .95})
		{
			const double m = ForceWeightedTrim::min_safe_factor(g, pair);
			CHECK(ForceWeightedTrim::safe_band_step(f, g, pair) == (f >= m));
		}
	CHECK(ForceWeightedTrim::min_safe_factor(.06, pair) == 1.);
	CHECK(ForceWeightedTrim::min_safe_factor(1.2, pair) == 0.);
	CHECK(ForceWeightedTrim::min_safe_factor(.08, pair) == Catch::Approx(.8913).epsilon(1e-3));
}

TEST_CASE("The force-weighted band defaults to the pair-basis guard", "[trim_controller][ef07]")
{
	// User decision 2026-09-28: pair is enforced for the experimental
	// force-weighted band; production rms has no band guard and its manifest
	// carries no EF-07 block.
	const auto mesh = one_pair_mesh();
	const json fw = GuardForm(mesh, json{{"band_statistic", "force_weighted"}}).model_description();
	REQUIRE(fw["coefficient_law"]["controller"].contains("ef07"));
	CHECK(fw["coefficient_law"]["controller"]["ef07"]["collapse_guard_basis"] == "pair");
	const json proxy = GuardForm(mesh, json{{"band_statistic", "force_weighted"}, {"collapse_guard_basis", "proxy"}}).model_description();
	CHECK_FALSE(proxy["coefficient_law"]["controller"].contains("ef07"));
	const json rms = GuardForm(mesh, json{{"band_statistic", "rms"}}).model_description();
	CHECK_FALSE(rms["coefficient_law"]["controller"].contains("ef07"));
	const json rms_default = GuardForm(mesh, json::object()).model_description();
	CHECK_FALSE(rms_default["coefficient_law"]["controller"].contains("ef07"));
}
