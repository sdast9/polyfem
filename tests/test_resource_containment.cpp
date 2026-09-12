// RB-05: candidate generation and resource failure containment.
//
// The swept candidate cache a contact form builds in line_search_begin is
// only valid for the line search that built it. PolySolve's line search has
// no unwind guard, so an exception escaping between line_search_begin and
// line_search_end (an allocation failure in the broad phase, a resource
// budget) leaves the cache marked active. Every retry path in the fork
// (ALSolver's scaled-weight retry, the stall restart) re-enters through
// ContactForm::init, which must never build the collision set from that
// partial or stale sweep: a candidate set that was never completed for the
// current coordinates cannot be treated as complete.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>

#include <ipc/broad_phase/hash_grid.hpp>

#include <stdexcept>
#include <type_traits>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	// 2D: one edge (-1,0)-(1,0) and a free vertex at (0, .2). With dhat = .1
	// the static candidate set at x = 0 is empty (gap .2 > dhat/2 inflation)
	// and a vertex displaced to gap .05 is one edge-vertex collision.
	const double support = .1; // dhat
	const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);

	ipc::CollisionMesh make_mesh()
	{
		Eigen::MatrixXd vertices(3, 2);
		vertices << -1, 0, 1, 0, 0, .2;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		return ipc::CollisionMesh(vertices, edges);
	}

	class ProbeForm : public BarrierContactForm
	{
	public:
		ProbeForm(const ipc::CollisionMesh &mesh, const ipc::BroadPhaseMethod method = ipc::BroadPhaseMethod::HASH_GRID)
			: BarrierContactForm(mesh, support, 1., false, false, false, false, false, false,
								 method, 1e-8, 1000000, BarrierStiffnessMode::Fixed)
		{
			set_barrier_stiffness(1.);
		}
		bool swept_cache_active() const { return use_cached_candidates_; }
	};

	Eigen::VectorXd displaced_vertex(const double dy)
	{
		Eigen::VectorXd x = zero;
		x[5] = dy;
		return x;
	}
} // namespace

TEST_CASE("A swept candidate cache left by an escaped line search is not treated as complete", "[resource_containment][contact_cache]")
{
	const auto mesh = make_mesh();
	const Eigen::VectorXd in_contact = displaced_vertex(-.15); // gap .05 < dhat
	ProbeForm fresh(mesh);
	fresh.init(in_contact);
	REQUIRE(fresh.collision_set().size() == 1);
	REQUIRE(fresh.value(in_contact) > 0);

	ProbeForm form(mesh);
	form.init(zero);
	REQUIRE(form.collision_set().empty());

	// A trial sweep away from the edge: no candidate lies within dhat/2 of
	// the swept vertex, so the cache is legitimately empty for that trial.
	form.line_search_begin(zero, displaced_vertex(.5));
	REQUIRE(form.swept_cache_active());
	REQUIRE(form.candidate_statistics().last == 0);

	// The line search is abandoned by an exception thrown after the build
	// (PolySolve calls no line_search_end while unwinding): nothing ends the
	// cached interval before the solver retries from the accepted state.
	SECTION("retry re-initializes the form at coordinates in contact")
	{
		form.init(in_contact);
		CHECK(!form.swept_cache_active());
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		CHECK(form.value(in_contact) == Catch::Approx(fresh.value(in_contact)));
		Eigen::VectorXd g, g_fresh;
		form.first_derivative(in_contact, g);
		fresh.first_derivative(in_contact, g_fresh);
		CHECK((g - g_fresh).norm() <= 1e-12 * (1 + g_fresh.norm()));
	}
	SECTION("a new time step updates the form at coordinates in contact")
	{
		form.update_quantities(0., in_contact);
		CHECK(!form.swept_cache_active());
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		CHECK(form.value(in_contact) == Catch::Approx(fresh.value(in_contact)));
	}
	SECTION("the next line search rebuilds its own sweep")
	{
		// Independent of the fix above: a new line_search_begin always
		// replaces the cache, and the collision set at the new coordinates
		// then comes from a sweep that contains them.
		form.line_search_begin(zero, in_contact);
		form.solution_changed(in_contact);
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		form.line_search_end();
		CHECK(!form.swept_cache_active());
	}
}

TEST_CASE("A broad-phase resource limit fails before allocation and leaves no partial candidate set", "[resource_containment]")
{
	// A resource failure is not a solver step failure: the handlers that
	// retry a std::runtime_error (smaller step, scaled AL weight, stall
	// restart) must not absorb it.
	static_assert(!std::is_base_of_v<std::runtime_error, ipc::BroadPhaseBudgetExceeded>);
	static_assert(std::is_base_of_v<std::exception, ipc::BroadPhaseBudgetExceeded>);

	const auto mesh = make_mesh();
	const Eigen::VectorXd in_contact = displaced_vertex(-.15);
	ProbeForm fresh(mesh);
	fresh.init(in_contact);
	REQUIRE(fresh.collision_set().size() == 1);

	SECTION("cell-item limit reached during a static build; repeat invocation after raising it")
	{
		ProbeForm form(mesh);
		ipc::BroadPhaseBudget budget;
		budget.max_cell_items = 1; // three vertex boxes and one edge box cover more than one cell
		form.set_broad_phase_budget(budget);
		REQUIRE(form.broad_phase_budget().max_cell_items == 1);
		try
		{
			form.init(in_contact);
			FAIL("the budget did not fire");
		}
		catch (const ipc::BroadPhaseBudgetExceeded &e)
		{
			CHECK(e.quantity == "cell_items");
			CHECK(e.limit == 1);
			CHECK(e.requested > e.limit);
			CHECK_THAT(e.what(), Catch::Matchers::ContainsSubstring("before allocation"));
		}
		CHECK(form.collision_set().empty()); // never built, never partially filled
		CHECK(!form.swept_cache_active());
		CHECK(form.candidate_statistics().builds == 0);

		budget.max_cell_items = 1000;
		form.set_broad_phase_budget(budget);
		form.init(in_contact);
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		CHECK(form.value(in_contact) == Catch::Approx(fresh.value(in_contact)));
		CHECK(form.candidate_statistics().intermediates_measured == false); // init is not a trial sweep
	}
	SECTION("limit reached during a trial sweep: the interval is not entered and the retry sees a complete set")
	{
		ProbeForm form(mesh);
		form.init(zero);
		ipc::BroadPhaseBudget budget;
		budget.max_candidate_emissions = 1; // the edge shares cells with all three vertices before filtering
		form.set_broad_phase_budget(budget);
		try
		{
			form.line_search_begin(zero, in_contact);
			FAIL("the budget did not fire");
		}
		catch (const ipc::BroadPhaseBudgetExceeded &e)
		{
			CHECK(e.quantity == "candidate_emissions");
			CHECK(e.requested > 1);
		}
		CHECK(!form.swept_cache_active());
		CHECK(form.candidate_statistics().builds == 0);
		CHECK(form.candidate_statistics().last == 0);
		// The abandoned sweep is not a candidate set: the retry from the
		// accepted state builds a complete collision set at the new coordinates.
		form.set_broad_phase_budget(ipc::BroadPhaseBudget());
		form.init(in_contact);
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		CHECK(form.value(in_contact) == Catch::Approx(fresh.value(in_contact)));
		// And a later line search works normally.
		form.line_search_begin(in_contact, zero);
		CHECK(form.swept_cache_active());
		CHECK(form.candidate_statistics().builds == 1);
		form.line_search_end();
	}
	SECTION("a generous limit is equivalent to no limit, and reports the intermediates")
	{
		ProbeForm unlimited(mesh), limited(mesh);
		ipc::BroadPhaseBudget budget;
		budget.max_cell_items = 1 << 20;
		budget.max_candidate_emissions = 1 << 20;
		limited.set_broad_phase_budget(budget);
		for (ProbeForm *form : {&unlimited, &limited})
		{
			form->init(zero);
			form->line_search_begin(zero, in_contact);
			form->solution_changed(in_contact);
		}
		CHECK(unlimited.candidate_statistics().last == limited.candidate_statistics().last);
		CHECK(unlimited.collision_set().size() == limited.collision_set().size());
		CHECK(unlimited.value(in_contact) == limited.value(in_contact));
		Eigen::VectorXd g0, g1;
		unlimited.first_derivative(in_contact, g0);
		limited.first_derivative(in_contact, g1);
		CHECK(g0 == g1);
		// The hash grid measures its items on every build; emissions are
		// counted only while a limit is enabled.
		const auto &s0 = unlimited.candidate_statistics();
		const auto &s1 = limited.candidate_statistics();
		CHECK(s0.intermediates_measured);
		CHECK(s0.last_cell_items > 0);
		CHECK(s0.last_cell_items == s1.last_cell_items);
		CHECK(s0.last_candidate_emissions == 0);
		CHECK(s1.last_candidate_emissions > 0);
		CHECK(s1.max_candidate_emissions == s1.last_candidate_emissions);
		const json state = limited.diagnostic_state();
		CHECK(state["candidate_count"]["scope"] == "Active swept candidate cache");
		unlimited.line_search_end();
		limited.line_search_end();
		const json ended = limited.diagnostic_state()["candidate_count"];
		CHECK(ended["broad_phase_intermediates"]["cell_items"]["last"].get<size_t>() == s1.last_cell_items);
		CHECK(ended["broad_phase_intermediates"]["candidate_emissions"]["max"].get<size_t>() == s1.max_candidate_emissions);
	}
	SECTION("a method that cannot enforce the limit is refused at configuration, not ignored")
	{
		ProbeForm bvh(mesh, ipc::BroadPhaseMethod::LBVH);
		ipc::BroadPhaseBudget budget;
		budget.max_cell_items = 10;
		CHECK_THROWS_WITH(bvh.set_broad_phase_budget(budget), Catch::Matchers::ContainsSubstring("cannot be enforced"));
		CHECK(!bvh.broad_phase_budget().enabled());
		bvh.init(in_contact);
		CHECK(bvh.collision_set().size() == fresh.collision_set().size());
		// Brute force bounds its box-pair comparisons.
		ProbeForm brute(mesh, ipc::BroadPhaseMethod::BRUTE_FORCE);
		budget.max_cell_items = 0;
		budget.max_candidate_emissions = 1;
		brute.set_broad_phase_budget(budget);
		CHECK_THROWS_AS(brute.init(in_contact), ipc::BroadPhaseBudgetExceeded);
		budget.max_candidate_emissions = 100;
		brute.set_broad_phase_budget(budget);
		brute.init(in_contact);
		CHECK(brute.collision_set().size() == fresh.collision_set().size());
	}
	SECTION("options are read from solver/contact/CCD/resource_limits: -1 automatic, 0 off, N explicit")
	{
		const auto absent = resource_limits_from_args(json::object());
		CHECK((absent.max_cell_items == -1 && absent.max_candidate_emissions == -1));
		const auto no_key = resource_limits_from_args(json{{"broad_phase", "hash_grid"}});
		CHECK((no_key.max_cell_items == -1 && no_key.max_candidate_emissions == -1));
		const auto partial = resource_limits_from_args(json{{"resource_limits", {{"max_cell_items", 5}}}});
		CHECK((partial.max_cell_items == 5 && partial.max_candidate_emissions == -1));
		const auto off = resource_limits_from_args(json{{"resource_limits", {{"max_cell_items", 0}, {"max_candidate_emissions", 0}}}});
		CHECK((off.max_cell_items == 0 && off.max_candidate_emissions == 0));
		const auto automatic = resource_limits_from_args(json{{"resource_limits", {{"max_cell_items", -1}, {"max_candidate_emissions", -1}}}});
		CHECK((automatic.max_cell_items == -1 && automatic.max_candidate_emissions == -1));
		CHECK_THROWS_WITH(resource_limits_from_args(json{{"resource_limits", {{"max_candidate_emissions", -2}}}}), Catch::Matchers::ContainsSubstring("-1 (automatic)"));
		CHECK_THROWS_WITH(resource_limits_from_args(json{{"resource_limits", {{"max_cell_items", "many"}}}}), Catch::Matchers::ContainsSubstring("-1 (automatic)"));
	}
	SECTION("automatic limits: the production defaults where enforceable, dropped with a notice elsewhere")
	{
		ProbeForm grid(mesh);
		grid.apply_resource_limits(ResourceLimits());
		CHECK(grid.broad_phase_budget().max_cell_items == ContactForm::default_max_cell_items);
		CHECK(grid.broad_phase_budget().max_candidate_emissions == ContactForm::default_max_candidate_emissions);
		grid.init(in_contact);
		CHECK(grid.collision_set().size() == fresh.collision_set().size());

		ProbeForm brute(mesh, ipc::BroadPhaseMethod::BRUTE_FORCE);
		brute.apply_resource_limits(ResourceLimits());
		CHECK(brute.broad_phase_budget().max_candidate_emissions == ContactForm::default_max_candidate_emissions);

		ProbeForm bvh(mesh, ipc::BroadPhaseMethod::LBVH);
		bvh.apply_resource_limits(ResourceLimits()); // no throw: automatic bounds are dropped
		CHECK(!bvh.broad_phase_budget().enabled());
		bvh.init(in_contact);
		CHECK(bvh.collision_set().size() == fresh.collision_set().size());
		ResourceLimits mixed;
		mixed.max_cell_items = -1;
		mixed.max_candidate_emissions = 7; // explicit: refused on a method that cannot enforce it
		CHECK_THROWS_WITH(bvh.apply_resource_limits(mixed), Catch::Matchers::ContainsSubstring("cannot be enforced"));
		CHECK(!bvh.broad_phase_budget().enabled());

		ResourceLimits off;
		off.max_cell_items = 0;
		off.max_candidate_emissions = 0;
		grid.apply_resource_limits(off);
		CHECK(!grid.broad_phase_budget().enabled());
		ResourceLimits custom;
		custom.max_cell_items = 12;
		custom.max_candidate_emissions = 0;
		grid.apply_resource_limits(custom);
		CHECK(grid.broad_phase_budget().max_cell_items == 12);
		CHECK(grid.broad_phase_budget().max_candidate_emissions == 0);
	}
}
