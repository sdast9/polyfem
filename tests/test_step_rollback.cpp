// RB-06: failed-attempt state isolation and rollback.
//
// A nonlinear solve attempt mutates the caller's solution and the forms'
// attempt state (contact snapshot, trim and memo, swept candidates, friction
// lag, AL multipliers and weights, lagged fields). Before RB-06 a failure
// left all of it where the attempt died: the caller's `sol` held the failed
// iterate and the forms carried the failed attempt's coefficients and lags
// (RB-04's failure record documents exactly that state). The step
// transaction captures every form's state at solve start (Form::save_state,
// FullNLProblem::save_state) and puts it back when the attempt fails, so the
// in-memory state after a failure equals the last accepted state, no success
// callback or output of the failed step fires, and a solve from the restored
// state -- a test action here, never a production retry (RB-08) -- equals a
// fresh control given identical inputs.
//
// Form-level round trips pin each state type; the scene-level test drives
// the real transient loop with the failure-injection hook
// (solver/advanced/failure_injection) through every injection phase.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/solver/FailureInjection.hpp>
#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/solver/forms/LaggedRegForm.hpp>
#include <polyfem/solver/forms/NormalAdhesionForm.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/Logger.hpp>

#include "VarFormTestAccess.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace polyfem;
using namespace polyfem::solver;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace
{
	// 2D: floor edge (0,1) from -1 to 1 at y = 0; free point 2 above it.
	// Node-major DOFs: point x = 4, point y = 5.
	ipc::CollisionMesh make_mesh(const double height = .2)
	{
		Eigen::MatrixXd p(3, 2);
		Eigen::MatrixXi e(1, 2);
		p.row(0) << -1, 0;
		p.row(1) << 1, 0;
		p.row(2) << 0, height;
		e.row(0) << 0, 1;
		return ipc::CollisionMesh(p, e);
	}

	Eigen::VectorXd lowered(const double dy)
	{
		Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
		x[5] = dy;
		return x;
	}

	// Semi-implicit barrier on the fixture above with a synthetic driving
	// Hessian (as tools/rb10/friction_probe.cpp and test_friction_lag.cpp).
	class Probe : public BarrierContactForm
	{
	public:
		Eigen::MatrixXd driving;
		Probe(const ipc::CollisionMesh &m, const json &opts = json::object())
			: BarrierContactForm(m, /*dhat=*/1, /*avg_mass=*/1, false, false, false, true, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts, Eigen::VectorXd::Ones(m.num_vertices()))
		{
			driving = 100 * Eigen::MatrixXd::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const Eigen::VectorXd &, StiffnessMatrix &h) { h = driving.sparseView(); });
		}
		void start(const Eigen::VectorXd &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false, false);
			refresh_semi_implicit_stiffness(x, false, true);
		}
		bool swept_cache_active() const { return use_cached_candidates_; }
		size_t memo_size() const { return kappa_cache_.size(); }
		size_t endpoint_memo_size() const { return endpoint_kappa_.size(); }
		const Eigen::MatrixXd &frozen_surface() const { return kappa_surface_; }
		double frozen_hessian_max() const { return kappa_hessian_max_; }
	};

	struct BarrierObservation
	{
		json state;
		double trim = 0;
		double value = 0;
		Eigen::VectorXd gradient;
		size_t memo = 0, endpoint_memo = 0;
		Eigen::MatrixXd surface;
		double hessian_max = 0;
	};

	BarrierObservation observe(Probe &f, const Eigen::VectorXd &x)
	{
		BarrierObservation o;
		o.state = f.diagnostic_state();
		o.trim = f.barrier_stiffness();
		// A private rebuild at x: value/gradient under the form's coefficient
		// state without touching its own collision set.
		const auto snapshot = f.diagnostic_snapshot(x);
		o.value = snapshot.value(x);
		snapshot.first_derivative(x, o.gradient);
		o.memo = f.memo_size();
		o.endpoint_memo = f.endpoint_memo_size();
		o.surface = f.frozen_surface();
		o.hessian_max = f.frozen_hessian_max();
		return o;
	}

	void check_same(const BarrierObservation &a, const BarrierObservation &b)
	{
		CHECK(a.state == b.state);
		CHECK(a.trim == b.trim);
		CHECK(a.value == b.value);
		CHECK(a.gradient == b.gradient);
		CHECK(a.memo == b.memo);
		CHECK(a.endpoint_memo == b.endpoint_memo);
		CHECK(a.surface == b.surface);
		CHECK(a.hessian_max == b.hessian_max);
	}
} // namespace

TEST_CASE("BarrierContactForm state round trip through every mutation path", "[rollback][contact_form]")
{
	auto m = make_mesh();
	Probe f(m);
	const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
	const Eigen::VectorXd x1 = lowered(-.1);
	const Eigen::VectorXd x2 = lowered(-.15);
	f.start(x0);
	REQUIRE(f.collision_set().size() == 1);
	const BarrierObservation at_start = observe(f, x0);
	const BarrierObservation at_start_x1 = observe(f, x1);

	// The transaction boundary: capture, then mutate through every path an
	// attempt uses -- a trial rebuild, an in-solve controller step, a bump,
	// a stall retune (refresh at new coordinates + trim), a swept cache left
	// active, a published-endpoint refresh (continuation seeds).
	std::vector<json> events;
	f.set_coefficient_observer([&](const json &e) { events.push_back(e); });
	auto saved = f.save_state();
	REQUIRE(saved);

	f.line_search_begin(x0, x1);
	f.solution_changed(x1);
	f.post_step(polysolve::nonlinear::PostStepData(4, json::object(), x1, Eigen::VectorXd::Zero(6)));
	f.bump_trim(8.);
	REQUIRE(f.barrier_stiffness() == Approx(8 * at_start.trim));
	f.retune_on_stall(x2, 2.);
	f.refresh_semi_implicit_stiffness(x2, true, true);
	REQUIRE(f.swept_cache_active());
	const BarrierObservation mutated = observe(f, x0);
	// The mutation is real: different coefficient state and objective at x0.
	CHECK(mutated.state != at_start.state);
	CHECK(mutated.value != at_start.value);

	const size_t events_before_rollback = events.size();
	f.restore_state(*saved, x0);
	check_same(observe(f, x0), at_start);
	check_same(observe(f, x1), at_start_x1);
	CHECK(!f.swept_cache_active());
	CHECK(f.collision_set().size() == 1);

	// The rollback is a coefficient event: before = the failed state at x0,
	// after = the restored state, both measured at the restored coordinates.
	REQUIRE(events.size() == events_before_rollback + 1);
	const json &rollback = events.back();
	CHECK(rollback["operation"] == "rollback");
	CHECK(rollback["before"]["objective"].get<double>() == Approx(mutated.value));
	CHECK(rollback["after"]["objective"].get<double>() == Approx(at_start.value));
	CHECK(rollback["after"]["state"] == at_start.state);
	CHECK(rollback["operation_threw"] == false);

	// The restored form keeps working: a new trial and refresh behave as at
	// the start (same memo growth, same values), i.e. the caches are coherent.
	f.line_search_begin(x0, x1);
	f.solution_changed(x1);
	CHECK(f.value(x1) == Approx(at_start_x1.value));
	f.line_search_end();
	CHECK(!f.swept_cache_active());
}

TEST_CASE("Form states are typed: a state from another form is refused", "[rollback][contact_form]")
{
	auto m = make_mesh();
	Probe f(m);
	f.start(Eigen::VectorXd::Zero(6));
	FrictionForm fr(m, nullptr, 1e-3, .3, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
	fr.init_lagging(Eigen::VectorXd::Zero(6));
	auto friction_state = fr.save_state();
	CHECK_THROWS_WITH(f.restore_state(*friction_state, Eigen::VectorXd::Zero(6)), ContainsSubstring("different form type"));
	auto barrier_state = f.save_state();
	CHECK_THROWS_WITH(fr.restore_state(*barrier_state, Eigen::VectorXd::Zero(6)), ContainsSubstring("different form type"));
}

TEST_CASE("FrictionForm lag round trip", "[rollback][friction_form]")
{
	auto m = make_mesh();
	Probe f(m);
	const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
	const Eigen::VectorXd x1 = lowered(-.12);
	f.start(x0);
	FrictionForm fr(m, nullptr, 1e-3, .3, ipc::BroadPhaseMethod::HASH_GRID, f, 2);
	fr.init_lagging(x0);
	REQUIRE(fr.friction_collision_set().size() == 1);
	const double N0 = fr.friction_collision_set()[0].normal_force_magnitude;
	Eigen::VectorXd slip = x0;
	slip[4] = 5e-3;
	Eigen::VectorXd g0;
	fr.first_derivative(slip, g0);

	auto saved = fr.save_state();
	// The lagging loop of a failed attempt: the lag rebuilt at another
	// iterate with a moved trim (a larger normal force), then the failure.
	f.bump_trim(4.);
	f.solution_changed(x1);
	fr.update_lagging(x1, 1);
	const double N1 = fr.friction_collision_set()[0].normal_force_magnitude;
	REQUIRE(N1 != N0);
	Eigen::VectorXd g1;
	fr.first_derivative(slip, g1);
	REQUIRE(g1 != g0);

	fr.restore_state(*saved, x0);
	CHECK(fr.friction_collision_set().size() == 1);
	CHECK(fr.friction_collision_set()[0].normal_force_magnitude == N0);
	Eigen::VectorXd g;
	fr.first_derivative(slip, g);
	// The barrier's trim is the barrier's state; in the default realized
	// mode the lag does not follow it, so the restored lag alone gives g0.
	CHECK(g == g0);
}

TEST_CASE("BCLagrangianForm multiplier and weight round trip", "[rollback][al_solver]")
{
	StiffnessMatrix mass(3, 3);
	mass.setIdentity();
	const std::vector<int> boundary{0, 2};
	Eigen::VectorXd target(3);
	target << .3, 0, -.4;
	BCLagrangianForm form(3, boundary, mass, 0, target);
	form.set_initial_weight(1);
	Eigen::VectorXd x(3);
	x << 1, 2, 3;
	form.update_lagrangian(x, 2.);
	REQUIRE(form.lagrangian_weight() == 2.);
	REQUIRE(form.lagrange_multipliers().norm() > 0);
	const Eigen::VectorXd mults = form.lagrange_multipliers();
	const double v = form.value(x);

	auto saved = form.save_state();
	form.update_lagrangian(x, 8.);
	form.set_scale(3.);
	REQUIRE(form.lagrange_multipliers() != mults);
	REQUIRE(form.value(x) != v);

	form.restore_state(*saved, x);
	CHECK(form.lagrangian_weight() == 2.);
	CHECK(form.lagrange_multipliers() == mults);
	CHECK(form.value(x) == v);
}

TEST_CASE("LaggedRegForm lag and enabled flag round trip", "[rollback]")
{
	LaggedRegForm form(/*n_lagging_iters=*/1);
	Eigen::VectorXd x0(2), x1(2);
	x0 << 1, 1;
	x1 << 3, -2;
	form.init_lagging(x0);
	REQUIRE(form.enabled());
	const double v = form.value(x1);
	auto saved = form.save_state();
	form.update_lagging(x1, 1); // disables the form and moves the lag
	REQUIRE(!form.enabled());
	form.restore_state(*saved, x0);
	CHECK(form.enabled());
	CHECK(form.value(x1) == v);
}

TEST_CASE("NormalAdhesionForm owns its collision set and does not treat an abandoned swept cache as complete", "[rollback][contact_cache]")
{
	// The RB-01 and RB-05 contracts applied to the adhesion form. RB-01: the
	// upstream function-static position cache let any instance in the
	// process (here: the instance of the previous section run) suppress this
	// instance's own rebuild at equal positions -- the first init of a fresh
	// form at coordinates in contact must build its own set. RB-05: after an
	// exception escapes a line search (no line_search_end), the next init
	// must not build the collision set from the stale swept candidates.
	auto mesh = make_mesh(.2);
	const double dhat_p = .05, dhat_a = .1, Y = 1.;
	const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
	const Eigen::VectorXd in_contact = lowered(-.15); // gap .05 < dhat_a
	{
		// Another instance evaluates the same positions first.
		NormalAdhesionForm other(mesh, dhat_p, dhat_a, Y, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000);
		other.init(in_contact);
		REQUIRE(other.collision_set().size() == 1);
	}
	NormalAdhesionForm fresh(mesh, dhat_p, dhat_a, Y, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000);
	fresh.init(in_contact);
	REQUIRE(fresh.collision_set().size() == 1);

	NormalAdhesionForm form(mesh, dhat_p, dhat_a, Y, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000);
	form.init(zero);
	REQUIRE(form.collision_set().empty());
	form.line_search_begin(zero, lowered(.5)); // a sweep away: empty candidates
	SECTION("a retry re-initializes at coordinates in contact")
	{
		form.init(in_contact);
		CHECK(form.collision_set().size() == fresh.collision_set().size());
		CHECK(form.value(in_contact) == Approx(fresh.value(in_contact)));
	}
	SECTION("a new step updates at coordinates in contact")
	{
		form.update_quantities(0., in_contact);
		CHECK(form.collision_set().size() == fresh.collision_set().size());
	}
	SECTION("the state round trip carries the collision set")
	{
		form.init(in_contact);
		auto saved = form.save_state();
		form.init(zero);
		REQUIRE(form.collision_set().empty());
		form.restore_state(*saved, in_contact);
		CHECK(form.collision_set().size() == 1);
		CHECK(form.value(in_contact) == Approx(fresh.value(in_contact)));
	}
}

// ---------------------------------------------------------------------------
// Scene level: the real transient loop with injected failures.

namespace
{
	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	json read_json(const std::filesystem::path &path)
	{
		std::ifstream file(path);
		REQUIRE(file.is_open());
		json j;
		file >> j;
		return j;
	}

	// The public transient contact fixture (cube on a slab, semi-implicit
	// barrier, prescribed top face) with friction, single-threaded, three
	// steps, diagnostics on. Every scenario runs the same inputs.
	json scene_args(const std::filesystem::path &out_dir, const json &failure_injection)
	{
		const std::filesystem::path scene = std::filesystem::path(POLYFEM_TEST_DIR) / ".." / "scenes" / "semi-implicit" / "transient-semi.json";
		json args = read_json(scene);
		args["root_path"] = scene.string();
		args["contact"]["friction_coefficient"] = .3;
		args["time"] = {{"dt", .25}, {"time_steps", 3}, {"quasistatic", false}};
		args["/solver/max_threads"_json_pointer] = 1;
		args["/solver/advanced/failure_injection"_json_pointer] = failure_injection;
		args["/output/directory"_json_pointer] = out_dir.string();
		args["/output/log/level"_json_pointer] = "warning";
		args["/output/paraview/file_name"_json_pointer] = "run.pvd";
		args["/output/physical_diagnostics"_json_pointer] = true;
		args["/output/manifest"_json_pointer] = "run-manifest.json";
		return args;
	}

	struct Run
	{
		std::unique_ptr<State> state;
		varform::NonlinearElasticTransientVarForm *form = nullptr;
		Eigen::MatrixXd sol;
		std::vector<int> callbacks; ///< steps the ForwardStepCallback reported
		std::vector<Eigen::VectorXd> endpoints;
		std::vector<json> fingerprints; ///< fingerprint after each accepted, advanced step

		void begin(const json &args)
		{
			state = std::make_unique<State>();
			state->init(args, true);
			state->set_max_threads(1);
			state->load_mesh();
			form = dynamic_cast<varform::NonlinearElasticTransientVarForm *>(state->variational_formulation.get());
			REQUIRE(form != nullptr);
			test::VarFormTestAccess::prepare(*form);
			test::VarFormTestAccess::begin_transient_run(*form, sol, [this](int step, const Eigen::MatrixXd &) { callbacks.push_back(step); });
		}
		varform::ForwardStepCallback callback()
		{
			return [this](int step, const Eigen::MatrixXd &) { callbacks.push_back(step); };
		}
		void solve(int t) { test::VarFormTestAccess::solve_transient_step(*form, t, sol, callback()); }
		void advance(int t)
		{
			test::VarFormTestAccess::advance_transient_step(*form, t, sol);
			endpoints.push_back(sol);
			fingerprints.push_back(test::VarFormTestAccess::attempt_state_fingerprint(*form, sol));
		}
		void end() { test::VarFormTestAccess::end_transient_run(*form); }
	};

	json injection(const std::string &phase, const int step, const int iteration = 0, const std::string &kind = "named")
	{
		return {{"phase", phase}, {"step", step}, {"iteration", iteration}, {"kind", kind}};
	}
} // namespace

TEST_CASE("A failed step attempt is rolled back and a solve from the restored state equals a fresh control", "[rollback][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rb06-rollback");

	// Control: the loop as production runs it, without injection.
	Run control;
	control.begin(scene_args(dir / "control", json::object()));
	for (int t = 1; t <= 3; ++t)
	{
		control.solve(t);
		control.advance(t);
	}
	control.end();
	REQUIRE(control.endpoints.size() == 3);
	REQUIRE(control.callbacks == std::vector<int>{0, 1, 2, 3});

	// The failure phases of the plan: after AL, after a coefficient retune,
	// inside the reduced solve, during friction lagging. Each scenario fails
	// step 2 once; the test then solves step 2 again from the restored state
	// and continues, and every endpoint must equal the control's.
	struct Scenario
	{
		std::string name;
		json injection;
		json extra = json::object();
	};
	const std::vector<Scenario> scenarios = {
		{"after_al", injection("after_al", 2)},
		{"reduced-iteration-1", injection("reduced", 2, 1)},
		{"reduced-runtime-error", injection("reduced", 2, 1, "runtime_error")},
		{"lagging-1", injection("lagging", 2, 1)},
	};
	for (const Scenario &scenario : scenarios)
	{
		DYNAMIC_SECTION(scenario.name)
		{
			json args = scene_args(dir / scenario.name, scenario.injection);
			for (const auto &[key, value] : scenario.extra.items())
				args[json::json_pointer(key)] = value;
			Run run;
			run.begin(args);
			run.solve(1);
			run.advance(1);
			REQUIRE(run.endpoints[0] == control.endpoints[0]);

			const Eigen::VectorXd before = run.sol;
			const json fingerprint_before = test::VarFormTestAccess::attempt_state_fingerprint(*run.form, before);
			const auto &integrator = *test::VarFormTestAccess::solve_data(*run.form).time_integrator;
			const Eigen::VectorXd x_prev = integrator.x_prev(), v_prev = integrator.v_prev(), a_prev = integrator.a_prev();
			CHECK_THROWS_WITH(run.solve(2), ContainsSubstring("Injected failure"));

			// Authoritative state after the failure = the last accepted state.
			CHECK(run.sol == before);
			CHECK(test::VarFormTestAccess::attempt_state_fingerprint(*run.form, run.sol) == fingerprint_before);
			CHECK(integrator.x_prev() == x_prev);
			CHECK(integrator.v_prev() == v_prev);
			CHECK(integrator.a_prev() == a_prev);
			// No success callback for the failed attempt.
			CHECK(run.callbacks == std::vector<int>{0, 1});
			// The failure is on record (RB-04 record and manifest), with the
			// rollback announced and verified.
			{
				std::ifstream stream(dir / scenario.name / "physical-diagnostics.jsonl");
				std::string line, last;
				while (std::getline(stream, line))
					if (!line.empty())
						last = line;
				const json record = json::parse(last);
				CHECK(record["outcome"] == "failed_attempt");
				CHECK(record["step"] == 2);
				CHECK(record["rollback"]["performed"] == "after this record");
				REQUIRE(record["solve_start"]["value"].is_array());
				const auto start = record["solve_start"]["value"].get<std::vector<double>>();
				REQUIRE(int(start.size()) == before.size());
				for (int i = 0; i < before.size(); ++i)
					CHECK(start[i] == before[i]);
				const json manifest = read_json(dir / scenario.name / "run-manifest.json");
				REQUIRE(manifest["steps"].size() == 2);
				CHECK(manifest["steps"][1]["outcome"] == "failed_attempt");
				CHECK(manifest["steps"][1]["rollback"]["performed"] == true);
				CHECK(manifest["steps"][1]["rollback"]["verified"] == true);
			}

			// The test's re-solve from the restored state (not a retry policy).
			test::VarFormTestAccess::set_failure_injection(*run.form, FailureInjection());
			run.solve(2);
			run.advance(2);
			run.solve(3);
			run.advance(3);
			run.end();
			REQUIRE(run.endpoints.size() == 3);
			for (int t = 0; t < 3; ++t)
			{
				INFO("step " << t + 1 << " max |difference| = " << (run.endpoints[t] - control.endpoints[t]).lpNorm<Eigen::Infinity>());
				CHECK(run.endpoints[t] == control.endpoints[t]);
				CHECK(run.fingerprints[t] == control.fingerprints[t]);
			}
			CHECK(run.callbacks == std::vector<int>{0, 1, 2, 3});
		}
	}
}

TEST_CASE("A stall retune followed by a failure is rolled back with the retune", "[rollback][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rb06-stall");
	// A soft stall budget that forces restarts (with their retunes) and still
	// converges: the control runs it too, so the only difference is the throw.
	const json restart = {{"enabled", true}, {"soft_iteration_limit", 4}, {"min_iterations", 0}, {"max_restarts", 8}};
	Run control;
	{
		json args = scene_args(dir / "control", json::object());
		args["/solver/contact/semi_implicit/restart"_json_pointer] = restart;
		control.begin(args);
		for (int t = 1; t <= 3; ++t)
		{
			control.solve(t);
			control.advance(t);
		}
		control.end();
	}
	json args = scene_args(dir / "injected", injection("after_stall_retune", 2));
	args["/solver/contact/semi_implicit/restart"_json_pointer] = restart;
	Run run;
	run.begin(args);
	run.solve(1);
	run.advance(1);
	const Eigen::VectorXd before = run.sol;
	const json fingerprint_before = test::VarFormTestAccess::attempt_state_fingerprint(*run.form, before);
	CHECK_THROWS_WITH(run.solve(2), ContainsSubstring("after the barrier stiffness retune"));
	CHECK(run.sol == before);
	CHECK(test::VarFormTestAccess::attempt_state_fingerprint(*run.form, run.sol) == fingerprint_before);
	const json manifest = read_json(dir / "injected" / "run-manifest.json");
	REQUIRE(manifest["steps"].size() == 2);
	CHECK(manifest["steps"][1]["stall_retunes"].get<int>() >= 1);
	CHECK(manifest["steps"][1]["rollback"]["verified"] == true);
	test::VarFormTestAccess::set_failure_injection(*run.form, FailureInjection());
	run.solve(2);
	run.advance(2);
	run.solve(3);
	run.advance(3);
	run.end();
	for (int t = 0; t < 3; ++t)
	{
		INFO("step " << t + 1 << " max |difference| = " << (run.endpoints[t] - control.endpoints[t]).lpNorm<Eigen::Infinity>());
		CHECK(run.endpoints[t] == control.endpoints[t]);
	}
}

TEST_CASE("A publication failure after an accepted solve publishes nothing of the step and fires no callback", "[rollback][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rb06-publication");
	Run run;
	run.begin(scene_args(dir / "injected", injection("before_publication", 2)));
	run.solve(1);
	run.advance(1);
	const Eigen::VectorXd before = run.sol;
	const auto &integrator = *test::VarFormTestAccess::solve_data(*run.form).time_integrator;
	const Eigen::VectorXd x_prev = integrator.x_prev();
	CHECK_THROWS_WITH(run.solve(2), ContainsSubstring("before the step callback"));
	// The solve was accepted (RB-04 boundary) -- the solution moved -- but
	// nothing of step 2 is published and the history has not advanced.
	CHECK(run.sol != before);
	CHECK(integrator.x_prev() == x_prev);
	CHECK(run.callbacks == std::vector<int>{0, 1});
	CHECK(!std::filesystem::exists(dir / "injected" / "step_2.vtu"));
	CHECK(!std::filesystem::exists(dir / "injected" / "step_2.vtm"));
	const std::string pvd = [&]() {
		std::ifstream file(dir / "injected" / "run.pvd");
		return std::string(std::istreambuf_iterator<char>(file), {});
	}();
	CHECK(pvd.find("step_1.vtm") != std::string::npos);
	CHECK(pvd.find("step_2.vtm") == std::string::npos);
	const json manifest = read_json(dir / "injected" / "run-manifest.json");
	REQUIRE(manifest["steps"].size() == 2);
	CHECK(manifest["steps"][1]["outcome"] == "accepted");
	CHECK(!manifest["steps"][1].contains("rollback"));
}
