// RBR-01 (RB-04 output kinematics): the exported velocity and acceleration
// are those of the saved solution in every time phase.
//
// The reviewed helper inferred "the history has already advanced to this
// solution" from x_prev() == solution. A held position -- a quasistatic dwell
// whose Newton solve returns in zero iterations, the ordinary case -- satisfies
// that equality without being the history head, and the nonlinear loop saves
// before advancing: the saved fields were the previous step's (v=.4, a=1.6 at
// the first held step of the reproduction below instead of 0 and -1.6). The
// phase is now stated by the integrator's owner (varform::OutputTimePhase).
//
// These tests drive the real nonlinear transient loops on a public prescribed
// motion that moves at step 1, holds for two steps and moves back at the last
// step, and check the saved VTU fields, the before-advance output (what the
// VTU is written from), the step callback and the after-advance output of the
// same solution against the Implicit Euler rule evaluated independently on
// the saved displacements, at every time step. The ordinary staged loop and
// the differentiable transient class (its own solve dispatch when
// differentiable=true, missed by the first RBR-01 repair: the 2026-09-21
// review) are exercised separately.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/OutputData.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/varforms/diff/DifferentiableNonlinearElasticVarForm.hpp>

#include "VarFormTestAccess.hpp"
#include "VtuTestUtils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace polyfem;
using Catch::Approx;

namespace
{
	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	// The public cube of the semi-implicit scenes, contact-free: -z face
	// fixed, +z face sheared by .1 over the first step, held for two steps
	// and moved back to .05 at the last (0.1*min(t/dt, 1) - 0.05*max((t-3dt)/dt, 0)),
	// quasistatic Implicit Euler, dt=.25, four steps, a supplied initial
	// velocity so that the initial export and the first acceleration are not
	// trivially zero, and a final step with nonzero kinematics so that the
	// export after the last advance is not a zero-motion tail.
	// first_grad_norm_tol equals grad_norm_tol so that a held step provably
	// returns in zero Newton iterations (the gradient that ended step 1 is
	// below it), i.e. the held solution is bit-identical to the previous
	// one: the case the reviewed equality heuristic could not tell from an
	// advanced history.
	constexpr double kDt = 0.25;
	constexpr int kSteps = 4;
	constexpr double kInitialVelocityX = 0.2;
	// Exact values on the +z face, by hand: u_x; v_x = (u_n - u_(n-1)) / dt
	// from v_0 = .2; a_x = (v_n - v_(n-1)) / dt from a_0 = 0.
	constexpr std::array<double, kSteps + 1> kFaceU = {{0.0, 0.1, 0.1, 0.1, 0.05}};
	constexpr std::array<double, kSteps + 1> kFaceV = {{0.2, 0.4, 0.0, 0.0, -0.2}};
	constexpr std::array<double, kSteps + 1> kFaceA = {{0.0, 0.8, -1.6, 0.0, -0.8}};
	constexpr bool kHeld[kSteps + 1] = {false, false, true, true, false}; // bit-identical to the previous endpoint

	json scene_args(const std::filesystem::path &out_dir)
	{
		const std::filesystem::path mesh = std::filesystem::path(POLYFEM_TEST_DIR) / ".." / "scenes" / "semi-implicit" / "cube.mesh";
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", mesh.string()},
										 {"volume_selection", 1},
										 {"surface_selection", json::array({{{"id", 1}, {"axis", "-z"}, {"position", 1e-6}},
																			{{"id", 2}, {"axis", "+z"}, {"position", 1.0 - 1e-6}}})}}});
		args["materials"] = {{"type", "NeoHookean"}, {"E", 1e5}, {"nu", 0.3}, {"rho", 1000.0}};
		args["time"] = {{"dt", 0.25}, {"time_steps", 4}, {"integrator", "ImplicitEuler"}, {"quasistatic", true}};
		args["boundary_conditions"] = {
			{"dirichlet_boundary", json::array({{{"id", 1}, {"value", json::array({0, 0, 0})}},
												{{"id", 2}, {"value", json::array({"0.1*min(t/0.25, 1) - 0.05*max((t-0.75)/0.25, 0)", "0", "0"})}}})},
			{"rhs", json::array({0.0, 0.0, 0.0})}};
		args["initial_conditions"] = {{"velocity", json::array({{{"id", 1}, {"value", json::array({kInitialVelocityX, 0, 0})}}})}};
		args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
		args["/solver/nonlinear/grad_norm_tol"_json_pointer] = 1e-10;
		args["/solver/nonlinear/first_grad_norm_tol"_json_pointer] = 1e-10;
		args["/solver/max_threads"_json_pointer] = 1;
		args["/output/directory"_json_pointer] = out_dir.string();
		args["/output/log/level"_json_pointer] = "warning";
		args["/output/paraview/file_name"_json_pointer] = "run.pvd";
		args["/output/paraview/options/velocity"_json_pointer] = true;
		args["/output/paraview/options/acceleration"_json_pointer] = true;
		return args;
	}

	// The velocity/acceleration output of the current in-memory state at
	// every node, as DOF-ordered vectors (the public after-solve API that the
	// VTU writer also goes through).
	std::pair<Eigen::VectorXd, Eigen::VectorXd> node_kinematics(const varform::VarForm &form, const Eigen::MatrixXd &sol, const int n_nodes)
	{
		io::OutputSample sample;
		sample.node_ids.setLinSpaced(n_nodes, 0, n_nodes - 1);
		sample.time = 0;
		io::OutputFieldOptions options;
		options.fields = {"velocity", "acceleration"};
		const auto fields = form.output_fields(sample, sol, options);
		Eigen::VectorXd velocity, acceleration;
		for (const auto &f : fields)
		{
			if (f.name != "velocity" && f.name != "acceleration")
				continue;
			REQUIRE(f.values.rows() == n_nodes);
			REQUIRE(f.values.cols() == 3);
			Eigen::VectorXd out(3 * n_nodes);
			for (int i = 0; i < n_nodes; ++i)
				out.segment<3>(3 * i) = f.values.row(i).transpose();
			(f.name == "velocity" ? velocity : acceleration) = out;
		}
		REQUIRE(velocity.size() == 3 * n_nodes);
		REQUIRE(acceleration.size() == 3 * n_nodes);
		return {velocity, acceleration};
	}

	double max_abs_diff(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b)
	{
		REQUIRE(a.rows() == b.rows());
		REQUIRE(a.cols() == b.cols());
		return a.size() == 0 ? 0.0 : (a - b).cwiseAbs().maxCoeff();
	}

	// Rest positions of the nodes, DOF order, and the +z face node indices.
	std::pair<Eigen::MatrixXd, std::vector<int>> node_positions(const varform::VarForm &form)
	{
		const auto debug = test::VarFormTestAccess::debug_data(form);
		REQUIRE(debug.n_bases > 0);
		Eigen::MatrixXd nodes = Eigen::MatrixXd::Zero(debug.n_bases, 3);
		for (const auto &element : *debug.bases)
			for (const auto &b : element.bases)
				if (b.global().size() == 1)
					nodes.row(b.global()[0].index) = b.global()[0].node;
		std::vector<int> face;
		for (int i = 0; i < debug.n_bases; ++i)
			if (nodes(i, 2) >= 1.0 - 1e-6)
				face.push_back(i);
		REQUIRE(!face.empty());
		return {nodes, face};
	}

	// Independent Implicit Euler rule on a sequence of accepted solutions:
	// v_n = (u_n - u_(n-1)) / dt, a_n = (v_n - v_(n-1)) / dt from the
	// supplied initial velocity (.2, 0, 0) and acceleration 0.
	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> euler_reference(const std::vector<Eigen::VectorXd> &endpoints)
	{
		REQUIRE(endpoints.size() == kSteps + 1);
		std::vector<Eigen::VectorXd> v(kSteps + 1), a(kSteps + 1);
		v[0] = Eigen::VectorXd::Zero(endpoints[0].size());
		for (int i = 0; 3 * i < endpoints[0].size(); ++i)
			v[0](3 * i) = kInitialVelocityX;
		a[0] = Eigen::VectorXd::Zero(endpoints[0].size());
		for (int t = 1; t <= kSteps; ++t)
		{
			v[t] = (endpoints[t] - endpoints[t - 1]) / kDt;
			a[t] = (v[t] - v[t - 1]) / kDt;
		}
		return {v, a};
	}

	// The fixture's teeth: the held steps returned the previous solution bit
	// for bit (zero Newton iterations), so an export cannot tell a hold from
	// an advanced history by comparing positions; the last step moves back.
	void check_hold_pattern(const std::vector<Eigen::VectorXd> &endpoints)
	{
		for (int t = 1; t <= kSteps; ++t)
		{
			CAPTURE(t);
			if (kHeld[t])
				CHECK(endpoints[t] == endpoints[t - 1]);
			else
				CHECK(endpoints[t] != endpoints[t - 1]);
		}
	}

	// Exact +z face values of the endpoint and of an export's kinematics.
	void check_face_values(const int t, const std::vector<int> &face, const Eigen::VectorXd &u, const Eigen::VectorXd &v, const Eigen::VectorXd &a)
	{
		for (const int i : face)
		{
			CAPTURE(i, t);
			CHECK(u(3 * i) == Approx(kFaceU[t]).margin(1e-15));
			CHECK(v(3 * i) == Approx(kFaceV[t]).margin(1e-14));
			CHECK(a(3 * i) == Approx(kFaceA[t]).margin(1e-14));
		}
	}

	// The saved step_N.vtu velocity and acceleration against the Implicit
	// Euler rule on the saved VTU displacements, and the hold pattern as saved.
	void check_saved_vtu_kinematics(const std::filesystem::path &dir)
	{
		Eigen::MatrixXd previous_u, previous_v;
		for (int t = 0; t <= kSteps; ++t)
		{
			CAPTURE(t);
			const auto vtu = dir / ("step_" + std::to_string(t) + ".vtu");
			REQUIRE(std::filesystem::exists(vtu));
			const Eigen::MatrixXd u = test::read_vtu_field(vtu, "displacement");
			const Eigen::MatrixXd v = test::read_vtu_field(vtu, "velocity");
			const Eigen::MatrixXd a = test::read_vtu_field(vtu, "acceleration");
			REQUIRE(u.rows() > 0);
			REQUIRE(u.cols() == 3);
			Eigen::MatrixXd expected_v, expected_a;
			if (t == 0)
			{
				expected_v = Eigen::MatrixXd::Zero(u.rows(), 3);
				expected_v.col(0).setConstant(kInitialVelocityX);
				expected_a = Eigen::MatrixXd::Zero(u.rows(), 3);
			}
			else
			{
				expected_v = (u - previous_u) / kDt;
				expected_a = (expected_v - previous_v) / kDt;
				if (kHeld[t])
					CHECK(u == previous_u);
				else
					CHECK(u != previous_u);
			}
			CHECK(max_abs_diff(v, expected_v) <= 1e-12);
			CHECK(max_abs_diff(a, expected_a) <= 1e-12);
			previous_u = u;
			previous_v = expected_v;
		}
	}
} // namespace

TEST_CASE("Saved VTU kinematics of a held prescribed motion follow the saved displacements", "[output_kinematics][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rbr01-held-motion");

	auto state = std::make_unique<State>();
	state->init(scene_args(dir), true);
	state->set_max_threads(1);
	state->load_mesh();
	auto *form = dynamic_cast<varform::NonlinearElasticTransientVarForm *>(state->variational_formulation.get());
	REQUIRE(form != nullptr);
	test::VarFormTestAccess::prepare(*form);
	const auto [nodes, face] = node_positions(*form);
	const int n_nodes = nodes.rows();

	// Drive the production loop stage by stage so the same solution can be
	// exported before its advance (the phase the VTU is written in) and after.
	Eigen::MatrixXd sol;
	std::vector<Eigen::VectorXd> endpoints;                                 // sol_0 .. sol_4
	std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> before, after; // node output per step
	test::VarFormTestAccess::begin_transient_run(*form, sol, {});
	REQUIRE(sol.size() == 3 * n_nodes);
	endpoints.push_back(sol.col(0));
	before.push_back(node_kinematics(*form, sol, n_nodes)); // the initial save: history head by construction
	after.push_back(before.back());
	for (int t = 1; t <= kSteps; ++t)
	{
		test::VarFormTestAccess::solve_transient_step(*form, t, sol, {});
		before.push_back(node_kinematics(*form, sol, n_nodes));
		test::VarFormTestAccess::advance_transient_step(*form, t, sol);
		after.push_back(node_kinematics(*form, sol, n_nodes));
		endpoints.push_back(sol.col(0));
	}
	test::VarFormTestAccess::end_transient_run(*form);

	check_hold_pattern(endpoints);
	const auto [v_ref, a_ref] = euler_reference(endpoints);

	SECTION("the output before and after the history advance is the saved solution's kinematics")
	{
		for (int t = 0; t <= kSteps; ++t)
		{
			CAPTURE(t);
			CHECK(max_abs_diff(before[t].first, v_ref[t]) <= 1e-12);
			CHECK(max_abs_diff(before[t].second, a_ref[t]) <= 1e-12);
			CHECK(max_abs_diff(after[t].first, v_ref[t]) <= 1e-12);
			CHECK(max_abs_diff(after[t].second, a_ref[t]) <= 1e-12);
			CHECK(max_abs_diff(before[t].first, after[t].first) <= 1e-12);
			CHECK(max_abs_diff(before[t].second, after[t].second) <= 1e-12);
		}
	}

	SECTION("the prescribed face carries the exact hold and return values")
	{
		for (int t = 0; t <= kSteps; ++t)
			check_face_values(t, face, endpoints[t], before[t].first, before[t].second);
	}

	SECTION("the saved VTU velocity and acceleration follow the saved VTU displacements")
	{
		check_saved_vtu_kinematics(dir);
	}

	std::filesystem::remove_all(dir);
}

// The differentiable transient class (State::init(args, true, true), the
// class the adjoint optimization drives through solve_pde) dispatches the
// step's solve on its own path when differentiable=true; the first RBR-01
// repair left the output time phase at HistoryHead on that path, so the
// step callback, the subsolve sequence and the saved timestep described the
// new displacement with the previous step's kinematics (.2 instead of .4 at
// the first move, .4 instead of 0 at the first hold: the 2026-09-21 review).
// Both modes are exercised through the public differentiable solve overload;
// the ordinary staged accessors above do not reach this dispatch.
TEST_CASE("Differentiable transient output states the current endpoint in both modes", "[output_kinematics][scene][differentiable]")
{
	logger().set_level(spdlog::level::warn);
	std::vector<std::vector<Eigen::VectorXd>> endpoints_by_mode;
	for (const bool differentiable : {false, true})
	{
		// A plain loop, not sections: the cross-mode comparison at the end
		// needs both runs in the same pass.
		CAPTURE(differentiable);
		{
			const auto dir = scratch_dir(std::string("rbr01-differentiable-") + (differentiable ? "true" : "false"));
			json args = scene_args(dir);
			args["/output/advanced/save_solve_sequence_debug"_json_pointer] = true; // the subsolve exports go through the same exporter

			auto state = std::make_unique<State>();
			state->init(args, true, /*is_adjoint_optimization=*/true);
			state->set_max_threads(1);
			state->load_mesh();
			auto *form = dynamic_cast<varform::DifferentiableNonlinearElasticTransientVarForm *>(state->variational_formulation.get());
			REQUIRE(form != nullptr);
			test::VarFormTestAccess::prepare(*form);
			const auto [nodes, face] = node_positions(*form);
			const int n_nodes = nodes.rows();

			// The step callback fires with the accepted endpoint before the
			// history advances (step 0: the initial state after the
			// integrator's init); its output is what the frame is written from.
			std::vector<Eigen::VectorXd> endpoints;
			std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> callback;
			std::vector<int> callback_steps;
			Eigen::MatrixXd sol;
			form->solve(
				sol, nullptr,
				[&](const int step, const Eigen::MatrixXd &solution) {
					callback_steps.push_back(step);
					endpoints.push_back(solution.col(0));
					callback.push_back(node_kinematics(*form, solution, n_nodes));
				},
				differentiable);
			REQUIRE(callback_steps == std::vector<int>({0, 1, 2, 3, 4}));
			REQUIRE(sol.size() == 3 * n_nodes);
			REQUIRE(sol.col(0) == endpoints.back());

			check_hold_pattern(endpoints);
			const auto [v_ref, a_ref] = euler_reference(endpoints);
			for (int t = 0; t <= kSteps; ++t)
			{
				CAPTURE(t);
				CHECK(max_abs_diff(callback[t].first, v_ref[t]) <= 1e-12);
				CHECK(max_abs_diff(callback[t].second, a_ref[t]) <= 1e-12);
				check_face_values(t, face, endpoints[t], callback[t].first, callback[t].second);
			}

			// After the run the history head is the last endpoint (the last
			// step moved, so this is not a zero-motion tail): the same
			// kinematics from the other phase.
			const auto [v_final, a_final] = node_kinematics(*form, sol, n_nodes);
			CHECK(max_abs_diff(v_final, v_ref[kSteps]) <= 1e-12);
			CHECK(max_abs_diff(a_final, a_ref[kSteps]) <= 1e-12);
			CHECK(max_abs_diff(v_final, callback[kSteps].first) <= 1e-12);

			check_saved_vtu_kinematics(dir);

			// The subsolve sequence of the last step ends at its accepted
			// endpoint: the last solve_N.vtu carries the same kinematics as
			// step_4.vtu (both exported before the advance).
			int last_subsolve = -1;
			for (const auto &entry : std::filesystem::directory_iterator(dir))
			{
				const std::string name = entry.path().filename().string();
				if (name.rfind("solve_", 0) == 0 && entry.path().extension() == ".vtu")
					last_subsolve = std::max(last_subsolve, std::stoi(name.substr(6, name.size() - 10)));
			}
			REQUIRE(last_subsolve >= 1);
			const auto subsolve = dir / ("solve_" + std::to_string(last_subsolve) + ".vtu");
			const auto frame = dir / ("step_" + std::to_string(kSteps) + ".vtu");
			CHECK(max_abs_diff(test::read_vtu_field(subsolve, "displacement"), test::read_vtu_field(frame, "displacement")) == 0.0);
			CHECK(max_abs_diff(test::read_vtu_field(subsolve, "velocity"), test::read_vtu_field(frame, "velocity")) <= 1e-12);
			CHECK(max_abs_diff(test::read_vtu_field(subsolve, "acceleration"), test::read_vtu_field(frame, "acceleration")) <= 1e-12);

			// Both dispatches solve the same problem on this fixture.
			endpoints_by_mode.push_back(endpoints);
			if (endpoints_by_mode.size() == 2)
				for (int t = 0; t <= kSteps; ++t)
				{
					CAPTURE(t);
					CHECK(max_abs_diff(endpoints_by_mode[0][t], endpoints_by_mode[1][t]) <= 1e-12);
				}

			std::filesystem::remove_all(dir);
		}
	}
}
