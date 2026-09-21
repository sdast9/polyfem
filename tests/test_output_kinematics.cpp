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
// This test drives the real nonlinear transient loop on a public prescribed
// motion that moves at step 1 and holds afterwards, and checks the saved VTU
// fields, the before-advance output (what the VTU is written from) and the
// after-advance output of the same solution against the Implicit Euler rule
// evaluated independently on the saved displacements, at every time step.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/OutputData.hpp>
#include <polyfem/utils/Logger.hpp>

#include "VarFormTestAccess.hpp"
#include "VtuTestUtils.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
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
	// fixed, +z face sheared by .1 over the first step and held afterwards
	// (0.1*min(t/dt, 1)), quasistatic Implicit Euler, dt=.25, four steps, a
	// supplied initial velocity so that the initial export and the first
	// acceleration are not trivially zero. first_grad_norm_tol equals
	// grad_norm_tol so that a held step provably returns in zero Newton
	// iterations (the gradient that ended step 1 is below it), i.e. the
	// held solution is bit-identical to the previous one: the case the
	// reviewed equality heuristic could not tell from an advanced history.
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
												{{"id", 2}, {"value", json::array({"0.1*min(t/0.25, 1)", "0", "0"})}}})},
			{"rhs", json::array({0.0, 0.0, 0.0})}};
		args["initial_conditions"] = {{"velocity", json::array({{{"id", 1}, {"value", json::array({0.2, 0, 0})}}})}};
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
} // namespace

TEST_CASE("Saved VTU kinematics of a held prescribed motion follow the saved displacements", "[output_kinematics][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rbr01-held-motion");
	const double dt = 0.25;
	const int steps = 4;

	auto state = std::make_unique<State>();
	state->init(scene_args(dir), true);
	state->set_max_threads(1);
	state->load_mesh();
	auto *form = dynamic_cast<varform::NonlinearElasticTransientVarForm *>(state->variational_formulation.get());
	REQUIRE(form != nullptr);
	test::VarFormTestAccess::prepare(*form);
	const auto debug = test::VarFormTestAccess::debug_data(*form);
	const int n_nodes = debug.n_bases;
	REQUIRE(n_nodes > 0);
	Eigen::MatrixXd nodes = Eigen::MatrixXd::Zero(n_nodes, 3);
	for (const auto &element : *debug.bases)
		for (const auto &b : element.bases)
			if (b.global().size() == 1)
				nodes.row(b.global()[0].index) = b.global()[0].node;

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
	for (int t = 1; t <= steps; ++t)
	{
		test::VarFormTestAccess::solve_transient_step(*form, t, sol, {});
		before.push_back(node_kinematics(*form, sol, n_nodes));
		test::VarFormTestAccess::advance_transient_step(*form, t, sol);
		after.push_back(node_kinematics(*form, sol, n_nodes));
		endpoints.push_back(sol.col(0));
	}
	test::VarFormTestAccess::end_transient_run(*form);

	// The fixture's teeth: the held steps returned the previous solution
	// bit for bit (zero Newton iterations), so the export cannot tell the
	// hold from an advanced history by comparing positions.
	CHECK(endpoints[1] != endpoints[0]);
	for (int t = 2; t <= steps; ++t)
	{
		CAPTURE(t);
		CHECK(endpoints[t] == endpoints[t - 1]);
	}

	// Independent Implicit Euler rule on the accepted solutions: v_n =
	// (u_n - u_(n-1)) / dt, a_n = (v_n - v_(n-1)) / dt from the supplied
	// initial velocity (.2, 0, 0) and acceleration 0.
	std::vector<Eigen::VectorXd> v_ref(steps + 1), a_ref(steps + 1);
	v_ref[0] = Eigen::VectorXd::Zero(3 * n_nodes);
	for (int i = 0; i < n_nodes; ++i)
		v_ref[0](3 * i) = 0.2;
	a_ref[0] = Eigen::VectorXd::Zero(3 * n_nodes);
	for (int t = 1; t <= steps; ++t)
	{
		v_ref[t] = (endpoints[t] - endpoints[t - 1]) / dt;
		a_ref[t] = (v_ref[t] - v_ref[t - 1]) / dt;
	}

	SECTION("the output before and after the history advance is the saved solution's kinematics")
	{
		for (int t = 0; t <= steps; ++t)
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

	SECTION("the prescribed face carries the exact hold values")
	{
		// +z face: u_x = .1 from step 1 on; v_x .4 then 0; a_x (.4-.2)/.25 = .8,
		// then -1.6 at the first held step, then 0.
		const double expected_v[steps + 1] = {0.2, 0.4, 0.0, 0.0, 0.0};
		const double expected_a[steps + 1] = {0.0, 0.8, -1.6, 0.0, 0.0};
		int checked = 0;
		for (int i = 0; i < n_nodes; ++i)
		{
			if (nodes(i, 2) < 1.0 - 1e-6)
				continue;
			++checked;
			for (int t = 0; t <= steps; ++t)
			{
				CAPTURE(i, t);
				CHECK(endpoints[t](3 * i) == Approx(t == 0 ? 0.0 : 0.1).margin(1e-15));
				CHECK(before[t].first(3 * i) == Approx(expected_v[t]).margin(1e-14));
				CHECK(before[t].second(3 * i) == Approx(expected_a[t]).margin(1e-14));
			}
		}
		CHECK(checked > 0);
	}

	SECTION("the saved VTU velocity and acceleration follow the saved VTU displacements")
	{
		Eigen::MatrixXd previous_u, previous_v;
		for (int t = 0; t <= steps; ++t)
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
				expected_v.col(0).setConstant(0.2);
				expected_a = Eigen::MatrixXd::Zero(u.rows(), 3);
			}
			else
			{
				expected_v = (u - previous_u) / dt;
				expected_a = (expected_v - previous_v) / dt;
			}
			CHECK(max_abs_diff(v, expected_v) <= 1e-12);
			CHECK(max_abs_diff(a, expected_a) <= 1e-12);
			if (t >= 2)
				CHECK(u == previous_u); // the hold, as saved
			previous_u = u;
			previous_v = expected_v;
		}
	}

	std::filesystem::remove_all(dir);
}
