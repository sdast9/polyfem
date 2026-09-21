// RBR-05: a fully prescribed body -- every degree of freedom Dirichlet --
// is a valid trivial solve, not a crash.
//
// A 2x1x1 P1 beam whose twelve vertices all lie on the boundary, with the
// whole boundary prescribed, has zero free DOFs. NLProblem::setup_constraints
// (upstream "fix case of no dofs", fb762251e) returned early on an empty
// reduced space without the affine offset Q1R1iTb_ or the penalty problem,
// so reduced_to_full(empty) added a size-0 vector to a full-size one, the
// elastic energy read a null vector and the process died with SIGSEGV right
// after step_0.vtu (evidence: outputs/rbr-05/). Whether a scene has free DOFs
// is a property of its discretization (the same scene at P2 has free
// interior edge nodes), so the empty reduced problem is solved by definition
// instead of refused: the reduced problem has no unknowns, the solution of
// every step is the prescribed values, the kinematics, forces and records are
// exported like any other step, and the outcome must not depend on the
// nonlinear tolerances (a zero first_grad_norm_tol would send PolySolve into
// a Newton step on a 0x0 system, an Linf norm type into maxCoeff of an
// empty gradient), so ALSolver::solve_reduced states it without calling the
// nonlinear solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/OutputData.hpp>
#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polysolve/linear/Solver.hpp>

#include "VarFormTestAccess.hpp"
#include "VtuTestUtils.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace polyfem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace
{
	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	// The 2x1x1 beam of the reproduction: unit cubes split into six Kuhn
	// tetrahedra, twelve vertices, every one of them on the boundary.
	std::filesystem::path write_beam_mesh(const std::filesystem::path &dir)
	{
		const auto path = dir / "beam.mesh";
		std::ofstream out(path);
		REQUIRE(out.is_open());
		out << "MeshVersionFormatted 2\nDimension 3\nVertices\n12\n";
		for (int i = 0; i <= 2; ++i)
			for (int j = 0; j <= 1; ++j)
				for (int k = 0; k <= 1; ++k)
					out << i << " " << j << " " << k << " 0\n";
		out << "Tetrahedra\n12\n"
			<< "1 5 7 8 0\n1 5 8 6 0\n1 3 8 7 0\n1 3 4 8 0\n1 2 6 8 0\n1 2 8 4 0\n"
			<< "5 9 11 12 0\n5 9 12 10 0\n5 7 12 11 0\n5 7 8 12 0\n5 6 10 12 0\n5 6 12 8 0\n"
			<< "End\n";
		return path;
	}

	// The reproduction: the whole boundary (surface_selection 1) prescribed
	// to move .1 in x over the first step and hold, Neo-Hookean, Implicit
	// Euler dt .25, three steps, a supplied initial velocity .2 so that the
	// exported kinematics are not trivially zero.
	json transient_args(const std::filesystem::path &mesh, const std::filesystem::path &out_dir)
	{
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", mesh.string()}, {"volume_selection", 1}, {"surface_selection", 1}}});
		args["materials"] = {{"type", "NeoHookean"}, {"E", 1e5}, {"nu", 0.3}, {"rho", 1000.0}};
		args["time"] = {{"dt", 0.25}, {"time_steps", 3}, {"integrator", "ImplicitEuler"}};
		args["boundary_conditions"] = {
			{"dirichlet_boundary", json::array({{{"id", 1}, {"value", json::array({"0.1*min(t/0.25, 1)", "0", "0"})}}})},
			{"rhs", json::array({0.0, 0.0, 0.0})}};
		args["initial_conditions"] = {{"velocity", json::array({{{"id", 1}, {"value", json::array({0.2, 0, 0})}}})}};
		args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
		args["/solver/max_threads"_json_pointer] = 1;
		args["/output/directory"_json_pointer] = out_dir.string();
		args["/output/log/level"_json_pointer] = "warning";
		args["/output/paraview/file_name"_json_pointer] = "run.pvd";
		args["/output/paraview/options/velocity"_json_pointer] = true;
		args["/output/paraview/options/acceleration"_json_pointer] = true;
		return args;
	}

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

	// The prescribed x-motion of the beam scenes at step t: .1 from step 1 on.
	Eigen::VectorXd prescribed_solution(const int t, const int n_nodes)
	{
		Eigen::VectorXd u = Eigen::VectorXd::Zero(3 * n_nodes);
		if (t >= 1)
			for (int i = 0; i < n_nodes; ++i)
				u(3 * i) = 0.1;
		return u;
	}

	// The subsolve records of a step: every one is the empty reduced solve.
	void check_trivial_subsolves(const io::OutStatsData &stats, const int steps)
	{
		for (int t = 1; t <= steps; ++t)
		{
			CAPTURE(t);
			int reduced = 0, others = 0;
			for (const auto &info : stats.solver_info)
			{
				if (!info.is_object() || info.value("t", -1) != t)
					continue;
				if (info.value("type", std::string()) != "rc")
				{
					++others;
					continue;
				}
				++reduced;
				REQUIRE(info.contains("info"));
				const json &detail = info["info"];
				CHECK(detail.value("outcome", std::string()) == "converged");
				CHECK(detail.value("iterations", -1) == 0);
				CHECK_THAT(detail.value("termination_reason", std::string()), ContainsSubstring("No free degrees of freedom"));
			}
			CHECK(reduced == 1);
			CHECK(others == 0); // the snap to the prescribed values needs no AL pass
		}
	}

	// A quartic in every coordinate with both coordinates prescribed: the
	// unit fixture of the empty reduced space.
	struct TwoQuartics : solver::Form
	{
		std::string name() const override { return "two-quartics"; }
		double value_unweighted(const Eigen::VectorXd &x) const override { return .25 * x.array().pow(4).sum(); }
		void first_derivative_unweighted(const Eigen::VectorXd &x, Eigen::VectorXd &g) const override { g = x.array().cube(); }
		void second_derivative_unweighted(const Eigen::VectorXd &x, StiffnessMatrix &h) const override
		{
			h.resize(x.size(), x.size());
			h.setZero();
			for (int i = 0; i < x.size(); ++i)
				h.coeffRef(i, i) = 3 * x[i] * x[i];
		}
	};
	const json linear_solver = {{"solver", "Eigen::SimplicialLDLT"}};
	json newton_parameters()
	{
		return {{"solver", "Newton"}, {"max_iterations", 100}, {"grad_norm_tol", 1e-12}, {"rel_grad_norm_tol", 0.0}, {"first_grad_norm_tol", 1e-12}, {"x_delta_tol", 0.0}, {"allow_non_grad_convergence", false}, {"line_search", {{"method", "Backtracking"}}}};
	}
} // namespace

TEST_CASE("An empty reduced problem is the prescribed values", "[fully_prescribed][al_solver]")
{
	const int n = 2;
	StiffnessMatrix mass(n, n);
	mass.setIdentity();
	const std::vector<int> boundary{0, 1};
	Eigen::VectorXd target(n);
	target << .3, -.2;
	auto bc = std::make_shared<solver::BCLagrangianForm>(n, boundary, mass, 0, target);
	std::shared_ptr<polysolve::linear::Solver> solver = polysolve::linear::Solver::create(linear_solver, logger());
	solver::NLProblem problem(n, 0, {std::make_shared<TwoQuartics>()}, {bc}, solver, 1, 1, mass, 1);

	SECTION("the transforms, energy and derivatives of the empty space")
	{
		CHECK(problem.full_size() == n);
		CHECK(problem.reduced_size() == 0);
		const Eigen::VectorXd anywhere = Eigen::VectorXd::Constant(n, 7);
		CHECK(problem.full_to_reduced(anywhere).size() == 0);
		const Eigen::VectorXd full = problem.reduced_to_full(Eigen::VectorXd(0));
		REQUIRE(full.size() == n);
		CHECK(full == target);
		CHECK(problem.value(Eigen::VectorXd(0)) == Approx(.25 * (std::pow(.3, 4) + std::pow(.2, 4))));
		Eigen::VectorXd grad;
		problem.gradient(Eigen::VectorXd(0), grad);
		CHECK(grad.size() == 0);
		StiffnessMatrix hessian;
		problem.hessian(Eigen::VectorXd(0), hessian);
		CHECK(hessian.rows() == 0);
		CHECK(hessian.cols() == 0);
		CHECK(problem.is_step_valid(anywhere, Eigen::VectorXd(0)));
		CHECK(problem.is_step_collision_free(anywhere, Eigen::VectorXd(0)));
		CHECK(problem.max_step_size(anywhere, Eigen::VectorXd(0)) == 1);
		// The full-size AL passes still see the penalty: the BC form's value
		// at a point off the target is not zero.
		problem.use_full_size();
		bc->set_initial_weight(1);
		CHECK(problem.value(anywhere) > .25 * 2 * std::pow(7., 4));
		problem.use_reduced_size();
	}

	SECTION("the AL stage snaps and the reduced stage states the solution without a nonlinear solve")
	{
		for (const bool tolerances_that_trip_polysolve : {false, true})
		{
			CAPTURE(tolerances_that_trip_polysolve);
			json params = newton_parameters();
			if (tolerances_that_trip_polysolve)
			{
				params["first_grad_norm_tol"] = 0.0;
				params["grad_norm_tol"] = 0.0;
				params["x_delta_tol"] = 1e-12;
				params["norm_type"] = "Linf";
			}
			bc->set_initial_weight(1);
			solver::ALSolver al({bc}, 1, 2, 1e8, .99, [](const auto &) {});
			Eigen::MatrixXd sol = Eigen::VectorXd::Constant(n, 10);
			REQUIRE_NOTHROW(al.solve_al(problem, sol, params, linear_solver, 1));
			REQUIRE_NOTHROW(al.solve_reduced(problem, sol, params, linear_solver, 1));
			REQUIRE(sol.size() == n);
			CHECK(sol.col(0) == target);
			CHECK(al.info().value("outcome", std::string()) == "converged");
			CHECK(al.info().value("iterations", -1) == 0);
			CHECK_THAT(al.info().value("termination_reason", std::string()), ContainsSubstring("No free degrees of freedom"));
			CHECK(al.info().value("energy", -1.0) == Approx(.25 * (std::pow(.3, 4) + std::pow(.2, 4))));
		}
	}
}

TEST_CASE("A fully prescribed transient body follows its prescribed motion", "[fully_prescribed][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rbr05-fully-prescribed");
	const auto mesh = write_beam_mesh(dir);
	const double dt = 0.25;
	const int steps = 3;

	json args = transient_args(mesh, dir / "output");
	SECTION("default tolerances") {}
	SECTION("tolerances and norm that would trip the nonlinear solver on an empty problem")
	{
		args["/solver/nonlinear/first_grad_norm_tol"_json_pointer] = 0.0;
		args["/solver/nonlinear/grad_norm_tol"_json_pointer] = 0.0;
		args["/solver/nonlinear/x_delta_tol"_json_pointer] = 1e-12;
		args["/solver/nonlinear/norm_type"_json_pointer] = "Linf";
	}
	SECTION("contact enabled on the lone body (no candidates)")
	{
		args["contact"] = {{"enabled", true}, {"dhat", 1e-3}};
	}
	SECTION("friction (the lagging loop over an empty reduced problem)")
	{
		args["contact"] = {{"enabled", true}, {"dhat", 1e-3}, {"friction_coefficient", 0.3}};
	}

	auto state = std::make_unique<State>();
	state->init(args, true);
	state->set_max_threads(1);
	state->load_mesh();
	auto *form = dynamic_cast<varform::NonlinearElasticTransientVarForm *>(state->variational_formulation.get());
	REQUIRE(form != nullptr);
	test::VarFormTestAccess::prepare(*form);
	const auto debug = test::VarFormTestAccess::debug_data(*form);
	const int n_nodes = debug.n_bases;
	REQUIRE(n_nodes == 12);

	Eigen::MatrixXd sol;
	std::vector<Eigen::VectorXd> endpoints;
	std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> before, after;
	test::VarFormTestAccess::begin_transient_run(*form, sol, {});
	REQUIRE(sol.size() == 3 * n_nodes);
	// The fixture's premise: not one free DOF.
	const auto &solve_data = test::VarFormTestAccess::solve_data(*form);
	REQUIRE(solve_data.nl_problem != nullptr);
	REQUIRE(solve_data.nl_problem->full_size() == 3 * n_nodes);
	REQUIRE(solve_data.nl_problem->reduced_size() == 0);
	endpoints.push_back(sol.col(0));
	before.push_back(node_kinematics(*form, sol, n_nodes));
	after.push_back(before.back());
	for (int t = 1; t <= steps; ++t)
	{
		CAPTURE(t);
		REQUIRE_NOTHROW(test::VarFormTestAccess::solve_transient_step(*form, t, sol, {}));
		before.push_back(node_kinematics(*form, sol, n_nodes));
		test::VarFormTestAccess::advance_transient_step(*form, t, sol);
		after.push_back(node_kinematics(*form, sol, n_nodes));
		endpoints.push_back(sol.col(0));
	}
	test::VarFormTestAccess::end_transient_run(*form);

	// Every step's solution is the prescribed values, nothing else.
	for (int t = 0; t <= steps; ++t)
	{
		CAPTURE(t);
		CHECK(max_abs_diff(endpoints[t], prescribed_solution(t, n_nodes)) <= 1e-15);
	}
	check_trivial_subsolves(test::VarFormTestAccess::stats(*form), steps);

	// The kinematics of the prescribed motion by the Implicit Euler rule,
	// exact values on every node: v = .2, .4, 0, 0; a = 0, .8, -1.6, 0.
	const double expected_v[steps + 1] = {0.2, 0.4, 0.0, 0.0};
	const double expected_a[steps + 1] = {0.0, 0.8, -1.6, 0.0};
	for (int t = 0; t <= steps; ++t)
	{
		CAPTURE(t);
		Eigen::VectorXd v = Eigen::VectorXd::Zero(3 * n_nodes), a = Eigen::VectorXd::Zero(3 * n_nodes);
		for (int i = 0; i < n_nodes; ++i)
		{
			v(3 * i) = expected_v[t];
			a(3 * i) = expected_a[t];
		}
		CHECK(max_abs_diff(before[t].first, v) <= 1e-14);
		CHECK(max_abs_diff(before[t].second, a) <= 1e-14);
		CHECK(max_abs_diff(after[t].first, v) <= 1e-14);
		CHECK(max_abs_diff(after[t].second, a) <= 1e-14);
	}

	// The saved VTUs carry the prescribed displacement and its kinematics.
	Eigen::MatrixXd previous_u, previous_v;
	for (int t = 0; t <= steps; ++t)
	{
		CAPTURE(t);
		const auto vtu = dir / "output" / ("step_" + std::to_string(t) + ".vtu");
		REQUIRE(std::filesystem::exists(vtu));
		const Eigen::MatrixXd u = test::read_vtu_field(vtu, "displacement");
		const Eigen::MatrixXd v = test::read_vtu_field(vtu, "velocity");
		const Eigen::MatrixXd a = test::read_vtu_field(vtu, "acceleration");
		REQUIRE(u.rows() > 0);
		REQUIRE(u.cols() == 3);
		Eigen::MatrixXd expected_u = Eigen::MatrixXd::Zero(u.rows(), 3);
		expected_u.col(0).setConstant(t >= 1 ? 0.1 : 0.0);
		CHECK(max_abs_diff(u, expected_u) <= 1e-15);
		Eigen::MatrixXd ev, ea;
		if (t == 0)
		{
			ev = Eigen::MatrixXd::Zero(u.rows(), 3);
			ev.col(0).setConstant(0.2);
			ea = Eigen::MatrixXd::Zero(u.rows(), 3);
		}
		else
		{
			ev = (u - previous_u) / dt;
			ea = (ev - previous_v) / dt;
		}
		CHECK(max_abs_diff(v, ev) <= 1e-12);
		CHECK(max_abs_diff(a, ea) <= 1e-12);
		previous_u = u;
		previous_v = ev;
	}

	std::filesystem::remove_all(dir);
}

TEST_CASE("A fully prescribed body driven within dhat of an obstacle keeps its prescribed motion", "[fully_prescribed][scene]")
{
	// The public slab (z = -.02) below the beam; the whole beam is driven
	// down .0195 over the first step and held, so its bottom face ends
	// .0005 above the slab, inside dhat: the barrier is active and the body,
	// having no free DOF, cannot respond to it. The snap is collision-free,
	// so no AL pass runs and the solution is the prescribed values.
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rbr05-fully-prescribed-contact");
	const auto mesh = write_beam_mesh(dir);
	const int steps = 3;
	json args = transient_args(mesh, dir / "output");
	const std::filesystem::path slab = std::filesystem::path(POLYFEM_TEST_DIR) / ".." / "scenes" / "semi-implicit" / "slab.obj";
	args["geometry"].push_back({{"mesh", slab.string()}, {"is_obstacle", true}});
	args["contact"] = {{"enabled", true}, {"dhat", 1e-3}};
	args["boundary_conditions"]["dirichlet_boundary"][0]["value"] = json::array({"0", "0", "-0.0195*min(t/0.25, 1)"});
	args["initial_conditions"]["velocity"][0]["value"] = json::array({0, 0, 0});

	auto state = std::make_unique<State>();
	state->init(args, true);
	state->set_max_threads(1);
	state->load_mesh();
	auto *form = dynamic_cast<varform::NonlinearElasticTransientVarForm *>(state->variational_formulation.get());
	REQUIRE(form != nullptr);
	test::VarFormTestAccess::prepare(*form);
	// The obstacle's vertices are nodes and prescribed DOFs of the same
	// problem, after the beam's.
	const auto debug = test::VarFormTestAccess::debug_data(*form);
	REQUIRE(debug.n_obstacle_vertices == 4);
	const int n_nodes = debug.n_bases - debug.n_obstacle_vertices;
	REQUIRE(n_nodes == 12);

	Eigen::MatrixXd sol;
	test::VarFormTestAccess::begin_transient_run(*form, sol, {});
	const auto &solve_data = test::VarFormTestAccess::solve_data(*form);
	REQUIRE(solve_data.nl_problem != nullptr);
	REQUIRE(solve_data.nl_problem->full_size() == 3 * debug.n_bases);
	REQUIRE(solve_data.nl_problem->reduced_size() == 0);
	REQUIRE(solve_data.contact_form != nullptr);
	REQUIRE(sol.size() == solve_data.nl_problem->full_size());
	for (int t = 1; t <= steps; ++t)
	{
		CAPTURE(t);
		REQUIRE_NOTHROW(test::VarFormTestAccess::solve_transient_step(*form, t, sol, {}));
		Eigen::VectorXd expected = Eigen::VectorXd::Zero(3 * n_nodes);
		for (int i = 0; i < n_nodes; ++i)
			expected(3 * i + 2) = -0.0195;
		CHECK(max_abs_diff(sol.col(0).head(3 * n_nodes), expected) <= 1e-15);
		CHECK(max_abs_diff(sol.col(0).tail(3 * debug.n_obstacle_vertices), Eigen::VectorXd::Zero(3 * debug.n_obstacle_vertices)) <= 1e-15);
		// The barrier is active at the held position (gap .0005 < dhat).
		CHECK(solve_data.contact_form->value(sol.col(0)) > 0);
		test::VarFormTestAccess::advance_transient_step(*form, t, sol);
	}
	test::VarFormTestAccess::end_transient_run(*form);
	check_trivial_subsolves(test::VarFormTestAccess::stats(*form), steps);

	std::filesystem::remove_all(dir);
}

TEST_CASE("A fully prescribed static body is its prescribed values", "[fully_prescribed][scene]")
{
	logger().set_level(spdlog::level::warn);
	const auto dir = scratch_dir("rbr05-fully-prescribed-static");
	const auto mesh = write_beam_mesh(dir);
	json args = transient_args(mesh, dir / "output");
	args.erase("time");
	args.erase("initial_conditions");
	args["boundary_conditions"]["dirichlet_boundary"][0]["value"] = json::array({0.1, 0, 0});

	auto state = std::make_unique<State>();
	state->init(args, true);
	state->set_max_threads(1);
	state->load_mesh();
	Eigen::MatrixXd sol;
	REQUIRE_NOTHROW(state->solve(sol));
	REQUIRE_NOTHROW(state->variational_formulation->export_data(sol)); // what the CLI does after the solve
	const int n_nodes = test::VarFormTestAccess::debug_data(*state->variational_formulation).n_bases;
	REQUIRE(n_nodes == 12);
	REQUIRE(sol.size() == 3 * n_nodes);
	CHECK(max_abs_diff(sol.col(0), prescribed_solution(1, n_nodes)) <= 1e-15);
	const auto *form = dynamic_cast<const varform::NonlinearElasticVarForm *>(state->variational_formulation.get());
	REQUIRE(form != nullptr);
	// The static solve is step 0 of the subsolve records.
	int reduced = 0;
	for (const auto &info : test::VarFormTestAccess::stats(*form).solver_info)
		if (info.is_object() && info.value("t", -1) == 0 && info.value("type", std::string()) == "rc")
		{
			++reduced;
			CHECK(info["info"].value("iterations", -1) == 0);
			CHECK_THAT(info["info"].value("termination_reason", std::string()), ContainsSubstring("No free degrees of freedom"));
		}
	CHECK(reduced == 1);
	const auto vtu = dir / "output" / "run.vtu";
	REQUIRE(std::filesystem::exists(vtu));
	const Eigen::MatrixXd u = test::read_vtu_field(vtu, "displacement");
	Eigen::MatrixXd expected_u = Eigen::MatrixXd::Zero(u.rows(), 3);
	expected_u.col(0).setConstant(0.1);
	CHECK(max_abs_diff(u, expected_u) <= 1e-15);

	std::filesystem::remove_all(dir);
}
