// RB-11 envelope stage: the time-dependent LinearElasticity path.
//
// Before this stage every time-dependent linear run segfaulted (InertiaForm
// built before the integrator's init, upstream #508), `time/quasistatic` was
// ignored (the mass term was always solved), and the force export divided
// unweighted quasistatic forms by dt^2 while reporting a nonzero inertia
// force that was never part of the solved equations. These tests pin:
//  1. a true transient run at dt != 1 completes and its exported forces
//     satisfy the assembled implicit-Euler equation on the free DOFs,
//  2. a quasistatic schedule reproduces the static solve of the same
//     load/BC problem (nonzero prescribed values, several steps) with the
//     same physical forces and a zero inertia force at any dt,
//  3. a time-dependent load is exported at the saved step's time,
//  4. (follow-up review 2026-09-14) the same for ImplicitNewmark, BDF2 and
//     BDF3 at every startup step: BDF's acceleration scaling changes while
//     its history grows, and the export must normalise by the scale of the
//     step actually solved — checked through the after-solve output API
//     against an independently replayed integrator (the saved fields go
//     through the same path and are checked by the review's probe).

#include <polyfem/State.hpp>
#include <polyfem/io/OutputData.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Logger.hpp>

#include "VarFormTestAccess.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace polyfem;

namespace
{
	std::filesystem::path scratch_dir(const std::string &tag)
	{
		const auto dir = std::filesystem::temp_directory_path()
						 / (tag + "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
		std::filesystem::create_directories(dir);
		return dir;
	}

	// [0,2]x[0,1]x[0,1] in 2x1x1 cells, six Kuhn tetrahedra per cell (MEDIT)
	std::string write_beam(const std::filesystem::path &dir)
	{
		const int nx = 2, ny = 1, nz = 1;
		std::vector<std::array<double, 3>> V;
		for (int i = 0; i <= nx; ++i)
			for (int j = 0; j <= ny; ++j)
				for (int k = 0; k <= nz; ++k)
					V.push_back({{double(i), double(j), double(k)}});
		const auto vid = [&](int i, int j, int k) { return (i * (ny + 1) + j) * (nz + 1) + k; };
		const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
		std::vector<std::array<int, 4>> T;
		for (int i = 0; i < nx; ++i)
			for (int j = 0; j < ny; ++j)
				for (int k = 0; k < nz; ++k)
					for (int p = 0; p < 6; ++p)
					{
						int cur[3] = {0, 0, 0};
						std::array<int, 4> t{{vid(i, j, k), 0, 0, 0}};
						for (int s = 0; s < 3; ++s)
						{
							cur[perms[p][s]] = 1;
							t[s + 1] = vid(i + cur[0], j + cur[1], k + cur[2]);
						}
						const auto &a = V[t[0]];
						const auto &b = V[t[1]];
						const auto &c = V[t[2]];
						const auto &d = V[t[3]];
						const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
						const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
						const double ad[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
						const double vol = (ab[1] * ac[2] - ab[2] * ac[1]) * ad[0] + (ab[2] * ac[0] - ab[0] * ac[2]) * ad[1] + (ab[0] * ac[1] - ab[1] * ac[0]) * ad[2];
						if (vol < 0)
							std::swap(t[2], t[3]);
						T.push_back(t);
					}
		const auto path = dir / "beam.mesh";
		std::ofstream out(path);
		out << "MeshVersionFormatted 2\nDimension 3\nVertices\n"
			<< V.size() << "\n";
		for (const auto &v : V)
			out << v[0] << " " << v[1] << " " << v[2] << " 0\n";
		out << "Tetrahedra\n"
			<< T.size() << "\n";
		for (const auto &t : T)
			out << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << " " << t[3] + 1 << " 0\n";
		out << "End\n";
		return path.string();
	}

	json base_args(const std::string &mesh_path, const std::filesystem::path &out_dir)
	{
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", mesh_path},
										 {"surface_selection", json::array({{{"id", 1}, {"axis", "-x"}, {"position", 1e-6}},
																			{{"id", 2}, {"axis", "+x"}, {"position", 2.0 - 1e-6}}})}}});
		args["materials"] = {{"type", "LinearElasticity"}, {"E", 1e5}, {"nu", 0.3}, {"rho", 1000.0}};
		args["boundary_conditions"] = {
			{"dirichlet_boundary", json::array({{{"id", 1}, {"value", json::array({0, 0, 0})}}})},
			{"rhs", json::array({0.0, 9.81, 0.0})}};
		args["/solver/linear/solver"_json_pointer] = "Eigen::SimplicialLDLT";
		args["/solver/max_threads"_json_pointer] = 1;
		args["/output/directory"_json_pointer] = out_dir.string();
		args["/output/log/level"_json_pointer] = "error";
		args["/output/paraview/file_name"_json_pointer] = "";
		args["/output/paraview/options/forces"_json_pointer] = true;
		args["/output/paraview/options/velocity"_json_pointer] = true;
		args["/output/paraview/options/acceleration"_json_pointer] = true;
		return args;
	}

	struct Solved
	{
		std::shared_ptr<State> state;
		Eigen::MatrixXd sol;
		Eigen::MatrixXd nodes; // rest positions, DOF order
		std::vector<bool> free_dof;
		int dim = 3;
	};

	Solved run(const json &args, const bool end_prescribed = false)
	{
		Solved r;
		r.state = std::make_shared<State>();
		r.state->init(args, true);
		r.state->set_max_threads(1);
		r.state->load_mesh();
		r.state->solve(r.sol);
		REQUIRE(r.sol.size() > 0);

		test::VarFormTestAccess::prepare(*r.state->variational_formulation);
		const auto debug = test::VarFormTestAccess::debug_data(*r.state->variational_formulation);
		REQUIRE(debug.bases != nullptr);
		r.nodes.setZero(debug.n_bases, 3);
		for (const auto &element : *debug.bases)
			for (const auto &b : element.bases)
				if (b.global().size() == 1)
					r.nodes.row(b.global()[0].index) = b.global()[0].node;
		r.free_dof.assign(debug.n_bases * 3, false);
		for (int i = 0; i < debug.n_bases; ++i)
			for (int d = 0; d < 3; ++d)
				r.free_dof[3 * i + d] = r.nodes(i, 0) > 1e-9 && (!end_prescribed || r.nodes(i, 0) < 2.0 - 1e-9); // Dirichlet faces x = 0 (and x = 2)
		return r;
	}

	// force fields sampled at every node, as a DOF vector
	Eigen::VectorXd force_field(const Solved &r, const std::string &name)
	{
		const int n = r.nodes.rows();
		io::OutputSample sample;
		sample.node_ids.setLinSpaced(n, 0, n - 1);
		sample.time = 0;
		io::OutputFieldOptions options;
		options.fields = {name};
		const auto fields = r.state->variational_formulation->output_fields(sample, r.sol, options);
		for (const auto &f : fields)
			if (f.name == name)
			{
				REQUIRE(f.values.rows() == n);
				REQUIRE(f.values.cols() == 3);
				Eigen::VectorXd out(3 * n);
				for (int i = 0; i < n; ++i)
					out.segment<3>(3 * i) = f.values.row(i).transpose();
				return out;
			}
		FAIL("field " + name + " not exported");
		return Eigen::VectorXd();
	}

	double free_max(const Solved &r, const Eigen::VectorXd &v)
	{
		double m = 0;
		for (int i = 0; i < v.size(); ++i)
			if (r.free_dof[i])
				m = std::max(m, std::abs(v(i)));
		return m;
	}
} // namespace

TEST_CASE("linear elastic transient run satisfies the implicit-Euler equation", "[linear_elastic][time_int][rb11_envelope]")
{
	const auto dir = scratch_dir("rb11-linear-transient");
	json args = base_args(write_beam(dir), dir / "out");
	args["time"] = {{"dt", 0.05}, {"time_steps", 3}};

	const Solved r = run(args); // segfaulted before RB-11 (InertiaForm before init)
	const Eigen::VectorXd elastic = force_field(r, "elastic_forces");
	const Eigen::VectorXd inertia = force_field(r, "inertia_forces");
	const Eigen::VectorXd body = force_field(r, "body_forces");

	// the beam moved and accelerates: nonzero inertia and displacement
	CHECK(r.sol.cwiseAbs().maxCoeff() > 1e-8);
	CHECK(free_max(r, inertia) > 1e-3 * free_max(r, elastic));

	// exported forces are the assembled equation of the saved step divided
	// by dt^2: -K u + f - M (u - x_tilde) / dt^2 = 0 on the free DOFs
	const Eigen::VectorXd residual = elastic + inertia + body;
	const double scale = std::max({free_max(r, elastic), free_max(r, inertia), free_max(r, body)});
	CAPTURE(free_max(r, residual), scale);
	CHECK(free_max(r, residual) <= 1e-9 * scale);

	// the elastic force is -K u for the assembled stiffness (no dt scaling left)
	StiffnessMatrix K;
	REQUIRE(test::VarFormTestAccess::build_stiffness_mat(*r.state->variational_formulation, K));
	const Eigen::VectorXd Ku = K * r.sol.col(0);
	CAPTURE(free_max(r, elastic + Ku), free_max(r, Ku));
	CHECK(free_max(r, elastic + Ku) <= 1e-9 * free_max(r, Ku));

	// gravity load: f = -rho * rhs per unit volume, downward
	CHECK(free_max(r, body) > 0);
	int i_max = 0;
	body.cwiseAbs().maxCoeff(&i_max);
	CHECK(i_max % 3 == 1);
	CHECK(body(i_max) < 0);
	std::filesystem::remove_all(dir);
}

TEST_CASE("linear elastic quasistatic schedule equals the static solve at any dt", "[linear_elastic][time_int][rb11_envelope]")
{
	const auto dir = scratch_dir("rb11-linear-quasistatic");
	const std::string mesh = write_beam(dir);
	// nonzero prescribed value on the free end, growing with t; the static
	// problem evaluates its expressions at t = 1
	const auto with_end_value = [&](json args) {
		args["boundary_conditions"]["dirichlet_boundary"].push_back({{"id", 2}, {"value", json::array({"0.01*t", "0.02*t", "0"})}});
		return args;
	};

	const Solved static_run = run(with_end_value(base_args(mesh, dir / "static")), /*end_prescribed=*/true);
	const Eigen::VectorXd static_body_check = force_field(static_run, "body_forces");
	CHECK(free_max(static_run, force_field(static_run, "elastic_forces") + static_body_check) <= 1e-9 * free_max(static_run, force_field(static_run, "elastic_forces")));
	const Eigen::VectorXd static_elastic = force_field(static_run, "elastic_forces");
	const Eigen::VectorXd static_body = force_field(static_run, "body_forces");
	CHECK(static_run.sol.cwiseAbs().maxCoeff() > 1e-3);

	for (const auto &[dt, steps] : std::vector<std::pair<double, int>>{{1.0, 1}, {0.5, 2}, {0.25, 4}})
	{
		DYNAMIC_SECTION("dt = " << dt << ", " << steps << " steps")
		{
			json args = with_end_value(base_args(mesh, dir / ("qs-" + std::to_string(steps))));
			args["time"] = {{"dt", dt}, {"time_steps", steps}, {"quasistatic", true}};
			const Solved qs = run(args, /*end_prescribed=*/true);
			REQUIRE(qs.sol.size() == static_run.sol.size());

			// the same static problem at t = 1: identical displacements
			CAPTURE((qs.sol - static_run.sol).cwiseAbs().maxCoeff());
			CHECK((qs.sol - static_run.sol).cwiseAbs().maxCoeff() <= 1e-10 * static_run.sol.cwiseAbs().maxCoeff());

			// physical forces, not divided by dt^2; no inertia in a quasistatic solve
			const Eigen::VectorXd elastic = force_field(qs, "elastic_forces");
			const Eigen::VectorXd body = force_field(qs, "body_forces");
			const Eigen::VectorXd inertia = force_field(qs, "inertia_forces");
			CAPTURE((elastic - static_elastic).cwiseAbs().maxCoeff(), static_elastic.cwiseAbs().maxCoeff());
			CHECK((elastic - static_elastic).cwiseAbs().maxCoeff() <= 1e-9 * static_elastic.cwiseAbs().maxCoeff());
			CHECK((body - static_body).cwiseAbs().maxCoeff() <= 1e-12 * static_body.cwiseAbs().maxCoeff());
			CHECK(inertia.cwiseAbs().maxCoeff() == 0.0);
			// equilibrium of the exported forces on the free DOFs
			CHECK(free_max(qs, elastic + body) <= 1e-9 * free_max(qs, elastic));
		}
	}
	std::filesystem::remove_all(dir);
}

TEST_CASE("linear elastic time-dependent load is exported at the saved step", "[linear_elastic][time_int][rb11_envelope]")
{
	const auto dir = scratch_dir("rb11-linear-load");
	const std::string mesh = write_beam(dir);
	json args = base_args(mesh, dir / "out");
	args["boundary_conditions"]["rhs"] = json::array({"0", "9.81*t", "0"});

	// two quasistatic steps of .5 end at t = 1: the full load
	json two = args;
	two["time"] = {{"dt", 0.5}, {"time_steps", 2}, {"quasistatic", true}};
	const Solved full = run(two);
	// one step of .5 ends at t = .5: half the load, half the displacement
	json one = args;
	one["/output/directory"_json_pointer] = (dir / "out-half").string();
	one["time"] = {{"dt", 0.5}, {"time_steps", 1}, {"quasistatic", true}};
	const Solved half = run(one);

	const Eigen::VectorXd body_full = force_field(full, "body_forces");
	const Eigen::VectorXd body_half = force_field(half, "body_forces");
	CAPTURE(body_full.cwiseAbs().maxCoeff(), body_half.cwiseAbs().maxCoeff());
	CHECK(body_full.cwiseAbs().maxCoeff() > 0);
	CHECK((body_full - 2.0 * body_half).cwiseAbs().maxCoeff() <= 1e-12 * body_full.cwiseAbs().maxCoeff());
	CHECK((full.sol - 2.0 * half.sol).cwiseAbs().maxCoeff() <= 1e-10 * full.sol.cwiseAbs().maxCoeff());
	std::filesystem::remove_all(dir);
}

namespace
{
	const std::vector<std::string> kIntegrators = {"ImplicitEuler", "ImplicitNewmark", "BDF2", "BDF3"};

	Eigen::VectorXd all_max_diff(const Eigen::VectorXd &a, const Eigen::VectorXd &b)
	{
		return (a - b).cwiseAbs();
	}
} // namespace

TEST_CASE("linear elastic quasistatic schedule equals the static solve for every integrator", "[linear_elastic][time_int][rb11_envelope]")
{
	// loads proportional to t (prescribed end value and gravity): the solution at
	// time T is T times the static solution at t = 1, for any number of steps
	const auto dir = scratch_dir("rb11-linear-qs-integrators");
	const std::string mesh = write_beam(dir);
	const auto scaled_loads = [&](json args) {
		args["boundary_conditions"]["rhs"] = json::array({"0", "9.81*t", "0"});
		args["boundary_conditions"]["dirichlet_boundary"].push_back({{"id", 2}, {"value", json::array({"0.01*t", "0.02*t", "0"})}});
		return args;
	};
	const Solved static_run = run(scaled_loads(base_args(mesh, dir / "static")), /*end_prescribed=*/true);
	const Eigen::VectorXd static_elastic = force_field(static_run, "elastic_forces");
	const Eigen::VectorXd static_body = force_field(static_run, "body_forces");
	REQUIRE(static_run.sol.cwiseAbs().maxCoeff() > 1e-3);
	REQUIRE(static_body.cwiseAbs().maxCoeff() > 0);

	const double dt = 0.25;
	for (const std::string &integrator : kIntegrators)
		for (int steps = 1; steps <= 4; ++steps)
		{
			DYNAMIC_SECTION(integrator << ", " << steps << " step(s) of " << dt)
			{
				json args = scaled_loads(base_args(mesh, dir / (integrator + "-" + std::to_string(steps))));
				args["time"] = {{"dt", dt}, {"time_steps", steps}, {"quasistatic", true}, {"integrator", integrator}};
				const Solved qs = run(args, /*end_prescribed=*/true);
				const double T = dt * steps;
				CAPTURE((qs.sol - T * static_run.sol).cwiseAbs().maxCoeff());
				CHECK((qs.sol - T * static_run.sol).cwiseAbs().maxCoeff() <= 1e-10 * static_run.sol.cwiseAbs().maxCoeff());

				const Eigen::VectorXd elastic = force_field(qs, "elastic_forces");
				const Eigen::VectorXd body = force_field(qs, "body_forces");
				const Eigen::VectorXd inertia = force_field(qs, "inertia_forces");
				// physical forces at the saved time, whatever the integrator's history scaling
				CAPTURE(all_max_diff(elastic, T * static_elastic).maxCoeff(), all_max_diff(body, T * static_body).maxCoeff());
				CHECK(all_max_diff(elastic, T * static_elastic).maxCoeff() <= 1e-9 * static_elastic.cwiseAbs().maxCoeff());
				CHECK(all_max_diff(body, T * static_body).maxCoeff() <= 1e-12 * static_body.cwiseAbs().maxCoeff());
				CHECK(inertia.cwiseAbs().maxCoeff() == 0.0);
				CHECK(free_max(qs, elastic + body) <= 1e-9 * free_max(qs, elastic));
			}
		}
	std::filesystem::remove_all(dir);
}

TEST_CASE("linear elastic transient forces follow the solved step for every integrator", "[linear_elastic][time_int][rb11_envelope]")
{
	// gravity from rest, dt = .05; after k steps the exported forces must be
	// -K u_k, the physical load, and -M (u_k - x_tilde_k) / s_k with the
	// predictor and scale of step k as an independently replayed integrator
	// gives them (both individual forces and their sum on the free DOFs, since
	// a common wrong scale would leave the residual zero)
	const auto dir = scratch_dir("rb11-linear-dyn-integrators");
	const std::string mesh = write_beam(dir);
	const double dt = 0.05;
	const Solved static_run = run(base_args(mesh, dir / "static"));
	const Eigen::VectorXd load = force_field(static_run, "body_forces"); // constant gravity load
	StiffnessMatrix K;
	REQUIRE(test::VarFormTestAccess::build_stiffness_mat(*static_run.state->variational_formulation, K));

	for (const std::string &integrator : kIntegrators)
	{
		DYNAMIC_SECTION(integrator)
		{
			std::vector<Eigen::VectorXd> history; // u_1 .. u_4 from runs of 1 .. 4 steps
			for (int steps = 1; steps <= 4; ++steps)
			{
				json args = base_args(mesh, dir / (integrator + "-" + std::to_string(steps)));
				args["time"] = {{"dt", dt}, {"time_steps", steps}, {"integrator", integrator}};
				const Solved dyn = run(args);
				const Eigen::VectorXd u = dyn.sol.col(0);
				const StiffnessMatrix &M = test::VarFormTestAccess::mass_matrix(*dyn.state->variational_formulation);
				REQUIRE(M.rows() == u.size());

				// independent replay of the integrator through the previous solutions
				auto replay = time_integrator::ImplicitTimeIntegrator::construct_time_integrator(integrator);
				const Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(u.size(), 1);
				replay->init(zero, zero, zero, dt);
				for (const Eigen::VectorXd &prev : history)
					replay->update_quantities(prev);
				const double s = replay->acceleration_scaling();
				const Eigen::VectorXd x_tilde = replay->x_tilde();
				history.push_back(u);

				const Eigen::VectorXd elastic = force_field(dyn, "elastic_forces");
				const Eigen::VectorXd inertia = force_field(dyn, "inertia_forces");
				const Eigen::VectorXd body = force_field(dyn, "body_forces");
				const Eigen::VectorXd Ku = K * u;
				const Eigen::VectorXd expected_inertia = -(M * (u - x_tilde)) / s;
				CAPTURE(steps, s, all_max_diff(elastic, -Ku).maxCoeff(), all_max_diff(inertia, expected_inertia).maxCoeff(), all_max_diff(body, load).maxCoeff());
				CHECK(all_max_diff(elastic, -Ku).maxCoeff() <= 1e-9 * Ku.cwiseAbs().maxCoeff());
				CHECK(all_max_diff(body, load).maxCoeff() <= 1e-12 * load.cwiseAbs().maxCoeff());
				CHECK(expected_inertia.cwiseAbs().maxCoeff() > 0);
				CHECK(all_max_diff(inertia, expected_inertia).maxCoeff() <= 1e-9 * expected_inertia.cwiseAbs().maxCoeff());
				const double scale = std::max({free_max(dyn, elastic), free_max(dyn, inertia), free_max(dyn, body)});
				CHECK(free_max(dyn, elastic + inertia + body) <= 1e-9 * scale);
			}
		}
	}
	std::filesystem::remove_all(dir);
}
