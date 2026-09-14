// RB-11 envelope stage: constitutive derivative consistency and rigid-motion
// behaviour of every elastic law on a small tetrahedral fixture.
//
// For each law the production ElasticForm is evaluated on a 6-tet unit cube
// (P1): finite-difference gradient/Hessian consistency at random small
// states, translation invariance of energy/gradient/Hessian, objectivity
// (a rigid rotation of the deformed body leaves the energy unchanged and
// rotates the forces and the tangent), fibre frame indifference for the
// anisotropic laws, and whether the reference configuration is stress-free.
// The linear laws are not objective: their rotation energy grows as theta^4,
// which the test pins as the documented envelope limit.

#include <polyfem/assembler/AssemblerUtils.hpp>
#include <polyfem/solver/forms/ElasticForm.hpp>
#include <polyfem/legacy/State.hpp>
#include <polyfem/utils/Logger.hpp>

#include <finitediff.hpp>

#include <Eigen/Geometry>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace polyfem;
using namespace polyfem::assembler;
using namespace polyfem::solver;
using polyfem::legacy::State;

namespace
{
	struct Law
	{
		std::string name;
		json params;
		bool objective;    // energy invariant under a rigid rotation of the deformed body
		bool stress_free;  // E(0) = 0 and grad E(0) = 0 at the reference configuration
		bool anisotropic;  // has a fibre direction (frame-indifference check)
	};

	std::vector<Law> laws()
	{
		const json fibre = json::array({0.6, 0.0, 0.8});
		return {
			{"LinearElasticity", {{"type", "LinearElasticity"}, {"E", 2e4}, {"nu", 0.3}}, false, true, false},
			{"HookeLinearElasticity", {{"type", "HookeLinearElasticity"}, {"E", 2e4}, {"nu", 0.3}}, false, true, false},
			{"SaintVenant", {{"type", "SaintVenant"}, {"E", 2e4}, {"nu", 0.3}}, true, true, false},
			{"NeoHookean", {{"type", "NeoHookean"}, {"E", 2e4}, {"nu", 0.3}}, true, true, false},
			{"NeoHookean-nearly-incompressible", {{"type", "NeoHookean"}, {"E", 2e4}, {"nu", 0.4999}}, true, true, false},
			{"IsochoricNeoHookean", {{"type", "IsochoricNeoHookean"}, {"E", 2e4}, {"nu", 0.49}}, true, true, false},
			{"MooneyRivlin", {{"type", "MooneyRivlin"}, {"c1", 5e3}, {"c2", 1e3}, {"k", 2e4}}, true, true, false},
			{"MooneyRivlin3Param", {{"type", "MooneyRivlin3Param"}, {"c1", 5e3}, {"c2", 1e3}, {"c3", 5e2}, {"d1", 2e4}}, true, true, false},
			{"MooneyRivlin3ParamSymbolic", {{"type", "MooneyRivlin3ParamSymbolic"}, {"c1", 5e3}, {"c2", 1e3}, {"c3", 5e2}, {"d1", 2e4}}, true, true, false},
			{"UnconstrainedOgden", {{"type", "UnconstrainedOgden"}, {"alphas", json::array({2.0, -2.0})}, {"mus", json::array({5e3, 1e3})}, {"Ds", json::array({1e-4, 1e-4})}}, true, true, false},
			{"IncompressibleOgden", {{"type", "IncompressibleOgden"}, {"c", json::array({5e3, 1e3})}, {"m", json::array({2.0, -2.0})}, {"k", 2e4}}, true, true, false},
			{"FixedCorotational", {{"type", "FixedCorotational"}, {"E", 2e4}, {"nu", 0.3}}, true, true, false},
			{"VolumePenalty", {{"type", "VolumePenalty"}, {"k", 2e4}}, true, true, false},
			{"HGOFiber", {{"type", "HGOFiber"}, {"k1", 1e4}, {"k2", 2.0}, {"fiber_direction", fibre}}, true, true, true},
			{"HGODispersion", {{"type", "HGODispersion"}, {"k1", 1e4}, {"k2", 2.0}, {"kappa", 0.2}, {"fiber_direction", fibre}}, true, true, true},
			{"ActiveFiber", {{"type", "ActiveFiber"}, {"activation", 0.5}, {"Tmax", 1e3}, {"fiber_direction", fibre}}, true, false, true},
			{"MaterialSum(NeoHookean,HGODispersion)",
			 {{"type", "MaterialSum"},
			  {"models", json::array({json{{"type", "NeoHookean"}, {"E", 2e4}, {"nu", 0.3}},
									  json{{"type", "HGODispersion"}, {"k1", 1e4}, {"k2", 2.0}, {"kappa", 0.1}, {"fiber_direction", fibre}}})}},
			 true, true, true},
		};
	}

	// unit cube, six Kuhn tetrahedra, positively oriented
	void cube_mesh(Eigen::MatrixXd &V, Eigen::MatrixXi &T)
	{
		V.resize(8, 3);
		int r = 0;
		for (int i = 0; i < 2; ++i)
			for (int j = 0; j < 2; ++j)
				for (int k = 0; k < 2; ++k)
					V.row(r++) << i, j, k;
		const auto vid = [](int i, int j, int k) { return (i * 2 + j) * 2 + k; };
		T.resize(6, 4);
		const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
		for (int p = 0; p < 6; ++p)
		{
			int cur[3] = {0, 0, 0};
			T(p, 0) = vid(0, 0, 0);
			for (int s = 0; s < 3; ++s)
			{
				cur[perms[p][s]] = 1;
				T(p, s + 1) = vid(cur[0], cur[1], cur[2]);
			}
			const Eigen::RowVector3d a = V.row(T(p, 0)), b = V.row(T(p, 1)), c = V.row(T(p, 2)), d = V.row(T(p, 3));
			if ((b - a).cross(c - a).dot(d - a) < 0)
				std::swap(T(p, 2), T(p, 3));
		}
	}

	struct Fixture
	{
		std::shared_ptr<State> state;
		std::shared_ptr<Assembler> assembler;
		std::unique_ptr<ElasticForm> form;
		Eigen::MatrixXd nodes; // n_bases x 3 rest positions in DOF order
		int n = 0;
	};

	Fixture make_fixture(const Law &law)
	{
		Fixture f;
		json args = json::object();
		args["geometry"] = json::array({{{"mesh", "unused.msh"}}});
		args["materials"] = law.params;
		args["/output/log/level"_json_pointer] = "error";
		args["/output/directory"_json_pointer] = "";
		args["/solver/max_threads"_json_pointer] = 1;
		// build_basis refuses a static problem without Dirichlet nodes; the form
		// itself is evaluated on the full DOF vector regardless of the BCs
		args["boundary_conditions"] = {{"dirichlet_boundary", json::array({{{"id", "all"}, {"value", json::array({0, 0, 0})}}})}};

		f.state = std::make_shared<State>();
		f.state->init(args, true);
		f.state->set_max_threads(1);
		Eigen::MatrixXd V;
		Eigen::MatrixXi T;
		cube_mesh(V, T);
		f.state->load_mesh(V, T);
		f.state->build_basis();
		f.n = f.state->n_bases;
		REQUIRE(f.n == 8);

		f.assembler = AssemblerUtils::make_assembler(law.params["type"].get<std::string>());
		f.state->set_materials(*f.assembler);

		f.form = std::make_unique<ElasticForm>(
			f.state->n_bases, f.state->bases, f.state->geom_bases(), *f.assembler,
			f.state->ass_vals_cache, /*t=*/0.0, /*dt=*/1.0, /*is_volume=*/true);

		f.nodes.resize(f.n, 3);
		for (int i = 0; i < f.n; ++i)
			f.nodes.row(i) = f.state->mesh_nodes->node_position(i);
		return f;
	}

	Eigen::VectorXd rigid_displacement(const Eigen::MatrixXd &nodes, const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const Eigen::VectorXd &u)
	{
		// x' = R (X + u) + t  ->  u' = x' - X
		Eigen::VectorXd out(u.size());
		for (int i = 0; i < nodes.rows(); ++i)
		{
			const Eigen::Vector3d X = nodes.row(i).transpose();
			const Eigen::Vector3d x = X + u.segment<3>(3 * i);
			out.segment<3>(3 * i) = R * x + t - X;
		}
		return out;
	}

	Eigen::Matrix3d rotation(const Eigen::Vector3d &axis, const double angle)
	{
		return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
	}

	Eigen::MatrixXd block_rotation(const Eigen::Matrix3d &R, const int n)
	{
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3 * n, 3 * n);
		for (int i = 0; i < n; ++i)
			B.block<3, 3>(3 * i, 3 * i) = R;
		return B;
	}

	double rel_inf(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b)
	{
		const double scale = std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff());
		if (scale == 0)
			return 0;
		return (a - b).cwiseAbs().maxCoeff() / scale;
	}

	Eigen::VectorXd random_state(const int n, const double amplitude, const int seed)
	{
		std::srand(seed);
		Eigen::VectorXd u = Eigen::VectorXd::Random(3 * n);
		return amplitude * u;
	}
} // namespace

TEST_CASE("elastic laws: finite-difference derivatives", "[rb11_envelope][material]")
{
	for (const Law &law : laws())
	{
		DYNAMIC_SECTION(law.name)
		{
			Fixture f = make_fixture(law);
			ElasticForm &form = *f.form;
			Eigen::VectorXd x = Eigen::VectorXd::Zero(3 * f.n);
			form.init(x);
			for (int trial = 0; trial < 6; ++trial)
			{
				// the stretched random states keep the fibre laws on both sides of I4 = 1
				x = random_state(f.n, 0.02, 100 + trial);
				CAPTURE(law.name, trial);

				Eigen::VectorXd grad;
				form.first_derivative(x, grad);
				Eigen::VectorXd fgrad;
				fd::finite_gradient(x, [&](const Eigen::VectorXd &y) { return form.value(y); }, fgrad, fd::AccuracyOrder::SECOND, 1e-7);
				CAPTURE((grad - fgrad).norm(), grad.norm());
				CHECK(fd::compare_gradient(grad, fgrad, 1e-5));

				StiffnessMatrix hess;
				form.second_derivative(x, hess);
				Eigen::MatrixXd fhess;
				fd::finite_jacobian(
					x, [&](const Eigen::VectorXd &y) { Eigen::VectorXd g; form.first_derivative(y, g); return g; },
					fhess, fd::AccuracyOrder::SECOND, 1e-7);
				CAPTURE((Eigen::MatrixXd(hess) - fhess).norm(), fhess.norm());
				// FixedCorotational's tangent goes through the polar decomposition;
				// its finite-difference Hessian carries more noise (upstream tests it at 1e-4)
				CHECK(fd::compare_hessian(Eigen::MatrixXd(hess), fhess, law.name == "FixedCorotational" ? 1e-4 : 1e-5));
			}
		}
	}
}

TEST_CASE("elastic laws: translation invariance", "[rb11_envelope][material]")
{
	for (const Law &law : laws())
	{
		DYNAMIC_SECTION(law.name)
		{
			Fixture f = make_fixture(law);
			ElasticForm &form = *f.form;
			Eigen::VectorXd u = random_state(f.n, 0.02, 7);
			form.init(u);
			const Eigen::Vector3d t(0.3, -1.2, 2.5);
			const Eigen::VectorXd ut = rigid_displacement(f.nodes, Eigen::Matrix3d::Identity(), t, u);

			Eigen::VectorXd g, gt;
			StiffnessMatrix H, Ht;
			form.first_derivative(u, g);
			form.first_derivative(ut, gt);
			form.second_derivative(u, H);
			form.second_derivative(ut, Ht);
			CAPTURE(law.name);
			CHECK(std::abs(form.value(u) - form.value(ut)) <= 1e-10 * std::max(1.0, std::abs(form.value(u))));
			CHECK(rel_inf(g, gt) <= 1e-9);
			CHECK(rel_inf(Eigen::MatrixXd(H), Eigen::MatrixXd(Ht)) <= 1e-9);
		}
	}
}

TEST_CASE("elastic laws: objectivity under a rigid rotation", "[rb11_envelope][material]")
{
	const Eigen::Vector3d axis(1.0, 2.0, -0.5);
	for (const Law &law : laws())
	{
		DYNAMIC_SECTION(law.name)
		{
			Fixture f = make_fixture(law);
			ElasticForm &form = *f.form;
			Eigen::VectorXd u = random_state(f.n, 0.02, 11);
			form.init(u);
			const Eigen::Matrix3d R = rotation(axis, 0.7);
			const Eigen::Vector3d t(0.1, 0.2, -0.3);
			const Eigen::VectorXd ur = rigid_displacement(f.nodes, R, t, u);
			const Eigen::MatrixXd B = block_rotation(R, f.n);

			Eigen::VectorXd g, gr;
			StiffnessMatrix H, Hr;
			form.first_derivative(u, g);
			form.first_derivative(ur, gr);
			form.second_derivative(u, H);
			form.second_derivative(ur, Hr);
			const double e = form.value(u), er = form.value(ur);
			CAPTURE(law.name, e, er);

			if (law.objective)
			{
				CHECK(std::abs(e - er) <= 1e-9 * std::max(1.0, std::abs(e)));
				// forces rotate with the body, the tangent transforms as R H R^T
				CHECK(rel_inf(gr, B * g) <= 1e-8);
				CHECK(rel_inf(Eigen::MatrixXd(Hr), B * Eigen::MatrixXd(H) * B.transpose()) <= 1e-7);
			}
			else
			{
				// small-strain laws: a pure rotation of the reference body carries
				// the energy c*theta^4 (epsilon = sym(R - I) = O(theta^2)); pinned as
				// the envelope limit, with the quartic scaling to 2 %
				const Eigen::VectorXd zero = Eigen::VectorXd::Zero(3 * f.n);
				const double e1 = form.value(rigid_displacement(f.nodes, rotation(axis, 1e-2), Eigen::Vector3d::Zero(), zero));
				const double e2 = form.value(rigid_displacement(f.nodes, rotation(axis, 2e-2), Eigen::Vector3d::Zero(), zero));
				CAPTURE(e1, e2);
				CHECK(e1 > 0);
				CHECK(e2 / e1 == Catch::Approx(16.0).epsilon(0.02));
				CHECK(std::abs(e - er) > 1e-6 * std::abs(e));
			}
		}
	}
}

TEST_CASE("elastic laws: stress-free reference configuration", "[rb11_envelope][material]")
{
	for (const Law &law : laws())
	{
		DYNAMIC_SECTION(law.name)
		{
			Fixture f = make_fixture(law);
			ElasticForm &form = *f.form;
			const Eigen::VectorXd zero = Eigen::VectorXd::Zero(3 * f.n);
			form.init(zero);
			Eigen::VectorXd g;
			form.first_derivative(zero, g);
			const double e = form.value(zero);
			CAPTURE(law.name, e, g.cwiseAbs().maxCoeff());
			if (law.stress_free)
			{
				CHECK(std::abs(e) <= 1e-12);
				CHECK(g.cwiseAbs().maxCoeff() <= 1e-9);
			}
			else
			{
				// ActiveFiber: the activation is an active stress at the reference by design
				CHECK(g.cwiseAbs().maxCoeff() > 1e-6);
			}
		}
	}
}

TEST_CASE("anisotropic laws: fibre frame indifference", "[rb11_envelope][material]")
{
	// rotating the reference body, its fibre and the deformation together
	// must leave the energy unchanged: E(F; a) = E(Q F Q^T; Q a)
	const Eigen::Matrix3d Q = rotation(Eigen::Vector3d(0.3, -1.0, 0.7), 1.1);
	for (const Law &law : laws())
	{
		if (!law.anisotropic)
			continue;
		DYNAMIC_SECTION(law.name)
		{
			Fixture f = make_fixture(law);

			// the rotated law: fibre Q a, everything else equal
			Law rotated = law;
			const auto rotate_fibre = [&](json &m) {
				const Eigen::Vector3d a(m["fiber_direction"][0].get<double>(), m["fiber_direction"][1].get<double>(), m["fiber_direction"][2].get<double>());
				const Eigen::Vector3d qa = Q * a;
				m["fiber_direction"] = json::array({qa.x(), qa.y(), qa.z()});
			};
			if (rotated.params.contains("models"))
				for (auto &m : rotated.params["models"])
					if (m.contains("fiber_direction"))
						rotate_fibre(m);
			if (rotated.params.contains("fiber_direction"))
				rotate_fibre(rotated.params);
			Fixture fr = make_fixture(rotated);

			// the rotated deformation on the same rest mesh: x' = Q (X + u) with
			// the rest positions themselves rotated is not representable on this
			// mesh, so use the affine identity F' = Q F Q^T instead: for a
			// homogeneous (affine) u the energy depends on F only.
			Eigen::Matrix3d F0;
			F0 << 1.05, 0.02, -0.01, 0.03, 0.97, 0.02, 0.0, 0.01, 1.02;
			const Eigen::Matrix3d F1 = Q * F0 * Q.transpose();
			const auto affine = [&](const Eigen::Matrix3d &F) {
				Eigen::VectorXd out(3 * f.n);
				for (int i = 0; i < f.n; ++i)
					out.segment<3>(3 * i) = (F - Eigen::Matrix3d::Identity()) * f.nodes.row(i).transpose();
				return out;
			};
			const double e0 = f.form->value(affine(F0));
			const double e1 = fr.form->value(affine(F1));
			CAPTURE(law.name, e0, e1);
			CHECK(std::abs(e0 - e1) <= 1e-9 * std::max(1.0, std::abs(e0)));
			CHECK(e0 != 0); // ActiveFiber's active stress makes its energy negative here; the others are positive
		}
	}
}
