#include <catch2/catch_test_macros.hpp>

#include <polyfem/State.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/ElasticForm.hpp>

#include "VarFormTestAccess.hpp"

#include <cmath>

using namespace polyfem;
using namespace polyfem::solver;

namespace
{
	template <typename Actual, typename Expected>
	void check_components(const Eigen::MatrixBase<Actual> &actual, const Eigen::MatrixBase<Expected> &expected, double tolerance)
	{
		REQUIRE(actual.rows() == expected.rows());
		REQUIRE(actual.cols() == expected.cols());
		const double error = (actual - expected).cwiseAbs().maxCoeff();
		CAPTURE(error, tolerance);
		CHECK(error <= tolerance);
	}

	struct PairResponse
	{
		double energy = 0;
		Eigen::Vector4d gradient = Eigen::Vector4d::Zero();
		Eigen::Matrix4d hessian = Eigen::Matrix4d::Zero();
		Eigen::Matrix4d projected_hessian = Eigen::Matrix4d::Zero();
	};

	// A single pair with no area weights or mollifier. The 2x4 Jacobian
	// [-I I] maps the two nodal displacements to the relative displacement.
	PairResponse pair_response(const Eigen::Vector2d &r, double support_sq, double stiffness)
	{
		const double s = r.squaredNorm();
		PairResponse response;
		if (s >= support_sq)
			return response;

		const double u = s - support_sq;
		const double log_ratio = std::log(s / support_sq);
		const double b_prime = -2 * u * log_ratio - u * u / s;
		const double b_second = -2 * log_ratio - 4 * u / s + u * u / (s * s);
		const Eigen::Matrix2d radial = r * r.transpose() / s;
		const Eigen::Matrix2d relative_hessian = stiffness * (2 * b_prime * Eigen::Matrix2d::Identity() + 4 * b_second * s * radial);
		// One eigenvector is along r. The other eigenvalue is 2*k*b',
		// which is negative inside the support and is clamped to zero.
		const Eigen::Matrix2d projected = stiffness * (2 * b_prime + 4 * b_second * s) * radial;
		Eigen::Matrix<double, 2, 4> difference;
		difference << -Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity();
		response.energy = -stiffness * u * u * log_ratio;
		response.gradient = difference.transpose() * (2 * stiffness * b_prime * r);
		response.hessian = difference.transpose() * relative_hessian * difference;
		response.projected_hessian = difference.transpose() * projected * difference;
		return response;
	}
} // namespace

TEST_CASE("Frozen semi-implicit pair matches the clamped-log force and tangent", "[physics_oracle][contact][semi_implicit_contact]")
{
	constexpr double support = 0.75;
	constexpr double form_weight = 2.4;
	constexpr double trim = 1.7;
	const Eigen::Vector2d r0(0.3, 0.4);
	Eigen::MatrixXd vertices(2, 2);
	vertices << 0., 0., r0.x(), r0.y();
	const Eigen::MatrixXi edges(0, 2);
	const ipc::CollisionMesh mesh(vertices, edges);
	BarrierContactForm form(mesh, support, 1., false, false, false, false, false, false,
							ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 100000, BarrierStiffnessMode::SemiImplicit,
							json::object(), Eigen::VectorXd::Ones(2));

	Eigen::Matrix4d driving;
	driving << 40., 0., -5., 0.,
		0., 60., 0., -7.,
		-5., 0., 80., 0.,
		0., -7., 0., 100.;
	form.set_weight(form_weight);
	form.set_barrier_stiffness(trim);
	form.set_system_hessian_provider([driving](const Eigen::VectorXd &, StiffnessMatrix &hessian) {
		hessian = driving.sparseView();
	});
	const Eigen::VectorXd origin = Eigen::VectorXd::Zero(4);
	form.init(origin);
	form.refresh_semi_implicit_stiffness(origin, false);

	// The initial normal is (3/5, 4/5). The coupled relative components
	// of H have stiffnesses 40+80+10 and 60+100+14. The two-node
	// contact vector has squared norm 2.
	const double curvature = (0.36 * 130. + 0.64 * 174.) / 2.;
	const double support_sq = support * support;
	const double stiffness = trim * curvature / support_sq;
	const Eigen::Vector2d relative_positions[] = {r0, Eigen::Vector2d(0.4, 0.25), Eigen::Vector2d(0.81, 0.)};
	for (const Eigen::Vector2d &r : relative_positions)
	{
		CAPTURE(r.x(), r.y());
		Eigen::VectorXd displacement = origin;
		displacement.tail<2>() = r - r0;
		form.solution_changed(displacement);
		const auto &collisions = form.collision_set();
		const bool active = r.squaredNorm() < support_sq;
		REQUIRE(collisions.size() == (active ? 1 : 0));
		if (active)
		{
			REQUIRE(collisions.vv_collisions.size() == 1);
			CHECK(std::abs(collisions.vv_collisions[0].stiffness_scale - curvature / (form_weight * support_sq)) <= 1e-11);
		}

		const PairResponse expected = pair_response(r, support_sq, stiffness);
		CHECK(std::abs(form.value(displacement) - expected.energy) <= 1e-11);
		const Eigen::VectorXd nodal_energy = form.value_per_element(displacement);
		check_components(nodal_energy, Eigen::Vector2d::Constant(expected.energy / 2.), 1e-11);
		Eigen::VectorXd gradient;
		form.first_derivative(displacement, gradient);
		check_components(gradient, expected.gradient, 1e-10);
		CHECK((gradient.head<2>() + gradient.tail<2>()).norm() <= 1e-12);
		CHECK(std::abs(r.x() * gradient(3) - r.y() * gradient(2)) <= 1e-12);

		StiffnessMatrix hessian;
		form.set_project_to_psd(false);
		form.second_derivative(displacement, hessian);
		check_components(Eigen::MatrixXd(hessian), expected.hessian, 1e-9);
		form.set_project_to_psd(true);
		form.second_derivative(displacement, hessian);
		check_components(Eigen::MatrixXd(hessian), expected.projected_hessian, 1e-9);
	}
}

TEST_CASE("Elastic form assembles the Hooke tangent at the current frame time", "[physics_oracle][elastic_form][orthotropic]")
{
	const json rotation = {"cos(pi*t/2)", "-sin(pi*t/2)", 0, "sin(pi*t/2)", "cos(pi*t/2)", 0, 0, 0, 1};
	const json args = {
		{"geometry", json::array({{{"mesh", "unused.msh"}}})},
		{"materials", {{"type", "HookeLinearElasticity"}, {"elasticity_tensor", {80000, 70000, 60000, 0.2, 0.17, 0.13, 5000, 12000, 9000}}, {"fiber_direction", rotation}}},
		{"space", {{"discr_order", 1}}},
		{"contact", {{"enabled", false}}},
		{"solver", {{"max_threads", 1}}},
		{"boundary_conditions", {{"dirichlet_boundary", json::array({{{"id", "all"}, {"value", {0, 0, 0}}}})}}},
	};
	State state;
	state.init(args, true);
	state.set_max_threads(1);
	Eigen::MatrixXd vertices(4, 3);
	vertices << 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 1.;
	Eigen::MatrixXi cells(1, 4);
	cells << 0, 1, 2, 3;
	state.set_mesh(vertices, cells);
	const auto formulation = std::dynamic_pointer_cast<varform::ElasticVarForm>(state.variational_formulation);
	REQUIRE(formulation);
	test::VarFormTestAccess::prepare(*formulation);
	const auto data = test::VarFormTestAccess::elastic_form_data(*formulation);
	REQUIRE(data.space.n_bases == 4);
	REQUIRE(data.bases.size() == 1);
	ElasticForm form(data.space.n_bases, data.bases, data.space.geometry_basis_list(), data.assembler, data.assembly_cache, 0., 1., true);

	Eigen::VectorXd unit_shear = Eigen::VectorXd::Zero(12);
	Eigen::VectorXd reference_tangent = Eigen::VectorXd::Zero(12);
	for (const auto &basis : data.bases[0].bases)
	{
		REQUIRE(basis.global().size() == 1);
		const auto &node = basis.global()[0];
		const auto &p = node.node;
		if (p(2) == 1.)
			unit_shear(3 * node.index) = 1.;
		const double gradient_x = p.squaredNorm() == 0. ? -1. : p(0);
		const double gradient_z = p.squaredNorm() == 0. ? -1. : p(2);
		reference_tangent.segment<3>(3 * node.index) << gradient_z / 6., 0., gradient_x / 6.;
	}
	REQUIRE(unit_shear.squaredNorm() == 1.);
	constexpr double displacement = 0.01;
	const Eigen::VectorXd x = displacement * unit_shear;
	for (const int time : {0, 1, 2, 1})
	{
		CAPTURE(time);
		// A z rotation exchanges which local shear plane supplies global
		// XZ. K e_apex is the nodal virtual work of sigma_xz = sigma_zx = G.
		const double shear_modulus = time % 2 ? 5000. : 12000.;
		form.update_quantities(time, x);
		CHECK(std::abs(form.value(x) - displacement * displacement * shear_modulus / 12.) <= 1e-12);
		Eigen::VectorXd gradient;
		form.first_derivative(x, gradient);
		check_components(gradient, displacement * shear_modulus * reference_tangent, 1e-10);
		StiffnessMatrix tangent;
		form.second_derivative(x, tangent);
		check_components(tangent * unit_shear, shear_modulus * reference_tangent, 1e-8);
	}
}
