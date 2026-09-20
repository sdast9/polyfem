#include <polyfem/time_integrator/ImplicitEuler.hpp>
#include <polyfem/time_integrator/ImplicitNewmark.hpp>
#include <polyfem/time_integrator/BDF.hpp>
#include <polyfem/varforms/ElasticVarForm.hpp>

#include <polyfem/utils/Logger.hpp>

#include <finitediff.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace polyfem;
using namespace polyfem::time_integrator;

TEST_CASE("time integrator", "[time_integrator]")
{
	const double dt = GENERATE(0.1, 0.01, 0.001);
	const int n = 10;
	Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(n);
	Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(n);
	Eigen::VectorXd a_prev = Eigen::VectorXd::Zero(n);

	std::shared_ptr<ImplicitTimeIntegrator> time_integrator;
	json params;

	SECTION("Implicit Euler")
	{
		time_integrator = std::make_shared<ImplicitEuler>();
		params = R"({})"_json;
	}
	SECTION("Implicit Newmark")
	{
		time_integrator = std::make_shared<ImplicitNewmark>();
		params = R"({
	        "gamma": 0.5,
	        "beta": 0.25
	    })"_json;
	}
	SECTION("BDF")
	{
		time_integrator = std::make_shared<ImplicitNewmark>();
		params = R"({
	        "steps": 2
	    })"_json;
	}

	time_integrator->init(x_prev, v_prev, a_prev, dt);

	CHECK(time_integrator->dt() == dt);

	const auto f = [&time_integrator](const Eigen::VectorXd &x) -> double {
		return 0.5 * time_integrator->compute_velocity(x).squaredNorm();
	};
	const auto gradf = [&time_integrator](const Eigen::VectorXd &x) -> Eigen::VectorXd {
		return time_integrator->dv_dx() * time_integrator->compute_velocity(x);
	};

	Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
	const int n_rand = 10;
	for (int rand = 0; rand < n_rand; ++rand)
	{
		// Test gradient with finite differences
		{
			const Eigen::VectorXd grad = gradf(x);

			Eigen::VectorXd fgrad;
			fd::finite_gradient(x, f, fgrad);

			if (!fd::compare_gradient(grad, fgrad))
			{
				logger().trace("Gradient mismatch");
				logger().trace("Gradient: {}", grad.transpose());
				logger().trace("Finite gradient: {}", fgrad.transpose());
			}

			CHECK(fd::compare_gradient(grad, fgrad));
		}

		// Test hessian with finite differences
		{
			const Eigen::MatrixXd hess =
				std::pow(time_integrator->dv_dx(), 2) * Eigen::MatrixXd::Identity(n, n);

			Eigen::MatrixXd fhess;
			fd::finite_jacobian(x, gradf, fhess);

			if (!fd::compare_hessian(hess, fhess))
			{
				logger().trace("Hessian mismatch");
				logger().trace("Hessian:\n{}", hess);
				logger().trace("Finite hessian:\n{}", fhess);
			}

			CHECK(fd::compare_hessian(hess, fhess));
		}

		x.setRandom();
		x /= 100;
	}
}

TEST_CASE("first order implicit Euler", "[time_integrator]")
{
	const double dt = 0.1;
	const int n = 10;
	Eigen::VectorXd x_prev = Eigen::VectorXd::LinSpaced(n, 1, n);
	Eigen::VectorXd v_prev = Eigen::VectorXd::Ones(n);
	Eigen::VectorXd a_prev = Eigen::VectorXd::Ones(n);

	ImplicitEuler time_integrator(ImplicitTimeIntegrator::DynamicOrder::First);
	time_integrator.init(x_prev, v_prev, a_prev, dt);

	CHECK((time_integrator.x_tilde() - x_prev).norm() < 1e-12);
	CHECK(time_integrator.acceleration_scaling() == dt);

	const Eigen::VectorXd x = 2 * x_prev;
	CHECK((time_integrator.compute_velocity(x) - (x - x_prev) / dt).norm() < 1e-12);
	CHECK(time_integrator.compute_acceleration(time_integrator.compute_velocity(x)).norm() < 1e-12);

	time_integrator.update_quantities(x);
	CHECK((time_integrator.x_prev() - x).norm() < 1e-12);
	CHECK(time_integrator.a_prev().norm() < 1e-12);
}

TEST_CASE("first order BDF", "[time_integrator]")
{
	const double dt = 0.1;
	const int n = 10;
	Eigen::MatrixXd x_prevs(n, 2);
	x_prevs.col(0) = Eigen::VectorXd::LinSpaced(n, 1, n);
	x_prevs.col(1) = Eigen::VectorXd::LinSpaced(n, 2, 2 * n);
	const Eigen::MatrixXd v_prevs = Eigen::MatrixXd::Ones(n, 2);
	const Eigen::MatrixXd a_prevs = Eigen::MatrixXd::Ones(n, 2);

	BDF time_integrator(2, ImplicitTimeIntegrator::DynamicOrder::First);
	time_integrator.init(x_prevs, v_prevs, a_prevs, dt);

	const Eigen::VectorXd x_tilde = 4.0 / 3.0 * x_prevs.col(0) - 1.0 / 3.0 * x_prevs.col(1);
	CHECK((time_integrator.x_tilde() - x_tilde).norm() < 1e-12);
	CHECK(time_integrator.acceleration_scaling() == 2.0 / 3.0 * dt);

	const Eigen::VectorXd x = 2 * x_prevs.col(0);
	CHECK((time_integrator.compute_velocity(x) - (x - x_tilde) / (2.0 / 3.0 * dt)).norm() < 1e-12);
	CHECK(time_integrator.compute_acceleration(time_integrator.compute_velocity(x)).norm() < 1e-12);

	time_integrator.update_quantities(x);
	CHECK((time_integrator.x_prev() - x).norm() < 1e-12);
	CHECK(time_integrator.a_prev().norm() < 1e-12);
}

TEST_CASE("time integrator factories", "[time_integrator]")
{
	const Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(1);
	const Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(1);
	const Eigen::VectorXd a_prev = Eigen::VectorXd::Zero(1);
	const double dt = 0.1;

	const auto default_integrator =
		ImplicitTimeIntegrator::construct_time_integrator("ImplicitEuler");
	CHECK(std::dynamic_pointer_cast<ImplicitEuler>(default_integrator) != nullptr);

	const auto first_order_bdf = ImplicitTimeIntegrator::construct_bdf_integrator(
		R"({"type": "ImplicitEuler"})"_json,
		ImplicitTimeIntegrator::DynamicOrder::First);
	first_order_bdf->init(x_prev, v_prev, a_prev, dt);
	CHECK(first_order_bdf->steps() == 1);
	CHECK(first_order_bdf->acceleration_scaling() == dt);

	const auto configured_bdf = ImplicitTimeIntegrator::construct_bdf_integrator(
		R"({"type": "BDF", "steps": 2})"_json);
	Eigen::MatrixXd x_prevs = Eigen::MatrixXd::Zero(1, 2);
	configured_bdf->init(x_prevs, x_prevs, x_prevs, dt);
	CHECK(configured_bdf->steps() == 2);
}

// ---------------------------------------------------------------------------
// RB-04 output kinematics / RBR-01 explicit output time state.
//
// The exported velocity and acceleration are those of the saved solution.
// The reviewed helper inferred "history already advanced" from
// x_prev() == solution; a held position (a quasistatic dwell, a body at
// rest) equals the head without being it, so the nonlinear loop -- which
// saves before advancing -- exported the previous step's kinematics for it.
// The owner now states the phase (varform::OutputTimePhase); these tests pin
// the helper against independently evaluated integrator rules.

namespace
{
	using varform::OutputTimePhase;
	using varform::saved_solution_kinematics;

	Eigen::VectorXd constant(const int n, const double value)
	{
		return Eigen::VectorXd::Constant(n, value);
	}

	// Textbook rules, written out here rather than read from the integrators.
	// Histories are most-recent-first, like the integrators' deques.
	struct ReferenceHistory
	{
		std::deque<Eigen::VectorXd> x, v, a;
	};

	std::pair<Eigen::VectorXd, Eigen::VectorXd> implicit_euler_rule(const ReferenceHistory &h, const Eigen::VectorXd &x, const double dt)
	{
		const Eigen::VectorXd v = (x - h.x[0]) / dt;
		return {v, (v - h.v[0]) / dt};
	}

	// Newmark-beta (gamma = 1/2, beta = 1/4): the position update solved for
	// the new acceleration, then the velocity update.
	std::pair<Eigen::VectorXd, Eigen::VectorXd> newmark_rule(const ReferenceHistory &h, const Eigen::VectorXd &x, const double dt)
	{
		const double gamma = .5, beta = .25;
		const Eigen::VectorXd a = (x - h.x[0] - dt * h.v[0] - dt * dt * (.5 - beta) * h.a[0]) / (beta * dt * dt);
		const Eigen::VectorXd v = h.v[0] + dt * ((1 - gamma) * h.a[0] + gamma * a);
		return {v, a};
	}

	// BDF-s with s = min(available history, order): y' at the new time is
	// (y - sum_i alpha_i y_(n-i)) / (beta dt) with the standard coefficients
	// (BDF1: {1}, 1; BDF2: {4/3, -1/3}, 2/3; BDF3: {18/11, -9/11, 2/11}, 6/11),
	// applied to x for v and to v for a.
	std::pair<Eigen::VectorXd, Eigen::VectorXd> bdf_rule(const ReferenceHistory &h, const Eigen::VectorXd &x, const double dt, const int order)
	{
		const int s = std::min<int>(order, h.x.size());
		REQUIRE(s >= 1);
		REQUIRE(s <= 3);
		static const std::vector<std::vector<double>> alphas = {{1.0}, {4.0 / 3.0, -1.0 / 3.0}, {18.0 / 11.0, -9.0 / 11.0, 2.0 / 11.0}};
		static const std::vector<double> betas = {1.0, 2.0 / 3.0, 6.0 / 11.0};
		Eigen::VectorXd sum_x = Eigen::VectorXd::Zero(x.size()), sum_v = Eigen::VectorXd::Zero(x.size());
		for (int i = 0; i < s; ++i)
		{
			sum_x += alphas[s - 1][i] * h.x[i];
			sum_v += alphas[s - 1][i] * h.v[i];
		}
		const Eigen::VectorXd v = (x - sum_x) / (betas[s - 1] * dt);
		return {v, (v - sum_v) / (betas[s - 1] * dt)};
	}
} // namespace

TEST_CASE("Saved-solution kinematics follow the stated output time phase", "[time_integrator][output_kinematics]")
{
	const int n = 4;
	const double dt = .25;
	const Eigen::VectorXd x0 = Eigen::VectorXd::LinSpaced(n, 0, 1);
	const Eigen::VectorXd v0 = constant(n, .5);
	const Eigen::VectorXd a0 = constant(n, -2);

	// Before initialization the kinematics are zero in either phase, not a
	// read of an empty history.
	ImplicitEuler uninitialized;
	for (const auto phase : {OutputTimePhase::HistoryHead, OutputTimePhase::CurrentStepBeforeAdvance})
	{
		const auto [v_none, a_none] = saved_solution_kinematics(uninitialized, x0, phase);
		CHECK(v_none.isZero());
		CHECK(a_none.isZero());
		CHECK(v_none.size() == n);
	}

	ImplicitEuler integrator;
	integrator.init(x0, v0, a0, dt);

	// The initial save reads the supplied initial velocity and acceleration.
	const auto [v_head, a_head] = saved_solution_kinematics(integrator, x0, OutputTimePhase::HistoryHead);
	CHECK(v_head == v0);
	CHECK(a_head == a0);

	// A new endpoint exported before advancing (the nonlinear time loop) is
	// differenced with the integrator's own rule against the history.
	const Eigen::VectorXd x1 = x0 + constant(n, .1);
	const auto [v1, a1] = saved_solution_kinematics(integrator, x1, OutputTimePhase::CurrentStepBeforeAdvance);
	CHECK((v1 - (x1 - x0) / dt).norm() < 1e-14);
	CHECK((a1 - (v1 - v0) / dt).norm() < 1e-14);
	CHECK(integrator.v_prev() == v0); // the helper does not advance the history

	// After advancing (the linear, incompressible and FSI embedding order)
	// the same solution reads the same kinematics from the head.
	integrator.update_quantities(x1);
	const auto [v1_after, a1_after] = saved_solution_kinematics(integrator, x1, OutputTimePhase::HistoryHead);
	CHECK(v1_after == v1);
	CHECK(a1_after == a1);

	// A solution of another size cannot be differenced against this history.
	for (const auto phase : {OutputTimePhase::HistoryHead, OutputTimePhase::CurrentStepBeforeAdvance})
	{
		const auto [v_bad, a_bad] = saved_solution_kinematics(integrator, Eigen::VectorXd::Zero(n + 1), phase);
		CHECK(v_bad.isZero());
		CHECK(a_bad.size() == n + 1);
	}
}

TEST_CASE("A held position exported before the history advances has the current step's kinematics", "[time_integrator][output_kinematics]")
{
	// RBR-01 reproduction (review of 2026-09-20): Implicit Euler, one DOF,
	// dt=.25, from rest; the first step moves to .1 and is advanced into the
	// history (v=.4, a=1.6); the next accepted solution holds .1 and is
	// exported before advancing, as the nonlinear loop does. Its kinematics
	// are v=(.1-.1)/.25=0 and a=(0-.4)/.25=-1.6, not the stored .4 and 1.6
	// that the equality heuristic returned.
	const double dt = .25;
	ImplicitEuler integrator;
	integrator.init(constant(1, 0), constant(1, 0), constant(1, 0), dt);
	const Eigen::VectorXd x1 = constant(1, .1);
	integrator.update_quantities(x1);
	REQUIRE(integrator.v_prev()(0) == .4);
	REQUIRE(integrator.a_prev()(0) == 1.6);

	const Eigen::VectorXd held = x1;
	REQUIRE(held == integrator.x_prev());
	const auto [v, a] = saved_solution_kinematics(integrator, held, OutputTimePhase::CurrentStepBeforeAdvance);
	CHECK(v(0) == 0.0);
	CHECK(a(0) == -1.6);

	// The head at the same time is still the previous step: its kinematics
	// are the stored ones, and they are what an after-advance export of the
	// previous step reads.
	const auto [v_head, a_head] = saved_solution_kinematics(integrator, held, OutputTimePhase::HistoryHead);
	CHECK(v_head(0) == .4);
	CHECK(a_head(0) == 1.6);

	// Once the held position is advanced, the head carries its kinematics.
	integrator.update_quantities(held);
	const auto [v_after, a_after] = saved_solution_kinematics(integrator, held, OutputTimePhase::HistoryHead);
	CHECK(v_after(0) == 0.0);
	CHECK(a_after(0) == -1.6);
}

TEST_CASE("Changed, identical, returning and advanced endpoints have explicit kinematics", "[time_integrator][output_kinematics]")
{
	// Implicit Euler, one DOF, dt=.25, from rest. Positions 0, .1, .1, 0, 0.
	// Expected by hand: v_n = (x_n - x_(n-1)) / .25, a_n = (v_n - v_(n-1)) / .25.
	struct Step
	{
		double x, v, a;
		const char *what;
	};
	const Step steps[] = {
		{.1, .4, 1.6, "changed endpoint"},
		{.1, 0.0, -1.6, "identical endpoint (hold)"},
		{0.0, -.4, -1.6, "return to an earlier position"},
		{0.0, 0.0, 1.6, "identical endpoint at the earlier position"},
	};
	const double dt = .25;
	ImplicitEuler integrator;
	integrator.init(constant(1, 0), constant(1, 0), constant(1, 0), dt);
	for (const Step &step : steps)
	{
		INFO(step.what);
		const Eigen::VectorXd x = constant(1, step.x);
		const auto [v, a] = saved_solution_kinematics(integrator, x, OutputTimePhase::CurrentStepBeforeAdvance);
		CHECK(v(0) == step.v);
		CHECK(a(0) == step.a);

		integrator.update_quantities(x);
		const auto [v_after, a_after] = saved_solution_kinematics(integrator, x, OutputTimePhase::HistoryHead);
		CHECK(v_after(0) == step.v);
		CHECK(a_after(0) == step.a);
	}
}

TEST_CASE("Every integrator's saved kinematics agree with its independently evaluated rule", "[time_integrator][output_kinematics]")
{
	// Three DOFs, dt=.2, nonzero initial velocity and acceleration; a
	// sequence with a changed endpoint, a held one, a return to the initial
	// position and a further change, so that startup (BDF2 after one step,
	// BDF3 after one and two) and mature histories are both exercised.
	const int n = 3;
	const double dt = .2;
	const Eigen::VectorXd x0 = Eigen::VectorXd::LinSpaced(n, -1, 1);
	const Eigen::VectorXd v0 = Eigen::VectorXd::LinSpaced(n, .3, -.3);
	const Eigen::VectorXd a0 = constant(n, .7);
	const std::vector<Eigen::VectorXd> endpoints = {
		x0 + Eigen::VectorXd::LinSpaced(n, .1, .3),
		x0 + Eigen::VectorXd::LinSpaced(n, .1, .3), // held
		x0,                                         // returned
		x0 - Eigen::VectorXd::LinSpaced(n, .2, .05),
		x0 - Eigen::VectorXd::LinSpaced(n, .2, .05), // held again, mature history
	};

	struct Scheme
	{
		std::string name;
		int bdf_order; // 0: not BDF
	};
	const Scheme schemes[] = {{"ImplicitEuler", 0}, {"ImplicitNewmark", 0}, {"BDF2", 2}, {"BDF3", 3}};
	for (const Scheme &scheme : schemes)
	{
		DYNAMIC_SECTION(scheme.name)
		{
			const auto integrator = ImplicitTimeIntegrator::construct_time_integrator(scheme.name);
			REQUIRE(integrator);
			integrator->init(x0, v0, a0, dt);
			ReferenceHistory reference;
			reference.x.push_front(x0);
			reference.v.push_front(v0);
			reference.a.push_front(a0);

			const auto [v_init, a_init] = saved_solution_kinematics(*integrator, x0, OutputTimePhase::HistoryHead);
			CHECK(v_init == v0);
			CHECK(a_init == a0);

			for (size_t k = 0; k < endpoints.size(); ++k)
			{
				const Eigen::VectorXd &x = endpoints[k];
				INFO("step " << k + 1 << (x == reference.x[0] ? " (held)" : ""));
				const auto [v_ref, a_ref] =
					scheme.name == "ImplicitEuler"     ? implicit_euler_rule(reference, x, dt)
					: scheme.name == "ImplicitNewmark" ? newmark_rule(reference, x, dt)
													   : bdf_rule(reference, x, dt, scheme.bdf_order);

				const auto [v, a] = saved_solution_kinematics(*integrator, x, OutputTimePhase::CurrentStepBeforeAdvance);
				CHECK((v - v_ref).cwiseAbs().maxCoeff() <= 1e-12 * std::max(1.0, v_ref.cwiseAbs().maxCoeff()));
				CHECK((a - a_ref).cwiseAbs().maxCoeff() <= 1e-12 * std::max(1.0, a_ref.cwiseAbs().maxCoeff()));

				integrator->update_quantities(x);
				const auto [v_after, a_after] = saved_solution_kinematics(*integrator, x, OutputTimePhase::HistoryHead);
				CHECK(v_after == v);
				CHECK(a_after == a);

				reference.x.push_front(x);
				reference.v.push_front(v_ref);
				reference.a.push_front(a_ref);
			}
		}
	}
}
