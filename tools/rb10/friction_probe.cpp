// RB-10 Stage 1: unit-level characterization of the lagged friction path.
// Assertions describe the existing implementation; a "check" that documents a
// defect is phrased as the measured behaviour, not as an endorsement of it.
//
// Fixture: one 2D point above a fixed floor edge (RB-02's probe mesh), a
// semi-implicit BarrierContactForm with a driving Hessian of 100 I, and a
// FrictionForm lagged at the pressed configuration. Every quantity is in
// the probe's own units (L = 1, dhat = 1, form weight 1, trim 1 at start).
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Logger.hpp>

#include <ipc/friction/smooth_friction_mollifier.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace polyfem;
using namespace polyfem::solver;
using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;

namespace
{
	int checks = 0;
	void check(bool ok, const std::string &why)
	{
		++checks;
		if (!ok)
			throw std::runtime_error(why);
	}
	bool close(double a, double b, double tol = 1e-10)
	{
		return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= tol * (1 + std::abs(b));
	}
	// Floor edge (0,1) from -L to L at y = 0; point 2 at (0, gap L) above it.
	// Node-major DOFs: point x = 4, point y = 5.
	ipc::CollisionMesh mesh(double L = 1, double gap = .2)
	{
		M p(3, 2);
		Eigen::MatrixXi e(1, 2);
		p.row(0) << -L, 0;
		p.row(1) << L, 0;
		p.row(2) << 0, gap * L;
		e.row(0) << 0, 1;
		ipc::CollisionMesh result(p, e);
		return result;
	}
	class Probe : public BarrierContactForm
	{
	public:
		M driving;
		Probe(const ipc::CollisionMesh &m, BarrierStiffnessMode mode = BarrierStiffnessMode::SemiImplicit, json opts = json::object())
			: BarrierContactForm(m, /*dhat=*/1, /*avg_mass=*/1, false, false, false,
								 /*use_adaptive_barrier_stiffness=*/mode != BarrierStiffnessMode::Fixed, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 mode, opts, V::Ones(m.num_vertices()))
		{
			driving = 100 * M::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const V &, StiffnessMatrix &h) { h = driving.sparseView(); });
		}
		// Mirrors production: the birth refresh prices the stencil during the
		// solve (post_step), then the between-steps refresh at the published
		// endpoint captures the coefficient that acted there (RB-20).
		void start(const V &x)
		{
			init(x);
			if (uses_semi_implicit_stiffness())
			{
				refresh_semi_implicit_stiffness(x, /*run_trim_controller=*/false, /*published_endpoint=*/false);
				refresh_semi_implicit_stiffness(x, /*run_trim_controller=*/false, /*published_endpoint=*/true);
			}
		}
		double kappa0() const { return collision_set_.size() ? collision_set_[0].stiffness_scale : std::numeric_limits<double>::quiet_NaN(); }
	};
	// Barrier normal force on the point (y component of the barrier gradient
	// at x, sign dropped): what the barrier actually supplies at x.
	double barrier_normal_force(Probe &f, const V &x)
	{
		f.solution_changed(x);
		V g;
		f.first_derivative(x, g);
		return std::abs(g[5]);
	}
	// The friction lag's normal force as the potential uses it: toolkit
	// weight * N_lag * trim_scale (form weight 1).
	double lagged_normal_force(const FrictionForm &fr)
	{
		const auto &set = fr.friction_collision_set();
		if (set.size() != 1)
			return std::numeric_limits<double>::quiet_NaN();
		return set[0].weight * set[0].normal_force_magnitude * fr.trim_scale();
	}
	json fd(const FrictionForm &fr, const V &x)
	{
		V g;
		fr.first_derivative(x, g);
		StiffnessMatrix hs;
		fr.second_derivative(x, hs);
		const M h = hs;
		V fdg(x.size());
		M fdh(h.rows(), h.cols());
		const double s = 1e-7, t = 1e-5;
		for (int i = 0; i < x.size(); ++i)
		{
			V p = x, n = x;
			p[i] += s;
			n[i] -= s;
			fdg[i] = (fr.value(p) - fr.value(n)) / (2 * s);
			for (int j = 0; j < x.size(); ++j)
			{
				V pp = x, pm = x, mp = x, mm = x;
				pp[i] += t;
				pp[j] += t;
				pm[i] += t;
				pm[j] -= t;
				mp[i] -= t;
				mp[j] += t;
				mm[i] -= t;
				mm[j] -= t;
				fdh(i, j) = (fr.value(pp) - fr.value(pm) - fr.value(mp) + fr.value(mm)) / (4 * t * t);
			}
		}
		const double eg = (fdg - g).norm() / (1 + g.norm()), eh = (fdh - h).norm() / (1 + h.norm());
		check(eg <= 1e-6, "friction gradient FD");
		check(eh <= 1e-4, "friction Hessian FD");
		return {{"gradient_relative_error", eg}, {"hessian_relative_error", eh}};
	}
} // namespace

namespace
{
	// Run one probe section; a failed check is recorded on the section and
	// the remaining sections still run, so every measurement is emitted.
	template <typename F>
	void section(json &out, const std::string &name, F &&body)
	{
		json &slot = out[name];
		try
		{
			body(slot);
			slot["passed"] = true;
		}
		catch (const std::exception &e)
		{
			slot["passed"] = false;
			slot["error"] = e.what();
			out["failed_sections"].push_back(name);
		}
	}
} // namespace

int main(int argc, char **argv)
{
	logger().set_level(spdlog::level::err);
	json out;
	out["failed_sections"] = json::array();
	out["fixture"] = "2D point above a floor edge; dhat = 1, gap .2, driving Hessian 100 I, form weight 1, trim 1; mu .3, epsv 1e-3 (probe units)";
	const double mu = .3, eps = 1e-3;
	const V z = V::Zero(6);

	// ----------------------------------------------------------------------
	// P1: frozen-lag constitutive sweep (no time integrator: quasistatic, so
	// the potential's "velocity" is the total displacement x).
	section(out, "P1_constitutive", [&](json &r) {
		auto m = mesh();
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		const double N_barrier = barrier_normal_force(f, z);
		const double N_lag = lagged_normal_force(fr);
		r["barrier_normal_force_at_lag"] = N_barrier;
		r["lagged_normal_force"] = N_lag;
		check(close(N_lag, N_barrier, 1e-12), "lagged normal force equals the barrier force at lag time");
		json sweep = json::array();
		double max_mag_err = 0, min_dissipation = std::numeric_limits<double>::infinity(), max_balance = 0;
		for (double ratio : {-5., -2., -1., -.5, -.25, -.1, .1, .25, .5, 1., 2., 5.})
		{
			V x = z;
			x[4] = ratio * eps;
			V g;
			fr.first_derivative(x, g);
			const double tangential = g[4];
			const double expected = mu * N_lag * ipc::smooth_friction_f1(std::abs(x[4]), eps) * (x[4] > 0 ? 1 : -1);
			const double dissipation = g.dot(x); // g . v with v = x (quasistatic convention)
			const double balance = std::abs(g[0] + g[2] + g[4]) + std::abs(g[1] + g[3] + g[5]);
			max_mag_err = std::max(max_mag_err, std::abs(tangential - expected) / (mu * N_lag));
			min_dissipation = std::min(min_dissipation, dissipation);
			max_balance = std::max(max_balance, balance);
			sweep.push_back({{"slip_over_epsv", ratio}, {"gradient_x_on_point", tangential}, {"expected_mu_N_f1_sign", expected}, {"g_dot_v", dissipation}, {"force_balance_residual", balance}, {"regime", std::abs(ratio) >= 1 ? "sliding" : "smoothing"}});
		}
		r["sweep"] = sweep;
		r["max_relative_magnitude_error"] = max_mag_err;
		r["min_g_dot_v"] = min_dissipation;
		r["max_force_balance_residual"] = max_balance;
		check(max_mag_err <= 1e-12, "sliding/smoothing magnitude mu N f1");
		check(min_dissipation >= 0, "g . v nonnegative for the potential's own velocity");
		check(max_balance <= 1e-12 * mu * N_lag, "friction forces balance between point and edge");
		// FD with the trim scale active (F6), inside and outside epsv.
		f.bump_trim(3);
		r["trim_scale_after_bump3"] = fr.trim_scale();
		check(close(fr.trim_scale(), 3), "trim scale follows the bump");
		V xin = z, xout = z;
		xin[4] = .4 * eps;
		xout[4] = 4 * eps;
		r["fd_inside_epsv_trim3"] = fd(fr, xin);
		r["fd_outside_epsv_trim3"] = fd(fr, xout);
		r["lagged_force_after_bump3"] = lagged_normal_force(fr);
		r["barrier_force_after_bump3"] = barrier_normal_force(f, z);
		check(close(lagged_normal_force(fr), barrier_normal_force(f, z), 1e-12), "F6: lagged normal force follows the trim bump");
	});

	// ----------------------------------------------------------------------
	// P2: normal-force transfer through every retuning path.
	section(out, "P2a_trim_actions", [&](json &r) {
		auto m = mesh();
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		f.bump_trim(2);
		f.bump_trim(.25);
		r["trim_bumps"] = {{"trim", f.barrier_stiffness()}, {"barrier_force", barrier_normal_force(f, z)}, {"lagged_force", lagged_normal_force(fr)}};
		check(close(lagged_normal_force(fr), barrier_normal_force(f, z), 1e-12), "F6 after two trim bumps");
		// calibration against a driving gradient pushing the point down
		f.set_system_gradient_provider([](const V &x, V &g) { g = V::Zero(x.size()); g[5] = 50; });
		const bool calibrated = f.calibrate_trim(z);
		r["calibration"] = {{"calibrated", calibrated}, {"trim", f.barrier_stiffness()}, {"barrier_force", barrier_normal_force(f, z)}, {"lagged_force", lagged_normal_force(fr)}};
		check(close(lagged_normal_force(fr), barrier_normal_force(f, z), 1e-12), "F6 after calibration");
	});
	// (b) mid-solve (non-endpoint) refresh with a changed driving Hessian:
	// with force continuation the lagged stencil (vetted at the endpoint)
	// keeps kappa; without it the barrier force moves while the lag keeps
	// the old magnitude.
	for (bool continuation : {true, false})
	{
		section(out, continuation ? "P2b_midsolve_refresh_continuation_on" : "P2b_midsolve_refresh_continuation_off", [&](json &r) {
			auto m = mesh();
			Probe f(m, BarrierStiffnessMode::SemiImplicit, {{"force_continuation", continuation}});
			f.start(z);
			FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
			fr.init_lagging(z);
			const double before = barrier_normal_force(f, z), kappa_before = f.kappa0();
			f.driving *= 4;
			f.refresh_semi_implicit_stiffness(z, false, /*published_endpoint=*/false);
			const double after = barrier_normal_force(f, z), kappa_after = f.kappa0();
			r = {{"kappa_before", kappa_before}, {"kappa_after", kappa_after}, {"barrier_force_before", before}, {"barrier_force_after", after}, {"lagged_force", lagged_normal_force(fr)}, {"lag_to_barrier_ratio", lagged_normal_force(fr) / after}};
			if (continuation)
				check(close(lagged_normal_force(fr), after, 1e-12), "continuation: lagged stencil keeps kappa through a mid-solve refresh");
			else
				check(close(after, 4 * before, 1e-10) && close(lagged_normal_force(fr), before, 1e-12), "no continuation: barrier force x4, lag stale");
		});
	}
	// (c) stall retune (refresh + trim action) with continuation.
	section(out, "P2c_stall_retune_continuation_on", [&](json &r) {
		auto m = mesh();
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		f.driving *= 4;
		const bool changed = f.retune_on_stall(z, 2);
		r = {{"changed", changed}, {"trim", f.barrier_stiffness()}, {"kappa", f.kappa0()}, {"barrier_force", barrier_normal_force(f, z)}, {"lagged_force", lagged_normal_force(fr)}};
		check(close(lagged_normal_force(fr), barrier_normal_force(f, z), 1e-12), "stall retune: lag follows trim, kappa continued");
	});
	// (d) between-steps refresh at a new endpoint after the point slid (same
	// stencil): continuation keeps kappa, trim controller on; then the next
	// solve's init_lagging re-bases.
	section(out, "P2d_between_steps_refresh", [&](json &r) {
		auto m = mesh();
		Probe f(m);
		f.start(z);
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
		fr.init_lagging(z);
		V x1 = z;
		x1[4] = .05;
		x1[5] = -.05; // pressed closer
		f.solution_changed(x1);
		const double kappa_solve = f.kappa0();
		f.driving *= 4;
		f.update_barrier_stiffness(x1, M()); // published endpoint (between steps)
		const double barrier_at_x1 = barrier_normal_force(f, x1);
		fr.init_lagging(x1);
		r = {{"kappa_during_solve", kappa_solve}, {"kappa_after_refresh", f.kappa0()}, {"trim", f.barrier_stiffness()}, {"barrier_force_at_x1", barrier_at_x1}, {"lagged_force_after_init_lagging", lagged_normal_force(fr)}};
		check(close(f.kappa0(), kappa_solve, 1e-12), "continuation keeps the solve's kappa at the published endpoint");
		check(close(lagged_normal_force(fr), barrier_at_x1, 1e-12), "between-steps refresh then init_lagging is consistent");
	});
	// (e) classic adaptive mode: the lag loop calls update_lagging and THEN
	// update_barrier_stiffness before a re-solve. Reproduce the ordering:
	// build the lag, then let the adaptive rule move the stiffness.
	section(out, "P2e_classic_adaptive_reorder", [&](json &r) {
		auto m = mesh();
		Probe f(m, BarrierStiffnessMode::Adaptive);
		f.init(z);
		// The classic rule clamps kappa = -<gB, gE>/|gB|^2 to [kappa_min,
		// 100 kappa_min] (here ~1e-5 .. 1e-3 in probe units); a driving
		// energy gradient of +.1 then +.5 on the point (a force pressing
		// it into the floor) keeps the balance inside the clamp window.
		V grad_energy = V::Zero(6);
		grad_energy[5] = .1;
		f.update_barrier_stiffness(z, grad_energy);
		const double kappa_a = f.barrier_stiffness();
		FrictionForm fr(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 2);
		fr.init_lagging(z);
		const double lag_force = lagged_normal_force(fr);
		grad_energy[5] = .5; // a different balance at the same x (as a re-solve sees after its own update)
		f.update_barrier_stiffness(z, grad_energy);
		const double kappa_b = f.barrier_stiffness();
		r = {{"stiffness_at_lag", kappa_a}, {"stiffness_after_update", kappa_b}, {"trim_scale", fr.trim_scale()}, {"lagged_force", lag_force}, {"barrier_force_after_update", barrier_normal_force(f, z)}, {"lag_to_barrier_ratio", lag_force / barrier_normal_force(f, z)}, {"note", "observation only: the classic rule is clamped to [kappa_min, 100 kappa_min] and on this fixture sits at the clamp for both balances, so the lag-loop ordering (update_lagging before update_barrier_stiffness) is not exercised here; see the scene-level classic-mode run"}};
		check(close(fr.trim_scale(), 1), "adaptive mode: no trim scale (by design)");
	});

	// ----------------------------------------------------------------------
	// P3: the quasistatic slip convention (velocity := total displacement)
	// versus a time integrator (velocity := (x - x_prev)/dt), on a two-step
	// history: step 1 slides +s (sliding regime), step 2 holds, reverses by
	// s/2 or advances by s/2. The RB-04 convention is g_solved-lag . dx.
	{
		const double s = 5 * eps;
		auto m = mesh();
		Probe f(m);
		f.start(z);
		V x1 = z;
		x1[4] = s;
		V hold = x1, reverse = x1, advance = x1;
		reverse[4] = s / 2;
		advance[4] = 1.5 * s;
		auto measure = [&](FrictionForm &fr, json &r) {
			fr.init_lagging(x1); // the lag a step-2 solve starts from
			for (const auto &[label, x2] : {std::pair<std::string, V>{"hold", hold}, {"reverse", reverse}, {"advance", advance}})
			{
				V g;
				fr.first_derivative(x2, g);
				const V dx = x2 - x1;
				r[label] = {{"tangential_gradient_on_point", g[4]}, {"g_dot_dx", g.dot(dx)}, {"step_slip", dx[4]}, {"force_magnitude_over_mu_N", std::abs(g[4]) / (mu * lagged_normal_force(fr))}};
			}
		};
		section(out, "P3_quasistatic_total_displacement", [&](json &r) {
			FrictionForm fq(m, nullptr, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
			measure(fq, r);
			check(r["hold"]["force_magnitude_over_mu_N"].get<double>() > .999, "quasistatic: a held body keeps a full sliding friction force");
			check(r["reverse"]["g_dot_dx"].get<double>() < 0, "quasistatic: the reversal step has negative g . dx");
			check(r["advance"]["g_dot_dx"].get<double>() > 0, "quasistatic: monotone advance dissipates");
		});
		section(out, "P3_implicit_euler_dt1", [&](json &r) {
			auto integrator = time_integrator::ImplicitTimeIntegrator::construct_time_integrator("ImplicitEuler");
			integrator->init(x1, V::Zero(6), V::Zero(6), /*dt=*/1);
			FrictionForm ft(m, integrator, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
			measure(ft, r);
			check(std::abs(r["hold"]["tangential_gradient_on_point"].get<double>()) < 1e-14, "transient: a held body has no friction force");
			check(r["reverse"]["g_dot_dx"].get<double>() > 0 && r["advance"]["g_dot_dx"].get<double>() > 0, "transient: both directions dissipate");
		});
		section(out, "P3_bdf2_dt1_history_plus_s", [&](json &r) {
			// BDF2 with a history that moved +s in the previous step: the
			// velocity is (3x - 4x1 + x0)/(2dt); a reversal step's increment
			// and velocity can disagree in sign. Observation only.
			auto integrator = time_integrator::ImplicitTimeIntegrator::construct_time_integrator("BDF2");
			M xs(6, 2), vs(6, 2), as(6, 2);
			xs.col(0) = x1;
			xs.col(1) = z; // x_{n-1}
			vs.setZero();
			as.setZero();
			vs.col(0) = x1 - z; // dt = 1
			integrator->init(xs, vs, as, 1);
			FrictionForm fb(m, integrator, eps, mu, ipc::BroadPhaseMethod::HASH_GRID, f, 1);
			measure(fb, r);
			r["reverse"]["velocity_x_on_point"] = integrator->compute_velocity(reverse)[4];
			r["hold"]["velocity_x_on_point"] = integrator->compute_velocity(hold)[4];
		});
	}
	out["checks"] = checks;
	out["passed"] = out["failed_sections"].empty();
	std::cout << out.dump(2) << std::endl;
	return out["passed"].get<bool>() ? 0 : 1;
}
