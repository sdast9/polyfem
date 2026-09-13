// RB-09 Benchmark B: a linear spring pressing one point onto a floor edge
// through the production semi-implicit BarrierContactForm, compared with the
// exact scalar equilibrium of the same energy and with the hard-contact limit.
//
// Fixture (probe units, L = 1): floor edge (0,1) from -2 to 2 at y = 0, point
// 2 at (0, y). Node-major DOFs: point x = 4, point y = 5. Spring energy
// k/2 |x_point - x_anchor|^2 with the anchor at (0, y_p(t)) pulled below the
// floor step by step; the floor nodes are held fixed. The production flow per
// step: init, Newton with the form's line-search hooks and post_step
// controller, then update_quantities and the between-steps
// update_barrier_stiffness refresh (calibration, continuation, controller).
//
// Reference: with one active vertex-edge collision the form's potential is
// w * trim * kappa_s * b(g^2, dhat^2), b(d, D) = -(d - D)^2 ln(d / D) (the form's
// weight() is w * trim), so the
// equilibrium gap is the root of k (g - y_p) + w trim kappa_s b'(g^2) 2 g on
// (0, dhat). The root is bisected to 1e-15 on the realized w, trim, kappa_s.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Logger.hpp>

#include <polysolve/nonlinear/PostStepData.hpp>

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
	ipc::CollisionMesh mesh(double L = 2)
	{
		M p(3, 2);
		Eigen::MatrixXi e(1, 2);
		p.row(0) << -L, 0;
		p.row(1) << L, 0;
		p.row(2) << 0, 0; // placed by the solution vector
		e.row(0) << 0, 1;
		return ipc::CollisionMesh(p, e);
	}
	class Probe : public BarrierContactForm
	{
	public:
		double k;
		V anchor; // full-size anchor vector (nonzero on the point DOFs only)
		Probe(const ipc::CollisionMesh &m, double dhat, double k_)
			: BarrierContactForm(m, dhat, /*avg_mass=*/1, false, false, false,
								 /*use_adaptive_barrier_stiffness=*/true, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, json::object(), V::Ones(m.num_vertices())),
			  k(k_), anchor(V::Zero(m.num_vertices() * 2))
		{
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const V &x, StiffnessMatrix &h) { h = (k * M::Identity(x.size(), x.size())).sparseView(); });
			set_system_gradient_provider([this](const V &x, V &g) { g = spring_gradient(x); });
		}
		V spring_gradient(const V &x) const
		{
			V g = V::Zero(x.size());
			g.segment<2>(4) = k * (x.segment<2>(4) - anchor.segment<2>(4));
			return g;
		}
		double spring_energy(const V &x) const { return .5 * k * (x.segment<2>(4) - anchor.segment<2>(4)).squaredNorm(); }
		double kappa_s() const { return collision_set_.size() == 1 ? collision_set_[0].stiffness_scale : std::numeric_limits<double>::quiet_NaN(); }
		size_t active() const { return collision_set_.size(); }
	};
	double clamped_log(double d, double D) { return d >= D ? 0.0 : -(d - D) * (d - D) * std::log(d / D); }
	double clamped_log_d(double d, double D) { return d >= D ? 0.0 : -2 * (d - D) * std::log(d / D) - (d - D) * (d - D) / d; }
	// Analytical force of the potential on the point (upward, positive) at gap g.
	double barrier_force(double coeff, double g, double dhat)
	{
		return -coeff * clamped_log_d(g * g, dhat * dhat) * 2 * g;
	}
	// Exact root of k (g - y_p) - F_barrier(g) = 0 on (0, dhat).
	double exact_gap(double k, double y_p, double coeff, double dhat)
	{
		auto f = [&](double g) { return k * (g - y_p) - barrier_force(coeff, g, dhat); };
		double lo = std::numeric_limits<double>::min(), hi = dhat;
		if (f(hi) <= 0)
			return std::numeric_limits<double>::quiet_NaN(); // no contact needed: equilibrium at or above dhat
		for (int i = 0; i < 4000 && hi - lo > 1e-16 * hi; ++i)
		{
			const double mid = .5 * (lo + hi);
			(f(mid) < 0 ? lo : hi) = mid;
		}
		return .5 * (lo + hi);
	}
	// One production-like step: Newton on spring + barrier with the form's
	// hooks; returns iterations. x is full size (floor nodes fixed).
	int newton(Probe &f, V &x, json &trace)
	{
		const double tol = 1e-13 * f.k * std::max(1.0, std::abs(f.anchor[5]));
		f.init(x);
		int it = 0;
		for (; it < 200; ++it)
		{
			f.solution_changed(x);
			V gb, g = f.spring_gradient(x);
			f.first_derivative(x, gb);
			g += gb;
			StiffnessMatrix hs;
			f.second_derivative(x, hs);
			M h = M(hs) + f.k * M::Identity(x.size(), x.size());
			const Eigen::Vector2d gp = g.segment<2>(4);
			if (gp.norm() <= tol)
			{
				polysolve::nonlinear::PostStepData data(it, json::object(), x, g);
				f.post_step(data);
				break;
			}
			const Eigen::Vector2d dir = -h.block<2, 2>(4, 4).ldlt().solve(gp);
			V x1 = x;
			x1.segment<2>(4) += dir;
			f.line_search_begin(x, x1);
			double alpha = std::min(1.0, f.max_step_size(x, x1));
			const double e0 = f.spring_energy(x) + f.value(x);
			V xa;
			for (int ls = 0; ls < 60; ++ls, alpha *= .5)
			{
				xa = x;
				xa.segment<2>(4) += alpha * dir;
				if (!f.is_step_collision_free(x, xa))
					continue;
				const double e1 = f.spring_energy(xa) + f.value(xa);
				if (std::isfinite(e1) && e1 <= e0)
					break;
			}
			f.line_search_end();
			x = xa;
			f.solution_changed(x);
			V gb2, g2 = f.spring_gradient(x);
			f.first_derivative(x, gb2);
			g2 += gb2;
			polysolve::nonlinear::PostStepData data(it + 1, json::object(), x, g2);
			f.post_step(data);
			trace.push_back({{"iteration", it + 1}, {"alpha", alpha}, {"gap", x[5]}, {"trim", f.barrier_stiffness()}, {"gradient_norm", g2.segment<2>(4).norm()}});
		}
		return it;
	}

	json run_case(double k, double dhat, int steps = 4)
	{
		json r = {{"k", k}, {"dhat", dhat}, {"steps", json::array()}};
		auto m = mesh();
		Probe f(m, dhat, k);
		V x = V::Zero(6);
		x[5] = 2 * dhat; // start outside the barrier support
		f.anchor[5] = 2 * dhat;
		f.init(x);
		f.update_barrier_stiffness(x, M()); // production's initial refresh (no contact yet)
		const double y_final = -1.0;
		double max_gap_err = 0, max_force_mismatch = 0, max_gradient_mismatch = 0, max_identity = 0;
		for (int t = 1; t <= steps; ++t)
		{
			f.anchor[5] = 2 * dhat + (y_final - 2 * dhat) * t / steps;
			json step = {{"step", t}, {"anchor_y", f.anchor[5]}, {"trace", json::array()}};
			const int iters = newton(f, x, step["trace"]);
			step["newton_iterations"] = iters;
			if (!step["trace"].empty())
				step["first_iterate_gap_over_dhat"] = step["trace"][0]["gap"].get<double>() / dhat;
			step["gap"] = x[5];
			step["gap_over_dhat"] = x[5] / dhat;
			step["active_collisions"] = f.active();
			step["trim_at_endpoint"] = f.barrier_stiffness();
			step["kappa_s"] = f.kappa_s();
			// ContactForm::weight() already carries the trim (weight_ * barrier_stiffness_)
			const double coeff = f.weight() * f.kappa_s();
			V gb;
			f.solution_changed(x);
			f.first_derivative(x, gb);
			const double form_force = -gb[5];
			const double spring_force = f.k * (f.anchor[5] - x[5]); // downward pull on the point, positive = pushes toward the floor
			step["barrier_force_form"] = form_force;
			step["barrier_force_analytical"] = barrier_force(coeff, x[5], dhat);
			step["spring_force"] = -spring_force; // the spring pulls down: equal and opposite to the barrier at equilibrium
			// Hard-contact reference (anchor below the floor): the point sits on
			// the floor and the spring carries k |y_p|; the barrier solution keeps
			// the point at gap g, so it carries k (|y_p| + g): an overshoot of k g.
			const bool pressed = f.anchor[5] < 0;
			step["hard_contact_counterpart"] = pressed ? "anchor below the floor" : "anchor above the floor: no hard-contact load (barrier pre-contact push only)";
			if (pressed)
			{
				step["hard_contact_force"] = f.k * (0 - f.anchor[5]);
				step["hard_contact_force_overshoot"] = f.k * x[5];
				step["hard_contact_relative_error"] = x[5] / std::abs(f.anchor[5]);
			}
			const double g_ref = exact_gap(f.k, f.anchor[5], coeff, dhat);
			step["gap_reference"] = g_ref;
			if (f.active() == 1 && std::isfinite(g_ref))
			{
				const double e_gap = std::abs(x[5] - g_ref) / dhat;
				const double e_force = std::abs(form_force + spring_force) / std::abs(spring_force);
				const double e_grad = std::abs(form_force - barrier_force(coeff, x[5], dhat)) / std::abs(form_force);
				// realized spring force k (g - y_p) = hard-contact force k (-y_p) + k g
				const double e_id = pressed ? std::abs(-spring_force - f.k * (0 - f.anchor[5]) - f.k * x[5]) / std::abs(f.k * f.anchor[5]) : 0.0;
				max_gap_err = std::max(max_gap_err, e_gap);
				max_force_mismatch = std::max(max_force_mismatch, e_force);
				max_gradient_mismatch = std::max(max_gradient_mismatch, e_grad);
				max_identity = std::max(max_identity, e_id);
				step["errors"] = {{"gap_vs_reference_over_dhat", e_gap}, {"spring_vs_barrier_force", e_force}, {"form_vs_analytical_gradient", e_grad}, {"hard_contact_identity", e_id}};
			}
			// between steps: production's update_quantities + refresh at the published endpoint
			f.update_quantities(t, x);
			f.update_barrier_stiffness(x, M());
			step["trim_after_between_steps_refresh"] = f.barrier_stiffness();
			step["kappa_s_after_refresh"] = f.kappa_s();
			r["steps"].push_back(step);
		}
		r["max_gap_error_over_dhat"] = max_gap_err;
		r["max_spring_vs_barrier_force"] = max_force_mismatch;
		r["max_form_vs_analytical_gradient"] = max_gradient_mismatch;
		r["max_hard_contact_identity"] = max_identity;
		r["failures"] = json::array();
		auto require = [&](bool ok, const std::string &why) {
			++checks;
			if (!ok)
				r["failures"].push_back(why);
		};
		require(max_gap_err <= 1e-10, "T11: equilibrium gap matches the exact scalar root to 1e-10 dhat");
		require(max_force_mismatch <= 1e-10, "T11: spring force equals the barrier force to 1e-10");
		require(max_gradient_mismatch <= 1e-10, "T11: the form's gradient equals w trim kappa_s b'(g^2) 2g to 1e-10");
		require(max_identity <= 1e-12, "T11: hard-contact force overshoot is exactly k g");
		r["passed"] = r["failures"].empty();
		return r;
	}
} // namespace

int main()
{
	logger().set_level(spdlog::level::err);
	json out;
	out["fixture"] = "2D point on a linear spring above a floor edge (L = 1, floor from -2 to 2); semi-implicit BarrierContactForm, form weight 1, trim 1 at start, driving Hessian k I; anchor from 2 dhat to -1 in 4 steps";
	out["cases"] = json::array();
	bool passed = true;
	std::string error;
	for (const auto &[k, dhat] : std::vector<std::pair<double, double>>{{100, .1}, {1, .1}, {1e4, .1}, {100, 1}, {100, .01}})
	{
		try
		{
			json c = run_case(k, dhat);
			if (!c["passed"].get<bool>())
			{
				passed = false;
				for (const auto &f : c["failures"])
					error += "k=" + std::to_string(k) + " dhat=" + std::to_string(dhat) + ": " + f.get<std::string>() + "; ";
			}
			out["cases"].push_back(c);
		}
		catch (const std::exception &e)
		{
			passed = false;
			error += std::string(e.what()) + "; ";
			out["cases"].push_back({{"k", k}, {"dhat", dhat}, {"error", e.what()}});
		}
	}
	out["checks"] = checks;
	out["passed"] = passed;
	if (!error.empty())
		out["error"] = error;
	std::cout << out.dump(1) << std::endl;
	return passed ? 0 : 1;
}
