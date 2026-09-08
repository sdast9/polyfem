// RB-02 characterization: assertions describe the existing law, not endorsement.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/utils/Logger.hpp>
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
	json number(double x)
	{
		if (std::isnan(x))
			return "NaN";
		if (!std::isfinite(x))
			return x > 0 ? "infinity" : "-infinity";
		return x;
	}
	bool close(double a, double b, double tol = 1e-10)
	{
		return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= tol * (1 + std::abs(b));
	}
	ipc::CollisionMesh mesh(double L = 1, double gap = .2, int count = 1)
	{
		M p(3 * count, 2);
		Eigen::MatrixXi e(count, 2);
		for (int i = 0; i < count; ++i)
		{
			p.row(3 * i) << (4 * i - 1) * L, 0;
			p.row(3 * i + 1) << (4 * i + 1) * L, 0;
			p.row(3 * i + 2) << 4 * i * L, gap * L;
			e.row(i) << 3 * i, 3 * i + 1;
		}
		ipc::CollisionMesh result(p, e);
		result.can_collide = [](size_t a, size_t b) { return a / 3 == b / 3; };
		return result;
	}
	class Probe : public BarrierContactForm
	{
	public:
		M driving;
		int provider_calls = 0;
		Probe(const ipc::CollisionMesh &m, double support = 1, json opts = json::object(), double weight = 1)
			: BarrierContactForm(m, support, 1, false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 BarrierStiffnessMode::SemiImplicit, opts, V::Ones(m.num_vertices()))
		{
			driving = 100 * M::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(weight);
			set_barrier_stiffness(1);
			set_system_hessian_provider([this](const V &, StiffnessMatrix &h) {
				++provider_calls;
				h = driving.sparseView();
			});
		}
		void start(const V &x)
		{
			init(x);
			refresh_semi_implicit_stiffness(x, false);
		}
		json state() const
		{
			json scales = json::array(), raw = json::array();
			for (size_t i = 0; i < collision_set_.size(); ++i)
				scales.push_back(number(collision_set_[i].stiffness_scale));
			for (const auto &entry : kappa_cache_)
				raw.push_back(number(entry.second));
			return {{"scales", scales}, {"cached_pre_cap", raw}, {"median", number(kappa_median_)}, {"cap", std::isfinite(kappa_cap_) ? json(kappa_cap_) : json("infinity")}, {"trim", barrier_stiffness()}, {"provider_calls", provider_calls}, {"snapshot_had_contacts", kappa_snapshot_had_contacts_}};
		}
	};
	json sample(Probe &f, const V &x)
	{
		f.solution_changed(x);
		V g;
		f.first_derivative(x, g);
		StiffnessMatrix h;
		f.second_derivative(x, h);
		check(g.allFinite() && M(h).allFinite() && std::isfinite(f.value(x)), "nonfinite evaluated sample");
		json r = f.state();
		r["energy"] = f.value(x);
		r["gradient_norm"] = g.norm();
		r["point_force_y"] = -g[5];
		r["hessian_norm"] = h.norm();
		return r;
	}
	void derivatives(Probe &f, const V &x, json &out)
	{
		f.solution_changed(x);
		V g;
		f.first_derivative(x, g);
		StiffnessMatrix hs;
		f.second_derivative(x, hs);
		const M h = hs;
		V fd(g.size());
		M fd_h(h.rows(), h.cols());
		const double s = 1e-6, t = 1e-4;
		auto energy = [&](const V &y) { f.solution_changed(y); return f.value(y); };
		for (int i = 0; i < x.size(); ++i)
		{
			V p = x, n = x;
			p[i] += s;
			n[i] -= s;
			fd[i] = (energy(p) - energy(n)) / (2 * s);
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
				fd_h(i, j) = (energy(pp) - energy(pm) - energy(mp) + energy(mm)) / (4 * t * t);
			}
		}
		out = {{"gradient_relative_error", (fd - g).norm() / (1 + g.norm())},
			   {"energy_hessian_relative_error", (fd_h - h).norm() / (1 + h.norm())}};
		check((fd - g).norm() <= 1e-6 * (1 + g.norm()), "energy gradient FD");
		check((fd_h - h).norm() <= 1e-5 * (1 + h.norm()), "energy Hessian FD");
		f.solution_changed(x);
	}
	void post(Probe &f, const V &x, int n)
	{
		V g;
		f.first_derivative(x, g);
		json info = json::object();
		f.post_step(polysolve::nonlinear::PostStepData(n, info, x, g));
	}
} // namespace
int main()
{
	logger().set_level(spdlog::level::off);
	json out;
	try
	{
		V z = V::Zero(6);
		// Rayleigh quotient for a horizontal edge: w contains only y DOFs.
		for (const std::string name : {"positive", "negative", "indefinite_positive_normal", "singular_zero_normal", "zero", "near_singular"})
		{
			auto m = mesh();
			Probe f(m);
			if (name == "negative")
				f.driving *= -1;
			if (name == "zero" || name == "singular_zero_normal")
				f.driving.setZero();
			if (name == "indefinite_positive_normal")
				for (int i = 0; i < 6; i += 2)
					f.driving(i, i) = -100;
			if (name == "singular_zero_normal")
				for (int i = 0; i < 6; i += 2)
					f.driving(i, i) = 100;
			if (name == "near_singular")
				f.driving *= 1e-14;
			f.start(z);
			auto r = sample(f, z);
			double expected = (name == "negative" || name == "zero" || name == "singular_zero_normal") ? 0 : name == "near_singular" ? 1e-12
																																	 : 100;
			check(f.collision_set().size() == 1, "single stencil");
			check(close(f.collision_set()[0].stiffness_scale, expected), name + " coefficient");
			if (expected == 0)
				check(r["energy"] == 0 && r["gradient_norm"] == 0, name + " no barrier");
			if (expected == 0)
			{
				V crossing = z;
				crossing[5] = -.4;
				f.line_search_begin(z, crossing);
				check(!f.is_step_collision_free(z, crossing), "zero coefficient retains CCD");
				const double alpha = f.max_step_size(z, crossing);
				check(alpha > 0 && alpha < 1, "zero coefficient CCD limits crossing");
				f.line_search_end();
				r["ccd_crossing_rejected"] = true;
				r["ccd_alpha"] = alpha;
			}
			out["curvature"][name] = r;
		}
		// Three disjoint edge-point pairs expose upper-median batch capping.
		for (const auto &values : {std::vector<double>{0, 0, 100}, std::vector<double>{1, 10, 1000}})
		{
			auto m = mesh(1, .2, 3);
			Probe f(m, 1, {{"kappa_spread", values[1] == 0 ? 1e4 : 2}});
			for (int i = 0; i < 3; ++i)
				f.driving.block(6 * i, 6 * i, 6, 6) = values[i] * M::Identity(6, 6);
			V x = V::Zero(18);
			f.start(x);
			auto r = sample(f, x);
			const double cap = 2 * values[1];
			for (size_t i = 0; i < f.collision_set().size(); ++i)
				check(f.collision_set()[i].stiffness_scale <= cap, "batch cap");
			check(f.collision_set().size() == 3, "three independent contacts");
			if (cap == 0)
				check(r["energy"] == 0 && r["gradient_norm"] == 0, "zero median suppresses positive contact");
			out["batches"].push_back(r);
		}
		// Existing options, unchanged production defaults; contrast zero median.
		for (double spread : {0., 2.})
		{
			auto m = mesh(1, .2, 3);
			Probe f(m, 1, {{"kappa_spread", spread}});
			f.driving.setZero();
			f.driving.block(12, 12, 6, 6) = 100 * M::Identity(6, 6);
			f.start(V::Zero(18));
			out["cap_control"].push_back(sample(f, V::Zero(18)));
		}
		for (double floor : {0., 1.})
		{
			auto m = mesh();
			Probe f(m, 1, {{"kappa_min", floor}});
			f.driving *= -1;
			f.start(z);
			auto r = sample(f, z);
			check(close(f.collision_set()[0].stiffness_scale, floor), "configured floor");
			out["floor_control"].push_back(r);
		}
		// Trials introduce a contact absent from the refresh batch. Change the
		// callback backing matrix AFTER freezing: trials must not consult it.
		for (double initial_gap : {.2, 1.2})
		{
			auto m = mesh(1, initial_gap);
			Probe f(m);
			f.start(z);
			f.driving *= 7;
			V a = z, b = z;
			a[5] = .3 - initial_gap;
			b[5] = .6 - initial_gap;
			f.line_search_begin(z, a);
			auto first = sample(f, a);
			V first_g;
			StiffnessMatrix first_h;
			f.first_derivative(a, first_g);
			f.second_derivative(a, first_h);
			auto other = sample(f, b);
			auto again = sample(f, a);
			check(first == again, "A B A deterministic frozen state");
			check(f.provider_calls == 1, "trial must not refresh Hessian");
			f.line_search_end();
			Probe reverse(m);
			reverse.start(z);
			reverse.driving *= 7;
			reverse.line_search_begin(z, a);
			sample(reverse, b);
			auto reversed = sample(reverse, a);
			reverse.line_search_end();
			check(close(first["energy"], reversed["energy"]), "B A order energy");
			check(close(first["gradient_norm"], reversed["gradient_norm"]), "B A force");
			check(close(first["hessian_norm"], reversed["hessian_norm"]), "B A Hessian");
			V reverse_g;
			StiffnessMatrix reverse_h;
			reverse.first_derivative(a, reverse_g);
			reverse.second_derivative(a, reverse_h);
			check((first_g - reverse_g).norm() <= 1e-10 * (1 + first_g.norm()), "B A full gradient");
			check((first_h - reverse_h).norm() <= 1e-10 * (1 + first_h.norm()), "B A full Hessian");
			json fd;
			derivatives(f, a, fd);
			auto before = sample(f, a);
			f.refresh_semi_implicit_stiffness(a, false);
			auto after = sample(f, a);
			check(close(after["energy"], 7 * double(before["energy"])), "refresh energy jump");
			check(close(after["gradient_norm"], 7 * double(before["gradient_norm"])), "refresh force jump");
			out["frozen_trials"].push_back({{"initial_gap", initial_gap}, {"first", first}, {"other", other}, {"finite_differences", fd}, {"before_refresh", before}, {"after_refresh", after}});
		}
		// Geometry-only refresh and a nearest-feature transition with one frozen
		// snapshot: an edge-vertex key and a vertex-vertex key need not share k.
		{
			auto m = mesh();
			Probe f(m);
			f.driving = 10 * M::Identity(6, 6);
			f.driving(4, 4) = f.driving(5, 5) = 100;
			f.start(z);
			V shifted = z;
			shifted[4] = .5;
			auto before = sample(f, shifted);
			f.refresh_semi_implicit_stiffness(shifted, false);
			auto after = sample(f, shifted);
			check(close(before["scales"][0], 70), "center snapshot curvature");
			check(close(after["scales"][0], 106.25 / 1.625), "shifted snapshot curvature");
			out["geometry_refresh"] = {{"before", before}, {"after", after}};
			// Return to the center snapshot before the feature-boundary sweep.
			f.solution_changed(z);
			f.refresh_semi_implicit_stiffness(z, false);
			for (double epsilon : {1e-3, 1e-5, 1e-7})
			{
				V left = z, right = z;
				left[4] = 1 - epsilon;
				right[4] = 1 + epsilon;
				f.line_search_begin(z, right);
				auto l = sample(f, left);
				auto r = sample(f, right);
				auto again = sample(f, left);
				// A new key grows the memoization map; compare the evaluated
				// objective, not the diagnostic list of all memoized keys.
				check(l["scales"] == again["scales"] && close(l["energy"], again["energy"])
						  && close(l["gradient_norm"], again["gradient_norm"])
						  && close(l["hessian_norm"], again["hessian_norm"]),
					  "feature trial order repeatability");
				check(close(l["scales"][0], 70) && close(r["scales"][0], 55), "feature coefficient change");
				out["feature_transition"].push_back({{"epsilon", epsilon}, {"left", l}, {"right", r}, {"energy_jump", double(r["energy"]) - double(l["energy"])}});
				f.line_search_end();
			}
		}
		// Converted length L and objective Q: H'=Q/L^2 H; k'=Q/L^4 k.
		auto base_mesh = mesh();
		Probe base(base_mesh);
		base.start(z);
		auto reference = sample(base, z);
		for (double L : {1e-3, 1., 1e3})
			for (double Q : {1e-2, 1., 1e2})
				for (double weight : {.25, 1., 4.})
				{
					auto m = mesh(L);
					Probe f(m, L, json::object(), weight);
					f.driving *= Q / (L * L);
					f.start(z);
					auto r = sample(f, z);
					check(close(double(r["energy"]) / Q, reference["energy"], 1e-9), "converted energy");
					check(close(double(r["gradient_norm"]) * L / Q, reference["gradient_norm"], 1e-9), "converted force");
					check(close(double(r["hessian_norm"]) * L * L / Q, reference["hessian_norm"], 1e-9), "converted Hessian");
					r["length_scale"] = L;
					r["objective_scale"] = Q;
					r["form_weight"] = weight;
					out["units"].push_back(r);
				}
		// The default first-contact conditioning cap contains H/kappa,
		// which scales like length squared: frozen-law covariance does not
		// imply covariance of the controller at unchanged numeric options.
		for (double L : {1e-3, 1., 1e3})
		{
			auto m = mesh(L, .8);
			Probe f(m, L);
			f.driving /= (L * L);
			f.init(z);
			f.refresh_semi_implicit_stiffness(z, true);
			auto r = sample(f, z);
			r["length_scale"] = L;
			check(close(f.barrier_stiffness(), std::min(1., 1e3 * L * L)), "conditioning cap length dependence");
			out["controller_units"].push_back(r);
		}
		// Nonfinite intermediate curvature is replaced by the literal 1e30.
		// No invalid geometry or collision query is used in these bounded cases.
		for (const std::string name : {"nan_hessian", "infinite_hessian", "finite_overflow", "tiny_positive_weight", "zero_weight"})
		{
			auto m = mesh();
			double weight = name == "zero_weight" ? 0 : name == "tiny_positive_weight" ? 1e-310
																					   : 1;
			Probe f(m, 1, json::object(), weight);
			if (name == "nan_hessian")
				f.driving(5, 5) = std::numeric_limits<double>::quiet_NaN();
			if (name == "infinite_hessian")
				f.driving(5, 5) = std::numeric_limits<double>::infinity();
			if (name == "finite_overflow")
			{
				V signs(6);
				signs << 0, -1, 0, -1, 0, 1;
				f.driving = 1e308 * (signs * signs.transpose());
			}
			f.start(z);
			auto r = f.state();
			double k = f.collision_set()[0].stiffness_scale;
			r["finite_assigned_scale"] = std::isfinite(k);
			if (name == "nan_hessian" || name == "infinite_hessian" || name == "finite_overflow")
				check(close(k, 1e30), "nonfinite fallback");
			if (weight < 1e-300)
			{
				check(!std::isfinite(k), "division after fallback can be nonfinite");
				r["evaluation"] = "rejected by probe finite-coefficient guard; no energy evaluation";
			}
			else
				r["sample"] = sample(f, z);
			out["arithmetic"][name] = r;
		}
		// At fixed x, explicit and post-step controller events change the model.
		for (const std::string event : {"bump", "calibrate", "refresh_controller", "first_contact_post", "conditioning_post", "emergency_post", "downward_post", "periodic_post", "stall_retune"})
		{
			double gap = event == "downward_post" ? .98 : (event == "first_contact_post" || event == "conditioning_post") ? 1.2
																														  : .2;
			auto m = mesh(1, gap);
			json opts = json::object();
			if (event == "downward_post")
				opts["controller_interval"] = 1;
			if (event == "periodic_post")
				opts["refresh_interval"] = 1;
			Probe f(m, 1, opts);
			f.start(z);
			V x = z;
			if (event == "first_contact_post" || event == "conditioning_post")
				x[5] = .8 - gap;
			if (event == "conditioning_post")
				f.set_barrier_stiffness(1e5);
			auto before = sample(f, x);
			if (event == "bump")
				f.bump_trim(2);
			if (event == "calibrate")
			{
				V g;
				f.first_derivative(x, g);
				f.set_system_gradient_provider([g](const V &, V &v) { v = -3 * g; });
				check(f.calibrate_trim(x), "calibration accepted");
			}
			if (event == "refresh_controller")
				f.refresh_semi_implicit_stiffness(x, true);
			if (event == "stall_retune")
				f.retune_on_stall(x, 2);
			if (event == "periodic_post" || event == "first_contact_post")
				f.driving *= 2;
			if (event.find("post") != std::string::npos)
			{
				int n = event == "emergency_post" ? 3 : 1;
				for (int i = 1; i <= n; ++i)
					post(f, x, i);
			}
			auto after = sample(f, x);
			check(!close(before["energy"], after["energy"]), event + " objective jump");
			out["retunes"][event] = {{"before", before}, {"after", after}, {"zero_displacement_energy_jump", double(after["energy"]) - double(before["energy"])}};
		}
		// No-op refresh and upward-only calibration controls.
		{
			auto m = mesh();
			Probe f(m);
			f.start(z);
			auto before = sample(f, z);
			f.refresh_semi_implicit_stiffness(z, false);
			auto after = sample(f, z);
			check(close(before["energy"], after["energy"]), "unchanged refresh objective");
			V g;
			f.first_derivative(z, g);
			f.set_system_gradient_provider([g](const V &, V &v) { v = -.5 * g; });
			check(f.calibrate_trim(z) && f.barrier_stiffness() == 1, "calibration never lowers trim");
			out["no_op_controls"] = {{"before", before}, {"after", after}, {"trim_after_lower_target", f.barrier_stiffness()}};
		}
		// A finite spread can overflow its product, disabling the effective cap.
		{
			auto m = mesh();
			Probe f(m, 1, {{"kappa_spread", 1e308}});
			f.start(z);
			out["overflowing_cap"] = sample(f, z);
			check(out["overflowing_cap"]["cap"] == "infinity", "cap product overflow");
		}
		// Lagged friction retains old normal force until explicit lagging update.
		{
			auto m = mesh();
			Probe f(m);
			f.start(z);
			FrictionForm friction(m, nullptr, .01, .5, ipc::BroadPhaseMethod::HASH_GRID, f, 2);
			friction.init_lagging(z);
			V slip = z;
			slip[4] = .02;
			double before = friction.value(slip);
			f.bump_trim(2);
			double lagged = friction.value(slip);
			friction.update_lagging(z, 1);
			double rebuilt = friction.value(slip);
			check(before > 0 && close(before, lagged) && close(rebuilt, 2 * before), "friction lag lifecycle");
			out["friction"] = {{"before", before}, {"after_trim_before_lag", lagged}, {"after_lag_update", rebuilt}};
		}
		out["checks"] = checks;
		out["passed"] = true;
	}
	catch (const std::exception &e)
	{
		out["passed"] = false;
		out["error"] = e.what();
		out["checks"] = checks;
	}
	std::cout << out.dump(2) << std::endl;
	return out["passed"].get<bool>() ? 0 : 1;
}
