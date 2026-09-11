// RB-02 characterization: assertions describe the existing law, not endorsement.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/utils/Logger.hpp>
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
			// RB-18 F7: negative uses |w^T H w| (B), singular-along-normal uses
			// max|H| / dhat^2 (E); only an identically zero Hessian stays zero.
			double expected = name == "zero" ? 0 : name == "near_singular" ? 1e-12
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
		// Three disjoint edge-point pairs expose upper-median batch capping
		// and (RB-18 F1) the relative floor. The historical {0,0,100} batch is
		// now resolved by F7 (zero curvature -> max|H|/dhat^2), see below; a
		// tiny positive outlier is what exercises the floor.
		for (const auto &values : {std::vector<double>{100, 100, 1e-12}, std::vector<double>{1, 10, 1000}})
		{
			auto m = mesh(1, .2, 3);
			Probe f(m, 1, {{"kappa_spread", values[2] < 1 ? 1e4 : 2}});
			for (int i = 0; i < 3; ++i)
				f.driving.block(6 * i, 6 * i, 6, 6) = values[i] * M::Identity(6, 6);
			V x = V::Zero(18);
			f.start(x);
			auto r = sample(f, x);
			// RB-18 F1: the median is over the positive values only, the cap
			// is spread * median and a relative floor median / spread applies.
			const double spread = values[2] < 1 ? 1e4 : 2;
			const double median = values[2] < 1 ? 100 : 10;
			const double cap = spread * median, floor = median / spread;
			check(f.collision_set().size() == 3, "three independent contacts");
			// The parallel broad phase does not order the set: compare sorted.
			std::vector<double> assigned, expected;
			for (size_t i = 0; i < f.collision_set().size(); ++i)
			{
				const double k = f.collision_set()[i].stiffness_scale;
				check(k <= cap * (1 + 1e-12) && k >= floor * (1 - 1e-12), "batch cap and floor");
				assigned.push_back(k);
				expected.push_back(std::min(std::max(values[i], floor), cap));
			}
			std::sort(assigned.begin(), assigned.end());
			std::sort(expected.begin(), expected.end());
			for (size_t i = 0; i < assigned.size(); ++i)
				check(close(assigned[i], expected[i]), "batch resolved coefficient");
			if (values[2] < 1)
				check(r["energy"] > 0 && r["gradient_norm"] > 0, "tiny outlier does not suppress the batch");
			out["batches"].push_back(r);
		}
		// The historical zero-median batch {0,0,100}: F7/E resolves the two
		// singular blocks from max|H| / dhat^2 = 100.
		{
			auto m = mesh(1, .2, 3);
			Probe f(m, 1, {{"kappa_spread", 1e4}});
			f.driving.setZero();
			f.driving.block(12, 12, 6, 6) = 100 * M::Identity(6, 6);
			V x = V::Zero(18);
			f.start(x);
			auto r = sample(f, x);
			for (size_t i = 0; i < f.collision_set().size(); ++i)
				check(close(f.collision_set()[i].stiffness_scale, 100), "zero batch resolved by global scale");
			r["case"] = "historical_zero_median";
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
			f.driving.setZero(); // RB-18 F7: a negative block would now give |q|
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
			// RB-18 F3: the cap is dimensionless (dhat^2-normalized); an
			// initial trim of 1 is below the cap of 1e3 at every length scale.
			check(close(f.barrier_stiffness(), 1.), "conditioning cap length independence (unbound)");
			out["controller_units"].push_back(r);
			// The same fixture with a high initial trim must be capped to the
			// same value, conditioning_cap = 1e3, at every length scale.
			auto m2 = mesh(L, .8);
			Probe g(m2, L);
			g.driving /= (L * L);
			g.set_barrier_stiffness(1e5);
			g.init(z);
			g.refresh_semi_implicit_stiffness(z, true);
			auto r2 = sample(g, z);
			r2["length_scale"] = L;
			r2["initial_trim"] = 1e5;
			check(close(g.barrier_stiffness(), 1e3), "conditioning cap length independence (bound)");
			out["controller_units"].push_back(r2);
		}
		// RB-18 F4: a NaN curvature, an overflowing curvature with nothing to
		// fall back on, and a nonpositive/subnormal form weight are errors
		// rather than a literal 1e30. No invalid geometry or collision query is
		// used in these bounded cases.
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
			json r = {{"case", name}};
			bool threw = false;
			std::string message;
			try
			{
				f.start(z);
			}
			catch (const std::exception &e)
			{
				threw = true;
				message = e.what();
			}
			r["threw"] = threw;
			r["message"] = message;
			check(threw, name + " rejected");
			if (name == "nan_hessian")
				check(message.find("NaN local curvature") != std::string::npos, "NaN message");
			else if (name == "infinite_hessian" || name == "finite_overflow")
				check(message.find("overflowing local curvature") != std::string::npos, "overflow message");
			else
				check(message.find("form weight") != std::string::npos, "weight message");
			out["arithmetic"].push_back(r);
		}
		// An overflowing curvature WITH a batch reference resolves to the cap,
		// and a nonpositive one with a previous value keeps that value (F2).
		{
			auto m = mesh(1, .2, 3);
			Probe f(m, 1, {{"kappa_spread", 2}});
			f.driving.setIdentity();
			f.driving *= 100;
			f.driving(17, 17) = std::numeric_limits<double>::infinity();
			f.start(V::Zero(18));
			auto r = sample(f, V::Zero(18));
			double largest = 0;
			for (size_t i = 0; i < f.collision_set().size(); ++i)
				largest = std::max(largest, f.collision_set()[i].stiffness_scale);
			check(close(largest, 200), "overflow resolves to batch cap");
			out["arithmetic"].push_back({{"case", "overflow_with_batch"}, {"sample", r}});
		}
		{
			auto m = mesh();
			Probe f(m);
			f.start(z);
			check(close(f.collision_set()[0].stiffness_scale, 100), "positive first snapshot");
			f.driving *= -1;
			f.refresh_semi_implicit_stiffness(z, false);
			auto r = sample(f, z);
			check(close(f.collision_set()[0].stiffness_scale, 100), "negative curvature keeps previous coefficient");
			out["arithmetic"].push_back({{"case", "negative_after_positive"}, {"sample", r}});
		}
		// A finite spread can overflow its product, disabling the effective cap.
		{
			auto m = mesh();
			Probe f(m, 1, {{"kappa_spread", 1e308}});
			f.start(z);
			out["overflowing_cap"] = sample(f, z);
			check(out["overflowing_cap"]["cap"] == "infinity", "cap product overflow disables the cap");
			check(close(f.collision_set()[0].stiffness_scale, 100), "overflowing cap leaves a finite coefficient");
		}
		// RB-18 F6: lagged friction follows the trim immediately (the potential
		// is linear in the lagged normal force); the explicit lagging update
		// then re-bases at the new trim and gives the same value.
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
			check(before > 0 && close(lagged, 2 * before) && close(rebuilt, 2 * before), "friction lag follows trim");
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
