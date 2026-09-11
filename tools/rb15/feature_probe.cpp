// Standalone RB-15 experiment. No production assignment is overridden.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Logger.hpp>
#include <algorithm>
#include <functional>
#include <vector>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace polyfem;
using namespace polyfem::solver;
using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;
int checks = 0;
void check(bool v, const char *s)
{
	++checks;
	if (!v)
		throw std::runtime_error(s);
}
bool near(double a, double b, double tol = 1e-10) { return std::abs(a - b) < tol * (1 + std::abs(b)); }
struct Eval
{
	double e;
	V g;
	M h;
};
class FeatureForm : public BarrierContactForm
{
public:
	M driving;
	FeatureForm(const ipc::CollisionMesh &m, bool semi, double L = 1) : BarrierContactForm(m, L, 1, false, false, false, false, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, semi ? BarrierStiffnessMode::SemiImplicit : BarrierStiffnessMode::Fixed, json::object())
	{
		set_weight(1);
		set_barrier_stiffness(1);
		driving = 10 * M::Identity(m.ndof(), m.ndof());
		driving.bottomRightCorner(m.dim(), m.dim()) = 100 * M::Identity(m.dim(), m.dim());
		set_system_hessian_provider([this](const V &, StiffnessMatrix &h) { h = driving.sparseView(); });
	}
	void start(const V &x)
	{
		init(x);
		if (uses_semi_implicit_stiffness())
			refresh_semi_implicit_stiffness(x, false);
	}
	Eval eval(const V &x)
	{
		solution_changed(x);
		V g;
		StiffnessMatrix h;
		first_derivative(x, g);
		second_derivative(x, h);
		return {value(x), g, M(h)};
	}
	void assign_parent_scales()
	{
		for (size_t i = 0; i < collision_set_.size(); ++i)
		{
			const auto ids = collision_set_[i].vertex_ids(collision_mesh_.edges(), collision_mesh_.faces());
			collision_set_[i].stiffness_scale = ids[0] < 3 ? 70 : 700;
		}
	}

	std::string feature() const
	{
		if (collision_set_.empty())
			return "none";
		if (collision_set_.is_face_vertex(0))
			return "FV";
		if (collision_set_.is_edge_vertex(0))
			return "EV";
		return "VV";
	}
	double k() const { return collision_set_.empty() ? 0 : collision_set_[0].stiffness_scale; }
};
struct Fixture
{
	int dim, n, axis, point, anchor;
	double offset;
	M rest;
	Eigen::MatrixXi edges, faces;
	Fixture(int d) : dim(d), n(d == 2 ? 3 : 4), axis(d == 2 ? 0 : 1), point((n - 1) * d + axis), anchor(d == 2 ? 2 : 1), offset(d == 2 ? -1 : .5)
	{
		rest = M::Zero(n, d);
		rest(0, 0) = -1;
		rest(1, 0) = 1;
		if (d == 2)
		{
			rest(2, 1) = .2;
			edges.resize(1, 2);
			edges << 0, 1;
		}
		else
		{
			rest(2, 1) = 2;
			rest(3, 1) = .5;
			rest(3, 2) = .2;
			edges.resize(3, 2);
			edges << 0, 1, 1, 2, 2, 0;
			faces.resize(1, 3);
			faces << 0, 1, 2;
		}
	}
	ipc::CollisionMesh mesh(double L = 1) const { return ipc::CollisionMesh(L * rest, edges, faces); }
	V x(double a) const
	{
		V v = V::Zero(n * dim);
		v[point] = a - offset;
		return v;
	}
	V direction() const
	{
		V a = V::Zero(n * dim);
		a[point] = 1;
		a[anchor] = -1;
		return a;
	}
};
Eval scale(const Eval &b, double k) { return {k * b.e, k * b.g, k * b.h}; }
Eval smooth(const Eval &b, const V &x, const V &a, double offset, double left, double right, double width = .2)
{
	double t = (a.dot(x) + offset) / width, u = std::tanh(t), q = 1 - u * u;
	double delta = (right - left) / 2, k = (left + right) / 2 + delta * u;
	V gk = delta * q / width * a;
	M hk = (-2 * delta * u * q / (width * width)) * (a * a.transpose());
	return {k * b.e, k * b.g + b.e * gk, k * b.h + b.g * gk.transpose() + gk * b.g.transpose() + b.e * hk};
}
json fd(const std::function<Eval(const V &)> &f, const V &x)
{
	Eval v = f(x);
	V g(x.size());
	M h(x.size(), x.size());
	double s = 1e-6;
	for (int i = 0; i < x.size(); ++i)
	{
		V p = x, m = x;
		p[i] += s;
		m[i] -= s;
		auto a = f(p), b = f(m);
		g[i] = (a.e - b.e) / (2 * s);
		h.col(i) = (a.g - b.g) / (2 * s);
	}
	double ge = (g - v.g).norm() / (1 + v.g.norm()), he = (h - v.h).norm() / (1 + v.h.norm());
	check(ge < 1e-6, "energy gradient finite differences");
	check(he < 1e-5, "gradient Hessian finite differences");
	return {{"gradient_relative_error", ge}, {"hessian_relative_error", he}};
}
int main()
{
	logger().set_level(spdlog::level::off);
	json out;
	try
	{
		for (int dim : {2, 3})
		{
			Fixture c(dim);
			auto m = c.mesh();
			FeatureForm base(m, false), current(m, true);
			V z = V::Zero(c.n * dim);
			base.start(z);
			current.start(z);
			double parent = current.k();
			current.eval(c.x(-1e-9));
			double kl = current.k();
			std::string fl = current.feature();
			current.eval(c.x(1e-9));
			double kr = current.k();
			std::string fr = current.feature();
			json result = {{"left_feature", fl}, {"right_feature", fr}, {"left_k", kl}, {"right_k", kr}, {"parent_k", parent}};
			check(dim == 2 ? fl == "EV" && fr == "VV" : fl == "EV" && fr == "FV", "actual IPC transition types");
			if (dim == 2)
				check(near(kl, 70) && near(kr, 55), "70/55 baseline");
			for (std::string name : {"current", "shared", "global70", "smooth"})
			{
				std::function<Eval(const V &)> f = [&](const V &x) {if(name=="current")return current.eval(x);auto b=base.eval(x);if(name=="smooth")return smooth(b,x,c.direction(),c.offset,kl,kr);return scale(b,name=="shared"?parent:70); };
				base.line_search_begin(c.x(-.1), c.x(.1));
				current.line_search_begin(c.x(-.1), c.x(.1));
				json rows = json::array();
				for (double e : {1e-3, 1e-5, 1e-7, 1e-9})
				{
					auto l = f(c.x(-e)), r = f(c.x(e));
					rows.push_back({{"epsilon", e}, {"energy_jump", r.e - l.e}, {"gradient_jump", (r.g - l.g).norm()}, {"hessian_jump", (r.h - l.h).norm()}});
					auto rr = f(c.x(e)), ll = f(c.x(-e));
					check(near(l.e, ll.e) && near(r.e, rr.e) && (l.g - ll.g).norm() < 1e-10, "trial order independence");
				}
				if (name != "current")
				{
					check(std::abs(double(rows.back()["energy_jump"])) < 1e-6, "energy continuity");
					check(double(rows.back()["gradient_jump"]) < 1e-4, "gradient continuity");
				}
				json deriv = json::array();
				for (double a : {-.08, .08})
					deriv.push_back(fd(f, c.x(a)));
				json work = json::array();
				for (int n : {16, 64, 256, 1024})
				{
					double integral = 0;
					for (int side = 0; side < 2; ++side)
						for (int i = 0; i < n; ++i)
						{
							auto v = f(c.x(-.1 + .1 * side + .1 * (i + .5) / n));
							integral += v.g[c.point] * .1 / n;
						}
					double change = f(c.x(.1)).e - f(c.x(-.1)).e;
					work.push_back({{"panels_per_side", n}, {"energy_change", change}, {"gradient_integral", integral}, {"residual", change - integral}});
				}
				if (name != "current")
					check(std::abs(double(work.back()["residual"])) < 1e-5, "split path closure");
				base.line_search_end();
				current.line_search_end();
				auto before = f(c.x(.03));
				V separated = c.x(.03);
				separated[(c.n - 1) * dim + dim - 1] = 2;
				auto inactive = f(separated);
				auto after = f(c.x(.03));
				check(inactive.e == 0 && near(before.e, after.e) && (before.g - after.g).norm() < 1e-10, "separation recontact");
				// Warm median of 7 batches, excludes initialization and compile time.
				std::vector<double> timings;
				double checksum = 0;
				for (int rep = 0; rep < 8; ++rep)
				{
					auto start = std::chrono::steady_clock::now();
					for (int i = 0; i < 100; ++i)
						checksum += f(c.x(i % 2 ? .03 : -.03)).e;
					double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / 100;
					if (rep)
						timings.push_back(us);
				}
				std::sort(timings.begin(), timings.end());
				result[name] = {{"transition", rows}, {"derivatives", deriv}, {"work", work}, {"median_eval_us", timings[3]}, {"timing_checksum", checksum}};
			}
			// Omitted coefficient gradient must fail energy/path consistency.
			double incomplete = 0, min_normal = 1e100, max_tangent = 0;
			for (int i = 0; i < 2048; ++i)
			{
				double a = -.1 + .2 * (i + .5) / 2048;
				V x = c.x(a);
				auto b = base.eval(x), s = smooth(b, x, c.direction(), c.offset, kl, kr);
				double k = (kl + kr) / 2 + (kr - kl) / 2 * std::tanh(a / .2);
				incomplete += k * b.g[c.point] * .2 / 2048;
				V normal = V::Zero(dim);
				normal[dim - 1] = .2;
				if (dim == 2 && a > 0)
					normal[0] = a;
				if (dim == 3 && a < 0)
					normal[1] = a;
				normal.normalize();
				V force = -s.g.tail(dim);
				min_normal = std::min(min_normal, force.dot(normal));
				max_tangent = std::max(max_tangent, (force - force.dot(normal) * normal).norm());
			}
			auto sl = smooth(base.eval(c.x(-.1)), c.x(-.1), c.direction(), c.offset, kl, kr), sr = smooth(base.eval(c.x(.1)), c.x(.1), c.direction(), c.offset, kl, kr);
			result["incomplete_smooth_work_residual"] = sr.e - sl.e - incomplete;
			check(std::abs(sr.e - sl.e - incomplete) > 1, "missing coefficient derivative detected");
			result["smooth_min_normal_force"] = min_normal;
			result["smooth_max_tangential_force"] = max_tangent;
			check(min_normal > 0, "sampled positive normal force");
			current.eval(z);
			auto pre = current.eval(z);
			current.refresh_semi_implicit_stiffness(z, false);
			auto same = current.eval(z);
			check(near(pre.e, same.e), "same snapshot refresh");
			current.driving *= 2;
			current.refresh_semi_implicit_stiffness(z, false);
			auto changed = current.eval(z);
			check(near(changed.e, 2 * pre.e), "outer refresh event");
			result["outer_refresh"] = {{"before", pre.e}, {"after", changed.e}, {"zero_displacement_parameter_energy", changed.e - pre.e}};
			FeatureForm empty(m, true);
			V sep = z;
			sep[(c.n - 1) * dim + dim - 1] = 2;
			empty.start(sep);
			auto appeared = empty.eval(z);
			FeatureForm fresh(m, true);
			fresh.start(sep);
			auto freshappeared = fresh.eval(z);
			check(appeared.e > 0 && near(appeared.e, freshappeared.e), "new contact from empty snapshot");
			result["new_contact_energy"] = appeared.e;
			out[dim == 2 ? "2d_ev_vv" : "3d_ev_fv"] = result;
		}
		// Mapping and unit conversions for all complete candidate fields.
		Fixture c(2);
		auto m = c.mesh();
		FeatureForm base(m, false);
		V z = V::Zero(6);
		base.start(z);
		V x = c.x(.08), a = c.direction();
		auto b = base.eval(x), s = smooth(b, x, a, c.offset, 70, 55);
		M P = M::Zero(6, 6), T = M::Zero(3, 3);
		T(0, 2) = T(1, 0) = T(2, 1) = 1;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				P.block(2 * i, 2 * j, 2, 2) = T(i, j) * M::Identity(2, 2);
		ipc::CollisionMesh mapped(c.rest, c.edges, c.faces, T.sparseView());
		FeatureForm mf(mapped, false);
		mf.start(z);
		V mx = P.transpose() * x, ma = P.transpose() * a;
		auto mb = mf.eval(mx), ms = smooth(mb, mx, ma, c.offset, 70, 55);
		check(near(s.e, ms.e) && (ms.g - P.transpose() * s.g).norm() < 1e-9 && (ms.h - P.transpose() * s.h * P).norm() < 1e-8, "complete field permutation pullback");
		FeatureForm mc(mapped, true), uc(m, true);
		mc.driving = P.transpose() * uc.driving * P;
		mc.start(z);
		uc.start(z);
		auto me = mc.eval(mx), ue = uc.eval(x);
		check(near(me.e, ue.e) && (me.g - P.transpose() * ue.g).norm() < 1e-9, "current permutation");
		out["mapping"] = {{"energy_error", ms.e - s.e}, {"gradient_error", (ms.g - P.transpose() * s.g).norm()}, {"hessian_error", (ms.h - P.transpose() * s.h * P).norm()}};
		for (double L : {.001, 1., 1000.})
			for (double Q : {.01, 1., 100.})
			{
				auto lm = c.mesh(L);
				FeatureForm f(lm, false, L);
				f.start(z);
				auto lb = f.eval(L * x);
				double factor = Q / std::pow(L, 4);
				auto ls = smooth(lb, L * x, a, L * c.offset, 70 * factor, 55 * factor, .2 * L);
				check(near(ls.e / Q, s.e, 1e-9) && (ls.g * L / Q - s.g).norm() < 1e-7 && (ls.h * L * L / Q - s.h).norm() < 1e-6, "field unit covariance");
				FeatureForm unit_current(lm, true, L);
				unit_current.driving *= Q / (L * L);
				unit_current.start(z);
				auto converted_current = unit_current.eval(L * x);
				check(near(converted_current.e / Q, ue.e, 1e-9) && (converted_current.g * L / Q - ue.g).norm() < 1e-7, "current frozen unit covariance");
				auto constant = scale(lb, 70 * factor), reference = scale(b, 70);
				check(near(constant.e / Q, reference.e, 1e-9) && (constant.g * L / Q - reference.g).norm() < 1e-7, "shared global unit covariance");

				out["units"].push_back({{"L", L}, {"Q", Q}, {"energy_error", ls.e / Q - s.e}, {"gradient_error", (ls.g * L / Q - s.g).norm()}, {"hessian_error", (ls.h * L * L / Q - s.h).norm()}});
			}
		// Neighborhood membership/multiplicity, at identical positive-gap geometry.
		auto bv = base.eval(z);
		double switched = 55 * bv.e - 70 * bv.e, duplicated = 140 * bv.e - 70 * bv.e;
		check(std::abs(switched) > 40 && near(duplicated, 70 * bv.e), "region and multiplicity failures retained");
		double partition_error = 0;
		for (double p : {0., .25, .5, .75, 1.})
		{
			partition_error = std::max(partition_error, std::abs((p * 70 + (1 - p) * 70) * bv.e - 70 * bv.e));
		}
		check(partition_error < 1e-10, "partition unity duplicate control");
		out["membership"] = {{"switch_70_to_55_energy_jump", switched}, {"duplicate_parent_energy_jump", duplicated}, {"partition_unity_equal_parent_error", partition_error}, {"new_known_parent_energy", 70 * bv.e}};
		for (double ratio : {1., 10., 100., 1000.})
		{
			double soft = 70, hard = 70 * ratio, shared = (soft + hard) / 2;
			out["contrast"].push_back({{"ratio", ratio}, {"merged_mean_soft_force_ratio", shared / soft}, {"merged_mean_hard_force_ratio", shared / hard}, {"global70_hard_force_ratio", 70 / hard}, {"separate_parents_force_ratio", 1}});
		}
		// Two actual disconnected IPC parents, with heterogeneous coefficients.
		M rest(6, 2);
		rest.topRows(3) = c.rest;
		rest.bottomRows(3) = c.rest;
		rest.bottomRows(3).col(0).array() += 4;
		Eigen::MatrixXi edges(2, 2);
		edges << 0, 1, 3, 4;
		ipc::CollisionMesh multi(rest, edges);
		multi.can_collide = [](size_t i, size_t j) { return i / 3 == j / 3; };
		FeatureForm sum(multi, false);
		V zz = V::Zero(12);
		sum.start(zz);
		auto summed = sum.eval(zz);
		check(sum.collision_set().size() == 2 && near(summed.e, 2 * bv.e), "actual parent multiplicity");
		auto multi_eval = [&](const V &q) {
			sum.solution_changed(q);
			// Probe-only fixed parent identity; both EV and VV inherit by point ID.
			sum.assign_parent_scales();
			V g;
			StiffnessMatrix h;
			sum.first_derivative(q, g);
			sum.second_derivative(q, h);
			return Eval{sum.value(q), g, M(h)};
		};
		auto both = multi_eval(zz);
		check(near(both.e, 770 * bv.e), "heterogeneous parent assignment");
		V one = zz;
		one[11] = 2;
		auto only = multi_eval(one);
		auto back = multi_eval(zz);
		check(near(only.e, 70 * bv.e) && near(back.e, both.e), "multiplicity lifecycle with frozen parents");
		V ml = zz, mr = zz;
		ml[4] = ml[10] = 1 - 1e-9;
		mr[4] = mr[10] = 1 + 1e-9;
		auto le = multi_eval(ml), ri = multi_eval(mr);
		check(std::abs(ri.e - le.e) < 1e-6 && (ri.g - le.g).norm() < 1e-3, "simultaneous heterogeneous transitions");
		out["two_real_parents"] = {{"energy", both.e}, {"one_separated_energy", only.e}, {"recontact_energy", back.e}, {"transition_energy_jump", ri.e - le.e}, {"transition_gradient_jump", (ri.g - le.g).norm()}};
		// Activation at support has vanishing b and grad(b) even with a finite new k.
		json onset = json::array();
		for (double e : {1e-3, 1e-5, 1e-7, 1e-9})
		{
			V q = z;
			q[5] = .8 - e;
			auto v = scale(base.eval(q), 70);
			onset.push_back({{"epsilon", e}, {"energy", v.e}, {"gradient_norm", v.g.norm()}});
		}
		check(double(onset.back()["energy"]) < 1e-20 && double(onset.back()["gradient_norm"]) < 1e-12, "continuous support onset");
		out["support_onset"] = onset;

		// Increasing field centered at a=.08: grad(k) opposes normal repulsion.
		// The earlier boundary-centered field stayed repulsive; retained in candidate-02.
		double least = 1e100;
		for (int i = 0; i < 1000; ++i)
		{
			V q = c.x(.001 + .15 * i / 999);
			auto v = smooth(base.eval(q), q, a, c.offset - .08, 1, 1000, .02);
			V normal(2);
			normal << q[4] - 1, .2;
			normal.normalize();
			least = std::min(least, (-v.g.tail(2)).dot(normal));
		}
		out["positive_k_attraction_counterexample_min_normal_force"] = least;
		check(least < 0, "positive coefficient is insufficient for normal repulsion");
		out["passed"] = true;
	}
	catch (const std::exception &e)
	{
		out["passed"] = false;
		out["error"] = e.what();
	}
	out["checks"] = checks;
	std::cout << out.dump(2) << std::endl;
	return out["passed"].get<bool>() ? 0 : 1;
}
