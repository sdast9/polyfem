// RBR-04 seam characterization (docs/rb-review-repair-plan-20260920.md):
// the improved-max (convergent) collision set combined with the semi-implicit
// parent-keyed coefficient law, on the real BarrierContactForm. The RB-21
// record's seam analysis predicts a finite energy jump |k1 - k2| / 2 * b(d)
// at the corner of two edges with unequal parent coefficients: the built
// vertex-vertex collision carries the duplicate-removal correction (a
// negative weight) while its stiffness_scale is the positive-parent mean, so
// weight * scale * b(d) is no longer the sum of the parents' potentials.
//
// Assertions characterize; they are not model acceptance. Without
// --expect-guard the unsupported configurations must construct and reproduce
// the finite jump (the failing control on the unrepaired source); with
// --expect-guard they must be refused by name at construction and every
// control must behave exactly as before.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Logger.hpp>

#include <ipc/utils/logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

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

	// The RB-21 corner: edge e0 = (v0, v1) along -x, edge e1 = (v1, v2) down
	// -y, and a free vertex v3 at (x, y). With `closed_free_vertex` the free
	// vertex carries its own edge e2 = (v3, v4) straight up, so that it has a
	// vertex area (area weighting) and the corner v1 sees e2 as a parent.
	// Seam A: x = 0 at y = gap (e0's interior region <-> beyond the corner).
	// Seam B: y = 0 at x = gap (beyond the corner <-> e1's interior region).
	ipc::CollisionMesh corner(bool closed_free_vertex, double x, double y)
	{
		const int n = closed_free_vertex ? 5 : 4;
		M p(n, 2);
		p.row(0) << -1, 0;
		p.row(1) << 0, 0;
		p.row(2) << 0, -1;
		p.row(3) << x, y;
		if (closed_free_vertex)
			p.row(4) << x, y + 1.3;
		Eigen::MatrixXi e(closed_free_vertex ? 3 : 2, 2);
		e.row(0) << 0, 1;
		e.row(1) << 1, 2;
		if (closed_free_vertex)
			e.row(2) << 3, 4;
		ipc::CollisionMesh mesh(p, e);
		mesh.can_collide = [](size_t a, size_t b) { return (a >= 3) != (b >= 3); };
		return mesh;
	}

	struct Config
	{
		std::string name;
		bool improved_max;
		bool area_weighting;
		BarrierStiffnessMode mode;
		std::string identity; // semi-implicit only
		bool heterogeneous;
		bool closed_free_vertex;
		std::string expectation; // "finite_jump" | "continuous" | "historical_stencil_jump"
		bool unsupported;        // refused once the RBR-04 guard exists
	};

	class Probe : public BarrierContactForm
	{
	public:
		M driving;
		const bool semi_implicit;

		Probe(const ipc::CollisionMesh &m, const Config &c)
			: BarrierContactForm(m, /*dhat=*/1, /*avg_mass=*/1, c.area_weighting, c.improved_max, /*use_physical_barrier=*/false,
								 /*use_adaptive_barrier_stiffness=*/c.mode != BarrierStiffnessMode::Fixed,
								 /*is_time_dependent=*/false, /*enable_shape_derivatives=*/false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, c.mode,
								 c.mode == BarrierStiffnessMode::SemiImplicit ? json({{"coefficient_identity", c.identity}}) : json(nullptr),
								 V::Ones(m.num_vertices())),
			  semi_implicit(c.mode == BarrierStiffnessMode::SemiImplicit)
		{
			const int n = m.num_vertices();
			driving = 100 * M::Identity(2 * n, 2 * n);
			if (c.heterogeneous)
			{
				const double d[5] = {30., 300., 3000., 100., 100.};
				for (int v = 0; v < n; ++v)
					driving.block(2 * v, 2 * v, 2, 2) = d[v] * M::Identity(2, 2);
			}
			set_weight(1);
			set_barrier_stiffness(1);
			if (semi_implicit)
				set_system_hessian_provider([this](const V &, StiffnessMatrix &h) { h = driving.sparseView(); });
		}

		// Snapshot at x as [kappa_continuity] does: solve start, then the
		// mid-solve refresh that installs the batch statistics.
		void snapshot(const V &x)
		{
			init(x);
			if (!semi_implicit)
				return;
			refresh_semi_implicit_stiffness(x, false, false);
			solution_changed(x);
			refresh_semi_implicit_stiffness(x, false, false);
		}

		json memo() const
		{
			json out = json::array();
			for (const auto &[key, k] : kappa_cache_)
				out.push_back({{"key", {key[0], key[1], key[2], key[3], key[4]}}, {"raw", number(k)}, {"resolved", number(resolve_stiffness(k, is_continued(key)))}});
			return out;
		}

		// The collision set at the current x: signed weights, parents and the
		// assigned scale, plus sum(weight * scale) = the coefficient that
		// multiplies b(d) when every collision sits at the same distance.
		json describe(const V &x, double &effective_coefficient) const
		{
			const Eigen::MatrixXd surface = compute_displaced_surface(x);
			const auto &E = collision_mesh_.edges();
			const auto &F = collision_mesh_.faces();
			json out = json::array();
			effective_coefficient = 0;
			for (size_t i = 0; i < collision_set_.size(); ++i)
			{
				const auto &c = collision_set_[i];
				json parents = json::array();
				for (const auto &p : c.parents)
					parents.push_back({{"type", int(p.type)}, {"id0", p.id0}, {"id1", p.id1}, {"weight", p.weight}});
				const auto vids = c.vertex_ids(E, F);
				const double d2 = c.compute_distance(c.dof(surface, E, F));
				out.push_back({{"type", collision_set_.is_vertex_vertex(i) ? "VV" : collision_set_.is_edge_vertex(i) ? "EV"
																				: collision_set_.is_edge_edge(i)     ? "EE"
																													 : "other"},
							   {"vertex_ids", {vids[0], vids[1], vids[2], vids[3]}},
							   {"weight", c.weight},
							   {"stiffness_scale", number(c.stiffness_scale)},
							   {"distance", std::sqrt(d2)},
							   {"parents", parents}});
				effective_coefficient += c.weight * c.stiffness_scale;
			}
			return out;
		}

		double barrier_value(double d) const
		{
			return barrier_potential().barrier()(d * d, dhat_ * dhat_);
		}
	};

	// Free-vertex position (x, y) -> full displacement from the mesh built at
	// the snapshot position.
	V displacement(const ipc::CollisionMesh &m, double x, double y)
	{
		V u = V::Zero(2 * m.num_vertices());
		u[6] = x - m.rest_positions()(3, 0);
		u[7] = y - m.rest_positions()(3, 1);
		if (m.num_vertices() > 4)
		{
			u[8] = u[6];
			u[9] = u[7];
		}
		return u;
	}

	struct Sample
	{
		double energy;
		double gradient_norm;
		V gradient;
	};

	Sample sample(Probe &f, const V &u)
	{
		f.solution_changed(u);
		Sample s;
		s.energy = f.value(u);
		f.first_derivative(u, s.gradient);
		s.gradient_norm = s.gradient.norm();
		check(std::isfinite(s.energy) && s.gradient.allFinite(), "nonfinite sample");
		return s;
	}

	// Cross one seam with offsets eps -> 0 and report the limiting behaviour.
	json cross_seam(Probe &f, const ipc::CollisionMesh &m, const std::string &seam,
					const std::function<std::pair<double, double>(double)> &position_of_offset,
					double seam_distance, const Config &c)
	{
		json out;
		out["seam"] = seam;
		out["seam_distance"] = seam_distance;
		const double b = f.barrier_value(seam_distance);
		out["barrier_value_at_seam"] = b;
		json rows = json::array();
		double last_jump = std::numeric_limits<double>::quiet_NaN();
		double left_coefficient = 0, right_coefficient = 0;
		json left_set, right_set;
		const std::vector<double> offsets = {1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9};
		for (const double eps : offsets)
		{
			const auto [xl, yl] = position_of_offset(-eps);
			const auto [xr, yr] = position_of_offset(eps);
			const V ul = displacement(m, xl, yl), ur = displacement(m, xr, yr);
			const Sample left = sample(f, ul);
			double lc;
			const json lset = f.describe(ul, lc);
			const Sample right = sample(f, ur);
			double rc;
			const json rset = f.describe(ur, rc);
			const double jump = right.energy - left.energy;
			rows.push_back({{"offset", eps},
							{"energy_left", left.energy},
							{"energy_right", right.energy},
							{"jump", jump},
							{"jump_over_offset", jump / eps},
							{"gradient_norm_left", left.gradient_norm},
							{"gradient_norm_right", right.gradient_norm},
							{"gradient_jump_norm", (right.gradient - left.gradient).norm()},
							{"sum_weight_scale_left", lc},
							{"sum_weight_scale_right", rc}});
			last_jump = jump;
			left_coefficient = lc;
			right_coefficient = rc;
			left_set = lset;
			right_set = rset;
		}
		out["samples"] = rows;
		out["collisions_left_of_seam"] = left_set;
		out["collisions_right_of_seam"] = right_set;
		// The limit of the jump is (sum w*s right - sum w*s left) * b(d_seam):
		// every collision on either side sits at the seam distance there.
		const double predicted = (right_coefficient - left_coefficient) * b;
		out["predicted_limit_jump"] = predicted;
		out["measured_jump_at_smallest_offset"] = last_jump;
		const double scale = std::max({std::abs(rows.front()["energy_left"].get<double>()), std::abs(rows.front()["energy_right"].get<double>()), 1e-300});
		// Finite jump: the jump converges to a nonzero limit; offset-proportional
		// variation: jump / offset stays bounded (the jump itself -> 0).
		const double j9 = std::abs(rows.back()["jump"].get<double>());
		const double j6 = std::abs(rows[4]["jump"].get<double>());
		const bool finite_jump = j9 > 1e-6 * scale && std::abs(j9 - j6) <= 1e-3 * j6;
		const bool proportional = j9 <= 1e-6 * scale;
		out["classification"] = finite_jump ? "finite_jump" : proportional ? "variation_proportional_to_offset"
																		   : "undetermined";
		check(std::abs(last_jump - predicted) <= 1e-6 * scale + 1e-4 * std::abs(predicted),
			  c.name + " " + seam + ": measured jump does not match sum(weight*scale) prediction");
		if (c.expectation == "continuous")
		{
			check(proportional, c.name + " " + seam + ": expected continuity, found " + out["classification"].get<std::string>());
			check(std::abs(right_coefficient - left_coefficient) <= 1e-9 * std::max(1.0, std::abs(left_coefficient)),
				  c.name + " " + seam + ": effective coefficient changes across the seam");
		}
		else
		{
			check(finite_jump, c.name + " " + seam + ": expected a finite jump, found " + out["classification"].get<std::string>());
		}
		return out;
	}

	json run_config(const Config &c, bool expect_guard)
	{
		json out;
		out["name"] = c.name;
		out["configuration"] = {{"use_improved_max_operator", c.improved_max},
								{"use_area_weighting", c.area_weighting},
								{"use_physical_barrier", false},
								{"barrier_stiffness", c.mode == BarrierStiffnessMode::SemiImplicit ? "semi_implicit" : c.mode == BarrierStiffnessMode::Adaptive ? "adaptive"
																																								: "fixed"},
								{"coefficient_identity", c.mode == BarrierStiffnessMode::SemiImplicit ? json(c.identity) : json(nullptr)},
								{"heterogeneous_parents", c.heterogeneous},
								{"closed_free_vertex", c.closed_free_vertex}};
		out["expectation"] = c.expectation;
		out["unsupported_after_guard"] = c.unsupported;
		// Snapshot with the point interior to e0 at x = -.3, y = .2 (both
		// parents (e0, v3) and (e1, v3) have candidates there).
		const double gap = .2;
		auto m = corner(c.closed_free_vertex, -.3, gap);
		std::unique_ptr<Probe> f;
		try
		{
			f = std::make_unique<Probe>(m, c);
		}
		catch (const std::exception &e)
		{
			out["constructed"] = false;
			out["refusal"] = e.what();
			check(expect_guard && c.unsupported,
				  c.name + ": construction refused unexpectedly: " + std::string(e.what()));
			check(std::string(e.what()).find("improved max operator") != std::string::npos,
				  c.name + ": refusal does not name the improved max operator: " + std::string(e.what()));
			return out;
		}
		out["constructed"] = true;
		check(!(expect_guard && c.unsupported), c.name + ": expected the RBR-04 guard to refuse this configuration");
		out["constructed_flags"] = {{"use_improved_max_operator", f->use_improved_max_operator()},
									{"use_area_weighting", f->use_area_weighting()},
									{"use_physical_barrier", f->use_physical_barrier()},
									{"use_convergent_formulation", f->use_convergent_formulation()},
									{"uses_semi_implicit_stiffness", f->uses_semi_implicit_stiffness()},
									{"collision_set_type", int(f->collision_set().collision_set_type())}};
		check(f->use_improved_max_operator() == c.improved_max, c.name + ": improved-max flag did not reach the collision set");
		check(f->use_area_weighting() == c.area_weighting, c.name + ": area-weighting flag did not reach the collision set");
		f->snapshot(V::Zero(2 * m.num_vertices()));
		out["memo_after_snapshot"] = f->memo();
		// Seam A: the vertical line x = 0 at height gap (distance to the corner
		// and to e0 both equal gap). Seam B: the horizontal line y = 0 at
		// x = gap (distance to the corner and to e1 both equal gap).
		out["seam_A"] = cross_seam(*f, m, "A: e0 interior region <-> beyond the corner (x = 0, y = gap)", [gap](double eps) { return std::make_pair(eps, gap); }, gap, c);
		out["seam_B"] = cross_seam(*f, m, "B: beyond the corner <-> e1 interior region (x = gap, y = 0)", [gap](double eps) { return std::make_pair(gap, eps); }, gap, c);
		out["memo_after_crossings"] = f->memo();
		return out;
	}
} // namespace

int main(int argc, char **argv)
{
	bool expect_guard = false;
	for (int i = 1; i < argc; ++i)
		if (std::strcmp(argv[i], "--expect-guard") == 0)
			expect_guard = true;
	logger().set_level(spdlog::level::off);
	// The toolkit warns on stdout that improved max without area weighting
	// "may lead to incorrect results"; recorded in the record, silenced here
	// so that stdout is the JSON report.
	ipc::logger().set_level(spdlog::level::off);
	const auto SI = BarrierStiffnessMode::SemiImplicit;
	const std::vector<Config> configs = {
		// The target: the public fragment's flags on the semi-implicit form.
		{"improved_max_semi_implicit_parent_heterogeneous", true, false, SI, "parent", true, false, "finite_jump", true},
		{"improved_max_area_weighted_semi_implicit_parent_heterogeneous", true, true, SI, "parent", true, true, "finite_jump", true},
		// Equal coefficients: the positive-parent mean equals every parent, so
		// weight * scale * b(d) is the homogeneous improved-max potential.
		{"improved_max_semi_implicit_parent_homogeneous", true, false, SI, "parent", false, false, "continuous", true},
		{"improved_max_area_weighted_semi_implicit_parent_homogeneous", true, true, SI, "parent", false, true, "continuous", true},
		// The explicit stencil control: discontinuous by its own (documented)
		// construction, with or without improved max.
		{"improved_max_semi_implicit_stencil_heterogeneous", true, false, SI, "stencil", true, false, "historical_stencil_jump", true},
		{"nonconvergent_semi_implicit_stencil_heterogeneous", false, false, SI, "stencil", true, false, "historical_stencil_jump", false},
		// Controls that must stay exactly as they are.
		{"nonconvergent_semi_implicit_parent_heterogeneous", false, false, SI, "parent", true, false, "continuous", false},
		{"nonconvergent_semi_implicit_parent_heterogeneous_closed", false, false, SI, "parent", true, true, "continuous", false},
		{"area_weighted_only_semi_implicit_parent_heterogeneous", false, true, SI, "parent", true, true, "continuous", false},
		{"improved_max_fixed", true, false, BarrierStiffnessMode::Fixed, "", false, false, "continuous", false},
		{"improved_max_adaptive", true, false, BarrierStiffnessMode::Adaptive, "", false, false, "continuous", false},
		{"improved_max_area_weighted_adaptive", true, true, BarrierStiffnessMode::Adaptive, "", false, true, "continuous", false},
		{"nonconvergent_adaptive", false, false, BarrierStiffnessMode::Adaptive, "", false, false, "continuous", false},
	};
	json out;
	out["expect_guard"] = expect_guard;
	out["configurations"] = json::array();
	try
	{
		for (const auto &c : configs)
			out["configurations"].push_back(run_config(c, expect_guard));
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
