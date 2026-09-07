#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/utils/Logger.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace polyfem;
using namespace polyfem::solver;
using V = Eigen::VectorXd;

// Characterization experiment, not a production solver or a golden test.
// Run each option in a fresh process: update_collision_set currently has a
// function-static position cache shared across contact-form instances.
int main(int argc, char **argv)
{
	if (argc != 3)
		throw std::runtime_error("usage: contact_floor_probe constraint_floor snapshot_gap");
	logger().set_level(spdlog::level::off);
	const double floor = std::stod(argv[1]);
	const double snapshot_gap = std::stod(argv[2]);
	Eigen::MatrixXd vertices(3, 2);
	vertices << -1, 0, 1, 0, 0, snapshot_gap;
	Eigen::MatrixXi edges(1, 2);
	edges << 0, 1;
	ipc::CollisionMesh mesh(vertices, edges);
	BarrierContactForm contact(mesh, 1., 1., false, false, false, true, false, false,
							   ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
							   BarrierStiffnessMode::SemiImplicit, {{"constraint_floor", floor}}, V::Ones(3));
	contact.set_system_hessian_provider([](const V &, StiffnessMatrix &h) {
		h.resize(6, 6);
		h.setIdentity();
		h *= 100.;
	});
	V x = V::Zero(6);
	contact.init(x);
	contact.update_barrier_stiffness(x, Eigen::MatrixXd());
	contact.set_barrier_stiffness(1.);
	auto at_gap = [&](double gap) -> V {
		V y = V::Zero(6);
		y[5] = gap - snapshot_gap;
		return y;
	};
	auto sample = [&](double gap) {
		V y = at_gap(gap), g;
		contact.solution_changed(y);
		contact.first_derivative(y, g);
		return json{{"gap", gap}, {"energy", contact.value(y)}, {"gradient_norm", g.norm()}, {"point_vertical_gradient", g[5]}, {"edge_vertical_gradient_sum", g[1] + g[3]}, {"collision_count", contact.collision_set().size()}, {"kappa", contact.collision_set().empty() ? 0. : contact.collision_set()[0].stiffness_scale}};
	};
	json result = {{"constraint_floor", floor}, {"snapshot_gap", snapshot_gap}, {"initial_collision_count", contact.collision_set().size()}};
	// Freeze the snapshot throughout a trial interval, including a pair that
	// did not exist at the snapshot when snapshot_gap is 1.2 (> dhat=1).
	V begin = at_gap(0.00010001), end = at_gap(0.00009999);
	contact.solution_changed(begin);
	contact.line_search_begin(begin, end);
	for (double gap : {0.00010001, 0.000100001, 0.000099999, 0.00009999, 0.00005, 0.000100001})
		result["trial_samples"].push_back(sample(gap));
	contact.line_search_end();
	const double eps = 1e-9, center = 1e-4;
	const double above = sample(center + eps)["energy"];
	const double below = sample(center - eps)["energy"];
	result["threshold_secant_dE_dgap"] = (above - below) / (2 * eps);
	// Model the same full-coordinate projection and subsequent elimination
	// used by the caller. Prescribed velocities are a direction-level example,
	// not a claim that the reduced solver advances BCs during its line search.
	x = at_gap(0.000099999);
	contact.solution_changed(x);
	for (double edge_velocity : {0., .25})
	{
		V dir = V::Zero(6);
		dir[1] = dir[3] = edge_velocity;
		dir[5] = -1.;
		const int pairs = contact.project_floor_pairs(x, dir);
		result["projection"].push_back({{"prescribed_edge_velocity", edge_velocity},
										{"pairs", pairs},
										{"projected_point_velocity", dir[5]},
										{"projected_edge_velocity", .5 * (dir[1] + dir[3])},
										{"full_gap_velocity", dir[5] - .5 * (dir[1] + dir[3])},
										{"gap_velocity_after_restoring_prescribed_edge", dir[5] - edge_velocity}});
	}
	// CCD still rejects a segment that crosses the edge in either model.
	V crossing = at_gap(-0.0001);
	contact.line_search_begin(x, crossing);
	result["crossing_segment_collision_free"] = contact.is_step_collision_free(x, crossing);
	const double alpha = contact.max_step_size(x, crossing);
	result["crossing_segment_max_step"] = alpha;
	result["ccd_limited_segment_collision_free"] = contact.is_step_collision_free(x, x + alpha * (crossing - x));
	contact.line_search_end();
	// Independent scalar force-balance reference using the real form, with
	// fixed edge and a downward constant load. Inversion is N/A (no elements).
	const double load = 4e6;
	double lo = 1e-6, hi = .01;
	const double rlo = double(sample(lo)["point_vertical_gradient"]) + load;
	const double rhi = double(sample(hi)["point_vertical_gradient"]) + load;
	result["scalar_equilibrium"] = {{"load", load}, {"lower_residual", rlo}, {"upper_residual", rhi}, {"bracketed", rlo < 0 && rhi > 0}, {"method", "bisection on real point force, frozen snapshot"}};
	if (rlo < 0 && rhi > 0)
	{
		for (int i = 0; i < 80; ++i)
		{
			const double mid = .5 * (lo + hi);
			if (double(sample(mid)["point_vertical_gradient"]) + load < 0)
				lo = mid;
			else
				hi = mid;
		}
		json equilibrium = sample(.5 * (lo + hi));
		equilibrium["free_vertical_residual"] = double(equilibrium["point_vertical_gradient"]) + load;
		equilibrium["relative_force_balance_error"] = std::abs(double(equilibrium["free_vertical_residual"])) / load;
		result["scalar_equilibrium"]["solution"] = equilibrium;
	}
	// Probe whether refresh reapplies the same floor convention at a fixed x.
	x = at_gap(.00005);
	result["refresh_at_fixed_position"]["before"] = sample(.00005);
	contact.update_barrier_stiffness(x, Eigen::MatrixXd());
	contact.set_barrier_stiffness(1.);
	result["refresh_at_fixed_position"]["after"] = sample(.00005);
	result["refresh_at_fixed_position"]["after_nearby_rebuild"] = sample(.000050001);
	result["refresh_at_fixed_position"]["returned_to_same_position"] = sample(.00005);
	std::cout << result.dump(2) << std::endl;
}
