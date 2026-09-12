#include "NonlinearElasticVarForm.hpp"

#include <polyfem/assembler/AssemblerUtils.hpp>
#include <polyfem/assembler/MacroStrain.hpp>

#include <polyfem/mesh/mesh2D/Mesh2D.hpp>
#include <polyfem/mesh/mesh3D/Mesh3D.hpp>
#include <polyfem/mesh/collision_proxy/CollisionProxy.hpp>
#include <polyfem/mesh/GeometryReader.hpp>

#include <polyfem/utils/Logger.hpp>
#include <polyfem/utils/MatrixUtils.hpp>
#include <polyfem/utils/StringUtils.hpp>
#include <polyfem/utils/Timer.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/Jacobian.hpp>

#include <polyfem/io/MatrixIO.hpp>
#include <polyfem/io/OBJWriter.hpp>
#include <polyfem/io/SolverCSVWriter.hpp>

#include <polyfem/solver/ALSolver.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/FrictionForm.hpp>
#include <polyfem/solver/forms/NormalAdhesionForm.hpp>
#include <polyfem/solver/forms/SmoothContactForm.hpp>
#include <polyfem/solver/forms/TangentialAdhesionForm.hpp>
#include <polyfem/solver/forms/lagrangian/PeriodicBoundaryLagrangianForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>

#include <igl/Timer.h>
#include <igl/edges.h>

#include <ipc/ipc.hpp>

#include <polysolve/linear/Solver.hpp>
#include <polysolve/nonlinear/Solver.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <chrono>
#include <polyfem/utils/getRSS.h>
#include <polyfem/solver/forms/InertiaForm.hpp>
#include <polyfem/solver/forms/BodyForm.hpp>
#include <polyfem/solver/forms/PressureForm.hpp>

namespace polyfem::varform
{
	using namespace solver;
	using namespace time_integrator;

	void NonlinearElasticVarForm::configure_coefficient_diagnostics(int step, const std::string &phase)
	{
		auto barrier = std::dynamic_pointer_cast<BarrierContactForm>(solve_data_.contact_form);
		if (!barrier)
			return;
		if (!args["output"].value("physical_diagnostics", false))
		{
			barrier->set_coefficient_observer(nullptr);
			return;
		}
		if (diagnostic_run_id_.empty())
			diagnostic_run_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
		barrier->set_coefficient_observer([this, step, phase](const json &event) {
			json record = event;
			record["schema"] = "polyfem.coefficient-event";
			record["version"] = 1;
			record["run_id"] = diagnostic_run_id_;
			record["step"] = step;
			record["phase"] = phase;
			record["scope"] = "Outer coefficient operations at identical full coordinates; excludes coordinate-only feature switches and time-weight changes";
			const double scale = solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
			record["acceleration_scaling"] = std::isfinite(scale) ? json(scale) : json(nullptr);
			const auto &delta = event.at("objective_change_at_fixed_coordinates");
			if (scale > 0 && std::isfinite(scale) && delta.is_number())
			{
				const double physical = delta.get<double>() / scale;
				record["energy_change_at_fixed_coordinates"] = std::isfinite(physical) ? json(physical) : json(nullptr);
			}
			else
				record["energy_change_at_fixed_coordinates"] = nullptr;
			record["free_contact_force_change_norm"] = nullptr;
			if (scale > 0 && std::isfinite(scale) && solve_data_.nl_problem
				&& event["before"]["gradient_objective"].is_array() && event["after"]["gradient_objective"].is_array())
			{
				const auto before = event["before"]["gradient_objective"].get<std::vector<double>>();
				const auto after = event["after"]["gradient_objective"].get<std::vector<double>>();
				if (before.size() == after.size())
				{
					Eigen::VectorXd change(before.size());
					for (size_t i = 0; i < before.size(); ++i)
						change[i] = (after[i] - before[i]) / scale;
					const double norm = solve_data_.nl_problem->full_to_reduced_grad(change).norm();
					if (std::isfinite(norm))
						record["free_contact_force_change_norm"] = norm;
				}
			}
			std::ofstream file(resolve_output_path("coefficient-events.jsonl"), std::ios::app);
			file << record.dump() << std::endl;
			if (!file)
				throw std::runtime_error("Could not write coefficient event");
		});
	}

	void NonlinearElasticVarForm::write_physical_diagnostics(
		int step, const Eigen::VectorXd &x, const Eigen::VectorXd &start,
		const std::string &outcome, const std::string &phase, const json &termination,
		const json &lagging, const json &observations, double elapsed, const std::string &error)
	{
		if (!args["output"].value("physical_diagnostics", false))
			return;
		const auto missing = [](const std::string &why) { return json{{"value", nullptr}, {"unavailable_reason", why}}; };
		const auto scalar = [&](double v) { return std::isfinite(v) ? json{{"value", v}} : missing("Nonfinite measurement"); };
		const auto vector = [&](const Eigen::VectorXd &v) {
			return v.allFinite() ? json{{"value", std::vector<double>(v.data(), v.data() + v.size())}} : missing("Nonfinite vector");
		};
		// Version 2 (RB-04 remainder): attempt summary and last proposal from the
		// iteration observer, the last internal iterate of a failed attempt,
		// retained candidate counts, and right-endpoint discrete work increments
		// under the declared convention. Version 1 fields keep their meaning.
		json record = {{"schema", "polyfem.physical-diagnostics"}, {"version", 2}, {"run_id", diagnostic_run_id_}, {"step", step}, {"attempt", 1}, {"outcome", outcome}, {"phase", phase}, {"termination", termination}, {"lagging", lagging}, {"error", error}, {"solve_wall_seconds", scalar(elapsed)}, {"units", {{"displacement", "internal length"}, {"residual", "internal force = objective gradient / acceleration_scaling"}, {"energy", "internal energy"}, {"work", "internal energy; right-endpoint discrete estimates, not path integrals"}, {"ordering", "full FEM/system node-major DOFs, obstacles appended"}}}};
		record["time"] = args["time"].is_object() ? scalar(args["time"].value("t0", 0.) + step * args["time"].value("dt", 0.)) : missing("Static solve has no physical time");
		record["accepted_displacement"] = outcome == "accepted" ? vector(x - start) : missing("Attempt did not return an accepted endpoint");
		record["attempt_summary"] = observations.value("summary", json{{"unavailable_reason", "No attempt observation"}});
		record["proposed_displacement"] = observations.contains("proposed_displacement")
											  ? observations["proposed_displacement"]
											  : missing("No line-search proposal was observed in this attempt");
		record["barrier_energy_at_solve_start"] = observations.contains("barrier_energy_at_solve_start")
													  ? observations["barrier_energy_at_solve_start"]
													  : missing("Not measured at solve start");
		record["endpoint"] = vector(x);
		if (outcome == "accepted")
			record["coordinate_state"] = "Returned nonlinear solve endpoint, before time advancement";
		else
		{
			record["coordinate_state"] = "Retained caller coordinates with current attempt form state; the last internal Newton iterate is in last_internal_iterate";
			record["last_internal_iterate"] = observations.contains("last_internal_iterate")
												  ? observations["last_internal_iterate"]
												  : missing("No accepted Newton iterate was observed before the failure");
		}
		record["work_convention"] = "Right-endpoint discrete increments: endpoint force dotted with the accepted displacement of this step, physical units; support force on the system sign; solved-lag friction; parameter-state energy change at the previous physical coordinates (docs/rb-04-work-convention.md). Not path integrals and not a physical balance.";
		record["physical_balance_pass"] = missing("No physical acceptance threshold is authorized for RB-04; the discrete budget terms are reported separately");
		const size_t rss = getCurrentRSS(), peak = getPeakRSS();
		record["current_rss_bytes"] = rss ? json{{"value", rss}} : missing("Platform RSS unavailable");
		record["peak_rss_bytes"] = peak ? json{{"value", peak}} : missing("Platform peak RSS unavailable");
		try
		{
			const double scale = solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
			record["acceleration_scaling"] = scalar(scale);
			if (!(scale > 0) || !std::isfinite(scale) || !x.allFinite())
				throw std::runtime_error("Invalid endpoint or acceleration scaling");
			Eigen::VectorXd residual = Eigen::VectorXd::Zero(x.size());
			bool complete = true;
			record["forms"] = json::array();
			for (const auto &form : forms)
			{
				if (!form->enabled())
					continue;
				// AL prepares feasibility; its penalty/multiplier forces are not physical forces.
				if (dynamic_cast<const AugmentedLagrangianForm *>(form.get()))
					continue;
				json row = {{"name", form->name()}, {"weight", scalar(form->weight())}};
				Eigen::VectorXd g;
				double e;
				if (const auto *barrier = dynamic_cast<const BarrierContactForm *>(form.get()))
				{
					const auto snapshot = barrier->diagnostic_snapshot(x);
					snapshot.first_derivative(x, g);
					e = snapshot.value(x);
					const auto start_snapshot = snapshot.diagnostic_snapshot(start);
					record["barrier_start_energy_with_endpoint_snapshot"] = scalar(start_snapshot.value(start) / scale);
					record["barrier_start_energy_scope"] = "Solve-start coordinates evaluated with returned endpoint coefficient snapshot; not a sum of optimization-event work";
					if (outcome == "accepted" && args["output"].value("physical_diagnostics_contact_path", false))
					{
						try
						{
							record["contact_path"] = snapshot.diagnostic_path(start, x);
						}
						catch (const std::exception &error)
						{
							record["contact_path"] = missing(error.what());
						}
					}
					record["contact"] = snapshot.diagnostic_state();
					const double d2 = snapshot.collision_set().compute_minimum_distance(collision_mesh_, snapshot.compute_displaced_surface(x));
					record["contact"]["min_gap"] = std::isfinite(d2) ? scalar(std::sqrt(d2)) : missing("No active pair within dhat; global minimum not measured");
					record["contact"]["gap_scope"] = "Minimum over endpoint active stencils only";
				}
				else if (form == solve_data_.elastic_form || form == solve_data_.body_form
						 || form == solve_data_.inertia_form || form == solve_data_.friction_form
						 || (form == solve_data_.pressure_form && boundary_.local_pressure_boundary.empty() && boundary_.local_pressure_cavity.empty()))
				{
					form->first_derivative(x, g);
					e = form->value(x);
				}
				else
				{
					row["measurement"] = missing("Form not covered by the observational endpoint contract");
					complete = false;
					record["forms"].push_back(row);
					continue;
				}
				if (g.size() != x.size() || !g.allFinite() || !std::isfinite(e))
					throw std::runtime_error("Invalid form measurement: " + form->name());
				if (form == solve_data_.elastic_form)
					record["elastic_energy"] = scalar(e / scale);
				if (form == solve_data_.contact_form)
					record["barrier_energy"] = scalar(e / scale);
				row["interpretation"] = form == solve_data_.inertia_form    ? "Incremental inertial objective; not kinetic energy"
										: form == solve_data_.friction_form ? "Frozen-lag friction potential; not integrated dissipation"
																			: "Potential energy contribution";
				row["objective"] = scalar(e);
				row["objective_divided_by_acceleration_scaling"] = scalar(e / scale);
				row["gradient_force_units"] = vector(g / scale);
				residual += g / scale;
				record["forms"].push_back(row);
			}
			record["residual_complete"] = complete;
			record["full_residual"] = complete ? vector(residual) : missing("Unsupported active form; component sum is partial");
			record["free_residual_norm"] = complete ? scalar(solve_data_.nl_problem->full_to_reduced_grad(residual).norm()) : missing("Unsupported active form");
			record["full_residual_with_pre_update_friction"] = missing("No complete paired final friction-lag observation");
			record["free_residual_norm_with_pre_update_friction"] = missing("No complete paired final friction-lag observation");
			if (complete && lagging.contains("friction_before_update") && lagging.contains("friction_after_update")
				&& lagging["friction_before_update"].contains("gradient_force_units")
				&& lagging["friction_after_update"].contains("gradient_force_units"))
			{
				const auto before = lagging["friction_before_update"]["gradient_force_units"].get<std::vector<double>>();
				const auto after = lagging["friction_after_update"]["gradient_force_units"].get<std::vector<double>>();
				if (before.size() == size_t(residual.size()) && after.size() == before.size())
				{
					Eigen::VectorXd prior = residual;
					for (size_t i = 0; i < before.size(); ++i)
						prior[i] += before[i] - after[i];
					record["full_residual_with_pre_update_friction"] = vector(prior);
					record["free_residual_norm_with_pre_update_friction"] = scalar(solve_data_.nl_problem->full_to_reduced_grad(prior).norm());
				}
			}
			record["reactions"] = json::array();
			for (const auto &constraint : solve_data_.al_form)
			{
				if (dynamic_cast<const BCLagrangianForm *>(constraint.get()))
				{
					const auto &A = constraint->constraint_matrix();
					const Eigen::VectorXd bc_error = A * x - constraint->constraint_value();
					record["bc_error_inf"] = scalar(bc_error.size() ? bc_error.lpNorm<Eigen::Infinity>() : 0);
					record["reactions"].push_back({{"constraint", constraint->name()},
												   {"sign", "External support force on system = residual on prescribed DOFs; opposite is force on support"},
												   {"full_dof_vector", complete ? vector(A.transpose() * (A * residual)) : missing("Unsupported active form")}});
				}
			}
			if (!record.contains("bc_error_inf"))
				record["bc_error_inf"] = missing("No simple Dirichlet selector constraint");
			if (!record.contains("contact"))
				record["contact"] = missing("No supported active barrier contact form");

			// Right-endpoint discrete work increments of this accepted step
			// (docs/rb-04-work-convention.md). These are the declared
			// bookkeeping terms of the discrete budget, not path integrals.
			if (outcome == "accepted")
			{
				const Eigen::VectorXd dx = x - start;
				if (complete)
				{
					double support_work = 0;
					bool has_support = false;
					for (const auto &constraint : solve_data_.al_form)
						if (dynamic_cast<const BCLagrangianForm *>(constraint.get()))
						{
							const auto &A = constraint->constraint_matrix();
							support_work += (A.transpose() * (A * residual)).dot(dx);
							has_support = true;
						}
					double body_work = 0;
					bool has_body = false;
					for (const auto &row : record["forms"])
						if (row["name"] == "body" && row.contains("gradient_force_units") && row["gradient_force_units"]["value"].is_array())
						{
							const auto g = row["gradient_force_units"]["value"].get<std::vector<double>>();
							body_work = -Eigen::Map<const Eigen::VectorXd>(g.data(), g.size()).dot(dx);
							has_body = true;
						}
					record["support_work_increment"] = has_support ? scalar(support_work) : missing("No simple Dirichlet selector constraint");
					record["support_work_increment_convention"] = "Support force on the system (residual on prescribed DOFs) dotted with the accepted displacement; W_D,right of the work convention";
					record["body_load_work_increment"] = has_body ? scalar(body_work) : json{{"value", 0.}, {"convention", "No body/traction load form; identically zero"}};
					record["external_work_increment"] = has_support ? scalar(support_work + body_work) : missing("No simple Dirichlet selector constraint");
				}
				else
				{
					record["support_work_increment"] = missing("Unsupported active form; residual is partial");
					record["body_load_work_increment"] = missing("Unsupported active form; residual is partial");
					record["external_work_increment"] = missing("Unsupported active form; residual is partial");
				}

				if (!solve_data_.friction_form || !solve_data_.friction_form->enabled())
					record["frictional_dissipation_increment"] = {{"value", 0.}, {"convention", "No friction form; identically zero"}};
				else if (lagging.contains("friction_before_update") && lagging["friction_before_update"].contains("gradient_force_units"))
				{
					const auto g = lagging["friction_before_update"]["gradient_force_units"].get<std::vector<double>>();
					if (g.size() == size_t(dx.size()))
						record["frictional_dissipation_increment"] = scalar(Eigen::Map<const Eigen::VectorXd>(g.data(), g.size()).dot(dx));
					else
						record["frictional_dissipation_increment"] = missing("Pre-update friction gradient has the wrong size");
				}
				else
					record["frictional_dissipation_increment"] = missing("No pre-update friction observation for this attempt");
				record["frictional_dissipation_increment_convention"] = "Solved-lag friction gradient (before the final lag update) dotted with the accepted displacement; C_f,right of the work convention, a resistance-work estimate, not integrated continuum dissipation";

				if (!solve_data_.contact_form || !solve_data_.contact_form->enabled() || !record.contains("barrier_start_energy_with_endpoint_snapshot"))
					record["retuning_energy_change"] = solve_data_.contact_form && solve_data_.contact_form->enabled()
														   ? missing("No endpoint coefficient snapshot evaluation at the start coordinates")
														   : json{{"value", 0.}, {"convention", "No barrier contact form; identically zero"}};
				else if (!trajectory_accounting_.previous_endpoint_barrier_energy_available)
					record["retuning_energy_change"] = missing("Barrier energy of the previous physical endpoint is unavailable in this process");
				else if (!record["barrier_start_energy_with_endpoint_snapshot"].contains("value") || record["barrier_start_energy_with_endpoint_snapshot"]["value"].is_null())
					record["retuning_energy_change"] = missing("Start energy under the endpoint snapshot is unavailable");
				else
					record["retuning_energy_change"] = scalar(record["barrier_start_energy_with_endpoint_snapshot"]["value"].get<double>() - trajectory_accounting_.previous_endpoint_barrier_energy);
				record["retuning_energy_change_convention"] = "B(x_(n-1); theta_n) - B(x_(n-1); theta_(n-1)): barrier energy change from the coefficient state of the previous published endpoint to this endpoint's state, at the previous physical coordinates; P of the work convention. For the first accepted step of this process theta_(n-1) is the state at solve start.";
				record["previous_endpoint_barrier_energy"] = trajectory_accounting_.previous_endpoint_barrier_energy_available
																 ? scalar(trajectory_accounting_.previous_endpoint_barrier_energy)
																 : missing("Unavailable in this process");
			}
			else
			{
				for (const auto *key : {"support_work_increment", "body_load_work_increment", "external_work_increment", "frictional_dissipation_increment", "retuning_energy_change"})
					record[key] = missing("Attempt did not return an accepted endpoint");
			}
			if (solve_data_.time_integrator && !args.value("/time/quasistatic"_json_pointer, true) && mass_.rows() == x.size())
			{
				const Eigen::VectorXd v = solve_data_.time_integrator->compute_velocity(x);
				record["velocity"] = vector(v);
				record["kinetic_energy"] = scalar(.5 * v.dot(mass_ * v));
			}
			else
				record["kinetic_energy"] = missing("Quasistatic/static or incompatible mass dimensions");
			// Fresh assembly values avoid changing the production assembly cache.
			double min_det = std::numeric_limits<double>::infinity();
			size_t count = 0;
			const int dim = mesh_->dimension();
			const auto &bases = space_.basis_list();
			const auto &gbases = space_.geometry_basis_list();
			for (size_t i = 0; i < bases.size(); ++i)
			{
				assembler::ElementAssemblyValues vals;
				vals.compute(i, dim == 3, bases[i], gbases[i]);
				for (int q = 0; q < vals.quadrature.points.rows(); ++q)
				{
					Eigen::MatrixXd F = Eigen::MatrixXd::Identity(dim, dim);
					for (const auto &b : vals.basis_values)
						for (const auto &g : b.global)
							F += g.val * x.segment(g.index * dim, dim) * b.grad_t_m.row(q);
					const double det = F.determinant();
					if (!std::isfinite(det))
						throw std::runtime_error("Nonfinite sampled det(F)");
					min_det = std::min(min_det, det);
					++count;
				}
			}
			record["min_det_F"] = scalar(min_det);
			record["det_F_sampling"] = {{"scope", "Element assembly quadrature points; not a global injectivity certificate"}, {"count", count}};
		}
		catch (const std::exception &e)
		{
			record["measurement_error"] = e.what();
			for (const auto *key : {"full_residual", "free_residual_norm", "bc_error_inf", "reactions", "contact", "kinetic_energy", "min_det_F",
									"support_work_increment", "body_load_work_increment", "external_work_increment", "frictional_dissipation_increment", "retuning_energy_change"})
				if (!record.contains(key))
					record[key] = missing(std::string("Measurement failed: ") + e.what());
		}
		for (const auto *key : {"elastic_energy", "barrier_energy"})
			if (!record.contains(key))
				record[key] = missing("Form absent, disabled or not measured");

		// Cumulative right-endpoint sums over the accepted endpoints of this
		// process; a single unavailable increment makes the sum unavailable.
		if (outcome == "accepted")
		{
			auto &acc = trajectory_accounting_;
			const auto accumulate = [&](const char *key, double &sum, bool &available) {
				const json &increment = record[key];
				if (available && increment.contains("value") && increment["value"].is_number())
					sum += increment["value"].get<double>();
				else
					available = false;
			};
			accumulate("external_work_increment", acc.external_work, acc.external_work_available);
			accumulate("frictional_dissipation_increment", acc.frictional_dissipation, acc.frictional_dissipation_available);
			accumulate("retuning_energy_change", acc.retuning_energy_change, acc.retuning_energy_change_available);
			++acc.accepted_steps;
			const auto cumulative = [&](double sum, bool available) {
				return available ? scalar(sum) : missing("An increment of an earlier or this accepted step in this process was unavailable");
			};
			record["external_work_cumulative"] = cumulative(acc.external_work, acc.external_work_available);
			record["frictional_dissipation_cumulative"] = cumulative(acc.frictional_dissipation, acc.frictional_dissipation_available);
			record["retuning_energy_change_cumulative"] = cumulative(acc.retuning_energy_change, acc.retuning_energy_change_available);
			record["cumulative_scope"] = fmt::format("Sum over the {} accepted endpoint(s) of this process; a restarted process starts at zero", acc.accepted_steps);
			// The published endpoint's coefficient state becomes theta_(n-1) of the next step.
			acc.has_previous_endpoint = true;
			acc.previous_endpoint_barrier_energy_available = record["barrier_energy"].contains("value") && record["barrier_energy"]["value"].is_number();
			acc.previous_endpoint_barrier_energy = acc.previous_endpoint_barrier_energy_available ? record["barrier_energy"]["value"].get<double>() : 0;
		}
		std::ofstream file(resolve_output_path("physical-diagnostics.jsonl"), std::ios::app);
		file << record.dump() << std::endl;
		if (!file)
			logger().warn("Could not write physical diagnostics for step {}", step);
	}

	void NonlinearElasticVarForm::init(const std::string &formulation, const Units &units, const json &args, const std::string &out_path)
	{
		json clean_args = args;
		const bool contact_dhat_was_explicit = clean_args["contact"].value("_dhat_was_explicit", false);
		clean_args["contact"].erase("_dhat_was_explicit");
		ElasticVarForm::init(formulation, units, clean_args, out_path);
		contact_dhat_was_explicit_ = contact_dhat_was_explicit;
	}

	void NonlinearElasticVarForm::reset()
	{
		ElasticVarForm::reset();
		collision_mesh_ = ipc::CollisionMesh();
		periodic_collision_mesh_ = ipc::CollisionMesh();
		periodic_collision_mesh_to_basis_.resize(0);
		obstacle.clear();
		solve_data_ = solver::SolveData();
		diagnostic_run_id_.clear();
		forms.clear();
		elasticity_pressure_assembler = nullptr;
		damping_assembler_ = nullptr;
		damping_prev_assembler_ = nullptr;
		contact_dhat_was_explicit_ = false;
	}

	void NonlinearElasticVarForm::load_mesh(const mesh::Mesh &mesh, const json &args)
	{
		ElasticVarForm::load_mesh(mesh, args);

		logger().info("Loading obstacles...");
		obstacle = mesh::read_obstacle_geometry(
			units,
			args["geometry"],
			utils::json_as_array(args["boundary_conditions"]["obstacle_displacements"]),
			utils::json_as_array(args["boundary_conditions"]["dirichlet_boundary"]),
			root_path, mesh.dimension());
	}

	io::OutputSpace NonlinearElasticVarForm::output_space() const
	{
		auto space = ElasticVarForm::output_space();
		space.collision_mesh = is_contact_enabled() ? &collision_mesh_ : nullptr;
		space.obstacle = &obstacle;
		return space;
	}

	std::vector<io::OutputField> NonlinearElasticVarForm::output_fields(
		const io::OutputSample &sample,
		const Eigen::MatrixXd &solution,
		const io::OutputFieldOptions &options) const
	{
		std::vector<io::OutputField> fields = elastic_output_fields(
			sample, solution, options, &obstacle, solve_data_.time_integrator.get(),
			solve_data_.named_forms(), solve_data_.elastic_form.get(), solve_data_.contact_form.get());
		if (!mesh_ || !problem || solution.size() <= 0)
			return fields;
		if (sample.domain != io::OutputSample::Domain::Contact)
			return fields;

		const int actual_dim = problem->is_scalar() ? 1 : mesh_->dimension();
		const auto &paraview_options = args["output"]["paraview"]["options"];
		const bool explicit_fields = !options.fields.empty();

		const auto has_field = [&](const std::string &name) {
			return std::any_of(fields.begin(), fields.end(), [&](const io::OutputField &field) {
				return field.association == io::OutputField::Association::Point && field.name == name;
			});
		};

		const auto append_collision_dof_field = [&](const std::string &name, const Eigen::MatrixXd &dof_values) {
			if (has_field(name) || dof_values.size() <= 0)
				return;

			Eigen::MatrixXd values = collision_mesh_.map_displacements(utils::unflatten(dof_values, actual_dim));
			if (values.rows() == sample.points.rows())
				fields.push_back({name, values, io::OutputField::Association::Point});
		};

		const auto append_collision_form_force = [&](const std::string &name, const std::shared_ptr<solver::Form> &form) {
			if (!form || !form->enabled() || sample.points.rows() != collision_mesh_.rest_positions().rows())
				return;

			Eigen::VectorXd force;
			form->first_derivative(solution.col(0), force);
			const double acceleration_scaling =
				solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
			force *= -1.0 / acceleration_scaling;
			append_collision_dof_field(name, force);
		};

		if (paraview_options["forces"] && !problem->is_scalar())
		{
			const double s = solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
			for (const auto &[name, form] : solve_data_.named_forms())
			{
				const std::string field_name = name + "_forces";
				if (!options.export_field(field_name))
					continue;

				Eigen::VectorXd force;
				if (form && form->enabled())
				{
					form->first_derivative(solution, force);
					force *= -1.0 / s;
				}
				else
				{
					force.setZero(solution.size());
				}
				append_collision_dof_field(field_name, force);
			}
		}

		if (options.export_field("gradient_of_elastic_potential") && solve_data_.elastic_form)
		{
			Eigen::VectorXd potential_grad;
			solve_data_.elastic_form->first_derivative(solution, potential_grad);
			append_collision_dof_field("gradient_of_elastic_potential", potential_grad);
		}

		if (options.export_field("gradient_of_contact_potential") && solve_data_.contact_form && solve_data_.contact_form->weight() > 0)
		{
			Eigen::VectorXd potential_grad;
			solve_data_.contact_form->first_derivative(solution, potential_grad);
			potential_grad *= -solve_data_.contact_form->barrier_stiffness() / solve_data_.contact_form->weight();
			append_collision_dof_field("gradient_of_contact_potential", potential_grad);
		}

		if (options.export_field("displacement"))
			append_collision_dof_field("displacement", solution);
		if (options.export_field("solution"))
			append_collision_dof_field("solution", solution);

		if ((paraview_options["contact_forces"] || explicit_fields) && options.export_field("contact_forces"))
			append_collision_form_force("contact_forces", solve_data_.contact_form);
		if ((paraview_options["friction_forces"] || explicit_fields) && options.export_field("friction_forces"))
			append_collision_form_force("friction_forces", solve_data_.friction_form);
		if ((paraview_options["normal_adhesion_forces"] || explicit_fields) && options.export_field("normal_adhesion_forces"))
			append_collision_form_force("normal_adhesion_forces", solve_data_.normal_adhesion_form);
		if ((paraview_options["tangential_adhesion_forces"] || explicit_fields) && options.export_field("tangential_adhesion_forces"))
			append_collision_form_force("tangential_adhesion_forces", solve_data_.tangential_adhesion_form);

		if (explicit_fields
			&& options.export_field("adaptive_dhat")
			&& args["contact"]["use_gcp_formulation"]
			&& args["contact"]["use_adaptive_dhat"])
		{
			const auto smooth_contact = std::dynamic_pointer_cast<solver::SmoothContactForm>(solve_data_.contact_form);
			if (smooth_contact)
			{
				const auto &set = smooth_contact->collision_set();
				if (actual_dim == 2)
				{
					Eigen::VectorXd dhats(collision_mesh_.num_edges());
					for (int e = 0; e < dhats.size(); ++e)
						dhats(e) = set.get_edge_dhat(e);
					fields.push_back({"dhat", dhats, io::OutputField::Association::Cell});
				}
				else
				{
					Eigen::VectorXd dhats(collision_mesh_.num_faces());
					for (int f = 0; f < dhats.size(); ++f)
						dhats(f) = set.get_face_dhat(f);
					fields.push_back({"dhat_face", dhats, io::OutputField::Association::Cell});

					Eigen::VectorXd vertex_dhats(collision_mesh_.num_vertices());
					for (int v = 0; v < vertex_dhats.size(); ++v)
						vertex_dhats(v) = set.get_vert_dhat(v);
					fields.push_back({"dhat_vert", vertex_dhats, io::OutputField::Association::Point});
				}
			}
		}

		return fields;
	}

	void NonlinearElasticVarForm::build_basis(mesh::Mesh &mesh, const bool iso_parametric, const json &args)
	{
		ElasticVarForm::build_basis(mesh, iso_parametric, args);

		// Legacy nonlinear/contact code assumes the displacement space includes obstacle vertices.
		// The shared build path only counts FE bases, so extend it here
		// before constructing collision/contact state.
		const int n_fe_bases = space_.n_bases;
		space_.n_bases += obstacle.n_vertices();

		if (is_contact_enabled())
		{
			logger().info("Building collision mesh...");
			build_collision_mesh(mesh, args);
			preprocess_contact_parameters();

			if (args["contact"]["periodic"])
				build_periodic_collision_mesh();
		}

		logger().info("Done!");

		for (int i = n_fe_bases; i < space_.n_bases; ++i)
		{
			for (int d = 0; d < mesh.dimension(); ++d)
				boundary_.boundary_nodes.push_back(i * mesh.dimension() + d);
		}

		boundary_.normalize_boundary_nodes();
	}

	void NonlinearElasticVarForm::preprocess_contact_parameters()
	{
		if (!is_contact_enabled())
			return;

		double min_boundary_edge_length = std::numeric_limits<double>::max();
		for (const auto &edge : collision_mesh_.edges().rowwise())
		{
			const VectorNd v0 = collision_mesh_.rest_positions().row(edge(0));
			const VectorNd v1 = collision_mesh_.rest_positions().row(edge(1));
			min_boundary_edge_length = std::min(min_boundary_edge_length, (v1 - v0).norm());
		}

		double dhat = Units::convert(args["contact"]["dhat"], units.length());
		args["contact"]["epsv"] = Units::convert(args["contact"]["epsv"], units.velocity());

		if (!contact_dhat_was_explicit_
			&& std::isfinite(min_boundary_edge_length)
			&& dhat > min_boundary_edge_length)
		{
			dhat = args["contact"]["dhat_percentage"].get<double>() * min_boundary_edge_length;
			logger().info("dhat set to {}", dhat);
		}
		else if (std::isfinite(min_boundary_edge_length) && dhat > min_boundary_edge_length)
		{
			logger().warn("dhat larger than min boundary edge, {} > {}", dhat, min_boundary_edge_length);
		}

		args["contact"]["dhat"] = dhat;
	}

	void NonlinearElasticVarForm::build_rhs_assembler()
	{
		json rhs_solver_params = args["solver"]["linear"];
		if (!rhs_solver_params.contains("Pardiso"))
			rhs_solver_params["Pardiso"] = {};
		rhs_solver_params["Pardiso"]["mtype"] = -2;

		const int size = problem->is_scalar() ? 1 : mesh_->dimension();

		solve_data_.rhs_assembler = std::make_shared<assembler::RhsAssembler>(
			*primary_assembler_, *mesh_, &obstacle,
			boundary_.dirichlet_nodes, boundary_.neumann_nodes,
			boundary_.dirichlet_nodes_position, boundary_.neumann_nodes_position,
			space_.n_bases, size, space_.basis_list(), space_.geometry_basis_list(), mass_ass_vals_cache_, *problem,
			args["space"]["advanced"]["bc_method"],
			rhs_solver_params,
			/*fe_space_id=*/-1);
		rhs_assembler_ = solve_data_.rhs_assembler;
	}

	void NonlinearElasticVarForm::build_collision_mesh(
		const mesh::Mesh &mesh,
		const json &args)
	{
		build_collision_mesh(
			mesh, space_.n_bases, space_.basis_list(), space_.geometry_basis_list(), boundary_.total_local_boundary, obstacle,
			args, [this](const std::string &p) { return utils::resolve_path(p, root_path, false); },
			space_.space_in_node_to_node, collision_mesh_);
	}

	void NonlinearElasticVarForm::build_collision_mesh(
		const mesh::Mesh &mesh,
		const int n_bases,
		const std::vector<basis::ElementBases> &bases,
		const std::vector<basis::ElementBases> &geom_bases,
		const std::vector<mesh::LocalBoundary> &total_local_boundary,
		const mesh::Obstacle &obstacle,
		const json &args,
		const std::function<std::string(const std::string &)> &resolve_input_path,
		const Eigen::VectorXi &in_node_to_node,
		ipc::CollisionMesh &collision_mesh_)
	{
		Eigen::MatrixXd collision_vertices;
		Eigen::VectorXi collision_codim_vids;
		Eigen::MatrixXi collision_edges, collision_triangles;
		std::vector<Eigen::Triplet<double>> displacement_map_entries;

		// RB-22: which extraction built the FE surface and what it skipped.
		// A contact-enabled scene must get a complete surface or a named
		// error; a silently partial or empty one corrupts the Hessian sizes
		// downstream (empty displacement map + padded vertex rows = an
		// identity map larger than the DOF vector).
		io::OutGeometryData::BoundaryExtractionReport extraction_report;
		std::string extraction_name = "preconstructed collision proxy";

		const auto extract_default_collision_mesh = [&]() {
			if (args.at("/space/basis_type"_json_pointer) == "Spline")
			{
				extraction_name = "max_order lattice sampling (spline basis)";
				io::OutGeometryData::extract_boundary_mesh_sampled(
					mesh, n_bases - obstacle.n_vertices(), bases, total_local_boundary,
					collision_vertices, collision_edges, collision_triangles, displacement_map_entries,
					/*sampling_order=*/0, &extraction_report);
			}
			else if (io::OutGeometryData::has_high_order_hex_boundary(mesh, bases, total_local_boundary))
			{
				// RB-22 default (user decision 2026-09-12): Q2+/serendipity
				// hexahedral boundaries get the DOF-resolution proxy -- every
				// proxy vertex is a node with an exact displacement-map row,
				// conforming by construction, the same oriented surface as the
				// max_order lattice on Lagrange Q2 -- instead of the Q1-only
				// default extraction that skipped their faces. Q1 hexahedra
				// keep the centroid split; simplicial meshes are untouched.
				extraction_name = "the DOF-resolution proxy (high-order hexahedra)";
				io::OutGeometryData::extract_boundary_mesh_nodal(
					mesh, n_bases - obstacle.n_vertices(), bases, total_local_boundary,
					collision_vertices, collision_edges, collision_triangles, displacement_map_entries,
					&extraction_report);
			}
			else
			{
				extraction_name = "the default boundary extraction";
				io::OutGeometryData::extract_boundary_mesh(
					mesh, n_bases - obstacle.n_vertices(), bases, total_local_boundary,
					collision_vertices, collision_edges, collision_triangles, displacement_map_entries,
					&extraction_report);
			}
		};

		if (args.contains("/contact/collision_mesh"_json_pointer)
			&& args.at("/contact/collision_mesh/enabled"_json_pointer).get<bool>())
		{
			const json collision_mesh_args = args.at("/contact/collision_mesh"_json_pointer);
			if (collision_mesh_args.contains("linear_map"))
			{
				assert(displacement_map_entries.empty());
				assert(collision_mesh_args.contains("mesh"));
				const std::string root_path = utils::json_value<std::string>(args, "root_path", "");
				// TODO: handle transformation per geometry
				const json transformation = utils::json_as_array(args["geometry"])[0]["transformation"];
				mesh::load_collision_proxy(
					utils::resolve_path(collision_mesh_args["mesh"], root_path),
					utils::resolve_path(collision_mesh_args["linear_map"], root_path),
					in_node_to_node, transformation, collision_vertices, collision_codim_vids,
					collision_edges, collision_triangles, displacement_map_entries);
			}
			else if (collision_mesh_args.contains("tessellation_type")
					 && collision_mesh_args["tessellation_type"] == "max_order")
			{
				extraction_name = "max_order lattice sampling";
				io::OutGeometryData::extract_boundary_mesh_sampled(
					mesh, n_bases - obstacle.n_vertices(), bases, total_local_boundary,
					collision_vertices, collision_edges, collision_triangles, displacement_map_entries,
					utils::json_value<int>(collision_mesh_args, "sampling_order", 0), &extraction_report);
			}
			else if (collision_mesh_args.contains("tessellation_type")
					 && collision_mesh_args["tessellation_type"] == "dof")
			{
				extraction_name = "the DOF-resolution proxy";
				io::OutGeometryData::extract_boundary_mesh_nodal(
					mesh, n_bases - obstacle.n_vertices(), bases, total_local_boundary,
					collision_vertices, collision_edges, collision_triangles, displacement_map_entries,
					&extraction_report);
			}
			else if (collision_mesh_args.contains("max_edge_length"))
			{
				logger().debug(
					"Building collision proxy with max edge length={} ...",
					collision_mesh_args["max_edge_length"].get<double>());
				igl::Timer timer;
				timer.start();
				build_collision_proxy(
					bases, geom_bases, total_local_boundary, n_bases, mesh.dimension(),
					collision_mesh_args["max_edge_length"], collision_vertices,
					collision_triangles, displacement_map_entries,
					collision_mesh_args["tessellation_type"]);
				if (collision_triangles.size())
					igl::edges(collision_triangles, collision_edges);
				timer.stop();
				logger().debug(fmt::format(
					std::locale("en_US.UTF-8"),
					"Done (took {:g}s, {:L} vertices, {:L} triangles)",
					timer.getElapsedTime(),
					collision_vertices.rows(), collision_triangles.rows()));
			}
			else
			{
				extract_default_collision_mesh();
			}
		}
		else
		{
			extract_default_collision_mesh();
		}

		std::vector<bool> is_orientable_vertex(collision_vertices.rows(), true);

		// n_bases already contains the obstacle vertices
		const int num_fe_nodes = n_bases - obstacle.n_vertices();
		const int num_fe_collision_vertices = collision_vertices.rows();
		assert(collision_edges.size() == 0 || collision_edges.maxCoeff() < num_fe_collision_vertices);
		assert(collision_triangles.size() == 0 || collision_triangles.maxCoeff() < num_fe_collision_vertices);

		// RB-22 contract: refuse an incomplete or empty FE collision surface
		// instead of building a partial collision mesh.
		if (!extraction_report.complete())
		{
			log_and_throw_error(
				"Contact is enabled but {} produced an incomplete collision surface: {}. "
				"Refusing to build a partial collision mesh; choose a boundary tessellation that supports these elements "
				"(contact/collision_mesh: {{\"enabled\": true, \"tessellation_type\": \"dof\" | \"max_order\"}}), "
				"lower the element order, use a supported element type, or disable contact.",
				extraction_name, extraction_report.describe());
		}
		const int n_fe_primitives = mesh.is_volume() ? int(collision_triangles.rows()) : int(collision_edges.rows());
		if (extraction_report.n_boundary_faces > 0 && n_fe_primitives == 0)
		{
			log_and_throw_error(
				"Contact is enabled but {} produced no collision {} from {} boundary {} of the FE mesh; refusing to build an empty collision surface.",
				extraction_name, mesh.is_volume() ? "faces" : "edges", extraction_report.n_boundary_faces, mesh.is_volume() ? "faces" : "edges");
		}
		if (displacement_map_entries.empty() && num_fe_collision_vertices != num_fe_nodes)
		{
			log_and_throw_error(
				"Collision mesh construction: the boundary extraction returned {} vertex rows with an identity displacement map, but the FE space has {} nodes; the collision mesh would not match the DOF vector.",
				num_fe_collision_vertices, num_fe_nodes);
		}
		if (mesh.is_volume())
		{
			// A degenerate (zero-area) FE collision face has no normal and
			// poisons every distance it enters. It means coincident or
			// collinear boundary node positions: known for Q3+ hexahedra,
			// whose edge/face node positions are not the images of their
			// reference nodes (RB-22 record) -- refuse instead of failing
			// later with "initial solution has intersections".
			int degenerate = 0, first = -1;
			for (int i = 0; i < collision_triangles.rows(); ++i)
			{
				const Eigen::RowVector3d a = collision_vertices.row(collision_triangles(i, 0));
				const Eigen::RowVector3d ab = collision_vertices.row(collision_triangles(i, 1)) - a;
				const Eigen::RowVector3d ac = collision_vertices.row(collision_triangles(i, 2)) - a;
				const double scale = std::max({ab.squaredNorm(), ac.squaredNorm(), (ac - ab).squaredNorm()});
				if (!(ab.cross(ac).norm() > 1e-12 * scale))
				{
					if (first < 0)
						first = i;
					++degenerate;
				}
			}
			if (degenerate > 0)
			{
				const auto at = [&](const int v) {
					return fmt::format("[{:g}, {:g}, {:g}]", collision_vertices(v, 0), collision_vertices(v, 1), collision_vertices(v, 2));
				};
				log_and_throw_error(
					"Contact is enabled but {} produced {} degenerate (zero-area) collision faces out of {} (first: face {} on vertices {}, {}, {} at {}, {}, {}); the boundary node positions are coincident or collinear. Known cause: Q3+ hexahedral bases store edge/face node positions that are not the geometric images of their reference nodes (see docs/rb-22-validation.md); use Q2 hexahedra or tetrahedra.",
					extraction_name, degenerate, collision_triangles.rows(), first,
					collision_triangles(first, 0), collision_triangles(first, 1), collision_triangles(first, 2),
					at(collision_triangles(first, 0)), at(collision_triangles(first, 1)), at(collision_triangles(first, 2)));
			}
		}
		{
			// RB-22 diagnostic: proxy size and how many FE surface vertices are
			// exact node selectors (single unit entry) versus interpolated rows.
			std::vector<int> counts(num_fe_collision_vertices, 0);
			std::vector<double> single(num_fe_collision_vertices, 0.);
			for (const auto &t : displacement_map_entries)
				if (t.row() < num_fe_collision_vertices && t.value() != 0.)
				{
					++counts[t.row()];
					single[t.row()] = t.value();
				}
			int selector_rows = 0, interpolated_rows = 0;
			std::vector<bool> on_surface(num_fe_collision_vertices, false);
			for (int i = 0; i < collision_triangles.size(); ++i)
				on_surface[collision_triangles(i)] = true;
			for (int i = 0; i < collision_edges.size(); ++i)
				on_surface[collision_edges(i)] = true;
			for (int i = 0; i < num_fe_collision_vertices; ++i)
			{
				if (!on_surface[i])
					continue;
				if (displacement_map_entries.empty() || (counts[i] == 1 && single[i] == 1.))
					++selector_rows;
				else
					++interpolated_rows;
			}
			logger().debug(
				"Collision mesh from {}: {} FE surface vertices ({} exact selector rows, {} interpolated rows), {} faces, {} edges",
				extraction_name, selector_rows + interpolated_rows, selector_rows, interpolated_rows,
				collision_triangles.rows(), collision_edges.rows());
		}

		// Append the obstacles to the collision mesh
		if (obstacle.n_vertices() > 0)
		{
			utils::append_rows(collision_vertices, obstacle.v());
			utils::append_rows(collision_codim_vids, obstacle.codim_v().array() + num_fe_collision_vertices);
			utils::append_rows(collision_edges, obstacle.e().array() + num_fe_collision_vertices);
			utils::append_rows(collision_triangles, obstacle.f().array() + num_fe_collision_vertices);

			for (int i = 0; i < obstacle.n_vertices(); i++)
			{
				is_orientable_vertex.push_back(false);
			}

			if (!displacement_map_entries.empty())
			{
				displacement_map_entries.reserve(displacement_map_entries.size() + obstacle.n_vertices());
				for (int i = 0; i < obstacle.n_vertices(); i++)
				{
					displacement_map_entries.emplace_back(num_fe_collision_vertices + i, num_fe_nodes + i, 1.0);
				}
			}
		}

		std::vector<bool> is_on_surface = ipc::CollisionMesh::construct_is_on_surface(
			collision_vertices.rows(), collision_edges);
		for (const int vid : collision_codim_vids)
		{
			is_on_surface[vid] = true;
		}

		Eigen::SparseMatrix<double> displacement_map;
		if (!displacement_map_entries.empty())
		{
			displacement_map.resize(collision_vertices.rows(), n_bases);
			displacement_map.setFromTriplets(displacement_map_entries.begin(), displacement_map_entries.end());
		}

		collision_mesh_ = ipc::CollisionMesh(
			is_on_surface, is_orientable_vertex, collision_vertices, collision_edges, collision_triangles,
			displacement_map);

		collision_mesh_.can_collide = [&collision_mesh_, num_fe_collision_vertices](size_t vi, size_t vj) {
			// obstacles do not collide with other obstacles
			return collision_mesh_.to_full_vertex_id(vi) < num_fe_collision_vertices
				   || collision_mesh_.to_full_vertex_id(vj) < num_fe_collision_vertices;
		};

		collision_mesh_.init_area_jacobians();
	}

	void NonlinearElasticVarForm::build_periodic_collision_mesh()
	{
		assert(!mesh_->is_volume());
		const int dim = mesh_->dimension();
		const int n_tiles = 2;

		if (mesh_->dimension() != 2)
			log_and_throw_error("Periodic collision mesh is only implemented in 2D!");
		if (obstacle.n_vertices() != 0)
			log_and_throw_error("Periodic contact does not support obstacles.");

		const int n_bases = space_.n_bases;
		const json &conditions = args["boundary_conditions"]["periodic"];
		Eigen::VectorXi periodic_dof_mask = Eigen::VectorXi::Zero(n_bases);
		Eigen::MatrixXd periodic_tile_offsets(dim, conditions.size());
		for (int i = 0; i < int(conditions.size()); ++i)
		{
			const json &condition = conditions[i];
			const std::array<int, 2> boundary_ids = {{condition["boundary_ids"][0].get<int>(),
													  condition["boundary_ids"][1].get<int>()}};
			const auto mapping = solver::PeriodicBoundaryLagrangianForm::build_mapping(
				n_bases * dim, dim, *mesh_, space_.basis_list(), boundary_.total_local_boundary,
				boundary_ids, condition.value("tolerance", 1e-5));
			periodic_tile_offsets.col(i) = mapping.translation.transpose();
			for (const int dof : mapping.boundary_dofs)
				periodic_dof_mask(dof) = 1;
		}

		Eigen::MatrixXd V(n_bases, dim);
		for (const auto &bs : space_.basis_list())
			for (const auto &b : bs.bases)
				for (const auto &g : b.global())
					V.row(g.index) = g.node;

		Eigen::MatrixXi E = collision_mesh_.edges();
		for (int i = 0; i < E.size(); i++)
		{
			E(i) = collision_mesh_.to_full_vertex_id(E(i));
			if (E(i) < 0 || E(i) >= n_bases)
				log_and_throw_error("Periodic contact requires collision vertices to map to FE basis nodes.");
		}

		Eigen::MatrixXd bbox(V.cols(), 2);
		bbox.col(0) = V.colwise().minCoeff();
		bbox.col(1) = V.colwise().maxCoeff();

		// remove boundary edges on periodic BC, buggy
		{
			std::vector<int> ind;
			for (int i = 0; i < E.rows(); i++)
			{
				if (!periodic_dof_mask(E(i, 0)) || !periodic_dof_mask(E(i, 1)))
					ind.push_back(i);
			}

			E = E(ind, Eigen::all).eval();
		}

		Eigen::MatrixXd Vtmp, Vnew;
		Eigen::MatrixXi Etmp, Enew;
		Vtmp.setZero(V.rows() * n_tiles * n_tiles, V.cols());
		Etmp.setZero(E.rows() * n_tiles * n_tiles, E.cols());

		if (periodic_tile_offsets.rows() != dim || periodic_tile_offsets.cols() != dim
			|| Eigen::FullPivLU<Eigen::MatrixXd>(periodic_tile_offsets).rank() != dim)
			log_and_throw_error("Periodic contact requires {} linearly independent periodic boundary pairs", dim);
		const Eigen::MatrixXd &tile_offset = periodic_tile_offsets;

		for (int i = 0, idx = 0; i < n_tiles; i++)
		{
			for (int j = 0; j < n_tiles; j++)
			{
				Eigen::Vector2d block_id;
				block_id << i, j;

				Vtmp.middleRows(idx * V.rows(), V.rows()) = V;
				for (int vid = 0; vid < V.rows(); vid++)
					Vtmp.block(idx * V.rows() + vid, 0, 1, 2) += (tile_offset * block_id).transpose();

				Etmp.middleRows(idx * E.rows(), E.rows()) = E.array() + idx * V.rows();
				idx += 1;
			}
		}

		// clean duplicated vertices
		Eigen::VectorXi indices;
		{
			std::vector<int> tmp;
			for (int i = 0; i < V.rows(); i++)
			{
				if (periodic_dof_mask(i))
					tmp.push_back(i);
			}

			indices.resize(tmp.size() * n_tiles * n_tiles);
			for (int i = 0; i < n_tiles * n_tiles; i++)
			{
				indices.segment(i * tmp.size(), tmp.size()) = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(tmp.data(), tmp.size());
				indices.segment(i * tmp.size(), tmp.size()).array() += i * V.rows();
			}
		}

		Eigen::VectorXi potentially_duplicate_mask(Vtmp.rows());
		potentially_duplicate_mask.setZero();
		potentially_duplicate_mask(indices).array() = 1;

		Eigen::MatrixXd candidates = Vtmp(indices, Eigen::all);

		Eigen::VectorXi SVI;
		std::vector<int> SVJ;
		SVI.setConstant(Vtmp.rows(), -1);
		int id = 0;
		double relative_tolerance = 1e-5;
		for (const json &condition : conditions)
			relative_tolerance = std::max(relative_tolerance, condition.value("tolerance", 1e-5));
		const double eps = (bbox.col(1) - bbox.col(0)).maxCoeff() * relative_tolerance;
		for (int i = 0; i < Vtmp.rows(); i++)
		{
			if (SVI[i] < 0)
			{
				SVI[i] = id;
				SVJ.push_back(i);
				if (potentially_duplicate_mask(i))
				{
					Eigen::VectorXd diffs = (candidates.rowwise() - Vtmp.row(i)).rowwise().norm();
					for (int j = 0; j < diffs.size(); j++)
						if (diffs(j) < eps)
							SVI[indices[j]] = id;
				}
				id++;
			}
		}

		Vnew = Vtmp(SVJ, Eigen::all);

		Enew.resizeLike(Etmp);
		for (int d = 0; d < Etmp.cols(); d++)
			Enew.col(d) = SVI(Etmp.col(d));

		std::vector<bool> is_on_surface = ipc::CollisionMesh::construct_is_on_surface(Vnew.rows(), Enew);

		Eigen::MatrixXi boundary_triangles;
		Eigen::SparseMatrix<double> displacement_map;
		periodic_collision_mesh_ = ipc::CollisionMesh(is_on_surface,
													  std::vector<bool>(Vnew.rows(), false),
													  Vnew,
													  Enew,
													  boundary_triangles,
													  displacement_map);

		periodic_collision_mesh_.init_area_jacobians();

		periodic_collision_mesh_to_basis_.setConstant(Vnew.rows(), -1);
		for (int i = 0; i < V.rows(); i++)
			for (int j = 0; j < n_tiles * n_tiles; j++)
				periodic_collision_mesh_to_basis_(SVI[j * V.rows() + i]) = i;

		if (periodic_collision_mesh_to_basis_.maxCoeff() + 1 != V.rows())
			log_and_throw_error("Failed to tile mesh!");
	}

	std::shared_ptr<assembler::PressureAssembler> NonlinearElasticVarForm::build_pressure_assembler() const
	{
		const int size = problem->is_scalar() ? 1 : mesh_->dimension();

		return std::make_shared<assembler::PressureAssembler>(
			*primary_assembler_, *mesh_, obstacle,
			boundary_.local_pressure_boundary,
			boundary_.local_pressure_cavity,
			boundary_.boundary_nodes,
			elastic_primitive_to_node(), elastic_node_to_primitive(),
			space_.n_bases, size, space_.basis_list(), space_.geometry_basis_list(), *problem);
	}

	void NonlinearElasticStaticVarForm::solve_problem(
		Eigen::MatrixXd &sol,
		const InitialConditionOverride *initial_condition_override,
		const ForwardStepCallback &post_step)
	{
		assert((!initial_condition_override || (initial_condition_override->velocity.size() == 0 && initial_condition_override->acceleration.size() == 0))
			   && "Static elasticity does not accept initial velocity or acceleration overrides");

		stats.spectrum.setZero();

		igl::Timer timer;
		timer.start();
		logger().info("Solving {}", primary_assembler_->name());

		{
			POLYFEM_SCOPED_TIMER("Setup RHS");

			// FIXME
			//  read_initial_x_from_file(
			//  resolve_input_path(args["input"]["data"]["state"]), "u",
			//  args["input"]["data"]["reorder"], in_node_to_node,
			//  mesh->dimension(), solution);

			if (initial_condition_override && initial_condition_override->solution.size() != 0)
				initial_solution(sol, initial_condition_override);
			else if (sol.size() <= 0)
				initial_solution(sol, initial_condition_override);

			if (initial_condition_override && initial_condition_override->solution.size() != 0)
				assert(sol.cols() == 1 && "Static initial solution override must have exactly one column");
			else if (sol.cols() != 1)
				log_and_throw_error("Static elasticity requires exactly one initial solution column.");
		}
		init_solve(sol, 1.0, initial_condition_override);

		solve_tensor_nonlinear(0, sol, true);
		if (post_step)
			post_step(0, sol);

		const std::string state_path = resolve_output_path(args["output"]["data"]["state"]);
		if (!state_path.empty())
			io::write_matrix(state_path, "u", sol);

		timer.stop();
		timings.solving_time = timer.getElapsedTime();
		logger().info(" took {}s", timings.solving_time);
	}

	void NonlinearElasticTransientVarForm::solve_problem(
		Eigen::MatrixXd &sol,
		const InitialConditionOverride *initial_condition_override,
		const ForwardStepCallback &post_step)
	{
		const bool save_stats = args["output"]["stats"];
		stats.spectrum.setZero();

		igl::Timer timer;
		timer.start();
		logger().info("Solving {}", primary_assembler_->name());

		{
			POLYFEM_SCOPED_TIMER("Setup RHS");

			// FIXME
			//  read_initial_x_from_file(
			//  resolve_input_path(args["input"]["data"]["state"]), "u",
			//  args["input"]["data"]["reorder"], in_node_to_node,
			//  mesh->dimension(), solution);

			if (initial_condition_override && initial_condition_override->solution.size() != 0)
				initial_solution(sol, initial_condition_override);
			else if (sol.size() <= 0)
				initial_solution(sol, initial_condition_override);

			if (sol.cols() > 1) // ignore previous solutions
				sol.conservativeResize(Eigen::NoChange, 1);
		}
		init_solve(sol, t0 + dt, initial_condition_override);
		configure_coefficient_diagnostics(0, "initial_state_after_setup");
		if (post_step)
			post_step(0, sol);

		// Write the total energy to a CSV file
		int save_i = 0;

		std::unique_ptr<io::EnergyCSVWriter> energy_csv = nullptr;
		std::unique_ptr<io::RuntimeStatsCSVWriter> stats_csv = nullptr;

		if (save_stats)
		{
			logger().debug("Saving nl stats to {} and {}", resolve_output_path("energy.csv"), resolve_output_path("stats.csv"));
			energy_csv = std::make_unique<io::EnergyCSVWriter>(resolve_output_path("energy.csv"), solve_data_);
			const io::OutputSpace space = output_space();
			stats_csv = std::make_unique<io::RuntimeStatsCSVWriter>(
				resolve_output_path("stats.csv"),
				space_.n_bases,
				space.mesh ? space.mesh->n_elements() : 0,
				t0, dt);
		}

		// Save the initial solution
		if (energy_csv)
			energy_csv->write(save_i, sol);
		save_timestep(t0, 0, t0, dt, sol);

		save_i++;

		for (int t = 1; t <= time_steps; ++t)
		{
			double forward_solve_time = 0, remeshing_time = 0, global_relaxation_time = 0;

			{
				POLYFEM_SCOPED_TIMER(forward_solve_time);
				solve_tensor_nonlinear(t, sol, true);
			}
			if (post_step)
				post_step(t, sol);

			// Always save the solution for consistency
			if (energy_csv)
				energy_csv->write(save_i, sol);
			save_timestep(t0 + dt * t, t, t0, dt, sol);
			save_i++;

			{
				POLYFEM_SCOPED_TIMER("Update quantities");
				configure_coefficient_diagnostics(t, "between_steps_after_endpoint");

				if (solve_data_.time_integrator)
					solve_data_.time_integrator->update_quantities(sol);

				solve_data_.nl_problem->update_quantities(t0 + (t + 1) * dt, sol);

				solve_data_.update_dt();
				solve_data_.update_barrier_stiffness(sol);
			}

			logger().info("{}/{}  t={}", t, time_steps, t0 + dt * t);
			notify_time_step(t, time_steps, t0, dt);

			save_elastic_step_state(t0, dt, t, solve_data_.time_integrator.get());
			if (stats_csv)
				stats_csv->write(t, forward_solve_time, remeshing_time, global_relaxation_time);
		}

		timer.stop();
		timings.solving_time = timer.getElapsedTime();
		logger().info(" took {}s", timings.solving_time);
	}

	void NonlinearElasticVarForm::init_forms(const json &args, const int dim, Eigen::MatrixXd &sol, const double t)
	{
		damping_assembler_ = std::make_shared<assembler::ViscousDamping>();
		set_materials(*damping_assembler_, mesh_->dimension());

		elasticity_pressure_assembler = build_pressure_assembler();

		// for backward solve
		damping_prev_assembler_ = std::make_shared<assembler::ViscousDampingPrev>();
		set_materials(*damping_prev_assembler_, mesh_->dimension());

		const ElementInversionCheck check_inversion = args["solver"]["advanced"]["check_inversion"];

		// NOTE: some stuff are legacy and hardcoded to be off
		forms = solve_data_.init_forms(
			// General
			units,
			dim, t, space_.space_in_node_to_node,
			// Elastic form
			space_.n_bases, *space_.bases, space_.geometry_basis_list(), *primary_assembler_, ass_vals_cache_, mass_ass_vals_cache_, args["solver"]["advanced"]["jacobian_threshold"], check_inversion,
			args["solver"]["advanced"]["conservative_max_iter"],
			// Body form
			0, boundary_.boundary_nodes, boundary_.local_boundary,
			boundary_.local_neumann_boundary,
			elastic_boundary_samples(), rhs_, sol, mass_assembler_->density(),
			// Pressure form
			boundary_.local_pressure_boundary, boundary_.local_pressure_cavity, elasticity_pressure_assembler,
			// Inertia form
			args.value("/time/quasistatic"_json_pointer, true), mass_,
			damping_assembler_->is_valid() ? damping_assembler_ : nullptr,
			// Lagged regularization form
			args["solver"]["advanced"]["lagged_regularization_weight"],
			args["solver"]["advanced"]["lagged_regularization_iterations"],
			// Augmented lagrangian form
			obstacle.ndof(), args["constraints"]["hard"], args["constraints"]["soft"], args["constraints"]["zero_mean"],
			// Contact form
			args["contact"]["enabled"], collision_mesh_, args["contact"]["dhat"],
			avg_mass_, args["contact"]["use_convergent_formulation"] ? bool(args["contact"]["use_area_weighting"]) : false,
			args["contact"]["use_convergent_formulation"] ? bool(args["contact"]["use_improved_max_operator"]) : false,
			args["contact"]["use_convergent_formulation"] ? bool(args["contact"]["use_physical_barrier"]) : false,
			args["solver"]["contact"]["barrier_stiffness"],
			args["solver"]["contact"]["initial_barrier_stiffness"],
			args["solver"]["contact"]["semi_implicit"],
			args["solver"]["contact"]["CCD"]["broad_phase"],
			args["solver"]["contact"]["CCD"]["tolerance"],
			args["solver"]["contact"]["CCD"]["max_iterations"],
			false,
			// Smooth Contact Form
			args["contact"]["use_gcp_formulation"],
			args["contact"]["alpha_t"],
			args["contact"]["alpha_n"],
			args["contact"]["use_adaptive_dhat"],
			args["contact"]["min_distance_ratio"],
			// Normal Adhesion Form
			args["contact"]["adhesion"]["adhesion_enabled"],
			args["contact"]["adhesion"]["dhat_p"],
			args["contact"]["adhesion"]["dhat_a"],
			args["contact"]["adhesion"]["adhesion_strength"],
			// Tangential Adhesion Form
			args["contact"]["adhesion"]["tangential_adhesion_coefficient"],
			args["contact"]["adhesion"]["epsa"],
			args["solver"]["contact"]["tangential_adhesion_iterations"],
			// Homogenization
			assembler::MacroStrainValue(),
			// Periodic contact
			false, Eigen::VectorXi(),
			// Friction form
			args["contact"]["friction_coefficient"],
			args["contact"]["epsv"],
			args["solver"]["contact"]["friction_iterations"],
			// Rayleigh damping form
			args["solver"]["rayleigh_damping"],

			// BC AL lumping
			args["solver"]["augmented_lagrangian"]["lumping"],

			// Boundary-ID periodic constraints
			mesh_.get(), &boundary_.total_local_boundary,
			args["boundary_conditions"]["periodic"], /*fe_space_id=*/-1);

		for (const auto &form : forms)
			form->set_output_dir(output_path);

		if (solve_data_.contact_form != nullptr)
		{
			solve_data_.contact_form->save_ccd_debug_meshes = args["output"]["advanced"]["save_ccd_debug_meshes"];
			solve_data_.contact_form->apply_resource_limits(solver::resource_limits_from_args(args["solver"]["contact"]["CCD"]));
		}
	}

	void NonlinearElasticVarForm::init_solve(
		Eigen::MatrixXd &sol,
		const double t,
		const InitialConditionOverride *initial_condition_override)
	{
		init_solve_data(sol, t, "", initial_condition_override);

		double characteristic_length = 0;
		if (args["solver"]["advanced"]["characteristic_length"] > 0)
		{
			characteristic_length = args["solver"]["advanced"]["characteristic_length"];
		}
		else
		{
			RowVectorNd min, max;
			mesh_->bounding_box(min, max);
			characteristic_length = (max - min).norm();
		}

		double characteristic_force_density = 0;
		if (args["solver"]["advanced"]["characteristic_force_density"] <= 0)
		{
			logger().warn("No user-specified force density was provided, defaulting to 10000.");
			characteristic_force_density = 10000;
		}
		else
		{
			characteristic_force_density = args["solver"]["advanced"]["characteristic_force_density"];
		}

		const int ndof = space_.n_bases * mesh_->dimension();
		solve_data_.nl_problem = std::make_shared<solver::NLProblem>(
			ndof, t, forms, solve_data_.al_form,
			polysolve::linear::Solver::create(args["solver"]["linear"], logger()),
			characteristic_length, characteristic_force_density, pure_mass_, mesh_->dimension());
		solve_data_.nl_problem->init(sol);
		solve_data_.nl_problem->update_quantities(t, sol);

		stats.solver_info = json::array();
	}

	void NonlinearElasticVarForm::init_solve_data(
		Eigen::MatrixXd &sol,
		const double t,
		const std::string &state_prefix,
		const InitialConditionOverride *initial_condition_override)
	{
		assert(sol.cols() == 1);
		assert(!problem->is_scalar()); // tensor

		// FIXME
		//  if (optimization_enabled != solver::CacheLevel::None)
		//  {
		//  	if (initial_sol_update.size() == ndof())
		//  		sol = initial_sol_update;
		//  	else
		//  		initial_sol_update = sol;
		//  }

		// --------------------------------------------------------------------
		// Check for initial intersections
		if (args["contact"]["enabled"])
		{
			POLYFEM_SCOPED_TIMER("Check for initial intersections");

			const Eigen::MatrixXd displaced = collision_mesh_.displace_vertices(
				utils::unflatten(sol, mesh_->dimension()));

			if (ipc::has_intersections(collision_mesh_, displaced, ipc::create_broad_phase(args["solver"]["contact"]["CCD"]["broad_phase"]).get()))
			{
				io::OBJWriter::write(
					resolve_output_path("intersection.obj"), displaced,
					collision_mesh_.edges(), collision_mesh_.faces());
				log_and_throw_error("Unable to solve, initial solution has intersections!");
			}
		}

		// --------------------------------------------------------------------

		if (problem->is_time_dependent())
		{
			POLYFEM_SCOPED_TIMER("Initialize time integrator");
			solve_data_.time_integrator = ImplicitTimeIntegrator::construct_time_integrator(args["time"]["integrator"]);

			Eigen::MatrixXd solution, velocity, acceleration;
			initial_solution(solution, initial_condition_override, state_prefix); // Reload this because we need all previous solutions
			solution.col(0) = sol;                                                // Make sure the current solution is the same as `sol`
			assert(solution.rows() == sol.size());
			initial_velocity(velocity, initial_condition_override, state_prefix);
			assert(velocity.rows() == sol.size());
			initial_acceleration(acceleration, initial_condition_override, state_prefix);
			assert(acceleration.rows() == sol.size());
			if (solution.cols() != velocity.cols() || solution.cols() != acceleration.cols())
			{
				log_and_throw_error(
					"Incompatible initial-condition history for transient solve: "
					"solution has {} columns, velocity has {}, acceleration has {}.",
					solution.cols(), velocity.cols(), acceleration.cols());
			}

			solve_data_.time_integrator->init(solution, velocity, acceleration, dt);
			assert(solve_data_.time_integrator != nullptr && "Transient nonlinear elasticity requires an initialized time integrator");
		}
		else
		{
			solve_data_.time_integrator = nullptr;
		}

		// --------------------------------------------------------------------
		// Initialize forms

		// --------------------------------------------------------------------
		// Initialize nonlinear problems

		init_forms(args, mesh_->dimension(), sol, t);

		if (pure_mass_.size() == 0)
			pure_mass_assembler_->assemble(mesh_->is_volume(), space_.n_bases, space_.basis_list(), space_.geometry_basis_list(), pure_mass_ass_vals_cache_, 0, pure_mass_, true);
	}

	void NonlinearElasticVarForm::prepare_for_embedding()
	{
		prepare();
	}

	void NonlinearElasticVarForm::initial_solution_for_embedding(
		Eigen::MatrixXd &solution, const std::string &state_prefix) const
	{
		initial_solution(solution, nullptr, state_prefix);
		if (solution.cols() > 1)
			solution.conservativeResize(Eigen::NoChange, 1);
	}

	void NonlinearElasticVarForm::init_forms_for_embedding(
		Eigen::MatrixXd &solution, const double t, const std::string &state_prefix)
	{
		prepare();
		init_solve_data(solution, t, state_prefix);
	}

	void NonlinearElasticVarForm::advance_for_embedding(const Eigen::VectorXd &solution)
	{
		assert(solve_data_.time_integrator);
		solve_data_.time_integrator->update_quantities(solution);
		solve_data_.update_dt();
	}

	void NonlinearElasticVarForm::update_barrier_stiffness_for_embedding(
		const Eigen::VectorXd &solution)
	{
		solve_data_.update_barrier_stiffness(solution);
	}

	bool NonlinearElasticVarForm::save_timestep_for_embedding(
		const double time, const int step, const double dt,
		const Eigen::MatrixXd &solution, paraviewo::VTMWriter &vtm,
		const std::string &block_prefix) const
	{
		return save_timestep_to_vtm(time, step, dt, solution, vtm, block_prefix);
	}

	int NonlinearElasticVarForm::embedding_ndof() const
	{
		return mesh_ ? space_.n_bases * mesh_->dimension() : 0;
	}

	void NonlinearElasticVarForm::solve_tensor_nonlinear(
		const int step,
		Eigen::MatrixXd &sol,
		const bool init_lagging)
	{
		const auto diagnostic_start_time = std::chrono::steady_clock::now();
		const bool diagnostics_enabled = args["output"].value("physical_diagnostics", false);
		configure_coefficient_diagnostics(step, "initialization");
		if (diagnostics_enabled && diagnostic_run_id_.empty())
			diagnostic_run_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
		const Eigen::VectorXd diagnostic_start = diagnostics_enabled ? Eigen::VectorXd(sol) : Eigen::VectorXd();
		std::string diagnostic_phase = "initialization";
		json diagnostic_termination = {{"unavailable_reason", "No completed subsolve"}};
		json diagnostic_lagging = {{"state", "not reached"}};
		const auto observe_friction = [&]() -> json {
			try
			{
				const auto &friction = solve_data_.friction_form;
				if (!friction || !friction->enabled())
					return {{"unavailable_reason", "No enabled friction form"}};
				const double scale = solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
				if (!(scale > 0) || !std::isfinite(scale))
					return {{"unavailable_reason", "Invalid acceleration scaling"}};
				Eigen::VectorXd g;
				friction->first_derivative(sol, g);
				g /= scale;
				if (!g.allFinite())
					return {{"unavailable_reason", "Nonfinite friction gradient"}};
				return {{"gradient_force_units", std::vector<double>(g.data(), g.data() + g.size())},
						{"scope", "Frozen friction gradient at the returned full coordinates on the indicated side of the final lag update"}};
			}
			catch (const std::exception &e)
			{
				return {{"unavailable_reason", e.what()}};
			}
		};

		// RB-04 remainder: passive attempt observation. The iteration observer
		// sees every trial sweep handed to the contact broad phase, the forms'
		// step bound, every line-search validity trial and every accepted
		// iterate, in full coordinates; it writes the opt-in
		// solver-attempts.jsonl stream and retains what the endpoint record
		// needs (attempt summary, last proposal, a failed attempt's last
		// internal iterate). It cannot alter the solve.
		constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
		struct AttemptObservation
		{
			std::ofstream stream;
			int minimize_index = 0, accepted_iterations = 0, rejected_proposals = 0, feasibility_checks = 0;
			int line_search_truncated = 0, step_bound_limited = 0;
			int validity_checks = 0, validity_rejections = 0, stall_retunes = 0;
			int aborted_proposals = 0; ///< RB-05: proposals whose line search an exception abandoned
			// Pending proposal of the current iteration.
			bool has_proposal = false;
			size_t proposal_builds = 0;         ///< completed broad-phase builds when the proposal was observed
			Eigen::VectorXd proposal_x0, trial; ///< trial = x1 - x0 of the sweep handed to the broad phase
			double trial_norm = 0, trial_linf = 0, step_bound = std::numeric_limits<double>::quiet_NaN();
			int iteration_validity_checks = 0, iteration_validity_rejections = 0;
			// Last accepted iterate.
			bool has_iterate = false;
			Eigen::VectorXd iterate;
			int iterate_iteration = -1, iterate_minimize_index = 0;
			json last_proposal;
		} attempts;
		json diagnostic_observations = json::object();
		const auto finite_or_null = [](double v) { return std::isfinite(v) ? json(v) : json(nullptr); };
		const auto info_number = [](const json *info, const char *key) {
			return info && info->contains(key) && (*info)[key].is_number() ? (*info)[key].get<double>() : std::numeric_limits<double>::quiet_NaN();
		};
		const auto trial_json = [&]() -> json {
			return {{"norm", attempts.trial_norm}, {"linf", attempts.trial_linf}, {"step_bound", finite_or_null(attempts.step_bound)}, {"validity_checks", attempts.iteration_validity_checks}, {"validity_rejections", attempts.iteration_validity_rejections}, {"scope", "Sweep handed to the contact broad phase after the finite-energy stage; step_bound is the forms' inversion/CCD fraction of it"}};
		};
		// PolySolve's solver info at post_step holds the objective and gradient
		// norm of the iterate the direction was computed from (x0), not of
		// the accepted point; the start row carries the start energy only.
		const auto write_attempt = [&](const std::string &kind, int iteration, const json &trial, const json &accepted, double energy, double grad_norm) {
			if (!attempts.stream.is_open())
				return;
			const json row = {{"schema", "polyfem.solver-attempt"}, {"version", 1}, {"run_id", diagnostic_run_id_}, {"step", step}, {"phase", diagnostic_phase}, {"minimize_index", attempts.minimize_index}, {"iteration", iteration}, {"kind", kind}, {"trial", trial}, {"accepted", accepted}, {"energy_objective_at_x0", finite_or_null(energy)}, {"gradient_norm_objective_at_x0", finite_or_null(grad_norm)}, {"units", "internal length; objective units at the iterate the proposal was computed from; Euclidean/Linf norms over full node-major DOFs"}};
			// Flushed per row: an uncaught solver exception terminates the
			// process without unwinding, so a buffered stream would lose the
			// failed attempt's history.
			attempts.stream << row.dump() << std::endl;
		};
		// A pending proposal with a step bound that no accepted iterate
		// followed was rejected by the line search; one without a step bound
		// was an ALSolver feasibility check, not a Newton proposal.
		const auto flush_pending_proposal = [&]() {
			if (!attempts.has_proposal)
				return;
			if (std::isfinite(attempts.step_bound))
			{
				++attempts.rejected_proposals;
				write_attempt("rejected", attempts.has_iterate ? attempts.iterate_iteration : 0, trial_json(), nullptr, NaN, NaN);
			}
			else
				++attempts.feasibility_checks;
			attempts.has_proposal = false;
		};
		if (auto contact = solve_data_.contact_form)
			contact->reset_candidate_statistics();
		if (diagnostics_enabled && solve_data_.nl_problem)
		{
			attempts.stream.open(resolve_output_path("solver-attempts.jsonl"), std::ios::app);
			if (!attempts.stream)
				logger().warn("Could not open solver-attempts.jsonl; attempt stream disabled for step {}", step);
			solve_data_.nl_problem->set_iteration_observer([&](const solver::IterationObservation &o) {
				using Kind = solver::IterationObservation::Kind;
				switch (o.kind)
				{
				case Kind::Validity:
					// Trials of a feasibility check (proposal without a step bound) are not line-search trials.
					if (attempts.has_proposal && !std::isfinite(attempts.step_bound))
						break;
					++attempts.validity_checks;
					++attempts.iteration_validity_checks;
					if (!o.valid)
					{
						++attempts.validity_rejections;
						++attempts.iteration_validity_rejections;
					}
					break;
				case Kind::Proposal:
					flush_pending_proposal();
					attempts.has_proposal = true;
					attempts.proposal_builds = solve_data_.contact_form ? solve_data_.contact_form->candidate_statistics().builds : 0;
					attempts.proposal_x0 = *o.x0;
					attempts.trial = *o.x1 - *o.x0;
					attempts.trial_norm = attempts.trial.norm();
					attempts.trial_linf = attempts.trial.size() ? attempts.trial.lpNorm<Eigen::Infinity>() : 0;
					attempts.step_bound = NaN;
					break;
				case Kind::StepBound:
					attempts.step_bound = o.step_bound;
					break;
				case Kind::LineSearchEnd:
					// A real line search ends before its accepted iterate is
					// reported; only a feasibility check is closed here.
					if (attempts.has_proposal && !std::isfinite(attempts.step_bound))
						flush_pending_proposal();
					break;
				case Kind::Accepted:
				{
					const double energy = info_number(o.solver_info, "energy");
					const double grad_norm = info_number(o.solver_info, "gradNorm");
					if (attempts.has_proposal && !std::isfinite(attempts.step_bound))
						flush_pending_proposal();
					if (!attempts.has_proposal)
					{
						// PolySolve reports the start point before its first
						// iteration; its gradient is not evaluated yet.
						++attempts.minimize_index;
						attempts.iteration_validity_checks = attempts.iteration_validity_rejections = 0;
						attempts.iterate_iteration = 0;
						write_attempt("start", 0, nullptr, nullptr, energy, NaN);
					}
					else
					{
						// o.iteration counts the iterations completed before this update.
						const int iteration = o.iteration + 1;
						const Eigen::VectorXd dx = *o.x1 - attempts.proposal_x0;
						const double fraction = attempts.trial_norm > 0 ? dx.dot(attempts.trial) / (attempts.trial_norm * attempts.trial_norm) : NaN;
						const json trial = trial_json();
						const json accepted = {{"fraction_of_trial", finite_or_null(fraction)}, {"norm", dx.norm()}, {"linf", dx.size() ? dx.lpNorm<Eigen::Infinity>() : 0}};
						// Backtracking beyond the forms' bound is a line-search
						// truncation; stopping at the bound is the bound's doing.
						if (std::isfinite(attempts.step_bound) && attempts.step_bound < 1)
							++attempts.step_bound_limited;
						if (std::isfinite(fraction) && std::isfinite(attempts.step_bound) && fraction < attempts.step_bound * (1 - 1e-12))
							++attempts.line_search_truncated;
						attempts.last_proposal = {{"trial_norm", attempts.trial_norm}, {"trial_linf", attempts.trial_linf}, {"step_bound", finite_or_null(attempts.step_bound)}, {"accepted_fraction_of_trial", finite_or_null(fraction)}, {"accepted_norm", dx.norm()}, {"iteration", iteration}, {"minimize_index", attempts.minimize_index}, {"scope", "Last Newton proposal accepted before this record; the per-iteration stream is solver-attempts.jsonl"}};
						++attempts.accepted_iterations;
						write_attempt("accepted", iteration, trial, accepted, energy, grad_norm);
						attempts.has_proposal = false;
						attempts.iteration_validity_checks = attempts.iteration_validity_rejections = 0;
						attempts.iterate_iteration = iteration;
					}
					attempts.has_iterate = true;
					attempts.iterate = *o.x1;
					attempts.iterate_minimize_index = attempts.minimize_index;
					break;
				}
				}
			});
			// Barrier energy of the start coordinates under the production
			// coefficient state at solve start: for the first accepted step of
			// this process it is B(x_0; theta_0) of the retuning convention.
			if (auto barrier = std::dynamic_pointer_cast<BarrierContactForm>(solve_data_.contact_form); barrier && barrier->enabled())
			{
				try
				{
					const double scale = solve_data_.time_integrator ? solve_data_.time_integrator->acceleration_scaling() : 1;
					if (!(scale > 0) || !std::isfinite(scale))
						throw std::runtime_error("Invalid acceleration scaling");
					const double energy = barrier->diagnostic_snapshot(diagnostic_start).value(diagnostic_start) / scale;
					if (!std::isfinite(energy))
						throw std::runtime_error("Nonfinite barrier energy");
					diagnostic_observations["barrier_energy_at_solve_start"] = {{"value", energy}, {"scope", "Start coordinates evaluated with the production coefficient state at solve start (private snapshot); physical energy units"}};
					if (!trajectory_accounting_.has_previous_endpoint)
					{
						trajectory_accounting_.previous_endpoint_barrier_energy = energy;
						trajectory_accounting_.previous_endpoint_barrier_energy_available = true;
					}
				}
				catch (const std::exception &e)
				{
					diagnostic_observations["barrier_energy_at_solve_start"] = {{"value", nullptr}, {"unavailable_reason", e.what()}};
				}
			}
		}
		// Clears the observer before the locals it captures are destroyed
		// (declared after them, so destroyed first), on every exit path.
		struct ObserverGuard
		{
			std::shared_ptr<solver::NLProblem> problem;
			~ObserverGuard()
			{
				if (problem)
					problem->set_iteration_observer(nullptr);
			}
		} observer_guard{diagnostics_enabled ? solve_data_.nl_problem : nullptr};

		const auto emit_diagnostics = [&](const std::string &outcome, const std::string &error = "") {
			try
			{
				flush_pending_proposal();
				json observations = diagnostic_observations;
				json candidates = nullptr;
				if (auto contact = solve_data_.contact_form)
				{
					const auto &stats = contact->candidate_statistics();
					candidates = {{"builds", stats.builds}, {"last", stats.last}, {"max", stats.max}};
					// RB-05: the broad phase's intermediate buffers, when measured.
					if (stats.intermediates_measured)
						candidates["intermediates"] = {{"cell_items_last", stats.last_cell_items}, {"cell_items_max", stats.max_cell_items}, {"candidate_emissions_last", stats.last_candidate_emissions}, {"candidate_emissions_max", stats.max_candidate_emissions}, {"scope", "Hash-grid (box, cell) items per build and pre-filter pair emissions per build (emissions counted only while solver.contact.CCD.resource_limits is enabled, otherwise 0)"}};
					else
						candidates["intermediates"] = nullptr;
				}
				observations["summary"] = {{"minimize_calls", attempts.minimize_index}, {"accepted_iterations", attempts.accepted_iterations}, {"rejected_proposals", attempts.rejected_proposals}, {"aborted_proposals", attempts.aborted_proposals}, {"feasibility_checks", attempts.feasibility_checks}, {"line_search_truncated", attempts.line_search_truncated}, {"step_bound_limited", attempts.step_bound_limited}, {"validity_checks", attempts.validity_checks}, {"validity_rejections", attempts.validity_rejections}, {"stall_retunes", attempts.stall_retunes}, {"broad_phase_candidates", candidates}, {"scope", "All PolySolve minimize calls of this attempt (AL, reduced, lagging); restarts and AL weights are in termination and the coefficient-event stream"}};
				if (attempts.last_proposal.is_object())
					observations["proposed_displacement"] = attempts.last_proposal;
				if (outcome != "accepted" && attempts.has_iterate && attempts.iterate.allFinite())
					observations["last_internal_iterate"] = {{"value", std::vector<double>(attempts.iterate.data(), attempts.iterate.data() + attempts.iterate.size())},
															 {"iteration", attempts.iterate_iteration},
															 {"minimize_index", attempts.iterate_minimize_index},
															 {"scope", "Last accepted Newton iterate observed inside the failed attempt, full coordinates; not the retained caller coordinates and not a restored accepted state. Its objective history is in solver-attempts.jsonl"}};
				write_physical_diagnostics(step, sol, diagnostic_start, outcome, diagnostic_phase,
										   diagnostic_termination, diagnostic_lagging, observations,
										   std::chrono::duration<double>(std::chrono::steady_clock::now() - diagnostic_start_time).count(), error);
			}
			catch (const std::exception &e)
			{
				logger().warn("Physical diagnostic emission failed: {}", e.what());
			}
		};
		try
		{
			assert(solve_data_.nl_problem != nullptr && "Nonlinear forms must initialize the nonlinear problem before solving");
			solver::NLProblem &nl_problem = *(solve_data_.nl_problem);

			assert(sol.size() == rhs_.size());

			if (nl_problem.uses_lagging())
			{
				if (init_lagging)
				{
					POLYFEM_SCOPED_TIMER("Initializing lagging");
					nl_problem.init_lagging(sol);
				}
				logger().info("Lagging iteration 1:");
			}

			save_subsolve(0, step, sol);

			std::shared_ptr<polysolve::nonlinear::Solver> nl_solver =
				polysolve::nonlinear::Solver::create(args["solver"]["augmented_lagrangian"]["nonlinear"], args["solver"]["linear"], units.characteristic_length(), logger());

			// Heuristic initializer from the weighted elastic Hessian only.
			// This is not a curvature bound relative to the BC metric: inertia,
			// other forms and coupling are omitted. The numeric floor 1 is
			// expressed in internal objective/displacement-squared units.
			double initial_al_weight;
			if (args["solver"]["augmented_lagrangian"]["initial_weight"].is_string())
			{
				assert(args["solver"]["augmented_lagrangian"]["initial_weight"] == "hessian_scaled");
				if (solve_data_.elastic_form == nullptr)
					log_and_throw_error("augmented_lagrangian/initial_weight=\"hessian_scaled\" requires an elastic form!");

				StiffnessMatrix elastic_hessian;
				solve_data_.elastic_form->second_derivative(sol, elastic_hessian);
				double max_entry = 0;
				for (int k = 0; k < elastic_hessian.outerSize(); ++k)
					for (StiffnessMatrix::InnerIterator it(elastic_hessian, k); it; ++it)
						max_entry = std::max(max_entry, std::abs(it.value()));

				const double multiplier = args["solver"]["augmented_lagrangian"]["initial_weight_multiplier"];
				initial_al_weight = std::max(multiplier * max_entry, 1.0);
				logger().info("Using hessian-scaled initial AL weight: {:g} (max |H| = {:g})", initial_al_weight, max_entry);
			}
			else
				initial_al_weight = args["solver"]["augmented_lagrangian"]["initial_weight"];

			// Stall detection: restart the nonlinear solve with retuned barrier
			// stiffness when the line search collapses (semi-implicit mode only).
			solver::StallRestartOptions stall_opts;
			std::function<bool(const Eigen::VectorXd &)> on_stall = nullptr;
			if (auto barrier_form = std::dynamic_pointer_cast<solver::BarrierContactForm>(solve_data_.contact_form);
				barrier_form != nullptr && barrier_form->uses_semi_implicit_stiffness())
			{
				const json &restart_opts = args["solver"]["contact"]["semi_implicit"]["restart"];
				stall_opts.enabled = restart_opts["enabled"];
				stall_opts.alpha_threshold = restart_opts["alpha_threshold"];
				stall_opts.patience = restart_opts["patience"];
				stall_opts.min_iterations = restart_opts["min_iterations"];
				stall_opts.soft_iteration_limit = restart_opts["soft_iteration_limit"];
				stall_opts.max_restarts = restart_opts["max_restarts"];

				const double stall_trim_factor = restart_opts["stall_trim_factor"];
				on_stall = [barrier_form, stall_trim_factor, &attempts](const Eigen::VectorXd &x) {
					++attempts.stall_retunes;
					return barrier_form->retune_on_stall(x, stall_trim_factor);
				};
			}

			ALSolver al_solver(
				solve_data_.al_form,
				initial_al_weight,
				args["solver"]["augmented_lagrangian"]["scaling"],
				args["solver"]["augmented_lagrangian"]["max_weight"],
				args["solver"]["augmented_lagrangian"]["eta"],
				[&](const Eigen::VectorXd &x) {
					this->solve_data_.update_barrier_stiffness(sol);
				},
				stall_opts, on_stall);

			al_solver.post_subsolve = [&](const double al_weight) {
				diagnostic_termination = al_solver.info();
				stats.solver_info.push_back(
					{{"type", al_weight > 0 ? "al" : "rc"},
					 {"t", step},
					 {"info", al_solver.info()}});
				if (al_weight > 0)
					stats.solver_info.back()["weight"] = al_weight;
				save_subsolve(stats.solver_info.size(), step, sol);
			};

			Eigen::MatrixXd prev_sol = sol;
			diagnostic_phase = "augmented_lagrangian";
			configure_coefficient_diagnostics(step, diagnostic_phase);
			try
			{
				al_solver.solve_al(nl_problem, sol,
								   args["solver"]["augmented_lagrangian"]["nonlinear"], args["solver"]["linear"], units.characteristic_length());
			}
			catch (...)
			{
				diagnostic_termination = al_solver.info();
				throw;
			}

			diagnostic_phase = "reduced";
			configure_coefficient_diagnostics(step, diagnostic_phase);
			try
			{
				al_solver.solve_reduced(nl_problem, sol,
										args["solver"]["nonlinear"], args["solver"]["linear"], units.characteristic_length());
			}
			catch (...)
			{
				diagnostic_termination = al_solver.info();
				throw;
			}

			if (args["space"]["advanced"]["count_flipped_els_continuous"])
			{
				const auto invalidList = utils::count_invalid(mesh_->dimension(), space_.basis_list(), space_.geometry_basis_list(), sol);
				logger().debug("Flipped elements (cnt {}) : {}", invalidList.size(), invalidList);
			}

			const double lagging_tol = args["solver"]["contact"].value("friction_convergence_tol", 1e-2) * units.characteristic_length();

			bool lagging_converged = !nl_problem.uses_lagging();
			diagnostic_lagging = {{"state", lagging_converged ? "not applicable" : "pending"}};
			for (int lag_i = 1; !lagging_converged; lag_i++)
			{
				Eigen::VectorXd tmp_sol = nl_problem.full_to_reduced(sol);

				diagnostic_phase = "lagging";
				configure_coefficient_diagnostics(step, diagnostic_phase);
				const json friction_before = diagnostics_enabled ? observe_friction() : json();
				nl_problem.update_lagging(tmp_sol, lag_i);

				Eigen::VectorXd grad;
				nl_problem.gradient(tmp_sol, grad);
				diagnostic_lagging = {{"iteration", lag_i}, {"updated_lag_residual_norm_objective", grad.norm()}, {"tolerance", lagging_tol}, {"state", "not converged"}};
				if (diagnostics_enabled)
				{
					diagnostic_lagging["friction_before_update"] = friction_before;
					diagnostic_lagging["friction_after_update"] = observe_friction();
				}
				const double delta_x_norm = (prev_sol - sol).lpNorm<Eigen::Infinity>();
				logger().debug("Lagging convergence grad_norm={:g} tol={:g} (||Δx||={:g})", grad.norm(), lagging_tol, delta_x_norm);
				if (grad.norm() <= lagging_tol)
				{
					logger().info(
						"Lagging converged in {:d} iteration(s) (grad_norm={:g} tol={:g})",
						lag_i, grad.norm(), lagging_tol);
					diagnostic_lagging["state"] = "converged";
					lagging_converged = true;
					break;
				}

				if (delta_x_norm <= 1e-12)
				{
					logger().warn(
						"Lagging produced tiny update between iterations {:d} and {:d} (grad_norm={:g} grad_tol={:g} ||Δx||={:g} Δx_tol={:g}); stopping early",
						lag_i - 1, lag_i, grad.norm(), lagging_tol, delta_x_norm, 1e-6);
					lagging_converged = false;
					break;
				}

				if (lag_i >= nl_problem.max_lagging_iterations())
				{
					logger().warn(
						"Lagging failed to converge with {:d} iteration(s) (grad_norm={:g} tol={:g})",
						lag_i, grad.norm(), lagging_tol);
					lagging_converged = false;
					break;
				}

				logger().info("Lagging iteration {:d}:", lag_i + 1);
				nl_problem.init(sol);
				solve_data_.update_barrier_stiffness(sol);
				nl_problem.normalize_forms();
				nl_solver->minimize(nl_problem, tmp_sol);
				diagnostic_termination = nl_solver->info();
				diagnostic_termination["termination_reason"] = polysolve::nonlinear::status_message(nl_solver->status());
				nl_problem.finish();
				prev_sol = sol;
				sol = nl_problem.reduced_to_full(tmp_sol);

				stats.solver_info.push_back(
					{{"type", "rc"},
					 {"t", step},
					 {"lag_i", lag_i},
					 {"info", nl_solver->info()}});
				save_subsolve(stats.solver_info.size(), step, sol);
			}
			diagnostic_phase = "returned_endpoint";
			emit_diagnostics("accepted");
		}
		catch (const std::exception &e)
		{
			diagnostic_termination = {{"exception", e.what()}, {"subsolve_state_at_failure", diagnostic_termination}};
			// RB-05: a proposal still pending when the attempt fails was neither
			// accepted nor rejected -- an exception abandoned its line search
			// (a broad-phase resource failure, an allocation failure). Record
			// it with its trial norms and whether the broad phase completed
			// its build, instead of letting the flush misfile it.
			if (attempts.has_proposal)
			{
				json trial = trial_json();
				trial["broad_phase_built"] = solve_data_.contact_form && solve_data_.contact_form->candidate_statistics().builds > attempts.proposal_builds;
				write_attempt("aborted", attempts.has_iterate ? attempts.iterate_iteration : 0, trial, nullptr, NaN, NaN);
				++attempts.aborted_proposals;
				attempts.has_proposal = false;
			}
			emit_diagnostics("failed_attempt", e.what());
			throw;
		}
	}

} // namespace polyfem::varform
