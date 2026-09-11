// RB-14 stage 3. Frozen copy of stage 2 assembly/solver with new estimator experiments.
// Keep fem_probe.cpp unchanged so earlier comparisons remain reproducible.
#include <polyfem/State.hpp>
#include <polyfem/varforms/NonlinearElasticVarForm.hpp>
#include <polyfem/solver/forms/ElasticForm.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Logger.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <fstream>
#include <iostream>
#include <set>
#include <chrono>
using namespace polyfem;
using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;

namespace
{
	using Clock = std::chrono::steady_clock;
	int checks = 0;
	void require(bool ok, const std::string &s)
	{
		++checks;
		if (!ok)
			throw std::runtime_error(s);
	}
	json vector(const V &v) { return std::vector<double>(v.data(), v.data() + v.size()); }
	double micros(Clock::time_point t) { return std::chrono::duration<double, std::micro>(Clock::now() - t).count(); }
	M block(const M &a, const std::vector<int> &ids)
	{
		M b(ids.size(), ids.size());
		for (int i = 0; i < b.rows(); i++)
			for (int j = 0; j < b.cols(); j++)
				b(i, j) = a(ids[i], ids[j]);
		return b;
	}
	V select(const V &a, const std::vector<int> &ids)
	{
		V b(ids.size());
		for (int i = 0; i < b.size(); i++)
			b[i] = a[ids[i]];
		return b;
	}
	class Contact : public solver::BarrierContactForm
	{
	public:
		Contact(const ipc::CollisionMesh &m, bool semi, const M &h, double scale) : BarrierContactForm(m, .1, 1, false, false, false, false, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000, semi ? solver::BarrierStiffnessMode::SemiImplicit : solver::BarrierStiffnessMode::Fixed, json::object(), V::Ones(3))
		{
			set_weight(semi ? scale : 1.);
			set_barrier_stiffness(1.);
			set_system_hessian_provider([h, scale](const V &, StiffnessMatrix &out) { out = (h * scale).sparseView(); });
		}
		void start(const V &x)
		{
			init(x);
			if (uses_semi_implicit_stiffness())
				refresh_semi_implicit_stiffness(x, false);
		}
		double coefficient()
		{
			require(collision_set_.size() == 1, "one selected EV contact");
			return collision_set_[0].stiffness_scale;
		}
	};
} // namespace
int main(int argc, char **argv)
{
	json out;
	try
	{
		if (argc != 2)
			throw std::runtime_error("expected config path");
		json cfg;
		std::ifstream(argv[1]) >> cfg;
		out["config"] = cfg;
		const int n = cfg.value("n", 2);
		const double dt = cfg.value("dt", .2), speed = cfg.value("speed", .3);
		M vertices((n + 1) * (n + 1), 2);
		Eigen::MatrixXi cells(2 * n * n, 3);
		for (int y = 0; y <= n; y++)
			for (int x = 0; x <= n; x++)
				vertices.row(y * (n + 1) + x) << double(x) / n, double(y) / n;
		int e = 0;
		for (int y = 0; y < n; y++)
			for (int x = 0; x < n; x++)
			{
				int a = y * (n + 1) + x, b = a + 1, c = a + n + 1, d = c + 1;
				if (cfg.value("flip", false))
				{
					cells.row(e++) << a, b, c;
					cells.row(e++) << b, d, c;
				}
				else
				{
					cells.row(e++) << a, b, d;
					cells.row(e++) << a, d, c;
				}
			}
		json args;
		args["geometry"][0]["mesh"] = "";
		args["materials"] = {{"type", "NeoHookean"}, {"E", cfg.value("E", 10.)}, {"nu", cfg.value("nu", .3)}, {"rho", cfg.value("rho", 1.)}};
		args["time"] = {{"time_steps", 1}, {"tend", dt}, {"integrator", {{"type", "ImplicitEuler"}}}};
		args["contact"] = {{"enabled", false}};
		args["space"] = {{"discr_order", 1}};
		args["initial_conditions"]["velocity"] = json::array({{{"id", 0}, {"value", {0., -speed}}}});
		args["boundary_conditions"]["rhs"] = {0., -cfg.value("load", 0.)};
		args["output"]["log"]["level"] = "off";
		auto prep = Clock::now();
		State state;
		state.init_logger("", spdlog::level::off, spdlog::level::off, true);
		state.init(args, false);
		state.set_log_level(spdlog::level::off);
		state.set_max_threads(1);
		state.load_mesh(vertices, cells);
		auto form = std::dynamic_pointer_cast<varform::NonlinearElasticVarForm>(state.variational_formulation);
		require(bool(form), "nonlinear embedding unavailable");
		form->prepare_for_embedding();
		M initial;
		form->initial_solution_for_embedding(initial);
		form->init_forms_for_embedding(initial, dt);
		const int N = initial.size(), nodes = N / 2;
		const double scale = form->embedding_time_integrator()->acceleration_scaling();
		require(std::abs(scale - dt * dt) < 1e-12, "implicit Euler scale");
		out["prepare_us"] = micros(prep);
		out["ndof"] = N;
		out["elements"] = cells.rows();
		out["scale"] = scale;
		M coords = M::Zero(nodes, 2);
		std::vector<std::set<int>> adjacency(nodes);
		std::vector<std::vector<int>> elements;
		auto graphstart = Clock::now();
		for (const auto &el : form->embedding_space().basis_list())
		{
			std::vector<int> ids;
			for (const auto &b : el.bases)
			{
				require(b.global().size() == 1, "P1 exact basis mapping");
				const auto &g = b.global()[0];
				coords.row(g.index) = g.node;
				ids.push_back(g.index);
			}
			require(ids.size() == 3, "triangle P1 fixture");
			elements.push_back(ids);
			for (int i : ids)
				for (int j : ids)
					adjacency[i].insert(j);
		}
		out["graph_build_us"] = micros(graphstart);
		int point = -1;
		std::vector<int> free;
		V x = V::Zero(N + 4);
		for (int i = 0; i < nodes; i++)
		{
			if ((coords.row(i) - Eigen::RowVector2d(.5, 0)).norm() < 1e-10)
				point = i;
			if (std::abs(coords(i, 1) - 1.) < 1e-10)
				x[2 * i + 1] = cfg.value("top", 0.);
			else
			{
				free.push_back(2 * i);
				free.push_back(2 * i + 1);
			}
		}
		require(point >= 0, "bottom center node");
		x[N + 1] = x[N + 3] = cfg.value("obstacle", 0.);
		const auto noncontact = [&](const V &u, V *grad, M *hessian) {
			double energy = 0.;
			if (grad)
				*grad = V::Zero(N);
			if (hessian)
				*hessian = M::Zero(N, N);
			for (auto &f : form->embedding_forms())
				if (f->enabled())
				{
					const V head = u.head(N);
					energy += f->value(head) / scale;
					if (grad)
					{
						V g;
						f->first_derivative(head, g);
						*grad += g / scale;
					}
					if (hessian && (f->name() == "elastic" || f->name() == "inertia"))
					{
						StiffnessMatrix h;
						f->second_derivative(head, h);
						*hessian += M(h) / scale;
					}
				}
			return energy;
		};
		V residual;
		M H;
		auto assembly = Clock::now();
		noncontact(x, &residual, &H);
		out["assembly_us"] = micros(assembly);
		out["forms"] = json::array();
		for (auto &f : form->embedding_forms())
			out["forms"].push_back({{"name", f->name()}, {"enabled", f->enabled()}, {"weight", f->weight()}});
		for (auto &f : form->embedding_forms())
			if (f->name() == "inertia")
			{
				V ig;
				StiffnessMatrix ih;
				f->first_derivative(x.head(N), ig);
				f->second_derivative(x.head(N), ih);
				double error = (ig - ih * (x.head(N) - form->embedding_time_integrator()->x_tilde())).norm();
				require(error < 1e-8, "actual inertia residual scaling");
				out["inertia_identity_error"] = error;
				V expected_predictor = V::Zero(N);
				for (int i = 0; i < nodes; i++)
					expected_predictor[2 * i + 1] = -speed * dt;
				require((form->embedding_time_integrator()->x_tilde() - expected_predictor).norm() < 1e-8, "actual initial velocity predictor");
			}
		// Actual exact-selector IPC map: obstacle proxy IDs 0/1 are not FEM IDs.
		M positions(3, 2);
		positions << -9.5, -.08, 10.5, -.08, .5, 0;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		StiffnessMatrix B(3, nodes + 2);
		B.insert(0, nodes) = 1.;
		B.insert(1, nodes + 1) = 1.;
		B.insert(2, point) = 1.;
		ipc::CollisionMesh cm(positions, edges, Eigen::MatrixXi(), B);
		V surfaceJ = V::Zero(6);
		surfaceJ[1] = surfaceJ[3] = -.5;
		surfaceJ[5] = 1.;
		V Jfull = cm.to_full_dof(surfaceJ), J = select(Jfull, free), r = select(residual, free);
		M A = block(H, free);
		const double d0 = .08 + x[2 * point + 1] - .5 * (x[N + 1] + x[N + 3]);
		require(d0 > 0 && d0 < .1, "valid initial contact gap");
		require(std::abs(J.sum() - 1.) < 1e-12, "reduced normal gap derivative");
		V direction = V::LinSpaced(N + 4, -.1, .1);
		M du(nodes + 2, 2);
		for (int i = 0; i < nodes + 2; i++)
			du.row(i) = direction.segment<2>(2 * i);
		M mapped = cm.map_displacements(du);
		V mappedflat(6);
		for (int i = 0; i < 3; i++)
			mappedflat.segment<2>(2 * i) = mapped.row(i).transpose();
		require(std::abs(Jfull.dot(direction) - surfaceJ.dot(mappedflat)) < 1e-8, "IPC virtual work");
		Contact semi(cm, true, H, scale);
		semi.start(x);
		const double assigned = semi.coefficient();
		const double expected = H(2 * point + 1, 2 * point + 1) / (1.5 * .1 * .1);
		out["current_assignment"] = {{"coefficient_trim1", assigned}, {"expected_uncapped", expected}, {"relative_error", std::abs(assigned - expected) / (1 + expected)}};
		require(std::abs(assigned - expected) / (1 + expected) < 1e-8, "actual mapped raw coefficient extraction");
		Contact contact(cm, false, H, scale);
		contact.start(x);
		// Derive unit normal force from actual mapped contact gradient.
		V target = x;
		target[2 * point + 1] += .05 - d0;
		contact.solution_changed(target);
		V bg;
		contact.first_derivative(target, bg);
		const double unitforce = -bg[2 * point + 1];
		require(unitforce > 0, "repulsive target force");
		StiffnessMatrix target_hessian;
		contact.second_derivative(target, target_hessian);
		const double unitcurvature = target_hessian.coeff(2 * point + 1, 2 * point + 1);
		require(unitcurvature > 0 && std::isfinite(unitcurvature), "positive target barrier curvature");
		out["unit_force_at_target"] = unitforce;
		out["unit_curvature_at_target"] = unitcurvature;
		require(std::abs(bg.segment(N, 4).sum() + bg.head(N).sum()) < 1e-8, "contact action reaction");
		contact.solution_changed(x);
		V q = V::Zero(N + 4);
		for (int i = 0; i < int(free.size()); i++)
			q[free[i]] = std::sin(i + 1.);
		q.normalize();
		const double eps = 1e-6;
		V gp, gm;
		double ep = noncontact(x + eps * q, &gp, nullptr), em = noncontact(x - eps * q, &gm, nullptr);
		double fdg = std::abs((ep - em) / (2 * eps) - residual.dot(q.head(N))) / (1 + residual.norm());
		double fdh = ((gp - gm) / (2 * eps) - H * q.head(N)).norm() / (1 + (H * q.head(N)).norm());
		out["fd"] = {{"gradient", fdg}, {"hessian", fdh}};
		require(fdg < 2e-5 && fdh < 2e-5, "noncontact residual/tangent FD");
		Eigen::SelfAdjointEigenSolver<M> eig(A);
		require(eig.info() == Eigen::Success, "eigenvalues");
		out["eigenvalue_range"] = {eig.eigenvalues()[0], eig.eigenvalues().tail(1)[0]};
		require(eig.eigenvalues()[0] > 0, "SPD reference");
		out["condition_number"] = eig.eigenvalues().tail(1)[0] / eig.eigenvalues()[0];
		Eigen::LLT<M> dense(A);
		V z = dense.solve(J), y = dense.solve(r);
		const double K = 1 / J.dot(z), p = d0 - J.dot(y);
		require((A * z - J).norm() / (1 + J.norm()) < 1e-8 && (A * y - r).norm() / (1 + r.norm()) < 1e-8, "dense reference residual");
		StiffnessMatrix sparse = A.sparseView();
		Eigen::SimplicialLLT<StiffnessMatrix> sparse_solver;
		sparse_solver.compute(sparse);
		require(sparse_solver.info() == Eigen::Success, "sparse reference SPD");
		V zs = sparse_solver.solve(J), ys = sparse_solver.solve(r);
		require((zs - z).norm() / (1 + z.norm()) < 1e-8 && (ys - y).norm() / (1 + y.norm()) < 1e-8, "dense/sparse independent reference");
		std::vector<double> factor_times, solve_times;
		for (int rep = 0; rep < 7; rep++)
		{
			auto t = Clock::now();
			sparse_solver.compute(sparse);
			factor_times.push_back(micros(t));
			t = Clock::now();
			V a = sparse_solver.solve(J), b = sparse_solver.solve(r);
			solve_times.push_back(micros(t));
			require(a.allFinite() && b.allFinite(), "timed sparse solve finite");
		}
		std::sort(factor_times.begin(), factor_times.end());
		std::sort(solve_times.begin(), solve_times.end());
		out["sparse_cost"] = {{"median_factor_us", factor_times[3]}, {"median_two_rhs_reusing_factor_us", solve_times[3]}};
		std::vector<int> bottom;
		for (int i = 0; i < int(free.size()); i++)
			if (free[i] % 2 == 1 && std::abs(coords(free[i] / 2, 1)) < 1e-12)
				bottom.push_back(i);
		M normals = M::Zero(J.size(), bottom.size());
		for (int i = 0; i < int(bottom.size()); i++)
			normals(bottom[i], i) = 1.;
		std::vector<double> batch_times, shared_times;
		M responses;
		V shared_y;
		for (int rep = 0; rep < 7; rep++)
		{
			auto t = Clock::now();
			responses = sparse_solver.solve(normals);
			batch_times.push_back(micros(t));
			t = Clock::now();
			shared_y = sparse_solver.solve(r);
			shared_times.push_back(micros(t));
		}
		for (int i = 0; i < responses.cols(); i++)
			require((A * responses.col(i) - normals.col(i)).norm() < 1e-8, "batched normal RHS residual");
		require((A * shared_y - r).norm() / (1 + r.norm()) < 1e-8, "shared predictor residual");
		std::sort(batch_times.begin(), batch_times.end());
		std::sort(shared_times.begin(), shared_times.end());
		out["shared_predictor_cost"] = {{"normal_rhs_count", bottom.size()}, {"median_batch_normal_solve_us", batch_times[3]}, {"median_shared_residual_solve_us", shared_times[3]}, {"dense_rhs_and_solution_bytes", 2 * normals.size() * sizeof(double)}};
		StiffnessMatrix L = sparse_solver.matrixL();
		out["sparse"] = {{"matrix_nnz", sparse.nonZeros()}, {"factor_nnz", L.nonZeros()}, {"matrix_payload_bytes", sparse.nonZeros() * (sizeof(double) + sizeof(int)) + (sparse.outerSize() + 1) * sizeof(int)}, {"factor_payload_bytes", L.nonZeros() * (sizeof(double) + sizeof(int)) + (L.outerSize() + 1) * sizeof(int)}};
		out["reference"] = {{"K", K}, {"p", p}, {"d0", d0}, {"dhat", .1}, {"target", .05}, {"free_dofs", free.size()}};
		const auto min_det = [&](const V &u) {double lowest=1e100;for(const auto&ids:elements){Eigen::Matrix2d rest,deformed;for(int j=0;j<2;j++){rest.col(j)=(coords.row(ids[j+1])-coords.row(ids[0])).transpose();deformed.col(j)=rest.col(j)+u.segment<2>(2*ids[j+1])-u.segment<2>(2*ids[0]);}lowest=std::min(lowest,deformed.determinant()/rest.determinant());}return lowest; };
		std::shared_ptr<solver::ElasticForm> elastic;
		for (auto &f : form->embedding_forms())
			if (f->name() == "elastic")
				elastic = std::dynamic_pointer_cast<solver::ElasticForm>(f);
		require(bool(elastic), "elastic form");
		const auto realize = [&](double k) {
			json record;
			record["k"] = k;
			if (!(k > 0))
			{
				record["status"] = "zero demand: unprotected solve not run";
				return record;
			}
			contact.set_barrier_stiffness(k);
			V u = x;
			json history = json::array();
			std::string status = "iteration limit";
			for (int it = 0; it < 80; it++)
			{
				V g;
				M h;
				double energy = noncontact(u, &g, &h);
				contact.solution_changed(u);
				V gc;
				contact.first_derivative(u, gc);
				StiffnessMatrix hc;
				contact.second_derivative(u, hc);
				energy += contact.value(u);
				V total = select(g, free) + select(gc, free);
				M tangent = block(h, free) + block(M(hc), free);
				double error = total.norm() / (1 + select(g, free).norm() + select(gc, free).norm());
				history.push_back({{"iteration", it}, {"residual", error}, {"gap", .08 + u[2 * point + 1] - u[N + 1]}, {"min_det", min_det(u)}});
				if (error <= 1e-8)
				{
					status = "converged";
					break;
				}
				Eigen::LLT<M> factor(tangent);
				if (factor.info() != Eigen::Success)
				{
					status = "indefinite Newton tangent";
					break;
				}
				V step = V::Zero(N + 4);
				V reduced = factor.solve(-total);
				for (int i = 0; i < int(free.size()); i++)
					step[free[i]] = reduced[i];
				const double slope = total.dot(reduced);
				if (!(slope < 0))
				{
					status = "non-descent";
					break;
				}
				double alpha = std::min(1., std::min(contact.max_step_size(u, u + step), elastic->max_step_size(u.head(N), (u + step).head(N))));
				bool accepted = false;
				for (int ls = 0; ls < 40; ls++, alpha *= .5)
				{
					V trial = u + alpha * step;
					if (min_det(trial) <= 0 || !contact.is_step_collision_free(u, trial) || !elastic->is_step_valid(u.head(N), trial.head(N)))
						continue;
					contact.solution_changed(trial);
					double trialenergy = noncontact(trial, nullptr, nullptr) + contact.value(trial);
					if (std::isfinite(trialenergy) && trialenergy <= energy + 1e-4 * alpha * slope)
					{
						u = trial;
						accepted = true;
						break;
					}
				}
				if (!accepted)
				{
					status = "backtracking exhausted";
					break;
				}
			}
			contact.solution_changed(u);
			V gc;
			contact.first_derivative(u, gc);
			V ng;
			noncontact(u, &ng, nullptr);
			record["status"] = status;
			record["history"] = history;
			record["gap"] = .08 + u[2 * point + 1] - u[N + 1];
			record["min_det"] = min_det(u);
			record["contact_action_reaction_error"] = std::hypot(gc(Eigen::seq(0, Eigen::last, 2)).sum(), gc(Eigen::seq(1, Eigen::last, 2)).sum());
			record["full_noncontact_gradient"] = vector(ng);
			record["contact_gradient"] = vector(gc);
			record["normal_force"] = -gc[2 * point + 1];
			record["displacement"] = vector(u);
			require(record["min_det"].get<double>() > 0, "positive determinant endpoint");
			require(record["contact_action_reaction_error"].get<double>() < 1e-8 * (1 + gc.norm()), "realized action reaction");
			return record;
		};
		out["candidates"] = json::array();
		for (const std::string method : {"full", "radius1", "physical025", "physical05", "physical1", "physical05_shared", "current_trim1", "full_fresh_current", "full_fresh_tangent"})
		{
			std::vector<int> ids;
			auto neighborhood_start = Clock::now();
			if (method.rfind("physical", 0) == 0)
			{
				double radius = method == "physical025" ? .25 : (method == "physical1" ? 1. : .5);
				for (int i = 0; i < int(free.size()); i++)
					if ((coords.row(free[i] / 2) - coords.row(point)).norm() <= radius + 1e-12)
						ids.push_back(i);
			}
			else if (method.rfind("radius", 0) == 0)
			{
				std::set<int> near{point};
				for (int t = 0; t < method.back() - '0'; t++)
				{
					auto copy = near;
					for (int node : copy)
						near.insert(adjacency[node].begin(), adjacency[node].end());
				}
				for (int i = 0; i < int(free.size()); i++)
					if (near.count(free[i] / 2))
						ids.push_back(i);
			}
			else
				for (int i = 0; i < int(free.size()); i++)
					ids.push_back(i);
			const double neighborhood_us = micros(neighborhood_start);
			double kh = 0, ph = 0;
			V local_z = V::Zero(J.size());
			std::vector<double> timings;
			M sub;
			V jr, rr;
			double extraction_us = 0, factor_us = 0;
			for (int rep = 0; rep < 7; rep++)
			{
				auto start = Clock::now();
				sub = block(A, ids);
				jr = select(J, ids);
				rr = select(r, ids);
				extraction_us = micros(start);
				auto factorstart = Clock::now();
				if (method == "diagonal")
				{
					kh = 1 / (jr.array().square() / sub.diagonal().array()).sum();
					ph = d0 - (jr.array() * rr.array() / sub.diagonal().array()).sum();
				}
				else if (method == "direction" || method == "current_trim1")
				{
					double v = jr.dot(sub * jr);
					kh = v / std::pow(jr.squaredNorm(), 2);
					ph = d0 - jr.squaredNorm() * jr.dot(rr) / v;
				}
				else
				{
					Eigen::LLT<M> factor(sub);
					require(factor.info() == Eigen::Success, "candidate SPD");
					V zj = factor.solve(jr), yr = factor.solve(rr);
					kh = 1 / jr.dot(zj);
					ph = d0 - jr.dot(yr);
					for (int i = 0; i < int(ids.size()); i++)
						local_z[ids[i]] = zj[i];
				}
				factor_us = micros(factorstart);
				timings.push_back(micros(start));
			}
			const double local_p = ph;
			if (method == "physical05_shared")
				ph = d0 - J.dot(shared_y);
			double exterior = 0, interior = 0;
			std::set<int> included(ids.begin(), ids.end());
			if (method != "current_trim1")
			{
				for (int i = 0; i < J.size(); i++)
					if (included.count(i))
						interior += (z[i] - local_z[i]) * r[i];
					else
						exterior += z[i] * r[i];
				require(std::abs(local_p - p - exterior - interior) < 1e-8 * (1 + std::abs(p)), "remote influence decomposition");
			}
			std::sort(timings.begin(), timings.end());
			double k = method == "current_trim1" ? assigned : kh * std::max(.05 - ph, 0.) / unitforce;
			std::string protection = "positive demand or omitted zero-demand solve";
			if (ph >= .05 && method == "full_fresh_current")
			{
				k = assigned;
				protection = "fresh current assignment";
			}
			if (ph >= .05 && method == "full_fresh_tangent")
			{
				k = K / unitcurvature;
				protection = "fresh reference/barrier tangent match";
			}

			// Frozen quadratic predicted/realized roots use the real fixed contact force.
			const auto linear_root = [&](double kk, double stiffness, double predictor) {
				if (kk <= 0)
					return json(nullptr);
				double lo = 0, hi = std::max(.1, predictor);
				contact.set_barrier_stiffness(1.);
				for (int it = 0; it < 100; it++)
				{
					double d = (lo + hi) / 2;
					V trial = x;
					trial[2 * point + 1] += d - d0;
					contact.solution_changed(trial);
					V g;
					contact.first_derivative(trial, g);
					double balance = stiffness * (d - predictor) + kk * g[2 * point + 1];
					if (balance > 0)
						hi = d;
					else
						lo = d;
				}
				return json((lo + hi) / 2);
			};
			json row = {{"method", method}, {"K", kh}, {"p", ph}, {"k", k}, {"K_relative_error", kh / K - 1}, {"p_error_over_dhat", (ph - p) / .1}, {"dofs", ids.size()}, {"median_extraction_and_solve_us", timings[3]}, {"last_extraction_us", extraction_us}, {"last_factor_solve_us", factor_us}, {"neighborhood_us", neighborhood_us}, {"dense_matrix_bytes", sub.size() * sizeof(double)}, {"predicted_gap", linear_root(k, kh, ph)}, {"full_quadratic_gap", linear_root(k, K, p)}};
			if (method != "current_trim1" && k > 0 && ph < .05)
				require(std::abs(row["predicted_gap"].get<double>() - .05) < 1e-9, "predicted target root");
			if (method.rfind("radius", 0) == 0 || method.rfind("physical", 0) == 0)
				require(kh >= K * (1 - 1e-8), "fixed neighborhood stiffness upper reference");
			row["protection"] = protection;
			row["local_predictor_before_sharing"] = local_p;
			row["exterior_residual_contribution"] = exterior;
			row["interior_relaxation_contribution"] = interior;
			row["requires_shared_full_predictor"] = method == "physical05_shared";
			if (method == "physical05_shared" && k > 0)
				require(row["full_quadratic_gap"].get<double>() >= .05 - 1e-8, "shared exact predictor one-sided quadratic target");
			auto solve_start = Clock::now();
			row["nonlinear"] = realize(k);
			row["nonlinear_solve_us"] = micros(solve_start);
			out["candidates"].push_back(row);
		}
		if (cfg.contains("stale_k"))
			out["stale_reuse"] = realize(cfg["stale_k"]);
		out["snapshot"] = {{"free_indices", free}, {"gap_derivative", vector(Jfull)}, {"residual", vector(residual)}, {"current_displacement", vector(x)}, {"point_node", point}, {"H", json::array()}};
		for (int i = 0; i < N; i++)
			out["snapshot"]["H"].push_back(vector(H.row(i).transpose()));
		out["checks"] = checks;
		out["passed"] = true;
	}
	catch (const std::exception &e)
	{
		out["error"] = e.what();
		out["checks"] = checks;
		out["passed"] = false;
	}
	std::cout << out.dump(2) << std::endl;
	return out["passed"].get<bool>() ? 0 : 1;
}
