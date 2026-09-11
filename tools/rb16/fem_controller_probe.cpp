// RB-16 isolated FEM controller comparison. Assembly adapted from RB-14; originals unchanged.
#include <polyfem/State.hpp>
#include <polyfem/varforms/NonlinearElasticVarForm.hpp>
#include <polyfem/solver/forms/ElasticForm.hpp>
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Logger.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <polysolve/nonlinear/PostStepData.hpp>
#include <ipc/barrier/barrier.hpp>
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
		double trial_displacement_cap() const override { return 50.; }
		size_t count() const { return collision_set_.size(); }
		double effective() const { return collision_set_.empty() ? 0. : barrier_stiffness() * collision_set_[0].stiffness_scale; }
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
		const double dt = .2, speed = 0.;
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
		double applied_force = 0., prescribed_top = 0.;
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
			energy -= applied_force * u[2 * point + 1];
			if (grad)
				(*grad)[2 * point + 1] -= applied_force;
			return energy;
		};
		V residual;
		M H;
		noncontact(x, &residual, &H);
		V J = V::Zero(free.size());
		for (int i = 0; i < int(free.size()); ++i)
			if (free[i] == 2 * point + 1)
				J[i] = 1.;
		M A = block(H, free);
		Eigen::LLT<M> initial_factor(A);
		require(initial_factor.info() == Eigen::Success, "initial SPD tangent");
		const double K0 = 1. / J.dot(initial_factor.solve(J));
		const double raw = H(2 * point + 1, 2 * point + 1) / .015;
		const double multiplier = cfg.value("multiplier", .01);
		const bool current = cfg.value("policy", std::string("outer")) == "current";
		const double L = .1 * std::sqrt(.5), U = .1 * std::sqrt(.9);
		const double factor_limit = cfg.value("factor", 2.);
		const double margin = cfg.value("hysteresis", 0.);
		const int window = cfg.value("window", 1);
		const std::string variant = cfg.value("variant", std::string("tangent"));
		M positions(3, 2);
		positions << -9.5, -.12, 10.5, -.12, .5, 0.;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		StiffnessMatrix B(3, nodes + 2);
		B.insert(0, nodes) = 1.;
		B.insert(1, nodes + 1) = 1.;
		B.insert(2, point) = 1.;
		ipc::CollisionMesh cm(positions, edges, Eigen::MatrixXi(), B);
		Contact contact(cm, current, H, 1.);
		contact.set_system_hessian_provider([&](const V &u, StiffnessMatrix &h) { M dense; noncontact(u, nullptr, &dense); h = dense.sparseView(); });
		contact.set_system_gradient_provider([&](const V &u, V &g) { V head; noncontact(u, &head, nullptr); g = V::Zero(N + 4); g.head(N) = head; });
		contact.set_barrier_stiffness(current ? multiplier : multiplier * raw);
		contact.start(x);
		require(contact.count() == 0, "initially absent contact");
		const auto gap = [&](const V &u) { return .12 + u[2 * point + 1] - .5 * (u[N + 1] + u[N + 3]); };
		const auto unit_force = [](double d) { return -2 * d * ipc::barrier_first_derivative(d * d, .01); };
		const auto min_det = [&](const V &u) { double lowest = 1e100; for (const auto &ids : elements) { Eigen::Matrix2d rest, deformed; for (int j=0;j<2;j++) { rest.col(j)=(coords.row(ids[j+1])-coords.row(ids[0])).transpose(); deformed.col(j)=rest.col(j)+u.segment<2>(2*ids[j+1])-u.segment<2>(2*ids[0]); } lowest=std::min(lowest,deformed.determinant()/rest.determinant()); } return lowest; };
		std::shared_ptr<solver::ElasticForm> elastic;
		for (auto &f : form->embedding_forms())
			if (f->name() == "elastic")
				elastic = std::dynamic_pointer_cast<solver::ElasticForm>(f);
		require(bool(elastic), "elastic form");
		const auto measure = [&](const V &u) {
			contact.solution_changed(u);
			V ng, cg;
			noncontact(u, &ng, nullptr);
			contact.first_derivative(u, cg);
			double residual = (select(ng, free) + select(cg, free)).norm() / (1 + select(ng, free).norm() + select(cg, free).norm());
			double bc = 0.;
			for (int i = 0; i < nodes; i++)
				if (std::abs(coords(i, 1) - 1.) < 1e-10)
					bc = std::max(bc, std::hypot(u[2 * i], u[2 * i + 1] - prescribed_top));
			bc = std::max(bc, u.tail(4).norm());
			double balance = std::hypot(cg(Eigen::seq(0, Eigen::last, 2)).sum(), cg(Eigen::seq(1, Eigen::last, 2)).sum());
			require(bc < 1e-12 && min_det(u) > 0 && gap(u) > 0, "feasible endpoint");
			require(balance < 1e-8 * (1 + cg.norm()), "contact action reaction");
			return json{{"gap", gap(u)}, {"active", contact.count()}, {"effective_k", contact.effective()}, {"trim_or_k", contact.barrier_stiffness()}, {"residual", residual}, {"min_det", min_det(u)}, {"bc_error", bc}, {"contact_balance_error", balance}, {"normal_force", -cg[2 * point + 1]}, {"contact_energy", contact.value(u)}, {"displacement", vector(u)}};
		};
		const auto estimate = [&](const V &u) {
			json result;
			if (variant == "unavailable")
				return json{{"status", "unavailable"}};
			V g;
			M h;
			noncontact(u, &g, &h);
			M reduced = block(h, free);
			Eigen::LLT<M> ll(reduced);
			if (ll.info() != Eigen::Success)
				return json{{"status", "indefinite"}};
			V z = ll.solve(J), y = ll.solve(select(g, free));
			double K = 1. / J.dot(z), p = gap(u) - J.dot(y);
			if (!(K > 0) || !std::isfinite(K) || !std::isfinite(p))
				return json{{"status", "invalid"}};
			require((reduced * z - J).norm() < 1e-8 * (1 + J.norm()), "compliance solve residual");
			double pm = p, pp = p, km = K, kp = K;
			if (variant == "narrow")
			{
				pm -= .001;
				pp += .001;
				km *= .9;
				kp *= 1.1;
			}
			if (variant == "wide")
			{
				pm -= .03;
				pp += .03;
				km *= .1;
				kp *= 10.;
			}
			if (variant == "misleading")
			{
				pm += .06;
				pp += .06;
			}
			result = {{"K", K}, {"p", p}, {"K_bounds", {km, kp}}, {"p_bounds", {pm, pp}}};
			if (pp > U)
			{
				result["status"] = pm > U ? "inactive" : "mixed";
				return result;
			}
			double lo = kp * std::max(L - pm, 0.) / unit_force(L), hi = km * (U - pp) / unit_force(U);
			result["lower"] = lo;
			result["upper"] = hi;
			result["status"] = lo > hi ? "empty" : hi == 0 ? "zero_only"
														   : "nonempty";
			return result;
		};
		json events = json::array(), endpoints = json::array(), solves = json::array();
		double event_energy = 0., displacement_energy = 0.;
		int load_index = 0;
		const auto change = [&](const std::string &reason, const std::function<void()> &operation) {
			auto before = measure(x);
			auto state = contact.diagnostic_state();
			operation();
			auto after = measure(x);
			double delta = after["contact_energy"].get<double>() - before["contact_energy"].get<double>();
			event_energy += delta;
			events.push_back({{"load_index", load_index}, {"reason", reason}, {"before", before}, {"after", after}, {"state_before", state}, {"state_after", contact.diagnostic_state()}, {"energy_change", delta}, {"displacement_work", 0.}, {"history_invalidated", true}});
		};

		// Paired arithmetic diagnostic: integrate the existing objective gradient.
		// No coefficient refresh is allowed inside this fixed-state path.
		const auto energy_integral = [&](const V &from, const V &to, bool five) {
			const std::vector<double> nodes3 = {-.7745966692414834, 0., .7745966692414834};
			const std::vector<double> weights3 = {5. / 9, 8. / 9, 5. / 9};
			const std::vector<double> nodes5 = {-.9061798459386640, -.5384693101056831, 0., .5384693101056831, .9061798459386640};
			const std::vector<double> weights5 = {.2369268850561891, .4786286704993665, .5688888888888889, .4786286704993665, .2369268850561891};
			const auto &ns = five ? nodes5 : nodes3;
			const auto &ws = five ? weights5 : weights3;
			V delta = to - from;
			long double sum = 0.;
			for (int q = 0; q < int(ns.size()); q++)
			{
				V u = from + (.5 + .5 * ns[q]) * delta, ng, cg;
				noncontact(u, &ng, nullptr);
				contact.solution_changed(u);
				contact.first_derivative(u, cg);
				long double slope = 0.;
				for (int i = 0; i < N + 4; i++)
					slope += (long double)delta[i] * (cg[i] + (i < N ? ng[i] : 0.));
				sum += .5 * ws[q] * slope;
			}
			contact.solution_changed(to);
			return double(sum);
		};

		if (cfg.value("derivative_only", false))
		{
			applied_force = .1 * K0;
			V u = x;
			u[2 * point + 1] = -.04;
			V q = V::Zero(N + 4);
			for (int i = 0; i < int(free.size()); i++)
				q[free[i]] = std::sin(i + 1.);
			q.normalize();
			const auto eval = [&](const V &v, V *g, M *h) {
				V ng, cg;
				M nh;
				double e = noncontact(v, g ? &ng : nullptr, h ? &nh : nullptr);
				contact.solution_changed(v);
				if (g)
				{
					contact.first_derivative(v, cg);
					*g = cg;
					g->head(N) += ng;
				}
				if (h)
				{
					StiffnessMatrix ch;
					contact.second_derivative(v, ch);
					*h = M(ch);
					h->topLeftCorner(N, N) += nh;
				}
				return e + contact.value(v);
			};
			V g, gp, gm;
			M h;
			eval(u, &g, &h);
			double epsilon = 1e-6;
			double ep = eval(u + epsilon * q, &gp, nullptr), em = eval(u - epsilon * q, &gm, nullptr);
			double ge = std::abs((ep - em) / (2 * epsilon) - g.dot(q)) / (1 + g.norm());
			double he = ((gp - gm) / (2 * epsilon) - h * q).norm() / (1 + (h * q).norm());
			require(ge < 2e-5 && he < 2e-5, "assembled total objective derivatives");
			V end = u + .001 * q;
			double direct = eval(end, nullptr, nullptr) - eval(u, nullptr, nullptr);
			double three = energy_integral(u, end, false), five = energy_integral(u, end, true);
			require(std::abs(direct - five) < 1e-9 * (1 + std::abs(direct)), "independent energy integral versus energy endpoints");
			out["derivative_control"] = {{"gradient_error", ge}, {"hessian_error", he}, {"direct_energy_difference", direct}, {"integral3", three}, {"integral5", five}};
			out["checks"] = checks;
			out["passed"] = true;
			std::cout << out.dump(2) << std::endl;
			return 0;
		}
		const auto solve = [&]() {
			json start_prediction = estimate(x);
			json history = json::array();
			std::string status = "iteration_limit";
			int evaluations = 0, quadrature_evaluations = 0;
			auto start = Clock::now();
			for (int it = 0; it < 80; it++)
			{
				V ng, cg;
				M nh;
				double energy = noncontact(x, &ng, &nh);
				contact.solution_changed(x);
				contact.first_derivative(x, cg);
				StiffnessMatrix ch;
				contact.second_derivative(x, ch);
				energy += contact.value(x);
				++evaluations;
				V g = select(ng, free) + select(cg, free);
				M h = block(nh, free) + block(M(ch), free);
				double error = g.norm() / (1 + select(ng, free).norm() + select(cg, free).norm());
				history.push_back({{"iteration", it}, {"residual", error}, {"gap", gap(x)}, {"effective_k", contact.effective()}});
				if (error <= 1e-8)
				{
					status = "converged";
					break;
				}
				Eigen::LLT<M> ll(h);
				if (ll.info() != Eigen::Success)
				{
					status = "indefinite_tangent";
					break;
				}
				V reduced = ll.solve(-g), step = V::Zero(N + 4);
				for (int i = 0; i < int(free.size()); i++)
					step[free[i]] = reduced[i];
				double slope = g.dot(reduced);
				if (!(slope < 0))
				{
					status = "non_descent";
					break;
				}
				auto frozen = contact.diagnostic_state();
				double trim = contact.barrier_stiffness();
				double alpha = std::min(1., std::min(contact.max_step_size(x, x + step), elastic->max_step_size(x.head(N), (x + step).head(N))));
				bool accepted = false;
				V trial;
				double old_contact_energy = contact.value(x);
				for (int ls = 0; ls < 40; ls++, alpha *= .5)
				{
					trial = x + alpha * step;
					++evaluations;
					if (min_det(trial) <= 0 || !contact.is_step_collision_free(x, trial) || !elastic->is_step_valid(x.head(N), trial.head(N)))
						continue;
					contact.solution_changed(trial);
					double te = noncontact(trial, nullptr, nullptr) + contact.value(trial);
					bool accept = std::isfinite(te) && te <= energy + 1e-4 * alpha * slope;
					if (cfg.value("arithmetic", std::string("raw")) == "integral")
					{
						double d3 = energy_integral(x, trial, false), d5 = energy_integral(x, trial, true);
						quadrature_evaluations += 8;
						double error = std::abs(d5 - d3);
						accept = std::isfinite(d5) && d5 + error <= 1e-4 * alpha * slope;
						history.back()["integral_difference"] = d5;
						history.back()["quadrature_disagreement"] = error;
						history.back()["raw_energy_difference"] = te - energy;
					}
					if (accept)
					{
						accepted = true;
						history.back()["backtracks"] = ls;
						break;
					}
				}
				require(contact.barrier_stiffness() == trim && contact.diagnostic_state()["refresh_id"] == frozen["refresh_id"], "frozen direction search coefficient state");
				if (!accepted)
				{
					contact.solution_changed(x);
					status = "backtracking_exhausted";
					break;
				}
				displacement_energy += contact.value(trial) - old_contact_energy;
				x = trial;
				if (current)
					change("post_step", [&]() {json info=json::object();contact.post_step(polysolve::nonlinear::PostStepData(it+1,info,x,g)); });
			}
			auto endpoint = measure(x);
			if (status == "converged")
				require(endpoint["residual"].get<double>() <= 1e-8, "actual coefficient convergence");
			json result = {{"load_index", load_index}, {"status", status}, {"history", history}, {"evaluations", evaluations}, {"microseconds", micros(start)}, {"endpoint", endpoint}};

			result["quadrature_gradient_evaluations"] = quadrature_evaluations;
			if (status != "converged" && cfg.value("diagnose", false))
			{
				V ng, cg;
				M nh;
				noncontact(x, &ng, &nh);
				contact.solution_changed(x);
				contact.first_derivative(x, cg);
				StiffnessMatrix ch;
				contact.second_derivative(x, ch);
				V g = select(ng, free) + select(cg, free);
				M h = block(nh, free) + block(M(ch), free);
				Eigen::LLT<M> ll(h);
				if (ll.info() == Eigen::Success)
				{
					V v = ll.solve(-g), step = V::Zero(N + 4);
					for (int i = 0; i < int(free.size()); i++)
						step[free[i]] = v[i];
					V trial = x + step;
					if (min_det(trial) > 0 && contact.is_step_collision_free(x, trial) && elastic->is_step_valid(x.head(N), trial.head(N)))
					{
						contact.solution_changed(x);
						double e0 = noncontact(x, nullptr, nullptr) + contact.value(x);
						contact.solution_changed(trial);
						double e1 = noncontact(trial, nullptr, nullptr) + contact.value(trial);
						double d3 = energy_integral(x, trial, false), d5 = energy_integral(x, trial, true);
						auto next = measure(trial);
						result["terminal_arithmetic_diagnostic"] = {{"raw_difference", e1 - e0}, {"integral3", d3}, {"integral5", d5}, {"armijo_rhs", 1e-4 * g.dot(v)}, {"full_newton_endpoint", next}};
						contact.solution_changed(x);
					}
				}
			}
			result["start_prediction"] = start_prediction;
			if (start_prediction.contains("K"))
			{
				double kh = start_prediction["K"], ph = start_prediction["p"];
				if (start_prediction.contains("p_bounds"))
					ph = .5 * (start_prediction["p_bounds"][0].get<double>() + start_prediction["p_bounds"][1].get<double>());
				double k = current ? contact.effective() : contact.barrier_stiffness();
				if (ph >= .1 || (k == 0 && ph > 0))
					result["predicted_gap"] = ph;
				else if (k > 0)
				{
					double lo = 0., hi = .1;
					for (int j = 0; j < 100; j++)
					{
						double mid = (lo + hi) / 2;
						if (kh * (mid - ph) - k * unit_force(mid) < 0)
							lo = mid;
						else
							hi = mid;
					}
					result["predicted_gap"] = (lo + hi) / 2;
				}
				if (result.contains("predicted_gap"))
					result["prediction_error_over_dhat"] = (gap(x) - result["predicted_gap"].get<double>()) / .1;
			}
			solves.push_back(result);
			return result;
		};
		const std::vector<double> knots = {0., -.25, 0., .10, -.15, 0.};
		std::vector<double> loads = {0.};
		const int subdivisions = cfg.value("subdivisions", 4);
		for (int i = 0; i < int(knots.size()) - 1; i++)
			for (int j = 1; j <= subdivisions; j++)
				loads.push_back((cfg.value("loading", std::string("point")) == "top" ? 1. : K0) * (knots[i] + (knots[i + 1] - knots[i]) * j / subdivisions));
		bool completed = true;
		auto total_start = Clock::now();
		for (double load : loads)
		{
			if (cfg.value("loading", std::string("point")) == "top")
			{
				V trial = x;
				for (int i = 0; i < nodes; i++)
					if (std::abs(coords(i, 1) - 1.) < 1e-10)
						trial[2 * i + 1] = load;
				if (min_det(trial) <= 0 || !elastic->is_step_valid(x.head(N), trial.head(N)) || !contact.is_step_collision_free(x, trial))
				{
					completed = false;
					out["prescribed_step_failure"] = {{"load_index", load_index}, {"requested_top", load}, {"reason", "infeasible prescribed step; no retry"}, {"last_feasible", measure(x)}};
					break;
				}
				prescribed_top = load;
				x = trial;
				contact.solution_changed(x);
				applied_force = 0.;
			}
			else
				applied_force = load;
			if (current)
				change("load_start_refresh", [&]() { contact.refresh_semi_implicit_stiffness(x); });
			int corrections = 0, observations = 0;
			std::string disposition = "current";
			json last;
			while (true)
			{
				last = solve();
				if (last["status"] != "converged")
				{
					completed = false;
					disposition = "numerical_failure";
					break;
				}
				if (current)
					break;
				auto pred = estimate(x);
				disposition = pred["status"].get<std::string>();
				if (disposition != "nonempty")
					break;
				if (gap(x) >= L - margin && gap(x) <= U + margin)
				{
					disposition = "within_trigger";
					break;
				}
				if (++observations < window)
					continue;
				double lo = pred["lower"], hi = pred["upper"], target = lo > 0 ? std::sqrt(lo * hi) : hi / 2;
				double old = contact.barrier_stiffness(), next = std::clamp(target, old / factor_limit, old * factor_limit);
				if (std::abs(next - old) <= 1e-14 * std::max(old, next))
				{
					disposition = "prediction_no_progress";
					break;
				}
				if (corrections >= 12)
				{
					disposition = "correction_budget";
					break;
				}
				require(next > 0 && next / old <= factor_limit * (1 + 1e-12) && old / next <= factor_limit * (1 + 1e-12), "bounded positive correction");
				change("interval_correction", [&]() { contact.set_barrier_stiffness(next); });
				events.back()["prediction"] = pred;
				corrections++;
				observations = 0;
			}
			auto endpoint = measure(x);
			endpoint["load_parameter"] = load;
			endpoint["force_parameter"] = applied_force;
			endpoint["prescribed_top"] = prescribed_top;
			endpoint["numerical_status"] = last["status"];
			endpoint["controller_disposition"] = disposition;
			endpoint["corrections"] = corrections;
			endpoint["prediction"] = estimate(x);
			endpoint["in_band"] = gap(x) >= L - 1e-8 && gap(x) <= U + 1e-8;
			endpoint["controller_rms"] = contact.count() ? json(gap(x)) : json(nullptr);
			endpoint["controller_severity"] = contact.count() ? json(gap(x) * gap(x)) : json(nullptr);
			endpoints.push_back(endpoint);
			if (!completed)
				break;
			if (current)
			{
				change("post_publication_refresh", [&]() { contact.refresh_semi_implicit_stiffness(x); });
				endpoints.back()["next_state_at_same_x"] = measure(x);
			}
			load_index++;
		}
		double final_energy = contact.value(x), accounting = final_energy - event_energy - displacement_energy;
		require(std::abs(accounting) < 1e-8 * (1 + std::abs(final_energy) + std::abs(event_energy) + std::abs(displacement_energy)), "cycle coefficient displacement energy identity");
		out["completed"] = completed;
		out["K0"] = K0;
		out["initial_raw_coefficient"] = raw;
		out["endpoints"] = endpoints;
		out["events"] = events;
		out["solves"] = solves;
		out["cost_us"] = micros(total_start);
		out["coefficient_energy_change"] = event_energy;
		out["contact_displacement_energy_change"] = displacement_energy;
		out["energy_identity_error"] = accounting;
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
