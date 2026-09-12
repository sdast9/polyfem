// RB-03: real mapping characterization; defect assertions are not approval.
#include <polyfem/solver/forms/BarrierContactForm.hpp>
#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp>
#include <polysolve/linear/Solver.hpp>
#include <polyfem/varforms/NonlinearElasticVarForm.hpp>
#include <polyfem/mesh/Mesh.hpp>
#include <polyfem/mesh/Obstacle.hpp>
#include <polyfem/utils/MatrixUtils.hpp>
#include <polyfem/utils/Logger.hpp>
#include <h5pp/h5pp.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace polyfem;
using namespace polyfem::solver;
using M = Eigen::MatrixXd;
using V = Eigen::VectorXd;
namespace
{
	int checks = 0;
	void check(bool v, const std::string &s)
	{
		++checks;
		if (!v)
			throw std::runtime_error(s);
	}
	bool close(double a, double b) { return std::abs(a - b) < 1e-9 * (1 + std::abs(b)); }
	M dofmap(const M &b)
	{
		M a = M::Zero(2 * b.rows(), 2 * b.cols());
		for (int i = 0; i < b.rows(); ++i)
			for (int j = 0; j < b.cols(); ++j)
				a.block<2, 2>(2 * i, 2 * j) = b(i, j) * M::Identity(2, 2);
		return a;
	}
	class Probe : public BarrierContactForm
	{
	public:
		Probe(const ipc::CollisionMesh &m, const M &h, bool semi = true)
			: BarrierContactForm(m, 1, 1, false, false, false, false, false, false, ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 semi ? BarrierStiffnessMode::SemiImplicit : BarrierStiffnessMode::Fixed, json::object(), V::Ones(m.num_vertices()))
		{
			set_weight(1);
			set_barrier_stiffness(1);
			set_system_hessian_provider([h](const V &, StiffnessMatrix &out) { out = h.sparseView(); });
		}
		void start(const V &x)
		{
			init(x);
			if (uses_semi_implicit_stiffness())
				refresh_semi_implicit_stiffness(x, false);
		}
		double coefficient() const
		{
			check(collision_set_.size() == 1, "one contact");
			return collision_set_[0].stiffness_scale;
		}
	};
	json verify(ipc::CollisionMesh &mesh, const M &b, const M &nodes, const V &offset, const M &q, bool semi = true)
	{
		const M a = dofmap(b);
		const int n = a.cols();
		Probe f(mesh, 100 * M::Identity(n, n), semi);
		f.start(offset);
		auto energy = [&](const V &x) {f.solution_changed(x);return f.value(x); };
		V g;
		f.first_derivative(offset, g);
		StiffnessMatrix hs;
		f.second_derivative(offset, hs);
		M h = hs;
		const double e = energy(offset);
		check(e > 0 && g.size() == n && h.rows() == n, "mapped dimensions and active energy");
		V fd(n);
		M fh(n, n);
		const double s = 1e-6, t = 1e-4;
		for (int i = 0; i < n; ++i)
		{
			V p = offset, m = offset;
			p[i] += s;
			m[i] -= s;
			fd[i] = (energy(p) - energy(m)) / (2 * s);
			for (int j = 0; j < n; ++j)
			{
				V pp = offset, pm = offset, mp = offset, mm = offset;
				pp[i] += t;
				pp[j] += t;
				pm[i] += t;
				pm[j] -= t;
				mp[i] -= t;
				mp[j] += t;
				mm[i] -= t;
				mm[j] -= t;
				fh(i, j) = (energy(pp) - energy(pm) - energy(mp) + energy(mm)) / (4 * t * t);
			}
		}
		double ge = (fd - g).norm() / (1 + g.norm()), he = (fh - h).norm() / (1 + h.norm());
		check(ge < 1e-6, "mapped energy gradient FD");
		check(he < 1e-5, "mapped energy Hessian FD");
		V y = V::LinSpaced(a.rows(), -.4, .7), du = V::LinSpaced(n, -.2, .3);
		V pulled = mesh.to_full_dof(y);
		double vw = std::abs(pulled.dot(du) - y.dot(a * du));
		check(vw < 1e-10, "virtual work");
		M surface_h = M::Identity(a.rows(), a.rows()) + y * y.transpose();
		StiffnessMatrix sh = surface_h.sparseView();
		check((M(mesh.to_full_dof(sh)) - a.transpose() * surface_h * a).norm() < 1e-10, "IPC Hessian chain rule");
		check((mesh.map_displacements(utils::unflatten(du, 2)) - b * utils::unflatten(du, 2)).norm() < 1e-12, "displacement map");
		V reduced = V::LinSpaced(q.cols(), -.1, .2);
		V direction = q * reduced;
		double reduced_fd = (energy(offset + s * direction) - energy(offset - s * direction)) / (2 * s);
		check(close(reduced_fd, g.dot(direction)), "prescribed offset reduced directional derivative");
		M forces = utils::unflatten(g, 2);
		double balance = forces.colwise().sum().norm();
		check(balance < 1e-9, "action reaction full DOFs");
		M trans = M::Zero(nodes.rows(), 2);
		trans.col(0).setConstant(.13);
		trans.col(1).setConstant(-.07);
		double translation_error = std::abs(energy(offset + utils::flatten(trans)) - e);
		check(translation_error < 1e-9 * (1 + e), "rigid translation");
		const double theta = .12;
		M r(2, 2);
		r << cos(theta), -sin(theta), sin(theta), cos(theta);
		M current = nodes + utils::unflatten(offset, 2);
		V rotated = utils::flatten(current * r.transpose() - nodes);
		double rotation_error = std::abs(energy(rotated) - e);
		check(rotation_error < 1e-9 * (1 + e), "rigid rotation at frozen coefficient");
		return {{"energy", e}, {"gradient_relative_error", ge}, {"hessian_relative_error", he}, {"virtual_work_error", vw}, {"force_balance_norm", balance}, {"translation_energy_error", translation_error}, {"rotation_energy_error", rotation_error}, {"full_dofs", n}, {"reduced_dofs", q.cols()}};
	}
	ipc::CollisionMesh tiny(const M &p, const M &b)
	{
		Eigen::MatrixXi e(1, 2);
		e << 0, 1;
		return ipc::CollisionMesh(p, e, Eigen::MatrixXi(), b.sparseView());
	}
	void proxy_files(const M &p, const M &b, const std::string &prefix)
	{
		std::ofstream obj(prefix + ".obj");
		for (int i = 0; i < p.rows(); ++i)
			obj << "v " << p(i, 0) << " " << p(i, 1) << " 0\n";
		if (p.rows() > 1)
			obj << "l 1 2\n";
		obj.close();
		h5pp::File file(prefix + ".h5", h5pp::FileAccess::REPLACE);
		std::vector<double> values;
		std::vector<int> rows, cols;
		for (int i = 0; i < b.rows(); ++i)
			for (int j = 0; j < b.cols(); ++j)
				if (b(i, j) != 0)
				{
					values.push_back(b(i, j));
					rows.push_back(i);
					cols.push_back(j);
				}
		file.writeDataset(values, "weight_triplets/values");
		file.writeDataset(rows, "weight_triplets/rows");
		file.writeDataset(cols, "weight_triplets/cols");
		file.writeAttribute(std::array<long, 2>{b.rows(), b.cols()}, "weight_triplets", "shape");
	}
	void external(ipc::CollisionMesh &out, const M &p, const M &b, const mesh::Obstacle &obstacle)
	{
		const std::string prefix = p.rows() > 1 ? "interpolated-proxy" : "obstacle-proxy";
		proxy_files(p, b, prefix);
		auto dummy = mesh::Mesh::create(2);
		Eigen::VectorXi order(b.cols());
		for (int i = 0; i < order.size(); ++i)
			order[i] = i;
		json args = {{"root_path", ""}, {"geometry", json::array({{{"transformation", {{"dimensions", nullptr}, {"scale", 1}, {"rotation", 0}, {"translation", {0, 0}}}}}})}, {"contact", {{"collision_mesh", {{"enabled", true}, {"mesh", prefix + ".obj"}, {"linear_map", prefix + ".h5"}}}}}};
		varform::NonlinearElasticVarForm::build_collision_mesh(*dummy, b.cols() + obstacle.n_vertices(), {}, {}, {}, obstacle, args, [](const std::string &s) { return s; }, order, out);
	}
} // namespace
int main()
{
	logger().set_level(spdlog::level::off);
	json out;
	try
	{
		M p(3, 2);
		p << -1, 0, 1, 0, 0, .2;
		M snodes(4, 2);
		snodes << -1, 0, 7, 8, 1, 0, 0, .2;
		M sb = M::Zero(3, 4);
		sb(0, 0) = sb(1, 2) = sb(2, 3) = 1;
		Eigen::MatrixXi se(1, 2);
		se << 0, 2;
		ipc::CollisionMesh sm(std::vector<bool>{true, false, true, true}, std::vector<bool>(4, false), snodes, se);
		out["surface_selection"] = verify(sm, sb, snodes, V::Zero(8), M::Identity(8, 8));
		M identity = M::Identity(3, 3);
		auto m = tiny(p, identity);
		out["identity"] = verify(m, identity, p, V::Zero(6), M::Identity(6, 6));
		M perm(3, 3);
		perm << 0, 0, 1, 0, 1, 0, 1, 0, 0;
		M pn = perm.transpose() * p;
		auto pm = tiny(p, perm);
		out["permutation"] = verify(pm, perm, pn, V::Zero(6), M::Identity(6, 6));
		V diagonal(6);
		diagonal << 10, 10, 20, 20, 100, 100;
		M h = diagonal.asDiagonal();
		Probe original(m, h), permuted(pm, h);
		original.start(V::Zero(6));
		permuted.start(V::Zero(6));
		V w(6);
		w << 0, -.5, 0, -.5, 0, 1;
		w.normalize();
		M pa = dofmap(perm);
		double expected = (pa.transpose() * w).dot(h * (pa.transpose() * w));
		double actual = permuted.coefficient();
		check(close(original.coefficient(), w.dot(h * w)), "identity curvature control");
		check(close(actual, expected), "permutation curvature matches mapped reference");
		out["permutation_curvature"] = {{"actual", actual}, {"required_by_permutation", expected}, {"identity", original.coefficient()}};
		V separated = V::Zero(6);
		separated[1] = 2.; // proxy contact point selects system node 0
		Probe born(pm, h);
		born.start(separated);
		check(born.collision_set().empty(), "permuted empty refresh snapshot");
		born.solution_changed(V::Zero(6));
		check(close(born.coefficient(), expected), "new contact uses mapped frozen stiffness");
		born.solution_changed(separated);
		born.solution_changed(V::Zero(6));
		check(close(born.coefficient(), expected), "reappearing contact uses mapped cache");
		M b = M::Zero(3, 4);
		b(0, 0) = b(1, 1) = 1;
		b(2, 2) = b(2, 3) = .5;
		M nodes(4, 2);
		nodes << -1, 0, 1, 0, -.2, .2, .2, .2;
		auto im = tiny(p, b);
		out["interpolation"] = verify(im, b, nodes, V::Zero(8), M::Identity(8, 8));
		out["fixed_form_interpolation"] = verify(im, b, nodes, V::Zero(8), M::Identity(8, 8), false);
		mesh::Obstacle empty;
		ipc::CollisionMesh real;
		external(real, p, b, empty);
		check((real.rest_positions() - p).norm() < 1e-12, "real external proxy positions");
		out["external_builder"] = verify(real, b, nodes, V::Zero(8), M::Identity(8, 8));
		V diag(8);
		diag << 10, 10, 20, 20, 100, 100, 400, 400;
		M hf = diag.asDiagonal();
		Probe ip(im, hf);
		ip.start(V::Zero(8));
		M a = dofmap(b), l = a.transpose();
		l.bottomRows(4) *= 2; // Euclidean minimum-norm right inverse.
		check((a * l - M::Identity(6, 6)).norm() < 1e-12, "explicit right inverse");
		M compliance = a * hf.inverse() * a.transpose();
		M energy_min = compliance.inverse();
		// RB-03 decision (2026-09-11): production condenses the parent block
		// onto the stencil (minimum-energy lift); the gap-normalized
		// direction |w|^4 (w^T B H B^T w)/(w^T B B^T w)^2 is its fallback.
		const V force_direction = a.transpose() * w;
		const double transpose_direction = force_direction.dot(hf * force_direction);
		const double gap_normalized_direction = transpose_direction / std::pow(force_direction.squaredNorm(), 2);
		out["interpolation_curvature"] = {{"actual", ip.coefficient()}, {"legacy_vertex_sampling_before_2026_09_11", 215. / 3.}, {"transpose_direction_candidate", transpose_direction}, {"gap_normalized_direction_fallback", gap_normalized_direction}, {"minimum_norm_lift_candidate", (l * w).dot(hf * (l * w))}, {"minimum_energy_lift_candidate", w.dot(energy_min * w)}};
		Probe unit(im, hf, false);
		unit.start(V::Zero(8));
		V unit_g;
		unit.first_derivative(V::Zero(8), unit_g);
		json comparisons = json::object();
		for (const auto &entry : out["interpolation_curvature"].items())
		{
			const double k = entry.value();
			comparisons[entry.key()] = {{"kappa", k}, {"energy", k * unit.value(V::Zero(8))}, {"force_norm", k * unit_g.norm()}};
		}
		out["candidate_force_effects"] = comparisons;
		// Real proxy point averaged from two FEM nodes; obstacle edge appended by production builder.
		M point(1, 2);
		point << 0, .2;
		M average(1, 2);
		average << .5, .5;
		mesh::Obstacle obstacle;
		M edge(2, 2);
		edge << -1, 0, 1, 0;
		Eigen::MatrixXi edges(1, 2);
		edges << 0, 1;
		obstacle.append_mesh(edge, Eigen::VectorXi(), edges, Eigen::MatrixXi(), json{{"value", {0, "0.05*t"}}}, "");
		ipc::CollisionMesh om;
		external(om, point, average, obstacle);
		M ob = M::Zero(3, 4);
		ob(0, 0) = ob(0, 1) = .5;
		ob(1, 2) = ob(2, 3) = 1;
		M onodes(4, 2);
		onodes << -.2, .2, .2, .2, -1, 0, 1, 0;
		M q = M::Zero(8, 4);
		q.topRows(4).setIdentity();
		out["fixed_obstacle"] = verify(om, ob, onodes, V::Zero(8), q);
		M displacement = M::Zero(8, 1);
		obstacle.update_displacement(1, displacement);
		V ox = displacement;
		check(close(ox[5], .05) && close(ox[7], .05) && ox.head(4).norm() == 0, "actual obstacle prescribed motion");
		out["moving_obstacle"] = verify(om, ob, onodes, ox, q);
		StiffnessMatrix mass(8, 8);
		mass.setIdentity();
		auto bc = std::make_shared<BCLagrangianForm>(8, std::vector<int>{4, 5, 6, 7}, mass, 0, ox);
		auto nf = std::make_shared<Probe>(om, 100 * M::Identity(8, 8));
		nf->start(ox);
		NLProblem problem(8, 0, {nf}, {bc}, polysolve::linear::Solver::create(json{{"solver", "Eigen::SimplicialLDLT"}}, logger()), 1, 1, mass, 1);
		problem.use_reduced_size();
		V z = V::Zero(4);
		check((problem.reduced_to_full(z) - ox).norm() < 1e-12, "NLProblem prescribed offsets");
		check(problem.full_to_reduced(ox).norm() < 1e-12, "NLProblem inverse coordinates");
		problem.solution_changed(z);
		V ng;
		problem.gradient(z, ng);
		StiffnessMatrix nh;
		problem.hessian(z, nh);
		V fg;
		nf->first_derivative(ox, fg);
		StiffnessMatrix fullh;
		nf->second_derivative(ox, fullh);
		check((ng - q.transpose() * fg).norm() < 1e-10, "NLProblem gradient projection");
		check((M(nh) - q.transpose() * M(fullh) * q).norm() < 1e-10, "NLProblem Hessian projection");
		V nfd(4);
		M nhfd(4, 4);
		const double ns = 1e-6;
		for (int j = 0; j < 4; ++j)
		{
			V pos = z, neg = z;
			pos[j] += ns;
			neg[j] -= ns;
			problem.solution_changed(pos);
			double pe = problem.value(pos);
			V pg;
			problem.gradient(pos, pg);
			problem.solution_changed(neg);
			double ne = problem.value(neg);
			V mg;
			problem.gradient(neg, mg);
			nfd[j] = (pe - ne) / (2 * ns);
			nhfd.col(j) = (pg - mg) / (2 * ns);
		}
		check((nfd - ng).norm() < 1e-6 * (1 + ng.norm()), "NLProblem reduced energy FD");
		check((nhfd - M(nh)).norm() < 1e-5 * (1 + nh.norm()), "NLProblem reduced gradient FD");
		out["NLProblem_prescribed"] = {{"gradient_relative_error", (nfd - ng).norm() / (1 + ng.norm())}, {"hessian_relative_error", (nhfd - M(nh)).norm() / (1 + nh.norm())}, {"reduced_dofs", problem.reduced_size()}};
		Probe reaction(om, 100 * M::Identity(8, 8));
		reaction.start(ox);
		V rg;
		reaction.first_derivative(ox, rg);
		const double ds = 1e-6;
		reaction.solution_changed(ox + ds * ox);
		double ep = reaction.value(ox + ds * ox);
		reaction.solution_changed(ox - ds * ox);
		double em = reaction.value(ox - ds * ox);
		check(std::abs((ep - em) / (2 * ds) - rg.dot(ox)) < 1e-6 * (1 + std::abs(rg.dot(ox))), "prescribed velocity work derivative");
		out["prescribed_contact_work"] = {{"dE_dt", rg.dot(ox)}, {"finite_difference_dE_dt", (ep - em) / (2 * ds)}, {"fem_force_y", -rg[1] - rg[3]}, {"obstacle_force_y", -rg[5] - rg[7]}};
		Probe obstacle_h(om, 100 * M::Identity(4, 4));
		obstacle_h.start(V::Zero(8));
		out["obstacle_curvature"] = {{"actual_with_fem_only_hessian", obstacle_h.coefficient()}, {"note", "proxy IDs are not FEM/obstacle DOF IDs"}};
		// Exact external selection: proxy point 0 selects FEM node 1;
		// appended obstacle proxies 1,2 select system nodes 2,3. Neither
		// obstacle contributes curvature to this FEM-only Hessian.
		M selector(1, 2);
		selector << 0, 1;
		ipc::CollisionMesh selected_obstacle;
		external(selected_obstacle, point, selector, obstacle);
		V selector_diagonal(4);
		selector_diagonal << 10, 10, 100, 100;
		Probe selected_h(selected_obstacle, M(selector_diagonal.asDiagonal()));
		selected_h.start(V::Zero(8));
		const double selected_k = selected_h.coefficient();
		check(close(selected_k, 100. * 2. / 3.), "external selector obstacle FEM-only curvature");
		selected_h.start(ox);
		check(close(selected_h.coefficient(), selected_k), "external moving obstacle selector curvature");
		// Interpolated stiffness (2026-09-11): the averaged point condenses
		// its two parents, (.25/100 + .25/400)^-1 = 320, and the selector
		// endpoints keep their blocks: (.25*10 + .25*20 + 320)/1.5.
		check(close(ip.coefficient(), w.dot(energy_min * w)), "interpolated contact condenses the parent block (minimum-energy lift)");
		check(close(ip.coefficient(), 327.5 / 1.5), "interpolated contact hand-derived value");
		// Obstacle proxies have no movable parent in a FEM-only Hessian and
		// contribute zero; the averaged FEM point condenses to
		// (.25/100 + .25/100)^-1 = 200 along its unit share 1/1.5 of w.
		check(close(obstacle_h.coefficient(), 400. / 3.), "interpolated obstacle condenses over FEM parents; obstacle rows contribute zero");
		out["obstacle_curvature"]["legacy_before_2026_09_11"] = 250. / 3.;
		// Fallback (i'): two rows on one node cannot prescribe independent
		// motion; the gap-normalized direction reads 9 * (.25*10 + .25*20)/1.5.
		M dup = M::Zero(3, 3);
		dup(0, 0) = dup(1, 1) = dup(2, 0) = 1;
		auto dm = tiny(p, dup);
		Probe dup_h(dm, h);
		dup_h.start(V::Zero(6));
		check(close(dup_h.coefficient(), 45.), "dependent map rows use the gap-normalized direction fallback");
		check(dup_h.diagnostic_state()["interpolated_direction_count"].get<int>() == 1 && dup_h.diagnostic_state()["interpolated_condensed_count"].get<int>() == 0, "dependent rows counted as direction fallback");
		// Fallback (i'): an indefinite parent block cannot be condensed;
		// (.25*10 + .25*20 - .25*100 + .25*400)/1.5 / (2/3)^2.
		V negative(8);
		negative << 10, 10, 20, 20, -100, -100, 400, 400;
		Probe indefinite(im, M(negative.asDiagonal()));
		indefinite.start(V::Zero(8));
		check(close(indefinite.coefficient(), 123.75), "indefinite parent block uses the gap-normalized direction fallback");
		check(ip.diagnostic_state()["interpolated_condensed_count"].get<int>() == 1 && ip.diagnostic_state()["interpolated_direction_count"].get<int>() == 0, "interpolated contact counted as condensed");
		out["interpolation_fallbacks"] = {{"dependent_rows", dup_h.coefficient()}, {"indefinite_parent_block", indefinite.coefficient()}};
		out["exact_selector_obstacle_curvature"] = {{"actual", selected_k}, {"expected", 100. * 2. / 3.}};
		out["checks"] = checks;
		out["status"] = "exact selector indexing repaired; interpolated stiffness = parent block condensed onto the stencil, gap-normalized direction fallback (2026-09-11)";
		std::cout << out.dump(2) << std::endl;
		return 0;
	}
	catch (const std::exception &e)
	{
		out["checks"] = checks;
		out["error"] = e.what();
		std::cout << out.dump(2) << std::endl;
		return 1;
	}
}
