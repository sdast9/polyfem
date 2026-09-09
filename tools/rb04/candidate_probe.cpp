// RB-04 candidate fixture. Helpers adapted from the RB-02 coefficient probe.
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
		Probe(const ipc::CollisionMesh &m, double support = 1, json opts = json::object(), double weight = 1, bool fixed = false, double coefficient = 70)
			: BarrierContactForm(m, support, 1, false, false, false, false, false, false,
								 ipc::BroadPhaseMethod::HASH_GRID, 1e-8, 1000000,
								 fixed ? BarrierStiffnessMode::Fixed : BarrierStiffnessMode::SemiImplicit, opts, V::Ones(m.num_vertices()))
		{
			driving = 100 * M::Identity(m.num_vertices() * 2, m.num_vertices() * 2);
			set_weight(weight);
			set_barrier_stiffness(fixed ? coefficient : 1);
			set_system_hessian_provider([this](const V &, StiffnessMatrix &h) {
				++provider_calls;
				h = driving.sparseView();
			});
		}
		void start(const V &x)
		{
			init(x);
			if (uses_semi_implicit_stiffness())
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
    try {
        V z = V::Zero(6);
        for (bool fixed : {false, true}) {
            auto m = mesh();
            Probe f(m, 1, json::object(), 1, fixed, 70);
            f.driving = 10 * M::Identity(6, 6);
            f.driving(4,4) = f.driving(5,5) = 100;
            f.start(z);
            json rows = json::array();
            for (double eps : {1e-3, 1e-5, 1e-7, 1e-9}) {
                V left=z, right=z, gl, gr;
                left[4]=1-eps; right[4]=1+eps;
                f.line_search_begin(z,right);
                auto l=sample(f,left); f.first_derivative(left,gl);
                auto r=sample(f,right); f.first_derivative(right,gr);
                rows.push_back({{"epsilon",eps},{"left",l},{"right",r},
                    {"energy_jump",double(r["energy"])-double(l["energy"])},
                    {"gradient_jump_norm",(gr-gl).norm()}});
                if (!fixed) check(close(l["scales"][0],70) && close(r["scales"][0],55),"RB02 feature reproduction");
                f.line_search_end();
            }
            json fd;
            V x=z; x[4]=.5;
            derivatives(f,x,fd);
            // Midpoint quadrature split at x=1 never samples the boundary itself.
            V a=z,b=z; a[4]=.9;b[4]=1.1;
            f.line_search_begin(z,b);
            f.solution_changed(a); const double ea=f.value(a);
            f.solution_changed(b); const double eb=f.value(b);
            json work=json::array();
            for(int n : {16,64,256,1024}) {
                double integral=0;
                for(int side=0;side<2;++side) for(int i=0;i<n;++i) {
                    V q=z,g; q[4]=.9+.1*side+.1*(i+.5)/n;
                    f.solution_changed(q);f.first_derivative(q,g);
                    integral+=g[4]*.1/n;
                }
                work.push_back({{"panels_per_side",n},{"gradient_integral",integral},
                    {"energy_change",eb-ea},{"energy_minus_integral",eb-ea-integral}});
            }
            f.line_search_end();
            check(fixed ? std::abs(double(work.back()["energy_minus_integral"]))<1e-5
                        : std::abs(double(work.back()["energy_minus_integral"]))>40,
                  "boundary path work limit");
            out[fixed ? "fixed70" : "semi"]={{"transition",rows},{"interior_derivatives",fd},{"path_work",work}};
            if(fixed) check(double(rows.back()["gradient_jump_norm"]) < 1e-4,"fixed limiting gradient continuity");
            else check(std::abs(double(rows.back()["energy_jump"]))>40,"semi nonvanishing energy jump");
        }
        auto m=mesh(); Probe fixed(m,1,json::object(),1,true,100), semi(m);
        fixed.start(z); semi.start(z);
        const auto ref=sample(fixed,z), other=sample(semi,z);
        check(close(ref["energy"],other["energy"]),"uniform curvature energy control");
        check(close(ref["gradient_norm"],other["gradient_norm"]),"uniform curvature force control");
        out["uniform100"]={{"fixed",ref},{"semi",other}};
        for(double L : {1e-3,1.,1e3}) for(double Q : {1e-2,1.,1e2}) {
            auto converted=mesh(L);
            Probe f(converted,L,json::object(),1,true,100*Q/std::pow(L,4));
            f.start(z); auto r=sample(f,z);
            check(close(double(r["energy"])/Q,ref["energy"],1e-9),"fixed converted energy");
            check(close(double(r["gradient_norm"])*L/Q,ref["gradient_norm"],1e-9),"fixed converted force");
            check(close(double(r["hessian_norm"])*L*L/Q,ref["hessian_norm"],1e-9),"fixed converted Hessian");
            r["length_scale"]=L;r["energy_scale"]=Q;out["units"].push_back(r);
        }
        out["passed"]=true;
    } catch(const std::exception &e) {out["passed"]=false;out["error"]=e.what();}
    out["checks"]=checks;
    std::cout<<out.dump(2)<<std::endl;
    return out["passed"].get<bool>() ? 0 : 1;
}
