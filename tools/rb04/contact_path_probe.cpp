// Reuse the exact RB-04/RB-02 fixture helpers without executing their main.
#define main previous_candidate_main
#include "candidate_probe.cpp"
#undef main
int main()
{
    logger().set_level(spdlog::level::off);
    json out;
    try {
        for (bool fixed : {false,true}) {
            auto m=mesh(); Probe f(m,1,json::object(),1,fixed,70);
            f.driving=10*M::Identity(6,6); f.driving(4,4)=f.driving(5,5)=100;
            V z=V::Zero(6),a=z,b=z; a[4]=.9;b[4]=1.1;f.start(z);
            const auto before=f.state(); const auto path=f.diagnostic_path(a,b);
            check(f.state()==before,"path observer preserves production snapshot");
            double jump=0;
            for(const auto &t:path["transitions"]) jump+=double(t["energy_jump_estimate_objective"]);
            check(fixed ? std::abs(jump)<1e-6 : std::abs(jump+44.497739402978)<1e-6,"known feature jump detector control");
            out[fixed ? "fixed" : "semi"]={{"detected_jump_sum",jump},{"transitions",path["transitions"]},{"quadrature",path["quadrature"]}};
        }
        out["passed"]=true;
    } catch(const std::exception &e) {out["passed"]=false;out["error"]=e.what();}
    out["checks"]=checks;std::cout<<out.dump(2)<<std::endl;
    return out["passed"].get<bool>()?0:1;
}
