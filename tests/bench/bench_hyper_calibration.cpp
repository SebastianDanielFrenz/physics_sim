// Focused held-crank dyno; generated cycle maps, actual MVEM/turbo/fuel physics.
#include "drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <cstdio>
#include <string>
#include <algorithm>
using namespace ps;
using namespace ps::drivetrain;
int main(int argc,char** argv) {
    if(argc<2) return 2;
    jobs::JobSystem pool(10); io::EngineMapOptions options; options.pool=&pool;
    const auto d=std::get<SimulatedEngineDesc>(io::load_engine_json(argv[1],options));
    const auto maps=SimulatedEngine::build_maps(d);
    std::printf("rpm,net_Nm,kW,boost_bar,charge_Pa,intake_Pa,exhaust_Pa,charge_K,turbo_rpm,VE,eta_i,lambda,spark_deg,MBT_deg,eta_spark,comb_Nm,pump_Nm,fric_Nm,coulomb_Nm,fuel_g_s,charge_ripple_Pa\n");
    for(real rpm:{3000.,4000.,4500.,5000.,6000.,7000.,7500.,8000.}) {
        SimulatedEngine e(d,maps); PreSolveResult r; real sum=0,fuel=0,lo=1e9,hi=0;
        const real omega=rpm*6.283185307179586/60, h=1./960;
        for(int i=0;i<14400;++i) {
            Sensors s; s.omega=omega; s.ignition=1;
            r=e.pre_solve(s,{1,0,1},{},h); (void)e.post_solve(omega,h);
            if(i>=13440) { sum+=(r.tau0-maps->c_k)/960; fuel+=r.fuel_flow_g_s/960; lo=std::min(lo,r.p_charge_pa); hi=std::max(hi,r.p_charge_pa); }
        }
        const auto t=e.evaluate_torque(omega,e.p_im(),true,1,false);
        std::printf("%.0f,%.3f,%.3f,%.4f,%.1f,%.1f,%.1f,%.2f,%.0f,%.5f,%.5f,%.4f,%.3f,%.3f,%.5f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f\n",rpm,sum,sum*omega/1000,r.boost_bar,r.p_charge_pa,r.p_intake_pa,r.p_exhaust_pa,r.t_charge_k,r.turbo_rpm,maps->maps.ve.value(rpm,e.p_im()),t.eta_i,t.lambda,t.spark_deg,t.theta_mbt_deg,t.eta_spark,t.tau_comb,t.tau_pump,t.tau_fric,maps->c_k,fuel,hi-lo);
        std::fflush(stdout);
    }
}
