#pragma once
#include "ps/types.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ps::drivetrain {
struct HybridChargePoint { real rpm=0,power_w=0; bool target_met=false,fuel_optimized=false; };

// First establish a feasible power, then minimize fuel per unit crank work.
// Ascending search retains the least-RPM candidate when fuel data is unavailable.
// Fixed bounded sampling is deterministic and allocation-free; no vehicle mutation.
template<class Capacity,class Fuel>
HybridChargePoint select_hybrid_charge_point(real requested,real low,real high,bool optimize,Capacity capacity,Fuel fuel) {
    HybridChargePoint best;
    if(!(requested>0&&low>0&&high>=low))return best;
    constexpr int steps=64;
    real available=0;
    for(int i=0;i<=steps;++i)available=std::max(available,capacity(low+(high-low)*real(i)/steps));
    const real power=std::min(requested,available);
    if(!(power>0))return best;
    real score=std::numeric_limits<real>::infinity();
    for(int i=0;i<=steps;++i){
        const real rpm=low+(high-low)*real(i)/steps;
        if(capacity(rpm)+real(1e-6)<power)continue;
        if(best.rpm==0)best={rpm,power,power+real(1e-6)>=requested,false};
        if(!optimize)break;
        const real f=fuel(rpm,power);
        if(std::isfinite(f)&&f>0&&f/power<score){
            score=f/power;best={rpm,power,power+real(1e-6)>=requested,true};
        }
    }
    return best;
}
}
