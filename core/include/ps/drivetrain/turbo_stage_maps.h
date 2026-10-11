#pragma once
#include "ps/drivetrain/table2d.h"
#include "ps/math/transcendental.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ps::drivetrain {
// Offline mean-line results at reference inlet conditions, PER turbo.
// Coverage is explicit: invalid compressor corners are never interpolated.
struct TurboStageMaps {
    Table2D compressor_pr, compressor_work_j_kg, turbine_flow_kg_s, turbine_work_j_kg, turbine_torque_nm_per_kg_s;
    std::vector<bool> compressor_valid;
    real reference_pa=101325, compressor_reference_k=288.15, turbine_reference_k=1000;
    real compressor_cp=1004.675;
    real exhaust_gamma=1.33, exhaust_r=287.05;
    int units=1;
    std::string source, solver;
    static real snap(const std::vector<real>& axis, real v) {
        for(real knot:axis) if(std::abs(v-knot)<=real(1e-12)*std::max(real(1),std::abs(knot))) return knot;
        return v;
    }

    static bool inside(const Table2D& t, real x, real y) {
        return !t.x.empty() && !t.y.empty() && x>=t.x.front() && x<=t.x.back() && y>=t.y.front() && y<=t.y.back();
    }
    bool compressor_covered(real rpm, real flow) const {
        rpm=snap(compressor_pr.x,rpm); flow=snap(compressor_pr.y,flow);
        if(!inside(compressor_pr,rpm,flow)) return false;
        const auto bx=Table2D::bracket_for(compressor_pr.x,rpm), by=Table2D::bracket_for(compressor_pr.y,flow);
        const real wx[2]={real(1)-bx.t,bx.t}, wy[2]={real(1)-by.t,by.t};
        const std::size_t xs[2]={bx.lo,bx.hi}, ys[2]={by.lo,by.hi};
        for(int i=0;i<2;++i) for(int j=0;j<2;++j)
            if(wx[i]*wy[j]>0 && !compressor_valid[xs[i]*compressor_pr.y.size()+ys[j]]) return false;
        return true;
    }
    struct CompressorSample {
        real weight=0, pressure_ratio=0, work=0, pressure_slope=0, weight_slope=0;
    };
    // Invalid nodes contribute neither values nor weight. The missing weight
    // belongs to the explicitly diagnosed authored fallback in TurboPair.
    // This partition of unity is continuous across mask boundaries; it never
    // invents a simulated value for an unsupported/choked node.
    CompressorSample compressor_sample(real rpm, real flow) const {
        rpm=snap(compressor_pr.x,rpm); flow=snap(compressor_pr.y,flow);
        CompressorSample out;
        if(!inside(compressor_pr,rpm,flow)) return out;
        const auto bx=Table2D::bracket_for(compressor_pr.x,rpm), by=Table2D::bracket_for(compressor_pr.y,flow);
        const real wx[2]={real(1)-bx.t,bx.t}, wy[2]={real(1)-by.t,by.t};
        const real dy=by.lo==by.hi ? 0 : real(1)/(compressor_pr.y[by.hi]-compressor_pr.y[by.lo]);
        const real sy[2]={-dy,dy};
        const std::size_t xs[2]={bx.lo,bx.hi}, ys[2]={by.lo,by.hi};
        for(int i=0;i<2;++i) for(int j=0;j<2;++j) {
            if(!compressor_valid[xs[i]*compressor_pr.y.size()+ys[j]]) continue;
            const real w=wx[i]*wy[j], slope=wx[i]*sy[j];
            out.weight+=w;
            out.pressure_ratio+=w*compressor_pr.at(xs[i],ys[j]);
            out.work+=w*compressor_work_j_kg.at(xs[i],ys[j]);
            out.pressure_slope+=slope*compressor_pr.at(xs[i],ys[j]);
            out.weight_slope+=slope;
        }
        out.weight=std::clamp(out.weight,real(0),real(1));
        return out;
    }
    // Compressible convergent-nozzle flux. Areas include their discharge
    // coefficient. This is also the independent wastegate bypass path.
    static real nozzle_flux(real p, real downstream, real k, real gamma, real r) {
        if(p<=downstream) return 0;
        const real q=std::max(downstream/p,ps::math::pow(real(2)/(gamma+real(1)),gamma/(gamma-real(1))));
        return p/std::sqrt(r*k)*std::sqrt(real(2)*gamma/(gamma-real(1)) *
            (ps::math::pow(q,real(2)/gamma)-ps::math::pow(q,(gamma+real(1))/gamma)));
    }
    real turbine_coordinate(real er) const {
        return std::sqrt(std::max(real(0),real(1)-ps::math::pow(er,-(exhaust_gamma-real(1))/exhaust_gamma)));
    }
    // No RPM/pressure clipping. Outside generated coverage the caller must
    // use an explicitly diagnosed fallback or request an offline extension.
    bool turbine_at(real rpm, real total_flow, real downstream, real k, real bypass_area,
                    real& upstream, real& power, real& starting_torque) const {
        const real nc=rpm/std::sqrt(k/turbine_reference_k);
        if(nc<turbine_flow_kg_s.x.front() || nc>turbine_flow_kg_s.x.back()) return false;
        if(total_flow==real(0)) { upstream=downstream; power=starting_torque=0; return true; }
        const auto flow_at=[&](real er) {
            const real p=downstream*er;
            return units*turbine_flow_kg_s.value(nc,turbine_coordinate(er))*p/reference_pa*std::sqrt(turbine_reference_k/k) +
                bypass_area*nozzle_flux(p,downstream,k,exhaust_gamma,exhaust_r);
        };
        real lo=1, hi=ps::math::pow(real(1)-turbine_flow_kg_s.y.back()*turbine_flow_kg_s.y.back(),-exhaust_gamma/(exhaust_gamma-real(1)));
        if(total_flow<flow_at(lo) || total_flow>flow_at(hi)) return false;
        for(int i=0;i<28;++i) { const real mid=(lo+hi)/2; if(flow_at(mid)<total_flow) lo=mid; else hi=mid; }
        const real er=(lo+hi)/2;
        upstream=downstream*er;
        const real chi=turbine_coordinate(er);
        const real mdot=units*turbine_flow_kg_s.value(nc,chi)*upstream/reference_pa*std::sqrt(turbine_reference_k/k);
        const real torque=mdot*turbine_torque_nm_per_kg_s.value(nc,chi)*std::sqrt(k/turbine_reference_k);
        power=torque*rpm*real(6.2831853071795864769/60);
        starting_torque=mdot*turbine_torque_nm_per_kg_s.value(real(0),chi)*std::sqrt(k/turbine_reference_k);
        return true;
    }
};
}
