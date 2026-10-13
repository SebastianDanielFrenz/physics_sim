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
    // ---- Stage seam S0: the compressor OUTSIDE its certified coverage ----------------------------------------------------
    // Where compressor_sample() leaves weight on unsupported corners (flow beyond a speed line's choke edge, or a speed
    // below the lowest / above the highest speed line with data), TurboPair evaluates compressor_extended() instead of the
    // authored head/work law. The extension uses only the map's own data and diagnoses:
    //   * along a speed line, valid knots interpolate linearly in flow (as inside coverage); beyond the line's LAST valid
    //     knot (its choke edge) the pressure ratio falls linearly to 1 at the line's FIRST invalid knot - the generator's own
    //     verdict there is "choked / no positive head" - and stays 1 beyond (deep choke: no head, PR never below 1 here);
    //     the specific work is held at the edge value (Euler work does not vanish in choke - the impeller still turns the
    //     flow; the lost head becomes heat in the charge through stage_work/cp). Below the first valid knot (the low-flow
    //     side) both are held flat. A line whose last valid knot is the axis end gets one more knot at the last spacing.
    //   * between speed lines with data, the two line values blend linearly in corrected speed. At or below the smaller of
    //     the two lines' edge flows m the lines are read at the same flow (the bilinear rule of the covered region, so the
    //     value is continuous across the coverage boundary). Beyond m the lines are read CHOKE-ALIGNED: the edge flow and the
    //     choke knot interpolate linearly in speed (e(N), n(N)) and each line is read at the same relative position of its
    //     own interval - [m, e(N)] onto [m, e_i], [e(N), n(N)] onto [e_i, n_i] (the beta-line idea of turbomachinery map
    //     interpolation, applied only outside coverage). Reading both lines at one flow there would put the slower line in
    //     its choke drop while the faster one still makes head, so the blend would choke at the SLOWER line's edge flow at
    //     every speed of the cell (a saw-tooth in the charge a speed-density engine settles at, measured 2026-10-13).
    //   * below the lowest / above the highest line with data, compressor affinity (similar velocity triangles): the line is
    //     read at flow f*N_line/N and the isentropic enthalpy rise PR^kappa - 1 and the work scale with (N/N_line)^2.
    // C0 everywhere in (speed, flow); C1 along flow inside a segment. Never certified: TurboPair flags it as a fallback.
    // Allocation-free; per call it scans at most two speed lines' masks.
    real air_kappa = real(2.0/7.0); // (gamma-1)/gamma of the compressor gas = R/cp; the loader sets it from geometry.air
    struct LineSample {
        real pressure_ratio = 1, work = 0, pressure_slope = 0; // dPR/d(corrected flow)
        bool beyond_edge = false;                               // flow past the line's last valid knot
    };
    bool line_has_data(std::size_t i) const {
        const std::size_t nf = compressor_pr.y.size();
        for (std::size_t j = 0; j < nf; ++j) if (compressor_valid[i*nf+j]) return true;
        return false;
    }
    // A line's choke edge: the flow of its last valid knot and of the knot after it (the first diagnosed invalid one; one
    // spacing further when the axis ends valid). The line must have data.
    struct LineEdge { real edge = 0, next = 0; };
    LineEdge line_edge(std::size_t i) const {
        const auto& fy = compressor_pr.y;
        const std::size_t nf = fy.size();
        std::size_t j1 = 0;
        for (std::size_t j = nf; j-- > 0;) if (compressor_valid[i*nf+j]) { j1 = j; break; }
        LineEdge le;
        le.edge = fy[j1];
        le.next = j1+1 < nf ? fy[j1+1] : fy[j1]+(j1 > 0 ? fy[j1]-fy[j1-1] : real(1));
        return le;
    }
    // One speed line (index i, which must have data) at corrected flow f.
    LineSample line_sample(std::size_t i, real f) const {
        const auto& fy = compressor_pr.y;
        const std::size_t nf = fy.size();
        const auto valid = [&](std::size_t j) { return bool(compressor_valid[i*nf+j]); };
        const auto pr = [&](std::size_t j) { return compressor_pr.values[i*nf+j]; };
        const auto work = [&](std::size_t j) { return compressor_work_j_kg.values[i*nf+j]; };
        LineSample out;
        // lo: the last valid knot at or below f; hi: the first valid knot at or above f.
        const auto b = Table2D::bracket_for(fy, f);
        std::size_t lo = nf, hi = nf;
        for (std::size_t j = b.lo + 1; j-- > 0;) if (valid(j) && fy[j] <= f) { lo = j; break; }
        for (std::size_t j = b.lo; j < nf; ++j) if (valid(j) && fy[j] >= f) { hi = j; break; }
        if (lo < nf && hi < nf) {
            if (hi == lo) { out.pressure_ratio = pr(lo); out.work = work(lo); return out; }
            const real df = fy[hi]-fy[lo], t = (f-fy[lo])/df;
            out.pressure_ratio = pr(lo)+t*(pr(hi)-pr(lo));
            out.work = work(lo)+t*(work(hi)-work(lo));
            out.pressure_slope = (pr(hi)-pr(lo))/df;
            return out;
        }
        if (lo == nf) { // below the line's first valid knot: held flat
            out.pressure_ratio = pr(hi); out.work = work(hi); return out;
        }
        // Beyond the choke edge lo: PR falls to 1 at the next knot (diagnosed invalid), work held.
        out.beyond_edge = true;
        out.work = work(lo);
        const real f_next = lo+1 < nf ? fy[lo+1] : fy[lo]+(lo > 0 ? fy[lo]-fy[lo-1] : real(1));
        const real span = f_next-fy[lo];
        if (f < f_next && span > real(0)) {
            out.pressure_slope = (real(1)-pr(lo))/span;
            out.pressure_ratio = pr(lo)+(f-fy[lo])*out.pressure_slope;
        } else out.pressure_ratio = real(1);
        return out;
    }
    struct ExtendedSample {
        real pressure_ratio = 1, work = 0, pressure_slope = 0; // slope: dPR/d(corrected flow)
        bool beyond_choke = false, speed_outside = false;
    };
    // Line i read at the affinity-similar flow f/r, scaled to speed ratio r = N/N_i (r > 0).
    ExtendedSample affinity_sample(std::size_t i, real f, real r) const {
        ExtendedSample out;
        out.speed_outside = true;
        if (!(r > real(0))) return out; // a resting wheel: PR 1, no work
        const LineSample s = line_sample(i, f/r);
        const real r2 = r*r;
        const real rise = ps::math::pow(std::max(real(1), s.pressure_ratio), air_kappa)-real(1);
        out.pressure_ratio = ps::math::pow(real(1)+rise*r2, real(1)/air_kappa);
        out.work = s.work*r2;
        // dPR'/df = (PR'/PR)^(1-kappa) * r * dPR/df' (chain rule of the enthalpy scaling; 0 where PR was floored at 1)
        out.pressure_slope = s.pressure_ratio > real(1) ?
            ps::math::pow(out.pressure_ratio/s.pressure_ratio, real(1)-air_kappa)*r*s.pressure_slope : real(0);
        out.beyond_choke = s.beyond_edge;
        return out;
    }
    ExtendedSample compressor_extended(real rpm, real flow) const {
        rpm=snap(compressor_pr.x,rpm); flow=snap(compressor_pr.y,flow);
        const auto& nx = compressor_pr.x;
        const std::size_t ns = nx.size();
        // The nearest speed lines with data at or below / at or above rpm.
        std::size_t lo = ns, hi = ns;
        for (std::size_t i = ns; i-- > 0;) if (nx[i] <= rpm && line_has_data(i)) { lo = i; break; }
        for (std::size_t i = 0; i < ns; ++i) if (nx[i] >= rpm && line_has_data(i)) { hi = i; break; }
        ExtendedSample out;
        if (lo == ns && hi == ns) return out; // no data at all
        if (lo == ns) return affinity_sample(hi, flow, rpm/nx[hi]);
        if (hi == ns) return affinity_sample(lo, flow, rpm/nx[lo]);
        const LineSample a = line_sample(lo, flow);
        if (hi == lo) {
            out.pressure_ratio = a.pressure_ratio; out.work = a.work; out.pressure_slope = a.pressure_slope;
            out.beyond_choke = a.beyond_edge; return out;
        }
        const real t = (rpm-nx[lo])/(nx[hi]-nx[lo]);
        const LineEdge ea = line_edge(lo), eb = line_edge(hi);
        const real m = std::min(ea.edge, eb.edge);
        LineSample b;
        if (flow <= m) {
            b = line_sample(hi, flow);
        } else {
            // Choke-aligned reading (see the block comment): fa/fb are the flows the two lines are read at, ga/gb = d f_i/d f.
            const real e = ea.edge+t*(eb.edge-ea.edge), n = ea.next+t*(eb.next-ea.next);
            const auto place = [&](const LineEdge& le, real& g) {
                if (flow <= e) { g = (le.edge-m)/(e-m); return m+(flow-m)*g; } // e > m here: flow > m
                g = (le.next-le.edge)/(n-e);
                return le.edge+(flow-e)*g;
            };
            real ga = 0, gb = 0;
            const real fa = place(ea, ga), fb = place(eb, gb);
            const LineSample a2 = line_sample(lo, fa);
            b = line_sample(hi, fb);
            out.pressure_ratio = a2.pressure_ratio+t*(b.pressure_ratio-a2.pressure_ratio);
            out.work = a2.work+t*(b.work-a2.work);
            out.pressure_slope = (real(1)-t)*a2.pressure_slope*ga+t*b.pressure_slope*gb;
            out.beyond_choke = a2.beyond_edge || b.beyond_edge;
            return out;
        }
        out.pressure_ratio = a.pressure_ratio+t*(b.pressure_ratio-a.pressure_ratio);
        out.work = a.work+t*(b.work-a.work);
        out.pressure_slope = a.pressure_slope+t*(b.pressure_slope-a.pressure_slope);
        out.beyond_choke = a.beyond_edge || b.beyond_edge;
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
