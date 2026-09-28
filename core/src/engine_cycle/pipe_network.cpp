// engine_cycle/pipe_network.cpp - quasi-1D finite-volume pipe core (ps_audio
// A2). See ps/engine_cycle/pipe_network.h for the model, numerics, citations
// and the stability checks.

#include "ps/engine_cycle/pipe_network.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace ps::engine_cycle {

namespace {

constexpr real kPi = 3.14159265358979323846;

// Sutherland's law for air (F. M. White, Viscous Fluid Flow, 3rd ed., 2006
// [A: table 1-2]): mu = mu0 (T/T0)^1.5 (T0 + S)/(T + S). Also used for burned
// gas [A].
constexpr real kSutherlandMu0 = 1.716e-5;  // Pa s
constexpr real kSutherlandT0 = 273.0;      // K
constexpr real kSutherlandS = 111.0;       // K
// Blasius smooth-pipe friction, Fanning form f = 0.0791 Re^-1/4 (Blasius 1913);
// laminar Poiseuille f = 16/Re. tau_w = f rho u|u|/2 = c u with
// c_lam = 8 mu / D, c_turb = (0.0791/2) rho |u| Re^-1/4.
constexpr real kBlasiusHalf = 0.0791 * 0.5;

[[noreturn]] void fail(const char* msg) { throw std::invalid_argument(std::string("PipeNetwork: ") + msg); }

[[noreturn]] void fail(const std::string& msg) { throw std::invalid_argument("PipeNetwork: " + msg); }

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

inline real mix_r(const GasModel& g, real x) { return (1.0 - x) * g.fresh.r + x * g.burned.r; }
inline real mix_cv(const GasModel& g, real t, real x) { return (1.0 - x) * g.fresh.cv(t) + x * g.burned.cv(t); }
inline real mix_e(const GasModel& g, real t, real x) { return (1.0 - x) * g.fresh.u(t) + x * g.burned.u(t); }

// Temperature from the specific sensible internal energy e and burned
// fraction x (gas.h's closed form with m = 1, m_b = x).
inline real temp_from_e(const GasModel& g, real e, real x) { return temperature_from_energy(g, 1.0, x, e); }

struct Flux {
    real m = 0.0, mom = 0.0, e = 0.0, x = 0.0;
};

// Physical flux of a primitive state (per unit area).
inline Flux physical_flux(const GasModel& g, real rho, real u, real p, real x) {
    const real r = mix_r(g, x);
    const real t = p / (rho * r);
    const real e = mix_e(g, t, x);
    const real mdot = rho * u;
    Flux f;
    f.m = mdot;
    f.mom = mdot * u + p;
    f.e = mdot * (e + p / rho + 0.5 * u * u);
    f.x = mdot * x;
    return f;
}

// HLLC (Toro 2009 ch. 10.4, eqs. 10.37-10.39) with Davis wave speeds (ch.
// 10.5.1). Primitive inputs; x is carried as a passive scalar.
inline Flux hllc(const GasModel& g, real rl, real ul, real pl, real xl, real rr, real ur, real pr, real xr) {
    const real rml = mix_r(g, xl);
    const real rmr = mix_r(g, xr);
    const real tl = pl / (rl * rml);
    const real tr = pr / (rr * rmr);
    const real cvl = mix_cv(g, tl, xl);
    const real cvr = mix_cv(g, tr, xr);
    const real al = std::sqrt((cvl + rml) / cvl * pl / rl);
    const real ar = std::sqrt((cvr + rmr) / cvr * pr / rr);
    const real el = rl * (mix_e(g, tl, xl) + 0.5 * ul * ul);  // total energy per unit volume
    const real er = rr * (mix_e(g, tr, xr) + 0.5 * ur * ur);

    const real sl = std::min(ul - al, ur - ar);
    const real sr = std::max(ul + al, ur + ar);

    Flux f;
    if (sl >= 0.0) {
        const real mdot = rl * ul;
        f.m = mdot;
        f.mom = mdot * ul + pl;
        f.e = ul * (el + pl);
        f.x = mdot * xl;
        return f;
    }
    if (sr <= 0.0) {
        const real mdot = rr * ur;
        f.m = mdot;
        f.mom = mdot * ur + pr;
        f.e = ur * (er + pr);
        f.x = mdot * xr;
        return f;
    }
    const real dl = rl * (sl - ul);
    const real dr = rr * (sr - ur);
    const real ss = (pr - pl + ul * dl - ur * dr) / (dl - dr);
    if (ss >= 0.0) {
        // F*_L = F_L + S_L (U*_L - U_L)
        const real c = dl / (sl - ss);  // rho*_L
        const real mdot = rl * ul;
        const real est = c * (el / rl + (ss - ul) * (ss + pl / dl));
        f.m = mdot + sl * (c - rl);
        f.mom = mdot * ul + pl + sl * (c * ss - mdot);
        f.e = ul * (el + pl) + sl * (est - el);
        f.x = mdot * xl + sl * (c * xl - rl * xl);
    } else {
        const real c = dr / (sr - ss);  // rho*_R
        const real mdot = rr * ur;
        const real est = c * (er / rr + (ss - ur) * (ss + pr / dr));
        f.m = mdot + sr * (c - rr);
        f.mom = mdot * ur + pr + sr * (c * ss - mdot);
        f.e = ur * (er + pr) + sr * (est - er);
        f.x = mdot * xr + sr * (c * xr - rr * xr);
    }
    return f;
}

inline real minmod(real a, real b) {
    if (a > 0.0 && b > 0.0) return a < b ? a : b;
    if (a < 0.0 && b < 0.0) return a > b ? a : b;
    return 0.0;
}

// Characteristic pressure-reservoir face (Open, Volume, Port). Interior face
// state (rho, un, p, x) with un along the OUTWARD normal (pipe -> reservoir).
// Returns the flux along the outward normal. Outflow: face pressure = the
// reservoir pressure, reached along the outgoing isentropic characteristic
// (u + 2a/(gamma-1) = const, interior gamma frozen); sonic cap. Inflow: isentropic
// from the reservoir's stagnation state (h0 = the reservoir's static enthalpy,
// a stagnant reservoir), matched to the same outgoing characteristic; sonic cap.
// The inflow energy flux is mdot * h0 exactly, so reservoir and pipe book the
// same energy.
template <class Res>
inline Flux reservoir_face(const GasModel& g, real rho, real un, real p, real x, const Res& res) {
    const real rm = mix_r(g, x);
    const real t = p / (rho * rm);
    const real cv = mix_cv(g, t, x);
    const real gamma = (cv + rm) / cv;
    const real a = std::sqrt(gamma * p / rho);
    if (un >= a) return physical_flux(g, rho, un, p, x);  // supersonic outflow

    const real gm1 = gamma - 1.0;
    const real ratio = res.p / p;
    const real a_ratio = ps::math::pow(ratio, gm1 / (2.0 * gamma));
    real a_s = a * a_ratio;
    real un_s = un + 2.0 * (a - a_s) / gm1;
    if (un_s >= 0.0) {
        real p_s = res.p;
        real rho_s = 0.0;
        if (un_s > a_s) {  // sonic exit
            a_s = gm1 / (gamma + 1.0) * (un + 2.0 * a / gm1);
            un_s = a_s;
            const real q = a_s / a;
            p_s = p * ps::math::pow(q, 2.0 * gamma / gm1);
            rho_s = rho * ps::math::pow(q, 2.0 / gm1);
        } else {
            rho_s = rho * ps::math::pow(ratio, 1.0 / gamma);
        }
        return physical_flux(g, rho_s, un_s, p_s, x);
    }

    // Inflow from the stagnant reservoir.
    const real gc = 0.5 * gm1;
    const real gr = 0.5 * (res.gamma - 1.0);
    const real j = un + a / gc;
    const real a2 = gc * gc + gr;
    const real b = gc * gc * j;
    const real c = gc * gc * j * j - res.a * res.a;
    real disc = b * b - a2 * c;
    if (disc < 0.0) disc = 0.0;
    un_s = (b - std::sqrt(disc)) / a2;
    if (un_s > 0.0) un_s = 0.0;
    real as2 = res.a * res.a - gr * un_s * un_s;
    if (un_s * un_s > as2) {  // choked inflow
        as2 = res.a * res.a / (1.0 + gr);
        un_s = -std::sqrt(as2);
    }
    const real as = std::sqrt(as2);
    const real p_s = res.p * ps::math::pow(as / res.a, 2.0 * res.gamma / (res.gamma - 1.0));
    const real rho_s = res.gamma * p_s / as2;
    const real mdot = rho_s * un_s;
    Flux f;
    f.m = mdot;
    f.mom = mdot * un_s + p_s;
    f.e = mdot * res.h_static;
    f.x = mdot * res.x;
    return f;
}

// Diameter at computational coordinate xc of a pipe with a straight
// extension of length es in front and ee behind a linear taper of length l.
inline real diameter_at(real xc, real es, real l, real d0, real d1) {
    if (xc <= es) return d0;
    if (xc >= es + l) return d1;
    return d0 + (d1 - d0) * (xc - es) / l;
}

// Exact integral of pi/4 d(x)^2 over [xa, xb] when d is linear on it.
inline real frustum_volume(real xa, real xb, real da, real db) {
    return 0.25 * kPi * (xb - xa) * (da * da + da * db + db * db) / 3.0;
}

real cell_volume_exact(real xa, real xb, real es, real l, real d0, real d1) {
    const real bps[2] = {es, es + l};
    real v = 0.0;
    real lo = xa;
    for (real bp : bps) {
        if (bp > lo && bp < xb) {
            v += frustum_volume(lo, bp, diameter_at(lo, es, l, d0, d1), diameter_at(bp, es, l, d0, d1));
            lo = bp;
        }
    }
    v += frustum_volume(lo, xb, diameter_at(lo, es, l, d0, d1), diameter_at(xb, es, l, d0, d1));
    return v;
}

real end_correction_length(EndCorrection c, real d) {
    switch (c) {
        case EndCorrection::Unflanged: return kUnflangedEndCorrection * 0.5 * d;
        case EndCorrection::Flanged: return kFlangedEndCorrection * 0.5 * d;
        case EndCorrection::None: break;
    }
    return 0.0;
}

std::string cfl_message(const std::string& name, real dx, real c, real h, real cfl) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "cell size %.4g m is below the CFL minimum %.4g m (max |u|+a %.4g m/s, h %.4g s, CFL limit %.4g)",
                  dx, c * h / cfl, c, h, cfl);
    return "pipe '" + name + "' " + buf;
}

std::string coupling_message(const char* what, const std::string& name, real v, real load, real h, real cfl) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%.4g m^3 is below the coupling minimum %.4g m^3 (h * sum(A * max |u|+a) / V = %.4g > CFL limit %.4g)",
                  v, h * load / cfl, h * load / v, cfl);
    return std::string(what) + " '" + name + "' " + buf;
}

std::string orifice_message(const std::string& name, real hr, real cfl, real cda) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "is too stiff for h: h * rate = %.4g > CFL limit %.4g (CdA %.4g m^2); enlarge the volumes or shorten h",
                  hr, cfl, cda);
    return "orifice '" + name + "' " + buf;
}

bool gas_state_ok(const GasStateDesc& s) {
    return std::isfinite(s.p_pa) && s.p_pa > 0.0 && std::isfinite(s.t_k) && s.t_k > 0.0 && s.x_b >= 0.0 &&
           s.x_b <= 1.0;
}

inline void fnv_fold(std::uint64_t& h, real v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int k = 0; k < 8; ++k) {
        h ^= (bits >> (8 * k)) & 0xffu;
        h *= 1099511628211ull;
    }
}

} // namespace

PipeNetwork::PipeNetwork(const PipeNetworkDesc& desc, real h) : h_(h) {
    if (!(std::isfinite(h) && h > 0.0)) fail("h must be finite and > 0");
    if (!(desc.cfl_limit > 0.0 && desc.cfl_limit <= 1.0)) fail("cfl_limit must be in (0, 1]");
    if (!gas_state_ok(desc.ambient)) fail("ambient state invalid (p > 0, T > 0, x_b in [0, 1])");
    gas_ = GasModel::from_desc(desc.gas);
    psi_.fresh = PsiClass::make(desc.gas.fresh.gamma_flow);
    psi_.burned = PsiClass::make(desc.gas.burned.gamma_flow);

    const int nv = static_cast<int>(desc.volumes.size());
    const int np = static_cast<int>(desc.ports.size());
    for (const VolumeDesc& v : desc.volumes) {
        if (!(v.volume_m3 > 0.0)) fail("volume '" + v.name + "': volume_m3 must be > 0");
        if (!gas_state_ok(v.initial)) fail("volume '" + v.name + "': initial state invalid");
    }
    for (const PortDesc& p : desc.ports) {
        if (!(p.volume_m3 > 0.0)) fail("port '" + p.name + "': volume_m3 must be > 0");
    }
    std::vector<int> port_uses(static_cast<std::size_t>(np), 0);

    // --- Pipes: geometry and cells ----------------------------------------
    int cells = 0;
    int faces = 0;
    pipes_.reserve(desc.pipes.size());
    for (const PipeDesc& p : desc.pipes) {
        if (!(p.length_m > 0.0 && p.d_start_m > 0.0 && p.d_end_m > 0.0))
            fail("pipe '" + p.name + "': length and diameters must be > 0");
        if (!(p.dx_target_m > 0.0)) fail("pipe '" + p.name + "': dx_target_m must be > 0");
        if (!(p.max_wave_speed_m_s > 0.0)) fail("pipe '" + p.name + "': max_wave_speed_m_s must be > 0");
        if (!gas_state_ok(p.initial)) fail("pipe '" + p.name + "': initial state invalid");
        for (const PipeEndDesc* e : {&p.start, &p.end}) {
            switch (e->kind) {
                case PipeEndDesc::Kind::Closed:
                    if (e->end_correction != EndCorrection::None)
                        fail("pipe '" + p.name + "': a closed end cannot carry an end correction");
                    break;
                case PipeEndDesc::Kind::Open: break;
                case PipeEndDesc::Kind::Volume:
                    if (e->index < 0 || e->index >= nv) fail("pipe '" + p.name + "': volume index out of range");
                    break;
                case PipeEndDesc::Kind::Port:
                    if (e->index < 0 || e->index >= np) fail("pipe '" + p.name + "': port index out of range");
                    ++port_uses[static_cast<std::size_t>(e->index)];
                    break;
            }
        }
        PipeRt rt;
        rt.start = p.start;
        rt.end = p.end;
        rt.wall = p.wall;
        rt.ext_start = end_correction_length(p.start.end_correction, p.d_start_m);
        rt.ext_end = end_correction_length(p.end.end_correction, p.d_end_m);
        rt.l_eff = p.length_m + rt.ext_start + rt.ext_end;
        rt.n = static_cast<int>(std::floor(rt.l_eff / p.dx_target_m));
        if (rt.n < 1) rt.n = 1;
        rt.dx = rt.l_eff / rt.n;
        rt.inv_dx = 1.0 / rt.dx;
        const real courant = h * p.max_wave_speed_m_s / rt.dx;
        if (courant > desc.cfl_limit) {
            fail(cfl_message(p.name, rt.dx, p.max_wave_speed_m_s, h, desc.cfl_limit));
        }
        rt.c0 = cells;
        rt.f0 = faces;
        cells += rt.n;
        faces += rt.n + 1;
        pipes_.push_back(rt);
    }
    for (int k = 0; k < np; ++k) {
        if (port_uses[static_cast<std::size_t>(k)] != 1)
            fail("port '" + desc.ports[static_cast<std::size_t>(k)].name + "' must be connected to exactly one pipe end");
    }

    const auto nc = static_cast<std::size_t>(cells);
    const auto nf = static_cast<std::size_t>(faces);
    for (std::vector<real>* v : {&r_, &ru_, &e_, &rx_, &vol_, &inv_vol_, &diam_, &area_src_, &w_r_, &w_u_, &w_p_,
                                 &w_x_, &w_g_, &ps_, &l_r_, &l_u_, &l_p_, &l_x_, &rr_, &ru2_, &rp_, &rx2_}) {
        v->assign(nc, 0.0);
    }
    for (std::vector<real>* v : {&af_, &fm_, &fmom_, &fe_, &fx_}) v->assign(nf, 0.0);

    for (std::size_t k = 0; k < pipes_.size(); ++k) {
        const PipeRt& rt = pipes_[k];
        const PipeDesc& p = desc.pipes[k];
        for (int f = 0; f <= rt.n; ++f) {
            const real d = diameter_at(f * rt.dx, rt.ext_start, p.length_m, p.d_start_m, p.d_end_m);
            af_[static_cast<std::size_t>(rt.f0 + f)] = 0.25 * kPi * d * d;
        }
        for (int i = 0; i < rt.n; ++i) {
            const auto c = static_cast<std::size_t>(rt.c0 + i);
            const real v = cell_volume_exact(i * rt.dx, (i + 1) * rt.dx, rt.ext_start, p.length_m, p.d_start_m,
                                             p.d_end_m);
            vol_[c] = v;
            inv_vol_[c] = 1.0 / v;
            const real a_mean = v / rt.dx;
            diam_[c] = std::sqrt(4.0 * a_mean / kPi);
            area_src_[c] = (af_[static_cast<std::size_t>(rt.f0 + i + 1)] - af_[static_cast<std::size_t>(rt.f0 + i)]) / v;
        }
    }

    // --- Volumes, ports, orifices, coupling checks ---------------------------
    vols_.resize(static_cast<std::size_t>(nv));
    for (int v = 0; v < nv; ++v) {
        vols_[static_cast<std::size_t>(v)].v = desc.volumes[static_cast<std::size_t>(v)].volume_m3;
        set_volume(v, desc.volumes[static_cast<std::size_t>(v)].initial);
    }
    ports_.resize(static_cast<std::size_t>(np));
    std::vector<real> vol_load(static_cast<std::size_t>(nv), 0.0);   // sum A * c_max per volume
    std::vector<real> port_load(static_cast<std::size_t>(np), 0.0);
    for (std::size_t k = 0; k < pipes_.size(); ++k) {
        const PipeRt& rt = pipes_[k];
        const real c = desc.pipes[k].max_wave_speed_m_s;
        const PipeEndDesc* ends[2] = {&rt.start, &rt.end};
        const real areas[2] = {af_[static_cast<std::size_t>(rt.f0)], af_[static_cast<std::size_t>(rt.f0 + rt.n)]};
        for (int s = 0; s < 2; ++s) {
            if (ends[s]->kind == PipeEndDesc::Kind::Volume)
                vol_load[static_cast<std::size_t>(ends[s]->index)] += areas[s] * c;
            if (ends[s]->kind == PipeEndDesc::Kind::Port) {
                PortRt& pr = ports_[static_cast<std::size_t>(ends[s]->index)];
                pr.pipe = static_cast<int>(k);
                pr.at_start = (s == 0);
                port_load[static_cast<std::size_t>(ends[s]->index)] += areas[s] * c;
            }
        }
    }
    for (int v = 0; v < nv; ++v) {
        const real vv = desc.volumes[static_cast<std::size_t>(v)].volume_m3;
        const real courant = h * vol_load[static_cast<std::size_t>(v)] / vv;
        if (courant > desc.cfl_limit)
            fail(coupling_message("volume", desc.volumes[static_cast<std::size_t>(v)].name, vv,
                                  vol_load[static_cast<std::size_t>(v)], h, desc.cfl_limit));
    }
    for (int k = 0; k < np; ++k) {
        const real vv = desc.ports[static_cast<std::size_t>(k)].volume_m3;
        const real courant = h * port_load[static_cast<std::size_t>(k)] / vv;
        if (courant > desc.cfl_limit)
            fail(coupling_message("port", desc.ports[static_cast<std::size_t>(k)].name, vv,
                                  port_load[static_cast<std::size_t>(k)], h, desc.cfl_limit));
        set_port_state(k, PortState{desc.ambient.p_pa, desc.ambient.t_k, desc.ambient.x_b});
    }
    // Regularised Psi slope at pressure equality (valve_flow.h): the larger of
    // the two gamma classes.
    const real slope = std::max(psi_.fresh.psi_reg, psi_.burned.psi_reg) / (1.0 - kPsiRegularisePr);
    for (const OrificeDesc& o : desc.orifices) {
        if (o.volume_a < 0 || o.volume_a >= nv) fail("orifice '" + o.name + "': volume_a must be a volume index");
        if (o.volume_b < -1 || o.volume_b >= nv || o.volume_b == o.volume_a)
            fail("orifice '" + o.name + "': volume_b must be another volume index or -1 (ambient)");
        if (!(o.cd_area_m2 > 0.0)) fail("orifice '" + o.name + "': cd_area_m2 must be > 0");
        if (!(o.max_wave_speed_m_s > 0.0)) fail("orifice '" + o.name + "': max_wave_speed_m_s must be > 0");
        real inv_v = 1.0 / desc.volumes[static_cast<std::size_t>(o.volume_a)].volume_m3;
        if (o.volume_b >= 0) inv_v += 1.0 / desc.volumes[static_cast<std::size_t>(o.volume_b)].volume_m3;
        const real rate = o.cd_area_m2 * slope * std::sqrt(1.4) * o.max_wave_speed_m_s * inv_v;
        if (h * rate > desc.cfl_limit)
            fail(orifice_message(o.name, h * rate, desc.cfl_limit, o.cd_area_m2));
        orifices_.push_back(OrificeRt{o.volume_a, o.volume_b, o.cd_area_m2});
    }

    // Ambient reservoir view.
    {
        const real x = clamp01(desc.ambient.x_b);
        const real t = desc.ambient.t_k;
        const real r = mix_r(gas_, x);
        const real cv = mix_cv(gas_, t, x);
        ambient_ = Reservoir{desc.ambient.p_pa, t, x, r, (cv + r) / cv, std::sqrt((cv + r) / cv * r * t),
                             mix_e(gas_, t, x) + r * t};
    }

    // Initial pipe states (uniform, at rest).
    for (std::size_t k = 0; k < pipes_.size(); ++k) {
        const GasStateDesc& s = desc.pipes[k].initial;
        for (int i = 0; i < pipes_[k].n; ++i) set_cell(static_cast<int>(k), i, s.p_pa, s.t_k, 0.0, s.x_b);
    }
}

PipeNetwork::Reservoir PipeNetwork::ambient_reservoir() const { return ambient_; }

void PipeNetwork::refresh_reservoir(VolumeRt& v) const {
    const CvState s = cv_state(gas_, v.m, v.u, v.mb, v.v);
    v.p = s.p;
    v.t = s.t;
    v.x = clamp01(s.x);
    v.r = mix_r(gas_, v.x);
    const real cv = mix_cv(gas_, v.t, v.x);
    v.gamma = (cv + v.r) / cv;
    v.a = std::sqrt(v.gamma * v.r * v.t);
    v.h_static = mix_e(gas_, v.t, v.x) + v.r * v.t;
}

void PipeNetwork::refresh_port(PortRt& p) const {
    p.x = clamp01(p.x);
    p.r = mix_r(gas_, p.x);
    const real cv = mix_cv(gas_, p.t, p.x);
    p.gamma = (cv + p.r) / cv;
    p.a = std::sqrt(p.gamma * p.r * p.t);
    p.h_static = mix_e(gas_, p.t, p.x) + p.r * p.t;
}

PipeCellState PipeNetwork::cell(int pipe, int i) const {
    const auto c = static_cast<std::size_t>(pipes_[static_cast<std::size_t>(pipe)].c0 + i);
    PipeCellState s;
    s.rho = r_[c];
    s.u = ru_[c] / s.rho;
    s.x_b = clamp01(rx_[c] / s.rho);
    const real e = e_[c] / s.rho - 0.5 * s.u * s.u;
    s.t = temp_from_e(gas_, e, s.x_b);
    const real rm = mix_r(gas_, s.x_b);
    s.p = s.rho * rm * s.t;
    const real cv = mix_cv(gas_, s.t, s.x_b);
    s.a = std::sqrt((cv + rm) / cv * rm * s.t);
    return s;
}

void PipeNetwork::set_cell(int pipe, int i, real p_pa, real t_k, real u_m_s, real x_b) {
    const auto c = static_cast<std::size_t>(pipes_[static_cast<std::size_t>(pipe)].c0 + i);
    const real x = clamp01(x_b);
    const real rho = p_pa / (mix_r(gas_, x) * t_k);
    r_[c] = rho;
    ru_[c] = rho * u_m_s;
    e_[c] = rho * (mix_e(gas_, t_k, x) + 0.5 * u_m_s * u_m_s);
    rx_[c] = rho * x;
}

CvState PipeNetwork::volume_state(int v) const {
    const VolumeRt& vr = vols_[static_cast<std::size_t>(v)];
    return cv_state(gas_, vr.m, vr.u, vr.mb, vr.v);
}

void PipeNetwork::set_volume(int v, const GasStateDesc& s) {
    VolumeRt& vr = vols_[static_cast<std::size_t>(v)];
    const real x = clamp01(s.x_b);
    vr.m = s.p_pa * vr.v / (mix_r(gas_, x) * s.t_k);
    vr.u = vr.m * mix_e(gas_, s.t_k, x);
    vr.mb = vr.m * x;
}

void PipeNetwork::set_volume_source(int v, real mdot_kg_s, real h_total_j_kg, real x_b) {
    VolumeRt& vr = vols_[static_cast<std::size_t>(v)];
    vr.src_mdot = mdot_kg_s;
    vr.src_h = h_total_j_kg;
    vr.src_x = clamp01(x_b);
}

void PipeNetwork::set_port_state(int port, const PortState& s) {
    PortRt& p = ports_[static_cast<std::size_t>(port)];
    p.p = s.p_pa;
    p.t = s.t_k;
    p.x = s.x_b;
    refresh_port(p);
}

real PipeNetwork::total_mass() const {
    real m = 0.0;
    for (std::size_t c = 0; c < r_.size(); ++c) m += r_[c] * vol_[c];
    for (const VolumeRt& v : vols_) m += v.m;
    return m;
}

real PipeNetwork::total_energy() const {
    real e = 0.0;
    for (std::size_t c = 0; c < e_.size(); ++c) e += e_[c] * vol_[c];
    for (const VolumeRt& v : vols_) e += v.u;
    return e;
}

real PipeNetwork::total_burned_mass() const {
    real m = 0.0;
    for (std::size_t c = 0; c < rx_.size(); ++c) m += rx_[c] * vol_[c];
    for (const VolumeRt& v : vols_) m += v.mb;
    return m;
}

std::uint64_t PipeNetwork::fold_state_hash(std::uint64_t h) const {
    for (std::size_t c = 0; c < r_.size(); ++c) {
        fnv_fold(h, r_[c]);
        fnv_fold(h, ru_[c]);
        fnv_fold(h, e_[c]);
        fnv_fold(h, rx_[c]);
    }
    for (const VolumeRt& v : vols_) {
        fnv_fold(h, v.m);
        fnv_fold(h, v.u);
        fnv_fold(h, v.mb);
    }
    return h;
}

void PipeNetwork::step() {
    const real h = h_;
    const real half_h = 0.5 * h;

    // 0. Start-of-step reservoir views of every volume.
    for (VolumeRt& v : vols_) refresh_reservoir(v);

    int fallbacks = 0;
    for (const PipeRt& pr : pipes_) {
        const int n = pr.n;
        const std::size_t c0 = static_cast<std::size_t>(pr.c0);

        // 1. Primitives and gamma of every cell.
        for (int i = 0; i < n; ++i) {
            const std::size_t c = c0 + static_cast<std::size_t>(i);
            const real rho = r_[c];
            const real u = ru_[c] / rho;
            const real x = clamp01(rx_[c] / rho);
            const real e = e_[c] / rho - 0.5 * u * u;
            const real t = temp_from_e(gas_, e, x);
            const real rm = mix_r(gas_, x);
            const real cv = mix_cv(gas_, t, x);
            w_r_[c] = rho;
            w_u_[c] = u;
            w_p_[c] = rho * rm * t;
            w_x_[c] = x;
            w_g_[c] = (cv + rm) / cv;
        }

        // 2-3. Limited slopes (zero in the end cells) and the Hancock half step.
        for (int i = 0; i < n; ++i) {
            const std::size_t c = c0 + static_cast<std::size_t>(i);
            real dr = 0.0, du = 0.0, dp = 0.0, dxs = 0.0;
            if (i > 0 && i < n - 1) {
                dr = minmod(w_r_[c] - w_r_[c - 1], w_r_[c + 1] - w_r_[c]);
                du = minmod(w_u_[c] - w_u_[c - 1], w_u_[c + 1] - w_u_[c]);
                dp = minmod(w_p_[c] - w_p_[c - 1], w_p_[c + 1] - w_p_[c]);
                dxs = minmod(w_x_[c] - w_x_[c - 1], w_x_[c + 1] - w_x_[c]);
            }
            const real rho = w_r_[c];
            const real u = w_u_[c];
            const real p = w_p_[c];
            const real gp = w_g_[c] * p;
            const real k = half_h * pr.inv_dx;
            const real sa = half_h * u * area_src_[c];
            const real d_r = -k * (u * dr + rho * du) - sa * rho;
            const real d_u = -k * (u * du + dp / rho);
            const real d_p = -k * (u * dp + gp * du) - sa * gp;
            const real d_x = -k * (u * dxs);
            real lr = rho - 0.5 * dr + d_r, rr = rho + 0.5 * dr + d_r;
            real lp = p - 0.5 * dp + d_p, rp = p + 0.5 * dp + d_p;
            if (lr > 0.0 && rr > 0.0 && lp > 0.0 && rp > 0.0) {
                l_r_[c] = lr;
                rr_[c] = rr;
                l_p_[c] = lp;
                rp_[c] = rp;
                l_u_[c] = u - 0.5 * du + d_u;
                ru2_[c] = u + 0.5 * du + d_u;
                l_x_[c] = clamp01(w_x_[c] - 0.5 * dxs + d_x);
                rx2_[c] = clamp01(w_x_[c] + 0.5 * dxs + d_x);
                ps_[c] = p + d_p;
            } else {
                ++fallbacks;
                l_r_[c] = rr_[c] = rho;
                l_u_[c] = ru2_[c] = u;
                l_p_[c] = rp_[c] = p;
                l_x_[c] = rx2_[c] = w_x_[c];
                ps_[c] = p;
            }
        }

        // 4. Interior faces.
        const std::size_t f0 = static_cast<std::size_t>(pr.f0);
        for (int i = 1; i < n; ++i) {
            const std::size_t cl = c0 + static_cast<std::size_t>(i - 1);
            const std::size_t cr = cl + 1;
            const Flux f = hllc(gas_, rr_[cl], ru2_[cl], rp_[cl], rx2_[cl], l_r_[cr], l_u_[cr], l_p_[cr], l_x_[cr]);
            const std::size_t fi = f0 + static_cast<std::size_t>(i);
            fm_[fi] = f.m;
            fmom_[fi] = f.mom;
            fe_[fi] = f.e;
            fx_[fi] = f.x;
        }

        // 5. Boundary faces. Start face: interior state = left face of cell 0,
        // outward normal -x. End face: right face of cell n-1, outward +x.
        for (int s = 0; s < 2; ++s) {
            const PipeEndDesc& end = (s == 0) ? pr.start : pr.end;
            const std::size_t c = (s == 0) ? c0 : c0 + static_cast<std::size_t>(n - 1);
            const std::size_t fi = (s == 0) ? f0 : f0 + static_cast<std::size_t>(n);
            const real rho = (s == 0) ? l_r_[c] : rr_[c];
            const real u = (s == 0) ? l_u_[c] : ru2_[c];
            const real p = (s == 0) ? l_p_[c] : rp_[c];
            const real x = (s == 0) ? l_x_[c] : rx2_[c];
            Flux f;
            if (end.kind == PipeEndDesc::Kind::Closed) {
                // Reflecting wall: mirror state, keep only the pressure (momentum) flux.
                const Flux m = (s == 0) ? hllc(gas_, rho, -u, p, x, rho, u, p, x) : hllc(gas_, rho, u, p, x, rho, -u, p, x);
                f.mom = m.mom;
            } else {
                const real un = (s == 0) ? -u : u;
                Flux fn;
                if (end.kind == PipeEndDesc::Kind::Open) {
                    fn = reservoir_face(gas_, rho, un, p, x, ambient_);
                } else if (end.kind == PipeEndDesc::Kind::Volume) {
                    fn = reservoir_face(gas_, rho, un, p, x, vols_[static_cast<std::size_t>(end.index)]);
                } else {
                    fn = reservoir_face(gas_, rho, un, p, x, ports_[static_cast<std::size_t>(end.index)]);
                }
                if (s == 0) {
                    f.m = -fn.m;
                    f.mom = fn.mom;
                    f.e = -fn.e;
                    f.x = -fn.x;
                } else {
                    f = fn;
                }
            }
            fm_[fi] = f.m;
            fmom_[fi] = f.mom;
            fe_[fi] = f.e;
            fx_[fi] = f.x;

            // Hand the face flux to whatever sits beyond it (into the pipe = +
            // at the start face, - at the end face).
            const real a = af_[fi];
            const real sign = (s == 0) ? 1.0 : -1.0;
            const real dm = sign * h * a * f.m;   // into the pipe across this face
            const real de = sign * h * a * f.e;
            const real dxb = sign * h * a * f.x;
            if (end.kind == PipeEndDesc::Kind::Open) {
                exch_.ambient_mass_kg += dm;
                exch_.ambient_energy_j += de;
                exch_.ambient_burned_kg += dxb;
            } else if (end.kind == PipeEndDesc::Kind::Port) {
                PortRt& port = ports_[static_cast<std::size_t>(end.index)];
                port.flux = PortFlux{dm, de, dxb};
                exch_.port_mass_kg += dm;
                exch_.port_energy_j += de;
                exch_.port_burned_kg += dxb;
            }
        }

        // 6. Conservative update with the well-balanced area source, then wall
        // friction and heat from the start-of-step state.
        for (int i = 0; i < n; ++i) {
            const std::size_t c = c0 + static_cast<std::size_t>(i);
            const std::size_t fl = f0 + static_cast<std::size_t>(i);
            const std::size_t fr = fl + 1;
            const real am = af_[fl];
            const real ap = af_[fr];
            const real k = h * inv_vol_[c];
            const real psrc = ps_[c];
            r_[c] -= k * (ap * fm_[fr] - am * fm_[fl]);
            ru_[c] -= k * (ap * (fmom_[fr] - psrc) - am * (fmom_[fl] - psrc));
            e_[c] -= k * (ap * fe_[fr] - am * fe_[fl]);
            rx_[c] -= k * (ap * fx_[fr] - am * fx_[fl]);
        }
        if (pr.wall.friction == WallFriction::LaminarBlasius || pr.wall.heat_transfer) {
            for (int i = 0; i < n; ++i) {
                const std::size_t c = c0 + static_cast<std::size_t>(i);
                const real rho = w_r_[c];
                const real u = w_u_[c];
                const real x = w_x_[c];
                const real rm = mix_r(gas_, x);
                const real t = w_p_[c] / (rho * rm);
                const real d = diam_[c];
                const real tt = t / kSutherlandT0;
                const real mu = kSutherlandMu0 * tt * std::sqrt(tt) * (kSutherlandT0 + kSutherlandS) / (t + kSutherlandS);
                real coef = 8.0 * mu / d;  // laminar c = tau_w / u
                const real au = std::fabs(u);
                if (au > 0.0) {
                    const real re = rho * au * d / mu;
                    const real ct = kBlasiusHalf * rho * au / std::sqrt(std::sqrt(re));
                    if (ct > coef) coef = ct;
                }
                const real per_vol = 4.0 / d;  // wetted perimeter / area
                if (pr.wall.friction == WallFriction::LaminarBlasius) ru_[c] -= h * per_vol * coef * u;
                if (pr.wall.heat_transfer) {
                    const real cp = mix_cv(gas_, t, x) + rm;
                    e_[c] += h * per_vol * coef * cp * (pr.wall.wall_temperature_k - t);
                }
            }
        }
    }

    // 7. Volumes: pipe-end fluxes (with the same face numbers), orifices,
    // sources. Orifices and sources use the start-of-step reservoir views.
    for (const PipeRt& pr : pipes_) {
        for (int s = 0; s < 2; ++s) {
            const PipeEndDesc& end = (s == 0) ? pr.start : pr.end;
            if (end.kind != PipeEndDesc::Kind::Volume) continue;
            const std::size_t fi = (s == 0) ? static_cast<std::size_t>(pr.f0) : static_cast<std::size_t>(pr.f0 + pr.n);
            const real sign = (s == 0) ? 1.0 : -1.0;  // + = into the pipe = out of the volume
            const real a = af_[fi];
            VolumeRt& v = vols_[static_cast<std::size_t>(end.index)];
            v.m -= sign * h * a * fm_[fi];
            v.u -= sign * h * a * fe_[fi];
            v.mb -= sign * h * a * fx_[fi];
        }
    }
    for (const OrificeRt& o : orifices_) {
        VolumeRt& va = vols_[static_cast<std::size_t>(o.a)];
        const bool amb = o.b < 0;
        const real pb = amb ? ambient_.p : vols_[static_cast<std::size_t>(o.b)].p;
        const real rb = amb ? ambient_.r : vols_[static_cast<std::size_t>(o.b)].r;
        const real tb = amb ? ambient_.t : vols_[static_cast<std::size_t>(o.b)].t;
        const real xb = amb ? ambient_.x : vols_[static_cast<std::size_t>(o.b)].x;
        const real hb = amb ? ambient_.h_static : vols_[static_cast<std::size_t>(o.b)].h_static;
        const OrificeFlow of = orifice_flow(psi_, o.cd_area, va.p, va.r, va.t, va.x, pb, rb, tb, xb);
        const real dm = h * of.mdot;  // a -> b
        const real de = dm * (of.from_1 ? va.h_static : hb);
        const real dxb = dm * (of.from_1 ? va.x : xb);
        va.m -= dm;
        va.u -= de;
        va.mb -= dxb;
        if (amb) {
            exch_.ambient_mass_kg -= dm;
            exch_.ambient_energy_j -= de;
            exch_.ambient_burned_kg -= dxb;
        } else {
            VolumeRt& vb = vols_[static_cast<std::size_t>(o.b)];
            vb.m += dm;
            vb.u += de;
            vb.mb += dxb;
        }
    }
    for (VolumeRt& v : vols_) {
        if (v.src_mdot == 0.0) continue;
        const real dm = h * v.src_mdot;
        const real de = dm * (v.src_mdot > 0.0 ? v.src_h : v.h_static);
        const real dxb = dm * (v.src_mdot > 0.0 ? v.src_x : v.x);
        v.m += dm;
        v.u += de;
        v.mb += dxb;
        exch_.source_mass_kg += dm;
        exch_.source_energy_j += de;
        exch_.source_burned_kg += dxb;
    }

    fallback_count_ = fallbacks;
    ++steps_;
}

} // namespace ps::engine_cycle
