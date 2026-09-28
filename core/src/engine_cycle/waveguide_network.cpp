// engine_cycle/waveguide_network.cpp - linear digital waveguide network
// (ps_audio A3). See ps/engine_cycle/waveguide_network.h for the model,
// numerics, citations and the build-time checks.

#include "ps/engine_cycle/waveguide_network.h"

#include "ps/math/transcendental.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ps::engine_cycle {

namespace {

constexpr real kPi = 3.14159265358979323846;

// Sutherland's law for air, the same constants as pipe_network.cpp (F. M.
// White, Viscous Fluid Flow, 3rd ed., 2006 [A: table 1-2]).
constexpr real kSutherlandMu0 = 1.716e-5;  // Pa s
constexpr real kSutherlandT0 = 273.0;      // K
constexpr real kSutherlandS = 111.0;       // K

[[noreturn]] void fail(const std::string& msg) { throw std::invalid_argument("WaveguideNetwork: " + msg); }

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

bool gas_state_ok(const GasStateDesc& s) {
    return std::isfinite(s.p_pa) && s.p_pa > 0.0 && std::isfinite(s.t_k) && s.t_k > 0.0 && s.x_b >= 0.0 &&
           s.x_b <= 1.0;
}

struct GasPoint {
    real rho = 0.0, c = 0.0, gamma = 1.4, mu = 0.0;
};

GasPoint gas_point(const GasModel& g, const GasStateDesc& s) {
    const real x = clamp01(s.x_b);
    const real t = s.t_k;
    const real r = g.r_mix(x);
    const real cv = (1.0 - x) * g.fresh.cv(t) + x * g.burned.cv(t);
    GasPoint p;
    p.gamma = (cv + r) / cv;
    p.rho = s.p_pa / (r * t);
    p.c = std::sqrt(p.gamma * r * t);
    const real tt = t / kSutherlandT0;
    p.mu = kSutherlandMu0 * tt * std::sqrt(tt) * (kSutherlandT0 + kSutherlandS) / (t + kSutherlandS);
    return p;
}

real end_correction_length(EndCorrection c, real d) {
    switch (c) {
        case EndCorrection::Unflanged: return kUnflangedEndCorrection * 0.5 * d;
        case EndCorrection::Flanged: return kFlangedEndCorrection * 0.5 * d;
        case EndCorrection::None: break;
    }
    return 0.0;
}

inline void fnv_fold(std::uint64_t& h, real v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int k = 0; k < 8; ++k) {
        h ^= (bits >> (8 * k)) & 0xffu;
        h *= 1099511628211ull;
    }
}

// Attenuation [Np] of a unit-DC-gain first-order shelf H = g (1 - q z^-1) /
// (1 - p z^-1), g = (1 - p)/(1 - q), at cos(omega) = cw. Build time only.
real shelf_attenuation(real p, real q, real cw) {
    const real g = (1.0 - p) / (1.0 - q);
    const real num = g * g * (1.0 - 2.0 * q * cw + q * q);
    const real den = 1.0 - 2.0 * p * cw + p * p;
    return -0.5 * ps::math::log(num / den);
}

// Solve the n x n system a x = b in place (Gaussian elimination, partial
// pivoting). n <= 8. Build time only.
void solve_small(real a[8][8], real b[8], int n) {
    for (int k = 0; k < n; ++k) {
        int piv = k;
        for (int i = k + 1; i < n; ++i)
            if (std::fabs(a[i][k]) > std::fabs(a[piv][k])) piv = i;
        if (piv != k) {
            for (int j = 0; j < n; ++j) {
                const real t = a[k][j];
                a[k][j] = a[piv][j];
                a[piv][j] = t;
            }
            const real t = b[k];
            b[k] = b[piv];
            b[piv] = t;
        }
        const real d = a[k][k];
        if (d == 0.0) continue;
        for (int i = k + 1; i < n; ++i) {
            const real f = a[i][k] / d;
            for (int j = k; j < n; ++j) a[i][j] -= f * a[k][j];
            b[i] -= f * b[k];
        }
    }
    for (int k = n - 1; k >= 0; --k) {
        real s = b[k];
        for (int j = k + 1; j < n; ++j) s -= a[k][j] * b[j];
        b[k] = (a[k][k] != 0.0) ? s / a[k][k] : 0.0;
    }
}

// Kirchhoff loss fit: per-pass target attenuation A_t(f) = alpha0 sqrt(f) L.
// kWgLossSections shelves with fixed pole corners F_k (geometric over
// [kWgLossCornerLowHz, min(kWgLossCornerHighHz, 0.45 fs)]) and zero corners
// G_k = F_k exp(h_k), h_k >= 0 fitted by Levenberg-Marquardt on the relative
// error over a 64-point log grid of the fit band. Returns (p_k, q_k) and the
// max relative error on the grid.
static_assert(kWgLossSections >= 2 && kWgLossSections <= 8, "fit_kirchhoff's normal equations are 8x8");
struct LossFit {
    real p[kWgLossSections] = {};
    real q[kWgLossSections] = {};
    real max_rel_err = 0.0;
};

LossFit fit_kirchhoff(real alpha0_l, real fs) {
    constexpr int kK = kWgLossSections;
    constexpr int kGrid = 64;
    const real f_lo = kWgLossFitLowHz;
    const real f_hi = (kWgLossFitHighHz < 0.4 * fs) ? kWgLossFitHighHz : 0.4 * fs;
    real cw[kGrid], target[kGrid];
    const real ratio = f_hi / f_lo;
    for (int j = 0; j < kGrid; ++j) {
        const real f = f_lo * ps::math::pow(ratio, static_cast<real>(j) / (kGrid - 1));
        cw[j] = ps::math::cos(2.0 * kPi * f / fs);
        target[j] = alpha0_l * std::sqrt(f);
    }
    LossFit fit;
    real corner[kK];
    const real c_lo = kWgLossCornerLowHz;
    const real c_hi = (kWgLossCornerHighHz < 0.45 * fs) ? kWgLossCornerHighHz : 0.45 * fs;
    for (int k = 0; k < kK; ++k) {
        corner[k] = c_lo * ps::math::pow(c_hi / c_lo, static_cast<real>(k) / (kK - 1));
        fit.p[k] = ps::math::exp(-2.0 * kPi * corner[k] / fs);
    }
    // Initial heights: a staircase through the target at the corners.
    real hgt[kK];
    {
        real acc = 0.0;
        for (int k = 0; k < kK; ++k) {
            const real t = alpha0_l * std::sqrt(corner[k]);
            real hk = 2.0 * (t - acc);
            if (hk < 0.0) hk = 0.0;
            hgt[k] = hk;
            acc += hk;
        }
    }
    auto zero_of = [&](int k, real hk) { return ps::math::exp(-2.0 * kPi * corner[k] * ps::math::exp(hk) / fs); };
    auto model = [&](const real* hh, int j) {
        real a = 0.0;
        for (int k = 0; k < kK; ++k) a += shelf_attenuation(fit.p[k], zero_of(k, hh[k]), cw[j]);
        return a;
    };
    // Levenberg-Marquardt on the relative error (30 iterations, lambda 1e-3;
    // heights clamped at 0 so every shelf attenuates - passivity).
    for (int it = 0; it < 30; ++it) {
        real jtj[8][8] = {};
        real jtr[8] = {};
        for (int j = 0; j < kGrid; ++j) {
            const real r = (model(hgt, j) - target[j]) / target[j];
            real jac[kK];
            for (int k = 0; k < kK; ++k) {
                real hp[kK], hm[kK];
                for (int m = 0; m < kK; ++m) hp[m] = hm[m] = hgt[m];
                const real dh = 1e-6 + 1e-6 * hgt[k];
                hp[k] += dh;
                hm[k] -= dh;
                jac[k] = (model(hp, j) - model(hm, j)) / (2.0 * dh * target[j]);
            }
            for (int a = 0; a < kK; ++a) {
                jtr[a] -= jac[a] * r;
                for (int b = 0; b < kK; ++b) jtj[a][b] += jac[a] * jac[b];
            }
        }
        real dmax = 0.0;
        for (int a = 0; a < kK; ++a) dmax = jtj[a][a] > dmax ? jtj[a][a] : dmax;
        for (int a = 0; a < kK; ++a) jtj[a][a] += 1e-3 * jtj[a][a] + 1e-12 * dmax;
        solve_small(jtj, jtr, kK);
        for (int k = 0; k < kK; ++k) {
            hgt[k] += jtr[k];
            if (hgt[k] < 0.0) hgt[k] = 0.0;
        }
    }
    for (int k = 0; k < kK; ++k) fit.q[k] = zero_of(k, hgt[k]);
    real worst = 0.0;
    for (int j = 0; j < kGrid; ++j) {
        const real e = std::fabs(model(hgt, j) - target[j]) / target[j];
        if (e > worst) worst = e;
    }
    fit.max_rel_err = worst;
    return fit;
}

std::string delay_message(const std::string& name, const char* dir, real delay, real l_eff, real speed, real h,
                          real filt) {
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "delay %.4g samples (%s) is below the minimum %.4g (L_eff %.4g m, wave speed %.4g m/s, h %.4g s, "
                  "filter delay taken off %.4g samples); lengthen the segment or merge it",
                  delay, dir, kWgMinDelaySamples, l_eff, speed, h, filt);
    return "segment '" + name + "' " + buf;
}

} // namespace

WaveguideNetwork::WaveguideNetwork(const WaveguideNetworkDesc& desc, real h) : h_(h) {
    if (!(std::isfinite(h) && h > 0.0)) fail("h must be finite and > 0");
    gas_ = GasModel::from_desc(desc.gas);
    const real fs = 1.0 / h;
    const int nn = static_cast<int>(desc.nodes.size());
    const int ns = static_cast<int>(desc.segments.size());

    // --- Topology ---------------------------------------------------------
    std::vector<int> degree(static_cast<std::size_t>(nn), 0);
    for (const WgSegmentDesc& s : desc.segments) {
        if (s.node_a < 0 || s.node_a >= nn || s.node_b < 0 || s.node_b >= nn)
            fail("segment '" + s.name + "': node index out of range");
        if (s.node_a == s.node_b) fail("segment '" + s.name + "': both ends on the same node");
        ++degree[static_cast<std::size_t>(s.node_a)];
        ++degree[static_cast<std::size_t>(s.node_b)];
    }
    for (int n = 0; n < nn; ++n) {
        const WgNodeDesc& nd = desc.nodes[static_cast<std::size_t>(n)];
        const int dg = degree[static_cast<std::size_t>(n)];
        switch (nd.kind) {
            case WgNodeDesc::Kind::Junction:
                if (dg < 1) fail("node '" + nd.name + "': a junction needs at least one segment");
                if (!(std::isfinite(nd.volume_m3) && nd.volume_m3 >= 0.0))
                    fail("node '" + nd.name + "': volume_m3 must be >= 0");
                if (nd.volume_m3 > 0.0 && !gas_state_ok(nd.volume_gas))
                    fail("node '" + nd.name + "': volume_gas invalid");
                break;
            case WgNodeDesc::Kind::Closed:
            case WgNodeDesc::Kind::Open:
            case WgNodeDesc::Kind::Anechoic:
                if (dg != 1) fail("node '" + nd.name + "': an end node needs exactly one segment");
                break;
            case WgNodeDesc::Kind::Resistance:
                if (dg != 2) fail("node '" + nd.name + "': a resistance element needs exactly two segments");
                if (!(nd.k_loss >= 0.0 && std::isfinite(nd.k_loss) && nd.r_linear_pa_s_m >= 0.0 &&
                      std::isfinite(nd.r_linear_pa_s_m) && nd.area_m2 >= 0.0 && std::isfinite(nd.area_m2) &&
                      std::isfinite(nd.flow_fraction)))
                    fail("node '" + nd.name + "': k_loss, r_linear_pa_s_m, area_m2 must be finite and >= 0");
                break;
        }
    }

    // --- Segments -----------------------------------------------------------
    segs_.resize(static_cast<std::size_t>(ns));
    for (int s = 0; s < ns; ++s) {
        const WgSegmentDesc& sd = desc.segments[static_cast<std::size_t>(s)];
        if (!(sd.length_m > 0.0 && std::isfinite(sd.length_m) && sd.diameter_m > 0.0 && std::isfinite(sd.diameter_m)))
            fail("segment '" + sd.name + "': length and diameter must be finite and > 0");
        if (!gas_state_ok(sd.gas)) fail("segment '" + sd.name + "': gas state invalid");
        const WgNodeDesc* ends[2] = {&desc.nodes[static_cast<std::size_t>(sd.node_a)],
                                     &desc.nodes[static_cast<std::size_t>(sd.node_b)]};
        const EndCorrection ecs[2] = {sd.end_a, sd.end_b};
        for (int e = 0; e < 2; ++e) {
            if (ends[e]->kind == WgNodeDesc::Kind::Closed && ecs[e] != EndCorrection::None)
                fail("segment '" + sd.name + "': a closed end cannot carry an end correction");
            if (ends[e]->kind == WgNodeDesc::Kind::Open && ends[e]->radiation && ecs[e] == EndCorrection::None)
                fail("segment '" + sd.name + "': a radiating open end needs an unflanged or flanged end correction");
        }
        Seg& g = segs_[static_cast<std::size_t>(s)];
        const GasPoint gp = gas_point(gas_, sd.gas);
        if (!(std::isfinite(sd.mean_velocity_m_s) && std::fabs(sd.mean_velocity_m_s) < gp.c))
            fail("segment '" + sd.name + "': |mean_velocity_m_s| must be below the sound speed");
        g.area = 0.25 * kPi * sd.diameter_m * sd.diameter_m;
        g.rho = gp.rho;
        g.c = gp.c;
        g.mu = gp.mu;
        g.gamma = gp.gamma;
        g.y = g.area / (g.rho * g.c);
        g.z = 1.0 / g.y;
        g.length = sd.length_m;
        g.l_eff = sd.length_m + end_correction_length(sd.end_a, sd.diameter_m) +
                  end_correction_length(sd.end_b, sd.diameter_m);
        g.loss = sd.loss;
        const real radius = 0.5 * sd.diameter_m;
        const real nu = g.mu / g.rho;
        g.alpha0 = std::sqrt(kPi * nu) * (1.0 + (g.gamma - 1.0) / std::sqrt(kWaveguidePrandtl)) / (radius * g.c);
        g.alpha_lam = 16.0 * g.mu / (g.rho * g.c * sd.diameter_m * sd.diameter_m);
    }

    // --- Nodes and ports ----------------------------------------------------
    nodes_.resize(static_cast<std::size_t>(nn));
    {
        int p0 = 0;
        for (int n = 0; n < nn; ++n) {
            nodes_[static_cast<std::size_t>(n)].p0 = p0;
            p0 += degree[static_cast<std::size_t>(n)];
        }
        ports_.resize(static_cast<std::size_t>(p0));
        for (int s = 0; s < ns; ++s) {
            const WgSegmentDesc& sd = desc.segments[static_cast<std::size_t>(s)];
            Node& na = nodes_[static_cast<std::size_t>(sd.node_a)];
            ports_[static_cast<std::size_t>(na.p0 + na.np++)] = Port{2 * s + 1, 2 * s, s};
            Node& nb = nodes_[static_cast<std::size_t>(sd.node_b)];
            ports_[static_cast<std::size_t>(nb.p0 + nb.np++)] = Port{2 * s, 2 * s + 1, s};
        }
    }
    // LaminarFriction: the segment's series resistance R' L = 2 Z alpha L,
    // lumped in series with its ports at Junction/Closed ends (header).
    for (Port& pt : ports_) pt.y = segs_[static_cast<std::size_t>(pt.seg)].y;
    for (int s = 0; s < ns; ++s) {
        const Seg& g = segs_[static_cast<std::size_t>(s)];
        if (g.loss != WgLoss::LaminarFriction) continue;
        const WgSegmentDesc& sd = desc.segments[static_cast<std::size_t>(s)];
        const auto norton = [&desc](int n) {
            const WgNodeDesc::Kind k = desc.nodes[static_cast<std::size_t>(n)].kind;
            return k == WgNodeDesc::Kind::Junction || k == WgNodeDesc::Kind::Closed;
        };
        const int ends = (norton(sd.node_a) ? 1 : 0) + (norton(sd.node_b) ? 1 : 0);
        if (ends == 0)
            fail("segment '" + sd.name + "': laminar friction needs a Junction or Closed node at one end at least");
        const real r = 2.0 * g.z * g.alpha_lam * g.length / static_cast<real>(ends);
        for (Port& pt : ports_) {
            if (pt.seg != s) continue;
            const int node = (pt.arrive == 2 * s) ? sd.node_b : sd.node_a;
            if (!norton(node)) continue;
            pt.r = r;
            pt.y = 1.0 / (g.z + r);
        }
    }
    for (int n = 0; n < nn; ++n) {
        const WgNodeDesc& nd = desc.nodes[static_cast<std::size_t>(n)];
        Node& nr = nodes_[static_cast<std::size_t>(n)];
        nr.kind = nd.kind;
        real ys = 0.0;
        for (int k = 0; k < nr.np; ++k) ys += ports_[static_cast<std::size_t>(nr.p0 + k)].y;
        if (nd.kind == WgNodeDesc::Kind::Junction && nd.volume_m3 > 0.0) {
            const GasPoint vp = gas_point(gas_, nd.volume_gas);
            nr.g_c = 2.0 * nd.volume_m3 / (h * vp.rho * vp.c * vp.c);
        }
        nr.y_sum = ys + nr.g_c;
        if (nd.kind == WgNodeDesc::Kind::Open && nd.radiation) {
            const int s = ports_[static_cast<std::size_t>(nr.p0)].seg;
            const WgSegmentDesc& sd = desc.segments[static_cast<std::size_t>(s)];
            const EndCorrection ec = (sd.node_a == n) ? sd.end_a : sd.end_b;
            const real kk = (ec == EndCorrection::Flanged) ? 1.0 : 0.5;  // |R| ~ 1 - K (ka)^2
            const real beta = fs * 0.5 * sd.diameter_m / segs_[static_cast<std::size_t>(s)].c;
            const real b = 2.0 * kk * beta * beta;  // p / (1 - p)^2
            nr.rad_p = ((2.0 * b + 1.0) - std::sqrt(4.0 * b + 1.0)) / (2.0 * b);
        }
        if (nd.kind == WgNodeDesc::Kind::Resistance) {
            nr.k_loss = nd.k_loss;
            nr.r_lin = nd.r_linear_pa_s_m;
            nr.frac = nd.flow_fraction;
            const Seg& s0 = segs_[static_cast<std::size_t>(ports_[static_cast<std::size_t>(nr.p0)].seg)];
            const Seg& s1 = segs_[static_cast<std::size_t>(ports_[static_cast<std::size_t>(nr.p0 + 1)].seg)];
            nr.area = nd.area_m2 > 0.0 ? nd.area_m2 : (s0.area < s1.area ? s0.area : s1.area);
            nr.rho = s0.rho;
        }
    }

    // --- Directions: delays, loss filters, allpasses ---------------------------
    dirs_.resize(static_cast<std::size_t>(2 * ns));
    int buf_total = 0;
    int shelf_total = 0;
    for (int s = 0; s < ns; ++s) {
        const WgSegmentDesc& sd = desc.segments[static_cast<std::size_t>(s)];
        Seg& g = segs_[static_cast<std::size_t>(s)];
        LossFit fit;
        if (g.loss == WgLoss::Kirchhoff) {
            fit = fit_kirchhoff(g.alpha0 * g.length, fs);
            g.fit_err = fit.max_rel_err;
        }
        for (int dd = 0; dd < 2; ++dd) {
            Dir& d = dirs_[static_cast<std::size_t>(2 * s + dd)];
            d.from = (dd == 0) ? sd.node_a : sd.node_b;
            d.to = (dd == 0) ? sd.node_b : sd.node_a;
            const real speed = (dd == 0) ? g.c + sd.mean_velocity_m_s : g.c - sd.mean_velocity_m_s;
            const Node& to = nodes_[static_cast<std::size_t>(d.to)];
            const real filt = (to.kind == WgNodeDesc::Kind::Open && to.rad_p > 0.0) ? to.rad_p / (1.0 - to.rad_p) : 0.0;
            d.delay = g.l_eff / speed * fs - filt;
            if (!(d.delay >= kWgMinDelaySamples))
                fail(delay_message(sd.name, dd == 0 ? "a->b" : "b->a", d.delay, g.l_eff, speed, h, filt));
            d.n = static_cast<int>(std::floor(d.delay - 0.5));
            const real frac = d.delay - d.n;
            d.ap_a = (1.0 - frac) / (1.0 + frac);
            d.buf0 = buf_total;
            buf_total += d.n;
            d.sh0 = shelf_total;
            if (g.loss == WgLoss::Kirchhoff) {
                d.nsh = kWgLossSections;
                for (int k = 0; k < kWgLossSections; ++k) {
                    const real gg = (1.0 - fit.p[k]) / (1.0 - fit.q[k]);
                    shelves_.push_back(Shelf{gg, -gg * fit.q[k], -fit.p[k]});
                }
            }
            shelf_total += d.nsh;
        }
    }
    buf_.assign(static_cast<std::size_t>(buf_total), 0.0);
    shelf_state_.assign(static_cast<std::size_t>(shelf_total), 0.0);
}

void WaveguideNetwork::set_node_inflow(int node, real q_m3_s) { nodes_[static_cast<std::size_t>(node)].inflow = q_m3_s; }

void WaveguideNetwork::set_node_wave(int node, real p_pa) { nodes_[static_cast<std::size_t>(node)].wave_in = p_pa; }

void WaveguideNetwork::begin_step() {
    for (Dir& d : dirs_) {
        real x = buf_[static_cast<std::size_t>(d.buf0 + d.w)];
        for (int k = 0; k < d.nsh; ++k) {
            const Shelf& sh = shelves_[static_cast<std::size_t>(d.sh0 + k)];
            real& st = shelf_state_[static_cast<std::size_t>(d.sh0 + k)];
            const real y = sh.b0 * x + st;
            st = sh.b1 * x - sh.a1 * y;
            x = y;
        }
        const real y = d.ap_a * x + d.ap_s;
        d.ap_s = x - d.ap_a * y;
        d.arriving = y;
    }
}

WgNorton WaveguideNetwork::junction_norton(int node) const {
    const Node& nr = nodes_[static_cast<std::size_t>(node)];
    WgNorton nt;
    real s = 0.0;
    for (int k = 0; k < nr.np; ++k) {
        const Port& pt = ports_[static_cast<std::size_t>(nr.p0 + k)];
        s += pt.y * dirs_[static_cast<std::size_t>(pt.arrive)].arriving;
    }
    nt.s = 2.0 * s + 2.0 * nr.g_c * nr.s_c;
    nt.y_sum = nr.y_sum;
    return nt;
}

void WaveguideNetwork::finish_step() {
    for (Node& nr : nodes_) {
        const Port* pt = &ports_[static_cast<std::size_t>(nr.p0)];
        switch (nr.kind) {
            case WgNodeDesc::Kind::Junction:
            case WgNodeDesc::Kind::Closed: {
                real s = 0.0;
                for (int k = 0; k < nr.np; ++k) s += pt[k].y * dirs_[static_cast<std::size_t>(pt[k].arrive)].arriving;
                const real pj = (2.0 * s + 2.0 * nr.g_c * nr.s_c + nr.inflow) / nr.y_sum;
                for (int k = 0; k < nr.np; ++k) {
                    const real a = dirs_[static_cast<std::size_t>(pt[k].arrive)].arriving;
                    // Line-side pressure pj + r q with q = y (2a - pj) the flow into the node.
                    const real pl = (pt[k].r > 0.0) ? pj + pt[k].r * pt[k].y * (2.0 * a - pj) : pj;
                    dirs_[static_cast<std::size_t>(pt[k].depart)].departing = pl - a;
                }
                if (nr.g_c > 0.0) nr.s_c = pj - nr.s_c;
                nr.p = pj;
                if (nr.np == 1) nr.arriving = dirs_[static_cast<std::size_t>(pt[0].arrive)].arriving;
                break;
            }
            case WgNodeDesc::Kind::Open: {
                const real y = dirs_[static_cast<std::size_t>(pt[0].arrive)].arriving;
                real x;
                if (nr.rad_p > 0.0) {
                    nr.rad_s = (1.0 - nr.rad_p) * y + nr.rad_p * nr.rad_s;
                    x = -nr.rad_s;
                } else {
                    x = -y;
                }
                dirs_[static_cast<std::size_t>(pt[0].depart)].departing = x;
                nr.arriving = y;
                nr.p = y + x;
                nr.q_out = segs_[static_cast<std::size_t>(pt[0].seg)].y * (y - x);
                break;
            }
            case WgNodeDesc::Kind::Anechoic: {
                const real y = dirs_[static_cast<std::size_t>(pt[0].arrive)].arriving;
                dirs_[static_cast<std::size_t>(pt[0].depart)].departing = nr.wave_in;
                nr.arriving = y;
                nr.p = y + nr.wave_in;
                break;
            }
            case WgNodeDesc::Kind::Resistance: {
                const Seg& s0 = segs_[static_cast<std::size_t>(pt[0].seg)];
                const Seg& s1 = segs_[static_cast<std::size_t>(pt[1].seg)];
                const real u = nr.frac * mdot_mean_ / (nr.rho * nr.area);
                const real r = (nr.r_lin + nr.k_loss * nr.rho * std::fabs(u)) / nr.area;
                const real y0 = dirs_[static_cast<std::size_t>(pt[0].arrive)].arriving;
                const real y1 = dirs_[static_cast<std::size_t>(pt[1].arrive)].arriving;
                const real q = 2.0 * (y0 - y1) / (r + s0.z + s1.z);
                dirs_[static_cast<std::size_t>(pt[0].depart)].departing = y0 - q * s0.z;
                dirs_[static_cast<std::size_t>(pt[1].depart)].departing = y1 + q * s1.z;
                nr.p = 2.0 * y0 - q * s0.z;
                break;
            }
        }
        nr.inflow = 0.0;
        nr.wave_in = 0.0;
    }
    for (Dir& d : dirs_) {
        buf_[static_cast<std::size_t>(d.buf0 + d.w)] = d.departing;
        if (++d.w == d.n) d.w = 0;
    }
}

void WaveguideNetwork::step() {
    begin_step();
    finish_step();
}

real WaveguideNetwork::mean_pressure_drop() const {
    real dp = 0.0;
    for (const Node& nr : nodes_) {
        if (nr.kind != WgNodeDesc::Kind::Resistance) continue;
        const real u = nr.frac * mdot_mean_ / (nr.rho * nr.area);
        dp += nr.r_lin * u + 0.5 * nr.k_loss * nr.rho * u * std::fabs(u);
    }
    return dp;
}

real WaveguideNetwork::linearised_mean_drop() const {
    real dp = 0.0;
    for (const Node& nr : nodes_) {
        if (nr.kind != WgNodeDesc::Kind::Resistance) continue;
        const real u = nr.frac * mdot_mean_ / (nr.rho * nr.area);
        dp += (nr.r_lin + nr.k_loss * nr.rho * std::fabs(u)) * u;
    }
    return dp;
}

real WaveguideNetwork::wave_energy() const {
    real e = 0.0;
    for (std::size_t k = 0; k < dirs_.size(); ++k) {
        const Dir& d = dirs_[k];
        real s = 0.0;
        for (int i = 0; i < d.n; ++i) {
            const real v = buf_[static_cast<std::size_t>(d.buf0 + i)];
            s += v * v;
        }
        s += d.ap_s * d.ap_s / (1.0 - d.ap_a * d.ap_a);
        e += segs_[k / 2].y * s;
    }
    for (const Node& nr : nodes_) e += nr.g_c * nr.s_c * nr.s_c;
    return h_ * e;
}

real WaveguideNetwork::wall_attenuation_target(int seg, real f_hz) const {
    const Seg& g = segs_[static_cast<std::size_t>(seg)];
    switch (g.loss) {
        case WgLoss::Kirchhoff: return g.alpha0 * std::sqrt(f_hz) * g.length;
        case WgLoss::LaminarFriction: return g.alpha_lam * g.length;
        case WgLoss::None: break;
    }
    return 0.0;
}

real WaveguideNetwork::wall_attenuation_filter(int seg, real f_hz) const {
    const Dir& d = dirs_[static_cast<std::size_t>(2 * seg)];
    const real cw = ps::math::cos(2.0 * kPi * f_hz * h_);
    real a = 0.0;
    for (int k = 0; k < d.nsh; ++k) {
        const Shelf& sh = shelves_[static_cast<std::size_t>(d.sh0 + k)];
        const real num = sh.b0 * sh.b0 + sh.b1 * sh.b1 + 2.0 * sh.b0 * sh.b1 * cw;
        const real den = 1.0 + sh.a1 * sh.a1 + 2.0 * sh.a1 * cw;
        a += -0.5 * ps::math::log(num / den);
    }
    return a;
}

std::uint64_t WaveguideNetwork::fold_state_hash(std::uint64_t h) const {
    for (real v : buf_) fnv_fold(h, v);
    for (real v : shelf_state_) fnv_fold(h, v);
    for (const Dir& d : dirs_) {
        fnv_fold(h, d.ap_s);
        fnv_fold(h, static_cast<real>(d.w));
    }
    for (const Node& nr : nodes_) {
        fnv_fold(h, nr.s_c);
        fnv_fold(h, nr.rad_s);
    }
    return h;
}

} // namespace ps::engine_cycle
