// tests/unit/test_pipe_network.cpp - ps_audio A2: the quasi-1D finite-volume
// pipe core (ps/engine_cycle/pipe_network.h). out/audio_plan.md section 7 rows
// V5 (Sod shock tube, closed-system conservation) and V3 (FV part: quarter-wave,
// half-wave and Helmholtz resonances with end corrections), plus the port
// hand-off, the CFL rejection, zero allocation, well-balancing and
// determinism. Each case names the defect it catches; the sabotage runs and
// their failing numbers are recorded in out/audio_a2_progress.md.

#include "ps/engine_cycle/pipe_network.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHFull = 1.0 / 46080.0;  // the cycle model's h_full, 1/(960*48) s

drivetrain::GasPropertiesDesc constant_gamma_gas(double gamma, double r = 287.0) {
    drivetrain::GasPropertiesDesc g;
    g.fresh = drivetrain::GasGroupDesc{r / (gamma - 1.0), 0.0, r, gamma};
    g.burned = g.fresh;
    return g;
}

PipeDesc make_pipe(const char* name, double length, double d0, double d1, double dx, PipeEndDesc start,
                   PipeEndDesc end, double c_max = 900.0) {
    PipeDesc p;
    p.name = name;
    p.length_m = length;
    p.d_start_m = d0;
    p.d_end_m = d1;
    p.dx_target_m = dx;
    p.max_wave_speed_m_s = c_max;
    p.start = start;
    p.end = end;
    return p;
}

PipeEndDesc closed_end() { return PipeEndDesc{}; }
PipeEndDesc open_end(EndCorrection c = EndCorrection::Unflanged) {
    return PipeEndDesc{PipeEndDesc::Kind::Open, -1, c};
}
PipeEndDesc volume_end(int v, EndCorrection c = EndCorrection::None) {
    return PipeEndDesc{PipeEndDesc::Kind::Volume, v, c};
}
PipeEndDesc port_end(int p) { return PipeEndDesc{PipeEndDesc::Kind::Port, p, EndCorrection::None}; }

double rel(double a, double b) { return std::fabs(a - b) / std::fabs(b); }

// Isentropic Gaussian pressure pulse (broadband excitation) around xc.
void gaussian_pulse(PipeNetwork& net, int pipe, double p0, double t0, double x_b, double amp, double xc, double w) {
    const double cv = net.gas().fresh.cv(t0) * (1.0 - x_b) + net.gas().burned.cv(t0) * x_b;
    const double r = net.gas().r_mix(x_b);
    const double gamma = (cv + r) / cv;
    for (int i = 0; i < net.cell_count(pipe); ++i) {
        const double s = (net.cell_center_m(pipe, i) - xc) / w;
        const double p = p0 * (1.0 + amp * std::exp(-s * s));
        const double t = t0 * std::pow(p / p0, (gamma - 1.0) / gamma);
        net.set_cell(pipe, i, p, t, 0.0, x_b);
    }
}

// Sound speed of the network's gas at (T, x_b), from its own cv(T).
double sound_speed(const PipeNetwork& net, double t, double x_b) {
    const double cv = net.gas().fresh.cv(t) * (1.0 - x_b) + net.gas().burned.cv(t) * x_b;
    const double r = net.gas().r_mix(x_b);
    return std::sqrt((cv + r) / cv * r * t);
}

// Hann window applied in place (ps_audio A8 V-radiation, mirrors
// test_waveguide_network.cpp's own hann()/mode-decay-rate technique).
std::vector<double> hann(std::vector<double> s) {
    const std::size_t n = s.size();
    for (std::size_t k = 0; k < n; ++k) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(n - 1));
        s[k] *= w;
    }
    return s;
}

// Hann-windowed DFT magnitude of s (sample period dt) at frequency f.
double dft_mag(const std::vector<double>& s, double dt, double f) {
    double re = 0.0, im = 0.0;
    const double w = 2.0 * kPi * f * dt;
    for (std::size_t n = 0; n < s.size(); ++n) {
        re += s[n] * std::cos(w * static_cast<double>(n));
        im -= s[n] * std::sin(w * static_cast<double>(n));
    }
    return std::sqrt(re * re + im * im);
}

// Spectral peak of the (mean-removed, Hann-windowed) signal inside [f_lo, f_hi]:
// a coarse scan, then golden-section refinement around the best bin.
double peak_frequency(std::vector<double> s, double dt, double f_lo, double f_hi) {
    double mean = 0.0;
    for (double v : s) mean += v;
    mean /= static_cast<double>(s.size());
    const std::size_t n = s.size();
    for (std::size_t k = 0; k < n; ++k) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(n - 1));
        s[k] = (s[k] - mean) * w;
    }
    const int scan = 400;
    double best_f = f_lo, best_m = -1.0;
    const double df = (f_hi - f_lo) / scan;
    for (int k = 0; k <= scan; ++k) {
        const double f = f_lo + df * k;
        const double m = dft_mag(s, dt, f);
        if (m > best_m) {
            best_m = m;
            best_f = f;
        }
    }
    double a = best_f - df, b = best_f + df;
    const double gr = 0.5 * (std::sqrt(5.0) - 1.0);
    double c = b - gr * (b - a), d = a + gr * (b - a);
    double mc = dft_mag(s, dt, c), md = dft_mag(s, dt, d);
    for (int it = 0; it < 60; ++it) {
        if (mc > md) {
            b = d;
            d = c;
            md = mc;
            c = b - gr * (b - a);
            mc = dft_mag(s, dt, c);
        } else {
            a = c;
            c = d;
            mc = md;
            d = a + gr * (b - a);
            md = dft_mag(s, dt, d);
        }
    }
    return 0.5 * (a + b);
}

// --- Exact Riemann solver for a calorically perfect gas (Toro 2009 ch. 4) ---
struct Prim1 {
    double rho, u, p;
};

double pressure_fn(double p, const Prim1& s, double g, double& dfdp) {
    const double a = std::sqrt(g * s.p / s.rho);
    if (p > s.p) {  // shock
        const double aa = 2.0 / ((g + 1.0) * s.rho);
        const double bb = (g - 1.0) / (g + 1.0) * s.p;
        const double q = std::sqrt(aa / (p + bb));
        dfdp = q * (1.0 - 0.5 * (p - s.p) / (bb + p));
        return (p - s.p) * q;
    }
    dfdp = 1.0 / (s.rho * a) * std::pow(p / s.p, -(g + 1.0) / (2.0 * g));
    return 2.0 * a / (g - 1.0) * (std::pow(p / s.p, (g - 1.0) / (2.0 * g)) - 1.0);
}

struct RiemannExact {
    Prim1 l, r;
    double g, p_star = 0.0, u_star = 0.0;

    RiemannExact(Prim1 left, Prim1 right, double gamma) : l(left), r(right), g(gamma) {
        double p = 0.5 * (l.p + r.p);
        for (int it = 0; it < 100; ++it) {
            double dl = 0.0, dr = 0.0;
            const double fl = pressure_fn(p, l, g, dl);
            const double fr = pressure_fn(p, r, g, dr);
            const double pn = std::max(1e-12 * p, p - (fl + fr + r.u - l.u) / (dl + dr));
            const bool done = std::fabs(pn - p) < 1e-14 * p;
            p = pn;
            if (done) break;
        }
        p_star = p;
        double d = 0.0;
        u_star = 0.5 * (l.u + r.u) + 0.5 * (pressure_fn(p, r, g, d) - pressure_fn(p, l, g, d));
    }

    Prim1 sample(double s) const {
        const double al = std::sqrt(g * l.p / l.rho);
        const double ar = std::sqrt(g * r.p / r.rho);
        const double gm = (g - 1.0) / (g + 1.0);
        if (s <= u_star) {
            if (p_star > l.p) {
                const double sl = l.u - al * std::sqrt((g + 1.0) / (2.0 * g) * p_star / l.p + (g - 1.0) / (2.0 * g));
                if (s <= sl) return l;
                return Prim1{l.rho * (p_star / l.p + gm) / (gm * p_star / l.p + 1.0), u_star, p_star};
            }
            const double shl = l.u - al;
            if (s <= shl) return l;
            const double asl = al * std::pow(p_star / l.p, (g - 1.0) / (2.0 * g));
            if (s > u_star - asl) return Prim1{l.rho * std::pow(p_star / l.p, 1.0 / g), u_star, p_star};
            const double u = 2.0 / (g + 1.0) * (al + 0.5 * (g - 1.0) * l.u + s);
            const double a = 2.0 / (g + 1.0) * (al + 0.5 * (g - 1.0) * (l.u - s));
            return Prim1{l.rho * std::pow(a / al, 2.0 / (g - 1.0)), u, l.p * std::pow(a / al, 2.0 * g / (g - 1.0))};
        }
        if (p_star > r.p) {
            const double sr = r.u + ar * std::sqrt((g + 1.0) / (2.0 * g) * p_star / r.p + (g - 1.0) / (2.0 * g));
            if (s >= sr) return r;
            return Prim1{r.rho * (p_star / r.p + gm) / (gm * p_star / r.p + 1.0), u_star, p_star};
        }
        const double shr = r.u + ar;
        if (s >= shr) return r;
        const double asr = ar * std::pow(p_star / r.p, (g - 1.0) / (2.0 * g));
        if (s < u_star + asr) return Prim1{r.rho * std::pow(p_star / r.p, 1.0 / g), u_star, p_star};
        const double u = 2.0 / (g + 1.0) * (-ar + 0.5 * (g - 1.0) * r.u + s);
        const double a = 2.0 / (g + 1.0) * (ar - 0.5 * (g - 1.0) * (r.u - s));
        return Prim1{r.rho * std::pow(a / ar, 2.0 / (g - 1.0)), u, r.p * std::pow(a / ar, 2.0 * g / (g - 1.0))};
    }
};

struct SodResult {
    double l1_rho = 0.0, l1_u = 0.0, l1_p = 0.0, l1_x = 0.0;
    double rho_max = 0.0, rho_min = 1e30;
    int fallbacks = 0;
};

// Sod's shock tube (Sod 1978; Toro 2009 Test 1) in SI units: rho 1 / 0.125
// kg/m^3, p 1e5 / 1e4 Pa, gamma 1.4, R 287, a 1 m closed tube split at 0.5 m,
// run to t = 0.2 * 1 m / sqrt(1e5 / 1) s (the nondimensional t = 0.2). The
// burned fraction marks the left gas (1) and the right gas (0): its exact
// profile is a step at the contact.
SodResult run_sod(int cells) {
    PipeNetworkDesc d;
    d.gas = constant_gamma_gas(1.4);
    PipeDesc p = make_pipe("sod", 1.0, 0.05, 0.05, 0.9999 / cells, closed_end(), closed_end(), 1000.0);
    p.wall.friction = WallFriction::None;
    d.pipes.push_back(p);
    const double t_end = 0.2 / std::sqrt(1.0e5);
    const int steps = cells;  // Courant ~0.4 at the fastest wave
    PipeNetwork net(d, t_end / steps);
    REQUIRE(net.cell_count(0) == cells);
    for (int i = 0; i < cells; ++i) {
        const bool left = net.cell_center_m(0, i) < 0.5;
        const double rho = left ? 1.0 : 0.125;
        const double pr = left ? 1.0e5 : 1.0e4;
        net.set_cell(0, i, pr, pr / (287.0 * rho), 0.0, left ? 1.0 : 0.0);
    }
    SodResult res;
    for (int s = 0; s < steps; ++s) {
        net.step();
        res.fallbacks += net.last_step_fallback_count();
    }
    const RiemannExact ex(Prim1{1.0, 0.0, 1.0e5}, Prim1{0.125, 0.0, 1.0e4}, 1.4);
    for (int i = 0; i < cells; ++i) {
        const double x = net.cell_center_m(0, i);
        const Prim1 w = ex.sample((x - 0.5) / t_end);
        const PipeCellState c = net.cell(0, i);
        res.l1_rho += std::fabs(c.rho - w.rho);
        res.l1_u += std::fabs(c.u - w.u);
        res.l1_p += std::fabs(c.p - w.p);
        const double x_ex = (x - 0.5) / t_end < ex.u_star ? 1.0 : 0.0;
        res.l1_x += std::fabs(c.x_b - x_ex);
        res.rho_max = std::max(res.rho_max, c.rho);
        res.rho_min = std::min(res.rho_min, c.rho);
    }
    // Normalised: mean absolute error over the tube, relative to the left state.
    res.l1_rho /= cells * 1.0;
    res.l1_u /= cells * std::sqrt(1.0e5);
    res.l1_p /= cells * 1.0e5;
    res.l1_x /= cells;
    return res;
}

// Pressure at a probe, recorded every `stride` steps.
struct Probe {
    int pipe = 0;
    int cell = 0;
    int volume = -1;  // >= 0: probe a volume instead
};

std::vector<double> record(PipeNetwork& net, const Probe& pr, double duration, int stride) {
    const int steps = static_cast<int>(duration / net.h());
    std::vector<double> s;
    s.reserve(static_cast<std::size_t>(steps / stride + 1));
    for (int k = 0; k < steps; ++k) {
        net.step();
        if (k % stride == 0) s.push_back(pr.volume >= 0 ? net.volume_state(pr.volume).p : net.cell(pr.pipe, pr.cell).p);
    }
    return s;
}

// An I4-like audio-grade network: plenum -> 4 runners -> 4 intake ports; 4
// exhaust ports -> 4 primaries -> collector volume -> collector pipe -> open.
// Geometry ASSUMED (audio_plan.md section 5's cell-count example).
PipeNetworkDesc i4_like_network() {
    PipeNetworkDesc d;
    VolumeDesc plenum{"plenum", 2.5e-3, GasStateDesc{60.0e3, 300.0, 0.0}};
    VolumeDesc airbox{"airbox", 10.0e-3, GasStateDesc{101325.0, 293.15, 0.0}};
    VolumeDesc coll{"collector", 5.0e-4, GasStateDesc{110.0e3, 800.0, 1.0}};
    d.volumes = {plenum, airbox, coll};
    d.orifices.push_back(OrificeDesc{"throttle", 0, 1, 2.0e-4, 500.0});
    d.orifices.push_back(OrificeDesc{"airbox_inlet", 1, -1, 3.0e-3, 500.0});
    for (int k = 0; k < 8; ++k) d.ports.push_back(PortDesc{"port" + std::to_string(k), k < 4 ? 1.5e-4 : 2.5e-4});
    for (int k = 0; k < 4; ++k) {
        PipeDesc r = make_pipe("runner", 0.35, 0.045, 0.040, 0.017, volume_end(0, EndCorrection::Flanged), port_end(k), 500.0);
        r.initial = GasStateDesc{60.0e3, 300.0, 0.0};
        d.pipes.push_back(r);
    }
    for (int k = 0; k < 4; ++k) {
        PipeDesc e = make_pipe("primary", 0.6, 0.036, 0.036, 0.032, port_end(4 + k), volume_end(2), 900.0);
        e.initial = GasStateDesc{110.0e3, 800.0, 1.0};
        e.wall.heat_transfer = true;
        e.wall.wall_temperature_k = 500.0;
        d.pipes.push_back(e);
    }
    PipeDesc c = make_pipe("collector", 0.3, 0.05, 0.05, 0.032, volume_end(2), open_end(), 900.0);
    c.initial = GasStateDesc{105.0e3, 750.0, 1.0};
    d.pipes.push_back(c);
    return d;
}

// Pulsing port states for the I4-like network (crank-angle-like phasing).
void drive_ports(PipeNetwork& net, std::uint64_t step) {
    const double t = static_cast<double>(step) * net.h();
    for (int k = 0; k < 4; ++k) {
        const double ph = 2.0 * kPi * (50.0 * t + 0.25 * k);
        net.set_port_state(k, PortState{60.0e3 + 20.0e3 * std::sin(ph), 310.0, 0.02});
        net.set_port_state(4 + k, PortState{130.0e3 + 120.0e3 * std::max(0.0, std::sin(ph)), 1000.0, 1.0});
    }
}

} // namespace

// ---------------------------------------------------------------------------

TEST_CASE("P1 pipe Sod shock tube vs the exact Riemann solution (V5)", "[drivetrain][cycle][pipe]") {
    // Catches: a wrong flux (sign, HLLC star state), a broken limiter or half
    // step, a passive-scalar transport bug. Bounds: 200 cells on 1 m, t = 0.2
    // (nondimensional), mean |error| over the tube relative to the left state.
    const RiemannExact ex(Prim1{1.0, 0.0, 1.0e5}, Prim1{0.125, 0.0, 1.0e4}, 1.4);
    // The reference itself: Toro 2009 Table 4.3, Test 1: p* 0.30313, u* 0.92745.
    CHECK(std::fabs(ex.p_star / 1.0e5 - 0.30313) < 1e-5);
    CHECK(std::fabs(ex.u_star / std::sqrt(1.0e5) - 0.92745) < 1e-5);

    const SodResult r = run_sod(200);
    std::printf("P1 Sod N=200: L1 rho %.5f u %.5f p %.5f x %.5f  rho range [%.6f, %.6f]  fallbacks %d\n", r.l1_rho,
                r.l1_u, r.l1_p, r.l1_x, r.rho_min, r.rho_max, r.fallbacks);
    CHECK(r.l1_rho < 0.0050);
    CHECK(r.l1_u < 0.0085);
    CHECK(r.l1_p < 0.0037);
    CHECK(r.l1_x < 0.0145);
    // No new extrema (a limiter that does not limit overshoots at the shock).
    CHECK(r.rho_max < 1.0 + 1e-6);
    CHECK(r.rho_min > 0.125 - 1e-6);
    CHECK(r.fallbacks == 0);
    // Converges: halving dx lowers the error.
    const SodResult r2 = run_sod(400);
    std::printf("P1 Sod N=400: L1 rho %.5f\n", r2.l1_rho);
    CHECK(r2.l1_rho < 0.7 * r.l1_rho);
}

TEST_CASE("P2 pipe closed network conserves mass, energy and burned mass (V5)", "[drivetrain][cycle][pipe]") {
    // Catches: a non-conservative junction or volume hand-off, a broken flux
    // telescoping, an orifice that books different amounts on its two sides.
    // Two tapered pipes into a junction volume, a third pipe to a second volume,
    // an orifice between the two volumes; every outer end closed. Friction ON
    // (it converts kinetic into internal energy and must not change E), heat
    // transfer OFF (the only energy exchange with the outside).
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"junction", 2.0e-4, GasStateDesc{150.0e3, 900.0, 1.0}});
    d.volumes.push_back(VolumeDesc{"chamber", 3.0e-3, GasStateDesc{100.0e3, 500.0, 0.0}});
    d.orifices.push_back(OrificeDesc{"link", 0, 1, 1.0e-4, 900.0});
    PipeDesc a = make_pipe("a", 0.5, 0.030, 0.042, 0.025, closed_end(), volume_end(0));
    a.initial = GasStateDesc{300.0e3, 1100.0, 1.0};
    PipeDesc b = make_pipe("b", 0.4, 0.040, 0.034, 0.03, volume_end(0), closed_end());
    b.initial = GasStateDesc{120.0e3, 700.0, 0.3};
    PipeDesc c = make_pipe("c", 0.7, 0.050, 0.050, 0.025, volume_end(0), volume_end(1, EndCorrection::Flanged));
    c.initial = GasStateDesc{90.0e3, 450.0, 0.0};
    d.pipes = {a, b, c};
    PipeNetwork net(d, kHFull);
    gaussian_pulse(net, 2, 90.0e3, 450.0, 0.0, 0.5, 0.35, 0.05);
    const double m0 = net.total_mass();
    const double e0 = net.total_energy();
    const double b0 = net.total_burned_mass();
    const int steps = 20000;
    double max_speed = 0.0;
    for (int s = 0; s < steps; ++s) {
        net.step();
        if (s % 97 == 0) {
            for (int p = 0; p < net.pipe_count(); ++p)
                for (int i = 0; i < net.cell_count(p); ++i) max_speed = std::max(max_speed, std::fabs(net.cell(p, i).u));
        }
    }
    const double dm = (net.total_mass() - m0) / m0;
    const double de = (net.total_energy() - e0) / e0;
    const double db = (net.total_burned_mass() - b0) / b0;
    std::printf("P2 closed network %d steps: rel drift mass %.3e energy %.3e burned %.3e, max |u| %.1f m/s\n", steps, dm,
                de, db, max_speed);
    CHECK(max_speed > 50.0);  // the run actually moved gas through the junction
    CHECK(std::fabs(dm) < 1e-12);
    CHECK(std::fabs(de) < 1e-12);
    CHECK(std::fabs(db) < 1e-12);
    CHECK(net.exchange().ambient_mass_kg == 0.0);
}

TEST_CASE("P3 pipe well-balanced taper: a quiescent tapered pipe stays at rest", "[drivetrain][cycle][pipe]") {
    // Catches: an area source inconsistent with the face pressure fluxes (a
    // dropped or mis-weighted p dA/dx term drives a spurious flow).
    PipeNetworkDesc d;
    PipeDesc p = make_pipe("taper", 0.6, 0.02, 0.06, 0.017, closed_end(), closed_end(), 500.0);
    d.pipes.push_back(p);
    PipeNetwork net(d, kHFull);
    for (int s = 0; s < 10000; ++s) net.step();
    double umax = 0.0;
    for (int i = 0; i < net.cell_count(0); ++i) umax = std::max(umax, std::fabs(net.cell(0, i).u));
    std::printf("P3 taper at rest: max |u| %.3e m/s after 10000 steps\n", umax);
    CHECK(umax < 1e-9);
}

TEST_CASE("P4 pipe resonances vs analytic quarter-wave, half-wave and Helmholtz (V3 FV)", "[drivetrain][cycle][pipe]") {
    // Catches: a missing or wrong end correction, a wrong sound speed, a
    // reservoir boundary that is not a pressure release, a volume boundary
    // with the wrong compliance. Tolerance 2 % at audio cell sizes, h_full.

    SECTION("closed-open quarter wave, intake grade dx 17 mm, 293 K air") {
        PipeNetworkDesc d;
        const double length = 0.40, dia = 0.05;
        d.pipes.push_back(make_pipe("qw", length, dia, dia, 0.017, closed_end(), open_end(), 500.0));
        PipeNetwork net(d, kHFull);
        gaussian_pulse(net, 0, 101325.0, 293.15, 0.0, 0.01, 0.05, 0.04);
        const int stride = 4;
        const std::vector<double> s = record(net, Probe{0, 0, -1}, 0.25, stride);
        const double c = sound_speed(net, 293.15, 0.0);
        const double f_ex = c / (4.0 * (length + kUnflangedEndCorrection * 0.5 * dia));
        const double f = peak_frequency(s, stride * net.h(), 0.8 * f_ex, 1.2 * f_ex);
        std::printf("P4 closed-open: f %.3f Hz vs c/(4(L+0.6133a)) %.3f Hz (%+.3f %%), cells %d dx %.4f\n", f, f_ex,
                    100.0 * (f / f_ex - 1.0), net.cell_count(0), net.cell_dx(0));
        CHECK(rel(f, f_ex) < 0.02);
    }
    SECTION("open-open half wave, exhaust grade dx 32 mm, 800 K burned gas") {
        PipeNetworkDesc d;
        const double length = 0.60, dia = 0.05;
        d.ambient = GasStateDesc{101325.0, 800.0, 1.0};
        PipeDesc p = make_pipe("hw", length, dia, dia, 0.032, open_end(), open_end(), 900.0);
        p.initial = GasStateDesc{101325.0, 800.0, 1.0};
        p.wall.heat_transfer = false;
        d.pipes.push_back(p);
        PipeNetwork net(d, kHFull);
        gaussian_pulse(net, 0, 101325.0, 800.0, 1.0, 0.01, 0.25 * net.effective_length_m(0), 0.064);
        const int stride = 4;
        const std::vector<double> s = record(net, Probe{0, net.cell_count(0) / 2, -1}, 0.25, stride);
        const double c = sound_speed(net, 800.0, 1.0);
        const double f_ex = c / (2.0 * (length + 2.0 * kUnflangedEndCorrection * 0.5 * dia));
        const double f = peak_frequency(s, stride * net.h(), 0.8 * f_ex, 1.2 * f_ex);
        std::printf("P4 open-open: f %.3f Hz vs c/(2(L+1.2266a)) %.3f Hz (%+.3f %%), cells %d dx %.4f\n", f, f_ex,
                    100.0 * (f / f_ex - 1.0), net.cell_count(0), net.cell_dx(0));
        CHECK(rel(f, f_ex) < 0.02);
    }
    SECTION("Helmholtz resonator: neck (flanged into the cavity, unflanged outside) on a 5 L volume") {
        PipeNetworkDesc d;
        const double length = 0.10, dia = 0.04, vol = 5.0e-3;
        d.volumes.push_back(VolumeDesc{"cavity", vol, GasStateDesc{1.01 * 101325.0, 293.15, 0.0}});
        d.pipes.push_back(make_pipe("neck", length, dia, dia, 0.017, volume_end(0, EndCorrection::Flanged), open_end(), 500.0));
        PipeNetwork net(d, kHFull);
        const int stride = 4;
        const std::vector<double> s = record(net, Probe{0, 0, 0}, 0.5, stride);
        const double c = sound_speed(net, 293.15, 0.0);
        const double area = 0.25 * kPi * dia * dia;
        const double l_eff = length + (kUnflangedEndCorrection + kFlangedEndCorrection) * 0.5 * dia;
        const double f_lumped = c / (2.0 * kPi) * std::sqrt(area / (vol * l_eff));
        // Distributed neck (a transmission line with a pressure-release far end
        // loaded by the cavity compliance): k L tan(k L) = A L / V.
        double lo = 0.0, hi = 1.5;
        for (int it = 0; it < 200; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (mid * std::tan(mid) < area * l_eff / vol) lo = mid;
            else hi = mid;
        }
        const double f_ex = 0.5 * (lo + hi) * c / (2.0 * kPi * l_eff);
        const double f = peak_frequency(s, stride * net.h(), 0.7 * f_ex, 1.3 * f_ex);
        std::printf("P4 Helmholtz: f %.3f Hz vs distributed %.3f Hz (%+.3f %%), lumped (c/2pi)sqrt(A/(V L_eff)) %.3f Hz (%+.3f %%)\n",
                    f, f_ex, 100.0 * (f / f_ex - 1.0), f_lumped, 100.0 * (f / f_lumped - 1.0));
        CHECK(rel(f, f_ex) < 0.02);
        CHECK(rel(f, f_lumped) < 0.02);
    }
}

TEST_CASE("V-radiation W1: the FV one-pole radiation filter's pole matches the Levine-Schwinger closed form",
          "[drivetrain][cycle][pipe][audio]") {
    // White-box: pipe_network.cpp builds each radiating end's pole (at
    // construction, `rad_p_start`/`rad_p_end`) from
    //   beta = (1/h) * 0.5*d / a_amb, b = 2*K*beta^2,
    //   pole = ((2b+1) - sqrt(4b+1)) / (2b),
    // K = 1 flanged, 0.5 unflanged - the SAME formula
    // test_waveguide_network.cpp's W2 validates end to end (a simulated
    // pulse-echo measurement matching |R| = 1 - K(ka)^2, H. Levine, J.
    // Schwinger, Phys. Rev. 73, 383 (1948)) for WaveguideNetwork's own Open
    // node. This test re-derives the SAME closed form independently here and
    // cross-checks PipeNetwork::radiation_pole() against it directly, for
    // several diameter/end-correction/timestep combinations - catching a
    // wrong end used (d_start vs d_end), a flanged/unflanged K mixup, the
    // wrong ambient sound speed, an h/fs inversion, or the flag not gating
    // construction at all (pole 0 when radiation is requested). A full
    // simulated pulse-echo re-derivation of |R| itself (W2's own approach)
    // is not attempted on the FV network: V-radiation W2 below measures the
    // qualitative simulated EFFECT instead, because the explicit
    // conservative FV scheme has real, non-negligible truncation-driven
    // numerical dissipation at audio-grade dx (test_pipe_waveguide.cpp's own
    // "[.][diag] FV numerical dissipation" case: a comparable pulse loses
    // 8-21 % to pure scheme dissipation over 1-3 ms) that swamps the tiny
    // (order 1 %, since |R| = 1 - K(ka)^2 with ka << 1) per-bounce physical
    // loss this fix models - a tight quantitative match is not achievable
    // from FV simulation alone at these resolutions.
    struct Case {
        double h, d, a, length;
        EndCorrection ec;
    };
    const std::vector<Case> cases = {
        {kHFull, 0.05, 343.0, 0.40, EndCorrection::Unflanged},
        {kHFull, 0.05, 343.0, 0.40, EndCorrection::Flanged},
        {kHFull, 0.08, 343.0, 0.60, EndCorrection::Unflanged},
        {kHFull / 2.0, 0.036, 400.0, 0.60, EndCorrection::Flanged},
    };
    for (const Case& c : cases) {
        PipeDesc p = make_pipe("rad", c.length, c.d, c.d, c.d < 0.06 ? 0.017 : 0.032,
                                closed_end(), PipeEndDesc{PipeEndDesc::Kind::Open, -1, c.ec, true}, 500.0);
        p.wall.friction = WallFriction::None;
        PipeNetworkDesc d;
        d.ambient = GasStateDesc{101325.0, 293.15, 0.0};
        d.pipes.push_back(p);
        PipeNetwork net(d, c.h);
        const double a = sound_speed(net, 293.15, 0.0);  // the network's OWN ambient sound speed
        const double fs = 1.0 / c.h;
        const double kk = (c.ec == EndCorrection::Flanged) ? 1.0 : 0.5;
        const double beta = fs * 0.5 * c.d / a;
        const double b = 2.0 * kk * beta * beta;
        const double pole_ref = ((2.0 * b + 1.0) - std::sqrt(4.0 * b + 1.0)) / (2.0 * b);
        const double pole = net.radiation_pole(0, 1);
        std::printf("V-radiation W1: d %.3f %s h %.3e: pole %.9f vs ref %.9f (rel %.2e)\n", c.d,
                    c.ec == EndCorrection::Flanged ? "flanged" : "unflanged", c.h, pole, pole_ref,
                    rel(pole, pole_ref));
        CHECK(rel(pole, pole_ref) < 1e-9);
        CHECK(pole > 0.0);
        CHECK(pole < 1.0);  // a stable one-pole lowpass
    }
    // A non-radiating end's pole stays exactly 0 (gating check).
    {
        PipeDesc p = make_pipe("norad", 0.40, 0.05, 0.05, 0.017, closed_end(), open_end(EndCorrection::Unflanged), 500.0);
        PipeNetworkDesc d;
        d.pipes.push_back(p);
        PipeNetwork net(d, kHFull);
        CHECK(net.radiation_pole(0, 1) == 0.0);
    }
}

TEST_CASE("V-radiation W1 sabotage diag: flanged/unflanged K swapped moves the pole off the reference",
          "[.][diag][audio]") {
    // Sabotage (by hand, matching pipe_network.cpp's own K selection): if
    // `kk = (e.end_correction == EndCorrection::Flanged) ? 1.0 : 0.5` were
    // swapped (0.5 flanged / 1.0 unflanged), the two cases below would each
    // compare against the OTHER's reference and fail W1's 1e-9 bound.
    const double h = kHFull, d = 0.05, a = 343.0;
    const double fs = 1.0 / h;
    for (int fl = 0; fl < 2; ++fl) {
        const double kk_correct = fl ? 1.0 : 0.5;
        const double kk_swapped = fl ? 0.5 : 1.0;  // sabotage
        const double beta = fs * 0.5 * d / a;
        const double b_correct = 2.0 * kk_correct * beta * beta;
        const double b_swapped = 2.0 * kk_swapped * beta * beta;
        const double pole_correct = ((2.0 * b_correct + 1.0) - std::sqrt(4.0 * b_correct + 1.0)) / (2.0 * b_correct);
        const double pole_swapped = ((2.0 * b_swapped + 1.0) - std::sqrt(4.0 * b_swapped + 1.0)) / (2.0 * b_swapped);
        std::printf("V-radiation W1 sabotage (%s): pole %.9f vs correct-K reference %.9f (rel %.4f, want > 1e-9)\n",
                    fl ? "flanged" : "unflanged", pole_swapped, pole_correct, rel(pole_swapped, pole_correct));
    }
}

TEST_CASE("V-radiation W2: turning radiation on measurably increases a closed-open resonator's simulated decay",
          "[drivetrain][cycle][pipe][audio][long]") {
    // Black-box, simulated: a closed-open quarter-wave resonator (P4's own
    // "closed-open quarter wave" geometry) with wall friction and heat
    // transfer both disabled (the ONLY remaining loss mechanisms are FV
    // numerical dissipation, present in BOTH cases below, and - only with
    // radiation on - the F3 fix's physical open-end loss), excited once with
    // a small Gaussian pulse and left to ring down. Decay rate measured the
    // way test_waveguide_network.cpp's W3 measures Kirchhoff wall-loss mode
    // decay: two Hann-windowed DFT magnitudes at the resonance frequency,
    // 1.5 s apart, decay_rate = -ln(ratio)/gap_time.
    //
    // This does NOT attempt a tight quantitative match to the analytic
    // Levine-Schwinger decay rate alpha = -c*ln|R| / (2*L_eff) (see W1's own
    // comment on why the FV scheme's numerical dissipation, accumulated over
    // the ~300 round trips a 1.5 s gap spans at this resonator's ~200 Hz,
    // confounds an absolute match - measured here: numerical-only decay
    // ~2.7 x the tiny physical rate alpha itself already predicts). Instead:
    // radiation must strictly increase the decay rate (catches a missing or
    // wrong-sign wiring bug at the reservoir_face() call site - the ONE
    // thing W1's formula-level check above cannot see, since that pole could
    // be computed correctly and simply never applied), and the EXCESS decay
    // it adds must be within an order of magnitude of the analytic
    // radiation-only rate (catches a wiring bug of the wrong SCALE, e.g. a
    // missing 0.5 factor or a units mixup) while allowing generously for the
    // numerical-dissipation confound W1 documents.
    auto make_net = [](bool radiation) {
        const double length = 0.40, dia = 0.05;
        PipeDesc p = make_pipe(
            "qw", length, dia, dia, 0.017, closed_end(),
            radiation ? PipeEndDesc{PipeEndDesc::Kind::Open, -1, EndCorrection::Unflanged, true} : open_end(EndCorrection::Unflanged),
            500.0);
        p.wall.friction = WallFriction::None;
        p.wall.heat_transfer = false;
        PipeNetworkDesc d;
        d.pipes.push_back(p);
        auto net = std::make_unique<PipeNetwork>(d, kHFull);
        gaussian_pulse(*net, 0, 101325.0, 293.15, 0.0, 0.01, 0.05, 0.04);
        return net;
    };
    auto measure = [](PipeNetwork& net, double f_lo, double f_hi) {
        const int stride = 4;
        const double dt = stride * net.h();
        const int n_win = 2304;  // 0.2 s
        const int gap = 17280;   // 1.5 s
        std::vector<double> s;
        s.reserve(static_cast<std::size_t>(gap + n_win));
        const int steps = (gap + n_win) * stride;
        for (int k = 0; k < steps; ++k) {
            net.step();
            if (k % stride == 0) s.push_back(net.cell(0, 0).p);
        }
        const double f = peak_frequency(std::vector<double>(s.begin(), s.begin() + n_win), dt, f_lo, f_hi);
        double mean0 = 0.0;
        for (int i = 0; i < n_win; ++i) mean0 += s[static_cast<std::size_t>(i)];
        mean0 /= n_win;
        std::vector<double> w0(s.begin(), s.begin() + n_win), w1(s.begin() + gap, s.begin() + gap + n_win);
        for (double& v : w0) v -= mean0;
        for (double& v : w1) v -= mean0;
        w0 = hann(w0);
        w1 = hann(w1);
        const double ratio = dft_mag(w1, dt, f) / dft_mag(w0, dt, f);
        return std::pair<double, double>{f, -std::log(ratio) / (gap * dt)};
    };

    const double length = 0.40, dia = 0.05, radius = 0.5 * dia;
    const double c = 343.2;  // 293.15 K air, matches sound_speed(net, 293.15, 0.0) to fp noise
    const double l_eff_est = length + kUnflangedEndCorrection * radius;
    const double f_est = c / (4.0 * l_eff_est);

    auto net_off = make_net(false);
    const auto [f_off, rate_off] = measure(*net_off, 0.8 * f_est, 1.2 * f_est);
    auto net_on = make_net(true);
    const auto [f_on, rate_on] = measure(*net_on, 0.8 * f_est, 1.2 * f_est);

    // Analytic radiation-only decay rate at the ON case's own measured
    // frequency/ka (the fix's radiation reactance shifts the resonance a
    // little beyond the plain end-correction estimate - using the MEASURED
    // f, not f_est, keeps ka and l_eff self-consistent).
    const double l_eff_on = c / (4.0 * f_on);
    const double ka_on = 2.0 * kPi * f_on * radius / c;
    const double r_ref = 1.0 - 0.5 * ka_on * ka_on;
    const double rate_ex = -c * std::log(r_ref) / (2.0 * l_eff_on);
    const double excess = rate_on - rate_off;
    std::printf(
        "V-radiation W2: off f %.3f Hz decay %.4f 1/s; on f %.3f Hz decay %.4f 1/s; excess %.4f 1/s vs analytic "
        "%.4f 1/s (ratio %.2fx)\n",
        f_off, rate_off, f_on, rate_on, excess, rate_ex, excess / rate_ex);
    CHECK(rate_on > rate_off);            // radiation strictly increases dissipation (sign)
    CHECK(excess > 0.1 * rate_ex);        // not negligible relative to the analytic loss (missing/too-weak wiring)
    CHECK(excess < 10.0 * rate_ex);       // not wildly too strong either (wrong-scale wiring)
    CHECK(std::isfinite(rate_on));
    CHECK(std::isfinite(rate_off));
}

TEST_CASE("P5 pipe port hand-off with a prescribed mass-flow/enthalpy source", "[drivetrain][cycle][pipe]") {
    // Catches: a port hand-off that books different mass/energy/burned mass on
    // the two sides (not conservative), a wrong flux sign or area at the port.
    // The test owns the port volume (as A5's cycle stepper will): exhaust-like
    // burned gas at 900 K is injected at 0.03 kg/s for 0.2 s, then stops; the
    // pipe carries it to an open end. Global closure over pipe + test volume
    // against the injected and the ambient-exchanged amounts.
    PipeNetworkDesc d;
    const double v_port = 2.5e-4;
    d.ports.push_back(PortDesc{"exhaust_port", v_port});
    PipeDesc p = make_pipe("primary", 0.5, 0.036, 0.036, 0.032, port_end(0), open_end(), 900.0);
    d.pipes.push_back(p);
    PipeNetwork net(d, kHFull);
    const GasModel& g = net.gas();

    double m = 101325.0 * v_port / (g.fresh.r * 293.15);
    double u = energy_from_temperature(g, m, 0.0, 293.15);
    double mb = 0.0;
    const double m_cv0 = m, u_cv0 = u;
    const double m0 = net.total_mass(), e0 = net.total_energy(), b0 = net.total_burned_mass();
    const double mdot_src = 0.03, t_src = 900.0;
    const double h_src = g.burned.h(t_src);
    double src_m = 0.0, src_e = 0.0;
    double port_m_sum = 0.0;
    double p_port_max = 0.0, p_port_end = 0.0, amb_out_window = 0.0;
    const int steps = static_cast<int>(0.3 / net.h());
    const int s_on_end = static_cast<int>(0.2 / net.h());
    const int s_win0 = static_cast<int>(0.15 / net.h());
    for (int s = 0; s < steps; ++s) {
        const CvState cs = cv_state(g, m, u, mb, v_port);
        net.set_port_state(0, PortState{cs.p, cs.t, cs.x});
        p_port_max = std::max(p_port_max, cs.p);
        p_port_end = cs.p;
        const double amb_before = net.exchange().ambient_mass_kg;
        net.step();
        const PortFlux& f = net.port_flux(0);
        const double dm_src = s < s_on_end ? mdot_src * net.h() : 0.0;
        m += dm_src - f.mass_kg;
        u += dm_src * h_src - f.energy_j;
        mb += dm_src - f.burned_kg;
        src_m += dm_src;
        src_e += dm_src * h_src;
        port_m_sum += f.mass_kg;
        if (s >= s_win0 && s < s_on_end) amb_out_window -= net.exchange().ambient_mass_kg - amb_before;
    }
    const PipeNetworkExchange& x = net.exchange();
    const double m_close = (net.total_mass() + m) - (m0 + m_cv0) - src_m - x.ambient_mass_kg;
    const double e_close = (net.total_energy() + u) - (e0 + u_cv0) - src_e - x.ambient_energy_j;
    const double b_close = (net.total_burned_mass() + mb) - b0 - src_m - x.ambient_burned_kg;
    const double mdot_out = amb_out_window / ((s_on_end - s_win0) * net.h());
    std::printf("P5 port: closure mass %.3e kg energy %.3e J burned %.3e kg (of %.4f kg / %.1f J injected); "
                "open-end outflow %.5f kg/s vs source %.5f (%+.3f %%); port p max %.0f Pa, end %.0f Pa\n",
                m_close, e_close, b_close, src_m, src_e, mdot_out, mdot_src, 100.0 * (mdot_out / mdot_src - 1.0),
                p_port_max, p_port_end);
    CHECK(std::fabs(m_close) < 1e-12 * src_m);
    CHECK(std::fabs(e_close) < 1e-12 * src_e);
    CHECK(std::fabs(b_close) < 1e-12 * src_m);
    CHECK(std::fabs(x.port_mass_kg - port_m_sum) <= 1e-15 * src_m);
    // Quasi-steady over the last 50 ms of injection: the open end carries the source.
    CHECK(rel(mdot_out, mdot_src) < 0.03);
    CHECK(p_port_max > 101325.0 + 1000.0);
    CHECK(std::fabs(p_port_end - 101325.0) < 2000.0);
}

TEST_CASE("P6 pipe CFL rejection at build time (dx 6.25 mm, h 21.7 us)", "[drivetrain][cycle][pipe]") {
    // Catches: a missing CFL check (this case went NaN in audio_plan.md M1).
    PipeNetworkDesc d;
    d.pipes.push_back(make_pipe("primary", 0.5, 0.036, 0.036, 0.00625, closed_end(), closed_end(), 900.0));
    std::string msg;
    try {
        PipeNetwork net(d, kHFull);
    } catch (const std::invalid_argument& e) {
        msg = e.what();
    }
    CHECK(msg ==
          "PipeNetwork: pipe 'primary' cell size 0.00625 m is below the CFL minimum 0.02441 m (max |u|+a 900 m/s, "
          "h 2.17e-05 s, CFL limit 0.8)");
    // Just above the minimum is accepted.
    d.pipes[0].dx_target_m = 0.0245;
    CHECK_NOTHROW(PipeNetwork(d, kHFull));

    SECTION("volume and orifice coupling checks") {
        PipeNetworkDesc v;
        v.volumes.push_back(VolumeDesc{"tiny_junction", 1.0e-5, GasStateDesc{}});
        v.pipes.push_back(make_pipe("a", 0.3, 0.04, 0.04, 0.032, closed_end(), volume_end(0)));
        std::string m2;
        try {
            PipeNetwork net(v, kHFull);
        } catch (const std::invalid_argument& e) {
            m2 = e.what();
        }
        CHECK(m2.rfind("PipeNetwork: volume 'tiny_junction' 1e-05 m^3 is below the coupling minimum", 0) == 0);

        PipeNetworkDesc o;
        o.volumes.push_back(VolumeDesc{"small", 1.0e-4, GasStateDesc{}});
        o.orifices.push_back(OrificeDesc{"throttle", 0, -1, 5.0e-3, 500.0});
        std::string m3;
        try {
            PipeNetwork net(o, kHFull);
        } catch (const std::invalid_argument& e) {
            m3 = e.what();
        }
        CHECK(m3.rfind("PipeNetwork: orifice 'throttle' is too stiff for h", 0) == 0);
    }
}

TEST_CASE("P7 pipe stepping allocates nothing inside a job", "[drivetrain][cycle][pipe]") {
    // Catches: a heap allocation in step() (scratch not preallocated).
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    PipeNetwork net(i4_like_network(), kHFull);
    ps::jobs::JobSystem js(2);
    auto run = [&](int n) {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit(
            [&] {
                for (int k = 0; k < n; ++k) {
                    drive_ports(net, net.steps());
                    net.step();
                }
            },
            &wg);
        js.wait(wg);
    };
    run(10);  // warm-up
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    run(2000);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
    CHECK(std::isfinite(net.total_energy()));
}

TEST_CASE("P8 pipe run is bit-identical run to run", "[drivetrain][cycle][pipe]") {
    // Catches: hidden state (a mutable static, uninitialised scratch read,
    // order dependence) that makes two identical runs differ.
    auto run = [] {
        PipeNetwork net(i4_like_network(), kHFull);
        for (int k = 0; k < 4000; ++k) {
            drive_ports(net, net.steps());
            net.step();
        }
        return net;
    };
    const PipeNetwork a = run();
    const PipeNetwork b = run();
    const std::uint64_t ha = a.fold_state_hash(1469598103934665603ull);
    const std::uint64_t hb = b.fold_state_hash(1469598103934665603ull);
    std::printf("P8 hash %016llx / %016llx, cells %d\n", static_cast<unsigned long long>(ha),
                static_cast<unsigned long long>(hb), a.total_cell_count());
    CHECK(ha == hb);
    CHECK(a.exchange().ambient_energy_j == b.exchange().ambient_energy_j);
    CHECK(std::isfinite(a.total_energy()));
}

TEST_CASE("pipe diag: what the CFL check prevents (dx 6.25 mm at h_full)", "[.][diag][drivetrain][cycle][pipe]") {
    // With the check in place this only reports the rejection. With the check
    // sabotaged out it runs audio_plan.md M1's failing case - 800 K burned gas
    // in a 6.25 mm-cell primary with a 20 % pulse - and reports the first step
    // with a non-finite or non-positive state.
    PipeNetworkDesc d;
    d.ambient = GasStateDesc{101325.0, 800.0, 1.0};
    PipeDesc p = make_pipe("primary", 0.5, 0.036, 0.036, 0.00625, closed_end(), closed_end(), 900.0);
    p.initial = GasStateDesc{101325.0, 800.0, 1.0};
    d.pipes.push_back(p);
    try {
        PipeNetwork net(d, kHFull);
        gaussian_pulse(net, 0, 101325.0, 800.0, 1.0, 0.2, 0.1, 0.03);
        int bad_step = -1;
        for (int s = 0; s < 20000 && bad_step < 0; ++s) {
            net.step();
            for (int i = 0; i < net.cell_count(0); ++i) {
                const PipeCellState c = net.cell(0, i);
                if (!(std::isfinite(c.p) && c.p > 0.0 && std::isfinite(c.rho) && c.rho > 0.0)) {
                    bad_step = s;
                    break;
                }
            }
        }
        std::printf("CFL diag: constructed (check disabled?) - first bad step %d, total energy %.6g\n", bad_step,
                    net.total_energy());
    } catch (const std::invalid_argument& e) {
        std::printf("CFL diag: rejected at build time: %s\n", e.what());
    }
}

TEST_CASE("pipe diag: micro-benchmark ns per cell-step (I4-like audio grade)", "[.][diag][drivetrain][cycle][pipe]") {
    for (int variant = 0; variant < 2; ++variant) {
        PipeNetworkDesc d = i4_like_network();
        if (variant == 1) {
            for (PipeDesc& p : d.pipes) {
                p.wall.friction = WallFriction::None;
                p.wall.heat_transfer = false;
            }
        }
        PipeNetwork net(d, kHFull);
        for (int k = 0; k < 2000; ++k) {
            drive_ports(net, net.steps());
            net.step();
        }
        const int steps = 46080;  // 1 s at h_full
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < steps; ++k) {
            drive_ports(net, net.steps());
            net.step();
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        // drive_ports costs 8 sin + state refreshes per step; measure it alone.
        const auto t2 = std::chrono::steady_clock::now();
        for (int k = 0; k < steps; ++k) drive_ports(net, static_cast<std::uint64_t>(k));
        const auto t3 = std::chrono::steady_clock::now();
        const double ns_drive = std::chrono::duration<double, std::nano>(t3 - t2).count();
        const double cells = net.total_cell_count();
        std::printf("pipe bench (%s): %d cells, %d steps: %.2f ns per cell-step (%.2f with port driving), %.3f s per "
                    "simulated s\n",
                    variant == 0 ? "friction+heat as built" : "no wall terms", net.total_cell_count(), steps,
                    (ns - ns_drive) / (steps * cells), ns / (steps * cells), ns * 1e-9);
        CHECK(std::isfinite(net.total_energy()));
    }
}
