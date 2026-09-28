// tests/unit/test_pipe_waveguide.cpp - ps_audio A3 part 2: the finite-volume
// pipe core coupled to the linear waveguide (pipe_network.h "Waveguide ends").
// out/audio_plan.md section 7 row V4 (FV vs WG on the same geometry at small
// amplitude, impulse responses within 1 dB up to 1.5 kHz) plus the interface
// cases I1-I4: transparency and energy hand-over, a quarter-wave resonance
// across the split, mean-flow closure, and the hybrid FV+WG network against the
// pure FV one. Sabotage runs and their failing numbers are recorded in
// out/audio_a3_progress.md.

#include "ps/engine_cycle/pipe_network.h"
#include "ps/engine_cycle/waveguide_network.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHFull = 1.0 / 46080.0;  // the cycle model's h_full, 1/(960*48) s
constexpr double kPAmb = 101325.0;
constexpr double kTAmb = 293.15;

double rel(double a, double b) { return std::fabs(a - b) / std::fabs(b); }

PipeDesc make_pipe(const char* name, double length, double d0, double d1, double dx, PipeEndDesc start,
                   PipeEndDesc end, double c_max = 500.0) {
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
PipeEndDesc open_end(EndCorrection c = EndCorrection::Unflanged) { return PipeEndDesc{PipeEndDesc::Kind::Open, -1, c}; }
PipeEndDesc volume_end(int v) { return PipeEndDesc{PipeEndDesc::Kind::Volume, v, EndCorrection::None}; }
PipeEndDesc wg_end(int node) { return PipeEndDesc{PipeEndDesc::Kind::Waveguide, node, EndCorrection::None}; }

WgNodeDesc wg_node(WgNodeDesc::Kind k, const char* name, double volume = 0.0, bool radiation = true) {
    WgNodeDesc n;
    n.kind = k;
    n.name = name;
    n.volume_m3 = volume;
    n.radiation = radiation;
    return n;
}
WgSegmentDesc wg_seg(const char* name, double length, double dia, int a, int b, EndCorrection eb, WgLoss loss) {
    WgSegmentDesc s;
    s.name = name;
    s.length_m = length;
    s.diameter_m = dia;
    s.node_a = a;
    s.node_b = b;
    s.end_b = eb;
    s.loss = loss;
    return s;
}

double sound_speed(const PipeNetwork& net, double t, double x_b) {
    const double cv = net.gas().fresh.cv(t) * (1.0 - x_b) + net.gas().burned.cv(t) * x_b;
    const double r = net.gas().r_mix(x_b);
    return std::sqrt((cv + r) / cv * r * t);
}

// Isentropic Gaussian pressure pulse (at rest) around xc in a pipe.
void gaussian_pulse(PipeNetwork& net, int pipe, double amp, double xc, double w) {
    const double cv = net.gas().fresh.cv(kTAmb);
    const double r = net.gas().r_mix(0.0);
    const double gamma = (cv + r) / cv;
    for (int i = 0; i < net.cell_count(pipe); ++i) {
        const double s = (net.cell_center_m(pipe, i) - xc) / w;
        const double p = kPAmb * (1.0 + amp * std::exp(-s * s));
        const double t = kTAmb * std::pow(p / kPAmb, (gamma - 1.0) / gamma);
        net.set_cell(pipe, i, p, t, 0.0, 0.0);
    }
}

// Linear acoustic energy in the FV pipes: sum V (p'^2/(2 rho c^2) + rho u^2/2).
double fv_acoustic_energy(const PipeNetwork& net, double c) {
    double e = 0.0;
    for (int k = 0; k < net.pipe_count(); ++k) {
        for (int i = 0; i < net.cell_count(k); ++i) {
            const PipeCellState s = net.cell(k, i);
            const double dp = s.p - kPAmb;
            e += net.cell_volume_m3(k, i) * (dp * dp / (2.0 * s.rho * c * c) + 0.5 * s.rho * s.u * s.u);
        }
    }
    return e;
}

double dft_mag(const std::vector<double>& s, double dt, double f) {
    double re = 0.0, im = 0.0;
    const double w = 2.0 * kPi * f * dt;
    for (std::size_t n = 0; n < s.size(); ++n) {
        re += s[n] * std::cos(w * static_cast<double>(n));
        im -= s[n] * std::sin(w * static_cast<double>(n));
    }
    return std::sqrt(re * re + im * im);
}

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
        const double m = dft_mag(s, dt, f_lo + df * k);
        if (m > best_m) {
            best_m = m;
            best_f = f_lo + df * k;
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

// 1/3-octave band levels [dB] of a (decayed) impulse response, |X(f)|^2 summed
// on a 1 Hz grid (phasor recurrence, renormalised every 1024 samples).
std::vector<double> third_octave_levels(const std::vector<double>& s, double dt, const std::vector<double>& centres) {
    std::vector<double> out;
    const double k3 = std::pow(2.0, 1.0 / 6.0);
    for (double fc : centres) {
        double sum = 0.0;
        for (double f = std::ceil(fc / k3); f < fc * k3; f += 1.0) {
            const double w = 2.0 * kPi * f * dt;
            const double cr = std::cos(w), ci = -std::sin(w);
            double pr = 1.0, pi = 0.0, re = 0.0, im = 0.0;
            for (std::size_t n = 0; n < s.size(); ++n) {
                re += s[n] * pr;
                im += s[n] * pi;
                const double nr = pr * cr - pi * ci;
                pi = pr * ci + pi * cr;
                pr = nr;
                if ((n & 1023u) == 1023u) {
                    const double m = std::sqrt(pr * pr + pi * pi);
                    pr /= m;
                    pi /= m;
                }
            }
            sum += re * re + im * im;
        }
        out.push_back(10.0 * std::log10(sum));
    }
    return out;
}

const std::vector<double>& band_centres() {
    static const std::vector<double> c = {100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0, 500.0,
                                          630.0, 800.0, 1000.0, 1250.0, 1600.0};
    return c;
}

// --- V4 / I4 geometry: a 2 cm^3 source volume, a tube tapered 2 -> 4 mm over
// 0.6 m, an unflanged open end without radiation loss on either side. Laminar
// friction is the only physical loss (the FV's small-amplitude limit;
// WgLoss::LaminarFriction, 60 -> 15 1/s). The tube is narrow so that this
// loss dominates the FV's own numerical dissipation (4-8 1/s at dx 8.5 mm,
// FV-dissipation diag below): at 4 -> 12 mm the FV rings 5-8 dB less than
// the waveguide above 800 Hz (V4 geometry sweep diag).
struct V4Geom {
    double volume = 2.0e-6;
    double length = 0.6;
    double d0 = 0.002;
    double d1 = 0.004;
    double dx = 0.0085;
    double h = kHFull / 2.0;
    double seconds = 0.3;
    int stride = 2;
    double dm = 8.0e-11;  // injected mass [kg]
    double diameter_at(double x) const { return d0 + (d1 - d0) * x / length; }
    double dt() const { return h * stride; }
};

// WG staircase of the taper from x0 to x1 (n segments, diameters at the
// segment midpoints); node indices first_node .. first_node + n.
void add_staircase(const V4Geom& g, WaveguideNetworkDesc& d, double x0, double x1, int n, int first_node, WgLoss loss) {
    for (int k = 0; k < n; ++k) {
        const double xa = x0 + (x1 - x0) * k / n;
        const double xb = x0 + (x1 - x0) * (k + 1) / n;
        const bool last = (k == n - 1);
        WgSegmentDesc s = wg_seg("", xb - xa, g.diameter_at(0.5 * (xa + xb)), first_node + k, first_node + k + 1,
                                 last ? EndCorrection::Unflanged : EndCorrection::None, loss);
        s.name = "stair" + std::to_string(k);
        d.segments.push_back(s);
        if (!last) d.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "j"));
    }
    d.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "mouth", 0.0, false));
}

std::vector<double> v4_fv_response(const V4Geom& g, bool with_friction = true) {
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"source", g.volume, GasStateDesc{kPAmb, kTAmb, 0.0}});
    d.pipes.push_back(make_pipe("taper", g.length, g.d0, g.d1, g.dx, volume_end(0), open_end()));
    if (!with_friction) d.pipes[0].wall.friction = WallFriction::None;
    PipeNetwork net(d, g.h);
    const double h_tot = net.gas().fresh.u(kTAmb) + net.gas().r_mix(0.0) * kTAmb;
    std::vector<double> s;
    const int steps = static_cast<int>(g.seconds / g.h);
    for (int k = 0; k < steps; ++k) {
        net.set_volume_source(0, k == 0 ? g.dm / g.h : 0.0, h_tot, 0.0);
        net.step();
        if (k % g.stride == 0) s.push_back(net.volume_state(0).p - kPAmb);
    }
    return s;
}

std::vector<double> v4_wg_response(const V4Geom& g, WgLoss loss, int segments) {
    WaveguideNetworkDesc d;
    d.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "source", g.volume));
    d.nodes.back().volume_gas = GasStateDesc{kPAmb, kTAmb, 0.0};
    add_staircase(g, d, 0.0, g.length, segments, 0, loss);
    WaveguideNetwork net(d, g.h);
    const double rho = net.segment_density(0);
    std::vector<double> s;
    const int steps = static_cast<int>(g.seconds / g.h);
    for (int k = 0; k < steps; ++k) {
        if (k == 0) net.set_node_inflow(0, g.dm / (rho * g.h));
        net.step();
        if (k % g.stride == 0) s.push_back(net.node_pressure(0));
    }
    return s;
}

// Hybrid: FV taper over [0, x_split] into a waveguide staircase over the rest.
std::vector<double> v4_hybrid_response(const V4Geom& g, double x_split, int segments, double mean_time_s = 0.05) {
    PipeNetworkDesc d;
    d.waveguide_mean_time_s = mean_time_s;
    d.volumes.push_back(VolumeDesc{"source", g.volume, GasStateDesc{kPAmb, kTAmb, 0.0}});
    d.pipes.push_back(make_pipe("taper", x_split, g.d0, g.diameter_at(x_split), g.dx, volume_end(0), wg_end(0)));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "split"));
    add_staircase(g, d.waveguide, x_split, g.length, segments, 0, WgLoss::LaminarFriction);
    PipeNetwork net(d, g.h);
    const double h_tot = net.gas().fresh.u(kTAmb) + net.gas().r_mix(0.0) * kTAmb;
    std::vector<double> s;
    const int steps = static_cast<int>(g.seconds / g.h);
    for (int k = 0; k < steps; ++k) {
        net.set_volume_source(0, k == 0 ? g.dm / g.h : 0.0, h_tot, 0.0);
        net.step();
        if (k % g.stride == 0) s.push_back(net.volume_state(0).p - kPAmb);
    }
    return s;
}

// Largest |s| over the last 10 % of the window relative to the peak: the
// response must have decayed, or window truncation leaks into the low bands.
double tail_ratio(const std::vector<double>& s) {
    double peak = 0.0, tail = 0.0;
    for (std::size_t k = 0; k < s.size(); ++k) {
        peak = std::fmax(peak, std::fabs(s[k]));
        if (k >= s.size() * 9 / 10) tail = std::fmax(tail, std::fabs(s[k]));
    }
    return tail / peak;
}

double worst_band_difference(const V4Geom& g, const std::vector<double>& a, const std::vector<double>& b,
                             const char* label, bool print = true) {
    const std::vector<double> la = third_octave_levels(a, g.dt(), band_centres());
    const std::vector<double> lb = third_octave_levels(b, g.dt(), band_centres());
    double worst = 0.0;
    for (std::size_t k = 0; k < la.size(); ++k) {
        const double diff = lb[k] - la[k];
        worst = std::fmax(worst, std::fabs(diff));
        if (print)
            std::printf("%s band %6.0f Hz: ref %8.3f dB, test %8.3f dB, diff %+.3f dB\n", label, band_centres()[k], la[k],
                        lb[k], diff);
    }
    std::printf("%s: tail/peak ref %.2e test %.2e\n", label, tail_ratio(a), tail_ratio(b));
    return worst;
}

} // namespace

TEST_CASE("V4 FV vs WG on the same geometry: impulse-response band levels within 1 dB to 1.5 kHz",
          "[drivetrain][cycle][pipe]") {
    // Catches: a WG loss filter that does not reproduce the FV's linear loss,
    // a FV taper without its area source term, a WG junction/compliance error
    // (plan V4). Geometry: V4Geom; the WG is a 40-step staircase of the FV's
    // linear taper; 1/3-octave band levels 100..1600 Hz of the source volume's
    // pressure over the window (the response has decayed by then).
    const V4Geom g;
    const std::vector<double> fv = v4_fv_response(g);
    const std::vector<double> wg = v4_wg_response(g, WgLoss::LaminarFriction, 40);
    const double worst = worst_band_difference(g, fv, wg, "V4 FV->WG");
    std::printf("V4: worst band difference %.3f dB\n", worst);
    CHECK(worst < 1.0);
}

TEST_CASE("I4 hybrid FV+WG vs pure FV on the same geometry: band levels within 1 dB", "[drivetrain][cycle][pipe]") {
    // Catches: an interface that reflects or loses energy, a wrong interface
    // admittance, a mean-flow term leaking into the acoustic path. The taper
    // is split at 0.25 m (FV upstream, WG staircase downstream).
    const V4Geom g;
    const std::vector<double> fv = v4_fv_response(g);
    const std::vector<double> hy = v4_hybrid_response(g, 0.25, 24);
    const double worst = worst_band_difference(g, fv, hy, "I4 FV->hybrid");
    std::printf("I4: worst band difference %.3f dB\n", worst);
    CHECK(worst < 1.0);
}

TEST_CASE("pipe diag: FV pulse energy loss", "[.][diag][drivetrain][cycle][pipe]") {
    for (double sigma : {0.05, 0.1}) {
        for (int refine : {1, 2}) {
            PipeNetworkDesc d;
            d.pipes.push_back(make_pipe("fv", 3.0, 0.04, 0.04, 0.0085 / refine, closed_end(), closed_end()));
            d.pipes[0].wall.friction = WallFriction::None;
            const double h = kHFull / (2.0 * refine);
            PipeNetwork net(d, h);
            gaussian_pulse(net, 0, 1.0e-3, 1.5, sigma);
            const double c = sound_speed(net, kTAmb, 0.0);
            const double e0 = fv_acoustic_energy(net, c);
            std::printf("FV pulse sigma %.2f m dx %.2f mm:", sigma, 8.5 / refine);
            for (int ms = 1; ms <= 3; ++ms) {
                const int n = static_cast<int>(0.001 / h);
                for (int k = 0; k < n; ++k) net.step();
                std::printf(" %d ms %+.3f %%", ms, 100.0 * (fv_acoustic_energy(net, c) / e0 - 1.0));
            }
            std::printf("\n");
        }
    }
}

TEST_CASE("pipe diag: V4 envelopes", "[.][diag][drivetrain][cycle][pipe]") {
    V4Geom g;
    g.d0 = 0.002;
    g.d1 = 0.004;
    g.seconds = 0.3;
    const std::vector<double> fv = v4_fv_response(g);
    const std::vector<double> wg = v4_wg_response(g, WgLoss::LaminarFriction, 40);
    const std::vector<double> hy = v4_hybrid_response(g, 0.25, 24);
    for (double t0 : {0.0, 0.005, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.29}) {
        const std::size_t a = static_cast<std::size_t>(t0 / g.dt()), b = static_cast<std::size_t>((t0 + 0.005) / g.dt());
        double mf = 0, mw = 0, mh = 0, sf = 0, sw = 0, sh = 0;
        for (std::size_t k = a; k < b && k < fv.size(); ++k) {
            mf = std::fmax(mf, std::fabs(fv[k])); mw = std::fmax(mw, std::fabs(wg[k])); mh = std::fmax(mh, std::fabs(hy[k]));
            sf += fv[k]; sw += wg[k]; sh += hy[k];
        }
        const double n = static_cast<double>(b - a);
        std::printf("env t %.3f: max fv %.3e wg %.3e hy %.3e | mean fv %+.3e wg %+.3e hy %+.3e\n", t0, mf, mw, mh, sf / n, sw / n, sh / n);
    }
}

TEST_CASE("pipe diag: V4 geometry sweep", "[.][diag][drivetrain][cycle][pipe]") {
    // Worst band differences FV vs WG (laminar and lossless) and FV vs hybrid
    // over tube diameters and FV cell sizes.
    struct Case { double d0, d1, dx; int refine; };
    for (const Case c : {Case{0.004, 0.012, 0.0085, 2}, Case{0.002, 0.004, 0.0085, 2}, Case{0.0015, 0.003, 0.0085, 2},
                         Case{0.002, 0.004, 0.00425, 4}, Case{0.004, 0.012, 0.00425, 4}}) {
        V4Geom g;
        g.d0 = c.d0;
        g.d1 = c.d1;
        g.dx = c.dx;
        g.h = kHFull / c.refine;
        g.stride = c.refine;
        g.seconds = 0.3;
        const std::vector<double> fv = v4_fv_response(g);
        const double w_lam = worst_band_difference(g, fv, v4_wg_response(g, WgLoss::LaminarFriction, 40), "sweep wg", c.d0 < 0.0021);
        worst_band_difference(g, fv, v4_wg_response(g, WgLoss::LaminarFriction, 80), "sweep wg80", c.d0 < 0.0021);
        const double w_none = worst_band_difference(g, fv, v4_wg_response(g, WgLoss::None, 40), "sweep wg-none", false);
        const double w_hy = worst_band_difference(g, fv, v4_hybrid_response(g, 0.25, 24), "sweep hybrid", c.d0 < 0.0021);
        std::printf("V4 sweep d %.1f->%.1f mm dx %.2f mm: WG laminar %.3f dB, WG lossless %.3f dB, hybrid %.3f dB\n",
                    c.d0 * 1e3, c.d1 * 1e3, c.dx * 1e3, w_lam, w_none, w_hy);
    }
}

TEST_CASE("I1 FV->WG face is transparent and hands the acoustic energy over", "[drivetrain][cycle][pipe]") {
    // Catches: a face that reflects (wrong Y_F, a factor 2 in the outgoing
    // wave), a face that does not hand p_J Q_F to the waveguide, a wrong
    // reference pressure. A Gaussian pulse in a closed-start FV pipe (same gas
    // and diameter as the WG segment) leaves through the face into an
    // anechoically terminated waveguide. sigma 0.1 m at dx 4.25 mm: the FV's
    // own numerical dissipation of a 6-cell pulse (sigma 0.05 m at 8.5 mm) is
    // 8 % of its energy per ms (FV pulse energy loss diag); here ~0.5 %.
    PipeNetworkDesc d;
    d.pipes.push_back(make_pipe("fv", 0.6, 0.04, 0.04, 0.00425, closed_end(), wg_end(0)));
    d.pipes[0].wall.friction = WallFriction::None;
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Anechoic, "sink"));
    d.waveguide.segments.push_back(wg_seg("wg", 0.5, 0.04, 0, 1, EndCorrection::None, WgLoss::None));
    const double h = kHFull / 4.0;
    PipeNetwork net(d, h);
    const double amp = 1.0e-3;
    gaussian_pulse(net, 0, amp, 0.3, 0.1);
    const double c = sound_speed(net, kTAmb, 0.0);
    const double e0 = fv_acoustic_energy(net, c);
    const double m0 = net.total_mass();
    double residual = 0.0;
    const int steps = static_cast<int>(0.010 / h);
    for (int k = 0; k < steps; ++k) {
        net.step();
        if (k * h > 0.005) {
            for (int i = 0; i < net.cell_count(0); ++i)
                residual = std::fmax(residual, std::fabs(net.cell(0, i).p - kPAmb));
        }
    }
    const double work = net.waveguide_face(0).acoustic_work_j;
    const double e_left = fv_acoustic_energy(net, c);
    const double closure = std::fabs(net.total_mass() - m0 - net.exchange().waveguide_mass_kg) / m0;
    std::printf("I1: residual after the pulse left %.4f Pa of %.1f Pa (%.3f %%), work into WG %.6e J of FV %.6e J "
                "(%+.3f %%), FV energy left %.2e, mass closure %.2e, fallbacks %llu\n",
                residual, amp * kPAmb, 100.0 * residual / (amp * kPAmb), work, e0, 100.0 * (work / e0 - 1.0), e_left,
                closure, static_cast<unsigned long long>(net.waveguide_face_fallbacks()));
    CHECK(residual < 0.01 * amp * kPAmb);
    CHECK(rel(work, e0) < 0.01);
    CHECK(closure < 1e-13);
}

TEST_CASE("I8 WG->FV: a wave from the waveguide enters the FV pipe without reflection", "[drivetrain][cycle][pipe]") {
    // Catches: a wrong FV-side admittance in the face junction (the returning
    // wave is split between the waveguide and the FV with the wrong weights), a
    // Norton source dropped. A Gaussian wave (sigma 0.1 m) launched by an
    // anechoic node travels 0.5 m of waveguide into a 1.2 m closed FV pipe;
    // what comes back to the anechoic node before the closed-end echo is the
    // face's reflection. The energy that entered the FV must equal the launched
    // wave energy h Y sum p+^2.
    PipeNetworkDesc d;
    d.pipes.push_back(make_pipe("fv", 1.2, 0.04, 0.04, 0.00425, wg_end(1), closed_end()));
    d.pipes[0].wall.friction = WallFriction::None;
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Anechoic, "src"));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    d.waveguide.segments.push_back(wg_seg("wg", 0.5, 0.04, 0, 1, EndCorrection::None, WgLoss::None));
    const double h = kHFull / 4.0;
    PipeNetwork net(d, h);
    const double c = sound_speed(net, kTAmb, 0.0);
    const double y = net.waveguide().segment_admittance(0);
    const double sig_t = 0.1 / c, t0 = 4.0 * sig_t;
    double e_in = 0.0, e_back = 0.0;
    // Echo from the closed end reaches the source after (0.5 + 2.4 + 0.5)/c ~ 9.9 ms.
    const int steps = static_cast<int>(0.0085 / h);
    double e_fv = 0.0;
    for (int k = 0; k < steps; ++k) {
        const double s = (k * h - t0) / sig_t;
        const double wave = 100.0 * std::exp(-0.5 * s * s);
        net.waveguide().set_node_wave(0, wave);
        net.step();
        e_in += h * y * wave * wave;
        const double back = net.waveguide().node_arriving_wave(0);
        e_back += h * y * back * back;
        if (k == static_cast<int>(0.0055 / h)) e_fv = fv_acoustic_energy(net, c);
    }
    std::printf("I8: launched %.6e J, into the FV %.6e J (%+.3f %%), reflected %.3e of launched\n", e_in, e_fv,
                100.0 * (e_fv / e_in - 1.0), e_back / e_in);
    CHECK(rel(e_fv, e_in) < 0.01);
    CHECK(e_back < 1.0e-6 * e_in);  // -60 dB
}

TEST_CASE("I2 quarter-wave resonance across the FV/WG split (V3 across the interface)", "[drivetrain][cycle][pipe]") {
    // Catches: a face that ignores the waveguide's returning waves (the
    // Norton source), a delay or length lost at the face, an area step at the
    // face. Closed FV start 0.3 m, WG 0.3 m to an ideal unflanged open end.
    PipeNetworkDesc d;
    const double dia = 0.04;
    d.pipes.push_back(make_pipe("fv", 0.3, dia, dia, 0.017, closed_end(), wg_end(0)));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "mouth", 0.0, false));
    d.waveguide.segments.push_back(wg_seg("wg", 0.3, dia, 0, 1, EndCorrection::Unflanged, WgLoss::None));
    PipeNetwork net(d, kHFull);
    gaussian_pulse(net, 0, 1.0e-4, 0.05, 0.04);
    std::vector<double> s;
    for (int k = 0; k < 11520; ++k) {
        net.step();
        s.push_back(net.cell(0, 0).p);
    }
    const double c = sound_speed(net, kTAmb, 0.0);
    const double f_ex = c / (4.0 * (0.6 + kUnflangedEndCorrection * 0.5 * dia));
    const double f = peak_frequency(s, kHFull, 0.8 * f_ex, 1.2 * f_ex);
    std::printf("I2: f %.3f Hz vs c/(4(L+0.6133a)) %.3f Hz (%+.4f %%)\n", f, f_ex, 100.0 * (f / f_ex - 1.0));
    CHECK(rel(f, f_ex) < 0.01);
}

TEST_CASE("I3 mean flow through the face: closure, reference pressure, whole flow into the waveguide",
          "[drivetrain][cycle][pipe]") {
    // Catches: a face that bypasses (part of) the flow around the waveguide
    // (mean Q_F would fall short of the face volume flow), a wrong reference
    // pressure, mass/energy booked on one side only. A 1 L volume fed 20 g/s of
    // 293 K air drives a 0.3 m FV pipe into a 0.4 m waveguide tailpipe with a
    // radiating end; no Resistance element, so the mean face pressure is ambient.
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"box", 1.0e-3, GasStateDesc{kPAmb, kTAmb, 0.0}});
    d.pipes.push_back(make_pipe("fv", 0.3, 0.04, 0.04, 0.017, volume_end(0), wg_end(0)));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "tail"));
    d.waveguide.segments.push_back(wg_seg("wg", 0.4, 0.04, 0, 1, EndCorrection::Unflanged, WgLoss::Kirchhoff));
    PipeNetwork net(d, kHFull);
    const double mdot = 0.02;
    const double h_tot = net.gas().fresh.u(kTAmb) + net.gas().r_mix(0.0) * kTAmb;
    net.set_volume_source(0, mdot, h_tot, 0.0);
    const double m0 = net.total_mass(), e0 = net.total_energy();
    const int steps = static_cast<int>(0.6 / kHFull);
    const int tail = static_cast<int>(0.1 / kHFull);
    double q_sum = 0.0, p_sum = 0.0, m_face = 0.0, rho_sum = 0.0;
    for (int k = 0; k < steps; ++k) {
        net.step();
        if (k >= steps - tail) {
            const WaveguideFaceView& f = net.waveguide_face(0);
            q_sum += f.q_m3_s;
            p_sum += f.p_pa;
            rho_sum += f.rho;
            m_face += f.rho * f.u_m_s * f.area_m2;
        }
    }
    const WaveguideFaceView& f = net.waveguide_face(0);
    const double q_mean = q_sum / tail, p_mean = p_sum / tail, m_mean = m_face / tail;
    const double vol_flow = m_mean / (rho_sum / tail);
    const PipeNetworkExchange& x = net.exchange();
    // Relative to the mass that went through (~7.5 m0 over the run).
    const double mass_closure =
        std::fabs(net.total_mass() - m0 - x.source_mass_kg - x.waveguide_mass_kg) / x.source_mass_kg;
    const double energy_closure =
        std::fabs(net.total_energy() - e0 - x.source_energy_j - x.waveguide_energy_j) / std::fabs(x.source_energy_j);
    std::printf("I3: face mdot %.6f (running mean %.6f) vs source %.6f kg/s, u_mean %.3f m/s, mean Q_F %.3e of %.3e "
                "m^3/s, mean p_f - p_amb %+.4f Pa, p_ref - p_amb %+.4f, closure mass %.2e energy %.2e\n",
                m_mean, f.mdot_mean_kg_s, mdot, f.u_mean_m_s, q_mean, vol_flow, p_mean - kPAmb, f.p_ref_pa - kPAmb,
                mass_closure, energy_closure);
    CHECK(rel(m_mean, mdot) < 0.01);
    CHECK(rel(f.mdot_mean_kg_s, mdot) < 0.01);
    CHECK(rel(q_mean, vol_flow) < 1e-3);
    CHECK(std::fabs(p_mean - kPAmb) < 0.5);
    CHECK(f.p_ref_pa == kPAmb);
    CHECK(mass_closure < 1e-12);
    CHECK(energy_closure < 1e-12);
}

TEST_CASE("I7 a waveguide resistance element's mean drop reaches the FV side",
          "[drivetrain][cycle][pipe]") {
    // Catches: a reference pressure without the linearisation correction (the
    // DC flow through the linearised element drops K rho u^2, twice the true
    // K rho u^2/2), a drop from the instantaneous instead of the mean flow. As
    // I3, with a catalyst-like Resistance node (k_loss 4 [A], at the 40 mm pipe
    // area) in the waveguide tailpipe. After the running mean settled the face
    // mean pressure must sit K rho u^2/2 above ambient (u = mdot/(rho A)), and
    // p_ref at ambient - K rho u^2/2 (pipe_network.h "Waveguide ends").
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"box", 1.0e-3, GasStateDesc{kPAmb, kTAmb, 0.0}});
    d.pipes.push_back(make_pipe("fv", 0.3, 0.04, 0.04, 0.017, volume_end(0), wg_end(0)));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    WgNodeDesc cat = wg_node(WgNodeDesc::Kind::Resistance, "cat");
    cat.k_loss = 4.0;
    d.waveguide.nodes.push_back(cat);
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "tail"));
    d.waveguide.segments.push_back(wg_seg("front", 0.2, 0.04, 0, 1, EndCorrection::None, WgLoss::Kirchhoff));
    d.waveguide.segments.push_back(wg_seg("rear", 0.4, 0.04, 1, 2, EndCorrection::Unflanged, WgLoss::Kirchhoff));
    PipeNetwork net(d, kHFull);
    const double mdot = 0.02;
    const double h_tot = net.gas().fresh.u(kTAmb) + net.gas().r_mix(0.0) * kTAmb;
    net.set_volume_source(0, mdot, h_tot, 0.0);
    const int steps = static_cast<int>(0.8 / kHFull);
    const int tail = static_cast<int>(0.1 / kHFull);
    double p_sum = 0.0, m_sum = 0.0;
    for (int k = 0; k < steps; ++k) {
        net.step();
        if (k >= steps - tail) {
            const WaveguideFaceView& f = net.waveguide_face(0);
            p_sum += f.p_pa;
            m_sum += f.rho * f.u_m_s * f.area_m2;
        }
    }
    const double rho_w = net.waveguide().segment_density(0);
    const double area = 0.25 * kPi * 0.04 * 0.04;
    const double u = mdot / (rho_w * area);
    const double dp_ref = 4.0 * rho_w * u * u / 2.0;
    const WaveguideFaceView& f = net.waveguide_face(0);
    const double p_mean = p_sum / tail;
    std::printf("I7: face mdot %.6f kg/s, p_ref - p_amb %.4f Pa, mean p_f - p_amb %.4f Pa vs K rho u^2/2 %.4f Pa "
                "(u %.3f m/s)\n",
                m_sum / tail, f.p_ref_pa - kPAmb, p_mean - kPAmb, dp_ref, u);
    CHECK(rel(m_sum / tail, mdot) < 0.01);
    CHECK(rel(f.p_ref_pa - kPAmb, -dp_ref) < 0.01);
    CHECK(rel(p_mean - kPAmb, dp_ref) < 0.03);
}

TEST_CASE("I5 pipe/waveguide build-time rejections and hash neutrality", "[drivetrain][cycle][pipe]") {
    // Catches: a waveguide end on a non-junction node, two pipe ends on one
    // node, an end correction on the FV side (it belongs to the WG segment),
    // and a network without a waveguide whose hash changed (P8 stays put).
    // expect_message(d, "") means "must not throw"; a non-empty expected
    // string means "must throw std::invalid_argument with exactly this
    // message". CHECK_NOTHROW/CHECK_THROWS_WITH, not try/catch + e.what():
    // catching an exception by reference and then reading it is the
    // clang-cl 19.1.5 Windows ASan crash pattern (vault TOOL-019, CLAUDE.md
    // "Windows ASan caveat") - Catch2's own catch is safe (built with
    // -asan-stack=0).
    auto expect_message = [](const PipeNetworkDesc& d, const std::string& expected) {
        if (expected.empty()) {
            CHECK_NOTHROW(PipeNetwork(d, kHFull));
        } else {
            CHECK_THROWS_WITH(PipeNetwork(d, kHFull), Catch::Matchers::Equals(expected));
        }
    };
    PipeNetworkDesc d;
    d.pipes.push_back(make_pipe("fv", 0.3, 0.04, 0.04, 0.017, closed_end(), wg_end(0)));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face"));
    d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "tail"));
    d.waveguide.segments.push_back(wg_seg("wg", 0.3, 0.04, 0, 1, EndCorrection::Unflanged, WgLoss::None));
    expect_message(d, "");
    PipeNetworkDesc e = d;
    e.pipes[0].end.index = 1;
    expect_message(e, "PipeNetwork: pipe 'fv': a waveguide end must attach to a Junction node");
    e = d;
    e.pipes[0].end.index = 2;
    expect_message(e, "PipeNetwork: pipe 'fv': waveguide node index out of range");
    e = d;
    e.pipes[0].end.end_correction = EndCorrection::Unflanged;
    expect_message(e,
                    "PipeNetwork: pipe 'fv': a waveguide end cannot carry an end correction (the waveguide segment does)");
    e = d;
    e.pipes[0].start = wg_end(0);
    expect_message(e, "PipeNetwork: waveguide node 'face' is attached to more than one pipe end");

    // Without a waveguide the state hash covers exactly the A2 state.
    PipeNetworkDesc plain;
    plain.pipes.push_back(make_pipe("p", 0.3, 0.04, 0.04, 0.017, closed_end(), open_end()));
    PipeNetwork a(plain, kHFull);
    gaussian_pulse(a, 0, 1.0e-3, 0.1, 0.04);
    for (int k = 0; k < 100; ++k) a.step();
    CHECK(a.waveguide().empty());
    CHECK(a.waveguide_face_count() == 0);
    CHECK(a.exchange().waveguide_mass_kg == 0.0);
}

TEST_CASE("I6 FV+WG stepping allocates nothing inside a job and is bit-identical run to run",
          "[drivetrain][cycle][pipe]") {
    // Catches: an allocation in the face or the waveguide step, uninitialised
    // interface state.
    auto build = [] {
        PipeNetworkDesc d;
        d.volumes.push_back(VolumeDesc{"box", 1.0e-3, GasStateDesc{kPAmb, kTAmb, 0.0}});
        d.pipes.push_back(make_pipe("fv", 0.3, 0.04, 0.04, 0.017, volume_end(0), wg_end(0)));
        d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Junction, "face", 5.0e-4));
        d.waveguide.nodes.push_back(wg_node(WgNodeDesc::Kind::Open, "tail"));
        d.waveguide.segments.push_back(wg_seg("wg", 0.4, 0.04, 0, 1, EndCorrection::Unflanged, WgLoss::Kirchhoff));
        return d;
    };
    auto drive = [](PipeNetwork& net, int k) {
        const double h_tot = net.gas().fresh.u(kTAmb) + net.gas().r_mix(0.0) * kTAmb;
        net.set_volume_source(0, 0.02 + 0.01 * std::sin(0.02 * k), h_tot, 0.0);
    };
    std::uint64_t hashes[2] = {};
    for (int run = 0; run < 2; ++run) {
        PipeNetwork net(build(), kHFull);
        for (int k = 0; k < 4000; ++k) {
            drive(net, k);
            net.step();
        }
        hashes[run] = net.fold_state_hash(1469598103934665603ull);
    }
    std::printf("I6 hash %016llx\n", static_cast<unsigned long long>(hashes[0]));
    CHECK(hashes[0] == hashes[1]);

    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    PipeNetwork net(build(), kHFull);
    ps::jobs::JobSystem js(2);
    auto run = [&](int n) {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit(
            [&] {
                for (int k = 0; k < n; ++k) {
                    drive(net, k);
                    net.step();
                }
            },
            &wg);
        js.wait(wg);
    };
    run(10);
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    run(2000);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
}

TEST_CASE("pipe diag: FV numerical dissipation vs cell size (closed-closed mode decay)",
          "[.][diag][drivetrain][cycle][pipe]") {
    // Mode amplitude decay rates of a frictionless closed-closed FV tube (1 m x
    // 20 mm) at three cell sizes, and with laminar friction in a 4 mm tube
    // against 16 mu/(rho D^2).
    for (int variant = 0; variant < 4; ++variant) {
        const int refine = variant < 3 ? (1 << variant) : 2;
        const double dia = variant < 3 ? 0.02 : 0.004;
        const double h = kHFull / refine;
        PipeNetworkDesc d;
        d.pipes.push_back(make_pipe("tube", 1.0, dia, dia, 0.017 / refine, closed_end(), closed_end()));
        if (variant < 3) d.pipes[0].wall.friction = WallFriction::None;
        PipeNetwork net(d, h);
        gaussian_pulse(net, 0, 1.0e-4, 0.03, 0.02);
        const int stride = refine;
        const int n_win = static_cast<int>(0.2 / h) / stride;
        const int gap = static_cast<int>(0.3 / h) / stride;
        std::vector<double> s;
        const int steps = (gap + n_win) * stride;
        for (int k = 0; k < steps; ++k) {
            net.step();
            if (k % stride == 0) s.push_back(net.cell(0, 0).p - kPAmb);
        }
        const double dt = h * stride;
        std::vector<double> w0(s.begin(), s.begin() + n_win), w1(s.begin() + gap, s.begin() + gap + n_win);
        for (std::vector<double>* w : {&w0, &w1}) {
            for (std::size_t k = 0; k < w->size(); ++k)
                (*w)[k] *= 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(w->size() - 1));
        }
        const double c = sound_speed(net, kTAmb, 0.0);
        for (int m : {1, 3, 6, 9, 12}) {
            const double f0 = m * c / 2.0;
            const double f = peak_frequency(std::vector<double>(s.begin(), s.begin() + n_win), dt, 0.97 * f0, 1.03 * f0);
            const double rate = -std::log(dft_mag(w1, dt, f) / dft_mag(w0, dt, f)) / (gap * dt);
            std::printf("FV dissipation %s dx %.2f mm D %.0f mm: mode %2d f %.1f Hz decay %.3f 1/s\n",
                        variant < 3 ? "no friction" : "laminar", 17.0 / refine, dia * 1000.0, m, f, rate);
        }
        if (variant == 3) {
            const double mu = 1.716e-5 * std::pow(kTAmb / 273.0, 1.5) * (273.0 + 111.0) / (kTAmb + 111.0);
            const double rho = kPAmb / (net.gas().r_mix(0.0) * kTAmb);
            std::printf("laminar reference 16 mu/(rho D^2) = %.3f 1/s\n", 16.0 * mu / (rho * dia * dia));
        }
    }
}
