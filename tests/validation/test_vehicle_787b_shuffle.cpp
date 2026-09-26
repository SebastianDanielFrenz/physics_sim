// tests/validation/test_vehicle_787b_shuffle.cpp - driveline shuffle after a
// throttle lift-off on data/vehicles/car_sedan_787b.json (owner request
// 2026-09-26: "the shaft should play a major role in the frequency
// oscillation that can be heard after lifting off the accelerator").
//
// WHAT IS MEASURED. The car (compliance "compliant": an input-shaft spring
// clutch.out-gearbox.in and the two halfshafts lumped into one spring on
// gearbox.out-diff.carrier - see the data file's source field for why) is
// held at 6500 rpm in 4th gear (about 49 m/s) with the clutch locked, the
// assists off and the throttle fixed; a road-load force at the CoM, PI-held
// for 4 s then frozen for 0.5 s, stands in for the inertial/aero load so the
// speed is steady (|dV/dt| < 0.05 m/s^2 is asserted). Then the throttle goes
// to 0 and the road load to 0 in the same tick. The lift-off is a torque
// step on a lightly damped spring-mass chain, so halfshaft torque (the sum
// of both hubs' PowertrainDebug::hub_shaft_torque - the real per-wheel
// shaft torque; PowertrainDebug::shaft_torque[1] is the lumped spring's
// carrier-side torque) and engine rpm ring at the driveline shuffle mode.
//
// ESTIMATOR. A variable-projection least-squares fit, per 240 Hz tick
// sample, of a + b*tau + c*tau^2 + exp(-sigma*tau)*(p*cos(wd*tau) +
// q*sin(wd*tau)) over 0.2 s starting at the first raw extremum after the
// lift (a coarse then a fine grid over fd = wd/2pi and zeta). The quadratic
// trend absorbs the slow speed/slip drift that remains after the lift (a
// linear trend fitted the 830 kg rpm drift instead of the mode).
//
// ANALYTIC PREDICTION (build_model/model_eigenvalues), everything read from
// the loaded VehicleDesc: rotating inertias referred to the driven wheels
// (engine + clutch in/out at (i_g*i_fd)^2; gearbox in at (i_g*i_fd)^2 and
// out at i_fd^2; diff carrier at i_fd^2 plus both driven wheels), the two
// shaft springs/dampers referred the same way, the engine's -d(torque)/d(omega)
// at throttle 0 from the motoring curve's slope, and the tyre as the Maxwell
// element the simulation's relaxation ODE is: spring K/sigma in series with
// damper K/V, K = sum of the driven tyres' dFx/dkappa from the real Pacejka
// model at the measured post-lift (kappa, Fz, V), sigma = relaxation_length_x;
// vehicle mass plus the undriven wheels' J/r^2. Eigenvalues from the
// characteristic polynomial. With Compliance::Rigid the same chain is one
// inertia and has no oscillatory eigenvalue at all.
//
// MEASURED (240 Hz tick, 960 Hz substep, 2026-09-26):
//   mass    throttle  model            halfshaft torque   engine rpm         amp/step  rigid amp/step
//   1500 kg 1.0       25.03 Hz z 0.226 23.87 Hz z 0.240   23.88 Hz z 0.233   0.260     0.0004
//    830 kg 0.5       25.78 Hz z 0.121 25.47 Hz z 0.119   24.80 Hz z 0.107   0.264     0.0013
// Frequency is 1-5 % under the model, zeta within 0.02. Tolerances: 7 % and
// 0.05 absolute. amp/step = fitted oscillation amplitude over the lift-off
// torque step (0.15 asserted); the rigid control's oscillation amplitude is
// 0.04-0.13 % of the step and neither rpm nor hub torque has more than one
// local extremum in 0.2 s (asserted <= 1 and < 2 %; compliant >= 4 extrema).
// The fitted rpm oscillation amplitude at the first extremum is 236 rpm at
// 1500 kg and 81 rpm at 830 kg (the smaller throttle step), on a ~6300 rpm
// overrun.
//
// Where the 1-5 % comes from (the substep/mesh-loss/amplitude diagnostic
// below): refining the substep 960 -> 3840 Hz moves the hub frequency up by
// 0.2-0.45 Hz toward the model (830 kg thr 0.5: 25.67 vs 25.78 Hz, zeta
// 0.121 vs 0.121; 1500 kg thr 0.3: 24.55 vs 24.81 Hz, zeta 0.250 vs 0.246),
// so the remaining 960 Hz offset is discretisation, not model error. At
// 1500 kg WOT the fine-substep zeta stays 0.04 above the model (0.266 vs
// 0.226): the post-lift kappa swing (+0.065 to -0.03) is large and the
// tyre's K is linearised at one point. Setting every mesh efficiency to 1
// changes frequency by <= 0.13 Hz and zeta by <= 0.005 - mesh losses are not
// what damps the mode; the tyre's K/V slip damper is.
//
// DEVIATION FROM THE BRIEF: at 830 kg the passenger tyre
// (passenger_car_225_45r17.json, not the 787B's slicks) cannot hold WOT in
// any gear - the rear wheels spin up onto the 9000 rpm limiter (4th: kappa
// 0.64) - so the 830 kg case uses throttle 0.5 (pre-lift kappa ~0.034).
//
// Not covered: the input-shaft mode (model 180.6 Hz, zeta 0.052) is above
// the 120 Hz Nyquist limit of the per-tick samples.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/force_element.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake"
#endif

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRpmToRadS = 2.0 * kPi / 60.0;

std::string vehicle_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_787b.json"; }

// A force at the chassis CoM opposing its current velocity, magnitude set by
// the test before each World::step() (same write-before-step pattern as
// test_vehicle_skidpad.cpp's TangentialSpeedForce).
class RoadLoad final : public IForceElement {
public:
    explicit RoadLoad(Frame frame) : frame_(frame) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        const real speed = ctx.frame_motion.linear.length();
        if (speed < real(1e-6)) return Wrench{};
        const Vec3 dir = ctx.frame_motion.linear * (real(1.0) / speed);
        return Wrench{dir * (-magnitude_), Vec3::zero()};
    }
    void set_magnitude(real m) { magnitude_ = m; }

private:
    Frame frame_;
    real magnitude_ = 0.0;
};

// ---------------------------------------------------------------------------
// Descriptor lookups (the analytic model reads everything from the loaded
// VehicleDesc - no hand-copied constants).
// ---------------------------------------------------------------------------
template <class T>
const T& component_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const T* p = std::get_if<T>(&c.params)) return *p;
    }
    throw std::runtime_error("component type not found");
}

const drivetrain::ShaftDesc& shaft_between(const vehicle::VehicleDesc& d, const std::string& a, const std::string& b) {
    auto key = [](const drivetrain::PortRef& p) { return p.is_wheel ? "wheel:" + p.component : p.component + "." + p.port; };
    for (const auto& s : d.powertrain.shafts) {
        if (s.ports.size() == 2 && key(s.ports[0]) == a && key(s.ports[1]) == b) return s;
    }
    throw std::runtime_error("shaft not found: " + a + " - " + b);
}

std::size_t wheel_index(const vehicle::VehicleDesc& d, const std::string& name) {
    for (std::size_t i = 0; i < d.wheels.size(); ++i) {
        if (d.wheels[i].name == name) return i;
    }
    throw std::runtime_error("wheel not found: " + name);
}

real curve_slope(const vehicle::Curve1D& c, real x) {
    for (std::size_t i = 1; i < c.x.size(); ++i) {
        if (x <= c.x[i]) return (c.y[i] - c.y[i - 1]) / (c.x[i] - c.x[i - 1]);
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// Linear eigen-analysis: characteristic polynomial (Faddeev-LeVerrier) and
// its roots (Durand-Kerner, then Newton polish).
// ---------------------------------------------------------------------------
using Mat = std::vector<std::vector<real>>;
using cplx = std::complex<real>;

std::vector<real> charpoly(const Mat& a) {
    const std::size_t n = a.size();
    std::vector<real> c(n + 1, 0.0);
    c[0] = 1.0;
    Mat m(n, std::vector<real>(n, 0.0));
    Mat am(n, std::vector<real>(n, 0.0));
    for (std::size_t k = 1; k <= n; ++k) {
        // M_k = A*M_{k-1} + c_{k-1} I, with M_0 = 0
        Mat mk(n, std::vector<real>(n, 0.0));
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) mk[i][j] = am[i][j] + (i == j ? c[k - 1] : 0.0);
        }
        m = mk;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                real s = 0.0;
                for (std::size_t l = 0; l < n; ++l) s += a[i][l] * m[l][j];
                am[i][j] = s;
            }
        }
        real tr = 0.0;
        for (std::size_t i = 0; i < n; ++i) tr += am[i][i];
        c[k] = -tr / static_cast<real>(k);
    }
    return c;
}

std::vector<cplx> poly_roots(const std::vector<real>& c) {
    const std::size_t n = c.size() - 1;
    auto eval = [&](cplx z) {
        cplx p = c[0];
        for (std::size_t k = 1; k <= n; ++k) p = p * z + c[k];
        return p;
    };
    auto deriv = [&](cplx z) {
        cplx p = static_cast<real>(n) * c[0];
        for (std::size_t k = 1; k < n; ++k) p = p * z + static_cast<real>(n - k) * c[k];
        return p;
    };
    // Cauchy bound for the initial circle.
    real bound = 0.0;
    for (std::size_t k = 1; k <= n; ++k) bound = std::max(bound, std::abs(c[k]));
    bound = std::min(real(1.0) + bound, real(1e6));
    real radius = 0.0;
    for (std::size_t k = 1; k <= n; ++k) radius = std::max(radius, std::pow(std::abs(c[k]), real(1.0) / static_cast<real>(k)));
    radius = std::max(radius, real(1.0));
    std::vector<cplx> z(n);
    for (std::size_t i = 0; i < n; ++i) z[i] = std::polar(radius, 2.0 * kPi * static_cast<real>(i) / static_cast<real>(n) + 0.4);
    for (int it = 0; it < 20000; ++it) {
        real change = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            cplx den = 1.0;
            for (std::size_t j = 0; j < n; ++j) {
                if (j != i) den *= (z[i] - z[j]);
            }
            const cplx dz = eval(z[i]) / den;
            z[i] -= dz;
            change = std::max(change, std::abs(dz) / std::max(real(1.0), std::abs(z[i])));
        }
        if (change < 1e-15) break;
    }
    for (auto& zi : z) {
        for (int it = 0; it < 5; ++it) {
            const cplx d = deriv(zi);
            if (std::abs(d) == 0.0) break;
            zi -= eval(zi) / d;
        }
    }
    return z;
}

// ---------------------------------------------------------------------------
// Analytic shuffle model, referred to the driven wheels (in-phase mode, both
// driven wheels lumped). States: omega_i of the rotating masses (n), twist of
// each spring between them (n-1), total driven tyre force F, vehicle speed V.
// Tyre: the simulation's own relaxation ODE d(kappa)/dt = (omega*r - V -
// V*kappa)/sigma with Fx = K*kappa linearised at the operating point is a
// Maxwell element - spring K/sigma in series with damper K/V (K = dFx/dkappa).
// ---------------------------------------------------------------------------
struct ModelInput {
    std::vector<real> j;     // rotating inertias at wheel reference (engine side first, wheels last)
    std::vector<real> k, c;  // springs/dampers between consecutive inertias, wheel reference
    real engine_damping = 0; // -d(engine torque)/d(omega) at wheel reference, on j[0]
    real k_tyre = 0;         // sum dFx/dkappa of the driven tyres (N)
    real sigma = 0;          // relaxation length (m)
    real v = 0;              // speed (m/s)
    real r = 0;              // wheel radius (m)
    real m_eff = 0;          // translational mass incl. undriven wheel spin inertia (kg)
};

std::vector<cplx> model_eigenvalues(const ModelInput& in) {
    const std::size_t n = in.j.size();
    const std::size_t ns = 2 * n + 1;
    Mat a(ns, std::vector<real>(ns, 0.0));
    auto w = [](std::size_t i) { return i; };
    auto th = [n](std::size_t i) { return n + i; };
    const std::size_t fi = 2 * n - 1, vi = 2 * n;
    for (std::size_t i = 0; i + 1 < n; ++i) {
        a[th(i)][w(i)] += 1.0;
        a[th(i)][w(i + 1)] -= 1.0;
        a[w(i)][th(i)] -= in.k[i] / in.j[i];
        a[w(i + 1)][th(i)] += in.k[i] / in.j[i + 1];
        a[w(i)][w(i)] -= in.c[i] / in.j[i];
        a[w(i)][w(i + 1)] += in.c[i] / in.j[i];
        a[w(i + 1)][w(i)] += in.c[i] / in.j[i + 1];
        a[w(i + 1)][w(i + 1)] -= in.c[i] / in.j[i + 1];
    }
    a[w(0)][w(0)] -= in.engine_damping / in.j[0];
    const std::size_t wl = n - 1;
    a[w(wl)][fi] -= in.r / in.j[wl];
    a[fi][w(wl)] += in.k_tyre / in.sigma * in.r;
    a[fi][vi] -= in.k_tyre / in.sigma;
    a[fi][fi] -= in.v / in.sigma;
    a[vi][fi] += 1.0 / in.m_eff;
    return poly_roots(charpoly(a));
}

struct OperatingPoint {
    real v = 0, rpm = 0;
    real fz_driven[2] = {0, 0};
    real kappa_driven[2] = {0, 0};
};

ModelInput build_model(const vehicle::VehicleDesc& d, real mass, int gear, const OperatingPoint& op, bool compliant) {
    const auto& eng = component_of<drivetrain::TorqueMapEngineDesc>(d);
    const auto& cl = component_of<drivetrain::ClutchDesc>(d);
    const auto& gb = component_of<drivetrain::GearboxDesc>(d);
    const auto& df = component_of<drivetrain::DifferentialDesc>(d);
    const real ig = gb.forward_ratios.at(static_cast<std::size_t>(gear - 1));
    const real ifd = df.ratio;
    const real g2 = (ig * ifd) * (ig * ifd);
    const real f2 = ifd * ifd;
    const std::size_t rl = wheel_index(d, "RL"), rr = wheel_index(d, "RR");
    const real r = d.wheels[rl].wheel_radius;

    const real j_engine = (eng.inertia_kgm2 + cl.in_inertia_kgm2 + cl.out_inertia_kgm2) * g2;
    const real j_gearbox = gb.input_inertia_kgm2 * g2 + gb.output_inertia_kgm2 * f2;
    const real j_wheels = df.carrier_inertia_kgm2 * f2 + d.wheels[rl].wheel_inertia + d.wheels[rr].wheel_inertia;

    ModelInput m;
    // Engine: throttle 0 above dfco -> torque = motoring curve; its slope is
    // the linear damping the solver also sees (engine_torque_map.cpp).
    m.engine_damping = -curve_slope(eng.motoring_torque_nm_vs_rpm, op.rpm) / kRpmToRadS * g2;
    if (compliant) {
        const auto& sc = shaft_between(d, "clutch.out", "gearbox.in");
        const auto& sh = shaft_between(d, "gearbox.out", "diff.carrier");
        m.j = {j_engine, j_gearbox, j_wheels};
        m.k = {sc.stiffness * g2, sh.stiffness * f2};
        m.c = {sc.damping * g2, sh.damping * f2};
    } else {
        m.j = {j_engine + j_gearbox + j_wheels};
    }
    auto model = vehicle::create_pacejka_tyre_model(d.wheels[rl].tyre);
    real k_sum = 0.0;
    for (int i = 0; i < 2; ++i) {
        vehicle::TyreSlipInput ti;
        ti.kappa = op.kappa_driven[i];
        ti.fz = op.fz_driven[i];
        ti.vx = op.v;
        k_sum += model->evaluate_with_slope(ti).dfx_dkappa;
    }
    m.k_tyre = k_sum;
    m.sigma = d.wheels[rl].tyre.relaxation_length_x;
    m.v = op.v;
    m.r = r;
    real j_undriven = 0.0;
    for (std::size_t i = 0; i < d.wheels.size(); ++i) {
        if (i != rl && i != rr) j_undriven += d.wheels[i].wheel_inertia / (d.wheels[i].wheel_radius * d.wheels[i].wheel_radius);
    }
    m.m_eff = mass + j_undriven;
    return m;
}

// ---------------------------------------------------------------------------
// Damped-sinusoid fit (variable projection): y(tau) ~ a0 + a1*tau +
// exp(-sigma*tau)*(b1*cos(wd*tau) + b2*sin(wd*tau)), tau = t - t[i0]. For a
// fixed (fd, zeta) the four amplitudes are linear least squares; (fd, zeta)
// come from a coarse grid then a fine grid around the best coarse point.
// ---------------------------------------------------------------------------
struct DampedFit {
    real fd = 0;     // damped frequency (Hz)
    real zeta = 0;   // damping ratio
    real amp = 0;    // oscillation amplitude at tau = 0
    real rss = 0;    // residual sum of squares
    std::size_t i0 = 0, i1 = 0;
};

constexpr int kFitCols = 5; // 1, tau, tau^2, exp*cos, exp*sin

real fit_rss(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1, real fd, real zeta, real* amp) {
    constexpr int n = kFitCols;
    const real wd = 2.0 * kPi * fd;
    const real wn = wd / std::sqrt(1.0 - zeta * zeta);
    const real sigma = zeta * wn;
    auto basis = [&](real tau, real* phi) {
        const real e = std::exp(-sigma * tau);
        phi[0] = 1.0;
        phi[1] = tau;
        phi[2] = tau * tau;
        phi[3] = e * std::cos(wd * tau);
        phi[4] = e * std::sin(wd * tau);
    };
    real m[n][n + 1] = {};
    for (std::size_t i = i0; i < i1; ++i) {
        real phi[n];
        basis(t[i] - t[i0], phi);
        for (int r = 0; r < n; ++r) {
            m[r][n] += phi[r] * y[i];
            for (int c = 0; c < n; ++c) m[r][c] += phi[r] * phi[c];
        }
    }
    // Gauss-Jordan with partial pivoting on the normal equations.
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r) {
            if (std::abs(m[r][col]) > std::abs(m[piv][col])) piv = r;
        }
        for (int c = 0; c <= n; ++c) std::swap(m[col][c], m[piv][c]);
        if (std::abs(m[col][col]) < 1e-300) return 1e300;
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const real f = m[r][col] / m[col][col];
            for (int c = col; c <= n; ++c) m[r][c] -= f * m[col][c];
        }
    }
    real coef[n];
    for (int r = 0; r < n; ++r) coef[r] = m[r][n] / m[r][r];
    real rss = 0.0;
    for (std::size_t i = i0; i < i1; ++i) {
        real phi[n];
        basis(t[i] - t[i0], phi);
        real model = 0.0;
        for (int c = 0; c < n; ++c) model += coef[c] * phi[c];
        rss += (y[i] - model) * (y[i] - model);
    }
    if (amp != nullptr) *amp = std::sqrt(coef[3] * coef[3] + coef[4] * coef[4]);
    return rss;
}

// Index of the first raw local extremum at or after `from`.
std::size_t first_extremum(const std::vector<real>& y, std::size_t from) {
    for (std::size_t i = std::max<std::size_t>(from, 1); i + 1 < y.size(); ++i) {
        if ((y[i] >= y[i - 1] && y[i] >= y[i + 1]) || (y[i] <= y[i - 1] && y[i] <= y[i + 1])) return i;
    }
    return from;
}

DampedFit fit_damped(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1) {
    DampedFit best;
    best.rss = 1e300;
    best.i0 = i0;
    best.i1 = i1;
    auto scan = [&](real f_lo, real f_hi, real f_step, real z_lo, real z_hi, real z_step) {
        DampedFit b = best;
        for (real f = f_lo; f <= f_hi + 1e-12; f += f_step) {
            for (real z = z_lo; z <= z_hi + 1e-12; z += z_step) {
                real amp = 0.0;
                const real rss = fit_rss(t, y, i0, i1, f, z, &amp);
                if (rss < b.rss) {
                    b.rss = rss;
                    b.fd = f;
                    b.zeta = z;
                    b.amp = amp;
                }
            }
        }
        best = b;
    };
    scan(5.0, 60.0, 0.5, 0.01, 0.80, 0.01);
    scan(std::max(real(1.0), best.fd - 0.6), best.fd + 0.6, 0.01, std::max(real(0.001), best.zeta - 0.012), std::min(real(0.95), best.zeta + 0.012), 0.0005);
    return best;
}

// ---------------------------------------------------------------------------
// One lift-off run.
// ---------------------------------------------------------------------------
struct LiftRun {
    real dt = 0;
    std::size_t lift = 0; // sample index of the first post-lift tick
    std::vector<real> t, rpm, shaft_h, hub_l, hub_r, speed, fz_l, fz_r, kappa_l, kappa_r, clutch_t, load, dfco;
    bool clutch_stuck_throughout = true;
    bool limiter_seen = false;
    real road_load = 0;
    real hold_accel = 0; // mean dV/dt over the last 0.25 s before the lift
};

LiftRun run_lift(const vehicle::VehicleDesc& d, real mass, int gear, real rpm0, real throttle, real post_s, real substep_hz = 960.0) {
    auto rig = ps_test::build_vehicle_rig_from_desc(d, mass, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, 4, Vec3::unit_y(), 240.0, substep_hz);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    const auto& gb = component_of<drivetrain::GearboxDesc>(d);
    const auto& df = component_of<drivetrain::DifferentialDesc>(d);
    const std::size_t rl = wheel_index(d, "RL"), rr = wheel_index(d, "RR");
    const real r = d.wheels[rl].wheel_radius;
    const real v0 = rpm0 * kRpmToRadS / (gb.forward_ratios.at(static_cast<std::size_t>(gear - 1)) * df.ratio) * r;

    w.backend().set_motion(rig.chassis, Motion{Vec3{v0, 0.0, 0.0}, Vec3::zero()});
    vehicle::VehicleResetOptions opts;
    opts.gear = gear;
    opts.clutch_locked = true;
    opts.hubs_roll_with_chassis = true;
    w.reset_vehicle(rig.vehicle, opts);
    w.set_control("gear_request", drivetrain::kGearRequestNone);
    w.set_control("clutch", 0.0);
    w.set_control("assist.auto_clutch", 0.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("throttle", throttle);

    RoadLoad load(Frame(rig.chassis));
    const ForceElementId load_id = w.add_force_element(&load);

    const real dt = w.dt();
    // Hold phase: PI on the road load keeps the speed at v0 while the engine
    // runs at `throttle` (the road load stands in for the inertial force of
    // a car accelerating at that throttle). Then the load is frozen.
    const real kp = 4.0 * mass, ki = 4.0 * mass;
    real integ = 0.0;
    const int hold_ticks = static_cast<int>(4.0 / dt + 0.5);
    const int freeze_ticks = static_cast<int>(0.5 / dt + 0.5);
    real f = 0.0;
    for (int i = 0; i < hold_ticks; ++i) {
        const real v = w.get_motion(rig.chassis).linear.x;
        integ += ki * (v - v0) * dt;
        f = std::max(real(0.0), integ + kp * (v - v0));
        load.set_magnitude(f);
        w.step();
    }
    load.set_magnitude(f);
    real v_start = 0.0;
    for (int i = 0; i < freeze_ticks; ++i) {
        if (i == freeze_ticks - static_cast<int>(0.25 / dt + 0.5)) v_start = w.get_motion(rig.chassis).linear.x;
        w.step();
    }
    LiftRun out;
    out.dt = dt;
    out.road_load = f;
    out.hold_accel = (w.get_motion(rig.chassis).linear.x - v_start) / 0.25;

    auto record = [&](real t) {
        const auto& st = w.powertrain_state(rig.vehicle);
        const auto dbg = w.powertrain_debug(rig.vehicle);
        out.t.push_back(t);
        out.rpm.push_back(st.engine_rpm);
        out.shaft_h.push_back(dbg.shaft_torque[1]);
        out.hub_l.push_back(dbg.hub_shaft_torque[rl]);
        out.hub_r.push_back(dbg.hub_shaft_torque[rr]);
        out.clutch_t.push_back(dbg.clutch_torque);
        out.speed.push_back(w.get_motion(rig.chassis).linear.x);
        out.fz_l.push_back(w.wheel_telemetry(rig.vehicle, rl).fz);
        out.fz_r.push_back(w.wheel_telemetry(rig.vehicle, rr).fz);
        out.kappa_l.push_back(w.wheel_telemetry(rig.vehicle, rl).slip_ratio);
        out.kappa_r.push_back(w.wheel_telemetry(rig.vehicle, rr).slip_ratio);
        out.load.push_back(st.engines.empty() ? 0.0 : st.engines[0].load);
        out.dfco.push_back(!st.engines.empty() && st.engines[0].dfco_active ? 1.0 : 0.0);
        if (!dbg.clutch_stuck) out.clutch_stuck_throughout = false;
        if (!st.engines.empty() && st.engines[0].limiter_active) out.limiter_seen = true;
    };
    const int pre_ticks = static_cast<int>(0.2 / dt + 0.5);
    for (int i = 0; i < pre_ticks; ++i) {
        w.step();
        record(static_cast<real>(i - pre_ticks + 1) * dt);
    }
    out.lift = out.t.size();
    w.set_control("throttle", 0.0);
    load.set_magnitude(0.0);
    const int post_ticks = static_cast<int>(post_s / dt + 0.5);
    for (int i = 0; i < post_ticks; ++i) {
        w.step();
        record(static_cast<real>(i + 1) * dt);
    }
    w.remove_force_element(load_id);
    return out;
}

void write_csv(const LiftRun& r, const std::string& path) {
    std::ofstream f(path);
    if (!f) return;
    f << "t,engine_rpm,hub_torque_RL,hub_torque_RR,lumped_shaft_torque,clutch_torque,engine_load,dfco,speed,fz_RL,fz_RR,kappa_RL,kappa_RR\n";
    char line[512];
    for (std::size_t i = 0; i < r.t.size(); ++i) {
        std::snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.0f,%.6f,%.3f,%.3f,%.6f,%.6f\n", r.t[i], r.rpm[i], r.hub_l[i], r.hub_r[i],
                      r.shaft_h[i], r.clutch_t[i], r.load[i], r.dfco[i], r.speed[i], r.fz_l[i], r.fz_r[i], r.kappa_l[i], r.kappa_r[i]);
        f << line;
    }
}

OperatingPoint op_after_lift(const LiftRun& r) {
    OperatingPoint op;
    const std::size_t k = r.lift + static_cast<std::size_t>(0.1 / r.dt + 0.5);
    op.v = r.speed[k];
    op.rpm = r.rpm[k];
    op.fz_driven[0] = r.fz_l[k];
    op.fz_driven[1] = r.fz_r[k];
    op.kappa_driven[0] = r.kappa_l[k];
    op.kappa_driven[1] = r.kappa_r[k];
    return op;
}

std::vector<real> hub_sum(const LiftRun& r) {
    std::vector<real> out(r.t.size());
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = r.hub_l[i] + r.hub_r[i];
    return out;
}

DampedFit fit_after_lift(const LiftRun& r, const std::vector<real>& y, real window_s) {
    const std::size_t i0 = first_extremum(y, r.lift);
    const std::size_t i1 = std::min(y.size(), i0 + static_cast<std::size_t>(window_s / r.dt + 0.5));
    return fit_damped(r.t, y, i0, i1);
}

// Strict local extrema of y in (from, to).
int count_extrema(const std::vector<real>& y, std::size_t from, std::size_t to) {
    int n = 0;
    for (std::size_t i = from + 1; i + 1 < std::min(to, y.size()); ++i) {
        if ((y[i] > y[i - 1] && y[i] > y[i + 1]) || (y[i] < y[i - 1] && y[i] < y[i + 1])) ++n;
    }
    return n;
}

// The model's modes between 5 and 100 Hz (the shuffle); fd = 0 if none.
struct ModelMode {
    real fd = 0, zeta = 0;
    int count = 0;
};
ModelMode shuffle_mode(const std::vector<cplx>& ev) {
    ModelMode m;
    for (const auto& e : ev) {
        const real fd = e.imag() / (2.0 * kPi);
        if (fd > 5.0 && fd < 100.0) {
            ++m.count;
            m.fd = fd;
            m.zeta = -e.real() / std::abs(e);
        }
    }
    return m;
}

struct ShuffleResult {
    ModelMode model_compliant, model_rigid;
    DampedFit hub, rpm, hub_rigid;
    real hub_step = 0, hub_step_rigid = 0;
    int rpm_extrema = 0, rpm_extrema_rigid = 0, hub_extrema = 0, hub_extrema_rigid = 0;
    LiftRun compliant, rigid;
};

constexpr real kFitWindowS = 0.2;
constexpr real kExtremaWindowS = 0.2;

ShuffleResult measure_shuffle(real mass, int gear, real rpm0, real throttle) {
    const vehicle::VehicleDesc d = io::load_vehicle_json(vehicle_path());
    vehicle::VehicleDesc rigid = d;
    rigid.powertrain.compliance = drivetrain::Compliance::Rigid;
    ShuffleResult res;
    res.compliant = run_lift(d, mass, gear, rpm0, throttle, 1.0);
    res.rigid = run_lift(rigid, mass, gear, rpm0, throttle, 1.0);
    res.model_compliant = shuffle_mode(model_eigenvalues(build_model(d, mass, gear, op_after_lift(res.compliant), true)));
    res.model_rigid = shuffle_mode(model_eigenvalues(build_model(d, mass, gear, op_after_lift(res.rigid), false)));

    const std::vector<real> hub = hub_sum(res.compliant);
    const std::vector<real> hub_r = hub_sum(res.rigid);
    res.hub = fit_after_lift(res.compliant, hub, kFitWindowS);
    res.rpm = fit_after_lift(res.compliant, res.compliant.rpm, kFitWindowS);
    res.hub_rigid = fit_after_lift(res.rigid, hub_r, kFitWindowS);
    const std::size_t settle = static_cast<std::size_t>(0.3 / res.compliant.dt + 0.5);
    res.hub_step = std::abs(hub[res.compliant.lift - 1] - hub[res.compliant.lift + settle]);
    res.hub_step_rigid = std::abs(hub_r[res.rigid.lift - 1] - hub_r[res.rigid.lift + settle]);
    const std::size_t win = static_cast<std::size_t>(kExtremaWindowS / res.compliant.dt + 0.5);
    res.rpm_extrema = count_extrema(res.compliant.rpm, res.compliant.lift, res.compliant.lift + win);
    res.rpm_extrema_rigid = count_extrema(res.rigid.rpm, res.rigid.lift, res.rigid.lift + win);
    res.hub_extrema = count_extrema(hub, res.compliant.lift, res.compliant.lift + win);
    res.hub_extrema_rigid = count_extrema(hub_r, res.rigid.lift, res.rigid.lift + win);

    std::printf("787B shuffle m=%.0f kg gear %d throttle %.2f: hold road load %.0f N, hold accel %.4f m/s^2\n", mass, gear, throttle,
                res.compliant.road_load, res.compliant.hold_accel);
    std::printf("  model (compliant): %.2f Hz zeta %.3f (%d mode(s) in 5-100 Hz); model (rigid): %d mode(s)\n", res.model_compliant.fd,
                res.model_compliant.zeta, res.model_compliant.count, res.model_rigid.count);
    std::printf("  halfshaft torque (hub sum): %.2f Hz zeta %.3f amp %.1f N*m (step %.1f N*m, ratio %.3f), extrema %d\n", res.hub.fd, res.hub.zeta,
                res.hub.amp, res.hub_step, res.hub.amp / res.hub_step, res.hub_extrema);
    std::printf("  engine rpm: %.2f Hz zeta %.3f amp %.1f rpm, extrema %d\n", res.rpm.fd, res.rpm.zeta, res.rpm.amp, res.rpm_extrema);
    std::printf("  rigid control: hub amp %.2f N*m (step %.1f, ratio %.4f), hub extrema %d, rpm extrema %d\n", res.hub_rigid.amp, res.hub_step_rigid,
                res.hub_rigid.amp / res.hub_step_rigid, res.hub_extrema_rigid, res.rpm_extrema_rigid);
    return res;
}

// Tolerances (measured with margin at the production 240/960 Hz, see the
// header): frequency within 7 % of the model, zeta within 0.05 absolute.
constexpr real kFreqRelTol = 0.07;
constexpr real kZetaAbsTol = 0.05;

void check_shuffle(const ShuffleResult& r) {
    // Operating-point sanity: a locked clutch, no limiter, a held speed.
    CHECK(r.compliant.clutch_stuck_throughout);
    CHECK(r.rigid.clutch_stuck_throughout);
    CHECK_FALSE(r.compliant.limiter_seen);
    CHECK_FALSE(r.rigid.limiter_seen);
    CHECK(std::abs(r.compliant.hold_accel) < 0.05);

    // The analytic model: one shuffle mode with the compliant shafts, none rigid.
    REQUIRE(r.model_compliant.count == 1);
    CHECK(r.model_rigid.count == 0);

    // Halfshaft torque and engine rpm ring at the model's mode.
    CHECK(std::abs(r.hub.fd / r.model_compliant.fd - 1.0) <= kFreqRelTol);
    CHECK(std::abs(r.hub.zeta - r.model_compliant.zeta) <= kZetaAbsTol);
    CHECK(std::abs(r.rpm.fd / r.model_compliant.fd - 1.0) <= kFreqRelTol);
    CHECK(std::abs(r.rpm.zeta - r.model_compliant.zeta) <= kZetaAbsTol);
    CHECK(r.hub.amp / r.hub_step > 0.15);
    CHECK(r.hub_extrema >= 4);
    CHECK(r.rpm_extrema >= 4);

    // Rigid control: the mode is gone.
    CHECK(r.hub_rigid.amp / r.hub_step_rigid < 0.02);
    CHECK(r.hub_extrema_rigid <= 1);
    CHECK(r.rpm_extrema_rigid <= 1);
}

vehicle::VehicleDesc lossless_copy(const vehicle::VehicleDesc& d) {
    vehicle::VehicleDesc out = d;
    for (auto& c : out.powertrain.components) {
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) {
            for (auto& e : g->forward_efficiency) e = 1.0;
        }
        if (auto* df = std::get_if<drivetrain::DifferentialDesc>(&c.params)) df->efficiency = 1.0;
    }
    return out;
}

} // namespace

TEST_CASE("787B lift-off shuffle at 1500 kg (Godot demo mass) in 4th at WOT matches the analytic driveline mode and the rigid control has none",
          "[validation][vehicle][powertrain][drivetrain]") {
    const ShuffleResult r = measure_shuffle(1500.0, 4, 6500.0, 1.0);
    check_shuffle(r);
}

TEST_CASE("787B lift-off shuffle at 830 kg (real car) in 4th at throttle 0.5 matches the analytic driveline mode and the rigid control has none",
          "[validation][vehicle][powertrain][drivetrain]") {
    // Throttle 0.5, not WOT: at 830 kg on the passenger tyre WOT spins the
    // rear wheels onto the limiter in every gear (see the header).
    const ShuffleResult r = measure_shuffle(830.0, 4, 6500.0, 0.5);
    check_shuffle(r);
}

TEST_CASE("787B lift-off shuffle: CSV of the lift-off window (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    for (const real mass : {830.0, 1500.0}) {
        const ShuffleResult r = measure_shuffle(mass, 4, 6500.0, mass < 1000.0 ? 0.5 : 1.0);
        char name[512];
        std::snprintf(name, sizeof(name), "%s/out/787b_shuffle_liftoff_%d.csv", PS_SOURCE_DIR, static_cast<int>(mass));
        write_csv(r.compliant, name);
        std::snprintf(name, sizeof(name), "%s/out/787b_shuffle_liftoff_%d_rigid.csv", PS_SOURCE_DIR, static_cast<int>(mass));
        write_csv(r.rigid, name);
        std::printf("  wrote %s/out/787b_shuffle_liftoff_%d[_rigid].csv\n", PS_SOURCE_DIR, static_cast<int>(mass));
    }
}

TEST_CASE("787B lift-off shuffle: substep convergence and mesh-loss/amplitude sweep (diagnostic)",
          "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc d = io::load_vehicle_json(vehicle_path());
    const vehicle::VehicleDesc lossless = lossless_copy(d);
    struct Pt {
        real mass;
        real thr;
    };
    for (const Pt pt : {Pt{1500.0, 1.0}, Pt{1500.0, 0.3}, Pt{830.0, 0.5}, Pt{830.0, 0.25}}) {
        for (const real sub : {960.0, 1920.0, 3840.0}) {
            for (int variant = 0; variant < 2; ++variant) {
                const vehicle::VehicleDesc& desc = variant == 0 ? d : lossless;
                const LiftRun r = run_lift(desc, pt.mass, 4, 6500.0, pt.thr, 1.0, sub);
                const ModelMode m = shuffle_mode(model_eigenvalues(build_model(desc, pt.mass, 4, op_after_lift(r), true)));
                const std::vector<real> hub = hub_sum(r);
                const DampedFit fh = fit_after_lift(r, hub, kFitWindowS);
                const DampedFit fr = fit_after_lift(r, r.rpm, kFitWindowS);
                std::printf("m=%.0f thr=%.2f substep=%.0f %-8s | model %.2f Hz z %.3f | hub %.2f Hz z %.3f | rpm %.2f Hz z %.3f\n", pt.mass, pt.thr,
                            sub, variant == 0 ? "as-is" : "lossless", m.fd, m.zeta, fh.fd, fh.zeta, fr.fd, fr.zeta);
            }
        }
    }
}
