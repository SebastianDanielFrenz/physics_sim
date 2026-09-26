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
//
// RING-DURATION SWEEP ([.][diag], 2026-09-26; owner: "compliant and rigid
// WAVs sound the same", a real 787B's rpm wiggles for ~500 ms, which needs
// zeta*omega <= ~6 1/s). "parameter sweep" / "best-case combinations" mutate
// the loaded VehicleDesc/TyreDesc in code one factor at a time and print,
// per config, the rpm/hub fits, A0/A250/A500 = max |rpm_compliant -
// rpm_rigid| in windows around 0/250/500 ms, the model mode and a modal-
// damping breakdown (zeta with the engine slope, shaft dampers, tyre
// removed one at a time, plus the perturbation estimates zeta_tyre =
// c_eff*phi2^2/(2wM), zeta_shaft = c*(1-phi2)^2/(2wM), zeta_eng = d/(2wM),
// M = J1*(1 + J1/J2), phi2 = -J1/J2 - which under-predict the tyre share
// because c_t/(w*J2) ~ 1 there). Baseline 1500 kg WOT 4th: zeta 0.226 =
// tyre ~0.145 + shaft dampers ~0.075 + engine slope ~0.006 + mesh loss ~0;
// e-fold 28 ms, A250 0.5 rpm, A500 0.02 rpm. No single plausible factor
// gets zeta under ~0.10 (6th gear at 8500 rpm 0.108, sigma_x 0.4 0.100, 830
// kg on mu 1.7 slicks 0.12-0.14, engine inertia 0.03 0.21, shaft c x0.1
// 0.163). Only shaft c x0.1 AND halfshaft k x0.5 on a light car in 6th at
// 8500 rpm reach zeta 0.04 (e-fold ~155 ms, A500 ~44-49 rpm).
// "verification checks": the A250/A500 of the c x0.1 configs at 240 Hz tick
// are the ~181 Hz input-shaft mode aliased to 76.5 Hz (163.5 Hz at a 960 Hz
// tick, same ~11 rpm), not the shuffle - such configs are run at 960 Hz
// tick. 1st gear at 14 m/s, throttle 0.25, genuinely rings (hub 18.8 Hz
// zeta 0.053, A500 44 rpm at both ticks): the tyre damper K*r^2/V is small
// against the reflected inertia there. "lumped lash experiment": the model
// chain as a nonlinear RK4 ODE with a halfshaft dead zone of 0-8 deg - lash
// adds a clunk at the lift but leaves A500 < 0.6 rpm (no 500 ms ring). Its
// lash-0 difference fit reproduces the model at 830 kg (25.73/0.124 vs
// 25.78/0.121) and in 6th (35.74/0.123 vs 35.68/0.115); the 1500 kg WOT
// fits bracket it (rpm 0.160, difference 0.299 vs 0.226 - only ~2 usable
// cycles). "sweep CSVs for listening" writes compliant + Rigid CSVs to
// PS_787B_SWEEP_OUT (else <source>/out/787b_sweep) for rpm_to_wav.py.

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
#include <cstdlib>
#include <filesystem>
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

DampedFit fit_damped(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1, real f_lo = 5.0, real f_hi = 60.0,
                     real f_step = 0.5) {
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
    scan(f_lo, f_hi, f_step, 0.01, 0.80, 0.01);
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
    std::vector<real> pitch_rate; // chassis angular velocity about world y (sweep diagnostic only, not in the CSV)
    std::vector<real> shaft_in;   // input-shaft spring torque, PowertrainDebug::shaft_torque[0] (sweep diagnostic only)
    bool clutch_stuck_throughout = true;
    bool limiter_seen = false;
    real road_load = 0;
    real hold_accel = 0; // mean dV/dt over the last 0.25 s before the lift
};

LiftRun run_lift(const vehicle::VehicleDesc& d, real mass, int gear, real rpm0, real throttle, real post_s, real substep_hz = 960.0,
                 real tick_hz = 240.0) {
    auto rig = ps_test::build_vehicle_rig_from_desc(d, mass, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, 4, Vec3::unit_y(), tick_hz, substep_hz);
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
        out.shaft_in.push_back(dbg.shaft_torque[0]);
        out.hub_l.push_back(dbg.hub_shaft_torque[rl]);
        out.hub_r.push_back(dbg.hub_shaft_torque[rr]);
        out.clutch_t.push_back(dbg.clutch_torque);
        out.speed.push_back(w.get_motion(rig.chassis).linear.x);
        out.pitch_rate.push_back(w.get_motion(rig.chassis).angular.y);
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

// The model mode (5-100 Hz) whose damped frequency is closest to f_ref, and
// how many there are. The tyre's relaxation state adds a second, heavily
// damped oscillatory mode in some configurations; shuffle_mode() above keeps
// only the last one found, which is not always the shuffle.
ModelMode mode_near(const std::vector<cplx>& ev, real f_ref) {
    ModelMode m;
    real best = 1e300;
    for (const auto& e : ev) {
        const real fd = e.imag() / (2.0 * kPi);
        if (fd > 5.0 && fd < 100.0) {
            ++m.count;
            if (std::abs(fd - f_ref) < best) {
                best = std::abs(fd - f_ref);
                m.fd = fd;
                m.zeta = -e.real() / std::abs(e);
            }
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

// ---------------------------------------------------------------------------
// Parameter sweep (diagnostic, 2026-09-26): which parameters set the
// shuffle's damping and duration. Every mutation is applied to the loaded
// desc in code; no data file is touched.
// ---------------------------------------------------------------------------
struct SweepConfig {
    const char* name = "";
    real mass = 1500.0;
    int gear = 4;
    real rpm0 = 6500.0;
    real throttle = 1.0;
    real pkx1_scale = 1.0;           // longitudinal slip stiffness (all four tyres)
    real sigma_x = 0.0;              // relaxation_length_x, 0 = keep (0.10 m)
    real lambda_mux = 0.0;           // longitudinal friction scale, 0 = keep (1.0)
    real engine_inertia = 0.0;       // kg*m^2, 0 = keep (0.06)
    real shaft_c_scale = 1.0;        // damping of both compliant shafts
    real halfshaft_k_scale = 1.0;    // stiffness of the lumped halfshaft spring
    real motoring_slope_scale = 1.0; // motoring curve slope above 3000 rpm, value at rpm0 kept
    bool lossless = false;           // every mesh efficiency 1
    real tick_hz = 240.0;            // World tick; substep = max(960, tick)
};

vehicle::VehicleDesc apply_config(const vehicle::VehicleDesc& base, const SweepConfig& c) {
    vehicle::VehicleDesc d = c.lossless ? lossless_copy(base) : base;
    for (auto& wh : d.wheels) {
        wh.tyre.pkx1 *= c.pkx1_scale;
        if (c.sigma_x > 0.0) wh.tyre.relaxation_length_x = c.sigma_x;
        if (c.lambda_mux > 0.0) wh.tyre.lambda_mux = c.lambda_mux;
    }
    for (auto& comp : d.powertrain.components) {
        if (auto* e = std::get_if<drivetrain::TorqueMapEngineDesc>(&comp.params)) {
            if (c.engine_inertia > 0.0) e->inertia_kgm2 = c.engine_inertia;
            if (c.motoring_slope_scale != 1.0) {
                // Scale the slope about the operating point only (x >= 3000
                // rpm), so the engine-braking torque at the lift is unchanged
                // and motoring(0) (the breakaway check) is untouched.
                auto& m = e->motoring_torque_nm_vs_rpm;
                const real y0 = m.evaluate(c.rpm0);
                for (std::size_t i = 0; i < m.x.size(); ++i) {
                    if (m.x[i] >= 3000.0) m.y[i] = y0 + c.motoring_slope_scale * (m.y[i] - y0);
                }
            }
        }
    }
    for (auto& s : d.powertrain.shafts) {
        if (s.stiffness <= 0.0) continue;
        s.damping *= c.shaft_c_scale;
        if (s.ports.size() == 2 && s.ports[0].component == "gearbox" && s.ports[1].component == "diff") s.stiffness *= c.halfshaft_k_scale;
    }
    return d;
}

// Modal damping breakdown on the analytic model: the eigenvalue with each
// damping source removed, plus a two-mass closed form (engine side J1 =
// engine + gearbox, wheel side J2, halfshaft spring k; tyre as a dashpot to
// ground, the Maxwell element's dissipative part c_t/(1 + (w*sigma/V)^2),
// c_t = K*r^2/V) - zeta_i = c_i * (relative modal velocity)^2 / (2*w*M)
// with the free-free mode shape (1, -J1/J2), M = J1*(1 + J1/J2).
struct ZetaBreakdown {
    real full = 0, no_engine = 0, no_shaft = 0, tyre_only = 0;
    real an_f = 0, an_tyre = 0, an_shaft = 0, an_engine = 0, an_input_shaft = 0;
    real c_tyre = 0, c_tyre_eff = 0, c_shaft = 0, d_engine = 0, j1 = 0, j2 = 0;
};

ZetaBreakdown zeta_breakdown(const ModelInput& m, real f_ref) {
    ZetaBreakdown b;
    const ModelMode full = mode_near(model_eigenvalues(m), f_ref);
    b.full = full.zeta;
    ModelInput t = m;
    t.engine_damping = 0.0;
    b.no_engine = mode_near(model_eigenvalues(t), full.fd).zeta;
    t = m;
    for (auto& c : t.c) c = 0.0;
    b.no_shaft = mode_near(model_eigenvalues(t), full.fd).zeta;
    t.engine_damping = 0.0;
    b.tyre_only = mode_near(model_eigenvalues(t), full.fd).zeta;
    if (m.j.size() == 3) {
        b.j1 = m.j[0] + m.j[1];
        b.j2 = m.j[2];
        const real w = std::sqrt(m.k[1] * (1.0 / b.j1 + 1.0 / b.j2));
        const real phi2 = -b.j1 / b.j2;
        const real mm = b.j1 * (1.0 + b.j1 / b.j2);
        b.c_tyre = m.k_tyre * m.r * m.r / m.v;
        b.c_tyre_eff = b.c_tyre / (1.0 + (w * m.sigma / m.v) * (w * m.sigma / m.v));
        b.c_shaft = m.c[1];
        b.d_engine = m.engine_damping;
        b.an_f = w / (2.0 * kPi);
        b.an_tyre = b.c_tyre_eff * phi2 * phi2 / (2.0 * w * mm);
        b.an_shaft = m.c[1] * (1.0 - phi2) * (1.0 - phi2) / (2.0 * w * mm);
        b.an_engine = m.engine_damping / (2.0 * w * mm);
        // Input shaft: relative motion across it in this mode from the
        // quasi-static twist, (J_gearbox_side inertia force)/k_input.
        const real rel = (m.j[0] * w * w / m.k[0]);
        b.an_input_shaft = m.c[0] * rel * rel / (2.0 * w * mm);
    }
    return b;
}

// Largest |y| over the samples whose time lies in [t_lo, t_hi] after the lift.
real max_abs_in(const LiftRun& r, const std::vector<real>& y, real t_lo, real t_hi) {
    real m = 0.0;
    for (std::size_t i = r.lift; i < y.size(); ++i) {
        const real t = r.t[i];
        if (t >= t_lo - 1e-9 && t <= t_hi + 1e-9) m = std::max(m, std::abs(y[i]));
    }
    return m;
}

// Least-squares quadratic over [i0, i1): max |residual| (a bound on any
// oscillation slower than the window, not a frequency estimate).
real quad_resid_max(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1) {
    real m[3][4] = {};
    for (std::size_t i = i0; i < i1; ++i) {
        const real tau = t[i] - t[i0];
        const real phi[3] = {1.0, tau, tau * tau};
        for (int r = 0; r < 3; ++r) {
            m[r][3] += phi[r] * y[i];
            for (int c = 0; c < 3; ++c) m[r][c] += phi[r] * phi[c];
        }
    }
    for (int col = 0; col < 3; ++col) {
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const real f = m[r][col] / m[col][col];
            for (int c = col; c < 4; ++c) m[r][c] -= f * m[col][c];
        }
    }
    const real a = m[0][3] / m[0][0], b = m[1][3] / m[1][1], c = m[2][3] / m[2][2];
    real worst = 0.0;
    for (std::size_t i = i0; i < i1; ++i) {
        const real tau = t[i] - t[i0];
        worst = std::max(worst, std::abs(y[i] - (a + b * tau + c * tau * tau)));
    }
    return worst;
}

struct SweepRow {
    SweepConfig cfg;
    ModelMode model;
    ZetaBreakdown bd;
    DampedFit rpm, rpm_long, hub;
    real a_lift = 0, a250 = 0, a500 = 0;  // max |rpm_compliant - rpm_rigid| in +-30 ms windows
    real fit_a250 = 0, fit_a500 = 0;      // rpm fit envelope amp*exp(-sigma*tau) there
    real kappa_pre = 0, v0 = 0, rpm_lift = 0;
    bool limiter = false, clutch_stuck = true;
    real hold_accel = 0;
    real rigid_lowfreq = 0;               // rigid run: max |rpm - quadratic| over 0.15..1.0 s after the lift
    DampedFit pitch, rpm_low;             // rigid run, 0.15..1.0 s, 0.5-10 Hz: chassis pitch rate and engine rpm
    LiftRun compliant, rigid;
};

SweepRow run_config(const SweepConfig& c) {
    const vehicle::VehicleDesc base = io::load_vehicle_json(vehicle_path());
    const vehicle::VehicleDesc d = apply_config(base, c);
    vehicle::VehicleDesc rigid = d;
    rigid.powertrain.compliance = drivetrain::Compliance::Rigid;
    SweepRow row;
    row.cfg = c;
    const real sub = std::max(real(960.0), c.tick_hz);
    row.compliant = run_lift(d, c.mass, c.gear, c.rpm0, c.throttle, 1.0, sub, c.tick_hz);
    row.rigid = run_lift(rigid, c.mass, c.gear, c.rpm0, c.throttle, 1.0, sub, c.tick_hz);
    const LiftRun& rc = row.compliant;
    const LiftRun& rr = row.rigid;
    const ModelInput mi = build_model(d, c.mass, c.gear, op_after_lift(rc), true);
    row.hub = fit_after_lift(rc, hub_sum(rc), kFitWindowS);
    row.model = mode_near(model_eigenvalues(mi), row.hub.fd);
    row.bd = zeta_breakdown(mi, row.hub.fd);
    {
        // rpm fitted in a band around the hub-torque mode, so a slower
        // component cannot capture the fit.
        const std::size_t i0 = first_extremum(rc.rpm, rc.lift);
        const real lo = std::max(real(5.0), 0.7 * row.hub.fd), hi = 1.3 * row.hub.fd;
        row.rpm = fit_damped(rc.t, rc.rpm, i0, std::min(rc.rpm.size(), i0 + static_cast<std::size_t>(kFitWindowS / rc.dt + 0.5)), lo, hi, 0.1);
        row.rpm_long = fit_damped(rc.t, rc.rpm, i0, std::min(rc.rpm.size(), i0 + static_cast<std::size_t>(0.5 / rc.dt + 0.5)), lo, hi, 0.1);
    }
    std::vector<real> diff(rc.rpm.size());
    for (std::size_t i = 0; i < diff.size(); ++i) diff[i] = rc.rpm[i] - rr.rpm[i];
    row.a_lift = max_abs_in(rc, diff, 0.0, 0.05);
    row.a250 = max_abs_in(rc, diff, 0.22, 0.28);
    row.a500 = max_abs_in(rc, diff, 0.47, 0.53);
    {
        const real wn = 2.0 * kPi * row.rpm.fd / std::sqrt(1.0 - row.rpm.zeta * row.rpm.zeta);
        const real sig = row.rpm.zeta * wn;
        const real t0 = rc.t[row.rpm.i0];
        row.fit_a250 = row.rpm.amp * std::exp(-sig * (0.25 - t0));
        row.fit_a500 = row.rpm.amp * std::exp(-sig * (0.50 - t0));
    }
    row.kappa_pre = 0.5 * (rc.kappa_l[rc.lift - 1] + rc.kappa_r[rc.lift - 1]);
    row.v0 = rc.speed[rc.lift - 1];
    row.rpm_lift = rc.rpm[rc.lift - 1];
    row.limiter = rc.limiter_seen || rr.limiter_seen;
    row.clutch_stuck = rc.clutch_stuck_throughout && rr.clutch_stuck_throughout;
    row.hold_accel = rc.hold_accel;
    const std::size_t i0 = rr.lift + static_cast<std::size_t>(0.15 / rr.dt + 0.5);
    row.rigid_lowfreq = quad_resid_max(rr.t, rr.rpm, i0, rr.rpm.size());
    row.pitch = fit_damped(rr.t, rr.pitch_rate, i0, rr.pitch_rate.size(), 0.5, 10.0, 0.1);
    row.rpm_low = fit_damped(rr.t, rr.rpm, i0, rr.rpm.size(), 0.5, 10.0, 0.1);
    return row;
}

void print_sweep_header() {
    std::printf("%-26s %5s %2s %5s %4s %6s %6s | %6s %6s | %6s %6s %6s | %6s %6s | %6s %6s %6s | %6s %6s | %6s %6s %6s %6s | %5s %5s %5s %5s %5s | %5s %5s\n", "config",
                "mass", "g", "rpm", "thr", "v", "kpre", "mod f", "mod z", "rpm f", "rpm z", "tau", "hub f", "hub z", "A0", "A250", "A500", "fA250",
                "fA500", "z_eig", "-eng", "-shft", "tyre", "an_f", "an_ty", "an_sh", "an_en", "an_in", "rLF", "pitF");
    std::printf("  (second line per config: model mode count, 0.5 s rpm fit, rigid-run pitch-rate fit f/zeta/amp and rigid-run rpm fit in 0.5-10 Hz)\n");
}

void print_sweep_row(const SweepRow& r) {
    const real wn = 2.0 * kPi * r.rpm.fd / std::sqrt(1.0 - r.rpm.zeta * r.rpm.zeta);
    const real tau_ms = 1000.0 / (r.rpm.zeta * wn);
    std::printf("%-26s %5.0f %2d %5.0f %4.2f %6.2f %6.4f | %6.2f %6.3f | %6.2f %6.3f %6.1f | %6.2f %6.3f | %6.1f %6.1f %6.2f | %6.2f %6.3f | %6.3f %6.3f %6.3f %6.3f | %5.1f %5.3f %5.3f "
                "%5.3f %5.3f | %5.1f %5.2f%s%s\n",
                r.cfg.name, r.cfg.mass, r.cfg.gear, r.rpm_lift, r.cfg.throttle, r.v0, r.kappa_pre, r.model.fd, r.model.zeta, r.rpm.fd, r.rpm.zeta, tau_ms,
                r.hub.fd, r.hub.zeta, r.a_lift, r.a250, r.a500, r.fit_a250, r.fit_a500, r.bd.full, r.bd.no_engine, r.bd.no_shaft, r.bd.tyre_only, r.bd.an_f,
                r.bd.an_tyre, r.bd.an_shaft, r.bd.an_engine, r.bd.an_input_shaft, r.rigid_lowfreq, r.pitch.fd, r.limiter ? " LIMITER" : "",
                r.clutch_stuck ? "" : " CLUTCH-SLIP");
    std::printf("    modes %d | rpm 0.5s %.2f Hz z %.3f amp %.1f | rigid pitch %.2f Hz z %.3f amp %.5f rad/s | rigid rpm low %.2f Hz z %.3f amp %.2f rpm | "
                "c_t %.1f c_eff %.1f c_hs %.1f d_eng %.2f J1 %.3f J2 %.3f (wheel ref)\n",
                r.model.count, r.rpm_long.fd, r.rpm_long.zeta, r.rpm_long.amp, r.pitch.fd, r.pitch.zeta, r.pitch.amp, r.rpm_low.fd, r.rpm_low.zeta,
                r.rpm_low.amp, r.bd.c_tyre, r.bd.c_tyre_eff, r.bd.c_shaft, r.bd.d_engine, r.bd.j1, r.bd.j2);
}

std::vector<SweepConfig> sweep_configs() {
    std::vector<SweepConfig> v;
    auto add = [&](const char* name, auto mut) {
        SweepConfig c;
        c.name = name;
        mut(c);
        v.push_back(c);
    };
    // Baselines (the asserted cases).
    add("base_1500_wot", [](SweepConfig&) {});
    add("base_830_thr0.5", [](SweepConfig& c) { c.mass = 830.0; c.throttle = 0.5; });
    // 1. Operating point.
    add("gear1_thr0.25", [](SweepConfig& c) { c.gear = 1; c.throttle = 0.25; });
    add("gear2_thr0.4", [](SweepConfig& c) { c.gear = 2; c.throttle = 0.4; });
    add("gear3", [](SweepConfig& c) { c.gear = 3; c.throttle = 0.7; });
    add("gear5", [](SweepConfig& c) { c.gear = 5; });
    add("gear6", [](SweepConfig& c) { c.gear = 6; });
    add("gear4_5000rpm", [](SweepConfig& c) { c.rpm0 = 5000.0; });
    add("gear4_8500rpm", [](SweepConfig& c) { c.rpm0 = 8500.0; });
    add("gear5_8500rpm", [](SweepConfig& c) { c.gear = 5; c.rpm0 = 8500.0; });
    add("gear6_8500rpm", [](SweepConfig& c) { c.gear = 6; c.rpm0 = 8500.0; });
    // 2. Tyre.
    add("pkx1_x1.5", [](SweepConfig& c) { c.pkx1_scale = 1.5; });
    add("pkx1_x2", [](SweepConfig& c) { c.pkx1_scale = 2.0; });
    add("pkx1_x0.5", [](SweepConfig& c) { c.pkx1_scale = 0.5; });
    add("sigma_0.05", [](SweepConfig& c) { c.sigma_x = 0.05; });
    add("sigma_0.20", [](SweepConfig& c) { c.sigma_x = 0.20; });
    add("sigma_0.40", [](SweepConfig& c) { c.sigma_x = 0.40; });
    add("pkx1_x2_sigma_0.20", [](SweepConfig& c) { c.pkx1_scale = 2.0; c.sigma_x = 0.20; });
    add("mu1.7_1500_wot", [](SweepConfig& c) { c.lambda_mux = 1.7; });
    add("mu1.7_830_wot", [](SweepConfig& c) { c.mass = 830.0; c.lambda_mux = 1.7; });
    // 3. Engine inertia.
    add("Je_0.03", [](SweepConfig& c) { c.engine_inertia = 0.03; });
    add("Je_0.02", [](SweepConfig& c) { c.engine_inertia = 0.02; });
    // 4. Shafts.
    add("shaft_c_x0.1", [](SweepConfig& c) { c.shaft_c_scale = 0.1; });
    add("halfshaft_k_x0.5", [](SweepConfig& c) { c.halfshaft_k_scale = 0.5; });
    add("shaft_c0.1_k0.5", [](SweepConfig& c) { c.shaft_c_scale = 0.1; c.halfshaft_k_scale = 0.5; });
    // 5. Engine motoring slope.
    add("motoring_slope_0", [](SweepConfig& c) { c.motoring_slope_scale = 0.0; });
    add("motoring_slope_x2", [](SweepConfig& c) { c.motoring_slope_scale = 2.0; });
    add("lossless_meshes", [](SweepConfig& c) { c.lossless = true; });
    // 6. Mass (same throttle as base).
    add("mass_830_wot", [](SweepConfig& c) { c.mass = 830.0; });
    return v;
}

// ---------------------------------------------------------------------------
// Verification helpers (diagnostic).
// ---------------------------------------------------------------------------
struct SpecPeak {
    real f = 0, amp = 0;
};

// Peak of a Hann-windowed DFT of y (linear trend removed) over [i0, i1),
// scanned from f_lo to f_hi. amp is the amplitude a steady sinusoid of that
// frequency would read (window-corrected); a decaying tone reads lower.
SpecPeak spectrum_peak(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1, real f_lo, real f_hi, real f_step) {
    const std::size_t n = i1 - i0;
    real st = 0, sy = 0, stt = 0, sty = 0;
    for (std::size_t i = i0; i < i1; ++i) {
        const real tau = t[i] - t[i0];
        st += tau;
        sy += y[i];
        stt += tau * tau;
        sty += tau * y[i];
    }
    const real nn = static_cast<real>(n);
    const real b = (nn * sty - st * sy) / (nn * stt - st * st);
    const real a = (sy - b * st) / nn;
    SpecPeak best;
    for (real f = f_lo; f <= f_hi + 1e-12; f += f_step) {
        real re = 0, im = 0, wsum = 0;
        for (std::size_t i = i0; i < i1; ++i) {
            const real tau = t[i] - t[i0];
            const real w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<real>(i - i0) / static_cast<real>(n - 1));
            const real v = (y[i] - (a + b * tau)) * w;
            re += v * std::cos(2.0 * kPi * f * tau);
            im += v * std::sin(2.0 * kPi * f * tau);
            wsum += w;
        }
        const real amp = 2.0 * std::sqrt(re * re + im * im) / wsum;
        if (amp > best.amp) {
            best.amp = amp;
            best.f = f;
        }
    }
    return best;
}

// Zero crossings of y minus its end-to-end line over [i0, i1), as a frequency.
real zero_cross_freq(const std::vector<real>& t, const std::vector<real>& y, std::size_t i0, std::size_t i1) {
    const real a = y[i0], b = (y[i1 - 1] - y[i0]) / (t[i1 - 1] - t[i0]);
    real mean = 0;
    for (std::size_t i = i0; i < i1; ++i) mean += y[i] - (a + b * (t[i] - t[i0]));
    mean /= static_cast<real>(i1 - i0);
    int n = 0;
    for (std::size_t i = i0 + 1; i < i1; ++i) {
        const real p = y[i - 1] - (a + b * (t[i - 1] - t[i0])) - mean;
        const real q = y[i] - (a + b * (t[i] - t[i0])) - mean;
        if ((p < 0.0) != (q < 0.0)) ++n;
    }
    return 0.5 * n / (t[i1 - 1] - t[i0]);
}

void print_modes(const std::vector<cplx>& ev) {
    for (const auto& e : ev) {
        if (e.imag() > 1e-6) std::printf(" [%.2f Hz z %.3f]", e.imag() / (2.0 * kPi), -e.real() / std::abs(e));
    }
    for (const auto& e : ev) {
        if (std::abs(e.imag()) <= 1e-6) std::printf(" [real %.2f 1/s]", e.real());
    }
}

// Lumped lash experiment (diagnostic, NOT a model of the simulation, which
// has no lash): the analytic chain of build_model() (engine+clutch | gearbox
// | wheels, input-shaft and lumped halfshaft springs, engine slope damper,
// tyre Maxwell element, vehicle mass) integrated as a nonlinear ODE (RK4,
// 20 us) with a symmetric dead zone of total width `lash` (rad at the wheel)
// in the halfshaft spring - spring and damper both act only in contact.
// Starts in the pre-lift equilibrium (t_drive through every spring, road
// load = tyre force, dV/dt = 0); at t = 0 the engine torque steps to t_after
// (plus the slope damper about the lift speed) and the road load to 0, as
// run_lift() does. A rigid reference (one inertia, same tyre/vehicle) gives
// the rpm the compliant chain is compared against.
struct LashOut {
    std::vector<real> t, rpm_c, rpm_r;
    real a0 = 0, a250 = 0, a500 = 0;
    DampedFit fit;       // rpm, default 5-60 Hz band
    DampedFit fit_band;  // rpm, 0.7-1.3x f_ref
    DampedFit fit_diff;  // rpm_c - rpm_r, 0.7-1.3x f_ref (no common slow trend)
    int gap_entries = 0;
    real last_gap_t = -1.0;
};

LashOut simulate_lash(const ModelInput& m, real ratio, real t_drive, real t_after, real lash, real v0, real f_ref) {
    const std::size_t n = m.j.size();
    const std::size_t ns = 2 * n + 1; // w[n], th[n-1], F, V
    const real h = 0.5 * lash;
    const real f0 = t_drive / m.r;
    const real w0 = v0 * (1.0 + f0 / m.k_tyre) / m.r;
    std::vector<real> x(ns, 0.0);
    for (std::size_t i = 0; i < n; ++i) x[i] = w0;
    for (std::size_t i = 0; i + 1 < n; ++i) x[n + i] = t_drive / m.k[i] + (i + 2 == n ? h : 0.0);
    x[2 * n - 1] = f0;
    x[2 * n] = v0;
    std::vector<real> xr = {w0, f0, v0}; // rigid: W, F, V
    real jsum = 0;
    for (real jj : m.j) jsum += jj;
    auto spring = [&](const std::vector<real>& s, std::size_t i, bool* contact) {
        const real th = s[n + i];
        const real dv = s[i] - s[i + 1];
        if (i + 2 != n || h <= 0.0) {
            if (contact) *contact = true;
            return m.k[i] * th + m.c[i] * dv;
        }
        real e = 0.0;
        if (th > h) e = th - h;
        else if (th < -h) e = th + h;
        if (contact) *contact = e != 0.0;
        return e != 0.0 ? m.k[i] * e + m.c[i] * dv : real(0.0);
    };
    auto deriv = [&](const std::vector<real>& s) {
        std::vector<real> ds(ns, 0.0);
        std::vector<real> tq(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) tq[i] = spring(s, i, nullptr);
        const real te = t_after - m.engine_damping * (s[0] - w0);
        const real f = s[2 * n - 1], v = s[2 * n];
        for (std::size_t i = 0; i < n; ++i) {
            real tsum = (i == 0 ? te : tq[i - 1]);
            if (i + 1 < n) tsum -= tq[i];
            else tsum -= m.r * f;
            ds[i] = tsum / m.j[i];
        }
        for (std::size_t i = 0; i + 1 < n; ++i) ds[n + i] = s[i] - s[i + 1];
        ds[2 * n - 1] = m.k_tyre / m.sigma * (m.r * s[n - 1] - v) - v / m.sigma * f;
        ds[2 * n] = f / m.m_eff;
        return ds;
    };
    auto deriv_r = [&](const std::vector<real>& s) {
        const real te = t_after - m.engine_damping * (s[0] - w0);
        return std::vector<real>{(te - m.r * s[1]) / jsum, m.k_tyre / m.sigma * (m.r * s[0] - s[2]) - s[2] / m.sigma * s[1], s[1] / m.m_eff};
    };
    auto rk4 = [](std::vector<real>& s, real dt, const auto& fn) {
        const auto k1 = fn(s);
        std::vector<real> tmp(s.size());
        for (std::size_t i = 0; i < s.size(); ++i) tmp[i] = s[i] + 0.5 * dt * k1[i];
        const auto k2 = fn(tmp);
        for (std::size_t i = 0; i < s.size(); ++i) tmp[i] = s[i] + 0.5 * dt * k2[i];
        const auto k3 = fn(tmp);
        for (std::size_t i = 0; i < s.size(); ++i) tmp[i] = s[i] + dt * k3[i];
        const auto k4 = fn(tmp);
        for (std::size_t i = 0; i < s.size(); ++i) s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    };
    LashOut out;
    const real dt = 2e-5;
    const int per_sample = static_cast<int>(1.0 / 240.0 / dt + 0.5);
    const real to_rpm = ratio / kRpmToRadS;
    bool in_gap = false;
    out.t.push_back(0.0);
    out.rpm_c.push_back(x[0] * to_rpm);
    out.rpm_r.push_back(xr[0] * to_rpm);
    for (int s = 1; s <= 240; ++s) {
        for (int k = 0; k < per_sample; ++k) {
            rk4(x, dt, deriv);
            rk4(xr, dt, deriv_r);
            bool contact = true;
            spring(x, n - 2, &contact);
            if (!contact && !in_gap) ++out.gap_entries;
            in_gap = !contact;
            if (in_gap) out.last_gap_t = (s - 1) / 240.0 + (k + 1) * dt;
        }
        out.t.push_back(s / 240.0);
        out.rpm_c.push_back(x[0] * to_rpm);
        out.rpm_r.push_back(xr[0] * to_rpm);
    }
    auto win = [&](real lo, real hi) {
        real a = 0;
        for (std::size_t i = 0; i < out.t.size(); ++i) {
            if (out.t[i] >= lo - 1e-9 && out.t[i] <= hi + 1e-9) a = std::max(a, std::abs(out.rpm_c[i] - out.rpm_r[i]));
        }
        return a;
    };
    out.a0 = win(0.0, 0.05);
    out.a250 = win(0.22, 0.28);
    out.a500 = win(0.47, 0.53);
    const std::size_t i0 = first_extremum(out.rpm_c, 1);
    const std::size_t i1 = std::min(out.t.size(), i0 + static_cast<std::size_t>(kFitWindowS * 240.0 + 0.5));
    out.fit = fit_damped(out.t, out.rpm_c, i0, i1);
    out.fit_band = fit_damped(out.t, out.rpm_c, i0, i1, 0.7 * f_ref, 1.3 * f_ref, 0.1);
    std::vector<real> diff(out.t.size());
    for (std::size_t i = 0; i < diff.size(); ++i) diff[i] = out.rpm_c[i] - out.rpm_r[i];
    const std::size_t j0 = first_extremum(diff, 1);
    out.fit_diff = fit_damped(out.t, diff, j0, std::min(out.t.size(), j0 + static_cast<std::size_t>(kFitWindowS * 240.0 + 0.5)), 0.7 * f_ref, 1.3 * f_ref, 0.1);
    return out;
}

// Sweep CSV output dir: PS_787B_SWEEP_OUT, else <source>/out/787b_sweep.
std::string sweep_out_dir() {
    std::string env;
#ifdef _WIN32
    char* e = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&e, &len, "PS_787B_SWEEP_OUT") == 0 && e != nullptr) env = e;
    std::free(e);
#else
    if (const char* e = std::getenv("PS_787B_SWEEP_OUT")) env = e;
#endif
    std::string dir = !env.empty() ? env : std::string(PS_SOURCE_DIR) + "/out/787b_sweep";
    std::filesystem::create_directories(dir);
    return dir;
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

// Parameter sweep (2026-09-26, owner: "compliant and rigid WAVs sound the
// same"): one factor at a time from the 1500 kg WOT baseline, then
// combinations. Columns: operating point (rpm/v/pre-lift kappa at the lift),
// the n-mass model's mode, the fitted rpm mode (0.2 s window) and its e-fold
// time tau = 1/(zeta*omega_n) in ms, the hub-torque fit, A0/A250/A500 = max
// |rpm_compliant - rpm_rigid| within 0-50 ms / 220-280 ms / 470-530 ms after
// the lift, fA250/fA500 = the fitted rpm envelope extrapolated there, the
// model's zeta with each damping source removed (-eng = no engine slope,
// -shft = no shaft dampers, tyre = neither), the two-mass closed form (an_f,
// an_ty/an_sh/an_en/an_in = zeta shares of tyre / halfshaft damper / engine
// slope / input-shaft damper), rLF = the RIGID run's max |rpm - quadratic|
// over 0.05-1.0 s (any slow oscillation, e.g. chassis pitch, would show
// here) and pitF = the rigid run's chassis pitch-rate zero-crossing
// frequency (Hz, crude).
TEST_CASE("787B lift-off shuffle: parameter sweep (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    print_sweep_header();
    for (const SweepConfig& c : sweep_configs()) print_sweep_row(run_config(c));
}

// Combinations toward the longest plausible ring (same columns).
TEST_CASE("787B lift-off shuffle: best-case combinations (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    std::vector<SweepConfig> v;
    auto add = [&](SweepConfig c) { v.push_back(c); };
    SweepConfig c;
    c = SweepConfig{};
    c.name = "830_mu1.7_g6_8500";
    c.mass = 830.0; c.lambda_mux = 1.7; c.gear = 6; c.rpm0 = 8500.0;
    add(c);
    c.name = "..+Je0.03";
    c.engine_inertia = 0.03;
    add(c);
    c.name = "..+c0.1";
    c.shaft_c_scale = 0.1;
    add(c);
    c.name = "..+k0.5";
    c.halfshaft_k_scale = 0.5;
    add(c);
    c.name = "..+slick pkx1x2 sig0.2";
    c.pkx1_scale = 2.0; c.sigma_x = 0.2;
    add(c);
    c.name = "..+lossless";
    c.lossless = true;
    add(c);
    c = SweepConfig{};
    c.name = "1500_g4_c0.1_k0.5_Je0.03";
    c.shaft_c_scale = 0.1; c.halfshaft_k_scale = 0.5; c.engine_inertia = 0.03;
    add(c);
    c = SweepConfig{};
    c.name = "1500_g4_sigma0.4_c0.1";
    c.sigma_x = 0.4; c.shaft_c_scale = 0.1;
    add(c);
    print_sweep_header();
    for (const SweepConfig& s : v) print_sweep_row(run_config(s));
}

// Verification of two sweep anomalies (diagnostic):
// (1) shaft_c_x0.1 leaves 10-20 rpm of compliant-minus-rigid rpm at 250/500
//     ms although its fitted shuffle envelope is < 1 rpm there. Hypothesis:
//     the input-shaft mode (model ~180 Hz) is under-damped at 0.1x c and
//     aliases into the 240 Hz tick samples. Discriminator: the same config
//     at a 960 Hz tick (960 Hz substep); the residual's spectral peak moves
//     to the true frequency (above the 120 Hz Nyquist of 240 Hz) if so.
// (2) gear 1/2 fits disagree with the model's shuffle mode: every model
//     mode, the pre-lift kappa, the free hub fit and a hub fit constrained
//     to 0.7-1.3x the model's closest mode are printed, at both ticks.
TEST_CASE("787B lift-off shuffle: verification checks (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    std::vector<SweepConfig> v;
    for (const real tick : {240.0, 960.0}) {
        SweepConfig c;
        c.tick_hz = tick;
        c.name = "base_1500_wot";
        v.push_back(c);
        c.name = "shaft_c_x0.1";
        c.shaft_c_scale = 0.1;
        v.push_back(c);
        c = SweepConfig{};
        c.tick_hz = tick;
        c.name = "gear1_thr0.25";
        c.gear = 1;
        c.throttle = 0.25;
        v.push_back(c);
        c.name = "gear2_thr0.4";
        c.gear = 2;
        c.throttle = 0.4;
        v.push_back(c);
    }
    for (const SweepConfig& c : v) {
        const SweepRow r = run_config(c);
        const LiftRun& rc = r.compliant;
        const LiftRun& rr = r.rigid;
        std::vector<real> diff(rc.rpm.size());
        for (std::size_t i = 0; i < diff.size(); ++i) diff[i] = rc.rpm[i] - rr.rpm[i];
        const std::size_t a = rc.lift + static_cast<std::size_t>(0.2 / rc.dt + 0.5), b = rc.lift + static_cast<std::size_t>(0.6 / rc.dt + 0.5);
        const real nyq = 0.5 * c.tick_hz;
        const SpecPeak pd = spectrum_peak(rc.t, diff, a, b, 2.0, nyq - 1.0, 0.25);
        const SpecPeak pin = spectrum_peak(rc.t, rc.shaft_in, a, b, 2.0, nyq - 1.0, 0.25);
        const SpecPeak pd_lo = spectrum_peak(rc.t, diff, a, b, 2.0, 60.0, 0.25);
        const std::vector<real> hub = hub_sum(rc);
        const DampedFit hub_c = [&] {
            const std::size_t i0 = first_extremum(hub, rc.lift);
            const std::size_t i1 = std::min(hub.size(), i0 + static_cast<std::size_t>(kFitWindowS / rc.dt + 0.5));
            return fit_damped(rc.t, hub, i0, i1, std::max(real(3.0), 0.7 * r.model.fd), 1.3 * r.model.fd, 0.1);
        }();
        std::printf("%-15s tick %4.0f | v %.2f rpm %.0f kpre %.4f%s | A0 %.1f A250 %.2f A500 %.2f rpm | diff 0.2-0.6s: peak %.2f Hz amp %.2f rpm, zc %.1f Hz, "
                    "peak<60Hz %.2f Hz amp %.2f | input-shaft torque peak %.2f Hz amp %.2f N*m\n",
                    c.name, c.tick_hz, r.v0, r.rpm_lift, r.kappa_pre, r.limiter ? " LIMITER" : "", r.a_lift, r.a250, r.a500, pd.f, pd.amp,
                    zero_cross_freq(rc.t, diff, a, b), pd_lo.f, pd_lo.amp, pin.f, pin.amp);
        std::printf("    hub fit free %.2f Hz z %.3f amp %.1f | hub fit near model %.2f Hz z %.3f amp %.1f | rpm fit %.2f Hz z %.3f | model modes:", r.hub.fd,
                    r.hub.zeta, r.hub.amp, hub_c.fd, hub_c.zeta, hub_c.amp, r.rpm.fd, r.rpm.zeta);
        const vehicle::VehicleDesc d = apply_config(io::load_vehicle_json(vehicle_path()), c);
        print_modes(model_eigenvalues(build_model(d, c.mass, c.gear, op_after_lift(rc), true)));
        std::printf("\n");
    }
}

// Lumped lash experiment (diagnostic; simulate_lash() above - an ODE on the
// analytic chain, not the simulation, which has no lash). Operating point,
// tyre K and drive torque from a real compliant run of each config; lash is
// the TOTAL driveline lash referred to the wheel, in degrees (assumed range,
// no cited 787B value). lash 0 must reproduce the linear model (sanity row).
TEST_CASE("787B lift-off shuffle: lumped lash experiment (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    std::vector<SweepConfig> v;
    SweepConfig c;
    c.name = "base_1500_wot";
    v.push_back(c);
    c.name = "base_830_thr0.5";
    c.mass = 830.0;
    c.throttle = 0.5;
    v.push_back(c);
    c = SweepConfig{};
    c.name = "830_mu1.7_g6_8500_Je0.03";
    c.mass = 830.0;
    c.lambda_mux = 1.7;
    c.gear = 6;
    c.rpm0 = 8500.0;
    c.engine_inertia = 0.03;
    v.push_back(c);
    for (const SweepConfig& cfg : v) {
        const vehicle::VehicleDesc d = apply_config(io::load_vehicle_json(vehicle_path()), cfg);
        const LiftRun r = run_lift(d, cfg.mass, cfg.gear, cfg.rpm0, cfg.throttle, 1.0);
        const ModelInput m = build_model(d, cfg.mass, cfg.gear, op_after_lift(r), true);
        const auto& eng = component_of<drivetrain::TorqueMapEngineDesc>(d);
        const auto& gb = component_of<drivetrain::GearboxDesc>(d);
        const auto& df = component_of<drivetrain::DifferentialDesc>(d);
        const real ratio = gb.forward_ratios.at(static_cast<std::size_t>(cfg.gear - 1)) * df.ratio;
        const real t_drive = std::abs(r.hub_l[r.lift - 1] + r.hub_r[r.lift - 1]);
        const real rpm_lift = r.rpm[r.lift - 1];
        const real t_after = eng.motoring_torque_nm_vs_rpm.evaluate(rpm_lift) * ratio;
        const real v0 = r.speed[r.lift - 1];
        const ModelMode mm = mode_near(model_eigenvalues(m), 25.0);
        std::printf("%s: v0 %.2f m/s rpm %.0f, drive %.1f N*m and overrun %.1f N*m at the wheels (motoring curve x ratio %.3f, mesh loss ignored), "
                    "linear model %.2f Hz z %.3f\n",
                    cfg.name, v0, rpm_lift, t_drive, t_after, ratio, mm.fd, mm.zeta);
        for (const real lash_deg : {0.0, 0.5, 1.0, 2.0, 4.0, 8.0}) {
            const LashOut o = simulate_lash(m, ratio, t_drive, t_after, lash_deg * kPi / 180.0, v0, mm.fd);
            std::printf("  lash %4.1f deg | fit %.2f Hz z %.3f amp %.1f rpm | band fit %.2f Hz z %.3f | diff fit %.2f Hz z %.3f | A0 %.1f A250 %.2f A500 %.2f rpm | gap entries %d, last in gap at %.3f s\n", lash_deg,
                        o.fit.fd, o.fit.zeta, o.fit.amp, o.fit_band.fd, o.fit_band.zeta, o.fit_diff.fd, o.fit_diff.zeta, o.a0, o.a250, o.a500, o.gap_entries, o.last_gap_t);
        }
    }
}

// CSVs for listening (diagnostic): the configs the sweep found most
// informative, compliant plus the matching Rigid control, same columns as
// the CSV case above. Written to PS_787B_SWEEP_OUT (else
// <source>/out/787b_sweep); render with tools/plots/rpm_to_wav.py.
TEST_CASE("787B lift-off shuffle: sweep CSVs for listening (diagnostic)", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    const std::string dir = sweep_out_dir();
    std::vector<SweepConfig> v;
    SweepConfig c;
    c.name = "base_1500_g4_wot";
    v.push_back(c);
    c = SweepConfig{};
    c.name = "light_830_g6_8500_mu1.7_Je0.03";
    c.mass = 830.0;
    c.lambda_mux = 1.7;
    c.gear = 6;
    c.rpm0 = 8500.0;
    c.engine_inertia = 0.03;
    v.push_back(c);
    c.name = "light_830_g6_8500_mu1.7_Je0.03_c0.1_k0.5";
    c.shaft_c_scale = 0.1;
    c.halfshaft_k_scale = 0.5;
    c.tick_hz = 960.0;
    v.push_back(c);
    c = SweepConfig{};
    c.name = "base_1500_g1_thr0.25";
    c.gear = 1;
    c.throttle = 0.25;
    v.push_back(c);
    for (const SweepConfig& s : v) {
        const SweepRow r = run_config(s);
        write_csv(r.compliant, dir + "/" + s.name + ".csv");
        write_csv(r.rigid, dir + "/" + s.name + "_rigid.csv");
        std::printf("  wrote %s/%s[_rigid].csv (tick %.0f Hz): rpm fit %.2f Hz z %.3f, A0 %.1f A250 %.2f A500 %.2f rpm\n", dir.c_str(), s.name, s.tick_hz,
                    r.rpm.fd, r.rpm.zeta, r.a_lift, r.a250, r.a500);
    }
}
