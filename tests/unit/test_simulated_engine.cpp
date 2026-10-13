// tests/unit/test_simulated_engine.cpp -- PHYS-033 P4 I7b C11: white-box,
// standalone tests of ps::drivetrain::SimulatedEngine (the mean-value
// engine model, core/src/drivetrain/engine_simulated.h/.cpp) against the
// in-memory fixtures in tests/support/mvem_fixtures.h. No Powertrain: the
// engine is driven directly (pre_solve/post_solve) either at a HELD crank
// speed (a dynamometer) or on its own crank inertia with a test-level
// linearly-implicit step omega += h*(tau0 - c_k)/(J + h*d) (the Coulomb
// row's kinetic capacity while turning forward; omega clamped at 0 as a
// stand-in for the row's stick at rest).
//
// I7 plan test table (section "I7b"): T1 steady fixed point, T2 manifold
// time constant (+ the explicit omega/p_im coupling at the smallest
// plenum, plan section 3.3 [A]), T4a electronic pedal law, T4b mechanical
// plate path, T5 indicated work closure, T6 dtau/domega, T7 state machine /
// DFCO / limiter / idle PI; plus the starvation factor (EngineTorqueMap's
// test_fuel_tank.cpp T10 counterpart), literal tau0/d pins, the hash layout
// and zero allocation.
//
// The reference model below (RefModel) is the test's OWN evaluation of the
// plan section 3.1 equations from the desc's tables with typed-in
// constants (R = 287.05, T = 288.15 K, p = 101325 Pa, c_cycle = 1/2); only
// the tabulated flow function Psi is taken from the engine (engine.psi()),
// which is itself checked against std::pow separately.

#include "drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"

#include "mvem_fixtures.h"

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;
constexpr real kRT = 287.05 * 288.15;
constexpr real kPamb = 101325.0;

real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }
real rpm_of(real w) { return std::abs(w) * 60.0 / (2.0 * kPi); }
real rel(real a, real b) { return std::abs(a - b) / std::max(std::abs(b), 1e-300); }

const std::shared_ptr<const SimulatedEngineMaps>& elec_maps() {
    static const auto m = SimulatedEngine::build_maps(fx::electronic_engine());
    return m;
}
const std::shared_ptr<const SimulatedEngineMaps>& mech_maps() {
    static const auto m = SimulatedEngine::build_maps(fx::mechanical_engine());
    return m;
}

Sensors running(real omega) {
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    return s;
}

ThrottleCommand cmd(real c) { return ThrottleCommand{c, 0.0, 1.0}; }

// Hold the crank at omega for n substeps; returns the last PreSolveResult.
PreSolveResult hold(SimulatedEngine& e, real omega, real c, int n, real h = kH, real ignition = 1.0) {
    PreSolveResult r;
    Sensors s = running(omega);
    s.ignition = ignition;
    for (int i = 0; i < n; ++i) {
        r = e.pre_solve(s, cmd(c), TcuRequest{}, h);
        (void)e.post_solve(omega, h);
    }
    return r;
}

// One substep of the engine alone on its own crank inertia J.
real crank_step(SimulatedEngine& e, real omega, real c, real J, real h, PreSolveResult* out = nullptr, real starter = 0.0) {
    Sensors s = running(omega);
    s.starter = starter;
    const PreSolveResult r = e.pre_solve(s, cmd(c), TcuRequest{}, h);
    real w = omega + h * (r.tau0 - r.friction_cap_kinetic) / (J + h * r.d);
    if (w < 0.0) w = 0.0;
    (void)e.post_solve(w, h);
    if (out) *out = r;
    return w;
}

// The test's own plan section 3.1 evaluation.
struct RefModel {
    SimulatedEngineDesc d;
    const SimulatedEngine* eng; // only for the tabulated Psi
    real vd, a_full, c = 0.5;

    RefModel(SimulatedEngineDesc desc, const SimulatedEngine* e) : d(std::move(desc)), eng(e) {
        vd = real(d.piston.cylinders) * kPi / 4.0 * d.piston.bore_m * d.piston.bore_m * d.piston.stroke_m;
        a_full = real(d.throttle_body.count) * kPi / 4.0 * d.throttle_body.diameter_m * d.throttle_body.diameter_m;
    }
    real m_thr(real p, real x, real xiac) const {
        const real bypass = d.throttle_input == ThrottleInput::Mechanical ? d.idle_bypass_area_m2 : 0.0;
        const real a = d.throttle_body.discharge_coefficient * a_full * d.throttle_body.area_fraction_vs_position.evaluate(x) + xiac * bypass;
        const real k = a / std::sqrt(kRT);
        return p <= kPamb ? k * kPamb * eng->psi(p / kPamb) : -k * p * eng->psi(kPamb / p);
    }
    real m_air_rev(real rpm, real p) const { return d.maps.ve.value(rpm, p) * p / kRT * vd * c; }
    real m_cyl(real p, real omega) const { return m_air_rev(rpm_of(omega), p) * std::max(omega, 0.0) / (2.0 * kPi); }
    real f(real p, real omega, real x, real xiac) const { return kRT / d.intake_manifold_volume_m3 * (m_thr(p, x, xiac) - m_cyl(p, omega)); }
    real p_root(real omega, real x, real xiac) const {
        real lo = 1e-3 * kPamb, hi = kPamb;
        for (int i = 0; i < 300; ++i) {
            const real mid = 0.5 * (lo + hi);
            if (mid <= lo || mid >= hi) break;
            (f(mid, omega, x, xiac) > 0.0 ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }
    // eta_i*f_lambda*eta_spark at (rpm, p)
    real gain(real rpm, real p) const {
        const real ma = m_air_rev(rpm, p);
        const real lam = d.ecu.lambda_target.value(rpm, p);
        real spark = d.ecu.spark_advance_deg.value(rpm, p);
        if (d.maps.has_theta_kl) spark = std::min(spark, d.maps.theta_kl_deg.value(rpm, p));
        return d.maps.eta_i.value(rpm, ma) * d.maps.f_lambda.evaluate(lam) *
               d.maps.eta_spark.evaluate(spark - d.maps.theta_mbt_deg.value(rpm, p));
    }
    real m_fuel_rev(real rpm, real p) const {
        return m_air_rev(rpm, p) / (d.ecu.lambda_target.value(rpm, p) * d.fuels[0].afr_stoich);
    }
    // tau (Coulomb row excluded), omega > 0, fired with fire fraction 1
    real tau(real omega, real p, bool fired) const {
        const real rpm = rpm_of(omega);
        const real comb = fired ? gain(rpm, p) * d.fuels[0].lhv_j_kg * m_fuel_rev(rpm, p) / (2.0 * kPi) : 0.0;
        const real pmep = d.maps.has_pmep_excess ? d.maps.pmep_excess_pa.value(rpm, p) : 0.0;
        const real pump = -(kPamb - p + pmep) * vd * c / (2.0 * kPi);
        const real fric = -(d.maps.friction_torque_nm.evaluate(rpm) - d.maps.friction_torque_nm.evaluate(0.0));
        return comb + pump + fric;
    }
};

// Psi reference with std::pow.
real psi_std(real pr) {
    const real g = 1.4;
    const real crit = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
    if (pr <= crit) return std::sqrt(g * std::pow(2.0 / (g + 1.0), (g + 1.0) / (g - 1.0)));
    return std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g)));
}

// Tail time constant of a first-order approach: -h/ln(rho), rho taken
// between the first samples at 1 % and 0.1 % of the initial deviation.
real tail_tau(const std::vector<real>& p, real p_inf, real h) {
    const real dev0 = std::abs(p.front() - p_inf);
    std::size_t i1 = 0, i2 = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const real dv = std::abs(p[i] - p_inf);
        if (i1 == 0 && dv < 1e-2 * dev0) i1 = i;
        if (i2 == 0 && dv < 1e-3 * dev0) {
            i2 = i;
            break;
        }
    }
    REQUIRE(i1 > 0);
    REQUIRE(i2 > i1);
    const real rho = std::pow(std::abs(p[i2] - p_inf) / std::abs(p[i1] - p_inf), 1.0 / real(i2 - i1));
    return -h / std::log(rho);
}

// Manifold response at a held omega to a mechanical plate step x0 -> x1.
std::vector<real> plate_step_response(SimulatedEngineDesc desc, real rpm, real x0, real x1, real h, real t_after) {
    auto maps = SimulatedEngine::build_maps(desc);
    SimulatedEngine e(desc, maps);
    hold(e, w_of(rpm), x0, int(3.0 / h), h);
    std::vector<real> p{e.p_im()};
    const int n = int(t_after / h);
    for (int i = 0; i < n; ++i) {
        hold(e, w_of(rpm), x1, 1, h);
        p.push_back(e.p_im());
    }
    return p;
}

std::uint64_t fnv_fold(std::uint64_t h, const void* v, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(v);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
    return h;
}
template <typename T>
std::uint64_t fold(std::uint64_t h, const T& v) {
    return fnv_fold(h, &v, sizeof(T));
}

} // namespace

// ---------------------------------------------------------------------------

TEST_CASE("C11 construction: Psi table, derived maps, refusals", "[drivetrain][engine][mvem]") {
    SimulatedEngine e(fx::electronic_engine(), elec_maps());
    const auto& m = e.maps();

    SECTION("Psi knots equal the isentropic flow function (std::pow) up to 0.99, linear to 0 above") {
        REQUIRE(m.psi.size() == 1001);
        real worst = 0.0;
        for (int i = 0; i < 990; ++i) worst = std::max(worst, std::abs(m.psi[std::size_t(i)] - psi_std(real(i) / 1000.0)));
        std::printf("[C11 Psi] worst |table - std::pow| over knots 0..989: %.3e\n", worst);
        CHECK(worst < 1e-14);
        const real p990 = psi_std(0.99);
        for (int i = 990; i <= 1000; ++i) CHECK(std::abs(m.psi[std::size_t(i)] - p990 * real(1000 - i) / 10.0) < 1e-14);
        CHECK(m.psi[1000] == 0.0);
        // choked plateau value sqrt(gamma*(2/(gamma+1))^((gamma+1)/(gamma-1))) = 0.684731...
        CHECK(std::abs(m.psi[0] - 0.6847314563772704) < 1e-13);
        // lookup: linear between knots, slope = knot difference * 1000
        real dpsi = 0.0;
        const real v = e.psi(0.7234, &dpsi);
        CHECK(std::abs(v - (m.psi[723] + 0.4 * (m.psi[724] - m.psi[723]))) < 1e-14);
        CHECK(std::abs(dpsi - (m.psi[724] - m.psi[723]) * 1000.0) < 1e-10);
    }
    SECTION("geometry and idle feed-forward") {
        CHECK(std::abs(m.v_disp_m3 - 4.0 * kPi / 4.0 * 0.086 * 0.086 * 0.086) < 1e-18);
        CHECK(m.c_cycle == 0.5);
        CHECK(m.c_k == 6.0);
        std::printf("[C11 maps] electronic idle_ff (plate) %.6f idle_p_im %.1f Pa; mechanical idle_ff (valve) %.6f idle_p_im %.1f Pa\n",
                    m.idle_ff, m.idle_p_im, mech_maps()->idle_ff, mech_maps()->idle_p_im);
        CHECK(m.idle_ff > 0.0);
        CHECK(m.idle_ff < 0.1);
        CHECK(mech_maps()->idle_ff > 0.0);
        CHECK(mech_maps()->idle_ff < 0.5);
        // the idle feed-forward is a zero-net-torque point at idle_rpm
        RefModel ref(fx::electronic_engine(), &e);
        const real p = ref.p_root(w_of(800.0), m.idle_ff, 0.0);
        CHECK(std::abs(ref.tau(w_of(800.0), p, true) - 6.0) < 1e-6);
        CHECK(rel(m.idle_p_im, p) < 1e-9);
    }
    SECTION("rotary and fuel-less descs are refused") {
        auto d = fx::electronic_engine();
        d.architecture = EngineArchitecture::Rotary;
        CHECK_THROWS_WITH(SimulatedEngine(d), Catch::Matchers::ContainsSubstring("rotary"));
        auto d2 = fx::electronic_engine();
        d2.fuels.clear();
        CHECK_THROWS_WITH(SimulatedEngine(d2), Catch::Matchers::ContainsSubstring("fuel"));
    }
}

TEST_CASE("C11 T1 steady fixed point: p_im equals the test's own bisection root, torque to 1e-9", "[drivetrain][engine][mvem]") {
    struct Case {
        bool mech;
        real rpm, c;
    };
    const Case cases[] = {{false, 3000.0, 0.4}, {false, 1800.0, 0.15}, {true, 2000.0, 0.3}, {true, 4500.0, 1.0}, {true, 1500.0, 0.1}};
    for (const Case& k : cases) {
        const auto desc = k.mech ? fx::mechanical_engine() : fx::electronic_engine();
        SimulatedEngine e(desc, k.mech ? mech_maps() : elec_maps());
        const PreSolveResult r = hold(e, w_of(k.rpm), k.c, 4800);
        RefModel ref(desc, &e);
        const real p_ref = ref.p_root(w_of(k.rpm), e.x_thr(), e.x_iac());
        const real t_ref = ref.tau(w_of(k.rpm), p_ref, true);
        std::printf("[C11 T1] %s %.0f rpm cmd %.2f: x_thr %.6f x_iac %.3g p_im %.4f Pa vs root %.4f (rel %.2e); tau0 %.9f vs %.9f (rel %.2e)\n",
                    k.mech ? "mech" : "elec", k.rpm, k.c, e.x_thr(), e.x_iac(), e.p_im(), p_ref, rel(e.p_im(), p_ref), r.tau0, t_ref,
                    rel(r.tau0, t_ref));
        CHECK(rel(e.p_im(), p_ref) < 1e-9);
        CHECK(rel(r.tau0, t_ref) < 1e-9);
        CHECK(std::abs(ref.f(e.p_im(), w_of(k.rpm), e.x_thr(), e.x_iac())) < 1e-6 * kRT / desc.intake_manifold_volume_m3 * ref.m_cyl(p_ref, w_of(k.rpm)));
    }
}

TEST_CASE("C11 T2 manifold time constant; linearly-implicit air step at the smallest plenum", "[drivetrain][engine][mvem]") {
    // Choked plate step 0.100 -> 0.103 at a held 1500 rpm: the throttle flow
    // does not depend on p_im (pr < 0.528), so tau = -1/f'(p_inf) with
    // f' = -RT/V * dm_cyl/dp.
    auto analytic_tau = [](const SimulatedEngineDesc& d, const SimulatedEngine& e, real rpm, real p) {
        const real n = w_of(rpm) / (2.0 * kPi);
        const real vd = e.maps().v_disp_m3;
        const real dmcyl = (d.maps.ve.value(rpm, p) + p * d.maps.ve.dvalue_dy(rpm, p)) / kRT * vd * 0.5 * n;
        return d.intake_manifold_volume_m3 / (kRT * dmcyl);
    };

    SECTION("T2a: 3 L plenum - analytic tau within 2 %, 100x finer step within 0.5 %") {
        const auto d = fx::mechanical_engine();
        SimulatedEngine e(d, mech_maps());
        RefModel ref(d, &e);
        const real p_inf = ref.p_root(w_of(1500.0), 0.103, 0.0);
        REQUIRE(p_inf / kPamb < 0.528);
        const real tau_a = analytic_tau(d, e, 1500.0, p_inf);
        const auto ph = plate_step_response(d, 1500.0, 0.100, 0.103, kH, 3.0);
        const auto pf = plate_step_response(d, 1500.0, 0.100, 0.103, kH / 100.0, 3.0);
        const real tau_h = tail_tau(ph, p_inf, kH);
        const real tau_f = tail_tau(pf, p_inf, kH / 100.0);
        std::printf("[C11 T2a] V 3 L, 1500 rpm: tau analytic %.6f s, h %.6f s (%.3f %%), h/100 %.6f s (%.3f %%); h vs h/100 %.3f %%\n", tau_a, tau_h,
                    100.0 * (tau_h / tau_a - 1.0), tau_f, 100.0 * (tau_f / tau_a - 1.0), 100.0 * (tau_h / tau_f - 1.0));
        CHECK(std::abs(tau_h / tau_a - 1.0) < 0.02);
        CHECK(std::abs(tau_h / tau_f - 1.0) < 0.005);
        CHECK(rel(ph.back(), p_inf) < 1e-9);
    }
    SECTION("T2b: smallest plenum (0.2 L) - the coarse step is backward Euler of the fine-step dynamics") {
        auto d = fx::mechanical_engine();
        d.intake_manifold_volume_m3 = 0.2e-3;
        auto maps = SimulatedEngine::build_maps(d);
        SimulatedEngine e(d, maps);
        RefModel ref(d, &e);
        const real p_inf = ref.p_root(w_of(1500.0), 0.103, 0.0);
        const real tau_a = analytic_tau(d, e, 1500.0, p_inf);
        const auto ph = plate_step_response(d, 1500.0, 0.100, 0.103, kH, 0.5);
        const auto pf = plate_step_response(d, 1500.0, 0.100, 0.103, kH / 100.0, 0.5);
        const real tau_h = tail_tau(ph, p_inf, kH);
        const real tau_f = tail_tau(pf, p_inf, kH / 100.0);
        const real tau_be = kH / std::log(1.0 + kH / tau_f);    // backward Euler of the fine-step tau
        const real tau_fe = -kH / std::log(1.0 - kH / tau_f);   // explicit Euler (no flow derivative)
        std::printf("[C11 T2b] V 0.2 L, 1500 rpm: tau analytic %.6f s, h/100 %.6f s, h %.6f s (%.2f %% vs h/100); backward-Euler prediction %.6f "
                    "(%.3f %%), explicit-Euler prediction %.6f\n",
                    tau_a, tau_f, tau_h, 100.0 * (tau_h / tau_f - 1.0), tau_be, 100.0 * (tau_h / tau_be - 1.0), tau_fe);
        CHECK(std::abs(tau_f / tau_a - 1.0) < 0.02);
        CHECK(std::abs(tau_h / tau_be - 1.0) < 0.005);
        // monotone approach, no overshoot
        for (std::size_t i = 1; i < ph.size(); ++i) CHECK(ph[i] >= ph[i - 1] - 1e-9);
        CHECK(*std::max_element(ph.begin(), ph.end()) <= p_inf * (1.0 + 1e-12));
    }
    SECTION("T2b: smallest plenum WOT step at a held 6000 rpm - stiff, monotone, no overshoot") {
        auto d = fx::mechanical_engine();
        d.intake_manifold_volume_m3 = 0.2e-3;
        const auto ph = plate_step_response(d, 6000.0, 0.3, 1.0, kH, 1.0);
        const real p_inf = ph.back();
        int n_settle = -1;
        for (std::size_t i = 1; i < ph.size(); ++i) {
            CHECK(ph[i] >= ph[i - 1] - 1e-9 * p_inf);
            if (n_settle < 0 && rel(ph[i], p_inf) < 1e-6) n_settle = int(i);
        }
        CHECK(*std::max_element(ph.begin(), ph.end()) <= p_inf * (1.0 + 1e-12));
        std::printf("[C11 T2b] V 0.2 L WOT step at 6000 rpm: p %.1f -> %.1f Pa, within 1e-6 after %d substeps\n", ph.front(), p_inf, n_settle);
        CHECK(n_settle > 0);
        CHECK(n_settle < 50);
    }
    SECTION("T2b: explicit omega/p_im coupling - free WOT run-up 1500 -> 5000 rpm, h vs h/100, smallest plenum and 3 L") {
        for (real v_im : {0.2e-3, 3.0e-3}) {
            auto d = fx::mechanical_engine();
            d.intake_manifold_volume_m3 = v_im;
            auto maps = SimulatedEngine::build_maps(d);
            auto run = [&](real h, std::vector<real>& trace) {
                SimulatedEngine e(d, maps);
                hold(e, w_of(1500.0), 0.1, int(2.0 / h), h);
                real w = w_of(1500.0), t = 0.0;
                trace.clear();
                const int every = int(std::lround(kH / h));
                int i = 0;
                while (w < w_of(5000.0) && t < 5.0) {
                    if (i % every == 0) trace.push_back(w);
                    const real w_new = crank_step(e, w, 1.0, d.inertia_kgm2, h);
                    CHECK(w_new >= w);
                    // interpolate the crossing time
                    if (w_new >= w_of(5000.0)) return t + h * (w_of(5000.0) - w) / (w_new - w);
                    w = w_new;
                    t += h;
                    ++i;
                }
                return t;
            };
            std::vector<real> th, tf;
            const real t_h = run(kH, th);
            const real t_f = run(kH / 100.0, tf);
            real worst = 0.0;
            for (std::size_t i = 0; i < std::min(th.size(), tf.size()); ++i) worst = std::max(worst, rel(th[i], tf[i]));
            std::printf("[C11 T2b] free run-up V %.1f L: t(1500->5000) h %.6f s, h/100 %.6f s (%.3f %%); worst pointwise omega %.3f %%\n",
                        v_im * 1e3, t_h, t_f, 100.0 * (t_h / t_f - 1.0), 100.0 * worst);
            CHECK(std::abs(t_h / t_f - 1.0) < 0.005);
            CHECK(worst < 0.005);
        }
    }
}

// Pedal plan P1 re-pin (2026-10-13): the electronic law is now T* = Z + r (F - Z) (Z = the idle air's fired torque, or the
// cut torque in DFCO; F = steady WOT; request_envelope), was motoring + r (wot - motoring). With the old target the P1
// engine misses by 1.2-5.1 % of the span (largest at r 0.1, where Z - motoring is a larger share).
TEST_CASE("C11 T4a electronic pedal law: steady torque = Z + request*(F - Z)", "[drivetrain][engine][mvem]") {
    const auto desc = fx::electronic_engine();
    const real ck = elec_maps()->c_k;
    auto steady_net = [&](real rpm, real r, real ignition, PedalEnvelope* env = nullptr) {
        SimulatedEngine e(desc, elec_maps());
        const PreSolveResult res = hold(e, w_of(rpm), r, 2880, kH, ignition);
        if (env != nullptr) *env = e.request_envelope(rpm);
        return res.tau0 - ck;
    };
    struct Pt {
        real rpm, r;
    };
    std::vector<Pt> pts;
    for (real rpm : {1500.0, 2500.0, 4000.0, 5500.0})
        for (real r : {0.1, 0.3, 0.6, 1.0}) pts.push_back({rpm, r});
    // off the inverse table's own grid (rpm step 250, request step 0.05)
    pts.push_back({3100.0, 0.45});
    pts.push_back({1830.0, 0.72});
    pts.push_back({4710.0, 0.18});
    pts.push_back({5960.0, 0.93});
    real worst_grid = 0.0, worst_off = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Pt& p = pts[i];
        const real wot = steady_net(p.rpm, 1.0, 1.0);
        const real mot = steady_net(p.rpm, 0.0, 0.0);
        PedalEnvelope env;
        const real t = steady_net(p.rpm, p.r, 1.0, &env);
        const real target = env.z_nm + p.r * (env.f_nm - env.z_nm);
        const real err = std::abs(t - target) / (env.f_nm - env.z_nm);
        (i < 16 ? worst_grid : worst_off) = std::max(i < 16 ? worst_grid : worst_off, err);
        std::printf("[C11 T4a] %.0f rpm r %.2f: mot %.3f wot %.3f -> %.4f vs %.4f N*m (%.4f %% of span)\n", p.rpm, p.r, mot, wot, t, target, 100.0 * err);
        CHECK(err < 0.01);
    }
    std::printf("[C11 T4a] worst error: grid %.2e, off-grid %.4f %% of (wot - mot)\n", worst_grid, 100.0 * worst_off);

    SECTION("a request step shows the actuator plus manifold lag, not a torque step") {
        SimulatedEngine e(desc, elec_maps());
        const real w = w_of(3000.0);
        const PreSolveResult r0 = hold(e, w, 0.3, 2880);
        const real t_hi = steady_net(3000.0, 0.6, 1.0) + ck;
        const real x_old = e.x_thr();
        // Pedal P1 re-pin: the plate target is the inverse at the torque-equivalent request of T* (not at the pedal); it is
        // taken as the plate a fresh engine settles at with pedal 0.6 (the lag converges to it). Was inverse_plate(3000,
        // 0.6) to 1e-15; with P1 that misses by 4.8e-5.
        SimulatedEngine e_ss(desc, elec_maps());
        (void)hold(e_ss, w, 0.6, 2880);
        const real target = e_ss.x_thr();
        const PreSolveResult r1 = hold(e, w, 0.6, 1);
        const real a = kH / (0.05 + kH);
        CHECK(std::abs(e.x_thr() - (x_old + a * (target - x_old))) < 1e-9);
        const real frac1 = (r1.tau0 - r0.tau0) / (t_hi - r0.tau0);
        int n90 = 1;
        real tau = r1.tau0;
        while ((tau - r0.tau0) < 0.9 * (t_hi - r0.tau0) && n90 < 5000) {
            tau = hold(e, w, 0.6, 1).tau0;
            ++n90;
        }
        std::printf("[C11 T4a] step 0.3 -> 0.6 at 3000 rpm: after one substep %.3f %% of the torque step; 90 %% after %.4f s\n", 100.0 * frac1,
                    n90 * kH);
        CHECK(frac1 < 0.05);
        CHECK(n90 * kH > 0.05);
        CHECK(n90 * kH < 1.0);
    }
}

TEST_CASE("C11 T4b mechanical plate path: plate == command, T1 torque, idle on the bypass valve", "[drivetrain][engine][mvem]") {
    const auto desc = fx::mechanical_engine();
    SECTION("x_thr is the command exactly and the steady torque is the fixed-point torque") {
        for (real c : {0.05, 0.2, 0.5, 0.9}) {
            SimulatedEngine e(desc, mech_maps());
            const PreSolveResult r = hold(e, w_of(2500.0), c, 4800);
            CHECK(e.x_thr() == c);
            CHECK(e.x_iac() == 0.0);
            RefModel ref(desc, &e);
            const real p = ref.p_root(w_of(2500.0), c, 0.0);
            const real t = ref.tau(w_of(2500.0), p, true);
            std::printf("[C11 T4b] 2500 rpm plate %.2f: tau0 %.9f vs fixed point %.9f (rel %.2e)\n", c, r.tau0, t, rel(r.tau0, t));
            CHECK(rel(r.tau0, t) < 1e-9);
            CHECK(r.throttle_eff == c);
        }
    }
    SECTION("idle is held by x_iac with the plate at 0 (free crank from 1200 rpm)") {
        SimulatedEngine e(desc, mech_maps());
        real w = w_of(1200.0);
        for (int i = 0; i < 15 * 960; ++i) {
            w = crank_step(e, w, 0.0, desc.inertia_kgm2, kH);
            CHECK(e.x_thr() == 0.0);
        }
        std::printf("[C11 T4b] mech idle after 15 s: %.4f rpm, x_thr %.3g, x_iac %.6f (ff %.6f)\n", rpm_of(w), e.x_thr(), e.x_iac(), e.maps().idle_ff);
        CHECK(std::abs(rpm_of(w) - 800.0) < 0.5);
        CHECK(std::abs(e.x_iac() - e.maps().idle_ff) < 1e-3);
        CHECK(e.state() == EngineState::Running);
    }
}

TEST_CASE("C11 T5 indicated work == eta_i*f_lambda*eta_spark*LHV*m_fuel", "[drivetrain][engine][mvem]") {
    for (bool reserved : {false, true}) {
        auto desc = fx::electronic_engine();
        if (reserved) desc = fx::with_reserved_maps(desc);
        auto maps = reserved ? SimulatedEngine::build_maps(desc) : elec_maps();
        SimulatedEngine e(desc, maps);
        RefModel ref(desc, &e);
        real w = w_of(800.0);
        real sum_w = 0.0, sum_ref = 0.0, worst = 0.0, fuel = 0.0;
        int dfco_substeps = 0;
        for (int i = 0; i < 2 * 960; ++i) {
            const real c = i < 960 ? 1.0 : 0.0;
            PreSolveResult r;
            Sensors s = running(w);
            r = e.pre_solve(s, cmd(c), TcuRequest{}, kH);
            real w_new = w + kH * (r.tau0 - r.friction_cap_kinetic) / (desc.inertia_kgm2 + kH * r.d);
            const PostSolveResult po = e.post_solve(w_new, kH);
            const real g = ref.gain(rpm_of(w), e.p_im());
            const real expect = g * desc.fuels[0].lhv_j_kg * po.fuel_mass_delta;
            sum_w += po.indicated_work_delta;
            sum_ref += expect;
            fuel += po.fuel_mass_delta;
            if (expect != 0.0) worst = std::max(worst, rel(po.indicated_work_delta, expect));
            else CHECK(po.indicated_work_delta == 0.0);
            if (r.dfco_active) {
                ++dfco_substeps;
                CHECK(po.fuel_mass_delta == 0.0);
            }
            CHECK(po.fuel_energy_in_delta == po.fuel_mass_delta * desc.fuels[0].lhv_j_kg);
            w = w_new;
        }
        std::printf("[C11 T5]%s WOT free rev 1 s + lift 1 s: indicated %.6f J, closure %.6f J, rel %.2e, worst substep %.2e, fuel %.6f g, dfco substeps %d\n",
                    reserved ? " (pmep_excess + theta_kl)" : "", sum_w, sum_ref, rel(sum_w, sum_ref), worst, fuel * 1e3, dfco_substeps);
        CHECK(rel(sum_w, sum_ref) < 1e-12);
        CHECK(worst < 1e-12);
        CHECK(fuel > 0.0);
        CHECK(dfco_substeps > 0);
    }
}

TEST_CASE("C11 T6 dtau/domega against a central difference, 1e-6 relative", "[drivetrain][engine][mvem]") {
    auto soft = fx::electronic_engine();
    soft.limiter.type = LimiterType::SoftCut;
    const auto reserved = fx::with_reserved_maps(fx::electronic_engine());
    auto soft_maps = SimulatedEngine::build_maps(soft);
    auto res_maps = SimulatedEngine::build_maps(reserved);
    SimulatedEngine e(fx::electronic_engine(), elec_maps());
    SimulatedEngine es(soft, soft_maps);
    SimulatedEngine er(reserved, res_maps);
    struct Case {
        const char* name;
        SimulatedEngine* eng;
        real rpm, p;
        bool fired;
        real s;
        bool starter;
    };
    const Case cases[] = {
        {"fired", &e, 3137.0, 63170.0, true, 1.0, false},
        {"fired low load", &e, 1234.0, 27345.0, true, 1.0, false},
        {"fired high rpm", &e, 6021.0, 97310.0, true, 1.0, false},
        {"unfired", &e, 4321.0, 31000.0, false, 1.0, false},
        {"soft-cut band", &es, 6700.0, 91000.0, true, 1.0, false},
        {"starter (unfired, 150 rpm)", &e, 150.0, 100000.0, false, 1.0, true},
        {"pmep_excess + theta_kl active", &er, 2345.0, 91234.0, true, 1.0, false},
        {"pmep_excess, theta_kl inactive", &er, 5432.0, 41234.0, true, 1.0, false},
        {"starving s = 0.5", &e, 3137.0, 63170.0, true, 0.5, false},
    };
    SECTION("central difference") {
        for (const Case& k : cases) {
            const real w = w_of(k.rpm);
            const real dw = 1e-3;
            const auto ev = k.eng->evaluate_torque(w, k.p, k.fired, k.s, k.starter);
            const real tp = k.eng->evaluate_torque(w + dw, k.p, k.fired, k.s, k.starter).tau;
            const real tm = k.eng->evaluate_torque(w - dw, k.p, k.fired, k.s, k.starter).tau;
            const real fd = (tp - tm) / (2.0 * dw);
            std::printf("[C11 T6] %-32s dtau/domega %.10f vs central %.10f (rel %.2e)\n", k.name, ev.dtau_domega, fd, rel(ev.dtau_domega, fd));
            CHECK(rel(ev.dtau_domega, fd) < 1e-6);
            CHECK(ev.dtau_domega != 0.0);
        }
    }
    SECTION("theta_kl really is active at the first reserved point and not at the second") {
        CHECK(fx::fx_kl(2345.0, 91234.0) < fx::fx_spark(2345.0, 91234.0));
        CHECK(fx::fx_kl(5432.0, 41234.0) > fx::fx_spark(5432.0, 41234.0));
    }
    SECTION("pre_solve's d is max(-dtau/domega, 0) at the post-step p_im") {
        SimulatedEngine a(fx::electronic_engine(), elec_maps());
        for (real rpm : {1500.0, 3000.0, 6000.0}) {
            const PreSolveResult r = hold(a, w_of(rpm), 1.0, 20);
            const auto ev = a.evaluate_torque(w_of(rpm), a.p_im(), true, 1.0, false);
            CHECK(r.tau0 == ev.tau);
            CHECK(r.d == std::max(-ev.dtau_domega, 0.0));
        }
    }
}

TEST_CASE("C11 literal tau0/d pins", "[drivetrain][engine][mvem]") {
    // Held crank, WOT, settled (4800 substeps), both off every map knot (a
    // central difference straddling a knot averages two segments while the
    // analytic slope takes the left one): electronic at 6150 rpm (VE
    // past its peak and friction rising: d > 0) and mechanical at 5500 rpm.
    SimulatedEngine e(fx::electronic_engine(), elec_maps());
    const PreSolveResult r = hold(e, w_of(6150.0), 1.0, 4800);
    std::printf("[C11 pin] elec 6150 rpm WOT: tau0 %.12f d %.12f p_im %.6f\n", r.tau0, r.d, e.p_im());
    CHECK(r.d > 0.0);
    CHECK(std::abs(r.tau0 - 206.225143321085) < 1e-9);
    CHECK(std::abs(r.d - 0.158848028239467) < 1e-12);
    SimulatedEngine m(fx::mechanical_engine(), mech_maps());
    const PreSolveResult rm = hold(m, w_of(5500.0), 1.0, 4800);
    std::printf("[C11 pin] mech 5500 rpm WOT: tau0 %.12f d %.12f p_im %.6f\n", rm.tau0, rm.d, m.p_im());
    CHECK(rm.d > 0.0);
    // The literals are this build's own output; that they are right is
    // cross-checked here against the test's independent section 3.1
    // re-derivation (RefModel) - tau0 at its bisected root, d as a central
    // difference of its torque at that frozen p_im.
    for (int which = 0; which < 2; ++which) {
        const SimulatedEngine& eng = which == 0 ? e : m;
        const PreSolveResult& pr = which == 0 ? r : rm;
        const real w = w_of(which == 0 ? 6150.0 : 5500.0);
        RefModel ref(which == 0 ? fx::electronic_engine() : fx::mechanical_engine(), &eng);
        const real p = ref.p_root(w, eng.x_thr(), eng.x_iac());
        const real fd = (ref.tau(w + 1e-3, p, true) - ref.tau(w - 1e-3, p, true)) / 2e-3;
        CHECK(rel(pr.tau0, ref.tau(w, p, true)) < 1e-9);
        CHECK(rel(pr.d, -fd) < 1e-6);
    }
    CHECK(std::abs(rm.tau0 - 213.966663718996) < 1e-9);
    CHECK(std::abs(rm.d - 0.0897737661879352) < 1e-12);
}

TEST_CASE("C11 fuel starvation (EngineTorqueMap's s_starve rule)", "[drivetrain][engine][mvem]") {
    // Two engines in the same state; the second gets 0, half and the full
    // demand of the first's booked fuel.
    const real w = w_of(3137.0);
    SimulatedEngine a(fx::electronic_engine(), elec_maps());
    hold(a, w, 0.8, 1000);
    for (real frac : {0.0, 0.5, 1.0, 2.0}) {
        SimulatedEngine ref = a; // copy: same state
        SimulatedEngine b = a;
        const PreSolveResult r_ref = ref.pre_solve(running(w), cmd(0.8), TcuRequest{}, kH);
        const PostSolveResult p_ref = ref.post_solve(w, kH);
        Sensors s = running(w);
        s.fuel_available_kg = frac * p_ref.fuel_mass_delta;
        const PreSolveResult r = b.pre_solve(s, cmd(0.8), TcuRequest{}, kH);
        const PostSolveResult p = b.post_solve(w, kH);
        const real sx = std::min(frac, 1.0);
        const auto e1 = a.evaluate_torque(w, b.p_im(), true, 1.0, false);
        const auto e0 = a.evaluate_torque(w, b.p_im(), false, 1.0, false);
        CHECK(r.fuel_supply_fraction == sx);
        CHECK(p.fuel_mass_delta == sx * p_ref.fuel_mass_delta);
        CHECK(std::abs(r.tau0 - (e0.tau + sx * (e1.tau - e0.tau))) < 1e-12 * std::abs(e1.tau));
        const real dtau = e0.dtau_domega + sx * (e1.dtau_domega - e0.dtau_domega);
        CHECK(std::abs(r.d - std::max(-dtau, 0.0)) < 1e-12);
        CHECK(std::abs(r.theta_c - sx * r_ref.theta_c) < 1e-15);
        CHECK(std::abs(p.indicated_work_delta - sx * p_ref.indicated_work_delta) < 1e-12 * std::abs(p_ref.indicated_work_delta));
        std::printf("[C11 starve] supply %.1fx demand: s %.3f, tau0 %.6f (full %.6f), d %.8f (full %.8f)\n", frac, r.fuel_supply_fraction, r.tau0, r_ref.tau0,
                    r.d, r_ref.d);
    }
}

TEST_CASE("C11 T7 state machine, DFCO, limiter, idle PI, ignition cut", "[drivetrain][engine][mvem]") {
    SECTION("state machine transitions") {
        auto d = fx::electronic_engine();
        d.initial_state = EngineInitialState::Off;
        SimulatedEngine e(d, elec_maps());
        CHECK(e.state() == EngineState::Off);
        CHECK(e.p_im() == kPamb);
        Sensors s = running(0.0);
        s.ignition = 0.0;
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Off);
        (void)e.post_solve(0.0, kH);
        s.ignition = 1.0;
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Stalled);
        (void)e.post_solve(0.0, kH);
        s.starter = 1.0;
        PreSolveResult r = e.pre_solve(s, cmd(0), TcuRequest{}, kH);
        CHECK(r.state == EngineState::Cranking);
        CHECK(r.starter_active);
        CHECK(r.tau0 == 140.0); // starter stall torque; pumping and friction are zero at omega = 0
        (void)e.post_solve(0.0, kH);
        // starter released below start_complete -> Stalled
        s.starter = 0.0;
        s.omega = w_of(300.0);
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Stalled);
        (void)e.post_solve(s.omega, kH);
        // bump start
        s.omega = w_of(600.0);
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Running);
        (void)e.post_solve(s.omega, kH);
        // stall below stall_rpm
        s.omega = w_of(390.0);
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Stalled);
        (void)e.post_solve(s.omega, kH);
        CHECK(e.idle_integrator() == 0.0);
        // backwards rotation stalls a running engine
        s.omega = w_of(600.0);
        (void)e.pre_solve(s, cmd(0), TcuRequest{}, kH);
        (void)e.post_solve(s.omega, kH);
        REQUIRE(e.state() == EngineState::Running);
        s.omega = -w_of(600.0);
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Stalled);
        (void)e.post_solve(s.omega, kH);
        s.ignition = 0.0;
        CHECK(e.pre_solve(s, cmd(0), TcuRequest{}, kH).state == EngineState::Off);
    }
    SECTION("crank start from rest, then idle on the plate (electronic)") {
        auto d = fx::electronic_engine();
        d.initial_state = EngineInitialState::Off;
        SimulatedEngine e(d, elec_maps());
        real w = 0.0, t_run = -1.0;
        for (int i = 0; i < 12 * 960; ++i) {
            PreSolveResult r;
            w = crank_step(e, w, 0.0, d.inertia_kgm2, kH, &r, i < 960 ? 1.0 : 0.0);
            if (t_run < 0.0 && e.state() == EngineState::Running) t_run = i * kH;
        }
        std::printf("[C11 T7] crank start: Running after %.4f s; idle after 12 s %.4f rpm, x_thr %.6f (ff %.6f), x_iac %.3g\n", t_run, rpm_of(w), e.x_thr(),
                    e.maps().idle_ff, e.x_iac());
        CHECK(t_run > 0.0);
        CHECK(t_run < 1.0);
        CHECK(std::abs(rpm_of(w) - 800.0) < 0.5);
        CHECK(std::abs(e.x_thr() - e.maps().idle_ff) < 1e-3);
        CHECK(e.x_iac() == 0.0);
    }
    SECTION("DFCO set, held, cleared") {
        SimulatedEngine e(fx::electronic_engine(), elec_maps());
        PreSolveResult r = hold(e, w_of(2500.0), 0.0, 5);
        CHECK(r.dfco_active);
        CHECK(e.evaluate_torque(w_of(2500.0), e.p_im(), false, 1.0, false).tau == r.tau0);
        CHECK(e.post_solve(w_of(2500.0), kH).fuel_mass_delta == 0.0);
        r = hold(e, w_of(2500.0), 0.005, 1);
        CHECK(r.dfco_active);
        r = hold(e, w_of(1100.0), 0.0, 1); // between re-enable and enable: held
        CHECK(r.dfco_active);
        r = hold(e, w_of(990.0), 0.0, 1); // below re-enable: cleared
        CHECK_FALSE(r.dfco_active);
        r = hold(e, w_of(1200.0), 0.0, 1); // below enable: not set again
        CHECK_FALSE(r.dfco_active);
        r = hold(e, w_of(2500.0), 0.0, 1);
        CHECK(r.dfco_active);
        // Pedal P1 re-pin: 0.02 now lies in the plan-2.3 DFCO residual band (fuel stays cut while T_cut + c (F - T_cut) is
        // below the fired floor); the pedal that leaves DFCO is the band edge (was CHECK_FALSE at 0.02).
        r = hold(e, w_of(2500.0), 0.02, 1);
        const PedalEnvelope env = e.request_envelope(2500.0);
        const real c_exit = (env.fired_floor_nm - env.cut_nm) / (env.f_nm - env.cut_nm);
        INFO("c_exit " << c_exit);
        CHECK(r.dfco_active == (0.02 < c_exit));
        r = hold(e, w_of(2500.0), c_exit + 0.005, 1);
        CHECK_FALSE(r.dfco_active);
    }
    SECTION("FuelCut limiter with hysteresis") {
        SimulatedEngine e(fx::electronic_engine(), elec_maps());
        PreSolveResult r = hold(e, w_of(6500.0), 1.0, 200);
        CHECK_FALSE(r.limiter_active);
        r = hold(e, w_of(6801.0), 1.0, 1);
        CHECK(r.limiter_active);
        CHECK(e.limiter_cut());
        CHECK(e.post_solve(w_of(6801.0), kH).fuel_mass_delta == 0.0);
        CHECK(r.theta_c == 0.0);
        r = hold(e, w_of(6700.0), 1.0, 1);
        CHECK(r.limiter_active);
        r = hold(e, w_of(6640.0), 1.0, 1);
        CHECK_FALSE(r.limiter_active);
        CHECK(r.theta_c > 0.0);
    }
    SECTION("SoftCut limiter: linear fire fraction over the band") {
        auto d = fx::electronic_engine();
        d.limiter.type = LimiterType::SoftCut;
        auto maps = SimulatedEngine::build_maps(d);
        SimulatedEngine e(d, maps);
        for (real rpm : {6550.0, 6650.0, 6700.0, 6790.0, 6800.0, 6900.0}) {
            const auto ev = e.evaluate_torque(w_of(rpm), 90000.0, true, 1.0, false);
            const auto ev_full = SimulatedEngine(fx::electronic_engine(), elec_maps()).evaluate_torque(w_of(rpm), 90000.0, true, 1.0, false);
            const real f = std::clamp((6800.0 - rpm) / 200.0, 0.0, 1.0);
            CHECK(std::abs(ev.fire - f) < 1e-12);
            CHECK(std::abs(ev.m_fuel_rev - f * ev_full.m_fuel_rev) <= 1e-12 * ev_full.m_fuel_rev);
        }
        PreSolveResult r = hold(e, w_of(6700.0), 1.0, 5);
        CHECK(r.limiter_active);
        r = hold(e, w_of(6550.0), 1.0, 1);
        CHECK_FALSE(r.limiter_active);
    }
    SECTION("idle PI acts on the plate (electronic) and on the bypass valve (mechanical)") {
        const real w = w_of(700.0);
        SimulatedEngine el(fx::electronic_engine(), elec_maps());
        const real ff_e = el.maps().idle_ff;
        const real x0 = el.x_thr();
        REQUIRE(x0 == ff_e);
        // Pedal released: the PI position plus the manifold-fill term, x with
        // f(p, x) = 4*f(p, theta_idle) linearised by a 1e-4 forward difference
        // (owner 2026-09-29 idle rework), evaluated at the pre-step p_im.
        auto fill = [&](const SimulatedEngine& eng, real theta, bool mech) {
            auto f_at = [&](real xi) { return mech ? eng.manifold_rhs(eng.p_im(), w, 0.0, xi).f : eng.manifold_rhs(eng.p_im(), w, xi, 0.0).f; };
            const real f0 = f_at(theta);
            const real dfdx = (f_at(theta + 1e-4) - f0) / 1e-4;
            REQUIRE(f0 > 0.0); // below idle the manifold is under its target: the fill term adds air
            return std::clamp(theta + 3.0 * f0 / dfdx, 0.0, mech ? 0.5 : 0.1);
        };
        const real x_idle_e = fill(el, ff_e + 3e-5 * 100.0, false);
        CHECK(x_idle_e > ff_e + 3e-5 * 100.0);
        (void)el.pre_solve(running(w), cmd(0.0), TcuRequest{}, kH);
        const real tgt = std::max(el.maps().inverse_plate.value(700.0, 0.0), x_idle_e);
        CHECK(std::abs(el.x_thr() - (x0 + kH / (0.05 + kH) * (tgt - x0))) < 1e-16);
        CHECK(el.x_iac() == 0.0);
        CHECK(std::abs(el.idle_integrator() - 6e-5 * 100.0 * kH) < 1e-18);

        SimulatedEngine me(fx::mechanical_engine(), mech_maps());
        const real ff_m = me.maps().idle_ff;
        REQUIRE(me.x_iac() == ff_m);
        REQUIRE(me.x_thr() == 0.0);
        const real x_idle_m = fill(me, std::clamp(ff_m + 1.5e-4 * 100.0, 0.0, 0.5), true);
        (void)me.pre_solve(running(w), cmd(0.0), TcuRequest{}, kH);
        CHECK(std::abs(me.x_iac() - x_idle_m) < 1e-12); // the forward difference amplifies last-bit differences (1.7e-14 measured)
        CHECK(me.x_thr() == 0.0);
    }
    SECTION("ignition cut: no combustion; the electronic plate closes, the mechanical plate stays at the command") {
        TcuRequest cut;
        cut.ignition_cut = true;
        SimulatedEngine el(fx::electronic_engine(), elec_maps());
        hold(el, w_of(3000.0), 0.5, 500);
        const real x_before = el.x_thr();
        PreSolveResult r = el.pre_solve(running(w_of(3000.0)), cmd(0.5), cut, kH);
        CHECK(el.post_solve(w_of(3000.0), kH).fuel_mass_delta == 0.0);
        CHECK(r.theta_c == 0.0);
        CHECK(el.x_thr() < x_before);
        SimulatedEngine me(fx::mechanical_engine(), mech_maps());
        hold(me, w_of(3000.0), 0.5, 500);
        r = me.pre_solve(running(w_of(3000.0)), cmd(0.5), cut, kH);
        CHECK(me.post_solve(w_of(3000.0), kH).fuel_mass_delta == 0.0);
        CHECK(me.x_thr() == 0.5);
        CHECK(r.theta_c == 0.0);
    }
    SECTION("floor/ceiling: electronic request and mechanical plate") {
        SimulatedEngine el(fx::electronic_engine(), elec_maps());
        (void)el.pre_solve(running(w_of(3000.0)), ThrottleCommand{0.9, 0.0, 0.4}, TcuRequest{}, kH);
        SimulatedEngine el2(fx::electronic_engine(), elec_maps());
        (void)el2.pre_solve(running(w_of(3000.0)), ThrottleCommand{0.4, 0.0, 1.0}, TcuRequest{}, kH);
        CHECK(el.x_thr() == el2.x_thr());
        SimulatedEngine me(fx::mechanical_engine(), mech_maps());
        (void)me.pre_solve(running(w_of(3000.0)), ThrottleCommand{0.1, 0.3, 1.0}, TcuRequest{}, kH);
        CHECK(me.x_thr() == 0.3);
    }
}

TEST_CASE("C11 hashed state layout, content hash, reset, zero allocation", "[drivetrain][engine][mvem]") {
    auto expected_hash = [](const SimulatedEngine& e, std::uint64_t seed) {
        std::uint64_t h = seed;
        h = fold(h, e.maps().content_hash);
        h = fold(h, e.state());
        h = fold(h, e.idle_integrator());
        h = fold(h, e.limiter_cut());
        h = fold(h, e.dfco_active());
        h = fold(h, e.p_im());
        h = fold(h, e.x_thr());
        if (e.maps().throttle_input == ThrottleInput::Mechanical) h = fold(h, e.x_iac());
        return h;
    };
    SECTION("fold layout: content hash first, then state, integrator, latches, p_im, x_thr (+ x_iac on mechanical)") {
        SimulatedEngine el(fx::electronic_engine(), elec_maps());
        SimulatedEngine me(fx::mechanical_engine(), mech_maps());
        hold(el, w_of(2500.0), 0.3, 50);
        hold(me, w_of(1000.0), 0.0, 50);
        REQUIRE(me.x_iac() > 0.0);
        for (std::uint64_t seed : {std::uint64_t(14695981039346656037ULL), std::uint64_t(12345)}) {
            CHECK(el.fold_state_hash(seed) == expected_hash(el, seed));
            CHECK(me.fold_state_hash(seed) == expected_hash(me, seed));
        }
    }
    SECTION("crank angle is not hashed; every hashed field moves the hash") {
        SimulatedEngine a(fx::mechanical_engine(), mech_maps());
        SimulatedEngine b(fx::mechanical_engine(), mech_maps());
        hold(a, w_of(1000.0), 0.0, 10);
        hold(b, w_of(1000.0), 0.0, 10);
        REQUIRE(a.fold_state_hash(1) == b.fold_state_hash(1));
        (void)b.post_solve(w_of(1000.0), kH); // advances crank angle only
        CHECK(a.crank_angle() != b.crank_angle());
        CHECK(a.fold_state_hash(1) == b.fold_state_hash(1));
        SimulatedEngine c = a;
        hold(c, w_of(1000.0), 0.0, 1);
        CHECK(c.fold_state_hash(1) != a.fold_state_hash(1));
    }
    SECTION("the maps' content hash separates engines in identical state") {
        auto d2 = fx::electronic_engine();
        d2.maps.friction_torque_nm.y.back() += 1.0; // 8000 rpm knot: touches only the steady tables above 7000 rpm
        auto maps2 = SimulatedEngine::build_maps(d2);
        SimulatedEngine a(fx::electronic_engine(), elec_maps());
        SimulatedEngine b(d2, maps2);
        REQUIRE(a.maps().idle_ff == b.maps().idle_ff);
        REQUIRE(a.p_im() == b.p_im());
        CHECK(a.maps().content_hash != b.maps().content_hash);
        CHECK(a.fold_state_hash(7) != b.fold_state_hash(7));
        // build_maps is deterministic
        CHECK(SimulatedEngine::build_maps(fx::electronic_engine())->content_hash == elec_maps()->content_hash);
        std::printf("[C11 hash] content hash electronic 0x%016llx mechanical 0x%016llx\n", (unsigned long long)elec_maps()->content_hash,
                    (unsigned long long)mech_maps()->content_hash);
    }
    SECTION("reset restores the initial state's air path") {
        SimulatedEngine a(fx::mechanical_engine(), mech_maps());
        const std::uint64_t h0 = a.fold_state_hash(3);
        hold(a, w_of(4000.0), 0.7, 300);
        REQUIRE(a.fold_state_hash(3) != h0);
        a.reset(EngineState::Running);
        CHECK(a.fold_state_hash(3) == h0);
        a.reset(EngineState::Off);
        CHECK(a.p_im() == kPamb);
        CHECK(a.x_iac() == 0.0);
    }
    SECTION("zero allocation after construction") {
        SimulatedEngine el(fx::electronic_engine(), elec_maps());
        SimulatedEngine me(fx::mechanical_engine(), mech_maps());
        namespace da = ps::jobs::debug_alloc;
        if (!da::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        da::reset();
        da::set_armed(true);
        da::enter_job();
        real w1 = w_of(800.0), w2 = w_of(800.0);
        for (int i = 0; i < 2000; ++i) {
            const real c = (i / 400) % 2 ? 1.0 : 0.0;
            w1 = crank_step(el, w1, c, 0.15, kH);
            w2 = crank_step(me, w2, c, 0.15, kH);
            (void)el.fold_state_hash(1);
        }
        da::exit_job();
        da::set_armed(false);
        CHECK(da::own_violation_count() == 0);
    }
}

TEST_CASE("Twin turbo hyper V8: fuel-compatible loading, boost, coast and energy", "[turbo][sim_engine]") {
    // The shipped engine has no maps block: loading generates them from its
    // cycle block (10 x 6 grid, boosted nodes), on a pool to keep this test short.
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions map_options;
    map_options.pool=&pool;
    const auto t_load=std::chrono::steady_clock::now();
    const auto loaded=ps::io::load_engine_json(std::string(PS_SOURCE_DIR)+"/data/engines/hyper_v8_one1_audio.json",map_options);
    std::printf("[turbo dyno] load incl. map generation: %.2f s\n",std::chrono::duration<double>(std::chrono::steady_clock::now()-t_load).count());
    auto desc=std::get<SimulatedEngineDesc>(loaded);
    REQUIRE(desc.turbo_pair.has_value());
    SimulatedEngine boosted(desc);
    auto na_desc=desc; na_desc.turbo_pair.reset();
    SimulatedEngine na(na_desc);
    const auto nr=hold(na,w_of(6000),1.0,9600);
    // WOT hold on the dyno; the last second is the steady window. The boost
    // controller (PI on charge pressure) must settle: the earlier narrow-band
    // proportional wastegate limit-cycled at 7-10 Hz (rotor 59k-137k rpm,
    // torque 780-1500 Nm at 6000 rpm), so a single sample was a random phase.
    struct Window { PreSolveResult last; real charge_min=1e9, charge_max=-1e9, tau_mean=0; };
    const auto settle=[&](SimulatedEngine& e, real rpm) {
        Window wdw;
        for(int i=0;i<14400;++i) {
            wdw.last=e.pre_solve(running(w_of(rpm)),cmd(1),TcuRequest{},kH);
            (void)e.post_solve(w_of(rpm),kH);
            if(i>=13440) {
                wdw.charge_min=std::min(wdw.charge_min,wdw.last.p_charge_pa); wdw.charge_max=std::max(wdw.charge_max,wdw.last.p_charge_pa);
                wdw.tau_mean+=(wdw.last.tau0-e.maps().c_k)/960;
            }
        }
        std::printf("[turbo dyno] %4.0f rpm: boost %.3f bar, charge %.0f Pa (ripple %.0f), rotor %.0f rpm, exhaust %.0f Pa, net torque %.1f Nm, power %.1f kW\n",
            rpm,wdw.last.boost_bar,wdw.last.p_charge_pa,wdw.charge_max-wdw.charge_min,wdw.last.turbo_rpm,wdw.last.p_exhaust_pa,wdw.tau_mean,wdw.tau_mean*w_of(rpm)/1000);
        return wdw;
    };
    const real target_charge=kPamb+desc.turbo_pair->target_boost_pa;
    for(real rpm : {3000.0, 4500.0}) {
        const auto wdw=settle(boosted,rpm);
        CHECK(wdw.charge_max-wdw.charge_min<1000);
        CHECK(wdw.tau_mean>1000);
        CHECK(std::abs(wdw.last.p_charge_pa-target_charge)<0.02*desc.turbo_pair->target_boost_pa);
    }
    const auto br_w=settle(boosted,6000);
    const auto& br=br_w.last;
    std::printf("[turbo dyno] NA at 6000 rpm: %.1f Nm\n",nr.tau0);
    const auto high_w=settle(boosted,7500);
    const auto& high=high_w.last;
    for(const Window* wdw : {&br_w,&high_w}) {
        CHECK(wdw->charge_max-wdw->charge_min<1000);   // settled, no limit cycle
        CHECK(std::abs(wdw->last.p_charge_pa-target_charge)<0.02*desc.turbo_pair->target_boost_pa); // on target
    }
    CHECK(std::isfinite(br.tau0)); CHECK(br.boost_bar>0.2); CHECK(br.boost_bar<2.3);
    CHECK(br.turbo_rpm>10000); CHECK(br_w.tau_mean>nr.tau0-na.maps().c_k);
    // Net shaft targets, including Coulomb friction; protect the physical fit.
    CHECK(std::abs(br_w.tau_mean-1371)<20);
    CHECK(std::abs(high_w.tau_mean*w_of(7500)/1000-1000)<15);
    SimulatedEngine top(desc);
    const auto top_w=settle(top,8000);
    CHECK(top_w.tau_mean>1000);
    CHECK(top_w.charge_max-top_w.charge_min<1000);
    CHECK(high.p_exhaust_pa>high.p_intake_pa);
    CHECK(high.fuel_flow_g_s>70);
    CHECK(high.fuel_flow_g_s<100);
    CHECK_FALSE(boosted.maps().maps.has_theta_kl);
    // A boosted engine's clutch feed-forward must carry current charge,
    // and fuel-cut limiter pulses must not command the clutch to reopen.
    CHECK(high.torque_demand>=high.tau0-boosted.maps().c_k-1e-6);
    SimulatedEngine limited(desc);
    (void)hold(limited,w_of(7500),1.0,14400);
    const auto cut=hold(limited,w_of(8300),1.0,1);
    CHECK(cut.limiter_active);
    CHECK(cut.tau0<0);
    CHECK(cut.torque_demand>1000);
    TcuRequest shift_cut; shift_cut.ignition_cut=true;
    const auto shifting=limited.pre_solve(running(w_of(8300)),cmd(1),shift_cut,kH);
    CHECK(shifting.torque_demand<0); // deliberate shift cut still bounds flywheel-energy transfer


    // Spool: part throttle at 4500 rpm, then WOT; time to 90 % of the boost target.
    {
        SimulatedEngine sp(desc);
        (void)hold(sp,w_of(4500),0.2,4800);
        int i90=-1;
        for(int i=0;i<4800 && i90<0;++i) {
            const auto r=sp.pre_solve(running(w_of(4500)),cmd(1),TcuRequest{},kH);
            (void)sp.post_solve(w_of(4500),kH);
            if(r.p_charge_pa-kPamb>0.9*desc.turbo_pair->target_boost_pa) i90=i;
        }
        std::printf("[turbo dyno] spool 4500 rpm, 0.2 -> WOT: 90 %% of target boost after %.3f s\n",i90*kH);
        CHECK(i90>0);
    }
    const auto hash_before=boosted.fold_state_hash(1);
    auto replacement=*desc.turbo_pair;
    replacement.rotor_inertia_kgm2*=2;
    boosted.replace_turbo_configuration(replacement);
    CHECK(boosted.fold_state_hash(1)!=hash_before);
    const auto after_swap=boosted.pre_solve(running(w_of(7500)),cmd(1),TcuRequest{},kH);
    CHECK(std::abs(after_swap.turbo_rpm-high.turbo_rpm)<1000);
    CHECK(std::abs(after_swap.p_intake_pa-high.p_intake_pa)<2000);
    (void)boosted.post_solve(w_of(7500),kH);
    const auto lifted=hold(boosted,w_of(6000),0.0,960);
    CHECK(lifted.turbo_rpm<high.turbo_rpm); CHECK(lifted.boost_bar<br.boost_bar);
    auto s=running(0); s.ignition=0;
    for(int i=0;i<19200;++i) { (void)boosted.pre_solve(s,cmd(0),TcuRequest{},kH); (void)boosted.post_solve(0,kH); }
    auto stopped=boosted.pre_solve(s,cmd(0),TcuRequest{},kH);
    // Engine stopped for 20 s: no exhaust, so nothing adds rotor energy; the shaft loss P_ref*(w/w_ref)^2 alone
    // decays it exponentially with tau = J*w_ref^2/P_ref (121 s for the doubled shipped 2.3e-4 kg*m^2 rotor) -
    // an upper bound. The compressor refilling the manifold after the stop takes the rest: measured 100509 rpm
    // = 0.947 x the shaft-loss-only 106100 (from 125159). Was a fixed "< 10000 rpm", which only the old
    // 2e-5 [AS] rotor met (tau 10.5 s).
    {
        const real wr=replacement.reference_rpm*2*kPi/60;
        const real tau=replacement.rotor_inertia_kgm2*wr*wr/replacement.shaft_loss_w_at_reference;
        const real expected=lifted.turbo_rpm*std::exp(-20.0/tau);
        std::printf("[turbo dyno] stopped 20 s: rotor %.0f -> %.0f rpm, shaft-loss decay alone %.0f rpm (tau %.1f s)\n",
                    lifted.turbo_rpm,stopped.turbo_rpm,expected,tau);
        CHECK(stopped.turbo_rpm<lifted.turbo_rpm);
        CHECK(stopped.turbo_rpm<=expected*1.001);
        CHECK(stopped.turbo_rpm>=expected*0.90);
    }
    TurboPair pair(*desc.turbo_pair); pair.reset(101325,288.15);
    for(int i=0;i<9600;++i) pair.step(kH,101325,288.15,0,0,0,1);
    CHECK(pair.energy_j==0); CHECK(pair.charge_pa==101325);
}
