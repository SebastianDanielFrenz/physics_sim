// tests/unit/test_engine_cycle.cpp - PHYS-033 P4 I7c C15a: the 0D crank-angle
// cycle model core (core/src/engine_cycle, white-box). Guards G1-G11 of
// out/i7c_plan.md section 5 C15a; each names the defect it catches. The
// engine is tests/support/cycle_fixtures.h's GENERIC 2 L I4 (ASSUMED
// geometry, not the A25A). [.][diag] cases print the G7/G9 tables and the
// cost figures.

#include "cycle_fixtures.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/geometry.h"
#include "ps/engine_cycle/heat_transfer.h"
#include "ps/engine_cycle/valve_flow.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

real rel(real a, real b) {
    const real s = std::fabs(b) > 0.0 ? std::fabs(b) : 1.0;
    return std::fabs(a - b) / s;
}

// Constant-gamma, single-species gas (both groups identical): cv = R/(g-1).
drivetrain::CycleModelDesc constant_gamma_desc(real gamma) {
    drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const real r = 287.0;
    d.gas.fresh = drivetrain::GasGroupDesc{r / (gamma - 1.0), 0.0, r, gamma};
    d.gas.burned = d.gas.fresh;
    return d;
}

OperatingPoint make_op(const drivetrain::CycleModelDesc& d, real rpm, real p_im) {
    OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_im;
    op.t_im_k = 300.0;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);
    return op;
}

real imep(real w, const CycleStepper& s) { return w / s.model().geom.v_d; }

// Closed cylinder filled with gas at (p0, T0) at crank angle theta0, fresh group.
void fill_closed_cylinder(CycleStepper& s, real theta0, real p0, real t0) {
    CycleState st = s.state();
    const CycleModel& m = s.model();
    const real v = m.geom.eval(theta0).v;
    const real mass = p0 * v / (m.gas.fresh.r * t0);
    st.y[kCylM] = mass;
    st.y[kCylMb] = 0.0;
    st.y[kCylU] = energy_from_temperature(m.gas, mass, 0.0, t0);
    for (int i = kPhysicalCount; i < kStateCount; ++i) st.y[i] = 0.0;
    s.set_state(st, theta0);
    s.reset_peak_pressure();
}

} // namespace

// ---------------------------------------------------------------------------
TEST_CASE("G1 cycle geometry: volumes, dV/dtheta and Heywood's slider-crank", "[drivetrain][cycle]") {
    // Catches: offset sign, rod-ratio or TDC-angle errors.
    const real bore = 0.086, stroke = 0.086, rod = 0.145, cr = 10.5;
    SECTION("e = 0 equals Heywood eq. 2.4 to 1e-14") {
        const CrankGeometry g = CrankGeometry::make(bore, stroke, rod, 0.0, cr);
        const real a = 0.5 * stroke;
        const real ap = kPi * bore * bore / 4.0;
        const real vd = ap * stroke;
        const real vc = vd / (cr - 1.0);
        for (int i = 0; i <= 72; ++i) {
            const real th = 10.0 * static_cast<real>(i);
            real sn = 0.0, cs = 0.0;
            ps::math::sincos(th * kDegToRad, sn, cs);
            const real s = a * cs + std::sqrt(rod * rod - a * a * sn * sn);
            const real v_ref = vc + ap * (rod + a - s);
            CHECK(rel(g.eval(th).v, v_ref) < 1e-14);
        }
    }
    for (const real e : {0.0, 0.0012, -0.0012}) {
        const CrankGeometry g = CrankGeometry::make(bore, stroke, rod, e, cr);
        CAPTURE(e);
        CHECK(rel(g.eval(0.0).v, g.v_c) < 1e-12);
        CHECK(rel(g.eval(360.0).v, g.v_c) < 1e-12);
        CHECK(rel(g.eval(g.theta_bdc_deg).v, g.v_c + g.v_d) < 1e-12);
        CHECK(rel(g.v_d / g.v_c + 1.0, cr) < 1e-12);
        // TDC/BDC are the volume extrema.
        CHECK(g.eval(0.01).v > g.v_c);
        CHECK(g.eval(g.theta_bdc_deg + 0.01).v < g.v_c + g.v_d);
        CHECK(g.eval(g.theta_bdc_deg - 0.01).v < g.v_c + g.v_d);
        for (int i = 0; i < 36; ++i) {
            const real th = 7.3 + 10.0 * static_cast<real>(i);
            const real dh = 1e-4;
            const real fd = (g.eval(th + dh).v - g.eval(th - dh).v) / (2.0 * dh);
            const real dv = g.eval(th).dv_ddeg;
            CHECK(std::fabs(dv - fd) <= 1e-8 * g.v_d / 57.3 + 1e-8 * std::fabs(dv));
        }
        if (e != 0.0) {
            // True stroke with offset is longer than 2r; BDC is not at 180.
            CHECK(g.v_d > kPi * bore * bore / 4.0 * stroke);
            CHECK(g.theta_bdc_deg != 180.0);
        }
    }
}

// ---------------------------------------------------------------------------
TEST_CASE("G2 cycle motoring closed cylinder: p_max = p0*r^gamma, zero net work", "[drivetrain][cycle]") {
    // Catches: energy-equation errors, the p*dV sign, integrator phase error.
    const real gamma = 1.4;
    const drivetrain::CycleModelDesc d = constant_gamma_desc(gamma);
    CycleStepper s(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    s.reset(make_op(d, 3000.0, 1.0e5));
    s.set_switches(CycleSwitches{false, false, false});
    const real p0 = 1.0e5, t0 = 300.0;
    fill_closed_cylinder(s, 180.0, p0, t0);
    const int n = 2000;
    const real h = 180.0 / (6.0 * 3000.0 * n);
    for (int i = 0; i < n; ++i) s.step(h);
    const real w_comp = s.state().y[kAccWork];
    const CrankGeometry& g = s.model().geom;
    const real r = (g.v_c + g.v_d) / g.v_c;
    const real p_expected = p0 * ps::math::pow(r, gamma);
    INFO("p_max " << s.p_max() << " expected " << p_expected);
    CHECK(rel(s.p_max(), p_expected) < 1e-6);
    CHECK(w_comp < 0.0);
    for (int i = 0; i < n; ++i) s.step(h);
    const real w_net = s.state().y[kAccWork];
    INFO("net work " << w_net << " compression work " << w_comp);
    CHECK(std::fabs(w_net / w_comp) < 1e-6);
}

// ---------------------------------------------------------------------------
TEST_CASE("G3 cycle Otto limit: eta = 1 - r^(1-gamma) with a 2 deg burn", "[drivetrain][cycle]") {
    // Catches: LHV/fuel-mass units, a missing heat-release term.
    const real gamma = 1.4;
    drivetrain::CycleModelDesc d = constant_gamma_desc(gamma);
    d.burn_duration_deg = 2.0;
    // Huge LHV and a tiny fuel mass: the heat of a stoichiometric charge with
    // 1/1000 of the fuel mass, so the added fuel mass (which the ideal Otto
    // cycle does not have) changes the charge by 7e-5 only.
    const real lhv = test::kCycleFixtureLhv * 1000.0;
    CycleStepper s(d, lhv, test::kCycleFixtureAfr);
    OperatingPoint op = make_op(d, 3000.0, 1.0e5);
    op.spark_deg_atdc = -1.0;
    s.reset(op);
    s.set_switches(CycleSwitches{false, true, false});
    fill_closed_cylinder(s, 180.0, 1.0e5, 300.0);
    const real m_air = s.state().y[kCylM];
    s.set_charge(m_air, m_air / (test::kCycleFixtureAfr * 1000.0));
    const int n = 4000;
    const real h = 360.0 / (6.0 * 3000.0 * n);
    for (int i = 0; i < n; ++i) s.step(h);
    const real w = s.state().y[kAccWork];
    const real q = s.state().y[kAccQComb];
    const CrankGeometry& g = s.model().geom;
    const real r = (g.v_c + g.v_d) / g.v_c;
    const real eta_otto = 1.0 - ps::math::pow(r, 1.0 - gamma);
    const real eta = w / q;
    INFO("eta " << eta << " otto " << eta_otto << " finite-burn/fuel-mass deviation " << (eta / eta_otto - 1.0));
    CHECK(q > 0.0);
    // All fuel burnt: the RK4 quadrature of the 2 deg Wiebe rate over ~22
    // steps is exact to ~2e-6; the heat equals eta_c * LHV * fuel at rounding.
    CHECK(rel(q, 0.98 * s.m_fuel() * lhv) < 1e-5);
    CHECK(rel(q, 0.98 * s.state().y[kAccFuel] * lhv) < 1e-12);
    CHECK(rel(eta, eta_otto) < 0.005);
    CHECK(eta < eta_otto);  // a finite burn can only lose efficiency
}

// ---------------------------------------------------------------------------
TEST_CASE("G4 cycle Wiebe: fraction positions and integral", "[drivetrain][cycle]") {
    // Catches: a wrong exponent (m vs m+1).
    const Wiebe w{5.0, 2.0, 60.0, 340.0};
    for (const real x : {0.1, 0.5, 0.9}) {
        const real pos = 340.0 + 60.0 * std::cbrt(-std::log1p(-x) / 5.0);
        CAPTURE(x);
        CHECK(std::fabs(w.angle_at_fraction(x) - pos) < 1e-12 * 360.0);
        CHECK(std::fabs(w.fraction(pos) - x) < 1e-12);
    }
    // Simpson over [theta0, theta0 + 2.5*dtheta] (the cutoff lies at y = 2).
    const int n = 200000;
    const real a = 340.0, b = 340.0 + 150.0;
    const real hh = (b - a) / n;
    real sum = w.rate_per_deg(a) + w.rate_per_deg(b);
    for (int i = 1; i < n; ++i) sum += (i % 2 ? 4.0 : 2.0) * w.rate_per_deg(a + hh * i);
    const real integral = sum * hh / 3.0;
    INFO("integral " << integral);
    CHECK(std::fabs(integral - 1.0) < 1e-9);
    // The rate is the derivative of the fraction.
    for (const real th : {345.0, 360.0, 371.3, 390.0}) {
        const real fd = (w.fraction(th + 1e-5) - w.fraction(th - 1e-5)) / 2e-5;
        CHECK(std::fabs(w.rate_per_deg(th) - fd) < 1e-8);
    }
    CHECK(rel(Wiebe::theta0_for_ca50(5.0, 2.0, 60.0, 368.0) + 60.0 * std::cbrt(std::log(2.0) / 5.0), 368.0) < 1e-14);
}

// ---------------------------------------------------------------------------
TEST_CASE("G5 cycle two-volume blowdown: mass, energy, composition conserved", "[drivetrain][cycle]") {
    // Catches: orifice direction or sign, enthalpy not transported, composition leak.
    // Production gas/valve helpers, RK4 in the test. Run twice with the
    // high-pressure volume on either side, so both orifice branches are used.
    const GasModel g = GasModel::from_desc(drivetrain::GasPropertiesDesc{});
    PsiPair psi;
    psi.fresh = PsiClass::make(1.40);
    psi.burned = PsiClass::make(1.30);
    const real cda = 1.0e-5;
    for (const bool hi_on_1 : {true, false}) {
        CAPTURE(hi_on_1);
        const real v1 = hi_on_1 ? 1.0e-3 : 2.0e-3;
        const real v2 = hi_on_1 ? 2.0e-3 : 1.0e-3;
        struct Y { real m1, u1, mb1, m2, u2, mb2; };
        auto f = [&](const Y& y) {
            const CvState a = cv_state(g, y.m1, y.u1, y.mb1, v1);
            const CvState b = cv_state(g, y.m2, y.u2, y.mb2, v2);
            const OrificeFlow of = orifice_flow(psi, cda, a.p, a.r, a.t, a.x, b.p, b.r, b.t, b.x);
            const real x_up = of.from_1 ? a.x : b.x;
            const real h_up = of.from_1 ? g.h_mix(a.t, a.x) : g.h_mix(b.t, b.x);
            return Y{-of.mdot, -of.mdot * h_up, -of.mdot * x_up, of.mdot, of.mdot * h_up, of.mdot * x_up};
        };
        auto axpy = [](const Y& y, real h, const Y& k) {
            return Y{y.m1 + h * k.m1, y.u1 + h * k.u1, y.mb1 + h * k.mb1,
                     y.m2 + h * k.m2, y.u2 + h * k.u2, y.mb2 + h * k.mb2};
        };
        // High side: burned at 5 bar / 1000 K, 1 l. Low side: fresh at 1 bar / 300 K, 2 l.
        const real m_hi = 5.0e5 * 1.0e-3 / (g.burned.r * 1000.0);
        const real u_hi = energy_from_temperature(g, m_hi, m_hi, 1000.0);
        const real m_lo = 1.0e5 * 2.0e-3 / (g.fresh.r * 300.0);
        const real u_lo = energy_from_temperature(g, m_lo, 0.0, 300.0);
        Y y = hi_on_1 ? Y{m_hi, u_hi, m_hi, m_lo, u_lo, 0.0} : Y{m_lo, u_lo, 0.0, m_hi, u_hi, m_hi};
        const real m0 = y.m1 + y.m2, u0 = y.u1 + y.u2, mb0 = y.mb1 + y.mb2;
        const real h = 2.0e-5;
        // Direction: the first step moves mass from the high side to the low side.
        const Y k0 = f(y);
        CHECK((hi_on_1 ? k0.m1 : k0.m2) < 0.0);
        for (int i = 0; i < 50000; ++i) {
            const Y k1 = f(y);
            const Y k2 = f(axpy(y, 0.5 * h, k1));
            const Y k3 = f(axpy(y, 0.5 * h, k2));
            const Y k4 = f(axpy(y, h, k3));
            const real h6 = h / 6.0;
            y.m1 += h6 * (k1.m1 + 2.0 * k2.m1 + 2.0 * k3.m1 + k4.m1);
            y.u1 += h6 * (k1.u1 + 2.0 * k2.u1 + 2.0 * k3.u1 + k4.u1);
            y.mb1 += h6 * (k1.mb1 + 2.0 * k2.mb1 + 2.0 * k3.mb1 + k4.mb1);
            y.m2 += h6 * (k1.m2 + 2.0 * k2.m2 + 2.0 * k3.m2 + k4.m2);
            y.u2 += h6 * (k1.u2 + 2.0 * k2.u2 + 2.0 * k3.u2 + k4.u2);
            y.mb2 += h6 * (k1.mb2 + 2.0 * k2.mb2 + 2.0 * k3.mb2 + k4.mb2);
        }
        const CvState a = cv_state(g, y.m1, y.u1, y.mb1, v1);
        const CvState b = cv_state(g, y.m2, y.u2, y.mb2, v2);
        const CvState& hi = hi_on_1 ? a : b;
        const CvState& lo = hi_on_1 ? b : a;
        INFO("p_hi " << hi.p << " p_lo " << lo.p << " T_hi " << hi.t << " T_lo " << lo.t);
        CHECK(rel(y.m1 + y.m2, m0) < 1e-13);
        CHECK(rel(y.u1 + y.u2, u0) < 1e-12);
        CHECK(rel(y.mb1 + y.mb2, mb0) < 1e-13);
        CHECK(rel(a.p, b.p) < 1e-6);
        // Burned gas has moved into the low side; the high side kept x_b = 1.
        CHECK(lo.mb > 0.1 * mb0);
        CHECK(rel(hi.x, 1.0) < 1e-12);
        CHECK(hi.p < 5.0e5);
        CHECK(lo.p > 1.0e5);
    }
}

// ---------------------------------------------------------------------------
TEST_CASE("G6 cycle open-system energy and mass closure per cycle (bookkeeping only)", "[drivetrain][cycle]") {
    // BOOKKEEPING-ONLY (standing rule): catches an RHS/accumulator mismatch.
    // The physics is checked by G2/G3/G7/G8.
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    CycleStepper s(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    for (const real p_im : {30.0e3, 100.0e3}) {
        CAPTURE(p_im);
        s.reset(make_op(d, 3000.0, p_im));
        const std::uint64_t n = steps_per_cycle(3000.0, kStepFull);
        for (std::uint64_t k = 0; k < 3 * n; ++k) s.step_aligned(k, n);
        s.reset_accumulators();
        const CycleState st0 = s.state();
        for (std::uint64_t k = 0; k < n; ++k) s.step_aligned(k, n);
        const real* y = s.state().y;
        const real du_cyl = y[kCylU] - st0.y[kCylU];
        const real rhs_cyl = -y[kAccWork] + y[kAccIvEnthalpy] - y[kAccEvEnthalpy] + y[kAccQComb] - y[kAccQWall];
        const real scale = std::fabs(y[kAccQComb]) + std::fabs(y[kAccWork]) + std::fabs(y[kAccEvEnthalpy]);
        INFO("dU_cyl " << du_cyl << " sum " << rhs_cyl << " scale " << scale);
        CHECK(std::fabs(du_cyl - rhs_cyl) < 1e-9 * scale);
        const real du_sys = (y[kCylU] + y[kIpU] + y[kEpU]) - (st0.y[kCylU] + st0.y[kIpU] + st0.y[kEpU]);
        const real rhs_sys = -y[kAccWork] + y[kAccRunnerEnthalpy] - y[kAccResEnthalpy] + y[kAccQComb] - y[kAccQWall];
        CHECK(std::fabs(du_sys - rhs_sys) < 1e-9 * scale);
        const real dm_sys = (y[kCylM] + y[kIpM] + y[kEpM]) - (st0.y[kCylM] + st0.y[kIpM] + st0.y[kEpM]);
        const real rhs_m = y[kAccRunnerMass] - y[kAccResMass] + y[kAccFuel];
        CHECK(std::fabs(dm_sys - rhs_m) < 1e-9 * std::fabs(y[kAccRunnerMass]));
        // Sanity: a firing cycle released heat and did positive work.
        CHECK(y[kAccQComb] > 0.0);
        CHECK(y[kAccWork] > 0.0);
    }
}

// ---------------------------------------------------------------------------
namespace {
struct G7Point { real rpm, p_im; };
const G7Point kG7Points[] = {{1000.0, 30.0e3}, {1000.0, 100.0e3}, {3000.0, 30.0e3},
                             {3000.0, 100.0e3}, {6600.0, 30.0e3}, {6600.0, 100.0e3}};
} // namespace

TEST_CASE("G7 cycle step convergence: h_full and h_preview vs h_full/4", "[drivetrain][cycle]") {
    // Catches: a too-coarse h, a stiff-flow bug.
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    for (const G7Point& pt : kG7Points) {
        CAPTURE(pt.rpm, pt.p_im);
        const OperatingPoint op = make_op(d, pt.rpm, pt.p_im);
        const PointResult ref = run_operating_point(proto, op, StepperSettings{kStepFull / 4.0, kCyclesFull});
        const PointResult full = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull});
        const PointResult prev = run_operating_point(proto, op, StepperSettings{kStepPreview, kCyclesFull});
        const real g_ref = imep(ref.w_gross_j, proto);
        const real n_ref = imep(ref.w_gross_j + ref.w_pump_j, proto);
        INFO("gross " << g_ref << " net " << n_ref << " m_air " << ref.m_air_j_cyc);
        CHECK(rel(imep(full.w_gross_j, proto), g_ref) < 0.0015);
        CHECK(std::fabs(imep(full.w_gross_j + full.w_pump_j, proto) - n_ref) < 0.0015 * g_ref);
        CHECK(rel(full.m_air_j_cyc, ref.m_air_j_cyc) < 0.0015);
        CHECK(rel(imep(prev.w_gross_j, proto), g_ref) < 0.005);
        CHECK(std::fabs(imep(prev.w_gross_j + prev.w_pump_j, proto) - n_ref) < 0.005 * g_ref);
        CHECK(rel(prev.m_air_j_cyc, ref.m_air_j_cyc) < 0.005);
    }
    // PHYS-038: crank-angle stepping (1 deg at 1000 rpm, h = 166.7 us). The
    // plan expected it outside 10 % (PHYS-038 measured +51 % IMEP in its own
    // study model). THIS model does not reproduce that: measured +0.61 %
    // gross IMEP / +0.49 % air at 100 kPa, +2.6 % / +1.7 % at 30 kPa. C15a
    // review: the 1 deg error is set by the open-valve flow stiffness (CdA
    // times the regularised Psi slope), not by crank-angle stepping as such
    // nor by the port/runner volumes. PHYS-038's quoted +51 % is its V12
    // (I4 +30.8 %); halving its valve area alone drops the I4 to -0.39 %
    // (threshold near 0.65x, non-monotonic above), while runner volume x4 or
    // orifice area /4 leave it unchanged (+31.2 % / +30.8 %). This fixture's
    // CdA*Psi' is ~0.52x PHYS-038's, below that threshold; with 4 valves the
    // 100 kPa error here grows to +2.2 % (review diag case at the end of this
    // file). What holds, and is asserted, is that 1 deg stepping is far worse
    // than h_full.
    const OperatingPoint op = make_op(d, 1000.0, 100.0e3);
    const PointResult ref = run_operating_point(proto, op, StepperSettings{kStepFull / 4.0, kCyclesFull});
    const PointResult full = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull});
    const PointResult coarse = run_operating_point(proto, op, StepperSettings{1.0 / 6000.0, kCyclesFull});
    const real e_g = rel(coarse.w_gross_j, ref.w_gross_j);
    const real e_m = rel(coarse.m_air_j_cyc, ref.m_air_j_cyc);
    const real f_g = rel(full.w_gross_j, ref.w_gross_j);
    const real f_m = rel(full.m_air_j_cyc, ref.m_air_j_cyc);
    INFO("1 deg stepping: gross IMEP error " << e_g << ", air error " << e_m << "; h_full " << f_g << ", " << f_m);
    const bool nonfinite = !std::isfinite(coarse.w_gross_j) || !std::isfinite(coarse.m_air_j_cyc);
    CHECK((nonfinite || (e_g > 10.0 * f_g && e_m > 10.0 * f_m && e_g > 0.003)));
}

// ---------------------------------------------------------------------------
TEST_CASE("G8 cycle Woschni against hand-evaluated literals", "[drivetrain][cycle]") {
    // Catches: Pa vs kPa, exponent sign. Literals from Python 3 (float64,
    // Heywood 1988 eq. 12.19 written term by term: B^-0.2 * (p/1000)^0.8 *
    // T^-0.55 * w^0.8).
    WoschniInput w;
    w.bore_m = 0.086;
    w.p_pa = 4.0e6;
    w.t_k = 1800.0;
    w.v_m3 = 1.0e-4;
    w.mean_piston_speed = 8.6;
    w.c1 = 2.28;
    w.c2 = 3.24e-3;
    w.v_d = 4.99e-4;
    w.p_ref_pa = 0.9e5;
    w.v_ref_m3 = 5.2e-4;
    w.t_ref_k = 350.0;
    CHECK(rel(woschni_velocity(w), 57.895997953416185) < 1e-12);
    CHECK(rel(woschni_h(w), 1689.2147325596234) < 1e-12);
    // Gas exchange: C1 = 6.18, no combustion term.
    WoschniInput gx;
    gx.bore_m = 0.086;
    gx.p_pa = 1.2e5;
    gx.t_k = 400.0;
    gx.v_m3 = 3.0e-4;
    gx.mean_piston_speed = 8.6;
    gx.c1 = 6.18;
    CHECK(rel(woschni_velocity(gx), 53.147999999999996) < 1e-12);
    CHECK(rel(woschni_h(gx), 218.2325123945232) < 1e-12);
}

// ---------------------------------------------------------------------------
TEST_CASE("G9 cycle count convergence and determinism", "[drivetrain][cycle]") {
    // Catches: a non-converged fixed N; hidden mutable/static state.
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const G7Point corners[] = {{1000.0, 30.0e3}, {1000.0, 100.0e3}, {6600.0, 30.0e3}, {6600.0, 100.0e3}};
    for (const G7Point& pt : corners) {
        CAPTURE(pt.rpm, pt.p_im);
        const OperatingPoint op = make_op(d, pt.rpm, pt.p_im);
        const PointResult a = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull});
        const PointResult b = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull + 4});
        const real ga = imep(a.w_gross_j + a.w_pump_j, proto), gb = imep(b.w_gross_j + b.w_pump_j, proto);
        const real gg = imep(b.w_gross_j, proto);
        INFO("net IMEP N " << ga << " N+4 " << gb);
        CHECK(std::fabs(ga - gb) < 0.002 * gg);
        CHECK(rel(a.w_gross_j, b.w_gross_j) < 0.002);
        CHECK(rel(a.m_air_j_cyc, b.m_air_j_cyc) < 0.002);
        const PointResult c = run_operating_point(proto, op, StepperSettings{kStepPreview, kCyclesPreview});
        const PointResult e = run_operating_point(proto, op, StepperSettings{kStepPreview, kCyclesPreview + 4});
        CHECK(std::fabs(imep(c.w_gross_j + c.w_pump_j, proto) - imep(e.w_gross_j + e.w_pump_j, proto)) <
              0.002 * gg);
        CHECK(rel(c.m_air_j_cyc, e.m_air_j_cyc) < 0.002);
    }
    // Bit-identical when repeated and in reverse order.
    std::vector<PointResult> fwd, rev;
    for (const G7Point& pt : corners) {
        fwd.push_back(run_operating_point(proto, make_op(d, pt.rpm, pt.p_im), StepperSettings{kStepPreview, 3}));
    }
    for (int i = 3; i >= 0; --i) {
        rev.push_back(run_operating_point(proto, make_op(d, corners[i].rpm, corners[i].p_im),
                                          StepperSettings{kStepPreview, 3}));
    }
    for (int i = 0; i < 4; ++i) {
        const PointResult& a = fwd[static_cast<std::size_t>(i)];
        const PointResult& b = rev[static_cast<std::size_t>(3 - i)];
        CHECK(a.w_gross_j == b.w_gross_j);
        CHECK(a.w_pump_j == b.w_pump_j);
        CHECK(a.m_air_j_cyc == b.m_air_j_cyc);
        CHECK(a.m_fuel_kg == b.m_fuel_kg);
        CHECK(a.residual_frac == b.residual_frac);
        CHECK(a.t_exh_k == b.t_exh_k);
        CHECK(a.h_exh_j == b.h_exh_j);
        CHECK(a.p_max_pa == b.p_max_pa);
        CHECK(a.steps == b.steps);
    }
    const PointResult again = run_operating_point(proto, make_op(d, 1000.0, 30.0e3), StepperSettings{kStepPreview, 3});
    CHECK(again.w_gross_j == fwd[0].w_gross_j);
    CHECK(again.m_air_j_cyc == fwd[0].m_air_j_cyc);
}

// ---------------------------------------------------------------------------
TEST_CASE("G10 cycle run_operating_point allocates nothing inside a job", "[drivetrain][cycle]") {
    // Catches: a heap allocation in the stepper.
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const OperatingPoint op = make_op(d, 6600.0, 100.0e3);
    ps::jobs::JobSystem js(2);
    PointResult r{};
    auto run = [&] {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit([&] { r = run_operating_point(proto, op, StepperSettings{kStepPreview, 2}); }, &wg);
        js.wait(wg);
    };
    run();  // warm-up
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    run();
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
    CHECK(r.w_gross_j > 0.0);
}

// ---------------------------------------------------------------------------
TEST_CASE("G11 cycle low-speed LIVC volumetric efficiency", "[drivetrain][cycle]") {
    // Catches: a backflow direction error (LIVC push-back missing).
    // 200 rpm, motored, adiabatic, no overlap, EVO = 720 - IVC, p_im = p_amb,
    // T_im = backflow T = 300 K. Hand estimate: the charge fills the cylinder
    // to plenum density; everything past IVC's volume is pushed back, and the
    // clearance volume keeps its residual: VE = (V(IVC) - V_c)/V_d.
    drivetrain::CycleModelDesc d = test::make_livc_cycle_desc();
    d.exhaust_backflow_temperature_k = 300.0;
    CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    proto.set_switches(CycleSwitches{true, false, false});
    OperatingPoint op = make_op(d, 200.0, 101325.0);
    const PointResult r = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull});
    const CrankGeometry& g = proto.model().geom;
    const real rho = op.p_im_pa / (proto.model().gas.fresh.r * op.t_im_k);
    const real ve = r.m_air_j_cyc / (rho * g.v_d);
    CycleStepper probe = proto;
    probe.reset(op);
    const real v_ivc = g.eval(probe.theta_ivc()).v;
    const real ve_hand = (v_ivc - g.v_c) / g.v_d;
    INFO("VE " << ve << " hand " << ve_hand << " IVC " << probe.theta_ivc());
    CHECK(rel(ve, ve_hand) < 0.05);
    CHECK(ve < 0.95);  // without push-back the VE would be ~1
}

// ---------------------------------------------------------------------------
// Diagnostics (not run by default): the G7/G9 tables and the cost figures.
TEST_CASE("cycle diag: G7 step table", "[.][diag][drivetrain][cycle]") {
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    std::printf("G7: rpm p_im_kPa | IMEPg_ref IMEPn_ref m_air_ref | dg_full dn_full dm_full | dg_prev dn_prev dm_prev | dg_1deg dm_1deg (1000 rpm only)\n");
    for (const G7Point& pt : kG7Points) {
        const OperatingPoint op = make_op(d, pt.rpm, pt.p_im);
        const PointResult ref = run_operating_point(proto, op, StepperSettings{kStepFull / 4.0, kCyclesFull});
        const PointResult full = run_operating_point(proto, op, StepperSettings{kStepFull, kCyclesFull});
        const PointResult prev = run_operating_point(proto, op, StepperSettings{kStepPreview, kCyclesFull});
        const real g = imep(ref.w_gross_j, proto), n = imep(ref.w_gross_j + ref.w_pump_j, proto);
        auto dn = [&](const PointResult& x) { return (imep(x.w_gross_j + x.w_pump_j, proto) - n) / g; };
        std::printf("G7: %5.0f %5.0f | %.5e %.5e %.5e | %+.2e %+.2e %+.2e | %+.2e %+.2e %+.2e", pt.rpm,
                    pt.p_im / 1000.0, g, n, ref.m_air_j_cyc, imep(full.w_gross_j, proto) / g - 1.0, dn(full),
                    full.m_air_j_cyc / ref.m_air_j_cyc - 1.0, imep(prev.w_gross_j, proto) / g - 1.0, dn(prev),
                    prev.m_air_j_cyc / ref.m_air_j_cyc - 1.0);
        if (pt.rpm == 1000.0) {
            const PointResult c = run_operating_point(proto, op, StepperSettings{1.0 / 6000.0, kCyclesFull});
            std::printf(" | %+.2e %+.2e", imep(c.w_gross_j, proto) / g - 1.0, c.m_air_j_cyc / ref.m_air_j_cyc - 1.0);
        }
        std::printf("   [resid %.4f T_exh %.1f p_max %.3e]\n", ref.residual_frac, ref.t_exh_k, ref.p_max_pa);
    }
}

TEST_CASE("cycle diag: G9 cycle-count table", "[.][diag][drivetrain][cycle]") {
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const G7Point corners[] = {{1000.0, 30.0e3}, {1000.0, 100.0e3}, {6600.0, 30.0e3}, {6600.0, 100.0e3}};
    for (const real h : {kStepFull, kStepPreview}) {
        for (const G7Point& pt : corners) {
            const OperatingPoint op = make_op(d, pt.rpm, pt.p_im);
            std::printf("G9 h=%.3e %5.0f rpm %5.0f kPa: (N, netIMEP rel to N=16 / gross, m_air rel)", h, pt.rpm,
                        pt.p_im / 1000.0);
            const PointResult ref = run_operating_point(proto, op, StepperSettings{h, 16});
            const real g = imep(ref.w_gross_j, proto), n = imep(ref.w_gross_j + ref.w_pump_j, proto);
            for (int N = 2; N <= 12; ++N) {
                const PointResult x = run_operating_point(proto, op, StepperSettings{h, N});
                std::printf(" (%d %+.1e %+.1e)", N, (imep(x.w_gross_j + x.w_pump_j, proto) - n) / g,
                            x.m_air_j_cyc / ref.m_air_j_cyc - 1.0);
            }
            std::printf("\n");
        }
    }
}

TEST_CASE("cycle diag: cost per RHS evaluation and per point", "[.][diag][drivetrain][cycle]") {
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    CycleStepper s = proto;
    s.reset(make_op(d, 3000.0, 100.0e3));
    const std::uint64_t n = steps_per_cycle(3000.0, kStepFull);
    for (std::uint64_t k = 0; k < 2 * n; ++k) s.step_aligned(k, n);
    const CycleState st = s.state();
    CycleState out;
    const int evals = 2000000;
    real sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < evals; ++i) {
        s.rhs(720.0 * static_cast<real>(i % 7200) / 7200.0, st, out);
        sink += out.y[kCylU];
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / evals;
    std::printf("cost: %.1f ns per RHS evaluation (one cylinder, 3 CVs, all 720 deg) [sink %g]\n", ns, sink);
    for (const real rpm : {1000.0, 3000.0, 6600.0}) {
        for (const auto& hs : {std::pair<real, int>{kStepFull, kCyclesFull}, std::pair<real, int>{kStepPreview, kCyclesPreview}}) {
            const OperatingPoint op = make_op(d, rpm, 100.0e3);
            const int reps = 5;
            PointResult r{};
            const auto a = std::chrono::steady_clock::now();
            for (int i = 0; i < reps; ++i) r = run_operating_point(proto, op, StepperSettings{hs.first, hs.second});
            const auto b = std::chrono::steady_clock::now();
            std::printf("cost: %5.0f rpm h=%.3e N=%d: %.2f ms per point (%llu steps)\n", rpm, hs.first, hs.second,
                        std::chrono::duration<double, std::milli>(b - a).count() / reps,
                        static_cast<unsigned long long>(r.steps));
        }
    }
}

// C15a review (Opus): P8 stability-edge check and the PHYS-038 bridge.
// Part A sweeps h at the G7 points; below RK4's real-axis edge the error
// scales ~h^4, at the edge it jumps. Part B repeats 1 deg stepping at
// 1000 rpm with more valves per cylinder (curtain area x1.5 / x2): PHYS-038's
// +31..51 % came from its larger valve CdA x Psi slope (its own bench:
// halving the valve areas drops the 1 deg IMEP error from +30.8 % to -0.39 %).
TEST_CASE("cycle diag: review h sweep and 1 deg valve-area probe", "[.][diag][drivetrain][cycle]") {
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const real h_us[] = {21.70, 30.0, 36.0, 39.06, 43.40, 47.74, 52.0, 60.0, 70.0, 86.8, 120.0, 166.7};
    std::printf("REV-A: rpm p_im_kPa h_us(req) h_us(used) | dg dn dm (rel. to h_full/4)\n");
    for (const G7Point& pt : kG7Points) {
        const OperatingPoint op = make_op(d, pt.rpm, pt.p_im);
        const PointResult ref = run_operating_point(proto, op, StepperSettings{kStepFull / 4.0, kCyclesFull});
        const real g = imep(ref.w_gross_j, proto), n = imep(ref.w_gross_j + ref.w_pump_j, proto);
        for (const real hu : h_us) {
            const real h = hu * 1.0e-6;
            const PointResult r = run_operating_point(proto, op, StepperSettings{h, kCyclesFull});
            const real h_used = 120.0 / (pt.rpm * static_cast<real>(steps_per_cycle(pt.rpm, h)));
            std::printf("REV-A: %5.0f %5.0f %7.2f %7.2f | %+.3e %+.3e %+.3e\n", pt.rpm, pt.p_im / 1000.0, hu, h_used * 1e6,
                        imep(r.w_gross_j, proto) / g - 1.0, (imep(r.w_gross_j + r.w_pump_j, proto) - n) / g,
                        r.m_air_j_cyc / ref.m_air_j_cyc - 1.0);
        }
    }
    std::printf("REV-B: p_im_kPa valves | IMEPg_ref | dg_1deg dm_1deg | dg_hfull dm_hfull\n");
    for (const real p_im : {100.0e3, 30.0e3}) {
        for (const int nv : {2, 3, 4}) {
            drivetrain::CycleModelDesc dv = d;
            dv.intake_valves = nv;
            dv.exhaust_valves = nv;
            const CycleStepper pv(dv, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
            const OperatingPoint op = make_op(dv, 1000.0, p_im);
            const PointResult ref = run_operating_point(pv, op, StepperSettings{kStepFull / 4.0, kCyclesFull});
            const PointResult c = run_operating_point(pv, op, StepperSettings{1.0 / 6000.0, kCyclesFull});
            const PointResult f = run_operating_point(pv, op, StepperSettings{kStepFull, kCyclesFull});
            std::printf("REV-B: %5.0f %d | %.5e | %+.3e %+.3e | %+.3e %+.3e\n", p_im / 1000.0, nv, imep(ref.w_gross_j, pv),
                        c.w_gross_j / ref.w_gross_j - 1.0, c.m_air_j_cyc / ref.m_air_j_cyc - 1.0, f.w_gross_j / ref.w_gross_j - 1.0,
                        f.m_air_j_cyc / ref.m_air_j_cyc - 1.0);
        }
    }
}
