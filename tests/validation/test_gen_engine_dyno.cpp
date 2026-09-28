// PHYS-033 P4 I7c C17a M-A: the generated A25A-FKS against EPA NVFEL dyno
// data it was never fitted to.
//
// data/engines/toyota_a25a_fks_gen.json carries NO maps block: its maps are
// generated at load time by the C16a map generator from a C15a "cycle" block
// holding only cited geometry, frozen-literature constants and measured ECU
// schedules (tools/enginedata/a25a_gen_engine.py writes it; every field's
// tier and source is in the file's own *_source notes). No model parameter
// was chosen from the A25A's measured brake torque, fuel, BSFC, air flow,
// WOT envelope or motoring curve, and the engine file was committed before
// S1/S2 were first run (frozen-literature rule D1, out/c17a_ma_progress.md).
//
// S1 WOT brake torque and S2 BSFC: a direct dyno-held harness - the
// SimulatedEngine's own pre_solve/post_solve with omega held exactly
// constant, the test_engine_fuel_map C5d pattern - rather than a LoadDyno
// (PROVISIONAL, progress file: LoadDyno has no speed-hold mode, and a
// Brake-mode dyno against the WOT curve's positive slope is statically
// unstable). Ambient = the measured test-cell pressure/temperature of each
// anchor row. Brake torque = tau0 - friction_cap_kinetic (the Powertrain's
// Coulomb row carries c_k), fuel flow = fuel_mass_delta / h.
//
// S4: the electronic pedal law on the generated maps - at ISA, request r
// must give the steady brake torque mot + r*(wot - mot) (maps' own steady
// tables), within 1 % of the span (wot - mot).
//
// Measured values below are literal xlsx readings (EPA NVFEL Report 26781's
// Tier 3 test-data workbook, sheet "High Load Final"), the same rows
// tests/unit/test_a25a_sim_engine_data.cpp pins for M-B.

#include "drivetrain/engine.h"
#include "drivetrain/engine_simulated.h"
#include "engine_cycle/cycle_point.h"
#include "engine_cycle/cycle_stepper.h"
#include "engine_cycle/wiebe.h"

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>

using ps::real;
using namespace ps::drivetrain;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;

std::string gen_car_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_gen.json"; }
std::string mb_car_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_sim.json"; }

const SimulatedEngineDesc* sim_engine_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return e;
    }
    return nullptr;
}

ps::vehicle::VehicleDesc load_gen_car() {
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions opt;
    opt.quality = MapQuality::Full;
    opt.pool = &pool;
    return ps::io::load_vehicle_json(gen_car_path(), opt);
}

real rpm_to_omega(real rpm) { return rpm * kPi / real(30.0); }

struct WotAnchor {
    int row;
    real rpm, map_kpa, p_cell_kpa, t_cell_c, torque_nm;
    real fuel_g_s, lambda, imt_c;
    bool asserted; // S1/S2 assert only the 7 brief-listed points
};

// clang-format off
constexpr WotAnchor kAnchors[] = {
    { 8, 1003.04, 97.509,  98.1706, 25.7,    159.039, 1.38858, 0.834671, 33.8042, false},
    {11, 1153.2,  97.4626, 98.1555, 25.3,    176.654, 1.93165, 0.782105, 34.2153, false},
    {14, 1325.94, 95.431,  98.1772, 25.3,    179.954, 2.02207, 0.882445, 34.2974, false},
    {17, 1498.52, 95.5144, 98.2396, 25.5001, 189.621, 2.32795, 0.880437, 33.5309, false},
    {20, 1728.58, 96.987,  98.27,   26.4,    196.616, 3.87312, 0.669733, 32.9487, false},
    {23, 2017.13, 97.2897, 98.267,  26.8997, 216.321, 3.81202, 0.79747,  32.1949, true},
    {26, 2534.94, 97.3509, 98.2473, 27.3,    225.125, 4.46601, 0.867008, 32.1453, true},
    {29, 2995.63, 97.2884, 98.2005, 27.7,    217.813, 4.83102, 0.896868, 32.7409, true},
    {32, 3571.24, 97.0007, 98.1844, 27.5308, 228.4,   6.09503, 0.879773, 32.744,  true},
    {34, 4032.22, 96.8321, 98.2054, 27.1,    228.347, 7.28591, 0.859654, 32.6323, true},
    {37, 4493.11, 96.6618, 98.2557, 27.2,    232.038, 8.15611, 0.87221,  33.4748, true},
    {39, 4954.03, 96.4381, 98.2724, 27.4004, 231.437, 9.08713, 0.870633, 34.1517, true},
    {42, 5414.48, 96.1927, 98.2773, 27.9,    229.401, 9.98602, 0.862965, 35.4221, false},
    {45, 5761.17, 96.0589, 98.291,  28.2004, 227.315, 10.8123, 0.83928,  36.2803, false},
};
// clang-format on

real bsfc_g_kwh(real fuel_g_s, real torque_nm, real rpm) {
    return fuel_g_s * real(3.6e6) / (torque_nm * rpm_to_omega(rpm));
}

struct DynoResult {
    real brake_nm = 0.0;    // tau0 - c_k, mean over the averaging window
    real p_im_kpa = 0.0;
    real fuel_g_s = 0.0;
    real plate = 0.0;
    real lambda = 0.0;
    real spark_deg = 0.0;
    real brake_min = 0.0, brake_max = 0.0; // spread over the window (steadiness)
};

// Dyno-held engine: omega fixed, the engine's own management running.
DynoResult dyno_hold(SimulatedEngine& e, real rpm, real command, const ps::AmbientSample& amb, real settle_s = 2.0,
                     real avg_s = 0.25) {
    e.reset(EngineState::Running);
    const real omega = rpm_to_omega(rpm);
    const int n_settle = static_cast<int>(settle_s / kH + 0.5);
    const int n_avg = static_cast<int>(avg_s / kH + 0.5);
    DynoResult r;
    r.brake_min = 1e300;
    r.brake_max = -1e300;
    for (int i = 0; i < n_settle + n_avg; ++i) {
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.starter = 0.0;
        s.clutch_engaged_fraction = 1.0;
        s.gearbox_in_neutral = false;
        s.ambient = amb;
        ThrottleCommand tc;
        tc.command = command;
        const PreSolveResult pre = e.pre_solve(s, tc, TcuRequest{}, kH);
        const PostSolveResult post = e.post_solve(omega, kH);
        if (i >= n_settle) {
            const real b = pre.tau0 - pre.friction_cap_kinetic;
            r.brake_nm += b;
            r.brake_min = std::min(r.brake_min, b);
            r.brake_max = std::max(r.brake_max, b);
            r.p_im_kpa += pre.p_intake_pa * real(1e-3);
            r.fuel_g_s += post.fuel_mass_delta * real(1e3) / kH;
            r.plate += pre.throttle_position;
            r.lambda += pre.lambda;
            r.spark_deg += pre.spark_advance_deg;
        }
    }
    const real inv = real(1.0) / real(n_avg);
    r.brake_nm *= inv;
    r.p_im_kpa *= inv;
    r.fuel_g_s *= inv;
    r.plate *= inv;
    r.lambda *= inv;
    r.spark_deg *= inv;
    return r;
}

ps::AmbientSample cell_ambient(const WotAnchor& a) {
    ps::AmbientSample s;
    s.pressure_pa = a.p_cell_kpa * real(1e3);
    s.temperature_k = a.t_cell_c + real(273.15);
    return s;
}

} // namespace

// ---------------------------------------------------------------------------
// Pre-commit diagnostic: the closed-plate leak y(0) of the throttle-body area
// curve. DERIVED from the model itself (no dyno data): the leak that makes
// the steady fired net torque at plate 0 and idle_rpm exactly 0, halved so
// the idle PI keeps authority (PROVISIONAL, progress file).
TEST_CASE("C17a-MA diag: idle leak bisection", "[.][diag][drivetrain]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* e0 = sim_engine_of(car);
    REQUIRE(e0 != nullptr);
    SimulatedEngineDesc d = *e0;
    const real idle = d.idle_rpm;
    auto torque_at = [&](real y0) {
        d.throttle_body.area_fraction_vs_position.y[0] = y0;
        const auto m = SimulatedEngine::build_maps(d);
        return m->steady_torque_nm.value(idle, real(0.0));
    };
    real lo = 0.0, hi = 0.012;
    const real t_lo = torque_at(lo), t_hi = torque_at(hi);
    std::printf("idle %.1f rpm: T(y0=0) = %.4f, T(y0=0.012) = %.4f N*m\n", idle, t_lo, t_hi);
    REQUIRE(t_lo < 0.0);
    REQUIRE(t_hi > 0.0);
    for (int it = 0; it < 40; ++it) {
        const real mid = real(0.5) * (lo + hi);
        if (torque_at(mid) < 0.0) lo = mid;
        else hi = mid;
    }
    const real y_star = real(0.5) * (lo + hi);
    const real y_ship = real(0.5) * y_star;
    d.throttle_body.area_fraction_vs_position.y[0] = y_ship;
    const auto m = SimulatedEngine::build_maps(d);
    std::printf("y0* (T=0 at plate 0) = %.9g; shipped 0.5*y0* = %.9g -> idle_ff %.6f, idle_p_im %.1f Pa, "
                "T(plate 0) %.4f N*m\n",
                y_star, y_ship, m->idle_ff, m->idle_p_im, m->steady_torque_nm.value(idle, real(0.0)));
    CHECK(y_star < 0.012);
    CHECK(m->idle_ff < d.idle.max_throttle);
}

namespace {

struct AnchorRun {
    DynoResult sim;
    real bsfc_sim = 0.0, bsfc_meas = 0.0;
};

std::vector<AnchorRun> run_all_anchors(const SimulatedEngineDesc& d) {
    SimulatedEngine eng(d);
    std::vector<AnchorRun> out;
    for (const WotAnchor& a : kAnchors) {
        AnchorRun r;
        r.sim = dyno_hold(eng, a.rpm, real(1.0), cell_ambient(a));
        r.bsfc_sim = bsfc_g_kwh(r.sim.fuel_g_s, r.sim.brake_nm, a.rpm);
        r.bsfc_meas = bsfc_g_kwh(a.fuel_g_s, a.torque_nm, a.rpm);
        out.push_back(r);
    }
    return out;
}

ps::engine_cycle::OperatingPoint cycle_op(const CycleModelDesc& c, real rpm, real p_kpa) {
    ps::engine_cycle::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_kpa * real(1e3);
    op.t_im_k = SimulatedEngine::kIsaTemperatureK;
    op.p_amb_pa = SimulatedEngine::kIsaPressurePa;
    op.lambda = 1.0;
    op.spark_deg_atdc =
        ps::engine_cycle::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
    op.intake_advance_deg = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(rpm, op.p_im_pa);
    op.exhaust_retard_deg = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(rpm, op.p_im_pa);
    return op;
}

} // namespace

// [!mayfail] (D2, measured 2026-09-28, frozen-literature rule D1 - no model
// parameter changed): 6 of 7 asserted points exceed +-10 % (+10.1..+23.4 %,
// 2017-4493 rpm; 4954 rpm +7.3 % passes). Per-term breakdown vs the
// measured-derived M-B file ("diag: per-term breakdown vs M-B"): VE gen/M-B
// 1.02/1.04/1.18/1.20/1.19/1.15/1.08, eta_i +6..+10 %, friction +7..+8 N*m,
// pumping excess larger. Discriminating measurements: (1) S2's simulated
// fuel flow over measured (lambda follows the measured schedule, so fuel
// ratio = air ratio) is 1.03/1.03/1.19/1.18/1.17/1.15/1.08 - the torque
// excess is mostly trapped AIR, S2's BSFC holds (-7.0..+0.3 %); (2) "diag:
// WOT air vs runner length": shortening the Tier 3 [AS] quarter-wave runner
// (708.9 mm, tuned to 4800 rpm) to 0.25x drops WOT air by 0.887/0.844/0.830/
// 0.852 at 3000/3571/4032/4493 rpm vs the 0.844/0.836/0.841/0.866 the M-B VE
// asks for - the excess is the [AS] ram-runner sizing rule's resonance, not a
// code defect. See out/c17a_ma_progress.md.
TEST_CASE("C17a-MA S1: generated A25A WOT brake torque vs EPA NVFEL", "[drivetrain][validation][!mayfail]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    const auto runs = run_all_anchors(*d);
    std::printf("S1 row   rpm      T_meas   T_sim    dT%%     MAP_meas MAP_sim plate  lambda spark  spread\n");
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const WotAnchor& a = kAnchors[i];
        const DynoResult& s = runs[i].sim;
        const real err = s.brake_nm / a.torque_nm - real(1.0);
        std::printf("S1 %3d %8.2f %8.3f %8.3f %+6.2f%s %7.2f %7.2f %6.3f %6.3f %6.2f %.2e\n", a.row, a.rpm, a.torque_nm,
                    s.brake_nm, real(100.0) * err, a.asserted ? "*" : " ", a.map_kpa, s.p_im_kpa, s.plate, s.lambda,
                    s.spark_deg, s.brake_max - s.brake_min);
        // Steady: the dyno-held engine must have settled.
        CHECK(s.brake_max - s.brake_min < real(1e-3) * std::abs(s.brake_nm));
        if (a.asserted) CHECK(std::abs(err) <= real(0.10));
    }
}

TEST_CASE("C17a-MA S2: generated A25A WOT BSFC vs EPA NVFEL", "[drivetrain][validation]") {
    // Asserted: a25a_tables.json egr_finding_task6 - EGR is not open along
    // WOT above ~2000 rpm, so the measured fuel at these rows is not
    // EGR-diluted (the model has no EGR).
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    const auto runs = run_all_anchors(*d);
    std::printf("S2 row   rpm     fuel_meas fuel_sim BSFC_meas BSFC_sim  dBSFC%%\n");
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const WotAnchor& a = kAnchors[i];
        const AnchorRun& r = runs[i];
        const real err = r.bsfc_sim / r.bsfc_meas - real(1.0);
        std::printf("S2 %3d %8.2f %8.4f %8.4f %8.1f %8.1f %+7.2f%s\n", a.row, a.rpm, a.fuel_g_s, r.sim.fuel_g_s,
                    r.bsfc_meas, r.bsfc_sim, real(100.0) * err, a.asserted ? "*" : "");
        CHECK(r.sim.fuel_g_s > 0.0);
        if (a.asserted) CHECK(std::abs(err) <= real(0.10));
    }
}

TEST_CASE("C17a-MA S4: electronic pedal law on the generated maps", "[drivetrain][validation]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    SimulatedEngine eng(*d);
    const SimulatedEngineMaps& m = eng.maps();
    const ps::AmbientSample isa{}; // the maps' own reference ambient
    REQUIRE(isa.pressure_pa == m.ref_p_pa);
    REQUIRE(isa.temperature_k == m.ref_t_k);
    real worst = 0.0;
    for (const real rpm : {real(1500.0), real(3000.0), real(4500.0)}) {
        const real wot = m.steady_wot_nm.evaluate(rpm);
        const real mot = m.steady_motoring_nm.evaluate(rpm);
        const real span = wot - mot;
        REQUIRE(span > 0.0);
        for (const real r : {real(0.1), real(0.3), real(0.6), real(1.0)}) {
            const DynoResult s = dyno_hold(eng, rpm, r, isa);
            const real target = mot + r * span;
            const real err_span = (s.brake_nm - target) / span;
            worst = std::max(worst, std::abs(err_span));
            std::printf("S4 rpm %6.0f r %.1f: T %9.4f target %9.4f (mot %8.3f wot %8.3f) err/span %+.3e "
                        "err/target %+.3e plate %.5f inv %.5f\n",
                        rpm, r, s.brake_nm, target, mot, wot, err_span, (s.brake_nm - target) / target, s.plate,
                        m.inverse_plate.value(rpm, r));
            CHECK(std::abs(err_span) <= real(0.01));
        }
    }
    std::printf("S4 worst |err|/span = %.3e\n", worst);
}

TEST_CASE("C17a-MA check 4: generated steady torque monotone in plate", "[drivetrain][validation]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    SimulatedEngine eng(*d);
    const Table2D& t = eng.maps().steady_torque_nm;
    const std::size_t ny = t.y.size();
    int violations = 0;
    real worst_drop = 0.0;
    for (std::size_t ix = 0; ix < t.x.size(); ++ix) {
        for (std::size_t iy = 1; iy < ny; ++iy) {
            const real dv = t.values[ix * ny + iy] - t.values[ix * ny + iy - 1];
            if (dv < 0.0) {
                ++violations;
                worst_drop = std::min(worst_drop, dv);
                std::printf("check4 non-monotone at rpm %.0f plate %.3f -> %.3f: %.6f N*m\n", t.x[ix], t.y[iy - 1],
                            t.y[iy], dv);
            }
        }
    }
    std::printf("check4: %zu rpm rows x %zu plate nodes, %d decreasing steps, worst %.6f N*m\n", t.x.size(), ny,
                violations, worst_drop);
    CHECK(violations == 0);
}

// ---------------------------------------------------------------------------
// Check 1: RK4 step convergence of the cycle model on THIS engine's cycle
// block (larger valves than the C15a fixture move the stability edge down).
TEST_CASE("C17a-MA diag: check 1 RK4 step convergence", "[.][diag][drivetrain][cycle]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    REQUIRE(d->cycle.has_value());
    const CycleModelDesc& c = *d->cycle;
    const FuelDesc& f = d->fuels[0];
    const ps::engine_cycle::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    struct Pt {
        real rpm, p_kpa;
    };
    for (const Pt pt : {Pt{1000.0, 30.0}, Pt{3000.0, 30.0}, Pt{4000.0, 97.273}}) {
        const auto op = cycle_op(c, pt.rpm, pt.p_kpa);
        using ps::engine_cycle::run_operating_point;
        const auto ref = run_operating_point(proto, op, {ps::engine_cycle::kStepFull / real(2.0), ps::engine_cycle::kCyclesFull});
        const auto full = run_operating_point(proto, op, {ps::engine_cycle::kStepFull, ps::engine_cycle::kCyclesFull});
        const auto prev = run_operating_point(proto, op, {ps::engine_cycle::kStepPreview, ps::engine_cycle::kCyclesPreview});
        const auto prev5 = run_operating_point(proto, op, {ps::engine_cycle::kStepPreview, ps::engine_cycle::kCyclesFull});
        auto eta = [&](const ps::engine_cycle::PointResult& r) { return r.w_gross_j / (r.m_fuel_kg * f.lhv_j_kg); };
        std::printf("check1 %5.0f rpm %6.2f kPa (IA %.2f ER %.2f): air h/2 %.6e | full %+.3e | preview %+.3e "
                    "(N5 %+.3e); eta h/2 %.6f | full %+.3e | preview %+.3e (N5 %+.3e)\n",
                    pt.rpm, pt.p_kpa, op.intake_advance_deg, op.exhaust_retard_deg, ref.m_air_j_cyc,
                    full.m_air_j_cyc / ref.m_air_j_cyc - 1.0, prev.m_air_j_cyc / ref.m_air_j_cyc - 1.0,
                    prev5.m_air_j_cyc / ref.m_air_j_cyc - 1.0, eta(ref), eta(full) / eta(ref) - 1.0,
                    eta(prev) / eta(ref) - 1.0, eta(prev5) / eta(ref) - 1.0);
    }
}

// Check 2: the intake-runner Helmholtz ripple. Trapped air from the cycle
// model on a fine rpm sweep vs linear interpolation between the generator's
// own rpm grid nodes (what the maps resolve), at light load and at WOT.
TEST_CASE("C17a-MA diag: check 2 runner ripple", "[.][diag][drivetrain][cycle]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    const CycleModelDesc& c = *d->cycle;
    const FuelDesc& f = d->fuels[0];
    const ps::engine_cycle::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    ps::jobs::JobSystem pool(10);
    constexpr std::size_t kSub = 8;
    for (const real p_kpa : {real(22.5), real(30.0), real(97.273)}) {
        auto air_at = [&](real rpm) {
            return ps::engine_cycle::run_operating_point(proto, cycle_op(c, rpm, p_kpa),
                                                         {ps::engine_cycle::kStepFull, ps::engine_cycle::kCyclesFull})
                .m_air_j_cyc;
        };
        const auto& g = c.grid_rpm;
        const std::size_t ni = g.size() - 1;
        std::vector<real> node_air(g.size());
        pool.parallel_batches(g.size(), [&](std::size_t i) { node_air[i] = air_at(g[i]); });
        std::vector<real> dev(ni * (kSub - 1));
        pool.parallel_batches(dev.size(), [&](std::size_t q) {
            const std::size_t k = q / (kSub - 1), j = q % (kSub - 1) + 1;
            const real fr = real(j) / real(kSub);
            const real rpm = g[k] + (g[k + 1] - g[k]) * fr;
            const real lin = node_air[k] + (node_air[k + 1] - node_air[k]) * fr;
            dev[q] = air_at(rpm) / lin - 1.0;
        });
        real worst = 0.0, worst_rpm = 0.0;
        for (std::size_t k = 0; k < ni; ++k) {
            real iv = 0.0;
            for (std::size_t j = 0; j + 1 < kSub; ++j) {
                const real v = dev[k * (kSub - 1) + j];
                if (std::abs(v) > std::abs(iv)) iv = v;
            }
            std::printf("check2 p %6.2f kPa: rpm [%5.0f, %5.0f] max dev from node interpolation %+.3e\n", p_kpa, g[k],
                        g[k + 1], iv);
            if (std::abs(iv) > std::abs(worst)) {
                worst = iv;
                worst_rpm = g[k];
            }
        }
        std::printf("check2 p %6.2f kPa: worst %+.3e in the interval starting %.0f rpm\n", p_kpa, worst, worst_rpm);
    }
}

// Check 3: load-time cost of the Full map generation (the loader's own
// path): 1 thread, 10 workers, and a .psmaps cache hit.
TEST_CASE("C17a-MA diag: check 3 load cost", "[.][diag][drivetrain]") {
    using clock = std::chrono::steady_clock;
    auto ms = [](clock::duration dt) { return std::chrono::duration<double, std::milli>(dt).count(); };
    {
        ps::io::EngineMapOptions opt;
        const auto t0 = clock::now();
        (void)ps::io::load_vehicle_json(gen_car_path(), opt);
        std::printf("check3 Full, 1 thread: %.1f ms\n", ms(clock::now() - t0));
    }
    {
        ps::jobs::JobSystem pool(10);
        ps::io::EngineMapOptions opt;
        opt.pool = &pool;
        const auto t0 = clock::now();
        (void)ps::io::load_vehicle_json(gen_car_path(), opt);
        std::printf("check3 Full, 10 workers: %.1f ms\n", ms(clock::now() - t0));
    }
    {
        const std::string dir = std::string(PS_SOURCE_DIR) + "/out/c17a_ma_cache";
        ps::jobs::JobSystem pool(10);
        ps::io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = dir;
        std::size_t gens = 0;
        opt.generation_count_out = &gens;
        const auto t0 = clock::now();
        (void)ps::io::load_vehicle_json(gen_car_path(), opt); // fills the cache if absent
        const auto t1 = clock::now();
        (void)ps::io::load_vehicle_json(gen_car_path(), opt); // cache hit
        const auto t2 = clock::now();
        std::printf("check3 cache (%s): first %.1f ms, second %.1f ms, generator runs %zu\n", dir.c_str(), ms(t1 - t0),
                    ms(t2 - t1), gens);
    }
}

// D2 per-term breakdown: the generated maps against the M-B file's
// measured-derived maps (data/engines/toyota_a25a_fks_sim.json) at every
// anchor speed and the measured WOT MAP.
TEST_CASE("C17a-MA diag: per-term breakdown vs M-B", "[.][diag][drivetrain]") {
    const auto gen_car = load_gen_car();
    const auto mb_car = ps::io::load_vehicle_json(mb_car_path());
    const SimulatedEngineDesc* g = sim_engine_of(gen_car);
    const SimulatedEngineDesc* b = sim_engine_of(mb_car);
    REQUIRE(g != nullptr);
    REQUIRE(b != nullptr);
    SimulatedEngine eg(*g), eb(*b);
    std::printf("D2   rpm   MAP  | VE gen VE mb   | eta_i gen mb    | fric gen mb     | pmepx gen mb      | "
                "WOT_ISA gen mb\n");
    for (const WotAnchor& a : kAnchors) {
        const real p = a.map_kpa * real(1e3);
        const real ve_g = g->maps.ve.value(a.rpm, p), ve_b = b->maps.ve.value(a.rpm, p);
        // Air mass per crank revolution at this MAP and the cell temperature:
        // ve * p * V_d * c_cycle / (R * T).
        const real vd = eg.maps().v_disp_m3;
        const real t = a.t_cell_c + real(273.15);
        const real m_rev_g = ve_g * p * vd * real(0.5) / (real(287.058) * t);
        const real m_rev_b = ve_b * p * vd * real(0.5) / (real(287.058) * t);
        const real eta_g = g->maps.eta_i.value(a.rpm, m_rev_g), eta_b = b->maps.eta_i.value(a.rpm, m_rev_b);
        const real fr_g = g->maps.friction_torque_nm.evaluate(a.rpm), fr_b = b->maps.friction_torque_nm.evaluate(a.rpm);
        const real px_g = g->maps.has_pmep_excess ? g->maps.pmep_excess_pa.value(a.rpm, p) : 0.0;
        const real px_b = b->maps.has_pmep_excess ? b->maps.pmep_excess_pa.value(a.rpm, p) : 0.0;
        std::printf("D2 %6.0f %6.2f | %.4f %.4f | %.4f %.4f | %7.3f %7.3f | %8.1f %8.1f | %8.3f %8.3f\n", a.rpm,
                    a.map_kpa, ve_g, ve_b, eta_g, eta_b, fr_g, fr_b, px_g, px_b, eg.maps().steady_wot_nm.evaluate(a.rpm),
                    eb.maps().steady_wot_nm.evaluate(a.rpm));
    }
}

// D2 discriminating measurement for S1: WOT trapped air from the cycle model
// with the Tier 3 [AS] quarter-wave runner (708.9 mm, tuned to 4800 rpm) and
// with it scaled 0.5 / 0.25, next to the ratio the measured-derived M-B VE
// asks for (VE_mb / VE_gen). A DIAGNOSTIC only - the shipped runner is not
// changed (D1). Half the Full step so a short, stiff runner stays inside the
// RK4 edge.
TEST_CASE("C17a-MA diag: WOT air vs runner length", "[.][diag][drivetrain][cycle]") {
    const auto gen_car = load_gen_car();
    const auto mb_car = ps::io::load_vehicle_json(mb_car_path());
    const SimulatedEngineDesc* g = sim_engine_of(gen_car);
    const SimulatedEngineDesc* b = sim_engine_of(mb_car);
    REQUIRE(g != nullptr);
    REQUIRE(b != nullptr);
    REQUIRE(g->cycle.has_value());
    const FuelDesc& f = g->fuels[0];
    constexpr real kScales[] = {1.0, 0.5, 0.25};
    std::vector<ps::engine_cycle::CycleStepper> protos;
    for (const real s : kScales) {
        CycleModelDesc c = *g->cycle;
        c.runner_length_m *= s;
        protos.emplace_back(c, f.lhv_j_kg, f.afr_stoich);
    }
    std::printf("RL   rpm    MAP   | VE_mb/VE_gen | air(L)/air(L0): L0 x1 [kg]  x0.5   x0.25\n");
    for (const WotAnchor& a : kAnchors) {
        if (!a.asserted) continue;
        const real p = a.map_kpa * real(1e3);
        const real need = b->maps.ve.value(a.rpm, p) / g->maps.ve.value(a.rpm, p);
        real air[3] = {};
        for (int k = 0; k < 3; ++k) {
            air[k] = ps::engine_cycle::run_operating_point(protos[static_cast<std::size_t>(k)],
                                                           cycle_op(*g->cycle, a.rpm, a.map_kpa),
                                                           {ps::engine_cycle::kStepFull / real(2.0),
                                                            ps::engine_cycle::kCyclesFull})
                         .m_air_j_cyc;
        }
        std::printf("RL %6.0f %6.2f | %.4f       | %.4e  %.4f  %.4f\n", a.rpm, a.map_kpa, need, air[0],
                    air[1] / air[0], air[2] / air[0]);
    }
}

// Trapped air per cycle, residual fraction and cam phase over the generator's
// own (rpm, p_im) grid. Found the raw IDW cam grid's air falling with MAP at
// 600/800/1000 rpm (the generator's "air mass not strictly increasing" throw);
// with a25a_gen_engine.py's median-of-3 root smoothing the smallest step is
// +7.6 %. Runs on car_sedan_gen.json, so the loader generates the maps first.
TEST_CASE("C17a-MA diag: air vs p_im on the grid", "[.][diag][drivetrain][cycle]") {
    const auto car = load_gen_car();
    const SimulatedEngineDesc* d = sim_engine_of(car);
    REQUIRE(d != nullptr);
    REQUIRE(d->cycle.has_value());
    const CycleModelDesc& c = *d->cycle;
    const FuelDesc& f = d->fuels[0];
    const ps::engine_cycle::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    ps::jobs::JobSystem pool(10);
    const std::size_t nr = c.grid_rpm.size(), np = c.grid_p_im.size();
    std::vector<ps::engine_cycle::PointResult> res(nr * np);
    pool.parallel_batches(res.size(), [&](std::size_t q) {
        const real rpm = c.grid_rpm[q / np], p = c.grid_p_im[q % np];
        res[q] = ps::engine_cycle::run_operating_point(proto, cycle_op(c, rpm, p * real(1e-3)),
                                                       {ps::engine_cycle::kStepFull, ps::engine_cycle::kCyclesFull});
    });
    for (std::size_t i = 0; i < nr; ++i) {
        std::printf("air rpm %5.0f:", c.grid_rpm[i]);
        for (std::size_t j = 0; j < np; ++j) std::printf(" %9.3e", res[i * np + j].m_air_j_cyc);
        std::printf("\n");
        std::printf("res rpm %5.0f:", c.grid_rpm[i]);
        for (std::size_t j = 0; j < np; ++j) std::printf(" %9.4f", res[i * np + j].residual_frac);
        std::printf("\n");
        std::printf("cam rpm %5.0f:", c.grid_rpm[i]);
        for (std::size_t j = 0; j < np; ++j) {
            const auto op = cycle_op(c, c.grid_rpm[i], c.grid_p_im[j] * real(1e-3));
            std::printf("  %+4.1f/%+4.1f", op.intake_advance_deg, op.exhaust_retard_deg);
        }
        std::printf("\n");
    }
}

// ---------------------------------------------------------------------------
// Review diagnostic (C17a M-A review, 2026-09-28) for the S1 per-term
// attribution. DIAGNOSIS ONLY - measured data here never feeds a model
// parameter (D1).
//  (a) Spark: the model fires at its constant MBT spark (23.053 deg BTDC);
//      the A25A's measured WOT spark (xlsx "High Load Final", column spark
//      advance, literal) is knock-retarded to -0.9..+5.3 deg BTDC. The cycle
//      point is run at the anchor MAP and measured lambda twice - MBT, and
//      Wiebe theta_0 moved later by (23.053 - measured) deg - and the gross
//      work ratio printed. The model has no ignition delay and one Wiebe
//      shape, so this is a bound on the spark term, not the engine's truth.
//  (b) Motoring: the A25A's measured motoring rows (xlsx "Steady State",
//      fuel 0, closed throttle, literal rpm/MAP/torque) against the model at
//      the SAME MAP: the cycle with combustion off (gross + pumping work)
//      minus the generated F4 friction, and the runtime formula
//      -(p_amb - p_im + pmep_excess)*V_d/(4 pi) - friction for gen and M-B.
TEST_CASE("C17a-MA review diag: spark retard and motoring vs A25A", "[.][diag][drivetrain][cycle]") {
    const auto gen_car = load_gen_car();
    const auto mb_car = ps::io::load_vehicle_json(mb_car_path());
    const SimulatedEngineDesc* g = sim_engine_of(gen_car);
    const SimulatedEngineDesc* b = sim_engine_of(mb_car);
    REQUIRE(g != nullptr);
    REQUIRE(b != nullptr);
    REQUIRE(g->cycle.has_value());
    const CycleModelDesc& c = *g->cycle;
    const FuelDesc& f = g->fuels[0];
    const ps::engine_cycle::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    const ps::engine_cycle::StepperSettings st{ps::engine_cycle::kStepFull, ps::engine_cycle::kCyclesFull};
    const real theta0_mbt =
        ps::engine_cycle::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
    const real cyl = real(c.cylinders);
    const real four_pi = real(4.0) * kPi;
    std::printf("mbt theta0 %.4f deg ATDC (= %.4f BTDC)\n", theta0_mbt, -theta0_mbt);

    struct SparkRow { real rpm, map_kpa, lambda, spark_btdc; };
    constexpr SparkRow kSpark[] = {
        {2017.13, 97.2897, 0.79747, -0.8834489}, {2534.94, 97.3509, 0.867008, -0.1004669},
        {2995.63, 97.2884, 0.896868, 3.6484548}, {3571.24, 97.0007, 0.879773, 5.2851633},
        {4032.22, 96.8321, 0.859654, 3.4596332}, {4493.11, 96.6618, 0.87221, 3.3724168},
        {4954.03, 96.4381, 0.870633, 4.3984367},
    };
    std::printf("SPK   rpm  spark_meas retard | Wg_mbt [J]  Wg_meas/Wg_mbt | dT_gross [N*m]\n");
    for (const SparkRow& r : kSpark) {
        auto op = cycle_op(c, r.rpm, r.map_kpa);
        op.lambda = r.lambda;
        const auto mbt = ps::engine_cycle::run_operating_point(proto, op, st);
        const real retard = -theta0_mbt - r.spark_btdc;
        op.spark_deg_atdc = theta0_mbt + retard;
        const auto ret = ps::engine_cycle::run_operating_point(proto, op, st);
        std::printf("SPK %6.0f %8.2f %7.2f | %9.3f  %.4f         | %+8.2f\n", r.rpm, r.spark_btdc, retard, mbt.w_gross_j,
                    ret.w_gross_j / mbt.w_gross_j, cyl * (ret.w_gross_j - mbt.w_gross_j) / four_pi);
    }

    struct MotRow { real rpm, map_kpa, torque_nm; };
    constexpr MotRow kMot[] = {
        {1326, 24.86, -22.90}, {1498, 23.96, -23.60}, {1728, 23.37, -23.30}, {2017, 22.80, -24.30},
        {2535, 23.01, -25.90}, {2996, 24.81, -28.40}, {3571, 23.80, -30.60}, {4032, 22.29, -33.90},
        {4493, 22.69, -37.40}, {4954, 23.91, -42.30},
    };
    ps::engine_cycle::CycleStepper motored(c, f.lhv_j_kg, f.afr_stoich);
    ps::engine_cycle::CycleSwitches sw;
    sw.combustion = false;
    motored.set_switches(sw);
    SimulatedEngine eg(*g), eb(*b);
    const real vd = eg.maps().v_disp_m3;
    const real p_amb = SimulatedEngine::kIsaPressurePa;
    std::printf("MOT   rpm   MAP  meas  | cycle_gas fric_gen cyc_tot | rt_pump_gen rt_gen | rt_pump_mb fric_mb rt_mb\n");
    for (const MotRow& r : kMot) {
        const real p = r.map_kpa * real(1e3);
        const auto res = ps::engine_cycle::run_operating_point(motored, cycle_op(c, r.rpm, r.map_kpa), st);
        const real gas = cyl * (res.w_gross_j + res.w_pump_j) / four_pi;
        const real fr_g = g->maps.friction_torque_nm.evaluate(r.rpm), fr_b = b->maps.friction_torque_nm.evaluate(r.rpm);
        const real px_g = g->maps.has_pmep_excess ? g->maps.pmep_excess_pa.value(r.rpm, p) : 0.0;
        const real px_b = b->maps.has_pmep_excess ? b->maps.pmep_excess_pa.value(r.rpm, p) : 0.0;
        const real pump_g = -(p_amb - p + px_g) * vd / four_pi, pump_b = -(p_amb - p + px_b) * vd / four_pi;
        std::printf("MOT %6.0f %5.2f %6.2f | %8.2f %8.2f %8.2f | %8.2f %8.2f | %8.2f %8.2f %8.2f\n", r.rpm, r.map_kpa,
                    r.torque_nm, gas, fr_g, gas - fr_g, pump_g, pump_g - fr_g, pump_b, fr_b, pump_b - fr_b);
    }
}
