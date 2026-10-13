// tests/unit/test_boost_shift_hold.cpp -- Boost B0 (2026-10-13): the boost controller through a gearshift.
//
// Before B0 a plate below 10 % meant "no boost requested": the wastegate was commanded open and the PID integral reset
// (turbo_pair.h, 40923cf / vault PHYS-058). An upshift's ignition cut (0.25-0.47 s on car_hyper_n2o) and post-shift
// throttle-ceiling ramp (0.1 s) close the plate, so every upshift threw the controller away: the gate opened to ~0.98,
// the rotor fell (stock car_hyper_n2o 130-135k -> 73k rpm after the high-gear shifts) and the boost took 2.1-2.3 s to
// come back [M]. B0: while the ManualTcu shifts (TcuRequest::hold_boost_control = its ignition cut or throttle ceiling
// < 1) the controller HOLDS - gate command = its position, integral frozen; a driver lift (no TCU limit) keeps the old
// rule. The PID error is normalised by min(target_boost, 1.8 bar) instead of the target (gains swept at 1.8 bar,
// PHYS-058): bit-identical for every shipped configuration (all <= 1.8 bar); a 15.48 bar garage target no longer gets
// an 8.6x weaker loop per Pa.
//
// Bands committed in out/boost_b0_progress.md before the first run (never widened):
//   B1  white-box TurboPair: hold keeps wastegate and integral bit-exact; without hold the old rule applies at once
//   B2  engine level (mvem electronic engine + hyper-like pair, crank held at 5000 rpm, WOT): a scripted upshift
//       (cut + 0.1 s ceiling ramp; see the case for the cut length); recovery to 90 % of the pre-shift charge with the
//       hold <= 0.7 x without, rotor speed at ramp end with the hold >= without, gate unmoved while held
//   B3  white-box: a 3.6 bar target at half plate regulates bit-identically to a 1.8 bar target at full plate
//   V1  [long] stock car_hyper_n2o, WOT from rest with auto-shift, 40 s: recovery (new-gear engagement -> charge back
//       >= 90 % of its max over the 0.25 s before the shift began, after a dip) 4->5, 5->6, 6->7 <= 2.0 s each,
//       1->2, 2->3, 3->4 <= 0.89/0.91/1.13 s (pre-B0 + 0.1 s, measured with the first metric version)
//   V2  [long][!mayfail] owner setup (assists V12 fixture + drive_modes_car_hyper_n2o, min_time_in_gear 2.0, kickdown
//       0.95), speed hold at 300 km/h, 60 s: <= 1 gear change in the last 20 s - fails with every controller variant,
//       see the case (a plant limit, P1/shift policy)
//   V3  [long] owner setup, constant pedal 0.7 (band set after the exploration run, see the case)
//
// Sabotages recorded at the bottom.

#include "vehicle_test_helpers.h"

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;

// ---- B2 ------------------------------------------------------------------------------------------------------------

struct ScriptedShift {
    real pre_charge_bar = 0.0;    // gauge, mean of the last 0.05 s before the cut
    real cut_max_charge_bar = 0.0; // gauge, max during cut + ramp (closed-plate deadhead)
    real rotor_rpm_ramp_end = 0.0;
    real wastegate_ramp_end = 0.0;
    real wastegate_pre_cut = 0.0;
    real wastegate_max_dev_held = 0.0; // max |gate - gate before the cut| over the steps the hold was requested
    bool dipped = false;          // the charge fell below 90 % of pre_charge after the ramp
    real recovery_s = -1.0;       // ramp end -> charge back >= 90 % after the dip (0 = never dipped, -1 = not in 4 s)
};

ScriptedShift scripted_upshift(bool hold) {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    SimulatedEngine e(d);
    const real omega = 5000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    ScriptedShift out;
    PreSolveResult r;
    constexpr int kSpool = 3 * 960, kCut = 259, kRamp = 96, kAfter = 4 * 960;
    real acc = 0.0;
    for (int i = 0; i < kSpool; ++i) {
        r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        if (i >= kSpool - 48) acc += (r.p_charge_pa - kPamb) / 1e5;
    }
    out.pre_charge_bar = acc / 48.0;
    out.wastegate_pre_cut = r.wastegate_opening;
    for (int i = 0; i < kCut + kRamp; ++i) {
        TcuRequest q;
        real ceiling = 0.0;
        if (i < kCut) {
            q.ignition_cut = true;
        } else {
            ceiling = std::min(1.0, real(i - kCut + 1) / real(kRamp));
        }
        q.hold_boost_control = hold && (q.ignition_cut || ceiling < 1.0);
        r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, ceiling}, q, kH);
        (void)e.post_solve(omega, kH);
        out.cut_max_charge_bar = std::max(out.cut_max_charge_bar, (r.p_charge_pa - kPamb) / 1e5);
        if (q.hold_boost_control)
            out.wastegate_max_dev_held = std::max(out.wastegate_max_dev_held, std::abs(r.wastegate_opening - out.wastegate_pre_cut));
    }
    out.rotor_rpm_ramp_end = r.turbo_rpm;
    out.wastegate_ramp_end = r.wastegate_opening;
    for (int i = 0; i < kAfter; ++i) {
        r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        const real c = (r.p_charge_pa - kPamb) / 1e5;
        if (c < 0.9 * out.pre_charge_bar) out.dipped = true;
        if (out.dipped && c >= 0.9 * out.pre_charge_bar) {
            out.recovery_s = (i + 1) * kH;
            return out;
        }
    }
    if (!out.dipped) out.recovery_s = 0.0;
    return out;
}

// ---- V1/V2 ---------------------------------------------------------------------------------------------------------

std::string cache_dir() {
    const std::string dir = std::string(PS_SOURCE_DIR) + "/out/boost_b0_map_cache";
    std::filesystem::create_directories(dir);
    return dir;
}

ps::vehicle::VehicleDesc load_car(bool owner) {
    ps::jobs::JobSystem pool(10);
    ps::io::VehicleLoadOptions opt;
    opt.maps.pool = &pool;
    opt.maps.cache_dir = cache_dir();
    std::string path = std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper_n2o.json";
    if (owner) {
        // The owner's racing_game garage setup (assists I2 V12 fixture: hyper_twin_large_n2o, target 15.48 bar, stage
        // map) with the shipped car's drive modes and auto-shift timing (the fixture predates assists I7).
        path = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/assists/v12/vehicle_v12.json";
        opt.electronics["drive_modes"] = std::string(PS_SOURCE_DIR) + "/data/electronics/drive_modes_car_hyper_n2o.json";
    }
    ps::vehicle::VehicleDesc d = ps::io::load_vehicle_json(path, opt);
    if (owner) {
        for (auto& c : d.controllers)
            if (auto* m = std::get_if<ManualTcuDesc>(&c.params)) {
                m->auto_shift.min_time_in_gear_s = 2.0;
                m->auto_shift.kickdown_throttle = 0.95;
            }
    }
    return d;
}

// A straight strip along +x: [-200, 20000] x [-20, 20] m in 20 m cells (no large triangles under the wheels, PHYS-047).
ps::MeshShape strip_mesh(ps::SurfaceId sid) {
    constexpr real kCell = 20.0, x0 = -200.0, x1 = 20000.0, y0 = -20.0, y1 = 20.0;
    const int nx = static_cast<int>((x1 - x0) / kCell), ny = static_cast<int>((y1 - y0) / kCell);
    ps::MeshShape m;
    for (int j = 0; j <= ny; ++j)
        for (int i = 0; i <= nx; ++i) m.vertices.push_back(ps::Vec3{x0 + i * kCell, y0 + j * kCell, 0.0});
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const auto a = static_cast<std::uint32_t>(j * (nx + 1) + i);
            const std::uint32_t b = a + 1, c = a + static_cast<std::uint32_t>(nx + 1) + 1, dd = a + static_cast<std::uint32_t>(nx + 1);
            m.indices.insert(m.indices.end(), {a, b, c, a, c, dd});
            m.triangle_surface_ids.push_back(sid);
            m.triangle_surface_ids.push_back(sid);
        }
    return m;
}

struct Upshift {
    int from = 0, to = 0;
    real recovery_s = -1.0; // engagement -> charge >= 90 % of the pre-shift max after a dip; 0 = no dip in 3 s, -1 = > 8 s
};

struct DriveResult {
    std::vector<Upshift> upshifts;
    std::vector<real> gear_change_t; // every change of the engaged (nonzero) gear
    int downshifts = 0;
    real seconds = 0.0;
    real late_kmh_min = 1e9, late_kmh_max = -1e9; // over the last 20 s
};

// WOT (0.6 up to 40 km/h) with auto-shift; hold_kmh > 0 switches to a PI speed hold on the pedal once reached
// (racing_game's gains: kp 0.08 per m/s, ki 0.05 per m/s/s, integral seeded 0.6); const_pedal >= 0 holds that pedal
// from the start instead.
DriveResult drive(const ps::vehicle::VehicleDesc& desc, real seconds, real hold_kmh, real const_pedal = -1.0) {
    ps::WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = 4;
    ps::World w(cfg);
    auto surfaces = std::make_shared<ps::io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    ps::BodyDesc ground;
    ground.shape = strip_mesh(surfaces->id_for("asphalt"));
    ground.motion = ps::BodyMotionType::Static;
    ground.friction = 1.0;
    w.create_body(ground);
    ps::BodyDesc chassis;
    chassis.shape = ps::BoxShape{ps::Vec3{2.0, 0.4, 0.15}};
    chassis.motion = ps::BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = ps::Vec3{0.0, 0.0, 0.5};
    const ps::BodyId body = w.create_body(chassis);
    w.set_surface_table(surfaces);
    const ps::VehicleId vid = w.create_vehicle(desc, body);
    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    ps_test::settle(w, 2.0);

    DriveResult res;
    res.seconds = seconds;
    const real dt = w.dt();
    const long steps = static_cast<long>(seconds / dt);
    real pedal = 0.0, integ = 0.0;
    bool started = false, holding = false;
    int prev_gear = w.powertrain_state(vid).gear, last_gear = prev_gear;
    std::vector<real> hist; // charge (gauge bar) over the last 60 ticks
    real shift_ref = 0.0, pending_ref = -1.0, pending_t = 0.0;
    bool dipped = false;
    ShiftPhase prev_phase = ShiftPhase::Idle;
    for (long k = 0; k < steps; ++k) {
        const real t = k * dt;
        const real kmh0 = w.get_motion(body).linear.x * 3.6;
        if (!started && (w.powertrain_state(vid).engine_state == EngineState::Running || t > 6.0)) {
            w.set_control("starter", 0.0);
            started = true;
        }
        if (started) {
            if (const_pedal >= 0.0) {
                pedal = const_pedal;
            } else if (hold_kmh > 0.0 && (holding || kmh0 >= hold_kmh)) {
                if (!holding) { holding = true; integ = 0.6; }
                const real e = (hold_kmh - kmh0) / 3.6;
                integ = std::clamp(integ + 0.05 * e * dt, 0.0, 1.0);
                pedal = std::clamp(0.08 * e + integ, 0.0, 1.0);
            } else {
                pedal = kmh0 < 40.0 ? 0.6 : 1.0;
            }
            w.set_control("throttle", pedal);
        }
        w.step();
        const auto& pt = w.powertrain_state(vid);
        const real charge = (!pt.engines.empty() && pt.engines[0].p_charge_pa > 0.0) ? (pt.engines[0].p_charge_pa - kPamb) / 1e5 : 0.0;
        // The reference is the charge before the SHIFT began (the TCU's phase leaving Idle - the ignition cut and its
        // closed-plate deadhead start there, before the gearbox reaches neutral).
        if (pt.shift_phase != ShiftPhase::Idle && prev_phase == ShiftPhase::Idle && !hist.empty())
            shift_ref = *std::max_element(hist.begin(), hist.end());
        prev_phase = pt.shift_phase;
        if (pt.gear != prev_gear) {
            if (pt.gear > 0) {
                if (last_gear > 0 && pt.gear != last_gear) res.gear_change_t.push_back(t);
                if (last_gear > 0 && pt.gear > last_gear) {
                    res.upshifts.push_back(Upshift{last_gear, int(pt.gear), -1.0});
                    pending_ref = shift_ref;
                    pending_t = t;
                    dipped = false;
                } else if (last_gear > 0 && pt.gear < last_gear) {
                    ++res.downshifts;
                    pending_ref = -1.0;
                }
                last_gear = pt.gear;
            }
            prev_gear = pt.gear;
        }
        hist.push_back(charge);
        if (hist.size() > 60) hist.erase(hist.begin());
        if (t >= seconds - 20.0) {
            const real kmh = w.get_motion(body).linear.x * 3.6;
            res.late_kmh_min = std::min(res.late_kmh_min, kmh);
            res.late_kmh_max = std::max(res.late_kmh_max, kmh);
        }
        if (pending_ref >= 0.0 && pt.gear > 0) {
            if (charge < 0.9 * pending_ref) dipped = true;
            if (pending_ref < 0.1) {
                res.upshifts.back().recovery_s = 0.0;
                pending_ref = -1.0;
            } else if (dipped && charge >= 0.9 * pending_ref) {
                res.upshifts.back().recovery_s = t - pending_t;
                pending_ref = -1.0;
            } else if (!dipped && t - pending_t > 3.0) {
                res.upshifts.back().recovery_s = 0.0;
                pending_ref = -1.0;
            } else if (t - pending_t > 8.0) {
                pending_ref = -1.0; // stays -1
            }
        }
        if (w.get_pose(body).position.x > 19500.0) break;
    }
    return res;
}

} // namespace

TEST_CASE("B1: a held boost controller keeps wastegate and integral bit-exact through a closed plate", "[turbo][drivetrain]") {
    TurboPair pair(tb::hyper_like_pair());
    pair.reset(kPamb, kTamb);
    // Spool at WOT until the PID regulates (gate partly open, integral inside (0, 1)).
    for (int i = 0; i < 2 * 960; ++i) pair.step(kH, kPamb, kTamb, 0.6, 0.65, 1150.0, 1.0);
    const real w0 = pair.wastegate, i0 = pair.boost_integral;
    INFO("wastegate " << w0 << " integral " << i0);
    REQUIRE(w0 > 0.05);
    REQUIRE(w0 < 0.95);
    REQUIRE(i0 > 0.0);
    // 0.3 s of a shift cut: plate 0.02, flows down, held.
    for (int i = 0; i < 288; ++i) {
        pair.step(kH, kPamb, kTamb, 0.05, 0.05, 900.0, 0.02, 0.0, nullptr, true);
        REQUIRE(pair.wastegate == w0);
        REQUIRE(pair.boost_integral == i0);
    }
    // The same closed plate without the hold: the pre-B0 rule at once (integral reset, gate moving open).
    pair.step(kH, kPamb, kTamb, 0.05, 0.05, 900.0, 0.02);
    CHECK(pair.boost_integral == 0.0);
    CHECK(pair.wastegate > w0);
}

TEST_CASE("B3: above 1.8 bar the boost PID's gains per Pa no longer shrink with the target", "[turbo][drivetrain]") {
    // A 3.6 bar target at half plate asks for the same 1.8 bar as a 1.8 bar target at full plate; with the error
    // normalised by min(target, 1.8 bar) the two controllers are the same controller, step for step.
    TurboPairDesc lo = tb::hyper_like_pair();
    TurboPairDesc hi = lo;
    REQUIRE(lo.target_boost_pa == 1.8e5);
    hi.target_boost_pa = 3.6e5;
    TurboPair a(lo), b(hi);
    a.reset(kPamb, kTamb);
    b.reset(kPamb, kTamb);
    bool regulated = false;
    for (int i = 0; i < 3 * 960; ++i) {
        a.step(kH, kPamb, kTamb, 0.6, 0.65, 1150.0, 1.0);
        b.step(kH, kPamb, kTamb, 0.6, 0.65, 1150.0, 0.5);
        REQUIRE(b.wastegate == a.wastegate);
        REQUIRE(b.boost_integral == a.boost_integral);
        REQUIRE(b.charge_pa == a.charge_pa);
        regulated = regulated || (a.wastegate > 0.05 && a.boost_integral > 0.0);
    }
    CHECK(regulated); // the comparison covers the PID's working range, not only a shut gate
}

// The cut lasts 0.27 s, the car's own high-gear upshift cut (stock car_hyper_n2o WOT: 0.25-0.29 s for 4->5..6->7,
// 0.34-0.47 s in the low gears [M]). The first design used 0.08 s and failed its band (held 0.740 s vs unheld 0.819 s,
// ratio 0.90 > 0.7, [M]): with the throttle actuator's lag the plate spent too little of so short a cut below 0.10 for
// the old rule to matter (unheld gate only 0.116 at ramp end). The band was kept; the cut length was corrected and the
// gate check (the hold covers the ceiling ramp too) added before the next run.
TEST_CASE("B2: holding the boost controller through a scripted upshift shortens the boost recovery", "[turbo][drivetrain]") {
    const ScriptedShift held = scripted_upshift(true);
    const ScriptedShift open = scripted_upshift(false);
    std::printf("B2 pre %.3f bar, gate %.3f | held: cut max %.3f bar, rotor %.0f rpm, gate %.3f (max dev %.3g), dipped %d, "
                "recovery %.4f s | unheld: cut max %.3f bar, rotor %.0f rpm, gate %.3f, dipped %d, recovery %.4f s\n",
                held.pre_charge_bar, held.wastegate_pre_cut, held.cut_max_charge_bar, held.rotor_rpm_ramp_end,
                held.wastegate_ramp_end, held.wastegate_max_dev_held, int(held.dipped), held.recovery_s, open.cut_max_charge_bar,
                open.rotor_rpm_ramp_end, open.wastegate_ramp_end, int(open.dipped), open.recovery_s);
    REQUIRE(held.pre_charge_bar == open.pre_charge_bar); // same spool, the script differs only in the hold
    REQUIRE(held.pre_charge_bar > 0.5);
    REQUIRE(open.dipped); // scenario precondition: the pre-B0 rule loses boost after the shift
    REQUIRE(open.recovery_s > 0.0);
    CHECK(held.wastegate_max_dev_held == 0.0); // cut AND ceiling ramp: the gate never moves
    CHECK(held.recovery_s >= 0.0);
    CHECK(held.recovery_s <= 0.7 * open.recovery_s);
    CHECK(held.rotor_rpm_ramp_end >= open.rotor_rpm_ramp_end);
}

TEST_CASE("V1: stock car_hyper_n2o recovers its boost after each WOT upshift", "[turbo][drivetrain][vehicle][long]") {
    const DriveResult r = drive(load_car(false), 40.0, 0.0);
    real band[8] = {0, 0.89, 0.91, 1.13, 2.0, 2.0, 2.0, 0}; // by the gear shifted FROM
    bool seen[8] = {};
    for (const Upshift& u : r.upshifts) {
        std::printf("V1 %d->%d recovery %.3f s\n", u.from, u.to, u.recovery_s);
        if (u.from < 1 || u.from > 6 || u.to != u.from + 1) continue;
        seen[u.from] = true;
        INFO(u.from << "->" << u.to);
        CHECK(u.recovery_s >= 0.0);
        CHECK(u.recovery_s <= band[u.from]);
    }
    for (int g = 1; g <= 6; ++g) {
        INFO("upshift from " << g);
        CHECK(seen[g]); // precondition: the 40 s WOT run reaches 7th
    }
}

// [!mayfail] (discriminating measurement, B0 diag 'B0 part-throttle dyno' at 3400 rpm = 7th gear at 300 km/h, owner
// setup, 20 s per point [M]): command 0.8 settles at 145-148 kW / 0.30 bar with the gate SHUT, 0.9 at 890 kW / 5.07 bar;
// no steady pedal gives the ~450-500 kW road load at 300 km/h in 7th (the 15.48 bar turbo cannot hold its rotor above
// ~50k rpm on that little exhaust energy, gate <= 1e-5 throughout the 7th-gear decay with every controller variant tried:
// pre-B0, hold in the cut only, hold through the whole shift, with and without the 1.8 bar normaliser - 7 downshifts in
// the last 20 s each). So the speed hold kicks down to 6th, where it overshoots, and the auto-shift's part-throttle
// upshift takes it back to 7th: a plant/shift-policy limit, not the boost controller. The fix belongs to the
// shift policy using the REACHABLE steady power per gear (P1's F), not to B0.
TEST_CASE("V2: the owner's 15.48 bar setup holds 300 km/h without gear hunting", "[turbo][drivetrain][vehicle][assist][long][!mayfail]") {
    const DriveResult r = drive(load_car(true), 60.0, 300.0);
    int late = 0;
    for (real t : r.gear_change_t) {
        std::printf("V2 gear change t=%.2f\n", t);
        if (t >= r.seconds - 20.0) ++late;
    }
    CHECK(late <= 1);
}

// V3: steady part throttle on the owner setup. Band set AFTER one exploration run of each variant with the B0 diag
// (constant pedal 0.7, 60 s, last 20 s: pre-B0 266.5-389.7 km/h - every upshift collapsed the boost, after 6->7 at 388
// km/h it never respooled and the car fell to 266 km/h at 104-145 kW; B0 385.7-390.8 km/h at 819-840 kW) [M]:
// no downshift and a speed range <= 15 km/h over the last 20 s.
TEST_CASE("V3: the owner's setup keeps its boost through the upshifts at a steady 0.7 pedal", "[turbo][drivetrain][vehicle][assist][long]") {
    const DriveResult r = drive(load_car(true), 60.0, 0.0, 0.7);
    for (const Upshift& u : r.upshifts) std::printf("V3 %d->%d recovery %.3f s\n", u.from, u.to, u.recovery_s);
    std::printf("V3 last 20 s %.1f..%.1f km/h, downshifts %d\n", r.late_kmh_min, r.late_kmh_max, r.downshifts);
    CHECK(r.downshifts == 0);
    CHECK(r.late_kmh_max - r.late_kmh_min <= 15.0);
}

// Measured with the fix [M] (release, 2026-10-13): B2 held 0.770 s vs unheld 2.037 s, gate unmoved (0.062);
// V1 0.562/0.504/0.433/0.000/0.000/0.000 s; V3 385.7-390.8 km/h, no downshift; V2 7 gear changes in the last 20 s.
//
// Sabotages (each built and run once, then reverted; numbers [M]):
//   S1 hold ignored in TurboPair::step (`if (hold_control && plate < -1)`): B1 fails (integral 0.380 vs 0.374), B2 fails
//      (gate 0.918 at ramp end, max dev 0.858; recovery 2.037 s both ways), V1 fails (4->5 2.11, 5->6 2.25, 6->7
//      2.18 s > 2.0); B3, V3 pass.
//   S2 hold only while plate < 0.10 (the ceiling ramp unholds): B2 fails (gate dev 0.0645 during the ramp); B1, B3,
//      V1 (0.596/0.554/0.546/0/0/0), V3 pass.
//   S3 error normalised by the target again: B3 fails at the first step (gate 0.00515 vs 0); B1, B2, V1, V3 pass.
//   S4 the Powertrain never sets TcuRequest::hold_boost_control: V1 fails (as S1); B1-B3, V3 pass (they drive the
//      pair/engine directly).
//   V3 is an integration guard: it fails only with BOTH the hold and the normaliser removed (pre-B0 exploration run:
//      266.5-389.7 km/h); S1 alone 381.7-389.0, S3 alone 383.7-390.2 km/h.
