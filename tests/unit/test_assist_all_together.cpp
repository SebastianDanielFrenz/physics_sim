// tests/unit/test_assist_all_together.cpp - assists plan I6 (vault PHYS-074), plan B6 row V11 for ALL assists on ONE car:
// 1 vs 10 workers equal state hash every tick AND zero own allocation per tick, while car_hyper_n2o runs
//   - drive modes (Sport -> Normal -> Sport mid-run, with the TC default override and its release by a driver write),
//   - traction-control mode changes (On -> Sport -> Drift by channel writes, plus the drive mode's own TC default),
//   - launch control (Armed -> Launch -> Handover) against the brake, TC suspended while it owns the car,
//   - rear-wing automation (cruise ramp, the launch wing request, an airbrake event above 20 m/s),
//   - auto-clutch / auto-blip / auto-shift with a kickdown,
//   - N2O armed (sprays in Sport, gated off in Normal).
// Before this file each assist had its own 1-vs-10 / zero-alloc case (test_assist_unit D1, _aero D1/V11, _launch V11,
// _low_speed V11, _drive_modes D2, _fitment S2/S3) but none with two or more assists live together, which is where an
// ordering or state-folding fault between assists would show.
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists I6 brief, plan B6 V11; all [AS] pass bands):
//   A1  state_hash equal at EVERY tick, 1 worker vs 10 workers (a vector compare; the first differing tick is reported)
//   A2  own allocation count 0 per run on both worker counts (debug_alloc, armed from the first scripted tick; the external
//       count - Jolt's QuadTree rebuild gap, a known pre-existing D8 gap - is REPORTED, never asserted)
//   N   non-vacuity (an equality between two runs that did nothing would prove nothing). Over the run on 1 worker:
//       launch_phase takes the values 1 (Armed), 2 (Launch) and 3 (Handover); traction_mode_effective takes >= 3 distinct
//       values; drive_mode_effective takes the values 1 and 2; at least 2 upshifts (gear_target increases); N2O sprays in
//       > 0 ticks while the effective drive mode is Sport and in 0 ticks while it is Normal; aero_control_target_deg >= 44.9
//       at some tick with speed > 20 m/s (the airbrake) and == launch_offset_deg 10 at some tick while the launch phase is
//       Armed (the launch wing request); >= 1 downshift whose throttle command is >= 0.95 (a kickdown) and a top speed above
//       40 m/s. (First run of the 5 s variant failed N on a too-short timeline - a setup fault, so the script was retimed to
//       12.5 s; A1/A2 and the N thresholds themselves were not touched. The run takes about 1 s in release, so there is
//       no [long] variant and no separate short one.)
//   P1  UI guard (assert, no run comparison; assists I7c semantics): after a drive-mode change the TC channel itself holds the
//       mode's code, and ONE write of any value - even the value the driver had before - takes effect at once.
//
// RECORDED SABOTAGES: see out/assists_i6_progress.md (temporary env-switched edits in core/src, reverted).

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using ps::vehicle::VehicleDesc;
namespace fs = std::filesystem;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";

const VehicleDesc& hyper_n2o() {
    static const VehicleDesc d = [] {
        static jobs::JobSystem pool(10);
        io::VehicleLoadOptions o;
        o.maps.pool = &pool;
        o.maps.cache_dir = std::string(PS_SOURCE_DIR) + "/out/assists_i6_map_cache";
        fs::create_directories(o.maps.cache_dir);
        return io::load_vehicle_json(kData + "/vehicles/car_hyper_n2o.json", o);
    }();
    return d;
}

// One scripted event at a tick: (tick, channel, value).
struct Ev {
    int tick;
    const char* channel;
    real value;
};

struct Script {
    real seconds;
    std::vector<Ev> events;
};

// Script (12.5 s, 240 Hz ticks): LC armed at once with the brake on, brake released at 0.6 s (Launch -> Handover), auto-shift on;
// TC On -> Sport (1.2 s) -> Drift (2.4 s) by channel writes; Normal at 1.6 s and Sport at 2.0 s (TC default overrides); a pedal
// dip to 0.3 at 5.0 s; Eco at 7.0 s, Sport at 7.5 s, a Normal blip at 8.0-8.25 s while N2O sprays; throttle 0 + brake from
// 9.0 s to 10.0 s (airbrake above 20 m/s); full pedal again at 10.25 s from a low rpm (kickdown).
Script full_script() {
    return {12.5,
            {{0, "assist.launch_control", 1.0},
             {0, "nitrous_arm", 1.0},
             {0, "brake", 1.0},
             {0, "throttle", 1.0},
             {0, "handbrake", 0.0},
             {144, "brake", 0.0},
             {144, "assist.auto_shift", 1.0},
             {144, "gear_request", drivetrain::kGearRequestNone},
             {288, "assist.traction_control", 3.0},
             {384, "assist.drive_mode", 1.0},
             {480, "assist.drive_mode", 2.0},
             {576, "assist.traction_control", 4.0},
             {1200, "throttle", 0.3},
             {1440, "throttle", 1.0},
             {1680, "assist.drive_mode", 0.0},
             {1800, "assist.drive_mode", 2.0},
             {1920, "assist.drive_mode", 1.0},
             {1980, "assist.drive_mode", 2.0},
             {2160, "throttle", 0.0},
             {2160, "brake", 1.0},
             {2400, "brake", 0.0},
             {2460, "throttle", 1.0}}};
}

struct Observed {
    std::vector<std::uint64_t> hashes;
    std::set<int> launch_phases, tc_modes, drive_modes;
    int upshifts = 0, downshifts = 0, kickdowns = 0;
    int spray_sport = 0, spray_normal = 0;
    double max_target_deg = 0, launch_target_deg = -1;
    double final_speed = 0, max_speed = 0;
    std::uint64_t own_alloc = 0, external_alloc = 0;
};

Observed run_script(const Script& s, int workers, bool trace = false) {
    auto rig = ps_test::build_vehicle_rig_from_desc(hyper_n2o(), 1500, {2, .4, .15}, .5, 0.0, workers);
    World& w = *rig.world;
    // Spawn state (before the first tick, so no mode change is seen): Sport, TC On, wing automation (the unit's default),
    // clutch/blip assists on, auto-shift off until the launch.
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("assist.traction_control", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.0);

    namespace da = jobs::debug_alloc;
    const bool use_alloc = da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    Observed o;
    const int n = static_cast<int>(s.seconds * 240.0 + 0.5);
    o.hashes.reserve(static_cast<std::size_t>(n));
    int prev_target = -99;
    real prev_command = 0;
    std::size_t next_event = 0;
    for (int k = 0; k < n; ++k) {
        while (next_event < s.events.size() && s.events[next_event].tick == k) {
            w.set_control(s.events[next_event].channel, s.events[next_event].value);
            ++next_event;
        }
        w.step();
        o.hashes.push_back(w.state_hash());
        const auto& pt = w.powertrain_state(rig.vehicle);
        const double speed = w.get_motion(rig.chassis).linear.x;
        o.launch_phases.insert(pt.launch_phase);
        o.tc_modes.insert(pt.traction_mode_effective);
        o.drive_modes.insert(pt.drive_mode_effective);
        if (prev_target != -99 && pt.gear_target > prev_target) ++o.upshifts;
        if (prev_target != -99 && pt.gear_target < prev_target) {
            ++o.downshifts;
            // A kickdown can fire on the very tick the pedal reaches full travel, so the command of that tick counts too.
            const real now_command = pt.engines.empty() ? real(0) : pt.engines[0].throttle_command;
            if (prev_command >= 0.95 || now_command >= 0.95) ++o.kickdowns;
        }
        prev_target = pt.gear_target;
        const real command = pt.engines.empty() ? real(0) : pt.engines[0].throttle_command;
        prev_command = command;
        if (!pt.engines.empty() && pt.engines[0].n2o_active) {
            if (pt.drive_mode_effective == 2) ++o.spray_sport;
            if (pt.drive_mode_effective == 1) ++o.spray_normal;
        }
        if (pt.aero_control_active) {
            if (speed > 20.0) o.max_target_deg = std::max(o.max_target_deg, static_cast<double>(pt.aero_control_target_deg));
            if (pt.launch_phase == 1) o.launch_target_deg = pt.aero_control_target_deg;
        }
        if (trace && k % 30 == 29) {
            std::printf("  t %5.2f v %6.2f gear %d tgt %d lc %d tc %d dm %d rpm %6.0f cmd %4.2f n2o %d wing %5.1f ce %4.2f\n", (k + 1) / 240.0, speed, pt.gear, pt.gear_target,
                        pt.launch_phase, pt.traction_mode_effective, pt.drive_mode_effective, pt.engine_rpm, double(command),
                        (!pt.engines.empty() && pt.engines[0].n2o_active) ? 1 : 0, double(pt.aero_control_target_deg), double(pt.clutch_engagement));
        }
        o.max_speed = std::max(o.max_speed, speed);
        o.final_speed = speed;
    }
    if (use_alloc) {
        da::set_armed(false);
        o.own_alloc = da::own_violation_count();
        o.external_alloc = da::external_violation_count();
    }
    return o;
}

void check_all_together(const Script& s) {
    if (!jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const Observed a = run_script(s, 1);
    const Observed b = run_script(s, 10);

    REQUIRE(a.hashes.size() == b.hashes.size());
    std::size_t first_diff = a.hashes.size();
    for (std::size_t i = 0; i < a.hashes.size(); ++i)
        if (a.hashes[i] != b.hashes[i]) {
            first_diff = i;
            break;
        }
    INFO("first differing tick " << first_diff << " of " << a.hashes.size());
    CHECK(first_diff == a.hashes.size());
    if (jobs::debug_alloc::hook_available()) {
        CHECK(a.own_alloc == 0);
        CHECK(b.own_alloc == 0);
    }

    std::printf("[V11-all] ticks %zu, phases %zu tc_modes %zu drive_modes %zu, up %d down %d kickdown %d, spray sport %d normal %d, "
                "max airbrake target %.2f deg, wing target Armed %.2f deg, speed max %.1f final %.1f m/s, own alloc %llu/%llu, external alloc %llu/%llu\n",
                a.hashes.size(), a.launch_phases.size(), a.tc_modes.size(), a.drive_modes.size(), a.upshifts, a.downshifts,
                a.kickdowns, a.spray_sport, a.spray_normal, a.max_target_deg, a.launch_target_deg, a.max_speed, a.final_speed,
                static_cast<unsigned long long>(a.own_alloc), static_cast<unsigned long long>(b.own_alloc),
                static_cast<unsigned long long>(a.external_alloc), static_cast<unsigned long long>(b.external_alloc));

    // N: non-vacuity.
    CHECK(a.launch_phases.count(1) == 1);
    CHECK(a.launch_phases.count(2) == 1);
    CHECK(a.launch_phases.count(3) == 1);
    CHECK(a.tc_modes.size() >= 3);
    CHECK(a.drive_modes.count(1) == 1);
    CHECK(a.drive_modes.count(2) == 1);
    CHECK(a.upshifts >= 2);
    CHECK(a.spray_sport > 0);
    CHECK(a.spray_normal == 0);
    CHECK(a.max_target_deg >= 44.9);
    CHECK(a.launch_target_deg == 10.0);
    CHECK(a.kickdowns >= 1);
    CHECK(a.max_speed > 40.0);
}

} // namespace

TEST_CASE("V11-all: car_hyper_n2o with every assist live - 1 vs 10 workers equal hash every tick, zero own allocation (12.5 s)",
          "[assist][vehicle][drivetrain][determinism][long]") {
    check_all_together(full_script());
}

// Assists I7c (old -> new): P1 used to guard "rewriting the SAME TC value does not release a drive mode's override"; the
// override state is gone - the mode change WRITES its default into the channel - so the pitfall inverts: the channel reads
// the mode's code after the change, and ONE write of any value (including 3, the value the driver had before) takes effect.
TEST_CASE("P1: after a drive-mode change the TC channel holds the mode's code; one write of any value (even a repeat of the earlier one) takes effect", "[assist][vehicle][drivetrain]") {
    auto rig = ps_test::build_vehicle_rig_from_desc(hyper_n2o(), 1500, {2, .4, .15}, .5, 0.0, 1);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", 1.0); // Normal: TC default On
    w.set_control("assist.traction_control", 3.0);
    w.set_control("assist.aero_control", 0.0);
    ps_test::settle(w, 0.2);
    REQUIRE(w.powertrain_state(rig.vehicle).traction_mode_effective == 3); // spawn: no write, the driver's value runs
    w.set_control("assist.drive_mode", 2.0); // Sport: TC default Relaxed -> written into the channel
    ps_test::settle(w, 0.05);
    CHECK(w.powertrain_state(rig.vehicle).traction_mode_effective == 2);
    CHECK(w.powertrain_state(rig.vehicle).traction_mode_requested == 2);
    CHECK(w.get_control("assist.traction_control") == 2.0); // I7c: the channel equals what the car does
    w.set_control("assist.traction_control", 3.0); // one press: the value the driver had before wins at once
    ps_test::settle(w, 0.05);
    CHECK(w.powertrain_state(rig.vehicle).traction_mode_effective == 3);
    w.set_control("assist.traction_control", 4.0); // a different value: the driver wins
    ps_test::settle(w, 0.05);
    CHECK(w.powertrain_state(rig.vehicle).traction_mode_effective == 4);
    w.set_control("assist.traction_control", 3.0); // and from then on 3 is a plain driver request again
    ps_test::settle(w, 0.05);
    CHECK(w.powertrain_state(rig.vehicle).traction_mode_effective == 3);
}

// Hidden: a printed trace of the script for tuning the timeline.
TEST_CASE("V11-all diag: trace", "[.][diag]") {
    (void)run_script(full_script(), 1, true);
}
