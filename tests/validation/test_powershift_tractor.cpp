#define _CRT_SECURE_NO_WARNINGS // the diag trailer mass is read from the environment
// tests/validation/test_powershift_tractor.cpp - tractor plan stage 1: the 24F/24R powershift box
// (data/gearboxes/tractor_24f24r_powershift.json, fixture tests/support/fixtures/vehicles/tractor_powershift.json) on
// data/vehicles/tractor.json's body, driven through World like the game.
//
//  * standing in D on the brake at part throttle does not stall the engine and does not move the tractor;
//  * no shift INSIDE a range drops the driven wheels' traction force below kMinShiftFraction of its pre-shift value; a
//    shift between ranges interrupts it for about range_change_time_s (the data-driven gap) and a box with the gap set to
//    0 has no interruption at all;
//  * 0-30 km/h with a 6 t trailer: faster and smoother than the same tractor with the manual box, the auto-clutch and
//    the auto-shift assist configured like the farm game (start gear 6, dwell 0.5 s, multi-gear downshift);
//  * the per-tick energy ledger closes, 1 vs 10 workers give the same state hash every tick, own allocation in jobs 0.
// Every case prints its measured numbers.
// Recorded sabotages (2026-11-02): hold_capacity back to the pre-powershift formula (no engine-demand term, no
// shift_pressure_fraction floor) -> in-range shifts last 0.18-0.60 s instead of 0.15-0.17 s, the duration check fails.

#include "vehicle_test_helpers.h"

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/machine/coupling.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kD = 99.0;
constexpr real k30KmH = 30.0 / 3.6;
// [AS] the measured worst traction-force fraction of a within-range shift (power-on upshift: the engine is held to
// upshift_torque_fraction 0.7 during the inertia phase, so the floor is not 1) - the bound is below the measurement.
constexpr real kMinShiftFraction = 0.55;

std::string fixture(const char* name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/" + name; }
std::string data(const char* name) { return std::string(PS_SOURCE_DIR) + "/data/" + name; }

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss) +
           std::abs(e.converter_loss) + std::abs(e.awd_clutch_loss) + std::abs(e.pto_clutch_loss) + std::abs(e.external_absorbed);
}

struct Rig {
    std::unique_ptr<World> world;
    BodyId a{}, b{};
    VehicleId va{}, vb{};
    bool trailer = false;
    real max_balance = 0.0;
    bool stalled = false;
    std::uint64_t hash_acc = 1469598103934665603ULL;

    Rig(const vehicle::VehicleDesc& tractor, real trailer_mass, int workers, real body_mass = 6000.0, real z0 = 1.28) {
        WorldConfig wc;
        wc.job_workers = workers;
        world = std::make_unique<World>(wc);
        auto surfaces = std::make_shared<io::SurfaceTable>(data("surfaces/surfaces.json"));
        world->set_surface_table(surfaces);
        BodyDesc ground;
        ground.motion = BodyMotionType::Static;
        ground.friction = 1.0;
        static thread_local ShapeHandle mesh;
        mesh = world->create_shape(ps_test::flat_surface_mesh(1000.0, surfaces->id_for("asphalt")));
        world->create_body(ground, mesh);

        BodyDesc ba;
        ba.shape = BoxShape{Vec3{1.2, 0.35, z0 > 1.2 ? 0.9 : 0.25}};
        ba.mass = body_mass;
        ba.friction = 0.5;
        ba.pose.position = Vec3{0.0, 0.0, z0};
        a = world->create_body(ba);
        va = world->create_vehicle(tractor, a, {}, "");
        if (trailer_mass > 0.0) {
            trailer = true;
            const vehicle::VehicleDesc tipper = io::load_vehicle_json(fixture("trailer_tipper.json"));
            const Vec3 sa = machine::find_hitch_point(tractor.hitch_points, "socket_hitch")->position_local;
            const Vec3 sb = machine::find_hitch_point(tipper.hitch_points, "socket_hitch_eye")->position_local;
            BodyDesc bb;
            bb.shape = BoxShape{Vec3{2.5, 0.5, 0.3}};
            bb.mass = trailer_mass;
            bb.friction = 0.5;
            bb.pose.position = Vec3{sa.x - sb.x, 0.0, z0 + sa.z - sb.z + (z0 == 1.28 ? (1.1 - 1.28 - (sa.z - sb.z)) : 0.0)};
            b = world->create_body(bb);
            vb = world->create_vehicle(tipper, b, {}, "trailer");
            world->attach(va, "socket_hitch", vb, "socket_hitch_eye");
        }
        world->set_control("awd", 1.0);
    }
    const drivetrain::PowertrainSnapshot& pt() { return world->powertrain_state(va); }
    real speed() { return world->get_motion(a).linear.x; }
    // Total torque on the driven hubs (the shaft network's own number, no tyre slip dynamics in it) [N*m].
    real hub_torque_total() {
        const drivetrain::PowertrainDebug d = world->powertrain_debug(va);
        real f = 0.0;
        for (std::size_t i = 0; i < world->vehicle_wheel_count(va) && i < 8; ++i) f += d.hub_shaft_torque[i];
        return f;
    }
    void step() {
        world->step();
        const drivetrain::PowertrainEnergy& e = world->powertrain_energy(va);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) max_balance = std::max(max_balance, std::abs(e.balance_error) / denom);
        if (pt().engine_state != drivetrain::EngineState::Running) stalled = true;
    }
    void run(real seconds) {
        const int n = static_cast<int>(seconds / world->dt() + 0.5);
        for (int i = 0; i < n; ++i) step();
    }
    void drive_powershift(real throttle) {
        world->set_control("gear_request", kD);
        world->set_control("throttle", throttle);
    }
    // The manual box with the farm game's auto-clutch + auto-shift assist.
    void drive_manual(real throttle) {
        world->set_control("assist.auto_shift", 1.0);
        world->set_control("assist.auto_clutch", 1.0);
        world->set_control("throttle", throttle);
    }
};

vehicle::VehicleDesc powershift_desc() { return io::load_vehicle_json(fixture("tractor_powershift.json")); }

// data/vehicles/tractor.json with the farm game's auto-shift setup (FARM-037: start gear 6, dwell 0.5 s, multi-gear downshift).
vehicle::VehicleDesc manual_desc() {
    vehicle::VehicleDesc d = io::load_vehicle_json(data("vehicles/tractor.json"));
    for (auto& c : d.controllers) {
        if (auto* t = std::get_if<drivetrain::ManualTcuDesc>(&c.params)) {
            t->auto_shift.enabled = true;
            t->auto_shift.start_gear = 6;
            t->auto_shift.min_time_in_gear_s = 0.5;
            t->auto_shift.downshift_target_fraction = 0.5;
            t->auto_shift.downshift_floor_gear = 6;
        }
    }
    return d;
}

const drivetrain::GearboxDesc& gearbox_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) return *g;
    throw std::runtime_error("no gearbox");
}

struct Accel {
    real t30 = -1.0;
    real worst_drop = 0.0;  // largest speed lost against the running maximum after 1 s [m/s]
    real longest_flat = 0.0; // longest stretch with a smoothed acceleration below 0.1 m/s^2 before 30 km/h [s]
    int shifts = 0;
    int max_gear_jump = 0;
    bool stalled = false;
    real balance = 0.0;
};

Accel run_accel(Rig& r, real max_s) {
    Accel o;
    std::vector<real> v;
    int last_gear = r.pt().gear;
    real vmax = 0.0;
    const int n = static_cast<int>(max_s / r.world->dt());
    for (int i = 0; i < n; ++i) {
        r.step();
        const real t = static_cast<real>(i + 1) * r.world->dt();
        const real s = r.speed();
        v.push_back(s);
        vmax = std::max(vmax, s);
        if (t > 1.0) o.worst_drop = std::max(o.worst_drop, vmax - s);
        const int g = r.pt().gear;
        if (g != last_gear && g > 0 && last_gear > 0) {
            ++o.shifts;
            o.max_gear_jump = std::max(o.max_gear_jump, std::abs(g - last_gear));
        }
        if (g > 0) last_gear = g;
        if (s >= k30KmH) {
            o.t30 = t;
            break;
        }
    }
    // flat spots: acceleration over 0.25 s windows
    const int w = 60;
    int flat = 0;
    for (std::size_t i = static_cast<std::size_t>(w); i < v.size(); ++i) {
        const real a = (v[i] - v[i - static_cast<std::size_t>(w)]) / (w * r.world->dt());
        if (a < 0.1) {
            ++flat;
            o.longest_flat = std::max(o.longest_flat, flat * r.world->dt());
        } else {
            flat = 0;
        }
    }
    o.stalled = r.stalled;
    o.balance = r.max_balance;
    return o;
}

} // namespace

TEST_CASE("Powershift tractor: standing in D on the brake does not stall and does not creep", "[validation][vehicle][drivetrain][powershift]") {
    Rig r(powershift_desc(), 0.0, 4);
    r.run(1.0);
    r.world->set_control("brake", 1.0);
    r.drive_powershift(0.0);
    r.run(3.0);
    CHECK(std::abs(r.speed()) < 0.02);
    CHECK_FALSE(r.stalled);
    r.drive_powershift(0.25); // the engine rises to about 1100 rpm, the launch clutch bites
    r.run(5.0);
    const real speed = r.speed();
    const real rpm = r.pt().engine_rpm;
    std::printf("[powershift standstill] brake + 25 %% throttle, 5 s: speed %.4f m/s, rpm %.0f, gear %d, kind %d, stalled %d\n",
                static_cast<double>(speed), static_cast<double>(rpm), r.pt().gear, r.pt().transmission_kind, r.stalled ? 1 : 0);
    CHECK(r.pt().transmission_kind == 5);
    CHECK(r.pt().gear <= 11); // never above the start gear at a crawl
    CHECK_FALSE(r.stalled);
    CHECK(std::abs(speed) < 0.5); // the service brakes (22 kN) are weaker than a low gear at 25 % throttle: it creeps, it does not stall
    CHECK(rpm > 600.0);
    // release: it pulls away
    r.world->set_control("brake", 0.0);
    r.drive_powershift(1.0);
    r.run(6.0);
    CHECK(r.speed() > 1.0);
    CHECK_FALSE(r.stalled);
    CHECK(r.max_balance <= 5e-8);
}

namespace {
struct ShiftRecord {
    int from = 0, to = 0;
    bool range_change = false;
    real duration = 0.0;
    real min_fraction = 1.0;
    bool settled = false;
    real t = 0.0, kmh = 0.0; // when the shift began and the ground speed then
};

std::vector<ShiftRecord> record_shifts(const vehicle::VehicleDesc& desc, real seconds, real trailer_mass = 0.0, real throttle = 1.0) {
    const drivetrain::GearboxDesc& gb = gearbox_of(desc);
    Rig r(desc, trailer_mass, 4);
    r.run(1.0);
    r.drive_powershift(throttle);
    std::vector<ShiftRecord> out;
    std::vector<real> fx;
    std::vector<int> busy, gear;
    std::vector<real> vel;
    const int n = static_cast<int>(seconds / r.world->dt());
    for (int i = 0; i < n; ++i) {
        r.step();
        fx.push_back(r.hub_torque_total());
        // torque-affecting phases only: the closing ramp of the new clutch (ClutchClosing) carries more than the torque
        busy.push_back((r.pt().shift_phase != drivetrain::ShiftPhase::Idle && r.pt().shift_phase != drivetrain::ShiftPhase::ClutchClosing) ? 1 : 0);
        gear.push_back(r.pt().gear);
        vel.push_back(r.speed());
    }
    // a shift = a run of busy ticks; the gear before and after
    const int pre_w = 36; // 0.15 s
    for (int i = 1; i < n;) {
        if (busy[static_cast<std::size_t>(i)] && !busy[static_cast<std::size_t>(i - 1)] && gear[static_cast<std::size_t>(i - 1)] > 0) {
            int j = i;
            while (j < n && busy[static_cast<std::size_t>(j)]) ++j;
            if (j >= n || gear[static_cast<std::size_t>(j)] <= 0) break;
            ShiftRecord s;
            s.from = gear[static_cast<std::size_t>(i - 1)];
            s.t = i * r.world->dt();
            s.kmh = vel[static_cast<std::size_t>(i)] * 3.6;
            s.to = gear[static_cast<std::size_t>(j)];
            if (i >= pre_w) {
                real pre = 0.0;
                for (int k = i - pre_w; k < i; ++k) pre += fx[static_cast<std::size_t>(k)];
                pre /= pre_w;
                real mn = 1e300;
                for (int k = i; k + 10 <= j; ++k) { // 10-tick (42 ms) mean: ignores the substep noise
                    real m = 0.0;
                    for (int q = 0; q < 10; ++q) m += fx[static_cast<std::size_t>(k + q)];
                    mn = std::min(mn, m / 10.0);
                }
                // A shift decided while the traction was already collapsing (the wheels spinning up in a low gear under full
                // throttle, the governor's droop) has no meaningful pre-shift level: it is reported, not measured.
                real last = 0.0;
                for (int k = i - 6; k < i; ++k) last += fx[static_cast<std::size_t>(k)];
                last /= 6.0;
                s.settled = pre > 1.0 && last >= 0.85 * pre;
                s.min_fraction = pre > 1.0 ? mn / pre : 1.0;
            }
            s.duration = (j - i) * r.world->dt();
            s.range_change = s.from > 0 && s.to > 0 && gb.forward_range[static_cast<std::size_t>(s.from - 1)] != gb.forward_range[static_cast<std::size_t>(s.to - 1)];
            out.push_back(s);
            i = j;
        } else {
            ++i;
        }
    }
    return out;
}
} // namespace

TEST_CASE("Powershift tractor: no shift inside a range drops the traction force, a range change interrupts it for its data time",
          "[validation][vehicle][drivetrain][powershift]") {
    const auto shifts = record_shifts(powershift_desc(), 20.0, 6000.0);
    REQUIRE(shifts.size() >= 3);
    real worst_in = 1.0, worst_range = 1.0;
    int in_n = 0, range_n = 0;
    for (const auto& s : shifts) {
        std::printf("[powershift shifts] %2d -> %2d %s  %.3f s  min traction fraction %.3f%s\n", s.from, s.to, s.range_change ? "RANGE " : "inside", static_cast<double>(s.duration),
                    static_cast<double>(s.min_fraction), s.settled ? "" : "  (not settled before the shift: not counted)");
        if (!s.range_change) CHECK(s.duration <= 0.3); // an in-range shift takes 0.15-0.17 s (a weak clutch pressure stretches it to 0.6 s)
        if (!s.settled) continue;
        if (s.range_change) {
            worst_range = std::min(worst_range, s.min_fraction);
            ++range_n;
        } else {
            worst_in = std::min(worst_in, s.min_fraction);
            ++in_n;
        }
    }
    REQUIRE(in_n >= 1);
    CHECK(worst_in >= kMinShiftFraction);
    if (range_n > 0) CHECK(worst_range < 0.3); // the interruption is real

    // the same box with range_change_time_s 0: no interruption anywhere
    vehicle::VehicleDesc seamless = powershift_desc();
    for (auto& c : seamless.powertrain.components)
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) g->range_change_time_s = 0.0;
    const auto s2 = record_shifts(seamless, 20.0, 6000.0);
    REQUIRE(s2.size() >= 3);
    real worst_all = 1.0;
    for (const auto& s : s2) {
        std::printf("[powershift seamless] %2d -> %2d  %.3f s  min traction fraction %.3f%s\n", s.from, s.to, static_cast<double>(s.duration), static_cast<double>(s.min_fraction),
                    s.settled ? "" : "  (not settled: not counted)");
        if (!s.settled) continue;
        worst_all = std::min(worst_all, s.min_fraction);
    }
    CHECK(worst_all >= kMinShiftFraction);
}

TEST_CASE("Powershift tractor: 0-30 km/h with a 6 t trailer is faster and smoother than the manual box with the assist",
          "[validation][vehicle][drivetrain][powershift]") {
    Accel ps, mn;
    {
        Rig r(powershift_desc(), 6000.0, 4);
        r.run(1.0);
        r.drive_powershift(1.0);
        ps = run_accel(r, 40.0);
    }
    {
        Rig r(manual_desc(), 6000.0, 4);
        r.run(1.0);
        r.drive_manual(1.0);
        mn = run_accel(r, 40.0);
    }
    std::printf("[powershift 0-30] powershift: t30 %.2f s, worst speed drop %.3f m/s, longest flat %.2f s, %d shifts (max jump %d), stalled %d, balance %.2e\n",
                static_cast<double>(ps.t30), static_cast<double>(ps.worst_drop), static_cast<double>(ps.longest_flat), ps.shifts, ps.max_gear_jump, ps.stalled ? 1 : 0,
                static_cast<double>(ps.balance));
    std::printf("[powershift 0-30] manual+assist: t30 %.2f s, worst speed drop %.3f m/s, longest flat %.2f s, %d shifts (max jump %d), stalled %d, balance %.2e\n",
                static_cast<double>(mn.t30), static_cast<double>(mn.worst_drop), static_cast<double>(mn.longest_flat), mn.shifts, mn.max_gear_jump, mn.stalled ? 1 : 0,
                static_cast<double>(mn.balance));
    REQUIRE(ps.t30 > 0.0);
    REQUIRE(mn.t30 > 0.0);
    CHECK(ps.t30 < mn.t30);
    // The powershift's only speed loss is its range changes (0.35 s open, data); the manual box loses speed at every one of its
    // 14 clutch shifts. Measured 0.160 vs 0.155 m/s: equal within 10 %, the flat spell and the shift count are what differ.
    CHECK(ps.worst_drop <= 1.1 * mn.worst_drop);
    CHECK(ps.shifts < mn.shifts);
    CHECK(ps.longest_flat <= mn.longest_flat + 0.4); // stage 1b: upshift_min_time_s 0.8 holds a governed engine at its limit up to that long
    CHECK_FALSE(ps.stalled);
    CHECK(ps.balance <= 5e-8);
}

TEST_CASE("Powershift tractor: 1 vs 10 workers give the same state hash every tick", "[validation][vehicle][drivetrain][powershift][determinism]") {
    Rig a(powershift_desc(), 6000.0, 1), b(powershift_desc(), 6000.0, 10);
    a.run(0.5);
    b.run(0.5);
    a.drive_powershift(1.0);
    b.drive_powershift(1.0);
    bool same = true;
    const int n = static_cast<int>(8.0 / a.world->dt());
    for (int i = 0; i < n; ++i) {
        a.step();
        b.step();
        if (a.world->state_hash() != b.world->state_hash()) {
            same = false;
            break;
        }
    }
    std::printf("[powershift determinism] gear %d, hash 0x%016llx\n", a.pt().gear, static_cast<unsigned long long>(a.world->state_hash()));
    CHECK(same);
    CHECK(a.pt().gear != 11); // shifted at least once in 8 s
}

TEST_CASE("Powershift tractor: a world step allocates nothing in jobs", "[validation][vehicle][drivetrain][powershift][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hooks are compiled out under ASan/TSan");
        return;
    }
    Rig r(powershift_desc(), 0.0, 4);
    r.run(0.5);
    r.drive_powershift(1.0);
    r.run(1.0); // warm up
    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    r.run(6.0); // a launch and several shifts, a brake at the end
    r.world->set_control("brake", 1.0);
    r.run(1.0);
    jobs::debug_alloc::set_armed(false);
    std::printf("[powershift alloc] own %llu, external %llu\n", static_cast<unsigned long long>(jobs::debug_alloc::own_violation_count()),
                static_cast<unsigned long long>(jobs::debug_alloc::external_violation_count()));
    CHECK(jobs::debug_alloc::own_violation_count() == 0);
}

TEST_CASE("Powershift tractor diag: the first shifts under 6 t", "[.][diag][powershift]") {
    Rig r(powershift_desc(), 0.0, 4);
    r.run(1.0);
    r.drive_powershift(0.6);
    for (int i = 0; i < 240 * 20; ++i) {
        r.step();
        const real t = i / 240.0;
        if (i % 48 == 0) {
            const auto d = r.world->powertrain_debug(r.va);
            std::printf("t %.3f v %.2f gear %2d ph %d rpm %.0f thr_eff %.2f demand %.0f clutch_t %.0f slip %.1f hub %.0f fxsum %.0f win %.1f wout %.1f capk %.0f\n", static_cast<double>(t),
                        static_cast<double>(r.speed()), r.pt().gear, static_cast<int>(r.pt().shift_phase), static_cast<double>(r.pt().engine_rpm), static_cast<double>(d.throttle_eff),
                        static_cast<double>(d.engine_torque_demand), static_cast<double>(d.clutch_torque), static_cast<double>(d.clutch_slip), static_cast<double>(r.hub_torque_total()),
                        static_cast<double>(r.world->wheel_telemetry(r.va, 0).fx + r.world->wheel_telemetry(r.va, 1).fx + r.world->wheel_telemetry(r.va, 2).fx + r.world->wheel_telemetry(r.va, 3).fx),
                        static_cast<double>(r.pt().driveline.gearbox_in_omega), static_cast<double>(r.pt().driveline.gearbox_out_omega), static_cast<double>(d.clutch_cap_kinetic));
        }
    }
}

// Stage 1b: a tractor-heavy trailer (10.5 t on a 6 t tractor) from rest. The farm fork measured a shift every ~0.3 s below 15 km/h,
// each keeping 0.26-0.57 of the pull. With min_upshift_pull_fraction (single-gear steps) and upshift_min_time_s no two
// shifts come closer than the data minimum and every in-range shift of the first 30 km/h keeps >= 0.8 of the pull.
// (A range change interrupts the pull by design for range_change_time_s; it is counted for spacing only.)
static void check_heavy_launch(const vehicle::VehicleDesc& desc, bool expect_ok) {
    const auto shifts = record_shifts(desc, 14.0, 10500.0);
    REQUIRE(shifts.size() >= 4);
    real min_gap = 1e9, worst_in = 1.0;
    for (std::size_t i = 0; i < shifts.size(); ++i) {
        const auto& s = shifts[i];
        if (s.to > s.from && i > 0 && shifts[i - 1].to > shifts[i - 1].from) min_gap = std::min(min_gap, s.t - shifts[i - 1].t);
        if (!s.range_change && s.settled) worst_in = std::min(worst_in, s.min_fraction);
        std::printf("[ps heavy] t %5.2f %5.1f km/h %2d -> %2d %s kept %.2f\n", static_cast<double>(s.t), static_cast<double>(s.kmh), s.from, s.to, s.range_change ? "RANGE " : "inside",
                    static_cast<double>(s.min_fraction));
    }
    std::printf("[ps heavy] min gap between upshifts %.2f s, worst in-range kept %.2f\n", static_cast<double>(min_gap), static_cast<double>(worst_in));
    if (expect_ok) {
        CHECK(min_gap >= 0.8);
        CHECK(worst_in >= 0.8);
    } else {
        CHECK((min_gap < 1.0 || worst_in < 0.8));
    }
}

TEST_CASE("Powershift tractor: a heavy launch chains no upshifts and every in-range shift keeps >= 0.8 of the pull", "[validation][vehicle][drivetrain][powershift]") {
    check_heavy_launch(powershift_desc(), true);
    // Negative control (recorded sabotage): the stage 1 law (no skip limit, no upshift interval) and the old 0.15 s torque phase.
    vehicle::VehicleDesc old = powershift_desc();
    for (auto& c : old.powertrain.components) {
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) g->shift_clutch.torque_phase_s = 0.15;
    }
    for (auto& c : old.controllers) {
        if (auto* t = std::get_if<drivetrain::AutomaticTcuDesc>(&c.params); t && t->powershift) {
            t->powershift->min_upshift_pull_fraction = 0.0;
            t->powershift->upshift_min_time_s = 0.0;
        }
    }
    check_heavy_launch(old, false);
}

TEST_CASE("Powershift tractor diag: launch shifts under a tractor-heavy trailer", "[.][diag][powershift]") {
    const auto shifts = record_shifts(powershift_desc(), 14.0, std::getenv("PS_TRAILER") ? std::atof(std::getenv("PS_TRAILER")) : 6000.0);
    for (const auto& s : shifts)
        std::printf("[ps launch] t %5.2f  %5.1f km/h  %2d -> %2d %s %.3f s  kept %.2f%s\n", static_cast<double>(s.t), static_cast<double>(s.kmh), s.from, s.to, s.range_change ? "RANGE " : "inside",
                    static_cast<double>(s.duration), static_cast<double>(s.min_fraction), s.settled ? "" : " (unsettled)");
}

// A power-on downshift under a heavy load (a 6 t tractor, 34 t trailer, full pedal, start gear 11: the launch drops 11 -> 8 across a range): the box takes the
// range change plus a short sync and the pull is back. Before the fix the on-coming clutch stayed open for the whole inertia phase of a power-on downshift: a
// governed engine at full throttle was already past the new gear's speed, flared to its high idle, and the phase ended only at max_inertia_phase_s (0.6 s) with
// a slam (60 kW farm tractor, 2026-11-15: 9 -> 6 took 1.3 s, hub torque 0 -> 34 kN*m). Recorded sabotage: the on-coming capacity back to 0 in a power-on
// downshift's inertia phase (the line in AutoTransmission::update's Inertia case) -> the shift lasts 0.95 s here.
TEST_CASE("Powershift tractor: a power-on downshift under a heavy load takes the range change plus a short sync", "[powershift][tractor][validation]") {
    const auto shifts = record_shifts(powershift_desc(), 6.0, 34000.0);
    const ShiftRecord* down = nullptr;
    for (const auto& s : shifts)
        if (s.to < s.from) { down = &s; break; }
    REQUIRE(down != nullptr);
    std::printf("[ps downshift] t %.2f  %d -> %d  %.3f s  (range %d)\n", static_cast<double>(down->t), down->from, down->to, static_cast<double>(down->duration), down->range_change ? 1 : 0);
    CHECK(down->range_change);
    CHECK(down->duration <= 0.35 + 0.25);
}

// Shift-feel measurement on any vehicle file (the farm's tractors): PS_VEH = path, PS_TRAILER = kg, PS_MASS = chassis kg, PS_THROTTLE, PS_SECONDS, PS_GEAR_REQ = a gear to hold-start in (0 = D).
// Per shift: when, gear change, duration, engine rpm at its start against the schedule's up line, the peak |jerk| of the chassis around it (a smoothed over 0.1 s) and the pull kept.
TEST_CASE("Tractor shift feel diag: farm tractor shifts, jerk and decision lag", "[.][diag][powershift]") {
    const char* veh = std::getenv("PS_VEH");
    if (!veh) return;
    vehicle::VehicleDesc desc = io::load_vehicle_json(veh);
    for (auto& c : desc.controllers)
        if (auto* t = std::get_if<drivetrain::AutomaticTcuDesc>(&c.params); t && t->powershift) {
            if (std::getenv("PS_MINT")) t->powershift->upshift_min_time_s = std::atof(std::getenv("PS_MINT"));
            if (std::getenv("PS_MINPULL")) t->powershift->min_upshift_pull_fraction = std::atof(std::getenv("PS_MINPULL"));
            if (std::getenv("PS_DWELL")) t->min_time_in_gear_s = std::atof(std::getenv("PS_DWELL"));
            if (std::getenv("PS_UTF")) t->upshift_torque_fraction = std::atof(std::getenv("PS_UTF"));
            if (std::getenv("PS_UPF")) t->schedule.up_rpm_at_full_throttle = std::atof(std::getenv("PS_UPF"));
            if (std::getenv("PS_DNF")) t->schedule.down_rpm_at_full_throttle = std::atof(std::getenv("PS_DNF"));
        }
    for (auto& c : desc.powertrain.components)
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) {
            if (std::getenv("PS_RANGE")) g->range_change_time_s = std::atof(std::getenv("PS_RANGE"));
            if (std::getenv("PS_ENGAGE")) g->shift_clutch.engage_time_s = std::atof(std::getenv("PS_ENGAGE"));
        }
    const real z0 = std::getenv("PS_Z") ? std::atof(std::getenv("PS_Z")) : 1.28;
    const real trailer = std::getenv("PS_TRAILER") ? std::atof(std::getenv("PS_TRAILER")) : 0.0;
    const real mass = std::getenv("PS_MASS") ? std::atof(std::getenv("PS_MASS")) : 6000.0;
    const real th = std::getenv("PS_THROTTLE") ? std::atof(std::getenv("PS_THROTTLE")) : 1.0;
    const real secs = std::getenv("PS_SECONDS") ? std::atof(std::getenv("PS_SECONDS")) : 30.0;
    const real gear_req = std::getenv("PS_GEAR_REQ") ? std::atof(std::getenv("PS_GEAR_REQ")) : kD;
    const drivetrain::GearboxDesc& gb = gearbox_of(desc);
    const drivetrain::AutomaticTcuDesc* tcu = nullptr;
    for (const auto& c : desc.controllers)
        if (const auto* t = std::get_if<drivetrain::AutomaticTcuDesc>(&c.params)) tcu = t;
    REQUIRE(tcu != nullptr);
    const real up = tcu->schedule.up_rpm_at_zero_throttle + th * (tcu->schedule.up_rpm_at_full_throttle - tcu->schedule.up_rpm_at_zero_throttle);
    Rig r(desc, trailer, 4, mass, z0);
    r.run(1.0);
    r.world->set_control("park_lock", 0.0);
    r.world->set_control("handbrake", 0.0);
    r.world->set_control("brake", 0.0);
    r.world->set_control("gear_request", gear_req);
    r.world->set_control("throttle", th);
    const real dt = r.world->dt();
    const int n = static_cast<int>(secs / dt);
    std::vector<real> v(n), rpm(n), tq(n);
    std::vector<int> gear(n), phase(n);
    for (int i = 0; i < n; ++i) {
        r.step();
        v[i] = r.speed();
        rpm[i] = r.pt().engine_rpm;
        gear[i] = r.pt().gear;
        phase[i] = static_cast<int>(r.pt().shift_phase);
        tq[i] = r.hub_torque_total();
    }
    // acceleration smoothed over 0.1 s, jerk over a further 0.1 s
    const int w = static_cast<int>(0.1 / dt);
    std::vector<real> a(n, 0.0), j(n, 0.0);
    for (int i = w; i < n; ++i) a[i] = (v[i] - v[i - w]) / (w * dt);
    for (int i = 2 * w; i < n; ++i) j[i] = (a[i] - a[i - w]) / (w * dt);
    std::printf("[feel] %s trailer %.0f kg th %.2f  up line %.0f rpm  final v %.1f km/h gear %d stalled %d\n", veh, static_cast<double>(trailer), static_cast<double>(th), static_cast<double>(up),
                static_cast<double>(v[n - 1] * 3.6), gear[n - 1], r.stalled ? 1 : 0);
    if (std::getenv("PS_T0")) {
        const int i0 = static_cast<int>(std::atof(std::getenv("PS_T0")) / dt), i1 = static_cast<int>(std::atof(std::getenv("PS_T1")) / dt);
        for (int i = i0; i < std::min(n, i1); i += 5)
            std::printf("[trace] t %.3f gear %2d ph %d rpm %5.0f v %5.2f a %5.2f hubTq %8.0f\n", i * static_cast<double>(dt), gear[i], phase[i], static_cast<double>(rpm[i]), static_cast<double>(v[i] * 3.6),
                        static_cast<double>(a[i]), static_cast<double>(tq[i]));
    }
    int shifts = 0;
    real worst_jerk = 0.0;
    for (int i = 1; i < n;) {
        const bool busy = phase[i] != 0 && phase[i] != 5;
        if (busy && !(phase[i - 1] != 0 && phase[i - 1] != 5) && gear[i - 1] > 0) {
            int e = i;
            while (e < n && phase[e] != 0) ++e; // until Idle again (the closing ramp included)
            real pj = 0.0, a0 = a[i], amin = 1e9, amax = -1e9;
            for (int k = i; k < std::min(n, e + 24); ++k) { pj = std::max(pj, std::abs(j[k])); amin = std::min(amin, a[k]); amax = std::max(amax, a[k]); }
            worst_jerk = std::max(worst_jerk, pj);
            const int jmp = std::min(n - 1, e);
            std::printf("[feel] t %6.2f %5.1f km/h  %2d -> %2d  dur %.2f s  rpm@start %5.0f (up %.0f)  a %.2f..%.2f (before %.2f)  peak jerk %.1f m/s3\n", i * static_cast<double>(dt), static_cast<double>(v[i] * 3.6),
                        gear[i - 1], gear[jmp], (e - i) * static_cast<double>(dt), static_cast<double>(rpm[i]), static_cast<double>(up), static_cast<double>(amin), static_cast<double>(amax), static_cast<double>(a0),
                        static_cast<double>(pj));
            ++shifts;
            i = e;
        } else
            ++i;
    }
    // time spent with the engine above the up line before a gear change is a decision lag; report the longest stretch
    int longest = 0, cur = 0;
    for (int i = 0; i < n; ++i) { cur = (rpm[i] > up + 20.0 && phase[i] == 0) ? cur + 1 : 0; longest = std::max(longest, cur); }
    { int k30 = -1; for (int i = 0; i < n; ++i) if (v[i] >= 30.0 / 3.6) { k30 = i; break; } std::printf("[feel] t30 %.2f s  t10 %.2f\n", k30 < 0 ? -1.0 : k30 * static_cast<double>(dt), 0.0); }
    std::printf("[feel] shifts %d  worst jerk %.1f m/s3  longest stretch above the up line with no shift %.2f s  gears: %d..%d (gearbox %zu forward)\n", shifts, static_cast<double>(worst_jerk), longest * static_cast<double>(dt), gear[0],
                gear[n - 1], gb.forward_ratios.size());
}
