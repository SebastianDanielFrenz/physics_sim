// tests/validation/test_hydro_cvt_tractor.cpp - tractor plan stage 2: the hydro-mechanical CVT
// (data/gearboxes/tractor_hydro_cvt.json, fixture tests/support/fixtures/vehicles/tractor_hydro.json) on
// data/vehicles/tractor.json's body, driven through World like the game.
//
//  * D on a 15 % slope with no brake and no throttle: ratio 0 holds the output rigid (active standstill), drift < 1 cm over
//    20 s, the engine keeps running; neutral on the same slope rolls away (negative control);
//  * 0-40 km/h with a 6 t trailer: no loss of speed on the way (no shift exists), acceleration and jerk bounded, the
//    engine stays in its band;
//  * through zero into reverse and back to forward;
//  * the per-tick energy ledger closes, 1 vs 10 workers give the same state hash every tick, own allocation in jobs 0.
// Every case prints its measured numbers.
// Recorded sabotage (2026-11-02): the row capacity at 2 % of the pressure limit while a direction is engaged -> the slope
// case drifts 187 m in 20 s (and the 0-40 / reverse cases fail), 3 of 6 cases fail.

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
#include <memory>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kD = 99.0;
constexpr real k40KmH = 40.0 / 3.6;


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

    Rig(const vehicle::VehicleDesc& tractor, real trailer_mass, int workers) {
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
        ba.shape = BoxShape{Vec3{1.2, 0.35, 0.9}};
        ba.mass = 6000.0;
        ba.friction = 0.5;
        ba.pose.position = Vec3{0.0, 0.0, 1.28};
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
            bb.pose.position = Vec3{sa.x - sb.x, 0.0, 1.1};
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
    void drive(real throttle, real gear = kD) {
        world->set_control("gear_request", gear);
        world->set_control("throttle", throttle);
    }
    // The manual box with the farm game's auto-clutch + auto-shift assist.
    void drive_manual(real throttle) {
        world->set_control("assist.auto_shift", 1.0);
        world->set_control("assist.auto_clutch", 1.0);
        world->set_control("throttle", throttle);
    }
};

vehicle::VehicleDesc hydro_desc() { return io::load_vehicle_json(fixture("tractor_hydro.json")); }

struct Run40 {
    real t40 = -1.0;
    real worst_drop = 0.0;  // largest speed lost against the running maximum [m/s]
    real max_accel = 0.0;   // largest 0.25 s mean acceleration [m/s^2]
    real max_jerk = 0.0;    // largest change of that acceleration per 0.25 s window pair [m/s^3]
    real min_rpm = 1e9, max_rpm = 0.0; // engine rpm after the first second of motion
    real balance = 0.0;
    bool stalled = false;
};

Run40 run_to_40(Rig& r, real command, real max_s) {
    Run40 o;
    std::vector<real> v;
    real vmax = 0.0;
    const int n = static_cast<int>(max_s / r.world->dt());
    for (int i = 0; i < n; ++i) {
        r.step();
        const real t = static_cast<real>(i + 1) * r.world->dt();
        const real s = r.speed();
        v.push_back(s);
        vmax = std::max(vmax, s);
        o.worst_drop = std::max(o.worst_drop, vmax - s);
        if (t > 2.0 && s < k40KmH) {
            o.min_rpm = std::min(o.min_rpm, r.pt().engine_rpm);
            o.max_rpm = std::max(o.max_rpm, r.pt().engine_rpm);
        }
        if (s >= k40KmH) {
            o.t40 = t;
            break;
        }
    }
    (void)command;
    const int w = 60; // 0.25 s
    std::vector<real> a;
    for (std::size_t i = static_cast<std::size_t>(w); i < v.size(); i += 6) a.push_back((v[i] - v[i - static_cast<std::size_t>(w)]) / (w * r.world->dt()));
    for (std::size_t i = 0; i < a.size(); ++i) {
        o.max_accel = std::max(o.max_accel, a[i]);
        if (i > 0) o.max_jerk = std::max(o.max_jerk, std::abs(a[i] - a[i - 1]) / (6 * r.world->dt()));
    }
    o.stalled = r.stalled;
    o.balance = r.max_balance;
    return o;
}
} // namespace

TEST_CASE("Hydro CVT tractor: holds a 15 % slope at 0 km/h without brakes", "[validation][vehicle][drivetrain][hydro_cvt]") {
    const vehicle::VehicleDesc desc = hydro_desc();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.15, 4);
    rig.world->set_control("awd", 1.0);
    rig.world->set_control("gear_request", kD);
    rig.world->set_control("throttle", 0.0);
    rig.world->set_control("brake", 0.0);
    rig.world->set_control("handbrake", 0.0);
    ps_test::settle(*rig.world, 3.0);
    const Pose p0 = rig.world->get_pose(rig.chassis);
    real max_balance = 0.0;
    bool stalled = false;
    const int n = static_cast<int>(20.0 / rig.world->dt() + 0.5);
    for (int i = 0; i < n; ++i) {
        rig.world->step();
        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) max_balance = std::max(max_balance, std::abs(e.balance_error) / denom);
        if (rig.world->powertrain_state(rig.vehicle).engine_state != drivetrain::EngineState::Running) stalled = true;
    }
    const Pose p1 = rig.world->get_pose(rig.chassis);
    const real drift = (p1.position - p0.position).length();
    const auto& st = rig.world->powertrain_state(rig.vehicle);
    std::printf("[hydro slope] D, 15 %% slope, no brake, 20 s: drift %.5f m, ratio k %.5f, engine %.0f rpm, kind %d, stalled %d, balance %.2e\n",
                static_cast<double>(drift), static_cast<double>(st.cvt_ratio), static_cast<double>(st.engine_rpm), st.transmission_kind, stalled ? 1 : 0,
                static_cast<double>(max_balance));
    CHECK(st.transmission_kind == 6);
    CHECK(drift < 0.01);
    CHECK_FALSE(stalled);
    CHECK(max_balance <= 5e-8);

    // Negative control: neutral on the same slope rolls away.
    auto roll = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.15, 4);
    roll.world->set_control("awd", 1.0);
    roll.world->set_control("gear_request", 0.0);
    roll.world->set_control("throttle", 0.0);
    ps_test::settle(*roll.world, 3.0);
    const Pose q0 = roll.world->get_pose(roll.chassis);
    const int m = static_cast<int>(10.0 / roll.world->dt() + 0.5);
    for (int i = 0; i < m; ++i) roll.world->step();
    const real rolled = (roll.world->get_pose(roll.chassis).position - q0.position).length();
    std::printf("[hydro slope] neutral negative control, 10 s: rolled %.2f m\n", static_cast<double>(rolled));
    CHECK(rolled > 1.0);
}

TEST_CASE("Hydro CVT tractor: 0-40 km/h with a 6 t trailer is smooth", "[validation][vehicle][drivetrain][hydro_cvt]") {
    Rig r(hydro_desc(), 6000.0, 4);
    r.run(1.0);
    r.drive(0.8);
    const Run40 o = run_to_40(r, 0.8, 40.0);
    std::printf("[hydro 0-40] t40 %.2f s, worst speed drop %.4f m/s, max accel %.2f m/s2, max jerk %.2f m/s3, engine %.0f-%.0f rpm, stalled %d, balance %.2e\n",
                static_cast<double>(o.t40), static_cast<double>(o.worst_drop), static_cast<double>(o.max_accel), static_cast<double>(o.max_jerk),
                static_cast<double>(o.min_rpm), static_cast<double>(o.max_rpm), o.stalled ? 1 : 0, static_cast<double>(o.balance));
    REQUIRE(o.t40 > 0.0);
    CHECK(o.t40 < 30.0);
    CHECK(o.worst_drop <= 0.05);   // no loss of speed at all on the way: there is no shift
    CHECK(o.max_accel <= 1.3);     // the data ramp is 0.95 m/s^2 (measured 1.15 over a 0.25 s window)
    CHECK(o.max_jerk <= 4.0);
    CHECK(o.min_rpm >= 900.0);
    CHECK(o.max_rpm <= 2300.0);
    CHECK_FALSE(o.stalled);
    CHECK(o.balance <= 5e-8);
}

TEST_CASE("Hydro CVT tractor: through zero into reverse and back", "[validation][vehicle][drivetrain][hydro_cvt]") {
    Rig r(hydro_desc(), 0.0, 4);
    r.run(1.0);
    r.drive(0.5, -1.0);
    r.run(8.0);
    const real v_rev = r.speed();
    std::printf("[hydro reverse] R at 50 %% command, 8 s: speed %.2f m/s, k %.3f, engine %.0f rpm\n", static_cast<double>(v_rev), static_cast<double>(r.pt().cvt_ratio),
                static_cast<double>(r.pt().engine_rpm));
    CHECK(v_rev < -1.0);
    CHECK(r.pt().cvt_ratio < 0.0);
    r.drive(0.0, -1.0);
    r.run(8.0);
    CHECK(std::abs(r.speed()) < 0.3);
    r.drive(0.5, kD);
    r.run(10.0);
    std::printf("[hydro reverse] D again, 10 s: speed %.2f m/s, k %.3f\n", static_cast<double>(r.speed()), static_cast<double>(r.pt().cvt_ratio));
    CHECK(r.speed() > 1.0);
    CHECK(r.pt().cvt_ratio > 0.0);
    CHECK_FALSE(r.stalled);
    CHECK(r.max_balance <= 5e-8);
}

TEST_CASE("Hydro CVT tractor: 1 vs 10 workers give the same state hash every tick", "[validation][vehicle][drivetrain][hydro_cvt][determinism]") {
    std::vector<std::uint64_t> h1, h10;
    for (int workers : {1, 10}) {
        Rig r(hydro_desc(), 6000.0, workers);
        r.run(0.5);
        r.drive(0.8);
        std::vector<std::uint64_t>& out = workers == 1 ? h1 : h10;
        for (int i = 0; i < 240 * 8; ++i) {
            r.step();
            out.push_back(r.world->state_hash());
        }
    }
    REQUIRE(h1.size() == h10.size());
    std::size_t bad = 0;
    for (std::size_t i = 0; i < h1.size(); ++i) bad += h1[i] != h10[i] ? 1 : 0;
    std::printf("[hydro determinism] %zu ticks, %zu hash differences, last hash 0x%llx\n", h1.size(), bad, static_cast<unsigned long long>(h1.back()));
    CHECK(bad == 0);
}

TEST_CASE("Hydro CVT tractor: a world step allocates nothing in jobs", "[validation][vehicle][drivetrain][hydro_cvt][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hooks are compiled out under ASan/TSan");
        return;
    }
    Rig r(hydro_desc(), 0.0, 4);
    r.run(0.5);
    r.drive(0.8);
    r.run(1.0); // warm up
    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    r.run(6.0); // a launch and the climb up the ratio, a brake at the end
    r.world->set_control("brake", 1.0);
    r.run(1.0);
    jobs::debug_alloc::set_armed(false);
    std::printf("[hydro alloc] own %llu, external %llu\n", static_cast<unsigned long long>(jobs::debug_alloc::own_violation_count()),
                static_cast<unsigned long long>(jobs::debug_alloc::external_violation_count()));
    CHECK(jobs::debug_alloc::own_violation_count() == 0);
}

TEST_CASE("Hydro CVT tractor diag: the 0-40 trace", "[.][diag][hydro_cvt]") {
    Rig r(hydro_desc(), 6000.0, 4);
    r.run(1.0);
    r.drive(0.8);
    for (int i = 0; i < 240 * 40; ++i) {
        r.step();
        if (i % 60 == 0) {
            const auto d = r.world->powertrain_debug(r.va);
            std::printf("t %.2f v %.2f rpm %.0f k %.3f thr_eff %.2f demand %.0f out_rpm %.0f hub %.0f\n", static_cast<double>(i / 240.0), static_cast<double>(r.speed()),
                        static_cast<double>(r.pt().engine_rpm), static_cast<double>(r.pt().cvt_ratio), static_cast<double>(d.throttle_eff),
                        static_cast<double>(d.engine_torque_demand), static_cast<double>(r.pt().driveline.gearbox_out_omega * 60.0 / 6.2831853),
                        static_cast<double>(r.hub_torque_total()));
        }
    }
}
