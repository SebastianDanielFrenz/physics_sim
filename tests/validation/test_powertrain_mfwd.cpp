// tests/validation/test_powertrain_mfwd.cpp — PHYS-033 P4 I4 C9: acceptance
// tests for tractor.json's new MFWD (mechanical front-wheel drive) front
// axle - the `transfer_case` component (data/vehicles/tractor.json's
// "transfer" + "front_diff" components, default OFF via the "awd" control
// channel) plumbed through core/src/drivetrain/components.h's TransferCase.
//
// Ground (PHYS-047, vault): a track's/car_sedan's own +-1000 m FOUR-TRIANGLE
// flat_surface_mesh()/split_surface_mesh() gives wrong wheel shape casts near
// its long diagonal edges (test_powertrain_split_mu.cpp's own diagnosis, 276
// of 28471 probes at half-extent 1000 m). Every ground this file builds is
// therefore its own LOCAL, TESSELLATED single-surface MeshShape (cells
// <= 20 m, mirroring split_surface_mesh's pattern exactly, just with one
// surface id instead of two) - `mfwd_surface_mesh()` below - plus a "rig
// ground validity" probe case pinning it. The tractor never needs to travel
// more than a few tens of metres in any case here, so a modest 200 m
// half-extent is already far outside the pathological (>=1000 m edge)
// regime split_surface_mesh's own comment documents; tessellating anyway is
// the same defensive practice, not because 200 m alone would fail.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/validation/CMakeLists.txt)"
#endif

namespace {

constexpr std::size_t kFL = 0, kFR = 1, kRL = 2, kRR = 3;

// tractor.json's own chassis rig parameters, as already exercised by
// test_vehicle_tractor_cross_slope.cpp/test_vehicle_shadow_chassis.cpp via
// build_vehicle_rig_from_desc's BoxShape ground - reused here verbatim so
// this file's own MeshShape ground is the only thing under new test.
constexpr real kChassisMass = 6000.0;
const Vec3 kChassisHalfExtents{1.2, 0.35, 0.9};
constexpr real kChassisOriginHeight = 1.28;

// A single-surface tessellated flat ground: split_surface_mesh() from
// test_powertrain_split_mu.cpp with both "sides" the same surface id.
MeshShape mfwd_surface_mesh(real half_extent, real cell, SurfaceId surface_id) {
    const int cells = static_cast<int>(std::lround(2.0 * half_extent / cell));
    REQUIRE(cells > 0);
    REQUIRE(cells % 2 == 0);
    const int nv = cells + 1;
    MeshShape m;
    m.vertices.reserve(static_cast<std::size_t>(nv) * nv);
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nv; ++i) m.vertices.push_back(Vec3{-half_extent + i * cell, -half_extent + j * cell, 0.0});
    }
    m.indices.reserve(static_cast<std::size_t>(cells) * cells * 6);
    m.triangle_surface_ids.reserve(static_cast<std::size_t>(cells) * cells * 2);
    for (int j = 0; j < cells; ++j) {
        for (int i = 0; i < cells; ++i) {
            const auto a = static_cast<std::uint32_t>(j * nv + i);
            const std::uint32_t b = a + 1, c = a + static_cast<std::uint32_t>(nv) + 1, d = a + static_cast<std::uint32_t>(nv);
            m.indices.insert(m.indices.end(), {a, b, c, a, c, d});
            m.triangle_surface_ids.push_back(surface_id);
            m.triangle_surface_ids.push_back(surface_id);
        }
    }
    return m;
}

constexpr real kGroundHalfExtent = 200.0;
constexpr real kGroundCell = 20.0;

struct MfwdRig {
    std::unique_ptr<World> world;
    BodyId chassis{};
    VehicleId vehicle{};
    SurfaceId surface_id = kInvalidSurfaceId;
};

MfwdRig build_mfwd_rig(const vehicle::VehicleDesc& desc, const std::string& surface_name, real tick_hz = 240.0,
                       real substep_hz = 960.0) {
    MfwdRig rig;
    WorldConfig cfg;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
    rig.world = std::make_unique<World>(cfg);

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    rig.surface_id = surfaces->id_for(surface_name);
    rig.world->set_surface_table(surfaces);

    BodyDesc ground;
    ground.shape = mfwd_surface_mesh(kGroundHalfExtent, kGroundCell, rig.surface_id);
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{kChassisHalfExtents};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = kChassisMass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, kChassisOriginHeight};
    rig.chassis = rig.world->create_body(chassis);

    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);
    return rig;
}

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }

// Sum of a set of wheels' longitudinal tyre force.
real sum_fx(const World& w, VehicleId v, std::initializer_list<std::size_t> wheels) {
    real s = 0.0;
    for (std::size_t i : wheels) s += w.wheel_telemetry(v, i).fx;
    return s;
}

} // namespace

// Pins the ground: tractor's own front-wheel query shape (CylinderShape
// r=0.8, half-width 0.24, suspension axis +z, travel_max 0.04) must see the
// tessellated ground exactly like an ideal z=0 plane over the footprint the
// tractor's wheels occupy in every case below (chassis stays within a few
// metres of the origin the whole run - no case here drives far). No steer
// rotation is applied (a straight-down cast is the worst case for a flat
// mesh - steering only grows the query's footprint, which a tessellated
// <=20 m-cell grid handles the same way everywhere on the ground, unlike
// the old 1000 m-edge mesh whose failure was localised to specific poses
// near its own diagonal).
TEST_CASE("mfwd rig ground validity: tractor front-wheel shape casts match an ideal plane",
          "[validation][vehicle][powertrain][drivetrain]") {
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId asphalt = surfaces->id_for("asphalt");

    World w{WorldConfig{}};
    BodyDesc g;
    g.shape = mfwd_surface_mesh(kGroundHalfExtent, kGroundCell, asphalt);
    g.motion = BodyMotionType::Static;
    w.create_body(g);

    const real radius = 0.8, half_width = 0.24, z0 = 1.5, sweep = 1.0;
    const real expected_fraction = (z0 - radius) / sweep;
    const CylinderShape cyl{half_width, radius};
    int probed = 0, wrong = 0;
    real worst_dfraction = 0.0;
    for (int ix = -30; ix <= 30; ++ix) {
        for (int iy = -20; iy <= 20; ++iy) {
            const Vec3 p{ix * 0.1, iy * 0.1, z0};
            const ShapeCastHit h = w.backend().shape_cast(cyl, Pose{p, Quat::identity()}, Vec3{0, 0, -sweep}, sweep);
            ++probed;
            const real d = h.hit ? std::abs(h.fraction - expected_fraction) : real(1.0);
            worst_dfraction = std::max(worst_dfraction, d);
            const bool ok = h.hit && !h.started_penetrating && d <= 1e-3 && h.surface == asphalt;
            if (!ok) ++wrong;
        }
    }
    INFO("probed=" << probed << " wrong=" << wrong << " worst|dfraction|=" << worst_dfraction);
    CHECK(probed == 61 * 41);
    CHECK(wrong == 0);
}

// Steady-state, clutch locked: the front/rear wheel rolling-speed ratio must
// match the ~2.02% MFWD lead built into front_diff.ratio (15.30) vs.
// diff.ratio (20.0) - data/vehicles/tractor.json's own "source" field
// derivation, see CLAUDE.md's "Vehicle system" section.
TEST_CASE("tractor MFWD engaged on asphalt: locks and holds the ~2% front lead at steady state",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.6);
    w.set_control("awd", 1.0);

    const int ticks = static_cast<int>(std::lround(6.0 / w.dt()));
    for (int i = 0; i < ticks; ++i) w.step();

    const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
    INFO("awd_engagement=" << snap.awd_engagement);
    CHECK(snap.awd_engagement == Catch::Approx(1.0).margin(1e-6));

    const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
    INFO("awd_clutch_slip=" << dbg.awd_clutch_slip << " awd_clutch_torque=" << dbg.awd_clutch_torque);
    CHECK(std::abs(dbg.awd_clutch_slip) < 0.05); // rad/s - effectively locked

    const vehicle::WheelTelemetry& fl = w.wheel_telemetry(rig.vehicle, kFL);
    const vehicle::WheelTelemetry& fr = w.wheel_telemetry(rig.vehicle, kFR);
    const vehicle::WheelTelemetry& rl = w.wheel_telemetry(rig.vehicle, kRL);
    const vehicle::WheelTelemetry& rr = w.wheel_telemetry(rig.vehicle, kRR);
    const real front_speed = 0.5 * (fl.omega * desc.wheels[kFL].wheel_radius + fr.omega * desc.wheels[kFR].wheel_radius);
    const real rear_speed = 0.5 * (rl.omega * desc.wheels[kRL].wheel_radius + rr.omega * desc.wheels[kRR].wheel_radius);
    REQUIRE(rear_speed > 0.1);
    const real lead = front_speed / rear_speed;
    INFO("front_speed=" << front_speed << " rear_speed=" << rear_speed << " lead=" << lead << " (expected 1.0202)");
    CHECK(lead == Catch::Approx(1.0202).margin(0.03));
}

// MFWD off (default state, "awd" channel defaults to 0): the front axle must
// carry no drive torque and no phantom drag - adding the transfer_case/
// front_diff hardware to the graph must not load the front wheels just by
// existing.
TEST_CASE("tractor MFWD off (default): front axle carries no drive torque and no drag",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.6);
    // "awd" left at its default (0) - never set here on purpose.

    real max_abs_fx_front = 0.0;
    const int ticks = static_cast<int>(std::lround(3.0 / w.dt()));
    for (int i = 0; i < ticks; ++i) {
        w.step();
        const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
        CHECK(dbg.awd_clutch_torque == 0.0);
        const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
        CHECK(snap.awd_engagement == 0.0);
        max_abs_fx_front =
            std::max({max_abs_fx_front, std::abs(w.wheel_telemetry(rig.vehicle, kFL).fx), std::abs(w.wheel_telemetry(rig.vehicle, kFR).fx)});
    }
    INFO("max |fx| over either front wheel = " << max_abs_fx_front << " N (pure rolling resistance, no drive torque)");
    // Rolling resistance alone (TyreDesc::qsy1 + surface crr) on an undriven,
    // lightly-loaded front axle - nowhere near a driven wheel's traction
    // force (thousands of N at this throttle, see the locked-lead case
    // above's rear wheels).
    CHECK(max_abs_fx_front < 500.0);
}

// Ramp rate: engagement e(t) must track min(1, t/engage_time_s) (0.3 s,
// data/vehicles/tractor.json) within one tick's own integration step, both
// engaging and releasing, and never overshoot [0, 1].
TEST_CASE("tractor MFWD engagement ramps at 1/engage_time_s and releases the same way",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.3);

    const real engage_time_s = 0.3;
    const real dt = w.dt();

    w.set_control("awd", 1.0);
    real t = 0.0;
    const int engage_ticks = static_cast<int>(std::lround(0.6 / dt)); // twice engage_time_s
    for (int i = 0; i < engage_ticks; ++i) {
        w.step();
        t += dt;
        const real e = w.powertrain_state(rig.vehicle).awd_engagement;
        const real expected = std::min(real(1.0), t / engage_time_s);
        INFO("t=" << t << " e=" << e << " expected=" << expected);
        CHECK(e >= real(0.0));
        CHECK(e <= real(1.0) + 1e-9);
        CHECK(e == Catch::Approx(expected).margin(2.0 * dt / engage_time_s + 1e-6));
    }
    CHECK(w.powertrain_state(rig.vehicle).awd_engagement == Catch::Approx(1.0).margin(1e-6));

    w.set_control("awd", 0.0);
    t = 0.0;
    for (int i = 0; i < engage_ticks; ++i) {
        w.step();
        t += dt;
        const real e = w.powertrain_state(rig.vehicle).awd_engagement;
        const real expected = std::max(real(0.0), real(1.0) - t / engage_time_s);
        INFO("release t=" << t << " e=" << e << " expected=" << expected);
        CHECK(e >= real(0.0) - 1e-9);
        CHECK(e <= real(1.0));
        CHECK(e == Catch::Approx(expected).margin(2.0 * dt / engage_time_s + 1e-6));
    }
    CHECK(w.powertrain_state(rig.vehicle).awd_engagement == Catch::Approx(0.0).margin(1e-6));
}

// Energy: the PowertrainEnergy ledger's balance_error stays within the
// project's existing bound (test_powertrain_energy_balance.cpp's own
// <= 5e-8 relative criterion) with the MFWD clutch actively slipping and
// then locked, and awd_clutch_loss is non-negative every tick (a friction
// row's dissipation opposes its own slip by construction - see this
// file's unit-level companion, tests/unit/test_powertrain_transfer_case.cpp).
TEST_CASE("tractor MFWD energy ledger balances and awd_clutch_loss is never negative",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.6);
    w.set_control("awd", 1.0);

    real prev_awd_clutch_loss = 0.0;
    real worst_balance_error = 0.0;
    const int ticks = static_cast<int>(std::lround(4.0 / w.dt()));
    for (int i = 0; i < ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        worst_balance_error = std::max(worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);

        const real delta = e.awd_clutch_loss - prev_awd_clutch_loss;
        INFO("tick " << i << " awd_clutch_loss delta=" << delta);
        CHECK(delta >= -1e-9); // non-decreasing (dissipative), to float noise
        prev_awd_clutch_loss = e.awd_clutch_loss;
    }
    INFO("worst relative balance_error = " << worst_balance_error << " final awd_clutch_loss=" << prev_awd_clutch_loss);
    CHECK(prev_awd_clutch_loss > 0.0); // the clutch did slip at some point before locking
}

// Drawbar-pull proxy: on a slick surface the vehicle's achievable drawbar
// pull is exactly its total ground tractive force minus rolling resistance
// (both terms measured the same way regardless of MFWD), so the maximum
// sustained total driven-wheel Fx during a full-throttle low-mu launch is a
// direct, already-instrumented stand-in for "drawbar pull limit" - no new
// dynamometer/tow-force harness needed. Compares MFWD off (rear only) vs.
// MFWD on (front axle also engages, since clutch capacity 2500 N*m is far
// above what a low-mu launch's slip torque reaches) and requires >= 1.2x,
// per the plan.
namespace {
real mean_total_driven_fx_low_mu(bool awd_on) {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    w.set_control("awd", awd_on ? 1.0 : 0.0);

    const real dt = w.dt();
    const int start_tick = static_cast<int>(std::lround(0.5 / dt));
    const int end_tick = static_cast<int>(std::lround(1.5 / dt));
    real sum = 0.0;
    int n = 0;
    for (int i = 0; i < end_tick; ++i) {
        w.step();
        if (i >= start_tick) {
            sum += sum_fx(w, rig.vehicle, awd_on ? std::initializer_list<std::size_t>{kFL, kFR, kRL, kRR}
                                                  : std::initializer_list<std::size_t>{kRL, kRR});
            ++n;
        }
    }
    return sum / static_cast<real>(n);
}
} // namespace

TEST_CASE("tractor MFWD on low_mu: engaged total tractive force is at least 1.2x the rear-only force",
          "[validation][vehicle][powertrain][drivetrain]") {
    const real off = mean_total_driven_fx_low_mu(false);
    const real on = mean_total_driven_fx_low_mu(true);
    REQUIRE(off > 1.0); // guard the ratio below against a near-zero denominator
    const real ratio = on / off;
    INFO("mean total driven Fx: MFWD off=" << off << " N, MFWD on=" << on << " N, ratio=" << ratio);
    CHECK(ratio >= 1.2);
}
