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
// NOTE (2026-09-26): tests/support/vehicle_test_helpers.h's own
// flat_surface_mesh() was independently tessellated on master after this
// file's branch point - this file's local mfwd_surface_mesh() is kept as-is
// (no need to switch over; both are now equally valid).
//
// C9 REVIEW ROUND (4th commit, 2026-09-26): a coordinator review found two
// of the six cases below methodologically inadequate even though they
// passed cleanly, and asked for two additions. Both replacements/additions
// are marked "REVIEW C9" at their TEST_CASE:
//  - the steady-state lead check (was: front/rear wheel ROLLING-SPEED ratio
//    vs. lead - a pure kinematic identity of the locked transfer-case row
//    plus the open differentials' own velocity constraint, true by
//    construction regardless of the tyre model, so it cannot fail) is
//    replaced by a check on the ACTUAL tyre-model slip ratios
//    (WheelTelemetry::slip_ratio) at a genuine driven steady state in gear
//    6 at ~3 m/s: mean(kappa_F) - mean(kappa_R) == lead - 1 (see that
//    TEST_CASE's own comment for the derivation of why this is NOT the same
//    tautology - it depends on the real per-wheel contact-patch speeds and
//    the tyre model's own slip-ratio definition, not just wheel omega and a
//    static radius).
//  - the drawbar-pull proxy (was: mean total driven Fx during a WOT low-mu
//    LAUNCH, a transient-dependent quantity) is replaced by an actual limit
//    search: a rearward horizontal IForceElement on the chassis, ramped
//    slowly at a fixed gear/throttle, reporting the highest sustained force
//    before the tractor stalls or its driven wheels lose useful traction
//    (criterion defined in that TEST_CASE's own header comment).
//  - two additions: a 1-vs-10-worker state_hash equality case with "awd"
//    switched on then off while driving (tagged [determinism]), and a
//    debug_alloc-armed zero-allocation check toggling "awd" while stepping
//    a full World (run against the debug preset).

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/force_element.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
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
                       real substep_hz = 960.0, int job_workers = 0) {
    MfwdRig rig;
    WorldConfig cfg;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
    cfg.job_workers = job_workers; // 0 = auto, matches every pre-existing caller exactly
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

// REVIEW C9: the front-axle "lead" ratio, computed ENTIRELY from the loaded
// VehicleDesc's own public fields (never a hardcoded numeric literal) -
// mirrors test_powertrain_split_mu.cpp's own diff_desc() helper pattern.
// lead = (r_front * diff_ratio_rear) / (r_rear * diff_ratio_front): with the
// transfer-case clutch locked (rear_ratio == front_ratio == 1.0 for
// tractor.json, so the front and rear differential CARRIERS spin at exactly
// the same speed), each open differential's own velocity constraint gives
// hub_avg = carrier / diff_ratio exactly, so
//   front_rolling_speed / rear_rolling_speed
//     = (r_front * hub_avg_front) / (r_rear * hub_avg_rear)
//     = (r_front / r_rear) * (diff_ratio_rear / diff_ratio_front) = lead.
drivetrain::DifferentialDesc diff_desc(const vehicle::VehicleDesc& d, const std::string& id) {
    for (const auto& c : d.powertrain.components) {
        if (c.id == id) return std::get<drivetrain::DifferentialDesc>(c.params);
    }
    throw std::runtime_error("diff_desc: component not found: " + id);
}

real mfwd_lead(const vehicle::VehicleDesc& d) {
    const real diff_ratio_rear = diff_desc(d, "diff").ratio;
    const real diff_ratio_front = diff_desc(d, "front_diff").ratio;
    const real r_front = 0.5 * (d.wheels[kFL].wheel_radius + d.wheels[kFR].wheel_radius);
    const real r_rear = 0.5 * (d.wheels[kRL].wheel_radius + d.wheels[kRR].wheel_radius);
    return (r_front * diff_ratio_rear) / (r_rear * diff_ratio_front);
}

// REVIEW C9 (item 2): a rearward horizontal force at the chassis CoM, whose
// magnitude is set from the test's own control loop before each step() -
// exactly TangentialSpeedForce's own "write before step(), read during
// step()" contract (test_vehicle_skidpad.cpp), just pointed the other way
// and driven open-loop by a slow ramp instead of closed-loop by a speed
// error.
class RearwardForce final : public IForceElement {
public:
    explicit RearwardForce(Frame frame) : frame_(frame) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        (void)ctx;
        return Wrench{Vec3{-magnitude_, 0.0, 0.0}, Vec3::zero()};
    }
    void set_magnitude(real m) { magnitude_ = m; }
    [[nodiscard]] real magnitude() const { return magnitude_; }

private:
    Frame frame_;
    real magnitude_ = 0.0;
};

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

// REVIEW C9 (item 1): steady-state at ~3 m/s in gear 6, MFWD engaged and
// locked - checks the ACTUAL tyre-model slip ratios, not wheel omega/radius
// alone (see the file header comment and mfwd_lead()'s own comment for why
// this is a materially different, non-tautological check from the rejected
// wheel-rolling-speed-ratio version). Held steady by a closed-loop throttle
// PI (not a free-rolling force element - the whole point is that the
// driveline must be under REAL torque, so a wheel's slip ratio means
// something, unlike test_vehicle_skidpad.cpp's free-rolling TangentialSpeedForce
// use case).
//
// Derivation: with the transfer-case clutch stuck (rear_ratio ==
// front_ratio == 1.0), the front/rear differential carriers spin at exactly
// the same speed, so each axle's rolling speed (an open diff's own velocity
// constraint) is EXACTLY hub_avg*r = (carrier/diff_ratio)*r - independent of
// tyre slip. So front_rolling_speed / rear_rolling_speed == lead EXACTLY,
// deterministically, whatever the tyre model does (this is precisely why
// the OLD wheel-speed-ratio test could not fail - it was reading this
// identity back out, not checking anything about the tyre model). This
// TEST_CASE instead reads WheelTelemetry::slip_ratio - kappa_i = (rolling
// speed_i - V_i)/V_i, where V_i is wheel i's OWN measured contact-patch
// forward speed, not assumed equal front-to-rear. Combining the two:
//   (1+kappa_F)*V_F / ((1+kappa_R)*V_R) == lead
// At a genuine steady state, straight ahead, V_F ~= V_R (chassis forward
// speed at both axles, to within pitch/suspension-travel differences), so
// kappa_F - kappa_R ~= (lead-1)*(1+kappa_R) ~= (lead-1) plus a small
// correction of order (lead-1)*kappa_R (a few 1e-4 for kappa_R of a few
// percent) - comfortably inside the +-0.003 margin below. Unlike the old
// check, this exercises the REAL slip-ratio code path (WheelInstance's
// actual dynamic radius/contact-patch-speed measurement), so a bug in that
// wiring (wrong wheel index, wrong radius definition, a front/rear swap)
// would show up here even though it could never show up in the old
// wheel-omega-ratio check.
TEST_CASE("tractor MFWD engaged in gear 6 at ~3 m/s: measured slip-ratio lead matches the loaded VehicleDesc's "
          "diff ratios/tyre radii",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();
    const real lead = mfwd_lead(desc);
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 6.0);
    w.set_control("awd", 1.0);

    constexpr real kTargetSpeed = 3.0; // m/s
    constexpr real kp = 0.08, ki = 0.05;
    constexpr real kThrottleMax = 1.0;
    real integral = 0.0;
    const real dt = w.dt();

    // Closed-loop throttle to a held target speed - drives the real
    // drivetrain (not a force-element bypass), so both axles carry genuine
    // torque and their tyre slip ratios mean something.
    auto step_with_speed_control = [&]() {
        const real speed = w.get_motion(rig.chassis).linear.x;
        const real err = kTargetSpeed - speed;
        integral = std::clamp(integral + err * dt, -20.0, 20.0);
        const real throttle = std::clamp(kp * err + ki * integral, 0.0, kThrottleMax);
        w.set_control("throttle", throttle);
        w.step();
    };

    const int settle_ticks = static_cast<int>(std::lround(25.0 / dt));
    for (int i = 0; i < settle_ticks; ++i) step_with_speed_control();

    // Measurement window: mean slip ratios, mean fx, clutch-stuck check and
    // awd_clutch_loss rate, all over the same trailing window.
    constexpr real kWindowSeconds = 3.0;
    const int window_ticks = static_cast<int>(std::lround(kWindowSeconds / dt));
    real sum_kappa_f = 0.0, sum_kappa_r = 0.0, sum_fx_f = 0.0, sum_fx_r = 0.0, sum_dv = 0.0;
    real max_abs_clutch_slip = 0.0;
    real loss_at_start = w.powertrain_energy(rig.vehicle).awd_clutch_loss;
    real prev_speed = w.get_motion(rig.chassis).linear.x;
    for (int i = 0; i < window_ticks; ++i) {
        step_with_speed_control();
        const vehicle::WheelTelemetry& fl = w.wheel_telemetry(rig.vehicle, kFL);
        const vehicle::WheelTelemetry& fr = w.wheel_telemetry(rig.vehicle, kFR);
        const vehicle::WheelTelemetry& rl = w.wheel_telemetry(rig.vehicle, kRL);
        const vehicle::WheelTelemetry& rr = w.wheel_telemetry(rig.vehicle, kRR);
        sum_kappa_f += 0.5 * (fl.slip_ratio + fr.slip_ratio);
        sum_kappa_r += 0.5 * (rl.slip_ratio + rr.slip_ratio);
        sum_fx_f += fl.fx + fr.fx;
        sum_fx_r += rl.fx + rr.fx;
        const real speed = w.get_motion(rig.chassis).linear.x;
        sum_dv += (speed - prev_speed);
        prev_speed = speed;
        const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
        max_abs_clutch_slip = std::max(max_abs_clutch_slip, std::abs(dbg.awd_clutch_slip));
    }
    const real n = static_cast<real>(window_ticks);
    const real mean_kappa_f = sum_kappa_f / n;
    const real mean_kappa_r = sum_kappa_r / n;
    const real mean_fx_f = sum_fx_f / n;
    const real mean_fx_r = sum_fx_r / n;
    const real mean_dv_dt = (sum_dv / n) / dt;
    const real loss_rate = (w.powertrain_energy(rig.vehicle).awd_clutch_loss - loss_at_start) / kWindowSeconds;

    INFO("lead (from VehicleDesc)=" << lead << " lead-1=" << (lead - 1.0));
    INFO("mean_kappa_f=" << mean_kappa_f << " mean_kappa_r=" << mean_kappa_r
                          << " kappa_f-kappa_r=" << (mean_kappa_f - mean_kappa_r));
    INFO("mean_fx_f=" << mean_fx_f << " mean_fx_r=" << mean_fx_r
                       << " rear fx share=" << (mean_fx_r / (mean_fx_f + mean_fx_r)));
    INFO("mean_dv_dt=" << mean_dv_dt << " (steadiness check) max_abs_clutch_slip=" << max_abs_clutch_slip
                        << " awd_clutch_loss rate=" << loss_rate << " W");

    // Steadiness/lock gates - report, don't silently pass a bad operating
    // point.
    CHECK(std::abs(mean_dv_dt) < 0.05); // m/s^2 - genuinely steady, not still accelerating
    CHECK(max_abs_clutch_slip < 0.05);  // rad/s - the row is stuck throughout the window

    CHECK(mean_fx_f > 0.0);                                          // front axle IS pulling
    CHECK((mean_kappa_f - mean_kappa_r) == Catch::Approx(lead - 1.0).margin(0.003));
    CHECK(loss_rate < 1.0); // W - ~0 dissipation while the clutch is stuck (a locked row carries torque, not slip)
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

// REVIEW C9 (item 2): drawbar-pull LIMIT, not a WOT-launch mean Fx (a
// transient-dependent quantity - how far into its own launch transient each
// config gets in a fixed 1s window depends on config-specific dynamics, not
// just its steady-state limit). This ramps a rearward horizontal force on
// the chassis (RearwardForce above) slowly at a FIXED gear/throttle and
// finds the highest force the tractor still sustains.
//
// CRITERION (the thing this test actually measures): at each instant, judge
// the tractor "still pulling" iff BOTH, over a trailing 0.5 s window:
//   (a) mean forward speed (chassis local +x) > kMinProgressSpeed (0.05 m/s)
//       - it has not been dragged to a standstill/stall by the load;
//   (b) mean |slip_ratio| over every DRIVEN wheel (World::wheel_is_driven,
//       so this automatically covers 2 wheels off / 4 wheels on) stays
//       below kMaxDrivenSlip (0.6) - the driven tyres are doing useful work,
//       not just spinning in place (well inside the tyre model's own +-3
//       relaxation safety clamp, which is a numerical bound, not a traction
//       definition).
// The reported "limit" is the ramp force at the first sample where either
// gate fails, i.e. the highest force sustained through both gates.
namespace {
class RampCriterionState {
public:
    void reset(std::size_t window_n) { window_n_ = window_n; }
    void push(real speed, real driven_slip) {
        speed_.push_back(speed);
        slip_.push_back(driven_slip);
        if (speed_.size() > window_n_) speed_.pop_front();
        if (slip_.size() > window_n_) slip_.pop_front();
    }
    [[nodiscard]] bool full() const { return speed_.size() >= window_n_; }
    [[nodiscard]] real mean_speed() const { return mean_of_deque(speed_); }
    [[nodiscard]] real mean_slip() const { return mean_of_deque(slip_); }

private:
    static real mean_of_deque(const std::deque<real>& d) {
        real s = 0.0;
        for (real v : d) s += v;
        return s / static_cast<real>(d.size());
    }
    std::deque<real> speed_, slip_;
    std::size_t window_n_ = 1;
};

// Returns {limit_force, final_speed} - final_speed is reported for
// diagnostics (e.g. to see whether the run ended by stalling vs. by hitting
// kMaxForce without ever failing the gates).
std::pair<real, real> drawbar_pull_limit(bool awd_on) {
    constexpr real kMinProgressSpeed = 0.05; // m/s
    constexpr real kMaxDrivenSlip = 0.6;     // dimensionless (|kappa|)
    constexpr real kWindowSeconds = 0.5;
    constexpr real kWarmupSeconds = 2.0; // let it start moving before the ramp's gates are evaluated
    constexpr real kRampRatePerSecond = 100.0; // N/s - slow relative to the vehicle's own response time
    constexpr real kMaxForce = 8000.0;         // N - generous headroom over the measured off/on limits

    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0); // fixed gear/throttle for the whole ramp, per the brief
    w.set_control("throttle", 1.0);
    w.set_control("awd", awd_on ? 1.0 : 0.0);

    RearwardForce rf(Frame(rig.chassis));
    w.add_force_element(&rf);

    const real dt = w.dt();
    const std::size_t window_n = static_cast<std::size_t>(std::lround(kWindowSeconds / dt));
    const int warmup_ticks = static_cast<int>(std::lround(kWarmupSeconds / dt));

    RampCriterionState state;
    state.reset(window_n);
    real force = 0.0;
    real limit = 0.0;
    real last_speed = 0.0;
    int tick = 0;
    while (force < kMaxForce) {
        force += kRampRatePerSecond * dt;
        rf.set_magnitude(force);
        w.step();
        ++tick;

        const std::vector<std::size_t> wheel_indices{kFL, kFR, kRL, kRR};
        real slip_sum = 0.0;
        int driven_count = 0;
        for (std::size_t i : wheel_indices) {
            if (!w.wheel_is_driven(rig.vehicle, i)) continue;
            slip_sum += std::abs(w.wheel_telemetry(rig.vehicle, i).slip_ratio);
            ++driven_count;
        }
        const real driven_slip = driven_count > 0 ? slip_sum / static_cast<real>(driven_count) : 0.0;
        const real speed = w.get_motion(rig.chassis).linear.x;
        last_speed = speed;
        state.push(speed, driven_slip);

        if (tick > warmup_ticks && state.full()) {
            if (state.mean_speed() < kMinProgressSpeed || state.mean_slip() > kMaxDrivenSlip) {
                limit = force - kRampRatePerSecond * kWindowSeconds; // force at the start of the failing window
                break;
            }
        }
        limit = force;
    }
    return {limit, last_speed};
}
} // namespace

TEST_CASE("tractor MFWD on low_mu: engaged drawbar-pull limit is at least 1.2x the rear-only limit",
          "[validation][vehicle][powertrain][drivetrain]") {
    const auto [off, off_final_speed] = drawbar_pull_limit(false);
    const auto [on, on_final_speed] = drawbar_pull_limit(true);
    REQUIRE(off > 1.0); // guard the ratio below against a near-zero denominator
    const real ratio = on / off;
    INFO("drawbar-pull limit: MFWD off=" << off << " N (final speed " << off_final_speed << " m/s), MFWD on=" << on
                                          << " N (final speed " << on_final_speed << " m/s), ratio=" << ratio);
    // Compare against the earlier (rejected) WOT-launch-mean-Fx proxy's 1.51
    // ratio (off=2733.96 N, on=4132.64 N) once this test's own numbers are
    // known - see the 4th commit's report for the measured values and, if
    // they differ substantially, the explanation (a genuine limit search
    // reaches each config's own true traction ceiling rather than sampling
    // a fixed-duration transient, so the two numbers are not expected to
    // agree exactly).
    CHECK(ratio >= 1.2);
}

// REVIEW C9 (item 3): 1-vs-10-worker state_hash equality with "awd" switched
// on mid-run then off again while driving - mirrors test_powertrain_diff.cpp's
// own "clutch_lsd/torsen vehicle state hash is worker-count invariant (1 vs
// 10 workers)" pattern (a lightweight in-file check, not
// tests/determinism/test_worker_count_hash.cpp's own heavier 1/2/4/10/24
// sweep - the transfer_case's own state (engagement e, the friction row's
// stuck/LCP set) is folded into World::state_hash() exactly like any other
// drivetrain component's state, so this is the same class of check).
// Tagged [determinism] so it runs in the determinism CI topic.
TEST_CASE("tractor MFWD state hash is worker-count invariant with awd switched on then off while driving "
          "(1 vs 10 workers)",
          "[validation][vehicle][powertrain][drivetrain][determinism]") {
    auto run = [](int workers) {
        const vehicle::VehicleDesc desc = load_tractor();
        MfwdRig rig = build_mfwd_rig(desc, "asphalt", 240.0, 960.0, workers);
        World& w = *rig.world;
        ps_test::settle(w, 1.0);
        w.set_control("gear_request", 3.0);
        w.set_control("throttle", 0.6);
        w.set_control("steer", 0.1);
        for (int i = 0; i < 240; ++i) w.step(); // driving, awd at its default (0)
        w.set_control("awd", 1.0);
        for (int i = 0; i < 240; ++i) w.step(); // awd engages while driving
        w.set_control("awd", 0.0);
        for (int i = 0; i < 240; ++i) w.step(); // awd releases while still driving
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    INFO("h1=0x" << std::hex << h1 << " h10=0x" << h10);
    CHECK(h1 == h10);
}

// REVIEW C9 (item 4): debug_alloc-armed zero-allocation check while "awd"
// toggles on and off during driving - follows test_powertrain_topology.cpp's/
// test_powertrain_diff.cpp's own "allocates nothing while toggling a control
// channel" pattern exactly (warm up unarmed, arm, step while toggling,
// disarm, assert own_violation_count() == 0, report the pre-existing
// Jolt-internal external count as a WARN only). Meant to be run against the
// debug preset (`ctest --preset debug -R ...` or the exe directly) - it is
// registered on ps_validation_tests, which the default fast CI's debug leg
// does not auto-run, so this is a manual/explicit invocation, same as any
// other targeted debug-preset check.
TEST_CASE("tractor MFWD allocates nothing while awd toggles on and off during driving",
          "[validation][vehicle][powertrain][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check "
             "here on this build.");
        return;
    }

    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 3.0);
    w.set_control("throttle", 0.6);
    w.set_control("steer", 0.1);
    for (int i = 0; i < 300; ++i) w.step(); // warm up every scratch buffer (transfer case included) before arming

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 240; ++i) {
        w.set_control("awd", (i / 30) % 2 == 0 ? 1.0 : 0.0); // toggle every 30 ticks (0.125 s at 240 Hz)
        w.step();
    }
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
    REQUIRE(own == 0);
}
