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
//
// C9 REVIEW ROUND 2 (5th commit, 2026-09-26): the drawbar-pull limit above
// (fixed WOT + a "mean driven |slip| > 0.6" stop condition) measured
// off=150.417 N vs. on=3383.75 N, ratio 22.5x - REJECTED as unphysical: with
// MFWD off, tractor.json's rear axle alone carries most of the vehicle's
// weight, so its low_mu traction limit (lambda_mu * tyre peak mu * static
// rear Fz) should be thousands of newtons, and the physically-expected
// on/off ratio (roughly 1/rear-weight-share) is ~1.4-1.8x, not 22.5x. The
// 22.5x explanation (a "transient vs. sustained limit" methodology
// difference) is RETRACTED - it was never verified and does not survive
// the discriminating measurement below.
// Root cause, CONFIRMED by "tractor MFWD off at WOT with zero drawbar load
// on low_mu..." further down: at fixed WOT with zero rearward load on
// low_mu, the rear-only driveline spins its tyres well past |kappa|=0.6
// almost immediately (measured max mean|kappa_driven| far above 0.6 - see
// that case's own INFO line for the exact number) - the old test's
// slip-based stop condition tripped on ordinary WOT wheelspin ONSET, before
// any real rearward load had accumulated on the 100 N/s ramp. It was
// measuring "how much force had ramped on by the time WOT wheelspin
// begins", not a sustained traction/pull limit.
// Redesign: a throttle PI (same kp/ki as the item-1 steady-state case
// above) holds a LOW target ground speed (1 m/s) in gear 1, settling there
// under its own throttle BEFORE any rearward load is applied; only then
// does RearwardForce ramp slowly (100 N/s). Driven-wheel slip is left
// completely FREE this round (no slip-based stop - that is how the tyre
// reaches its own peak); the sole stop condition is the trailing 1 s
// window's mean speed dropping below half the target (0.5 m/s) - the
// throttle PI (pinned near its own ceiling under load) can no longer hold
// the tractor moving against the drawbar force. The analytic sanity band
// (drawbar_force_estimate()) uses the REAL Pacejka tyre model (via
// create_pacejka_tyre_model(), mirroring test_vehicle_skidpad.cpp's
// TyreLimitedSteadyStateSolver pattern: real VehicleDesc/TyreDesc data
// through the real tyre model, not a hand-approximated mu*Fz formula) to
// find each driven tyre's own peak Fx at its measured static Fz and the
// low_mu surface's lambda_mu (passed straight into the model exactly like
// wheel_instance.cpp's own wiring), minus every wheel's rolling resistance
// (wheel_instance.cpp's own crr formula, reproduced here since ITyreModel::
// evaluate() itself never includes it). Fz_driven is the MEASURED static
// per-wheel load (a settled rig, zero throttle/rearward force) rather than
// a hand geometric calculation - not corrected for drawbar-force load
// transfer at the RearwardForce's own application height (the chassis CoM):
// that correction would need this file to duplicate the exact same
// roll/pitch load-transfer model test_vehicle_skidpad.cpp's solver already
// implements for a different (cornering) geometry, and the chosen +-25%
// band already comfortably covers a quasi-static longitudinal load-transfer
// term of that size at this vehicle's modest CoM height. See "tractor MFWD
// on low_mu: engaged drawbar-pull limit..." below for the measured
// values/estimates/ratio.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/tyre_model.h"
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
    real max_speed = 0.0;
    real max_engine_rpm = 0.0;
    const int ticks = static_cast<int>(std::lround(3.0 / w.dt()));
    for (int i = 0; i < ticks; ++i) {
        w.step();
        const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
        CHECK(dbg.awd_clutch_torque == 0.0);
        const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
        CHECK(snap.awd_engagement == 0.0);
        max_abs_fx_front =
            std::max({max_abs_fx_front, std::abs(w.wheel_telemetry(rig.vehicle, kFL).fx), std::abs(w.wheel_telemetry(rig.vehicle, kFR).fx)});
        max_speed = std::max(max_speed, w.get_motion(rig.chassis).linear.x);
        max_engine_rpm = std::max(max_engine_rpm, snap.engine_rpm);
    }
    INFO("max |fx| over either front wheel = " << max_abs_fx_front << " N (pure rolling resistance, no drive torque), "
         << "max chassis speed = " << max_speed << " m/s, max engine rpm = " << max_engine_rpm);
    // Rolling resistance alone (TyreDesc::qsy1 + surface crr + the qsy4*(v/v0)^4
    // term) on an undriven, lightly-loaded front axle - nowhere near a driven
    // wheel's traction force (thousands of N at this throttle, see the
    // locked-lead case above's rear wheels).
    //
    // PHYS-033 P4 I4 C12 (2026-09-26, re-measured, not tuned to pass): the
    // engine's new all-speed governor makes "throttle"=0.6 deliver FULL WOT
    // torque (theta_gov saturates to 1) for as long as engine rpm sits more
    // than one droop band (200 rpm) below its speed target n_nl = idle_rpm +
    // 0.6*(high_idle_rpm - idle_rpm) = 1780 rpm - a real diesel governor's
    // job is exactly this ("floor it until you approach the requested
    // speed"). Pre-governor, throttle=0.6 mapped directly to ~60% of the WOT
    // torque curve the whole time instead. The tractor therefore accelerates
    // harder over this test's 3 s window than before (measured max engine
    // rpm=1765.9, essentially AT the 1780 rpm target - governed, not
    // open-loop - by the end) and reaches a higher chassis speed (measured
    // max=0.682 m/s), and the front tyre's rolling resistance has a
    // qsy4*(v/v0)^4 term that grows steeply with that speed. The measured
    // max |fx|=817.97 N is still two orders of magnitude below a driven
    // wheel's traction force and is pure rolling resistance throughout (the
    // awd_clutch_torque/awd_engagement checks above confirm zero front
    // drive torque every tick) - the bound below is raised with headroom to
    // match this genuine, governor-driven operating-point change, not
    // loosened to paper over a regression.
    CHECK(max_abs_fx_front < 1000.0);
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

// REVIEW C9 ROUND 2 (item 2, 5th commit): drawbar-pull LIMIT via a
// throttle-PI-held LOW target speed, not a fixed-WOT ramp - see the file
// header's "C9 REVIEW ROUND 2" comment for why the fixed-WOT version was
// rejected (its slip-based stop condition tripped on ordinary WOT wheelspin
// onset, confirmed by the discriminating diagnostic further down) and for
// the analytic sanity-band methodology below.
namespace {

// A discriminating measurement, not a hand-approximated formula: the REAL
// Pacejka tyre model's own peak longitudinal force at a fixed load/surface,
// found by sweeping kappa at alpha=gamma=0 (mirrors
// test_vehicle_skidpad.cpp's TyreLimitedSteadyStateSolver - real
// VehicleDesc/TyreDesc data through the real create_pacejka_tyre_model(),
// not mu*Fz by hand). lambda_mu is passed straight into TyreSlipInput
// exactly like wheel_instance.cpp's own wiring, so the surface's friction
// scale is applied by the model itself, not multiplied in afterward.
real tyre_peak_fx(const vehicle::TyreDesc& tyre_desc, real fz, real lambda_mu, real vx_for_model) {
    const std::unique_ptr<vehicle::ITyreModel> tm = vehicle::create_pacejka_tyre_model(tyre_desc);
    real best = 0.0;
    constexpr int kSteps = 400;
    constexpr real kKappaMax = 3.0; // the tyre model's own relaxation safety clamp (tyre_model.h)
    for (int i = 1; i <= kSteps; ++i) {
        vehicle::TyreSlipInput in;
        in.kappa = kKappaMax * static_cast<real>(i) / static_cast<real>(kSteps);
        in.alpha = 0.0;
        in.fz = fz;
        in.gamma = 0.0;
        in.vx = vx_for_model;
        in.lambda_mu = lambda_mu;
        in.crr_surface = 0.0; // ITyreModel::evaluate() never applies rolling resistance - see wheel_instance.cpp
        best = std::max(best, tm->evaluate(in).fx);
    }
    return best;
}

// Rolling-resistance drag at one wheel - reproduces wheel_instance.cpp's own
// crr_total/froll_ramp formula exactly (ITyreModel::evaluate() itself never
// includes this term, see tyre_peak_fx's comment above), using plain
// std::pow (test-only analytic estimate, not simulated state - the D7
// platform-libm-determinism rule applies to core/src, not to this sanity
// check). fx_pure_for_qsy2 is 0 here: both data/tyres/tractor_{front,rear}
// .json ship qsy2=0.0, so this term is exactly zero for the current data
// regardless of what is passed - kept symbolic (rather than dropped) so a
// future tyre file with qsy2 != 0 is still handled correctly.
real tyre_rolling_resistance(const vehicle::TyreDesc& tyre_desc, real fz, real fx_pure_for_qsy2, real vx,
                             real crr_surface) {
    const real ramp = std::clamp(vx / tyre_desc.rolling_resistance_v_eps, real(-1.0), real(1.0));
    const real crr_total = tyre_desc.qsy1 + crr_surface + tyre_desc.qsy2 * std::abs(fx_pure_for_qsy2) / tyre_desc.fz0 +
                            tyre_desc.qsy4 * std::pow(vx / tyre_desc.v0, real(4.0));
    return ramp * fz * crr_total;
}

struct StaticAxleLoads {
    real front_fz_per_wheel = 0.0;
    real rear_fz_per_wheel = 0.0;
};

// Fz_driven, MEASURED (not a hand geometric calculation): settle a rig on
// the same low_mu surface with every control at its default (throttle 0,
// no rearward force), then read each axle's own per-wheel Fz straight from
// wheel_telemetry - the sim's own settled corner weights, matching whatever
// this vehicle file's suspension/geometry actually produces. NOT corrected
// for drawbar-force load transfer at RearwardForce's own application height
// (the chassis CoM) - see the file header's "C9 REVIEW ROUND 2" comment for
// why that correction is left out (the +-25% band already covers a
// quasi-static longitudinal load-transfer term of this vehicle's size).
StaticAxleLoads measure_static_axle_loads(const vehicle::VehicleDesc& desc) {
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    for (int i = 0; i < static_cast<int>(std::lround(1.0 / w.dt())); ++i) w.step(); // let any residual settle die out
    StaticAxleLoads out;
    out.front_fz_per_wheel = 0.5 * (w.wheel_telemetry(rig.vehicle, kFL).fz + w.wheel_telemetry(rig.vehicle, kFR).fz);
    out.rear_fz_per_wheel = 0.5 * (w.wheel_telemetry(rig.vehicle, kRL).fz + w.wheel_telemetry(rig.vehicle, kRR).fz);
    return out;
}

// F_est = (sum of each driven axle's peak Fx, from the real tyre model at
// its own measured static Fz and the surface's lambda_mu) - (rolling
// resistance at EVERY wheel, driven or not - wheel_instance.cpp's own
// pre_froll_ term is independent of the drivetrain).
real drawbar_force_estimate(const vehicle::VehicleDesc& desc, const StaticAxleLoads& loads, real lambda_mu,
                             real crr_surface, bool awd_on, real target_speed) {
    const vehicle::TyreDesc& front_tyre = desc.wheels[kFL].tyre;
    const vehicle::TyreDesc& rear_tyre = desc.wheels[kRL].tyre;

    const real rear_peak_per_wheel = tyre_peak_fx(rear_tyre, loads.rear_fz_per_wheel, lambda_mu, target_speed);
    real driven_peak_total = 2.0 * rear_peak_per_wheel;
    if (awd_on) {
        const real front_peak_per_wheel = tyre_peak_fx(front_tyre, loads.front_fz_per_wheel, lambda_mu, target_speed);
        driven_peak_total += 2.0 * front_peak_per_wheel;
    }

    const real roll_total = 2.0 * tyre_rolling_resistance(front_tyre, loads.front_fz_per_wheel, 0.0, target_speed, crr_surface) +
                             2.0 * tyre_rolling_resistance(rear_tyre, loads.rear_fz_per_wheel, 0.0, target_speed, crr_surface);
    return driven_peak_total - roll_total;
}

struct DrawbarResult {
    real limit_force = 0.0;
    real throttle_at_limit = 0.0;
    real mean_kappa_driven_at_limit = 0.0;
    real final_speed = 0.0;
};

real mean_of_deque(const std::deque<real>& d) {
    if (d.empty()) return 0.0;
    real s = 0.0;
    for (real v : d) s += v;
    return s / static_cast<real>(d.size());
}

// Speed-controlled drawbar-pull limit search (REVIEW C9 round 2): a
// throttle PI (same kp/ki as the item-1 steady-state case above) holds
// kTargetSpeed in gear 1, settling there under its OWN throttle before any
// rearward load is applied; only then does RearwardForce ramp slowly.
// Driven-wheel slip is left completely FREE (no slip-based stop - "that is
// how the tyre reaches its peak"); the sole stop condition is the trailing
// window's mean speed collapsing below half the target.
DrawbarResult drawbar_pull_limit(bool awd_on) {
    constexpr real kTargetSpeed = 1.0; // m/s - low, held speed
    constexpr real kp = 0.08, ki = 0.05;
    constexpr real kThrottleMax = 1.0;
    // PHYS-033 P4 I4 C12 (2026-09-26, re-measured, not tuned to pass): this
    // was 5.0 s before the engine gained an all-speed governor. The
    // governor inserts an inner engine-speed control loop (droop band 200
    // rpm) between this PI's "throttle" output and the torque the engine
    // actually delivers, so the same kp/ki now converge more slowly than
    // before - with the old 5 s window this PI's clamped integral (+-20,
    // ki=0.05, so max ki*integral contribution 1.0) hadn't accumulated
    // enough to saturate throttle to 1.0 yet, and the search settled at a
    // false equilibrium well below kTargetSpeed (measured: throttle~0.264,
    // final speed~0.32 m/s, i.e. off.limit_force collapsed to ~0 the moment
    // the very first ramp window's stop condition tripped). 30 s gives the
    // same PI time to saturate throttle to 1.0 against the governed plant
    // (confirmed by measurement: throttle_at_limit=1 for both off/on at
    // 30 s) before the drawbar-load ramp begins, restoring the intended
    // "already at its own best speed under zero load" precondition. Only
    // this settle time changed - kp/ki, the ramp itself and the 25%/1.2x
    // acceptance bounds below are untouched.
    constexpr real kSettleAtZeroLoadSeconds = 30.0; // hold target speed under zero load before ramping
    constexpr real kRampRatePerSecond = 100.0;     // N/s - slow relative to the vehicle's own response time
    constexpr real kMaxForce = 8000.0;             // N - generous headroom over the estimated off/on limits
    constexpr real kWindowSeconds = 1.0;           // trailing window for the stop condition/reported means
    constexpr real kMinMeanSpeedFraction = 0.5;    // stop once trailing mean speed < this * target

    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("awd", awd_on ? 1.0 : 0.0);

    RearwardForce rf(Frame(rig.chassis));
    w.add_force_element(&rf);

    const real dt = w.dt();
    real integral = 0.0;
    auto step_with_speed_control = [&]() -> real {
        const real speed = w.get_motion(rig.chassis).linear.x;
        const real err = kTargetSpeed - speed;
        integral = std::clamp(integral + err * dt, -20.0, 20.0);
        const real throttle = std::clamp(kp * err + ki * integral, 0.0, kThrottleMax);
        w.set_control("throttle", throttle);
        w.step();
        return throttle;
    };

    // Phase 1: settle at ZERO rearward load - the tractor must already be
    // holding its target speed under its own throttle before any drawbar
    // load is applied.
    const int settle_ticks = static_cast<int>(std::lround(kSettleAtZeroLoadSeconds / dt));
    for (int i = 0; i < settle_ticks; ++i) step_with_speed_control();

    // Phase 2: ramp the rearward force slowly.
    const std::size_t window_n = static_cast<std::size_t>(std::lround(kWindowSeconds / dt));
    std::deque<real> speed_window, kappa_window, throttle_window;
    const std::vector<std::size_t> wheel_indices{kFL, kFR, kRL, kRR};

    real force = 0.0;
    DrawbarResult result;
    while (force < kMaxForce) {
        force += kRampRatePerSecond * dt;
        rf.set_magnitude(force);
        const real throttle = step_with_speed_control();

        real slip_sum = 0.0;
        int driven_count = 0;
        for (std::size_t i : wheel_indices) {
            if (!w.wheel_is_driven(rig.vehicle, i)) continue;
            slip_sum += std::abs(w.wheel_telemetry(rig.vehicle, i).slip_ratio);
            ++driven_count;
        }
        const real driven_slip = driven_count > 0 ? slip_sum / static_cast<real>(driven_count) : 0.0;
        const real speed = w.get_motion(rig.chassis).linear.x;
        result.final_speed = speed;

        speed_window.push_back(speed);
        if (speed_window.size() > window_n) speed_window.pop_front();
        kappa_window.push_back(driven_slip);
        if (kappa_window.size() > window_n) kappa_window.pop_front();
        throttle_window.push_back(throttle);
        if (throttle_window.size() > window_n) throttle_window.pop_front();

        if (speed_window.size() >= window_n && mean_of_deque(speed_window) < kMinMeanSpeedFraction * kTargetSpeed) {
            result.limit_force = force - kRampRatePerSecond * kWindowSeconds; // force at the start of the failing window
            result.mean_kappa_driven_at_limit = mean_of_deque(kappa_window);
            result.throttle_at_limit = mean_of_deque(throttle_window);
            return result;
        }
        result.limit_force = force;
    }
    // Reached kMaxForce without ever failing the speed gate - report the
    // trailing window state at the cap for diagnostics.
    result.mean_kappa_driven_at_limit = mean_of_deque(kappa_window);
    result.throttle_at_limit = mean_of_deque(throttle_window);
    return result;
}
} // namespace

// Confirms the root cause of the rejected round-1 result (see the file
// header's "C9 REVIEW ROUND 2" comment): at fixed WOT with ZERO rearward
// load on low_mu, the rear-only driveline's own tyres spin well past the
// old |kappa| > 0.6 stop condition almost immediately - i.e. the round-1
// test's "150.417 N" was the ramp force accumulated by the time ordinary
// WOT wheelspin began, not a sustained traction/pull limit. Reported, not
// asserted against a specific bound - the printed max|kappa_driven| value
// itself, compared against the old 0.6 threshold, IS the discriminating
// result.
TEST_CASE("tractor MFWD off at WOT with zero drawbar load on low_mu: discriminating measurement for the "
          "rejected round-1 drawbar-pull result",
          "[validation][vehicle][powertrain][drivetrain][.][diag]") {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    w.set_control("awd", 0.0); // MFWD off: rear-only driveline

    real max_mean_abs_kappa_driven = 0.0;
    const int ticks = static_cast<int>(std::lround(3.0 / w.dt()));
    for (int i = 0; i < ticks; ++i) {
        w.step();
        real slip_sum = 0.0;
        int n = 0;
        for (std::size_t idx : {kFL, kFR, kRL, kRR}) {
            if (!w.wheel_is_driven(rig.vehicle, idx)) continue;
            slip_sum += std::abs(w.wheel_telemetry(rig.vehicle, idx).slip_ratio);
            ++n;
        }
        if (n > 0) max_mean_abs_kappa_driven = std::max(max_mean_abs_kappa_driven, slip_sum / static_cast<real>(n));
    }
    INFO("max mean|driven slip_ratio| over 3s at WOT/zero drawbar load on low_mu = " << max_mean_abs_kappa_driven
         << " (round-1's stop condition tripped at 0.6)");
    CHECK(max_mean_abs_kappa_driven > 0.0); // sanity: actually slipping under WOT, not stalled/airborne
}

TEST_CASE("tractor MFWD on low_mu: engaged drawbar-pull limit is at least 1.2x the rear-only limit",
          "[validation][vehicle][powertrain][drivetrain]") {
    constexpr real kTargetSpeed = 1.0; // must match drawbar_pull_limit's own kTargetSpeed
    const vehicle::VehicleDesc desc = load_tractor();

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId low_mu_id = surfaces->id_for("low_mu");
    const real lambda_mu = surfaces->lambda_mu_for(low_mu_id);
    const real crr_surface = surfaces->crr_for(low_mu_id);

    const StaticAxleLoads loads = measure_static_axle_loads(desc);
    const real est_off = drawbar_force_estimate(desc, loads, lambda_mu, crr_surface, false, kTargetSpeed);
    const real est_on = drawbar_force_estimate(desc, loads, lambda_mu, crr_surface, true, kTargetSpeed);

    const DrawbarResult off = drawbar_pull_limit(false);
    const DrawbarResult on = drawbar_pull_limit(true);

    // INFO lines placed BEFORE the guard REQUIREs (PHYS-033 P4 I4 C12
    // diagnosis, 2026-09-26) so a guard failure still prints the measured
    // fields that explain it - a REQUIRE aborts the test case immediately,
    // and Catch2 only prints INFO messages active at the point of failure.
    INFO("static loads: front_fz_per_wheel=" << loads.front_fz_per_wheel << " N rear_fz_per_wheel="
                                              << loads.rear_fz_per_wheel << " N (lambda_mu=" << lambda_mu
                                              << " crr=" << crr_surface << ")");
    INFO("MFWD off: measured limit=" << off.limit_force << " N, estimate=" << est_off << " N, throttle@limit="
                                      << off.throttle_at_limit << ", mean driven kappa@limit="
                                      << off.mean_kappa_driven_at_limit << ", final speed=" << off.final_speed << " m/s");
    INFO("MFWD on:  measured limit=" << on.limit_force << " N, estimate=" << est_on << " N, throttle@limit="
                                      << on.throttle_at_limit << ", mean driven kappa@limit="
                                      << on.mean_kappa_driven_at_limit << ", final speed=" << on.final_speed << " m/s");

    REQUIRE(off.limit_force > 1.0); // guard the ratio below against a near-zero denominator
    REQUIRE(est_off > 1.0);         // guard the relative-error checks below
    const real ratio = on.limit_force / off.limit_force;
    INFO("ratio (on/off) = " << ratio);

    CHECK(std::abs(off.limit_force - est_off) <= 0.25 * std::abs(est_off));
    CHECK(std::abs(on.limit_force - est_on) <= 0.25 * std::abs(est_on));
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
