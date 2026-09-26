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
// Round-2 redesign: a throttle PI aimed at a 1 m/s target speed in gear 1
// before a slow (100 N/s) rearward-force ramp, with the trailing 1 s mean
// speed falling below half the target as the only stop condition.
// C12 REVIEW ROUND 2 (2026-09-26): that target was never reachable - gear 1
// at the engine's 2400 rpm high idle is 0.613 m/s (gear 1 21.0 x
// transfer.rear_ratio 1.0 x diff 20.0, rear radius 1.025 m), so the PI could
// never hold it and the "held target" never existed. Replaced by an OECD
// Code 2 style procedure (drawbar_pull_limit()): gear 1, "hand_throttle"
// (the governor lever) 1, foot throttle 0; settle at zero drawbar load until
// the mean chassis speed of consecutive 1 s windows changes by < 0.1 % (a
// 60 s timeout fails the test), v0 = the last window's mean; then the same
// 100 N/s ramp. C12 REVIEW ROUND 3: the limit is the traction SATURATION
// point - the drawbar force where the driven tyres' summed fx (trailing
// 0.25 s mean) peaks during the ramp. The first trailing 1 s window whose
// mean speed is < 0.5*v0 only STOPS the ramp: a 6000 kg chassis needs ~2 s
// of ramp to lose half its speed after saturation, so a limit read there
// is biased high by a ramp-rate-dependent ~210 N. Slip is left free. The analytic
// sanity band (drawbar_force_estimate()) uses the REAL Pacejka tyre model (via
// create_pacejka_tyre_model(), mirroring test_vehicle_skidpad.cpp's
// TyreLimitedSteadyStateSolver pattern: real VehicleDesc/TyreDesc data
// through the real tyre model, not a hand-approximated mu*Fz formula) to
// find each driven tyre's own peak Fx at its measured static Fz and the
// low_mu surface's lambda_mu (passed straight into the model exactly like
// wheel_instance.cpp's own wiring), minus every wheel's rolling resistance
// (wheel_instance.cpp's own crr formula, reproduced here since ITyreModel::
// evaluate() itself never includes it). Fz_driven is the MEASURED static
// per-wheel load (a settled rig, zero throttle/rearward force) rather than
// a hand geometric calculation - deliberately NOT corrected for load
// transfer: the ground traction force X acts at the contact patches, a CoM
// height h below the RearwardForce (applied AT the CoM), so it moves X*h/L
// from the front to the rear axle. The test case below measures that
// transfer and each other term of the remaining gap to F_est at the
// saturation point, instead of folding a modelled correction into the
// estimate. See "tractor MFWD on low_mu: engaged drawbar-pull limit..."
// below for the measured values/estimates/ratio.

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
#include <cstdio>
#include <deque>
#include <iterator>
#include <memory>
#include <sstream>
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

// ---------------------------------------------------------------------------
// MFWD-off front-axle torque balance (C12 review round 2, fix 2): shared by
// the acceptance case and the [.][diag] CSV dump below.
// ---------------------------------------------------------------------------
namespace {

struct FrontAxleRow {
    real t = 0.0;
    real fx[2]{}, fz[2]{}, kappa[2]{}, omega[2]{};
    real j_alpha[2]{}; // J_hub * domega/dt (this tick)
    real r_fx[2]{};    // wheel_radius * fx
    real t_shaft[2]{}; // PowertrainDebug::hub_shaft_torque
    real resid[2]{};   // J_hub*domega/dt - (t_shaft - r*fx)
    real t_pred[2]{};  // coef*(J_c*alpha_c - loss_tau): reflected carrier + transfer.front inertia (+ lagged mesh loss)
    real fx_inertial[2]{}; // -(J_hub*alpha_k + (ratio^2/4)*J_c*(alpha_L+alpha_R))/r: pure inertia, no mesh loss
    real ax = 0.0, pitch_rate = 0.0, speed = 0.0, rpm = 0.0, theta_gov = 0.0, throttle_eff = 0.0;
    real awd_e = 0.0, awd_torque = 0.0, clutch_engagement = 0.0;
};

struct FrontAxleDiag {
    std::vector<FrontAxleRow> rows;
    std::size_t peak = 0; // index of the row with the largest max(|fx_FL|, |fx_FR|)
    real j_hub = 0.0, j_carrier = 0.0, ratio = 0.0, efficiency = 0.0, radius = 0.0;
};

// Rig and controls: settle 1 s, gear_request 1, throttle 0.6, "awd" left at
// its default 0 (never set), 3 s run. At tick_hz == substep_hz every
// per-tick read is the ONLY substep's value, so the hub/carrier balances
// below close exactly; at 240/960 (the default rig rates) they are tick-mean
// vs. last-substep and only approximate.
FrontAxleDiag run_mfwd_off_front_axle_diag(real tick_hz, real substep_hz) {
    const vehicle::VehicleDesc desc = load_tractor();
    MfwdRig rig = build_mfwd_rig(desc, "asphalt", tick_hz, substep_hz);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.6);

    FrontAxleDiag d;
    const drivetrain::DifferentialDesc fd = diff_desc(desc, "front_diff");
    real transfer_front_inertia = 0.0;
    for (const auto& c : desc.powertrain.components) {
        if (c.id == "transfer") transfer_front_inertia = std::get<drivetrain::TransferCaseDesc>(c.params).front_inertia_kgm2;
    }
    d.j_hub = desc.wheels[kFL].wheel_inertia;
    d.radius = desc.wheels[kFL].wheel_radius;
    d.j_carrier = fd.carrier_inertia_kgm2 + transfer_front_inertia; // the one node "transfer.front"+"front_diff.carrier"
    d.ratio = fd.ratio;
    d.efficiency = fd.efficiency;
    const real coef = -fd.ratio / 2.0; // Differential::hub_coef()

    const real dt = w.dt();
    real omega_prev[2] = {w.wheel_omega(rig.vehicle, kFL), w.wheel_omega(rig.vehicle, kFR)};
    real speed_prev = w.get_motion(rig.chassis).linear.x;
    real lambda_prev = w.powertrain_debug(rig.vehicle).hub_shaft_torque[kFL] / coef; // front_diff row multiplier
    const int ticks = static_cast<int>(std::lround(3.0 / dt));
    real max_abs = -1.0;
    for (int i = 0; i < ticks; ++i) {
        w.step();
        const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
        const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
        const Motion m = w.get_motion(rig.chassis);
        FrontAxleRow r;
        r.t = (i + 1) * dt;
        real alpha[2];
        for (int k = 0; k < 2; ++k) {
            const std::size_t wi = k == 0 ? kFL : kFR;
            const vehicle::WheelTelemetry& tel = w.wheel_telemetry(rig.vehicle, wi);
            r.fx[k] = tel.fx;
            r.fz[k] = tel.fz;
            r.kappa[k] = tel.slip_ratio;
            r.omega[k] = w.wheel_omega(rig.vehicle, wi);
            alpha[k] = (r.omega[k] - omega_prev[k]) / dt;
            r.j_alpha[k] = d.j_hub * alpha[k];
            r.r_fx[k] = d.radius * tel.fx;
            r.t_shaft[k] = dbg.hub_shaft_torque[wi];
            r.resid[k] = r.j_alpha[k] - (r.t_shaft[k] - r.r_fx[k]);
            omega_prev[k] = r.omega[k];
        }
        // Carrier node: J_c*alpha_c = lambda + loss_tau, alpha_c = ratio*(alpha_L+alpha_R)/2
        // (the open diff's own rigid row), loss_tau = -(1-eta)*|lambda_prev|*sign(omega_c,old)
        // (Differential::pre_solve). Per-hub prediction = coef*lambda.
        const real alpha_c = d.ratio * 0.5 * (alpha[0] + alpha[1]);
        const real omega_c_old = d.ratio * 0.5 * ((r.omega[0] - alpha[0] * dt) + (r.omega[1] - alpha[1] * dt));
        const real loss_tau = -(1.0 - d.efficiency) * std::abs(lambda_prev) * (omega_c_old > 0.0 ? 1.0 : (omega_c_old < 0.0 ? -1.0 : 0.0));
        const real lambda_pred = d.j_carrier * alpha_c - loss_tau;
        r.t_pred[0] = r.t_pred[1] = coef * lambda_pred;
        for (int k = 0; k < 2; ++k) {
            r.fx_inertial[k] = -(d.j_hub * alpha[k] + coef * coef * d.j_carrier * (alpha[0] + alpha[1])) / d.radius;
        }
        lambda_prev = dbg.hub_shaft_torque[kFL] / coef;

        r.speed = m.linear.x;
        r.ax = (m.linear.x - speed_prev) / dt;
        speed_prev = m.linear.x;
        r.pitch_rate = m.angular.y;
        r.rpm = snap.engine_rpm;
        const real band = 0.0909 * 2200.0; // diesel_i4_4l5_turbo.json governor droop*rated_rpm
        r.theta_gov = std::clamp((snap.governor_setpoint_rpm - snap.engine_rpm) / band, 0.0, 1.0);
        r.throttle_eff = dbg.throttle_eff;
        r.awd_e = snap.awd_engagement;
        r.awd_torque = dbg.awd_clutch_torque;
        r.clutch_engagement = snap.clutch_engagement;
        const real a = std::max(std::abs(r.fx[0]), std::abs(r.fx[1]));
        if (a > max_abs) {
            max_abs = a;
            d.peak = d.rows.size();
        }
        d.rows.push_back(r);
    }
    return d;
}

void print_front_axle_diag(const FrontAxleDiag& d, const char* label) {
    std::printf("=== %s: J_hub=%.4g J_carrier=%.4g ratio=%.4g eta=%.4g r=%.4g peak at t=%.5f s ===\n", label, d.j_hub, d.j_carrier,
                d.ratio, d.efficiency, d.radius, d.rows[d.peak].t);
    std::printf("t,fx_FL,fx_FR,fz_FL,kappa_FL,omega_FL,Jalpha_FL,rfx_FL,Tshaft_FL,resid_FL,Tpred_FL,resid_FR,ax,pitch_rate,speed,"
                "rpm,theta_gov,throttle_eff,clutch_e,awd_e,awd_T,fxI_FL,fxI_FR,dT_FL\n");
    for (std::size_t i = 0; i < d.rows.size(); ++i) {
        const FrontAxleRow& r = d.rows[i];
        std::printf("%.5f,%.3f,%.3f,%.1f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3e,%.3f,%.3e,%.4f,%.5f,%.5f,%.1f,%.4f,%.4f,%.4f,%.3g,%.3g,%.3f,%.3f,%.3e\n", r.t,
                    r.fx[0], r.fx[1], r.fz[0], r.kappa[0], r.omega[0], r.j_alpha[0], r.r_fx[0], r.t_shaft[0], r.resid[0],
                    r.t_pred[0], r.resid[1], r.ax, r.pitch_rate, r.speed, r.rpm, r.theta_gov, r.throttle_eff, r.clutch_engagement,
                    r.awd_e, r.awd_torque, r.fx_inertial[0], r.fx_inertial[1], r.t_pred[0] - r.t_shaft[0]);
    }
}

} // namespace

// MFWD off (default state, "awd" channel defaults to 0): the front axle must
// carry no drive torque and no phantom drag - adding the transfer_case/
// front_diff hardware to the graph must not load the front wheels just by
// existing. The front hubs are not free wheels, though: with the transfer
// clutch open they still back-drive their open front_diff and the transfer
// case's front output (one shaft node, J_c = carrier 0.2 + transfer.front
// 0.05 = 0.25 kg*m^2, i.e. (ratio^2/2)*J_c = 29.3 kg*m^2 reflected onto each
// front wheel when both turn together, beside the hub's own 25.6), so every
// change of front wheel speed costs a torque
// that only the tyre can supply - as fx.
//
// C12 review round 2 (2026-09-26), measured with the diagnostic case below
// (same rig and controls, 240/960 and 960/960):
//  - The peak front |fx| is a launch transient, not rolling resistance
//    (WheelTelemetry::fx does not include rolling resistance) and not a drag.
//    The launch in gear 1 runs at full rack (theta_gov = 1 until the engine
//    is within one droop band of its 1780 rpm setpoint): chassis ax peaks at
//    5.3 m/s^2 and front Fz falls from ~10.3 kN to 1.9 kN. That excites the
//    front wheels' torsional mode - hub plus reflected front-driveline
//    inertia against the tyre's longitudinal relaxation compliance, lightly
//    damped at ~0.7 m/s (period ~0.08 s). Peak 817.95 N (FR) at
//    t = 0.346 s after gear_request at 240/960 (960/960: 838.11 N at
//    t = 0.345 s), clutch still slipping (engagement 0.46), engine 1745 rpm;
//    below 12 N from t = 2 s on.
//  - Hub balance J_hub*domega/dt = hub_shaft_torque - r*fx closes to
//    1.1e-11 N*m at 960/960 (only approximately at 240/960, whose per-tick
//    reads are the last substep's while domega/dt is the tick mean).
//  - hub_shaft_torque = coef*(J_c*alpha_c - loss_tau) (coef = -ratio/2,
//    alpha_c = ratio*(alpha_L + alpha_R)/2, loss_tau the diff's lagged
//    -(1-eta)*|lambda_prev|*sign(omega_c) mesh loss) to 2.3e-11 N*m: nothing
//    else acts on the front driveline node, and awd_clutch_torque is exactly
//    0 every tick.
//  - At the peak fx/fx_inertial = 1.033 (FR, 960/960), with fx_inertial =
//    -(J_hub*alpha_k + (ratio^2/4)*J_c*(alpha_L + alpha_R))/r (pure inertia,
//    no loss); the remainder is that lagged mesh loss.
//  - Pre-C12 (8fca97c, the same diagnostic run in a temporary worktree): peak
//    465.74 N at t = 0.183 s, same mechanism (fx/fx_inertial 1.034 at
//    960/960, both closures <= 2.7e-11 N*m) with a softer launch - throttle
//    0.6 then meant 60% of the WOT curve (ax up to 4.45 m/s^2, front Fz down
//    to 3.3 kN). The governor gives full rack at the same pedal, so the
//    launch and its front-wheel transient are harder.
// This case therefore asserts the mechanism, not a magnitude, at 960/960
// where every per-tick read is the tick's only substep: (1) zero transfer
// clutch torque and engagement every tick; (2) both balances close to
// 1e-9 N*m (measured <= 2.3e-11) - a drag torque anywhere on the front axle
// would show up as a residual; (3) at the peak |fx/fx_inertial - 1| <=
// 1 - eta = 0.06: r*(fx - fx_inertial) = -coef*loss_tau, of magnitude
// (1-eta)*|coef*lambda_prev|, i.e. (1-eta) times the shaft share of r*fx
// when lambda_prev ~ lambda (measured 0.033, shaft share 0.57).
TEST_CASE("tractor MFWD off (default): front axle carries no drive torque and no drag",
          "[validation][vehicle][powertrain][drivetrain]") {
    const FrontAxleDiag d = run_mfwd_off_front_axle_diag(960.0, 960.0);
    REQUIRE(!d.rows.empty());

    int awd_active_ticks = 0;
    real worst_hub_resid = 0.0;
    real worst_driveline_resid = 0.0;
    for (const FrontAxleRow& r : d.rows) {
        if (r.awd_torque != 0.0 || r.awd_e != 0.0) ++awd_active_ticks;
        for (int k = 0; k < 2; ++k) {
            worst_hub_resid = std::max(worst_hub_resid, std::abs(r.resid[k]));
            worst_driveline_resid = std::max(worst_driveline_resid, std::abs(r.t_pred[k] - r.t_shaft[k]));
        }
    }
    const FrontAxleRow& pk = d.rows[d.peak];
    const int k = std::abs(pk.fx[0]) >= std::abs(pk.fx[1]) ? 0 : 1;
    INFO("peak front |fx| = " << std::abs(pk.fx[k]) << " N (" << (k == 0 ? "FL" : "FR") << ") at t = " << pk.t
                              << " s, fx_inertial = " << pk.fx_inertial[k] << " N, J_hub*alpha = " << pk.j_alpha[k]
                              << " N*m, hub_shaft_torque = " << pk.t_shaft[k] << " N*m, clutch engagement = "
                              << pk.clutch_engagement << ", engine rpm = " << pk.rpm);
    INFO("worst hub balance residual = " << worst_hub_resid << " N*m, worst front-driveline residual = "
                                         << worst_driveline_resid << " N*m, ticks with transfer-clutch torque or "
                                         << "engagement = " << awd_active_ticks);
    CHECK(awd_active_ticks == 0);
    CHECK(worst_hub_resid <= 1e-9);
    CHECK(worst_driveline_resid <= 1e-9);
    REQUIRE(std::abs(pk.fx_inertial[k]) > 100.0); // the peak is a real transient, so the ratio below is meaningful
    const real peak_ratio = pk.fx[k] / pk.fx_inertial[k];
    INFO("fx / fx_inertial at the peak = " << peak_ratio << " (bound: within 1 - eta = " << (1.0 - d.efficiency) << ")");
    CHECK(std::abs(peak_ratio - 1.0) <= 1.0 - d.efficiency);
}

TEST_CASE("tractor MFWD off: front-axle torque-balance diagnostic for the peak front |fx|",
          "[validation][vehicle][powertrain][drivetrain][.][diag]") {
    const FrontAxleDiag a = run_mfwd_off_front_axle_diag(240.0, 960.0);
    print_front_axle_diag(a, "240/960");
    const FrontAxleDiag b = run_mfwd_off_front_axle_diag(960.0, 960.0);
    print_front_axle_diag(b, "960/960");
    CHECK(!a.rows.empty());
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

// Drawbar-pull LIMIT (REVIEW C9 ROUND 2, procedure replaced in C12 review
// round 2): an OECD Code 2 style governed-speed test, not a fixed-WOT ramp -
// see the file header's "C9 REVIEW ROUND 2" and "C12 REVIEW ROUND 2"
// comments for why the fixed-WOT version and the 1 m/s speed-PI version
// were rejected, and for the analytic sanity-band methodology below.
namespace {

// A discriminating measurement, not a hand-approximated formula: the REAL
// Pacejka tyre model's own peak longitudinal force at a fixed load/surface,
// found by sweeping kappa at alpha=gamma=0 (mirrors
// test_vehicle_skidpad.cpp's TyreLimitedSteadyStateSolver - real
// VehicleDesc/TyreDesc data through the real create_pacejka_tyre_model(),
// not mu*Fz by hand). lambda_mu is passed straight into TyreSlipInput
// exactly like wheel_instance.cpp's own wiring, so the surface's friction
// scale is applied by the model itself, not multiplied in afterward.
struct TyrePeak {
    real fx = 0.0;    // N, peak longitudinal force
    real kappa = 0.0; // slip ratio at which it occurs (sweep resolution 3.0/400 = 0.0075)
};

TyrePeak tyre_peak(const vehicle::TyreDesc& tyre_desc, real fz, real lambda_mu, real vx_for_model) {
    const std::unique_ptr<vehicle::ITyreModel> tm = vehicle::create_pacejka_tyre_model(tyre_desc);
    TyrePeak best;
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
        const real fx = tm->evaluate(in).fx;
        if (fx > best.fx) {
            best.fx = fx;
            best.kappa = in.kappa;
        }
    }
    return best;
}

// Rolling-resistance drag at one wheel - reproduces wheel_instance.cpp's own
// crr_total/froll_ramp formula exactly (ITyreModel::evaluate() itself never
// includes this term, see tyre_peak's comment above), using plain
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
// for load transfer (the ground traction force acting a CoM height below
// RearwardForce's own application point) - see the file header's "C9 REVIEW
// ROUND 2" comment and the measured transfer in the drawbar test case.
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
                             real crr_surface, bool awd_on, real speed) {
    const vehicle::TyreDesc& front_tyre = desc.wheels[kFL].tyre;
    const vehicle::TyreDesc& rear_tyre = desc.wheels[kRL].tyre;

    const real rear_peak_per_wheel = tyre_peak(rear_tyre, loads.rear_fz_per_wheel, lambda_mu, speed).fx;
    real driven_peak_total = 2.0 * rear_peak_per_wheel;
    if (awd_on) {
        const real front_peak_per_wheel = tyre_peak(front_tyre, loads.front_fz_per_wheel, lambda_mu, speed).fx;
        driven_peak_total += 2.0 * front_peak_per_wheel;
    }

    const real roll_total = 2.0 * tyre_rolling_resistance(front_tyre, loads.front_fz_per_wheel, 0.0, speed, crr_surface) +
                             2.0 * tyre_rolling_resistance(rear_tyre, loads.rear_fz_per_wheel, 0.0, speed, crr_surface);
    return driven_peak_total - roll_total;
}

struct DrawbarResult {
    bool settled = false;       // the zero-load window-drift gate passed before the timeout
    real settle_time_s = 0.0;   // time from gear_request until it passed
    real v0 = 0.0;              // m/s, mean chassis speed over the last zero-load 1 s window
    real rpm_v0 = 0.0;          // mean engine rpm over that window
    real theta_gov_v0 = 0.0;    // mean governor rack over that window
    bool reached_stop = false; // the speed gate failed before kMaxForce
    int not_running_ticks = 0; // ramp ticks with engine_state != Running

    // The reported LIMIT: traction saturation, where the sum of the driven
    // wheels' telemetry fx (pure tyre force, no rolling resistance) peaks
    // during the ramp. Taken on a trailing kSatMeanSeconds mean of that sum,
    // not the raw per-tick value: with MFWD engaged the per-tick signal
    // carries single-tick spikes (see the measured-results comment). Every
    // "_at_sat" quantity is a mean over that same window.
    real limit_force = 0.0;    // N, mean drawbar force over the window (its centre - the ramp is linear)
    real peak_driven_fx = 0.0; // N, the maximum window mean itself
    real chassis_speed_at_sat = 0.0;
    real chassis_height_at_sat = 0.0; // m, chassis CoM z above the z = 0 ground
    real accel_at_sat = 0.0;          // m/s^2, chassis (v_last - v_first)/(n-1)/dt over the window
    real kappa_at_sat[4] = {};        // |slip_ratio| per wheel (FL, FR, RL, RR)
    real fx_at_sat[4] = {};           // N, telemetry fx per wheel
    real fz_at_sat[4] = {};           // N, telemetry fz per wheel
    real min_rpm_at_sat = 0.0;
    real max_theta_gov_at_sat = 0.0;
    // Noise check: the argmax of the raw per-tick sum.
    real limit_force_per_tick = 0.0;
    real peak_driven_fx_per_tick = 0.0;
    real fz_per_tick_at_peak[4] = {}; // N, per-wheel fz at that single tick

    // The ramp's STOP condition (not the limit): the first trailing 1 s
    // window whose mean chassis speed is below 0.5*v0. Means over that window.
    real stop_force = 0.0; // N, force at the start of the failing window
    real mean_speed_at_stop = 0.0;
    real mean_kappa_rear_at_stop = 0.0;  // mean |slip_ratio| of RL/RR
    real mean_kappa_front_at_stop = 0.0; // mean |slip_ratio| of FL/FR
    real mean_rpm_at_stop = 0.0;
    real min_rpm_at_stop = 0.0;
    real mean_theta_gov_at_stop = 0.0;
};

real mean_of_deque(const std::deque<real>& d) {
    if (d.empty()) return 0.0;
    real s = 0.0;
    for (real v : d) s += v;
    return s / static_cast<real>(d.size());
}

const drivetrain::TorqueMapEngineDesc& engine_desc(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* e = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) return *e;
    }
    throw std::runtime_error("engine_desc: no torque_map engine component");
}

// Chassis speed in gear 1 with the engine at no-load high idle and no tyre
// slip: omega_engine / (gear 1 * transfer.rear_ratio * diff.ratio) * r_rear.
real gear1_high_idle_speed(const vehicle::VehicleDesc& d) {
    real gear1 = 0.0, rear_ratio = 0.0;
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) gear1 = g->forward_ratios.at(0);
        if (const auto* t = std::get_if<drivetrain::TransferCaseDesc>(&c.params)) rear_ratio = t->rear_ratio;
    }
    const real overall = gear1 * rear_ratio * diff_desc(d, "diff").ratio;
    constexpr real kRpmToRadS = 3.14159265358979323846 / 30.0;
    return engine_desc(d).governor.high_idle_rpm * kRpmToRadS / overall * d.wheels[kRL].wheel_radius;
}

// OECD Code 2 style drawbar test on low_mu: gear 1, hand throttle (the
// governor lever) fully open, foot throttle 0. Phase 1 lets the tractor
// settle at zero drawbar load until the mean chassis speed of consecutive
// 1 s windows changes by less than 0.1 % (v0 = the last window's mean); a
// timeout returns settled = false, which the caller REQUIREs. Phase 2 ramps
// the rearward force at 100 N/s. The LIMIT is the drawbar force at the tick
// where the summed driven-wheel fx peaks (traction saturation); the ramp
// STOPS at the first trailing 1 s window whose mean speed falls below
// 0.5 * v0 (a speed collapse that, with a 6000 kg chassis, lags the
// saturation by a ramp-rate-dependent amount, so it is not the limit).
DrawbarResult drawbar_pull_limit(bool awd_on) {
    constexpr real kSettleWindowSeconds = 1.0;
    constexpr real kSettleDriftFraction = 0.001; // consecutive window means within 0.1 %
    constexpr real kSettleTimeoutSeconds = 60.0;
    constexpr real kRampRatePerSecond = 100.0; // N/s
    constexpr real kMaxForce = 8000.0;         // N
    constexpr real kWindowSeconds = 1.0;       // trailing window for the stop condition/reported means
    constexpr real kMinMeanSpeedFraction = 0.5;
    constexpr real kSatMeanSeconds = 0.25; // trailing mean for the saturation-argmax noise check

    const vehicle::VehicleDesc desc = load_tractor();
    const drivetrain::TorqueMapEngineDesc& eng = engine_desc(desc);
    const real governor_band = eng.governor.droop * eng.governor.rated_rpm;
    MfwdRig rig = build_mfwd_rig(desc, "low_mu");
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("awd", awd_on ? 1.0 : 0.0);
    w.set_control("throttle", 0.0);
    w.set_control("hand_throttle", 1.0);

    RearwardForce rf(Frame(rig.chassis));
    w.add_force_element(&rf);

    const real dt = w.dt();
    auto theta_gov = [&](const PowertrainSnapshot& snap) {
        return std::clamp((snap.governor_setpoint_rpm - snap.engine_rpm) / governor_band, 0.0, 1.0);
    };
    DrawbarResult result;

    // Phase 1: settle at zero drawbar load.
    const int settle_window_ticks = static_cast<int>(std::lround(kSettleWindowSeconds / dt));
    const int settle_timeout_ticks = static_cast<int>(std::lround(kSettleTimeoutSeconds / dt));
    real prev_mean = 0.0;
    bool have_prev = false;
    for (int elapsed = 0; elapsed < settle_timeout_ticks;) {
        real sum_v = 0.0, sum_rpm = 0.0, sum_theta = 0.0;
        for (int i = 0; i < settle_window_ticks; ++i, ++elapsed) {
            w.step();
            const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
            sum_v += w.get_motion(rig.chassis).linear.x;
            sum_rpm += snap.engine_rpm;
            sum_theta += theta_gov(snap);
        }
        const real mean = sum_v / settle_window_ticks;
        if (have_prev && mean > 0.0 && std::abs(mean - prev_mean) < kSettleDriftFraction * mean) {
            result.settled = true;
            result.settle_time_s = elapsed * dt;
            result.v0 = mean;
            result.rpm_v0 = sum_rpm / settle_window_ticks;
            result.theta_gov_v0 = sum_theta / settle_window_ticks;
            break;
        }
        prev_mean = mean;
        have_prev = true;
    }
    if (!result.settled) return result;

    // Phase 2: ramp the drawbar force.
    const std::size_t window_n = static_cast<std::size_t>(std::lround(kWindowSeconds / dt));
    const std::size_t sat_mean_n = static_cast<std::size_t>(std::lround(kSatMeanSeconds / dt));
    std::deque<real> speed_w, kappa_rear_w, kappa_front_w, rpm_w, theta_w;
    // Trailing kSatMeanSeconds windows for the saturation point.
    std::deque<real> sat_fx_sum_w, sat_force_w, sat_speed_w, sat_h_w, sat_rpm_w, sat_theta_w;
    std::deque<real> sat_kappa_w[4], sat_fx_w[4], sat_fz_w[4];
    auto push_n = [](std::deque<real>& q, real v, std::size_t n) {
        q.push_back(v);
        if (q.size() > n) q.pop_front();
    };
    auto push = [&](std::deque<real>& q, real v) { push_n(q, v, window_n); };
    auto push_sat = [&](std::deque<real>& q, real v) { push_n(q, v, sat_mean_n); };
    constexpr std::size_t kWheels[4] = {kFL, kFR, kRL, kRR};
    real force = 0.0;
    while (force < kMaxForce) {
        force += kRampRatePerSecond * dt;
        rf.set_magnitude(force);
        w.step();
        const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
        if (snap.engine_state != EngineState::Running) ++result.not_running_ticks;
        const real speed = w.get_motion(rig.chassis).linear.x;
        push(speed_w, speed);
        push(kappa_rear_w, 0.5 * (std::abs(w.wheel_telemetry(rig.vehicle, kRL).slip_ratio) +
                                  std::abs(w.wheel_telemetry(rig.vehicle, kRR).slip_ratio)));
        push(kappa_front_w, 0.5 * (std::abs(w.wheel_telemetry(rig.vehicle, kFL).slip_ratio) +
                                   std::abs(w.wheel_telemetry(rig.vehicle, kFR).slip_ratio)));
        push(rpm_w, snap.engine_rpm);
        push(theta_w, theta_gov(snap));

        real driven_fx = w.wheel_telemetry(rig.vehicle, kRL).fx + w.wheel_telemetry(rig.vehicle, kRR).fx;
        if (awd_on) driven_fx += w.wheel_telemetry(rig.vehicle, kFL).fx + w.wheel_telemetry(rig.vehicle, kFR).fx;
        if (driven_fx > result.peak_driven_fx_per_tick) {
            result.peak_driven_fx_per_tick = driven_fx;
            result.limit_force_per_tick = force;
            for (int k = 0; k < 4; ++k) result.fz_per_tick_at_peak[k] = w.wheel_telemetry(rig.vehicle, kWheels[k]).fz;
        }
        push_sat(sat_fx_sum_w, driven_fx);
        push_sat(sat_force_w, force);
        push_sat(sat_speed_w, speed);
        push_sat(sat_h_w, w.get_pose(rig.chassis).position.z);
        push_sat(sat_rpm_w, snap.engine_rpm);
        push_sat(sat_theta_w, theta_gov(snap));
        for (int k = 0; k < 4; ++k) {
            const auto& t = w.wheel_telemetry(rig.vehicle, kWheels[k]);
            push_sat(sat_kappa_w[k], std::abs(t.slip_ratio));
            push_sat(sat_fx_w[k], t.fx);
            push_sat(sat_fz_w[k], t.fz);
        }
        if (sat_fx_sum_w.size() >= sat_mean_n && mean_of_deque(sat_fx_sum_w) > result.peak_driven_fx) {
            result.peak_driven_fx = mean_of_deque(sat_fx_sum_w);
            result.limit_force = mean_of_deque(sat_force_w);
            result.chassis_speed_at_sat = mean_of_deque(sat_speed_w);
            result.chassis_height_at_sat = mean_of_deque(sat_h_w);
            result.accel_at_sat =
                (sat_speed_w.back() - sat_speed_w.front()) / (static_cast<real>(sat_speed_w.size() - 1) * dt);
            for (int k = 0; k < 4; ++k) {
                result.kappa_at_sat[k] = mean_of_deque(sat_kappa_w[k]);
                result.fx_at_sat[k] = mean_of_deque(sat_fx_w[k]);
                result.fz_at_sat[k] = mean_of_deque(sat_fz_w[k]);
            }
            result.min_rpm_at_sat = *std::min_element(sat_rpm_w.begin(), sat_rpm_w.end());
            result.max_theta_gov_at_sat = *std::max_element(sat_theta_w.begin(), sat_theta_w.end());
        }

        if (speed_w.size() >= window_n && mean_of_deque(speed_w) < kMinMeanSpeedFraction * result.v0) {
            result.reached_stop = true;
            result.stop_force = force - kRampRatePerSecond * kWindowSeconds; // force at the start of the failing window
            result.mean_speed_at_stop = mean_of_deque(speed_w);
            result.mean_kappa_rear_at_stop = mean_of_deque(kappa_rear_w);
            result.mean_kappa_front_at_stop = mean_of_deque(kappa_front_w);
            result.mean_rpm_at_stop = mean_of_deque(rpm_w);
            result.min_rpm_at_stop = *std::min_element(rpm_w.begin(), rpm_w.end());
            result.mean_theta_gov_at_stop = mean_of_deque(theta_w);
            return result;
        }
    }
    result.stop_force = force;
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

// Measured (C12 review round 3, 2026-09-26, release, 240/960; static loads
// 10300.6 N front / 19131.2 N rear per wheel, wheelbase 2.8 m, h_cg 1.280 m):
//  - Zero-load settle: v0 = 0.6024 m/s (off, gate passed after 8 s) and
//    0.6044 m/s (on, 6 s), engine 2366 rpm, theta_gov 0.170 - governed in the
//    droop band at the rolling-resistance load. Kinematic prediction at
//    2400 rpm high idle 0.6134 m/s; at the measured 2366 rpm 0.6047 m/s, the
//    rest being rear-tyre slip.
//  - Limit (traction saturation, argmax of the trailing 0.25 s mean of the
//    driven fx sum): off 1666.9 N vs. estimate 1564.6 N (+6.5 %), on
//    3141.9 N vs. 3338.6 N (-5.9 %), ratio 1.885. Why a window mean, not the
//    raw per-tick sum: the per-tick argmax lands 4 N (off) / 12 N (on) away,
//    but with MFWD engaged the per-tick signal spikes - at its argmax tick RL
//    fz reads 21373 N against a 19907 N window mean and the driven fx sum
//    5073 N against 4963 N - so single-tick force/Fz values do not close the
//    chassis force balance there; the window means do (below).
//  - Saturation -> stop (0.5*v0 speed gate at 1882.1 / 3352.9 N): 215.2 N
//    (off) and 211.0 N (on), i.e. ~2.1 s of the 100 N/s ramp.
//  - Gap to the static-Fz estimate, term by term at the saturation window:
//    off, +102.3 N = +59.4 N load transfer (rear axle +742.5 N vs. the
//    quasi-static X*h/L 742.6 N; both rear tyres' peak fx 1703.5 -> 1733.9 N)
//    + 42.9 N chassis inertia (X - F = -42.4 N vs. m*dv/dt -43.5 N at
//    dv/dt -0.0072 m/s^2 - ramp-rate dependent). The rear tyres deliver
//    3467.1 N of their 3467.9 N peak at the measured Fz: saturated.
//    On, -196.7 N = +3.3 N load transfer (front axle -1429.5 N vs. X*h/L
//    1424.5 N; per-wheel peak fx front 887.0 -> 830.4/829.8 N, rear
//    1703.5 -> 1766.9/1762.0 N nearly cancel) - 225.8 N of driven tyres
//    below their peak + 25.8 N inertia (X - F -25.8 N vs. m*dv/dt -29.8 N).
//    The shortfall is the front axle (1444.3 N delivered vs. 1660.2 N peak;
//    the rear is 9.9 N short): the 2 % kinematic lead pins the open front
//    diff's MEAN slip at 0.0344 (rear 0.0135 + 0.0209, desc lead 0.0202),
//    past the 0.015 low_mu peak slip, and the open diff splits it into FL
//    |kappa| 0.061 on the falling side and FR 0.0075 below the peak at equal
//    force (722 N each) - the axle carries twice the falling-side force, not
//    two peaks. So the off gap is load transfer plus inertia; the on gap is
//    the kinematic lead, which a static peak-sum estimate does not model.
//  - Traction discriminator: at saturation engine rpm >= 2362.5 and
//    theta_gov <= 0.187 over the window (torque in reserve, far above the
//    1500 rpm WOT-curve peak); over the stop window mean |kappa| rear 0.551
//    (off), 0.525 rear / 0.619 front (on), vs. a model peak slip of 0.015 at
//    static Fz (sweep resolution 0.0075), engine Running on every ramp tick,
//    2363-2365 rpm, theta_gov 0.175 / 0.183. The limit is traction, not
//    engine.
//  - C9's speed-PI version gave 1740.4 / 3227.9 N, ratio 1.855 (estimates
//    1564.2 / 3338.2 N).
TEST_CASE("tractor MFWD on low_mu: engaged drawbar-pull limit is at least 1.2x the rear-only limit",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor();

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId low_mu_id = surfaces->id_for("low_mu");
    const real lambda_mu = surfaces->lambda_mu_for(low_mu_id);
    const real crr_surface = surfaces->crr_for(low_mu_id);

    const StaticAxleLoads loads = measure_static_axle_loads(desc);
    const DrawbarResult off = drawbar_pull_limit(false);
    const DrawbarResult on = drawbar_pull_limit(true);

    const real v_high_idle = gear1_high_idle_speed(desc);
    const vehicle::Curve1D& wot = engine_desc(desc).wot_torque_nm_vs_rpm;
    const real wot_peak_rpm =
        wot.x[static_cast<std::size_t>(std::distance(wot.y.begin(), std::max_element(wot.y.begin(), wot.y.end())))];
    const real wheelbase = desc.wheels[kFL].attachment_local.x - desc.wheels[kRL].attachment_local.x;

    // Load-transfer bookkeeping at the saturation window (reported, not
    // asserted): the per-wheel Fz the sim measured there; the quasi-static
    // axle transfer X*h/L that the ground longitudinal force X (every wheel's
    // fx minus its rolling resistance at its measured Fz) predicts about the
    // chassis CoM at its measured height h - RearwardForce acts AT the CoM,
    // so it adds no pitch moment of its own; the "hitch height" is the CoM
    // height; the F_est formula of drawbar_force_estimate() re-evaluated at
    // the measured per-wheel Fz; and the chassis force balance X - F = m*a.
    struct SatAnalysis {
        real peak_static[4] = {};         // N, each tyre's peak fx at its static Fz
        real peak_at_sat_fz[4] = {};      // N, each tyre's peak fx at its measured saturation Fz
        real driven_peak_static = 0.0;    // N, sum over driven tyres
        real driven_peak_at_sat_fz = 0.0; // N, sum over driven tyres
        real roll_static = 0.0;           // N, every wheel's rolling resistance at its static Fz
        real roll_at_sat_fz = 0.0;        // N, same at its measured Fz
        real est_at_sat_fz = 0.0;         // N, F_est at the measured per-wheel Fz
        real x_ground = 0.0;              // N, sum of (fx - rolling resistance) over all four wheels
        real fz_total = 0.0;              // N, all four wheels
        real rear_fz_gain = 0.0;          // N, rear axle minus static
        real front_fz_loss = 0.0;         // N, static minus front axle
        real fz_transfer_pred = 0.0;      // N, X*h/L
    };
    auto analyse = [&](const DrawbarResult& r, bool awd_on) {
        SatAnalysis a;
        const real v = r.chassis_speed_at_sat;
        for (int k = 0; k < 4; ++k) {
            const bool front = k < 2;
            const vehicle::TyreDesc& tyre = desc.wheels[front ? kFL : kRL].tyre;
            const real fz_static = front ? loads.front_fz_per_wheel : loads.rear_fz_per_wheel;
            const real roll = tyre_rolling_resistance(tyre, r.fz_at_sat[k], 0.0, v, crr_surface);
            a.roll_static += tyre_rolling_resistance(tyre, fz_static, 0.0, v, crr_surface);
            a.roll_at_sat_fz += roll;
            a.x_ground += r.fx_at_sat[k] - roll;
            a.peak_static[k] = tyre_peak(tyre, fz_static, lambda_mu, v).fx;
            a.peak_at_sat_fz[k] = tyre_peak(tyre, r.fz_at_sat[k], lambda_mu, v).fx;
            if (!front || awd_on) {
                a.driven_peak_static += a.peak_static[k];
                a.driven_peak_at_sat_fz += a.peak_at_sat_fz[k];
            }
            a.fz_total += r.fz_at_sat[k];
        }
        a.est_at_sat_fz = a.driven_peak_at_sat_fz - a.roll_at_sat_fz;
        a.rear_fz_gain = r.fz_at_sat[2] + r.fz_at_sat[3] - 2.0 * loads.rear_fz_per_wheel;
        a.front_fz_loss = 2.0 * loads.front_fz_per_wheel - r.fz_at_sat[0] - r.fz_at_sat[1];
        a.fz_transfer_pred = a.x_ground * r.chassis_height_at_sat / wheelbase;
        return a;
    };

    // INFO lines placed BEFORE the guard REQUIREs so a guard failure still
    // prints the measured fields that explain it (Catch2 only prints INFO
    // messages active at the point of failure).
    INFO("static loads: front_fz_per_wheel=" << loads.front_fz_per_wheel << " N rear_fz_per_wheel="
                                              << loads.rear_fz_per_wheel << " N (lambda_mu=" << lambda_mu
                                              << " crr=" << crr_surface << "), wheelbase " << wheelbase << " m");
    INFO("zero-load settle: off v0=" << off.v0 << " m/s after " << off.settle_time_s << " s (rpm " << off.rpm_v0
                                     << ", theta_gov " << off.theta_gov_v0 << "), on v0=" << on.v0 << " m/s after "
                                     << on.settle_time_s << " s (rpm " << on.rpm_v0 << ", theta_gov " << on.theta_gov_v0
                                     << "); gear-1 high-idle kinematic speed " << v_high_idle << " m/s");
    auto describe = [&](const DrawbarResult& r, bool awd_on) {
        const SatAnalysis a = analyse(r, awd_on);
        auto four = [](const real (&x)[4]) {
            std::ostringstream q;
            q << x[0] << "/" << x[1] << "/" << x[2] << "/" << x[3];
            return q.str();
        };
        std::ostringstream o;
        o << "SATURATION limit=" << r.limit_force << " N (window-mean driven fx " << r.peak_driven_fx
          << " N; per-tick argmax at " << r.limit_force_per_tick << " N, driven fx " << r.peak_driven_fx_per_tick
          << " N, fz " << four(r.fz_per_tick_at_peak) << "), speed " << r.chassis_speed_at_sat << " m/s, dv/dt "
          << r.accel_at_sat << " m/s^2, h_cg " << r.chassis_height_at_sat << " m, rpm min " << r.min_rpm_at_sat
          << ", theta_gov max " << r.max_theta_gov_at_sat << "; FL/FR/RL/RR |kappa| " << four(r.kappa_at_sat)
          << " fx " << four(r.fx_at_sat) << " fz " << four(r.fz_at_sat) << "; tyre peak fx at static Fz "
          << four(a.peak_static) << ", at measured Fz " << four(a.peak_at_sat_fz) << "; total Fz " << a.fz_total
          << " vs. static " << 2.0 * (loads.front_fz_per_wheel + loads.rear_fz_per_wheel)
          << " N; rear-axle Fz gain " << a.rear_fz_gain << " N, front-axle loss " << a.front_fz_loss
          << " N vs. X*h/L " << a.fz_transfer_pred << " N; driven peak sum static " << a.driven_peak_static
          << " N, at measured Fz " << a.driven_peak_at_sat_fz << " N; roll static " << a.roll_static
          << " N, at measured Fz " << a.roll_at_sat_fz << " N; F_est at measured Fz " << a.est_at_sat_fz
          << " N; X - F " << a.x_ground - r.limit_force << " N vs. m*dv/dt " << kChassisMass * r.accel_at_sat
          << " N | STOP (0.5*v0 gate) at " << r.stop_force << " N (gap " << r.stop_force - r.limit_force
          << " N), mean speed " << r.mean_speed_at_stop << " m/s, mean|kappa| rear " << r.mean_kappa_rear_at_stop
          << " front " << r.mean_kappa_front_at_stop << ", rpm mean " << r.mean_rpm_at_stop << " min "
          << r.min_rpm_at_stop << ", theta_gov mean " << r.mean_theta_gov_at_stop << "; not-running ticks "
          << r.not_running_ticks;
        return o.str();
    };
    INFO("MFWD off: " << describe(off, false));
    INFO("MFWD on:  " << describe(on, true));
    REQUIRE(off.settled);
    REQUIRE(on.settled);
    REQUIRE(off.reached_stop);
    REQUIRE(on.reached_stop);

    const real est_off = drawbar_force_estimate(desc, loads, lambda_mu, crr_surface, false, off.v0);
    const real est_on = drawbar_force_estimate(desc, loads, lambda_mu, crr_surface, true, on.v0);
    const TyrePeak rear_peak = tyre_peak(desc.wheels[kRL].tyre, loads.rear_fz_per_wheel, lambda_mu, off.v0);
    const TyrePeak front_peak = tyre_peak(desc.wheels[kFL].tyre, loads.front_fz_per_wheel, lambda_mu, on.v0);
    INFO("estimates: off=" << est_off << " N, on=" << est_on << " N; tyre peak slip at static Fz: rear kappa="
                           << rear_peak.kappa << ", front kappa=" << front_peak.kappa << "; WOT-curve peak at "
                           << wot_peak_rpm << " rpm");
    REQUIRE(off.limit_force > 1.0); // guard the ratio below against a near-zero denominator
    REQUIRE(est_off > 1.0);         // guard the relative-error checks below
    const real ratio = on.limit_force / off.limit_force;
    INFO("ratio (on/off) = " << ratio);

    CHECK(std::abs(off.limit_force - est_off) <= 0.25 * std::abs(est_off));
    CHECK(std::abs(on.limit_force - est_on) <= 0.25 * std::abs(est_on));
    CHECK(ratio >= 1.2);

    // The traction saturation comes before the speed collapse that stops the
    // ramp (a limit read after the stop would be measuring something else).
    CHECK(off.limit_force < off.stop_force);
    CHECK(on.limit_force < on.stop_force);

    // Traction discriminator. At saturation the engine is running on its
    // governor with torque in reserve (theta_gov < 1, rpm above the
    // WOT-curve peak); after it the driven tyres run past their own peak slip
    // while the engine stays there - the collapse is wheelspin, not a lug.
    CHECK(off.not_running_ticks == 0);
    CHECK(on.not_running_ticks == 0);
    CHECK(off.max_theta_gov_at_sat < 1.0);
    CHECK(on.max_theta_gov_at_sat < 1.0);
    CHECK(off.min_rpm_at_sat >= wot_peak_rpm);
    CHECK(on.min_rpm_at_sat >= wot_peak_rpm);
    CHECK(off.mean_kappa_rear_at_stop > rear_peak.kappa);
    CHECK(on.mean_kappa_rear_at_stop > rear_peak.kappa);
    CHECK(on.mean_kappa_front_at_stop > front_peak.kappa);
    CHECK(off.min_rpm_at_stop >= wot_peak_rpm);
    CHECK(on.min_rpm_at_stop >= wot_peak_rpm);
    CHECK(off.mean_theta_gov_at_stop < 1.0); // governor below full rack: engine torque in reserve
    CHECK(on.mean_theta_gov_at_stop < 1.0);
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
