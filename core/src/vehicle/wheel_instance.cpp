#include "wheel_instance.h"

#include "suspension.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::vehicle {
namespace {

real sign(real x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

// Low-speed lateral carcass damping (PHYS wiggle fix, see CLAUDE.md's
// vehicle-system section for the full derivation and measured numbers).
//
// IMPORTANT - this is a COMPENSATION, not a repair. step_slip_relaxation's
// own decay rate (a = vx_mag/sigma) is shared by both slip channels and
// vanishes as the wheel's ROLLING speed -> 0. The longitudinal channel is
// saved from the consequences of that by rolling resistance (froll below -
// a genuine Coulomb-type drag along the SAME axis as kappa's restoring
// force, still nonzero at vanishing speed); the lateral channel has no
// equivalent, making the alpha_eq/chassis-yaw-rate loop a marginally-stable
// (zeta -> 0) CONTINUOUS oscillator. Measured (test_vehicle_standstill_
// wiggle.cpp): a plain yaw-rate kick at true standstill, no steering, no
// forward motion, does NOT merely ring down slowly - it GROWS into a
// saturated ~5.4 Hz / ~0.92 rad/s peak-to-peak limit cycle by t~8s at the
// normal 240 Hz tick rate, but plainly DECAYS under the identical initial
// condition at 4x finer tick rate. That confirms the growth is a NUMERICAL
// instability, not a physical one: VehicleInstance::step_tick's
// one-tick-delayed force application (chassis motion held at tick-start
// motion0 across all 4 substeps, wrench applied once per 240 Hz tick) turns
// the continuous system's zero damping margin negative.
//
// The term below does NOT fix that delay - the delay is still there. It
// gives the CONTINUOUS system enough real damping margin that the same
// delay can no longer tip it negative (delay eats into a real margin
// instead of a zero one). kZetaMin (below) is therefore sized against THIS
// SPECIFIC 240 Hz one-tick delay, not derived purely from tyre/carcass
// physics - it is an empirically-bounded compensating constant, not a
// measured physical damping coefficient. If VehicleInstance::step_tick's
// application delay is ever fixed properly (e.g. applying the wrench once
// per SUBSTEP against an updated chassis velocity instead of once per tick
// against a tick-start snapshot, or integrating the lateral relaxation
// implicitly), THIS TERM MUST BE REVISITED, not left stacked on top - a
// properly-fixed delay may need little or none of this compensation, and
// an unnecessarily strong damper left in place after that fix would still
// cost the aggregate-yaw-mode stability margin derived below for no reason.
// That delay fix was considered for this bug and rejected as too large in
// scope here: it touches the tick/substep pipeline every vehicle validation
// test (skidpad tuned to 0.34%, the braking [!mayfail] numbers, step-steer,
// accel-standing-start) is measured against, would very likely move every
// vehicle scenario's state_hash (not just near-standstill ones, unlike this
// additive term), and is a substantially bigger, riskier change than a
// time-boxed bug fix should carry. It is plausibly the better long-term
// fix; flagged to the task's coordinator as its own PLAN.md item rather
// than attempted here.
//
// c_lat is derived from a per-wheel single-DOF estimate of the
// alpha_eq/lateral-force natural frequency: omega_n = sqrt(Ky/(m*sigma_y))
// with Ky = fz0*pky1 (pky2 == 0 for every tyre file in this repo today, so
// Ky is EXACTLY linear in load - re-derive if a future tyre file gives
// pky2 != 0). The reference mass is tied to the tyre's own FIXED nominal
// load, m_corner = fz0/kGravityRef (kGravityRef is a fixed modelling
// constant here, not the sim's actual gravity - no World access from this
// file; fz0, not the live/instantaneous fz - see the function's own
// comment below for why that distinction is load-bearing, found via a real
// regression). omega_n and c_lat/m_corner are therefore fixed per tyre
// file, independent of whatever the live fz happens to be doing at
// runtime, including at a hard bump-stop impact.
//
// That single-wheel check is NOT the binding constraint, though, and
// picking kZetaMin from it alone is unsafe - found empirically, not just
// derived: kZetaMin=2.5 (which the single-wheel check alone still called
// "comfortable") made the wiggle MUCH WORSE than no fix at all (amplitude
// exceeding the original bug). Reason: all four wheels' damping FORCES sum
// onto the same shared YAW degree of freedom as a TORQUE (each wheel's
// contribution weighted by its own moment arm r_i from the chassis CoM),
// so the aggregate rotational-mode damping is sum_i(c_lat_i * r_i^2), not
// a single wheel's c_lat alone - and because a real chassis's mass sits
// closer to the CoM than "concentrated at the 4 wheels" (r^2 = 2.43 m^2 vs
// I_yaw/M = 0.81 m^2 for car_sedan, a factor of 3), the YAW mode is far
// more sensitive to this damper than the lateral-TRANSLATION mode is (for
// which the aggregate check collapses back to exactly the single-wheel
// case, m_corner and M cancelling identically). The correct bound is the
// AGGREGATE explicit-integration check for the yaw mode:
//   (2*kZetaMin*omega_n*sum_i(m_corner_i*r_i^2) / I_yaw) * dt < threshold
// For car_sedan (I_yaw estimated as a rectangular-plate approximation,
// M*(wheelbase^2+track^2)/12 = 1215 kg*m^2): ratio=1 (explicit-Euler
// multiplier 0, i.e. NO sign flip between ticks) at kZetaMin~=1.24;
// ratio=2 (multiplier -1, i.e. true divergence) at kZetaMin~=2.47 -
// matching the observed kZetaMin=2.5 blowup almost exactly. tractor's own
// bound is looser (~2.27 at ratio=1, larger I_yaw/M for its size) so
// car_sedan is the binding case across the data files this repo ships
// today; re-run this check for any future vehicle file.
//
// ratio=1 is NOT the line between "clean" and "marginal" - read the
// explicit-Euler multiplier (1-ratio), not just which side of ratio=1 a
// value falls on: |multiplier| (how much a tick's correction shrinks by)
// is SMALLEST, i.e. decay is FASTEST, right at ratio=1, and only grows
// toward the dangerous side as ratio approaches 2. kZetaMin=1.5 gives
// ratio=1.213, multiplier=-0.213 - an alternating-sign correction decaying
// by ~79% EVERY 4.17 ms TICK, invisible at any observable timescale and
// still a materially faster theoretical decay than e.g. kZetaMin=0.7's
// ratio=0.566/multiplier=+0.434 (monotonic, but slower). This single-mode
// estimate also does not capture the whole real system - it is an
// aggregate-YAW-ONLY linearization, and kZetaMin=1.0 (ratio=0.809, on the
// "monotonic" side of this same estimate) measurably left a slow secondary
// regrowth bump in test_vehicle_standstill_wiggle.cpp's telemetry that this
// single mode does not predict, evidence of at least one further, more
// weakly-damped mode this estimate does not model. kZetaMin=1.5 was the
// best of {0.7, 1.0, 1.5, 2.5} actually tried against that telemetry (0.7
// and 1.0 left a slow secondary regrowth within the 12-24 s observation
// window; 1.5 decayed with NO regrowth at all, confirmed over 24 s; 2.5
// diverged) - chosen from measurement, with the analytical bound used only
// to sanity-check the ORDER of magnitude (it correctly flagged 2.5 as
// unsafe) rather than to pick the exact value.
//
// Faded by TOTAL contact-patch speed (not just the longitudinal component)
// down to zero by kFadeSpeedMps: a wheel with near-zero rolling speed but
// real, fast LATERAL motion (sliding, spinning, being pushed sideways)
// must not get this drag applied to a genuinely fast slide - keying the
// fade on longitudinal speed alone would do exactly that. kFadeSpeedMps
// (2 m/s) is well below every other vehicle validation test's operating
// speed (skidpad/step-steer/braking all run well above it), so this term
// is a strict no-op there.
constexpr real kGravityRef = 9.81;
constexpr real kZetaMin = 1.5;
constexpr real kFadeSpeedMps = 2.0;

// fz0 (the tyre's own FIXED reference/nominal load, TyreDesc::fz0), NOT the
// live/instantaneous fz, feeds m_corner here - found necessary, not just a
// simplification: an earlier version used the live fz (which does cancel
// analytically out of Ky/m_corner for a QUASI-STATIC load, the "load-
// independent" property this file used to claim), but c_lat then scaled
// linearly with fz EVERY TICK, including mid-transient. A hard full-
// throttle standing start on a FWD car transfers load violently between
// front corners as the suspension reacts (measured: fz spiking to ~41 kN,
// ~11x nominal, within a few ticks) - tying the damper's own strength to
// that swinging live fz turned it into a positive-feedback loop (bigger
// transient load -> stronger damper -> bigger force -> more roll -> bigger
// load spike -> ...) that blew up test_vehicle_accel_standing_start.cpp's
// asphalt case into a multi-rotation spin-out within ~0.3 s
// (test_vehicle_standstill_wiggle.cpp's diagnostic reproduced this exactly
// and confirmed causation by disabling the term). Using the FIXED fz0
// instead breaks that feedback path entirely - the damper's strength no
// longer depends on anything this force loop itself influences - at the
// cost of the load-independence argument now only holding approximately
// (fz0 is close to each wheel's typical static corner load for both
// vehicle files in this repo, so kZetaMin's tuning against the near-static
// standstill-wiggle telemetry still applies).
real lateral_standstill_damping_force(real fz0, real pky1, real sigma_y, real vx_contact, real vy_contact) {
    const real omega_n = std::sqrt(pky1 * kGravityRef / std::max(sigma_y, real(1e-6)));
    const real m_corner = fz0 / kGravityRef;
    const real c_lat = real(2.0) * kZetaMin * m_corner * omega_n;
    const real speed_total = std::sqrt(vx_contact * vx_contact + vy_contact * vy_contact);
    const real fade = std::clamp(real(1.0) - speed_total / kFadeSpeedMps, real(0.0), real(1.0));
    return -c_lat * fade * vy_contact;
}

// FNV-1a over a POD value's bytes - same convention as
// articulation::Instance::fold_state_hash (core/src/articulation/instance.cpp),
// so World::state_hash's fold order is uniform across subsystems.
template <typename T>
std::uint64_t fnv1a(std::uint64_t h, const T& v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

WheelInstance::WheelInstance(WheelDesc desc, std::unique_ptr<ITyreModel> tyre_model)
    : desc_(std::move(desc)), tyre_(std::move(tyre_model)) {}

void WheelInstance::measure_contact(const Pose& chassis_pose, real steer_angle, const IRigidBackend& backend) {
    const SuspensionDesc& susp = desc_.suspension;
    const Vec3 axis_local = susp.axis.normalized();
    const Vec3 axis_world = chassis_pose.orientation.rotate(axis_local);

    // Cylinder's local Y is its spin axis (ps/backend/shape_desc.h's
    // CylinderShape convention); steer rotates the spin axis about the
    // (approximately vertical) suspension axis before the chassis
    // orientation is applied.
    const Quat steer_q = Quat::from_axis_angle(axis_local, steer_angle);
    CylinderShape shape{desc_.wheel_width * real(0.5), desc_.wheel_radius};

    const Vec3 bump_pos_local = axis_local * susp.travel_max + desc_.attachment_local;
    Pose query_pose;
    query_pose.position = chassis_pose.transform_point(bump_pos_local);
    query_pose.orientation = chassis_pose.orientation * steer_q;

    const Vec3 sweep_world = -axis_world * susp.travel_max;
    const ShapeCastHit hit = backend.shape_cast(shape, query_pose, sweep_world, susp.travel_max);

    if (hit.hit) {
        hit_ground_ = true;
        x_ = std::clamp(susp.travel_max * (real(1.0) - hit.fraction), real(0.0), susp.travel_max);
        contact_point_world_ = hit.point;
        contact_surface_ = hit.surface;
    } else {
        hit_ground_ = false;
        x_ = 0.0;
        contact_surface_ = kInvalidSurfaceId;
    }
}

WheelSubstepOutput WheelInstance::step(const WheelSubstepInput& in) {
    const SuspensionDesc& susp = desc_.suspension;
    const Vec3 axis_local = susp.axis.normalized();
    const Vec3 axis_world = in.chassis_pose.orientation.rotate(axis_local);
    const Quat steer_q = Quat::from_axis_angle(axis_local, in.steer_angle);
    const Vec3 forward_local = steer_q.rotate(Vec3::unit_x());
    const Vec3 lateral_local = steer_q.rotate(Vec3::unit_y());
    const Vec3 forward_world = in.chassis_pose.orientation.rotate(forward_local);
    const Vec3 lateral_world = in.chassis_pose.orientation.rotate(lateral_local);

    const real xdot = in.dt > 0.0 ? (x_ - x_prev_) / in.dt : 0.0;
    const real fz = hit_ground_ ? total_suspension_force(susp, x_, xdot, in.partner_compression) : 0.0;

    const Vec3 wheel_center_local = desc_.attachment_local + axis_local * x_;
    const Vec3 wheel_center_world = in.chassis_pose.transform_point(wheel_center_local);

    WheelSubstepOutput out;
    out.suspension_force_world = axis_world * fz;
    out.attachment_point_world = wheel_center_world;

    const real gamma = desc_.static_camber + susp.camber_vs_travel.evaluate(x_);

    real fx = 0.0, fy = 0.0, mz = 0.0, mx = 0.0, froll = 0.0;
    real vx_contact = 0.0, vy_contact = 0.0;

    if (hit_ground_ && fz > 0.0) {
        const Vec3 contact_point = contact_point_world_;
        const Vec3 r = contact_point - in.chassis_pose.position;
        const Vec3 contact_velocity = in.chassis_motion.velocity_at(r);
        vx_contact = dot(contact_velocity, forward_world);
        vy_contact = dot(contact_velocity, lateral_world);

        const real slip_velocity_x = omega_ * desc_.wheel_radius - vx_contact;
        const real slip_velocity_y = -vy_contact; // out/handover_p3.md: sign chosen so the relaxed alpha matches the SAE/Pacejka convention (restoring Fy)
        const real vx_mag = std::abs(vx_contact);
        step_slip_relaxation(relax_, slip_velocity_x, slip_velocity_y, vx_mag,
                              tyre_->desc().relaxation_length_x, tyre_->desc().relaxation_length_y, in.dt);

        const real lambda_mu = in.surfaces ? in.surfaces->lambda_mu_for(contact_surface_) : real(1.0);
        const real crr_surface = in.surfaces ? in.surfaces->crr_for(contact_surface_) : real(0.0);

        TyreSlipInput slip_in;
        slip_in.kappa = relax_.kappa_eq;
        slip_in.alpha = relax_.alpha_eq;
        slip_in.fz = fz;
        slip_in.gamma = gamma;
        slip_in.vx = vx_contact;
        slip_in.lambda_mu = lambda_mu;
        slip_in.crr_surface = crr_surface;
        const TyreForces f = tyre_->evaluate(slip_in);
        fx = f.fx;
        fy = f.fy + lateral_standstill_damping_force(tyre_->desc().fz0, tyre_->desc().pky1,
                                                       tyre_->desc().relaxation_length_y, vx_contact, vy_contact);
        mz = f.mz;
        mx = f.mx;

        // Rolling resistance: a drag FORCE at the contact patch (matches
        // data/surfaces/surfaces.json's own documented convention,
        // "F_roll = crr * N"), not folded into the slip-based Fx above -
        // it is a hysteresis loss, not a slip force, and vanishes at
        // standstill (sign(0) = 0) rather than fighting the kappa_eq
        // deflection spring that already holds a parked car (see
        // out/handover_p3.md decision 2).
        const real crr_total = tyre_->desc().qsy1 + crr_surface + tyre_->desc().qsy2 * std::abs(f.fx_pure) / tyre_->desc().fz0 +
                                tyre_->desc().qsy4 * ps::math::pow(vx_contact / tyre_->desc().v0, real(4.0));
        froll = -sign(vx_contact) * fz * crr_total;
    }

    out.tyre_force_world = forward_world * (fx + froll) + lateral_world * fy;
    out.contact_point_world = hit_ground_ ? contact_point_world_ : wheel_center_world;
    out.extra_torque_world = axis_world * mz + forward_world * mx;

    // PLAN.md M1, task 2 of the P3 fix-up phase: integrate the render-only
    // spin angle using THIS substep's start-of-step omega_ (still the old
    // value here, before the spin-integration block below overwrites it) -
    // a plain explicit-Euler accumulation is enough for a value nothing
    // else ever reads back (wheel_instance.h's own comment on why this is
    // excluded from fold_state_hash).
    spin_angle_ += omega_ * in.dt;
    last_steer_angle_ = in.steer_angle;

    // --- wheel spin integration (substep) ---
    const real tyre_spin_torque = -fx * desc_.wheel_radius; // reaction of Fx on the wheel's own rotation
    const real omega_dot = (in.drive_torque_cmd + tyre_spin_torque) / std::max(desc_.wheel_inertia, real(1e-6));
    real omega_after = omega_ + omega_dot * in.dt;
    const real max_brake_delta = in.brake_torque_cmd * in.dt / std::max(desc_.wheel_inertia, real(1e-6));
    if (std::abs(omega_after) <= max_brake_delta) {
        omega_after = 0.0;
    } else {
        omega_after -= sign(omega_after) * max_brake_delta;
    }
    omega_ = omega_after;

    x_prev_ = x_;

    last_telemetry_.slip_ratio = relax_.kappa_eq;
    last_telemetry_.slip_angle = relax_.alpha_eq;
    last_telemetry_.fx = fx;
    last_telemetry_.fy = fy;
    last_telemetry_.fz = fz;
    last_telemetry_.omega = omega_;
    last_telemetry_.compression = x_;
    last_telemetry_.surface = contact_surface_;

    return out;
}

std::uint64_t WheelInstance::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    h = fnv1a(h, x_);
    h = fnv1a(h, omega_);
    h = fnv1a(h, relax_.kappa_eq);
    h = fnv1a(h, relax_.alpha_eq);
    return h;
}

} // namespace ps::vehicle
