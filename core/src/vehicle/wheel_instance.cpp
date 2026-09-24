#include "wheel_instance.h"

#include "suspension.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace ps::vehicle {
namespace {

real sign(real x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

// Low-speed lateral carcass damping (PHYS-033 section 4.3, I1 plan section
// 3.6). Physical framing: real carcass/relaxation damping the tyre model's
// relaxation ODE does not otherwise supply at vanishing ROLLING speed - the
// longitudinal channel is saved by rolling resistance (froll below, a
// genuine Coulomb-type drag along the SAME axis as kappa's restoring force,
// nonzero at any speed); the lateral channel has no equivalent, so the
// alpha_eq/chassis-yaw-rate loop is a marginally-stable (zeta -> 0)
// continuous oscillator (PHYS-024's original finding: a plain yaw-rate kick
// at true standstill GROWS into a saturated limit cycle at 240 Hz but
// plainly decays at 4x finer tick rate).
//
// NOT a compensation for a numerical delay (that framing is retired as of
// PHYS-033 I1's W1 shadow chassis - see CLAUDE.md's "Vehicle system"
// section for the current derivation of VehicleInstance::step_tick's
// per-substep wrench). This term supplies the physical damping margin the
// relaxation model itself is missing at low speed; TyreDesc's
// low_speed_damping_ratio/low_speed_damping_vx set its strength and fade,
// per tyre file (ps/vehicle/tyre_desc.h's own comment has the full
// derivation and the explicit-integration stability bound below).
//
// c = 2*ratio*sqrt(k_lat*m_ref), k_lat = pky1*fz0/relaxation_length_y (at
// the tyre's FIXED nominal load fz0, never the live/instantaneous fz -
// PHYS-024's positive-feedback lesson: tying this to live fz turned a
// hard-acceleration weight-transfer transient into a positive-feedback
// spin-out). Faded to zero above low_speed_damping_vx on TOTAL contact-patch
// speed (not just longitudinal) - a wheel with near-zero rolling speed but
// real, fast lateral motion (sliding/spinning/being pushed sideways) must
// not get this drag applied to a genuine fast slide.
//
// Stability (PHYS-033 section 4.3): the damper is explicit (evaluated from
// this substep's own twist), and all four wheels' damping torques sum onto
// shared chassis DOFs (yaw most sensitive, per the section's derivation) -
// a per-vehicle-file check (car_sedan and tractor) that
// gamma = h*sum_i(c_i*(1/M + (r_i x y_i)^T I^-1 (r_i x y_i))) < 1 at the
// configured ratio lives in test_vehicle_shadow_chassis.cpp.
//
// commit B set low_speed_damping_ratio = 1.5 to reproduce the old
// hard-coded kZetaMin bit-for-bit (the expression below intentionally keeps
// that OLD operation order - omega_n, then m_corner, then c - for that
// reason). Commit E re-measured against the W1 shadow chassis (the delay
// kZetaMin was originally sized against no longer exists) and RETAINED 1.5
// - see tyre_desc.h's own field comment and test_vehicle_standstill_
// wiggle.cpp's "zeta sweep diagnostic" for why: two existing acceptance
// tests (full-throttle standing start, parking slope) turned out to be
// non-monotonic in this ratio over [0, 1.5], and 1.5 was the only value
// tried that kept both green simultaneously.
constexpr real kGravityRef = 9.81; // fixed modelling constant, not World::gravity - no World access from this file

real low_speed_lateral_damping_force(const TyreDesc& t, real vx_contact, real vy_contact) {
    if (t.low_speed_damping_ratio <= real(0.0)) return 0.0;
    const real omega_n = std::sqrt(t.pky1 * kGravityRef / std::max(t.relaxation_length_y, real(1e-6)));
    const real m_corner = t.fz0 / kGravityRef;
    const real c_lat = real(2.0) * t.low_speed_damping_ratio * m_corner * omega_n;
    const real speed_total = std::sqrt(vx_contact * vx_contact + vy_contact * vy_contact);
    const real fade = std::clamp(real(1.0) - speed_total / t.low_speed_damping_vx, real(0.0), real(1.0));
    return -c_lat * fade * vy_contact;
}

// PHYS-033 I1b defect 2 fix (decision D-B): assembles the ONE force vector
// (plus a pure couple) the chassis receives from this wheel, replacing the
// old separate suspension-force-along-chassis-axis + tyre-force-along-
// chassis-forward/lateral assembly. `n_load` is the suspension force's
// MAGNITUDE (still measured along the chassis-fixed suspension axis - only
// its point of application and the axis it acts along toward the chassis
// change here, not how it is computed); `f_long`/`f_lat` are Fx+F_roll/Fy.
// Kept BILATERAL exactly like the code this replaces (`axis_world * fz`
// applied whenever hit_ground_, wheel_instance.cpp's old :229): a negative
// n_load (the rebound damper) still pulls the chassis toward the ground
// along -n, so this must not clamp n_load to >= 0 (PHYS-033 I1b plan
// section 9 item 3 flags making this a proper max(F_s, 0) lift-off rule as
// a separate, out-of-scope follow-up).
WheelSubstepOutput assemble_contact_wrench(const ContactFrame& cf, bool hit, real n_load, real f_long, real f_lat,
                                            real mz, real mx, const Vec3& airborne_point) {
    WheelSubstepOutput o;
    if (!hit) {
        o.contact_point_world = airborne_point;
        return o;
    }
    o.contact_force_world = cf.normal * n_load + cf.long_dir * f_long + cf.lat_dir * f_lat;
    o.contact_point_world = cf.point;
    o.extra_torque_world = cf.normal * mz + cf.long_dir * mx;
    return o;
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
        contact_surface_ = hit.surface;

        // PHYS-033 I1b defect 2 fix (decision D-B): the ground-plane contact
        // frame this wheel's forces resolve along, built here (pass 1) and
        // consumed by step() (pass 2/assemble_contact_wrench below). `spin`
        // (the CylinderShape's local Y, ps/backend/shape_desc.h's own
        // convention) is ALWAYS perpendicular to axis_world by construction
        // (steer_q only rotates about axis_local, so it can never rotate the
        // wheel's spin axis to have a component along axis_local) - the
        // fallback branches below rely on that to guarantee a non-degenerate
        // in-plane component even when the hit normal itself is unusable.
        constexpr real kMinNormalDotAxis = real(0.25); // |angle(n, suspension axis)| <= ~75.5 deg; beyond that n is a wall/side hit
        constexpr real kMinInPlane = real(0.2);        // |n projected into the wheel mid-plane|; below it, n is (nearly) the spin axis
        const Vec3 spin = query_pose.orientation.rotate(Vec3::unit_y());
        const Vec3 centre = hit.point; // the swept cylinder's own centre at the hit
        ContactFrame cf;
        Vec3 n = hit.normal;
        bool trusted = !hit.started_penetrating && std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z) &&
                       std::abs(n.length() - real(1)) < real(1e-3) && dot(n, axis_world) >= kMinNormalDotAxis;
        if (!trusted) n = axis_world; // fallback 1: bottomed, self-hit, wall
        Vec3 n_perp = n - spin * dot(n, spin);
        if (n_perp.length() < kMinInPlane) { // fallback 2: side-wall contact
            trusted = false;
            n = axis_world;
            n_perp = n - spin * dot(n, spin); // axis_world _|_ spin by construction, so this is never degenerate
        }
        cf.normal = n;
        cf.normal_fallback = !trusted;
        cf.long_dir = cross(spin, n).normalized(); // |spin x n| = |n_perp| >= kMinInPlane whenever trusted, or exactly 1 in the fallback
        cf.lat_dir = cross(n, cf.long_dir);
        cf.point = centre; // I1b commit C7 (decision D-A) replaces this with the true contact-patch point
        assert(dot(cf.long_dir, query_pose.orientation.rotate(Vec3::unit_x())) > real(0.0)); // sign follows from y x z = x
        contact_frame_ = cf;
    } else {
        hit_ground_ = false;
        x_ = 0.0;
        contact_surface_ = kInvalidSurfaceId;
        // contact_frame_ is left stale on a miss - step() never reads it
        // unless hit_ground_ is true.
    }
}

WheelSubstepOutput WheelInstance::step(const WheelSubstepInput& in) {
    const SuspensionDesc& susp = desc_.suspension;
    const Vec3 axis_local = susp.axis.normalized();

    const real xdot = in.dt > 0.0 ? (x_ - x_prev_) / in.dt : 0.0;
    const real fz = hit_ground_ ? total_suspension_force(susp, x_, xdot, in.partner_compression) : 0.0;

    const Vec3 wheel_center_local = desc_.attachment_local + axis_local * x_;
    const Vec3 wheel_center_world = in.chassis_pose.transform_point(wheel_center_local);

    const real gamma = desc_.static_camber + susp.camber_vs_travel.evaluate(x_);

    real fx = 0.0, fy = 0.0, mz = 0.0, mx = 0.0, froll = 0.0;
    real vx_contact = 0.0, vy_contact = 0.0;

    if (hit_ground_ && fz > 0.0) {
        // PHYS-033 I1b defect 2 fix: contact velocity resolved along the
        // GROUND-plane contact frame (measure_contact's own pass 1), not the
        // chassis-fixed forward/lateral axes the old code used - see
        // ContactFrame's own comment.
        const ContactFrame& cf = contact_frame_;
        const Vec3 r = cf.point - in.chassis_pose.position;
        const Vec3 contact_velocity = in.chassis_motion.velocity_at(r);
        vx_contact = dot(contact_velocity, cf.long_dir);
        vy_contact = dot(contact_velocity, cf.lat_dir);

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
        fy = f.fy + low_speed_lateral_damping_force(tyre_->desc(), vx_contact, vy_contact);
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

    // Bilateral in n_load (fz, even when <= 0 - see assemble_contact_wrench's
    // own comment): matches the old axis_world*fz-whenever-hit_ground_
    // behaviour exactly (:tyre terms fx/fy/mz/mx/froll are already zero
    // unless fz > 0, from the `if` above, as before).
    return assemble_contact_wrench(contact_frame_, hit_ground_, fz, fx + froll, fy, mz, mx, wheel_center_world);
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
