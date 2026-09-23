#include "wheel_instance.h"

#include "suspension.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::vehicle {
namespace {

real sign(real x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

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
        fy = f.fy;
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
