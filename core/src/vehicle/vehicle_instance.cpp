#include "vehicle_instance.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::vehicle {
namespace {

Pose extrapolate_pose(const Pose& p0, const Motion& m0, real t) {
    Pose out;
    out.position = p0.position + m0.linear * t;
    const real w = m0.angular.length();
    if (w > real(1e-9)) {
        out.orientation = (Quat::from_axis_angle(m0.angular / w, w * t) * p0.orientation).normalized();
    } else {
        out.orientation = p0.orientation;
    }
    return out;
}

} // namespace

VehicleInstance::VehicleInstance(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces)
    : chassis_(chassis), surfaces_(surfaces), desc_(desc) {
    wheels_.reserve(desc_.wheels.size());
    for (const WheelDesc& w : desc_.wheels) {
        wheels_.push_back(std::make_unique<WheelInstance>(w, create_pacejka_tyre_model(w.tyre)));
    }

    // Ackermann geometry reference (vehicle_desc.h's SteeringDesc comment):
    // rear axle x = average x of non-front wheels; wheelbase = average
    // front x - rear axle x. Falls back to sane defaults if a vehicle has
    // no front/rear split (e.g. every wheel is_front=false) so construction
    // never divides by zero.
    real rear_sum = 0.0, front_sum = 0.0;
    int rear_n = 0, front_n = 0;
    for (const WheelDesc& w : desc_.wheels) {
        if (w.is_front) { front_sum += w.attachment_local.x; ++front_n; }
        else { rear_sum += w.attachment_local.x; ++rear_n; }
    }
    rear_axle_x_ = rear_n > 0 ? rear_sum / rear_n : 0.0;
    const real front_x = front_n > 0 ? front_sum / front_n : rear_axle_x_ + 2.5;
    wheelbase_ = std::max(real(0.5), front_x - rear_axle_x_);
}

real VehicleInstance::ackermann_wheel_angle(std::size_t wheel_index, real delta_avg) const {
    const WheelDesc& w = desc_.wheels[wheel_index];
    if (!w.steered) return 0.0;
    if (!w.is_front) {
        // Rear steer: proportional, opposite sign (out/handover_p3.md /
        // vehicle_desc.h: a documented simplification, no speed-dependent
        // phase reversal).
        return -desc_.steering.rear_steer_fraction * delta_avg;
    }
    const real dx = w.attachment_local.x - rear_axle_x_;
    const real dy = w.attachment_local.y;
    const real tan_avg = ps::math::tan(delta_avg);
    real delta_ackermann;
    if (std::abs(tan_avg) < real(1e-6)) {
        delta_ackermann = delta_avg;
    } else {
        const real R = wheelbase_ / tan_avg; // turn centre's lateral offset from the rear-axle line
        const real denom = R - dy;
        // atan(dx / denom), NOT atan2(dx, denom): dx (a front wheel's x
        // distance ahead of the rear axle) is ALWAYS POSITIVE, and
        // atan2's result is confined to (0, pi) whenever its first
        // argument is positive - it can never return a negative wheel
        // angle, no matter how hard right lock (negative delta_avg, this
        // project's ISO 8855 convention, +y = left) is applied. That
        // produced a large POSITIVE (left-pointing) front wheel angle at
        // full right lock instead of a small negative one, which yawed
        // the car left, killed forward speed, and eventually drove it
        // backwards - see tests/unit/test_vehicle_ackermann_symmetry.cpp
        // and tests/validation/test_vehicle_steer_mirror.cpp for the
        // regression coverage. atan(dx / denom) is the correct geometric
        // Ackermann form (delta_i = atan(L / (R - dy))): dx > 0 always,
        // so the quotient's sign already carries denom's sign and atan is
        // odd, giving the correct signed small-angle result directly with
        // no four-quadrant ambiguity to get wrong.
        delta_ackermann = std::abs(denom) < real(1e-6) ? delta_avg : ps::math::atan(dx / denom);
    }
    // ackermann_fraction (f, validated to [-1, 1] by ps::io::load_vehicle_json
    // - see vehicle_desc.h's SteeringDesc::ackermann_fraction comment) is
    // negative for anti-Ackermann: rewriting as delta_avg + f*(delta_ackermann
    // - delta_avg) makes it clear this only ever pushes a wheel's angle
    // toward (f>0) or away from (f<0) delta_ackermann relative to the
    // parallel-steer baseline delta_avg - it can flip which wheel (inner vs.
    // outer) gets the larger magnitude, but for any f in the validated range
    // it cannot flip a wheel's SIGN (turning direction) on its own; checked
    // against car_sedan.json at full lock (out/handover_anti_ackermann.md /
    // vault PHYS-025): f=-1 still turns both front wheels toward the
    // commanded direction, it just makes the outer wheel's magnitude exceed
    // the inner's instead of the reverse. The one caveat this does not
    // remove is the SAME geometric singularity full (f=1) Ackermann already
    // has, independent of f's sign: as R (this wheel's turn-centre offset)
    // approaches dy, denom above approaches zero and delta_ackermann itself
    // blows up (guarded by the abs(denom) check above) - an extremely
    // short-wheelbase/wide-track vehicle at a very tight lock could still
    // see large swings here, not specific to f<0.
    const real f = desc_.steering.ackermann_fraction;
    return (real(1.0) - f) * delta_avg + f * delta_ackermann;
}

void VehicleInstance::step_tick(IRigidBackend& backend, const ControlInputs& controls, real tick_dt, real substep_rate_hz) {
    const Pose pose0 = backend.get_pose(chassis_);
    const Motion motion0 = backend.get_motion(chassis_);

    // Steer target, rate-limited (SteeringDesc::steer_rate_limit == 0 means
    // unlimited) - computed once per tick, held across this tick's
    // substeps, same spirit as D3's "external wrench held constant".
    const real steer_target = controls.steer * desc_.steering.max_wheel_angle;
    real delta_avg = steer_target;
    if (desc_.steering.steer_rate_limit > 0.0) {
        const real max_delta = desc_.steering.steer_rate_limit * tick_dt;
        delta_avg = std::clamp(steer_target, prev_steer_avg_ - max_delta, prev_steer_avg_ + max_delta);
    }
    prev_steer_avg_ = delta_avg;

    // Drive/brake torque commands: computed once per tick from each
    // wheel's spin speed at tick start (out/handover_p3.md: a tick is
    // 4.17 ms at 240 Hz, short enough that recomputing per substep would
    // not change the placeholder drive's behaviour materially).
    real driven_omega_sum = 0.0;
    int driven_n = 0;
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        if (desc_.wheels[i].driven) { driven_omega_sum += wheels_[i]->spin_speed(); ++driven_n; }
    }
    real drive_torque_per_wheel = 0.0;
    if (driven_n > 0) {
        const DriveDesc& d = desc_.drive;
        const real wheel_rpm_avg = (driven_omega_sum / driven_n) * real(60.0) / (real(2.0) * real(3.14159265358979323846));
        const real engine_rpm_raw = std::abs(wheel_rpm_avg) * d.final_drive_ratio;
        const real engine_rpm = std::clamp(engine_rpm_raw, d.idle_rpm, d.rev_limit_rpm);
        real engine_torque = d.torque_curve_nm_vs_rpm.evaluate(engine_rpm) * controls.throttle;
        if (engine_rpm_raw > d.rev_limit_rpm) engine_torque = 0.0; // crude rev limiter, PLAN.md M1's placeholder drive
        const real wheel_torque_total = engine_torque * d.final_drive_ratio * d.drivetrain_efficiency;
        drive_torque_per_wheel = wheel_torque_total / driven_n;
    }

    std::vector<real> brake_torque(wheels_.size(), 0.0);
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        const WheelDesc& w = desc_.wheels[i];
        const BrakeDesc& b = desc_.brakes;
        real t = (w.is_front ? b.max_torque_front : b.max_torque_rear) * controls.brake;
        if (!w.is_front) t += b.max_torque_rear * b.handbrake_rear_fraction * controls.handbrake;
        brake_torque[i] = t;
    }

    const std::size_t num_substeps = std::max<std::size_t>(1, static_cast<std::size_t>(tick_dt * substep_rate_hz + real(0.5)));
    const real substep_dt = tick_dt / static_cast<real>(num_substeps);

    Vec3 force_sum = Vec3::zero();
    Vec3 torque_sum = Vec3::zero();

    for (std::size_t s = 0; s < num_substeps; ++s) {
        const real t = static_cast<real>(s) * substep_dt;
        const Pose chassis_pose = extrapolate_pose(pose0, motion0, t);
        const Vec3 com = chassis_pose.position;

        // Pass 1: every wheel's ground contact, before pass 2 (ARB needs
        // the partner's fresh measurement - wheel_instance.h's comment).
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            const real delta = ackermann_wheel_angle(i, delta_avg);
            wheels_[i]->measure_contact(chassis_pose, delta, backend);
        }

        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            const WheelDesc& w = desc_.wheels[i];
            const real delta = ackermann_wheel_angle(i, delta_avg);
            WheelSubstepInput in;
            in.chassis_pose = chassis_pose;
            in.chassis_motion = motion0;
            in.chassis_com_world = com;
            in.steer_angle = delta;
            in.brake_torque_cmd = brake_torque[i];
            in.drive_torque_cmd = w.driven ? drive_torque_per_wheel : 0.0;
            in.dt = substep_dt;
            in.partner_compression = w.arb_partner >= 0 ? wheels_[static_cast<std::size_t>(w.arb_partner)]->compression() : 0.0;
            in.surfaces = surfaces_;

            const WheelSubstepOutput out = wheels_[i]->step(in);
            force_sum += out.suspension_force_world + out.tyre_force_world;
            torque_sum += cross(out.attachment_point_world - com, out.suspension_force_world);
            torque_sum += cross(out.contact_point_world - com, out.tyre_force_world);
            torque_sum += out.extra_torque_world;
        }
    }

    const real n = static_cast<real>(num_substeps);
    backend.apply_wrench(chassis_, Wrench{force_sum / n, torque_sum / n});
}

std::uint64_t VehicleInstance::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& w : wheels_) h = w->fold_state_hash(h);
    return h;
}

} // namespace ps::vehicle
