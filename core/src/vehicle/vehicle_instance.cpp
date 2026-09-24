#include "vehicle_instance.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <variant>

namespace ps::vehicle {
namespace {

// PHYS-033 I1 W1 (section 3.2): advances an orientation by one substep's
// angular velocity, exactly Jolt's own AddRotationStep form (and this
// file's old extrapolate_pose's rotation half) - used here with the NEW
// (post-update) angular velocity, matching Jolt's own symplectic order
// (F8: position/orientation integrated AFTER the velocity solve).
Quat integrate_orientation(const Quat& q, const Vec3& w, real h) {
    const real wlen = w.length();
    if (wlen > real(1e-9)) {
        return (Quat::from_axis_angle(w / wlen, wlen * h) * q).normalized();
    }
    return q;
}

} // namespace

VehicleInstance::VehicleInstance(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                                   const ControlChannelIndices& control_indices)
    : chassis_(chassis), surfaces_(surfaces), control_indices_(control_indices), desc_(desc),
      powertrain_(desc_.powertrain, desc_.brake_system, desc_.wheels, desc_.name) {
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

void VehicleInstance::step_tick(IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                                 real tick_dt, real substep_rate_hz) {
    // PLAN.md P4 I0: built locally (stack, no allocation) from this
    // vehicle's own pre-resolved indices - see ControlChannelIndices.
    ControlInputs controls;
    controls.steer = channel_values[control_indices_.steer];
    controls.throttle = channel_values[control_indices_.throttle];
    controls.brake = channel_values[control_indices_.brake];
    controls.handbrake = channel_values[control_indices_.handbrake];

    const Pose pose0 = backend.get_pose(chassis_);
    const Motion motion0 = backend.get_motion(chassis_);
    // PHYS-033 I1 W1: read ONCE per tick, at the tick-start rotation - Jolt
    // itself applies the whole tick's accumulated torque with the inverse
    // inertia at the rotation at STEP START (I1 plan section 2, condition
    // 1), so re-reading this mid-tick (e.g. at each substep's own rotated
    // pose) would silently break the exact-agreement claim below.
    const BodyIntegrationParams ip = backend.get_integration_params(chassis_);

    // Diagnostic only (section 3.5): how far did Jolt's actual chassis
    // velocity end up from the shadow's prediction made at the end of last
    // tick? Zero on the very first tick (no prior prediction exists yet).
    if (shadow_valid_) {
        shadow_dv_error_ = (motion0.linear - shadow_end_pred_.linear).length();
        shadow_dw_error_ = (motion0.angular - shadow_end_pred_.angular).length();
    }

    // External acceleration estimate, held CONSTANT over this whole tick's
    // substeps (F12: World::stage_gather runs after this vehicle's stage,
    // so this is always last tick's gathered force-element wrench - the
    // same one-tick lag today's real apply_wrench-based external forces
    // already have).
    const Wrench ext = ext_wrench_accum_;
    ext_wrench_accum_ = Wrench{};
    const Vec3 a_ext = env.gravity * ip.gravity_factor + ext.force * ip.inv_mass;
    const Vec3 alpha_ext = ip.inv_inertia_world * ext.torque;

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

    const std::size_t num_substeps = std::max<std::size_t>(1, static_cast<std::size_t>(tick_dt * substep_rate_hz + real(0.5)));
    const real substep_dt = tick_dt / static_cast<real>(num_substeps);

    Vec3 force_sum = Vec3::zero();
    Vec3 torque_sum = Vec3::zero();

    // PHYS-033 P4 I2 commit 5b (plan section 8): snapshot the ledger's
    // cumulative fields before this tick's substeps, so end_tick() below can
    // compute this TICK's own balance_error from the deltas since here.
    powertrain_.begin_tick();

    // PHYS-033 I1 W1 (section 3.3): the shadow chassis. pose_s/twist_s is
    // this vehicle's OWN local estimate of the chassis pose/twist at the
    // START of substep s - seeded from this tick's real start state and
    // never read back from the backend mid-tick. Ordering within a substep
    // is load-bearing (I1 plan section 2, claim B): wheels see twist_s
    // (their alpha/kappa relaxation updates using it inside
    // WheelInstance::pre_solve), and only AFTER their forces are assembled
    // does the twist advance - do not reorder pass 1/pre_solve/post_solve,
    // and do not update twist per wheel.
    Pose pose_s = pose0;
    Motion twist_s = motion0;
    for (std::size_t s = 0; s < num_substeps; ++s) {
        const Vec3 com = pose_s.position;

        // Pass 1: every wheel's ground contact, before pre_solve (ARB needs
        // the partner's fresh measurement - wheel_instance.h's comment).
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            const real delta = ackermann_wheel_angle(i, delta_avg);
            wheels_[i]->measure_contact(pose_s, delta, backend);
        }

        // PHYS-033 P4 I2 commit 5b (plan section 7 step 4): pre_solve -
        // powertrain.begin_substep(); components in ARRAY order; wheels in
        // WheelDesc order (each wheel's hub torque-law terms feed straight
        // into the network's Assembly via add_hub_terms).
        powertrain_.begin_substep();
        const drivetrain::ComponentSubstepCtx ctx{substep_dt, controls.throttle, controls.brake, controls.handbrake};
        powertrain_.pre_solve_components(ctx);
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            const WheelDesc& w = desc_.wheels[i];
            const real delta = ackermann_wheel_angle(i, delta_avg);
            WheelPreSolveInput in;
            in.chassis_pose = pose_s;
            in.chassis_motion = twist_s;
            in.chassis_com_world = com;
            in.steer_angle = delta;
            in.dt = substep_dt;
            in.partner_compression = w.arb_partner >= 0 ? wheels_[static_cast<std::size_t>(w.arb_partner)]->compression() : 0.0;
            in.surfaces = surfaces_;

            const real omega_old = powertrain_.hub_omega(i);
            const HubTerms ht = wheels_[i]->pre_solve(in, omega_old);
            powertrain_.add_hub_terms(i, ht.tau0, ht.d);
        }

        // step 5: the network solve (also folds in integrator_loss/rotor_ke,
        // powertrain.h's own comment).
        powertrain_.solve(substep_dt);

        // step 6: post_solve - components in ARRAY order, then wheels in
        // WheelDesc order (each wheel's force feeds the chassis wrench and
        // its own wheel_out/tyre_loss ledger deltas).
        powertrain_.post_solve_components(ctx);

        Vec3 force_s = Vec3::zero();
        Vec3 torque_s = Vec3::zero();
        for (std::size_t i = 0; i < wheels_.size(); ++i) {
            const real omega_new = powertrain_.hub_omega(i);
            const WheelSubstepOutput out = wheels_[i]->post_solve(omega_new);
            powertrain_.add_wheel_energy(out.wheel_out_j, out.tyre_loss_j);
            // PHYS-033 I1b defect 2 fix (decision D-B): ONE force vector per
            // wheel (already resolved along the ground-plane contact frame,
            // WheelInstance::post_solve's own assemble_contact_wrench),
            // acting at ONE point - replaces the old separate suspension-
            // force-at-attachment + tyre-force-at-contact-point assembly,
            // which double-counted nothing but resolved both along the
            // chassis' own (possibly tilted-relative-to-ground) axes.
            force_s += out.contact_force_world;
            torque_s += cross(out.contact_point_world - com, out.contact_force_world) + out.extra_torque_world;
        }
        force_sum += force_s;
        torque_sum += torque_s;

        // Shadow update (I1 plan section 3.3, step 8): velocity FIRST, from
        // THIS substep's own wrench, then pose with the NEW velocity - the
        // same symplectic order Jolt itself uses (F8). Tick-start inv_mass/
        // inv_inertia_world/gravity_factor throughout (ip, read once above);
        // no gyroscopic term (I1 plan section 2, claim A item 2 - Jolt
        // applies none by default and this backend never enables it).
        twist_s.linear += substep_dt * (force_s * ip.inv_mass + a_ext);
        twist_s.angular += substep_dt * (ip.inv_inertia_world * torque_s + alpha_ext);
        pose_s.position += twist_s.linear * substep_dt;
        pose_s.orientation = integrate_orientation(pose_s.orientation, twist_s.angular, substep_dt);
    }

    // PHYS-033 P4 I2 commit 5b (plan section 8): this tick's balance_error,
    // from the ledger deltas since begin_tick() above.
    powertrain_.end_tick();

    // Predict what Jolt will report as this chassis's velocity next tick
    // (diagnostic only, section 3.5) - includes Jolt's own end-of-step
    // damping factor (F6: applied ONCE, after the force update), which the
    // shadow otherwise ignores intra-tick (I1 plan section 2, claim A item
    // 4: an O(c*dt) error, negligible for any damping coefficient this
    // project's data uses).
    shadow_end_pred_.linear = twist_s.linear * std::max(real(0.0), real(1.0) - ip.linear_damping * tick_dt);
    shadow_end_pred_.angular = twist_s.angular * std::max(real(0.0), real(1.0) - ip.angular_damping * tick_dt);
    shadow_valid_ = true;

    const real n = static_cast<real>(num_substeps);
    backend.apply_wrench(chassis_, Wrench{force_sum / n, torque_sum / n});
}

std::uint64_t VehicleInstance::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& w : wheels_) h = w->fold_state_hash(h);
    // PHYS-033 P4 I2 commit 5b (plan section 6): the shaft network's own
    // state (hub omega and every other node's, friction-row stick/sets,
    // topology state, each lossy gear's lambda_mesh_prev) - see
    // powertrain.h's fold_state_hash.
    h = powertrain_.fold_state_hash(h);
    return h;
}

} // namespace ps::vehicle
