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
      powertrain_(desc_.powertrain, desc_.brake_system, desc_.wheels, desc_.controllers, desc_.name) {
    wheels_.reserve(desc_.wheels.size());
    for (const WheelDesc& w : desc_.wheels) {
        wheels_.push_back(std::make_unique<WheelInstance>(w, create_pacejka_tyre_model(w.tyre)));
    }
    hub_omega_old_scratch_.assign(desc_.wheels.size(), real(0.0));
    // PHYS-033 P4 I4 C3: sized once here (control_indices_.component's size
    // is fixed for this vehicle's whole lifetime), never resized again -
    // step_tick's per-tick refill below is then allocation-free.
    component_channel_values_.assign(control_indices_.component.size(), real(0.0));

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

    // PHYS-033 P4 I3 plan section 7.1: the shaft-network powertrain's own
    // channels, read once per tick (their per-substep CONSUMPTION is gated
    // by ComponentSubstepCtx::first_substep_of_tick inside Powertrain::
    // pre_solve_components - plan section 6.5.1 - so re-reading the same
    // tick-granularity values every substep here is harmless).
    const real clutch_pedal = channel_values[control_indices_.clutch];
    const real gear_request = channel_values[control_indices_.gear_request];
    const real shift_up_count = channel_values[control_indices_.shift_up_count];
    const real shift_down_count = channel_values[control_indices_.shift_down_count];
    const real ignition = channel_values[control_indices_.ignition];
    const real starter = channel_values[control_indices_.starter];
    const bool assist_auto_clutch = channel_values[control_indices_.assist_auto_clutch] > real(0.5);
    const bool assist_auto_blip = channel_values[control_indices_.assist_auto_blip] > real(0.5);
    const bool assist_auto_shift = channel_values[control_indices_.assist_auto_shift] > real(0.5);

    // PHYS-033 P4 I4 C3 (plan section 2.4): every generic component-declared
    // channel's value, copied once per tick (not per substep, same
    // granularity as the reads just above) - component_channel_values_ is
    // already sized to control_indices_.component.size() from the
    // constructor, so this never allocates.
    for (std::size_t i = 0; i < control_indices_.component.size(); ++i) {
        component_channel_values_[i] = channel_values[control_indices_.component[i]];
    }

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
        drivetrain::ComponentSubstepCtx ctx;
        ctx.h = substep_dt;
        ctx.throttle = controls.throttle;
        ctx.brake = controls.brake;
        ctx.handbrake = controls.handbrake;
        ctx.ambient = env.ambient; // PHYS-033 P4 I7b C9
        ctx.clutch_pedal = clutch_pedal;
        ctx.gear_request = gear_request;
        ctx.shift_up_count = shift_up_count;
        ctx.shift_down_count = shift_down_count;
        ctx.ignition = ignition;
        ctx.starter = starter;
        ctx.assist_auto_clutch = assist_auto_clutch;
        ctx.assist_auto_blip = assist_auto_blip;
        ctx.assist_auto_shift = assist_auto_shift;
        ctx.first_substep_of_tick = (s == 0);
        // PHYS-033 P4 I4 C3: points at this tick's already-copied component-
        // channel buffer (see the per-tick copy above) - rebuilt each
        // substep alongside the rest of ctx, but the buffer itself is only
        // refilled once per tick.
        ctx.channels = component_channel_values_.data();
        ctx.channel_count = static_cast<int>(component_channel_values_.size());
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
            hub_omega_old_scratch_[i] = omega_old; // PHYS-033 P4 I3 plan section 6.7: needed after solve() below
            const HubTerms ht = wheels_[i]->pre_solve(in, omega_old);
            powertrain_.add_hub_terms(i, ht.tau0, ht.d);
        }

        // step 5: the network solve (also folds in integrator_loss/rotor_ke
        // and captures each rotor port's alpha, powertrain.h's own comment).
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

            // PHYS-033 P4 I3 plan section 6.7: this hub's own rotor-reaction
            // contribution, using the wheel's WORLD spin axis (unlike the
            // fixed chassis-local axes powertrain_'s own rotor_ports_ use) -
            // Powertrain::rotor_port_* never sees wheel hubs (Phase M of its
            // constructor skips them, always-zero axis).
            if (powertrain_.rotor_reaction_enabled()) {
                const real alpha = (omega_new - hub_omega_old_scratch_[i]) / substep_dt;
                torque_s += -powertrain_.hub_inertia(i) * alpha * wheels_[i]->spin_axis_world();
            }
        }

        // PHYS-033 P4 I3 plan section 6.6: aero drag, a single bluff element
        // at the chassis CoM (F_drag = -0.5*rho*CdA*|v|*v) - added directly
        // here (not an IForceElement, this file's own top area / plan
        // section 6.6's "why" note) so it is exact within the W1 shadow.
        if (desc_.aero.drag_area_m2 > real(0.0)) {
            const Vec3 v = twist_s.linear;
            const real speed = v.length();
            force_s += v * (-real(0.5) * env.air_density * desc_.aero.drag_area_m2 * speed);
        }

        // PHYS-033 P4 I3 plan section 6.7: rotor reaction from every non-hub
        // rotating port (engine crank, clutch in/out, gearbox in/out, diff
        // carrier) - fixed chassis-local axes, rotated to world here.
        if (powertrain_.rotor_reaction_enabled()) {
            Vec3 t_react_local = Vec3::zero();
            for (std::size_t i = 0; i < powertrain_.rotor_port_count(); ++i) {
                t_react_local += -powertrain_.rotor_port_inertia(i) * powertrain_.rotor_port_alpha(i) * powertrain_.rotor_port_axis_local(i);
            }
            torque_s += pose_s.orientation.rotate(t_react_local);
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

    // PHYS-033 P4 I3 plan section 4.3/6.1.8/6.5.2/7.6 (commit 6 scope: "...
    // telemetry, snapshot fill"): fill this tick's PowertrainSnapshot from
    // the powertrain's real state (last substep's cached results - the whole
    // tick already agrees on gear/clutch/engine state by construction, since
    // gear_request/counters are consumed only once per tick, plan section
    // 6.5.1). World::powertrain_state/WorldSnapshot::vehicles[i].powertrain
    // both read this. World::powertrain_debug() stays a stub (last_engine_
    // result()'s throttle_eff/idle_integrator and the gearbox/diff mesh
    // torques are commit 7's job per powertrain.h's own comment).
    {
        drivetrain::PowertrainSnapshot& snap = powertrain_state_;
        if (powertrain_.has_gearbox()) {
            const auto& gb = powertrain_.gearbox();
            snap.gear = static_cast<std::int8_t>(gb.current_position());
            snap.gear_target = static_cast<std::int8_t>(gb.target());
            if (powertrain_.has_tcu() && powertrain_.tcu_clutch_mode() == drivetrain::ManualTcu::ClutchMode::Shift) {
                switch (gb.phase()) {
                    case drivetrain::Gearbox::Phase::Idle:
                        // ClutchOpening precedes the gearbox's own phases
                        // (waiting for e_auto -> 0); ClutchClosing follows
                        // them (dog-in already done, current == target,
                        // e_auto ramping back to 1) - plan section 6.5.2.
                        snap.shift_phase = (gb.current_position() != gb.target()) ? drivetrain::ShiftPhase::ClutchOpening
                                                                                   : drivetrain::ShiftPhase::ClutchClosing;
                        break;
                    case drivetrain::Gearbox::Phase::Releasing:
                        snap.shift_phase = drivetrain::ShiftPhase::Releasing;
                        break;
                    case drivetrain::Gearbox::Phase::Selecting:
                        snap.shift_phase = drivetrain::ShiftPhase::Selecting;
                        break;
                    case drivetrain::Gearbox::Phase::Synchronising:
                        snap.shift_phase = drivetrain::ShiftPhase::Synchronising;
                        break;
                }
            } else {
                snap.shift_phase = drivetrain::ShiftPhase::Idle;
            }
        } else {
            snap.gear = 0;
            snap.gear_target = 0;
            snap.shift_phase = drivetrain::ShiftPhase::Idle;
        }

        if (powertrain_.has_engine()) {
            const auto& pre = powertrain_.last_engine_result();
            snap.engine_rpm = std::abs(powertrain_.engine_omega()) * (real(60.0) / (real(2.0) * real(3.14159265358979323846)));
            switch (pre.state) {
                case drivetrain::EngineTorqueMap::EngineState::Off: snap.engine_state = drivetrain::EngineState::Off; break;
                case drivetrain::EngineTorqueMap::EngineState::Stalled: snap.engine_state = drivetrain::EngineState::Stalled; break;
                case drivetrain::EngineTorqueMap::EngineState::Cranking: snap.engine_state = drivetrain::EngineState::Cranking; break;
                case drivetrain::EngineTorqueMap::EngineState::Running: snap.engine_state = drivetrain::EngineState::Running; break;
            }
        } else {
            snap.engine_rpm = real(0.0);
            snap.engine_state = drivetrain::EngineState::Off;
        }

        // Speedo (plan section 7.6): gearbox-output-derived, mean radius of
        // the diff's driven hubs (not the chassis' own velocity - it reads
        // the transmission output, like a real speedometer, and keeps
        // turning with the wheels in Neutral). Same formula the TCU uses as
        // its launch-vs-shift speed proxy (powertrain.h's approx_speed_mps).
        snap.speedo_mps = powertrain_.approx_speed_mps();

        snap.clutch_engagement = powertrain_.has_clutch() ? powertrain_.clutch_engagement() : real(1.0);
        snap.assist_active_bits = (assist_auto_clutch ? 1u : 0u) | (assist_auto_blip ? 2u : 0u) | (assist_auto_shift ? 4u : 0u);

        snap.engines.clear();
        if (powertrain_.has_engine()) {
            const auto& eng = powertrain_.engine();
            const auto& pre = powertrain_.last_engine_result();
            drivetrain::EngineSoundState ess;
            ess.crank_angle = eng.crank_angle();
            ess.omega = powertrain_.engine_omega();
            // PHYS-033 P4 I7a C2: theta_c (the post-limiter combustion
            // fraction, engine_torque_map.cpp's own `theta_c = comb ?
            // theta_eff*f_lim : 0`) is now published directly on
            // PreSolveResult - no more DFCO-gated throttle_eff proxy.
            ess.load = pre.theta_c;
            ess.cut_mask = pre.limiter_active || pre.dfco_active ? ~std::uint64_t(0) : std::uint64_t(0);
            ess.engine_state = snap.engine_state;
            ess.starter_active = pre.starter_active;
            ess.limiter_active = pre.limiter_active;
            ess.dfco_active = pre.dfco_active;
            // PHYS-033 P4 I7a C3: the throttle-command stage's combined
            // command this tick - sound only, never hashed.
            ess.throttle_command = powertrain_.throttle_command();
            // PHYS-033 P4 I7a C5, broadened C5d: nonzero for a fuel map or
            // the crude no-map estimate, 0 for an engine with no fuel model
            // at all (pre's own default, pre_solve never touches it then).
            ess.fuel_flow_g_s = pre.fuel_flow_g_s;
            // PHYS-033 P4 I7b C12: mean-value engine sound state (0 from the
            // graph engine, whose PreSolveResult leaves these at default).
            ess.throttle_position = pre.throttle_position;
            ess.p_intake_pa = pre.p_intake_pa;
            ess.lambda = pre.lambda;
            ess.spark_advance_deg = pre.spark_advance_deg;
            ess.exhaust_mass_flow_kg_s = pre.exhaust_mass_flow_kg_s;
            snap.engines.push_back(ess);
        }

        drivetrain::DrivelineSoundState& dl = snap.driveline;
        dl.gearbox_in_omega = powertrain_.gearbox_in_omega();
        dl.gearbox_out_omega = powertrain_.gearbox_out_omega();
        dl.gear_load_torque = real(0.0); // deferred with PowertrainDebug's mesh torques (commit 7)
        dl.clutch_slip = powertrain_.has_clutch() ? powertrain_.clutch_slip() : real(0.0);
        dl.clutch_torque = powertrain_.clutch_torque();
        dl.gear = snap.gear;
        dl.grind_active = powertrain_.has_gearbox() ? powertrain_.last_gearbox_result().grind_active : false;
        dl.grind_events = powertrain_.has_gearbox() ? powertrain_.gearbox().grind_events() : 0u;

        snap.diff_count = powertrain_.fill_diff_snapshot(snap.diffs); // PHYS-033 P4 I4 C8
        snap.awd_engagement = powertrain_.awd_engagement(); // PHYS-033 P4 I4 C9
        snap.park_lock_engaged = powertrain_.park_lock_engaged(); // PHYS-033 P4 I4 C10
        // PHYS-033 P4 I4 C11 (plan section 3.5): the pto's own engagement/mode/
        // out-node speed, all 0.0 for every vehicle with no pto.
        snap.pto_engagement = powertrain_.pto_engagement();
        snap.pto_mode = powertrain_.pto_mode();
        snap.pto_rpm = powertrain_.pto_rpm();
        // PHYS-033 P4 I4 C12: the governor's no-load setpoint/hand_throttle,
        // both 0.0 for every vehicle with no governor.
        snap.governor_setpoint_rpm = powertrain_.governor_setpoint_rpm();
        snap.hand_throttle = powertrain_.governor_hand_throttle();
        // PHYS-033 P4 I7a C5, broadened C5d: mirrors EngineSoundState::
        // fuel_flow_g_s above - 0.0 only for a vehicle with no fuel model.
        snap.fuel_flow_g_s = powertrain_.fuel_flow_g_s();
        snap.fuel_mass_kg = powertrain_.fuel_tank_mass(); // PHYS-033 P4 I7a C6: 0.0 without a tank
    }

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

void VehicleInstance::reset(const vehicle::VehicleResetOptions& opts, const IRigidBackend& backend) {
    real hub_speed_common = real(0.0);
    if (opts.hubs_roll_with_chassis) {
        const Pose pose = backend.get_pose(chassis_);
        const Motion motion = backend.get_motion(chassis_);
        // Chassis-local +x forward speed (this project's own convention,
        // e.g. test_vehicle_accel_standing_start.cpp's own forward-speed
        // reads) - same value for every wheel regardless of steer angle, a
        // reasonable approximation for a teleport (this file's own header
        // comment on VehicleInstance::reset).
        hub_speed_common = pose.orientation.inverse().rotate(motion.linear).x;
    }
    std::vector<real> hub_speeds(desc_.wheels.size(), real(0.0));
    for (std::size_t i = 0; i < desc_.wheels.size(); ++i) {
        const real radius = desc_.wheels[i].wheel_radius;
        hub_speeds[i] = radius > real(0.0) ? hub_speed_common / radius : real(0.0);
        wheels_[i]->reset_relaxation();
    }
    powertrain_.reset(opts, hub_speeds);
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
