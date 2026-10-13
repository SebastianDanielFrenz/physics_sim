#include "vehicle_instance.h"

#include "../drivetrain/engine_sound_state.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <variant>
#include <stdexcept>

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
    aero::validate(desc_.aero);
    validate_traction_control(desc_.traction_control);
    traction_sensors_.resize(desc_.wheels.size());
    substep_telemetry_.resize(kMaxSubstepTelemetry * desc_.wheels.size());
    {
        // Engine garage P8 (A15): fitment. Without a manual_tcu there is nothing to un-fit (the flags stay true and
        // the powertrain ignores them), but the SNAPSHOT reports the auto_* assists as not fitted.
        bool have_tcu = false;
        for (const drivetrain::ControllerDesc& cd : desc_.controllers) {
            if (const auto* tcu = std::get_if<drivetrain::ManualTcuDesc>(&cd.params)) {
                have_tcu = true;
                fit_clutch_ = tcu->auto_clutch.fitted;
                fit_blip_ = tcu->auto_blip.fitted;
                fit_shift_ = tcu->auto_shift.fitted;
            } else if (const auto* lc = std::get_if<drivetrain::LaunchControlDesc>(&cd.params)) {
                fit_launch_ = lc->fitted;
            }
        }
        fit_traction_ = desc_.traction_control.fitted;
        if (desc_.assist_unit && desc_.assist_unit->launch_control) {
            // Assists I4 (D-LC1): the unit's launch control. A car has either the P8 two-step (a launch_control
            // controller) or this one (owner Q5): both would drive TcuRequest::limiter_rpm.
            for (const drivetrain::ControllerDesc& cd : desc_.controllers) {
                if (std::holds_alternative<drivetrain::LaunchControlDesc>(cd.params)) {
                    throw std::runtime_error("vehicle '" + desc_.name +
                                             "': launch control: a car has either a two-step (launch_control) or an assist-unit launch control, not both");
                }
            }
            if (!assist_grade_info(desc_.assist_unit->grade).allows_launch_control) {
                throw std::runtime_error("vehicle '" + desc_.name + "': the assist unit's grade does not allow launch control");
            }
            const LaunchAssistDesc& l = *desc_.assist_unit->launch_control;
            try {
                validate_launch_assist(l);
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("vehicle '" + desc_.name + "': " + e.what());
            }
            real idle = real(0), limit = real(0);
            bool have_engine = false;
            for (const drivetrain::ComponentDesc& c : desc_.powertrain.components) {
                if (const auto* te = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) { idle = te->idle_rpm; limit = te->limiter.rpm; have_engine = true; break; }
                if (const auto* se = std::get_if<drivetrain::SimulatedEngineDesc>(&c.params)) { idle = se->idle_rpm; limit = se->limiter.rpm; have_engine = true; break; }
            }
            const real rpm = launch_assist_rpm(l, limit);
            if (!have_engine || !(rpm > idle) || !(rpm < limit)) {
                throw std::runtime_error("vehicle '" + desc_.name + "': launch control: the launch rpm must lie between the engine's idle and its own limiter");
            }
            launch_.emplace(l, rpm);
            fit_launch_ = true;
        }
        if (const auto aero_surface = find_aero_control_surface(desc_.aero, desc_.assist_unit)) {
            fit_aero_ = true;
            aero_control_surface_ = *aero_surface;
        }
        if (desc_.drive_modes) {
            try {
                validate_drive_modes(*desc_.drive_modes);
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("vehicle '" + desc_.name + "': " + e.what());
            }
            lc_default_state_ = any_launch_control_default(*desc_.drive_modes);
        }
        tc_mode_desc_ = resolve_traction_mode_descs(desc_.traction_control, desc_.assist_unit);
        tc_modes_available_bits_ = traction_modes_available_bits(desc_.assist_unit, fit_traction_);
        fitted_bits_ = ((have_tcu && fit_clutch_) ? 1u : 0u) | ((have_tcu && fit_blip_) ? 2u : 0u) | ((have_tcu && fit_shift_) ? 4u : 0u) |
                       (fit_traction_ ? 8u : 0u) | (fit_launch_ ? 16u : 0u) | (desc_.powertrain.anti_lag_fitted ? 32u : 0u) | (fit_aero_ ? 64u : 0u);
    }
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
    has_axle_balance_=front_n>0&&rear_n>0;
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
    // Engine garage P8 (A15): a not-fitted assist ignores its channel and stays off.
    const bool assist_auto_clutch = fit_clutch_ && channel_values[control_indices_.assist_auto_clutch] > real(0.5);
    const bool assist_auto_blip = fit_blip_ && channel_values[control_indices_.assist_auto_blip] > real(0.5);
    const bool assist_auto_shift = fit_shift_ && channel_values[control_indices_.assist_auto_shift] > real(0.5);
    // Assists I1: the requested mode code, and the mode actually run (a stateless function of channel and desc). A car
    // without a unit offers Off/On only, so a value > 0.5 is On exactly as before the unit existed.
    //
    // Assists I7 (drive modes): on a car with a drive_modes part the effective drive mode is resolved first (a pure
    // function of the assist.drive_mode channel and the desc). When the effective mode CHANGES, the mode's
    // traction_control_default (and, I7b, launch_control_default) is the request for this tick AND is queued as a
    // pending write of the control channel itself (assists I7c; World::step applies it after the pipeline), so the
    // channel always equals what the car does and the driver's next write wins, whatever its value. A mode without a
    // default leaves the channel alone. Only (initialised, last_mode) is state, hashed only on a car with the part.
    real tc_channel_value = channel_values[control_indices_.assist_traction_control];
    real lc_channel_value = fit_launch_ ? channel_values[control_indices_.assist_launch_control] : real(0);
    pending_write_count_ = 0;
    const DriveModeParams* drive_params = nullptr;
    DriveMode drive_mode_req = DriveMode::Normal, drive_mode_eff = DriveMode::Normal;
    if (desc_.drive_modes) {
        const DriveModesDesc& dm = *desc_.drive_modes;
        drive_mode_req = drive_mode_requested(dm, channel_values[control_indices_.assist_drive_mode]);
        drive_mode_eff = resolve_drive_mode(dm, drive_mode_req);
        drive_params = &dm.params[static_cast<std::size_t>(drive_mode_eff)];
        if (!drive_state_.initialised) {
            drive_state_.initialised = true;
            drive_state_.last_mode = static_cast<std::uint8_t>(drive_mode_eff);
        } else if (static_cast<std::uint8_t>(drive_mode_eff) != drive_state_.last_mode) {
            drive_state_.last_mode = static_cast<std::uint8_t>(drive_mode_eff);
            if (drive_params->traction_control_default) {
                tc_channel_value = static_cast<real>(static_cast<std::uint8_t>(*drive_params->traction_control_default));
                pending_writes_[pending_write_count_++] = {control_indices_.assist_traction_control, tc_channel_value};
            }
            if (fit_launch_ && lc_default_state_ && drive_params->launch_control_default) {
                lc_channel_value = *drive_params->launch_control_default ? real(1) : real(0);
                pending_writes_[pending_write_count_++] = {control_indices_.assist_launch_control, lc_channel_value};
            }
        }
    }
    const TractionMode tc_mode_requested = traction_mode_requested(tc_channel_value);
    const TractionMode tc_mode_effective = resolve_traction_mode(tc_modes_available_bits_, tc_mode_requested);
    const bool assist_traction = tc_mode_effective != TractionMode::Off;
    const TractionControlDesc& tc_desc = tc_mode_desc_[static_cast<std::size_t>(tc_mode_effective)];
    const bool assist_launch = fit_launch_ && lc_channel_value > real(0.5) &&
                               (drive_params == nullptr || drive_params->launch_control_allowed);
    // The drive mode's pedal bend, applied to the throttle the TC / launch ceilings are fractions of, so a ceiling stays
    // a fraction of the COMMAND the engine sees (the car's own pedal curve is not applied here - none of the cars with
    // a TC has one).
    const real ceiling_throttle = (drive_params != nullptr && drive_params->pedal_bend) ? pedal_bend_apply(*drive_params->pedal_bend, controls.throttle) : controls.throttle;

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
    // Assists I5 (D-AE1): rear-wing automation. With the channel on the controller OWNS the surface command for this
    // tick (stateless: a pure function of the forward airspeed, the brake channel and the launch hook); the World's
    // own command is parked in a local and put back after the substeps, so with the channel off - or later switched
    // off - World::set_aero_surface_command stands exactly as it did. forward airspeed = chassis-frame x of (chassis
    // velocity - the vehicle's own wind), read at tick start like racing_game's law did.
    const bool assist_aero = fit_aero_ && channel_values[control_indices_.assist_aero_control] >= real(0.5);
    real aero_control_target_deg = 0;
    real aero_world_command = 0;
    if (assist_aero) {
        const aero::SurfaceDesc& wing = desc_.aero.surfaces[aero_control_surface_];
        const real forward_speed = pose0.orientation.inverse().rotate(motion0.linear - env.wind_world).x;
        aero_world_command = aero_state_.surface_command[aero_control_surface_];
        aero_state_.surface_command[aero_control_surface_] =
            aero_control_command(*desc_.assist_unit->aero_control, wing.max_offset_rad, wing.min_offset_rad, forward_speed, controls.brake,
                                 aero_launch_request_, &aero_control_target_deg);
    }
    // Virtual ideal six-axis IMU: finite-difference acceleration, gravity
    // compensated using attitude. Only this sensor reading reaches TC.
    real imu_accel=0;
    if(desc_.traction_control.enabled||assist_traction||traction_state_.initialized) {
        if(imu_have_previous_&&tick_dt>0) imu_accel=pose0.orientation.inverse().rotate((motion0.linear-imu_previous_velocity_world_)/tick_dt).x;
        imu_previous_velocity_world_=motion0.linear;imu_have_previous_=true;
    }
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
    Vec3 aero_force_sum{}, aero_torque_sum{};
    real fan_energy_used=0, front_downforce_sum=0, rear_downforce_sum=0;

    // PHYS-033 P4 I2 commit 5b (plan section 8): snapshot the ledger's
    // cumulative fields before this tick's substeps, so end_tick() below can
    // compute this TICK's own balance_error from the deltas since here.
    // Engine thermal plan T1 (section 2.2): this tick's environment for the thermal model - the vehicle's own
    // ambient (per-vehicle air with an environment, else the World's) and the ram airspeed along the chassis
    // forward axis (chassis-local +x), at the tick-start state. Set BEFORE begin_tick (an "ambient" start reads it).
    if (powertrain_.has_thermal()) {
        const real ram = pose0.orientation.inverse().rotate(motion0.linear - env.wind_world).x;
        powertrain_.set_thermal_env(env.ambient.temperature_k, env.ambient.pressure_pa, ram > real(0.0) ? ram : real(0.0));
    }
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
    substep_telemetry_count_ = std::min(num_substeps, kMaxSubstepTelemetry);
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
        const real yaw=pose_s.orientation.inverse().rotate(twist_s.angular).z;
        for(std::size_t i=0;i<wheels_.size();++i) {
            const auto& w=desc_.wheels[i];auto& sensor=traction_sensors_[i];
            sensor={powertrain_.hub_omega(i)*w.wheel_radius,ackermann_wheel_angle(i,delta_avg),w.attachment_local.x,w.attachment_local.y,powertrain_.wheel_is_driven(i)};
        }
        // Assists I4 (D-LC1): the unit's launch control decides first, from the previous substep's sensors (its own
        // powertrain reads and TC's estimator state), so this substep's TC call already knows whether the controller
        // owns the car. While it does (Armed, Launch) traction control is SUSPENDED - it still runs, because the
        // slip PI needs its ground-speed estimate - and its outputs are not handed to the powertrain.
        LaunchAssist::Outputs lc_out;
        if(launch_) {
            LaunchAssist::Inputs li;
            li.enabled=assist_launch;
            li.auto_clutch=assist_auto_clutch;
            li.engine_running=powertrain_.has_engine()&&powertrain_.last_engine_result().state==drivetrain::EngineTorqueMap::EngineState::Running;
            li.throttle=controls.throttle;li.brake=controls.brake;
            li.speed_mps=powertrain_.approx_speed_mps();
            if(powertrain_.has_gearbox()) {
                const auto& gb=powertrain_.gearbox();
                li.gear=gb.current_position();
                li.shifting=gb.target()!=gb.current_position()||gb.phase()!=drivetrain::Gearbox::Phase::Idle;
            }
            li.slip_valid=traction_state_.initialized;
            li.slip=traction_driven_slip(traction_sensors_,traction_state_.estimated_speed_m_s,yaw,launch_->desc().slip_floor_mps);
            launch_slip_target_=launch_->desc().slip_target[static_cast<std::size_t>(tc_mode_effective)];
            li.slip_target=launch_slip_target_;
            li.dt=substep_dt;li.first_substep_of_tick=(s==0);
            lc_out=launch_->step(li);
            if(lc_out.handover&&assist_traction&&traction_state_.initialized) {
                // Handover: TC starts from the launch ceiling (no jump): desired = 1 - kp*err - integral, so with
                // integral = 1 - ceiling the unloaded law sits at the ceiling.
                traction_state_.torque_fraction=std::clamp(lc_out.handover_ceiling,tc_desc.min_fraction,real(1));
                traction_state_.integral=std::clamp(real(1)-traction_state_.torque_fraction,real(0),real(1));
            }
        }
        const bool lc_owns=lc_out.owns;
        const real traction_fraction_raw=step_traction_control(tc_desc,traction_state_,traction_sensors_,imu_accel,yaw,
            controls.throttle,std::max(controls.brake,controls.handbrake),assist_traction||lc_owns,substep_dt);
        // TC's output reaches the powertrain only when TC is really on and not suspended by the launch control.
        const real traction_fraction=(assist_traction&&!lc_owns)?traction_fraction_raw:real(1);
        ctx.traction_ignition_cut=assist_traction&&!lc_owns&&tc_desc.ignition_cut&&traction_fraction<real(.75)&&traction_state_.positive_slip>tc_desc.target_slip+real(.04);
        ctx.traction_throttle_ceiling=traction_fraction<real(1)?std::clamp(ceiling_throttle,real(0),real(1))*traction_fraction:real(1);
        if(lc_owns||lc_out.overlay) {
            // overlay = the soft hand-over: traction control is back, the launch limit and the clutch fraction stay
            // in force until the fraction has climbed to 1 (launch_assist.h)
            ctx.launch_limiter_rpm=lc_out.limiter_rpm;
            ctx.launch_hold_clutch=lc_out.hold_clutch;
            ctx.launch_clutch_ramp=lc_out.clutch_ramp;
            ctx.launch_throttle_ceiling=lc_out.ceiling<real(1)?std::clamp(ceiling_throttle,real(0),real(1))*lc_out.ceiling:real(1);
        }
        // Assists I2 (PHYS-074 D-LS1/D-LS2), gated on TC's effective mode != Off (coordinator decision 2026-10-12;
        // a car with its own traction_control block and no assist unit counts too): the TC fraction and floor for
        // the crawl-speed clutch path, and the ground-speed reference for the auto-clutch lock decision - the TC
        // estimate once initialised, else the mean undriven-wheel peripheral speed, NaN without either. With TC Off
        // the ctx keeps its defaults (fraction 1, NaN), so a TC-off car is bit-identical. Unhashed.
        if(assist_traction&&!lc_owns) {
            ctx.traction_torque_fraction=traction_fraction;
            ctx.traction_floor_fraction=tc_desc.min_fraction;
        }
        if(assist_traction||lc_owns) {
            if(traction_state_.initialized) {
                ctx.ground_speed_mps=traction_state_.estimated_speed_m_s;
            } else {
                real undriven_sum=0;int undriven=0;
                for(const auto& sensor:traction_sensors_) if(!sensor.driven){undriven_sum+=sensor.peripheral_speed_m_s;++undriven;}
                if(undriven>0)ctx.ground_speed_mps=undriven_sum/static_cast<real>(undriven);
            }
        }
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
        ctx.assist_launch_control = assist_launch;
        if (drive_params != nullptr) { // assists I7: the effective drive mode's throttle bend, shift/close overrides, nitrous gate
            if (drive_params->pedal_bend) { ctx.drive_bend_x = drive_params->pedal_bend->x; ctx.drive_bend_y = drive_params->pedal_bend->y; }
            if (drive_params->up_rpm_at_zero_throttle) ctx.drive_up_rpm_zero = *drive_params->up_rpm_at_zero_throttle;
            if (drive_params->up_rpm_at_full_throttle) ctx.drive_up_rpm_full = *drive_params->up_rpm_at_full_throttle;
            if (drive_params->down_rpm_at_zero_throttle) ctx.drive_down_rpm_zero = *drive_params->down_rpm_at_zero_throttle;
            if (drive_params->down_rpm_at_full_throttle) ctx.drive_down_rpm_full = *drive_params->down_rpm_at_full_throttle;
            if (drive_params->clutch_close_time_s) ctx.drive_close_time_s = *drive_params->clutch_close_time_s;
            ctx.nitrous_arm_allowed = drive_params->nitrous_arm_allowed;
        }
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
            in.grip = env.grip;

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
            // Assists I2a (diag only, unhashed, preallocated): this substep's wheel telemetry.
            if (s < kMaxSubstepTelemetry) substep_telemetry_[s * wheels_.size() + i] = wheels_[i]->telemetry();
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

        // Aero sees the SAME shadow pose/twist as the tyres, not a stale backend state.
        // Local probes also work against overhead roads when the chassis is inverted.
        std::array<aero::GroundSample,aero::max_surfaces+aero::max_fans+1> aero_ground{};
        auto probe=[&](Vec3 point,Vec3 direction,real range) {
            aero::GroundSample result;
            if(range<=0) return result;
            const Vec3 origin=pose_s.transform_point(point), dir=pose_s.transform_direction(direction);
            const auto hit=backend.ray_cast_excluding(origin,dir,range,chassis_);
            // Only a surface facing this probe may form a road/seal. A miss is never ground.
            if(hit.hit&&dot(hit.normal,-dir)>.5) result={true,range*hit.fraction};
            return result;
        };
        for(std::size_t i=0;i<desc_.aero.surfaces.size();++i) {
            const auto& surface=desc_.aero.surfaces[i];
            const real command=aero_state_.surface_command[i];
            const real target=command>=0?command*surface.max_offset_rad:-command*surface.min_offset_rad;
            const real offset=aero_state_.offset_rad[i]+std::clamp(target-aero_state_.offset_rad[i],-surface.rate_rad_s*substep_dt,surface.rate_rad_s*substep_dt);
            const real lift=surface.max_offset_rad>0 ? surface.max_lift_m*std::clamp(offset/surface.max_offset_rad,real(0),real(1)) : real(0);
            aero_ground[i]=probe(surface.position_local+surface.normal_local*lift,surface.ground.probe_local,surface.ground.range_m);
        }
        for(std::size_t i=0;i<desc_.aero.fans.size();++i) {
            const auto& fan=desc_.aero.fans[i];
            aero_ground[desc_.aero.surfaces.size()+i]=probe(fan.position_local,fan.direction_local,fan.probe_range_m);
        }
        if(desc_.aero.coefficient_map.table&&!desc_.aero.coefficient_map.table->data().axes[3].empty()) {
            const auto& map=desc_.aero.coefficient_map;
            aero_ground[desc_.aero.surfaces.size()+desc_.aero.fans.size()]=probe(map.ground_position_local,map.ground_probe_local,map.ground_range_m);
        }
        auto aero_sample=aero::step(desc_.aero,aero_state_,pose_s,twist_s,
            aero::Environment{env.air_density,env.wind_world},substep_dt,aero_ground,env.wakes,chassis_,env.extra_wakes);
        force_s+=aero_sample.wrench_world.force; torque_s+=aero_sample.wrench_world.torque;
        aero_force_sum+=aero_sample.wrench_world.force; aero_torque_sum+=aero_sample.wrench_world.torque;
        fan_energy_used+=aero_sample.fan_energy_used_j;
        if(has_axle_balance_) {
            const real pitch=pose_s.orientation.inverse().rotate(aero_sample.wrench_world.torque).y;
            const real front=(pitch-rear_axle_x_*aero_sample.downforce_n)/wheelbase_;
            front_downforce_sum+=front; rear_downforce_sum+=aero_sample.downforce_n-front;
        } else {
            front_downforce_sum+=aero_sample.front_downforce_n; rear_downforce_sum+=aero_sample.rear_downforce_n;
        }
        aero_telemetry_=aero_sample; // surface/actuator values describe the last substep

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

    if (assist_aero) aero_state_.surface_command[aero_control_surface_] = aero_world_command; // I5: the World's command stands
    // Assists I4: the launch control's aero request for the NEXT tick's controller (yield order LC > airbrake > cruise).
    if (launch_) aero_launch_request_ = launch_->owns();

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

        snap.traction_control_intervening=traction_state_.intervening;
        snap.traction_torque_fraction=traction_state_.torque_fraction;
        snap.traction_estimated_speed_m_s=traction_state_.estimated_speed_m_s;
        snap.traction_positive_slip=traction_state_.positive_slip;
        snap.clutch_engagement = powertrain_.has_clutch() ? powertrain_.clutch_engagement() : real(1.0);
        snap.assist_active_bits = (assist_auto_clutch ? 1u : 0u) | (assist_auto_blip ? 2u : 0u) | (assist_auto_shift ? 4u : 0u) | (assist_traction ? 8u : 0u) |
                                (assist_launch ? 16u : 0u) | (assist_aero ? 64u : 0u);
        snap.assist_fitted_bits = fitted_bits_;
        snap.traction_mode_requested = static_cast<std::uint8_t>(tc_mode_requested);
        snap.traction_mode_effective = static_cast<std::uint8_t>(tc_mode_effective);
        snap.traction_modes_available_bits = tc_modes_available_bits_;
        if (desc_.drive_modes) { // assists I7
            snap.drive_mode_requested = static_cast<std::uint8_t>(drive_mode_req);
            snap.drive_mode_effective = static_cast<std::uint8_t>(drive_mode_eff);
            snap.drive_modes_available_bits = desc_.drive_modes->modes_bits;
        }
        snap.launch_control_active = powertrain_.launch_control_active() || (launch_ && launch_->owns());
        snap.launch_phase = launch_ ? static_cast<std::uint8_t>(launch_->phase()) : std::uint8_t(0);
        snap.launch_slip_target = launch_ ? launch_slip_target_ : real(0);
        snap.aero_control_active = assist_aero;
        snap.aero_control_target_deg = aero_control_target_deg;

        // ps_audio A10 (vault PHYS-054): the fill itself now lives in
        // drivetrain::fill_engine_sound_state, shared with EngineRig, so a
        // car and a wheel-less rig publish byte-identical sound state from
        // the same code. fill_engine_sound_state's ess.engine_state comes
        // from last_engine_result().state directly, which is already
        // ps::drivetrain::EngineState (the switch above computing
        // snap.engine_state is an identity map on the same value - see that
        // file's own comment) - so this is bit-identical to the old inline
        // fill.
        snap.engines.clear();
        if (powertrain_.has_engine()) snap.engines.push_back(drivetrain::fill_engine_sound_state(powertrain_));

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
        snap.n2o_bottle_kg = powertrain_.n2o_bottle_mass(); // N2O plan N1: 0.0 without a kit
        snap.thermal = powertrain_.thermal_snapshot(); // engine thermal plan T1: has_thermal false without a cooling_system
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
    aero_telemetry_.wrench_world={aero_force_sum/n,aero_torque_sum/n};
    aero_telemetry_.fan_energy_used_j=fan_energy_used;
    aero_telemetry_.fan_power_w=fan_energy_used/tick_dt;
    aero_telemetry_.axle_balance_valid=has_axle_balance_;
    aero_telemetry_.front_downforce_n=front_downforce_sum/n; aero_telemetry_.rear_downforce_n=rear_downforce_sum/n;
    aero_telemetry_.downforce_n=aero_telemetry_.front_downforce_n+aero_telemetry_.rear_downforce_n;
    aero_telemetry_.front_balance=std::abs(aero_telemetry_.downforce_n)>1e-9 ? aero_telemetry_.front_downforce_n/aero_telemetry_.downforce_n : real(0);
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
    traction_state_={};drive_state_={};pending_write_count_=0;imu_have_previous_=false;imu_previous_velocity_world_={};
    if(launch_){launch_->reset();aero_launch_request_=false;}
    powertrain_.reset(opts, hub_speeds);
    // Teleport resets actuator dynamics/commands, but does not create or discard prepaid energy.
    const real energy=aero_state_.fan_energy_j; aero_state_={}; aero_state_.fan_energy_j=energy;
    aero_telemetry_={};
}

std::uint64_t VehicleInstance::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& w : wheels_) h = w->fold_state_hash(h);
    // PHYS-033 P4 I2 commit 5b (plan section 6): the shaft network's own
    // state (hub omega and every other node's, friction-row stick/sets,
    // topology state, each lossy gear's lambda_mesh_prev) - see
    // powertrain.h's fold_state_hash.
    h = powertrain_.fold_state_hash(h);
    h=fold_traction_control_hash(desc_.traction_control,traction_state_,h);
    if(launch_)h=launch_->fold_state_hash(h); // assists I4: hashed only on a car that carries the block
    if(desc_.assist_unit&&desc_.assist_unit->has_traction_control&&
       (desc_.assist_unit->traction_control.modes_bits&~kTractionModesBaseBits)!=0) {
        // Assists I1 (D-TC6): the mode table (desc only), only on a car whose unit offers modes beyond Off/On - a
        // unit with just Off/On (Street) adds nothing the base desc does not already say, so such a car hashes like
        // one without a unit. The effective mode is a pure function of channel and desc, so it is not folded.
        const auto fold=[&](const auto& v){const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}};
        const std::uint64_t tag=0x4153534953544d44ULL;fold(tag);
        const AssistTractionDesc& t=desc_.assist_unit->traction_control;
        fold(t.modes_bits);fold(static_cast<std::uint8_t>(t.default_mode));
        for(int m=2;m<kTractionModeCount;++m) {
            if(!(t.modes_bits&traction_mode_bit(static_cast<TractionMode>(m))))continue;
            const TractionControlDesc& d=tc_mode_desc_[static_cast<std::size_t>(m)];
            fold(static_cast<std::uint8_t>(m));
            fold(d.target_slip);fold(d.kp);fold(d.ki_per_s);fold(d.attack_rate_per_s);fold(d.recovery_rate_per_s);
            fold(d.min_fraction);fold(d.ignition_cut);
        }
    }
    if(desc_.drive_modes) {
        // Assists I7: the mode table (desc) and the mode state (last effective mode, the TC override and the last seen
        // TC channel value), only on a car with a drive_modes part.
        const auto fold=[&](const auto& v){const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}};
        const std::uint64_t tag=0x445249564d4f4445ULL;fold(tag);
        const DriveModesDesc& dm=*desc_.drive_modes;
        fold(dm.modes_bits);fold(static_cast<std::uint8_t>(dm.default_mode));
        for(int m=0;m<kDriveModeCount;++m) {
            if(!(dm.modes_bits&drive_mode_bit(static_cast<DriveMode>(m))))continue;
            const DriveModeParams& p=dm.params[static_cast<std::size_t>(m)];
            fold(static_cast<std::uint8_t>(m));
            fold(static_cast<std::uint8_t>(p.pedal_bend?1:0));if(p.pedal_bend){fold(p.pedal_bend->x);fold(p.pedal_bend->y);}
            for(const std::optional<real>* o:{&p.up_rpm_at_zero_throttle,&p.up_rpm_at_full_throttle,&p.down_rpm_at_zero_throttle,&p.down_rpm_at_full_throttle,&p.clutch_close_time_s}) {
                fold(static_cast<std::uint8_t>(*o?1:0));if(*o)fold(**o);
            }
            fold(static_cast<std::uint8_t>(p.traction_control_default?static_cast<int>(*p.traction_control_default)+1:0));
            fold(p.launch_control_allowed);fold(p.nitrous_arm_allowed);
            if(lc_default_state_)fold(static_cast<std::uint8_t>(p.launch_control_default?(*p.launch_control_default?2:1):0)); // I7b: only with a launch_control_default
        }
        fold(drive_state_.initialised);fold(drive_state_.last_mode);
    }
    if(desc_.traction_control.enabled||traction_state_.initialized||traction_state_.enabled) {
        for(real v:{imu_previous_velocity_world_.x,imu_previous_velocity_world_.y,imu_previous_velocity_world_.z,real(imu_have_previous_?1:0)}) {
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }
    }
    return aero::fold_state_hash(desc_.aero,aero_state_,h);
}

void VehicleInstance::set_aero_surface_command(std::size_t i,real command) {
    if(i>=desc_.aero.surfaces.size()||!std::isfinite(command)||command < -1||command > 1)
        throw std::invalid_argument("invalid aero surface index/command [-1,1]");
    aero_state_.surface_command[i]=command;
}
void VehicleInstance::set_aero_fan_command(std::size_t i,real command) {
    if(i>=desc_.aero.fans.size()||!std::isfinite(command)||command < 0||command > 1)
        throw std::invalid_argument("invalid aero fan index/command [0,1]");
    aero_state_.fan_command[i]=command;
}
void VehicleInstance::credit_aero_fan_energy(real joules) {
    if(desc_.aero.fans.empty()||!std::isfinite(joules)||joules<0||!std::isfinite(aero_state_.fan_energy_j+joules))
        throw std::invalid_argument("invalid aero fan energy credit");
    aero_state_.fan_energy_j+=joules;
}

} // namespace ps::vehicle
