#include "manual_tcu.h"

#include "ps/drivetrain/powertrain_desc.h" // kGearRequestNone

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {
namespace {
constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
// "The driver has the throttle pressed" for the TCU's own decisions (the
// auto-shift's N->1 select and the reverse-rotation hold below). PROVISIONAL:
// no AutoClutchDesc field carries such a threshold, and the value is the one
// the N->1 auto-select already used as a literal; 5 % of the throttle request
// is above pedal noise yet below any deliberate hill-start input.
constexpr real kDriverThrottleThreshold = real(0.05);
// Road speed (m/s, magnitude, gearbox-output derived) up to which a throttle
// press launches the clutch while the car rolls backwards in a forward gear (a
// hill start). PROVISIONAL, no data field: ~18 km/h of roll-back is more than
// a driver lets a car roll before moving off, and far below the "sliding
// backwards at 60-200 km/h in 7th with the throttle pinned" case that
// test_powertrain_tcu.cpp's "Hyper auto clutch protects a running crank..."
// keeps open - engaging onto a driveline reversing that fast would drag the
// crank backwards through zero, which no launch law can arrest.
constexpr real kReverseLaunchMaxSpeedMps = real(5.0);
real lerp(real a, real b, real t) { return a + (b - a) * t; }
real move_toward(real x, real target, real max_delta) {
    if (target > x) return std::min(x + max_delta, target);
    if (target < x) return std::max(x - max_delta, target);
    return x;
}

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

ManualTcu::Outputs ManualTcu::step(const Sensors& s, real h, bool first_substep_of_tick) {
    // Assists I7: a drive mode's override of the post-shift close time (NaN = the car's own value) and of the four
    // auto-shift rpm points. Unset (NaN) is bit-identical to the pre-I7 reads of desc_.
    const auto pick = [](real override_value, real own) { return std::isnan(override_value) ? own : override_value; };
    const real shift_req = pick(s.shift_throttle_request, s.throttle_request); // pedal P1: driver demand, before the TC ceiling
    const real close_time_s = pick(s.close_time_s, desc_.auto_clutch.close_time_s);
    // --- 6.5.1: request arbitration (once per tick only) --------------------
    int desired_target = s.gearbox_target; // default: no change
    if (first_substep_of_tick) {
        const std::int64_t up = static_cast<std::int64_t>(std::floor(s.shift_up_count + real(0.5)));
        const std::int64_t dn = static_cast<std::int64_t>(std::floor(s.shift_down_count + real(0.5)));
        if (!seeded_) {
            last_up_ = up;
            last_dn_ = dn;
            seeded_ = true;
        }
        const std::int64_t d_up = std::max<std::int64_t>(0, up - last_up_);
        const std::int64_t d_dn = std::max<std::int64_t>(0, dn - last_dn_);
        last_up_ = up;
        last_dn_ = dn;

        const bool valid = s.gear_request >= static_cast<real>(s.min_gear) - real(0.5) &&
                            s.gear_request <= static_cast<real>(s.max_gear) + real(0.5) && s.gear_request > kGearRequestNone + real(0.5);
        if (valid) {
            desired_target = static_cast<int>(std::floor(s.gear_request + real(0.5)));
            pending_ = 0;
        } else {
            pending_ = std::clamp(pending_ + static_cast<int>(d_up) - static_cast<int>(d_dn), -desc_.max_pending_steps, desc_.max_pending_steps);
            if (s.gearbox_phase == Gearbox::Phase::Idle && pending_ != 0) {
                const int step = pending_ > 0 ? 1 : -1;
                desired_target = std::clamp(s.gearbox_current + step, s.min_gear, s.max_gear);
                pending_ -= step;
            } else if (s.assist_auto_shift && pending_ == 0 && s.gearbox_phase == Gearbox::Phase::Idle) {
                // Gated on the runtime assist.auto_shift channel (plan
                // section 7.1; AutoShiftDesc::enabled is only that channel's
                // default, applied by World::create_vehicle). Gating on the
                // desc flag left the HUD's auto-shift toggle (and
                // assist_active_bits bit2) ON while nothing shifted: 1st at
                // WOT sat on the limiter for 25 s.
                // 6.5.4: auto-shift driver aid.
                if (d_up != 0 || d_dn != 0) time_in_gear_ = real(0.0); // a manual press overrides / resets the timer
                const real engine_rpm = std::abs(s.engine_omega) * kRadPerSecToRpm;
                const real out_rpm = std::abs(s.gearbox_out_omega) * kRadPerSecToRpm;
                if (s.gearbox_current == 0) {
                    if (shift_req > kDriverThrottleThreshold && out_rpm < real(100.0)) desired_target = std::clamp(desc_.auto_shift.start_gear, s.min_gear, s.max_gear);
                } else if (s.gearbox_current >= 1) {
                    // Assists I7 kickdown bypass: a downshift with the throttle command at or above kickdown_throttle
                    // (0 = never) ignores min_time_in_gear_s; upshifts always wait for it. The bypass is a driver action:
                    // it is spent by a gear change and re-armed only once the command has fallen below kickdown_throttle,
                    // so a pedal held at full throttle through an upshift cannot bounce the car back down on the rpm
                    // collapse that follows the shift (measured, H2: Sport pull 2-3 bounces with the plain rule).
                    const bool dwell_ok = time_in_gear_ >= desc_.auto_shift.min_time_in_gear_s;
                    const bool kickdown = desc_.auto_shift.kickdown_throttle > real(0.0) && shift_req >= desc_.auto_shift.kickdown_throttle &&
                                          !kickdown_spent_;
                    if (dwell_ok || kickdown) {
                        const real up_th = lerp(pick(s.up_rpm_at_zero_throttle, desc_.auto_shift.up_rpm_at_zero_throttle),
                                                pick(s.up_rpm_at_full_throttle, desc_.auto_shift.up_rpm_at_full_throttle), shift_req);
                        const real down_th = lerp(pick(s.down_rpm_at_zero_throttle, desc_.auto_shift.down_rpm_at_zero_throttle),
                                                  pick(s.down_rpm_at_full_throttle, desc_.auto_shift.down_rpm_at_full_throttle), shift_req);
                        if (dwell_ok && engine_rpm > up_th && s.gearbox_current < s.max_gear) desired_target = s.gearbox_current + 1;
                        else if (engine_rpm < down_th && s.gearbox_current > 1) desired_target = s.gearbox_current - 1;
                    }
                }
            }
        }
    }

    // time_in_gear_ tracks dwell in the CURRENT gear, reset on any gear change
    // and by a manual override above; only meaningful for auto-shift.
    if (!last_target_seen_valid_ || last_target_seen_ != s.gearbox_current) {
        if (last_target_seen_valid_ && desc_.auto_shift.kickdown_throttle > real(0.0)) kickdown_spent_ = true; // a gear change spends the kickdown (re-armed below)
        time_in_gear_ = real(0.0);
        last_target_seen_ = s.gearbox_current;
        last_target_seen_valid_ = true;
    } else {
        time_in_gear_ += h;
    }

    if (shift_req < desc_.auto_shift.kickdown_throttle) kickdown_spent_ = false;

    // --- 6.5.2: auto-clutch ---------------------------------------------------
    const real rpm = std::abs(s.engine_omega) * kRadPerSecToRpm;
    const bool gearbox_shifting_now = !(s.gearbox_phase == Gearbox::Phase::Idle && s.gearbox_current == desired_target);
    // Owner ruling: a Neutral -> gear engagement at or below
    // launch_speed_threshold_mps is a LAUNCH, not a mid-drive shift - keep
    // the TCU in (or move it to) Launch mode instead of forcing Shift, so
    // the clutch follows the rpm-gated Launch law (this function's own
    // Launch-mode block below) through the gearbox's own Selecting/
    // Synchronising phases and into lockup, rather than the fast, rpm-blind
    // Shift-mode ClutchClosing reclose. The gearbox's own phase_step is
    // unaffected (it chases desired_target regardless of TCU mode).
    const bool launching_from_neutral =
        s.gearbox_current == 0 && desired_target != 0 && std::abs(s.approx_speed_mps) < desc_.auto_clutch.launch_speed_threshold_mps;
    if (launching_from_neutral) {
        if (mode_ == ClutchMode::Shift) mode_ = ClutchMode::Launch; // e.g. re-requesting 1st before an old shift settled
    } else if (desired_target != s.gearbox_target || (s.gearbox_target != s.gearbox_current && mode_ != ClutchMode::Shift)) {
        // A new shift just got requested (this tick's own desired_target
        // differs from what the gearbox is currently chasing), or the
        // gearbox is already mid-shift from a PRIOR tick's request and the
        // TCU has not yet noticed (first call after construction/reset).
        mode_ = ClutchMode::Shift;
        // Remember the direction while the old gear is still reported: the
        // gearbox reports current == 0 from Selecting onward.
        upshift_cut_armed_ = desired_target > s.gearbox_current && s.gearbox_current > 0;
    }

    // Post-shift engagement onto a driveline too slow to carry the engine
    // (owner test drive 2026-09-25, "drive off in 2nd from walking pace"):
    // once the dog is in, a Shift-mode reclose is rpm-blind - it closes the
    // clutch at 1/close_time_s whatever the engine does. Selecting 2nd at
    // ~5 km/h puts the gearbox input at ~350 rpm, below stall_rpm, so the
    // forced reclose dragged the engine down to the input shaft's speed and
    // stalled it; the Launch law then held the clutch open (rpm 0 < bite)
    // and the car coasted with a dead engine whatever the throttle said.
    // The same threshold the Launch law uses to decide the driveline CAN
    // carry the engine (rolling_reengage_rpm, signed input RPM) now also
    // decides the opposite: below it, the post-shift engagement is a
    // launch (rpm-gated slip law, anti-stall), not a shift. Also a direct
    // anti-stall net for the closing phase itself. Neutral (current == 0)
    // is unaffected (the Launch law closes fully in N anyway).
    if (mode_ == ClutchMode::Shift && !gearbox_shifting_now && s.gearbox_current != 0) {
        const real in_rpm = s.gearbox_in_omega * kRadPerSecToRpm;
        if (in_rpm < desc_.auto_clutch.rolling_reengage_rpm || rpm < desc_.auto_clutch.anti_stall_rpm) mode_ = ClutchMode::Launch;
    }

    // Torque-following close (AutoClutchDesc::sync_time_s, owner test drive
    // 2026-09-25 "a lot of wheel spin in 2nd"). Closing onto a driveline that
    // is already moving - the post-shift reclose once the dog is in, and the
    // Launch law's rolling re-engagement - used to ramp e_auto to 1 at a
    // fixed rate whatever the engine did. On car_sedan.json that put the
    // full 300 Nm kinetic capacity (1.5x the engine's peak) across a clutch
    // still slipping by 2000-4000 rpm (the engine flares during
    // ClutchClosing: the upshift cut ends with the gearbox phase), so the
    // driven wheels got the engine's torque PLUS the flywheel's slip energy
    // and broke traction in 2nd - measured probe: FL/FR kappa 1.3/1.2 right
    // after a WOT 1->2 at 27-35 km/h. The same fixed-rate ramp in the Launch
    // law's re-engagement was the 2220 -> 1441 rpm kick in the walking-pace
    // 2nd-gear probe. The close now sets the clutch's kinetic capacity to
    //   T = max(0, sign(slip) * T_engine + T_sync),
    //   T_sync = min(J_engine * |slip| / sync_time_s, max_sync_torque_nm)
    // i.e. the clutch carries what the engine itself produces plus a bounded
    // synchronising torque, so the engine's own speed converges on the input
    // shaft's (net engine torque = -sign(slip) * T_sync: exponential with
    // time constant sync_time_s, rate-limited by max_sync_torque_nm), and
    // the wheels never see more than T_engine + max_sync_torque_nm through
    // the clutch. Once |slip| < lock_slip_rpm the TCU hands over to Locked
    // (engage_rate_per_s to full capacity; the static row then carries only
    // what the driveline needs, so the last step is jerk-free). e_auto
    // follows the target at 1/close_time_s (the existing shift-close rate)
    // in both directions. Rejected alternatives (see the design record):
    // extending the upshift ignition cut through ClutchClosing INSTEAD of
    // this law (the flywheel energy still has to go somewhere - but the two
    // combine: since the owner's "speed ticking through the shift" report
    // the cut is held until the engine is speed-matched and this law then
    // finishes the close, see the upshift-cut block below), an uncapped sync term (J*|slip|/tau alone reaches 235 Nm
    // at a 4000 rpm flare - the same dump), and a slip-proportional
    // e = k*slip law (no notion of engine torque: under-carries at WOT, so
    // the engine flares to the limiter, and over-carries off-throttle).
    // Launch from rest (the rpm-gated Launch law) is unchanged.
    bool emergency_release = false;
    real e_tgt = e_auto_;
    real rate = desc_.auto_clutch.release_rate_per_s;
    if (mode_ == ClutchMode::Shift) {
        if (gearbox_shifting_now) {
            e_tgt = real(0.0);
            rate = real(1.0) / desc_.auto_clutch.open_time_s;
        } else {
            e_tgt = torque_following_close() ? sync_close_target(s) : real(1.0);
            rate = real(1.0) / close_time_s;
        }
    } else if (mode_ == ClutchMode::Locked) {
        if (s.engine_omega * kRadPerSecToRpm < desc_.auto_clutch.anti_stall_rpm ||
            s.gearbox_in_omega * kRadPerSecToRpm < desc_.auto_clutch.anti_stall_rpm) {
            mode_ = ClutchMode::Launch;
            emergency_release = true;
        }
        else {
            e_tgt = real(1.0);
            rate = desc_.auto_clutch.engage_rate_per_s;
        }
    }
    bool sync_closing = false; // Launch-mode rolling re-engagement under the torque-following close
    bool engaging_from_neutral = false;
    if (mode_ == ClutchMode::Launch) {
        if (s.gearbox_current == 0 && s.gearbox_phase == Gearbox::Phase::Idle) {
            e_tgt = real(1.0);
        } else if (s.gearbox_current == 0) {
            // The gearbox is still Selecting/Synchronising the launch gear
            // (current == 0, not Idle): hold the clutch OPEN until the dog
            // is in. The rpm-gated law below used to close it as soon as a
            // throttle-flared engine passed bite_rpm (N->1 requested at WOT,
            // the gamepad's natural "hold throttle, press shift"), so the
            // synchro had to fight the engine through the clutch: it never
            // matched speeds, only dragged the car through its own slip,
            // and unless that drag happened to push the car past
            // launch_speed_threshold_mps (re-classifying the engagement as a
            // rolling shift, which opens the clutch) the gearbox aborted
            // after abort_after_s and left the car in Neutral with the
            // engine on the limiter. A real driver launches with the pedal
            // down until the gear is in, whatever the rpm.
            e_tgt = real(0.0);
            engaging_from_neutral = true;
        } else {
            const real full = desc_.auto_clutch.full_rpm_min +
                               s.throttle_request * (desc_.auto_clutch.full_rpm_max - desc_.auto_clutch.full_rpm_min);
            const real denom = full - desc_.auto_clutch.bite_rpm;
            e_tgt = denom > real(0.0) ? std::clamp((rpm - desc_.auto_clutch.bite_rpm) / denom, real(0.0), real(1.0)) : real(1.0);
            // Rolling re-engagement (AutoClutchDesc::rolling_reengage_rpm,
            // compared SIGNED so a backwards-turning input never counts as
            // rolling): in gear, not shifting, and the input shaft fast
            // enough to carry the engine - close, like after any shift.
            // ca118ec used anti_stall_rpm here; that closed every launch from
            // rest at ~6 km/h onto an engine near idle (2026-10-04).
            // Torque-following (see the Shift-mode close above) when the
            // network data is available, else the legacy ramp to 1.
            const real in_rpm = s.gearbox_in_omega * kRadPerSecToRpm;
            if (s.gearbox_current != 0 && s.gearbox_phase == Gearbox::Phase::Idle && in_rpm > desc_.auto_clutch.rolling_reengage_rpm) {
                e_tgt = torque_following_close() ? sync_close_target(s) : real(1.0);
                sync_closing = torque_following_close();
            }
        }
        rate = (e_tgt >= e_auto_) ? desc_.auto_clutch.engage_rate_per_s : desc_.auto_clutch.release_rate_per_s;
        if (sync_closing) rate = real(1.0) / close_time_s;
        if (engaging_from_neutral) rate = std::max(rate, real(1.0) / desc_.auto_clutch.open_time_s);
        // Anti-stall panic dump (PHYS-033 P4 I3 C8 smoke-test finding, see
        // AutoClutchDesc::anti_stall_release_rate_per_s's own comment):
        // releasing below anti_stall_rpm specifically opens at LEAST this
        // fast, on top of whatever release_rate_per_s already gives - never
        // slower than the ordinary rate.
        if (e_tgt < e_auto_ && rpm < desc_.auto_clutch.anti_stall_rpm) {
            rate = std::max(rate, desc_.auto_clutch.anti_stall_release_rate_per_s);
        }
        const real slip_rpm = std::abs(s.engine_omega - s.gearbox_in_omega) * kRadPerSecToRpm;
        // Assists I2 (PHYS-074 D-LS2): no Locked transition while the input rpm the ground speed implies is below
        // anti_stall_rpm - the clutch would lock onto spinning wheels, and the Locked anti-stall then dumps it as
        // soon as traction control pulls the wheels (and the locked engine) down. Unlocked, the friction row still
        // sticks physically whenever capacity exceeds the torque; only the Locked-mode bookkeeping waits. NaN (no
        // reference: TC Off) compares false - the pre-I2 behaviour.
        const bool lock_onto_spin =
            s.ground_speed_mps * s.ground_speed_to_input_omega * kRadPerSecToRpm < desc_.auto_clutch.anti_stall_rpm;
        if (slip_rpm < desc_.auto_clutch.lock_slip_rpm && rpm > desc_.auto_clutch.bite_rpm && s.gearbox_current != 0 &&
            !lock_onto_spin) {
            mode_ = ClutchMode::Locked;
            e_tgt = real(1.0);
            rate = desc_.auto_clutch.engage_rate_per_s;
        }
    }
    // A locked clutch below the actual/coupled anti-stall threshold must
    // release this substep: a 25 ms ramp can already stall a light flywheel.
    // Opposite shaft rotation also stays disconnected (reverse_hold_, see
    // manual_tcu.h): entered only on a clear reversal, left above
    // -lock_slip_rpm. Entering at -lock_slip_rpm alone dumped the clutch on
    // brief negative input transients of ordinary cars (an unsettled WOT
    // start, locked-diff split-mu launch and braking - 2026-10-04).
    // The hold applies only WITHOUT driver throttle at hill-start pace (owner ruling 2026-10-04:
    // rolling backwards without throttle the clutch disengages; rolling
    // backwards WITH throttle - a hill start in a forward gear - it engages
    // gradually while the rpm stays above stall). With the throttle pressed
    // (road speed below kReverseLaunchMaxSpeedMps; faster, the hold stays on
    // whatever the throttle) the ordinary rpm-gated Launch law above decides: e rises with engine
    // rpm at engage_rate_per_s and falls to 0 at bite_rpm (> anti_stall_rpm),
    // the rolling re-engagement stays signed so it never fires on a reversing
    // input, and the anti-stall panic release still applies. While the
    // throttle is pressed reverse_hold_ is cleared (not latched, so it cannot
    // re-open the clutch mid-hill-start) and anti_stall_released_ is left
    // alone: it only feeds the latch below, and keeps its meaning if the
    // driver lifts while the car is still rolling back with the clutch fully
    // open (e_auto 0 - the hold then re-latches). A Locked clutch still
    // dumps instantly on emergency_release whatever the throttle.
    const real in_rpm_signed = s.gearbox_in_omega * kRadPerSecToRpm;
    const bool in_gear_idle = s.gearbox_current != 0 && s.gearbox_phase == Gearbox::Phase::Idle;
    const bool driver_throttle =
        s.throttle_request > kDriverThrottleThreshold && std::abs(s.approx_speed_mps) < kReverseLaunchMaxSpeedMps;
    if (!in_gear_idle || in_rpm_signed >= -desc_.auto_clutch.lock_slip_rpm || driver_throttle) {
        reverse_hold_ = false;
    } else if (in_rpm_signed < -desc_.auto_clutch.anti_stall_rpm || emergency_release ||
               (anti_stall_released_ && e_auto_ <= real(0.0))) {
        reverse_hold_ = true;
    }
    if (in_gear_idle && (emergency_release || reverse_hold_)) {
        mode_ = ClutchMode::Launch;
        e_tgt = real(0.0);
        rate = std::max(rate, e_auto_ / h);
    }
    e_auto_ = move_toward(e_auto_, e_tgt, rate * h);
    if (mode_ == ClutchMode::Shift && !gearbox_shifting_now) {
        const real slip_rpm = std::abs(s.engine_omega - s.gearbox_in_omega) * kRadPerSecToRpm;
        if (torque_following_close()) {
            // Synchronised: lock (the anti-stall/launch hand-off above still
            // covers a driveline that turns out too slow on the way).
            if (slip_rpm < desc_.auto_clutch.lock_slip_rpm) mode_ = ClutchMode::Locked;
        } else if (e_auto_ >= real(1.0) - real(1e-6)) {
            mode_ = (slip_rpm < desc_.auto_clutch.lock_slip_rpm) ? ClutchMode::Locked : ClutchMode::Launch;
        }
    }
    if (emergency_release) anti_stall_released_ = true;
    if (mode_ == ClutchMode::Locked) anti_stall_released_ = false;
    // (AutoClutchDesc/AutoBlipDesc::enabled are likewise only the default of
    // their assist.* channels; the outputs below are gated on the channels.)

    // --- 6.5.3: auto-blip and upshift cut --------------------------------------
    real throttle_floor = real(0.0);
    bool ignition_cut = false;
    {
        const bool downshifting = desired_target < s.gearbox_current && desired_target > 0;
        const bool blip_window = downshifting &&
                                  (s.gearbox_phase == Gearbox::Phase::Selecting || s.gearbox_phase == Gearbox::Phase::Synchronising ||
                                   (mode_ == ClutchMode::Shift && !gearbox_shifting_now)); // "ClutchClosing"
        if (blip_window && s.synchro_target_ratio != real(0.0)) {
            const real omega_t = s.synchro_target_ratio * s.gearbox_out_omega;
            throttle_floor =
                std::clamp(desc_.auto_blip.kp_per_rpm * (omega_t - s.engine_omega) * kRadPerSecToRpm, real(0.0), desc_.auto_blip.max_throttle);
        }
    }
    // The upshift cut covers the WHOLE gearbox shift (Releasing, Selecting,
    // Synchronising), as test_powertrain_tcu.cpp's own "ignition_cut is set
    // while the gearbox is mid-shift" case always intended. Keyed on
    // desired_target > gearbox_current alone it silently ended at
    // Selecting, where the real Gearbox reports current == 0 (neutral):
    // the engine then ran unloaded at the driver's WOT for the rest of the
    // shift and flared to the limiter (measured: 5888 -> 6717 rpm in 0.1 s
    // on a 1->2 at 31 km/h), which the post-shift close then had to dump
    // into the driven wheels.
    //
    // Speed-matched upshift (owner test drive 2026-09-25: "I still see the
    // speed ticking throughout the entire shift"): with the auto-clutch on,
    // the cut also stays on through the post-shift close (Shift mode, dog
    // in) until the engine has fallen to within
    // AutoBlipDesc::upshift_cut_release_slip_rpm of the new gear's input
    // shaft, like an AMT's ECU. Ending it with the gearbox phase left the
    // engine at full power for the whole ~0.8 s close: the torque-following
    // clutch carried engine torque + the sync term (~245 Nm vs. the 200 Nm
    // peak), so the car pulled at 3.5-4 m/s^2 - harder than locked 2nd -
    // and the driver felt no shift at all. While the cut holds, the close
    // law's target is max(0, motoring + T_sync) ~ 0, so the car coasts and
    // the engine loses the gear step on its own motoring torque. A stuck
    // close cannot hold the cut forever: upshift_cut_max_hold_s after the
    // dog is in the cut is released regardless. Once released (or if the
    // TCU hands the engagement to Launch/Locked) the cut never re-arms
    // within the same shift, and the throttle comes back through a ceiling
    // ramped 0 -> 1 over upshift_torque_ramp_s instead of a step, which the
    // torque-following close follows (its feed-forward is the engine's
    // demand, ramp included). Manual clutch (assist.auto_clutch off): the
    // cut still ends with the gearbox phase - the driver's own pedal, not
    // the TCU, decides when the engine is re-connected.
    bool cut_now = false;
    if (desc_.auto_blip.upshift_cut) {
        const bool upshifting = (desired_target > s.gearbox_current && s.gearbox_current > 0) || upshift_cut_armed_;
        if (upshifting && gearbox_shifting_now) {
            cut_now = true;
            upshift_cut_hold_t_ = real(0.0);
        } else if (upshift_cut_armed_ && s.assist_auto_clutch && mode_ == ClutchMode::Shift) {
            upshift_cut_hold_t_ += h;
            const real flare_rpm = (s.engine_omega - s.gearbox_in_omega) * kRadPerSecToRpm;
            cut_now = flare_rpm > desc_.auto_blip.upshift_cut_release_slip_rpm && upshift_cut_hold_t_ <= desc_.auto_blip.upshift_cut_max_hold_s;
        }
    }
    if (!cut_now && !gearbox_shifting_now) upshift_cut_armed_ = false; // released, or never armed: one cut per shift
    if (cut_now) {
        throttle_ceiling_ = real(0.0);
    } else {
        throttle_ceiling_ = desc_.auto_blip.upshift_torque_ramp_s > real(0.0)
                                ? move_toward(throttle_ceiling_, real(1.0), h / desc_.auto_blip.upshift_torque_ramp_s)
                                : real(1.0);
    }
    ignition_cut = cut_now;

    ManualTcu::Outputs out;
    out.gearbox_target = desired_target;
    out.e_auto = s.assist_auto_clutch ? e_auto_ : real(1.0);
    // Assists I2 (PHYS-074 D-LS1): traction control's crawl-speed clutch-capacity cap, on the output only (the e_auto
    // state keeps following its own law, so the clutch is back where the law has it once TC recovers).
    if (s.assist_auto_clutch && s.clutch_capacity_cap_nm < std::numeric_limits<real>::infinity() && desc_.clutch_capacity_nm > real(0.0))
        out.e_auto = std::min(out.e_auto, std::max(real(0.0), s.clutch_capacity_cap_nm) / desc_.clutch_capacity_nm);
    out.throttle_floor = s.assist_auto_blip ? throttle_floor : real(0.0);
    out.ignition_cut = s.assist_auto_blip && ignition_cut;
    out.throttle_ceiling = s.assist_auto_blip ? throttle_ceiling_ : real(1.0);
    return out;
}

real ManualTcu::sync_close_target(const Sensors& s) const {
    const real slip = s.engine_omega - s.gearbox_in_omega;
    const real dir = slip >= real(0.0) ? real(1.0) : real(-1.0);
    const real t_sync =
        std::min(desc_.engine_inertia * std::abs(slip) / desc_.auto_clutch.sync_time_s, desc_.auto_clutch.max_sync_torque_nm);
    const real t = std::max(real(0.0), dir * s.engine_torque_nm + t_sync);
    return std::min(t / desc_.clutch_capacity_nm, real(1.0));
}

std::uint64_t ManualTcu::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    h = fnv1a(h, seeded_);
    h = fnv1a(h, last_up_);
    h = fnv1a(h, last_dn_);
    h = fnv1a(h, pending_);
    h = fnv1a(h, mode_);
    h = fnv1a(h, e_auto_);
    h = fnv1a(h, time_in_gear_);
    h = fnv1a(h, upshift_cut_armed_);
    h = fnv1a(h, upshift_cut_hold_t_);
    h = fnv1a(h, throttle_ceiling_);
    // Folded only when set, so vehicles that never trigger them hash as before.
    if (reverse_hold_) h = fnv1a(h, reverse_hold_);
    if (kickdown_spent_) h = fnv1a(h, std::uint8_t(3));
    if (anti_stall_released_) h = fnv1a(h, std::uint8_t(2));
    return h;
}

} // namespace ps::drivetrain
