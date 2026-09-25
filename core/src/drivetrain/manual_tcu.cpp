#include "manual_tcu.h"

#include "ps/drivetrain/powertrain_desc.h" // kGearRequestNone

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {
namespace {
constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
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
            } else if (desc_.auto_shift.enabled && pending_ == 0 && s.gearbox_phase == Gearbox::Phase::Idle) {
                // 6.5.4: auto-shift driver aid.
                if (d_up != 0 || d_dn != 0) time_in_gear_ = real(0.0); // a manual press overrides / resets the timer
                const real engine_rpm = std::abs(s.engine_omega) * kRadPerSecToRpm;
                const real out_rpm = std::abs(s.gearbox_out_omega) * kRadPerSecToRpm;
                if (s.gearbox_current == 0) {
                    if (s.throttle_pedal > real(0.05) && out_rpm < real(100.0)) desired_target = 1;
                } else if (s.gearbox_current >= 1 && time_in_gear_ >= desc_.auto_shift.min_time_in_gear_s) {
                    const real up_th = lerp(desc_.auto_shift.up_rpm_at_zero_throttle, desc_.auto_shift.up_rpm_at_full_throttle, s.throttle_pedal);
                    const real down_th =
                        lerp(desc_.auto_shift.down_rpm_at_zero_throttle, desc_.auto_shift.down_rpm_at_full_throttle, s.throttle_pedal);
                    if (engine_rpm > up_th && s.gearbox_current < s.max_gear) desired_target = s.gearbox_current + 1;
                    else if (engine_rpm < down_th && s.gearbox_current > 1) desired_target = s.gearbox_current - 1;
                }
            }
        }
    }

    // time_in_gear_ tracks dwell in the CURRENT gear, reset on any gear change
    // and by a manual override above; only meaningful for auto-shift.
    if (!last_target_seen_valid_ || last_target_seen_ != s.gearbox_current) {
        time_in_gear_ = real(0.0);
        last_target_seen_ = s.gearbox_current;
        last_target_seen_valid_ = true;
    } else {
        time_in_gear_ += h;
    }

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
    // The same threshold the Launch law already uses to decide the
    // driveline CAN carry the engine (rolling_reengage_rpm) now also
    // decides the opposite: below it, the post-shift engagement is a
    // launch (rpm-gated slip law, anti-stall), not a shift. Also a direct
    // anti-stall net for the closing phase itself. Neutral (current == 0)
    // is unaffected (the Launch law closes fully in N anyway).
    if (mode_ == ClutchMode::Shift && !gearbox_shifting_now && s.gearbox_current != 0) {
        const real in_rpm = std::abs(s.gearbox_in_omega) * kRadPerSecToRpm;
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
    // extending the upshift ignition cut through ClutchClosing (a torque
    // hole of up to ~0.7 s per shift; the flywheel energy still has to go
    // somewhere), an uncapped sync term (J*|slip|/tau alone reaches 235 Nm
    // at a 4000 rpm flare - the same dump), and a slip-proportional
    // e = k*slip law (no notion of engine torque: under-carries at WOT, so
    // the engine flares to the limiter, and over-carries off-throttle).
    // Launch from rest (the rpm-gated Launch law) is unchanged.
    real e_tgt = e_auto_;
    real rate = desc_.auto_clutch.release_rate_per_s;
    if (mode_ == ClutchMode::Shift) {
        if (gearbox_shifting_now) {
            e_tgt = real(0.0);
            rate = real(1.0) / desc_.auto_clutch.open_time_s;
        } else {
            e_tgt = torque_following_close() ? sync_close_target(s) : real(1.0);
            rate = real(1.0) / desc_.auto_clutch.close_time_s;
        }
    } else if (mode_ == ClutchMode::Locked) {
        if (rpm < desc_.auto_clutch.anti_stall_rpm) mode_ = ClutchMode::Launch;
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
                               s.throttle_pedal * (desc_.auto_clutch.full_rpm_max - desc_.auto_clutch.full_rpm_min);
            const real denom = full - desc_.auto_clutch.bite_rpm;
            e_tgt = denom > real(0.0) ? std::clamp((rpm - desc_.auto_clutch.bite_rpm) / denom, real(0.0), real(1.0)) : real(1.0);
            // Rolling re-engagement (AutoClutchDesc::rolling_reengage_rpm):
            // in gear, not shifting, and the input shaft fast enough to
            // carry the engine - close, like after any shift.
            // Torque-following (see the Shift-mode close above) when the
            // network data is available, else the legacy ramp to 1.
            const real in_rpm = std::abs(s.gearbox_in_omega) * kRadPerSecToRpm;
            if (s.gearbox_current != 0 && s.gearbox_phase == Gearbox::Phase::Idle && in_rpm > desc_.auto_clutch.rolling_reengage_rpm) {
                e_tgt = torque_following_close() ? sync_close_target(s) : real(1.0);
                sync_closing = torque_following_close();
            }
        }
        rate = (e_tgt >= e_auto_) ? desc_.auto_clutch.engage_rate_per_s : desc_.auto_clutch.release_rate_per_s;
        if (sync_closing) rate = real(1.0) / desc_.auto_clutch.close_time_s;
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
        if (slip_rpm < desc_.auto_clutch.lock_slip_rpm && rpm > desc_.auto_clutch.bite_rpm && s.gearbox_current != 0) {
            mode_ = ClutchMode::Locked;
            e_tgt = real(1.0);
            rate = desc_.auto_clutch.engage_rate_per_s;
        }
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
    if (!desc_.auto_clutch.enabled) e_auto_ = real(1.0);

    // --- 6.5.3: auto-blip and upshift cut --------------------------------------
    real throttle_floor = real(0.0);
    bool ignition_cut = false;
    if (desc_.auto_blip.enabled) {
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
    if (!gearbox_shifting_now) upshift_cut_armed_ = false;
    if (desc_.auto_blip.upshift_cut) {
        const bool upshifting = (desired_target > s.gearbox_current && s.gearbox_current > 0) || upshift_cut_armed_;
        ignition_cut = upshifting && gearbox_shifting_now;
    }

    ManualTcu::Outputs out;
    out.gearbox_target = desired_target;
    out.e_auto = s.assist_auto_clutch ? e_auto_ : real(1.0);
    out.throttle_floor = s.assist_auto_blip ? throttle_floor : real(0.0);
    out.ignition_cut = s.assist_auto_blip && ignition_cut;
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
    return h;
}

} // namespace ps::drivetrain
