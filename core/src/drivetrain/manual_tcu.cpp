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
    }

    real e_tgt = e_auto_;
    real rate = desc_.auto_clutch.release_rate_per_s;
    if (mode_ == ClutchMode::Shift) {
        if (gearbox_shifting_now) {
            e_tgt = real(0.0);
            rate = real(1.0) / desc_.auto_clutch.open_time_s;
        } else {
            e_tgt = real(1.0);
            rate = real(1.0) / desc_.auto_clutch.close_time_s;
        }
    } else if (mode_ == ClutchMode::Locked) {
        if (rpm < desc_.auto_clutch.anti_stall_rpm) mode_ = ClutchMode::Launch;
        else {
            e_tgt = real(1.0);
            rate = desc_.auto_clutch.engage_rate_per_s;
        }
    }
    if (mode_ == ClutchMode::Launch) {
        if (s.gearbox_current == 0 && s.gearbox_phase == Gearbox::Phase::Idle) {
            e_tgt = real(1.0);
        } else {
            const real full = desc_.auto_clutch.full_rpm_min +
                               s.throttle_pedal * (desc_.auto_clutch.full_rpm_max - desc_.auto_clutch.full_rpm_min);
            const real denom = full - desc_.auto_clutch.bite_rpm;
            e_tgt = denom > real(0.0) ? std::clamp((rpm - desc_.auto_clutch.bite_rpm) / denom, real(0.0), real(1.0)) : real(1.0);
        }
        rate = (e_tgt >= e_auto_) ? desc_.auto_clutch.engage_rate_per_s : desc_.auto_clutch.release_rate_per_s;
        const real slip_rpm = std::abs(s.engine_omega - s.gearbox_in_omega) * kRadPerSecToRpm;
        if (slip_rpm < desc_.auto_clutch.lock_slip_rpm && rpm > desc_.auto_clutch.bite_rpm && s.gearbox_current != 0) {
            mode_ = ClutchMode::Locked;
            e_tgt = real(1.0);
            rate = desc_.auto_clutch.engage_rate_per_s;
        }
    }
    e_auto_ = move_toward(e_auto_, e_tgt, rate * h);
    if (mode_ == ClutchMode::Shift && !gearbox_shifting_now && e_auto_ >= real(1.0) - real(1e-6)) {
        const real slip_rpm = std::abs(s.engine_omega - s.gearbox_in_omega) * kRadPerSecToRpm;
        mode_ = (slip_rpm < desc_.auto_clutch.lock_slip_rpm) ? ClutchMode::Locked : ClutchMode::Launch;
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
    if (desc_.auto_blip.upshift_cut) {
        const bool upshifting = desired_target > s.gearbox_current && s.gearbox_current > 0;
        ignition_cut = upshifting && gearbox_shifting_now;
    }

    ManualTcu::Outputs out;
    out.gearbox_target = desired_target;
    out.e_auto = s.assist_auto_clutch ? e_auto_ : real(1.0);
    out.throttle_floor = s.assist_auto_blip ? throttle_floor : real(0.0);
    out.ignition_cut = s.assist_auto_blip && ignition_cut;
    return out;
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
    return h;
}

} // namespace ps::drivetrain
