#include "auto_transmission.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::drivetrain {
namespace {
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * real(3.14159265358979323846));
constexpr real kHoldMargin = real(1.3);          // [AS] C_h over the torque handed over
constexpr real kHoldFloorFraction = real(0.15);  // [AS] of the rated capacity
constexpr real kInertiaBoost = real(1.5);        // [AS] on-coming capacity over C_h in a power-on upshift's inertia phase
constexpr real kPowerOnThrottle = real(0.1);     // [AS] downshift with the command above this = power on
constexpr real kInhibitRpm = real(300.0);        // [AS] no direction change above this input rpm in 1st / R1
constexpr real kLaunchRateUp = real(4.0), kLaunchRateDown = real(12.0), kLaunchRateStall = real(40.0); // 1/s [AS]
constexpr real kLaunchMinThrottle = real(0.05);

template <typename T>
std::uint64_t fnv1a(std::uint64_t h, const T& v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

real lerp(real a, real b, real t) { return a + (b - a) * t; }

real slew(real x, real target, real up_rate, real down_rate, real h) {
    if (target > x) return std::min(target, x + up_rate * h);
    return std::max(target, x - down_rate * h);
}
} // namespace

AutoTransmission::AutoTransmission(Desc desc) : desc_(std::move(desc)) {
    const int n = forward_count(), m = reverse_count();
    if (n < 1) throw std::runtime_error("AutoTransmission: needs at least one forward gear");
    if (desc_.initial_gear < -m || desc_.initial_gear > n) throw std::runtime_error("AutoTransmission: initial_gear out of range");
    reset(desc_.initial_gear, false);
}

real AutoTransmission::ratio(int gear) const {
    if (gear > 0) return desc_.forward_ratios[static_cast<std::size_t>(gear - 1)];
    if (gear < 0) return desc_.reverse_ratios[static_cast<std::size_t>(-gear - 1)];
    return real(1.0);
}

void AutoTransmission::reset(int gear, bool clutch_locked) {
    const int n = forward_count(), m = reverse_count();
    gear = std::clamp(gear, -m, n);
    cur_ = tgt_ = next_ = gear;
    slot_ = 0;
    phase_ = Phase::Idle;
    t_phase_ = real(0.0);
    c_hold_ = real(0.0);
    e_engage_ = real(1.0);
    upshift_ = power_on_ = false;
    pre_ = 0;
    pre_cand_ = 0;
    t_pre_ = real(0.0);
    selector_ = gear > 0 ? n : (gear < 0 ? -1 : 0);
    seeded_ = false;
    last_up_ = last_dn_ = 0;
    req_valid_ = false;
    last_req_ = 0;
    manual_ = false;
    manual_gear_ = std::max(gear, 1);
    manual_t_ = real(0.0);
    time_in_gear_ = real(0.0);
    kickdown_spent_ = false;
    lock_req_ = false;
    e_lock_ = real(0.0);
    launch_locked_ = clutch_locked && gear != 0;
    e_launch_ = launch_locked_ ? real(1.0) : real(0.0);
}

void AutoTransmission::choose_target(const Sensors& s, real th) {
    const int n = forward_count(), m = reverse_count();
    const AutomaticTcuDesc& t = desc_.tcu;
    // Selector and paddles, once per tick.
    if (s.first_substep) {
        const auto up = static_cast<std::int64_t>(std::floor(s.shift_up_count + real(0.5)));
        const auto dn = static_cast<std::int64_t>(std::floor(s.shift_down_count + real(0.5)));
        if (!seeded_) {
            last_up_ = up;
            last_dn_ = dn;
            seeded_ = true;
        }
        const auto d_up = std::max<std::int64_t>(0, up - last_up_);
        const auto d_dn = std::max<std::int64_t>(0, dn - last_dn_);
        last_up_ = up;
        last_dn_ = dn;
        if (s.gear_request > kGearRequestNone + real(0.5)) {
            const int req = static_cast<int>(std::floor(s.gear_request + real(0.5)));
            if (!req_valid_ || req != last_req_) {
                selector_ = req == 0 ? 0 : (req < 0 ? -std::min(-req, m) : std::min(req, n));
                manual_ = false;
            }
            req_valid_ = true;
            last_req_ = req;
        }
        if (selector_ > 0 && (d_up != 0 || d_dn != 0)) {
            if (!manual_) {
                manual_ = true;
                manual_gear_ = std::clamp(cur_ > 0 ? cur_ : 1, 1, selector_);
            }
            manual_gear_ = std::clamp(manual_gear_ + static_cast<int>(d_up) - static_cast<int>(d_dn), 1, selector_);
            manual_t_ = real(0.0);
        }
    }
    if (manual_) {
        manual_t_ += s.h;
        if (t.manual_timeout_s > real(0.0) && manual_t_ >= t.manual_timeout_s) manual_ = false;
    }
    if (th < t.kickdown_throttle - real(0.05)) kickdown_spent_ = false;

    const real w_out = s.output_omega;
    auto in_rpm = [&](int g) { return std::abs(ratio(g) * w_out) * kRadPerSecToRpm; };
    const real fwd_rpm = ratio(1) * w_out * kRadPerSecToRpm; // signed: > 0 rolling forward

    if (selector_ == 0) {
        tgt_ = 0;
        return;
    }
    if (selector_ < 0) {
        // R: refused while rolling forward (stays in N until slow enough).
        if (cur_ > 0 || (cur_ == 0 && fwd_rpm > kInhibitRpm)) {
            tgt_ = cur_ > 0 && fwd_rpm > kInhibitRpm ? cur_ : 0;
            if (fwd_rpm <= kInhibitRpm) tgt_ = 0;
            return;
        }
        tgt_ = std::max(selector_, -m);
        return;
    }
    // D.
    if (cur_ <= 0) {
        if (fwd_rpm < -kInhibitRpm && cur_ < 0) {
            tgt_ = cur_; // rolling back fast in R: hold R until slow
            return;
        }
        tgt_ = (cur_ < 0) ? 0 : (manual_ ? manual_gear_ : 1);
        return;
    }
    if (phase_ != Phase::Idle) return; // one decision per completed shift
    const int g = cur_;
    const ShiftScheduleDesc& sch = s.mode_channel >= real(0.5) ? t.sport_schedule : t.schedule;
    const real r = in_rpm(g);
    const real up = lerp(sch.up_rpm_at_zero_throttle, sch.up_rpm_at_full_throttle, th);
    const real dn = lerp(sch.down_rpm_at_zero_throttle, sch.down_rpm_at_full_throttle, th);
    int desired = g;
    if (manual_) {
        desired = manual_gear_;
        const real forced_up = t.manual_up_rpm > real(0.0) ? t.manual_up_rpm : sch.up_rpm_at_full_throttle;
        if (desired == g && r > forced_up && g < selector_) desired = g + 1;
        if (desired == g && r < sch.down_rpm_at_zero_throttle && g > 1) desired = g - 1;
        manual_gear_ = desired;
    } else if (g > selector_) {
        desired = g - 1;
    } else {
        const bool dwell = time_in_gear_ >= t.min_time_in_gear_s;
        bool kicked = false;
        if (!kickdown_spent_ && th >= t.kickdown_throttle && g > 1) {
            int best = g;
            for (int k = 1; k < g; ++k) {
                if (in_rpm(k) <= t.kickdown_max_rpm) {
                    best = k;
                    break;
                }
            }
            kickdown_spent_ = true;
            if (best < g) {
                desired = best;
                kicked = true;
            }
        }
        if (!kicked) {
            if (dwell && r > up && g < selector_ && in_rpm(g + 1) > dn) {
                desired = g + 1;
            } else if (g > 1 && in_rpm(g - 1) < up * real(0.95) &&
                       ((dwell && r < dn) || r < sch.down_rpm_at_zero_throttle)) {
                desired = g - 1; // below the zero-throttle line = stall guard, no dwell
            }
        }
    }
    tgt_ = desired;
}

void AutoTransmission::start_shift(int next, const Sensors& s, real th) {
    const real C = desc_.clutch.capacity_nm;
    next_ = next;
    upshift_ = std::abs(ratio(next)) < std::abs(ratio(cur_));
    power_on_ = th > kPowerOnThrottle;
    c_hold_ = std::min(C, kHoldMargin * std::max(std::abs(s.input_torque_prev), kHoldFloorFraction * C));
    t_phase_ = real(0.0);
    // A rolling DCT launch counts as done once a shift starts.
    launch_locked_ = true;
    e_launch_ = real(1.0);
    phase_ = (desc_.dual_clutch && desc_.clutch.preselect_time_s > real(0.0) && pre_ != next) ? Phase::Preselect : Phase::Torque;
    if (phase_ == Phase::Torque) pre_ = next;
}

void AutoTransmission::complete_shift() {
    const int old = cur_;
    cur_ = next_;
    slot_ = 1 - slot_;
    phase_ = Phase::Closing;
    t_phase_ = real(0.0);
    time_in_gear_ = real(0.0);
    pre_ = desc_.dual_clutch ? old : 0; // the old gear stays engaged on the now idle shaft
}

void AutoTransmission::update_launch(const Sensors& s, real th) {
    if (!desc_.dual_clutch) {
        e_launch_ = real(1.0);
        return;
    }
    if (cur_ == 0) {
        e_launch_ = real(0.0);
        launch_locked_ = false;
        return;
    }
    if (phase_ != Phase::Idle && phase_ != Phase::Engage) return;
    const AutomaticTcuDesc& t = desc_.tcu;
    const real ne = std::abs(s.engine_omega) * kRadPerSecToRpm;
    const real slip_rpm = std::abs(s.input_omega - ratio(cur_) * s.output_omega) * kRadPerSecToRpm;
    if (launch_locked_ && ne < t.launch_anti_stall_rpm) launch_locked_ = false;
    if (launch_locked_) {
        e_launch_ = real(1.0);
        return;
    }
    const real C = desc_.clutch.capacity_nm;
    const real full = lerp(t.launch_full_rpm_min, t.launch_full_rpm_max, th);
    real e_tgt = th >= kLaunchMinThrottle ? std::clamp((ne - t.launch_bite_rpm) / std::max(full - t.launch_bite_rpm, real(1.0)), real(0.0), real(1.0))
                                          : real(0.0);
    if (C > real(0.0)) e_tgt = std::max(e_tgt, std::min(real(1.0), t.creep_torque_nm / C));
    real down_rate = kLaunchRateDown;
    if (ne < t.launch_anti_stall_rpm) {
        e_tgt = real(0.0);
        down_rate = kLaunchRateStall;
    }
    e_launch_ = slew(e_launch_, e_tgt, kLaunchRateUp, down_rate, s.h);
    if (slip_rpm < t.launch_lock_slip_rpm && ne >= t.launch_bite_rpm && th >= kLaunchMinThrottle) {
        launch_locked_ = true;
        e_launch_ = real(1.0);
    }
}

// DualClutch: while a gear is held and no shift runs, the idle shaft preselects the gear the schedule will most likely
// ask for next (up above the midpoint of the current up/down lines, else down), so a scheduled shift goes straight to
// the clutch handover. A shift whose gear is not preselected yet still preselects first (Phase::Preselect).
void AutoTransmission::update_preselect(const Sensors& s, real th) {
    if (!desc_.dual_clutch || desc_.clutch.preselect_time_s <= real(0.0)) return;
    if (phase_ != Phase::Idle || cur_ <= 0) {
        pre_cand_ = 0;
        t_pre_ = real(0.0);
        return;
    }
    const AutomaticTcuDesc& t = desc_.tcu;
    const ShiftScheduleDesc& sch = s.mode_channel >= real(0.5) ? t.sport_schedule : t.schedule;
    const real r = std::abs(ratio(cur_) * s.output_omega) * kRadPerSecToRpm;
    const real up = lerp(sch.up_rpm_at_zero_throttle, sch.up_rpm_at_full_throttle, th);
    const real dn = lerp(sch.down_rpm_at_zero_throttle, sch.down_rpm_at_full_throttle, th);
    int want = r > real(0.5) * (up + dn) ? cur_ + 1 : cur_ - 1;
    if (want > forward_count()) want = cur_ - 1;
    if (want < 1) want = cur_ + 1 <= forward_count() ? cur_ + 1 : 0;
    if (want == 0 || want == pre_) {
        pre_cand_ = 0;
        t_pre_ = real(0.0);
        return;
    }
    if (want != pre_cand_) {
        pre_cand_ = want;
        t_pre_ = real(0.0);
    }
    t_pre_ += s.h;
    if (t_pre_ >= desc_.clutch.preselect_time_s) {
        pre_ = pre_cand_;
        pre_cand_ = 0;
        t_pre_ = real(0.0);
    }
}

void AutoTransmission::update_lockup(const Sensors& s, real th) {
    if (!desc_.has_lockup) return;
    const AutomaticTcuDesc& t = desc_.tcu;
    const real turbine_rpm = std::abs(s.input_omega) * kRadPerSecToRpm;
    const bool shifting = phase_ != Phase::Idle;
    if (lock_req_) {
        if (!t.lockup_enabled || shifting || cur_ < t.lockup_min_gear || turbine_rpm < t.lockup_release_rpm ||
            th > t.lockup_max_throttle + real(0.1))
            lock_req_ = false;
    } else if (t.lockup_enabled && !shifting && cur_ >= t.lockup_min_gear && turbine_rpm > t.lockup_engage_rpm &&
               th <= t.lockup_max_throttle) {
        lock_req_ = true;
    }
    e_lock_ = slew(e_lock_, lock_req_ ? real(1.0) : real(0.0), real(1.0) / desc_.lockup_engage_time_s,
                   real(1.0) / desc_.lockup_release_time_s, s.h);
}

AutoTransmission::Outputs AutoTransmission::step(const Sensors& s) {
    const real th = std::clamp(s.throttle, real(0.0), real(1.0));
    const ShiftClutchDesc& cd = desc_.clutch;
    const real C = cd.capacity_nm;
    time_in_gear_ += s.h;

    choose_target(s, th);

    // --- the shift machine ---
    switch (phase_) {
        case Phase::Idle:
        case Phase::Closing:
        case Phase::Engage:
            if (phase_ == Phase::Closing) {
                t_phase_ += s.h;
                if (t_phase_ >= cd.engage_time_s) phase_ = Phase::Idle;
            } else if (phase_ == Phase::Engage) {
                t_phase_ += s.h;
                e_engage_ = std::min(real(1.0), t_phase_ / cd.engage_time_s);
                if (e_engage_ >= real(1.0)) phase_ = Phase::Idle;
            }
            if (tgt_ != cur_ && (phase_ == Phase::Idle || tgt_ == 0 || (tgt_ > 0) != (cur_ > 0))) {
                if (tgt_ == 0 || cur_ == 0 || (tgt_ > 0) != (cur_ > 0)) {
                    if (cur_ != 0) {
                        cur_ = 0; // release to N (a direction change goes through N)
                        phase_ = Phase::Idle;
                        pre_ = 0;
                    } else {
                        cur_ = tgt_;
                        phase_ = Phase::Engage;
                        t_phase_ = real(0.0);
                        e_engage_ = real(0.0);
                        time_in_gear_ = real(0.0);
                        pre_ = 0;
                    }
                } else {
                    const int step = desc_.dual_clutch ? cur_ + (tgt_ > cur_ ? 1 : -1) : tgt_;
                    start_shift(step, s, th);
                }
            }
            break;
        case Phase::Preselect:
            t_phase_ += s.h;
            if (t_phase_ >= cd.preselect_time_s) {
                pre_ = next_;
                phase_ = Phase::Torque;
                t_phase_ = real(0.0);
                c_hold_ = std::min(C, kHoldMargin * std::max(std::abs(s.input_torque_prev), kHoldFloorFraction * C));
            }
            break;
        case Phase::Torque:
            t_phase_ += s.h;
            if (t_phase_ >= cd.torque_phase_s) {
                phase_ = Phase::Inertia;
                t_phase_ = real(0.0);
            }
            break;
        case Phase::Inertia: {
            t_phase_ += s.h;
            const real slip_on = s.input_omega - ratio(next_) * s.output_omega;
            if (std::abs(slip_on) < cd.sync_slip_rad_s || t_phase_ >= cd.max_inertia_phase_s) complete_shift();
            break;
        }
    }
    if (cur_ == 0) {
        e_engage_ = real(0.0);
    }

    update_launch(s, th);
    update_lockup(s, th);
    update_preselect(s, th);

    // --- row commands ---
    Outputs o;
    RowCmd& act = o.row[slot_];
    RowCmd& onc = o.row[1 - slot_];
    act.ratio = ratio(cur_);
    onc.ratio = ratio(phase_ == Phase::Idle || phase_ == Phase::Engage || phase_ == Phase::Closing ? (pre_ != 0 ? pre_ : cur_) : next_);
    const real e = desc_.dual_clutch ? e_launch_ : real(1.0);
    switch (phase_) {
        case Phase::Idle:
            act.cap_kinetic = cur_ != 0 ? C * e : real(0.0);
            break;
        case Phase::Engage:
            act.cap_kinetic = C * e_engage_ * e;
            break;
        case Phase::Preselect:
            act.cap_kinetic = C * e;
            break;
        case Phase::Torque: {
            const real x = std::min(real(1.0), t_phase_ / cd.torque_phase_s);
            act.cap_kinetic = (real(1.0) - x) * c_hold_;
            onc.cap_kinetic = (upshift_ || !power_on_) ? x * c_hold_ : real(0.0);
            break;
        }
        case Phase::Inertia:
            // Power-on upshift: the on-coming clutch is over-pressed (and the engine torque cut by
            // upshift_torque_fraction) so the input drops to the new gear's speed in tens of ms.
            onc.cap_kinetic = (upshift_ && power_on_) ? std::min(C, kInertiaBoost * c_hold_)
                                                      : ((upshift_ || !power_on_) ? c_hold_ : real(0.0));
            break;
        case Phase::Closing: {
            const real x = std::min(real(1.0), t_phase_ / cd.engage_time_s);
            act.cap_kinetic = lerp(c_hold_, C, x) * e;
            break;
        }
    }
    act.cap_static = cd.static_factor * act.cap_kinetic;
    onc.cap_static = cd.static_factor * onc.cap_kinetic;
    if (desc_.has_lockup) {
        o.lockup_cap_kinetic = e_lock_ * desc_.lockup_capacity_nm;
        o.lockup_cap_static = desc_.lockup_static_factor * o.lockup_cap_kinetic;
    }
    o.throttle_ceiling = (phase_ == Phase::Inertia && upshift_ && power_on_) ? desc_.tcu.upshift_torque_fraction : real(1.0);
    o.current = cur_;
    o.target = tgt_;
    o.phase = phase_;
    o.launch_engagement = desc_.dual_clutch ? (phase_ == Phase::Idle || phase_ == Phase::Engage ? e_launch_ * (phase_ == Phase::Engage ? e_engage_ : real(1.0)) : real(1.0))
                                            : real(1.0);
    if (cur_ == 0) o.launch_engagement = real(0.0);
    return o;
}

std::uint64_t AutoTransmission::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    h = fnv1a(h, cur_);
    h = fnv1a(h, tgt_);
    h = fnv1a(h, next_);
    h = fnv1a(h, slot_);
    h = fnv1a(h, phase_);
    h = fnv1a(h, t_phase_);
    h = fnv1a(h, c_hold_);
    h = fnv1a(h, e_engage_);
    h = fnv1a(h, upshift_);
    h = fnv1a(h, power_on_);
    h = fnv1a(h, pre_);
    h = fnv1a(h, pre_cand_);
    h = fnv1a(h, t_pre_);
    h = fnv1a(h, selector_);
    h = fnv1a(h, seeded_);
    h = fnv1a(h, last_up_);
    h = fnv1a(h, last_dn_);
    h = fnv1a(h, req_valid_);
    h = fnv1a(h, last_req_);
    h = fnv1a(h, manual_);
    h = fnv1a(h, manual_gear_);
    h = fnv1a(h, manual_t_);
    h = fnv1a(h, time_in_gear_);
    h = fnv1a(h, kickdown_spent_);
    h = fnv1a(h, lock_req_);
    h = fnv1a(h, e_lock_);
    h = fnv1a(h, e_launch_);
    h = fnv1a(h, launch_locked_);
    return h;
}

} // namespace ps::drivetrain
