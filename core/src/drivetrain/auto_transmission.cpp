#include "auto_transmission.h"

#include "ps/math/transcendental.h"

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
constexpr real kLaunchRateUp = real(4.0), kLaunchRateDown = real(12.0); // 1/s [AS]
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
    if (desc_.cvt) {
        if (!desc_.tcu.cvt) throw std::runtime_error("AutoTransmission: a cvt gearbox needs the automatic_tcu's cvt block");
        if (desc_.dual_clutch) throw std::runtime_error("AutoTransmission: a cvt is not a dual-clutch box");
        cos_beta_ = ps::math::cos(desc_.cvt_desc.belt.sheave_half_angle_deg * real(3.14159265358979323846) / real(180.0));
    }
    if (desc_.hydro) {
        if (!desc_.tcu.hydro) throw std::runtime_error("AutoTransmission: a hydro_cvt gearbox needs the automatic_tcu's hydro block");
        if (desc_.dual_clutch || desc_.cvt || desc_.powershift || desc_.has_lockup) throw std::runtime_error("AutoTransmission: a hydro_cvt is its own kind");
        if (n != 1 || m != 1) throw std::runtime_error("AutoTransmission: a hydro_cvt has one forward and one reverse position (D and R)");
    } else if (desc_.tcu.hydro) {
        throw std::runtime_error("AutoTransmission: the automatic_tcu's hydro block is for a hydro_cvt gearbox only");
    }
    if (desc_.powershift) {
        if (!desc_.tcu.powershift) throw std::runtime_error("AutoTransmission: a powershift gearbox needs the automatic_tcu's powershift block");
        if (desc_.dual_clutch || desc_.cvt) throw std::runtime_error("AutoTransmission: a powershift box is neither a dual-clutch nor a cvt box");
        const PowershiftTcuDesc& ps = *desc_.tcu.powershift;
        if (ps.start_gear < 1 || ps.start_gear > n) throw std::runtime_error("AutoTransmission: powershift start_gear out of range");
        if (!(ps.target_fraction > real(0.0)) || !(ps.target_fraction < real(1.0)) || ps.max_skip < 1) {
            throw std::runtime_error("AutoTransmission: powershift target_fraction must be in (0, 1) and max_skip >= 1");
        }
        if (!desc_.forward_range.empty() && static_cast<int>(desc_.forward_range.size()) != n) {
            throw std::runtime_error("AutoTransmission: forward_range must have one entry per forward gear");
        }
    } else if (desc_.tcu.powershift) {
        throw std::runtime_error("AutoTransmission: the automatic_tcu's powershift block is for a powershift gearbox only");
    }
    reset(desc_.initial_gear, false);
}

int AutoTransmission::range_of(int gear) const {
    if (gear <= 0 || desc_.forward_range.empty()) return 0;
    return desc_.forward_range[static_cast<std::size_t>(gear - 1)];
}

real AutoTransmission::ratio(int gear) const {
    if (gear != 0 && desc_.hydro) return ratio_; // tractor plan stage 2: k = w_out / w_in, signed
    if (gear > 0 && desc_.cvt) return ratio_; // CVT plan C1: D is the variator
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
    rev_held_ = false;
    e_launch_ = launch_locked_ ? real(1.0) : real(0.0);
    if (desc_.hydro) {
        ratio_ = real(0.0);
        v_ref_ = real(0.0);
        n_target_ = desc_.tcu.hydro->idle_rpm;
        lever_i_ = real(0.0);
        a_ref_ = real(0.0);
        engine_cmd_ = real(-1.0);
    }
    if (desc_.cvt) {
        ratio_ = hold_ratio_ > real(0.0) ? hold_ratio_ : desc_.cvt_desc.ratio_low;
        step_ = 0;
        t_step_ = real(0.0);
        clamp_ = hold_ ? desc_.cvt_desc.belt.clamp_force_max_n : desc_.cvt_desc.belt.clamp_force_min_n;
    }
}

void AutoTransmission::set_dyno_hold(bool on, real cvt_ratio) {
    hold_ = on;
    hold_ratio_ = 0.0;
    if (!on || !desc_.cvt || !(cvt_ratio > real(0.0))) return;
    hold_ratio_ = std::clamp(cvt_ratio, desc_.cvt_desc.ratio_high, desc_.cvt_desc.ratio_low);
    ratio_ = hold_ratio_;
    clamp_ = desc_.cvt_desc.belt.clamp_force_max_n;
}

int AutoTransmission::nearest_step() const {
    int best = 1;
    real best_d = std::abs(desc_.forward_ratios[0] - ratio_) / desc_.forward_ratios[0];
    for (int k = 2; k <= forward_count(); ++k) {
        const real r = desc_.forward_ratios[static_cast<std::size_t>(k - 1)];
        const real d = std::abs(r - ratio_) / r;
        if (d < best_d) {
            best_d = d;
            best = k;
        }
    }
    return best;
}

real AutoTransmission::belt_capacity() const {
    const CvtBeltDesc& b = desc_.cvt_desc.belt;
    const real rp = b.pulley_radius_sum_m / (real(1.0) + ratio_);
    return real(2.0) * b.friction_coefficient * clamp_ * rp / cos_beta_;
}

// CVT plan C1: the variator's target ratio (continuous or stepped), its rate-limited motion and the clamping force.
ShiftScheduleDesc AutoTransmission::active_schedule(const Sensors& s) const {
    ShiftScheduleDesc sch = s.mode_channel >= real(0.5) ? desc_.tcu.sport_schedule : desc_.tcu.schedule;
    if (std::isfinite(s.up_rpm_at_zero_throttle)) sch.up_rpm_at_zero_throttle = s.up_rpm_at_zero_throttle;
    if (std::isfinite(s.up_rpm_at_full_throttle)) sch.up_rpm_at_full_throttle = s.up_rpm_at_full_throttle;
    if (std::isfinite(s.down_rpm_at_zero_throttle)) sch.down_rpm_at_zero_throttle = s.down_rpm_at_zero_throttle;
    if (std::isfinite(s.down_rpm_at_full_throttle)) sch.down_rpm_at_full_throttle = s.down_rpm_at_full_throttle;
    return sch;
}

void AutoTransmission::update_cvt(const Sensors& s, real th) {
    if (hold_) { // powertrain dyno: the variator stays where it was put
        step_ = 0;
        if (hold_ratio_ > real(0.0)) ratio_ = hold_ratio_;
        update_cvt_clamp(s);
        return;
    }
    const CvtDesc& c = desc_.cvt_desc;
    const AutomaticTcuDesc& t = desc_.tcu;
    const CvtTcuDesc& law = *t.cvt;
    const real out_rpm = std::abs(s.output_omega) * kRadPerSecToRpm;
    const bool sport = s.mode_channel >= real(0.5);
    const int n = forward_count();
    const int top = std::clamp(selector_ > 0 ? selector_ : n, 1, n);
    real r_t = c.ratio_low;
    real rate = c.ratio_rate_per_s;
    auto step_rpm = [&](int k) { return desc_.forward_ratios[static_cast<std::size_t>(k - 1)] * out_rpm; };
    t_step_ += s.h;
    if (cur_ > 0 && (manual_ || (sport && law.stepped_sport))) {
        const ShiftScheduleDesc sch = active_schedule(s);
        if (manual_) {
            manual_gear_ = std::clamp(manual_gear_, 1, top);
            const real forced_up = t.manual_up_rpm > real(0.0) ? t.manual_up_rpm : sch.up_rpm_at_full_throttle;
            if (step_rpm(manual_gear_) > forced_up && manual_gear_ < top) ++manual_gear_;
            else if (step_rpm(manual_gear_) < sch.down_rpm_at_zero_throttle && manual_gear_ > 1) --manual_gear_;
            if (manual_gear_ != step_) t_step_ = real(0.0);
            step_ = manual_gear_;
        } else {
            if (step_ == 0) {
                step_ = std::min(nearest_step(), top);
                t_step_ = real(0.0);
            }
            const real up = lerp(sch.up_rpm_at_zero_throttle, sch.up_rpm_at_full_throttle, th);
            const real dn = lerp(sch.down_rpm_at_zero_throttle, sch.down_rpm_at_full_throttle, th);
            const bool dwell = t_step_ >= t.min_time_in_gear_s;
            const real r = step_rpm(step_);
            if (dwell && r > up && step_ < top && step_rpm(step_ + 1) > dn) {
                ++step_;
                t_step_ = real(0.0);
            } else if (step_ > 1 && step_rpm(step_ - 1) < up * real(0.95) && ((dwell && r < dn) || r < sch.down_rpm_at_zero_throttle)) {
                --step_;
                t_step_ = real(0.0);
            } else if (step_ > top) {
                step_ = top;
                t_step_ = real(0.0);
            }
        }
        r_t = desc_.forward_ratios[static_cast<std::size_t>(step_ - 1)];
        rate = c.step_rate_per_s;
    } else {
        step_ = 0;
        if (cur_ > 0) {
            const vehicle::Curve1D& curve =
                (sport && !law.stepped_sport && !law.sport_target_rpm_vs_throttle.x.empty()) ? law.sport_target_rpm_vs_throttle : law.target_rpm_vs_throttle;
            real n_t = curve.evaluate(th);
            if (!law.min_rpm_vs_output_rpm.x.empty()) n_t = std::max(n_t, law.min_rpm_vs_output_rpm.evaluate(out_rpm));
            r_t = out_rpm > real(1.0) ? std::clamp(n_t / out_rpm, c.ratio_high, c.ratio_low) : c.ratio_low;
        }
    }
    // A launch-clutch CVT stays in the launch ratio until its clutch has locked.
    if (cur_ > 0 && slip_launch() && !launch_locked_) r_t = c.ratio_low;
    if (r_t != ratio_) {
        const real lr = ps::math::log(ratio_), lt = ps::math::log(r_t);
        const real d = rate * s.h;
        ratio_ = (std::abs(lt - lr) <= d) ? r_t : ps::math::exp(lr + (lt > lr ? d : -d));
        ratio_ = std::clamp(ratio_, c.ratio_high, c.ratio_low);
    }
    update_cvt_clamp(s);
}

// Clamping force: lag toward the force that carries max(|input torque|, |engine demand|) x the safety factor.
// Tractor plan stage 2 (HydroTcuDesc): the output speed reference, the engine rpm target (least fuel at the demanded power),
// the governor lever that realises it, and the ratio k = reference / engine speed.
void AutoTransmission::update_hydro(const Sensors& s, real th) {
    const HydroCvtDesc& hd = desc_.hydro_desc;
    const HydroTcuDesc& law = *desc_.tcu.hydro;
    constexpr real kRpmToRad = real(1.0) / kRadPerSecToRpm;
    const real w_den = std::max(std::abs(s.input_omega), real(0.5) * law.idle_rpm * kRpmToRad);
    engine_cmd_ = real(-1.0);
    if (cur_ == 0 || phase_ == Phase::Engage) { // N, or the direction clutch filling: follow the measured ratio
        ratio_ = std::clamp(s.output_omega / w_den, -hd.ratio_reverse_max, hd.ratio_forward_max);
        v_ref_ = s.output_omega;
        a_ref_ = real(0.0);
        lever_i_ = real(0.0);
        n_target_ = std::abs(s.engine_omega) * kRadPerSecToRpm;
        return;
    }
    const real dir = cur_ > 0 ? real(1.0) : real(-1.0);
    const real w_t = dir * (cur_ > 0 ? real(1.0) : law.reverse_speed_fraction) * law.speed_rpm_vs_command.evaluate(th) * kRpmToRad;
    // S-curve: the acceleration follows a proportional approach to the target, limited in size and in build-up rate.
    const real a_max = law.speed_ramp_rpm_per_s * kRpmToRad;
    const real a_tgt = std::clamp((w_t - v_ref_) * real(2.0), -a_max, a_max);
    const real da = law.speed_jerk_rpm_per_s2 * kRpmToRad * s.h;
    a_ref_ += std::clamp(a_tgt - a_ref_, -da, da);
    v_ref_ += a_ref_ * s.h;
    const real lead = law.max_lead_rpm * kRpmToRad;
    v_ref_ = std::clamp(v_ref_, s.output_omega - lead, s.output_omega + lead);
    // Demanded power: what flows, or what the speed deficit asks for.
    const real deficit = std::clamp((std::abs(w_t) - dir * s.output_omega) / (law.speed_deficit_rpm * kRpmToRad), real(0.0), real(1.0));
    const real p_load = std::abs(s.input_torque_prev) * std::abs(s.input_omega) / law.reference_power_w;
    const real p = std::clamp(std::max(p_load, std::min(deficit, law.deficit_power_max)), real(0.0), real(1.0));
    const real n_raw = law.engine_rpm_vs_power.evaluate(p);
    n_target_ += (n_raw - n_target_) * std::min(real(1.0), s.h / law.engine_rpm_tau_s);
    const real n_e = std::abs(s.engine_omega) * kRadPerSecToRpm;
    const real u0 = (n_target_ - law.idle_rpm) / (law.high_idle_rpm - law.idle_rpm);
    const real u_unsat = u0 + lever_i_;
    if ((u_unsat < real(1.0) || n_target_ - n_e < real(0.0)) && (u_unsat > real(0.0) || n_target_ - n_e > real(0.0))) {
        lever_i_ = std::clamp(lever_i_ + law.lever_ki_per_rpm_s * (n_target_ - n_e) * s.h, real(-0.3), real(0.4));
    }
    engine_cmd_ = std::clamp(u0 + lever_i_, real(0.0), real(1.0));
    // Ratio: the reference over the engine speed, unloading the engine below its stall band.
    const real n_ok = std::max(law.stall_rpm_ok, law.lug_fraction * n_target_);
    const real f = std::clamp((n_e - law.stall_rpm_min) / std::max(n_ok - law.stall_rpm_min, real(1.0)), real(0.0), real(1.0));
    real k_cmd = v_ref_ * f / w_den;
    k_cmd = cur_ > 0 ? std::clamp(k_cmd, real(0.0), hd.ratio_forward_max) : std::clamp(k_cmd, -hd.ratio_reverse_max, real(0.0));
    const real dk = hd.ratio_rate_per_s * s.h;
    ratio_ += std::clamp(k_cmd - ratio_, -dk, dk);
}

void AutoTransmission::update_cvt_clamp(const Sensors& s) {
    const real th = std::clamp(s.throttle, real(0.0), real(1.0));
    const CvtDesc& c = desc_.cvt_desc;
    const CvtBeltDesc& b = c.belt;
    const real rp = b.pulley_radius_sum_m / (real(1.0) + ratio_);
    const real per_n = real(2.0) * b.friction_coefficient * rp / cos_beta_; // N*m of capacity per N of clamping force
    real t_in = std::max(std::abs(s.input_torque_prev), std::abs(s.engine_torque_demand));
    // During a starting-clutch launch the clamp is raised to what the clutch can carry once it locks, so the lock-up
    // (which syncs the engine through the clutch's static capacity) never slips the belt. Not while creeping (no throttle).
    if (cur_ > 0 && slip_launch() && !launch_locked_ && th >= kLaunchMinThrottle)
        t_in = std::max(t_in, desc_.clutch.static_factor * desc_.clutch.capacity_nm);
    const real f_t = std::clamp(b.clamp_safety_factor * t_in / per_n, b.clamp_force_min_n, b.clamp_force_max_n);
    clamp_ += (f_t - clamp_) * std::min(real(1.0), s.h / b.clamp_time_constant_s);
}

void AutoTransmission::choose_target(const Sensors& s, real th) {
    const int n = forward_count(), m = reverse_count();
    if (hold_) { // powertrain dyno: the requested gear, no selector, schedule, kickdown or dwell
        if (s.gear_request > kGearRequestNone + real(0.5)) {
            const int req = static_cast<int>(std::floor(s.gear_request + real(0.5)));
            tgt_ = req == 0 ? 0 : (req < 0 ? -std::min(-req, m) : ((desc_.cvt || desc_.hydro) ? 1 : std::min(req, n)));
        }
        return;
    }
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
                // CVT plan C1: manual mode starts on the virtual step nearest the variator's ratio.
                manual_gear_ = std::clamp(desc_.cvt ? (cur_ > 0 ? nearest_step() : 1) : (cur_ > 0 ? cur_ : 1), 1, selector_);
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
    const real fwd_rpm = (desc_.hydro ? w_out : ratio(1) * w_out) * kRadPerSecToRpm; // signed: > 0 rolling forward

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
        // D from N: the gear the schedule fits at the present speed (1 at standstill). Engaging 1st while rolling fast made
        // the box shift up through every gear (or over-rev the engine) after a hybrid's ICE connected at speed.
        tgt_ = (cur_ < 0) ? 0 : ((manual_ && !desc_.cvt && !desc_.hydro) ? manual_gear_ : ((desc_.cvt || desc_.hydro) ? 1 : fit_gear(s, th)));
        return;
    }
    if (desc_.hydro) { // tractor plan stage 2: D is one position; the ratio law runs in update_hydro
        tgt_ = 1;
        return;
    }
    if (desc_.cvt) { // CVT plan C1: D is one gear; the variator law runs in update_cvt
        tgt_ = 1;
        return;
    }
    if (phase_ != Phase::Idle) return; // one decision per completed shift
    const int g = cur_;
    const ShiftScheduleDesc sch = active_schedule(s);
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
        desired = desc_.powershift ? selector_ : g - 1;
    } else if (desc_.powershift) {
        desired = powershift_desired(s, th, g);
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

// Tractor plan stage 1 (PowershiftTcuDesc): hold the engine in the rpm band of the driver's command. Aims at
// target = down + target_fraction * (up - down); an upshift may skip gears, a downshift goes straight to the gear that
// restores the target - never one gear per dwell, and never below the start gear unless a lower gear reaches the target. A
// downshift below the zero-throttle line (stall guard) ignores the dwell.
int AutoTransmission::powershift_desired(const Sensors& s, real th, int g) const {
    const PowershiftTcuDesc& ps = *desc_.tcu.powershift;
    const AutomaticTcuDesc& t = desc_.tcu;
    const ShiftScheduleDesc sch = active_schedule(s);
    const real w_out = s.output_omega;
    auto in_rpm = [&](int k) { return std::abs(ratio(k) * w_out) * kRadPerSecToRpm; };
    const real r = in_rpm(g);
    const real up = lerp(sch.up_rpm_at_zero_throttle, sch.up_rpm_at_full_throttle, th);
    const real dn = lerp(sch.down_rpm_at_zero_throttle, sch.down_rpm_at_full_throttle, th);
    const real target = dn + ps.target_fraction * (up - dn);
    const bool dwell = time_in_gear_ >= t.min_time_in_gear_s;
    const int n = forward_count();
    const int top = std::clamp(selector_ > 0 ? selector_ : n, 1, n);
    if (dwell && r > up && g < top && in_rpm(g + 1) > dn) {
        int best = g + 1;
        for (int k = g + 2; k <= std::min(top, g + ps.max_skip); ++k) {
            if (in_rpm(k) < target) break;
            best = k;
        }
        return best;
    }
    const bool stall_guard = r < sch.down_rpm_at_zero_throttle;
    if (g > 1 && ((dwell && r < dn) || stall_guard)) {
        // No gear reaches the target (a crawl or a standstill): the start gear, never lower than it - a lower gear
        // that cannot lift the engine into the band is no use, and a stop ends in the gear the tractor pulls away in.
        int best = std::min(ps.start_gear, g);
        for (int k = g - 1; k >= 1; --k) {
            if (in_rpm(k) >= target) {
                best = k;
                break;
            }
        }
        if (best == g) return g;
        // Never into a gear the upshift line would send straight back (hunting) unless the engine is about to stall.
        if (!stall_guard && in_rpm(best) >= up * real(0.95)) return g;
        return best;
    }
    return g;
}

int AutoTransmission::fit_gear(const Sensors& s, real th) const {
    const int top = std::clamp(selector_ > 0 ? selector_ : forward_count(), 1, forward_count());
    if (s.output_omega * ratio(1) <= real(0.0)) return desc_.powershift ? std::min(desc_.tcu.powershift->start_gear, top) : 1;
    const ShiftScheduleDesc sch = active_schedule(s);
    const real up = lerp(sch.up_rpm_at_zero_throttle, sch.up_rpm_at_full_throttle, th);
    // Powershift: never below the start gear (a creep or a jitter at standstill must not pick gear 1 of 24).
    for (int g = desc_.powershift ? std::min(desc_.tcu.powershift->start_gear, top) : 1; g < top; ++g)
        if (std::abs(ratio(g) * s.output_omega) * kRadPerSecToRpm <= up) return g;
    return top;
}

// The clutch pressure a shift holds. Powershift (tractor plan stage 1): the engine's own demand counts as well as the torque
// the rows carried last substep (a governed diesel near its top speed is lightly loaded at the decision, its demand is
// back at full the moment its rpm falls) and the floor is the data's shift_pressure_fraction of the rated capacity.
real AutoTransmission::hold_capacity(const Sensors& s) const {
    const real C = desc_.clutch.capacity_nm;
    if (!desc_.powershift) return std::min(C, kHoldMargin * std::max(std::abs(s.input_torque_prev), kHoldFloorFraction * C));
    const real t = std::max(std::abs(s.input_torque_prev), std::max(real(0.0), s.engine_torque_demand));
    return std::min(C, kHoldMargin * std::max(t, desc_.tcu.powershift->shift_pressure_fraction * C));
}

void AutoTransmission::start_shift(int next, const Sensors& s, real th) {
    next_ = next;
    upshift_ = std::abs(ratio(next)) < std::abs(ratio(cur_));
    power_on_ = th > kPowerOnThrottle;
    c_hold_ = hold_capacity(s);
    t_phase_ = real(0.0);
    // A rolling DCT launch counts as done once a shift starts.
    launch_locked_ = true;
    e_launch_ = real(1.0);
    phase_ = (desc_.dual_clutch && desc_.clutch.preselect_time_s > real(0.0) && pre_ != next) ? Phase::Preselect : Phase::Torque;
    if (phase_ == Phase::Torque) pre_ = next;
    // Tractor plan stage 1: a shift into another range first opens both clutches (the range selector moves).
    if (desc_.powershift && desc_.range_change_time_s > real(0.0) && next > 0 && cur_ > 0 && range_of(next) != range_of(cur_)) phase_ = Phase::Range;
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
    if (!slip_launch()) {
        e_launch_ = real(1.0);
        return;
    }
    if (cur_ == 0) {
        e_launch_ = real(0.0);
        launch_locked_ = false;
        return;
    }
    // Emergency pressure dump must run during handover/closing too. A normal
    // release ramp can drag a light crank through its stall threshold in one
    // wheel-lock transient; rotation against the engine is not healthy rpm.
    if (s.engine_omega * kRadPerSecToRpm < desc_.tcu.launch_anti_stall_rpm) {
        e_launch_ = real(0.0);
        launch_locked_ = false;
        return;
    }
    if (s.clutch_capacity_cap_nm < desc_.clutch.capacity_nm) {
        launch_locked_ = false;
        e_launch_ = std::min(e_launch_, std::max(real(0.0), s.clutch_capacity_cap_nm / desc_.clutch.capacity_nm));
    }
    if (phase_ != Phase::Idle && phase_ != Phase::Engage && phase_ != Phase::Closing) return;
    if (s.rev_match_hold) {
        e_launch_ = real(0.0);
        launch_locked_ = false;
        rev_held_ = true;
        return;
    }
    if (rev_held_) {
        rev_held_ = false;
        const real slip = std::abs(s.input_omega - ratio(cur_) * s.output_omega) * kRadPerSecToRpm;
        if (slip < desc_.tcu.launch_lock_slip_rpm) {
            launch_locked_ = true;
            e_launch_ = real(1.0);
            return;
        }
    }
    if (hold_) { // powertrain dyno: a gear in = the starting clutch closed
        launch_locked_ = true;
        e_launch_ = real(1.0);
        return;
    }
    // DCT launch control (assist unit): Armed holds the clutch open, Launch sets its capacity directly.
    if (s.launch_hold_open) {
        e_launch_ = real(0.0);
        launch_locked_ = false;
        return;
    }
    if (s.launch_cap_nm >= real(0.0)) {
        const real C = desc_.clutch.capacity_nm;
        e_launch_ = C > real(0.0) ? std::clamp(s.launch_cap_nm / C, real(0.0), real(1.0)) : real(0.0);
        launch_locked_ = false;
        return;
    }
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
    e_launch_ = slew(e_launch_, e_tgt, kLaunchRateUp, kLaunchRateDown, s.h);
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
    const ShiftScheduleDesc sch = active_schedule(s);
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
    if (hold_) { // powertrain dyno: converter locked in every gear
        lock_req_ = cur_ != 0;
        e_lock_ = lock_req_ ? real(1.0) : real(0.0);
        return;
    }
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
    ShiftClutchDesc cd = desc_.clutch;
    if (std::isfinite(s.close_time_s)) cd.engage_time_s = s.close_time_s;
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
                c_hold_ = hold_capacity(s);
            }
            break;
        case Phase::Range:
            t_phase_ += s.h;
            if (t_phase_ >= desc_.range_change_time_s) {
                phase_ = Phase::Inertia;
                t_phase_ = real(0.0);
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
    if (!hold_) update_preselect(s, th);
    if (desc_.cvt) update_cvt(s, th);
    if (desc_.hydro) update_hydro(s, th);

    // --- row commands ---
    Outputs o;
    RowCmd& act = o.row[slot_];
    RowCmd& onc = o.row[1 - slot_];
    act.ratio = ratio(cur_);
    onc.ratio = ratio(phase_ == Phase::Idle || phase_ == Phase::Engage || phase_ == Phase::Closing ? (pre_ != 0 ? pre_ : cur_) : next_);
    const real e = slip_launch() ? e_launch_ : real(1.0);
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
        case Phase::Range: // both clutches open: the interruption
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
    // Both shift rows can transmit wheel braking torque. Opening only the
    // current row leaves the oncoming clutch able to stall the engine.
    if (slip_launch() && s.engine_omega * kRadPerSecToRpm < desc_.tcu.launch_anti_stall_rpm) {
        act.cap_kinetic = onc.cap_kinetic = real(0.0);
    }
    // TC owns a TOTAL input-referred capacity: sharing it keeps a handover
    // from applying the limit once per clutch (and doubling the allowance).
    const real sum_capacity = act.cap_kinetic + onc.cap_kinetic;
    if (sum_capacity > s.clutch_capacity_cap_nm && sum_capacity > real(0.0)) {
        const real scale = std::max(real(0.0), s.clutch_capacity_cap_nm) / sum_capacity;
        act.cap_kinetic *= scale;
        onc.cap_kinetic *= scale;
    }
    act.cap_static = cd.static_factor * act.cap_kinetic;
    onc.cap_static = cd.static_factor * onc.cap_kinetic;
    if (desc_.cvt) {
        // CVT plan C1: one row; the belt limits it (row B carries the same ratio at no capacity).
        const real belt = belt_capacity();
        if (cur_ != 0) {
            act.cap_static = std::min(act.cap_static, desc_.cvt_desc.belt.static_factor * belt);
            act.cap_kinetic = std::min(act.cap_kinetic, belt);
        }
        onc.ratio = act.ratio;
        onc.cap_static = onc.cap_kinetic = real(0.0);
        o.cvt_ratio = ratio_;
        o.cvt_step = step_;
        o.belt_capacity_nm = belt;
        o.clamp_force_n = clamp_;
    }
    if (desc_.hydro) {
        // The one row (output = node a) carries k; the other holds the same ratio at no capacity.
        onc.ratio = act.ratio;
        onc.cap_static = onc.cap_kinetic = real(0.0);
        o.cvt_ratio = ratio_;
        o.engine_command = engine_cmd_;
        o.engine_target_rpm = n_target_;
    }
    if (desc_.has_lockup) {
        o.lockup_cap_kinetic = e_lock_ * desc_.lockup_capacity_nm;
        o.lockup_cap_static = desc_.lockup_static_factor * o.lockup_cap_kinetic;
    }
    o.throttle_ceiling = (phase_ == Phase::Inertia && upshift_ && power_on_) ? desc_.tcu.upshift_torque_fraction : real(1.0);
    o.current = cur_;
    o.target = tgt_;
    o.phase = phase_;
    o.launch_engagement = slip_launch() ? (phase_ == Phase::Idle || phase_ == Phase::Engage ? e_launch_ * (phase_ == Phase::Engage ? e_engage_ : real(1.0)) : real(1.0))
                                            : real(1.0);
    if (slip_launch()) o.launch_engagement = std::min(o.launch_engagement, s.clutch_capacity_cap_nm / C);
    if (cur_ == 0 || (slip_launch() && s.engine_omega * kRadPerSecToRpm < desc_.tcu.launch_anti_stall_rpm))
        o.launch_engagement = real(0.0);
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
    if (rev_held_) h = fnv1a(h, rev_held_);
    if (desc_.cvt) { // CVT plan C1: only a cvt folds its variator (every other box's hash unchanged)
        h = fnv1a(h, ratio_);
        h = fnv1a(h, step_);
        h = fnv1a(h, t_step_);
        h = fnv1a(h, clamp_);
    }
    if (desc_.hydro) { // tractor plan stage 2
        h = fnv1a(h, ratio_);
        h = fnv1a(h, v_ref_);
        h = fnv1a(h, n_target_);
        h = fnv1a(h, lever_i_);
        h = fnv1a(h, a_ref_);
    }
    return h;
}

} // namespace ps::drivetrain
