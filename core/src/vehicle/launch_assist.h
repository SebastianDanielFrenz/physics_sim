// core/src/vehicle/launch_assist.h - launch control as a grade-gated assist of the assist unit (assists I4, D-LC1,
// vault PHYS-074, docs/data_formats/electronics.md). INTERNAL (ps::vehicle). Header-only, allocation-free, no libm.
//
// The controller only DECIDES; VehicleInstance wires its outputs into the powertrain (ComponentSubstepCtx::launch_*)
// and the aero hook. One substep per call, inputs from the previous substep's sensors like every other exchange.
//
//   Idle     : nothing. -> Armed on enable AND auto-clutch AND engine running AND throttle > arm_throttle AND brake >
//              arm_brake AND speedo < arm_max_speed_mps AND gear 1 (not shifting).
//   Armed    : second rev limit = launch_rpm through the engine's own limiter (TcuRequest::limiter_rpm), the auto-clutch
//              is held open, traction control is suspended (its estimator keeps running), aero asks for the launch
//              wing. -> Launch when brake < release_brake; -> Idle when disabled, throttle < 0.5, speed >=
//              arm_max_speed_mps, gear != 1 or a shift starts.
//   Launch   : the clutch is the torque limiter. Its capacity fraction c (0..1, multiplying the torque-following
//              capacity of the auto-clutch, see Powertrain) grows from 0 at most as fast as 1/clutch_ramp_s and at
//              clutch_up_per_s * (1 - slip/target), and is pulled back at clutch_down_per_s * (slip/target - 1) when
//              the driven wheels slip above the mode's target: a slip regulator on the fast actuator. The second rev
//              limit STAYS at launch_rpm (the engine is the held source, boost builds under load); the engine's
//              throttle ceiling comes from the classic slip PI (TC's one-sided law) as the second, slower line.
//              -> Handover on speed > handover_speed_mps or t > max_launch_s ("soft") or on throttle < 0.5, a shift,
//              gear != 1, channel/auto-clutch/engine stopping ("hard").
//   Handover : traction control runs again and starts from the PI's last ceiling (the caller seeds its fraction and
//              integral, no jump), the wing returns to the aero controller. After a SOFT hand-over the launch limit and
//              the clutch fraction stay in force until c has climbed to 1 (so the clutch reaches the plain
//              torque-following close without a step and the engine is not released against a half-open clutch); c
//              climbs at the ramp rate while the slip is at or below the target and is pulled back above it, open-loop
//              again after max_launch_s in the overlay; after a HARD one nothing is kept. -> Idle.
//
// Precedence with the low-speed fix (assists I2, D-LS1..3): while LaunchAssist owns the car (Armed, Launch) the
// powertrain sees TC neutral (fraction 1, no ceiling, no ignition cut), so TC's crawl-speed clutch cap never fires;
// the launch clutch cap and throttle ceiling are the only intervention. In Idle/Handover I2 is untouched (the launch
// clutch fraction of a soft hand-over only RAISES toward the torque-following cap; min() with TC's cap still applies).
#pragma once

#include "ps/types.h"
#include "ps/vehicle/assist_unit_desc.h"

#include <algorithm>
#include <cstdint>

namespace ps::vehicle {

enum class LaunchPhase : std::uint8_t { Idle = 0, Armed = 1, Launch = 2, Handover = 3 };

class LaunchAssist {
public:
    struct Inputs {
        bool enabled = false;       // channel on and fitted
        bool auto_clutch = false;   // the auto-clutch assist is on: without it nothing can hold the clutch open
        bool engine_running = false;
        real throttle = 0, brake = 0;
        real speed_mps = 0;         // speedo speed (gearbox-output derived)
        int gear = 0;               // current gearbox position (0 = neutral or mid-shift)
        bool shifting = false;      // a gear change is requested or in progress
        real slip = 0;              // driven wheels' worst positive slip (traction_driven_slip)
        bool slip_valid = false;    // the TC estimator is initialised
        real slip_target = 0.1;     // the effective traction mode's target
        real dt = 0;
        bool first_substep_of_tick = false;
    };

    struct Outputs {
        LaunchPhase phase = LaunchPhase::Idle;
        real limiter_rpm = 0;       // > 0: the launch limit is in force (Armed, Launch, soft Handover)
        bool hold_clutch = false;   // Armed
        real clutch_ramp = -1;      // < 0 none; else the 0..1 capacity fraction (Launch, soft Handover)
        real ceiling = 1;           // Launch: the slip PI's throttle fraction, else 1
        bool owns = false;          // Armed or Launch: traction control is suspended
        bool overlay = false;       // limiter/clutch outputs are valid although TC is back (soft Handover)
        bool handover = false;      // true on the one call that leaves Launch
        real handover_ceiling = 1;  // valid with handover: the fraction traction control starts from
    };

    LaunchAssist(const LaunchAssistDesc& d, real launch_rpm) : desc_(d), launch_rpm_(launch_rpm) {}

    Outputs step(const Inputs& in) {
        Outputs out;
        const real speed = in.speed_mps < real(0) ? -in.speed_mps : in.speed_mps;
        const bool common_ok = in.enabled && in.auto_clutch && in.engine_running;
        switch (phase_) {
        case LaunchPhase::Idle:
            if (common_ok && in.throttle > desc_.arm_throttle && in.brake > desc_.arm_brake && speed < desc_.arm_max_speed_mps && in.gear == 1 &&
                !in.shifting) {
                phase_ = LaunchPhase::Armed;
            }
            break;
        case LaunchPhase::Armed:
            if (!common_ok || in.throttle < real(0.5) || speed >= desc_.arm_max_speed_mps || in.gear != 1 || in.shifting) {
                phase_ = LaunchPhase::Idle;
            } else if (in.brake < desc_.release_brake) {
                phase_ = LaunchPhase::Launch;
                t_ = real(0);
                cfrac_ = real(0);
                ceiling_ = real(1);
                integral_ = real(0);
            }
            break;
        case LaunchPhase::Launch: {
            t_ += in.dt;
            const bool hard = !common_ok || in.throttle < real(0.5) || in.shifting || in.gear != 1;
            const bool soft = speed > desc_.handover_speed_mps || t_ > desc_.max_launch_s;
            if (hard || soft) {
                phase_ = LaunchPhase::Handover;
                keep_ = soft && !hard && cfrac_ < real(1);
                t_overlay_ = real(0);
                out.handover = true;
                out.handover_ceiling = ceiling_;
                break;
            }
            const real ramp_rate = desc_.clutch_ramp_s > real(0) ? real(1) / desc_.clutch_ramp_s : real(1e9);
            if (in.slip_valid) {
                // clutch capacity fraction: an integral regulator on the normalised slip error, ramp-rate limited upward
                const real rel = in.slip / in.slip_target - real(1);
                const real rate = rel < real(0) ? std::min(ramp_rate, desc_.clutch_up_per_s * std::min(real(1), -rel)) : -desc_.clutch_down_per_s * rel;
                cfrac_ = std::clamp(cfrac_ + rate * in.dt, real(0), real(1));
                // throttle ceiling: TC's one-sided PI law on the same slip
                const real err = in.slip - in.slip_target;
                const real step = (err < real(0) && in.slip < in.slip_target * real(0.25)) ? -desc_.recovery_rate_per_s : desc_.ki_per_s * err;
                integral_ = std::clamp(integral_ + step * in.dt, real(0), real(1));
                const real desired = std::clamp(real(1) - desc_.kp * std::max(real(0), err) - integral_, desc_.min_fraction, real(1));
                const real rate_c = desired < ceiling_ ? desc_.attack_rate_per_s : desc_.recovery_rate_per_s;
                ceiling_ += std::clamp(desired - ceiling_, -rate_c * in.dt, rate_c * in.dt);
            } else {
                cfrac_ = std::min(real(1), cfrac_ + ramp_rate * in.dt);
            }
            break;
        }
        case LaunchPhase::Handover:
            if (keep_) {
                // The clutch keeps regulating the slip until it is closed: it climbs at the ramp rate only while the
                // driven wheels are at or below the target and is pulled back above it (an open-loop ramp dumped the
                // held launch-rpm engine, at full boost, into the wheels: slip 2.0 on car_hyper in the Sport TC mode).
                // After max_launch_s in the overlay the ramp is open-loop again, so the hand-over always ends.
                const real ramp_rate = desc_.clutch_ramp_s > real(0) ? real(1) / desc_.clutch_ramp_s : real(1e9);
                t_overlay_ += in.dt;
                real rate = ramp_rate;
                if (in.slip_valid && t_overlay_ <= desc_.max_launch_s) {
                    const real rel = in.slip / in.slip_target - real(1);
                    if (rel > real(0)) rate = -desc_.clutch_down_per_s * rel;
                }
                cfrac_ = std::clamp(cfrac_ + rate * in.dt, real(0), real(1));
                if (cfrac_ >= real(1) || !common_ok) keep_ = false;
            }
            if (!keep_ && in.first_substep_of_tick) phase_ = LaunchPhase::Idle;
            break;
        }
        out.phase = phase_;
        out.owns = phase_ == LaunchPhase::Armed || phase_ == LaunchPhase::Launch;
        out.overlay = phase_ == LaunchPhase::Handover && keep_;
        const bool limit = out.owns || out.overlay;
        out.limiter_rpm = limit ? launch_rpm_ : real(0);
        out.hold_clutch = phase_ == LaunchPhase::Armed;
        out.clutch_ramp = (phase_ == LaunchPhase::Launch || out.overlay) ? cfrac_ : real(-1);
        out.ceiling = phase_ == LaunchPhase::Launch ? ceiling_ : real(1);
        return out;
    }

    [[nodiscard]] LaunchPhase phase() const { return phase_; }
    [[nodiscard]] bool owns() const { return phase_ == LaunchPhase::Armed || phase_ == LaunchPhase::Launch; }
    [[nodiscard]] real launch_rpm() const { return launch_rpm_; }
    [[nodiscard]] real ceiling() const { return ceiling_; }
    [[nodiscard]] real clutch_fraction() const { return cfrac_; }
    [[nodiscard]] real launch_time_s() const { return t_; }
    [[nodiscard]] const LaunchAssistDesc& desc() const { return desc_; }

    void reset() {
        phase_ = LaunchPhase::Idle;
        t_ = real(0);
        cfrac_ = real(0);
        ceiling_ = real(1);
        integral_ = real(0);
        keep_ = false;
        t_overlay_ = real(0);
    }

    // FNV-1a over the phase byte and the state reals (fixed order) plus the resolved launch rpm; the caller folds it
    // only on a car that carries the block.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const {
        const auto fold = [&](const auto& v) {
            const auto* b = reinterpret_cast<const unsigned char*>(&v);
            for (std::size_t i = 0; i < sizeof(v); ++i) {
                h ^= b[i];
                h *= 1099511628211ULL;
            }
        };
        const std::uint64_t tag = 0x4c41554e43484c43ULL;
        fold(tag);
        fold(static_cast<std::uint8_t>(phase_));
        fold(static_cast<std::uint8_t>(keep_ ? 1 : 0));
        fold(launch_rpm_);
        fold(t_);
        fold(cfrac_);
        fold(ceiling_);
        fold(integral_);
        fold(t_overlay_);
        return h;
    }

private:
    LaunchAssistDesc desc_;
    real launch_rpm_ = 0;
    LaunchPhase phase_ = LaunchPhase::Idle;
    bool keep_ = false;
    real t_ = 0, cfrac_ = 0, ceiling_ = 1, integral_ = 0, t_overlay_ = 0;
};

} // namespace ps::vehicle
