// core/src/drivetrain/launch_control.h - launch control (two-step), engine garage P8 (A16).
// INTERNAL (ps::drivetrain). Header-only, allocation-free, no state beyond one latch.
//
// What it is: a SECOND rev limit that the engine's own limiter applies while the control is active
// (TcuRequest::limiter_rpm, read by EngineTorqueMap and SimulatedEngine: FuelCut engines cut fuel at it with
// their own hysteresis, SoftCut engines taper over their own band below it). The controller itself only decides
// WHEN it is active; it never touches torque.
//
// Active latch (docs/data_formats/electronics.md):
//   inactive -> active : the arm channel is on AND the car is nearly stationary (speed < max_speed_mps -
//                        kSpeedHysteresisMps) AND the clutch is in (effective engagement < kClutchInEngagement);
//   active -> inactive : the arm channel goes off OR speed > max_speed_mps.
// The latch is what lets the car LAUNCH with the limit still held: once active, releasing the clutch does not
// end it - only leaving the speed window or switching the channel off does. Re-arming needs the clutch in again.
//
// "Clutch in" means the clutch's EFFECTIVE engagement (min of the pedal curve and the auto-clutch, the value
// the clutch itself uses this substep) is below kClutchInEngagement. With a manual clutch that is the pedal
// pressed; with the auto-clutch assist it is the launch law's open clutch at idle (engagement 0 below bite rpm),
// so arming works before the throttle is opened. A vehicle without a clutch can never arm [AS].

#pragma once

#include "ps/drivetrain/controller_desc.h"
#include "ps/types.h"

#include <cstdint>

namespace ps::drivetrain {

class LaunchControl {
public:
    // [AS] hysteresis on the entry speed so that a car rolling at the window edge does not chatter.
    static constexpr real kSpeedHysteresisMps = 0.25;
    // [AS] effective clutch engagement below which the clutch counts as "in".
    static constexpr real kClutchInEngagement = 0.5;

    struct Inputs {
        bool switch_on = false;          // arm channel >= 0.5 and the control is fitted
        real speed_mps = 0.0;            // speedo speed (gearbox-output derived), signed or not
        bool has_clutch = false;
        real clutch_engagement = 1.0;    // effective engagement this substep
    };

    explicit LaunchControl(const LaunchControlDesc& d) : desc_(d) {}

    // One substep. Returns the second limiter speed (rpm) while active, else 0 (= no second limit).
    real step(const Inputs& in) {
        const real speed = in.speed_mps < real(0.0) ? -in.speed_mps : in.speed_mps;
        if (!active_) {
            if (in.switch_on && in.has_clutch && in.clutch_engagement < kClutchInEngagement &&
                speed < desc_.max_speed_mps - kSpeedHysteresisMps) {
                active_ = true;
            }
        } else if (!in.switch_on || speed > desc_.max_speed_mps) {
            active_ = false;
        }
        return active_ ? desc_.rpm : real(0.0);
    }

    [[nodiscard]] bool active() const { return active_; }
    [[nodiscard]] const LaunchControlDesc& desc() const { return desc_; }
    void reset() { active_ = false; }

    // FNV-1a over the one latch byte; the caller folds it only when the component exists.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const {
        const std::uint8_t b = active_ ? std::uint8_t(1) : std::uint8_t(0);
        h ^= b;
        h *= 1099511628211ULL;
        return h;
    }

private:
    LaunchControlDesc desc_;
    bool active_ = false;
};

} // namespace ps::drivetrain
