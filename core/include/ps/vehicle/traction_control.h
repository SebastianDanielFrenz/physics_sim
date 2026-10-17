#pragma once
#include "ps/types.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace ps::vehicle {
struct TractionControlDesc {
    bool enabled=false; // runtime channel default; legacy native vehicles unchanged
    bool fitted=true;   // engine garage P8 (A15): false = the car has no TC, the channel is ignored. Not hashed (default true = unchanged).
    real target_slip=.08, reference_floor_m_s=2.0;
    real kp=2.0, ki_per_s=6.0, attack_rate_per_s=20.0, recovery_rate_per_s=8.0;
    // Assists I1 (PHYS-074): per-mode extras, set only by an assist unit's mode table. The defaults are today's
    // controller (torque fraction may reach 0, ignition cut allowed), so a car without a unit is bit-identical.
    // Not folded into the state hash (a unit folds its resolved mode table itself).
    real min_fraction=0.0;
    bool ignition_cut=true;
};
struct TractionWheelSensor {
    real peripheral_speed_m_s=0, steer_rad=0, x_m=0, y_m=0;
    bool driven=false;
    bool braked=false; // this corner's brake (service or handbrake) is applied - the per-wheel brake signal an ESC reads
};

// Actual physics state at each contact patch, refreshed every substep. No speed estimator.
struct TractionWheelState {
    real peripheral_speed_m_s = 0;
    real ground_speed_m_s = 0; // chassis velocity at this patch, projected along its rolling direction
    real normal_load_n = 0;
    bool driven = false;
    bool contact = false;
};
struct TractionControlState {
    // Legacy snapshot field name retained: this is now the actual chassis longitudinal velocity.
    real estimated_speed_m_s = 0;
    real torque_fraction = 1, positive_slip = 0;
    bool initialized = false, enabled = false, intervening = false;
};
inline void validate_traction_control(const TractionControlDesc& d) {
    for(real v:{d.target_slip,d.reference_floor_m_s,d.kp,d.ki_per_s,d.attack_rate_per_s,d.recovery_rate_per_s})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("traction_control: parameters must be finite and positive");
    if(!std::isfinite(d.min_fraction)||d.min_fraction<0||d.min_fraction>=1)throw std::invalid_argument("traction_control: min_fraction must be in [0, 1)");
    if(d.target_slip>=1)throw std::invalid_argument("traction_control: target_slip must be below 1");
}

// Ignore airborne and unloaded corners. Reverse wheelspin is positive in the direction of travel;
// a stopped/reversing wheel while the car still rolls forward is braking, not acceleration slip.
inline real traction_driven_slip(std::span<const TractionWheelState> wheels, real floor_m_s) {
    real slip = 0;
    for (const auto& w : wheels) {
        if (!w.driven || !w.contact || !(w.normal_load_n > 1) ||
            !std::isfinite(w.normal_load_n) || !std::isfinite(w.ground_speed_m_s) ||
            !std::isfinite(w.peripheral_speed_m_s)) continue;
        const real direction = std::abs(w.ground_speed_m_s) > .1 ?
            (w.ground_speed_m_s < 0 ? -1 : 1) : (w.peripheral_speed_m_s < 0 ? -1 : 1);
        slip = std::max(slip, direction * (w.peripheral_speed_m_s - w.ground_speed_m_s) /
                       std::max(floor_m_s, std::abs(w.ground_speed_m_s)));
    }
    return slip;
}
inline real step_traction_control(const TractionControlDesc& d, TractionControlState& s,
        std::span<const TractionWheelState> wheels, real longitudinal_speed_m_s,
        real pedal, real brake, bool enabled, real dt) {
    if (!enabled || !std::isfinite(longitudinal_speed_m_s) || !std::isfinite(pedal) ||
        !std::isfinite(brake) || !std::isfinite(dt) || dt <= 0) {
        s = {};
        return 1;
    }
    s.enabled = s.initialized = true;
    s.estimated_speed_m_s = longitudinal_speed_m_s;
    s.positive_slip = traction_driven_slip(wheels, d.reference_floor_m_s);
    s.intervening = false;
    // No contact means no acceleration-slip intervention: free revs while airborne.
    // Coasting and braking also clear the previous cut immediately.
    bool loaded = false;
    for (const auto& w : wheels)
        loaded |= w.driven && w.contact && w.normal_load_n > 1 && std::isfinite(w.normal_load_n) &&
                  std::isfinite(w.peripheral_speed_m_s) && std::isfinite(w.ground_speed_m_s);
    if (!loaded || pedal <= .02 || brake > .05) {
        s.torque_fraction = 1;
        return 1;
    }
    // A bounded torque regulator. Excess spin lowers the ceiling; grip returning restores it.
    // There is no accumulated estimator error or PI wind-up to hold a false cut after a slide.
    const real error = s.positive_slip - d.target_slip;
    s.torque_fraction = std::clamp(s.torque_fraction, d.min_fraction, real(1));
    if (error > 0) {
        const real desired = std::max(d.min_fraction, std::min(
            s.torque_fraction - d.ki_per_s * error * dt, real(1) - d.kp * error));
        s.torque_fraction = std::max(desired, s.torque_fraction - d.attack_rate_per_s * dt);
    } else {
        const real recovery = std::clamp(-error / d.target_slip, real(.1), real(1));
        s.torque_fraction = std::min(real(1), s.torque_fraction + d.recovery_rate_per_s * recovery * dt);
    }
    s.intervening = s.torque_fraction < .995;
    return s.torque_fraction;
}
inline std::uint64_t fold_traction_control_hash(const TractionControlDesc& d, const TractionControlState& s, std::uint64_t h) {
    if (!d.enabled && !s.initialized && !s.enabled) return h;
    const auto fold = [&](const auto& v) {
        const auto* b = reinterpret_cast<const unsigned char*>(&v);
        for (std::size_t i = 0; i < sizeof(v); ++i) { h ^= b[i]; h *= 1099511628211ULL; }
    };
    const std::uint64_t tag = 0x5452414354494f32ULL; fold(tag); fold(d.enabled);
    fold(d.target_slip); fold(d.reference_floor_m_s); fold(d.kp); fold(d.ki_per_s);
    fold(d.attack_rate_per_s); fold(d.recovery_rate_per_s);
    fold(s.estimated_speed_m_s); fold(s.torque_fraction); fold(s.positive_slip);
    fold(s.initialized); fold(s.enabled);
    return h;
}
}
