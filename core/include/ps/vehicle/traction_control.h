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
    real target_slip=.08, reference_floor_m_s=2.0;
    real kp=2.0, ki_per_s=6.0, attack_rate_per_s=20.0, recovery_rate_per_s=8.0;
};
struct TractionWheelSensor {
    real peripheral_speed_m_s=0, steer_rad=0, x_m=0, y_m=0;
    bool driven=false;
};
struct TractionControlState {
    real estimated_speed_m_s=0, integral=0, torque_fraction=1, positive_slip=0;
    bool initialized=false, enabled=false, intervening=false;
};
inline void validate_traction_control(const TractionControlDesc& d) {
    for(real v:{d.target_slip,d.reference_floor_m_s,d.kp,d.ki_per_s,d.attack_rate_per_s,d.recovery_rate_per_s})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("traction_control: parameters must be finite and positive");
    if(d.target_slip>=1)throw std::invalid_argument("traction_control: target_slip must be below 1");
}
// SENSOR-ONLY interface: no ground speed, GPS, true tyre slip, contact/load or surface.
// IMU acceleration is gravity-compensated longitudinal acceleration, m/s^2.
// An AWD controller cannot observe constant common-mode wheelspin from wheel
// speeds alone. It propagates its wheel-initialized estimate with the IMU and
// only reanchors on coast; the estimate is explicitly not a GPS speed reading.
inline real step_traction_control(const TractionControlDesc& d,TractionControlState& s,
        std::span<const TractionWheelSensor> wheels,real imu_acceleration_m_s2,
        real imu_yaw_rate_rad_s,real pedal,real brake,bool enabled,real dt) {
    s.enabled=enabled;s.intervening=false;s.positive_slip=0;
    if(!enabled||wheels.empty()||!std::isfinite(imu_acceleration_m_s2)||!std::isfinite(imu_yaw_rate_rad_s)||
       !std::isfinite(pedal)||!std::isfinite(brake)||!std::isfinite(dt)||dt<=0) {
        s.integral=0;s.torque_fraction=1;s.initialized=false;return 1;
    }
    real reference=0;bool have_reference=false;
    for(const auto& w:wheels) {
        if(!std::isfinite(w.peripheral_speed_m_s)||!std::isfinite(w.steer_rad))continue;
        const real c=std::cos(w.steer_rad);
        if(c<.5)continue;
        const real v=(w.peripheral_speed_m_s-imu_yaw_rate_rad_s*(w.x_m*std::sin(w.steer_rad)-w.y_m*c))/c;
        if(!have_reference||std::abs(v)<std::abs(reference)){reference=v;have_reference=true;}
    }
    if(!have_reference){s.integral=0;s.torque_fraction=1;s.initialized=false;return 1;}
    if(!s.initialized){s.estimated_speed_m_s=reference;s.initialized=true;}
    else s.estimated_speed_m_s+=imu_acceleration_m_s2*dt;
    if(pedal<=.02||brake>.05) {
        // Coast reanchors from wheel sensors; braking is never a TC event.
        s.estimated_speed_m_s+=(reference-s.estimated_speed_m_s)*std::min(real(1),dt*real(4));
        s.integral=0;s.torque_fraction=1;return 1;
    }
    const real direction=std::abs(s.estimated_speed_m_s)>.5 ? (s.estimated_speed_m_s<0?-1:1) : (reference<0?-1:1);
    for(const auto& w:wheels) {
        if(!w.driven||!std::isfinite(w.peripheral_speed_m_s)||!std::isfinite(w.steer_rad))continue;
        const real c=std::cos(w.steer_rad);if(c<.5)continue;
        const real v=(w.peripheral_speed_m_s-imu_yaw_rate_rad_s*(w.x_m*std::sin(w.steer_rad)-w.y_m*c))/c;
        s.positive_slip=std::max(s.positive_slip,direction*(v-s.estimated_speed_m_s)/std::max(d.reference_floor_m_s,std::abs(s.estimated_speed_m_s)));
    }
    const real error=s.positive_slip-d.target_slip;
    // Unwind quickly once wheelspin has actually stopped; a saturated PI
    // integrator must not hold the launch clutch at idle for seconds.
    const real integral_step=(error<0&&s.positive_slip<d.target_slip*.25)?-d.recovery_rate_per_s:d.ki_per_s*error;
    s.integral=std::clamp(s.integral+integral_step*dt,real(0),real(1));
    const real desired=std::clamp(real(1)-d.kp*std::max(real(0),error)-s.integral,real(0),real(1));
    const real rate=desired<s.torque_fraction?d.attack_rate_per_s:d.recovery_rate_per_s;
    s.torque_fraction+=std::clamp(desired-s.torque_fraction,-rate*dt,rate*dt);
    s.intervening=s.torque_fraction<.995;return s.torque_fraction;
}
inline std::uint64_t fold_traction_control_hash(const TractionControlDesc& d,const TractionControlState& s,std::uint64_t h) {
    if(!d.enabled&&!s.initialized&&!s.enabled)return h;
    const auto fold=[&](const auto& v){const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}};
    const std::uint64_t tag=0x5452414354494f4eULL;fold(tag);fold(d.enabled);
    fold(d.target_slip);fold(d.reference_floor_m_s);fold(d.kp);fold(d.ki_per_s);fold(d.attack_rate_per_s);fold(d.recovery_rate_per_s);
    fold(s.estimated_speed_m_s);fold(s.integral);fold(s.torque_fraction);fold(s.initialized);fold(s.enabled);
    return h;
}
}
