#pragma once
#include "ps/math/transcendental.h"
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
// Which wheel evidence the speed estimate was corrected with in the last call (step_traction_control).
enum class TractionReference : std::uint8_t {
    None=0,       // not initialised
    Direct=1,     // a free wheel (not driven, or coasting, and not braked) or the standstill update: a measurement
    LowerBound=2, // braking without a free wheel: no wheel is faster than the car, the fastest one is a lower bound
    UpperBound=3, // driving without a free wheel (AWD): no wheel is slower than the car, the slowest one is an upper bound
};
struct TractionControlState {
    real estimated_speed_m_s=0, integral=0, torque_fraction=1, positive_slip=0;
    // Speed-estimator state (hashed): the wheel reference of the last call, its low-passed rate of change and how
    // long that rate has agreed with the IMU.
    real wheel_reference_m_s=0, wheel_accel_m_s2=0, consistent_s=0;
    TractionReference reference=TractionReference::None;
    bool initialized=false, enabled=false, intervening=false;
};
// Speed-estimator constants (step_traction_control). [AS] all; not data keys - they describe the estimator, not a car.
inline constexpr real kTractionDirectGainPerS=20.0;     // correction gain toward a free wheel / a violated bound, 1/s
inline constexpr real kTractionDirectMaxRate=20.0;      // ... rate cap, m/s^2 (a bouncing wheel cannot yank the estimate)
inline constexpr real kTractionConsistentGainPerS=5.0;  // AWD: gain toward the slowest wheel once the evidence below holds, 1/s
inline constexpr real kTractionConsistentMaxRate=3.0;   // ... rate cap, m/s^2
inline constexpr real kTractionConsistentTolerance=1.0; // |wheel reference rate - IMU| that counts as agreeing, m/s^2
inline constexpr real kTractionConsistentHoldS=0.1;     // ... for this long
inline constexpr real kTractionConsistentMaxFraction=0.3; // ... and only while the drive torque is cut to at most this fraction
inline constexpr real kTractionWheelAccelTauS=0.03;     // low-pass of the wheel reference rate, s
inline constexpr real kTractionStandstillSpeed=0.05;    // standstill update: every wheel below this, m/s
inline constexpr real kTractionStandstillAccel=0.3;     // ... and |IMU| below this, m/s^2
inline void validate_traction_control(const TractionControlDesc& d) {
    for(real v:{d.target_slip,d.reference_floor_m_s,d.kp,d.ki_per_s,d.attack_rate_per_s,d.recovery_rate_per_s})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("traction_control: parameters must be finite and positive");
    if(!std::isfinite(d.min_fraction)||d.min_fraction<0||d.min_fraction>=1)throw std::invalid_argument("traction_control: min_fraction must be in [0, 1)");
    if(d.target_slip>=1)throw std::invalid_argument("traction_control: target_slip must be below 1");
}
// A wheel's speed sensor as the ground speed it implies at the vehicle origin: steer- and yaw-compensated
// (v = vx + tan(steer)*vy for a free-rolling wheel). False for a non-finite reading or more than 60 deg of steer.
inline bool traction_wheel_ground_speed(const TractionWheelSensor& w,real imu_yaw_rate_rad_s,real& v) {
    if(!std::isfinite(w.peripheral_speed_m_s)||!std::isfinite(w.steer_rad))return false;
    real sn=0,c=1;ps::math::sincos(w.steer_rad,sn,c);
    if(c<.5)return false;
    v=(w.peripheral_speed_m_s-imu_yaw_rate_rad_s*(w.x_m*sn-w.y_m*c))/c;
    return true;
}
// SENSOR-ONLY interface: no ground speed, GPS, true tyre slip, contact/load or surface. The inputs are the wheel speed
// sensors, the per-corner brake signal, the IMU's longitudinal acceleration (gravity-compensated, m/s^2) and yaw rate.
//
// Speed estimate (the vehicle reference speed of an ESC): predicted with the IMU every call, then corrected with
// the wheels, so an offset picked up anywhere (a slide, a kerb, the IMU missing the rotating-frame term, switching
// TC off and on) decays instead of staying:
//  - Direct: a free wheel (not braked; not driven, or every wheel while coasting) rolls at ground speed. The
//    estimate is pulled to the fastest free wheel at kTractionDirectGainPerS, rate-capped. A standstill (every wheel
//    below kTractionStandstillSpeed, IMU quiet) is a direct measurement of 0.
//  - LowerBound (braking, no free wheel): braked wheels can only be slower than the car. The estimate is pulled up
//    to the fastest wheel when below it; above it only the IMU acts. Locked wheels can no longer drag it to zero.
//  - UpperBound (driving, no free wheel: AWD): driven wheels can only be faster. Above the slowest wheel the
//    estimate is pulled down; below it, toward it only while the drive torque is cut (fraction at most
//    kTractionConsistentMaxFraction) and the slowest wheel's rate has matched the IMU for kTractionConsistentHoldS:
//    a cut wheel that keeps pace with the body is rolling, not spinning. Constant common-mode wheelspin with torque
//    on is not observable from these sensors and is not corrected.
inline real step_traction_control(const TractionControlDesc& d,TractionControlState& s,
        std::span<const TractionWheelSensor> wheels,real imu_acceleration_m_s2,
        real imu_yaw_rate_rad_s,real pedal,real brake,bool enabled,real dt) {
    s.enabled=enabled;s.intervening=false;s.positive_slip=0;
    const auto reset=[&]{s.integral=0;s.torque_fraction=1;s.initialized=false;s.reference=TractionReference::None;
                         s.wheel_reference_m_s=0;s.wheel_accel_m_s2=0;s.consistent_s=0;};
    if(!enabled||wheels.empty()||!std::isfinite(imu_acceleration_m_s2)||!std::isfinite(imu_yaw_rate_rad_s)||
       !std::isfinite(pedal)||!std::isfinite(brake)||!std::isfinite(dt)||dt<=0){reset();return 1;}
    const bool coast=pedal<=.02;
    real sum=0,max_abs=0;int valid=0;
    for(const auto& w:wheels){real v;if(!traction_wheel_ground_speed(w,imu_yaw_rate_rad_s,v))continue;sum+=v;max_abs=std::max(max_abs,std::abs(v));++valid;}
    if(valid==0){reset();return 1;}
    const real direction=s.initialized&&std::abs(s.estimated_speed_m_s)>.5 ? (s.estimated_speed_m_s<0?-1:1) : (sum<0?-1:1);
    // Fastest free wheel, fastest wheel, slowest wheel - all in the direction of travel.
    real free_best=0,fastest=0,slowest=0;bool have_free=false,have_any=false,any_braked=false;
    for(const auto& w:wheels) {
        real v;if(!traction_wheel_ground_speed(w,imu_yaw_rate_rad_s,v))continue;
        const real dv=direction*v;
        if(!have_any){fastest=slowest=dv;have_any=true;}else{fastest=std::max(fastest,dv);slowest=std::min(slowest,dv);}
        if(w.braked){any_braked=true;continue;}
        if(w.driven&&!coast)continue;
        if(!have_free||dv>free_best){free_best=dv;have_free=true;}
    }
    TractionReference kind;real measured;
    if(max_abs<kTractionStandstillSpeed&&std::abs(imu_acceleration_m_s2)<kTractionStandstillAccel){kind=TractionReference::Direct;measured=0;}
    else if(have_free){kind=TractionReference::Direct;measured=direction*free_best;}
    else if(any_braked){kind=TractionReference::LowerBound;measured=direction*fastest;}
    else {kind=TractionReference::UpperBound;measured=direction*slowest;}
    if(!s.initialized) {
        s.estimated_speed_m_s=measured;s.initialized=true;
        s.wheel_accel_m_s2=imu_acceleration_m_s2;s.consistent_s=0;
    } else {
        s.estimated_speed_m_s+=imu_acceleration_m_s2*dt;
        if(kind!=s.reference){s.wheel_accel_m_s2=imu_acceleration_m_s2;s.consistent_s=0;}
        else {
            const real raw=(measured-s.wheel_reference_m_s)/dt;
            s.wheel_accel_m_s2+=(raw-s.wheel_accel_m_s2)*std::min(real(1),dt/kTractionWheelAccelTauS);
            s.consistent_s=std::abs(s.wheel_accel_m_s2-imu_acceleration_m_s2)<=kTractionConsistentTolerance?s.consistent_s+dt:0;
        }
        const real error=measured-s.estimated_speed_m_s;
        const bool hard=kind==TractionReference::Direct||(kind==TractionReference::LowerBound&&direction*error>0)||
                        (kind==TractionReference::UpperBound&&direction*error<0);
        if(hard) {
            s.estimated_speed_m_s+=std::clamp(error*std::min(real(1),dt*kTractionDirectGainPerS),-kTractionDirectMaxRate*dt,kTractionDirectMaxRate*dt);
        } else if(kind==TractionReference::UpperBound&&s.consistent_s>=kTractionConsistentHoldS&&s.torque_fraction<=kTractionConsistentMaxFraction) {
            s.estimated_speed_m_s+=std::clamp(error*std::min(real(1),dt*kTractionConsistentGainPerS),-kTractionConsistentMaxRate*dt,kTractionConsistentMaxRate*dt);
        }
    }
    s.reference=kind;s.wheel_reference_m_s=measured;
    if(coast||brake>.05){s.integral=0;s.torque_fraction=1;return 1;} // braking is never a TC event
    const real slip_direction=std::abs(s.estimated_speed_m_s)>.5 ? (s.estimated_speed_m_s<0?-1:1) : direction;
    for(const auto& w:wheels) {
        real v;if(!w.driven||!traction_wheel_ground_speed(w,imu_yaw_rate_rad_s,v))continue;
        s.positive_slip=std::max(s.positive_slip,slip_direction*(v-s.estimated_speed_m_s)/std::max(d.reference_floor_m_s,std::abs(s.estimated_speed_m_s)));
    }
    const real error=s.positive_slip-d.target_slip;
    // Unwind quickly once wheelspin has actually stopped; a saturated PI
    // integrator must not hold the launch clutch at idle for seconds.
    const real integral_step=(error<0&&s.positive_slip<d.target_slip*.25)?-d.recovery_rate_per_s:d.ki_per_s*error;
    s.integral=std::clamp(s.integral+integral_step*dt,real(0),real(1));
    const real desired=std::clamp(real(1)-d.kp*std::max(real(0),error)-s.integral,d.min_fraction,real(1));
    const real rate=desired<s.torque_fraction?d.attack_rate_per_s:d.recovery_rate_per_s;
    s.torque_fraction+=std::clamp(desired-s.torque_fraction,-rate*dt,rate*dt);
    s.intervening=s.torque_fraction<.995;return s.torque_fraction;
}
// Assists I4 (launch control): the driven wheels' worst positive slip against a given speed estimate, in TC's own
// formula (steer- and yaw-compensated peripheral speeds, direction from the estimate) but with the caller's speed
// floor instead of d.reference_floor_m_s. Sensor-only like step_traction_control; 0 without a finite input. Does not
// touch the controller state.
inline real traction_driven_slip(std::span<const TractionWheelSensor> wheels,real estimate_m_s,real imu_yaw_rate_rad_s,
        real reference_floor_m_s,real fallback_direction=1) {
    if(!std::isfinite(estimate_m_s)||!std::isfinite(imu_yaw_rate_rad_s)||!(reference_floor_m_s>0))return 0;
    const real direction=std::abs(estimate_m_s)>.5 ? (estimate_m_s<0?-1:1) : fallback_direction;
    real slip=0;
    for(const auto& w:wheels) {
        real v;if(!w.driven||!traction_wheel_ground_speed(w,imu_yaw_rate_rad_s,v))continue;
        slip=std::max(slip,direction*(v-estimate_m_s)/std::max(reference_floor_m_s,std::abs(estimate_m_s)));
    }
    return slip;
}
inline std::uint64_t fold_traction_control_hash(const TractionControlDesc& d,const TractionControlState& s,std::uint64_t h) {
    if(!d.enabled&&!s.initialized&&!s.enabled)return h;
    const auto fold=[&](const auto& v){const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}};
    const std::uint64_t tag=0x5452414354494f4eULL;fold(tag);fold(d.enabled);
    fold(d.target_slip);fold(d.reference_floor_m_s);fold(d.kp);fold(d.ki_per_s);fold(d.attack_rate_per_s);fold(d.recovery_rate_per_s);
    fold(s.estimated_speed_m_s);fold(s.integral);fold(s.torque_fraction);fold(s.initialized);fold(s.enabled);
    fold(s.wheel_reference_m_s);fold(s.wheel_accel_m_s2);fold(s.consistent_s);fold(static_cast<std::uint8_t>(s.reference));
    return h;
}
}
