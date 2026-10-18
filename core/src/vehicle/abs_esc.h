// core/src/vehicle/abs_esc.h - INTERNAL: the per-substep ABS / ESC law (assists I5 part 2 / I6). The descriptors are
// public in ps/vehicle/abs_esc_desc.h; docs/data_formats/electronics.md "abs" / "esc" is the description of the law.
//
// Sensor-only ABS / ESC: the inputs are the wheels' peripheral speeds (TractionWheelSensor, steer and yaw
// compensated for steering and yaw), the gyro yaw rate, the IMU longitudinal acceleration, the brake / handbrake
// channels and the front road-wheel angle. The output is one service-brake command per wheel (ctx.wheel_brake) and an
// engine torque fraction. Every buffer is sized once (AbsEscState::resize) - step_abs_esc never allocates.
#pragma once

#include "ps/math/transcendental.h"
#include "ps/types.h"
#include "ps/vehicle/abs_esc_desc.h"
#include "ps/vehicle/traction_control.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace ps::vehicle {

struct AbsEscState {
    real reference_speed_m_s = 0; // v_ref
    bool initialized = false;
    std::vector<real> scale;      // ABS per-wheel service-brake scale [0, 1]
    std::vector<real> integral;   // ABS per-wheel integral [0, 1]
    std::vector<real> esc_brake;  // ESC per-wheel added brake command [0, max_brake]
    real front_split = 0;         // ABS: allowed scale difference between the front wheels (grows at front_split_rate_per_s)
    std::vector<real> applied;    // ABS per-wheel scale applied this substep (select-low and front split included; output)
    // per substep outputs (not state)
    bool abs_active = false, esc_active = false;
    real yaw_rate_target = 0, esc_torque_fraction = 1;
    void resize(std::size_t n) {
        scale.assign(n, real(1));
        integral.assign(n, real(0));
        esc_brake.assign(n, real(0));
        applied.assign(n, real(1));
    }
    void reset() {
        reference_speed_m_s = 0;
        initialized = false;
        std::fill(scale.begin(), scale.end(), real(1));
        std::fill(integral.begin(), integral.end(), real(0));
        std::fill(esc_brake.begin(), esc_brake.end(), real(0));
        std::fill(applied.begin(), applied.end(), real(1));
        front_split = 0;
        abs_active = esc_active = false;
        yaw_rate_target = 0;
        esc_torque_fraction = 1;
    }
};

struct AbsEscInputs {
    std::span<const TractionWheelSensor> wheels;
    std::span<const std::uint8_t> is_front; // 1 = front axle
    real imu_accel = 0, yaw_rate = 0;
    // The simulation's own chassis velocity along the body x axis at the reference point (NaN = not provided). Used as v_ref
    // when ground_truth is set (AbsReferenceSpeed::GroundTruth); the estimator below then does not run.
    real ground_speed = std::numeric_limits<real>::quiet_NaN();
    bool ground_truth = false;
    real brake = 0, handbrake = 0;
    real front_wheel_angle = 0; // mean front road-wheel angle, rad
    real wheelbase = 2.5;
    real dt = 0;
    const AbsDesc* abs = nullptr;            // non-null = ABS on this substep
    const EscDesc* esc = nullptr;            // non-null = ESC fitted
    EscMode esc_mode = EscMode::Off;         // effective ESC mode
    std::span<real> wheel_brake_out;         // per wheel: service command after ABS / ESC
};

// Ground-equivalent speed of a wheel from its sensor; NaN when the wheel is steered past 60 deg.
[[nodiscard]] inline real abs_wheel_ground_speed(const TractionWheelSensor& w, real yaw) {
    if (!std::isfinite(w.peripheral_speed_m_s) || !std::isfinite(w.steer_rad)) return std::numeric_limits<real>::quiet_NaN();
    real sn = 0, c = 1;
    ps::math::sincos(w.steer_rad, sn, c);
    if (c < real(.5)) return std::numeric_limits<real>::quiet_NaN();
    return (w.peripheral_speed_m_s - yaw * (w.x_m * sn - w.y_m * c)) / c;
}

// The speed the braking reference is lifted to, in the direction `dir` of travel: the fastest wheel that no drive torque can
// spin above the ground. A driven wheel still turns under power at brake onset (a part-throttle launch leaves it several
// percent fast, 12 % at full throttle) and lifting the reference to it left v_ref 0.7-2.7 m/s above the true speed: ABS then
// regulated against a slip the car never had, released the brakes as the speed fell and the stop took 1.5-4.5x as long. With
// at least one undriven wheel only undriven wheels count; a car with every wheel driven has no such wheel and keeps the fastest.
[[nodiscard]] inline real abs_lift_speed(std::span<const TractionWheelSensor> wheels, real yaw, real dir) {
    real fastest = 0, fastest_undriven = 0;
    bool any_undriven = false;
    for (const TractionWheelSensor& w : wheels) {
        const real v = abs_wheel_ground_speed(w, yaw);
        if (!std::isfinite(v)) continue;
        fastest = std::max(fastest, dir * v);
        if (!w.driven) {
            any_undriven = true;
            fastest_undriven = std::max(fastest_undriven, dir * v);
        }
    }
    return any_undriven ? fastest_undriven : fastest;
}

inline void step_abs_esc(AbsEscState& s, const AbsEscInputs& in) {
    const std::size_t n = in.wheels.size();
    s.abs_active = false;
    s.esc_active = false;
    s.esc_torque_fraction = 1;
    s.yaw_rate_target = 0;
    const real dt = in.dt;
    const bool esc_on = in.esc != nullptr && in.esc_mode != EscMode::Off;

    // Reference speed (shared by ABS and ESC), signed: forward > 0. "Fastest" / "slowest" are by magnitude in the
    // direction of travel, so a car sliding backward (after a spin) is braked like one going forward.
    // While only ESC brakes (no pedal, no handbrake) the slowest wheel ESC does not brake rolls freely: its sensor speed
    // is the longitudinal speed at the reference point, so the reference follows it (the non-braking law). The integrated
    // IMU alone runs above the true speed in a yawing slide (no yaw-rate x lateral-velocity term) and made ABS release the
    // ESC-braked wheel.
    bool have = false, have_free = false;
    real vmax = 0, vmin = 0, vmin_free = 0; // signed speeds of the wheel with the largest / smallest |v|
    for (std::size_t i = 0; i < n; ++i) {
        const real v = abs_wheel_ground_speed(in.wheels[i], in.yaw_rate);
        if (!std::isfinite(v)) continue;
        if (!have || std::abs(v) > std::abs(vmax)) vmax = v;
        if (!have || std::abs(v) < std::abs(vmin)) vmin = v;
        have = true;
        if (!(s.esc_brake[i] > real(.02))) {
            if (!have_free || std::abs(v) < std::abs(vmin_free)) vmin_free = v;
            have_free = true;
        }
    }
    bool braking = in.handbrake > real(.02) || in.brake > real(.02);
    if (!braking && have_free) vmin = vmin_free;
    else if (!braking) braking = true; // ESC brakes every wheel
    if (in.ground_truth && std::isfinite(in.ground_speed)) {
        // Ground truth (default): v_ref IS the chassis speed along body x. Nothing to integrate, lift or relax.
        s.reference_speed_m_s = in.ground_speed;
        s.initialized = true;
    } else if (have && std::isfinite(in.imu_accel)) {
        if (!s.initialized) {
            s.reference_speed_m_s = braking ? vmax : vmin;
            s.initialized = true;
        } else {
            s.reference_speed_m_s += in.imu_accel * dt;
            const real dir = s.reference_speed_m_s >= 0 ? real(1) : real(-1);
            if (braking) {
                // lifted to the wheel moving fastest in the reference's direction (a wheel turning the other way lifts nothing)
                const real fastest = abs_lift_speed(in.wheels, in.yaw_rate, dir);
                if (fastest > dir * s.reference_speed_m_s) s.reference_speed_m_s = dir * fastest;
                // Every braked wheel fully released on the last substep and still slower than the reference: the
                // integrated IMU has run away (it lacks the yaw-rate x lateral-velocity term in a slide), so the
                // reference relaxes toward the fastest wheel instead of holding the brakes off for good.
                if (in.abs != nullptr) {
                    bool all_released = true, any = false;
                    for (std::size_t i = 0; i < n; ++i) {
                        if (s.integral[i] <= real(0) && s.scale[i] >= real(1)) continue; // not regulated
                        any = true;
                        if (s.applied[i] > in.abs->reanchor_scale) all_released = false;
                    }
                    if (any && all_released) {
                        // The correction cancels a slow IMU runaway, so its pull-down is bounded: on very low grip every
                        // wheel locks even at a few percent of the brake, the fastest wheel is then a locked wheel, and an
                        // unbounded relaxation dragged the reference down with the wheels (ice: 27.8 -> 20 m/s in 0.25 s while
                        // the car did 27.5) and ABS regulated against a slip error that was no longer there.
                        const real pull = (dir * fastest - s.reference_speed_m_s) * std::min(real(1), in.abs->reanchor_rate_per_s * dt);
                        s.reference_speed_m_s += dir * std::max(dir * pull, -in.abs->reanchor_max_decel_mps2 * dt);
                    }
                }
                // every wheel turns the other way faster than the reference: the car moves that way (e.g. after a spin)
                if (dir * vmax < 0 && std::abs(vmax) > std::abs(s.reference_speed_m_s)) s.reference_speed_m_s = vmax;
            } else {
                const real rate = in.abs ? in.abs->reanchor_rate_per_s : real(10);
                s.reference_speed_m_s += (vmin - s.reference_speed_m_s) * std::min(real(1), rate * dt);
            }
        }
    }
    const real v_ref = s.reference_speed_m_s;

    // ESC: yaw-rate target and the per-corner brake command.
    if (in.esc != nullptr) {
        const EscDesc& e = *in.esc;
        bool want_any = false;
        real over = 0;
        bool oversteer = false;
        real side = 0;
        if (esc_on && s.initialized && v_ref >= e.min_speed_mps) {
            const EscModeParams& p = e.params[static_cast<std::size_t>(in.esc_mode)];
            const real vr = v_ref / e.characteristic_speed_mps;
            real rt = v_ref * in.front_wheel_angle / (in.wheelbase * (real(1) + vr * vr));
            const real cap = e.max_lateral_accel_mps2 / v_ref;
            rt = std::clamp(rt, -cap, cap);
            s.yaw_rate_target = rt;
            const real err = in.yaw_rate - rt;
            const real db = p.deadband_rad_s + p.deadband_fraction * std::abs(rt);
            over = std::abs(err) - db;
            if (over > 0) {
                want_any = true;
                oversteer = (in.yaw_rate * rt < 0) || (std::abs(in.yaw_rate) > std::abs(rt));
                side = err > 0 ? real(-1) : real(1); // brake the wheels with sign(y) = sign(-err)
                s.esc_torque_fraction = std::clamp(real(1) - p.torque_gain_per_rad_s * over, real(0), real(1));
                for (std::size_t i = 0; i < n; ++i) {
                    const bool front = in.is_front[i];
                    const bool on_side = in.wheels[i].y_m * side > 0;
                    const real target = (on_side && front == oversteer) ? std::clamp(p.brake_gain_per_rad_s * over, real(0), p.max_brake) : real(0);
                    const real step = e.brake_rate_per_s * dt;
                    s.esc_brake[i] += std::clamp(target - s.esc_brake[i], -step, step);
                }
            }
        }
        if (!esc_on) {
            std::fill(s.esc_brake.begin(), s.esc_brake.end(), real(0));
        } else if (!want_any) {
            const real step = e.brake_rate_per_s * dt;
            for (std::size_t i = 0; i < n; ++i) s.esc_brake[i] = std::max(real(0), s.esc_brake[i] - step);
        }
        for (std::size_t i = 0; i < n; ++i) if (s.esc_brake[i] > real(1e-6)) s.esc_active = true;
        if (s.esc_torque_fraction < real(1)) s.esc_active = true;
    }

    // ABS: per-wheel scale on the service command.
    for (std::size_t i = 0; i < n; ++i) {
        const real cmd = std::min(real(1), std::max(real(0), in.brake) + s.esc_brake[i]);
        in.wheel_brake_out[i] = cmd;
    }
    if (in.abs != nullptr) {
        const AbsDesc& a = *in.abs;
        const real speed = std::abs(v_ref);
        const real dir = v_ref >= 0 ? real(1) : real(-1);
        bool front_regulated = false;
        for (std::size_t i = 0; i < n; ++i) {
            const real cmd = in.wheel_brake_out[i];
            const real v = abs_wheel_ground_speed(in.wheels[i], in.yaw_rate);
            if (!s.initialized || !(speed >= a.min_speed_mps) || !(cmd > real(.02)) || !std::isfinite(v)) {
                s.scale[i] = 1;
                s.integral[i] = 0;
                continue;
            }
            if (in.is_front[i]) front_regulated = true;
            const real slip = (speed - dir * v) / std::max(speed, a.slip_floor_mps);
            const real err = slip - a.target_slip;
            s.integral[i] = std::clamp(s.integral[i] + a.ki_per_s * err * dt, real(0), real(1));
            const real desired = std::clamp(real(1) - a.kp * std::max(real(0), err) - s.integral[i], real(0), real(1));
            const real rate = desired < s.scale[i] ? a.release_rate_per_s : a.apply_rate_per_s;
            s.scale[i] += std::clamp(desired - s.scale[i], -rate * dt, rate * dt);
        }
        // Yaw-moment build-up limit: the higher front scale may exceed the lower one by front_split, which grows from 0
        // at front_split_rate_per_s up to front_split_max while the front axle is regulated (split-mu yaw).
        real front_low = 1;
        for (std::size_t i = 0; i < n; ++i) if (in.is_front[i]) front_low = std::min(front_low, s.scale[i]);
        if (front_regulated) s.front_split = std::min(a.front_split_max, s.front_split + a.front_split_rate_per_s * dt);
        else s.front_split = 0;
        real rear_low = 1;
        if (a.rear_select_low) for (std::size_t i = 0; i < n; ++i) if (!in.is_front[i]) rear_low = std::min(rear_low, s.scale[i]);
        for (std::size_t i = 0; i < n; ++i) {
            real sc = s.scale[i];
            if (in.is_front[i]) sc = std::min(sc, front_low + s.front_split);
            else if (a.rear_select_low) sc = rear_low;
            s.applied[i] = sc;
            if (sc < real(.995)) s.abs_active = true;
            in.wheel_brake_out[i] *= sc;
        }
    } else {
        std::fill(s.scale.begin(), s.scale.end(), real(1));
        std::fill(s.integral.begin(), s.integral.end(), real(0));
        std::fill(s.applied.begin(), s.applied.end(), real(1));
        s.front_split = 0;
    }
}

inline std::uint64_t fold_abs_esc_hash(const AbsEscState& s, std::uint64_t h) {
    const auto fold = [&](const auto& v) { const auto* b = reinterpret_cast<const unsigned char*>(&v); for (std::size_t i = 0; i < sizeof(v); ++i) { h ^= b[i]; h *= 1099511628211ULL; } };
    const std::uint64_t tag = 0x414253455343310aULL;
    fold(tag);
    fold(s.reference_speed_m_s);
    fold(s.initialized);
    for (real v : s.scale) fold(v);
    for (real v : s.integral) fold(v);
    for (real v : s.esc_brake) fold(v);
    fold(s.front_split);
    return h;
}

} // namespace ps::vehicle
