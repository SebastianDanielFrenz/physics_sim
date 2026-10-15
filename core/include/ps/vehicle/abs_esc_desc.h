// ps/vehicle/abs_esc_desc.h - anti-lock brakes (assists I5 part 2) and electronic stability control (assists I6), two
// blocks of the assist unit (ps/vehicle/assist_unit_desc.h, docs/data_formats/electronics.md "abs" / "esc").
//
// Both are SENSOR-ONLY like traction control: wheel peripheral speeds, the steering angle and an ideal IMU (gyro yaw
// rate, longitudinal acceleration) - no ground speed, no tyre slip, no contact load, no surface. Both act only through
// the car's existing service-brake friction rows (DirectTorqueBrakes): ABS scales each corner's share of the brake
// command, ESC adds a per-corner brake command and cuts engine torque through the traction-control throttle ceiling.
// Channels assist.abs (0/1) and assist.esc (mode code) are registered only on a car whose unit carries the block.
#pragma once

#include "ps/types.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ps::vehicle {

struct AbsDesc {
    bool default_enabled = true;      // the channel assist.abs's default (0/1)
    real target_slip = 0.12;          // [AS] braking slip ratio the controller holds (peak of a passenger tyre, 0.10-0.15)
    real min_speed_mps = 2.0;         // [AS] below this reference speed ABS hands the full command back (wheels may lock)
    real slip_floor_mps = 3.0;        // [AS] slip = (v_ref - v_wheel) / max(v_ref, floor)
    // The per-wheel law: scale = clamp(1 - kp * max(0, slip - target) - integral, 0, 1), integral += ki * (slip - target) dt
    // clamped to [0, 1], scale rate-limited (release_rate_per_s down, apply_rate_per_s up). [AS], tuned on car_sedan.
    real kp = 3.0;
    real ki_per_s = 30.0;
    real release_rate_per_s = 60.0;
    real apply_rate_per_s = 15.0;
    real reanchor_rate_per_s = 10.0;  // [AS] off the brake the reference speed relaxes toward the slowest wheel at this rate
    real reanchor_scale = 0.05;       // [AS] braking with every regulated wheel's scale <= this: relax toward the fastest wheel
    bool rear_select_low = true;      // [AS] the rear axle runs the lower of its wheels' scales (split-mu yaw)
    // [AS] yaw-moment build-up limit (split mu): the higher front scale <= the lower one + split, split growing from 0 at
    // front_split_rate_per_s up to front_split_max while the front axle is regulated. 1 / any rate = no limit.
    real front_split_max = 1.0;
    real front_split_rate_per_s = 1.0;
};

// ESC mode CODES of the channel assist.esc. Append-only. Intervention: On > Sport > Off.
enum class EscMode : std::uint8_t { Off = 0, On = 1, Sport = 2 };
inline constexpr int kEscModeCount = 3;
inline constexpr const char* kEscModeNames[kEscModeCount] = {"off", "on", "sport"};
[[nodiscard]] constexpr std::uint8_t esc_mode_bit(EscMode m) { return static_cast<std::uint8_t>(1u << static_cast<unsigned>(m)); }

// The parameters a mode may change (mode Sport = mode On with its overrides applied by the loader).
struct EscModeParams {
    real deadband_rad_s = 0.05;       // [AS] no intervention while |r - r_target| <= deadband + deadband_fraction * |r_target|
    real deadband_fraction = 0.10;    // [AS]
    real brake_gain_per_rad_s = 2.0;  // [AS] corner brake command per rad/s of yaw-rate error beyond the deadband
    real max_brake = 0.8;             // [AS] cap of that command (fraction of the corner's max_torque_nm)
    real torque_gain_per_rad_s = 1.0; // [AS] engine torque cut per rad/s beyond the deadband (fraction of the command)
};

struct EscDesc {
    std::uint8_t modes_bits = 0b011;  // off, on (sport only when listed)
    EscMode default_mode = EscMode::On;
    // Yaw-rate target: the linear bicycle model r = v delta / (L (1 + (v / v_ch)^2)), delta = the mean front road-wheel
    // angle, L = the car's wheelbase, limited to |r| <= max_lateral_accel / v.
    real characteristic_speed_mps = 25.0;  // [AS] passenger car 20-30 m/s (understeer gradient L g / v_ch^2)
    real max_lateral_accel_mps2 = 8.5;     // [AS] ~0.87 g, a dry-road passenger car
    real min_speed_mps = 5.0;              // [AS] ESC is inactive below this reference speed (and in reverse)
    real brake_rate_per_s = 10.0;          // [AS] rate of each corner's ESC brake command (0 -> 1 in 0.1 s)
    std::array<EscModeParams, kEscModeCount> params{}; // index = mode code; [0] unused, [2] = sport (= on unless overridden)
};

[[nodiscard]] constexpr int esc_mode_rank(EscMode m) {
    constexpr int r[kEscModeCount] = {0, 2, 1};
    return r[static_cast<int>(m)];
}

// Channel value -> requested mode: non-finite or < 0.5 is Off, else clamp(round(value), 1, 2).
[[nodiscard]] inline EscMode esc_mode_requested(real v) {
    if (!std::isfinite(v) || !(v >= real(0.5))) return EscMode::Off;
    const real r = std::round(v);
    return r >= real(2) ? EscMode::Sport : EscMode::On;
}

// The mode run: a requested mode the unit does not offer maps to the nearest MORE intervening offered mode (Sport -> On);
// an unoffered Off maps to the least intervening offered mode. On is always offered (validate_esc).
[[nodiscard]] constexpr EscMode resolve_esc_mode(std::uint8_t bits, EscMode req) {
    if (bits & esc_mode_bit(req)) return req;
    if (req == EscMode::Off) return (bits & esc_mode_bit(EscMode::Sport)) ? EscMode::Sport : EscMode::On;
    return EscMode::On;
}

// Shared by the loader and VehicleInstance (code-built descs). Throw std::invalid_argument.
inline void validate_abs(const AbsDesc& d) {
    for (real v : {d.target_slip, d.min_speed_mps, d.slip_floor_mps, d.kp, d.ki_per_s, d.release_rate_per_s, d.apply_rate_per_s, d.reanchor_rate_per_s}) {
        if (!std::isfinite(v) || !(v > real(0))) throw std::invalid_argument("abs: parameters must be finite and > 0");
    }
    if (!(d.target_slip < real(1))) throw std::invalid_argument("abs: target_slip must be below 1");
    if (!std::isfinite(d.reanchor_scale) || d.reanchor_scale < real(0) || !(d.reanchor_scale < real(1))) {
        throw std::invalid_argument("abs: reanchor_scale must be in [0, 1)");
    }
    if (!std::isfinite(d.front_split_max) || d.front_split_max < real(0) || d.front_split_max > real(1)) {
        throw std::invalid_argument("abs: front_split_max must be in [0, 1]");
    }
    if (!std::isfinite(d.front_split_rate_per_s) || !(d.front_split_rate_per_s > real(0))) {
        throw std::invalid_argument("abs: front_split_rate_per_s must be finite and > 0");
    }
}

inline void validate_esc(const EscDesc& d) {
    if (!(d.modes_bits & esc_mode_bit(EscMode::On)) || (d.modes_bits & ~std::uint8_t(0b111)) != 0) {
        throw std::invalid_argument("esc: modes must contain 'on' (and only off, on, sport)");
    }
    if (!(d.modes_bits & esc_mode_bit(d.default_mode))) throw std::invalid_argument("esc: default_mode is not one of the offered modes");
    for (real v : {d.characteristic_speed_mps, d.max_lateral_accel_mps2, d.min_speed_mps, d.brake_rate_per_s}) {
        if (!std::isfinite(v) || !(v > real(0))) throw std::invalid_argument("esc: speeds, accelerations and rates must be finite and > 0");
    }
    for (int m = 1; m < kEscModeCount; ++m) {
        const EscModeParams& p = d.params[static_cast<std::size_t>(m)];
        for (real v : {p.deadband_rad_s, p.deadband_fraction, p.brake_gain_per_rad_s, p.torque_gain_per_rad_s}) {
            if (!std::isfinite(v) || v < real(0)) throw std::invalid_argument(std::string("esc: mode '") + kEscModeNames[m] + "': deadbands and gains must be finite and >= 0");
        }
        if (!std::isfinite(p.max_brake) || p.max_brake < real(0) || p.max_brake > real(1)) {
            throw std::invalid_argument(std::string("esc: mode '") + kEscModeNames[m] + "': max_brake must be in [0, 1]");
        }
    }
}

} // namespace ps::vehicle
