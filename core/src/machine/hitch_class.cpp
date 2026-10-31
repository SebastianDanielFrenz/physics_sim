#include "ps/machine/hitch_class.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::machine {

namespace {

constexpr real kDeg = 3.14159265358979323846 / 180.0;
// Yaw and roll come from atan2, so a limit near +-180 degrees would read a wrapped angle as the far side of the stop.
constexpr real kMaxAngle = 170.0 * kDeg;
// A row on yaw or roll scales with 1/cos(pitch): pitch has to stay away from 90 degrees whenever either has one.
constexpr real kMaxPitch = 75.0 * kDeg;

[[noreturn]] void reject(const HitchClassDesc& d, const std::string& what) {
    throw std::invalid_argument("hitch class '" + d.name + "': " + what);
}

void check_axis(const HitchClassDesc& d, const HitchAxis& a, const char* name, real largest) {
    if (a.mode != HitchAxisMode::Limited) return;
    if (!std::isfinite(a.min_rad) || !std::isfinite(a.max_rad) || !(a.min_rad < 0.0) || !(a.max_rad > 0.0)) reject(d, std::string(name) + " limits must be finite, min < 0 < max");
    if (a.min_rad < -largest || a.max_rad > largest) reject(d, std::string(name) + " limits must stay within +-" + std::to_string(static_cast<int>(largest / kDeg + 0.5)) + " degrees");
}

void check_positive(const HitchClassDesc& d, real v, const char* name) {
    if (!std::isfinite(v) || !(v > 0.0)) reject(d, std::string(name) + " must be finite and > 0");
}

} // namespace

void validate_hitch_class(const HitchClassDesc& d) {
    check_axis(d, d.yaw, "yaw", kMaxAngle);
    check_axis(d, d.pitch, "pitch", kMaxPitch);
    check_axis(d, d.roll, "roll", kMaxAngle);
    if (d.pitch.mode == HitchAxisMode::Free && (d.yaw.mode != HitchAxisMode::Free || d.roll.mode != HitchAxisMode::Free)) {
        reject(d, "pitch must be limited or locked when yaw or roll is not free");
    }
    check_positive(d, d.frequency_hz, "frequency_hz");
    if (!std::isfinite(d.damping_ratio) || d.damping_ratio < 0.0) reject(d, "damping_ratio must be finite and >= 0");
    check_positive(d, d.latch_time_s, "latch.time_s");
    check_positive(d, d.max_latch_error_m, "latch.max_error_m");
    check_positive(d, d.max_latch_angle_rad, "latch.max_angle_deg");
    check_positive(d, d.max_latch_speed_m_s, "latch.max_speed_m_s");
}

} // namespace ps::machine
