#include "input_map_core.h"

#include <cmath>
#include <algorithm>

namespace ps_godot {

namespace {
float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
float clamp11(float v) { return std::clamp(v, -1.0f, 1.0f); }
} // namespace

float shape_bidirectional_axis(float raw, const AxisConfig& cfg) {
    raw = clamp11(raw);
    if (cfg.invert) raw = -raw;
    const float mag = std::fabs(raw);
    const float dz = std::clamp(cfg.deadzone, 0.0f, 0.999f);
    if (mag <= dz) return 0.0f;
    // Rescale [dz, 1] -> [0, 1] so the usable travel still reaches full
    // deflection, then reapply the sign.
    float shaped = (mag - dz) / (1.0f - dz);
    shaped = clamp01(shaped);
    if (cfg.curve_exponent != 1.0f) shaped = std::pow(shaped, cfg.curve_exponent);
    return raw < 0.0f ? -shaped : shaped;
}

float shape_unidirectional_axis(float raw, const TriggerConfig& cfg) {
    raw = clamp01(raw);
    const float dz = std::clamp(cfg.deadzone, 0.0f, 0.999f);
    if (raw <= dz) return 0.0f;
    float shaped = (raw - dz) / (1.0f - dz);
    shaped = clamp01(shaped);
    if (cfg.curve_exponent != 1.0f) shaped = std::pow(shaped, cfg.curve_exponent);
    return shaped;
}

ControlChannels InputMapper::process(const RawInput& raw) const {
    ControlChannels out;
    out.steer = shape_bidirectional_axis(raw.steer_axis, config_.steer);
    out.throttle = shape_unidirectional_axis(raw.throttle_trigger, config_.throttle);
    out.brake = shape_unidirectional_axis(raw.brake_trigger, config_.brake);
    out.handbrake = shape_unidirectional_axis(raw.handbrake_trigger, config_.handbrake);
    out.clutch = shape_unidirectional_axis(raw.clutch_trigger, config_.clutch);
    out.shift_up = raw.shift_up_pressed;
    out.shift_down = raw.shift_down_pressed;
    out.camera = raw.camera_pressed;
    out.reset_vehicle = raw.reset_vehicle_pressed;
    return out;
}

} // namespace ps_godot
