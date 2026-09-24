#include "input_map_core.h"

#include <cmath>
#include <algorithm>

namespace ps_godot {

namespace {
float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
float clamp11(float v) { return std::clamp(v, -1.0f, 1.0f); }

// Shared clutch-button release ramp (PLAN.md P4 I3 section 12.2): 0.35 s
// linear decay from 1 to 0 once the button is released.
constexpr float kClutchButtonReleaseTimeS = 0.35f;
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

ControlChannels InputMapper::process(const RawInput& raw, float dt) {
    ControlChannels out;
    // RawInput::steer_axis is documented (input_map_core.h) in the
    // hardware/joystick convention shared by input_map.cpp's gamepad
    // (JOY_AXIS_LEFT_X) and keyboard (get_axis("ps_steer_left",
    // "ps_steer_right")) sources alike: -1 = full left, +1 = full right.
    // The sim's "steer" control channel is ISO 8855 (PLAN.md D4: x forward,
    // y left, z up) where + = steering LEFT (core/src/vehicle/
    // vehicle_instance.cpp's Ackermann code, data/models/README.md's sign
    // conventions). This file is the one place that converts RawInput to
    // named ISO-convention control channels (see the header comment above),
    // so the negation belongs here rather than in either input source or in
    // a per-user "invert" preference (AxisConfig::invert, layered on top of
    // this via shape_bidirectional_axis's own sign flip, stays a separate,
    // independent toggle).
    out.steer = -shape_bidirectional_axis(raw.steer_axis, config_.steer);
    out.throttle = shape_unidirectional_axis(raw.throttle_trigger, config_.throttle);
    out.brake = shape_unidirectional_axis(raw.brake_trigger, config_.brake);
    out.handbrake = shape_unidirectional_axis(raw.handbrake_trigger, config_.handbrake);

    // Clutch: analog pedal (as before) OR a digital button, smoothed on
    // release (PLAN.md 12.2) - whichever gives the larger value wins, same
    // merge convention input_map.cpp already uses between gamepad/keyboard
    // sources for the other analog axes. A vehicle with no digital clutch
    // button pressed (clutch_button_held always false) keeps
    // clutch_button_level_ pinned at 0 and this reduces to exactly the old
    // analog-only behaviour.
    const float analog_clutch = shape_unidirectional_axis(raw.clutch_trigger, config_.clutch);
    if (raw.clutch_button_held) {
        clutch_button_level_ = 1.0f;
    } else {
        clutch_button_level_ = std::max(0.0f, clutch_button_level_ - dt / kClutchButtonReleaseTimeS);
    }
    out.clutch = std::max(analog_clutch, clutch_button_level_);

    // Edge detection against last frame's levels (PLAN.md 12.2: "it
    // produces edges (pressed this frame) with a prev bitset").
    if (raw.shift_up_held && !prev_shift_up_) ++shift_up_count_;
    if (raw.shift_down_held && !prev_shift_down_) ++shift_down_count_;
    out.shift_up_count = shift_up_count_;
    out.shift_down_count = shift_down_count_;

    out.camera = raw.camera_held && !prev_camera_;
    out.reset_vehicle = raw.reset_vehicle_held && !prev_reset_vehicle_;
    out.debug_toggle = raw.debug_toggle_held && !prev_debug_toggle_;

    if (raw.ignition_toggle_held && !prev_ignition_toggle_) {
        ignition_ = ignition_ > 0.5f ? 0.0f : 1.0f;
    }
    out.ignition = ignition_;

    out.starter = raw.starter_held ? 1.0f : 0.0f;

    out.toggle_auto_shift = raw.auto_shift_toggle_held && !prev_auto_shift_toggle_;
    out.toggle_auto_clutch = raw.auto_clutch_toggle_held && !prev_auto_clutch_toggle_;
    out.toggle_auto_blip = raw.auto_blip_toggle_held && !prev_auto_blip_toggle_;

    prev_shift_up_ = raw.shift_up_held;
    prev_shift_down_ = raw.shift_down_held;
    prev_camera_ = raw.camera_held;
    prev_reset_vehicle_ = raw.reset_vehicle_held;
    prev_debug_toggle_ = raw.debug_toggle_held;
    prev_ignition_toggle_ = raw.ignition_toggle_held;
    prev_auto_shift_toggle_ = raw.auto_shift_toggle_held;
    prev_auto_clutch_toggle_ = raw.auto_clutch_toggle_held;
    prev_auto_blip_toggle_ = raw.auto_blip_toggle_held;

    return out;
}

} // namespace ps_godot
