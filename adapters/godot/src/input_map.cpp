#include "input_map.h"

#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/core/class_db.hpp>

using godot::ClassDB;
using godot::D_METHOD;
using godot::Input;

namespace ps_godot {

PsInputMap::PsInputMap() {
    InputMapConfig cfg;
    cfg.steer.deadzone = 0.08f;
    cfg.throttle.deadzone = 0.02f;
    cfg.brake.deadzone = 0.02f;
    cfg.handbrake.deadzone = 0.02f;
    cfg.clutch.deadzone = 0.02f;
    mapper_.set_config(cfg);
}

RawInput PsInputMap::gather_raw_input() const {
    Input* input = Input::get_singleton();
    RawInput raw;
    if (input == nullptr) return raw;

    // Analog: raw joypad axis, bypassing Godot's own action deadzone so
    // input_map_core.h's deadzone/curve is the only shaping applied.
    const float joy_steer = input->get_joy_axis(joy_device_, godot::JOY_AXIS_LEFT_X);
    const float joy_throttle = input->get_joy_axis(joy_device_, godot::JOY_AXIS_TRIGGER_RIGHT);
    const float joy_brake = input->get_joy_axis(joy_device_, godot::JOY_AXIS_TRIGGER_LEFT);

    // Keyboard fallback via named, remappable Godot InputMap actions
    // (demo/project.godot [input] section). get_axis on two digital actions
    // naturally returns exactly -1/0/1 with no deadzone concept involved,
    // so merging it with the joypad's raw axis by taking the
    // larger-magnitude source is deadzone-safe either way.
    const float key_steer = input->get_axis("ps_steer_left", "ps_steer_right");
    const float key_throttle = input->get_action_strength("ps_throttle");
    const float key_brake = input->get_action_strength("ps_brake");

    raw.steer_axis = std::fabs(key_steer) > std::fabs(joy_steer) ? key_steer : joy_steer;
    raw.throttle_trigger = key_throttle > joy_throttle ? key_throttle : joy_throttle;
    raw.brake_trigger = key_brake > joy_brake ? key_brake : joy_brake;

    const bool joy_handbrake = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_A);
    raw.handbrake_trigger = (joy_handbrake || input->is_action_pressed("ps_handbrake_button")) ? 1.0f : 0.0f;

    const bool joy_clutch = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_X);
    raw.clutch_trigger = (joy_clutch || input->is_action_pressed("ps_clutch_button")) ? 1.0f : 0.0f;

    raw.shift_up_pressed = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_RIGHT_SHOULDER)
        || input->is_action_just_pressed("ps_shift_up");
    raw.shift_down_pressed = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_LEFT_SHOULDER)
        || input->is_action_just_pressed("ps_shift_down");
    raw.camera_pressed = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_Y)
        || input->is_action_just_pressed("ps_camera");
    raw.reset_vehicle_pressed = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_BACK)
        || input->is_action_just_pressed("ps_reset_vehicle");

    return raw;
}

void PsInputMap::_process(double /*delta*/) {
    channels_ = mapper_.process(gather_raw_input());
}

void PsInputMap::set_steer_deadzone(float v) {
    InputMapConfig cfg = mapper_.config();
    cfg.steer.deadzone = v;
    mapper_.set_config(cfg);
}
void PsInputMap::set_steer_curve(float v) {
    InputMapConfig cfg = mapper_.config();
    cfg.steer.curve_exponent = v;
    mapper_.set_config(cfg);
}
void PsInputMap::set_steer_invert(bool v) {
    InputMapConfig cfg = mapper_.config();
    cfg.steer.invert = v;
    mapper_.set_config(cfg);
}
void PsInputMap::set_trigger_deadzone(float v) {
    InputMapConfig cfg = mapper_.config();
    cfg.throttle.deadzone = v;
    cfg.brake.deadzone = v;
    cfg.handbrake.deadzone = v;
    cfg.clutch.deadzone = v;
    mapper_.set_config(cfg);
}

void PsInputMap::rumble(float /*low_frequency*/, float /*high_frequency*/, float /*duration_s*/) {
    // PLAN.md P2: "a rumble hook that does nothing yet" - fed from tyre
    // slip/ABS once P3/P4 exist. Godot's own API for this once wired up:
    // Input::get_singleton()->start_joy_vibration(joy_device_, low, high, duration).
}

void PsInputMap::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_steer"), &PsInputMap::get_steer);
    ClassDB::bind_method(D_METHOD("get_throttle"), &PsInputMap::get_throttle);
    ClassDB::bind_method(D_METHOD("get_brake"), &PsInputMap::get_brake);
    ClassDB::bind_method(D_METHOD("get_handbrake"), &PsInputMap::get_handbrake);
    ClassDB::bind_method(D_METHOD("get_clutch"), &PsInputMap::get_clutch);
    ClassDB::bind_method(D_METHOD("get_shift_up"), &PsInputMap::get_shift_up);
    ClassDB::bind_method(D_METHOD("get_shift_down"), &PsInputMap::get_shift_down);
    ClassDB::bind_method(D_METHOD("get_camera"), &PsInputMap::get_camera);
    ClassDB::bind_method(D_METHOD("get_reset_vehicle"), &PsInputMap::get_reset_vehicle);

    ClassDB::bind_method(D_METHOD("set_steer_deadzone", "value"), &PsInputMap::set_steer_deadzone);
    ClassDB::bind_method(D_METHOD("get_steer_deadzone"), &PsInputMap::get_steer_deadzone);
    ClassDB::bind_method(D_METHOD("set_steer_curve", "value"), &PsInputMap::set_steer_curve);
    ClassDB::bind_method(D_METHOD("get_steer_curve"), &PsInputMap::get_steer_curve);
    ClassDB::bind_method(D_METHOD("set_steer_invert", "value"), &PsInputMap::set_steer_invert);
    ClassDB::bind_method(D_METHOD("get_steer_invert"), &PsInputMap::get_steer_invert);
    ClassDB::bind_method(D_METHOD("set_trigger_deadzone", "value"), &PsInputMap::set_trigger_deadzone);
    ClassDB::bind_method(D_METHOD("get_trigger_deadzone"), &PsInputMap::get_trigger_deadzone);
    ClassDB::bind_method(D_METHOD("set_joy_device", "device"), &PsInputMap::set_joy_device);
    ClassDB::bind_method(D_METHOD("get_joy_device"), &PsInputMap::get_joy_device);
    ClassDB::bind_method(D_METHOD("rumble", "low_frequency", "high_frequency", "duration_s"), &PsInputMap::rumble);

    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "steer_deadzone"), "set_steer_deadzone", "get_steer_deadzone");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "steer_curve"), "set_steer_curve", "get_steer_curve");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::BOOL, "steer_invert"), "set_steer_invert", "get_steer_invert");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "trigger_deadzone"), "set_trigger_deadzone", "get_trigger_deadzone");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::INT, "joy_device"), "set_joy_device", "get_joy_device");
}

} // namespace ps_godot
