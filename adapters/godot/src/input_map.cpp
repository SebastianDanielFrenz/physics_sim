#include "input_map.h"
#include "frame_timing.h"
#include "ps_simulation.h"

#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <algorithm>

using godot::ClassDB;
using godot::D_METHOD;
using godot::Dictionary;
using godot::Input;
using godot::String;
using godot::Vector3;

namespace ps_godot {

namespace {
// self-stopping duration (owner spec: "duration ~0.1 s so it self-stops if
// frames stop") - called every active frame regardless, so this only
// matters if the render loop itself stalls.
constexpr float kRumbleDurationS = 0.1f;
} // namespace

PsInputMap::PsInputMap() {
    InputMapConfig cfg;
    cfg.steer.deadzone = 0.08f;
    cfg.throttle.deadzone = 0.02f;
    cfg.brake.deadzone = 0.02f;
    cfg.handbrake.deadzone = 0.02f;
    cfg.clutch.deadzone = 0.02f;
    mapper_.set_config(cfg);

    // Demo rumble feel (owner, 2026-09-24: "the vibration is too
    // aggressive"). haptics_core.h's defaults stay the unit-tested
    // reference; the demo softens them: no buzz in ordinary cornering (light
    // slip starts at 3 deg / 6 % instead of 1.5 deg / 4 %), bigger bumps
    // needed for a body-G jolt, lower stage levels and 60 % overall.
    HapticsTuning ht;
    ht.slip_angle_threshold_rad = 0.0523599f; // 3 deg
    ht.slip_angle_saturate_rad = 0.0872665f;  // 5 deg
    ht.slip_ratio_threshold = 0.06f;
    ht.slip_ratio_saturate = 0.15f;
    ht.body_g_threshold_mps2 = 1.5f;
    ht.body_g_saturate_mps2 = 20.0f;
    ht.grip_limit_saturate_utilization = 3.5f;
    ht.stage_levels = {0.0f, 0.12f, 0.30f, 0.50f, 0.75f};
    ht.strength = 0.6f;
    haptics_.set_tuning(ht);
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

    // PHYS-033 P4 I3 section 12.2: every digital action below is now read as
    // a plain LEVEL (is_joy_button_pressed / is_action_pressed), not an edge
    // - InputMapper::process (input_map_core.cpp) does its own edge
    // detection against a `prev` bitset, identically for every source. The
    // digital clutch button is a level too (clutch_button_held), smoothed on
    // release inside InputMapper rather than snapped to 0/1 here.
    // Owner remap (2026-09-24): B = upshift, X = downshift, LB = clutch
    // (hold), right stick click = debug draw (below). RB is unbound.
    const bool joy_clutch = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_LEFT_SHOULDER);
    raw.clutch_button_held = joy_clutch || input->is_action_pressed("ps_clutch_button");

    raw.shift_up_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_B)
        || input->is_action_pressed("ps_shift_up");
    raw.shift_down_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_X)
        || input->is_action_pressed("ps_shift_down");
    raw.camera_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_Y)
        || input->is_action_pressed("ps_camera");
    raw.reset_vehicle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_BACK)
        || input->is_action_pressed("ps_reset_vehicle");
    raw.debug_toggle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_RIGHT_STICK)
        || input->is_action_pressed("ps_debug_toggle");

    // New bindings (PLAN.md P4 I3 section 12.2): Menu/Start = starter
    // (hold), D-pad Up/Left/Down/Right = ignition/auto-shift/auto-clutch/
    // auto-blip toggles. Keyboard: I/K/F5/F6/F7 (demo/project.godot
    // [input]'s ps_ignition/ps_starter/ps_toggle_auto_shift/
    // ps_toggle_auto_clutch/ps_toggle_auto_blip actions).
    raw.starter_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_START)
        || input->is_action_pressed("ps_starter");
    raw.ignition_toggle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_DPAD_UP)
        || input->is_action_pressed("ps_ignition");
    raw.auto_shift_toggle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_DPAD_LEFT)
        || input->is_action_pressed("ps_toggle_auto_shift");
    raw.auto_clutch_toggle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_DPAD_DOWN)
        || input->is_action_pressed("ps_toggle_auto_clutch");
    raw.auto_blip_toggle_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_DPAD_RIGHT)
        || input->is_action_pressed("ps_toggle_auto_blip");

    return raw;
}

void PsInputMap::_process(double delta) {
    const ScopedAdapterTimer timer; // frame_timing.h
    RawInput raw = gather_raw_input();
    // Steering assist (input_map_core.h's SteerAssistConfig): needs this
    // vehicle's geometry once, and its ground speed every frame.
    if (simulation_ != nullptr && !vehicle_name_.is_empty()) {
        if (!steer_assist_configured_) {
            const godot::Vector2 geo = simulation_->get_vehicle_steer_geometry(vehicle_name_);
            if (geo.x > 0.0f && geo.y > 0.0f) {
                InputMapConfig cfg = mapper_.config();
                cfg.steer_assist.wheelbase_m = geo.x;
                cfg.steer_assist.max_wheel_angle_rad = geo.y;
                mapper_.set_config(cfg);
                steer_assist_configured_ = true;
            }
        }
        raw.vehicle_speed_mps = simulation_->get_vehicle_ground_speed_mps(vehicle_name_);
    }
    channels_ = mapper_.process(raw, static_cast<float>(delta));

    // Rumble enable toggle (owner request: "add an enable toggle on a free
    // key" - demo/project.godot's ps_rumble_toggle action, physical key H,
    // verified free of any existing binding). A plain UI convenience, not a
    // named ControlChannels channel, so it uses Godot's own edge-detected
    // is_action_just_pressed directly rather than going through
    // input_map_core.h's RawInput/InputMapper.
    Input* input = Input::get_singleton();
    if (input != nullptr && input->is_action_just_pressed("ps_rumble_toggle")) {
        set_rumble_enabled(!rumble_enabled_);
    }

    update_haptics(static_cast<float>(delta));
}

void PsInputMap::_exit_tree() {
    // Owner spec: "stop_joy_vibration on disable/pause/exit/tree exit".
    if (rumble_was_active_) {
        Input* input = Input::get_singleton();
        if (input != nullptr) input->stop_joy_vibration(joy_device_);
        rumble_was_active_ = false;
    }
}

void PsInputMap::update_haptics(float dt) {
    if (simulation_ == nullptr && !simulation_path_.is_empty()) {
        // Resolved lazily (not in _ready()/the constructor): main.gd adds
        // PsSimulation and PsInputMap as siblings and only sets
        // simulation_path after both are in the tree, and godot-cpp's
        // get_node_or_null needs this node to already be inside the tree
        // to resolve a relative/absolute NodePath at all. Same is_empty()
        // guard as debug_draw.cpp's own resolve_simulation() - an empty
        // NodePath into get_node_or_null prints a Godot-side error (which
        // would fail smoke_test.ps1's "no errors printed" check).
        simulation_ = godot::Object::cast_to<PsSimulation>(get_node_or_null(simulation_path_));
    }

    const bool active = simulation_ != nullptr && !vehicle_name_.is_empty() && simulation_->is_running() && rumble_enabled_;
    if (!active) {
        if (rumble_was_active_) {
            Input* input = Input::get_singleton();
            if (input != nullptr) input->stop_joy_vibration(joy_device_);
            rumble_was_active_ = false;
        }
        return;
    }

    // --- Gather this frame's HapticsInput from PsSimulation's existing
    //     race-free getters (AdapterSnapshot-backed - see ps_simulation.h's
    //     VehicleDynamicSnapshot comment) - no shaping math here, that's
    //     entirely haptics_core.h's job. ---
    HapticsInput in;
    in.dt_s = dt;

    const Vector3 accel = simulation_->get_vehicle_chassis_accel_local(vehicle_name_);
    in.accel_x = accel.x;
    in.accel_y = accel.y;
    in.accel_z = accel.z;

    const int wheel_count = std::min(static_cast<int>(simulation_->get_vehicle_wheel_count(vehicle_name_)), kMaxHapticsWheels);
    in.wheel_count = wheel_count;
    const float ground_speed = simulation_->get_vehicle_ground_speed_mps(vehicle_name_);
    for (int w = 0; w < wheel_count; ++w) {
        HapticsWheelInput& hw = in.wheels[static_cast<std::size_t>(w)];
        hw.normal_load_n = simulation_->get_wheel_load_n(vehicle_name_, w);
        hw.slip_ratio = simulation_->get_wheel_slip_ratio(vehicle_name_, w);
        hw.slip_angle_rad = simulation_->get_wheel_slip_angle(vehicle_name_, w);
        // No dedicated "in contact" getter exists on PsSimulation - normal
        // load is itself the contact signal (an airborne/unloaded wheel's
        // suspension applies no force, fz == 0 exactly - core/CLAUDE.md's
        // wheel_state.h: load == WheelTelemetry::fz). A small epsilon
        // avoids treating float noise around exactly 0 as "in contact".
        hw.in_contact = hw.normal_load_n > 1.0f;
        hw.ground_speed_mps = ground_speed;
        hw.wheel_surface_speed_mps = std::fabs(simulation_->get_wheel_omega(vehicle_name_, w)
                                               * simulation_->get_wheel_radius(vehicle_name_, w));
    }

    const Dictionary pt = simulation_->get_vehicle_powertrain(vehicle_name_);
    if (pt.size() > 0) {
        in.engine.rpm = static_cast<float>(pt.get("rpm", 0.0));
        in.engine.limiter_active = static_cast<bool>(pt.get("limiter", false));
        in.engine.grind_active = static_cast<bool>(pt.get("grind_active", false));
        in.engine.stalled = String(pt.get("engine_state", "")) == String("stalled");
        // limiter_rpm intentionally left at 0 (HapticsEngineInput's own
        // "unknown/not applicable" default): PsSimulation exposes only the
        // discrete `limiter` bool (get_vehicle_powertrain's "limiter" key),
        // not the numeric limiter RPM itself, so the "faint buzz
        // approaching the limiter" ramp (HapticsTuning::
        // engine_limiter_margin_rpm) never activates - only the sharp
        // limiter_active edge does. Adding a numeric getter would be a
        // small additive PsSimulation change if a future pass wants the
        // ramp too; not needed for this task's acceptance.
    }

    const HapticsOutput out = haptics_.update(in);
    Input* input = Input::get_singleton();
    if (input != nullptr) {
        input->start_joy_vibration(joy_device_, out.weak, out.strong, kRumbleDurationS);
    }
    rumble_was_active_ = true;
}

void PsInputMap::set_rumble_enabled(bool v) {
    rumble_enabled_ = v;
    if (!v && rumble_was_active_) {
        Input* input = Input::get_singleton();
        if (input != nullptr) input->stop_joy_vibration(joy_device_);
        rumble_was_active_ = false;
    }
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

void PsInputMap::rumble(float low_frequency, float high_frequency, float duration_s) {
    // Manual/direct call (PLAN.md P2's old stub, now implemented) - a thin
    // pass-through, unrelated to the automatic per-frame haptics_ path in
    // update_haptics() above (which calls start_joy_vibration itself every
    // active frame). Kept for a caller that wants to drive the motors
    // directly instead.
    Input* input = Input::get_singleton();
    if (input != nullptr) {
        input->start_joy_vibration(joy_device_, low_frequency, high_frequency, duration_s);
    }
}

void PsInputMap::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_steer"), &PsInputMap::get_steer);
    ClassDB::bind_method(D_METHOD("get_throttle"), &PsInputMap::get_throttle);
    ClassDB::bind_method(D_METHOD("get_brake"), &PsInputMap::get_brake);
    ClassDB::bind_method(D_METHOD("get_handbrake"), &PsInputMap::get_handbrake);
    ClassDB::bind_method(D_METHOD("get_clutch"), &PsInputMap::get_clutch);
    ClassDB::bind_method(D_METHOD("get_shift_up_count"), &PsInputMap::get_shift_up_count);
    ClassDB::bind_method(D_METHOD("get_shift_down_count"), &PsInputMap::get_shift_down_count);
    ClassDB::bind_method(D_METHOD("get_camera"), &PsInputMap::get_camera);
    ClassDB::bind_method(D_METHOD("get_reset_vehicle"), &PsInputMap::get_reset_vehicle);
    ClassDB::bind_method(D_METHOD("get_debug_toggle"), &PsInputMap::get_debug_toggle);
    ClassDB::bind_method(D_METHOD("get_ignition"), &PsInputMap::get_ignition);
    ClassDB::bind_method(D_METHOD("get_starter"), &PsInputMap::get_starter);
    ClassDB::bind_method(D_METHOD("get_toggle_auto_shift"), &PsInputMap::get_toggle_auto_shift);
    ClassDB::bind_method(D_METHOD("get_toggle_auto_clutch"), &PsInputMap::get_toggle_auto_clutch);
    ClassDB::bind_method(D_METHOD("get_toggle_auto_blip"), &PsInputMap::get_toggle_auto_blip);

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

    ClassDB::bind_method(D_METHOD("set_simulation_path", "path"), &PsInputMap::set_simulation_path);
    ClassDB::bind_method(D_METHOD("get_simulation_path"), &PsInputMap::get_simulation_path);
    ClassDB::bind_method(D_METHOD("set_vehicle_name", "name"), &PsInputMap::set_vehicle_name);
    ClassDB::bind_method(D_METHOD("get_vehicle_name"), &PsInputMap::get_vehicle_name);
    ClassDB::bind_method(D_METHOD("set_rumble_enabled", "enabled"), &PsInputMap::set_rumble_enabled);
    ClassDB::bind_method(D_METHOD("get_rumble_enabled"), &PsInputMap::get_rumble_enabled);

    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "steer_deadzone"), "set_steer_deadzone", "get_steer_deadzone");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "steer_curve"), "set_steer_curve", "get_steer_curve");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::BOOL, "steer_invert"), "set_steer_invert", "get_steer_invert");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::FLOAT, "trigger_deadzone"), "set_trigger_deadzone", "get_trigger_deadzone");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::INT, "joy_device"), "set_joy_device", "get_joy_device");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::NODE_PATH, "simulation_path"), "set_simulation_path", "get_simulation_path");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::STRING, "vehicle_name"), "set_vehicle_name", "get_vehicle_name");
    ClassDB::add_property("PsInputMap", godot::PropertyInfo(godot::Variant::BOOL, "rumble_enabled"), "set_rumble_enabled", "get_rumble_enabled");
}

} // namespace ps_godot
