#include "input_map.h"
#include "frame_timing.h"
#include "ps_simulation.h"

#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

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

// Bounded, allocation-free copy into HapticsWheelInput::surface_name's fixed
// buffer (haptics_core.h's own "allocation-free per call" contract) - the
// one piece of glue this task allows (PsSimulation::get_wheel_surface_name
// already exists and returns a plain godot::String; ps_simulation.h/.cpp
// itself is not touched). Truncates to the buffer's capacity minus the NUL
// terminator; every name in data/surfaces/surfaces.json ("asphalt", "kerb",
// "dirt", "field", "grass", "low_mu") is well under that.
void copy_surface_name(const String& s, std::array<char, 16>& out) {
    out.fill('\0');
    const godot::CharString utf8 = s.utf8();
    const char* data = utf8.get_data();
    std::size_t i = 0;
    for (; i < out.size() - 1 && data[i] != '\0'; ++i) out[i] = data[i];
    out[i] = '\0';
}
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
    // reference; the demo softens the body-G/grip-limit/stage feel on top
    // of them. The light-slip thresholds (slip_ratio/slip_angle_threshold)
    // used to be overridden here too, but as of the 2026-09-25 "good
    // asphalt doesn't cause rumble" fix the CORE defaults are themselves
    // the correct, measured-against-real-telemetry values (haptics_core.h's
    // own top comment, sources 3/4) - overriding them here again would
    // just make the demo MORE sensitive than the tuning that was actually
    // measured against, so this constructor no longer touches them.
    HapticsTuning ht;
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
    // (hold), right stick click = debug draw (below). Owner remap
    // (2026-10-05): Y = N2O arm toggle, RB = camera (was Y).
    const bool joy_clutch = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_LEFT_SHOULDER);
    raw.clutch_button_held = joy_clutch || input->is_action_pressed("ps_clutch_button");

    raw.shift_up_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_B)
        || input->is_action_pressed("ps_shift_up");
    raw.shift_down_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_X)
        || input->is_action_pressed("ps_shift_down");
    raw.camera_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_RIGHT_SHOULDER)
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

    // Tractor demo (2026-09-28): keyboard-only (demo/project.godot's
    // ps_toggle_awd/ps_toggle_diff_lock/ps_toggle_pto/ps_toggle_pto_speed/
    // ps_toggle_park_lock/ps_hand_throttle_up/ps_hand_throttle_down
    // actions, physical keys T/L/P/O/N/]/[) - every gamepad button already
    // has a job (see the remap comment above).
    raw.toggle_awd_held = input->is_action_pressed("ps_toggle_awd");
    raw.toggle_diff_lock_held = input->is_action_pressed("ps_toggle_diff_lock");
    raw.toggle_pto_held = input->is_action_pressed("ps_toggle_pto");
    raw.toggle_pto_speed_held = input->is_action_pressed("ps_toggle_pto_speed");
    raw.toggle_park_lock_held = input->is_action_pressed("ps_toggle_park_lock");
    raw.toggle_n2o_held = input->is_joy_button_pressed(joy_device_, godot::JOY_BUTTON_Y)
        || input->is_action_pressed("ps_toggle_n2o");
    raw.hand_throttle_up_held = input->is_action_pressed("ps_hand_throttle_up");
    raw.hand_throttle_down_held = input->is_action_pressed("ps_hand_throttle_down");

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
        // Bug B (owner, 2026-09-25: "didn't feel the kerbs at all") - source
        // 7's surface-texture match (haptics_core.h) needs the ground surface
        // name under this wheel; PsSimulation already exposes it.
        copy_surface_name(simulation_->get_wheel_surface_name(vehicle_name_, w), hw.surface_name);
    }

    const Dictionary pt = simulation_->get_vehicle_powertrain(vehicle_name_);
    if (pt.size() > 0) {
        in.engine.rpm = static_cast<float>(pt.get("rpm", 0.0));
        in.engine.grind_active = static_cast<bool>(pt.get("grind_active", false));
        in.engine.stalled = String(pt.get("engine_state", "")) == String("stalled");
        // Deliberately not reading the "limiter" key into HapticsEngineInput
        // (owner requirement, 2026-09-25 - see haptics_core.h's own top
        // comment on source 6): a bare rev-limiter flag must not drive
        // rumble on its own, and HapticsEngineInput no longer has a field
        // for it at all.
    }

    const HapticsOutput out = haptics_.update(in);

    // PS_HAPTICS_LOG debug readout (haptics_core.h's own top comment) -
    // gated on an env var so a normal run pays nothing beyond one cached
    // bool check per frame; read once (std::getenv is a linear scan of the
    // environment block, not something to repeat every frame) and logged
    // at most once a second so a multi-minute drive's log stays readable.
    if (!haptics_log_checked_) {
        haptics_log_checked_ = true;
#if defined(_WIN32)
        // getenv() is flagged deprecated (-Werror on this toolchain's ucrt
        // headers) - _dupenv_s is the suggested replacement, same pattern
        // as core/src/jobs/debug_alloc.cpp's own backtrace_budget().
        char* env = nullptr;
        std::size_t env_len = 0;
        if (_dupenv_s(&env, &env_len, "PS_HAPTICS_LOG") == 0 && env != nullptr) {
            haptics_log_enabled_ = env[0] != '\0' && env[0] != '0';
            std::free(env);
        }
#else
        const char* env = std::getenv("PS_HAPTICS_LOG");
        haptics_log_enabled_ = env != nullptr && env[0] != '\0' && env[0] != '0';
#endif
    }
    if (haptics_log_enabled_) {
        haptics_log_accum_s_ += dt;
        if (haptics_log_accum_s_ >= 1.0f) {
            haptics_log_accum_s_ = 0.0f;
            const HapticsSourceDebug& d = haptics_.debug();
            const int gear = pt.size() > 0 ? static_cast<int>(pt.get("gear", 0)) : 0;
            std::printf(
                "PS_HAPTICS_LOG t=%.2f speed_kmh=%.1f gear=%d src1_body_g=%.4f src2_grip=%.4f "
                "src3_load=%.4f src3_load_rms_raw=%.5f src4_slip=%.4f src5_land=%.4f src6_eng=%.4f src7_surf=%.4f "
                "strong_raw=%.4f weak_raw=%.4f strong_out=%.4f weak_out=%.4f\n",
                static_cast<double>(simulation_->get_sim_time()),
                static_cast<double>(ground_speed * 3.6f),
                gear,
                static_cast<double>(d.source1_body_g), static_cast<double>(d.source2_grip_limit),
                static_cast<double>(d.source3_load_noise), static_cast<double>(d.source3_load_noise_rms_raw),
                static_cast<double>(d.source4_light_slip),
                static_cast<double>(d.source5_landing), static_cast<double>(d.source6_engine),
                static_cast<double>(d.source7_surface_texture),
                static_cast<double>(d.strong_raw), static_cast<double>(d.weak_raw),
                static_cast<double>(d.strong_out), static_cast<double>(d.weak_out));
        }
    }

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

Dictionary PsInputMap::get_haptics_debug() const {
    Dictionary d;
    const HapticsSourceDebug& hd = haptics_.debug();
    d["source1_body_g"] = hd.source1_body_g;
    d["source2_grip_limit"] = hd.source2_grip_limit;
    d["source3_load_noise"] = hd.source3_load_noise;
    d["source3_load_noise_rms_raw"] = hd.source3_load_noise_rms_raw;
    d["source4_light_slip"] = hd.source4_light_slip;
    d["source5_landing"] = hd.source5_landing;
    d["source6_engine"] = hd.source6_engine;
    d["source7_surface_texture"] = hd.source7_surface_texture;
    d["strong_raw"] = hd.strong_raw;
    d["weak_raw"] = hd.weak_raw;
    d["strong_out"] = hd.strong_out;
    d["weak_out"] = hd.weak_out;
    return d;
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
    ClassDB::bind_method(D_METHOD("get_toggle_awd"), &PsInputMap::get_toggle_awd);
    ClassDB::bind_method(D_METHOD("get_toggle_diff_lock"), &PsInputMap::get_toggle_diff_lock);
    ClassDB::bind_method(D_METHOD("get_toggle_pto"), &PsInputMap::get_toggle_pto);
    ClassDB::bind_method(D_METHOD("get_toggle_pto_speed"), &PsInputMap::get_toggle_pto_speed);
    ClassDB::bind_method(D_METHOD("get_toggle_park_lock"), &PsInputMap::get_toggle_park_lock);
    ClassDB::bind_method(D_METHOD("get_toggle_n2o"), &PsInputMap::get_toggle_n2o);
    ClassDB::bind_method(D_METHOD("get_hand_throttle"), &PsInputMap::get_hand_throttle);

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
    ClassDB::bind_method(D_METHOD("get_haptics_debug"), &PsInputMap::get_haptics_debug);

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
