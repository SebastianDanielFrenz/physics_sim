// adapters/godot/src/input_map_core.h — deadzone/response-curve/trigger
// mapping (PLAN.md P2: "the mapping logic (deadzone, curve, trigger
// normalisation) lives in plain C++ so it can be unit-tested with
// synthetic input").
//
// Godot-free: RawInput below is filled by input_map.h (the thin godot::Input
// / joypad-reading wrapper, same directory) from either a gamepad or the
// keyboard - this file never knows which. Output is named control channels
// in plain SI/ISO terms (steer -1..1, throttle/brake/clutch/handbrake
// 0..1), exactly what PLAN.md P2 requires reaches the sim: "never as
// Godot types".

#pragma once

namespace ps_godot {

// Deadzone + response-curve shaping for a bidirectional axis (steer stick,
// -1..1). Radial-style scaled deadzone: |raw| <= deadzone maps to exactly
// 0; the remaining travel is rescaled to still reach +-1 at |raw| == 1, so
// the whole post-deadzone range is usable (no dead band at the top end).
// curve_exponent applies |x|^exponent with the original sign preserved
// (1.0 = linear, >1.0 = gentler near centre for fine control, <1.0 = more
// aggressive near centre).
struct AxisConfig {
    float deadzone = 0.08f;
    float curve_exponent = 1.0f;
    bool invert = false;
};

// Deadzone + curve for a unidirectional axis (analog trigger, 0..1:
// throttle, brake, clutch, analog handbrake). Same rescaling idea, but
// one-sided.
struct TriggerConfig {
    float deadzone = 0.02f;
    float curve_exponent = 1.0f;
};

[[nodiscard]] float shape_bidirectional_axis(float raw, const AxisConfig& cfg);
[[nodiscard]] float shape_unidirectional_axis(float raw, const TriggerConfig& cfg);

// Raw, unshaped input for one tick, gathered from whichever source is
// active (gamepad and/or keyboard - input_map.h merges them before calling
// process() below, e.g. by taking whichever magnitude is larger per axis
// and OR-ing digital buttons). Every field is already in the -1..1 / 0..1
// range a joypad API or a synthetic test would produce, but NOT yet
// deadzone/curve shaped.
struct RawInput {
    float steer_axis = 0.0f;      // -1 (full left) .. +1 (full right)
    float throttle_trigger = 0.0f; // 0..1
    float brake_trigger = 0.0f;    // 0..1
    float handbrake_trigger = 0.0f; // 0..1 (analog) or 0/1 (digital button, keyboard)
    float clutch_trigger = 0.0f;   // 0..1

    // Digital, edge-triggered actions (true only on the tick they were
    // pressed - input_map.h is responsible for turning a held key/button
    // into a single edge using Godot's own Input::is_action_just_pressed,
    // or a test can set these directly for a synthetic single-frame press).
    bool shift_up_pressed = false;
    bool shift_down_pressed = false;
    bool camera_pressed = false;
    bool reset_vehicle_pressed = false;
};

// Shaped output: exactly the named control channels PLAN.md P2 lists
// (steer, throttle, brake, handbrake, shift up/down, clutch, camera, reset
// vehicle). Consumed today by PsSimulation's control-channel registry (see
// ps_simulation.h's report-flagged request: core has no such registry yet
// for a real vehicle to read from, since no vehicle exists before P3).
struct ControlChannels {
    float steer = 0.0f;      // -1..1
    float throttle = 0.0f;   // 0..1
    float brake = 0.0f;      // 0..1
    float handbrake = 0.0f;  // 0..1
    float clutch = 0.0f;     // 0..1
    bool shift_up = false;
    bool shift_down = false;
    bool camera = false;
    bool reset_vehicle = false;
};

// Per-channel remappable configuration (deadzone/curve/invert). Only the
// analog channels need shaping config; digital actions are pass-through.
struct InputMapConfig {
    AxisConfig steer{};
    TriggerConfig throttle{};
    TriggerConfig brake{};
    TriggerConfig handbrake{};
    TriggerConfig clutch{};
};

class InputMapper {
public:
    explicit InputMapper(InputMapConfig config = {}) : config_(config) {}

    [[nodiscard]] ControlChannels process(const RawInput& raw) const;

    [[nodiscard]] const InputMapConfig& config() const { return config_; }
    void set_config(const InputMapConfig& config) { config_ = config; }

private:
    InputMapConfig config_;
};

} // namespace ps_godot
