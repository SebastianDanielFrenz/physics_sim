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
//
// PHYS-033 P4 I3 section 12.2: edge detection, shift counters, the ignition
// toggle and the clutch-button release ramp all moved INTO this file (were
// previously input_map.cpp's job, done with Godot's own
// is_action_just_pressed - a bug source, since a gamepad button read via
// is_joy_button_pressed has no such built-in edge helper, so RB/LB/Y/B/Back
// used to be read as levels while camera/debug/reset were pressed-edges from
// keyboard actions only - see this section's "existing, fixed" table).
// RawInput now carries plain LEVELS for every digital action; InputMapper
// keeps its own previous-frame bitset and does the edge detection itself,
// identically for gamepad and keyboard sources alike.

#pragma once

#include <cstdint>

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
    float clutch_trigger = 0.0f;   // 0..1, analog path - see clutch_button_held below for the digital path
    // Vehicle ground speed for the steering assist (SteerAssistConfig),
    // m/s; negative = unknown (no assist this frame).
    float vehicle_speed_mps = -1.0f;

    // Digital LEVELS (true while the underlying key/button is held down,
    // every frame - NOT edge-triggered any more; PLAN.md P4 I3 section
    // 12.2). InputMapper below owns edge detection against its own `prev`
    // bitset, identically for every source, so a synthetic test can drive
    // these directly frame by frame instead of hand-rolling a single-frame
    // pulse.
    bool shift_up_held = false;
    bool shift_down_held = false;
    bool camera_held = false;
    bool reset_vehicle_held = false;
    // PLAN.md M1 debug draw toggle (contact points / tyre force vectors /
    // suspension travel) - additive, same level-held convention as the
    // other digital actions here.
    bool debug_toggle_held = false;

    // Digital clutch button (e.g. gamepad X) - an ALTERNATIVE input path to
    // clutch_trigger above, smoothed on release inside InputMapper::process
    // (PLAN.md 12.2: "clutch = held ? 1 : max(0, clutch_prev - dt/0.35)").
    // The two paths are combined by taking whichever is larger, same merge
    // convention input_map.cpp already uses for the analog axes, so a
    // vehicle with only an analog clutch pedal (clutch_trigger, this field
    // left false) behaves exactly as before.
    bool clutch_button_held = false;

    // PHYS-033 P4 I3 section 12.2 additions: ignition/starter/assist
    // channels.
    bool ignition_toggle_held = false; // toggles ControlChannels::ignition on a rising edge
    bool starter_held = false;         // level, passed straight through as ControlChannels::starter
    bool auto_shift_toggle_held = false;  // tri-state toggle request, see ControlChannels::toggle_auto_shift
    bool auto_clutch_toggle_held = false; // ditto, assist.auto_clutch
    bool auto_blip_toggle_held = false;   // ditto, assist.auto_blip

    // Tractor demo (2026-09-28, PHYS-033 P4 I4 C4/C9/C10/C11): tri-state
    // toggle requests for the tractor's drivetrain component channels
    // ("awd", "diff_lock", "pto", "park_lock" - see core's CLAUDE.md
    // "Drivetrain" section), same rising-edge convention as the assist
    // toggles above. Harmless on a vehicle with no such channel (main.gd's
    // World::set_control("awd", ...) etc. on a vehicle that never declared
    // that channel is simply never read back by anything, PLAN.md's control
    // channel map is global/string-keyed - see "Adapter seams").
    bool toggle_awd_held = false;
    bool toggle_diff_lock_held = false;
    bool toggle_pto_held = false;
    bool toggle_pto_speed_held = false; // flips the pto_speed channel between 540/1000 (main.gd owns the actual values, same pattern as the other tri-state toggles)
    bool toggle_park_lock_held = false;

    // Tractor demo: hand_throttle is a STATEFUL RAMP (0..1), not a tri-state
    // toggle - mirrors clutch_button_level_'s ramp below. Held up/down keys
    // increase/decrease it at kHandThrottleRampPerSecond; releasing both
    // holds the last value (no auto-return-to-zero, matching a real hand
    // throttle lever).
    bool hand_throttle_up_held = false;
    bool hand_throttle_down_held = false;
};

// Shaped output: exactly the named control channels PLAN.md P2/P4 I3 list
// (steer, throttle, brake, handbrake, clutch, shift up/down counters,
// camera, reset vehicle, debug toggle, ignition, starter, assist toggles).
// Consumed by main.gd, which forwards most of these 1:1 to
// PsSimulation.set_control (PLAN.md section 12.2's own bullet list).
struct ControlChannels {
    float steer = 0.0f;      // -1..1
    float throttle = 0.0f;   // 0..1
    float brake = 0.0f;      // 0..1
    float handbrake = 0.0f;  // 0..1
    float clutch = 0.0f;     // 0..1

    // Monotonic counters (PLAN.md 6.5.1: the TCU consumes DIFFERENCES
    // between ticks, never resets these to 0 itself) - +1 per rising edge
    // of shift_up_held/shift_down_held. Exact up to 2^24 presses as a
    // float once forwarded through World::set_control (PLAN.md 12.1).
    std::uint32_t shift_up_count = 0;
    std::uint32_t shift_down_count = 0;

    // Real edges now (rising edge of the corresponding *_held level) -
    // PLAN.md 12.2: "Camera, reset and debug become real edges."
    bool camera = false;
    bool reset_vehicle = false;
    bool debug_toggle = false; // PLAN.md M1: debug draw on/off

    float ignition = 1.0f; // toggled on ignition_toggle_held's rising edge; starts ON (Q-I3-4: spawn with the engine running)
    float starter = 0.0f;  // level pass-through of starter_held

    // Tri-state "no change / toggle" (PLAN.md 12.2): true only on the frame
    // the corresponding toggle button's rising edge fires. InputMapper does
    // NOT track the assist's actual on/off state - the caller (main.gd)
    // reads the current value via World::get_control("assist.x") and flips
    // it with set_control("assist.x", 1 - that) on a true here.
    bool toggle_auto_shift = false;
    bool toggle_auto_clutch = false;
    bool toggle_auto_blip = false;

    // Tractor demo (2026-09-28): same tri-state "no change / toggle"
    // contract as the assist toggles above - main.gd reads the current
    // World::get_control(name) value and flips it (toggle_pto_speed_held's
    // 540/1000 flip logic also lives in main.gd, since InputMapper below
    // never sees the sim's own channel values).
    bool toggle_awd = false;
    bool toggle_diff_lock = false;
    bool toggle_pto = false;
    bool toggle_pto_speed = false;
    bool toggle_park_lock = false;

    // Tractor demo: stateful ramp, 0..1 - forwarded straight to
    // World::set_control("hand_throttle", ...) every frame (not edge-
    // triggered, unlike the toggles above).
    float hand_throttle = 0.0f;
};

// Speed-sensitive steering assist for sticks/keyboards (owner test drive,
// 2026-09-24: "tire grip is complete and utter ass"). A stick maps linearly
// to the full max_wheel_angle (30 deg on car_sedan.json) at ANY speed, so at
// 60 km/h anything past ~30 % stick already puts the front tyres far beyond
// their ~4.5 deg peak slip angle: the front washes out and the car feels
// gripless although the tyre model reaches ~0.9 g (skidpad test). A
// steering wheel and a real driver never do that; every console racing
// game scales the pad's steering range with speed. The assist SCALES the
// steer channel (full stick = the limit, so the whole stick travel stays
// usable) to the angle a steady corner at the tyre's limit needs:
//   delta_max(v) = wheelbase * lat_accel_limit / v^2 + slip_margin * peak_slip_angle
// (kinematic Ackermann angle for the tightest radius v^2/a_lat, plus the
// front slip angle at the grip peak), as a fraction of max_wheel_angle,
// clamped to [min_fraction, 1]. Full lock stays available at parking speed.
struct SteerAssistConfig {
    bool enabled = true;
    float wheelbase_m = 0.0f;          // <= 0: unknown -> assist inactive
    float max_wheel_angle_rad = 0.0f;  // <= 0: unknown -> assist inactive
    float lat_accel_limit_mps2 = 9.0f; // ~0.9 g, the car's measured asphalt limit
    float peak_slip_angle_rad = 0.0785398f; // 4.5 deg (vault PHYS-036)
    float slip_margin = 1.3f;          // lets the driver reach a bit past the peak on purpose
    float min_fraction = 0.08f;
};

// Per-channel remappable configuration (deadzone/curve/invert). Only the
// analog channels need shaping config; digital actions are pass-through.
struct InputMapConfig {
    SteerAssistConfig steer_assist{};
    AxisConfig steer{};
    TriggerConfig throttle{};
    TriggerConfig brake{};
    TriggerConfig handbrake{};
    TriggerConfig clutch{};
};

class InputMapper {
public:
    explicit InputMapper(InputMapConfig config = {}) : config_(config) {}

    // Stateful (PLAN.md P4 I3 section 12.2): `dt` drives the clutch-button
    // release ramp; internal previous-frame state drives edge detection for
    // shift_up/shift_down/camera/reset_vehicle/debug_toggle/ignition_toggle/
    // auto_*_toggle. No longer `const` - each call mutates the mapper.
    [[nodiscard]] ControlChannels process(const RawInput& raw, float dt);

    [[nodiscard]] const InputMapConfig& config() const { return config_; }
    void set_config(const InputMapConfig& config) { config_ = config; }

    // Exposed for input_map.h (PLAN.md: "PsInputMap keeps uint32 counters")
    // and for tests; mirrors the last process() call's ControlChannels
    // fields of the same name.
    [[nodiscard]] std::uint32_t shift_up_count() const { return shift_up_count_; }
    [[nodiscard]] std::uint32_t shift_down_count() const { return shift_down_count_; }
    // Fraction of max_wheel_angle the last process() call allowed (1 = no
    // assist limit); for the HUD and tests.
    [[nodiscard]] float last_steer_limit() const { return last_steer_limit_; }

private:
    InputMapConfig config_;
    float last_steer_limit_ = 1.0f;

    bool prev_shift_up_ = false;
    bool prev_shift_down_ = false;
    bool prev_camera_ = false;
    bool prev_reset_vehicle_ = false;
    bool prev_debug_toggle_ = false;
    bool prev_ignition_toggle_ = false;
    bool prev_auto_shift_toggle_ = false;
    bool prev_auto_clutch_toggle_ = false;
    bool prev_auto_blip_toggle_ = false;

    // Tractor demo (2026-09-28) tri-state toggle edge-detection state -
    // same pattern as the assist toggles above.
    bool prev_toggle_awd_ = false;
    bool prev_toggle_diff_lock_ = false;
    bool prev_toggle_pto_ = false;
    bool prev_toggle_pto_speed_ = false;
    bool prev_toggle_park_lock_ = false;

    std::uint32_t shift_up_count_ = 0;
    std::uint32_t shift_down_count_ = 0;
    float ignition_ = 1.0f;

    // Digital clutch-button smoothing state (PLAN.md 12.2), independent of
    // the analog clutch_trigger path - see RawInput::clutch_button_held.
    float clutch_button_level_ = 0.0f;

    // Tractor demo: hand-throttle ramp state (0..1), see
    // RawInput::hand_throttle_up_held/down_held and
    // ControlChannels::hand_throttle.
    float hand_throttle_ = 0.0f;
};

} // namespace ps_godot
