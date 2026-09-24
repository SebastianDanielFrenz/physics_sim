// adapters/godot/tests/test_input_map.cpp — synthetic-input test for the
// deadzone/curve/trigger mapping (PLAN.md P2 acceptance: "the input mapping
// ... is tested with synthetic input events") plus PHYS-033 P4 I3 section
// 12.2's stateful edge-detection/counter/toggle/clutch-ramp behaviour.
//
// Plain main(), no test framework (PLAN.md P2 asks for "a plain C++ test in
// the adapter's own test target, not in tests/" - that directory is owned
// by the other agent working in this repo right now). Prints one line per
// check and returns the failure count as the process exit code, so CTest
// (adapters/godot/CMakeLists.txt registers this via add_test) and the
// headless smoke-test script can both tell pass from fail from the exit
// code alone.

#include "../src/input_map_core.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what);
    }
}

bool close(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

// A frame at 60 fps, the same rate PLAN.md 12.2's clutch-ramp test bullet
// asks for ("0.35 s to 0 at dt = 1/60").
constexpr float kDt60 = 1.0f / 60.0f;

} // namespace

using namespace ps_godot;

int main() {
    // --- Bidirectional axis (steer) -----------------------------------
    {
        AxisConfig cfg; // deadzone 0.08, curve 1.0, no invert
        check(shape_bidirectional_axis(0.0f, cfg) == 0.0f, "steer: centred stick -> 0");
        check(shape_bidirectional_axis(0.05f, cfg) == 0.0f, "steer: within deadzone -> 0");
        check(shape_bidirectional_axis(-0.05f, cfg) == 0.0f, "steer: within deadzone (negative) -> 0");
        check(close(shape_bidirectional_axis(1.0f, cfg), 1.0f), "steer: full right -> 1.0 exactly (post-deadzone rescale)");
        check(close(shape_bidirectional_axis(-1.0f, cfg), -1.0f), "steer: full left -> -1.0 exactly");
        // Just past the deadzone boundary should be just past zero, not a
        // jump to some large value (continuity of the rescale).
        const float just_past = shape_bidirectional_axis(0.09f, cfg);
        check(just_past > 0.0f && just_past < 0.05f, "steer: just past deadzone is small, not a jump");
    }
    {
        AxisConfig cfg;
        cfg.invert = true;
        check(close(shape_bidirectional_axis(1.0f, cfg), -1.0f), "steer: invert flips full-right to -1");
        check(close(shape_bidirectional_axis(-1.0f, cfg), 1.0f), "steer: invert flips full-left to +1");
    }
    {
        // Curve > 1: gentler near centre. Half-deflection input should map
        // to a smaller output than the linear case.
        AxisConfig linear;
        AxisConfig curved;
        curved.curve_exponent = 2.0f;
        const float lin = shape_bidirectional_axis(0.5f, linear);
        const float crv = shape_bidirectional_axis(0.5f, curved);
        check(crv < lin, "steer: exponent 2.0 is gentler than linear at half deflection");
        check(crv > 0.0f, "steer: curved output still positive (sign preserved)");
    }
    check(shape_bidirectional_axis(2.0f, AxisConfig{}) <= 1.0f, "steer: out-of-range input is clamped, never exceeds 1.0");

    // --- Unidirectional axis (throttle/brake/clutch/handbrake) --------
    {
        TriggerConfig cfg; // deadzone 0.02
        check(shape_unidirectional_axis(0.0f, cfg) == 0.0f, "trigger: released -> 0");
        check(shape_unidirectional_axis(0.01f, cfg) == 0.0f, "trigger: within deadzone -> 0");
        check(close(shape_unidirectional_axis(1.0f, cfg), 1.0f), "trigger: fully pressed -> 1.0");
        check(shape_unidirectional_axis(-0.5f, cfg) == 0.0f, "trigger: negative input clamped to 0 first");
    }

    // --- Full InputMapper::process, synthetic RawInput frames ---------
    {
        InputMapper mapper; // defaults
        RawInput raw;
        raw.steer_axis = 0.5f;
        raw.throttle_trigger = 0.8f;
        raw.brake_trigger = 0.0f;
        raw.handbrake_trigger = 1.0f;
        raw.clutch_trigger = 0.3f;
        raw.shift_up_held = true;
        const ControlChannels ch = mapper.process(raw, kDt60);
        // raw.steer_axis follows the hardware convention (+ = right, see
        // input_map_core.h); the sim's ISO 8855 "steer" channel is + = left
        // (PLAN.md D4, vehicle_instance.cpp's Ackermann code), so a positive
        // raw axis must come out negative here.
        check(ch.steer < 0.0f && ch.steer > -1.0f, "process: steer channel in expected open range, sign-flipped from raw (+raw=right -> -channel)");
        check(ch.throttle > 0.0f, "process: throttle channel non-zero for pressed trigger");
        check(ch.brake == 0.0f, "process: brake channel zero for released trigger");
        check(close(ch.handbrake, 1.0f), "process: handbrake fully applied");
        check(ch.shift_up_count == 1, "process: one held-this-frame shift_up frame produces one rising edge -> count 1");
        check(ch.shift_down_count == 0, "process: shift_down not held this frame -> count 0");
        check(ch.camera == false && ch.reset_vehicle == false, "process: other digital edges default false");
    }
    {
        // Remapping: change the mapper's config at runtime (deadzone/curve
        // are configurable per PLAN.md P2) and confirm the new config takes
        // effect immediately.
        InputMapper mapper;
        InputMapConfig cfg = mapper.config();
        cfg.steer.deadzone = 0.5f;
        mapper.set_config(cfg);
        RawInput raw;
        raw.steer_axis = 0.3f; // now inside the widened deadzone
        check(mapper.process(raw, kDt60).steer == 0.0f, "remap: widened deadzone swallows a value that used to pass through");
    }

    // --- Steer sign convention (bug regression: keyboard/gamepad steering
    // reversed - PHYS report, M1) -----------------------------------------
    // RawInput::steer_axis is hardware convention (+ = right: Godot's
    // JOY_AXIS_LEFT_X for the gamepad, get_axis("ps_steer_left",
    // "ps_steer_right") for the keyboard - both agree, physical_keycode 65
    // 'A' = left = -1, 68 'D' = right = +1). The sim's "steer" control
    // channel is ISO 8855, + = LEFT. InputMapper::process must convert
    // between the two for BOTH input sources identically, so a synthetic
    // frame standing in for either source must come out steering the
    // correct way.
    {
        InputMapper mapper;
        RawInput raw;
        raw.steer_axis = 1.0f; // stick/D-key fully right (hardware convention)
        const ControlChannels ch = mapper.process(raw, kDt60);
        check(close(ch.steer, -1.0f), "steer sign: raw full-right (hardware) -> channel -1 (ISO right)");
    }
    {
        InputMapper mapper;
        RawInput raw;
        raw.steer_axis = -1.0f; // stick/A-key fully left (hardware convention)
        const ControlChannels ch = mapper.process(raw, kDt60);
        check(close(ch.steer, 1.0f), "steer sign: raw full-left (hardware) -> channel +1 (ISO left)");
    }

    // --- PHYS-033 P4 I3 section 12.2: shift edge detection / counters -----
    {
        // Holding RB (shift_up_held) for 10 frames gives exactly 1 count -
        // the rising edge fires once, then the level stays "already seen".
        InputMapper mapper;
        RawInput raw;
        raw.shift_up_held = true;
        ControlChannels ch{};
        for (int i = 0; i < 10; ++i) {
            ch = mapper.process(raw, kDt60);
        }
        check(ch.shift_up_count == 1, "shift_up: holding for 10 frames gives exactly 1 count");
        check(mapper.shift_up_count() == 1, "shift_up: InputMapper::shift_up_count() mirrors the last ControlChannels value");
    }
    {
        // 3 press/release cycles within 3 frames give 3 - one rising edge
        // per cycle (press frame, release frame, alternating).
        InputMapper mapper;
        RawInput raw;
        ControlChannels ch{};
        for (int i = 0; i < 3; ++i) {
            raw.shift_down_held = true;
            ch = mapper.process(raw, kDt60);
            raw.shift_down_held = false;
            ch = mapper.process(raw, kDt60);
        }
        check(ch.shift_down_count == 3, "shift_down: 3 press/release cycles give exactly 3 counts");
    }
    {
        // Counters are monotonic and never consumed/reset by the mapper
        // itself (PLAN.md 6.5.1: the TCU consumes differences) - two
        // separate single-frame presses, with a released frame between,
        // must accumulate rather than restart.
        InputMapper mapper;
        RawInput raw;
        raw.shift_up_held = true;
        (void)mapper.process(raw, kDt60);
        raw.shift_up_held = false;
        (void)mapper.process(raw, kDt60);
        raw.shift_up_held = true;
        const ControlChannels ch = mapper.process(raw, kDt60);
        check(ch.shift_up_count == 2, "shift_up: counter is monotonic across separate presses (2 presses -> count 2)");
    }

    // --- Camera/reset/debug become real edges (PLAN.md 12.2) --------------
    {
        InputMapper mapper;
        RawInput raw;
        raw.camera_held = true;
        raw.reset_vehicle_held = true;
        raw.debug_toggle_held = true;
        const ControlChannels first = mapper.process(raw, kDt60);
        check(first.camera && first.reset_vehicle && first.debug_toggle, "camera/reset/debug: rising edge fires true on the first held frame");
        const ControlChannels held = mapper.process(raw, kDt60); // still held next frame - must NOT re-fire
        check(!held.camera && !held.reset_vehicle && !held.debug_toggle, "camera/reset/debug: still held next frame -> no re-fire (edge, not level)");
    }

    // --- Ignition toggle (PLAN.md 12.2: "toggled on edges, initial 1") ----
    {
        InputMapper mapper;
        RawInput raw;
        const ControlChannels initial = mapper.process(raw, kDt60);
        check(initial.ignition > 0.5f, "ignition: starts ON (Q-I3-4: spawn with the engine running)");

        raw.ignition_toggle_held = true;
        const ControlChannels toggled_off = mapper.process(raw, kDt60);
        check(toggled_off.ignition < 0.5f, "ignition: rising edge toggles it OFF");

        const ControlChannels still_held = mapper.process(raw, kDt60); // held, no new edge
        check(still_held.ignition < 0.5f, "ignition: holding the toggle button does not re-toggle");

        raw.ignition_toggle_held = false;
        (void)mapper.process(raw, kDt60); // release
        raw.ignition_toggle_held = true;
        const ControlChannels toggled_on = mapper.process(raw, kDt60);
        check(toggled_on.ignition > 0.5f, "ignition: a second rising edge toggles it back ON");
    }

    // --- Starter is a level, not an edge -----------------------------------
    {
        InputMapper mapper;
        RawInput raw;
        raw.starter_held = true;
        const ControlChannels a = mapper.process(raw, kDt60);
        const ControlChannels b = mapper.process(raw, kDt60);
        check(a.starter > 0.5f && b.starter > 0.5f, "starter: level pass-through stays 1 for as long as it's held");
        raw.starter_held = false;
        const ControlChannels c = mapper.process(raw, kDt60);
        check(c.starter < 0.5f, "starter: drops to 0 the frame it's released");
    }

    // --- Assist toggles: tri-state "no change / toggle" --------------------
    {
        InputMapper mapper;
        RawInput raw;
        raw.auto_shift_toggle_held = true;
        raw.auto_clutch_toggle_held = true;
        raw.auto_blip_toggle_held = true;
        const ControlChannels first = mapper.process(raw, kDt60);
        check(first.toggle_auto_shift && first.toggle_auto_clutch && first.toggle_auto_blip, "assist toggles: rising edge requests a toggle");
        const ControlChannels held = mapper.process(raw, kDt60);
        check(!held.toggle_auto_shift && !held.toggle_auto_clutch && !held.toggle_auto_blip, "assist toggles: holding does not repeat the request");
    }

    // --- Clutch-button smoothing (PLAN.md 12.2: 0.35 s to 0 at dt = 1/60) --
    {
        InputMapper mapper;
        RawInput raw;
        raw.clutch_button_held = true;
        const ControlChannels pressed = mapper.process(raw, kDt60);
        check(close(pressed.clutch, 1.0f), "clutch button: held -> clutch 1.0");

        raw.clutch_button_held = false;
        // 0.35 s / (1/60 s per frame) = 21 frames to fully decay.
        const int decay_frames = static_cast<int>(std::lround(0.35 / kDt60));
        ControlChannels last{};
        for (int i = 0; i < decay_frames; ++i) {
            last = mapper.process(raw, kDt60);
        }
        check(close(last.clutch, 0.0f, 1e-3f), "clutch button: decays to ~0 after 0.35 s at dt=1/60");

        // Mid-ramp: somewhere strictly between fully-pressed and released,
        // not a snap to either extreme.
        InputMapper mapper2;
        RawInput raw2;
        raw2.clutch_button_held = true;
        (void)mapper2.process(raw2, kDt60);
        raw2.clutch_button_held = false;
        const ControlChannels mid = mapper2.process(raw2, kDt60);
        check(mid.clutch > 0.0f && mid.clutch < 1.0f, "clutch button: one frame into the release ramp is strictly between 0 and 1");
    }
    {
        // Analog pedal still passes through unaffected when the digital
        // button is never used (clutch_button_held always false) - PLAN.md
        // 12.2: "analog input passes through as before".
        InputMapper mapper;
        RawInput raw;
        raw.clutch_trigger = 0.6f;
        const ControlChannels ch = mapper.process(raw, kDt60);
        check(close(ch.clutch, shape_unidirectional_axis(0.6f, TriggerConfig{})), "clutch: analog-only input passes through exactly as before");
    }

    if (g_failures == 0) {
        std::printf("test_input_map: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_input_map: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
