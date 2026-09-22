// adapters/godot/tests/test_input_map.cpp — synthetic-input test for the
// deadzone/curve/trigger mapping (PLAN.md P2 acceptance: "the input mapping
// ... is tested with synthetic input events").
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
        raw.shift_up_pressed = true;
        const ControlChannels ch = mapper.process(raw);
        check(ch.steer > 0.0f && ch.steer < 1.0f, "process: steer channel in expected open range");
        check(ch.throttle > 0.0f, "process: throttle channel non-zero for pressed trigger");
        check(ch.brake == 0.0f, "process: brake channel zero for released trigger");
        check(close(ch.handbrake, 1.0f), "process: handbrake fully applied");
        check(ch.shift_up == true, "process: digital shift_up passes through unshaped");
        check(ch.shift_down == false, "process: shift_down not pressed this frame");
        check(ch.camera == false && ch.reset_vehicle == false, "process: other digital actions default false");
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
        check(mapper.process(raw).steer == 0.0f, "remap: widened deadzone swallows a value that used to pass through");
    }

    if (g_failures == 0) {
        std::printf("test_input_map: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_input_map: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
