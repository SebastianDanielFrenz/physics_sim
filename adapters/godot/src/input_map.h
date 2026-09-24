// adapters/godot/src/input_map.h — Godot-facing input node (PLAN.md P2:
// Xbox One/Series controller via Godot's joypad API, plus keyboard; steer,
// throttle, brake, handbrake, shift up/down, clutch, camera, reset vehicle;
// remappable; a rumble hook that does nothing yet).
//
// All deadzone/curve/trigger-normalisation math lives in input_map_core.h
// (Godot-free, unit-tested by adapters/godot/tests/test_input_map.cpp).
// This class only gathers raw joypad/keyboard state each frame (via
// godot::Input) and a small remap table, then calls InputMapper::process -
// it does no shaping math of its own.
//
// Remapping: every action below is also a normal Godot InputMap action
// (see demo/project.godot's [input] section: ps_shift_up, ps_shift_down,
// ps_camera, ps_reset_vehicle, ps_clutch_button, ps_handbrake_button), so a
// user can rebind the digital actions with Godot's own Input Map UI/project
// settings without touching this class. The analog channels (steer/
// throttle/brake) are read directly from the joypad axes/triggers (raw,
// bypassing Godot's own per-action deadzone) specifically so this class's
// own deadzone/curve config is the one that applies - joy_device selects
// which physical controller to read.

#pragma once

#include "input_map_core.h"

#include <godot_cpp/classes/node.hpp>

#include <cstdint>

namespace godot {
class InputEvent;
}

namespace ps_godot {

class PsInputMap : public godot::Node {
    GDCLASS(PsInputMap, godot::Node)

public:
    PsInputMap();

    void _process(double delta) override;

    // Shaped channel getters (exposed to GDScript/the demo HUD).
    [[nodiscard]] float get_steer() const { return channels_.steer; }
    [[nodiscard]] float get_throttle() const { return channels_.throttle; }
    [[nodiscard]] float get_brake() const { return channels_.brake; }
    [[nodiscard]] float get_handbrake() const { return channels_.handbrake; }
    [[nodiscard]] float get_clutch() const { return channels_.clutch; }
    // PHYS-033 P4 I3 section 12.2: shift_up/shift_down are now monotonic
    // counters (edge-detected inside InputMapper, PLAN.md 6.5.1's own
    // "consumed as differences" contract), not per-frame booleans.
    [[nodiscard]] std::int64_t get_shift_up_count() const { return static_cast<std::int64_t>(channels_.shift_up_count); }
    [[nodiscard]] std::int64_t get_shift_down_count() const { return static_cast<std::int64_t>(channels_.shift_down_count); }
    [[nodiscard]] bool get_camera() const { return channels_.camera; }
    [[nodiscard]] bool get_reset_vehicle() const { return channels_.reset_vehicle; }
    [[nodiscard]] bool get_debug_toggle() const { return channels_.debug_toggle; }
    [[nodiscard]] float get_ignition() const { return channels_.ignition; }
    [[nodiscard]] float get_starter() const { return channels_.starter; }
    [[nodiscard]] bool get_toggle_auto_shift() const { return channels_.toggle_auto_shift; }
    [[nodiscard]] bool get_toggle_auto_clutch() const { return channels_.toggle_auto_clutch; }
    [[nodiscard]] bool get_toggle_auto_blip() const { return channels_.toggle_auto_blip; }

    // Remap config (deadzone/curve/invert), PLAN.md P2 "configurable".
    void set_steer_deadzone(float v);
    [[nodiscard]] float get_steer_deadzone() const { return mapper_.config().steer.deadzone; }
    void set_steer_curve(float v);
    [[nodiscard]] float get_steer_curve() const { return mapper_.config().steer.curve_exponent; }
    void set_steer_invert(bool v);
    [[nodiscard]] bool get_steer_invert() const { return mapper_.config().steer.invert; }
    void set_trigger_deadzone(float v); // applies to throttle/brake/handbrake/clutch alike
    [[nodiscard]] float get_trigger_deadzone() const { return mapper_.config().throttle.deadzone; }

    void set_joy_device(int device) { joy_device_ = device; }
    [[nodiscard]] int get_joy_device() const { return joy_device_; }

    // Rumble hook (PLAN.md P2: "does nothing yet"; will be fed from tyre
    // slip/ABS once P3/P4 exist). low/high in 0..1, duration in seconds.
    void rumble(float low_frequency, float high_frequency, float duration_s);

protected:
    static void _bind_methods();

private:
    [[nodiscard]] RawInput gather_raw_input() const;

    InputMapper mapper_;
    ControlChannels channels_;
    int joy_device_ = 0;
};

} // namespace ps_godot
