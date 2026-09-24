// adapters/godot/src/input_map.h — Godot-facing input node (PLAN.md P2:
// Xbox One/Series controller via Godot's joypad API, plus keyboard; steer,
// throttle, brake, handbrake, shift up/down, clutch, camera, reset vehicle;
// remappable; multi-stage gamepad rumble driven from the chassis' own
// telemetry).
//
// All deadzone/curve/trigger-normalisation math lives in input_map_core.h;
// all rumble shaping lives in haptics_core.h (both Godot-free, unit-tested
// headless by adapters/godot/tests/test_input_map.cpp and test_haptics.cpp
// respectively). This class only gathers raw joypad/keyboard state and (for
// rumble) the running vehicle's own race-free telemetry each frame, then
// calls InputMapper::process / HapticsCore::update - it does no shaping
// math of its own.
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
//
// Rumble wiring (owner request, PLAN.md P2's old "rumble hook that does
// nothing yet"): `simulation_path`/`vehicle_name` (set by main.gd, like
// hud.gd's own NodePath properties) let this node reach the running
// vehicle's chassis-acceleration/wheel/powertrain telemetry through
// PsSimulation's existing race-free getters - see haptics_core.h's own
// header comment for the six sources combined into the two motors.
// start_joy_vibration is called every frame with a short self-stopping
// duration; stop_joy_vibration is called once on disable/pause/exit-tree so
// a paused/quit game does not leave the controller buzzing.

#pragma once

#include "haptics_core.h"
#include "input_map_core.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/node_path.hpp>

#include <cstdint>
#include <string>

namespace godot {
class InputEvent;
}

namespace ps_godot {

class PsSimulation;

class PsInputMap : public godot::Node {
    GDCLASS(PsInputMap, godot::Node)

public:
    PsInputMap();

    void _process(double delta) override;
    void _exit_tree() override;

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

    // Manual/direct rumble call (unchanged signature from the old PLAN.md
    // P2 stub, now actually implemented) - a thin pass-through to
    // godot::Input::start_joy_vibration on joy_device_, for a caller that
    // wants to drive the motors itself instead of (or in addition to) the
    // automatic per-frame haptics_ path below.
    void rumble(float low_frequency, float high_frequency, float duration_s);

    // --- Gamepad rumble (haptics_core.h) --------------------------------
    // `simulation_path` + `vehicle_name` let this node reach the running
    // vehicle's telemetry through PsSimulation's race-free getters (set by
    // main.gd, same NodePath-export pattern as hud.gd's own
    // simulation_path/vehicle_name). Left unset (vehicle_name == ""), no
    // rumble is ever driven - safe default for any scene that doesn't wire
    // this up.
    void set_simulation_path(const godot::NodePath& path) { simulation_path_ = path; simulation_ = nullptr; }
    [[nodiscard]] godot::NodePath get_simulation_path() const { return simulation_path_; }
    void set_vehicle_name(const godot::String& name) { vehicle_name_ = name; steer_assist_configured_ = false; }
    [[nodiscard]] godot::String get_vehicle_name() const { return vehicle_name_; }

    // Enable toggle (owner request: "add an enable toggle on a free key") -
    // bound so hud.gd can show "Rumble: on/off" the same way it reads
    // get_steer()/get_throttle() etc. The H key itself
    // (demo/project.godot's ps_rumble_toggle action) is handled inside
    // _process via Godot's own is_action_just_pressed - a UI convenience,
    // not a named ControlChannels channel (input_map_core.h never sees it).
    [[nodiscard]] bool get_rumble_enabled() const { return rumble_enabled_; }
    void set_rumble_enabled(bool v);

protected:
    static void _bind_methods();

private:
    [[nodiscard]] RawInput gather_raw_input() const;
    void update_haptics(float dt);

    InputMapper mapper_;
    ControlChannels channels_;
    int joy_device_ = 0;

    godot::NodePath simulation_path_;
    godot::String vehicle_name_;
    bool steer_assist_configured_ = false;
    // Cached PsSimulation* - resolved lazily in update_haptics (godot-cpp
    // Node::get_node_or_null needs the node to already be inside the tree,
    // which is not guaranteed yet in _ready()/the constructor); re-resolved
    // whenever it comes back null (path unset yet, or the sim node hasn't
    // been added as a sibling yet at this point in main.gd's own _ready
    // order - see input_map.cpp's own comment).
    PsSimulation* simulation_ = nullptr;
    HapticsCore haptics_;
    bool rumble_enabled_ = true;
    bool prev_rumble_toggle_held_ = false;
    bool rumble_was_active_ = false; // whether the last call actually started vibration (so stop_joy_vibration is called at most once per "stop" transition, not every idle frame)
};

} // namespace ps_godot
