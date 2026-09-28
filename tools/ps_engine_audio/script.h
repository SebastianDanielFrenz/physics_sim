// script.h - ps_audio A10: tools/ps_engine_audio's timed-event script format
// ("physics_sim.engine_audio_script/1"): a list of {t, <control values>}
// events applied to an EngineRig (and, if a dyno is configured, its torque
// scale) as sim time reaches each event's `t`, held at their last value
// afterwards - the same convention ps::World::set_control and
// ps::EngineRig's own setters already use. See data/scripts/engine_audio/
// free_rev.json and stall.json for shipped examples.
#pragma once

#include "ps/types.h"

#include <optional>
#include <string>
#include <vector>

namespace ps_tools {

struct ScriptEvent {
    ps::real t = 0.0;
    std::optional<ps::real> starter;      // EngineRig::set_starter
    std::optional<ps::real> ignition;     // EngineRig::set_ignition
    std::optional<ps::real> throttle;     // EngineRig::set_throttle
    std::optional<ps::real> dyno_scale;   // EngineRig::set_dyno_torque_scale (only meaningful with --dyno-brake-nm)
};

struct EngineAudioScript {
    std::string source;
    std::vector<ScriptEvent> events; // ascending t, t[0] == 0 not required
    // <= 0: derive from the last event's t plus a fixed tail (main.cpp's
    // own constant) so the engine's last state is actually heard.
    ps::real duration_s = -1.0;
};

// Throws std::runtime_error on a malformed file: bad/missing "format", an
// event missing "t", a non-ascending "t" sequence, an event with none of
// starter/ignition/throttle/dyno_scale set, or an unknown top-level/event
// key.
[[nodiscard]] EngineAudioScript load_engine_audio_script(const std::string& path);

} // namespace ps_tools
