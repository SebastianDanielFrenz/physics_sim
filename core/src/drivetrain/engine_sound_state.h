// drivetrain/engine_sound_state.h - ps_audio A10 (out/audio_plan.md section
// 6): the EngineSoundState fill, factored out of
// VehicleInstance::powertrain_state() (core/src/vehicle/vehicle_instance.cpp)
// so a car's snapshot and a wheel-less EngineRig publish byte-identical sound
// state from the same code, not two hand-kept copies (vault PHYS-054's A5
// finding). INTERNAL (not part of ps_core's public API) - both callers are
// themselves internal TUs (vehicle_instance.cpp, engine_rig.cpp).
#pragma once

#include "ps/drivetrain/powertrain_state.h" // EngineSoundState

namespace ps::drivetrain {

class Powertrain; // core/src/drivetrain/powertrain.h

// Returns a default-constructed EngineSoundState{} if !pt.has_engine();
// otherwise fills every field from pt's cached last_engine_result()/
// engine_omega()/engine()/throttle_command() - this substep's already-
// computed engine state, no new computation. Every EngineSoundState field is
// explicitly NOT hashed (see powertrain_state.h), so this call never touches
// determinism-sensitive state.
[[nodiscard]] EngineSoundState fill_engine_sound_state(const Powertrain& pt);

} // namespace ps::drivetrain
