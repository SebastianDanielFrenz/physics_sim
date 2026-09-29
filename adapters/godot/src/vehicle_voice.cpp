#include "vehicle_voice.h"

#include "engine_sound_glue.h"

#include <variant>

namespace ps_godot {

std::unique_ptr<VehicleVoice> VehicleVoice::try_create(const ps::vehicle::VehicleDesc& desc, std::string& out_reason,
                                                         const ps_audio::VoiceSettings& settings, ps::real dx_scale) {
    const ps::drivetrain::SimulatedEngineDesc* sim = nullptr;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            sim = s;
            break; // a vehicle has at most one engine component
        }
    }
    if (sim == nullptr) {
        out_reason = "no \"kind\": \"simulated\" engine component (a torque-map engine has no live cycle model)";
        return nullptr;
    }
    if (!sim->cycle.has_value()) {
        out_reason = "the simulated engine has no \"cycle\" block (maps-only, no live pipe-network model to render)";
        return nullptr;
    }
    if (sim->fuels.empty()) {
        out_reason = "the simulated engine has no fuel";
        return nullptr;
    }
    out_reason = "ok";

    // ps_audio A11 live dx (see vehicle_voice.h's header comment for the
    // measurement): a MUTABLE COPY of the engine's own cycle desc, scaled
    // here only - the loaded VehicleDesc (physics, map generation, the
    // offline WAV tools) is never touched.
    ps::drivetrain::CycleModelDesc live_cycle = *sim->cycle;
    live_cycle.resolution.audio_dx_intake_m *= dx_scale;
    live_cycle.resolution.audio_dx_exhaust_m *= dx_scale;

    // VehicleVoice's private ctor is only reachable through this factory,
    // so std::make_unique cannot be used (it needs a public ctor) -
    // new+unique_ptr is the ordinary workaround.
    return std::unique_ptr<VehicleVoice>(
        new VehicleVoice(live_cycle, sim->fuels[0].lhv_j_kg, sim->fuels[0].afr_stoich, settings));
}

VehicleVoice::VehicleVoice(const ps::drivetrain::CycleModelDesc& cycle, ps::real lhv_j_kg, ps::real afr_stoich,
                            const ps_audio::VoiceSettings& settings)
    : voice_(cycle, lhv_j_kg, afr_stoich, settings),
      // 4 x 256 frames = 21 ms of ring (default 8 = 43 ms): the ring stays
      // full, so its whole length is latency between an rpm change and the
      // speaker (owner 2026-09-29). The render thread refills a 256-frame
      // block in ~3.5 ms (0.66 core), far inside the ring.
      runner_(std::vector<ps_audio::Voice*>{&voice_}, ps_audio::RunnerSettings{256, 4, 500}) {}

VehicleVoice::~VehicleVoice() { runner_.stop(); }

void VehicleVoice::push_tick(const ps::drivetrain::EngineSoundState& ess, ps::real t_sim_s) {
    voice_.push_input(ps_tools::make_engine_input(ess, t_sim_s));
}

} // namespace ps_godot
