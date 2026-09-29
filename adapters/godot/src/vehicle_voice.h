// adapters/godot/src/vehicle_voice.h - ps_audio A11 ("engine sound in
// Godot, minimal"): the adapter's own engine voice for one vehicle. Owns a
// ps_audio::Voice (the live 0D cycle + pipe-network render model) plus a
// ps_audio::Runner (its own render thread, decoupled from both the sim
// thread and Godot's frame rate) and feeds it every sim tick from that
// vehicle's own published PowertrainSnapshot::engines[0] via
// tools/common/engine_sound_glue.h's shared mapping.
//
// Coordinator scope change (2026-09-29, owner ruling "can I drive the same
// engine that drives the audio?"): there is no separate "sound engine" to
// load - try_create() takes the vehicle's ALREADY-LOADED VehicleDesc (as
// spawn_vehicle stores in PsSimulation::vehicle_desc_by_name_) and looks at
// its OWN engine component. A vehicle whose engine is not a "kind":
// "simulated" one with a "cycle" block (e.g. car_sedan.json's torque-map
// engine, car_sedan_sim.json's map-only engine) has no live cycle model to
// render at all - try_create() returns nullptr and the caller logs once,
// no error, no fallback "silent placeholder" engine.
//
// Godot-free (only ps:: and ps_audio:: types) so it could in principle be
// reused by a non-Godot frontend, mirroring fall_detector.h's own seam.
#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps_audio/runner.h"
#include "ps_audio/voice.h"

#include <memory>

namespace ps_godot {

class VehicleVoice {
public:
    // Returns nullptr (no throw) if `desc`'s powertrain has no "kind":
    // "simulated" engine component with a "cycle" block, or that engine has
    // no fuel - both are ordinary, expected cases (most vehicles), not
    // errors. `out_reason` is set to a short one-line explanation either
    // way (useful for the caller's one-time log line).
    static std::unique_ptr<VehicleVoice> try_create(const ps::vehicle::VehicleDesc& desc, std::string& out_reason,
                                                      const ps_audio::VoiceSettings& settings = {});

    ~VehicleVoice();
    VehicleVoice(const VehicleVoice&) = delete;
    VehicleVoice& operator=(const VehicleVoice&) = delete;

    // Starts/stops the Runner's own render thread. Safe to call start()
    // more than once (Runner::start() is a no-op if already running).
    void start() { runner_.start(); }
    void stop() { runner_.stop(); }

    // Sim-thread call, once per tick, per vehicle: pushes this tick's sound
    // state into the voice's lock-free input queue (SPSC: this is the ONE
    // producer). A full queue (the render thread has fallen far behind)
    // drops the sample - Voice::push_input's own contract, not handled
    // specially here.
    void push_tick(const ps::drivetrain::EngineSoundState& ess, ps::real t_sim_s);

    void set_paused(bool paused) { voice_.set_paused(paused); }

    [[nodiscard]] ps_audio::Runner& runner() { return runner_; }
    [[nodiscard]] const ps_audio::Runner& runner() const { return runner_; }
    [[nodiscard]] int channel_count() const { return runner_.channel_count(); }
    [[nodiscard]] double sample_rate_hz() const { return static_cast<double>(voice_.sample_rate_hz()); }

private:
    VehicleVoice(const ps::drivetrain::CycleModelDesc& cycle, ps::real lhv_j_kg, ps::real afr_stoich,
                 const ps_audio::VoiceSettings& settings);

    ps_audio::Voice voice_;
    ps_audio::Runner runner_; // holds a non-owning Voice* into voice_ above - member order matters (Voice first)
};

} // namespace ps_godot
