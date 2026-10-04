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
//
// Live dx (2026-09-29 measurement, tests/bench/bench_audio_voice's audio-
// grade I4 fixture, RelWithDebInfo release build, this machine, 3000 rpm):
// at the engine file's OWN default dx (intake 17 mm / exhaust 32 mm,
// ps/drivetrain/engine_cycle_desc.h's ResolutionCycleDesc defaults) one
// voice costs ~20.9 us/sample against the 48 kHz budget of 20.83 us -
// 1.00 of one core, i.e. NO margin at all (any scheduling jitter on that
// thread underruns). Doubling dx (34/64 mm) costs ~13.8 us/sample = 0.66
// of one core (34% margin); 1.5x (25.5/48 mm) costs ~16.3 us = 0.78 core
// (22% margin). kLiveDxScale below picks the doubled (2x) grid as the
// live default for comfortable real-time margin pre-A9 (SIMD) - see
// CLAUDE.md's ps_audio A7/A8b section ("live play needs A9"). This is a
// one-line multiplier, not a data-file edit (car_sedan_pipes.json/
// demo_i4_pipes.json keep their own dx for the OFFLINE map generator and
// tools/ps_engine_audio's WAV renders, which have no real-time budget).
#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps_audio/runner.h"
#include "ps_audio/native_output.h"
#include "ps_audio/voice.h"

#include <cstdint>
#include <cstdio>
#include <memory>

namespace ps_godot {

class VehicleVoice {
public:
    // Live-dx scale factor applied to the engine's own audio_dx_intake_m/
    // audio_dx_exhaust_m before building the Voice - see the header
    // comment above for the measurement behind this default.
    static constexpr ps::real kLiveDxScale = 2.0;

    // Returns nullptr (no throw) if `desc`'s powertrain has no "kind":
    // "simulated" engine component with a "cycle" block, or that engine has
    // no fuel - both are ordinary, expected cases (most vehicles), not
    // errors. `out_reason` is set to a short one-line explanation either
    // way (useful for the caller's one-time log line). `dx_scale` (default
    // kLiveDxScale) multiplies the engine file's own audio_dx_intake_m/
    // audio_dx_exhaust_m for this live Voice only - the loaded desc (and
    // the offline map generator / ps_engine_audio's own renders) are
    // untouched.
    static std::unique_ptr<VehicleVoice> try_create(const ps::vehicle::VehicleDesc& desc, std::string& out_reason,
                                                      const ps_audio::VoiceSettings& settings = {},
                                                      ps::real dx_scale = kLiveDxScale);

    ~VehicleVoice();
    VehicleVoice(const VehicleVoice&) = delete;
    VehicleVoice& operator=(const VehicleVoice&) = delete;

    // Starts/stops the Runner's own render thread. Safe to call start()
    // more than once (Runner::start() is a no-op if already running).
    void set_audio_bank(ps_audio::AudioBankPtr bank) { voice_.set_audio_bank(std::move(bank)); }
    void start() {
        if(!output_->connected()) output_=std::make_shared<ps_audio::RunnerOutput>(runner_,static_cast<int>(sample_rate_hz()));
        runner_.start();
    }
    void stop() { output_->invalidate(); runner_.stop(); }

    // Sim-thread call, once per tick, per vehicle: pushes this tick's sound
    // state into the voice's lock-free input queue (SPSC: this is the ONE
    // producer). A full queue (the render thread has fallen far behind)
    // drops the sample - Voice::push_input's own contract, not handled
    // specially here.
    void push_tick(const ps::drivetrain::EngineSoundState& ess, ps::real t_sim_s);
    // Diagnostic (env PS_VOICE_OUTPUT_RAW): appends the frames a consumer read
    // from the Runner as raw interleaved float32, and one line per read
    // (steady-clock us, frames) to <path>.reads.csv. No-op when unset.
    void record_output(const float* samples, std::size_t frames);

    void set_paused(bool paused) { voice_.set_paused(paused); }

    std::shared_ptr<ps_audio::RunnerOutput> native_output() const { return output_; }
    [[nodiscard]] ps_audio::Runner& runner() { return runner_; }
    [[nodiscard]] const ps_audio::Runner& runner() const { return runner_; }
    [[nodiscard]] ps_audio::Voice& voice() { return voice_; }
    // Immutable source metadata; channel must be in [0, channel_count()).
    // The returned reference remains valid for this VehicleVoice's lifetime.
    [[nodiscard]] const ps::engine_cycle::RadiatingEnd& source(int channel) const { return voice_.source(channel); }
    [[nodiscard]] int channel_count() const { return runner_.channel_count(); }
    [[nodiscard]] double sample_rate_hz() const { return static_cast<double>(voice_.sample_rate_hz()); }

private:
    VehicleVoice(const ps::drivetrain::CycleModelDesc& cycle, ps::real lhv_j_kg, ps::real afr_stoich,
                 const ps_audio::VoiceSettings& settings);

    ps_audio::Voice voice_;
    std::shared_ptr<ps_audio::RunnerOutput> output_;
    ps_audio::Runner runner_; // holds a non-owning Voice* into voice_ above - member order matters (Voice first)
    // Diagnostic input recording (audio-steps investigation, 2026-10-04): when the
    // environment variable PS_VOICE_INPUT_CSV names a file, every pushed
    // EngineInput is appended to it as CSV (flushed every 240 ticks) -
    // replayable by ps_engine_audio --input-csv.
    std::FILE* record_file_ = nullptr;
    std::uint64_t recorded_rows_ = 0;
    std::FILE* output_file_ = nullptr;
    std::FILE* output_reads_file_ = nullptr;
};

} // namespace ps_godot
