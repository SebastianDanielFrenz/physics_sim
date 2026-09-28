// ps_audio/voice.h - one engine voice (ps_audio A7, out/audio_plan.md section 4):
// an EngineInput SPSC queue (physics thread -> audio thread), the time map
// with its rate PLL, and the LiveEngine that renders it.
//
// Threads: push_input() is the ONE producer thread (physics / game); render()
// and every diagnostic accessor are the ONE consumer thread (the Runner, or a
// test calling render() directly); set_paused() may be called from any thread.
//
// Per output sample the voice advances the time map, interpolates the
// snapshots bracketing the read point (omega, manifold pressure, throttle,
// lambda and spark linearly; flags and the crank angle from the snapshot at or
// before it), and steps the LiveEngine. Pitch comes from the interpolated omega
// only - never from the clock rate. While the time map is locked (|r - 1| <=
// lock_tolerance) and the snapshots carry a crank angle, the crank advance
// gets a phase correction toward the snapshot crank angle, capped at
// crank_max_rel (1 %) of omega. Hitch (no snapshot for more than two intervals):
// the last inputs are held. Resume: the time map jumps, no catch-up.
// set_paused(true) fades the output to zero over pause_fade_s (50 ms) and then
// freezes the physics; set_paused(false) re-seats the time map and fades in.
//
// Queue drains, the pause state and the PLL run at control points every
// control_interval output samples counted from the voice's first sample, so
// the output depends only on the input script and the seed, not on how the
// caller splits render() calls. Allocation-free after construction.
#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/types.h"
#include "ps_audio/engine_input.h"
#include "ps_audio/live_engine.h"
#include "ps_audio/spsc.h"
#include "ps_audio/time_map.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace ps_audio {

using ps::real;

struct VoiceSettings {
    LiveEngineSettings engine{};   // engine.substeps is overridden by `substeps`
    TimeMapSettings time_map{};
    // 0 = run the load-time stability probe (probe_stability) and use its answer; 1 or 2 forced.
    int substeps = 0;
    real stability_bound = 0.01;   // [A] brief A7: 1 % trapped air
    int control_interval = 32;     // output samples between control points
    real pause_fade_s = 0.05;
    real crank_tau_s = 0.1;        // phase-correction time constant [A]
    real crank_max_rel = 0.01;     // correction cap, fraction of omega (brief A7)
};

class Voice {
public:
    static constexpr std::size_t kQueueCapacity = 64;
    static constexpr int kHistory = 8;

    Voice(const ps::drivetrain::CycleModelDesc& desc, real lhv_j_kg, real afr_st, const VoiceSettings& settings);
    Voice(const Voice&) = delete;
    Voice& operator=(const Voice&) = delete;

    // Producer thread. false when the queue is full (the snapshot is dropped).
    bool push_input(const EngineInput& in) { return queue_.push(in); }
    // Any thread.
    void set_paused(bool paused) { pause_req_.store(paused, std::memory_order_release); }

    // Consumer thread: `frames` output samples, interleaved per source
    // (out[f * source_count() + s]), pressure at 1 m [Pa].
    void render(float* out, int frames);

    [[nodiscard]] int source_count() const { return engine_.source_count(); }
    [[nodiscard]] const ps::engine_cycle::RadiatingEnd& source(int k) const { return engine_.source(k); }
    [[nodiscard]] real sample_rate_hz() const { return fs_; }
    [[nodiscard]] const StabilityProbeResult& stability() const { return probe_; }

    // Diagnostics (consumer thread).
    [[nodiscard]] const LiveEngine& engine() const { return engine_; }
    [[nodiscard]] const TimeMap& time_map() const { return tm_; }
    [[nodiscard]] std::uint64_t samples_rendered() const { return n_; }
    [[nodiscard]] real gain() const { return gain_; }
    [[nodiscard]] bool frozen() const { return frozen_; }
    [[nodiscard]] real last_omega_input() const { return ctl_.omega_rad_s; }
    [[nodiscard]] real last_omega_applied() const { return omega_applied_; }
    [[nodiscard]] real crank_target_deg() const { return target_theta_; }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const { return engine_.fold_state_hash(h); }

private:
    void control_point();
    void interpolate();

    VoiceSettings s_;
    StabilityProbeResult probe_;
    LiveEngine engine_;
    TimeMap tm_;
    real fs_ = 48000.0;
    real h_ = 1.0 / 48000.0;
    SpscQueue<EngineInput, kQueueCapacity> queue_;
    std::atomic<bool> pause_req_{false};

    std::array<EngineInput, kHistory> hist_{};  // oldest .. newest
    int hist_n_ = 0;
    int lower_ = -1;           // index of the snapshot at or before t_read (last sample)
    real lower_t_ = -1.0;      // its simulation time (to detect a new crossing)
    LiveControl ctl_{};
    real omega_applied_ = 0.0;
    real target_theta_ = 0.0;
    bool target_valid_ = false;
    real gain_ = 1.0;
    bool pausing_ = false;
    bool frozen_ = false;
    std::uint64_t n_ = 0;
    std::array<real, kLiveMaxSources> frame_{};
};

// Offline render (allocates the buffer; not for the real-time path): `frames`
// output samples of every source, interleaved as Voice::render writes them.
[[nodiscard]] std::vector<float> render_to_buffer(Voice& voice, std::size_t frames);

}  // namespace ps_audio
