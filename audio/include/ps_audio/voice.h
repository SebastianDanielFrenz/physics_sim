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
#include "ps_audio/audio_bank.h"

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

// ps_audio A8: a broadband turbulence-noise floor added at each radiating
// end, above and outside the pipe models' own resolved acoustics (out/
// audio_plan.md section 2.2: "High-frequency content above the resolved
// band... a flow-noise layer adds seeded noise"). Deterministic: seeded from
// the voice's LiveEngineSettings::seed (the same seed combustion_variation.h
// already uses), never from wall-clock/thread state.
//
// Model (every numeric default [A], PROVISIONAL - see out/audio_a8_progress.md
// for the calibration status): white noise (SplitMix64,
// ps::engine_cycle::splitmix64_next/unit_uniform - the same generator
// combustion_variation.h uses) is bandpassed by a cascade of one-pole
// filters (a stable, always-well-behaved shape for coefficients that move
// every control point, unlike a resonant biquad at a fast-changing centre
// frequency) whose centre tracks a Strouhal-scaled frequency f = St*u/D
// (u = the outlet's instantaneous volume flow / its physical area, D =
// its hydraulic diameter; St ~ 0.2 is a generic order-of-magnitude
// shear-layer/orifice-tone value - see textbook aeroacoustics, e.g. Blake,
// "Mechanics of Flow-Induced Sound and Vibration", 1986 - not fitted to
// this engine). Level scales as level_coeff_pa * u^level_exponent
// (Lighthill, "On sound generated aerodynamically", Proc. Roy. Soc. A 211,
// 1952, gives u^8 for the ACOUSTIC POWER of a free subsonic jet;
// level_exponent defaults to a much-reduced 3 - deliberately NOT a literal
// implementation of Lighthill's free-jet analogy, just borrowing its
// qualitative "noise grows steeply with flow speed" shape for a practical,
// audibly-modest broadband floor), then modulated by the instantaneous
// resolved outlet pressure (a bounded fraction of frame/pressure_ref_pa) so
// the floor breathes with the exhaust pulse instead of sitting flat.
// level_coeff_pa defaults to 0 (no contribution at all - byte-identical to
// A7 aside from the F3 radiation fix) since audio_plan.md gives no target
// level to calibrate a nonzero default against; V9/V10's tests set a
// nonzero value explicitly.
struct FlowNoiseSettings {
    bool enabled = true;
    real strouhal = 0.2;            // [A] Strouhal number, f = strouhal * u / D
    real level_exponent = 3.0;      // [A] reduced from Lighthill's u^8 (see above)
    real level_coeff_pa = 0.0;      // [A] Pa per (m/s)^level_exponent at 1 m; 0 = disabled
    real pressure_mod = 0.3;        // [A] weight of the resolved-pressure envelope term
    real pressure_ref_pa = 2000.0;  // [A] resolved-pressure scale for that term
    real min_hz = 20.0;             // filter passband clamp (avoids a near-DC or near-Nyquist pole)
    real max_hz = 8000.0;
};

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
    FlowNoiseSettings flow_noise{};  // ps_audio A8
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

    // Called on a preparation/control thread; retains old banks until teardown
    // so a crossfade cannot free a large sample array on the audio thread.
    void set_audio_bank(AudioBankPtr bank);

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
    void flow_noise_control_point();  // ps_audio A8: recompute per-source filter poles
    real flow_noise_sample(int s);    // ps_audio A8: one filtered+shaped noise sample, source s

    // ps_audio A8: per-source flow-noise state (white-noise RNG, one-pole
    // highpass then lowpass cascade state and current pole coefficients -
    // recomputed only at control points, see flow_noise_control_point()).
    struct FlowNoiseState {
        std::uint64_t rng = 0;
        real hp_x1 = 0.0, hp_y1 = 0.0;
        real lp_y1 = 0.0;
        real a_hp = 0.0, a_lp = 0.0;
        real gain_pa = 0.0;  // level_coeff_pa * u^level_exponent, held between control points
    };
    std::array<FlowNoiseState, kLiveMaxSources> flow_noise_{};

    std::atomic<AudioBankPtr> requested_bank_;
    AudioBankPtr active_bank_, previous_bank_;
    std::vector<AudioBankPtr> retained_banks_;
    BankPlayback bank_playback_, previous_playback_;
    real bank_fade_ = 1.0, bank_weight_ = 0.0;
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
