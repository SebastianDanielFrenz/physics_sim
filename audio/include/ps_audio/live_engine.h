// ps_audio/live_engine.h - the streaming multi-cylinder engine of one voice
// (ps_audio A7, out/audio_plan.md section 3 A7). The same EngineCore as the map
// generator's CycleEngine (n phased CycleSteppers + the engine's PipeNetwork
// with its installation waveguide), stepped continuously instead of in whole
// prescribed cycles:
//
// - one physics step per output sample (h = 1/fs), or two with a half-band
//   decimator when the load-time stability probe asks for it (substeps 2);
//   4/8/16 (a half-band cascade) only when a caller forces them;
// - time-varying speed: every step advances the crank by omega*h (degrees),
//   omega given per sample by the caller (the voice interpolates it from the
//   snapshots and applies the capped crank-phase correction);
// - the throttle Cd*A follows the snapshot's manifold pressure through a slow
//   plenum loop: a one-pole filter (plenum_filter_tau_s) on the plenum pressure
//   and a multiplicative integral Cd*A *= 1 + h/tau * (p_target - p_f)/p_target
//   (plenum_pi_tau_s ~ 50 ms [A]); without a pressure target the throttle
//   fraction maps linearly onto [min, max] Cd*A (PROVISIONAL);
// - per combustion event (at each cylinder's IVC): lambda, start of burn and
//   burn duration from the inputs plus the seeded cycle-to-cycle variation;
//   fuel cut / DFCO at IVC and ignition cut at the start of burn leave the
//   charge unburned (it is pumped out, no afterfire);
// - omega = 0: the crank stops (cylinders frozen in angle, valves at their
//   frozen lift), the pipes ring out; no division by speed anywhere.
//
// Output per radiating end (EngineCore::radiating_ends: each tailpipe outlet,
// the intake mouth) = a monopole at 1 m, p = rho_amb/(4 pi) dq/dt [Pa], q the
// volume outflow. PROVISIONAL placeholder radiation: A8 replaces it with the
// calibrated radiation model.
//
// Deterministic (ps::math only, plain double, scalar); allocation-free after
// construction.
#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/engine_cycle/combustion_variation.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/types.h"
#include "ps_audio/halfband.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ps_audio {

using ps::real;

inline constexpr int kLiveMaxSources = 8;
// Physics steps per output sample: 1, or a power of two up to 16 decimated by a
// cascade of half-band stages (2 = one stage, the probe's choice; 4-16 are for
// offline listening tests at finer pipe cells - a cost of that many times the
// per-step work).
inline constexpr int kLiveMaxSubsteps = 16;
inline constexpr int kLiveMaxDecimStages = 4;

struct LiveEngineSettings {
    real sample_rate_hz = 48000.0;
    int substeps = 1;             // physics steps per output sample: 1, 2, 4, 8 or 16
    real dx_intake_m = 0.0;       // 0 = desc.resolution.audio_dx_intake_m
    real dx_exhaust_m = 0.0;      // 0 = desc.resolution.audio_dx_exhaust_m
    real p_amb_pa = 101325.0;
    real t_amb_k = 288.15;
    real initial_rpm = 800.0;          // network start / throttle feed-forward
    real initial_p_intake_pa = 40000.0;
    real plenum_filter_tau_s = 0.01;   // [A]
    real plenum_pi_tau_s = 0.05;       // [A] brief A7
    real min_throttle_cd_area_m2 = 1e-6;  // [A] closed-plate leakage floor
    bool plenum_loop = true;           // false: the throttle Cd*A stays at the feed-forward start
    // ps_audio A9b: physics-step worker count (>= 1; 1 = serial, the default
    // and bit-identical to before A9b). The pipes and the cylinders both run on
    // this one team; the result is bit-identical for every count.
    int threads = 1;
    std::uint64_t seed = 0;
    real variation_scale = 1.0;        // 0 = no cycle-to-cycle variation
    ps::engine_cycle::CombustionVariationDesc variation{};
};

// Inputs of one output sample (already interpolated by the voice).
struct LiveControl {
    real omega_rad_s = 0.0;        // crank speed used for the crank advance (>= 0)
    real omega_input_rad_s = -1.0; // diagnostic only: the interpolated snapshot omega before the
                                   // phase correction (< 0: same as omega_rad_s); logged per firing
    real p_charge_pa = 0.0, t_charge_k = 0.0; // compressor supply, 0 = atmospheric path
    real p_intake_pa = 0.0;        // plenum pressure target (<= 0: use throttle)
    real throttle = 0.0;
    real lambda = 1.0;
    real start_of_burn_deg_atdc = 0.0;  // resolved start of burn (spark + ignition delay, or MBT)
    std::uint32_t flags = 0;       // EngineInputFlags
};

struct FiringEvent {
    real t_s = 0.0;        // engine time of the firing TDC (fractional step interpolation)
    int cylinder = 0;      // 0-based
    real omega_rad_s = 0.0;  // snapshot omega of that step (LiveControl::omega_input_rad_s)
};

class LiveEngine {
public:
    // Throws std::invalid_argument on a desc without pipe blocks, a bad
    // setting, or a failed network stability check.
    LiveEngine(const ps::drivetrain::CycleModelDesc& desc, real lhv_j_kg, real afr_st,
               const LiveEngineSettings& settings);
    LiveEngine(const LiveEngine&) = delete;
    LiveEngine& operator=(const LiveEngine&) = delete;

    // One output sample: `substeps` physics steps, one pressure sample [Pa at
    // 1 m] per source into out[0 .. source_count()).
    void step_sample(const LiveControl& c, real* out);

    [[nodiscard]] int source_count() const { return sources_; }
    [[nodiscard]] const ps::engine_cycle::RadiatingEnd& source(int k) const {
        return core_.radiating_ends()[static_cast<std::size_t>(k)];
    }
    [[nodiscard]] int cylinders() const { return core_.cylinders(); }
    [[nodiscard]] int substeps() const { return settings_.substeps; }
    [[nodiscard]] real h() const { return h_; }
    [[nodiscard]] real time_s() const { return t_; }
    [[nodiscard]] int step_threads() const { return core_.step_team() ? core_.step_team()->size() : 1; }
    [[nodiscard]] real crank_deg() const { return theta_; }  // cylinder 1, [0, 720)
    [[nodiscard]] real plenum_pressure_pa() const;
    [[nodiscard]] real plenum_filtered_pa() const { return p_f_; }
    [[nodiscard]] real throttle_cd_area() const { return core_.throttle_cd_area(); }
    [[nodiscard]] const ps::engine_cycle::EngineCore& core() const { return core_; }
    // The model's MBT start of burn [deg ATDC] (ca50_mbt through the Wiebe).
    [[nodiscard]] real mbt_start_of_burn_deg() const { return sob_mbt_; }
    [[nodiscard]] real ignition_delay_deg() const { return ign_delay_; }

    // Per-cylinder fresh air through the intake valve over the last full cycle
    // (between consecutive IVC events) [kg], and the IVC event count.
    [[nodiscard]] real last_fresh_air_kg(int c) const { return fresh_last_[static_cast<std::size_t>(c)]; }
    [[nodiscard]] std::uint64_t ivc_events(int c) const { return events_[static_cast<std::size_t>(c)]; }
    // Charges that were cut (fuel cut / DFCO at IVC, or ignition cut at the start of burn).
    [[nodiscard]] std::uint64_t cut_events() const { return cuts_; }

    // Firing-TDC log (ring of kFiringLog entries; total count grows).
    static constexpr int kFiringLog = 4096;
    [[nodiscard]] std::uint64_t firing_count() const { return firing_count_; }
    [[nodiscard]] const FiringEvent& firing(std::uint64_t i) const { return firing_[i % kFiringLog]; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
    void physics_step(const LiveControl& c, real* p_out);

    LiveEngineSettings settings_;
    ps::engine_cycle::EngineCore core_;
    int sources_ = 0;
    real h_ = 0.0;
    real t_ = 0.0;
    real theta_ = 0.0;
    real p_f_ = 0.0;
    real rho_amb_ = 0.0;
    real sob_mbt_ = 0.0, ign_delay_ = 0.0, burn_deg_ = 0.0;
    std::array<real, kLiveMaxSources> q_prev_{};
    // Substep outputs of one sample, and one half-band cascade per source
    // (stage 0 at the physics rate).
    std::array<std::array<real, kLiveMaxSources>, kLiveMaxSubsteps> sub_{};
    std::array<std::array<HalfbandDecimator, kLiveMaxDecimStages>, kLiveMaxSources> dec_{};
    int stages_ = 0;
    std::vector<real> fresh_acc_prev_, fresh_last_;
    std::vector<std::uint64_t> events_;
    std::uint64_t cuts_ = 0;
    std::vector<FiringEvent> firing_;
    std::uint64_t firing_count_ = 0;
};

// Load-time stability probe (brief A7 item 3): two cycles at the stiffest grid
// corner (lowest grid rpm, lowest grid p_im) with the throttle held at its
// feed-forward, at h and at h/2; the relative difference of the second
// cycle's fresh air (summed over cylinders) beyond `bound` [A: 1 %] or a
// non-finite state asks for 2 substeps.
struct StabilityProbeResult {
    real rpm = 0.0;
    real p_im_pa = 0.0;
    real air_h_kg = 0.0;
    real air_h2_kg = 0.0;
    real rel_diff = 0.0;
    bool finite = true;
    int substeps = 1;
};

[[nodiscard]] StabilityProbeResult probe_stability(const ps::drivetrain::CycleModelDesc& desc, real lhv_j_kg,
                                                   real afr_st, const LiveEngineSettings& base, real bound = 0.01);

}  // namespace ps_audio
