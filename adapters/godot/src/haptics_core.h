// adapters/godot/src/haptics_core.h — multi-stage gamepad rumble shaping
// for chassis vibration (owner request: "controller vibration (multi
// stage) for when the chassis vibrates - due to drifting, wheel spin,
// uneven ground, basically anything that causes G-forces").
//
// Godot-free, same contract as input_map_core.h (this directory): plain
// structs in, plain struct out, no Godot type, no ps_core type, allocation-
// free per call (fixed-size std::array, no heap/container churn) - so it
// stays unit-testable headless (adapters/godot/tests/test_haptics.cpp) and
// portable to a future non-Godot front end (CLAUDE.md "engine-neutral
// logic"). The Godot side (input_map.cpp/ps_simulation.cpp) only gathers
// plain numbers from the race-free adapter snapshot into a HapticsInput
// each frame, calls HapticsCore::update(), and forwards the resulting
// HapticsOutput to godot::Input::start_joy_vibration - it does no shaping
// math of its own, mirroring PsInputMap/InputMapper's own division of
// labour.
//
// Two motors, standard Xbox-layout convention: `strong` is the LOW-
// frequency motor, `weak` is the HIGH-frequency motor, both 0..1.
//
// Sources combined into the two motors (see HapticsCore::update's own
// comment for exactly how):
//   1. Body-G TRANSIENT (STRONG): a high-pass of chassis proper
//      acceleration (gravity already removed by the caller - see
//      HapticsInput::accel_x/y/z below) - steady-state cornering/braking G
//      decays to silence, but a CHANGE (a kerb hit, a landing, a snap)
//      pulses and decays.
//   2. Sustained GRIP-LIMIT (STRONG) - owner refinement, 2026-09-24: "a
//      real car rumbles for as long as a tyre works beyond its grip limit"
//      (sustained understeer push, a held drift, wheelspin, a locked
//      wheel) - NOT high-passed, a direct per-wheel function of the
//      current slip vs. that tyre's own peak (HapticsTuning::
//      peak_slip_ratio/peak_slip_angle_rad below), weighted by the wheel's
//      load share and saturating. Below the peak (e.g. hard but grippy
//      cornering) this contributes nothing; the moment any wheel's
//      combined slip crosses its peak it starts rumbling and KEEPS
//      rumbling for as long as it stays past the peak - source 1 above
//      covers the transient snap, this covers the sustained condition.
//   3. Road texture / load noise (WEAK, some STRONG bleed via the
//      combiner): a high-pass of each wheel's normal load relative to its
//      own (slowly tracked) mean load, RMS'd over a short window - a kerb
//      or dirt buzzes, smooth asphalt is near silent.
//   4. Light tyre slip (WEAK): the same per-wheel slip signal as source 2
//      but against MUCH lower thresholds (a burnout/drift starts buzzing
//      well before it crosses the real grip-limit peak).
//   5. Airborne landing (STRONG pulse): any wheel's contact edge
//      (no-contact -> contact) triggers a short pulse scaled by the
//      vertical accel spike, decaying over HapticsTuning::
//      landing_pulse_decay_s. Combined with source 1 via max (not sum) to
//      avoid double-counting the same physical event.
//   6. Engine (WEAK, optional, small): a faint buzz near the rev limiter
//      plus a short pulse on a gear grind or a stall edge. Every field in
//      HapticsEngineInput defaults to "no engine data" (limiter_rpm <= 0),
//      so a vehicle with no drivetrain telemetry wired up yet still
//      produces valid, silent output from this source alone.
//
// "Multi-stage": the combined 0..1 intensity per motor is quantised into
// named stages (off/light/medium/strong/max, HapticsTuning::stage_levels)
// with HYSTERESIS between stage transitions (HapticsTuning::
// stage_hysteresis - up-transition at stage_up_thresholds[i], down-
// transition only once clearly below that by the hysteresis margin) so an
// input hovering at a boundary does not flicker the output stage every
// frame, then attack/release-smoothed (HapticsTuning::attack_s/release_s)
// so a stage change still feels like a short ramp, not a step.
//
// Determinism (PLAN.md D7-style: same input sequence -> bit-identical
// output): every operation here is plain float arithmetic with no
// randomness, no wall-clock read and no container iteration order
// dependency - two HapticsCore instances fed the same HapticsInput
// sequence produce bit-identical HapticsOutput sequences.

#pragma once

#include <array>
#include <cstdint>

namespace ps_godot {

// Fixed upper bound on wheels processed per call (allocation-free -
// std::array, not std::vector). Every vehicle in this project has 4
// wheels (data/vehicles/*.json); 8 leaves headroom without ever touching
// the heap.
inline constexpr int kMaxHapticsWheels = 8;

// One wheel's contact-relevant state for this frame, gathered from the
// adapter snapshot's WheelState/WheelTelemetry (ps/vehicle/wheel_state.h,
// ps/vehicle/wheel_telemetry.h) - duplicated here as plain floats rather
// than taking those types directly, so this file has zero dependency on
// ps_core (same reasoning as input_map_core.h: RawInput/ControlChannels
// don't take Godot types either, for the same "stays link-independent,
// stays portable" reason).
struct HapticsWheelInput {
    float normal_load_n = 0.0f;    // Fz, N - 0 when not in contact
    float slip_ratio = 0.0f;       // dimensionless, signed
    float slip_angle_rad = 0.0f;   // radians, signed
    bool in_contact = false;
};

// Engine/gearbox flags (PowertrainSnapshot subset, ps/drivetrain/
// powertrain_state.h) for the optional, small "source 6" buzz. Every
// field defaults to "no data" so a vehicle with nothing wired up here
// (limiter_rpm <= 0) contributes silence from this source alone.
struct HapticsEngineInput {
    float rpm = 0.0f;
    float limiter_rpm = 0.0f;   // <= 0 => unknown/not applicable, source contributes 0
    bool limiter_active = false;
    bool grind_active = false;
    bool stalled = false;
};

// Everything HapticsCore::update() needs for one frame. `dt_s` is
// explicitly the caller's own choice of clock (see the .cpp/adapter
// wiring for which one this project uses and why) - HapticsCore treats it
// as "seconds since the previous update() call" and nothing else.
struct HapticsInput {
    float dt_s = 0.0f;

    // Chassis PROPER (gravity-removed) linear acceleration, CHASSIS frame
    // (x forward, y left, z up - ISO 8855, matching every other channel in
    // this adapter, e.g. input_map_core.h's ControlChannels::steer). "Proper"
    // = coordinate acceleration minus the gravity vector (what an
    // accelerometer bolted to the chassis would read: ~0 in free fall,
    // ~+9.81 upward at rest on flat ground) - see haptics_core.cpp's
    // adapter-wiring comment (ps_simulation.cpp) for exactly how the
    // caller derives this from consecutive published snapshots.
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;

    std::array<HapticsWheelInput, kMaxHapticsWheels> wheels{};
    int wheel_count = 0; // clamped to kMaxHapticsWheels by the caller

    HapticsEngineInput engine{};

    // True while the sim is not advancing (paused, or the frame's data is
    // otherwise stale) - forces both motors to 0 for this call without
    // resetting internal filter/stage state, so rumble resumes cleanly
    // rather than re-attacking from cold.
    bool paused = false;
};

struct HapticsOutput {
    float weak = 0.0f;   // high-frequency motor, 0..1
    float strong = 0.0f; // low-frequency motor, 0..1
};

// Every tuning knob named so a test can pin one source at a time (owner
// requirement) and so the constants have one documented home instead of
// being scattered as magic numbers through update()'s body.
struct HapticsTuning {
    // --- Source 1: body-G TRANSIENT (STRONG), high-pass + threshold/saturate ---
    // fc ~1-2 Hz (owner spec): steady cornering/braking G is well below
    // this and decays away; a kerb hit/landing/snap has most of its energy
    // well above it and passes through as a pulse.
    float body_g_highpass_fc_hz = 1.5f;
    float body_g_threshold_mps2 = 0.5f;  // |a_hp| at/below this -> 0
    float body_g_saturate_mps2 = 15.0f;  // |a_hp| at/above this -> 1.0

    // --- Source 2: sustained GRIP-LIMIT (STRONG), owner refinement ---
    // "Peak" values a real Pacejka MF5.2 tyre curve would need solving
    // (B/C/D/E coefficients) to derive exactly per data/tyres/*.json file -
    // deliberately NOT attempted here (adapters/ may not reimplement core
    // tyre physics, CLAUDE.md "do not touch core/", and re-deriving the
    // Magic Formula peak outside core would duplicate, and risk diverging
    // from, the real model). Instead: fixed, documented estimates.
    // peak_slip_angle_rad: this project's core/CLAUDE.md itself records
    // "qbz1 sets a realistic ~4-4.5 deg peak slip angle" for the shipped
    // passenger-car tyre data (vault PHYS-036) - 4.5 deg chosen (the upper,
    // more conservative end, so this source does not start rumbling
    // slightly early on a real car at its true limit).
    // peak_slip_ratio: no equivalent measured note exists in this project;
    // 0.10 is the commonly-cited peak longitudinal slip ratio for a road
    // tyre (most MF5.2-family passenger tyre datasets peak in the
    // 0.08-0.12 range) - an estimate, not a measured value, and the single
    // named knob to revisit if a future task measures the real peak per
    // data/tyres/*.json.
    float peak_slip_ratio = 0.10f;
    float peak_slip_angle_rad = 0.0785398f; // 4.5 deg
    // Utilisation (|slip| / peak) at which this source reaches full
    // strength - 1.0 is exactly at the peak (start of rumble), so this is
    // "how far past the peak until it's maxed out".
    float grip_limit_saturate_utilization = 2.5f;

    // --- Source 3: road texture / load noise (WEAK, some STRONG bleed) ---
    float load_highpass_fc_hz = 3.0f;   // rejects slow load-transfer trends, keeps fast texture
    float load_static_tau_s = 1.0f;     // slow low-pass tracking each wheel's own "static" load
    float load_rms_window_s = 0.15f;
    float load_noise_threshold = 0.02f; // RMS(high-pass/static) at/below this -> 0
    float load_noise_saturate = 0.25f;

    // --- Source 4: LIGHT tyre slip (WEAK) - well below the grip-limit peak ---
    float slip_ratio_threshold = 0.04f;
    float slip_ratio_saturate = 0.15f;
    float slip_angle_threshold_rad = 0.0261799f; // 1.5 deg
    float slip_angle_saturate_rad = 0.1047198f;  // 6 deg

    // --- Source 5: airborne landing (STRONG pulse) ---
    float landing_accel_threshold_mps2 = 10.0f;
    float landing_accel_saturate_mps2 = 40.0f;
    float landing_pulse_decay_s = 0.15f;

    // --- Source 6: engine (WEAK, optional, small) ---
    float engine_limiter_margin_rpm = 300.0f; // starts buzzing this far below the limiter
    float engine_limiter_weight = 0.35f;      // "faint" cap on the limiter buzz alone
    float engine_grind_weak = 0.6f;
    float engine_grind_decay_s = 0.2f;
    float engine_stall_pulse = 0.4f;
    float engine_stall_decay_s = 0.3f;

    // --- Stage quantisation + hysteresis + attack/release smoothing ---
    // 5 stages: off / light / medium / strong / max (stage_levels below).
    // stage_up_thresholds[i] is the combined intensity needed to move from
    // stage i to i+1; moving back down from i+1 to i needs the intensity
    // to drop below stage_up_thresholds[i] - stage_hysteresis (not just
    // below stage_up_thresholds[i] again) - the hysteresis margin is what
    // stops a value hovering exactly at a boundary from flickering the
    // output stage every frame.
    std::array<float, 4> stage_up_thresholds{0.12f, 0.30f, 0.55f, 0.80f};
    float stage_hysteresis = 0.06f;
    std::array<float, 5> stage_levels{0.0f, 0.20f, 0.45f, 0.70f, 1.0f};
    float attack_s = 0.03f;  // rising toward a higher stage target
    float release_s = 0.12f; // falling toward a lower stage target

    float strength = 1.0f; // global 0..1 scale, applied last
    bool enabled = true;
};

// Stateful (like InputMapper): owns every source's filter/RMS/pulse/stage/
// smoothing state across calls. One instance per vehicle/controller.
class HapticsCore {
public:
    explicit HapticsCore(HapticsTuning tuning = HapticsTuning{}) : tuning_(tuning) {}

    [[nodiscard]] HapticsOutput update(const HapticsInput& in);

    [[nodiscard]] const HapticsTuning& tuning() const { return tuning_; }
    void set_tuning(const HapticsTuning& t) { tuning_ = t; }

    // Clears every filter/RMS/pulse/stage/smoothing state back to a fresh
    // instance's - for a vehicle teleport/reset (main.gd's "reset vehicle"
    // control, adapters/godot/demo/scripts/main.gd's _reset_vehicle), so
    // the chassis-velocity discontinuity a teleport causes does not read
    // as a spurious G-spike/landing pulse on the next frame.
    void reset();

private:
    // First-order exponential high-pass (leaky differentiator):
    // y[n] = alpha*(y[n-1] + x[n] - x[n-1]), alpha = RC/(RC+dt),
    // RC = 1/(2*pi*fc). Deterministic, allocation-free, needs only the
    // previous x and y.
    struct Highpass1 {
        float prev_x = 0.0f;
        float prev_y = 0.0f;
        bool init = false;
        float step(float x, float dt, float fc);
    };

    struct WheelFilter {
        Highpass1 load_hp;
        float static_load = 0.0f;
        float ac_ms2 = 0.0f; // EMA of the high-pass-relative signal squared (RMS^2)
        bool init = false;
    };

    HapticsTuning tuning_;

    Highpass1 accel_hp_x_, accel_hp_y_, accel_hp_z_;
    std::array<WheelFilter, kMaxHapticsWheels> wheel_filters_{};

    float landing_pulse_level_ = 0.0f;
    std::array<bool, kMaxHapticsWheels> prev_in_contact_{};
    bool prev_contact_init_ = false;

    float engine_grind_level_ = 0.0f;
    float engine_stall_level_ = 0.0f;
    bool prev_stalled_ = false;

    // Stage/smoothing state, separate per motor.
    int weak_stage_ = 0;
    int strong_stage_ = 0;
    float weak_smoothed_ = 0.0f;
    float strong_smoothed_ = 0.0f;
};

} // namespace ps_godot
