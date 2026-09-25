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
//   3. Load-EVENT noise (WEAK, some STRONG bleed via the combiner): a
//      high-pass of each wheel's normal load relative to its own (slowly
//      tracked) mean load, RMS'd over a short window - a real physical
//      event (a kerb edge, a pothole/bump) buzzes. Fades in with ground
//      speed (HapticsWheelInput::ground_speed_mps, same reasoning/knobs
//      style as sources 2/4's speed_fade below) - owner bug, 2026-09-25:
//      "accelerating in 3rd gear is enough to vibrate the controller". Real
//      ps_run telemetry traced this to a LAUNCH transient, not 3rd gear or
//      wheelspin: pulling away from a near-standstill drives a fast,
//      large front-wheel load swing (measured: FL Fz ~932N -> ~4000+N
//      within ~0.25s) that this source's high-pass/RMS reads as maximum-
//      intensity "road texture" for ~1.5-2s while it rings down, even on
//      dead-flat ground with no bump/kerb anywhere - and that ~2s tail
//      overlaps the 1st/2nd/3rd upshift a hard launch does in quick
//      succession, so the vibration is felt "in 3rd gear" even though the
//      trigger is the initial pull-away. A car isn't "rolling over
//      texture" before it is actually rolling - this source was the one
//      exception among the speed-gated sources; steady, already-at-speed
//      WOT cruising (confirmed with real telemetry, including a hard
//      throttle stab while already moving) produces no output from this
//      source at all.
//      Owner refinement, 2026-09-25: "good asphalt doesn't cause rumble" -
//      driving steadily on the test_ground asphalt straight at 50/100/150
//      km/h (part throttle in 3rd/4th too) was STILL producing audible
//      buzz from this source even after the launch-transient speed gate
//      above, because ordinary AMBIENT load ripple on a real (triangulated
//      mesh, not a mathematically flat plane) asphalt surface - suspension
//      settle, mesh-seam micro-bumps, small aero/engine-vibration coupling
//      - was landing above `load_noise_threshold`.
//      MEASURED (this session, real telemetry - not an estimate) via the
//      demo's `--m1-haptics-cruise` scripted drive (main.gd) with
//      PS_HAPTICS_LOG=1/HapticsSourceDebug::source3_load_noise_rms_raw (the
//      PRE-threshold RMS(load_rel) this comment's numbers below refer to):
//      ~230 s of steady/settling asphalt cruise on the long_straight,
//      part-throttle in 3rd (43 km/h)/6th (86, 131 km/h) - a TYPICAL
//      (median) sample sits at raw RMS ~0.011-0.013; real mesh-seam
//      micro-bumps produce brief (1-2 sample, ~1 s) spikes, the highest
//      observed 0.0429 (a single dominant-wheel sample at 55 km/h in 3rd);
//      the run's overall max across launch+shifts+cruise was 0.0429. Every
//      one of these stayed below `load_noise_threshold` (0.05), so
//      `source3_load_noise` (the POST-threshold value) read exactly 0.0
//      for the entire drive - this source never fired once. That is real
//      margin (~17% above the single highest observed spike, ~4x above the
//      typical steady-state sample), not the wide "many-x" margin an
//      earlier, pre-measurement draft of this comment claimed - a spike
//      only modestly rougher than what this track's mesh happened to
//      produce could plausibly cross 0.05, so this threshold is not
//      something to raise carelessly if a future track/mesh needs
//      revisiting. For comparison, the existing synthetic bump pin
//      (tests/test_haptics.cpp's "load noise on one wheel" case: a
//      +-800N/4000N, i.e. ~20%, 10 Hz oscillation - a stand-in for an
//      actual kerb-edge tap or pothole, not a measurement of a real one)
//      clears the threshold comfortably, confirming it still catches an
//      actual bump-scale event; this project has no logged REAL kerb/
//      pothole impact number to compare against, unlike the ambient-cruise
//      figures above which are real. Source 3's JOB narrowed accordingly to
//      real load EVENTS only; steady-state road texture (asphalt vs. kerb
//      vs. dirt/grass/field vs. low_mu) is now source 7's job below, not
//      this one's.
//   4. Light tyre slip (WEAK): the same per-wheel slip signal as source 2
//      but against thresholds set CLEARLY ABOVE ordinary traction slip (a
//      burnout/drift starts buzzing well before it crosses the real
//      grip-limit peak, but normal FWD-under-throttle traction slip must
//      not). Owner refinement, 2026-09-25: the previous slip_ratio
//      threshold (0.04) sat INSIDE the normal traction-slip band a FWD car
//      shows under ordinary throttle in 2nd/3rd (~0.03-0.05, this file's
//      own measured-cruise numbers above), so ordinary driving could buzz
//      this source - raised to sit clearly above that band (0.065) with
//      saturation just past the real grip-limit peak (peak_slip_ratio
//      0.10) so this source ramps through the "approaching the limit" zone
//      and hands off smoothly to source 2's sustained rumble right at the
//      peak. slip_angle_threshold_rad raised the same way (1.5 -> 3.5 deg,
//      still comfortably below the ~4.5 deg peak_slip_angle_rad).
//   5. Airborne landing (STRONG pulse): any wheel's contact edge
//      (no-contact -> contact) triggers a short pulse scaled by the
//      vertical accel spike, decaying over HapticsTuning::
//      landing_pulse_decay_s. Combined with source 1 via max (not sum) to
//      avoid double-counting the same physical event.
//   6. Engine (WEAK, optional, small): a short pulse on a gear grind or a
//      stall edge - both real, discrete drivetrain events. Deliberately
//      does NOT buzz near/at the rev limiter (owner requirement,
//      2026-09-25: a rev limiter must not cause rumble on its own - only a
//      physically-derived quantity, e.g. an engine-mount motion/torque-
//      reaction channel, may - and no such channel exists in this
//      codebase yet, core/CLAUDE.md's drivetrain section has no
//      engine-mount model). Every field in HapticsEngineInput defaults to
//      "no engine data", so a vehicle with no drivetrain telemetry wired
//      up yet still produces valid, silent output from this source alone.
//   7. Surface texture / rumble strip (WEAK) - owner bug, 2026-09-25:
//      "didn't feel the kerbs on the asphalt straight at all", then
//      generalised the same day ("road texture should come from the
//      SURFACE ... a data table: asphalt 0, kerb, dirt/field/grass/low_mu -
//      pick plausible strengths; a future 'rough_asphalt' surface would
//      just be a table row"). Per-wheel HapticsWheelInput::surface_name is
//      looked up in HapticsTuning::surface_texture_rules (name -> strength;
//      the mapping lives here, not in the Godot glue - CLAUDE.md
//      "engine-neutral logic", a future non-Godot front end or a new
//      surfaces.json entry needs only a tuning change) and, like sources
//      2/4/3 above, fades in with ground speed - a stationary or crawling
//      wheel isn't rolling over a rumble strip yet. Every data/surfaces/
//      surfaces.json name now has an entry (HapticsTuning::
//      surface_texture_rules' own field comment has the per-surface
//      strengths and their justification - derived from that file's own
//      published rolling-resistance coefficients as a roughness proxy,
//      since no other per-surface "texture" measurement exists in this
//      codebase); asphalt is explicit strength 0.0 (documents the decision
//      instead of relying on "no match" default silence) - THE reference
//      surface is silent by design, matching the owner's "good asphalt
//      doesn't cause rumble" requirement.
//
// Debug readout (owner requirement, 2026-09-25, "stays useful for the
// owner's future reports"): HapticsCore::debug() returns a
// HapticsSourceDebug snapshot of every source's own intensity/level from
// the MOST RECENT update() call (before stage quantisation/hysteresis/
// smoothing) plus the final per-motor raw/output values - cheap (plain
// struct copy, no allocation), always kept up to date regardless of
// whether any caller reads it. The Godot glue (input_map.cpp) logs this
// once a second to stdout when the PS_HAPTICS_LOG env var is set, and
// exposes it to GDScript (PsInputMap::get_haptics_debug) for the demo
// HUD - see those files for exactly how.
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
    // Speeds that make the slip values physically meaningful (owner bug,
    // 2026-09-24: a stalled car at rest kept rumbling). At (near) standstill
    // the tyre's relaxed slip ratio/angle are carcass DEFLECTION states, not
    // sliding - nothing is scrubbing, so nothing should vibrate. The
    // slip-ratio sources fade in with max(ground, wheel surface) speed (a
    // burnout at rest still rumbles: the wheel surface is moving); the
    // slip-angle sources fade in with ground speed only. Negative = unknown,
    // no fade (keeps callers that don't supply speeds working unchanged).
    float ground_speed_mps = -1.0f;        // |chassis velocity|, m/s
    float wheel_surface_speed_mps = -1.0f; // |omega * R|, m/s

    // Ground surface under this wheel this frame (matches ps::io::
    // SurfaceTable's names in data/surfaces/surfaces.json, e.g. "kerb",
    // "asphalt" - PsSimulation::get_wheel_surface_name), NUL-padded/
    // truncated to 15 bytes. Empty (data[0] == '\0', the default - "no
    // data"/off the mapped surface list) or a name matching no
    // HapticsTuning::surface_texture_rules entry contributes nothing from
    // source 7 below. A fixed buffer rather than std::string so this
    // header stays allocation-free (this file's own "allocation-free per
    // call" contract, PLAN.md D7 style determinism).
    std::array<char, 16> surface_name{};
};

// Engine/gearbox flags (PowertrainSnapshot subset, ps/drivetrain/
// powertrain_state.h) for the optional, small "source 6" buzz. Every field
// defaults to "no data"/off, so a vehicle with nothing wired up here
// contributes silence from this source alone. `rpm` is currently unused by
// source 6 (kept for a future physically-derived engine-motion channel -
// see this file's own top comment on why a bare rev-limiter buzz was
// removed) but harmless to leave populated by a caller.
struct HapticsEngineInput {
    float rpm = 0.0f;
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

// One HapticsTuning::surface_texture_rules entry (source 7, Bug B: rumble
// strips/kerbs). `name` uses the same fixed-buffer type and NUL-padding
// convention as HapticsWheelInput::surface_name so update() can match them
// with a plain std::array::operator== - no strlen, no allocation.
struct SurfaceTextureRule {
    std::array<char, 16> name{}; // e.g. "kerb"; "" (name[0] == '\0') = unused slot
    float strength = 0.0f;       // 0..1, one wheel's source-7 contribution at full speed/full load-share
};
// 8 (every data/surfaces/surfaces.json name today, plus headroom for a
// future entry - e.g. a "rough_asphalt" row - without another header
// change), same "fixed capacity, no heap" reasoning as kMaxHapticsWheels.
inline constexpr int kMaxSurfaceTextureRules = 8;

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
    // Slip sources (2 and 4) fade in between these speeds - see
    // HapticsWheelInput::ground_speed_mps for why.
    float slip_speed_fade_lo_mps = 0.5f;
    float slip_speed_fade_hi_mps = 2.0f;

    // --- Source 3: load-EVENT noise (WEAK, some STRONG bleed) - a real
    //     physical event (kerb edge, pothole/bump), NOT ambient road
    //     texture (that is source 7's job, below) and NOT ordinary
    //     steady-cruise ripple. See this file's own top comment (source 3,
    //     owner refinement 2026-09-25) for the measured settled-cruise RMS
    //     ceiling `load_noise_threshold` is set against. ---
    float load_highpass_fc_hz = 3.0f;   // rejects slow load-transfer trends, keeps fast texture
    float load_static_tau_s = 1.0f;     // slow low-pass tracking each wheel's own "static" load
    float load_rms_window_s = 0.15f;
    float load_noise_threshold = 0.05f; // RMS(high-pass/static) at/below this -> 0 - see top comment
    float load_noise_saturate = 0.25f;
    // Fades in with ground speed (this file's own top comment, source 3 -
    // owner bug, 2026-09-25: a launch's own suspension/load-transfer
    // transient was maxing this source out while the car was still under
    // ~1 m/s). Same lo/hi meaning as slip_speed_fade_lo_mps/hi_mps above,
    // a separate knob since this source's physical trigger (the whole
    // chassis bouncing) isn't the same as a single wheel sliding.
    float load_noise_speed_fade_lo_mps = 0.5f;
    float load_noise_speed_fade_hi_mps = 2.0f;

    // --- Source 4: LIGHT tyre slip (WEAK) - clearly above ordinary
    //     traction slip, well below the grip-limit peak. Owner refinement,
    //     2026-09-25: raised from 0.04/1.5deg (which sat INSIDE a FWD car's
    //     normal ~0.03-0.05 under-throttle traction slip band) - see this
    //     file's own top comment, source 4. ---
    float slip_ratio_threshold = 0.065f;
    float slip_ratio_saturate = 0.11f;   // just past peak_slip_ratio (0.10) - hands off to source 2 right at the limit
    float slip_angle_threshold_rad = 0.0610865f; // 3.5 deg
    float slip_angle_saturate_rad = 0.1047198f;  // 6 deg

    // --- Source 5: airborne landing (STRONG pulse) ---
    float landing_accel_threshold_mps2 = 10.0f;
    float landing_accel_saturate_mps2 = 40.0f;
    float landing_pulse_decay_s = 0.15f;

    // --- Source 6: engine (WEAK, optional, small) - grind/stall only, see
    //     this file's own top comment for why a bare rev-limiter buzz was
    //     removed (owner requirement, 2026-09-25). ---
    float engine_grind_weak = 0.6f;
    float engine_grind_decay_s = 0.2f;
    float engine_stall_pulse = 0.4f;
    float engine_stall_decay_s = 0.3f;

    // --- Source 7: surface texture / rumble strip (WEAK) - owner bug,
    //     2026-09-25: kerbs weren't felt at all; generalised the same day
    //     to a full per-surface table (this file's own top comment, source
    //     7). HapticsWheelInput::surface_name is matched against `name`
    //     here (exact, NUL-padded compare - first match wins, an empty
    //     `name` marks an unused slot); `strength` is this surface's
    //     source-7 contribution for one wheel at full speed, before the
    //     per-wheel load-share weighting update() applies (so N wheels on
    //     the surface, or one heavily loaded wheel, naturally read stronger
    //     than a single lightly loaded one - task's "optionally scaling
    //     with wheel count/load"). The name -> strength mapping lives here,
    //     not in the Godot glue (CLAUDE.md "engine-neutral logic") - a
    //     future surfaces.json entry or non-Godot front end only needs a
    //     tuning change.
    //
    //     Every data/surfaces/surfaces.json name has an entry. No direct
    //     "texture roughness" measurement exists anywhere in this codebase
    //     (surfaces.json only carries lambda_mu/crr), so each strength is a
    //     plausible estimate using that file's own crr (rolling-resistance
    //     coefficient) as a roughness proxy - a smoother surface has both
    //     lower crr and less texture buzz, a rougher one has both higher
    //     crr and more:
    //       asphalt 0.013 crr -> 0.0  (THE reference surface; owner
    //                                  requirement 2026-09-25, "good
    //                                  asphalt doesn't cause rumble" -
    //                                  explicit entry, not just "no rule
    //                                  matched", to document the decision)
    //       kerb    0.013 crr -> 0.55 (same crr as asphalt - it buzzes not
    //                                  because the surface material is
    //                                  rough but because it is a raised
    //                                  physical EDGE/step a wheel rides
    //                                  along; kept clearly felt, as before)
    //       dirt    0.04  crr -> 0.25
    //       low_mu  0.01  crr -> 0.0  (smoothest of all per crr - polished,
    //                                  wet-ice-like; smooth surfaces are
    //                                  silent (owner, 2026-09-25). Its
    //                                  danger is grip loss, which sources
    //                                  2/4 already make felt the moment
    //                                  a tyre actually slides.)
    //       grass   0.08  crr -> 0.40
    //       field   0.09  crr -> 0.45 (roughest surface in the table)
    std::array<SurfaceTextureRule, kMaxSurfaceTextureRules> surface_texture_rules{
        SurfaceTextureRule{{'a', 's', 'p', 'h', 'a', 'l', 't'}, 0.0f},
        SurfaceTextureRule{{'k', 'e', 'r', 'b'}, 0.55f},
        SurfaceTextureRule{{'d', 'i', 'r', 't'}, 0.25f},
        SurfaceTextureRule{{'l', 'o', 'w', '_', 'm', 'u'}, 0.0f},
        SurfaceTextureRule{{'g', 'r', 'a', 's', 's'}, 0.40f},
        SurfaceTextureRule{{'f', 'i', 'e', 'l', 'd'}, 0.45f}};
    // Same fade-in-with-speed reasoning as sources 2/3/4 - a stationary or
    // crawling wheel isn't rolling over a rumble strip yet. A higher `hi`
    // than the slip/load-noise sources: real rumble strips are meant to be
    // felt across ordinary road speeds, not just near walking pace.
    float surface_texture_speed_fade_lo_mps = 0.5f;
    float surface_texture_speed_fade_hi_mps = 3.0f;

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

// Per-source snapshot from the most recent update() call (this file's own
// top comment, "Debug readout") - every value here is the RAW per-source
// intensity/level BEFORE the final combine/stage-quantisation/hysteresis/
// smoothing update() applies, i.e. exactly what each source itself
// computed this call, so a caller/report can see precisely which source(s)
// are firing instead of only the two already-combined/smoothed motor
// outputs. Plain floats, no allocation, cheap to copy every call.
struct HapticsSourceDebug {
    float source1_body_g = 0.0f;          // body-G transient, post threshold/saturate
    float source2_grip_limit = 0.0f;      // sustained grip-limit, post load-share weighting
    float source3_load_noise = 0.0f;      // load-event noise, dominant wheel, POST threshold/saturate
    float source3_load_noise_rms_raw = 0.0f; // dominant wheel's RMS(load_rel), PRE threshold/saturate - what load_noise_threshold is actually compared against (measurement/report use, see this file's top comment on source 3)
    float source4_light_slip = 0.0f;      // light slip, post load-share weighting
    float source5_landing = 0.0f;         // landing pulse level (post-decay, pre-combine)
    float source6_engine = 0.0f;          // engine grind/stall
    float source7_surface_texture = 0.0f; // surface texture, post load-share weighting
    float strong_raw = 0.0f;              // combined STRONG motor intensity before stage/smoothing
    float weak_raw = 0.0f;                // combined WEAK motor intensity before stage/smoothing
    float strong_out = 0.0f;              // this call's actual HapticsOutput::strong (post everything)
    float weak_out = 0.0f;                // this call's actual HapticsOutput::weak
};

// Stateful (like InputMapper): owns every source's filter/RMS/pulse/stage/
// smoothing state across calls. One instance per vehicle/controller.
class HapticsCore {
public:
    explicit HapticsCore(HapticsTuning tuning = HapticsTuning{}) : tuning_(tuning) {}

    [[nodiscard]] HapticsOutput update(const HapticsInput& in);

    [[nodiscard]] const HapticsTuning& tuning() const { return tuning_; }
    void set_tuning(const HapticsTuning& t) { tuning_ = t; }

    // Debug readout (this file's own top comment) - valid after the first
    // update() call; default-constructed (all zero) before that.
    [[nodiscard]] const HapticsSourceDebug& debug() const { return debug_; }

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
    HapticsSourceDebug debug_;

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
