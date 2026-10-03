// tools/common/engine_sound_glue.h - ps_audio A11: the ONE shared mapping
// from a ps::drivetrain::EngineSoundState (a vehicle's or EngineRig's
// per-tick sound-relevant state, ps_core public API) to a
// ps_audio::EngineInput (the SPSC snapshot a Voice consumes, ps_audio public
// API).
//
// Lives here, in tools/common, rather than in either library: ps_audio must
// never include ps_core (audio/CMakeLists.txt), and ps_core must never
// include ps_audio (CLAUDE.md's core/ never includes ps_audio/ rule). A
// header-only helper that includes both is fine as long as it lives OUTSIDE
// both targets and is only ever included by a binary that already links
// both - tools/ps_engine_audio (A10) and adapters/godot's ps_godot_adapter
// (A11) are exactly that. Before A11 this mapping was duplicated as a
// static function inside tools/ps_engine_audio/main.cpp
// (make_engine_input(const ps::EngineRig&)); this header is the same logic,
// parameterised on the EngineSoundState + sim time instead of an EngineRig,
// so tools/ps_engine_audio's own output stays bit-identical (verified by
// A10's R4 WAV-hash test) while adapters/godot can feed it snapshots that
// never came from an EngineRig at all (a real vehicle's own engine).
//
// Sign conventions cross-checked against docs/data_formats/engine.md: a
// spark schedule's `spark_advance_deg` is documented "[deg BTDC]" (deg_atdc.md
// line ~591: "Wiebe start at -(schedule) + ignition_delay_deg (deg ATDC)"),
// matching ps_audio::EngineInput::spark_deg_atdc's own "negative=before TDC"
// convention exactly - so spark_deg_atdc = -spark_advance_deg, no further
// adjustment needed here (the cycle model itself applies
// ignition_delay_deg).
#pragma once

#include "ps/drivetrain/powertrain_state.h"
#include "ps_audio/engine_input.h"

#include <cmath>
#include <cstdint>

namespace ps_tools {

// Straight field mapping, no physics/policy added here: omega_rad_s is
// ess.omega verbatim (0 whenever the source reports 0, e.g. an engine that
// has never run or is genuinely stationary - Off/Stalled do not force a
// zero here, since a real vehicle's crank can still be turning while Off,
// e.g. push-started or towed; Voice's own pause/tail handling, not this
// mapping, is what makes Off/Stalled sound right, per audio_plan.md
// section 4). `t_sim_s` is supplied by the caller (EngineRig has its own
// sim clock; a vehicle snapshot uses the World's sim time) since
// EngineSoundState itself carries no timestamp.
inline ps_audio::EngineInput make_engine_input(const ps::drivetrain::EngineSoundState& ess, ps::real t_sim_s) {
    ps_audio::EngineInput in;
    in.t_sim_s = t_sim_s;
    in.omega_rad_s = ess.omega;
    // EngineSoundState::crank_angle is in radians, period 4*pi (two crank
    // revolutions); EngineInput::crank_angle_deg wants degrees, period 720,
    // same phase origin (both come from the same underlying
    // ps::engine_cycle crank-angle convention, 0 = cylinder 1 gas-exchange
    // TDC - fill_engine_sound_state does not re-zero it).
    constexpr ps::real kRadToDeg = 180.0 / 3.14159265358979323846;
    ps::real deg = std::fmod(ess.crank_angle * kRadToDeg, 720.0);
    if (deg < 0.0) deg += 720.0;
    in.crank_angle_deg = deg;
    in.p_charge_pa=ess.p_charge_pa; in.t_charge_k=ess.t_charge_k;
    in.p_intake_pa = ess.p_intake_pa;
    in.throttle = ess.throttle_position;
    in.lambda = ess.lambda;
    in.spark_deg_atdc = -ess.spark_advance_deg;
    in.spark_given = true;
    std::uint32_t flags = 0;
    if(ess.engine_state!=ps::drivetrain::EngineState::Running && ess.engine_state!=ps::drivetrain::EngineState::Cranking)
        flags|=ps_audio::kInputIgnitionCut;
    if (ess.dfco_active) flags |= ps_audio::kInputDfco;
    if (ess.limiter_active) flags |= ps_audio::kInputFuelCut;
    in.flags = flags;
    return in;
}

} // namespace ps_tools
