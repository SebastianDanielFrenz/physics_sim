// ps_audio/engine_input.h - the per-tick engine snapshot the physics thread
// hands to an audio voice (ps_audio A7, out/audio_plan.md section 4). Plain
// data, trivially copyable, pushed through the voice's SPSC queue. A10 fills
// it from ps::drivetrain::EngineSoundState; A7 tests script it directly.
#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps_audio {

using ps::real;

enum EngineInputFlags : std::uint32_t {
    kInputIgnitionCut = 1u << 0,  // no spark: charges trapped from here on do not burn
    kInputFuelCut = 1u << 1,      // limiter fuel cut: no fuel for charges trapped from here on
    kInputDfco = 1u << 2,         // deceleration fuel cut-off (same effect as kInputFuelCut)
};

struct EngineInput {
    real t_sim_s = 0.0;            // simulation time of the snapshot [s], strictly increasing
    real omega_rad_s = 0.0;        // crank speed [rad/s] (>= 0; the only source of pitch)
    // Crank angle [deg] in the cycle model's convention (0 = cylinder 1
    // gas-exchange TDC, 360 = its firing TDC, period 720); < 0 = not supplied.
    real crank_angle_deg = -1.0;
    real p_charge_pa = 0.0, t_charge_k = 0.0; // compressor supply, 0 = atmospheric path
    real p_intake_pa = 0.0;        // manifold (plenum) pressure target [Pa]; <= 0 = use `throttle`
    real throttle = 0.0;           // throttle plate fraction [0, 1] (only when p_intake_pa <= 0)
    real lambda = 1.0;             // air-fuel equivalence ratio of the next charges
    // Spark timing [deg ATDC, negative = before TDC], used when spark_given.
    // Start of burn = spark + the desc's ignition_delay_deg; without a spark
    // the model's MBT start of burn (ca50_mbt_deg_atdc through the Wiebe).
    real spark_deg_atdc = 0.0;
    bool spark_given = false;      // false: ignore spark_deg_atdc, burn at MBT
    std::uint32_t flags = 0;       // EngineInputFlags
};

}  // namespace ps_audio
