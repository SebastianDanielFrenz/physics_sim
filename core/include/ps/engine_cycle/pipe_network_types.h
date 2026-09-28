// engine_cycle/pipe_network_types.h - small descriptor types shared by the
// finite-volume pipe network (pipe_network.h, ps_audio A2) and the linear
// waveguide network (waveguide_network.h, ps_audio A3). Split out of
// pipe_network.h in A3 so both headers can use them without an include cycle;
// the definitions are unchanged.

#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

// End corrections as a multiple of the end's radius.
inline constexpr real kUnflangedEndCorrection = 0.6133;  // Levine & Schwinger 1948, low ka
inline constexpr real kFlangedEndCorrection = 0.8216;    // Rayleigh; Norris & Sheng 1989
inline constexpr real kDefaultPipeCflLimit = 0.8;        // out/audio_plan.md section 4

enum class EndCorrection : std::uint8_t { None, Unflanged, Flanged };

struct GasStateDesc {
    real p_pa = 101325.0;
    real t_k = 293.15;
    real x_b = 0.0;  // burned mass fraction
};

} // namespace ps::engine_cycle
