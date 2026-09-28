// ps/drivetrain/isa_air.h — ISA (ISO 2533:1975) sea-level standard
// atmosphere plus the dry-air gas constant, as three named, header-only
// constants. Extracted (ps_audio A1, out/audio_plan.md D-11) from
// core/src/drivetrain/engine_simulated.h's own static constexpr members of
// the same values: engine_cycle moved into its own ps_engine_cycle CMake
// target (linked by ps_core AND, later, ps_audio, which never links
// ps_core - see CLAUDE.md's "Vehicle system"/"Drivetrain" sections once A1
// lands), and core/src/engine_cycle/map_generator.cpp's own extraction
// ("computed exactly as SimulatedEngine::build_maps does", its own header
// comment) used to reach into engine_simulated.h directly for these 3
// values - a ps_core-INTERNAL header (core/src/drivetrain, not part of
// ps_core's public include/ps surface), which would have made
// ps_engine_cycle depend on ps_core (backwards: ps_core links
// ps_engine_cycle, not the other way round). Splitting just these 3 literal
// constants out to a small header-only file both sides include keeps a
// single source of truth (SimulatedEngine::kGasConstant/kIsaPressurePa/
// kIsaTemperatureK now simply alias these) without engine_cycle needing
// anything else from core/src/drivetrain.
//
// ps::AmbientSample (ps/world/ambient.h) carries the SAME ISA sea-level
// pressure/temperature as its struct's DEFAULT MEMBER VALUES, but that type
// represents a live/settable ambient reading (and the graph engine ignores
// it entirely, owner ruling Q10) - a different concept from these fixed
// named constants, so it is not reused here to avoid coupling the two.

#pragma once

#include "ps/types.h"

namespace ps::drivetrain {

inline constexpr real kIsaAirGasConstantJKgK = 287.05; // R, J/(kg*K), dry air
inline constexpr real kIsaAirPressurePa = 101325.0;    // ISO 2533:1975 sea level
inline constexpr real kIsaAirTemperatureK = 288.15;    // ISO 2533:1975 sea level (15 degC)

} // namespace ps::drivetrain
