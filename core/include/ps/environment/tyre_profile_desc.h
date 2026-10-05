// ps/environment/tyre_profile_desc.h - descriptor of a tyre profile
// (weather plan 5.1, 7.5; W6): what a wheel needs to turn a surface condition
// into a friction factor. Loaded from physics_sim.tyre_profile/1
// (docs/data_formats/tyre_profile.md, ps/environment/tyre_profile_io.h).
//
// W6 uses the per-condition factors (dry/damp/wet/wet_grass), the tread depth
// limits and the reference inflation pressure; the tread-depth exponents are
// carried for W7 and the thermal/wear blocks are W9/W10 (the loader accepts
// `null` for them, an object is reserved).
//
// Lives in ps_environment (no ps_core dependency, like surface_condition_desc.h).

#pragma once

#include "ps/types.h"

#include <cstdint>
#include <string>

namespace ps::environment {

// Tyre class (plan 5.1). winter_studded and mud_terrain are reserved names.
enum class TyreClass : std::uint8_t {
    Slick,
    Summer,
    AllSeason,
    Winter,
    WinterStudded,
    OffroadAT,
    MudTerrain,
    AgriculturalLug,
};

// The multiplicative per-condition factors p[cond] of one profile relative to
// the reference summer tyre (plan 5.2). W6 uses dry/damp/wet/wet_grass; the
// W8/W9 conditions stay at 1.0.
struct TyreConditionFactors {
    real dry = 1.0;
    real damp = 1.0;
    real wet = 1.0;
    real wet_grass = 1.0;
};

// Tread-depth exponents of the tread factor (plan 5.2 step 4; W7 reads them).
struct TyreTreadExponents {
    real wet = 0.5;
    real snow_packed = 1.0;
};

struct TyreProfileDesc {
    std::string name;                      // file stem / reference name
    TyreClass tyre_class = TyreClass::Summer;
    real tread_new_mm = 8.0;               // > 0
    real tread_min_mm = 1.6;               // >= 0, <= tread_new_mm
    real groove_void_ratio = 0.3;          // [0, 1]
    std::string siping = "none";           // "none" | "low" | "high"
    real inflation_pressure_kpa = 220.0;   // > 0
    TyreConditionFactors condition_factors;
    TyreTreadExponents tread_exponents;
    std::string source;
};

const char* tyre_class_name(TyreClass c);
bool parse_tyre_class(const std::string& text, TyreClass& out);

// Validates a desc (ranges above). Returns an empty string when it is valid,
// else the problem (the loader turns it into std::runtime_error).
std::string tyre_profile_validation_error(const TyreProfileDesc& d);

} // namespace ps::environment
