// core/src/environment/tyre_profile.cpp - see ps/environment/tyre_profile_desc.h.

#include "ps/environment/tyre_profile_desc.h"

#include <cmath>

namespace ps::environment {

const char* tyre_class_name(TyreClass c) {
    switch (c) {
    case TyreClass::Slick: return "slick";
    case TyreClass::Summer: return "summer";
    case TyreClass::AllSeason: return "all_season";
    case TyreClass::Winter: return "winter";
    case TyreClass::WinterStudded: return "winter_studded";
    case TyreClass::OffroadAT: return "offroad_at";
    case TyreClass::MudTerrain: return "mud_terrain";
    case TyreClass::AgriculturalLug: return "agricultural_lug";
    }
    return "summer";
}

bool parse_tyre_class(const std::string& text, TyreClass& out) {
    if (text == "slick") out = TyreClass::Slick;
    else if (text == "summer") out = TyreClass::Summer;
    else if (text == "all_season") out = TyreClass::AllSeason;
    else if (text == "winter") out = TyreClass::Winter;
    else if (text == "winter_studded") out = TyreClass::WinterStudded;
    else if (text == "offroad_at") out = TyreClass::OffroadAT;
    else if (text == "mud_terrain") out = TyreClass::MudTerrain;
    else if (text == "agricultural_lug") out = TyreClass::AgriculturalLug;
    else return false;
    return true;
}

std::string tyre_profile_validation_error(const TyreProfileDesc& d) {
    if (!(d.tread_new_mm > 0.0)) return "tread_new_mm must be > 0";
    if (!(d.tread_min_mm >= 0.0 && d.tread_min_mm <= d.tread_new_mm))
        return "tread_min_mm must be in [0, tread_new_mm]";
    if (!(d.groove_void_ratio >= 0.0 && d.groove_void_ratio <= 1.0)) return "groove_void_ratio must be in [0, 1]";
    if (!(d.inflation_pressure_kpa > 0.0)) return "inflation_pressure_kpa must be > 0";
    if (d.siping != "none" && d.siping != "low" && d.siping != "high")
        return "siping must be \"none\", \"low\" or \"high\"";
    const auto positive = [](real v) { return v > 0.0 && std::isfinite(v); };
    if (!positive(d.condition_factors.dry) || !positive(d.condition_factors.damp) ||
        !positive(d.condition_factors.wet) || !positive(d.condition_factors.wet_grass))
        return "condition_factors must all be > 0";
    if (!positive(d.tread_exponents.wet) || !positive(d.tread_exponents.snow_packed))
        return "tread_exponents must be > 0";
    return {};
}

} // namespace ps::environment
