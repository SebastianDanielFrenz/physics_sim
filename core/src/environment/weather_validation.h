// core/src/environment/weather_validation.h - INTERNAL: meaning checks of the
// W3 weather parts of a ClimateDesc, called by climate_validation_error.
#pragma once

#include "ps/environment/climate_desc.h"

#include <string>

namespace ps::environment::detail {

// First problem as "climate: <field> ..." or an empty string.
std::string climate_weather_validation_error(const ClimateDesc& d);

} // namespace ps::environment::detail
