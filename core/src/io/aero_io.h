#pragma once
#include "ps/aero/aerodynamics.h"
#include <nlohmann/json_fwd.hpp>
namespace ps::io { aero::Desc read_aero(const nlohmann::json&); }
