#pragma once
#include "ps/aero/aerodynamics.h"
#include <nlohmann/json_fwd.hpp>
#include <string>
namespace ps::io { aero::Desc read_aero(const nlohmann::json&, const std::string& base_dir={}); }
