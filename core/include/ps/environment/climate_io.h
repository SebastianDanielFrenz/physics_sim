// ps/environment/climate_io.h - loader for physics_sim.climate/1
// (docs/data_formats/climate.md; weather plan 7.1), W1 subset: location, sun,
// calendar and seasons. Like every ps loader it rejects unknown keys.
//
// Lives in ps_environment_io (nlohmann PRIVATE), not in ps_core's io/, so a
// game server needs neither Jolt nor the vehicle code.

#pragma once

#include "ps/environment/climate_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>

namespace ps::io {

// Reads the file, resolves its `extends` chain (load_json_with_extends), then
// parse_climate_json. Throws std::runtime_error with the file name and the
// offending field.
environment::ClimateDesc load_climate_json(const std::filesystem::path& path);

// Parses an already merged JSON object; `origin` prefixes error messages.
environment::ClimateDesc parse_climate_json(const nlohmann::json& root, const std::string& origin);

} // namespace ps::io
