// ps/environment/surface_conditions_io.h - loader for
// physics_sim.surface_conditions/1 (docs/data_formats/surface_conditions.md;
// weather plan 7.4, W5). Keyed by surface NAME (the names of
// data/surfaces/surfaces.json); supports `extends` (a game ships its own
// override). Rejects unknown keys; the keys of later increments (texture,
// drainage, friction, crr, soil, canopy, abrasiveness, maintenance) are
// rejected as reserved.
//
// Lives in ps_environment_io (nlohmann PRIVATE), like climate_io.h.

#pragma once

#include "ps/environment/surface_condition_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>

namespace ps::io {

// Reads the file, resolves its `extends` chain, then parse_surface_conditions_json.
// Throws std::runtime_error naming the file and the offending field.
environment::SurfaceConditionsDesc load_surface_conditions_json(const std::filesystem::path& path);

// Parses an already merged JSON object; `origin` prefixes error messages.
environment::SurfaceConditionsDesc parse_surface_conditions_json(const nlohmann::json& root, const std::string& origin);

} // namespace ps::io
