// ps/environment/tyre_profile_io.h - loader for physics_sim.tyre_profile/1
// (docs/data_formats/tyre.md; weather plan 7.5, W6). One profile per file;
// supports `extends` (a game ships its own variant).
//
// Lives in ps_environment_io (nlohmann PRIVATE), like surface_conditions_io.h.

#pragma once

#include "ps/environment/tyre_profile_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>

namespace ps::io {

// Reads the file, resolves its `extends` chain, then parse_tyre_profile_json.
// Throws std::runtime_error naming the file and the offending field.
environment::TyreProfileDesc load_tyre_profile_json(const std::filesystem::path& path);

// Parses an already merged JSON object; `origin` prefixes error messages.
environment::TyreProfileDesc parse_tyre_profile_json(const nlohmann::json& root, const std::string& origin);

} // namespace ps::io
