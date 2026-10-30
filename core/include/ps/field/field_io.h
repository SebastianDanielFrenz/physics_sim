// ps/field/field_io.h - loaders for physics_sim.soil/1 and physics_sim.crop/1
// (docs/data_formats/soil.md, crop.md). They support `extends` (the shared
// merge-patch rule of ps_environment_io), reject unknown keys, validate the
// descriptor, and refuse a parameter without a source tag: every number in a
// block must have an entry in that block's "source" object whose text starts
// with [C] cited, [D] derived, [M] measured or [AS] assumed.
//
// Lives in ps_field_io (nlohmann PRIVATE), like ps_environment_io.

#pragma once

#include "ps/field/crop_desc.h"
#include "ps/field/soil_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>

namespace ps::io {

// Reads the file, resolves its `extends` chain, then parses. Throws
// std::runtime_error naming the file and the offending field.
field::SoilDesc load_soil_json(const std::filesystem::path& path);
field::SoilDesc parse_soil_json(const nlohmann::json& root, const std::string& origin);
field::CropDesc load_crop_json(const std::filesystem::path& path);
field::CropDesc parse_crop_json(const nlohmann::json& root, const std::string& origin);

} // namespace ps::io
