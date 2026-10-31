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
#include "ps/field/management_desc.h"
#include "ps/field/nutrient_desc.h"
#include "ps/field/soil_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace ps::io {

// Reads the file, resolves its `extends` chain, then parses. Throws
// std::runtime_error naming the file and the offending field.
field::SoilDesc load_soil_json(const std::filesystem::path& path);
field::SoilDesc parse_soil_json(const nlohmann::json& root, const std::string& origin);
field::CropDesc load_crop_json(const std::filesystem::path& path);
field::CropDesc parse_crop_json(const nlohmann::json& root, const std::string& origin);
// physics_sim.field_material/1 (docs/data_formats/field_material.md): one fertiliser, manure or liming material.
field::MaterialDesc load_material_json(const std::filesystem::path& path);
field::MaterialDesc parse_material_json(const nlohmann::json& root, const std::string& origin);
// physics_sim.field_methods/1: the application methods (one file, a list).
std::vector<field::MethodDesc> load_methods_json(const std::filesystem::path& path);
std::vector<field::MethodDesc> parse_methods_json(const nlohmann::json& root, const std::string& origin);

// physics_sim.weed_group/1 (docs/data_formats/weed_group.md) and physics_sim.herbicide/1 (herbicide.md).
field::WeedGroupDesc load_weed_group_json(const std::filesystem::path& path);
field::WeedGroupDesc parse_weed_group_json(const nlohmann::json& root, const std::string& origin);
field::HerbicideDesc load_herbicide_json(const std::filesystem::path& path);
field::HerbicideDesc parse_herbicide_json(const nlohmann::json& root, const std::string& origin);

// physics_sim.operation/1 (docs/data_formats/operation.md): one OperationEffect record. Names in the file (crop,
// herbicide, material, method) are resolved against the palettes of the field they will be applied to.
struct OperationPalette {
    std::vector<std::string> crops, herbicides, materials, methods;   // index + 1 = id in the field's palette
};
field::OperationEffect load_operation_json(const std::filesystem::path& path, const OperationPalette& palette);
field::OperationEffect parse_operation_json(const nlohmann::json& root, const std::string& origin, const OperationPalette& palette);

} // namespace ps::io
