#pragma once
// Internal: the stage-map JSON -> TurboStageMaps parse shared by vehicle_io.cpp (performance_map) and turbo_stage_io.cpp.
#include "ps/drivetrain/turbo_stage_maps.h"

#include <nlohmann/json.hpp>

#include <memory>
#include <string>

namespace ps::io::detail {
// sj must already have passed require_format(..., "turbo_stage_maps", 1, ...). Throws std::runtime_error.
std::shared_ptr<drivetrain::TurboStageMaps> stage_maps_from_json(const nlohmann::json& sj, int units, const std::string& path);
} // namespace ps::io::detail
