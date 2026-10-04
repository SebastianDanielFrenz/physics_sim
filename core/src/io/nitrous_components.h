// core/src/io/nitrous_components.h - INTERNAL: the vehicle-component builders
// vehicle_io.cpp calls for "type": "n2o_bottle" / "nitrous_kit" components
// (N2O plan N1, D3). The public file loaders are ps/io/nitrous_io.h.
#pragma once

#include "ps/drivetrain/nitrous_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace ps::io::detail {

// {"id", "type": "n2o_bottle", "ref": "<bottle file>", "initial_kg"?, "pressure"?}:
// the part file's default, then the component's optional overrides (bottom-up).
// `base_dir` is the vehicle file's directory; `ctx` prefixes every error.
drivetrain::N2oBottleDesc build_n2o_bottle_component(const nlohmann::json& cj, const std::string& base_dir, const std::string& ctx);

// {"id", "type": "nitrous_kit", "ref": "<kit file>", "controller"?: "<controller file>"}.
drivetrain::NitrousKitDesc build_nitrous_kit_component(const nlohmann::json& cj, const std::string& base_dir, const std::string& ctx);

} // namespace ps::io::detail
