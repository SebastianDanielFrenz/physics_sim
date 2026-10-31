// ps/io/hitch_io.h — JSON loading for hitch classes (physics_sim.hitch_class/1,
// docs/data_formats/hitch.md). "extends" works as in every part file; unknown
// keys are rejected. nlohmann::json stays confined to hitch_io.cpp.

#pragma once

#include "ps/machine/hitch_class.h"

#include <string>

namespace ps::io {

[[nodiscard]] machine::HitchClassDesc load_hitch_class_json(const std::string& path);

} // namespace ps::io
