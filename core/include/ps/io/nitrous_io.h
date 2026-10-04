// ps/io/nitrous_io.h - JSON loading for the nitrous-oxide part files (N2O plan
// N0/N1; PLAN.md D6: data files, not code). nlohmann::json stays confined to
// nitrous_io.cpp. Every loader throws std::runtime_error with a message
// naming the file/field on a malformed, unreadable or out-of-range input;
// unknown keys are rejected (new-block rule), a "format" mismatch is a hard
// error, and any key ending in "_pa" or "_psi" is rejected by name (the files
// are in bar, as turbo.md).
//
// Relative paths inside a part file (a bottle's "oxidizer", a kit's
// "default_controller") resolve against that file's own directory.

#pragma once

#include "ps/drivetrain/nitrous_desc.h"
#include "ps/drivetrain/oxidizer_desc.h"

#include <string>

namespace ps::io {

// Loads a physics_sim.oxidizer/1 file (data/oxidizers/*.json, docs/data_formats/oxidizer.md).
drivetrain::OxidizerDesc load_oxidizer_json(const std::string& path);

// physics_sim.n2o_bottle/1 (data/n2o_bottles/*.json, docs/data_formats/nitrous.md): loads the oxidizer it
// names; initial_kg is the capacity.
drivetrain::N2oBottleDesc load_n2o_bottle_json(const std::string& path);

// physics_sim.nitrous_controller/1 (data/nitrous_controllers/*.json).
drivetrain::NitrousControllerDesc load_nitrous_controller_json(const std::string& path);

// physics_sim.nitrous_kit/1 (data/nitrous_kits/*.json). The kit's controller
// is `controller_override_path` when non-empty (the vehicle component's
// override, already resolved by the caller), else the file's
// "default_controller"; a kit with neither is rejected.
drivetrain::NitrousKitDesc load_nitrous_kit_json(const std::string& path, const std::string& controller_override_path = std::string());

} // namespace ps::io
