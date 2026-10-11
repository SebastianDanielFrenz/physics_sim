// ps/io/electronics_io.h - JSON loading for electronics part files (engine garage P8, A15/A16,
// docs/data_formats/electronics.md). A part is grafted onto a vehicle as a `controllers` entry of type
// "electronics" ({"id", "type": "electronics", "ref": path}) or by the user layer
// (VehicleLoadOptions::electronics). nlohmann::json stays confined to electronics_io.cpp.

#pragma once

#include "ps/drivetrain/controller_desc.h"

#include <string>
#include <vector>

namespace ps::io {

enum class ElectronicsKind {
    LaunchControl, // launch_control: a second rev limit (two-step)
    AssistFitment, // assist_fitment: marks assists as fitted (fits)
};

struct ElectronicsPart {
    ElectronicsKind kind = ElectronicsKind::LaunchControl;
    std::string name, source;
    drivetrain::LaunchControlDesc launch; // kind == LaunchControl
    std::vector<std::string> fits;        // kind == AssistFitment; each one of kElectronicsAssistNames
};

// The assist names an assist_fitment part may list (and a vehicle's `fitted` switches cover).
inline constexpr const char* kElectronicsAssistNames[] = {"auto_clutch", "auto_blip", "auto_shift", "traction_control", "anti_lag"};

// Loads a physics_sim.electronics/1 file (its "extends" chain resolved, depth 8, cycles rejected). Throws
// std::runtime_error on a missing/wrong format or kind, an unknown key, a non-finite or non-positive rpm /
// max_speed_mps, an empty channel, an unknown or duplicate assist name.
ElectronicsPart load_electronics_json(const std::string& path);

} // namespace ps::io
