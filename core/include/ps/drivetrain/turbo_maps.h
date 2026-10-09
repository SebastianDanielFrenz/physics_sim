#pragma once
#include "ps/drivetrain/table2d.h"
#include "ps/vehicle/curve1d.h"
#include <cstdint>
#include <memory>
#include "ps/drivetrain/turbo_stage_maps.h"
namespace ps::drivetrain {
struct TurboPerformanceMaps {
    Table2D head_pa; // normalized shaft speed x normalized total flow
    vehicle::Curve1D compressor_temperature_ratio; // pressure ratio -> outlet/inlet T
    std::shared_ptr<const TurboStageMaps> stages; // optional offline geometry solver output
    std::uint64_t content_hash = 0;
};
}
