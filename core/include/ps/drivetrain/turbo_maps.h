#pragma once
#include "ps/drivetrain/table2d.h"
#include "ps/vehicle/curve1d.h"
#include <cstdint>
namespace ps::drivetrain {
struct TurboPerformanceMaps {
    Table2D head_pa; // normalized shaft speed x normalized total flow
    vehicle::Curve1D compressor_temperature_ratio; // pressure ratio -> outlet/inlet T
    std::uint64_t content_hash = 0;
};
}
