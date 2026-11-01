// ps/machine/work.h — what a tool did, as swept ground.
//
// A working tool writes one quad per tick: its edge at the tick's start and end.
// The tick only appends to a buffer sized at setup. Between ticks World::drain_work
// hands the quads over ordered by (tick, vehicle, tool), and work_coverage turns them
// into each cell's covered share. The soil never changes inside a tick.
//
// WorkLedger gathers those shares over a pass. A farm applies a cell once, with the
// whole pass's coverage: ps::field applies a thin strip as f x coverage and repeats
// as 1 - (1 - f)^coverage, so a thousand strips applied one by one would not add up
// to one pass.

#pragma once

#include "ps/types.h"
#include "ps/world/ids.h"

#include <cstdint>
#include <vector>

namespace ps::machine {

struct WorkPoint {
    real x = 0.0, y = 0.0;
};

struct WorkQuad {
    std::uint64_t tick = 0;
    VehicleId vehicle{};
    std::uint32_t tool = 0; // index in the vehicle's machine; its desc names the operation
    WorkPoint left_start, right_start, right_end, left_end;
    real depth_m = 0.0;   // mean over the tick
    real speed_m_s = 0.0; // along the tool, mean over the tick
    real quality = 1.0;   // 0..1, see tool_quality
};

// The world's cells: cell (ix, iy) is index ix + nx * iy and covers
// [x0 + ix * cell_m, x0 + (ix + 1) * cell_m) x [y0 + iy * cell_m, ...).
struct WorkGrid {
    real x0 = 0.0, y0 = 0.0;
    real cell_m = 0.5;
    int nx = 0, ny = 0;
};

struct CellCoverage {
    int cell = 0;
    real coverage = 0.0; // share of the cell's area
};

// The area of the quad per cell, ascending cell, the part outside the grid dropped. Appends to `out`. A tool that
// pivots crosses its own edge: the quad then is two triangles and both count as worked.
void work_coverage(const WorkQuad& quad, const WorkGrid& grid, std::vector<CellCoverage>& out);

// The swept area of the quad in square metres, whatever a grid would cut off.
[[nodiscard]] real work_area(const WorkQuad& quad);

// One cell of one tool's pass.
struct WorkCell {
    int cell = 0;
    VehicleId vehicle{};
    std::uint32_t tool = 0;
    real coverage = 0.0;
    real depth_m = 0.0; // coverage-weighted mean
    real quality = 0.0; // coverage-weighted mean
};

// Per (cell, vehicle, tool). add() in drain order; take() gives the cells in ascending (cell, vehicle, tool) and empties
// the ledger. Sim thread, between ticks: this one may allocate.
class WorkLedger {
public:
    explicit WorkLedger(const WorkGrid& grid) : grid_(grid) {}
    void add(const WorkQuad& quad);
    [[nodiscard]] std::vector<WorkCell> take();
    [[nodiscard]] bool empty() const { return cells_.empty(); }

private:
    WorkGrid grid_;
    std::vector<WorkCell> cells_;
    std::vector<CellCoverage> scratch_;
};

} // namespace ps::machine
