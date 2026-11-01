#include "ps/machine/work.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <tuple>

namespace ps::machine {

namespace {

struct Triangle {
    WorkPoint a, b, c;
};

[[nodiscard]] real doubled_area(const WorkPoint& a, const WorkPoint& b, const WorkPoint& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Where segments p0-p1 and q0-q1 cross strictly inside both, if they do.
[[nodiscard]] bool crossing(const WorkPoint& p0, const WorkPoint& p1, const WorkPoint& q0, const WorkPoint& q1, WorkPoint& at) {
    const real d = (p1.x - p0.x) * (q1.y - q0.y) - (p1.y - p0.y) * (q1.x - q0.x);
    if (d == 0.0) return false;
    const real t = ((q0.x - p0.x) * (q1.y - q0.y) - (q0.y - p0.y) * (q1.x - q0.x)) / d;
    const real u = ((q0.x - p0.x) * (p1.y - p0.y) - (q0.y - p0.y) * (p1.x - p0.x)) / d;
    if (!(t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0)) return false;
    at = WorkPoint{p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y)};
    return true;
}

// The quad as triangles that do not overlap: two, whichever way it folds. A bow tie (the tool pivots between its ends)
// splits at the crossing of its two sides; a dart splits along the diagonal through its reflex corner.
[[nodiscard]] std::size_t split(const WorkQuad& q, std::array<Triangle, 2>& out) {
    const WorkPoint& a = q.left_start;
    const WorkPoint& b = q.right_start;
    const WorkPoint& c = q.right_end;
    const WorkPoint& d = q.left_end;
    WorkPoint x;
    if (crossing(a, d, b, c, x)) {
        out[0] = Triangle{a, b, x};
        out[1] = Triangle{c, d, x};
        return 2;
    }
    if (crossing(a, b, c, d, x)) {
        out[0] = Triangle{b, c, x};
        out[1] = Triangle{d, a, x};
        return 2;
    }
    const real abd = doubled_area(a, b, d), bcd = doubled_area(b, c, d);
    if ((abd >= 0.0) == (bcd >= 0.0)) {
        out[0] = Triangle{a, b, d};
        out[1] = Triangle{b, c, d};
    } else {
        out[0] = Triangle{a, b, c};
        out[1] = Triangle{a, c, d};
    }
    return 2;
}

// Clips a convex polygon to one side of an axis-aligned line. At most one vertex more each time.
constexpr std::size_t kMaxPolygon = 8;
using Polygon = std::array<WorkPoint, kMaxPolygon>;

template <typename Inside, typename Intersect>
[[nodiscard]] std::size_t clip(const Polygon& in, std::size_t n, Polygon& out, Inside inside, Intersect intersect) {
    std::size_t m = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const WorkPoint& p = in[i];
        const WorkPoint& q = in[(i + 1) % n];
        const bool pin = inside(p), qin = inside(q);
        if (pin) out[m++] = p;
        if (pin != qin) out[m++] = intersect(p, q);
    }
    return m;
}

[[nodiscard]] real polygon_area(const Polygon& p, std::size_t n) {
    real sum = 0.0;
    for (std::size_t i = 1; i + 1 < n; ++i) sum += doubled_area(p[0], p[i], p[i + 1]);
    return 0.5 * std::abs(sum);
}

void add_triangle(const Triangle& t, const WorkGrid& g, std::vector<CellCoverage>& out) {
    const real min_x = std::min({t.a.x, t.b.x, t.c.x}), max_x = std::max({t.a.x, t.b.x, t.c.x});
    const real min_y = std::min({t.a.y, t.b.y, t.c.y}), max_y = std::max({t.a.y, t.b.y, t.c.y});
    const int ix0 = std::max(0, static_cast<int>(std::floor((min_x - g.x0) / g.cell_m)));
    const int ix1 = std::min(g.nx - 1, static_cast<int>(std::floor((max_x - g.x0) / g.cell_m)));
    const int iy0 = std::max(0, static_cast<int>(std::floor((min_y - g.y0) / g.cell_m)));
    const int iy1 = std::min(g.ny - 1, static_cast<int>(std::floor((max_y - g.y0) / g.cell_m)));
    const real cell_area = g.cell_m * g.cell_m;
    const auto at_x = [](real x) {
        return [x](const WorkPoint& p, const WorkPoint& q) { return WorkPoint{x, p.y + (q.y - p.y) * (x - p.x) / (q.x - p.x)}; };
    };
    const auto at_y = [](real y) {
        return [y](const WorkPoint& p, const WorkPoint& q) { return WorkPoint{p.x + (q.x - p.x) * (y - p.y) / (q.y - p.y), y}; };
    };
    Polygon base{t.a, t.b, t.c};
    for (int iy = iy0; iy <= iy1; ++iy) {
        const real y_lo = g.y0 + g.cell_m * static_cast<real>(iy), y_hi = y_lo + g.cell_m;
        Polygon row1, row;
        const std::size_t n1 = clip(base, 3, row1, [&](const WorkPoint& p) { return p.y >= y_lo; }, at_y(y_lo));
        const std::size_t n2 = clip(row1, n1, row, [&](const WorkPoint& p) { return p.y <= y_hi; }, at_y(y_hi));
        for (int ix = ix0; ix <= ix1 && n2 >= 3; ++ix) {
            const real x_lo = g.x0 + g.cell_m * static_cast<real>(ix), x_hi = x_lo + g.cell_m;
            Polygon cell1, cell;
            const std::size_t m1 = clip(row, n2, cell1, [&](const WorkPoint& p) { return p.x >= x_lo; }, at_x(x_lo));
            const std::size_t m2 = clip(cell1, m1, cell, [&](const WorkPoint& p) { return p.x <= x_hi; }, at_x(x_hi));
            if (m2 < 3) continue;
            const real area = polygon_area(cell, m2);
            if (area > 0.0) out.push_back(CellCoverage{ix + g.nx * iy, area / cell_area});
        }
    }
}

} // namespace

void work_coverage(const WorkQuad& quad, const WorkGrid& grid, std::vector<CellCoverage>& out) {
    const std::size_t first = out.size();
    std::array<Triangle, 2> triangles;
    const std::size_t n = split(quad, triangles);
    for (std::size_t i = 0; i < n; ++i) add_triangle(triangles[i], grid, out);
    // Both triangles can land in one cell: merge, ascending.
    std::sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(), [](const CellCoverage& a, const CellCoverage& b) { return a.cell < b.cell; });
    std::size_t kept = first;
    for (std::size_t i = first; i < out.size(); ++i) {
        if (kept > first && out[kept - 1].cell == out[i].cell) {
            out[kept - 1].coverage += out[i].coverage;
        } else {
            out[kept++] = out[i];
        }
    }
    out.resize(kept);
}

real work_area(const WorkQuad& quad) {
    std::array<Triangle, 2> triangles;
    const std::size_t n = split(quad, triangles);
    real area = 0.0;
    for (std::size_t i = 0; i < n; ++i) area += 0.5 * std::abs(doubled_area(triangles[i].a, triangles[i].b, triangles[i].c));
    return area;
}

void WorkLedger::add(const WorkQuad& quad) {
    scratch_.clear();
    work_coverage(quad, grid_, scratch_);
    const auto key = [](const WorkCell& c) { return std::make_tuple(c.cell, c.vehicle.index, c.tool); };
    for (const CellCoverage& c : scratch_) {
        WorkCell probe;
        probe.cell = c.cell;
        probe.vehicle = quad.vehicle;
        probe.tool = quad.tool;
        const auto it = std::lower_bound(cells_.begin(), cells_.end(), probe, [&](const WorkCell& a, const WorkCell& b) { return key(a) < key(b); });
        WorkCell& cell = (it != cells_.end() && key(*it) == key(probe)) ? *it : *cells_.insert(it, probe);
        cell.depth_m += quad.depth_m * c.coverage;
        cell.quality += quad.quality * c.coverage;
        cell.coverage += c.coverage;
    }
}

std::vector<WorkCell> WorkLedger::take() {
    std::vector<WorkCell> out;
    out.swap(cells_);
    for (WorkCell& c : out) {
        c.depth_m /= c.coverage;
        c.quality /= c.coverage;
    }
    return out;
}

} // namespace ps::machine
