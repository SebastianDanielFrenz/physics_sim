// ps/drivetrain/table2d.h — Table2D, a bilinear-interpolated, clamped 2D
// lookup table (e.g. fuel flow g/s over engine rpm x load). Public,
// header-only (no consumer yet - PHYS-033 P4 I7a plan section on the
// coming full-simulated-engine fuel map), no algorithm code lives outside
// this file. Mirrors ps/vehicle/curve1d.h's Curve1D: same bracket-search
// style, same "clamp at the edges rather than extrapolate" semantics, same
// convention for the slope at a knot and beyond the domain - see value()/
// dvalue_dx()/dvalue_dy() below for the exact rules, matched line-for-line
// against Curve1D's own evaluate() and (components.cpp's/engine_torque_map.
// cpp's shared) curve_slope_at().

#pragma once

#include "ps/types.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace ps::drivetrain {

// x (rows) and y (columns) must each be strictly ascending and have at
// least 2 points; validate() checks this (see below - there is no loader
// for this type yet, so the check lives on the type itself rather than in
// an ps::io/*.cpp free function the way Curve1D's require_ascending does).
//
// `values` is ROW-MAJOR over (x, y): values[ix * y.size() + iy] is the
// table's value at (x[ix], y[iy]) - i.e. values[x_index][y_index] if it
// were a nested array, matching the fuel-map JSON's own
// fuel_flow_g_s[x_index][y_index] layout exactly.
struct Table2D {
    std::vector<real> x;
    std::vector<real> y;
    std::vector<real> values;

    [[nodiscard]] real at(std::size_t ix, std::size_t iy) const { return values[ix * y.size() + iy]; }

    // Bracket for one axis's evaluate()/slope: the [lo, hi] node pair a
    // query value falls between, plus the interpolation fraction t in
    // [0, 1] such that xv == lerp(xs[lo], xs[hi], t). Reproduces Curve1D::
    // evaluate()'s own bracket search exactly (same "xv <= xs[i]" test, so
    // an interior knot picks the segment ENDING at that knot - t == 1 -
    // never the segment starting there): xv <= xs.front() clamps to the
    // degenerate {0, 0} bracket (t == 0, so only the front value is used);
    // xv >= xs.back() clamps to the degenerate {n, n} bracket at the last
    // node; otherwise the loop finds the first xs[i] >= xv and returns
    // {i - 1, i, t}.
    struct Bracket {
        std::size_t lo, hi;
        real t;
    };
    [[nodiscard]] static Bracket bracket_for(const std::vector<real>& xs, real xv) {
        if (xv <= xs.front()) return {0, 0, real(0.0)};
        const std::size_t n = xs.size() - 1;
        if (xv >= xs.back()) return {n, n, real(0.0)};
        for (std::size_t i = 1; i < xs.size(); ++i) {
            if (xv <= xs[i]) {
                const real dx = xs[i] - xs[i - 1];
                const real t = dx > real(0.0) ? (xv - xs[i - 1]) / dx : real(0.0);
                return {i - 1, i, t};
            }
        }
        return {n, n, real(0.0)}; // unreachable given xv < xs.back() above and xs ascending
    }

    // Bilinear value, clamped at every edge (never extrapolates) exactly
    // like Curve1D::evaluate(): a query outside [x.front(), x.back()] x
    // [y.front(), y.back()] reads the nearest edge/corner's row or column
    // as a flat (zero-slope) extension. Bracket both axes, then the usual
    // two-stage lerp (along x at each of the y-bracket's two rows, then
    // along y) - degenerates cleanly to Curve1D-style 1D clamped-linear
    // interpolation whenever either bracket is degenerate (lo == hi).
    [[nodiscard]] real value(real xv, real yv) const {
        const Bracket bx = bracket_for(x, xv);
        const Bracket by = bracket_for(y, yv);
        const real v00 = at(bx.lo, by.lo);
        const real v10 = at(bx.hi, by.lo);
        const real v01 = at(bx.lo, by.hi);
        const real v11 = at(bx.hi, by.hi);
        const real vy0 = v00 + bx.t * (v10 - v00); // along x, at the y-bracket's low row
        const real vy1 = v01 + bx.t * (v11 - v01); // along x, at the y-bracket's high row
        return vy0 + by.t * (vy1 - vy0);           // along y between those two
    }

    // d(value)/dx of the same clamped bilinear surface value() evaluates -
    // zero outside the x domain (matching Curve1D's own curve_slope_at()
    // convention: AT or beyond x.front()/x.back() the slope is defined as
    // 0, same as truly outside, even though the interior segment touching
    // that node has a nonzero slope approached from the other side).
    // Otherwise: within the bilinear cell (bx.lo, bx.hi) x (by.lo, by.hi),
    // v(x, y) is linear in x at each fixed y, so d(v)/dx is the (y-lerped)
    // difference across the x-cell divided by the x-cell's width - constant
    // across the cell in x, linearly blended in y via by.t.
    [[nodiscard]] real dvalue_dx(real xv, real yv) const {
        if (xv <= x.front() || xv >= x.back()) return real(0.0);
        const Bracket bx = bracket_for(x, xv); // interior: bx.lo < bx.hi guaranteed by the guard above
        const Bracket by = bracket_for(y, yv);
        const real v00 = at(bx.lo, by.lo);
        const real v10 = at(bx.hi, by.lo);
        const real v01 = at(bx.lo, by.hi);
        const real v11 = at(bx.hi, by.hi);
        const real d_lo = v10 - v00; // change across the x-cell at the y-bracket's low row
        const real d_hi = v11 - v01; // ... at the high row
        const real dx = x[bx.hi] - x[bx.lo];
        return (d_lo + by.t * (d_hi - d_lo)) / dx;
    }

    // d(value)/dy — the y-axis mirror of dvalue_dx above (same zero-outside-
    // the-domain convention, same "linear in y at fixed x, x-lerp the two
    // rates" derivation).
    [[nodiscard]] real dvalue_dy(real xv, real yv) const {
        if (yv <= y.front() || yv >= y.back()) return real(0.0);
        const Bracket bx = bracket_for(x, xv);
        const Bracket by = bracket_for(y, yv); // interior: by.lo < by.hi guaranteed by the guard above
        const real v00 = at(bx.lo, by.lo);
        const real v10 = at(bx.hi, by.lo);
        const real v01 = at(bx.lo, by.hi);
        const real v11 = at(bx.hi, by.hi);
        const real d_lo = v01 - v00; // change across the y-cell at the x-bracket's low column
        const real d_hi = v11 - v10; // ... at the high column
        const real dy = y[by.hi] - y[by.lo];
        return (d_lo + bx.t * (d_hi - d_lo)) / dy;
    }

    // Returns an empty string when this table is well-formed, otherwise a
    // description of the first problem found (x/y each >= 2 points,
    // finite, strictly ascending; values.size() == x.size() * y.size() and
    // every value finite). No standalone loader exists yet for this type
    // (unlike Curve1D, which is validated by ps::io::vehicle_io.cpp's
    // free-function require_ascending() once a consumer's JSON is parsed),
    // so the check lives on the type itself; a future fuel-map loader
    // should call this rather than duplicating it.
    [[nodiscard]] std::string validate() const {
        if (x.size() < 2) return "Table2D: x must have at least 2 points";
        if (y.size() < 2) return "Table2D: y must have at least 2 points";
        for (real v : x) {
            if (!std::isfinite(v)) return "Table2D: x values must be finite";
        }
        for (real v : y) {
            if (!std::isfinite(v)) return "Table2D: y values must be finite";
        }
        for (std::size_t i = 1; i < x.size(); ++i) {
            if (!(x[i] > x[i - 1])) return "Table2D: x values must be strictly ascending";
        }
        for (std::size_t i = 1; i < y.size(); ++i) {
            if (!(y[i] > y[i - 1])) return "Table2D: y values must be strictly ascending";
        }
        if (values.size() != x.size() * y.size()) return "Table2D: values.size() must equal x.size() * y.size()";
        for (real v : values) {
            if (!std::isfinite(v)) return "Table2D: values must be finite";
        }
        return {};
    }
};

} // namespace ps::drivetrain
