// tests/unit/test_table2d.cpp — ps::drivetrain::Table2D (PHYS-033 P4 I7a
// commit C1): a bilinear-interpolated, clamped 2D lookup table with no
// consumer yet (a later commit uses it for an engine fuel map). Mirrors
// ps/vehicle/curve1d.h's own clamping/knot-slope conventions in 2D - see
// table2d.h's own header comment for the exact rules this file pins.

#include "ps/drivetrain/table2d.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

using ps::drivetrain::Table2D;

namespace {

// A deliberately NON-SQUARE (3 x-points x 4 y-points), non-uniformly-spaced
// grid with a genuine bilinear cross term (value is NOT separable into
// f(x) + g(y) - see the derivation below), so a row/column-major mixup or
// an accidental x <-> y swap anywhere in value()/dvalue_dx()/dvalue_dy()
// produces either an out-of-bounds read (the axes have different lengths)
// or a value nowhere near the hand-computed ones below, rather than
// silently matching by coincidence.
//
// Node values follow value(ix, iy) = 10*ix + 100*iy + 7*ix*iy (ix/iy are
// grid INDICES, not coordinates - the x/y coordinates themselves are
// non-uniformly spaced, so this is not simply "linear in x and y"). Table:
//
//   x \ y   0.0   1.0   4.0    6.0
//   0.0       0   100   200    300
//   2.0      10   117   224    331
//   5.0      20   134   248    362
//
// e.g. at(1,1) = 10*1 + 100*1 + 7*1*1 = 117; at(2,2) = 10*2 + 100*2 + 7*2*2 = 248.
Table2D make_grid() {
    Table2D t;
    t.x = {0.0, 2.0, 5.0};
    t.y = {0.0, 1.0, 4.0, 6.0};
    t.values = {
        0.0, 100.0, 200.0, 300.0,  // x = 0.0
        10.0, 117.0, 224.0, 331.0, // x = 2.0
        20.0, 134.0, 248.0, 362.0, // x = 5.0
    };
    return t;
}

} // namespace

TEST_CASE("Table2D is exact at every node", "[drivetrain][table2d]") {
    const Table2D t = make_grid();
    for (std::size_t ix = 0; ix < t.x.size(); ++ix) {
        for (std::size_t iy = 0; iy < t.y.size(); ++iy) {
            CHECK(t.value(t.x[ix], t.y[iy]) == t.at(ix, iy));
        }
    }
}

TEST_CASE("Table2D bilinear interpolation matches hand-computed values", "[drivetrain][table2d]") {
    const Table2D t = make_grid();

    // Cell midpoint: x = 1.0 is the midpoint of the [0.0, 2.0] x-segment
    // (tx = (1.0 - 0.0) / (2.0 - 0.0) = 0.5); y = 0.5 is the midpoint of the
    // [0.0, 1.0] y-segment (ty = (0.5 - 0.0) / (1.0 - 0.0) = 0.5). Corners:
    // v00 = at(0,0) = 0, v10 = at(1,0) = 10, v01 = at(0,1) = 100,
    // v11 = at(1,1) = 117. Bilinear value = v00 + tx*(v10-v00) +
    // ty*(v01-v00) + tx*ty*(v11-v10-v01+v00)
    //   = 0 + 0.5*10 + 0.5*100 + 0.5*0.5*(117-10-100+0)
    //   = 5 + 50 + 0.25*7 = 5 + 50 + 1.75 = 56.75.
    // Under a transposed (axis-swapped) row-major layout this cell doesn't
    // even exist the same way (x has 3 points, y has 4), so a swap bug
    // reads outright the wrong pair of rows/columns here.
    CHECK(t.value(1.0, 0.5) == Catch::Approx(56.75).epsilon(1e-12));

    // Off-centre interior point, away from both a midpoint and any knot:
    // x = 3.0 is 1/3 of the way through [2.0, 5.0] (tx = (3-2)/(5-2) = 1/3);
    // y = 3.0 is 2/3 of the way through [1.0, 4.0] (ty = (3-1)/(4-1) = 2/3).
    // Corners: v00 = at(1,1) = 117, v10 = at(2,1) = 134, v01 = at(1,2) = 224,
    // v11 = at(2,2) = 248. Bilinear value = v00 + tx*(v10-v00) +
    // ty*(v01-v00) + tx*ty*(v11-v10-v01+v00)
    //   = 117 + (1/3)*17 + (2/3)*107 + (1/3)*(2/3)*7
    //   = (1053 + 51 + 642 + 14) / 9 = 1760/9 (exactly, as ninths).
    // This point's tx != ty (1/3 vs 2/3), so swapping the two axes'
    // fractions anywhere in the implementation would change the result -
    // unlike the symmetric midpoint case above, this assertion cannot pass
    // by accident under such a swap.
    CHECK(t.value(3.0, 3.0) == Catch::Approx(1760.0 / 9.0).epsilon(1e-12));
}

TEST_CASE("Table2D clamps outside the grid on every side and at every corner", "[drivetrain][table2d]") {
    const Table2D t = make_grid();

    // Sides: one axis clamped, the other an interior (non-knot) value -
    // reads the clamped edge's row/column exactly as value() at the edge
    // node would, per node-value hand-computation above.
    // x below range, y interior in [1,4] (ty = 2/3): row x=0.0 gives
    // at(0,1)=100, at(0,2)=200 -> lerp = 100 + (2/3)*100 = 500/3.
    CHECK(t.value(-100.0, 3.0) == Catch::Approx(500.0 / 3.0).epsilon(1e-12));
    // x above range, y interior in [1,4]: row x=5.0 gives at(2,1)=134,
    // at(2,2)=248 -> lerp = 134 + (2/3)*114 = 134 + 76 = 210 exactly.
    CHECK(t.value(1000.0, 3.0) == Catch::Approx(210.0).epsilon(1e-12));
    // y below range, x interior in [2,5] (tx = 1/3): column y=0.0 gives
    // at(1,0)=10, at(2,0)=20 -> lerp = 10 + (1/3)*10 = 40/3.
    CHECK(t.value(3.0, -100.0) == Catch::Approx(40.0 / 3.0).epsilon(1e-12));
    // y above range, x interior in [2,5]: column y=6.0 gives at(1,3)=331,
    // at(2,3)=362 -> lerp = 331 + (1/3)*31 = 1024/3.
    CHECK(t.value(3.0, 1000.0) == Catch::Approx(1024.0 / 3.0).epsilon(1e-12));

    // Corners: both axes clamped -> exactly the corner node's value.
    CHECK(t.value(-100.0, -100.0) == t.at(0, 0));
    CHECK(t.value(-100.0, 1000.0) == t.at(0, 3));
    CHECK(t.value(1000.0, -100.0) == t.at(2, 0));
    CHECK(t.value(1000.0, 1000.0) == t.at(2, 3));
}

TEST_CASE("Table2D dvalue_dx/dvalue_dy match a central finite difference", "[drivetrain][table2d]") {
    const Table2D t = make_grid();

    // Interior, non-knot point inside the (x in [2,5], y in [1,4]) cell -
    // the one cell with a nonzero cross term (7), so this also exercises
    // the cross-term contribution to both partials, not just a locally
    // separable (planar) patch.
    const double px = 3.5;
    const double py = 2.0;

    // Central finite difference: since value() is exactly bilinear (linear
    // in x at any fixed y, and vice versa) within a cell, the SECOND
    // derivative along either axis is identically zero there, so central
    // differencing carries no truncation error at all - only floating-
    // point rounding, which h = 1e-3 (well inside both the [2,5] and
    // [1,4] cells, nowhere near a knot) keeps far below the 1e-9 relative
    // bound asked for below.
    const double h = 1e-3;
    const double fd_dx = (t.value(px + h, py) - t.value(px - h, py)) / (2.0 * h);
    const double fd_dy = (t.value(px, py + h) - t.value(px, py - h)) / (2.0 * h);

    const double dx = t.dvalue_dx(px, py);
    const double dy = t.dvalue_dy(px, py);

    CHECK(dx == Catch::Approx(fd_dx).epsilon(1e-9));
    CHECK(dy == Catch::Approx(fd_dy).epsilon(1e-9));

    // Cross-check against the hand-derived analytic slopes for this cell:
    // v00=at(1,1)=117, v10=at(2,1)=134, v01=at(1,2)=224, v11=at(2,2)=248,
    // dx_cell = 5-2 = 3, dy_cell = 4-1 = 3, tx = (3.5-2)/3 = 0.5,
    // ty = (2.0-1)/3 = 1/3.
    // dvalue_dx = (d_lo + ty*(d_hi-d_lo)) / dx_cell, d_lo = v10-v00 = 17,
    //   d_hi = v11-v01 = 24 -> (17 + (1/3)*7) / 3 = (58/3) / 3 = 58/9.
    // dvalue_dy = (e_lo + tx*(e_hi-e_lo)) / dy_cell, e_lo = v01-v00 = 107,
    //   e_hi = v11-v10 = 114 -> (107 + 0.5*7) / 3 = 110.5 / 3.
    CHECK(dx == Catch::Approx(58.0 / 9.0).epsilon(1e-12));
    CHECK(dy == Catch::Approx(110.5 / 3.0).epsilon(1e-12));
}

TEST_CASE("Table2D slope is zero outside the domain, on either axis, at either edge", "[drivetrain][table2d]") {
    const Table2D t = make_grid();

    // Beyond the domain, and AT the boundary node itself (Curve1D's own
    // curve_slope_at() convention: <= front or >= back reads 0, matching
    // the flat clamped extension value() itself produces there).
    CHECK(t.dvalue_dx(-100.0, 2.0) == 0.0);
    CHECK(t.dvalue_dx(t.x.front(), 2.0) == 0.0);
    CHECK(t.dvalue_dx(1000.0, 2.0) == 0.0);
    CHECK(t.dvalue_dx(t.x.back(), 2.0) == 0.0);

    CHECK(t.dvalue_dy(3.0, -100.0) == 0.0);
    CHECK(t.dvalue_dy(3.0, t.y.front()) == 0.0);
    CHECK(t.dvalue_dy(3.0, 1000.0) == 0.0);
    CHECK(t.dvalue_dy(3.0, t.y.back()) == 0.0);
}

TEST_CASE("Table2D validate() rejects each bad input", "[drivetrain][table2d]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    {
        Table2D t = make_grid();
        CHECK(t.validate().empty()); // sanity: the fixture itself is valid
    }
    {
        Table2D t = make_grid();
        t.x = {0.0}; // too few x points
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.y = {0.0}; // too few y points
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.x = {0.0, 2.0, 2.0}; // not strictly ascending (equal)
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.x = {0.0, 2.0, 1.0}; // not strictly ascending (decreasing)
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.y = {0.0, 1.0, 4.0, 3.0}; // not strictly ascending
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.values.pop_back(); // values.size() != x.size() * y.size()
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.x[1] = nan; // non-finite x
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.y[1] = inf; // non-finite y
        CHECK_FALSE(t.validate().empty());
    }
    {
        Table2D t = make_grid();
        t.values[5] = nan; // non-finite value
        CHECK_FALSE(t.validate().empty());
    }
}
