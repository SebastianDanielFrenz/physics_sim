// ps/drivetrain/map_pressure_index.h - engine garage P7d: the y coordinate at which a simulated engine's
// p_im-axis tables (ve, pmep_excess_pa, theta_kl_deg) are read. Header-only; the ONE implementation the runtime
// (SimulatedEngine), the dyno's grid probe and the tests share.
//
// Without a wide-open-throttle curve (SimulatedEngineMapsDesc::wot_p_im_pa empty: every file map, every lumped or
// compressor-fed generated map) the index is the absolute manifold pressure: y = p, dy/dp = 1, dy/drpm = 0, and
// callers keep their old lookup bit for bit (they branch on has_wot_rows()).
//
// With the curve (generated maps of an ambient-fed pipe engine, generator v4), the tables' last row (y_top, 101325 Pa)
// holds the WOT state of each rpm, whose plenum pressure p_wot(rpm) varies with rpm, and the rows below it are the
// author's grid rows (all below every p_wot). The pressure is first referred to the generation ambient (ISA):
//     q = p * (101325 / p_amb)                                          (the identity at ISA)
// then the band between the last author row p_last and p_wot(rpm) is stretched onto [p_last, y_top]:
//     y = q                                                   for q <= p_last
//     y = p_last + (q - p_last) * (y_top - p_last) / (p_wot(rpm) - p_last)   above (exactly y_top at q == p_wot(rpm))
// so at ISA the runtime reads every author node at its own pressure and every WOT node at its own p_wot exactly, and
// a WOT plenum at another ambient reads the WOT row too. Above p_wot the same line continues (the tables clamp at
// y_top). PROVISIONAL (progress file D1): referring part-throttle rows to ambient is an approximation either way.
#pragma once

#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/types.h"

namespace ps::drivetrain {

struct MapPressureIndex {
    real y = 0.0;       // table coordinate [Pa]
    real dy_dp = 1.0;   // d y / d p_im at fixed rpm and ambient
    real dy_drpm = 0.0; // d y / d rpm at fixed p_im (through p_wot(rpm)); the tables' own d/dx is added by the caller
};

[[nodiscard]] inline bool has_wot_rows(const SimulatedEngineMapsDesc& m) { return !m.wot_p_im_pa.x.empty(); }

// p_wot(rpm) with Curve1D's clamping, EXACT at a knot (Curve1D::evaluate is only within rounding there), and its slope
// with Table2D's convention (an interior knot takes the segment ending there, 0 at/beyond either end).
inline real wot_pressure_at(const vehicle::Curve1D& c, real rpm, real* slope) {
    *slope = real(0.0);
    const std::size_t n = c.x.size();
    if (rpm <= c.x.front()) return c.y.front();
    if (rpm >= c.x.back()) return c.y.back();
    for (std::size_t i = 1; i < n; ++i) {
        if (rpm <= c.x[i]) {
            const real dx = c.x[i] - c.x[i - 1];
            *slope = (c.y[i] - c.y[i - 1]) / dx;
            if (rpm == c.x[i]) return c.y[i];
            const real t = (rpm - c.x[i - 1]) / dx;
            return c.y[i - 1] + t * (c.y[i] - c.y[i - 1]);
        }
    }
    return c.y.back();
}

// Precondition (SimulatedEngine validates it): with WOT rows, ve.y has >= 2 rows and every wot_p_im_pa value lies
// strictly above ve.y[size - 2].
[[nodiscard]] inline MapPressureIndex map_pressure_index(const SimulatedEngineMapsDesc& m, real rpm, real p, real p_amb) {
    MapPressureIndex r;
    if (!has_wot_rows(m)) {
        r.y = p;
        return r;
    }
    const real k = kIsaAirPressurePa / p_amb;
    const real q = p * k;
    const std::size_t n = m.ve.y.size();
    const real p_last = m.ve.y[n - 2];
    const real y_top = m.ve.y[n - 1];
    if (q <= p_last) {
        r.y = q;
        r.dy_dp = k;
        return r;
    }
    real dqw_drpm = real(0.0);
    const real qw = wot_pressure_at(m.wot_p_im_pa, rpm, &dqw_drpm);
    const real span = qw - p_last;
    const real s = (y_top - p_last) / span;
    r.y = q == qw ? y_top : p_last + (q - p_last) * s;
    r.dy_dp = s * k;
    r.dy_drpm = -(q - p_last) * s / span * dqw_drpm;
    return r;
}

} // namespace ps::drivetrain
