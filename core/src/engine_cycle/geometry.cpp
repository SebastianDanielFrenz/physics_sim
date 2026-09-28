#include "ps/engine_cycle/geometry.h"

#include "ps/math/transcendental.h"

#include <cmath>
#include <stdexcept>

namespace ps::engine_cycle {

CrankGeometry CrankGeometry::make(real bore_m, real stroke_m, real rod_m, real pin_offset_m,
                                  real compression_ratio) {
    if (!(bore_m > 0.0) || !(stroke_m > 0.0) || !(rod_m > 0.0)) {
        throw std::invalid_argument("engine_cycle: bore, stroke and rod must be > 0");
    }
    if (!(compression_ratio > 1.0)) throw std::invalid_argument("engine_cycle: compression_ratio must be > 1");
    CrankGeometry g;
    g.bore = bore_m;
    g.stroke = stroke_m;
    g.r = 0.5 * stroke_m;
    g.l = rod_m;
    g.e = pin_offset_m;
    if (!(g.l - g.r > std::fabs(g.e))) {
        throw std::invalid_argument("engine_cycle: rod must exceed crank radius + |pin offset|");
    }
    g.area_piston = 0.25 * kPi * bore_m * bore_m;
    g.phi_tdc = ps::math::asin(g.e / (g.l + g.r));
    g.phi_bdc = kPi + ps::math::asin(g.e / (g.l - g.r));
    g.x_tdc = std::sqrt((g.l + g.r) * (g.l + g.r) - g.e * g.e);
    g.x_bdc = std::sqrt((g.l - g.r) * (g.l - g.r) - g.e * g.e);
    g.true_stroke = g.x_tdc - g.x_bdc;
    g.v_d = g.area_piston * g.true_stroke;
    g.v_c = g.v_d / (compression_ratio - 1.0);
    g.theta_bdc_deg = (g.phi_bdc - g.phi_tdc) / kDegToRad;
    return g;
}

CrankGeometry::Eval CrankGeometry::eval(real theta_deg) const {
    const real phi = theta_deg * kDegToRad + phi_tdc;
    real s = 0.0;
    real c = 0.0;
    ps::math::sincos(phi, s, c);
    const real q = r * s - e;
    const real root = std::sqrt(l * l - q * q);
    const real x = r * c + root;
    const real dx_dphi = -r * s - q * r * c / root;
    Eval out;
    out.x = x;
    out.v = v_c + area_piston * (x_tdc - x);
    out.dv_ddeg = -area_piston * dx_dphi * kDegToRad;
    return out;
}

} // namespace ps::engine_cycle
