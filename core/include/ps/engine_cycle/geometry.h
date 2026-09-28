// engine_cycle/geometry.h - slider-crank with piston-pin offset (PHYS-033 P4
// I7c C15a, i7c plan section 3 "Geometry"). INTERNAL (ps::engine_cycle).
//
// phi = geometric crank angle from the cylinder axis, r = stroke/2, l = rod,
// e = pin offset:
//   x(phi)  = r*cos(phi) + sqrt(l^2 - (r*sin(phi) - e)^2)   pin height above the crank axis
//   x_TDC   = sqrt((l+r)^2 - e^2),  phi_TDC = asin(e/(l+r))
//   x_BDC   = sqrt((l-r)^2 - e^2),  phi_BDC = pi + asin(e/(l-r))
//   V       = V_c + A_p*(x_TDC - x(phi)),  V_c = V_d_true/(CR - 1)
// The API angle theta is measured from TRUE TDC (theta = 0 at phi_TDC), so
// phi = theta*pi/180 + phi_TDC. asin is evaluated once at construction via
// ps::math; per evaluation only ps::math::sincos and sqrt are used.

#pragma once

#include "ps/types.h"

namespace ps::engine_cycle {

inline constexpr real kPi = 3.14159265358979323846;
inline constexpr real kDegToRad = kPi / 180.0;

struct CrankGeometry {
    real bore = 0.0;
    real stroke = 0.0;       // nominal, 2r
    real r = 0.0;
    real l = 0.0;
    real e = 0.0;
    real area_piston = 0.0;  // pi*B^2/4
    real phi_tdc = 0.0;      // rad
    real phi_bdc = 0.0;      // rad
    real x_tdc = 0.0;
    real x_bdc = 0.0;
    real true_stroke = 0.0;  // x_tdc - x_bdc
    real v_d = 0.0;          // true swept volume, one cylinder
    real v_c = 0.0;          // clearance volume
    real theta_bdc_deg = 0.0;  // BDC relative to true TDC (180 without offset)

    // Throws std::invalid_argument on a non-physical geometry.
    static CrankGeometry make(real bore_m, real stroke_m, real rod_m, real pin_offset_m, real compression_ratio);

    struct Eval {
        real x;         // pin height
        real v;         // m^3
        real dv_ddeg;   // m^3 per crank degree
    };
    // theta_deg: crank angle from true TDC, any value (reduced mod 360 by
    // the caller for precision - sincos is periodic anyway).
    [[nodiscard]] Eval eval(real theta_deg) const;
};

} // namespace ps::engine_cycle
