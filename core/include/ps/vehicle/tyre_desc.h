// ps/vehicle/tyre_desc.h — Pacejka Magic Formula (MF 5.2/6.1 subset)
// parameter set (PLAN.md D6/5.1/P3).
//
// Naming follows Pacejka, "Tyre and Vehicle Dynamics", 3rd ed. (Butterworth-
// Heinemann, 2012), ch. 4's own coefficient names (P*/Q*/R* prefixes) so a
// standard MF ".tir" parameter file (see ps/io/tir_parser.h) maps onto this
// struct field-for-field. This is a SUBSET, not the full MF6.1/6.2 equation
// set - see core/src/vehicle/pacejka_tyre_model.cpp's header comment for
// exactly which terms are implemented and which are a documented, simplified
// stand-in (Mz0's own B/C/D/E curve instead of the full pneumatic-trail x Fy
// decomposition; Mx's reduced linear-in-camber form; no rBx3/rVy* camber-in-
// combined-slip extensions). PLAN.md P3 explicitly allows "published or
// plausible" data with the source noted (data/tyres/*.json carries that
// note) - this struct itself assigns no meaning to any specific numeric
// value.
//
// Pure data; no algorithm code (that lives in core/src/vehicle, internal,
// behind ps/vehicle/tyre_model.h's ITyreModel seam - PLAN.md 5.1: "separate
// tyre-model interface so a brush/TMeasy or soft-soil model can replace it
// per surface").

#pragma once

#include "ps/types.h"

#include <string>

namespace ps::vehicle {

struct TyreDesc {
    std::string name;
    std::string source; // required: paper/book/.tir provenance, PLAN.md section 8

    // --- [DIMENSION]/[VERTICAL] ---
    real unloaded_radius = 0.31;      // R0, m
    real width = 0.205;               // m (informational + CylinderShape width)
    real rim_radius = 0.2286;         // m (9in; informational only, P3 does not model the rim)
    real fz0 = 4000.0;                // nominal load, N
    real vertical_stiffness = 200000.0; // N/m; informational (Fz itself comes from the suspension model, not tyre carcass vertical deflection, in this phase's simplified wheel-contact model - see out/handover_p3.md decision 1)

    // --- rolling-resistance/reference speed ---
    real v0 = 16.7;   // reference speed, m/s (~60 km/h), for the QSY4 speed^4 term
    real qsy1 = 0.01; // baseline rolling-resistance coefficient (tyre's own contribution; surface crr multiplies this - see pacejka_tyre_model.cpp)
    real qsy2 = 0.0;  // Fx-dependent rolling resistance term
    real qsy4 = 0.0002; // speed^4 term coefficient

    // --- relaxation lengths (Pacejka ch. 7's transient model; see
    // out/handover_p3.md decision 2 for the exact ODE) ---
    real relaxation_length_x = 0.10; // m
    real relaxation_length_y = 0.15; // m

    // --- friction scaling (combined with the surface table's lambda_mu) ---
    real lambda_mux = 1.0;
    real lambda_muy = 1.0;

    // --- [LONGITUDINAL_COEFFICIENTS]: pure-slip Fx0(kappa, Fz, gamma) ---
    real pcx1 = 1.65;
    real pdx1 = 1.05, pdx2 = -0.10, pdx3 = 0.0;
    real pex1 = 0.10, pex2 = 0.0, pex3 = 0.0, pex4 = 0.0;
    real pkx1 = 18.0, pkx2 = 0.0, pkx3 = 0.05;
    real phx1 = 0.0, phx2 = 0.0;
    real pvx1 = 0.0, pvx2 = 0.0;

    // --- [LATERAL_COEFFICIENTS]: pure-slip Fy0(alpha, Fz, gamma) ---
    real pcy1 = 1.35;
    real pdy1 = 1.0, pdy2 = -0.12, pdy3 = -2.0;
    real pey1 = -0.6, pey2 = -0.1, pey3 = 0.0, pey4 = 0.0;
    real pky1 = 16.0, pky2 = 0.0;
    real phy1 = 0.0, phy2 = 0.0, phy3 = 0.0;
    real pvy1 = 0.0, pvy2 = 0.0, pvy3 = -0.3, pvy4 = 0.0;

    // --- combined-slip weighting (Gxa reduces Fx0 for nonzero alpha, Gyk
    // reduces Fy0 for nonzero kappa) ---
    real rbx1 = 12.0, rbx2 = -8.0, rcx1 = 1.0, rhx1 = 0.0;
    real rby1 = 9.0, rby2 = 5.0, rby3 = 0.0, rcy1 = 1.0, rhy1 = 0.0, rhy2 = 0.0;
    real rvy1 = 0.0, rvy2 = 0.0, rvy3 = 0.0, rvy4 = 20.0, rvy5 = 2.0, rvy6 = 0.0;

    // --- [ALIGNING_COEFFICIENTS]: Mz0(alpha, Fz, gamma) - simplified own
    // B/C/D/E curve, see this file's top comment ---
    real qbz1 = 8.0, qbz2 = 0.0, qbz3 = 0.0;
    real qcz1 = 1.15;
    real qdz1 = 0.10, qdz2 = -0.02;
    real qez1 = -1.0, qez2 = 0.0;
    real qhz1 = 0.0, qhz2 = 0.0, qhz3 = 0.0;

    // --- overturning moment Mx(Fz, gamma, Fy) - simplified linear form ---
    real qsx1 = 0.0, qsx2 = 0.05, qsx3 = 0.02;
};

} // namespace ps::vehicle
