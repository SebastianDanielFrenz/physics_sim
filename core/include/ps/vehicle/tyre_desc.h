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

struct RollingResistanceAnchors {
    real rated_speed_kmh = 0.0;  // V_r, the tyre's speed-symbol rating
    real crr_base = 0.0;         // becomes qsy1
    real fraction = 0.0;         // f, in (0,1)
    real factor = 1.0;           // crr(f*V_r)/crr_base, >= 1
    real factor_at_rated = 1.0;  // crr(V_r)/crr_base, > factor
};
enum class RollingResistanceSpeedLaw { FourthPower, Quadratic };

struct TyreDesc {
    // Source coefficients/anchors are retained when selecting a runtime law.
    RollingResistanceSpeedLaw rolling_resistance_speed_law = RollingResistanceSpeedLaw::FourthPower;
    RollingResistanceAnchors rolling_resistance_anchors{}; // rated_speed_kmh == 0: explicit QSY/.tir
    real rolling_resistance_quadratic = 0.0; // coefficient multiplying (v/reference)^2
    real rolling_resistance_reference_speed_m_s = 0.0;

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
    // MF 6.1's linear-speed term: qsy3*|Vx/v0| (tyre-rolling-resistance work,
    // out/tyre_crr_progress.md). Default 0 = every file that never mentions it
    // is bit-identical to before. Normally DERIVED from a tyre file's optional
    // "rolling_resistance" anchor block (ps::vehicle::derive_rolling_resistance,
    // tyre_model.h), raw values stay legal for measured data.
    real qsy3 = 0.0;
    real qsy4 = 0.0002; // speed^4 term coefficient

    // Parking-slope creep fix (defect D1): wheel_instance.cpp's pre_solve
    // used to scale rolling resistance by sign(vx_contact), which never
    // settles at true standstill - vx_contact is a substep-sampled contact
    // velocity that is never exactly 0.0 (float/double noise, plus Jolt
    // itself integrating position with the end-of-tick velocity while the
    // tyre samples substep-start velocities), so the sign flipped every
    // substep: a period-2 limit cycle at 960 Hz that, together with a
    // separate Jolt defect (see CLAUDE.md's "Jolt backend" section on
    // IRigidBackend::set_integrate_sub_threshold_rotation), made a parked,
    // handbraked car creep uphill. Fix: rolling resistance now ramps
    // LINEARLY from 0 at vx_contact = 0 to its full sign(vx)*crr_total*fz
    // magnitude at |vx_contact| = rolling_resistance_v_eps
    // (std::clamp(vx_contact / rolling_resistance_v_eps, -1, 1) in place of
    // sign(vx_contact) - wheel_instance.cpp's pre_solve), i.e. a regularised
    // Coulomb drag - a real hysteresis-loss force must fade continuously to
    // zero with speed, not switch sign discontinuously through it. Stability:
    // this adds an effective viscous rate c ~= crr*Fz/rolling_resistance_v_eps
    // per wheel (~780 N*s/m for a car wheel at the default) - h*sum(c)/m is
    // far below the explicit-integration stability bound for the tick rates
    // this project uses.
    real rolling_resistance_v_eps = 0.05; // m/s; the sign(vx) ramp's half-width


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
    //
    // PHYS-036 (2026-09-24): qdz1/qdz2 are NEGATIVE (opposite sign to
    // pdy1/pdy2 above) so Mz0 = -t*Fy0 with trail t > 0 - Pacejka's own
    // aligning-moment sign convention (ch. 4). The un-fixed defaults
    // (+0.10/-0.02) gave Mz0 the SAME sign as Fy0, an aligning moment that
    // steers further into a turn instead of resisting - see
    // data/tyres/passenger_car_225_45r17.json's "aligning" source note and
    // core/src/vehicle/pacejka_tyre_model.cpp's header comment. qbz1 = 43.4
    // (not the un-fixed 8.0) makes the curve peak at ~4 deg of slip, matching
    // this same source note's target range (real self-aligning-torque curves
    // peak around 2-6 deg, well before Fy0's own ~9 deg peak at fz0) - qbz1
    // alone sets the peak's alpha location at fixed qcz1/qez1 (same B-scaling
    // property the Fx0/Fy0 curves share).
    real qbz1 = 43.4, qbz2 = 0.0, qbz3 = 0.0;
    real qcz1 = 1.15;
    real qdz1 = -0.10, qdz2 = 0.02;
    real qez1 = -1.0, qez2 = 0.0;
    real qhz1 = 0.0, qhz2 = 0.0, qhz3 = 0.0;

    // --- overturning moment Mx(Fz, gamma, Fy) - simplified linear form ---
    real qsx1 = 0.0, qsx2 = 0.05, qsx3 = 0.02;

    // --- low-speed lateral carcass damping (PHYS-033 section 4.3, I1 plan
    // section 3.6/4) ---
    //
    // A viscous force -c*fade*vy applied to the wheel's lateral contact
    // velocity, faded out above low_speed_damping_vx (on TOTAL contact-patch
    // speed, sqrt(vx^2+vy^2) - PHYS-024 found this load-bearing: a wheel
    // with near-zero ROLLING speed but a real, fast lateral slide/spin must
    // not get this term). c = 2*ratio*sqrt(k_lat*m_ref), with the lateral
    // deflection stiffness k_lat = pky1*fz0/relaxation_length_y (at the
    // NOMINAL load) and m_ref = fz0/9.81 - anchored to the tyre's FIXED
    // fz0, never the live/instantaneous Fz (PHYS-024's positive-feedback
    // lesson: tying this to live Fz turned a hard-acceleration weight-
    // transfer transient into a runaway spin-out).
    //
    // This is NOT a compensation for VehicleInstance::step_tick's one-tick
    // chassis-state delay (that delay is fully removed by the I1 W1 shadow
    // chassis - see CLAUDE.md's "Vehicle system" section) - it is real
    // carcass/relaxation damping the tyre model's own relaxation ODE does
    // not otherwise supply at vanishing rolling speed (the longitudinal
    // channel is saved by rolling resistance, a genuine drag along the same
    // axis that ramps continuously up to its full Coulomb magnitude within
    // rolling_resistance_v_eps of standstill rather than vanishing outright
    // like this term does; the lateral channel has none). See
    // wheel_instance.cpp's low_speed_lateral_damping_force for the explicit-
    // integration stability bound (PHYS-033 section 4.3) that caps a safe
    // ratio, and CLAUDE.md for how the shipped default was measured.
    // 1.5 (PHYS-033 I1 commit E): RETAINED at the commit B value after a
    // {0, 0.1, ..., 1.5} sweep (run after the I1 W1 shadow chassis removed
    // the numerical delay this term originally compensated for) found no
    // SMALLER value that keeps every existing acceptance test passing.
    // Two existing tests turned out to depend on this ratio in the fade
    // band (vx < low_speed_damping_vx), and both are non-monotonic/
    // near-chaotic in it over [0, 1.5], not smoothly improving as the plan
    // anticipated:
    //   - the full-throttle asphalt standing-start test's peak longitudinal
    //     slip (max|kappa|, existing bound < 0.3) sits around 0.70-0.80 for
    //     most ratios in [0, 1.5] but drops to ~0.12 at isolated points
    //     (1.0, 1.4, 1.5) - a threshold in the standing-start transient's
    //     alpha/Gxa/kappa feedback loop (residual lateral slip discounts
    //     the combined-slip Gxa weighting on Fx, which the closed-loop
    //     wheel-spin dynamics answers with more kappa), not a smooth
    //     function of the ratio.
    //   - the parking-slope test's 30 s creep (existing bound < 1 mm) is
    //     similarly non-monotonic (measured 0.4-2.6 mm across the same
    //     sweep, passing at some ratios and failing at others with no
    //     visible trend).
    //   Of the ratios tried, only 1.5 (and, by luck, a couple of isolated
    //   points like 1.0/1.4 for the first test alone) passed the first
    //   test, and only 1.5 passed BOTH simultaneously. See
    //   test_vehicle_standstill_wiggle.cpp's "zeta sweep diagnostic" for
    //   the full table. Given two acceptance-tested scenarios are not
    //   smooth in this parameter here, further trial-and-error narrowing is
    //   not principled - the spec's own suggested next step (an eigenvalue/
    //   linearised stability tool, PHYS-033 section 4.3's "ps_linearize")
    //   would characterise this properly; not built in I1 (out of scope,
    //   flagged to the coordinator).
    real low_speed_damping_ratio = 1.5; // dimensionless; 0 disables the term entirely
    real low_speed_damping_vx = 2.0;    // m/s; fade reaches 0 at this TOTAL contact speed
};

} // namespace ps::vehicle
