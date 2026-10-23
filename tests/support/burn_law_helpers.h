// tests/support/burn_law_helpers.h - the burn law (vault PHYS-079) at a test's own IVC state: the measured-boundary-
// condition harnesses (knock calibration, burn law calibration) scale the cycle's IVC capture to the measured trapped air
// and evaluate the law on that scaled state, exactly as CycleStepper::burn_law_at_ivc does on its own capture.
#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/engine_cycle/burn_law.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"

namespace ps::test_support {

// The law's input at a cylinder state (m, u, mb) captured at crank angle crank_deg.
inline engine_cycle::BurnLawInput burn_law_input(const engine_cycle::CycleStepper& proto, real crank_deg, real m, real u,
                                                 real mb, real rpm, real lambda) {
    const real v = proto.model().geom.eval(crank_deg).v;
    const engine_cycle::CvState c = engine_cycle::cv_state(proto.model().gas, m, u, mb, v);
    engine_cycle::BurnLawInput in;
    in.rpm = rpm;
    in.lambda = lambda;
    in.t_ivc_k = c.t;
    in.p_ivc_pa = c.p;
    in.residual_fraction = c.x;
    in.volume_ratio = v / proto.model().geom.v_c;
    return in;
}

// The burn angles a desc runs at that state: the law (Standard) or the desc's constants (Fixed).
inline engine_cycle::BurnAngles burn_angles_at(const drivetrain::CycleModelDesc& c, const engine_cycle::CycleStepper& proto,
                                               real crank_deg, real m, real u, real mb, real rpm, real lambda) {
    if (c.burn_law == drivetrain::CycleModelDesc::BurnLawMode::Fixed) {
        engine_cycle::BurnAngles b;
        b.wiebe_duration_deg = c.burn_duration_deg;
        b.ignition_delay_deg = c.ignition_delay_deg;
        return b;
    }
    return engine_cycle::evaluate_burn_law(c.burn_law_params, c.wiebe_a, c.wiebe_m,
                                           burn_law_input(proto, crank_deg, m, u, mb, rpm, lambda));
}

} // namespace ps::test_support
