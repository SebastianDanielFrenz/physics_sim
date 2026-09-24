// core/src/drivetrain/clutch.h — the I3 dry clutch (PHYS-033 P4 I3 plan
// section 6.2). A new class, framework-agnostic like engine_torque_map.h -
// not yet wired into Powertrain (that lands in commit 6, "the switch").
// Stateless on purpose: a clutch's kinetic/static capacity depends only on
// THIS substep's pedal/auto-clutch inputs, not on any history of its own
// (the static-vs-kinetic selection is the NETWORK's own row_stuck_prev, not
// something this class needs to track).

#pragma once

#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <algorithm>
#include <utility>

namespace ps::drivetrain {

class Clutch {
public:
    struct Desc {
        real capacity_nm = 0.0;                       // KINETIC capacity at engagement 1
        real static_factor = 1.15;                     // static = static_factor * kinetic (>= 1)
        vehicle::Curve1D engagement_vs_pedal;           // pedal 0 (released) .. 1 (pressed) -> engagement 1 .. 0
        real in_inertia_kgm2 = 0.0;                     // port "in" (pressure plate; usually folded into the engine)
        real out_inertia_kgm2 = 0.0;                    // port "out" (driven disc)
    };

    struct Result {
        real engagement = 0.0;     // e = min(e_pedal, e_auto)
        real cap_static = 0.0;     // static_factor * cap_kinetic
        real cap_kinetic = 0.0;    // e * capacity_nm
    };

    explicit Clutch(Desc desc) : desc_(std::move(desc)) {}

    // e_auto: 1.0 if no auto-clutch controller is active (TCU default).
    [[nodiscard]] Result pre_solve(real pedal, real e_auto) const {
        const real e_pedal = std::clamp(desc_.engagement_vs_pedal.evaluate(pedal), real(0.0), real(1.0));
        const real e = std::min(e_pedal, std::clamp(e_auto, real(0.0), real(1.0)));
        Result r;
        r.engagement = e;
        r.cap_kinetic = e * desc_.capacity_nm;
        r.cap_static = desc_.static_factor * r.cap_kinetic;
        return r;
    }

    // Ledger contribution for this substep (plan section 6.2: "clutch_loss
    // += -h*v'*lambda"), given the caller's own row_velocity/row_lambda
    // reads (post-solve, from whichever ShaftNetwork friction row a future
    // commit assigns this clutch).
    [[nodiscard]] static real clutch_loss_delta(real h, real row_velocity_new, real row_lambda) {
        return -h * row_velocity_new * row_lambda;
    }

    [[nodiscard]] const Desc& desc() const { return desc_; }

private:
    Desc desc_;
};

} // namespace ps::drivetrain
