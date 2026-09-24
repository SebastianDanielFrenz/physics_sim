// core/src/vehicle/tyre_model.cpp — ITyreModel::evaluate_with_slope's
// default implementation (PHYS-033 P4 I2 plan section 5.7): central finite
// difference on kappa, fixed step 1e-6, around evaluate(). A model (e.g.
// Pacejka - pacejka_tyre_model.cpp) may override this with an analytic
// derivative; models that don't get this default.

#include "ps/vehicle/tyre_model.h"

namespace ps::vehicle {

TyreForcesWithSlope ITyreModel::evaluate_with_slope(const TyreSlipInput& in) const {
    TyreForcesWithSlope out;
    out.forces = evaluate(in);

    constexpr real h = 1e-6;
    TyreSlipInput plus = in;
    plus.kappa += h;
    TyreSlipInput minus = in;
    minus.kappa -= h;
    const TyreForces f_plus = evaluate(plus);
    const TyreForces f_minus = evaluate(minus);
    out.dfx_dkappa = (f_plus.fx - f_minus.fx) / (real(2.0) * h);
    return out;
}

} // namespace ps::vehicle
