// ps/vehicle/curve1d.h — Curve1D, a piecewise-linear table (e.g. damper
// force vs. velocity, camber vs. suspension travel, or engine torque vs.
// rpm). Split out of vehicle_desc.h (PHYS-033 P4 I2 commit 5a) so
// ps/drivetrain/powertrain_desc.h (TorqueMapEngineDesc::wot_torque_nm_vs_rpm)
// can use it without including vehicle_desc.h, which itself includes
// powertrain_desc.h (VehicleDesc::powertrain) - including vehicle_desc.h
// from powertrain_desc.h would be circular.

#pragma once

#include "ps/types.h"

#include <vector>

namespace ps::vehicle {

// Points must be sorted by x (ascending); evaluate() clamps outside the
// given range rather than extrapolating.
struct Curve1D {
    std::vector<real> x;
    std::vector<real> y;

    [[nodiscard]] real evaluate(real xv) const {
        if (x.empty()) return 0.0;
        if (xv <= x.front()) return y.front();
        if (xv >= x.back()) return y.back();
        for (std::size_t i = 1; i < x.size(); ++i) {
            if (xv <= x[i]) {
                const real t = (xv - x[i - 1]) / (x[i] - x[i - 1]);
                return y[i - 1] + t * (y[i] - y[i - 1]);
            }
        }
        return y.back();
    }
};

} // namespace ps::vehicle
