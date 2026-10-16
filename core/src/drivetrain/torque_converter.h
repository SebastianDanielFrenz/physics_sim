// core/src/drivetrain/torque_converter.h — the hydrodynamic torque converter of an Automatic gearbox (automatic
// gearbox plan A1; descriptor ps::drivetrain::TorqueConverterDesc in powertrain_desc.h, which has the model). Pure
// and stateless: evaluate() maps (w_pump, w_turbine) to the two node torques plus the slope the Powertrain feeds the
// network as an implicit pump-turbine coupling. The lock-up clutch is a friction row owned by the Powertrain.
//
// Linearisation (why the converter is stable at any substep): the converter is a stiff function of the SLIP between
// pump and turbine near coupling (K rises steeply towards SR = 1). The Powertrain applies the explicit torques
// evaluated at omega_old plus an implicit coupling c on v = w_pump - w_turbine with the explicit part shifted by
// +-c*v_old, so the applied torque is T(omega_old) - c*(v_new - v_old): first-order exact along the slip direction,
// fully implicit, symmetric. c = |dT_pump/dw_turbine| (>= 0).

#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/types.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {

class TorqueConverter {
public:
    struct Result {
        real tau_pump = 0.0;    // torque ON the pump node (negative = load on the engine in forward flow)
        real tau_turbine = 0.0; // torque ON the turbine node
        real slip_slope = 0.0;  // c >= 0, N*m*s/rad on v = w_pump - w_turbine
        real speed_ratio = 0.0; // SR = w_turbine / w_pump in forward flow, w_pump / w_turbine in overrun (each in [-1, 1])
        bool overrun = false;
    };

    explicit TorqueConverter(const TorqueConverterDesc& d) : desc_(d) {
        last_x_ = desc_.k_factor.x.empty() ? real(0.0) : desc_.k_factor.x.back();
    }

    // Capacity factor C(SR) = 1 / K(SR)^2 in N*m / rpm^2; linear to 0 between the last K point and SR = 1.
    [[nodiscard]] real capacity(real sr) const {
        sr = std::clamp(sr, real(0.0), real(1.0));
        if (sr <= last_x_) {
            const real k = desc_.k_factor.evaluate(sr);
            return real(1.0) / (k * k);
        }
        const real k = desc_.k_factor.y.back();
        const real c_last = real(1.0) / (k * k);
        if (last_x_ >= real(1.0)) return real(0.0);
        return c_last * (real(1.0) - sr) / (real(1.0) - last_x_);
    }

    [[nodiscard]] real torque_ratio(real sr) const {
        return desc_.torque_ratio.evaluate(std::clamp(sr, real(0.0), real(1.0)));
    }

    [[nodiscard]] Result evaluate(real w_pump, real w_turbine) const {
        constexpr real kToRpm = real(60.0) / (real(2.0) * real(3.14159265358979323846));
        constexpr real kDsr = real(1e-3);
        Result r;
        const real ap = std::abs(w_pump), at = std::abs(w_turbine);
        if (ap >= at) {
            if (ap <= real(0.0)) return r; // both at rest
            const real sr = w_turbine / w_pump;
            const real src = std::max(sr, real(0.0));
            const real n = w_pump * kToRpm;
            const real cap = capacity(src);
            const real tp = cap * n * std::abs(n); // signed with the pump's direction
            r.tau_pump = -tp;
            r.tau_turbine = torque_ratio(src) * tp;
            r.speed_ratio = sr;
            // dT_p/dw_t = n|n| * C'(SR) / w_pump (finite difference on C, which may be piecewise linear).
            const real a = std::max(real(0.0), src - kDsr), b = std::min(real(1.0), src + kDsr);
            const real dc = (capacity(b) - capacity(a)) / (b - a);
            r.slip_slope = std::abs(n * std::abs(n) * dc / w_pump);
        } else {
            // Overrun: the turbine drives the pump (coasting); same capacity law with the roles swapped, no multiplication.
            const real sr = w_pump / w_turbine;
            const real src = std::max(sr, real(0.0));
            const real n = w_turbine * kToRpm;
            const real cap = capacity(src);
            const real t = cap * n * std::abs(n);
            r.tau_turbine = -t;
            r.tau_pump = t;
            r.speed_ratio = sr;
            r.overrun = true;
            const real a = std::max(real(0.0), src - kDsr), b = std::min(real(1.0), src + kDsr);
            const real dc = (capacity(b) - capacity(a)) / (b - a);
            r.slip_slope = std::abs(n * std::abs(n) * dc / w_turbine);
        }
        return r;
    }

    [[nodiscard]] const TorqueConverterDesc& desc() const { return desc_; }

private:
    TorqueConverterDesc desc_;
    real last_x_ = 0.0;
};

} // namespace ps::drivetrain
