// engine_cycle/cycle_point.cpp - see cycle_point.h.

#include "ps/engine_cycle/cycle_point.h"

#include <cmath>

namespace ps::engine_cycle {

std::uint64_t steps_per_cycle(real rpm, real h_s) {
    const real n = std::floor(120.0 / (rpm * h_s) + 0.5);
    return n < 1.0 ? 1u : static_cast<std::uint64_t>(n);
}

PointResult run_operating_point(const CycleStepper& proto, const OperatingPoint& op,
                                const StepperSettings& settings) {
    CycleStepper s = proto;
    s.reset(op);
    const std::uint64_t n = steps_per_cycle(op.rpm, settings.h_s);
    const std::uint64_t cycles = settings.cycles < 1 ? 1u : static_cast<std::uint64_t>(settings.cycles);
    const std::uint64_t total = n * cycles;
    const std::uint64_t last_start = n * (cycles - 1);
    for (std::uint64_t k = 0; k < total; ++k) {
        if (k == last_start) {
            s.reset_accumulators();
            s.reset_peak_pressure();
        }
        s.step_aligned(k, n);
    }
    const real* y = s.state().y;
    PointResult r;
    r.w_gross_j = y[kAccWorkGross];
    r.w_pump_j = y[kAccWork] - y[kAccWorkGross];
    r.m_air_j_cyc = y[kAccIvFresh];
    r.m_fuel_kg = s.m_fuel();
    r.residual_frac = s.residual_fraction_ivc();
    r.t_exh_k = y[kAccEvOutMass] > 0.0 ? y[kAccEvOutMassT] / y[kAccEvOutMass] : 0.0;
    r.h_exh_j = y[kAccEvEnthalpy];
    r.p_max_pa = s.p_max();
    r.ca50_deg = s.wiebe().angle_at_fraction(0.5) - 360.0;
    r.steps = total;
    r.ivc = PointResult::Ivc{s.ivc_crank_deg(), s.ivc_cyl_m(), s.ivc_cyl_u(), s.ivc_cyl_mb()};
    return r;
}

} // namespace ps::engine_cycle
