// engine_cycle/cycle_point.h - run the cycle model at one operating point
// to a periodic state and extract the per-cycle results (PHYS-033 P4 I7c
// C15a, out/i7c_plan.md section 5 C15a, D5). INTERNAL.

#pragma once

#include "cycle_stepper.h"
#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

struct PointResult {
    real w_gross_j = 0.0;     // integral p dV over compression + expansion [theta_bdc, theta_bdc + 360)
    real w_pump_j = 0.0;      // integral p dV over the gas-exchange half (negative = pumping loss)
    real m_air_j_cyc = 0.0;   // kg: net fresh air through the intake valve, last cycle
    real m_fuel_kg = 0.0;     // fuel burnt in the last cycle (trapped air at IVC / (lambda * AFR_st))
    real residual_frac = 0.0; // burned mass fraction at IVC
    real t_exh_k = 0.0;       // mass-averaged temperature of the exhaust-valve outflow
    real h_exh_j = 0.0;       // net enthalpy through the exhaust valve per cycle
    real p_max_pa = 0.0;      // peak cylinder pressure at step ends, last cycle
    real ca50_deg = 0.0;      // Wiebe CA50, deg after firing TDC (analytic)
    std::uint64_t steps = 0;
};

// D5 stepper settings. h_s is the requested step; the step actually used is
// cycle_time / round(cycle_time / h_s) (PROVISIONAL P4).
struct StepperSettings {
    real h_s = 1.0 / (960.0 * 48.0);
    int cycles = 5;
};

// D5 step sizes, HALVED from the plan's 1/(960*24) and 1/(960*12) s
// (PROVISIONAL P8, out/c15a_progress.md): with realistic port volumes the
// plan's values miss G7 at part load (1000 rpm / 30 kPa: trapped air -0.23 %
// at 43.4 us, -1.3 % at 86.8 us vs h/4). The plan's own G7 fallback.
inline constexpr real kStepFull = 1.0 / (960.0 * 48.0);
inline constexpr real kStepPreview = 1.0 / (960.0 * 24.0);

// Cycles per point (D5), fixed by G9 (PROVISIONAL P9, out/c15a_progress.md):
// the worst grid corner (6600 rpm / 30 kPa) is within 3.5e-6 (N = 5) and
// 5.4e-5 (N = 4) of N = 16 in net IMEP and trapped air, the G9 bound being
// 2e-3 between N and N + 4.
inline constexpr int kCyclesFull = 5;
inline constexpr int kCyclesPreview = 4;

// Steps per 720 deg cycle for the requested step: round(120/(rpm*h)), >= 1.
[[nodiscard]] std::uint64_t steps_per_cycle(real rpm, real h_s);

// Copies proto (no heap allocation: the model is shared), resets it to `op`,
// runs settings.cycles cycles of steps_per_cycle steps each and returns the
// last cycle's results.
[[nodiscard]] PointResult run_operating_point(const CycleStepper& proto, const OperatingPoint& op,
                                              const StepperSettings& settings);

} // namespace ps::engine_cycle
