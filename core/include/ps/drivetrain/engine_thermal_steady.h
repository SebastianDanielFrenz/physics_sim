// ps/drivetrain/engine_thermal_steady.h - closed-form pieces and the steady state of the lumped engine thermal model
// (core/src/drivetrain/engine_thermal.{h,cpp}), for callers that need the temperatures an engine SETTLES at without
// running a time history (a garage setup check, a cooling-upgrade preview). Every helper here is the one the runtime
// model itself calls, so the steady state below is the fixed point of EngineThermal::step for constant inputs
// (tests/unit/test_engine_thermal_jacket.cpp J4 runs both and compares).
//
// Determinism (PLAN.md D7): + - * / and ps::math::exp/pow only. No allocation.

#pragma once

#include "ps/drivetrain/engine_thermal_desc.h"
#include "ps/types.h"

namespace ps::drivetrain {

// Coolant pump flow (l/min) at `engine_rpm`: engine pump_l_min_per_krpm x rpm/1000 x cooling pump.flow_factor.
[[nodiscard]] real coolant_pump_flow_l_min(const EngineThermalConfig& cfg, real engine_rpm);

// Water-jacket film coefficient h(Q) = max(htc_min, htc_ref * (Q / Q_ref)^n) (W/(m2 K)); 0 without a jacket.
[[nodiscard]] real water_jacket_htc_w_m2_k(const EngineThermalDesc& engine, real coolant_flow_l_min);

// Jacket conductance G_j = wetted_area * h(pump flow at engine_rpm) (W/K); 0 without a jacket.
[[nodiscard]] real water_jacket_conductance_w_k(const EngineThermalConfig& cfg, real engine_rpm);

// Lump capacities (J/K). Without a jacket metal = the whole engine lump and coolant = 0.
struct ThermalCapacities {
    real metal_j_k = 0.0, coolant_j_k = 0.0, oil_j_k = 0.0;
};
[[nodiscard]] ThermalCapacities thermal_capacities(const EngineThermalDesc& engine);

// Radiator air mass flow (kg/s): rho_air(p, T) * frontal area * max(ram_coefficient * ram, fan face velocity if on).
[[nodiscard]] real radiator_air_mass_flow_kg_s(const CoolingSystemDesc& cooling, real ram_speed_mps, bool fan_on, real ambient_k,
                                              real ambient_pressure_pa);
// Radiator conductance G_rad = eps * C_min (W/K), cross-flow both unmixed; 0 if either flow is <= 0.
[[nodiscard]] real radiator_conductance_w_k(const EngineThermalConfig& cfg, real air_mass_flow_kg_s, real coolant_mass_flow_kg_s);

struct SteadyThermalInput {
    real engine_rpm = 0.0;
    real wall_heat_w = 0.0;     // wall_heat_fraction x released fuel power
    real friction_heat_w = 0.0; // friction power (split engine/oil by friction_to_oil_fraction)
    real ambient_k = 288.15;
    real ambient_pressure_pa = 101325.0;
    real ram_speed_mps = 0.0;
    // Where the thermostat's play sits relative to its wax temperature, clamped to +-hysteresis/2: 0 = centred,
    // -hysteresis/2 = reached from below (a warm-up), +hysteresis/2 = reached from above (a cool-down).
    real thermostat_play_offset_k = 0.0;
};

struct SteadyThermalResult {
    bool bounded = true;       // false: no balance below ambient + 600 K (the reported values are that bound)
    real coolant_c = 0.0, metal_c = 0.0, oil_c = 0.0;
    real thermostat_opening = 0.0;
    bool fan_on = false;
    // The fan-on balance lies at or below the fan's off temperature: the fan cycles and the mean coolant temperature
    // lies between the two solutions; the fan-on solution is the one reported.
    bool fan_cycling = false;
    real coolant_flow_l_min = 0.0;
    real jacket_conductance_w_k = 0.0; // 0 without a jacket
    real radiator_conductance_w_k = 0.0;
    // Heat flows at the balance (W). heat in = radiator + block_air + oil_air.
    real radiator_w = 0.0, block_air_w = 0.0, oil_air_w = 0.0;
    real jacket_w = 0.0; // metal -> coolant through the jacket (0 without one)
};

// The temperatures EngineThermal settles at for constant inputs. The thermostat is taken at the opening
// x = clamp((T_coolant + thermostat_play_offset_k - open) / (full_open - open)); the fan is solved off first and on if
// the off balance reaches its on temperature. Bisection on the coolant temperature between ambient and ambient + 600 K
// (the balance is monotone in it).
[[nodiscard]] SteadyThermalResult solve_steady_thermal(const EngineThermalConfig& cfg, const SteadyThermalInput& in);

} // namespace ps::drivetrain
