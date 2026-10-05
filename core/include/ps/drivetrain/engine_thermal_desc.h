// ps/drivetrain/engine_thermal_desc.h - public descriptors of the lumped
// engine thermal model (engine thermal plan T1, out/engine_thermal_plan.md,
// vault PHYS-063). Pure data, no algorithm code. The model itself is
// core/src/drivetrain/engine_thermal.{h,cpp} (internal, ps::drivetrain).
//
// Two modular part files (owner rule "modular parts with overrides"):
//   - physics_sim.engine_thermal/1 (EngineThermalDesc): the engine side -
//     capacities, heat split, oil, thermostat. Named by an engine file's
//     "default_thermal", overridable per vehicle by the engine component's
//     "thermal" key.
//   - physics_sim.cooling_system/1 (CoolingSystemDesc): the vehicle side -
//     radiator, fan, gauge, warnings. Named by the vehicle's "cooling_system";
//     naming one is what makes a vehicle thermal (opt-in).
//
// Every field is in SI and ABSOLUTE (per-litre data-file forms are expanded by
// the loader against displacement_l); temperatures are in K (data files use
// degrees C, converted at load).

#pragma once

#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <cstdint>
#include <string>

namespace ps::drivetrain {

constexpr real kCelsiusToKelvin = 273.15;

// Where a thermal vehicle's temperatures start (plan section 8.4).
enum class ThermalStartKind : std::uint8_t {
    Warm = 0,    // coolant at T_open + 0.25*(T_full - T_open), oil at T_ref, wax/play = coolant, fan off
    Ambient = 1, // coolant = oil = wax = play = the first tick's ambient temperature (cold soak)
};

struct EngineThermalDesc {
    std::string source;
    real displacement_l = 0.0; // informational (the per-litre forms were expanded against it)

    // Lump capacities (J/K): engine = block + head + coolant, oil = oil + oil-wetted metal.
    real engine_capacity_j_k = 0.0;
    real oil_capacity_j_k = 0.0;

    // Heat split (plan section 4).
    real wall_heat_fraction = 0.0;          // f_wall: share of the RELEASED fuel energy that reaches the engine lump
    real friction_to_oil_fraction = 0.0;    // beta: share of the friction heat that goes into the oil lump
    real motoring_friction_fraction = 1.0;  // phi_f: friction share of a graph engine's motoring curve (ignored by simulated engines)

    // Oil and the viscosity friction law (plan section 6.1).
    struct Oil {
        real kv40_mm2_s = 0.0, kv100_mm2_s = 0.0;
        real density_15c_kg_m3 = 0.0; // density at 15.6 C (the data sheet's reference temperature)
        real reference_temperature_k = 0.0; // T_ref: friction maps apply unchanged here
        real hydrodynamic_fraction = 0.0;   // phi_h: hydrodynamic share of the friction map at hydrodynamic_reference_rpm
        // T1b: the viscosity increment (k - 1) * F(n_ref) * sqrt(n / n_ref) (hydrodynamic friction ~ sqrt(mu * speed) at
        // fixed load) is added to the map's speed-dependent part only; the Coulomb/breakaway (boundary) part is never
        // scaled. n_ref is where the whole map is scaled by exactly k.
        real hydrodynamic_reference_rpm = 0.0;
        real friction_scale_min = 1.0;      // k_min; there is NO upper cap (owner 2026-10-05)
    } oil;

    // Conductances (W/K); the "_per_m_s" ones multiply the chassis ram air speed.
    real oil_coolant_time_constant_s = 0.0; // G_oc = C_oil / tau_oc
    real block_air_w_k = 0.0, block_air_w_k_per_m_s = 0.0;
    real oil_air_w_k = 0.0, oil_air_w_k_per_m_s = 0.0;

    // Coolant (50/50 glycol) and pump.
    real coolant_cp_j_kg_k = 0.0, coolant_density_kg_m3 = 0.0;
    real pump_l_min_per_krpm = 0.0;

    // Thermostat: wax first-order lag + play (backlash) hysteresis (plan section 5.1).
    struct Thermostat {
        real open_k = 0.0, full_open_k = 0.0;
        real hysteresis_k = 0.0;
        real time_constant_s = 0.0;
    } thermostat;
};

struct CoolingSystemDesc {
    std::string source;
    struct Radiator {
        real frontal_area_m2 = 0.0;
        real ram_coefficient = 0.0;        // k_ram: face velocity = k_ram * ram speed
        real ua_ref_w_k = 0.0;             // UA at air_mass_flow_ref_kg_s
        real air_mass_flow_ref_kg_s = 0.0;
        real ua_exponent = 0.6;
    } radiator;
    struct Fan {
        real face_velocity_m_s = 0.0;
        real on_k = 0.0, off_k = 0.0; // hysteresis band (off_k < on_k)
        real power_w = 0.0;           // reported only (a future electrical bus)
    } fan;
    real oil_cooler_air_w_k_per_m_s = 0.0; // added to the oil-air conductance's ram term
    vehicle::Curve1D gauge_x_k_y;          // coolant temperature (K) -> needle position 0..1
    struct Warnings {
        real overheat_k = 0.0, cold_k = 0.0;
    } warnings;
};

// What a vehicle that opts in carries in PowertrainDesc::thermal (filled by
// the loader; a code-built desc fills it directly).
struct EngineThermalConfig {
    EngineThermalDesc engine;
    CoolingSystemDesc cooling;
    ThermalStartKind start = ThermalStartKind::Warm;
};

// World::set_engine_thermal_state's argument and engine_thermal_state's
// result. Setting it also puts the thermostat wax/play at the coolant
// temperature and switches the fan off.
struct EngineThermalState {
    real coolant_c = 0.0, oil_c = 0.0;
};

// Bits of EngineThermalSnapshot::warning_bits.
constexpr std::uint32_t kThermalWarningOverheat = 1u << 0;
constexpr std::uint32_t kThermalWarningCold = 1u << 1;
// bits 2 (derate) and 3 (limp) are reserved for T2.

// Unhashed, filled once per tick (PowertrainSnapshot::thermal).
struct EngineThermalSnapshot {
    bool has_thermal = false;
    real coolant_c = 0.0, oil_c = 0.0;
    real thermostat_opening = 0.0; // x, 0..1
    bool fan_on = false;
    real fan_power_w = 0.0;        // for a future electrical bus
    real radiator_heat_w = 0.0;    // this tick's G_rad * (T_e' - T_a)
    real wall_heat_w = 0.0;        // this tick's mean wall heat into the engine lump
    real friction_heat_w = 0.0;    // this tick's mean friction heat (engine + oil lumps)
    real gauge = 0.0;              // 0..1 needle position from the gauge curve
    std::uint32_t warning_bits = 0;
    real friction_scale = 1.0;     // plant k
};

} // namespace ps::drivetrain
