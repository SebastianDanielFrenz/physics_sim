// core/src/drivetrain/engine_thermal.h - the lumped engine thermal model
// (engine thermal plan T1, out/engine_thermal_plan.md, vault PHYS-063).
// INTERNAL (ps::drivetrain). A Powertrain-owned typed member like FuelTank and
// NitrousSystem: no port, node or row, not an IComponent.
//
// State: two lumps (engine = block + head + coolant; oil = oil + oil-wetted
// metal), the thermostat wax temperature (first-order lag), the thermostat's
// play (backlash) state and the fan on/off bit. One backward-Euler closed-form
// 2x2 step per tick (unconditionally stable, exact ledger closure), after the
// substep loop; the control states update explicitly from the previous
// temperatures. Heat is accumulated per substep (accumulate()) and consumed by
// step().
//
// Outputs feeding the engines: plant_friction_scale() = k(T_oil) and
// ecu_friction_scale() = k(T_oil quantised to 1 K relative to T_ref), both
// from a k curve built at construction on a 5 K grid anchored at T_ref (so
// both are EXACTLY 1.0 at and above T_ref when friction_scale_min is 1).
// Both are one tick old by construction (computed at the end of step()).
//
// Determinism (PLAN.md D7): + - * / and ps::math::exp/log/pow only (std::sqrt
// is the carved-out exact one); no unordered containers. Zero allocation after
// construction (PLAN.md D8). Thermal runs in simulation time only - nothing
// here reads the environment clock.
//
// Hashed state: T_e, T_o, T_wax, T_play, fan_on - nothing else (the heat
// accumulators are zero at every tick boundary; ledger and snapshot are not
// hashed).

#pragma once

#include "ps/drivetrain/engine_thermal_desc.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <cstdint>
#include <optional>

namespace ps::drivetrain {

// Per-tick inputs from the vehicle (the same on every substep of a tick).
struct ThermalEnv {
    real ambient_k = 288.15;
    real ambient_pressure_pa = 101325.0;
    real ram_speed_mps = 0.0; // chassis airspeed along its forward axis, >= 0
    real engine_rpm = 0.0;    // pump speed; set by the owner right before step()
};

// Cumulative thermal sub-ledger (J, since construction), unhashed.
struct EngineThermalLedger {
    real wall_in = 0.0;
    real friction_in = 0.0;
    real radiator_out = 0.0;
    real block_air_out = 0.0;
    real oil_air_out = 0.0;
    real heater_out = 0.0; // T7
    real stored = 0.0;     // C_e*(T_e - T_e0) + C_o*(T_o - T_o0) since construction/last set_state baseline carried
    real balance_error = 0.0;
    real abs_flow_sum = 0.0; // sum of |flows| booked, for the relative closure check
};

class EngineThermal {
public:
    explicit EngineThermal(const EngineThermalConfig& cfg);

    // ---- per tick -----------------------------------------------------------------------------------------------
    void set_env(const ThermalEnv& env) { env_ = env; }
    // Applies a pending set_state request and, for an "ambient" start, the cold soak at this tick's ambient.
    void begin_tick();
    // One substep's heat: released fuel energy, friction heat (engine smooth + Coulomb row), substep length.
    void accumulate(real released_heat_j, real friction_heat_j, real h) {
        released_ += released_heat_j;
        friction_ += friction_heat_j;
        dt_acc_ += h;
    }
    // The BE step over the accumulated time. A no-op when no time was accumulated. Resets the accumulators.
    void step(real engine_rpm);

    // ---- outputs ------------------------------------------------------------------------------------------------
    [[nodiscard]] real plant_friction_scale() const { return plant_k_; }
    [[nodiscard]] real ecu_friction_scale() const { return ecu_k_; }
    [[nodiscard]] real motoring_friction_fraction() const { return cfg_.engine.motoring_friction_fraction; }
    [[nodiscard]] const EngineThermalSnapshot& snapshot() const { return snapshot_; }
    [[nodiscard]] const EngineThermalLedger& ledger() const { return ledger_; }
    [[nodiscard]] const EngineThermalConfig& config() const { return cfg_; }
    [[nodiscard]] real engine_temperature_k() const { return t_e_; }
    [[nodiscard]] real oil_temperature_k() const { return t_o_; }
    [[nodiscard]] real wax_temperature_k() const { return t_wax_; }
    [[nodiscard]] real play_temperature_k() const { return t_play_; }
    [[nodiscard]] bool fan_on() const { return fan_on_; }

    // World::set_engine_thermal_state: queued, applied at the next begin_tick(). Also sets wax/play to the
    // coolant temperature and switches the fan off.
    void request_state(const EngineThermalState& s) {
        pending_ = s;
        has_pending_ = true;
    }
    // Immediate version for construction-time code and tests (EngineRig): same effect as a request applied now.
    void set_state(const EngineThermalState& s);

    // Test hook (U9): pins both friction scales to `k` whatever the oil temperature is. Not a data key.
    void set_friction_scale_override_for_test(real k) {
        override_k_ = k;
        plant_k_ = k;
        ecu_k_ = k;
    }
    void clear_friction_scale_override_for_test();

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // ---- pure helpers (public for the unit tests) ---------------------------------------------------------------
    // Walther / ASTM D341 kinematic viscosity (mm2/s) of the configured oil at absolute temperature `t_k`.
    [[nodiscard]] real kinematic_viscosity_mm2_s(real t_k) const;
    // Dynamic viscosity (mPa*s, with the density correction) - only ratios are used.
    [[nodiscard]] real dynamic_viscosity(real t_k) const;
    // k = max(k_min, 1 + phi_h*(sqrt(mu/mu_ref) - 1)), evaluated exactly (not through the curve).
    [[nodiscard]] real friction_scale_exact(real t_k) const;
    // The gridded curve the runtime uses.
    [[nodiscard]] real friction_scale_from_curve(real t_k) const { return k_curve_.evaluate(t_k); }
    [[nodiscard]] const vehicle::Curve1D& k_curve() const { return k_curve_; }

    // Thermostat update from the OLD engine temperature (BE first-order wax lag, play operator, opening 0..1).
    static void thermostat_update(const EngineThermalDesc::Thermostat& th, real dt, real t_engine, real& t_wax, real& t_play,
                                  real& opening);
    // Fan hysteresis: on at >= on_k, off at <= off_k, unchanged inside the band.
    [[nodiscard]] static bool fan_update(const CoolingSystemDesc::Fan& fan, bool was_on, real t_engine) {
        if (!was_on) return t_engine >= fan.on_k;
        return !(t_engine <= fan.off_k);
    }

    // Cross-flow, both fluids unmixed effectiveness (Incropera Table 11.3) for NTU and Cr = Cmin/Cmax.
    [[nodiscard]] static real radiator_effectiveness(real ntu, real cr);
    // Radiator conductance G_rad = eps*Cmin (W/K) for the given air/coolant mass flows; 0 if either is <= 0.
    [[nodiscard]] real radiator_conductance(real air_mass_flow_kg_s, real coolant_mass_flow_kg_s) const;

private:
    void rebuild_scales();
    void reset_baseline();

    EngineThermalConfig cfg_;
    ThermalEnv env_;

    // Hashed state.
    real t_e_ = 0.0, t_o_ = 0.0, t_wax_ = 0.0, t_play_ = 0.0;
    bool fan_on_ = false;

    // Not hashed.
    real released_ = 0.0, friction_ = 0.0, dt_acc_ = 0.0; // accumulators, zero at tick boundaries
    bool ambient_start_pending_ = false;
    bool has_pending_ = false;
    EngineThermalState pending_;
    real plant_k_ = 1.0, ecu_k_ = 1.0;
    std::optional<real> override_k_;

    // Walther fit (log10 form) and the k curve.
    real walther_a_ = 0.0, walther_b_ = 0.0;
    real mu_ref_ = 1.0;
    vehicle::Curve1D k_curve_;

    // Ledger bookkeeping: the balance is (stored change since the baseline) - (in - out since the baseline) + carry.
    real t_e0_ = 0.0, t_o0_ = 0.0;
    real in_since_ = 0.0, out_since_ = 0.0, balance_carry_ = 0.0;
    EngineThermalLedger ledger_;
    EngineThermalSnapshot snapshot_;
};

} // namespace ps::drivetrain
