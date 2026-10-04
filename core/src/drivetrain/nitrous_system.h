// drivetrain/nitrous_system.h - INTERNAL (N2O plan N1, section 3). The nitrous
// hardware and its controller as a Powertrain-owned typed member (like
// ThrottleStage/FuelTank - NOT an IComponent: no port, node or row). It owns
// the bottle fill state and the solenoid/safety state, and once per substep,
// BEFORE the engine's pre_solve, turns the control inputs into the flows the
// kit would deliver (plan section 3, steps 1-7 and 9). In N1 the engine
// IGNORES the result (Sensors::nitrous is filled but unused): every delivered
// kg is booked as unburnt pass-through; N2 wires the engine's charge model in.
//
// Units: SI inside; gauge pressures in the descriptors are bar (ISA sea-level
// 1.01325 bar reference, kit gauges are read and rated at sea level - a
// documented choice, the bottle state does not move with the live ambient).
//
// Hashing: fold_state_hash folds, only when a bottle/kit exists (Powertrain
// guards that): the oxidizer tables' content hash, the bottle mass, the
// solenoid state, the active time, the safety latch and the previous-substep
// engine flags the controller reads. Flows, pressures, counters and the pending
// substep delivery are functions of that state and the inputs (unhashed).
//
// Determinism/alloc (PLAN.md D7/D8): + - * / sqrt and table lookups only; no
// allocation after construction.
#pragma once

#include "engine.h"
#include "nitrous_flow.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/nitrous_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <memory>

namespace ps::drivetrain {

// safety_cut codes published as sound state.
enum class NitrousSafetyCut : std::uint8_t { None = 0, BottlePressure = 1, FuelStarve = 2, DutyTime = 3 };

struct NitrousInputs {
    real h = 0.0;
    // Channel values (already resolved by Powertrain; the declared default when a channel is unwired).
    real arm = 0.0;
    real heater = 0.0;
    real purge = 0.0;
    real throttle_command = 0.0; // the throttle stage's command (after TCU floor/ceiling)
    real omega = 0.0;            // crank omega_old, rad/s
    bool has_gearbox = false;
    int gear = 0;                // gearbox position: 0 neutral, > 0 forward, < 0 reverse
    real speed_mps = 0.0;        // speedo-formula road speed
    bool engine_running = false; // previous substep's EngineState::Running
    bool ignition_cut = false;   // the (lagged) TCU upshift ignition cut
    real p_intake_pa = 0.0;      // IEngine::intake_pressure_pa()
    real p_ambient_pa = 101325.0;
};

// What the system decided this substep (plain data, valid until the next step()).
struct NitrousStatus {
    bool solenoid_open = false;      // the controller requests spraying and no safety cut holds
    bool flowing = false;            // open and the bottle is not empty
    real flow_fraction = 0.0;        // progressive duty (0 while not flowing)
    real mdot_n_kg_s = 0.0;          // delivered N2O flow (the emptying substep delivers exactly the rest)
    real mdot_kf_kg_s = 0.0;         // kit fuel flow
    real mdot_purge_kg_s = 0.0;      // purge vent flow
    real retard_deg = 0.0;
    real bottle_p_pa = 0.0;          // absolute bottle pressure this substep
    real bottle_t_k = 0.0;           // saturation temperature at it
    real h_in_j_kg = 0.0;            // liquid enthalpy at the jet
    NitrousSafetyCut safety_cut = NitrousSafetyCut::None;
    bool purge_active = false;
};

// What commit() booked this substep (the Powertrain adds it to the ledger).
struct NitrousBooking {
    real n2o_kg = 0.0;   // delivered to the engine
    real purge_kg = 0.0; // vented
};

class NitrousSystem {
public:
    // arm_slot/heater_slot/purge_slot index Powertrain's declared control channels (-1: not wired).
    NitrousSystem(const N2oBottleDesc& bottle, const NitrousKitDesc& kit, real fuel_density_kg_m3, int arm_slot, int heater_slot,
                  int purge_slot);

    [[nodiscard]] int arm_slot() const { return arm_slot_; }
    [[nodiscard]] int heater_slot() const { return heater_slot_; }
    [[nodiscard]] int purge_slot() const { return purge_slot_; }
    [[nodiscard]] const NitrousKitDesc& kit() const { return kit_; }
    [[nodiscard]] const N2oBottleDesc& bottle() const { return bottle_; }
    [[nodiscard]] const OxidizerTables& tables() const { return *tables_; }

    // Once per substep, before the engine's pre_solve. Updates the controller state and returns the status;
    // remembers this substep's delivery for commit().
    const NitrousStatus& step(const NitrousInputs& in);
    // After the engine's pre_solve: the flags the controller reads NEXT substep (limiter or DFCO active;
    // fuel_supply_fraction < 1). Hashed here so the state hash covers them.
    void observe(bool engine_cut, bool starving) {
        prev_engine_cut_ = engine_cut;
        prev_starving_ = starving;
    }
    [[nodiscard]] NitrousInput engine_input() const; // the block handed to Sensors::nitrous
    // After the engine's post_solve: withdraws the bottle. Returns what was delivered and vented.
    NitrousBooking commit();

    [[nodiscard]] const NitrousStatus& status() const { return status_; }
    [[nodiscard]] real bottle_kg() const { return m_bottle_; }
    [[nodiscard]] real bottle_capacity_kg() const { return bottle_.capacity_kg; }
    // Bottle gauge pressure (bar) of the last step(); the resting state before the first one.
    [[nodiscard]] real bottle_gauge_bar() const { return status_.bottle_p_pa * real(1e-5) - kIsaAirPressurePa * real(1e-5); }
    [[nodiscard]] std::uint32_t activations() const { return activations_; }
    [[nodiscard]] std::uint32_t purge_events() const { return purge_events_; }
    [[nodiscard]] real active_time_s() const { return t_active_; }

    // Tick-boundary refill/drain (Powertrain applies a pending request in begin_tick): clamped to
    // [0, capacity]; returns the signed change actually applied.
    real set_bottle_kg(real kg);

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // Pressure of the bottle (Pa abs) for a fill state and heater state - the pure function step() uses.
    [[nodiscard]] real bottle_pressure_pa(real fill_kg, bool heater_on) const;

private:
    N2oBottleDesc bottle_;
    NitrousKitDesc kit_;
    std::unique_ptr<OxidizerTables> tables_;
    real fuel_density_kg_m3_ = 0.0;
    real nitrous_area_m2_ = 0.0, fuel_area_m2_ = 0.0, purge_area_m2_ = 0.0;
    int arm_slot_ = -1, heater_slot_ = -1, purge_slot_ = -1;

    // Hashed state.
    real m_bottle_ = 0.0;
    bool solenoid_open_ = false;
    real t_active_ = 0.0;
    std::uint8_t latch_ = 0;       // NitrousSafetyCut value while a cut holds
    bool prev_engine_cut_ = false; // as the controller saw them
    bool prev_starving_ = false;

    // Unhashed.
    NitrousStatus status_;
    real pending_n_kg_ = 0.0, pending_p_kg_ = 0.0;
    bool pending_empties_ = false;
    std::uint32_t activations_ = 0, purge_events_ = 0;
    bool purge_prev_on_ = false;
};

} // namespace ps::drivetrain
