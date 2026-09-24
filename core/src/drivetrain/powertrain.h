// core/src/drivetrain/powertrain.h — Powertrain (PHYS-033 P4 I2 plan
// section 3/4, commit 5b): builds one ShaftNetwork + component set from a
// vehicle's (PowertrainDesc, BrakeSystemDesc, WheelDesc list) at
// construction (sim thread, ordinary allocation allowed - PLAN.md D8 only
// forbids allocation INSIDE the per-substep job), then drives the fixed
// per-substep sequence VehicleInstance::step_tick calls into (plan section
// 7): begin_substep -> pre_solve_components + each wheel's own pre_solve
// (via add_hub_terms) -> solve -> post_solve_components + each wheel's own
// post_solve. Internal (ps::drivetrain); owned one-per-vehicle by
// VehicleInstance.

#pragma once

#include "components.h"
#include "network.h"

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/vehicle/vehicle_desc.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps::drivetrain {

class Powertrain {
public:
    // Throws std::runtime_error (naming vehicle_name) on any graph/topology
    // error - plan section 4's validation (unknown ports, double-connected
    // ports, an unconnected non-wheel port, a non-rigid shaft, a lossy gear
    // sharing its input node with another rigid row, a non-positive wheel
    // hub inertia, a cluster whose reduced mass matrix is not invertible).
    Powertrain(const PowertrainDesc& pt, const BrakeSystemDesc& brakes, const std::vector<vehicle::WheelDesc>& wheels,
               const std::string& vehicle_name);

    [[nodiscard]] NodeId hub_node(std::size_t wheel_index) const { return hub_nodes_[wheel_index]; }
    [[nodiscard]] real hub_omega(std::size_t wheel_index) const { return network_.omega(hub_nodes_[wheel_index]); }
    // Initial-condition seeding only (VehicleInstance's constructor, once -
    // matches network.h's ShaftNetwork::set_omega's own contract).
    void seed_hub_omega(std::size_t wheel_index, real omega) { network_.set_omega(hub_nodes_[wheel_index], omega); }
    // Replaces the old P3Shim::driven: true iff this wheel's hub port was
    // joined by a shaft to anything (plan section 4.1) - an unconnected hub
    // is a free wheel. Backs World::wheel_is_driven unchanged (vehicle_
    // instance.h's own comment: "survives the 5b runtime switch unchanged").
    [[nodiscard]] bool wheel_is_driven(std::size_t wheel_index) const { return driven_[wheel_index] != 0; }

    // Per-substep sequence (plan section 7), called in this exact order by
    // VehicleInstance::step_tick:
    void begin_substep(); // Assembly::begin_substep(_rows) - zeroes this substep's tau0/d/capacities
    void add_hub_terms(std::size_t wheel_index, real tau0, real d) { assembly_.torque(hub_nodes_[wheel_index], tau0, d); }
    void pre_solve_components(const ComponentSubstepCtx& ctx);
    void solve(real h); // network_.solve(h, assembly_), then integrator_loss/rotor_ke (plan section 8)
    void post_solve_components(const ComponentSubstepCtx& ctx);
    // Called once per wheel's own post_solve (WheelSubstepOutput's
    // wheel_out_j/tyre_loss_j) - these two ledger terms are NOT a component
    // (they are the hub/tyre boundary itself, plan section 8), so
    // VehicleInstance adds them directly rather than through IComponent.
    void add_wheel_energy(real wheel_out_delta, real tyre_loss_delta) {
        ledger_.wheel_out += wheel_out_delta;
        ledger_.tyre_loss += tyre_loss_delta;
    }

    // Tick-level ledger bookkeeping (plan section 8's "Check per tick"):
    // begin_tick() snapshots the cumulative ledger before this tick's
    // substeps; end_tick() computes balance_error from the deltas since
    // then. Both plain struct copies (PowertrainEnergy is all `real`
    // fields) - no allocation.
    void begin_tick() { ledger_tick_start_ = ledger_; }
    void end_tick();

    [[nodiscard]] const PowertrainEnergy& ledger() const { return ledger_; }
    [[nodiscard]] int friction_cap_hits() const { return network_.friction_cap_hits(); }
    [[nodiscard]] int nonspd_hits() const { return network_.nonspd_hits(); }
    [[nodiscard]] int lcp_stage3_hits() const { return network_.lcp_stage3_hits(); }

    // Network omega/stick-sets/topology-state (ShaftNetwork::fold_state_hash)
    // plus every component's own additional state (GearPair's
    // lambda_mesh_prev_) - plan section 6's full hash list.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    ShaftNetwork network_;
    Assembly assembly_;
    std::vector<std::unique_ptr<IComponent>> components_;
    std::vector<NodeId> hub_nodes_;
    std::vector<std::uint8_t> driven_; // parallel to hub_nodes_, WheelDesc order
    std::vector<real> omega_old_;      // per-node scratch, sized once at construction; filled fresh in solve()
    PowertrainEnergy ledger_{};
    PowertrainEnergy ledger_tick_start_{};
};

} // namespace ps::drivetrain
