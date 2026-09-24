// core/src/drivetrain/components.h — the I2 component catalogue (PHYS-033
// P4 I2 plan section 6): TorqueMapEngine, GearPair, OpenDifferential,
// DirectTorqueBrakes. Each owns exactly the per-substep scratch it needs to
// go from pre_solve (this substep's tau0/d contribution, read before
// ShaftNetwork::solve) to post_solve (energy-ledger bookkeeping, read after)
// - no allocation in either call (PLAN.md D8), only plain members already
// sized at construction. Internal (ps::drivetrain), built and owned by
// Powertrain (powertrain.h).
//
// IComponent::declare() from the plan's own full-design sketch (PHYS-033
// section 2.3) is intentionally NOT reproduced here: I2 has only three
// component kinds plus the brakes, and Powertrain's constructor (which
// already has to walk PowertrainDesc's ports/shafts in the plan's own
// deterministic order to number nodes at all, plan section 4.1) resolves
// every NodeId directly and passes it to each component's own constructor
// instead of a build-time callback - equivalent result, fewer moving parts
// for I2's small closed set of kinds. A future increment adding many more
// component kinds (I3+) may want the callback form back; not needed yet.

#pragma once

#include "network.h"

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h"

#include <cstdint>
#include <vector>

namespace ps::drivetrain {

// Per-substep values every component's pre_solve/post_solve reads (I2: the
// four control channels' CURRENT values, already resolved by
// VehicleInstance::step_tick from World's control_channel_values_, plus
// this substep's h) - kept independent of ps::vehicle::ControlInputs so
// core/src/drivetrain never has to include vehicle_instance.h.
struct ComponentSubstepCtx {
    real h = 0.0;
    real throttle = 0.0;
    real brake = 0.0;
    real handbrake = 0.0;
};

// PHYS-033 I3 (plan section 5.4, generalised from I2's GearPair-only inline
// form): reconstructs a lossy rigid row's own mesh torque (row units - the
// row's reaction on `recon_node` is `coef_at_recon_node * lambda`) from
// `recon_node`'s own momentum balance THIS substep. `recon_node` must be a
// node that carries no OTHER rigid row in any topology state where this row
// is present (the "reconstruction node" rule) - for I2's GearPair this is
// always its own `in` node with coef 1.0 (dividing by exactly 1.0 is exact
// in IEEE754, so this refactor is bit-identical to the old inline form).
// `loss_tau_already_applied` is the caller's own mesh-loss torque, already
// folded into `assembly.tau0(recon_node)` by its own pre_solve with d == 0
// (subtracted back out here, exactly as the I2 form did) - pass 0 if no
// loss torque was applied this substep.
[[nodiscard]] real reconstruct_mesh_torque(NodeId recon_node, real coef_at_recon_node, real loss_tau_already_applied,
                                            const ShaftNetwork& net, const Assembly& assembly, real omega_old_at_recon, real h);

class IComponent {
public:
    virtual ~IComponent() = default;
    virtual void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) = 0;
    virtual void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                             PowertrainEnergy& ledger) = 0;
    // Default: no additional state beyond what ShaftNetwork::fold_state_hash
    // already covers (omega/stick-sets/topology state). GearPair overrides
    // this to also fold its own lambda_mesh_prev_ (plan section 6: "each
    // gear's lambda_mesh,prev" is explicitly part of the hash).
    [[nodiscard]] virtual std::uint64_t fold_state_hash(std::uint64_t seed) const { return seed; }
};

// engine (kind torque_map, I2 semantics == the P3 placeholder -
// powertrain_desc.h's EngineLimiterDesc comment). One node: crank.
class TorqueMapEngine final : public IComponent {
public:
    TorqueMapEngine(NodeId crank, TorqueMapEngineDesc desc) : crank_(crank), desc_(std::move(desc)) {}

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;

private:
    NodeId crank_;
    TorqueMapEngineDesc desc_;
    // Stashed by pre_solve, read by post_solve (plan section 6's ledger
    // formula needs the EXACT tau0/d/omega_old this substep used, not a
    // recomputation from omega_new).
    real tau0_ = 0.0;
    real d_ = 0.0;
    real omega_old_ = 0.0;
};

// gear_pair / final_drive: w_in = ratio * w_out (rigid row, registered by
// Powertrain's constructor, not here). Dissipative mesh-loss torque at the
// input node when efficiency < 1 (plan section 5.4).
class GearPair final : public IComponent {
public:
    GearPair(NodeId in, NodeId out, GearPairDesc desc) : in_(in), out_(out), desc_(std::move(desc)) {}

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;

    [[nodiscard]] NodeId in_node() const { return in_; }

private:
    NodeId in_, out_;
    GearPairDesc desc_;
    real lambda_mesh_prev_ = 0.0; // STATE: persists substep to substep (plan section 5.4/6)
    real own_loss_tau_ = 0.0;     // this substep's own mesh-loss torque (pre_solve output, read back by post_solve)
    real omega_old_in_ = 0.0;
};

// differential (kind open): carrier = ratio * avg(left, right) (rigid row,
// registered by Powertrain's constructor). No per-substep torque or ledger
// term of its own (plan section 6 table: "-").
class OpenDifferential final : public IComponent {
public:
    OpenDifferential(NodeId carrier, NodeId left, NodeId right, DifferentialDesc desc)
        : carrier_(carrier), left_(left), right_(right), desc_(desc) {}

    void pre_solve(const ComponentSubstepCtx&, const ShaftNetwork&, Assembly&) override {}
    void post_solve(const ComponentSubstepCtx&, const ShaftNetwork&, const Assembly&, PowertrainEnergy&) override {}

private:
    NodeId carrier_, left_, right_;
    DifferentialDesc desc_;
};

// direct_torque brakes (plan section 4.2/6): one friction row per wheel hub
// that has a service-brake corner or is in the handbrake list (service +
// handbrake MERGED into one row per hub at build time - two Coulomb rows on
// the same relative velocity would make W singular). One instance covers
// every braked wheel of the vehicle.
class DirectTorqueBrakes final : public IComponent {
public:
    struct Row {
        FrictionRowId row;
        real corner_max_nm = 0.0;
        real handbrake_max_nm = 0.0;
    };

    DirectTorqueBrakes(std::vector<Row> rows, real static_factor) : rows_(std::move(rows)), static_factor_(static_factor) {}

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;

private:
    std::vector<Row> rows_;
    real static_factor_;
};

} // namespace ps::drivetrain
