// core/src/drivetrain/components.h — the component catalogue (PHYS-033 P4
// I2/I3 plan section 6): GearPair, OpenDifferential, DirectTorqueBrakes.
// engine/clutch/gearbox are I3 additions (engine_torque_map.h, clutch.h,
// gearbox.h) but are deliberately NOT IComponent subclasses living in this
// file's catalogue - Powertrain (powertrain.h/.cpp) owns them as typed
// members instead and drives them directly, because the plan's substep
// order (section 7.2 step 3/3b) needs controller-phase orchestration
// (gearbox.phase_step -> a possible topology-state switch -> tcu.step,
// ALL before any component's pre_solve) that a generic
// IComponent::pre_solve(ctx, net, assembly) signature has no room for -
// see powertrain.cpp's own comment on Powertrain::pre_solve_components.
//
// Each component here owns exactly the per-substep scratch it needs to go
// from pre_solve (this substep's tau0/d contribution, read before
// ShaftNetwork::solve) to post_solve (energy-ledger bookkeeping, read
// after) - no allocation in either call (PLAN.md D8), only plain members
// already sized at construction. Internal (ps::drivetrain), built and owned
// by Powertrain (powertrain.h).

#pragma once

#include "network.h"

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h"

#include <cstdint>
#include <vector>

namespace ps::drivetrain {

// Per-substep values every component's pre_solve/post_solve reads. PHYS-033
// I3 extends I2's four fields with the controller-phase outputs Powertrain::
// pre_solve_components computes BEFORE this ctx reaches any IComponent
// below (plan section 7.2 step 3, "controllers on sensor structs") - kept
// independent of ps::vehicle::ControlInputs/ManualTcu so this stays a plain
// value type with no dependency on vehicle_instance.h or manual_tcu.h.
struct ComponentSubstepCtx {
    real h = 0.0;
    real throttle = 0.0;
    real brake = 0.0;
    real handbrake = 0.0;

    // --- raw per-tick channel inputs (VehicleInstance fills these; consumed
    // by Powertrain::pre_solve_components's controller-orchestration phase,
    // plan section 7.2 step 3, before any IComponent below ever sees this
    // ctx - components read only the COMPUTED fields further down). ---
    real clutch_pedal = 0.0; // channel "clutch"
    real gear_request = kGearRequestNone;
    real shift_up_count = 0.0, shift_down_count = 0.0;
    real ignition = 1.0; // channel "ignition"
    real starter = 0.0;  // channel "starter"
    bool assist_auto_clutch = true, assist_auto_blip = true, assist_auto_shift = false;
    bool first_substep_of_tick = false;

    // --- computed by Powertrain::pre_solve_components's controller phase,
    // read by individual IComponents' own pre_solve (plan section 7.2 step 4):
    real e_auto = 1.0;                  // this substep's TCU output (clutch/gearbox: NOT lagged, plan section 7.2)
    real clutch_engaged_fraction = 1.0; // = the clutch's own pre_solve(clutch_pedal, e_auto).engagement this substep
    bool gearbox_in_neutral = true;
    real tcu_throttle_floor = 0.0; // LAGGED one substep (engine_torque_map.h's TcuRequest, plan section 7.2's order-independence)
    bool tcu_ignition_cut = false; // LAGGED one substep
    real tcu_throttle_ceiling = 1.0; // LAGGED one substep
};

// PHYS-033 I3 (plan section 5.4, generalised from I2's GearPair-only inline
// form): reconstructs a lossy rigid row's own mesh torque (row units - the
// row's reaction on `recon_node` is `coef_at_recon_node * lambda`) from
// `recon_node`'s own momentum balance THIS substep. `recon_node` must be a
// node that carries no OTHER rigid row in any topology state where this row
// is present (the "reconstruction node" rule) - for GearPair/Gearbox this is
// always their own `in` node with coef 1.0, for OpenDifferential it is the
// `left` hub node with coef -ratio/2 (dividing by exactly 1.0 for the
// gear case is exact in IEEE754, so GearPair's use of this stays
// bit-identical to the old inline form). `loss_tau_already_applied` is the
// caller's own mesh-loss torque, already folded into `assembly.tau0(recon_
// node)` by ITS OWN pre_solve with d == 0 IF that loss was applied AT
// recon_node itself (subtracted back out here) - pass 0 if no loss torque
// was applied to recon_node this substep (e.g. OpenDifferential applies its
// loss at the CARRIER node, not at the left-hub reconstruction node, so its
// call passes 0 here).
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
    // this to also fold its own lambda_mesh_prev_ (plan section 7.7).
    [[nodiscard]] virtual std::uint64_t fold_state_hash(std::uint64_t seed) const { return seed; }
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
    real lambda_mesh_prev_ = 0.0; // STATE: persists substep to substep (plan section 5.4/7.7)
    real own_loss_tau_ = 0.0;     // this substep's own mesh-loss torque (pre_solve output, read back by post_solve)
    real omega_old_in_ = 0.0;
};

// differential (kind open): carrier = ratio * avg(left, right) (rigid row,
// registered by Powertrain's constructor). PHYS-033 I3 (plan section 5.4)
// lifts I2's "efficiency must be exactly 1" ban: a lossy diff's mesh-loss
// torque is applied at the CARRIER (input) node and reconstructed from the
// LEFT hub node (coefficient -ratio/2, the lowest-index node the diff's own
// row touches that carries no other rigid row - plan section 5.1's graph
// table).
class OpenDifferential final : public IComponent {
public:
    OpenDifferential(NodeId carrier, NodeId left, NodeId right, DifferentialDesc desc)
        : carrier_(carrier), left_(left), right_(right), desc_(desc) {}

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;

    // PHYS-033 P4 I3 commit 7: World::powertrain_debug's diff_mesh_torque/
    // hub_shaft_torque read-only accessors. Reuses lambda_mesh_prev_ AS-IS
    // (already reconstructed every substep for both shipped vehicles, whose
    // diffs are both efficiency < 1 - car_sedan 0.97, tractor 0.94 - see
    // this class's own post_solve) rather than adding any new hashed state
    // or changing when/how it is computed.
    [[nodiscard]] real lambda_mesh() const { return lambda_mesh_prev_; }
    [[nodiscard]] NodeId left_node() const { return left_; }
    [[nodiscard]] NodeId right_node() const { return right_; }
    // hub_shaft_torque[i] = hub_coef() * lambda_mesh() for i in {left, right}
    // (plan section 4.5's own formula, "coef*lambda_diff"; both hubs share
    // the same coefficient - an open diff always splits torque 50/50).
    [[nodiscard]] real hub_coef() const { return -desc_.ratio / real(2.0); }

private:
    NodeId carrier_, left_, right_;
    DifferentialDesc desc_;
    real lambda_mesh_prev_ = 0.0; // STATE: persists substep to substep, like GearPair's
    real own_loss_tau_ = 0.0;
    real omega_old_carrier_ = 0.0;
    real omega_old_left_ = 0.0;
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
