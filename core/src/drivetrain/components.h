// core/src/drivetrain/components.h — the component catalogue (PHYS-033 P4
// I2/I3 plan section 6): GearPair, Differential, DirectTorqueBrakes.
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

    // PHYS-033 P4 I4 C3 (plan section 2.4): every generic component-declared
    // control channel's current value, in Powertrain::declare_control_
    // channels' own order (== VehicleInstance::component_channel_values_'s
    // order, copied once per tick, not per substep - PLAN.md D8). Points
    // into a per-vehicle buffer that outlives the substep; may be null when
    // channel_count == 0 (every shipped vehicle today), so index only below
    // channel_count.
    const real* channels = nullptr;
    int channel_count = 0;
};

// PHYS-033 I3 (plan section 5.4, generalised from I2's GearPair-only inline
// form): reconstructs a lossy rigid row's own mesh torque (row units - the
// row's reaction on `recon_node` is `coef_at_recon_node * lambda`) from
// `recon_node`'s own momentum balance THIS substep. `recon_node` must be a
// node that carries no OTHER rigid row in any topology state where this row
// is present (the "reconstruction node" rule) - for GearPair/Gearbox this is
// always their own `in` node with coef 1.0, for a leaf Differential it is the
// `left` hub node with coef -ratio/2 (dividing by exactly 1.0 for the
// gear case is exact in IEEE754, so GearPair's use of this stays
// bit-identical to the old inline form). `loss_tau_already_applied` is the
// caller's own mesh-loss torque, already folded into `assembly.tau0(recon_
// node)` by ITS OWN pre_solve with d == 0 IF that loss was applied AT
// recon_node itself (subtracted back out here) - pass 0 if no loss torque
// was applied to recon_node this substep (e.g. Differential applies its
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

// PHYS-033 P4 I4 plan section 2.7: pair reconstruction of a diff row's own
// multiplier from BOTH output nodes' momentum balances. For node i in {L, R}:
//   b_i = J_i*dw_i/h - (tau0_i - loss_tau_i - d_i*dw_i) - sum_friction_rows_at_i
//         - other_rigid_reaction_i
// and b_L + b_R = coef_sum * lambda (coef_sum = -ratio for a diff row, whose
// L/R coefficients are -ratio/2 each). A reaction that acts equal-and-
// opposite on L and R (a future lock row, an LSD friction row, an equal-
// ratio L-R coupling) cancels in the sum; `other_rigid_reaction_*` is every
// OTHER rigid row's KNOWN reaction at that node (e.g. a child diff's
// 1.0*lambda_child when L/R is that child's carrier - the non-leaf case).
// Separate from reconstruct_mesh_torque above, which stays untouched (the
// leaf path's float operations are hashed via lambda_mesh_prev_).
[[nodiscard]] real reconstruct_mesh_torque_pair(NodeId left, NodeId right, real coef_sum, real loss_tau_left, real loss_tau_right,
                                                 real other_rigid_reaction_left, real other_rigid_reaction_right,
                                                 const ShaftNetwork& net, const Assembly& assembly, real omega_old_left,
                                                 real omega_old_right, real h);

// differential (kind open): carrier = ratio * avg(left, right) (rigid row,
// registered by Powertrain's constructor). PHYS-033 I3 (plan section 5.4)
// lifts I2's "efficiency must be exactly 1" ban: a lossy diff's mesh-loss
// torque is applied at the CARRIER (input) node.
//
// PHYS-033 P4 I4 C4 (plan sections 2.5/2.7, multi-diff): the row multiplier
// is reconstructed one of two ways, chosen once at build time by Powertrain
// (configure_reconstruction):
//  - Leaf: the LEFT node carries no other rigid row in any topology state.
//    The I3 path, literally (reconstruct_mesh_torque at `left` with coef
//    -ratio/2, loss 0) - every shipped vehicle's diff.
//  - Nested: L and/or R also carry child diffs' rows (they ARE those
//    children's carriers, coef +1.0 there) and nothing else rigid - an open
//    centre diff feeding axle diffs. Pair reconstruction with each child's
//    own row multiplier (child->compute_row_lambda, recursive, pure) as the
//    known other-rigid reaction. The child's own loss torque at its carrier
//    is an applied torque already in tau0 there and correctly stays in.
//  - None: anything else; Powertrain's constructor rejects it when
//    efficiency < 1 (no reconstruction available), else lambda reads 0.
//  - LeafPair (I4 C5): a LOCKABLE leaf diff (L and R carry no other rigid
//    row apart from this diff's own lock row). The left-only path is wrong
//    while locked (the lock row's reaction also acts at L), so it always
//    uses the pair path with zero other-rigid reactions: the lock reaction
//    (+mu at L, -mu at R) cancels in b_L + b_R, and unlocked the pair sum is
//    exact too - one path for both states, no switch in the hashed float ops
//    at engage/release. Plain (non-lockable) leaf diffs keep Leaf.
class Differential final : public IComponent {
public:
    enum class ReconMode : std::uint8_t { Leaf, Nested, None, LeafPair };

    Differential(NodeId carrier, NodeId left, NodeId right, DifferentialDesc desc)
        : carrier_(carrier), left_(left), right_(right), desc_(desc) {}

    // Build-time only (Powertrain's constructor, after every Differential is
    // constructed). `children_*` are the diffs whose carrier is this diff's
    // left/right node (Nested mode only; non-owning, owned by the same
    // Powertrain's components_).
    void configure_reconstruction(ReconMode mode, std::vector<const Differential*> children_left,
                                  std::vector<const Differential*> children_right) {
        mode_ = mode;
        children_left_ = std::move(children_left);
        children_right_ = std::move(children_right);
    }

    // PHYS-033 P4 I4 C6: build-time only, called once by Powertrain's
    // constructor for a kind == Viscous diff (never for Open/Locked - a
    // default-constructed CouplingId{index=0} is a real, valid id, so
    // has_viscous_coupling_ is the only safe "none assigned" sentinel).
    void set_viscous_coupling(CouplingId id) {
        has_viscous_coupling_ = true;
        viscous_coupling_id_ = id;
    }

    // PHYS-033 P4 I4 C7 (plan section 3.2): build-time only, called once by
    // Powertrain's constructor for a kind == ClutchLsd/Torsen diff - its own
    // L-R friction row (a = left, b = right, ratio_b = 1), present in every
    // topology state. Same "has_" sentinel reason as set_viscous_coupling.
    void set_lsd_row(FrictionRowId id) {
        has_lsd_row_ = true;
        lsd_row_ = id;
    }
    // Build-time only, a LOCKABLE ClutchLsd/Torsen diff: its lock bit in the
    // network's composed topology state (s = g + n_g * sum_i lock_i*2^i,
    // powertrain.cpp Phase F), read back from net.topology_state() in
    // pre_solve - the LSD row gets capacity 0 while locked (plan section
    // 2.6: a friction row on a relative velocity the lock row already pins
    // at exactly 0 would only add a singular W column).
    void set_lock_bit(int bit, int gear_state_count) {
        lock_bit_ = bit;
        gear_state_count_ = gear_state_count;
    }

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;

    // This substep's row multiplier (row units: reaction on carrier = 1.0*
    // lambda, on each of L/R = -ratio/2*lambda), a PURE function of the
    // solved network/assembly and this diff's own pre_solve captures - valid
    // only between Powertrain::solve and the next begin_substep. Callable by
    // a parent diff before or after this diff's own post_solve (same value
    // either way). Returns 0 in ReconMode::None.
    [[nodiscard]] real compute_row_lambda(const ShaftNetwork& net, const Assembly& assembly, real h) const;

    // PHYS-033 P4 I3 commit 7 / I4 C4: World::powertrain_debug's
    // diff_mesh_torque/hub_shaft_torque read-only accessors. lambda_row_ is
    // this substep's row multiplier (NOT hashed): bit-identical to the hashed
    // lambda_mesh_prev_ whenever efficiency < 1 (both shipped vehicles), and
    // additionally filled for an efficiency-1 diff (where lambda_mesh_prev_
    // stays 0, the I3 rule) so a lossless centre diff still reports its
    // torque.
    [[nodiscard]] real lambda_mesh() const { return lambda_row_; }
    [[nodiscard]] real lambda_mesh_prev() const { return lambda_mesh_prev_; } // hashed state (tests)
    [[nodiscard]] NodeId carrier_node() const { return carrier_; }
    [[nodiscard]] NodeId left_node() const { return left_; }
    [[nodiscard]] NodeId right_node() const { return right_; }
    [[nodiscard]] real ratio() const { return desc_.ratio; }
    [[nodiscard]] ReconMode recon_mode() const { return mode_; }
    // hub_shaft_torque[i] = hub_coef() * lambda_mesh() for i in {left, right}
    // (plan section 4.5's own formula, "coef*lambda_diff"; both hubs share
    // the same coefficient - an open diff always splits torque 50/50).
    [[nodiscard]] real hub_coef() const { return -desc_.ratio / real(2.0); }
    // PHYS-033 P4 I4 C7 (tests/debug): the LSD/Torsen row and the capacity
    // pre_solve set on it THIS substep (kinetic; static = static_factor *
    // kinetic for ClutchLsd, == kinetic for Torsen), plus whether it used
    // drive_factor (true) or coast_factor (false). Zero/false for every
    // other kind.
    [[nodiscard]] bool has_lsd_row() const { return has_lsd_row_; }
    [[nodiscard]] FrictionRowId lsd_row() const { return lsd_row_; }
    [[nodiscard]] real lsd_cap_kinetic() const { return lsd_cap_kinetic_; }
    [[nodiscard]] bool lsd_drive() const { return lsd_drive_; }

    // PHYS-033 P4 I4 C8 (plan section 3.1): this substep's lock/LSD/viscous
    // TRANSFER torque (added to the left hub, subtracted from the right) -
    // debug/telemetry only, not hashed. computed once in post_solve, cached
    // here for World::powertrain_debug/PowertrainSnapshot to read back.
    [[nodiscard]] real transfer_torque() const { return transfer_torque_prev_; }
    // "Is this diff currently a rigid lock" for the DiffSnapshot::locked
    // telemetry field: kind == Locked (unconditional) or a lockable diff
    // whose bit is set in the network's current composed topology state.
    // Powertrain::fill_diff_snapshot uses its own diff_lock_engaged()
    // instead (no need to touch this diff's own lock_bit_/gear_state_count_
    // for that); this accessor exists for anything else that wants it
    // straight off the component.
    [[nodiscard]] DifferentialKind kind() const { return desc_.kind; }
    [[nodiscard]] bool lockable() const { return desc_.lockable; }

private:
    // PHYS-033 P4 I4 C8: the lock/LSD/viscous transfer torque, reconstructed
    // from the (b_left - b_right)/2 antisymmetric-residual pair (see
    // reconstruct_lock_torque_pair in components.cpp) plus, for a
    // ClutchLsd/Torsen row (whose own reaction node_b DOES subtract, unlike
    // a rigid lock row or the viscous coupling), the row's own lambda added
    // back. Works uniformly for Locked/lockable-Open/Viscous/ClutchLsd/
    // Torsen and reads ~0 for a plain Open diff - see post_solve.
    [[nodiscard]] real compute_transfer_torque(const ShaftNetwork& net, const Assembly& assembly, real h) const;

    NodeId carrier_, left_, right_;
    DifferentialDesc desc_;
    ReconMode mode_ = ReconMode::Leaf;
    std::vector<const Differential*> children_left_, children_right_;
    real lambda_mesh_prev_ = 0.0; // STATE: persists substep to substep, like GearPair's
    // This substep's row multiplier. Debug only and NOT hashed for kind Open/
    // Locked/Viscous (unchanged since C4). PHYS-033 P4 I4 C7: for kind
    // ClutchLsd/Torsen it is also STATE - the next substep's pre_solve reads
    // it as the previous substep's axle torque (T_axle = |ratio*lambda_row_|)
    // and drive/coast sign - so fold_state_hash folds it for exactly those
    // two kinds (not lambda_mesh_prev_, which the I3 rule forces to 0 for a
    // lossless diff).
    real lambda_row_ = 0.0;
    real own_loss_tau_ = 0.0;
    real omega_old_carrier_ = 0.0;
    real omega_old_left_ = 0.0;
    real omega_old_right_ = 0.0;
    // PHYS-033 P4 I4 C6: kind == Viscous only (see set_viscous_coupling above).
    bool has_viscous_coupling_ = false;
    CouplingId viscous_coupling_id_{};
    // PHYS-033 P4 I4 C7: kind == ClutchLsd/Torsen only (see set_lsd_row/
    // set_lock_bit above). lsd_cap_kinetic_/lsd_drive_ are this substep's
    // pre_solve outputs (derived from hashed state, not hashed themselves).
    bool has_lsd_row_ = false;
    FrictionRowId lsd_row_{};
    int lock_bit_ = -1; // -1: not lockable
    int gear_state_count_ = 1;
    real lsd_cap_kinetic_ = 0.0;
    bool lsd_drive_ = false;
    real transfer_torque_prev_ = 0.0; // PHYS-033 P4 I4 C8: debug/telemetry only, not hashed (see transfer_torque() above)
};

// transfer_case (PHYS-033 P4 I4 C9, plan section 3.3): mechanical front-
// wheel-drive (MFWD). Rigid row omega_in = rear_ratio*omega_rear (registered
// by Powertrain's constructor, present in EVERY topology state, exactly like
// a gear_pair's own row); ONE Coulomb friction row (a = in, b = front,
// ratio_b = front_ratio) whose capacity ramps with `engagement_` (e in
// [0,1], persistent substep-to-substep STATE - the only per-component state
// this component folds into fold_state_hash, since the sedan has no transfer
// case and its hashed bytes must stay unchanged). `Assembly::
// begin_substep_rows` resets every row's ratio_b to 1.0 each substep, so
// pre_solve re-asserts front_ratio_ unconditionally (a BUILD-TIME constant
// here, unlike the gearbox synchro row's per-gear ratio).
class TransferCase final : public IComponent {
public:
    TransferCase(NodeId in, NodeId front, FrictionRowId row, TransferCaseDesc desc, int channel_slot)
        : in_(in), front_(front), row_(row), desc_(std::move(desc)), channel_slot_(channel_slot) {}

    void pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) override;
    void post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                     PowertrainEnergy& ledger) override;
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;

    [[nodiscard]] real engagement() const { return engagement_; }
    [[nodiscard]] real front_ratio() const { return desc_.front_ratio; }

private:
    NodeId in_, front_;
    FrictionRowId row_;
    TransferCaseDesc desc_;
    int channel_slot_ = -1; // into ComponentSubstepCtx::channels
    real engagement_ = 0.0; // STATE: persists substep to substep (ramped toward the channel's target)
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
