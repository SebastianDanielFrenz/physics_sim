#include "components.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {
namespace {

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

// PHYS-033 P4 I4 C8 (plan section 3.1): the antisymmetric counterpart of
// reconstruct_mesh_torque_pair's SUM below - (b_left - b_right)/2 instead of
// (b_left + b_right)/coef_sum, using the SAME per-node residual b_i (a rigid
// row that reacts +coef at left and -coef*(ratio_b/ratio_a analog) ... at
// right cancels in the SUM but survives, doubled, in the DIFFERENCE). For a
// Differential's own open row (coefficient -ratio/2 at BOTH left and right -
// symmetric, not antisymmetric) the difference is exactly 0, so this isolates
// whatever OTHER antisymmetric reaction acts equal-and-opposite on left/
// right: an active {L:+1,R:-1} lock row (kind Locked, or a lockable diff
// currently engaged) or the kind-Viscous L-R coupling (its reaction, -c*v at
// left and +c*v at right, is never subtracted here - node_b only subtracts
// FRICTION rows, and a coupling is neither a friction row nor eliminated via
// the rigid null-space, so it shows up in the residual exactly like an
// unknown rigid reaction would). A ClutchLsd/Torsen row IS a friction row, so
// node_b's own loop already subtracts its reaction out of b_left/b_right -
// Differential::compute_transfer_torque adds net.row_lambda(lsd_row_) back
// separately for those two kinds. Scope: exact for a Leaf/LeafPair diff
// (left_/right_ carry no OTHER rigid row - true of every diff kind reachable
// this way, C4-C7's own build-time checks enforce it); not meaningful for a
// hypothetical Nested (centre) diff of kind != Open, which nothing sheds
// today (centre LSD/Torsen is rejected, and a lossy centre Locked/Viscous is
// rejected too - only a lossless one may be Nested, and that path never
// calls this function).
real reconstruct_lock_torque_pair(NodeId left, NodeId right, const ShaftNetwork& net, const Assembly& assembly,
                                   real omega_old_left, real omega_old_right, real h) {
    auto node_b = [&](NodeId node, real omega_old) {
        const real omega_new = net.omega(node);
        const real dw = omega_new - omega_old;
        const real J = net.node_inertia(node);
        const real tau0_others = assembly.tau0(node); // this diff's own loss torque lives at carrier_, never left_/right_
        const real d_others = assembly.d(node);
        real row_sum = real(0.0);
        for (int r = 0; r < net.friction_row_count(); ++r) {
            const FrictionRowId rid{static_cast<std::uint16_t>(r)};
            if (net.friction_row_a(rid) == node) row_sum += net.row_lambda(rid);
            const NodeId rb = net.friction_row_b(rid);
            if (rb.valid() && rb == node) row_sum -= assembly.ratio_b(rid) * net.row_lambda(rid);
        }
        return J * dw / h - (tau0_others - d_others * dw) - row_sum;
    };
    const real b_left = node_b(left, omega_old_left);
    const real b_right = node_b(right, omega_old_right);
    return (b_left - b_right) / real(2.0);
}

} // namespace

real reconstruct_mesh_torque(NodeId recon_node, real coef_at_recon_node, real loss_tau_already_applied, const ShaftNetwork& net,
                              const Assembly& assembly, real omega_old_at_recon, real h) {
    const real omega_new = net.omega(recon_node);
    const real J = net.node_inertia(recon_node);
    // "others" excludes the caller's own loss term, which its pre_solve
    // added with d == 0 - so the assembled d at recon_node is already
    // all-others, and only tau0 needs the subtraction (plan section 5.4).
    const real tau0_others = assembly.tau0(recon_node) - loss_tau_already_applied;
    const real d_others = assembly.d(recon_node);

    real row_sum = real(0.0);
    for (int r = 0; r < net.friction_row_count(); ++r) {
        const FrictionRowId rid{static_cast<std::uint16_t>(r)};
        if (net.friction_row_a(rid) == recon_node) row_sum += net.row_lambda(rid);
        const NodeId rb = net.friction_row_b(rid);
        // PHYS-033 P4 I4 plan section 2.2: a row's b-side reaction is
        // -ratio_b*lambda (network.cpp's own B-matrix/velocity convention,
        // "v = omega(a) - ratio_b*omega(b)"), not the old unconditional
        // -lambda. Every I2/I3 row's ratio_b defaults to exactly 1.0
        // (Assembly::begin_substep_rows), so this is bit-identical to the
        // old expression for them - only a future non-1.0 row touching a
        // reconstruction node (I4's PTO/MFWD friction rows) needs it.
        if (rb.valid() && rb == recon_node) row_sum -= assembly.ratio_b(rid) * net.row_lambda(rid);
    }

    const real balance =
        J * (omega_new - omega_old_at_recon) / h - (tau0_others - d_others * (omega_new - omega_old_at_recon)) - row_sum;
    return balance / coef_at_recon_node;
}

// --- GearPair ---------------------------------------------------------------

void GearPair::pre_solve(const ComponentSubstepCtx&, const ShaftNetwork& net, Assembly& assembly) {
    omega_old_in_ = net.omega(in_);
    if (desc_.efficiency < real(1.0)) {
        own_loss_tau_ = -(real(1.0) - desc_.efficiency) * std::abs(lambda_mesh_prev_) * sign(omega_old_in_);
        assembly.torque(in_, own_loss_tau_, real(0.0));
    } else {
        own_loss_tau_ = real(0.0);
    }
}

void GearPair::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                           PowertrainEnergy& ledger) {
    if (desc_.efficiency >= real(1.0)) {
        lambda_mesh_prev_ = real(0.0);
        return; // no loss element built this substep; nothing to reconstruct or book
    }

    const real omega_new = net.omega(in_);
    const real lambda_mesh = reconstruct_mesh_torque(in_, real(1.0), own_loss_tau_, net, assembly, omega_old_in_, ctx.h);
    lambda_mesh_prev_ = lambda_mesh;

    // The loss torque actually applied THIS substep (own_loss_tau_, stashed
    // by pre_solve above) - plan section 6: "ledger.gear_loss +=
    // -h*omega'_in*tau_loss".
    ledger.gear_loss += -ctx.h * omega_new * own_loss_tau_;
}

std::uint64_t GearPair::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    const auto* bytes = reinterpret_cast<const unsigned char*>(&lambda_mesh_prev_);
    for (std::size_t i = 0; i < sizeof(lambda_mesh_prev_); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

real reconstruct_mesh_torque_pair(NodeId left, NodeId right, real coef_sum, real loss_tau_left, real loss_tau_right,
                                   real other_rigid_reaction_left, real other_rigid_reaction_right, const ShaftNetwork& net,
                                   const Assembly& assembly, real omega_old_left, real omega_old_right, real h) {
    auto node_b = [&](NodeId node, real loss_tau, real other_rigid, real omega_old) {
        const real omega_new = net.omega(node);
        const real dw = omega_new - omega_old;
        const real J = net.node_inertia(node);
        const real tau0_others = assembly.tau0(node) - loss_tau;
        const real d_others = assembly.d(node);
        real row_sum = real(0.0);
        for (int r = 0; r < net.friction_row_count(); ++r) {
            const FrictionRowId rid{static_cast<std::uint16_t>(r)};
            if (net.friction_row_a(rid) == node) row_sum += net.row_lambda(rid);
            const NodeId rb = net.friction_row_b(rid);
            if (rb.valid() && rb == node) row_sum -= assembly.ratio_b(rid) * net.row_lambda(rid);
        }
        return J * dw / h - (tau0_others - d_others * dw) - row_sum - other_rigid;
    };
    const real b_left = node_b(left, loss_tau_left, other_rigid_reaction_left, omega_old_left);
    const real b_right = node_b(right, loss_tau_right, other_rigid_reaction_right, omega_old_right);
    return (b_left + b_right) / coef_sum;
}

// --- Differential ---------------------------------------------------------

void Differential::pre_solve(const ComponentSubstepCtx&, const ShaftNetwork& net, Assembly& assembly) {
    omega_old_carrier_ = net.omega(carrier_);
    omega_old_left_ = net.omega(left_); // needed by the leaf reconstruction (recon node is `left_`, not `carrier_`)
    omega_old_right_ = net.omega(right_); // needed by the nested (pair) reconstruction only
    if (desc_.efficiency < real(1.0)) {
        // Applied at the CARRIER (the row's input node, plan section 5.4's
        // "loss application... at the row's input node" rule) - NOT at the
        // reconstruction node(s) (left_/right_), mirroring GearPair's own
        // in-node-is-both pattern except the nodes differ here.
        own_loss_tau_ = -(real(1.0) - desc_.efficiency) * std::abs(lambda_mesh_prev_) * sign(omega_old_carrier_);
        assembly.torque(carrier_, own_loss_tau_, real(0.0));
    } else {
        own_loss_tau_ = real(0.0);
    }
    // PHYS-033 P4 I4 C6: kind == Viscous - the open row above still holds;
    // this is an ADDITIONAL implicit L-R coupling (ra = rb = 1, the plain
    // omega_L - omega_R relative speed), parameterised fresh every substep
    // (Assembly::set_coupling's own contract - couplings default to c=0/
    // inert if never set, so this call is required every substep it applies).
    if (has_viscous_coupling_) assembly.set_coupling(viscous_coupling_id_, real(1.0), real(1.0), desc_.viscous_nm_s_rad);

    // PHYS-033 P4 I4 C7 (plan section 3.2): kind == ClutchLsd/Torsen - the
    // L-R friction row's capacity from the PREVIOUS substep's row multiplier
    // (lambda_row_, set by the last post_solve and hashed for these kinds).
    // A one-substep lag (1.04 ms at 960 Hz, owner-approved, no fixed-point
    // iteration): the capacity is a known number when the box-LCP runs, so
    // the solve stays one LCP per substep.
    //   T_axle = |ratio * lambda_prev|  (= |sum of the diff row's reaction on
    //            L and R|, each -ratio/2*lambda)
    //   drive  <=> P_prev = hub_coef*lambda_prev*(w_L,old + w_R,old) > 0,
    //            i.e. sum_hubs T_hub,prev*omega_hub,old with T_hub,prev the
    //            diff ROW's share of each hub torque (== the carrier's input
    //            power, -lambda_prev*w_carrier) - a pure function of hashed
    //            state (network omegas + lambda_row_), so no extra flag.
    //            The LSD row's own antisymmetric +-mu reaction is left out on
    //            purpose: the ramp responds to the direction torque flows
    //            through the carrier, not to the slip-power it dissipates.
    if (has_lsd_row_) {
        const bool locked = lock_bit_ >= 0 && (((net.topology_state() / gear_state_count_) >> lock_bit_) & 1) != 0;
        const real t_axle = std::abs(desc_.ratio * lambda_row_);
        real cap_kinetic = real(0.0), cap_static = real(0.0);
        if (desc_.kind == DifferentialKind::ClutchLsd) {
            const real p_prev = hub_coef() * lambda_row_ * (omega_old_left_ + omega_old_right_);
            lsd_drive_ = p_prev > real(0.0);
            const real f = lsd_drive_ ? desc_.drive_factor : desc_.coast_factor;
            cap_kinetic = (desc_.preload_nm + f * t_axle) / real(2.0);
            cap_static = desc_.static_factor * cap_kinetic;
        } else { // Torsen
            lsd_drive_ = false;
            cap_kinetic = (t_axle / real(2.0)) * (desc_.torque_bias_ratio - real(1.0)) / (desc_.torque_bias_ratio + real(1.0));
            cap_static = cap_kinetic;
        }
        if (locked) cap_kinetic = cap_static = real(0.0); // plan section 2.6: capacity 0 while the lock row holds
        lsd_cap_kinetic_ = cap_kinetic;
        assembly.set_friction_capacity(lsd_row_, cap_static, cap_kinetic);
    }
}

real Differential::compute_row_lambda(const ShaftNetwork& net, const Assembly& assembly, real h) const {
    switch (mode_) {
    case ReconMode::Leaf: {
        // Reconstruction node: the LEFT node (plan section 5.1's graph table
        // - the diff row's coefficient there is -ratio/2, and it carries no
        // OTHER rigid row). The loss torque itself was applied at carrier_
        // (a different node), so `loss_tau_already_applied` here is 0 -
        // left_'s own assembly.tau0 never received it. I3-literal: these are
        // exactly I3 OpenDifferential::post_solve's float operations.
        const real coef_at_left = -desc_.ratio / real(2.0);
        return reconstruct_mesh_torque(left_, coef_at_left, real(0.0), net, assembly, omega_old_left_, h);
    }
    case ReconMode::Nested: {
        // Each child's row reacts on its own carrier (== our L or R) with
        // coefficient +1.0, so its known reaction there is 1.0*lambda_child.
        real other_left = real(0.0), other_right = real(0.0);
        for (const Differential* c : children_left_) other_left += c->compute_row_lambda(net, assembly, h);
        for (const Differential* c : children_right_) other_right += c->compute_row_lambda(net, assembly, h);
        return reconstruct_mesh_torque_pair(left_, right_, -desc_.ratio, real(0.0), real(0.0), other_left, other_right, net, assembly,
                                            omega_old_left_, omega_old_right_, h);
    }
    case ReconMode::LeafPair:
        // I4 C5: a lockable leaf - the lock row's equal-and-opposite L/R
        // reaction cancels in the pair sum, so this is exact locked or not.
        return reconstruct_mesh_torque_pair(left_, right_, -desc_.ratio, real(0.0), real(0.0), real(0.0), real(0.0), net, assembly,
                                            omega_old_left_, omega_old_right_, h);
    case ReconMode::None:
    default:
        return real(0.0);
    }
}

void Differential::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly& assembly,
                               PowertrainEnergy& ledger) {
    if (desc_.efficiency >= real(1.0)) {
        lambda_mesh_prev_ = real(0.0); // I3 rule: no loss element, nothing hashed beyond 0
        lambda_row_ = compute_row_lambda(net, assembly, ctx.h); // debug only - except ClutchLsd/Torsen (I4 C7: next substep's T_axle)
    } else {
        const real lambda_mesh = compute_row_lambda(net, assembly, ctx.h);
        lambda_mesh_prev_ = lambda_mesh;
        lambda_row_ = lambda_mesh;

        const real omega_new_carrier = net.omega(carrier_);
        // Shared with GearPair/Gearbox in the ledger (plan section 5.4: "Ledger:
        // gear_loss sums both").
        ledger.gear_loss += -ctx.h * omega_new_carrier * own_loss_tau_;
    }
    // PHYS-033 P4 I4 C6: kind == Viscous - booked regardless of the
    // efficiency branch above (an independent dissipation channel; a
    // lossless (efficiency == 1) Viscous diff still dissipates through its
    // coupling). coupling_velocity is this substep's SOLVED v = w_L - w_R.
    if (has_viscous_coupling_) {
        const real v = net.coupling_velocity(viscous_coupling_id_);
        ledger.diff_loss += ctx.h * desc_.viscous_nm_s_rad * v * v;
    }
    // PHYS-033 P4 I4 C7: the LSD/Torsen friction row's slip dissipation,
    // DirectTorqueBrakes' own -h*v*lambda form (v = w_L - w_R solved, lambda
    // its multiplier; >= 0 whenever the row slips, 0 while stuck).
    if (has_lsd_row_) ledger.diff_loss += -ctx.h * net.row_velocity(lsd_row_) * net.row_lambda(lsd_row_);

    // PHYS-033 P4 I4 C8 (plan section 3.1): the lock/LSD/viscous transfer
    // torque, cached for World::powertrain_debug/PowertrainSnapshot. Debug/
    // telemetry only - never read back into dynamics, never hashed.
    transfer_torque_prev_ = compute_transfer_torque(net, assembly, ctx.h);
}

real Differential::compute_transfer_torque(const ShaftNetwork& net, const Assembly& assembly, real h) const {
    // See reconstruct_lock_torque_pair's own header comment: generic across
    // Locked/lockable-Open/Viscous/ClutchLsd/Torsen, reads ~0 for a plain
    // Open diff (no antisymmetric reaction to find).
    real transfer = reconstruct_lock_torque_pair(left_, right_, net, assembly, omega_old_left_, omega_old_right_, h);
    if (has_lsd_row_) transfer += net.row_lambda(lsd_row_); // subtracted out of the residual above; add it back
    return transfer;
}

std::uint64_t Differential::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    auto fold = [&h](const real& value) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(value); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    };
    fold(lambda_mesh_prev_);
    // PHYS-033 P4 I4 C7: only ClutchLsd/Torsen read lambda_row_ back into
    // dynamics (next substep's capacity/drive-coast), so only they fold it -
    // every Open/Locked/Viscous diff hashes byte-identically to C6.
    if (has_lsd_row_) fold(lambda_row_);
    return h;
}

// --- TransferCase (PHYS-033 P4 I4 C9) --------------------------------------

void TransferCase::pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork&, Assembly& assembly) {
    // Ramp engagement_ toward the channel's target (>= 0.5 -> 1, else 0) at
    // rate 1/engage_time_s; engage_time_s == 0 is instant (mirrors Clutch's
    // own engagement-vs-pedal shape, but as a simple ramp - plan section 3.3
    // has no curve here, just a rate).
    const real cmd = (ctx.channels != nullptr && channel_slot_ >= 0 && channel_slot_ < ctx.channel_count)
                         ? ctx.channels[channel_slot_]
                         : real(0.0);
    const real target = cmd >= real(0.5) ? real(1.0) : real(0.0);
    if (desc_.clutch.engage_time_s <= real(0.0)) {
        engagement_ = target;
    } else {
        const real rate = real(1.0) / desc_.clutch.engage_time_s;
        const real max_step = rate * ctx.h;
        if (target > engagement_) engagement_ = std::min(target, engagement_ + max_step);
        else if (target < engagement_) engagement_ = std::max(target, engagement_ - max_step);
    }

    // Assembly::begin_substep_rows resets every row's ratio_b to 1.0 each
    // substep (network.h) - front_ratio_ is a build-time constant, so it must
    // be re-asserted every substep (unlike the gearbox synchro row, whose
    // ratio changes per gear and is set conditionally).
    assembly.set_friction_row_ratio(row_, desc_.front_ratio);
    const real cap_kinetic = engagement_ * desc_.clutch.capacity_nm;
    const real cap_static = desc_.clutch.static_factor * cap_kinetic;
    assembly.set_friction_capacity(row_, cap_static, cap_kinetic);
}

void TransferCase::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly&, PowertrainEnergy& ledger) {
    const real v = net.row_velocity(row_);
    const real lambda = net.row_lambda(row_);
    // Same -h*v*lambda dissipation form as every other friction row in this
    // file (clutch/brakes/LSD) - PowertrainEnergy::awd_clutch_loss.
    ledger.awd_clutch_loss += -ctx.h * v * lambda;
}

std::uint64_t TransferCase::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    const auto* bytes = reinterpret_cast<const unsigned char*>(&engagement_);
    for (std::size_t i = 0; i < sizeof(engagement_); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

// --- ParkLock ----------------------------------------------------------------

void ParkLock::pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) {
    const real cmd = (ctx.channels != nullptr && channel_slot_ >= 0 && channel_slot_ < ctx.channel_count)
                         ? ctx.channels[channel_slot_]
                         : real(0.0);
    if (cmd < real(0.5)) {
        engaged_ = false; // release is immediate, no ramp
    } else if (!engaged_) {
        // net.omega(shaft_) is the value ShaftNetwork::solve() left behind
        // LAST substep (solve() has not run yet this substep) - "omega_old"
        // in the spec's own words. Above max_engage_speed_rad_s the pawl
        // ratchets: the request stays pending, re-checked every substep.
        if (std::abs(net.omega(shaft_)) <= desc_.max_engage_speed_rad_s) {
            engaged_ = true;
        }
    }
    // else: already engaged and still requested - holds regardless of the
    // speed the solve computes (the pawl is now clamping the shaft itself;
    // it cannot spontaneously pop out mid-hold).

    const real cap_kinetic = engaged_ ? desc_.capacity_nm : real(0.0);
    const real cap_static = desc_.static_factor * cap_kinetic;
    assembly.set_friction_capacity(row_, cap_static, cap_kinetic);
}

void ParkLock::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly&, PowertrainEnergy& ledger) {
    const real v = net.row_velocity(row_);
    const real lambda = net.row_lambda(row_);
    // Same -h*v*lambda dissipation form as every other friction row in this
    // file; booked into brake_loss per the spec (a park pawl is a brake).
    ledger.brake_loss += -ctx.h * v * lambda;
}

std::uint64_t ParkLock::fold_state_hash(std::uint64_t seed) const {
    // Only `engaged_` is persistent, hash-relevant state - pending is
    // recomputed every substep from the channel + the network's own
    // (already-hashed) omega, exactly like DiffLockDesc's lock_pending_.
    std::uint64_t h = seed;
    h ^= engaged_ ? std::uint64_t(1) : std::uint64_t(0);
    h *= 1099511628211ULL;
    return h;
}

// --- DirectTorqueBrakes ------------------------------------------------------

void DirectTorqueBrakes::pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork&, Assembly& assembly) {
    for (const Row& r : rows_) {
        const real cap_kinetic = std::max(real(0.0), r.corner_max_nm * ctx.brake + r.handbrake_max_nm * ctx.handbrake);
        const real cap_static = static_factor_ * cap_kinetic;
        assembly.set_friction_capacity(r.row, cap_static, cap_kinetic);
    }
}

void DirectTorqueBrakes::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly&,
                                     PowertrainEnergy& ledger) {
    for (const Row& r : rows_) {
        const real v = net.row_velocity(r.row);
        const real lambda = net.row_lambda(r.row);
        ledger.brake_loss += -ctx.h * v * lambda;
    }
}

} // namespace ps::drivetrain
