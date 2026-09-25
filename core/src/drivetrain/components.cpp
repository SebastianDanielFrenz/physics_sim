#include "components.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {
namespace {

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

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
        lambda_row_ = compute_row_lambda(net, assembly, ctx.h); // debug only
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
}

std::uint64_t Differential::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    const auto* bytes = reinterpret_cast<const unsigned char*>(&lambda_mesh_prev_);
    for (std::size_t i = 0; i < sizeof(lambda_mesh_prev_); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
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
