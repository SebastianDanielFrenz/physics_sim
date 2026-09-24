#include "components.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {
namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

// Slope (d torque / d rpm) of the SAME bracket TorqueMapEngineDesc::
// wot_torque_nm_vs_rpm::evaluate(rpm) would use, i.e. the segment
// x[i-1] < rpm <= x[i] (Curve1D's own bracket rule, ps/vehicle/curve1d.h) -
// zero outside the curve's own range, where evaluate() clamps and the
// engine's own pre_solve never calls this anyway (rpm is always clamped to
// [idle_rpm, limiter.rpm], both required to be within the curve's domain by
// the loader).
real curve_slope_at(const vehicle::Curve1D& c, real rpm) {
    if (c.x.size() < 2) return real(0.0);
    if (rpm <= c.x.front() || rpm >= c.x.back()) return real(0.0);
    for (std::size_t i = 1; i < c.x.size(); ++i) {
        if (rpm <= c.x[i]) {
            const real dx = c.x[i] - c.x[i - 1];
            return dx > real(0.0) ? (c.y[i] - c.y[i - 1]) / dx : real(0.0);
        }
    }
    return real(0.0);
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
        if (rb.valid() && rb == recon_node) row_sum -= net.row_lambda(rid);
    }

    const real balance =
        J * (omega_new - omega_old_at_recon) / h - (tau0_others - d_others * (omega_new - omega_old_at_recon)) - row_sum;
    return balance / coef_at_recon_node;
}

// --- TorqueMapEngine -------------------------------------------------------

void TorqueMapEngine::pre_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, Assembly& assembly) {
    const real omega_old = net.omega(crank_);
    omega_old_ = omega_old;

    const real rpm_raw = std::abs(omega_old) * kRadPerSecToRpm;
    const real rpm = std::clamp(rpm_raw, desc_.idle_rpm, desc_.limiter.rpm);
    real tau = ctx.throttle * desc_.wot_torque_nm_vs_rpm.evaluate(rpm);
    if (rpm_raw > desc_.limiter.rpm) tau = real(0.0); // hard_cut (plan section 6 / powertrain_desc.h's EngineLimiterDesc comment)

    real dtau_domega = real(0.0);
    if (rpm_raw > desc_.idle_rpm && rpm_raw < desc_.limiter.rpm) {
        const real slope_per_rpm = curve_slope_at(desc_.wot_torque_nm_vs_rpm, rpm);
        dtau_domega = ctx.throttle * slope_per_rpm * kRadPerSecToRpm * sign(omega_old);
    }

    tau0_ = tau;
    d_ = std::max(-dtau_domega, real(0.0));
    assembly.torque(crank_, tau0_, d_);
}

void TorqueMapEngine::post_solve(const ComponentSubstepCtx& ctx, const ShaftNetwork& net, const Assembly&,
                                  PowertrainEnergy& ledger) {
    const real omega_new = net.omega(crank_);
    const real tau_lin = tau0_ - d_ * (omega_new - omega_old_);
    ledger.engine_out += ctx.h * omega_new * tau_lin;
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
