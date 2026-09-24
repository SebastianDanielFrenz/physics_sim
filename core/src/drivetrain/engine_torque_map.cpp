#include "engine_torque_map.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::drivetrain {
namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

// Slope (d torque / d rpm) of the same bracket Curve1D::evaluate(rpm) would
// use - zero outside the curve's own domain (matches components.cpp's own
// curve_slope_at exactly; duplicated here rather than shared, since this
// class deliberately does not depend on components.h - see this file's
// header comment).
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

EngineTorqueMap::EngineTorqueMap(Desc desc) : desc_(std::move(desc)) {
    c_k_ = -desc_.motoring_torque_nm_vs_rpm.evaluate(real(0.0));
    if (desc_.breakaway_torque_nm < c_k_) {
        throw std::runtime_error("EngineTorqueMap: breakaway_torque_nm must be >= -motoring_torque_nm_vs_rpm(0)");
    }
    const real w_idle = desc_.wot_torque_nm_vs_rpm.evaluate(desc_.idle_rpm);
    const real m_idle = desc_.motoring_torque_nm_vs_rpm.evaluate(desc_.idle_rpm);
    const real denom = w_idle - m_idle;
    theta_ff_ = denom > real(0.0) ? std::clamp(-m_idle / denom, real(0.0), desc_.idle.max_throttle) : real(0.0);
    state_ = desc_.initial_state == InitialState::Running ? EngineState::Running : EngineState::Off;
}

EngineTorqueMap::PreSolveResult EngineTorqueMap::pre_solve(const Sensors& sensors, const TcuRequest& tcu, real h) {
    const real omega_old = sensors.omega;
    omega_old_ = omega_old;
    const real rpm = std::abs(omega_old) * kRadPerSecToRpm;

    // --- 6.1.3: state machine (evaluated first) -----------------------------
    const EngineState prev_state = state_;
    EngineState next = state_;
    if (sensors.ignition <= real(0.5)) {
        next = EngineState::Off;
    } else {
        switch (state_) {
            case EngineState::Off:
                next = EngineState::Stalled;
                break;
            case EngineState::Stalled: {
                const bool interlock_ok = desc_.starter.interlock == StarterInterlock::None ||
                                           (sensors.gearbox_in_neutral || sensors.clutch_engaged_fraction < real(0.1));
                if (sensors.starter > real(0.5) && interlock_ok) {
                    next = EngineState::Cranking;
                } else if (omega_old > real(0.0) && rpm >= desc_.start_complete_rpm) {
                    next = EngineState::Running; // bump start
                }
                break;
            }
            case EngineState::Cranking:
                if (rpm >= desc_.start_complete_rpm) next = EngineState::Running;
                else if (sensors.starter <= real(0.5)) next = EngineState::Stalled;
                break;
            case EngineState::Running:
                if (rpm < desc_.stall_rpm || omega_old < real(0.0)) next = EngineState::Stalled;
                break;
        }
    }
    if (next != prev_state && (next == EngineState::Off || next == EngineState::Stalled || next == EngineState::Cranking)) {
        idle_integrator_ = real(0.0);
    }
    state_ = next;
    const bool starter_active = (state_ == EngineState::Cranking);

    // --- 6.1.4: throttle path -------------------------------------------------
    const real theta_req = tcu.ignition_cut ? real(0.0) : std::max(sensors.pedal_throttle, tcu.throttle_floor);

    if (state_ == EngineState::Running && theta_req < real(0.01) && rpm > desc_.dfco_enable_rpm) {
        dfco_active_ = true;
    } else if (theta_req >= real(0.01) || rpm < desc_.dfco_reenable_rpm) {
        dfco_active_ = false;
    }

    real theta_idle = real(0.0);
    if ((state_ == EngineState::Running || state_ == EngineState::Cranking) && !dfco_active_) {
        const real e = desc_.idle_rpm - rpm;
        theta_idle = std::clamp(theta_ff_ + desc_.idle.kp_per_rpm * e + idle_integrator_, real(0.0), desc_.idle.max_throttle);
        idle_integrator_ = std::clamp(idle_integrator_ + desc_.idle.ki_per_rpm_s * e * h, -theta_ff_, desc_.idle.max_throttle - theta_ff_);
    } else {
        idle_integrator_ = real(0.0);
        theta_idle = real(0.0);
    }
    const real theta_eff = std::max(theta_req, theta_idle);

    // --- 6.1.5: limiter ---------------------------------------------------------
    real f_lim = real(1.0);
    bool limiter_active = false;
    if (desc_.limiter.type == LimiterType::FuelCut) {
        if (rpm >= desc_.limiter.rpm) limiter_cut_ = true;
        else if (rpm <= desc_.limiter.rpm - desc_.limiter.hysteresis_rpm) limiter_cut_ = false;
        f_lim = limiter_cut_ ? real(0.0) : real(1.0);
        limiter_active = limiter_cut_;
    } else { // SoftCut
        f_lim = std::clamp((desc_.limiter.rpm - rpm) / desc_.limiter.band_rpm, real(0.0), real(1.0));
        limiter_active = f_lim < real(1.0);
    }

    // --- 6.1.6: combustion -------------------------------------------------------
    const bool comb = (state_ == EngineState::Running || state_ == EngineState::Cranking) && rpm >= desc_.fire_rpm && omega_old > real(0.0);
    const real theta_c = comb ? theta_eff * f_lim : real(0.0);

    // --- 6.1.7: node torque (without the Coulomb row) ---------------------------
    const real s = sign(omega_old);
    const real m_rpm = desc_.motoring_torque_nm_vs_rpm.evaluate(rpm);
    const real w_rpm = desc_.wot_torque_nm_vs_rpm.evaluate(rpm);
    tau_smooth_ = s * (m_rpm + c_k_);
    tau_comb_ = theta_c * (w_rpm - m_rpm);
    const real omega_s0 = desc_.starter.free_speed_rpm / kRadPerSecToRpm;
    const bool starter_in_speed_range = starter_active && omega_old >= real(0.0) && omega_old < omega_s0;
    tau_start_ = starter_active ? desc_.starter.stall_torque_nm * std::max(real(0.0), real(1.0) - std::max(omega_old, real(0.0)) / omega_s0)
                                 : real(0.0);
    const real tau_node = tau_smooth_ + tau_comb_ + tau_start_;

    const real m_slope = curve_slope_at(desc_.motoring_torque_nm_vs_rpm, rpm);
    const real w_slope = curve_slope_at(desc_.wot_torque_nm_vs_rpm, rpm);
    real dtau_domega = kRadPerSecToRpm * m_slope + theta_c * kRadPerSecToRpm * (w_slope - m_slope);
    if (desc_.limiter.type == LimiterType::SoftCut) {
        const bool in_band = rpm < desc_.limiter.rpm && rpm > desc_.limiter.rpm - desc_.limiter.band_rpm;
        const real dflim_domega = in_band ? -kRadPerSecToRpm / desc_.limiter.band_rpm : real(0.0);
        dtau_domega += theta_eff * (w_rpm - m_rpm) * dflim_domega;
    }
    if (starter_in_speed_range) dtau_domega -= desc_.starter.stall_torque_nm / omega_s0;
    const real d = std::max(-dtau_domega, real(0.0));

    tau0_ = tau_node;
    d_ = d;

    PreSolveResult r;
    r.tau0 = tau0_;
    r.d = d_;
    r.friction_cap_static = desc_.breakaway_torque_nm;
    r.friction_cap_kinetic = c_k_;
    r.state = state_;
    r.rpm = rpm;
    r.throttle_eff = theta_eff;
    r.limiter_active = limiter_active;
    r.dfco_active = dfco_active_;
    r.starter_active = starter_active;
    r.torque_estimate = tau_smooth_ + tau_comb_;
    return r;
}

EngineTorqueMap::PostSolveResult EngineTorqueMap::post_solve(real omega_new, real h) {
    const real tau_lin = tau0_ - d_ * (omega_new - omega_old_);
    const real work = h * omega_new * tau_lin;
    // Split proportionally by term (plan section 6.1.8: "the identity only
    // needs the sum"): tau0_ == tau_smooth_ + tau_comb_ + tau_start_ exactly
    // (the single assembly.torque call this substep), so the starter's own
    // share is tau_start_/tau0_.
    const real frac_starter = tau0_ != real(0.0) ? tau_start_ / tau0_ : real(0.0);

    PostSolveResult r;
    r.starter_in_delta = work * frac_starter;
    r.engine_out_delta = work - r.starter_in_delta;

    crank_angle_ = std::fmod(crank_angle_ + omega_new * h, real(4.0) * kPi);
    return r;
}

std::uint64_t EngineTorqueMap::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    auto fold = [&h](const auto& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
        for (std::size_t i = 0; i < sizeof(v); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    };
    fold(state_);
    fold(idle_integrator_);
    fold(limiter_cut_);
    fold(dfco_active_);
    return h;
}

} // namespace ps::drivetrain
