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
    // N12 review fix: validate the fuel-model desc invariants here rather
    // than trusting every caller (to_engine_desc is the only one today, but
    // this is the class's own construction-time contract, same as the
    // breakaway_torque_nm check right below). has_fuel_map without
    // has_fuel_model would leave the map wired up but the ledger's own
    // has_fuel_model gate off downstream in Powertrain::post_solve_components -
    // a silent no-op fuel model, not a loud construction-time error.
    if (desc_.has_fuel_map && !desc_.has_fuel_model) {
        throw std::runtime_error("EngineTorqueMap: has_fuel_map requires has_fuel_model");
    }
    if (desc_.has_fuel_model && !desc_.has_fuel_map) {
        // crude no-map path: eta_i must be a valid (open) probability and
        // the active fuel's LHV must be positive, or mdot's division below
        // is meaningless/NaN/infinite.
        if (!(desc_.eta_i > real(0.0)) || !(desc_.eta_i < real(1.0))) {
            throw std::runtime_error("EngineTorqueMap: eta_i must be in (0, 1) for the crude no-map fuel model");
        }
        if (!(desc_.fuel.lhv_j_kg > real(0.0))) {
            throw std::runtime_error("EngineTorqueMap: fuel.lhv_j_kg must be > 0 for the crude no-map fuel model");
        }
    }
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

EngineTorqueMap::PreSolveResult EngineTorqueMap::pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) {
    const real omega_old = sensors.omega;
    omega_old_ = omega_old;
    p_ambient_last_ = sensors.ambient.pressure_pa;
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
    // PHYS-033 P4 I4 C12 (plan section 3.6): an all-speed diesel governor
    // with droop REPLACES the throttle command in the request expression
    // below when present - no governor block means req_input ==
    // throttle.command exactly, so the rest of this function is
    // byte-identical to before this commit (governor_setpoint_rpm/theta_gov
    // both stay 0.0).
    //
    // PHYS-033 P4 I7a C3: the lever = max(hand_throttle, pedal) combination
    // (and any pedal_map/linkage_curve) now happens in the throttle_stage.h
    // stage BEFORE this function ever runs - throttle.command is already
    // that combined value, so the governor's own lever here is simply
    // throttle.command (no separate max()/clamp() left to do). n_nl =
    // idle_rpm + lever*(high_idle_rpm - idle_rpm) is the governor's no-load
    // setpoint; band = droop*rated_rpm (ISO 3046-4 steady-state droop
    // referred to rated speed); theta_gov = clamp((n_nl - rpm)/band, 0, 1).
    real theta_gov = real(0.0);
    real governor_setpoint_rpm = real(0.0);
    real governor_band = real(0.0);
    real req_input = throttle.command;
    if (desc_.has_governor) {
        const real lever = throttle.command;
        governor_setpoint_rpm = desc_.idle_rpm + lever * (desc_.governor.high_idle_rpm - desc_.idle_rpm);
        governor_band = desc_.governor.droop * desc_.governor.rated_rpm;
        theta_gov = std::clamp((governor_setpoint_rpm - rpm) / governor_band, real(0.0), real(1.0));
        req_input = theta_gov;
    }

    const real theta_req = tcu.ignition_cut ? real(0.0) : std::max(std::min(req_input, throttle.ceiling), throttle.floor);

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
    // PHYS-033 P4 I7a C6: the ECU's commanded combustion fraction (the pre-C6
    // theta_c); the fired one, theta_c below, is this times the starvation
    // factor s_starve decided further down.
    const real theta_c_demand = comb ? theta_eff * f_lim : real(0.0);

    // PHYS-033 P4 I4 C12: is the governor the ACTIVE argument all the way
    // through to theta_c this substep? theta_gov must flow through UNCHANGED
    // to theta_c - not clipped by the TCU's ceiling/floor (theta_req ==
    // theta_gov, an exact compare: std::min/std::max are no-ops when the
    // value is already within range), not beaten by the idle PI (theta_eff
    // == theta_req; a TIE is treated as "not beaten", i.e. still governor-
    // active - the chosen convention at that kink), strictly inside the
    // droop band (0 < theta_gov < 1 - AT either edge the clamp's own zero
    // slope wins, the chosen convention there, consistent with theta_gov's
    // own clamp() above), combustion firing (comb - tau_comb is identically
    // 0 otherwise, so its derivative through theta_eff is 0 too), and no
    // ignition cut (theta_req is then a constant 0, independent of omega).
    const bool governor_engaged = desc_.has_governor && comb && !tcu.ignition_cut && theta_req == theta_gov &&
                                   theta_eff == theta_req && theta_gov > real(0.0) && theta_gov < real(1.0);

    // --- 6.1.7: node torque (without the Coulomb row) ---------------------------
    const real s = sign(omega_old);
    const real m_rpm = desc_.motoring_torque_nm_vs_rpm.evaluate(rpm);
    const real w_rpm = desc_.wot_torque_nm_vs_rpm.evaluate(rpm);
    // Engine thermal plan T1: oil-viscosity friction. k_g = 1 + phi_f*(k - 1) scales the smooth friction (the part
    // of the motoring curve that is friction, not pumping); k = 1.0 gives k_g == 1.0 and x*1.0 == x, bit-identical.
    const real k_plant = sensors.friction_scale;
    const real k_g = real(1.0) + desc_.thermal_friction_fraction * (k_plant - real(1.0));
    tau_smooth_ = (s * (m_rpm + c_k_)) * k_g;
    const real tau_comb_demand = theta_c_demand * (w_rpm - m_rpm);

    // PHYS-033 P4 I7a C5/C5d: fuel DEMAND at this substep-start rpm and the
    // commanded theta_c (the map's own "load" axis - see
    // docs/data_formats/engine.md). Table2D clamps at its own domain edges,
    // same as Curve1D above; theta_c_demand is already in [0, 1] by
    // construction (theta_eff clamped, f_lim clamped). mdot_kg_s_ is never
    // touched when there is no fuel model at all.
    real fuel_flow_g_s = real(0.0);
    if (desc_.has_fuel_map) {
        const real mdot_g_s = desc_.fuel_map.value(rpm, theta_c_demand);
        mdot_kg_s_ = mdot_g_s / real(1000.0);
        fuel_flow_g_s = mdot_g_s;
    } else if (desc_.has_fuel_model) {
        // PHYS-033 P4 I7a C5d: crude constant-efficiency estimate for an
        // engine with no measured map - mdot = max(0, tau_comb*omega_old) /
        // (eta_i * LHV). Uses omega_old (this substep's start speed, the
        // same substep-start evaluation point the map path above uses) and
        // the commanded tau_comb so post_solve's fuel_mass_delta stays
        // consistent with indicated_work_delta = h*omega_new*tau_comb_
        // (the omega_old-vs-omega_new asymmetry between the two is the same
        // one the map path already has between pre_solve's rpm and
        // post_solve's omega_new). max(0, .) matches the map convention
        // (fuel_flow_g_s[.][0] == 0 exactly) of booking no fuel on overrun/
        // DFCO/fuel-cut, where tau_comb is <= 0 or exactly 0.
        const real mech_power = std::max(real(0.0), tau_comb_demand * omega_old);
        mdot_kg_s_ = mech_power / (desc_.eta_i * desc_.fuel.lhv_j_kg);
        fuel_flow_g_s = mdot_kg_s_ * real(1000.0);
    }

    // PHYS-033 P4 I7a C6: fuel starvation. The feeding tank can deliver at
    // most sensors.fuel_available_kg this substep. When the demand
    // h*mdot exceeds it, combustion is scaled by s_starve = available /
    // (h*mdot) in [0, 1) - tau_comb, its share of d and the burnt fuel all
    // scale by the same factor, held constant within the substep - and the
    // booked mass is EXACTLY the available mass, so the tank's own
    // `mass - booked` lands on exactly 0.0 on the emptying substep. The map
    // and crude demands use the same rule. Otherwise s_starve == 1 and every
    // product below is x*1.0 == x (the build has FP contraction off), so an
    // engine with no tank, or a tank that covers the demand, is bit-identical
    // to pre-C6.
    real s_starve = real(1.0);
    if (desc_.has_fuel_model) {
        booked_fuel_mass_kg_ = h * mdot_kg_s_;
        if (booked_fuel_mass_kg_ > sensors.fuel_available_kg) {
            const real available = std::max(real(0.0), sensors.fuel_available_kg);
            s_starve = available / booked_fuel_mass_kg_;
            booked_fuel_mass_kg_ = available;
            mdot_kg_s_ = available / h;
            fuel_flow_g_s = mdot_kg_s_ * real(1000.0);
        }
    }
    const real theta_c = theta_c_demand * s_starve;
    tau_comb_ = tau_comb_demand * s_starve;
    const real omega_s0 = desc_.starter.free_speed_rpm / kRadPerSecToRpm;
    const bool starter_in_speed_range = starter_active && omega_old >= real(0.0) && omega_old < omega_s0;
    tau_start_ = starter_active ? desc_.starter.stall_torque_nm * std::max(real(0.0), real(1.0) - std::max(omega_old, real(0.0)) / omega_s0)
                                 : real(0.0);
    const real tau_node = tau_smooth_ + tau_comb_ + tau_start_;

    const real m_slope = curve_slope_at(desc_.motoring_torque_nm_vs_rpm, rpm);
    const real w_slope = curve_slope_at(desc_.wot_torque_nm_vs_rpm, rpm);
    // PHYS-033 P4 I7a C6: each combustion term times s_starve (x*1.0 == x
    // when not starving - same operands, same order as pre-C6).
    // Thermal T1: only the friction slope (m_slope) carries k_g; the combustion term theta*(w - m) is untouched.
    real dtau_domega = kRadPerSecToRpm * m_slope * k_g + theta_c_demand * kRadPerSecToRpm * (w_slope - m_slope) * s_starve;
    if (desc_.limiter.type == LimiterType::SoftCut) {
        const bool in_band = rpm < desc_.limiter.rpm && rpm > desc_.limiter.rpm - desc_.limiter.band_rpm;
        const real dflim_domega = in_band ? -kRadPerSecToRpm / desc_.limiter.band_rpm : real(0.0);
        dtau_domega += theta_eff * (w_rpm - m_rpm) * dflim_domega * s_starve;
    }
    // PHYS-033 P4 I4 C12: tau_comb = theta_eff*f_lim*(w-m); when the governor
    // is the active argument (governor_engaged), theta_eff == theta_gov and
    // d(theta_gov)/drpm = -1/governor_band strictly inside the droop band,
    // drpm/domega = kRadPerSecToRpm (same no-sign-factor convention as
    // m_slope/w_slope above) => d(theta_gov)/domega = -kRadPerSecToRpm/band.
    // Contribution to dtau/domega: f_lim*(w_rpm-m_rpm)*d(theta_gov)/domega -
    // exactly the spec's own formula.
    if (governor_engaged) {
        const real dgov_domega = -kRadPerSecToRpm / governor_band;
        dtau_domega += f_lim * (w_rpm - m_rpm) * dgov_domega * s_starve;
    }
    if (starter_in_speed_range) dtau_domega -= desc_.starter.stall_torque_nm / omega_s0;
    const real d = std::max(-dtau_domega, real(0.0));

    tau0_ = tau_node;
    d_ = d;

    PreSolveResult r;
    r.tau0 = tau0_;
    r.d = d_;
    r.friction_cap_static = desc_.breakaway_torque_nm * k_plant; // thermal T1: Coulomb caps scale with k
    r.friction_cap_kinetic = c_k_ * k_plant;
    r.state = state_;
    r.rpm = rpm;
    r.throttle_eff = theta_eff;
    r.limiter_active = limiter_active;
    r.dfco_active = dfco_active_;
    r.starter_active = starter_active;
    r.torque_estimate = tau_smooth_ + tau_comb_;
    r.governor_setpoint_rpm = governor_setpoint_rpm;
    // Thermal T1: the graph engine has no separate ECU model, so its torque demand uses the plant k. Written as the
    // unscaled expression plus the friction increments so that k = 1.0 adds exact zeros (bit-identical).
    r.torque_demand = s * m_rpm + (comb ? theta_eff : real(0.0)) * (w_rpm - m_rpm) +
                      (s * (m_rpm + c_k_) * (k_g - real(1.0)) - s * c_k_ * (k_plant - real(1.0)));
    // PHYS-033 P4 I7a C6: the FIRED fraction (theta_c_demand*s_starve) -
    // torque_demand above stays the ECU's unscaled request (a dry tank does
    // not change what the ECU asks for), torque_estimate follows the fired
    // tau_comb_.
    r.theta_c = theta_c;
    r.fuel_flow_g_s = fuel_flow_g_s; // delivered flow (== demand unless starving)
    r.fuel_supply_fraction = s_starve;
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

    // PHYS-033 P4 I7a C5 review fix: indicated_work_delta used to be a
    // proportional split of `work` by tau_comb_/tau0_ (the same construction
    // as frac_starter above). That ratio is singular whenever tau0_ -> 0
    // while tau_comb_ and d(omega)/dt stay nonzero - e.g. the idle PI
    // refilling torque right after a DFCO cut, with rpm still falling and a
    // non-flat motoring curve so tau_smooth_ + tau_start_ happens to cancel
    // tau_comb_ at that instant. A near-zero denominator there blows the
    // split up arbitrarily, producing a spurious negative engine_heat and
    // false (or hidden) fuel_efficiency_violations. Compute combustion's own
    // work directly instead - h*omega_new*tau_comb_, the same "work over
    // this substep at the solved omega_new" construction engine_out's own
    // `work` uses, just for tau_comb_ alone rather than tau_lin - which is
    // well-defined regardless of tau0_. Owner ruling (post-review): computed
    // and booked for EVERY torque_map engine, fuel map or not - an engine
    // with no fuel map still models combustion mechanically (its own
    // wot/motoring curves fire regardless of has_fuel_map), so
    // indicated_work is meaningful there too (N10 review fix: as of C5d it
    // DOES feed the crude no-map fuel estimate below, not merely "a future"
    // one); only fuel_mass/fuel_energy_in/engine_heat/
    // fuel_efficiency_violations (which need an actual mass/LHV to convert
    // against) stay gated on desc_.has_fuel_model (broadened from
    // has_fuel_map by C5d - see below), in Powertrain::
    // post_solve_components.
    r.indicated_work_delta = h * omega_new * tau_comb_;

    // PHYS-033 P4 I7a C5d: gate broadened from has_fuel_map to
    // has_fuel_model - mdot_kg_s_ is now populated by EITHER the map
    // (pre_solve's has_fuel_map branch) or the crude no-map estimate
    // (pre_solve's has_fuel_model branch above), and both book the same way
    // here (fuel_mass_delta = h*mdot, fuel_energy_in_delta = mass*LHV;
    // desc_.fuel is the map's calibrated fuel or the crude path's active
    // fuel respectively - to_engine_desc resolves which).
    if (desc_.has_fuel_model) {
        // PHYS-033 P4 I7a C6: decided in pre_solve - h*mdot_kg_s_ (same h,
        // same operands as the pre-C6 expression here) or, when starving,
        // exactly the available tank mass.
        r.fuel_mass_delta = booked_fuel_mass_kg_;
        r.fuel_energy_in_delta = r.fuel_mass_delta * desc_.fuel.lhv_j_kg;
        // Thermal T1 (plan 4.1): the graph engine has no lambda, eta_c = 1: everything burnt is released.
        r.released_heat_delta = r.fuel_energy_in_delta;
    }
    // Thermal T1 (plan 4.4): the smooth friction's share (phi_f) of the friction work turns into coolant/oil heat;
    // the rest of the motoring curve is pumping (exhaust, T3). tau_smooth_ already carries k_g. Booked even when a
    // tiny negative so the thermal closure stays exact.
    r.friction_heat_delta = -(h * omega_old_ * tau_smooth_ * desc_.thermal_friction_fraction);

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
