// core/src/engine_cycle/knock.cpp - engine garage P7: see ps/engine_cycle/knock.h.
#include "ps/engine_cycle/knock.h"

#include "ps/engine_cycle/gas.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::engine_cycle {

real octane_index(real ron, real mon) { return ron - kOctaneIndexK * (ron - mon); }

real douaud_eyzat_inv_on_term(real oi, real prefactor_s) {
    return ps::math::pow(oi / real(100.0), -kDouaudEyzatOnExponent) / prefactor_s;
}

real douaud_eyzat_rate(real inv_on_term, real p_pa, real t_k, real b_k, real p_exponent) {
    const real p_atm = p_pa / real(101325.0);
    return inv_on_term * ps::math::exp(p_exponent * ps::math::log(p_atm) - b_k / t_k);
}

namespace {

// Isentropic unburned charge of fixed composition: (a + r)*ln(T/T0) + b*(T - T0) = r*ln(p/p0), Newton from t_guess.
struct EndGas {
    real a = 0.0, b = 0.0, r = 0.0; // mixture cv = a + b*T, gas constant r
    real t0 = 0.0, p0 = 0.0;

    [[nodiscard]] real temperature(real p, real t_guess) const {
        const real rhs = r * ps::math::log(p / p0);
        real t = t_guess;
        for (int i = 0; i < 12; ++i) {
            const real g = (a + r) * ps::math::log(t / t0) + b * (t - t0) - rhs;
            const real dg = (a + r) / t + b;
            const real dt = g / dg;
            t -= dt;
            if (std::abs(dt) <= real(1e-10) * t) break;
        }
        return t;
    }
};

} // namespace

KnockReplayResult knock_replay(const KnockReplayInput& in, real spark_btdc) {
    if (in.proto == nullptr) throw std::invalid_argument("knock_replay: no stepper");
    if (!(in.ivc.m > real(0.0))) throw std::runtime_error("knock_replay: the operating point has no IVC state");
    const real rate_deg_s = real(6.0) * in.op.rpm;
    if (!(rate_deg_s > real(0.0))) throw std::invalid_argument("knock_replay: rpm must be > 0");
    if (!(in.step_deg > real(0.0))) throw std::invalid_argument("knock_replay: step must be > 0");

    CycleStepper s = *in.proto; // shares the immutable model; no allocation
    s.reset(in.op);
    s.set_switches(CycleSwitches{false, true, true});
    CycleState st = s.state();
    st.y[kCylM] = in.ivc.m;
    st.y[kCylU] = in.ivc.u;
    st.y[kCylMb] = in.ivc.mb;
    s.set_state(st, in.ivc.crank_deg);
    s.set_charge(in.ivc.m - in.ivc.mb, real(0.0));
    const CycleModel& m = s.model();
    const real v0 = m.geom.eval(in.ivc.crank_deg).v;
    const CvState c0 = cv_state(m.gas, in.ivc.m, in.ivc.u, in.ivc.mb, v0);
    s.set_woschni_reference(c0.p, v0, c0.t);
    s.set_combustion_event(in.op.lambda, -spark_btdc + in.ignition_delay_deg, in.burn_duration_deg, true);

    EndGas eg;
    const real x = c0.x;
    const real ff = in.fuel_mass_fraction;
    if (!(ff >= real(0.0) && ff < real(1.0))) throw std::invalid_argument("knock_replay: fuel_mass_fraction must be in [0, 1)");
    const real fa = (real(1.0) - ff) * m.gas.fresh.a + ff * in.vapour_cv_scale * kFuelVapourCvA_JkgK;
    const real fb = (real(1.0) - ff) * m.gas.fresh.b + ff * in.vapour_cv_scale * kFuelVapourCvB_JkgK2;
    const real fr = (real(1.0) - ff) * m.gas.fresh.r + ff * kFuelVapourR_JkgK;
    eg.a = (real(1.0) - x) * fa + x * m.gas.burned.a;
    eg.b = (real(1.0) - x) * fb + x * m.gas.burned.b;
    eg.r = (real(1.0) - x) * fr + x * m.gas.burned.r;
    eg.t0 = c0.t + in.end_gas_t0_offset_k;
    eg.p0 = c0.p;
    if (!(eg.t0 > real(0.0))) throw std::invalid_argument("knock_replay: end-gas IVC temperature must stay > 0 K");
    // P7b diagnostic lever: the single-zone (air + residual) isentrope, against which the bulk's wall-heat deficit is
    // measured up to the Wiebe start.
    EndGas sz;
    sz.a = (real(1.0) - x) * m.gas.fresh.a + x * m.gas.burned.a;
    sz.b = (real(1.0) - x) * m.gas.fresh.b + x * m.gas.burned.b;
    sz.r = (real(1.0) - x) * m.gas.fresh.r + x * m.gas.burned.r;
    sz.t0 = c0.t;
    sz.p0 = c0.p;
    bool wall = in.wall_heat_end_gas;
    const real theta_burn = s.wiebe().theta0_deg;
    real t_sz_isen = sz.t0;
    real t_eg_isen = eg.t0;

    // End of the integral: the burned fraction end_burn_fraction, or EVO if that comes first.
    real theta_stop = s.wiebe().angle_at_fraction(in.end_burn_fraction);
    real evo = s.theta_evo();
    while (evo <= in.ivc.crank_deg) evo += real(720.0);
    theta_stop = std::min(theta_stop, evo);
    if (!(theta_stop > in.ivc.crank_deg)) throw std::runtime_error("knock_replay: IVC lies after the end of the burn");

    KnockReplayResult r;
    real theta = in.ivc.crank_deg;
    real t_eg = eg.t0;
    real f0 = douaud_eyzat_rate(in.inv_on_term, c0.p, t_eg, in.activation_k, in.p_exponent);
    r.p_max_pa = c0.p;
    r.p_end_pa = c0.p;
    while (theta < theta_stop) {
        const real dth = std::min(in.step_deg, theta_stop - theta);
        s.step_live(theta, dth, dth / rate_deg_s);
        theta += dth;
        const real* y = s.state().y;
        const CvState c = cv_state(m.gas, y[kCylM], y[kCylU], y[kCylMb], m.geom.eval(theta).v);
        if (c.p > r.p_max_pa) r.p_max_pa = c.p;
        if (wall) {
            // Bulk with wall heat, shifted by the vapour difference of the isentropes, until the Wiebe start; then the
            // end-gas isentrope restarts from that state.
            t_sz_isen = sz.temperature(c.p, t_sz_isen);
            t_eg_isen = eg.temperature(c.p, t_eg_isen);
            t_eg = t_eg_isen - (t_sz_isen - c.t);
            if (theta >= theta_burn) {
                eg.t0 = t_eg;
                eg.p0 = c.p;
                wall = false;
            }
        } else {
            t_eg = eg.temperature(c.p, t_eg);
        }
        r.p_end_pa = c.p;
        const real f1 = douaud_eyzat_rate(in.inv_on_term, c.p, t_eg, in.activation_k, in.p_exponent);
        r.integral += real(0.5) * (f0 + f1) * dth / rate_deg_s;
        f0 = f1;
        if (r.integral >= real(1.0)) {
            r.knock = true;
            if (in.stop_at_knock) break;
        }
    }
    r.t_end_gas_k = t_eg;
    return r;
}

KnockLimitResult knock_limited_spark(const KnockReplayInput& in, real mbt_btdc, real margin_deg) {
    KnockLimitResult out;
    real lo = mbt_btdc - kKnockSearchBelowMbtDeg;
    real hi = mbt_btdc + kKnockSearchAboveMbtDeg;
    ++out.replays;
    if (!knock_replay(in, hi).knock) {
        out.theta_kl_btdc = hi;
    } else {
        out.limited = true;
        ++out.replays;
        if (knock_replay(in, lo).knock) {
            out.saturated = true;
        } else {
            while (hi - lo > kKnockSearchToleranceDeg) {
                const real mid = real(0.5) * (lo + hi);
                ++out.replays;
                if (knock_replay(in, mid).knock) hi = mid;
                else lo = mid;
            }
        }
        out.theta_kl_btdc = lo - margin_deg;
    }
    out.theta_kl_btdc = std::clamp(out.theta_kl_btdc, real(-30.0), real(70.0));
    return out;
}

} // namespace ps::engine_cycle
