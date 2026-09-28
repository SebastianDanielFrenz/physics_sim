// engine_cycle/cycle_stepper.cpp - see cycle_stepper.h.

#include "cycle_stepper.h"

#include "heat_transfer.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::engine_cycle {

real wrap720(real deg) {
    return deg - 720.0 * std::floor(deg / 720.0);
}

namespace {

real lift_at(const vehicle::Curve1D& c, real shift, real theta_deg) {
    if (c.x.empty()) return 0.0;
    const real d = wrap720(theta_deg - shift - c.x.front());
    const real span = c.x.back() - c.x.front();
    if (d >= span) return 0.0;
    const real l = c.evaluate(c.x.front() + d);
    return l > 0.0 ? l : 0.0;
}

void check_curve(const vehicle::Curve1D& c, const char* what) {
    if (c.x.size() < 2 || c.x.size() != c.y.size()) {
        throw std::invalid_argument(std::string("CycleModelDesc: ") + what + " needs >= 2 points and x/y of equal size");
    }
    for (std::size_t i = 1; i < c.x.size(); ++i) {
        if (!(c.x[i] > c.x[i - 1])) {
            throw std::invalid_argument(std::string("CycleModelDesc: ") + what + " x must be strictly increasing");
        }
    }
}

} // namespace

CycleStepper::CycleStepper(const drivetrain::CycleModelDesc& d, real lhv_j_kg, real afr_st) {
    if (d.cycle != 4) throw std::invalid_argument("CycleModelDesc: only 4-stroke is supported");
    check_curve(d.intake_lift.lift_m_vs_deg, "intake_lift");
    check_curve(d.exhaust_lift.lift_m_vs_deg, "exhaust_lift");
    check_curve(d.cd_vs_l_over_d, "cd_vs_l_over_d");
    if (d.intake_lift.lift_m_vs_deg.x.back() - d.intake_lift.lift_m_vs_deg.x.front() >= 720.0 ||
        d.exhaust_lift.lift_m_vs_deg.x.back() - d.exhaust_lift.lift_m_vs_deg.x.front() >= 720.0) {
        throw std::invalid_argument("CycleModelDesc: a lift profile span must be < 720 deg");
    }
    if (!(d.intake_valve_d_m > 0.0) || !(d.exhaust_valve_d_m > 0.0) || d.intake_valves < 1 || d.exhaust_valves < 1 ||
        !(d.runner_length_m > 0.0) || !(d.runner_diameter_m > 0.0) || !(d.intake_port_m3 > 0.0) ||
        !(d.exhaust_port_m3 > 0.0) || !(lhv_j_kg > 0.0) || !(afr_st > 0.0) || !(d.burn_duration_deg > 0.0)) {
        throw std::invalid_argument("CycleModelDesc: valve, runner, port, burn and fuel values must be > 0");
    }

    auto m = std::make_shared<CycleModel>();
    m->geom = CrankGeometry::make(d.bore_m, d.stroke_m, d.rod_m, d.pin_offset_m, d.compression_ratio);
    m->gas = GasModel::from_desc(d.gas);
    m->psi.fresh = PsiClass::make(d.gas.fresh.gamma_flow);
    m->psi.burned = PsiClass::make(d.gas.burned.gamma_flow);
    m->intake_lift = d.intake_lift.lift_m_vs_deg;
    m->exhaust_lift = d.exhaust_lift.lift_m_vs_deg;
    m->cd_vs_ld = d.cd_vs_l_over_d;
    m->intake_phase_ref = d.intake_lift.phase_ref_deg;
    m->exhaust_phase_ref = d.exhaust_lift.phase_ref_deg;
    m->intake_curtain = static_cast<real>(d.intake_valves) * kPi * d.intake_valve_d_m;
    m->exhaust_curtain = static_cast<real>(d.exhaust_valves) * kPi * d.exhaust_valve_d_m;
    m->intake_d = d.intake_valve_d_m;
    m->exhaust_d = d.exhaust_valve_d_m;
    m->runner_area = 0.25 * kPi * d.runner_diameter_m * d.runner_diameter_m;
    m->runner_length = d.runner_length_m;
    m->v_ip = d.intake_port_m3;
    m->v_ep = d.exhaust_port_m3;
    m->reservoir_cd_area = m->runner_area;  // Cd = 1 [AS, PROVISIONAL P6]
    m->wiebe_a = d.wiebe_a;
    m->wiebe_m = d.wiebe_m;
    m->burn_duration = d.burn_duration_deg;
    m->woschni_c1_gx = d.woschni_c1_gx;
    m->woschni_c1_cc = d.woschni_c1_cc;
    m->woschni_c2 = d.woschni_c2;
    m->t_wall = d.wall_temperature_k;
    m->head_area_factor = d.head_area_factor;
    m->combustion_eff_lean = d.combustion_eff_lean;
    m->t_backflow = d.exhaust_backflow_temperature_k;
    m->lhv = lhv_j_kg;
    m->afr_st = afr_st;
    model_ = std::move(m);
}

real CycleStepper::lift_intake(real theta_deg) const {
    return lift_at(model_->intake_lift, intake_shift_, theta_deg);
}

real CycleStepper::lift_exhaust(real theta_deg) const {
    return lift_at(model_->exhaust_lift, exhaust_shift_, theta_deg);
}

void CycleStepper::reset(const OperatingPoint& op) {
    const CycleModel& m = *model_;
    op_ = op;
    theta_ = 0.0;
    deg_per_s_ = 6.0 * op.rpm;
    intake_shift_ = m.intake_phase_ref - op.intake_advance_deg;
    exhaust_shift_ = m.exhaust_phase_ref + op.exhaust_retard_deg;
    theta_ivc_ = wrap720(m.intake_lift.x.back() + intake_shift_);
    theta_evo_ = wrap720(m.exhaust_lift.x.front() + exhaust_shift_);
    theta_spark_ = wrap720(360.0 + op.spark_deg_atdc);
    eta_c_ = op.lambda >= 1.0 ? m.combustion_eff_lean : m.combustion_eff_lean * op.lambda;
    wiebe_ = Wiebe{m.wiebe_a, m.wiebe_m, m.burn_duration, 360.0 + op.spark_deg_atdc};
    m_air_conv_ = 0.0;
    m_fuel_ = 0.0;
    x_b_ivc_ = 0.0;
    p_ref_ = v_ref_ = t_ref_ = 0.0;
    p_max_ = 0.0;

    state_ = CycleState{};
    const GasModel& g = m.gas;
    const real v_c = m.geom.v_c;
    const real t_b = m.t_backflow;
    const real m_cyl = op.p_amb_pa * v_c / (g.burned.r * t_b);
    state_.y[kCylM] = m_cyl;
    state_.y[kCylMb] = m_cyl;
    state_.y[kCylU] = energy_from_temperature(g, m_cyl, m_cyl, t_b);
    const real m_ip = op.p_im_pa * m.v_ip / (g.fresh.r * op.t_im_k);
    state_.y[kIpM] = m_ip;
    state_.y[kIpMb] = 0.0;
    state_.y[kIpU] = energy_from_temperature(g, m_ip, 0.0, op.t_im_k);
    state_.y[kRunnerMdot] = 0.0;
    const real m_ep = op.p_amb_pa * m.v_ep / (g.burned.r * t_b);
    state_.y[kEpM] = m_ep;
    state_.y[kEpMb] = m_ep;
    state_.y[kEpU] = energy_from_temperature(g, m_ep, m_ep, t_b);
}

void CycleStepper::reset_accumulators() {
    for (int i = kPhysicalCount; i < kStateCount; ++i) state_.y[i] = 0.0;
}

CycleInputs CycleStepper::evaluate(real theta_deg, const CycleState& s) const {
    const CycleModel& m = *model_;
    const GasModel& g = m.gas;
    CycleInputs in;
    in.theta_deg = theta_deg;
    const CrankGeometry::Eval ge = m.geom.eval(theta_deg);
    in.v_cyl = ge.v;
    in.dv_dt = ge.dv_ddeg * deg_per_s_;
    in.cyl = cv_state(g, s.y[kCylM], s.y[kCylU], s.y[kCylMb], ge.v);
    in.ip = cv_state(g, s.y[kIpM], s.y[kIpU], s.y[kIpMb], m.v_ip);
    in.ep = cv_state(g, s.y[kEpM], s.y[kEpU], s.y[kEpMb], m.v_ep);

    if (switches_.valves) {
        in.lift_in_m = lift_intake(theta_deg);
        in.lift_ex_m = lift_exhaust(theta_deg);
        if (in.lift_in_m > 0.0) {
            const real cda = m.intake_curtain * in.lift_in_m * m.cd_vs_ld.evaluate(in.lift_in_m / m.intake_d);
            in.mdot_iv = orifice_flow(m.psi, cda, in.ip.p, in.ip.r, in.ip.t, in.ip.x,
                                      in.cyl.p, in.cyl.r, in.cyl.t, in.cyl.x).mdot;
        }
        if (in.lift_ex_m > 0.0) {
            const real cda = m.exhaust_curtain * in.lift_ex_m * m.cd_vs_ld.evaluate(in.lift_ex_m / m.exhaust_d);
            in.mdot_ev = orifice_flow(m.psi, cda, in.cyl.p, in.cyl.r, in.cyl.t, in.cyl.x,
                                      in.ep.p, in.ep.r, in.ep.t, in.ep.x).mdot;
        }
    }
    in.mdot_res = orifice_flow(m.psi, m.reservoir_cd_area, in.ep.p, in.ep.r, in.ep.t, in.ep.x,
                               op_.p_amb_pa, g.burned.r, m.t_backflow, 1.0).mdot;

    if (switches_.wall_heat) {
        const CrankGeometry& geo = m.geom;
        in.wall_area = 2.0 * geo.area_piston * m.head_area_factor +
                       kPi * geo.bore * (geo.x_tdc - ge.x + geo.v_c / geo.area_piston);
        WoschniInput w;
        w.bore_m = geo.bore;
        w.p_pa = in.cyl.p;
        w.t_k = in.cyl.t;
        w.v_m3 = ge.v;
        w.mean_piston_speed = 2.0 * geo.stroke * op_.rpm / 60.0;
        const bool gas_exchange = in.lift_in_m > 0.0 || in.lift_ex_m > 0.0;
        w.c1 = gas_exchange ? m.woschni_c1_gx : m.woschni_c1_cc;
        const bool after_spark = switches_.combustion &&
                                 wrap720(theta_deg - theta_spark_) < wrap720(theta_evo_ - theta_spark_);
        w.c2 = after_spark ? m.woschni_c2 : 0.0;
        w.v_d = geo.v_d;
        w.p_ref_pa = p_ref_;
        w.v_ref_m3 = v_ref_;
        w.t_ref_k = t_ref_;
        in.woschni_h = woschni_h(w);
        in.q_wall_w = in.woschni_h * in.wall_area * (in.cyl.t - m.t_wall);
    }
    return in;
}

void CycleStepper::rhs(real theta_deg, const CycleState& s, CycleState& dydt) const {
    const CycleModel& m = *model_;
    const GasModel& g = m.gas;
    const CycleInputs in = evaluate(theta_deg, s);

    // Intake valve stream (+ into the cylinder); upstream properties.
    const bool iv_from_port = in.mdot_iv >= 0.0;
    const real x_iv = iv_from_port ? in.ip.x : in.cyl.x;
    const real h_iv = iv_from_port ? g.h_mix(in.ip.t, in.ip.x) : g.h_mix(in.cyl.t, in.cyl.x);
    // Exhaust valve stream (+ out of the cylinder).
    const bool ev_from_cyl = in.mdot_ev >= 0.0;
    const real x_ev = ev_from_cyl ? in.cyl.x : in.ep.x;
    const real t_ev = ev_from_cyl ? in.cyl.t : in.ep.t;
    const real h_ev = ev_from_cyl ? g.h_mix(in.cyl.t, in.cyl.x) : g.h_mix(in.ep.t, in.ep.x);
    // Runner (+ into the intake port): fresh at T_im on inflow.
    const real mdot_r = s.y[kRunnerMdot];
    const real x_r = mdot_r >= 0.0 ? 0.0 : in.ip.x;
    const real h_r = mdot_r >= 0.0 ? g.fresh.h(op_.t_im_k) : g.h_mix(in.ip.t, in.ip.x);
    // Reservoir orifice (+ out of the exhaust port): backflow is burned gas at T_backflow.
    const bool res_from_port = in.mdot_res >= 0.0;
    const real x_res = res_from_port ? in.ep.x : 1.0;
    const real h_res = res_from_port ? g.h_mix(in.ep.t, in.ep.x) : g.burned.h(m.t_backflow);

    // Combustion: the trapped fresh charge converts to the burned group and
    // the fuel enters (burned group, zero sensible energy) at the Wiebe rate
    // (PROVISIONAL P3); heat release eta_c * LHV * fuel rate.
    real conv_rate = 0.0, fuel_rate = 0.0, q_comb = 0.0;
    if (switches_.combustion && m_fuel_ > 0.0) {
        const real dxb_dt = wiebe_.rate_per_deg(theta_deg) * deg_per_s_;
        conv_rate = m_air_conv_ * dxb_dt;
        fuel_rate = m_fuel_ * dxb_dt;
        q_comb = eta_c_ * m.lhv * fuel_rate;
    }

    const real pdv = in.cyl.p * in.dv_dt;
    real* d = dydt.y;
    d[kCylM] = in.mdot_iv - in.mdot_ev + fuel_rate;
    d[kCylMb] = in.mdot_iv * x_iv - in.mdot_ev * x_ev + conv_rate + fuel_rate;
    d[kCylU] = -pdv + in.mdot_iv * h_iv - in.mdot_ev * h_ev + q_comb - in.q_wall_w;
    d[kIpM] = mdot_r - in.mdot_iv;
    d[kIpMb] = mdot_r * x_r - in.mdot_iv * x_iv;
    d[kIpU] = mdot_r * h_r - in.mdot_iv * h_iv;
    d[kRunnerMdot] = m.runner_area / m.runner_length * (op_.p_im_pa - in.ip.p);
    d[kEpM] = in.mdot_ev - in.mdot_res;
    d[kEpMb] = in.mdot_ev * x_ev - in.mdot_res * x_res;
    d[kEpU] = in.mdot_ev * h_ev - in.mdot_res * h_res;

    const bool gross = wrap720(theta_deg - m.geom.theta_bdc_deg) < 360.0;
    const real ev_out = in.mdot_ev > 0.0 ? in.mdot_ev : 0.0;
    d[kAccWork] = pdv;
    d[kAccWorkGross] = gross ? pdv : 0.0;
    d[kAccIvFresh] = in.mdot_iv * (1.0 - x_iv);
    d[kAccIvMass] = in.mdot_iv;
    d[kAccEvOutMass] = ev_out;
    d[kAccEvOutMassT] = ev_out * t_ev;
    d[kAccEvEnthalpy] = in.mdot_ev * h_ev;
    d[kAccIvEnthalpy] = in.mdot_iv * h_iv;
    d[kAccQComb] = q_comb;
    d[kAccQWall] = in.q_wall_w;
    d[kAccFuel] = fuel_rate;
    d[kAccRunnerMass] = mdot_r;
    d[kAccRunnerEnthalpy] = mdot_r * h_r;
    d[kAccResMass] = in.mdot_res;
    d[kAccResEnthalpy] = in.mdot_res * h_res;
}

void CycleStepper::rk4(real theta0, real theta_mid, real theta1, real h) {
    CycleState k1, k2, k3, k4, tmp;
    rhs(theta0, state_, k1);
    const real hh = 0.5 * h;
    for (int i = 0; i < kStateCount; ++i) tmp.y[i] = state_.y[i] + hh * k1.y[i];
    rhs(theta_mid, tmp, k2);
    for (int i = 0; i < kStateCount; ++i) tmp.y[i] = state_.y[i] + hh * k2.y[i];
    rhs(theta_mid, tmp, k3);
    for (int i = 0; i < kStateCount; ++i) tmp.y[i] = state_.y[i] + h * k3.y[i];
    rhs(theta1, tmp, k4);
    const real h6 = h / 6.0;
    for (int i = 0; i < kStateCount; ++i) {
        state_.y[i] += h6 * (k1.y[i] + 2.0 * k2.y[i] + 2.0 * k3.y[i] + k4.y[i]);
    }
}

void CycleStepper::after_step(real theta_before, real dtheta) {
    const CycleModel& m = *model_;
    const real theta_end = wrap720(theta_before + dtheta);
    const CrankGeometry::Eval ge = m.geom.eval(theta_end);
    const CvState cyl = cv_state(m.gas, state_.y[kCylM], state_.y[kCylU], state_.y[kCylMb], ge.v);
    if (cyl.p > p_max_) p_max_ = cyl.p;

    // IVC capture at the first step end at or after IVC.
    const real d_ivc = wrap720(theta_ivc_ - theta_before);
    if (switches_.valves && d_ivc > 0.0 && d_ivc <= dtheta) {
        m_air_conv_ = cyl.m - cyl.mb;
        m_fuel_ = m_air_conv_ / (op_.lambda * m.afr_st);
        x_b_ivc_ = cyl.x;
        p_ref_ = cyl.p;
        v_ref_ = ge.v;
        t_ref_ = cyl.t;
    }
    theta_ = theta_end;
}

void CycleStepper::step(real h) {
    const real theta0 = theta_;
    const real dtheta = deg_per_s_ * h;
    rk4(theta0, theta0 + 0.5 * dtheta, theta0 + dtheta, h);
    after_step(theta0, dtheta);
}

void CycleStepper::step_aligned(std::uint64_t k, std::uint64_t n) {
    const std::uint64_t km = k % n;
    const real rn = static_cast<real>(n);
    const real theta0 = 720.0 * static_cast<real>(km) / rn;
    const real theta1 = 720.0 * static_cast<real>(km + 1) / rn;
    const real theta_mid = 720.0 * static_cast<real>(2 * km + 1) / (2.0 * rn);
    const real h = 120.0 / (op_.rpm * rn);
    rk4(theta0, theta_mid, theta1, h);
    after_step(theta0, theta1 - theta0);
}

} // namespace ps::engine_cycle
