#pragma once
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/math/transcendental.h"
#include "ps/drivetrain/turbo_maps.h"
#include "psi_table.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ps::drivetrain {
// Lumped mean-value model with a quasi-steady authored compressor head curve,
// shaft energy balance and a bypass wastegate. Not an OEM compressor map.
class TurboPair {
public:
    TurboPairDesc desc;
    real energy_j = 0.0, charge_pa = 101325.0, charge_k = 288.15;
    real exhaust_pa = 101325.0, wastegate = 0.0;
    real boost_integral = 0.0; // PI controller's integral (closing) term, [0, 1]
    real compressor_w = 0.0, turbine_w = 0.0;
    // Sound/HUD outputs of the latest step (I7d D1), derived from the state
    // above and the step's inputs: never hashed, never read back by the model.
    real compressor_flow_kg_s = 0.0;  // total compressor mass flow (the pair)
    real compressor_pr = 1.0;         // compressor outlet pressure over ambient, >= 1
    // Blow-off valve (I7d D2). bov_opening is dynamic state, hashed only when
    // desc.blow_off_valve exists; the rest is sound/HUD only and never hashed.
    real bov_opening = 0.0;           // valve position in [0, 1]
    real bov_flow_kg_s = 0.0;         // vent flow of the latest step (inside compressor_flow_kg_s)
    std::uint32_t bov_events = 0;     // rising edges: opening > 0.1, re-armed below 0.05
    bool bov_latched = false;
    // Surge flag (I7d D3): pure bookkeeping, never hashed, never read back by the model.
    bool surge_active = false;
    std::uint32_t surge_events = 0;   // entries into surge; hysteresis 1.05 x the line
    // Intercooler drop (I7d D4), derived from the state and the step's flow: never hashed.
    real intercooler_dp_pa = 0.0;     // k * (flow/max)^2 of the latest step
    real compressor_outlet_pa = 101325.0; // charge_pa + intercooler_dp_pa (the pressure the compressor makes)
    // Diagnostics of the latest step (I7d D5 tests): turbine-inlet sensible heat and
    // temperature (after the 1200 K cap). Never hashed, never read back.
    real exhaust_heat_w = 0.0;
    real exhaust_k = 288.15;
    // Gas constant of the vent orifice, the same value as SimulatedEngine::kGasConstant.
    static constexpr real kGasConstant = 287.05;
    explicit TurboPair(const TurboPairDesc& d) : desc(d) {}
    real omega() const { return std::sqrt(real(2.0)*energy_j/desc.rotor_inertia_kgm2); }
    real rpm() const { return omega()*real(60.0/6.2831853071795864769); }
    void reset(real p, real t) { energy_j=0; charge_pa=p; charge_k=t; exhaust_pa=p; wastegate=0; boost_integral=0;
                                 compressor_flow_kg_s=0; compressor_pr=1; intercooler_dp_pa=0; compressor_outlet_pa=p;
                                 bov_opening=0; bov_flow_kg_s=0; bov_latched=false; surge_active=false; }
    // plate: the throttle plate position in [0, 1]; it scales the boost target.
    // p_im / psi (I7d D2): the manifold pressure the valve diaphragm senses and the
    // engine's tabulated Psi(pr) (SimulatedEngineMaps::psi), used by the vent
    // orifice only; both are ignored without a blow-off valve.
    // extra_exhaust_w (I7d D5): exhaust enthalpy on top of the authored 25 %
    // fuel-energy fraction (anti-lag: the indicated work a spark retard
    // removes), added in full to the turbine inlet's sensible heat. 0 without
    // anti-lag, and the arithmetic is then the same as before.
    void step(real h, real ambient_pa, real ambient_k, real air_kg_s, real exhaust_kg_s,
              real fuel_w, real plate, real p_im = real(0.0), const std::vector<real>* psi = nullptr,
              real extra_exhaust_w = real(0.0)) {
        const real wr = desc.reference_rpm*real(6.2831853071795864769/60.0);
        const real speed2 = real(2.0)*energy_j/(desc.rotor_inertia_kgm2*wr*wr);
        // Vent flow at the START of the step state (valve opening, charge state
        // of the previous substep): a one-substep explicit hand-off, no algebraic
        // loop. It adds to the compressor flow (head curve and compressor power)
        // and is not air the engine sees.
        real vent = real(0.0);
        if (desc.blow_off_valve && psi && bov_opening > real(0.0)) {
            const real q = ambient_pa/charge_pa; // Psi is 0 at q >= 1: no reverse flow through the valve
            if (q < real(1.0))
                vent = bov_opening*desc.blow_off_valve->effective_area_m2*charge_pa/
                       std::sqrt(kGasConstant*charge_k)*psi_table_lookup(*psi,q);
        }
        bov_flow_kg_s = vent;
        const real comp_flow = std::max(real(0.0),air_kg_s) + vent;
        const real flow = comp_flow/desc.max_mass_flow_kg_s;
        const real head = desc.maps ? desc.maps->head_pa.value(std::sqrt(speed2),flow) :
            desc.reference_head_pa*speed2*std::max(real(0.0),real(1.0)-flow*flow);
        // Pressure equilibrium includes compressor capacity; actual manifold
        // filling remains the engine's existing mass balance through its throttle.
        // I7d D4: the compressor makes head above ambient, the cooler drops dp of it; charge_pa is
        // the pressure AFTER the cooler (what the throttle and the boost controller see). With
        // dp == 0 both lines are the same arithmetic as before.
        const real dp = desc.intercooler_dp_pa_at_max_flow*flow*flow;
        const real desired = ambient_pa + head - dp;
        const real charge_old = charge_pa;
        charge_pa += h/(real(0.025)+h)*(desired-charge_pa);
        const real pr = std::max(real(1.0),(charge_pa+dp)/ambient_pa);
        intercooler_dp_pa = dp; compressor_outlet_pa = charge_pa + dp;
        const real rise = ps::math::pow(pr,real(2.0/7.0))-real(1.0);
        const real outlet_k = ambient_k*(desc.maps ? desc.maps->compressor_temperature_ratio.evaluate(pr) : real(1.0)+rise/desc.compressor_efficiency);
        charge_k = ambient_k + (outlet_k-ambient_k)*(real(1.0)-desc.intercooler_effectiveness);
        compressor_w = comp_flow*real(1005.0)*ambient_k*rise/desc.compressor_efficiency;
        compressor_flow_kg_s = comp_flow; compressor_pr = pr;
        if (desc.surge_line) {
            // pr is the compressor outlet over ambient, flow the pair's fraction of its max
            // flow (= each turbo's fraction, the pair being symmetric).
            const real line = desc.surge_line->flow_fraction_vs_pr.evaluate(pr);
            if (!surge_active) { if (pr > real(1.05) && flow < line) { surge_active = true; ++surge_events; } }
            else if (pr <= real(1.05) || flow > real(1.05)*line) surge_active = false;
        }
        // Boost control, the usual ECU arrangement: a PID controller on the
        // charge pressure after the intercooler (the throttle's upstream
        // pressure), toward ambient + target_boost_pa * plate. Its output is
        // the wastegate opening command; the actuator follows with a
        // first-order lag. The error is normalised by target_boost_pa, so
        // the gains are per unit of full boost. The rotor already integrates
        // net shaft power, so the derivative term (on the measurement, not
        // the error, so a target step does not kick) supplies the damping a
        // plain PI lacks. The integral is the closing effort in [0, 1] and
        // stops integrating while the output is saturated in the direction
        // the error pushes (conditional anti-windup; spool-up otherwise winds
        // it to 1 and overshoots onto the rotor speed cap). Below 10 % plate
        // no boost is requested: wastegate commanded open, integral reset.
        real opening = real(1.0);
        if (plate < real(0.10)) {
            boost_integral = real(0.0);
        } else {
            const real target = ambient_pa + desc.target_boost_pa*std::min(plate,real(1.0));
            const real e = (target-charge_pa)/desc.target_boost_pa;
            const real rate = (charge_pa-charge_old)/(h*desc.target_boost_pa);
            const real closing = desc.wastegate_kp*e - desc.wastegate_kd_s*rate + boost_integral;
            const bool saturated = (closing >= real(1.0) && e > real(0.0)) || (closing <= real(0.0) && e < real(0.0));
            if (!saturated) boost_integral = std::clamp(boost_integral + h*desc.wastegate_ki_per_s*e,real(0.0),real(1.0));
            opening = std::clamp(real(1.0)-closing,real(0.0),real(1.0));
        }
        wastegate += h/(desc.wastegate_actuator_tau_s+h)*(opening-wastegate);
        const real turbine_flow = std::max(real(0.0),exhaust_kg_s)*(real(1.0)-wastegate);
        // Exhaust sensible heat from an authored 25% fuel-energy fraction,
        // capped to avoid using an unbounded temperature at near-zero flow.
        real heat_w = real(0.25)*std::max(real(0.0),fuel_w);
        if (extra_exhaust_w > real(0.0)) heat_w += extra_exhaust_w;
        exhaust_heat_w = heat_w;
        exhaust_k = std::clamp(ambient_k + heat_w/
            (real(1100.0)*std::max(real(0.001),exhaust_kg_s)),ambient_k,real(1200.0));
        const real rho = ambient_pa/(real(287.05)*exhaust_k);
        exhaust_pa = ambient_pa + turbine_flow*turbine_flow/(real(2.0)*rho*desc.turbine_area_m2*desc.turbine_area_m2);
        turbine_w = turbine_flow*real(1100.0)*exhaust_k*desc.turbine_efficiency*
            (real(1.0)-ps::math::pow(ambient_pa/exhaust_pa,real(0.24812030075188)));
        const real loss = desc.shaft_loss_w_at_reference*speed2;
        energy_j = std::clamp(energy_j+h*(turbine_w-compressor_w-loss),real(0.0),
            real(0.5)*desc.rotor_inertia_kgm2*wr*wr*real(2.25));
        if (desc.blow_off_valve) {
            const auto& v = *desc.blow_off_valve;
            const real command = std::clamp((charge_pa-p_im-v.crack_dp_pa)/(v.full_open_dp_pa-v.crack_dp_pa),real(0.0),real(1.0));
            bov_opening += h/(v.tau_s+h)*(command-bov_opening);
            if (!bov_latched) { if (bov_opening > real(0.1)) { ++bov_events; bov_latched = true; } }
            else if (bov_opening < real(0.05)) bov_latched = false;
        }
    }
};
}
