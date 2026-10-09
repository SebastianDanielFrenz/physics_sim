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
// Lumped shaft/charge model with optional offline geometry stage maps.
// Legacy authored approximation remains an explicitly diagnosed fallback.
// Neither source is an OEM-calibrated compressor map.
class TurboPair {
public:
    TurboPairDesc desc;
    real energy_j = 0.0, charge_pa = 101325.0, charge_k = 288.15;
    real exhaust_pa = 101325.0, wastegate = 0.0;
    real boost_integral = 0.0; // PI controller's integral (closing) term, [0, 1]
    real compressor_w = 0.0, turbine_w = 0.0;
    bool compressor_stage_used=false, turbine_stage_used=false; // explicit coverage diagnostics
    bool stage_map_fallback=false;
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
    // Exhaust thermal mass (exhaust_thermal): wall_k is the lumped wall temperature of
    // the exhaust manifolds and turbine housings - dynamic state, hashed by the engine.
    real wall_k = 288.15;
    // Diagnostics of the latest step, derived from the state and the step's inputs:
    // never hashed, never read back. exhaust_k is the turbine-inlet gas temperature
    // (after the wall); there is no temperature cap (the engine's energy balance
    // bounds the port temperature, the wall only moves it toward wall_k).
    real port_k = 288.15;            // gas temperature handed in at the exhaust ports (>= ambient)
    real exhaust_k = 288.15;         // turbine-inlet gas temperature
    real gas_to_wall_w = 0.0;        // heat the gas gave the wall this step (W; < 0 = the wall heats the gas)
    real wall_to_ambient_w = 0.0;    // heat the wall lost to the ambient this step (W)
    // Specific heat of the exhaust gas used for the turbine and the wall exchange (J/(kg K)).
    static constexpr real kExhaustCp = 1100.0;
    // Gas constant of the vent orifice, the same value as SimulatedEngine::kGasConstant.
    static constexpr real kGasConstant = 287.05;
    // Invert the compressible convergent-nozzle relation. Once choked, flow
    // is linear in upstream total pressure, not quadratic in ambient-density
    // velocity. Both turbine and bypass see this common upstream pressure.
    static real compressible_exhaust_pressure(real flow, real area, real downstream, real temperature) {
        if(flow<=real(0)) return downstream;
        constexpr real gamma=real(1.33);
        const real critical_ratio=ps::math::pow(real(2)/(gamma+real(1)),gamma/(gamma-real(1)));
        const real critical_pressure=downstream/critical_ratio;
        const real critical_flow=area*TurboStageMaps::nozzle_flux(critical_pressure,downstream,temperature,gamma,kGasConstant);
        if(flow>=critical_flow) return critical_pressure*(flow/critical_flow);
        real lo=downstream,hi=critical_pressure;
        for(int i=0;i<24;++i) {
            const real mid=(lo+hi)*real(.5);
            if(area*TurboStageMaps::nozzle_flux(mid,downstream,temperature,gamma,kGasConstant)<flow) lo=mid;
            else hi=mid;
        }
        return (lo+hi)*real(.5);
    }
    explicit TurboPair(const TurboPairDesc& d) : desc(d) {
        if (desc.exhaust_wall_initial_k > real(0.0)) wall_k = desc.exhaust_wall_initial_k;
    }
    real omega() const { return std::sqrt(real(2.0)*energy_j/desc.rotor_inertia_kgm2); }
    real rpm() const { return omega()*real(60.0/6.2831853071795864769); }
    void reset(real p, real t) { energy_j=0; charge_pa=p; charge_k=t; exhaust_pa=p; wastegate=0; boost_integral=0;
                                 compressor_flow_kg_s=0; compressor_pr=1; intercooler_dp_pa=0; compressor_outlet_pa=p;
                                 bov_opening=0; bov_flow_kg_s=0; bov_latched=false; surge_active=false;
                                 wall_k = desc.exhaust_wall_initial_k > real(0.0) ? desc.exhaust_wall_initial_k : t;
                                 port_k=t; exhaust_k=t; gas_to_wall_w=0; wall_to_ambient_w=0;
                                 compressor_stage_used=turbine_stage_used=stage_map_fallback=false; }
    // plate: the throttle plate position in [0, 1]; it scales the boost target.
    // port_k: the exhaust gas temperature at the cylinder ports (the engine's own energy
    // balance, engine_simulated.cpp); exhaust_kg_s the exhaust mass flow. Only numerical
    // guards: port_k below ambient and a negative flow count as ambient / no flow.
    // p_im / psi (I7d D2): the manifold pressure the valve diaphragm senses and the
    // engine's tabulated Psi(pr) (SimulatedEngineMaps::psi), used by the vent
    // orifice only; both are ignored without a blow-off valve.
    void step(real h, real ambient_pa, real ambient_k, real air_kg_s, real exhaust_kg_s,
              real port_gas_k, real plate, real p_im = real(0.0), const std::vector<real>* psi = nullptr) {
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
        // Generated tables accelerate the analytic approximation inside their
        // sampled domain; outside it evaluate that same law without clamping.
        const real speed=std::sqrt(speed2);
        const real speed_edge=desc.maps ? desc.maps->head_pa.x.back():real(1.5);
        const bool overspeed=speed>speed_edge;
        // Extend the terminal speed line with compressor affinity scaling:
        // flow capacity grows with speed, pressure head with speed squared.
        // A fixed choke flow at every speed would make an unattainable boost
        // request feed unlimited shaft energy into an unchanged flow ceiling.
        const real ratio=overspeed ? speed/speed_edge:real(1.0);
        const real corrected_flow=flow/ratio;
        real head=desc.maps ?
            desc.maps->head_pa.value(std::min(speed,speed_edge),corrected_flow)*ratio*ratio:
            desc.reference_head_pa*speed2*std::max(real(0.0),real(1.0)-corrected_flow*corrected_flow);

        const auto* stages=desc.maps ? desc.maps->stages.get():nullptr;
        compressor_stage_used=false; turbine_stage_used=false; stage_map_fallback=false;
        real stage_work=0, stage_slope=0;
        if(stages) {
            const real theta=ambient_k/stages->compressor_reference_k;
            const real nc=stages->snap(stages->compressor_pr.x,rpm()/std::sqrt(theta));
            const real wc=stages->snap(stages->compressor_pr.y,comp_flow/stages->units*std::sqrt(theta)/(ambient_pa/stages->reference_pa));
            const auto sample=stages->compressor_sample(nc,wc);
            if(sample.weight>real(0)) {
                compressor_stage_used=true;
                const real old_head=head;
                const real authored_work=real(1005)*ambient_k*(ps::math::pow(real(1)+std::max(real(0),head)/ambient_pa,real(2.0/7.0))-real(1))/desc.compressor_efficiency;
                const real authored_slope=real(2)*desc.reference_head_pa*speed_edge*speed_edge*flow*flow;
                head=(real(1)-sample.weight)*old_head+ambient_pa*(sample.pressure_ratio-sample.weight);
                stage_work=(real(1)-sample.weight)*authored_work+sample.work*theta;
                stage_slope=(real(1)-sample.weight)*authored_slope+
                    (old_head+ambient_pa)*sample.weight_slope*wc-ambient_pa*sample.pressure_slope*wc;
                stage_map_fallback=sample.weight<real(1)-real(1e-12);
            } else stage_map_fallback=true;
        }
        // Pressure equilibrium includes compressor capacity; actual manifold
        // filling remains the engine's existing mass balance through its throttle.
        // I7d D4: the compressor makes head above ambient, the cooler drops dp of it; charge_pa is
        // the pressure AFTER the cooler (what the throttle and the boost controller see). With
        // dp == 0 both lines are the same arithmetic as before.
        const real dp = desc.intercooler_dp_pa_at_max_flow*flow*flow;
        const real desired = ambient_pa + head - dp;
        const real charge_old = charge_pa;
        const real alpha=h/(real(0.025)+h);
        // Resolve the stiff pressure/flow feedback semi-implicitly outside
        // the table domain. The flow/head slope grows with rotor energy;
        // damping the iteration changes its transient, not its equilibrium.
        // Near an open throttle the pressure difference is small and the
        // orifice's flow sensitivity is much greater than density alone.
        const real flow_gain=p_im>real(0.0) && charge_old>p_im ?
            real(1.0)+real(0.5)*p_im/std::max(ambient_pa*real(0.001),charge_old-p_im):real(1.0);
        const real slope=compressor_stage_used ? std::max(real(0),stage_slope*flow_gain/std::max(ambient_pa,charge_old)) : !overspeed ? real(0.0) :
            real(2.0)*desc.reference_head_pa*speed_edge*speed_edge*flow*flow*flow_gain/std::max(ambient_pa,charge_old);
        charge_pa += alpha/(real(1.0)+alpha*slope)*(desired-charge_pa);
        const real pr = std::max(real(1.0),(charge_pa+dp)/ambient_pa);
        intercooler_dp_pa = dp; compressor_outlet_pa = charge_pa + dp;
        const real rise = ps::math::pow(pr,real(2.0/7.0))-real(1.0);
        const real outlet_k = compressor_stage_used ? ambient_k+stage_work/stages->compressor_cp : ambient_k*(desc.maps && pr<=desc.maps->compressor_temperature_ratio.x.back() ? desc.maps->compressor_temperature_ratio.evaluate(pr) : real(1.0)+rise/desc.compressor_efficiency);
        charge_k = ambient_k + (outlet_k-ambient_k)*(real(1.0)-desc.intercooler_effectiveness);
        // The compressor power is bounded by the head the CURRENT rotor speed makes at this flow:
        // charge_pa lags the head by 25 ms, and after a lift the slowing wheel must not keep paying
        // for a pressure ratio it no longer produces (the stored charge above that head is not
        // compressor work). Outlet temperature and the published ratio stay on the charge state.
        const real pr_wheel = real(1.0) + std::max(real(0.0),head)/ambient_pa;
        const real rise_w = pr <= pr_wheel ? rise : ps::math::pow(pr_wheel,real(2.0/7.0))-real(1.0);
        compressor_w = compressor_stage_used ? comp_flow*stage_work : comp_flow*real(1005.0)*ambient_k*rise_w/desc.compressor_efficiency;
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
        // it to 1 and overshoots the boost target). Below 10 % plate
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
        // Wastegate: a bypass orifice in parallel with the turbine. Both see the same manifold
        // pressure, so the exhaust splits by effective area (the same incompressible orifice
        // form as exhaust_pa below): turbine share A_t/(A_t + A_wg*opening), and the manifold
        // pressure follows from the total flow through the combined area. A fully open gate
        // therefore still drives the turbine (a real turbo idles at tens of thousands of rpm).
        const real total_flow = std::max(real(0.0),exhaust_kg_s);
        const real flow_area = desc.turbine_area_m2 + desc.wastegate_area()*wastegate;
        const real turbine_flow = total_flow*(desc.turbine_area_m2/flow_area);
        // Exhaust thermal mass: the whole exhaust flow (turbine and wastegate path share the
        // manifold) passes a lumped wall of heat capacity C, gas-side conductance K and outer
        // conductance Ka. Well-stirred gas side: the gas leaves at T_out = (G*T_port + K*T_w)/(G + K),
        // G = cp*mdot, so it gives the wall Keff*(T_port - T_w), Keff = K*G/(K + G) - bounded for
        // every flow, and at vanishing flow the gas leaves at the wall temperature (after a lift-off
        // the wall gives heat back to the low flow). The wall step is backward Euler (unconditionally
        // stable), so C*(T_w' - T_w) = h*(gas_to_wall - wall_to_ambient) holds to round-off.
        {
            const real g = kExhaustCp*std::max(real(0.0),exhaust_kg_s);
            const real tg = std::max(port_gas_k,ambient_k);
            const real k = desc.exhaust_wall_gas_conductance_w_k, ka = desc.exhaust_wall_ambient_conductance_w_k;
            const real keff = (g > real(0.0) && k > real(0.0)) ? k*g/(k+g) : real(0.0);
            const real c = desc.exhaust_wall_heat_capacity_j_k;
            wall_k = (c*wall_k + h*(keff*tg + ka*ambient_k))/(c + h*(keff + ka));
            gas_to_wall_w = keff*(tg - wall_k);
            wall_to_ambient_w = ka*(wall_k - ambient_k);
            port_k = tg;
            exhaust_k = (g + k > real(0.0)) ? (g*tg + k*wall_k)/(g + k) : tg;
        }
        const real rho = ambient_pa/(real(287.05)*exhaust_k);
        exhaust_pa = ambient_pa + total_flow*total_flow/(real(2.0)*rho*flow_area*flow_area);
        if(desc.compressible_exhaust)
            exhaust_pa=compressible_exhaust_pressure(total_flow,flow_area,ambient_pa,exhaust_k);
        turbine_w = turbine_flow*kExhaustCp*exhaust_k*desc.turbine_efficiency*
            (real(1.0)-ps::math::pow(ambient_pa/exhaust_pa,real(0.24812030075188)));

        real starting_torque=0;
        if(stages) {
            real p=0, power=0;
            if(stages->turbine_at(rpm(),total_flow,ambient_pa,exhaust_k,desc.wastegate_area()*wastegate,p,power,starting_torque)) {
                turbine_stage_used=true; exhaust_pa=p; turbine_w=power;
            } else stage_map_fallback=true;
        }
        const real loss = desc.shaft_loss_w_at_reference*speed2;
        // No artificial rotor-speed ceiling: shaft power determines speed.
        // Overspeed damage/protection is a separate future model.
        const bool at_rest=energy_j==real(0);
        energy_j = std::max(real(0.0),energy_j+h*(turbine_w-compressor_w-loss));
        if(at_rest && turbine_stage_used && starting_torque>real(0))
            energy_j+=real(0.5)*h*h*starting_torque*starting_torque/desc.rotor_inertia_kgm2;
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
