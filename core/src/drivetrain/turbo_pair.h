#pragma once
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/math/transcendental.h"
#include "ps/drivetrain/turbo_maps.h"
#include <algorithm>
#include <cmath>

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
    explicit TurboPair(const TurboPairDesc& d) : desc(d) {}
    real omega() const { return std::sqrt(real(2.0)*energy_j/desc.rotor_inertia_kgm2); }
    real rpm() const { return omega()*real(60.0/6.2831853071795864769); }
    void reset(real p, real t) { energy_j=0; charge_pa=p; charge_k=t; exhaust_pa=p; wastegate=0; boost_integral=0; }
    // plate: the throttle plate position in [0, 1]; it scales the boost target.
    void step(real h, real ambient_pa, real ambient_k, real air_kg_s, real exhaust_kg_s,
              real fuel_w, real plate) {
        const real wr = desc.reference_rpm*real(6.2831853071795864769/60.0);
        const real speed2 = real(2.0)*energy_j/(desc.rotor_inertia_kgm2*wr*wr);
        const real flow = std::max(real(0.0),air_kg_s)/desc.max_mass_flow_kg_s;
        const real head = desc.maps ? desc.maps->head_pa.value(std::sqrt(speed2),flow) :
            desc.reference_head_pa*speed2*std::max(real(0.0),real(1.0)-flow*flow);
        // Pressure equilibrium includes compressor capacity; actual manifold
        // filling remains the engine's existing mass balance through its throttle.
        const real desired = ambient_pa + head;
        const real charge_old = charge_pa;
        charge_pa += h/(real(0.025)+h)*(desired-charge_pa);
        const real pr = std::max(real(1.0),charge_pa/ambient_pa);
        const real rise = ps::math::pow(pr,real(2.0/7.0))-real(1.0);
        const real outlet_k = ambient_k*(desc.maps ? desc.maps->compressor_temperature_ratio.evaluate(pr) : real(1.0)+rise/desc.compressor_efficiency);
        charge_k = ambient_k + (outlet_k-ambient_k)*(real(1.0)-desc.intercooler_effectiveness);
        compressor_w = std::max(real(0.0),air_kg_s)*real(1005.0)*ambient_k*rise/desc.compressor_efficiency;
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
        const real exhaust_k = std::clamp(ambient_k + real(0.25)*std::max(real(0.0),fuel_w)/
            (real(1100.0)*std::max(real(0.001),exhaust_kg_s)),ambient_k,real(1200.0));
        const real rho = ambient_pa/(real(287.05)*exhaust_k);
        exhaust_pa = ambient_pa + turbine_flow*turbine_flow/(real(2.0)*rho*desc.turbine_area_m2*desc.turbine_area_m2);
        turbine_w = turbine_flow*real(1100.0)*exhaust_k*desc.turbine_efficiency*
            (real(1.0)-ps::math::pow(ambient_pa/exhaust_pa,real(0.24812030075188)));
        const real loss = desc.shaft_loss_w_at_reference*speed2;
        energy_j = std::clamp(energy_j+h*(turbine_w-compressor_w-loss),real(0.0),
            real(0.5)*desc.rotor_inertia_kgm2*wr*wr*real(2.25));
    }
};
}
