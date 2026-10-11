#include "engine_sound_state.h"

#include "powertrain.h"

namespace ps::drivetrain {

EngineSoundState fill_engine_sound_state(const Powertrain& pt) {
    EngineSoundState ess;
    if (!pt.has_engine()) return ess;

    const IEngine& eng = pt.engine();
    const EngineTorqueMap::PreSolveResult& pre = pt.last_engine_result();

    ess.crank_angle = eng.crank_angle();
    ess.omega = pt.engine_omega();
    // PHYS-033 P4 I7a C2: theta_c (the post-limiter combustion fraction) is
    // published directly on PreSolveResult.
    ess.load = pre.theta_c;
    ess.cut_mask = pre.limiter_active || pre.dfco_active ? ~std::uint64_t(0) : std::uint64_t(0);
    // pre.state is already ps::drivetrain::EngineState (EngineTorqueMap::
    // EngineState is a `using` alias to the same public enum, engine_torque_
    // map.h:45) - no mapping needed.
    ess.engine_state = pre.state;
    ess.starter_active = pre.starter_active;
    ess.limiter_active = pre.limiter_active;
    ess.dfco_active = pre.dfco_active;
    ess.launch_active = pt.launch_control_active(); // engine garage P8
    // PHYS-033 P4 I7a C3: the throttle-command stage's combined command this
    // tick - sound only, never hashed.
    ess.throttle_command = pt.throttle_command();
    // PHYS-033 P4 I7a C5, broadened C5d: nonzero for a fuel map or the crude
    // no-map estimate, 0 for an engine with no fuel model at all.
    ess.fuel_flow_g_s = pre.fuel_flow_g_s;
    // PHYS-033 P4 I7b C12: mean-value engine sound state (0 from the graph
    // engine, whose PreSolveResult leaves these at default).
    ess.throttle_position = pre.throttle_position;
    ess.p_charge_pa=pre.p_charge_pa; ess.t_charge_k=pre.t_charge_k;
    ess.wastegate_opening=pre.wastegate_opening; ess.compressor_flow_kg_s=pre.compressor_flow_kg_s;
    ess.compressor_pressure_ratio=pre.compressor_pressure_ratio;
    ess.turbo_compressor_stage_used=pre.turbo_compressor_stage_used;
    ess.turbo_turbine_stage_used=pre.turbo_turbine_stage_used;
    ess.turbo_stage_map_fallback=pre.turbo_stage_map_fallback;
    ess.bov_opening=pre.bov_opening; ess.bov_flow_kg_s=pre.bov_flow_kg_s;
    ess.bov_vent=pre.bov_vent; ess.bov_events=pre.bov_events;
    ess.surge_active=pre.surge_active; ess.surge_events=pre.surge_events;
    ess.anti_lag_active=pre.anti_lag_active; ess.anti_lag_activations=pre.anti_lag_activations;
    ess.anti_lag_extra_exhaust_w=pre.anti_lag_extra_exhaust_w;
    ess.exhaust_port_k=pre.exhaust_port_k; ess.turbine_inlet_k=pre.turbine_inlet_k; ess.exhaust_wall_k=pre.exhaust_wall_k;
    ess.anti_lag_manifold_burn_w=pre.anti_lag_manifold_burn_w;
    ess.p_intake_pa = pre.p_intake_pa;
    ess.turbo_rpm=pre.turbo_rpm; ess.boost_bar=pre.boost_bar; ess.p_exhaust_pa=pre.p_exhaust_pa;
    ess.lambda = pre.lambda;
    ess.spark_advance_deg = pre.spark_advance_deg;
    ess.exhaust_mass_flow_kg_s = pre.exhaust_mass_flow_kg_s;
    if (const NitrousSystem* n2o = pt.nitrous()) {
        const NitrousStatus& ns = n2o->status();
        ess.n2o_active = ns.flowing ? std::uint8_t(1) : std::uint8_t(0);
        ess.n2o_safety_cut = static_cast<std::uint8_t>(ns.safety_cut);
        ess.n2o_flow_g_s = ns.mdot_n_kg_s * real(1000.0);
        ess.n2o_purge_flow_g_s = ns.mdot_purge_kg_s * real(1000.0);
        ess.n2o_bottle_kg = n2o->bottle_kg();
        ess.n2o_bottle_bar = n2o->bottle_gauge_bar();
        ess.n2o_retard_deg = ns.retard_deg;
        ess.n2o_activations = n2o->activations();
        ess.n2o_purge_events = n2o->purge_events();
        ess.n2o_solenoid_open = ns.solenoid_open ? std::uint8_t(1) : std::uint8_t(0); // N2O plan N4
        ess.n2o_flow_fraction = ns.flow_fraction;
        ess.n2o_purge_active = ns.purge_active ? std::uint8_t(1) : std::uint8_t(0);
        ess.n2o_injection = n2o->kit().injection == NitrousInjection::DirectPort ? std::uint8_t(1) : std::uint8_t(0);
        ess.charge_temperature_k = pre.charge_temperature_k; // N2O plan N2 (0 unless the charge model ran)
        ess.n2o_lambda_combined = pre.n2o_lambda_combined;
        ess.n2o_kit_fuel_g_s = pre.n2o_kit_fuel_g_s;
    }
    return ess;
}

} // namespace ps::drivetrain
