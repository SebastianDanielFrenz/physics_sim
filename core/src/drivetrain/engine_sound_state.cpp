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
    ess.bov_opening=pre.bov_opening; ess.bov_flow_kg_s=pre.bov_flow_kg_s;
    ess.bov_vent=pre.bov_vent; ess.bov_events=pre.bov_events;
    ess.p_intake_pa = pre.p_intake_pa;
    ess.turbo_rpm=pre.turbo_rpm; ess.boost_bar=pre.boost_bar; ess.p_exhaust_pa=pre.p_exhaust_pa;
    ess.lambda = pre.lambda;
    ess.spark_advance_deg = pre.spark_advance_deg;
    ess.exhaust_mass_flow_kg_s = pre.exhaust_mass_flow_kg_s;
    return ess;
}

} // namespace ps::drivetrain
