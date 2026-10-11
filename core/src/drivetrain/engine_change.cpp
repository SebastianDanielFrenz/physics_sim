// core/src/drivetrain/engine_change.cpp - engine garage P4 (A11): classify_engine_change / list_engine_changes / turbo_table_key.
//
// The classification is DERIVED from the caches, not parallel to them:
//   Maps        <- ps::drivetrain::engine_map_cache_key (core/src/engine_cycle/map_cache.cpp) called on both descs;
//   TurboTables <- turbo_table_key, the same parameter list as the turbo-table key in ps::io::load_turbo_configuration_json
//                  (the test compares it with that loader's maps->content_hash).
// Every other difference is Runtime; identical descriptors are None. The per-field table below only has to be complete for
// None-versus-Runtime (a field it forgets reads as None, which tests/unit/test_engine_rules.cpp catches field by field) - it
// cannot make a map edit look cheap.
#include "ps/drivetrain/engine_rules.h"

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/turbo_maps.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <variant>

namespace ps::drivetrain {

namespace {

// ---- exact comparison helpers (any bit-level difference counts; NaN never occurs in a loaded descriptor) ------------------
[[nodiscard]] bool same(const vehicle::Curve1D& a, const vehicle::Curve1D& b) { return a.x == b.x && a.y == b.y; }
[[nodiscard]] bool same(const Table2D& a, const Table2D& b) { return a.x == b.x && a.y == b.y && a.values == b.values; }
[[nodiscard]] bool same_tables(const std::vector<Table2D>& a, const std::vector<Table2D>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!same(a[i], b[i])) return false;
    }
    return true;
}
[[nodiscard]] bool same(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
[[nodiscard]] bool same(const IdleControlDesc& a, const IdleControlDesc& b) {
    return a.kp_per_rpm == b.kp_per_rpm && a.ki_per_rpm_s == b.ki_per_rpm_s && a.max_throttle == b.max_throttle &&
           a.max_torque_nm == b.max_torque_nm;
}
[[nodiscard]] bool same(const StarterDesc& a, const StarterDesc& b) {
    return a.stall_torque_nm == b.stall_torque_nm && a.free_speed_rpm == b.free_speed_rpm && a.interlock == b.interlock;
}
[[nodiscard]] bool same(const GovernorDesc& a, const GovernorDesc& b) {
    return a.high_idle_rpm == b.high_idle_rpm && a.rated_rpm == b.rated_rpm && a.droop == b.droop && a.channel == b.channel;
}
// FuelDesc::source is provenance text only (the generator never reads it, engine_map_cache_key excludes it).
[[nodiscard]] bool same(const FuelDesc& a, const FuelDesc& b) {
    return a.name == b.name && a.phase == b.phase && a.lhv_j_kg == b.lhv_j_kg && a.afr_stoich == b.afr_stoich &&
           a.density_kg_m3 == b.density_kg_m3 && a.has_knock == b.has_knock &&
           (!a.has_knock || (a.knock.model == b.knock.model && a.knock.ron == b.knock.ron && a.knock.mon == b.knock.mon));
}
[[nodiscard]] bool same(const std::vector<FuelDesc>& a, const std::vector<FuelDesc>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!same(a[i], b[i])) return false;
    }
    return true;
}

// ---- turbo --------------------------------------------------------------------------------------------------------------
[[nodiscard]] std::uint64_t stage_state(const TurboPairDesc& t) {
    return (t.maps && t.maps->stages) ? t.maps->content_hash : std::uint64_t(0);
}

[[nodiscard]] bool turbo_tables_differ(const TurboPairDesc& a, const TurboPairDesc& b) {
    return turbo_table_key(a) != turbo_table_key(b) || stage_state(a) != stage_state(b);
}

// The turbo fields that feed NO derived table (the head map depends on the key parameters only): applied by the engine at
// construction / by World::replace_turbo_configuration. Provenance strings (BlowOffValveDesc::source ...) are not compared.
[[nodiscard]] bool turbo_runtime_differs(const TurboPairDesc& a, const TurboPairDesc& b, const char** pointer, const char** slot) {
    const auto hit = [&](const char* sl, const char* p) {
        *slot = sl;
        *pointer = p;
        return true;
    };
    if (a.compressible_exhaust != b.compressible_exhaust) return hit("forced_induction", "/compressible_exhaust");
    if (a.wastegate_area_m2.has_value() != b.wastegate_area_m2.has_value() ||
        (a.wastegate_area_m2 && *a.wastegate_area_m2 != *b.wastegate_area_m2)) {
        return hit("turbo", "/wastegate_area_m2");
    }
    if (a.intercooler_dp_pa_at_max_flow != b.intercooler_dp_pa_at_max_flow) {
        return hit("forced_induction", "/intercooler_pressure_drop_bar_at_max_flow");
    }
    if (a.blow_off_valve.has_value() != b.blow_off_valve.has_value()) return hit("forced_induction", "/blow_off_valve");
    if (a.blow_off_valve) {
        const auto& x = *a.blow_off_valve;
        const auto& y = *b.blow_off_valve;
        if (x.vent != y.vent || x.crack_dp_pa != y.crack_dp_pa || x.full_open_dp_pa != y.full_open_dp_pa ||
            x.effective_area_m2 != y.effective_area_m2 || x.tau_s != y.tau_s) {
            return hit("forced_induction", "/blow_off_valve");
        }
    }
    if (a.surge_line.has_value() != b.surge_line.has_value() ||
        (a.surge_line && !same(a.surge_line->flow_fraction_vs_pr, b.surge_line->flow_fraction_vs_pr))) {
        return hit("turbo", "/surge_line");
    }
    if (a.anti_lag.has_value() != b.anti_lag.has_value()) return hit("forced_induction", "/anti_lag");
    if (a.anti_lag) {
        const auto& x = *a.anti_lag;
        const auto& y = *b.anti_lag;
        if (x.channel != y.channel || x.default_enabled != y.default_enabled || x.arm_below_command != y.arm_below_command ||
            x.min_rpm != y.min_rpm || x.max_rpm != y.max_rpm || x.retard_deg != y.retard_deg || x.plate_floor != y.plate_floor ||
            x.max_active_s != y.max_active_s || x.spark_advance_deg.has_value() != y.spark_advance_deg.has_value() ||
            (x.spark_advance_deg && *x.spark_advance_deg != *y.spark_advance_deg) ||
            x.cylinder_cut_fraction != y.cylinder_cut_fraction || x.manifold_burn_fraction != y.manifold_burn_fraction ||
            x.lambda != y.lambda || x.boost_target_fraction != y.boost_target_fraction) {
            return hit("forced_induction", "/anti_lag");
        }
    }
    if (a.in_cylinder_wall_heat_fraction != b.in_cylinder_wall_heat_fraction ||
        a.exhaust_wall_heat_capacity_j_k != b.exhaust_wall_heat_capacity_j_k ||
        a.exhaust_wall_gas_conductance_w_k != b.exhaust_wall_gas_conductance_w_k ||
        a.exhaust_wall_ambient_conductance_w_k != b.exhaust_wall_ambient_conductance_w_k ||
        a.exhaust_wall_initial_k != b.exhaust_wall_initial_k) {
        return hit("forced_induction", "/exhaust_thermal");
    }
    return false;
}

// ---- the cycle block ------------------------------------------------------------------------------------------------------
// The cycle exactly as the loader hands it to the generator: the throttle is compressor-fed iff the engine has a turbo (and
// pipe blocks exist - the flag is meaningful, and keyed, only there).
[[nodiscard]] CycleModelDesc effective_cycle(const SimulatedEngineDesc& e) {
    CycleModelDesc c = *e.cycle;
    if (c.intake.has_value()) c.intake->compressor_supply = e.turbo_pair.has_value();
    // Engine garage P7: a knock-limited engine's knock fuels are the engine's fuels list with their knock blocks (the
    // loader's rule), so a fuels-list or octane edit made on the desc alone is classified like a reloaded file (Maps).
    if (c.knock.enabled) {
        c.knock.lambda_target = e.ecu.lambda_target; // P7b: the loader's rule (the replay's lambda)
        c.knock.fuels.clear();
        for (const FuelDesc& f : e.fuels) {
            c.knock.fuels.push_back(CycleModelDesc::KnockFuel{f.name, f.has_knock ? f.knock.ron : real(0.0), f.has_knock ? f.knock.mon : real(0.0)});
        }
    }
    return c;
}

[[nodiscard]] const FuelDesc& key_fuel(const SimulatedEngineDesc& e) {
    static const FuelDesc kNone{};
    return e.fuels.empty() ? kNone : e.fuels.front();
}

// Cycle fields OUTSIDE the cache key that still change what the engine does (audio source layout): Runtime. Paths and citation
// strings are provenance only: None.
[[nodiscard]] bool cycle_nonkey_differs(const CycleModelDesc& a, const CycleModelDesc& b) {
    if (a.banks != b.banks || a.bank_angle_deg != b.bank_angle_deg) return true;
    if (a.installation.has_value() != b.installation.has_value()) return true; // (also a key change for pipe engines)
    if (a.installation) {
        if (!same(a.installation->intake.inlet_position_m, b.installation->intake.inlet_position_m)) return true;
        const auto& x = a.installation->exhaust.elements;
        const auto& y = b.installation->exhaust.elements;
        if (x.size() != y.size()) return true;
        for (std::size_t i = 0; i < x.size(); ++i) {
            if (!same(x[i].position_m, y[i].position_m)) return true;
        }
    }
    return false;
}

void push(std::vector<EngineFieldChange>& out, const char* slot, const char* pointer, EngineChange c) {
    out.push_back(EngineFieldChange{slot, pointer, c});
}

std::vector<EngineFieldChange> list_simulated(const SimulatedEngineDesc& a, const SimulatedEngineDesc& b) {
    std::vector<EngineFieldChange> out;
    const auto rt = [&](bool differs, const char* pointer) {
        if (differs) push(out, "engine", pointer, EngineChange::Runtime);
    };

    // --- Respawn: nothing can be rebuilt from the old engine ---
    if (a.architecture != b.architecture) push(out, "engine", "/architecture", EngineChange::Respawn);
    if (a.ignition != b.ignition) push(out, "engine", "/ignition", EngineChange::Respawn);
    // The cylinder count: design 6.3 "a different engine: swap". The map key does cover it (cycle.cylinders), but a cylinder
    // count is also the firing order, the inertia and the number of audio sources - nothing carries over, so Respawn wins.
    const bool both_cycle = a.cycle.has_value() && b.cycle.has_value();
    if (a.piston.cylinders != b.piston.cylinders || (both_cycle && a.cycle->cylinders != b.cycle->cylinders)) {
        push(out, "engine", "/cylinders", EngineChange::Respawn);
    }
    if (a.rotary.rotors != b.rotary.rotors) push(out, "engine", "/cylinders", EngineChange::Respawn);

    // --- Maps: the generator's inputs, by the cache key itself ---
    if (a.cycle.has_value() != b.cycle.has_value()) {
        push(out, "engine", "/cycle", EngineChange::Maps);
    } else if (both_cycle) {
        const CycleModelDesc ca = effective_cycle(a);
        const CycleModelDesc cb = effective_cycle(b);
        if (engine_map_cache_key(ca, key_fuel(a), MapQuality::Full) != engine_map_cache_key(cb, key_fuel(b), MapQuality::Full)) {
            push(out, "engine", "/cycle", EngineChange::Maps);
        } else if (cycle_nonkey_differs(*a.cycle, *b.cycle)) {
            push(out, "engine", "/cycle", EngineChange::Runtime);
        }
    }

    // --- TurboTables / turbo Runtime ---
    if (a.turbo_pair.has_value() != b.turbo_pair.has_value()) {
        push(out, "forced_induction", "", EngineChange::TurboTables);
    } else if (a.turbo_pair) {
        if (turbo_tables_differ(*a.turbo_pair, *b.turbo_pair)) push(out, "forced_induction", "", EngineChange::TurboTables);
        const char* pointer = "";
        const char* slot = "";
        if (turbo_runtime_differs(*a.turbo_pair, *b.turbo_pair, &pointer, &slot)) push(out, slot, pointer, EngineChange::Runtime);
    }

    // --- Runtime: everything else the engine constructor reads ---
    // Bore and stroke: the loader copies them into the cycle block (cycle.bore_m = piston.bore_m), so on a cycle-class engine they
    // are generator inputs even when only the piston fields of a descriptor were edited (design 6.3: "M for every row").
    if (a.piston.bore_m != b.piston.bore_m) push(out, "engine", "/bore_mm", both_cycle ? EngineChange::Maps : EngineChange::Runtime);
    if (a.piston.stroke_m != b.piston.stroke_m) push(out, "engine", "/stroke_mm", both_cycle ? EngineChange::Maps : EngineChange::Runtime);
    rt(a.rotary.rotor_radius_m != b.rotary.rotor_radius_m || a.rotary.eccentricity_m != b.rotary.eccentricity_m ||
           a.rotary.rotor_width_m != b.rotary.rotor_width_m,
       "/rotary");
    rt(a.intake_manifold_volume_m3 != b.intake_manifold_volume_m3, "/intake_manifold_volume_l");
    rt(a.throttle_body.diameter_m != b.throttle_body.diameter_m || a.throttle_body.count != b.throttle_body.count ||
           a.throttle_body.discharge_coefficient != b.throttle_body.discharge_coefficient ||
           !same(a.throttle_body.area_fraction_vs_position, b.throttle_body.area_fraction_vs_position),
       "/throttle_body");
    rt(a.throttle_input != b.throttle_input, "/throttle_input");
    rt(a.throttle_actuator_tau_s != b.throttle_actuator_tau_s, "/throttle_actuator_tau_s");
    rt(a.idle_bypass_area_m2 != b.idle_bypass_area_m2 || !same(a.idle, b.idle), "/idle_control");
    rt(!same(a.ecu.lambda_target, b.ecu.lambda_target), "/ecu/lambda_target");
    rt(!same(a.ecu.spark_advance_deg, b.ecu.spark_advance_deg), "/ecu/spark_advance_deg");
    rt(!same(a.maps.ve, b.maps.ve) || !same(a.maps.eta_i, b.maps.eta_i) || !same(a.maps.f_lambda, b.maps.f_lambda) ||
           !same(a.maps.eta_spark, b.maps.eta_spark) || !same(a.maps.theta_mbt_deg, b.maps.theta_mbt_deg) ||
           !same(a.maps.friction_torque_nm, b.maps.friction_torque_nm) || a.maps.has_pmep_excess != b.maps.has_pmep_excess ||
           !same(a.maps.pmep_excess_pa, b.maps.pmep_excess_pa) || a.maps.has_theta_kl != b.maps.has_theta_kl ||
           !same(a.maps.theta_kl_deg, b.maps.theta_kl_deg) || !same_tables(a.maps.theta_kl_by_fuel, b.maps.theta_kl_by_fuel) ||
           a.maps_pending != b.maps_pending,
       "/maps");
    rt(a.inertia_kgm2 != b.inertia_kgm2, "/inertia_kgm2");
    rt(a.breakaway_torque_nm != b.breakaway_torque_nm, "/breakaway_torque_nm");
    rt(a.idle_rpm != b.idle_rpm, "/idle_rpm");
    rt(a.stall_rpm != b.stall_rpm, "/stall_rpm");
    rt(a.fire_rpm != b.fire_rpm, "/fire_rpm");
    rt(a.start_complete_rpm != b.start_complete_rpm, "/start_complete_rpm");
    rt(a.dfco_enable_rpm != b.dfco_enable_rpm, "/dfco_enable_rpm");
    rt(a.dfco_reenable_rpm != b.dfco_reenable_rpm, "/dfco_reenable_rpm");
    rt(a.limiter.type != b.limiter.type, "/limiter/type");
    rt(a.limiter.rpm != b.limiter.rpm, "/limiter/rpm");
    rt(a.limiter.hysteresis_rpm != b.limiter.hysteresis_rpm, "/limiter/hysteresis_rpm");
    rt(a.limiter.band_rpm != b.limiter.band_rpm, "/limiter/band_rpm");
    rt(!same(a.starter, b.starter), "/starter");
    rt(!same(a.crank_axis_local, b.crank_axis_local), "/crank_axis_local");
    rt(a.initial_state != b.initial_state, "/initial_state");
    rt(!same(a.fuels, b.fuels), "/fuels");
    return out;
}

std::vector<EngineFieldChange> list_torque_map(const TorqueMapEngineDesc& a, const TorqueMapEngineDesc& b) {
    std::vector<EngineFieldChange> out;
    const auto rt = [&](bool differs, const char* pointer) {
        if (differs) push(out, "engine", pointer, EngineChange::Runtime);
    };
    if (a.ignition != b.ignition) push(out, "engine", "/ignition", EngineChange::Respawn);
    rt(!same(a.wot_torque_nm_vs_rpm, b.wot_torque_nm_vs_rpm), "/wot_torque_nm_vs_rpm");
    rt(!same(a.motoring_torque_nm_vs_rpm, b.motoring_torque_nm_vs_rpm), "/motoring_torque_nm_vs_rpm");
    rt(a.inertia_kgm2 != b.inertia_kgm2, "/inertia_kgm2");
    rt(a.breakaway_torque_nm != b.breakaway_torque_nm, "/breakaway_torque_nm");
    rt(a.idle_rpm != b.idle_rpm, "/idle_rpm");
    rt(a.stall_rpm != b.stall_rpm, "/stall_rpm");
    rt(a.fire_rpm != b.fire_rpm, "/fire_rpm");
    rt(a.start_complete_rpm != b.start_complete_rpm, "/start_complete_rpm");
    rt(a.dfco_enable_rpm != b.dfco_enable_rpm, "/dfco_enable_rpm");
    rt(a.dfco_reenable_rpm != b.dfco_reenable_rpm, "/dfco_reenable_rpm");
    rt(!same(a.idle, b.idle), "/idle_control");
    rt(a.limiter.type != b.limiter.type, "/limiter/type");
    rt(a.limiter.rpm != b.limiter.rpm, "/limiter/rpm");
    rt(a.limiter.hysteresis_rpm != b.limiter.hysteresis_rpm, "/limiter/hysteresis_rpm");
    rt(a.limiter.band_rpm != b.limiter.band_rpm, "/limiter/band_rpm");
    rt(!same(a.starter, b.starter), "/starter");
    rt(!same(a.crank_axis_local, b.crank_axis_local), "/crank_axis_local");
    rt(a.initial_state != b.initial_state, "/initial_state");
    rt(a.has_governor != b.has_governor || !same(a.governor, b.governor), "/governor");
    rt(a.throttle_input != b.throttle_input, "/throttle_input");
    rt(a.has_fuel_map != b.has_fuel_map || !same(a.fuel, b.fuel) || !same(a.fuel_map, b.fuel_map), "/fuel_map");
    rt(a.has_ignition != b.has_ignition, "/ignition");
    rt(!same(a.fuels, b.fuels), "/fuels");
    rt(a.indicated_efficiency != b.indicated_efficiency, "/indicated_efficiency");
    return out;
}

} // namespace

std::uint64_t turbo_table_key(const TurboPairDesc& t) {
    // Mirrors core/src/io/vehicle_io.cpp (load_turbo_configuration_json): FNV-1a over the bytes of this parameter list, in this
    // order. A new key parameter there must be added here; tests/unit/test_engine_rules.cpp compares both on the shipped
    // configurations.
    std::uint64_t key = 14695981039346656037ULL;
    const real parameters[] = {real(1.0),
                               t.reference_head_pa,
                               t.rotor_inertia_kgm2,
                               t.reference_rpm,
                               t.max_mass_flow_kg_s,
                               t.compressor_efficiency,
                               t.turbine_efficiency,
                               t.turbine_area_m2,
                               t.target_boost_pa,
                               t.intercooler_effectiveness,
                               t.shaft_loss_w_at_reference,
                               t.wastegate_kp,
                               t.wastegate_ki_per_s,
                               t.wastegate_kd_s,
                               t.wastegate_actuator_tau_s};
    for (real v : parameters) {
        unsigned char bytes[sizeof(v)];
        std::memcpy(bytes, &v, sizeof(v));
        for (unsigned char b : bytes) {
            key ^= b;
            key *= 1099511628211ULL;
        }
    }
    return key;
}

const char* to_string(EngineChange c) {
    switch (c) {
    case EngineChange::None: return "None";
    case EngineChange::Runtime: return "Runtime";
    case EngineChange::TurboTables: return "TurboTables";
    case EngineChange::Maps: return "Maps";
    case EngineChange::Respawn: return "Respawn";
    }
    return "Unknown";
}

std::vector<EngineFieldChange> list_engine_changes(const ComponentParams& before, const ComponentParams& after) {
    const auto* sa = std::get_if<SimulatedEngineDesc>(&before);
    const auto* sb = std::get_if<SimulatedEngineDesc>(&after);
    const auto* ta = std::get_if<TorqueMapEngineDesc>(&before);
    const auto* tb = std::get_if<TorqueMapEngineDesc>(&after);
    if ((sa == nullptr && ta == nullptr) || (sb == nullptr && tb == nullptr)) {
        throw std::invalid_argument("classify_engine_change: both arguments must be engines (TorqueMapEngineDesc or SimulatedEngineDesc)");
    }
    if (sa != nullptr && sb != nullptr) return list_simulated(*sa, *sb);
    if (ta != nullptr && tb != nullptr) return list_torque_map(*ta, *tb);
    // A different engine kind: nothing carries over.
    return {EngineFieldChange{"engine", "/kind", EngineChange::Respawn}};
}

EngineChange classify_engine_change(const ComponentParams& before, const ComponentParams& after) {
    EngineChange worst = EngineChange::None;
    for (const EngineFieldChange& c : list_engine_changes(before, after)) worst = std::max(worst, c.change);
    return worst;
}

EngineChange classify_engine_change(const SimulatedEngineDesc& before, const SimulatedEngineDesc& after) {
    EngineChange worst = EngineChange::None;
    for (const EngineFieldChange& c : list_simulated(before, after)) worst = std::max(worst, c.change);
    return worst;
}

EngineChange classify_engine_change(const TorqueMapEngineDesc& before, const TorqueMapEngineDesc& after) {
    EngineChange worst = EngineChange::None;
    for (const EngineFieldChange& c : list_torque_map(before, after)) worst = std::max(worst, c.change);
    return worst;
}

} // namespace ps::drivetrain
