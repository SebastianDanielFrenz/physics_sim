// core/src/drivetrain/engine_params_util.h - engine garage P3. Internal (not on ps_core's public include path).
//
// The one rule EngineRig's desc constructor and run_dyno_sweep share for an explicit EngineMapsPtr: it replaces the
// WHOLE `maps` block of a simulated engine desc, exactly what World::replace_engine_maps /
// Powertrain::request_replace_engine_maps do (`updated.maps = new_engine_maps->maps`), so a hot-swapped car, an
// EngineRig and a dyno sweep that are given the same EngineMaps run the same numbers.
#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace ps::drivetrain {

// `maps` null: `engine` unchanged. Otherwise `engine` must hold a SimulatedEngineDesc (else std::runtime_error);
// its maps are replaced by maps->maps and maps_pending is cleared - a generated EngineMaps is the COMPLETE set the
// pending desc lacked (the garage flow: validate-only load, then generate_engine_maps, then run). A desc that is
// still pending with maps == nullptr is left pending: SimulatedEngine / build_maps reject it with their own message.
[[nodiscard]] inline ComponentParams with_engine_maps(const ComponentParams& engine, const EngineMapsPtr& maps,
                                                       const char* who) {
    if (!maps) return engine;
    const SimulatedEngineDesc* sim = std::get_if<SimulatedEngineDesc>(&engine);
    if (sim == nullptr) {
        throw std::runtime_error(std::string(who) + ": explicit engine maps were given, but the engine is not a simulated engine");
    }
    SimulatedEngineDesc updated = *sim;
    // Engine garage P7d (a P7 gap): a knock-limited engine without an ECU spark table tracks MBT (the loader sets
    // ecu.spark_advance_deg = maps.theta_mbt_deg, vehicle_io.cpp) - but only once its maps exist, so a validate-only
    // (pending) load reaches here with an empty table. Apply the same rule to the maps that complete it.
    if (updated.maps_pending && updated.ecu.spark_advance_deg.x.empty() && maps->maps.has_theta_kl) {
        updated.ecu.spark_advance_deg = maps->maps.theta_mbt_deg;
    }
    updated.maps = maps->maps;
    updated.maps_pending = false;
    return ComponentParams{std::move(updated)};
}

} // namespace ps::drivetrain
