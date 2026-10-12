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

// Engine garage P9: the one rule for putting an EngineMaps into a desc - profile 0's set into `maps`, every further
// cam profile's set (EngineMaps::cam_profiles) into cam_profile_maps. The set count must match the desc's profiles
// (a hot swap replaces every profile's maps together, never one of them).
inline void set_engine_maps(SimulatedEngineDesc& desc, const EngineMaps& maps, const char* who) {
    const std::size_t want = desc.cam_profile_ids.empty() ? 0 : desc.cam_profile_ids.size() - 1;
    if (maps.cam_profiles.size() != want) {
        throw std::runtime_error(std::string(who) + ": the engine maps carry " + std::to_string(maps.cam_profiles.size() + 1) +
                                 " cam profile set(s), the engine has " + std::to_string(want + 1) +
                                 " (engine garage P9: every profile's maps are replaced together)");
    }
    desc.maps = maps.maps;
    desc.cam_profile_maps.clear();
    for (const auto& p : maps.cam_profiles) {
        if (!p) throw std::runtime_error(std::string(who) + ": a null cam profile map set");
        desc.cam_profile_maps.push_back(p->maps);
    }
}

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
    set_engine_maps(updated, *maps, who);
    updated.maps_pending = false;
    return ComponentParams{std::move(updated)};
}

} // namespace ps::drivetrain
