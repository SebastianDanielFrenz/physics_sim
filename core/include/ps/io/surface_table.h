// ps/io/surface_table.h — shared name<->SurfaceId table (PLAN.md section 6:
// "surface ids must survive collision queries and contact events, so the
// tyre/contact code can look up a surface"). Loads a surfaces.json-shaped
// file (data/surfaces/surfaces.json: name -> {lambda_mu, crr, ...}) and
// assigns each named surface a SurfaceId, so a static collision loader
// (core/src/io/test_ground_loader.h) and a future real-world land-cover
// source can both tag their geometry with ids that mean the same thing.
//
// nlohmann::json stays confined to surface_table.cpp - this header only
// exposes SurfaceId (ps/types.h) and plain strings.

#pragma once

#include "ps/types.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace ps::io {

class SurfaceTable {
public:
    // Throws std::runtime_error on a malformed or unreadable file.
    explicit SurfaceTable(const std::string& surfaces_json_path);

    // Ids are assigned by ALPHABETICAL order of the surface name, not raw
    // file order: nlohmann::json's default object type (std::map-backed)
    // does not preserve a file's own key order, and PLAN.md D7 needs this
    // assignment to be deterministic regardless of how the JSON happens to
    // be formatted. Two SurfaceTable instances loading the SAME
    // surfaces.json always agree on every name's id.
    [[nodiscard]] SurfaceId id_for(const std::string& name) const;
    // kInvalidSurfaceId if `id` is out of range (e.g. never assigned by
    // this table).
    [[nodiscard]] const std::string& name_for(SurfaceId id) const;
    [[nodiscard]] std::size_t size() const { return names_.size(); }

    // PLAN.md section 6 / P3: the tyre model is the first consumer of the
    // actual physical values (lambda_mu, crr) rather than just the
    // name<->id assignment above. 1.0 / 0.0 (dry-asphalt-like, no rolling
    // resistance) for kInvalidSurfaceId or an id this table never assigned,
    // so a caller that gets no surface id from a collision query (e.g. a
    // plain Box/Sphere shape with no per-triangle material) still gets a
    // sane default rather than undefined behaviour.
    [[nodiscard]] real lambda_mu_for(SurfaceId id) const;
    [[nodiscard]] real crr_for(SurfaceId id) const;

private:
    std::vector<std::string> names_;                      // index == SurfaceId
    std::vector<real> lambda_mu_;                          // index == SurfaceId
    std::vector<real> crr_;                                // index == SurfaceId
    std::unordered_map<std::string, SurfaceId> name_to_id_;
};

} // namespace ps::io
