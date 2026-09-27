// core/src/terrain/procedural_terrain_source.h — internal: TerrainSpec (the
// parsed contents of a terrain.json file) and ProceduralTerrainSource (the
// ITerrainSource implementation that evaluates it). See
// ps/terrain/procedural_terrain.h for the public factory.
//
// Every formula here is a line-for-line port of
// tools/trackgen/trackgen/terrain.py (read, not modified - that package is
// owned by a separate session): a MurmurHash3-style 32-bit integer hash,
// bilinear value noise with a smoothstep-remapped fractional coordinate,
// fixed-parameter octaves, and disc/corridor flat-zone masks. PLAN.md D7:
// only integer hashing and plain double arithmetic (+,-,*,/, floor, sqrt) -
// no transcendental call anywhere in the height/surface path itself. The
// two flat-zone corridor headings (0 and 90 degrees in every terrain.json
// this project defines) go through ps::math::sin/cos exactly once each, at
// TerrainSpec load time (not per point), purely so this file never calls
// the platform libm at all, matching the rest of core/ - see
// TerrainSpec::Zone::forward/left below.

#pragma once

#include "ps/terrain/terrain_source.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::terrain {

struct Octave {
    real wavelength = 1.0;
    real amplitude = 0.0;
    std::uint32_t seed_offset = 0;
};

struct SurfaceRule {
    std::string surface;
    real wavelength = 1.0;
    std::uint32_t seed_offset = 0;
    real min_value01 = 0.5;
};

// A flat zone: either a disc (radius/blend around a centre) or an oriented
// rectangular corridor (centre, forward/left axes already resolved from
// heading_deg at load time, half_length/half_width, blend). See
// _disc_mask/_corridor_mask in terrain.py.
struct FlatZone {
    enum class Kind { Disc, Corridor } kind = Kind::Disc;
    Vec3 center = Vec3::zero(); // x, y used; z ignored
    real radius = 0.0;          // disc only
    real blend = 0.0;
    // Corridor only - forward/left unit axes in the world xy plane (z==0),
    // resolved once from heading_deg at spec-load time via ps::math.
    Vec3 forward = Vec3::unit_x();
    Vec3 left = Vec3::unit_y();
    real half_length = 0.0;
    real half_width = 0.0;
};

struct TerrainSpec {
    std::uint32_t seed = 0;
    std::vector<Octave> rolling_octaves;
    Octave hill_octave;
    real roughness_near_m = 0.0;
    real roughness_far_m = 0.0;
    std::vector<FlatZone> flat_zones;
    std::vector<SurfaceRule> surface_rules;
    std::string default_surface = "grass";
    real tile_size_m = 256.0;

    // id 0 == default_surface; then each rule's surface, in the order
    // first seen (skipping duplicates) - see surface_id_for_name below.
    std::vector<std::string> surface_names;

    [[nodiscard]] SurfaceId surface_id_for_name(const std::string& name) const {
        for (std::size_t i = 0; i < surface_names.size(); ++i) {
            if (surface_names[i] == name) return static_cast<SurfaceId>(i);
        }
        return kInvalidSurfaceId;
    }
};

// Parses a terrain.json file's contents (format "physics_sim.terrain/1")
// into a TerrainSpec. Throws std::runtime_error on a malformed file.
TerrainSpec load_terrain_spec(const std::string& path);

// height(x, y) - PLAN.md M1's acceptance test evaluates this directly
// against terrain_golden.json. Exposed as a free function (not only via
// ITerrainSource::fill_tile) so a test can probe it at exact golden-file
// coordinates without going through the tiling machinery.
real terrain_height(const TerrainSpec& spec, real x, real y);

// The per-cell surface id at tile (tile_x, tile_y), evaluated at the tile's
// centre point ((tile_x+0.5)*tile_size_m, ...) - see terrain.py's
// terrain_surface_cell. tile_size_m defaults to spec.tile_size_m.
SurfaceId terrain_surface_cell(const TerrainSpec& spec, std::int64_t tile_x, std::int64_t tile_y, real tile_size_m);

class ProceduralTerrainSource final : public ITerrainSource {
public:
    explicit ProceduralTerrainSource(TerrainSpec spec, std::uint32_t samples_per_tile = 64);

    [[nodiscard]] real tile_size_m() const override { return spec_.tile_size_m; }
    [[nodiscard]] std::uint32_t samples_per_tile() const override { return samples_per_tile_; }
    [[nodiscard]] TileKey key_for_world_xy(real x, real y) const override;
    [[nodiscard]] TileKey key_offset(TileKey key, std::int32_t dx, std::int32_t dy) const override;
    [[nodiscard]] Vec3 tile_origin_xy(TileKey key) const override;
    void fill_tile(TileKey key, TileSample& out) const override;
    // A pure function of the fixed spec: fill_tile is safe on TileManager's
    // background prefetch lane for any key (terrain prefetch ring).
    [[nodiscard]] PrefetchSupportKind prefetch_support() const override { return PrefetchSupportKind::Immutable; }

    [[nodiscard]] const TerrainSpec& spec() const { return spec_; }

private:
    TerrainSpec spec_;
    std::uint32_t samples_per_tile_;
};

} // namespace ps::terrain
