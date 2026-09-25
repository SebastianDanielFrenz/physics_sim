// ps/terrain/terrain_source.h — ITerrainSource seam (PLAN.md section 6:
// "Seams for later terrain sources" / M1's procedural terrain, pulled
// forward from P8).
//
// "ITerrainSource: fills a tile (heights, per-cell surface ids) for a tile
// key. The procedural terrain.json spec is one implementation; geodata will
// be another (P11). Streaming, caching, collision and the adapter's render
// meshes sit above this interface and do not know the source." Tile keys
// are opaque (not raw planar x/y indices) so a geodetic/cube-sphere tiling
// (P11) can replace the planar grid without touching any caller of this
// interface - a caller navigates tiles only through key_for_world_xy/
// key_offset/tile_origin_xy below, never by decoding TileKey::packed itself.
//
// No Jolt or Godot type appears here (PLAN.md D1/D2).

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps::terrain {

// Opaque tile identifier. The planar procedural source below packs
// (tile_x, tile_y) into the 64 bits; callers must never unpack this
// themselves - use ITerrainSource::key_offset/tile_origin_xy instead, which
// stay meaningful under a future non-planar tiling too.
struct TileKey {
    std::int64_t packed = 0;
    constexpr bool operator==(const TileKey&) const = default;
};

// One filled tile: a square grid of samples_per_tile x samples_per_tile
// height samples (row-major, y-major - heights[y*n+x], same convention as
// ps::HeightfieldShape) plus one surface id per (samples_per_tile-1)^2 cell,
// same row-major convention. tile_origin_xy is the world-space (x, y) of
// sample (0, 0) (z is not meaningful for the origin - height comes from the
// samples themselves).
//
// heights are always ABSOLUTE world-space elevation (e.g. real-world
// terrain fed through this interface can sit anywhere from roughly
// 90-880 m ASL) - this contract does not change. ps::terrain::TileManager
// (core/src/terrain/tile_manager.h/.cpp, internal) is the ONLY thing that
// ever shifts a tile vertically: it subtracts a per-tile datum (each tile's
// own absolute height range snapped to a multiple of 128 m) before handing
// the sample to the collision backend, and adds the datum back via the
// tile's own body pose - because JoltPhysics' HeightFieldShape fixes its
// 16-bit quantisation range once, at first construction, to a generous but
// FIXED +-200 m window, which real-world absolute elevation does not fit on
// its own. A tile whose own local relief alone exceeds that +-200 m window
// is still uploaded (clamped, not dropped) and counted - see TileManager::
// relief_overflow_count() - far more relief than any real terrain needs at
// this project's tile sizes and sample spacing. Nothing that reads
// ITerrainSource directly (e.g. a caller's own fill_tile probe) ever sees
// anything but the real, absolute height.
struct TileSample {
    std::vector<real> heights;
    std::vector<SurfaceId> cell_surface_ids;
    std::uint32_t samples_per_tile = 0;
    real tile_size_m = 0.0;
    Vec3 tile_origin_xy = Vec3::zero();
};

class ITerrainSource {
public:
    virtual ~ITerrainSource() = default;

    // Constant across every tile this source ever produces (PLAN.md M1:
    // single fixed resolution, no LOD - see the tile manager's own header
    // comment for why).
    [[nodiscard]] virtual real tile_size_m() const = 0;
    [[nodiscard]] virtual std::uint32_t samples_per_tile() const = 0;

    // The tile containing world point (x, y).
    [[nodiscard]] virtual TileKey key_for_world_xy(real x, real y) const = 0;
    // The tile dx/dy steps away from `key` in this source's own tiling
    // (planar: (tile_x+dx, tile_y+dy); a future non-planar source defines
    // its own notion of "step" here, e.g. along a geodetic grid axis).
    [[nodiscard]] virtual TileKey key_offset(TileKey key, std::int32_t dx, std::int32_t dy) const = 0;
    // World-space (x, y) of sample (0, 0) of the given tile.
    [[nodiscard]] virtual Vec3 tile_origin_xy(TileKey key) const = 0;

    // Fills `out` for `key`, deterministically (PLAN.md D7: same key ->
    // bit-identical output) and with no shared mutable state, so this may
    // be called concurrently for different keys from different jobs
    // (PLAN.md D8). `out`'s heights/cell_surface_ids vectors are already
    // sized by the caller (samples_per_tile()^2 / (samples_per_tile()-1)^2)
    // and must not be resized - implementations only overwrite content.
    virtual void fill_tile(TileKey key, TileSample& out) const = 0;
};

} // namespace ps::terrain
