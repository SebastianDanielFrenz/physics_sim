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

// See ITerrainSource::prefetch_support()'s doc comment below for what each
// value promises about background-lane concurrency safety.
enum class PrefetchSupportKind { TickOnly, Immutable, Snapshot };

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

    // Terrain prefetch ring (owner-approved 2026-09-27, TerrainConfig::
    // prefetch_margin_tiles/prefetch_max_tiles): what this source can
    // safely promise about being called from TileManager's BACKGROUND
    // build lane, concurrently with the sim thread's own tick (which may
    // be calling fill_tile for OTHER keys, or nothing terrain-related at
    // all, at the same time - never the SAME key twice concurrently).
    //
    //   TickOnly  (default): fill_tile is only ever safe to call from the
    //             World tick that needs the result (today's only caller,
    //             and every source written before this feature existed -
    //             this default reproduces today's behaviour exactly, so a
    //             source that does not override this cannot be prefetched
    //             at all no matter what TerrainConfig says: TileManager
    //             falls back to the synchronous per-tick path for it).
    //   Immutable: fill_tile has no shared mutable state and is safe to
    //             call concurrently for any key at any time (already this
    //             interface's EXISTING contract, see fill_tile's own doc
    //             comment above) - e.g. ProceduralTerrainSource, a pure
    //             function of its own fixed spec. acquire_prefetch_inputs
    //             below need not be overridden.
    //   Snapshot: fill_tile reads live state and is safe only during the
    //             tick, while owner mutation is excluded (e.g. a mutable
    //             resident-height set changed between World::step calls -
    //             see the racing_game/geo2map_engine bridge). Such a source
    //             instead overrides acquire_prefetch_inputs to hand back an
    //             immutable, independently-owned snapshot of whatever
    //             fill_tile would have read for that key (captured
    //             synchronously, on the tick side), and fill_tile_from
    //             to fill a tile from that snapshot alone, with no access
    //             to the source's own live state - safe to run on the
    //             background lane. prefetch_inputs_current lets the
    //             pinned-install tick reject a snapshot that has gone
    //             stale (the source's live data changed since the
    //             snapshot was captured) rather than install stale
    //             content; TileManager falls back to a synchronous
    //             fill_tile on that key when it does.
    [[nodiscard]] virtual PrefetchSupportKind prefetch_support() const { return PrefetchSupportKind::TickOnly; }

    // Captures whatever fill_tile_from(key, ...) below will need for
    // `key`, called inside World::step (never the background lane) at
    // the moment TileManager decides to enqueue a background build for
    // this key. Returns false (out left untouched) to decline prefetching
    // this particular key right now (e.g. its inputs are not ready yet) -
    // TileManager then falls back to the synchronous per-tick path for it,
    // exactly as if this source were TickOnly. Default: declines (correct
    // for TickOnly and unnecessary for Immutable, which fill_tile alone
    // already satisfies). Tick-side callbacks may run on a JobSystem
    // worker, not necessarily the OS thread that called World::step. Live
    // source mutation must be excluded until World::step returns.
    // The ticket releases this handle inside the tick;
    // releasing it must not deallocate snapshot storage. Snapshot sources
    // must retain ownership and reclaim retired snapshots on their owner
    // thread outside World::step (for example, an owner-thread purge).
    virtual bool acquire_prefetch_inputs(TileKey /*key*/, std::shared_ptr<const void>& /*out*/) const { return false; }

    // Fills `out` for `key` using ONLY `inputs` (the pointer previously
    // returned via acquire_prefetch_inputs, still owned by the caller) -
    // never this source's own live/mutable state. Safe to call from the
    // background lane, concurrently with anything else. Default forwards
    // to fill_tile (correct for Immutable, whose fill_tile is already
    // concurrency-safe on its own and never calls this override at all
    // since acquire_prefetch_inputs's default declines).
    virtual void fill_tile_from(TileKey key, const void* /*inputs*/, TileSample& out) const { fill_tile(key, out); }

    // Called on the tick side at the pinned install tick, right before
    // installing a background-built tile: true if `inputs` (still exactly
    // what acquire_prefetch_inputs returned for `key`) is still an
    // accurate snapshot of what fill_tile would produce for `key` right
    // now. False makes TileManager discard the background result and fall
    // back to a synchronous fill_tile for this tick instead of installing
    // stale content. Default: always current (correct for TickOnly, which
    // never reaches here, and Immutable, which is current by construction).
    [[nodiscard]] virtual bool prefetch_inputs_current(TileKey /*key*/, const void* /*inputs*/) const { return true; }
};

} // namespace ps::terrain
