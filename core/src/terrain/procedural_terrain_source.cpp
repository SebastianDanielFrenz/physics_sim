#include "procedural_terrain_source.h"

#include "ps/math/transcendental.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ps::terrain {
namespace {

using json = nlohmann::json;

// MurmurHash3-style fmix32 finalizer with a seeded lattice-coordinate mix
// (terrain.py's hash2d) - exact uint32_t wraparound arithmetic, bit-for-bit
// portable.
std::uint32_t hash2d(std::int64_t ix, std::int64_t iy, std::uint32_t seed) {
    std::uint32_t h = static_cast<std::uint32_t>(static_cast<std::int32_t>(ix)) * 0x27D4EB2Du;
    h = h + static_cast<std::uint32_t>(static_cast<std::int32_t>(iy)) * 0x165667B1u;
    h = h + seed;
    h ^= h >> 15;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
}

real lattice_value(std::int64_t ix, std::int64_t iy, std::uint32_t seed) {
    return (static_cast<real>(hash2d(ix, iy, seed)) / 4294967295.0) * 2.0 - 1.0;
}

real smoothstep01(real t) { return t * t * (3.0 - 2.0 * t); }

real value_noise(real x, real y, real wavelength, std::uint32_t seed) {
    const real gx = x / wavelength;
    const real gy = y / wavelength;
    const real fix0 = std::floor(gx);
    const real fiy0 = std::floor(gy);
    const real fx = gx - fix0;
    const real fy = gy - fiy0;
    const real sx = smoothstep01(fx);
    const real sy = smoothstep01(fy);
    const std::int64_t ix0 = static_cast<std::int64_t>(fix0);
    const std::int64_t iy0 = static_cast<std::int64_t>(fiy0);
    const real v00 = lattice_value(ix0, iy0, seed);
    const real v10 = lattice_value(ix0 + 1, iy0, seed);
    const real v01 = lattice_value(ix0, iy0 + 1, seed);
    const real v11 = lattice_value(ix0 + 1, iy0 + 1, seed);
    const real a = v00 + (v10 - v00) * sx;
    const real b = v01 + (v11 - v01) * sx;
    return a + (b - a) * sy;
}

real roughness(real x, real y, real near_r, real far_r) {
    const real d = std::sqrt(x * x + y * y);
    if (d <= near_r) return 0.0;
    if (d >= far_r) return 1.0;
    return smoothstep01((d - near_r) / (far_r - near_r));
}

real height_noise(const TerrainSpec& spec, real x, real y) {
    real h = 0.0;
    for (const Octave& oc : spec.rolling_octaves) {
        h += oc.amplitude * value_noise(x, y, oc.wavelength, spec.seed + oc.seed_offset);
    }
    const real r = roughness(x, y, spec.roughness_near_m, spec.roughness_far_m);
    if (r > 0.0) {
        h += r * spec.hill_octave.amplitude * value_noise(x, y, spec.hill_octave.wavelength, spec.seed + spec.hill_octave.seed_offset);
    }
    return h;
}

real disc_mask(const FlatZone& z, real x, real y) {
    const real d = std::sqrt((x - z.center.x) * (x - z.center.x) + (y - z.center.y) * (y - z.center.y)) - z.radius;
    if (d <= 0.0) return 1.0;
    if (d >= z.blend) return 0.0;
    return 1.0 - smoothstep01(d / z.blend);
}

real corridor_mask(const FlatZone& z, real x, real y) {
    const real dx = x - z.center.x;
    const real dy = y - z.center.y;
    const real along = dx * z.forward.x + dy * z.forward.y;
    const real across = dx * z.left.x + dy * z.left.y;
    const real d_along = std::max(real(0.0), std::abs(along) - z.half_length);
    const real d_across = std::max(real(0.0), std::abs(across) - z.half_width);
    const real d = std::sqrt(d_along * d_along + d_across * d_across);
    if (d <= 0.0) return 1.0;
    if (d >= z.blend) return 0.0;
    return 1.0 - smoothstep01(d / z.blend);
}

real flat_mask(const TerrainSpec& spec, real x, real y) {
    real m = 0.0;
    for (const FlatZone& z : spec.flat_zones) {
        const real zm = (z.kind == FlatZone::Kind::Disc) ? disc_mask(z, x, y) : corridor_mask(z, x, y);
        m = std::max(m, zm);
    }
    return m;
}

// ------------------------------------------------------------- JSON loading
Octave read_octave(const json& j) {
    Octave oc;
    oc.wavelength = j.at("wavelength").get<real>();
    oc.amplitude = j.at("amplitude").get<real>();
    oc.seed_offset = j.at("seed_offset").get<std::uint32_t>();
    return oc;
}

FlatZone read_zone(const json& j) {
    FlatZone z;
    const std::string kind = j.at("kind").get<std::string>();
    const auto center = j.at("center");
    z.center = Vec3{center.at(0).get<real>(), center.at(1).get<real>(), 0.0};
    z.blend = j.at("blend").get<real>();
    if (kind == "disc") {
        z.kind = FlatZone::Kind::Disc;
        z.radius = j.at("radius").get<real>();
    } else if (kind == "corridor") {
        z.kind = FlatZone::Kind::Corridor;
        z.half_length = j.at("half_length").get<real>();
        z.half_width = j.at("half_width").get<real>();
        const real heading_deg = j.at("heading_deg").get<real>();
        const real heading_rad = heading_deg * (3.14159265358979323846 / 180.0);
        real s = 0.0, c = 0.0;
        ps::math::sincos(heading_rad, s, c);
        z.forward = Vec3{c, s, 0.0};
        z.left = Vec3{-s, c, 0.0};
    } else {
        throw std::runtime_error("terrain.json: unknown flat_zones kind '" + kind + "'");
    }
    return z;
}

SurfaceRule read_rule(const json& j) {
    SurfaceRule r;
    r.surface = j.at("surface").get<std::string>();
    r.wavelength = j.at("wavelength").get<real>();
    r.seed_offset = j.at("seed_offset").get<std::uint32_t>();
    r.min_value01 = j.at("min").get<real>();
    return r;
}

} // namespace

TerrainSpec load_terrain_spec(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open terrain spec: " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    json j;
    try {
        j = json::parse(ss.str());
    } catch (const json::parse_error& e) {
        throw std::runtime_error("terrain.json parse error in " + path + ": " + e.what());
    }

    TerrainSpec spec;
    spec.seed = j.at("seed").get<std::uint32_t>();

    const auto& hn = j.at("height_noise");
    for (const auto& oc : hn.at("rolling_octaves")) spec.rolling_octaves.push_back(read_octave(oc));
    spec.hill_octave = read_octave(hn.at("hill_octave"));
    spec.roughness_near_m = hn.at("roughness").at("near_radius_m").get<real>();
    spec.roughness_far_m = hn.at("roughness").at("far_radius_m").get<real>();

    for (const auto& z : j.at("flat_zones").at("zones")) spec.flat_zones.push_back(read_zone(z));

    const auto& sr = j.at("surface_rules");
    spec.default_surface = sr.at("default").get<std::string>();
    for (const auto& r : sr.at("rules")) spec.surface_rules.push_back(read_rule(r));

    spec.tile_size_m = j.at("tiling").at("tile_size_m").get<real>();

    spec.surface_names.push_back(spec.default_surface);
    for (const SurfaceRule& r : spec.surface_rules) {
        if (spec.surface_id_for_name(r.surface) == kInvalidSurfaceId) spec.surface_names.push_back(r.surface);
    }
    return spec;
}

real terrain_height(const TerrainSpec& spec, real x, real y) {
    return height_noise(spec, x, y) * (1.0 - flat_mask(spec, x, y));
}

SurfaceId terrain_surface_cell(const TerrainSpec& spec, std::int64_t tile_x, std::int64_t tile_y, real tile_size_m) {
    const real cx = (static_cast<real>(tile_x) + 0.5) * tile_size_m;
    const real cy = (static_cast<real>(tile_y) + 0.5) * tile_size_m;
    for (const SurfaceRule& rule : spec.surface_rules) {
        const real v01 = value_noise(cx, cy, rule.wavelength, spec.seed + rule.seed_offset) * 0.5 + 0.5;
        if (v01 >= rule.min_value01) return spec.surface_id_for_name(rule.surface);
    }
    return spec.surface_id_for_name(spec.default_surface);
}

namespace {
// TileKey packs (tile_x, tile_y) as two signed 32-bit halves - plenty for
// any tile_size_m/world-extent combination this project defines (a +-50 km
// world at 256 m tiles needs tile indices in [-196, 195]); a caller must
// still only ever go through key_for_world_xy/key_offset/tile_origin_xy
// (PLAN.md section 6: opaque keys), never decode this directly.
TileKey pack_key(std::int64_t tile_x, std::int64_t tile_y) {
    const auto ux = static_cast<std::uint32_t>(static_cast<std::int32_t>(tile_x));
    const auto uy = static_cast<std::uint32_t>(static_cast<std::int32_t>(tile_y));
    return TileKey{static_cast<std::int64_t>((static_cast<std::uint64_t>(ux) << 32) | uy)};
}
void unpack_key(TileKey key, std::int64_t& tile_x, std::int64_t& tile_y) {
    const auto u = static_cast<std::uint64_t>(key.packed);
    tile_x = static_cast<std::int32_t>(static_cast<std::uint32_t>(u >> 32));
    tile_y = static_cast<std::int32_t>(static_cast<std::uint32_t>(u & 0xFFFFFFFFu));
}
} // namespace

ProceduralTerrainSource::ProceduralTerrainSource(TerrainSpec spec, std::uint32_t samples_per_tile)
    : spec_(std::move(spec)), samples_per_tile_(samples_per_tile) {}

TileKey ProceduralTerrainSource::key_for_world_xy(real x, real y) const {
    const std::int64_t tx = static_cast<std::int64_t>(std::floor(x / spec_.tile_size_m));
    const std::int64_t ty = static_cast<std::int64_t>(std::floor(y / spec_.tile_size_m));
    return pack_key(tx, ty);
}

TileKey ProceduralTerrainSource::key_offset(TileKey key, std::int32_t dx, std::int32_t dy) const {
    std::int64_t tx, ty;
    unpack_key(key, tx, ty);
    return pack_key(tx + dx, ty + dy);
}

Vec3 ProceduralTerrainSource::tile_origin_xy(TileKey key) const {
    std::int64_t tx, ty;
    unpack_key(key, tx, ty);
    return Vec3{static_cast<real>(tx) * spec_.tile_size_m, static_cast<real>(ty) * spec_.tile_size_m, 0.0};
}

void ProceduralTerrainSource::fill_tile(TileKey key, TileSample& out) const {
    std::int64_t tx, ty;
    unpack_key(key, tx, ty);
    const real spacing = spec_.tile_size_m / static_cast<real>(samples_per_tile_ - 1);
    const Vec3 origin = tile_origin_xy(key);

    out.samples_per_tile = samples_per_tile_;
    out.tile_size_m = spec_.tile_size_m;
    out.tile_origin_xy = origin;

    const std::uint32_t n = samples_per_tile_;
    for (std::uint32_t iy = 0; iy < n; ++iy) {
        const real wy = origin.y + static_cast<real>(iy) * spacing;
        for (std::uint32_t ix = 0; ix < n; ++ix) {
            const real wx = origin.x + static_cast<real>(ix) * spacing;
            out.heights[static_cast<std::size_t>(iy) * n + ix] = terrain_height(spec_, wx, wy);
        }
    }

    // Surface ids are per terrain-tile CELL (spec_.tile_size_m granularity,
    // one id for this WHOLE tile's open ground - terrain.py's
    // terrain_surface_cell is defined at that granularity, not at the
    // sample sub-grid level) - every sample cell within one tile shares the
    // same tile-level surface id, matching terrain.py's own model exactly
    // (surface is a per (tile_x,tile_y) property, independent of the
    // height mesh's own internal sampling resolution).
    const SurfaceId sid = terrain_surface_cell(spec_, tx, ty, spec_.tile_size_m);
    std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), sid);
}

} // namespace ps::terrain
