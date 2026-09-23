// adapters/godot/src/fall_detector.h - diagnostic only (PHYS-021 follow-up):
// notices when a vehicle's chassis has fallen below where solid ground
// should be and formats ONE report block per fall event, so an owner-
// reported "fell through the floor" can be debugged if it recurs. Godot-
// free (plain ps:: types); PsSimulation calls it from its sim-thread
// post_step and prints the block. The simulation keeps running either way.
//
// Where ground "should be": the terrain source's own height at the chassis
// XY. It comes from ITerrainSource::fill_tile, i.e. the raw procedural
// height, independent of what is currently resident or holed in the
// collision world. Static features never sit more than 0.5 m below it
// (test_ground_collision.json's lowest surface is z=-0.5 on terrain.json's
// height-0 flat zones). A resting chassis centre sits ~0.6 m above the
// surface (~0.15 m even upside-down). So a chassis centre more than
// kFallBelowTerrainM = 2.0 m below the terrain height is at least 1.5 m
// below any legitimate resting position: it has fallen through.
// The detector re-arms once the chassis is back above
// (terrain - kFallBelowTerrainM + kRearmMarginM), e.g. after a reset.
//
// Cost: one fill_tile (64x64 samples) only when the chassis crosses into a
// different terrain tile, into a buffer allocated once; otherwise an array
// lookup plus a bilinear interpolation per vehicle per tick.

#pragma once

#include "ps/io/surface_table.h"
#include "ps/terrain/terrain_source.h"
#include "ps/world/world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace ps_godot {

class FallDetector {
public:
    static constexpr ps::real kFallBelowTerrainM = 2.0;
    static constexpr ps::real kRearmMarginM = 1.0;

    // True exactly once per fall event (the tick the chassis first drops
    // below the threshold); false otherwise, including while it keeps
    // falling. `terrain` may be null (no terrain source installed): then
    // the detector never fires.
    bool update(const ps::terrain::ITerrainSource* terrain, const ps::Vec3& chassis_pos) {
        if (terrain == nullptr) return false;
        last_terrain_height_ = terrain_height_at(*terrain, chassis_pos.x, chassis_pos.y);
        last_threshold_z_ = last_terrain_height_ - kFallBelowTerrainM;
        const bool below = chassis_pos.z < last_threshold_z_;
        if (below && armed_) {
            armed_ = false;
            return true;
        }
        if (!armed_ && chassis_pos.z > last_threshold_z_ + kRearmMarginM) armed_ = true;
        return false;
    }

    [[nodiscard]] ps::real last_terrain_height() const { return last_terrain_height_; }
    [[nodiscard]] ps::real last_threshold_z() const { return last_threshold_z_; }
    [[nodiscard]] ps::terrain::TileKey last_tile_key() const { return tile_key_; }

    // One multi-line block, no trailing newline. Sim thread only (reads
    // live World/backend state right after World::step()).
    std::string format_report(const std::string& vehicle_name, ps::World& world, ps::VehicleId vid, ps::BodyId chassis,
                              const ps::terrain::ITerrainSource* terrain, const ps::io::SurfaceTable* surfaces) const {
        const ps::Pose pose = world.get_pose(chassis);
        const ps::Motion motion = world.get_motion(chassis);
        const ps::Vec3& p = pose.position;
        std::string out;
        char line[512];
        auto add = [&](const char* s) {
            out += s;
            out += '\n';
        };
        std::snprintf(line, sizeof line, "ps_godot FALL DETECTED vehicle=%s tick=%llu sim_time=%.4f s", vehicle_name.c_str(),
                      static_cast<unsigned long long>(world.tick()), world.sim_time());
        add(line);
        std::snprintf(line, sizeof line, "  chassis pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) |v|=%.3f m/s", p.x, p.y, p.z, motion.linear.x,
                      motion.linear.y, motion.linear.z, motion.linear.length());
        add(line);
        std::snprintf(line, sizeof line, "  terrain height here=%.3f m, fall threshold z=%.3f m (terrain - %.1f m)", last_terrain_height_,
                      last_threshold_z_, kFallBelowTerrainM);
        add(line);
        if (terrain != nullptr) {
            const ps::Vec3 o = terrain->tile_origin_xy(tile_key_);
            std::snprintf(line, sizeof line,
                          "  tile under car: key=%lld origin=(%.1f, %.1f) size=%.1f m | resident=%zu starved=%zu precomputed=%zu",
                          static_cast<long long>(tile_key_.packed), o.x, o.y, terrain->tile_size_m(), world.terrain_resident_tile_count(),
                          world.terrain_starved_tile_count(), world.terrain_precomputed_tile_count());
            add(line);
        }
        // What collision exists under the chassis XY right now. Box centred
        // ABOVE every surface (Jolt's collide ignores back faces beyond the
        // query centre - vault PHYS-025).
        {
            std::vector<ps::ShapeContactHit> hits;
            ps::Pose probe;
            probe.position = ps::Vec3{p.x, p.y, last_terrain_height_ + 100.0};
            world.backend().collide_shape_into(ps::BoxShape{ps::Vec3{0.05, 0.05, 150.0}}, probe, hits);
            std::string list;
            std::size_t static_hits = 0;
            for (const ps::ShapeContactHit& h : hits) {
                if (h.body == chassis || !world.backend().is_static(h.body)) continue;
                ++static_hits;
                char item[96];
                std::snprintf(item, sizeof item, " body%u@z=%.3f(%s)", static_cast<unsigned>(h.body.index), h.point.z, surface_name(surfaces, h.surface).c_str());
                list += item;
            }
            std::snprintf(line, sizeof line, "  static collision under chassis XY (collide probe): %zu hit(s)%s", static_hits,
                          static_hits ? list.c_str() : " - NONE (hole)");
            add(line);
        }
        const std::size_t wheels = world.vehicle_wheel_count(vid);
        for (std::size_t w = 0; w < wheels; ++w) {
            const ps::vehicle::WheelState ws = world.wheel_state(vid, w);
            std::snprintf(line, sizeof line, "  wheel %s: contact=%s load=%.1f N travel=%.3f m surface=%s(%u) slip_ratio=%.3f slip_angle=%.3f",
                          world.wheel_name(vid, w).c_str(), ws.load > 0.0 ? "yes" : "no", ws.load, ws.suspension_travel,
                          surface_name(surfaces, ws.surface).c_str(), static_cast<unsigned>(ws.surface), ws.slip_ratio, ws.slip_angle);
            add(line);
        }
        out += "ps_godot END FALL REPORT";
        return out;
    }

private:
    static std::string surface_name(const ps::io::SurfaceTable* surfaces, ps::SurfaceId id) {
        if (id == ps::kInvalidSurfaceId) return "none";
        return surfaces ? surfaces->name_for(id) : std::string("?");
    }

    // Bilinear interpolation of the tile's own samples (Jolt triangulates
    // each 4 m quad; the difference is negligible against a 2 m threshold).
    ps::real terrain_height_at(const ps::terrain::ITerrainSource& terrain, ps::real x, ps::real y) {
        const ps::terrain::TileKey key = terrain.key_for_world_xy(x, y);
        const std::uint32_t n = terrain.samples_per_tile();
        if (!have_tile_ || key.packed != tile_key_.packed) {
            if (sample_.heights.size() != static_cast<std::size_t>(n) * n) {
                sample_.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
                sample_.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
                sample_.samples_per_tile = n;
            }
            terrain.fill_tile(key, sample_);
            tile_key_ = key;
            have_tile_ = true;
        }
        const ps::real spacing = terrain.tile_size_m() / static_cast<ps::real>(n - 1);
        const ps::Vec3 o = terrain.tile_origin_xy(key);
        const ps::real fx = std::clamp((x - o.x) / spacing, ps::real(0), static_cast<ps::real>(n - 1));
        const ps::real fy = std::clamp((y - o.y) / spacing, ps::real(0), static_cast<ps::real>(n - 1));
        const auto ix = std::min(static_cast<std::uint32_t>(fx), n - 2);
        const auto iy = std::min(static_cast<std::uint32_t>(fy), n - 2);
        const ps::real tx = fx - ix, ty = fy - iy;
        auto h = [&](std::uint32_t cx, std::uint32_t cy) { return sample_.heights[static_cast<std::size_t>(cy) * n + cx]; };
        const ps::real a = h(ix, iy) + (h(ix + 1, iy) - h(ix, iy)) * tx;
        const ps::real b = h(ix, iy + 1) + (h(ix + 1, iy + 1) - h(ix, iy + 1)) * tx;
        return a + (b - a) * ty;
    }

    bool armed_ = true;
    bool have_tile_ = false;
    ps::terrain::TileKey tile_key_{};
    ps::terrain::TileSample sample_;
    ps::real last_terrain_height_ = 0.0;
    ps::real last_threshold_z_ = 0.0;
};

} // namespace ps_godot
