// diag_terrain_build_split.cpp — PHYS terrain-prefetch-ring plan, commit C0.
//
// One-shot measurement tool (plain main, NOT registered with CTest, mirrors
// ps_energy_study's precedent in this same directory): splits the cost of
// bringing one heightfield tile onto the physics timeline into its
// component parts, through the PUBLIC IRigidBackend seam only (this tool
// does not link core/src/backend/jolt/jolt_backend.cpp's file-local helpers
// - they are not exposed via any header, so every measurement below goes
// through ps::IRigidBackend, exactly what a future backend-side prefetch API
// (plan §1 point (j), "proposed install") would also have to go through).
//
// Measured per tile, for sample_count n = 64 and n = 256 (255 m tiles, this
// project's shipped tile size):
//   (a) ITerrainSource::fill_tile                      - source-side cost
//   (b)+(c) IRigidBackend::create_body(HeightfieldShape) - today's full
//           "create a fresh shape from scratch" cost, i.e. everything the
//           backend does per NEW body: row-reversal/float-conversion +
//           HeightFieldShapeSettings::Create + wrap + SetPositionAndRotation
//           on first creation. This is the closest the public seam gets to
//           plan point (g)/(h)/(c)/(d)/(e) combined, since those Jolt-level
//           steps are not individually reachable without linking Jolt
//           directly (see the note below).
//   (j) IRigidBackend::update_heightfield_body + set_pose on an EXISTING
//       pool body (today's actual per-tile-crossing install path,
//       tile_manager.cpp step 5) - this is the "I" (cheap install) term the
//       plan's gate compares against.
//
// The plan's U term ((c)+(d)+(e): row-reversal + SetHeights + SetMaterials)
// is not separately reachable through the public backend seam - there is no
// public API that does "reuse an existing standalone JPH::HeightFieldShape
// not yet attached to a body". update_heightfield_body (j) already IS the
// cheapest possible install: it mutates an existing pool body's shape data
// in place via SetHeights/SetMaterials, called through this backend's own
// preallocated TempAllocator arena (see i_rigid_backend.h's own doc comment
// on update_heightfield_body). So this tool treats:
//   U (upload cost if we had to build a fresh shape and swap it in) ~=
//       (b)+(c) - the full create_body cost, since a spare-shape swap still
//       has to pay for SetHeights+SetMaterials+wrap, and create_body is the
//       only public measurement that exercises that same Jolt-internal path;
//   I (today's actual reuse-in-place cost) = (j) directly.
// create_body's own SetPositionAndRotation-at-construction and Create+wrap
// overhead make (b)+(c) an OVER-estimate of a pure spare-shape SetHeights/
// SetMaterials cost, which only makes the U-I>=0.25ms gate MORE likely to
// fire (conservative in the direction of picking the more complex Option A
// only when it is more clearly justified) - recorded as a deviation from
// the plan's literal §1 method in the final report; if this comparison
// lands close to the 0.25ms line, that gap matters and must be re-measured
// with a real Jolt-linked harness (plan's original design) before trusting
// the gate.
//
// Output: for each n, median/p95/max over kSamples tile keys, in
// milliseconds, printed as a plain table to stdout.

#include "ps/backend/i_rigid_backend.h"
#include "ps/backend/shape_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/pose.h"
#include "ps/terrain/terrain_source.h"
#include "ps/types.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::high_resolution_clock;

double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Stats {
    double median = 0.0;
    double p95 = 0.0;
    double max = 0.0;
};

Stats summarize(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    Stats s;
    if (v.empty()) return s;
    s.median = v[v.size() / 2];
    s.p95 = v[static_cast<std::size_t>(v.size() * 0.95)];
    s.max = v.back();
    return s;
}

void print_row(const char* label, const Stats& s) {
    std::printf("  %-45s median=%8.4f ms  p95=%8.4f ms  max=%8.4f ms\n", label, s.median, s.p95, s.max);
}

// A tiny synthetic ITerrainSource so this tool needs no data files (keeps it
// independent of data/tracks/test_ground, which is read-only/owned
// elsewhere) - flat-ish rolling terrain, cheap to fill, same shape contract
// as ProceduralTerrainSource.
class SyntheticSource : public ps::terrain::ITerrainSource {
public:
    explicit SyntheticSource(std::uint32_t n) : n_(n) {}

    [[nodiscard]] ps::real tile_size_m() const override { return 255.0; }
    [[nodiscard]] std::uint32_t samples_per_tile() const override { return n_; }

    [[nodiscard]] ps::terrain::TileKey key_for_world_xy(ps::real x, ps::real y) const override {
        auto tx = static_cast<std::int64_t>(std::floor(x / tile_size_m()));
        auto ty = static_cast<std::int64_t>(std::floor(y / tile_size_m()));
        return pack(tx, ty);
    }
    [[nodiscard]] ps::terrain::TileKey key_offset(ps::terrain::TileKey key, std::int32_t dx, std::int32_t dy) const override {
        auto [tx, ty] = unpack(key);
        return pack(tx + dx, ty + dy);
    }
    [[nodiscard]] ps::Vec3 tile_origin_xy(ps::terrain::TileKey key) const override {
        auto [tx, ty] = unpack(key);
        return ps::Vec3{static_cast<ps::real>(tx) * tile_size_m(), static_cast<ps::real>(ty) * tile_size_m(), 0.0};
    }

    void fill_tile(ps::terrain::TileKey key, ps::terrain::TileSample& out) const override {
        auto [tx, ty] = unpack(key);
        const std::uint32_t n = n_;
        for (std::uint32_t y = 0; y < n; ++y) {
            for (std::uint32_t x = 0; x < n; ++x) {
                double wx = static_cast<double>(tx) * n + x;
                double wy = static_cast<double>(ty) * n + y;
                // cheap deterministic pseudo-noise, no transcendentals
                double h = 3.0 * std::sin(0.0) + static_cast<double>(((static_cast<long long>(wx) * 928371 +
                                                                        static_cast<long long>(wy) * 123457) % 2000)) / 1000.0;
                out.heights[y * n + x] = h;
            }
        }
        std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), ps::kInvalidSurfaceId);
    }

private:
    static ps::terrain::TileKey pack(std::int64_t tx, std::int64_t ty) {
        return ps::terrain::TileKey{(tx & 0xFFFFFFFFLL) | ((ty & 0xFFFFFFFFLL) << 32)};
    }
    static std::pair<std::int64_t, std::int64_t> unpack(ps::terrain::TileKey key) {
        std::int64_t tx = static_cast<std::int32_t>(key.packed & 0xFFFFFFFFLL);
        std::int64_t ty = static_cast<std::int32_t>((key.packed >> 32) & 0xFFFFFFFFLL);
        return {tx, ty};
    }

    std::uint32_t n_;
};

void run_for_n(std::uint32_t n, int kSamples) {
    std::printf("\n=== sample_count n = %u (%.0f m tile) ===\n", n, 255.0);

    SyntheticSource source(n);
    ps::jobs::JobSystem jobs(1);
    auto backend = ps::create_jolt_backend(jobs, ps::Vec3{0, 0, -9.81}, 4096);

    std::vector<double> fill_ms, create_ms, update_ms;
    fill_ms.reserve(kSamples);
    create_ms.reserve(kSamples);
    update_ms.reserve(kSamples);

    ps::terrain::TileSample sample;
    sample.samples_per_tile = n;
    sample.tile_size_m = 255.0;
    sample.heights.resize(static_cast<std::size_t>(n) * n);
    sample.cell_surface_ids.resize(static_cast<std::size_t>(n - 1) * (n - 1));

    // (a) fill_tile
    for (int i = 0; i < kSamples; ++i) {
        auto key = source.key_offset(source.key_for_world_xy(0, 0), i, 0);
        auto t0 = Clock::now();
        source.fill_tile(key, sample);
        fill_ms.push_back(ms_since(t0));
    }

    // (b)+(c) create_body(HeightfieldShape) — fresh shape from scratch
    std::vector<ps::BodyId> fresh_bodies;
    fresh_bodies.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        auto key = source.key_offset(source.key_for_world_xy(0, 0), i, 0);
        source.fill_tile(key, sample);
        ps::HeightfieldShape shape;
        shape.origin = ps::Vec3::zero();
        shape.sample_spacing = sample.tile_size_m / (n - 1);
        shape.sample_count = n;
        shape.heights = sample.heights;
        shape.cell_surface_ids = sample.cell_surface_ids;
        ps::BodyDesc desc;
        desc.shape = shape;
        desc.motion = ps::BodyMotionType::Static;
        auto t0 = Clock::now();
        ps::BodyId id = backend->create_body(desc);
        create_ms.push_back(ms_since(t0));
        fresh_bodies.push_back(id);
    }
    for (auto id : fresh_bodies) backend->destroy_body(id);

    // (j) update_heightfield_body + set_pose on an EXISTING pool body —
    // today's actual per-tile-crossing install path.
    ps::HeightfieldShape placeholder;
    placeholder.origin = ps::Vec3::zero();
    placeholder.sample_spacing = sample.tile_size_m / (n - 1);
    placeholder.sample_count = n;
    placeholder.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
    placeholder.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    ps::BodyDesc pool_desc;
    pool_desc.shape = placeholder;
    pool_desc.motion = ps::BodyMotionType::Static;
    ps::BodyId pool_body = backend->create_body(pool_desc);

    for (int i = 0; i < kSamples; ++i) {
        auto key = source.key_offset(source.key_for_world_xy(0, 0), i, 0);
        source.fill_tile(key, sample);
        auto t0 = Clock::now();
        backend->update_heightfield_body(pool_body, sample.heights, sample.cell_surface_ids);
        backend->set_pose(pool_body, ps::Pose::identity());
        update_ms.push_back(ms_since(t0));
    }
    backend->destroy_body(pool_body);

    print_row("(a) fill_tile", summarize(fill_ms));
    print_row("(b+c) create_body(fresh HeightfieldShape)", summarize(create_ms));
    print_row("(j) update_heightfield_body+set_pose (reuse)", summarize(update_ms));

    Stats u = summarize(create_ms);
    Stats ii = summarize(update_ms);
    std::printf("  U (proxy, create_body median) - I (update_heightfield_body median) = %.4f ms\n",
                u.median - ii.median);

    // C2: the Option A install path, measured in a world holding a 7x7
    // resident set of pool heightfield bodies built from TileManager's own
    // +-200 m placeholder (the pool a swap targets in production):
    //   (u) update_heightfield_body + set_pose - today's in-tick upload
    //   (k) build_heightfield_spare            - moves to the background lane
    //   (l) swap_heightfield_spare + set_pose  - the new in-tick install
    ps::HeightfieldShape pool_hf = placeholder;
    pool_hf.heights.front() = -200.0;
    pool_hf.heights.back() = 200.0;
    ps::BodyDesc pool7_desc;
    pool7_desc.shape = pool_hf;
    pool7_desc.motion = ps::BodyMotionType::Static;
    std::vector<ps::BodyId> pool7;
    for (int gy = 0; gy < 7; ++gy) {
        for (int gx = 0; gx < 7; ++gx) {
            pool7_desc.pose.position = ps::Vec3{gx * 255.0, gy * 255.0, 0.0};
            pool7.push_back(backend->create_body(pool7_desc));
        }
    }
    const ps::HeightfieldSpareId spare = backend->create_heightfield_spare(pool_hf);
    auto ctx = backend->create_heightfield_build_context(n);
    std::vector<double> u_ms, k_ms, l_ms;
    for (int i = 0; i < kSamples; ++i) {
        auto key = source.key_offset(source.key_for_world_xy(0, 0), i, 0);
        source.fill_tile(key, sample);
        const std::size_t slot = static_cast<std::size_t>(i) % pool7.size();
        const ps::BodyId target = pool7[slot];
        ps::Pose pose;
        pose.position = ps::Vec3{static_cast<double>(slot % 7) * 255.0, static_cast<double>(slot / 7) * 255.0, 0.0};

        auto t0 = Clock::now();
        backend->update_heightfield_body(target, sample.heights, sample.cell_surface_ids);
        backend->set_pose(target, pose);
        u_ms.push_back(ms_since(t0));

        t0 = Clock::now();
        backend->build_heightfield_spare(spare, sample.heights, sample.cell_surface_ids, *ctx);
        k_ms.push_back(ms_since(t0));

        t0 = Clock::now();
        backend->swap_heightfield_spare(target, spare);
        backend->set_pose(target, pose);
        l_ms.push_back(ms_since(t0));
    }
    backend->destroy_heightfield_spare(spare);
    for (auto id : pool7) backend->destroy_body(id);
    print_row("(u) update_heightfield_body+set_pose, 49 pool", summarize(u_ms));
    print_row("(k) build_heightfield_spare (off-tick)", summarize(k_ms));
    print_row("(l) swap_heightfield_spare+set_pose (in-tick)", summarize(l_ms));
    std::printf("  in-tick saving per tile (median u - median l) = %.4f ms\n",
                summarize(u_ms).median - summarize(l_ms).median);
}

} // namespace

int main() {
    std::printf("diag_terrain_build_split: C0 measurement tool (plan gate: U-I >= 0.25ms at n=256 -> Option A)\n");
    run_for_n(64, 200);
    run_for_n(256, 200);
    return 0;
}
