// Terrain prefetch ring, commit C2: IRigidBackend's detached heightfield
// "spare" (create/build/swap_heightfield_spare) must be indistinguishable
// from today's in-tick update_heightfield_body upload.
//
// Two independent backends hold the same static pool body, created from
// TileManager's own +-200 m placeholder desc (tile_manager.cpp's pool
// creation). Backend A installs the target tile the old way
// (update_heightfield_body + set_pose). Backend B installs it through a
// spare that carries DIFFERENT prior content (the spare first held another
// tile, was swapped into the body, and so now holds the body's own previous
// shape) and is rebuilt with the target tile on a separate std::thread,
// concurrently with backend B stepping dynamic bodies - the production
// pattern (background lane builds while the tick runs). Then swap + set_pose.
// Checked bitwise: Jolt-level digest (quantised samples, range blocks,
// active edges, material indices), a grid of ray casts, down-casts of a
// small sphere (surface ids + normals) and overlap queries, and the backend
// state_hash after dropping balls on both.

#include "ps/backend/i_rigid_backend.h"
#include "ps/backend/shape_desc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

using namespace ps;

namespace {

constexpr std::uint32_t kN = 64;
constexpr real kTileSize = 255.0;

HeightfieldShape placeholder_desc() {
    // Mirrors tile_manager.cpp's pool placeholder exactly.
    HeightfieldShape hf;
    hf.origin = Vec3::zero();
    hf.sample_count = kN;
    hf.sample_spacing = kTileSize / static_cast<real>(kN - 1);
    hf.heights.assign(static_cast<std::size_t>(kN) * kN, 0.0);
    hf.heights.front() = -200.0;
    hf.heights.back() = 200.0;
    hf.cell_surface_ids.assign(static_cast<std::size_t>(kN - 1) * (kN - 1), kInvalidSurfaceId);
    return hf;
}

// Deterministic tile content: rolling relief, several surface ids, a few
// no-collision holes (the suppression sentinel) so every Jolt code path the
// real tiles use is exercised.
void make_tile(std::uint32_t seed, std::vector<real>& heights, std::vector<SurfaceId>& ids) {
    heights.assign(static_cast<std::size_t>(kN) * kN, 0.0);
    ids.assign(static_cast<std::size_t>(kN - 1) * (kN - 1), kInvalidSurfaceId);
    std::uint64_t s = 0x9E3779B97F4A7C15ULL * (seed + 1);
    auto next = [&s]() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    };
    for (std::uint32_t y = 0; y < kN; ++y) {
        for (std::uint32_t x = 0; x < kN; ++x) {
            const real ramp = 0.05 * static_cast<real>(x) - 0.03 * static_cast<real>(y);
            const real noise = static_cast<real>(next() % 1000) / 500.0;
            heights[y * kN + x] = static_cast<real>(seed) * 3.0 + ramp + noise;
        }
    }
    for (std::size_t i = 0; i < ids.size(); ++i) ids[i] = static_cast<SurfaceId>((i / 97 + seed) % 6);
    // A small hole block.
    for (std::uint32_t y = 20; y < 24; ++y)
        for (std::uint32_t x = 30; x < 33; ++x) heights[y * kN + x] = kHeightfieldNoCollision;
}

std::uint64_t bits(real v) {
    std::uint64_t u = 0;
    std::memcpy(&u, &v, sizeof(u));
    return u;
}

void fold(std::uint64_t& h, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        h ^= (v >> (8 * i)) & 0xFFu;
        h *= 1099511628211ULL;
    }
}
void fold(std::uint64_t& h, const Vec3& v) {
    fold(h, bits(v.x));
    fold(h, bits(v.y));
    fold(h, bits(v.z));
}

// Bitwise fingerprint of every query result over a grid covering the tile.
std::uint64_t query_fingerprint(const IRigidBackend& b, const Vec3& tile_pos, int* hits_out) {
    std::uint64_t h = 1469598103934665603ULL;
    int hits = 0;
    std::vector<ShapeContactHit> contacts;
    for (int gy = 0; gy < 23; ++gy) {
        for (int gx = 0; gx < 23; ++gx) {
            const real x = tile_pos.x + 1.3 + gx * 11.07;
            const real y = tile_pos.y + 0.7 + gy * 11.13;
            const RayCastHit r = b.ray_cast(Vec3{x, y, 500.0}, Vec3{0, 0, -1}, 1000.0);
            fold(h, r.hit ? 1u : 0u);
            if (r.hit) {
                ++hits;
                fold(h, bits(r.fraction));
                fold(h, r.point);
                fold(h, r.normal);
            }
            Pose p;
            p.position = Vec3{x, y, 500.0};
            const ShapeCastHit c = b.shape_cast(SphereShape{0.4}, p, Vec3{0, 0, -1000.0}, 1.0);
            fold(h, c.hit ? 1u : 0u);
            if (c.hit) {
                fold(h, bits(c.fraction));
                fold(h, c.normal);
                fold(h, c.contact_point);
                fold(h, static_cast<std::uint64_t>(c.surface));
            }
            contacts.clear();
            Pose q;
            q.position = Vec3{x, y, 0.0};
            b.collide_shape_into(BoxShape{Vec3{1.0, 1.0, 60.0}}, q, contacts);
            fold(h, contacts.size());
            for (const auto& ct : contacts) {
                fold(h, ct.point);
                fold(h, ct.normal);
                fold(h, bits(ct.penetration));
                fold(h, static_cast<std::uint64_t>(ct.surface));
            }
        }
    }
    if (hits_out) *hits_out = hits;
    return h;
}

struct Rig {
    jobs::JobSystem jobs{2};
    std::unique_ptr<IRigidBackend> backend = create_jolt_backend(jobs, Vec3{0, 0, -9.81}, 256);
    BodyId tile;
    std::vector<BodyId> balls;
    Pose tile_pose;

    Rig() {
        BodyDesc d;
        d.shape = placeholder_desc();
        d.motion = BodyMotionType::Static;
        d.friction = 0.8;
        d.pose.position = Vec3{0, 0, -1.0e6}; // parked, like an unassigned pool slot
        tile = backend->create_body(d);
        tile_pose.position = Vec3{-100.0, -80.0, 0.0};
    }

    void add_balls() {
        for (int i = 0; i < 12; ++i) {
            BodyDesc bd;
            bd.shape = SphereShape{0.5 + 0.05 * i};
            bd.motion = BodyMotionType::Dynamic;
            bd.mass = 1.0 + i;
            bd.friction = 0.6;
            bd.allow_sleep = false;
            bd.pose.position = Vec3{tile_pose.position.x + 20.0 + 17.0 * i, tile_pose.position.y + 30.0 + 13.0 * (i % 5), 60.0};
            balls.push_back(backend->create_body(bd));
        }
    }
};

} // namespace

TEST_CASE("Heightfield spare built off-thread installs bit-identically to update_heightfield_body", "[backend][terrain]") {
    std::vector<real> target_h, other_h, first_h;
    std::vector<SurfaceId> target_ids, other_ids, first_ids;
    make_tile(7, target_h, target_ids);
    make_tile(3, other_h, other_ids);
    make_tile(11, first_h, first_ids);

    // A: today's path.
    Rig a;
    a.backend->update_heightfield_body(a.tile, target_h, target_ids);
    a.backend->set_pose(a.tile, a.tile_pose);

    // B: spare path with a non-trivial history.
    Rig b;
    const HeightfieldSpareId spare = b.backend->create_heightfield_spare(placeholder_desc());
    auto ctx = b.backend->create_heightfield_build_context(kN);
    b.backend->update_heightfield_body(b.tile, first_h, first_ids); // body shape content "first"
    b.backend->build_heightfield_spare(spare, other_h, other_ids, *ctx);
    b.backend->swap_heightfield_spare(b.tile, spare); // body = "other", spare = old "first" shape
    b.backend->set_pose(b.tile, b.tile_pose);
    REQUIRE(b.backend->debug_heightfield_spare_digest(spare) != a.backend->debug_heightfield_digest(a.tile));

    // Build the target into the spare on another thread while backend B
    // keeps stepping dynamic bodies over the body's current ("other") shape.
    b.add_balls();
    std::thread builder([&] { b.backend->build_heightfield_spare(spare, target_h, target_ids, *ctx); });
    for (int i = 0; i < 20; ++i) b.backend->step(1.0 / 240.0);
    builder.join();
    for (BodyId id : b.balls) b.backend->destroy_body(id);
    b.balls.clear();

    const std::uint64_t a_digest = a.backend->debug_heightfield_digest(a.tile);
    CHECK(b.backend->debug_heightfield_spare_digest(spare) == a_digest);

    b.backend->swap_heightfield_spare(b.tile, spare);
    b.backend->set_pose(b.tile, b.tile_pose);
    CHECK(b.backend->debug_heightfield_digest(b.tile) == a_digest);

    // Negative control: the digest does see content.
    {
        Rig c;
        c.backend->update_heightfield_body(c.tile, other_h, other_ids);
        CHECK(c.backend->debug_heightfield_digest(c.tile) != a_digest);
        std::vector<SurfaceId> one_id_changed = target_ids;
        one_id_changed[1234] = static_cast<SurfaceId>((one_id_changed[1234] + 1) % 6);
        c.backend->update_heightfield_body(c.tile, target_h, one_id_changed);
        CHECK(c.backend->debug_heightfield_digest(c.tile) != a_digest);
        c.backend->update_heightfield_body(c.tile, target_h, target_ids);
        CHECK(c.backend->debug_heightfield_digest(c.tile) == a_digest);
    }

    // Every query result bitwise equal.
    int hits_a = 0, hits_b = 0;
    const std::uint64_t qa = query_fingerprint(*a.backend, a.tile_pose.position, &hits_a);
    const std::uint64_t qb = query_fingerprint(*b.backend, b.tile_pose.position, &hits_b);
    CHECK(hits_a > 400); // the grid really hits the tile (a few rays fall into the hole)
    CHECK(hits_a == hits_b);
    CHECK(qa == qb);

    // Dropped balls: identical trajectories. B has one body index more in its
    // history (the balls created and destroyed above), so compare the balls'
    // own poses/velocities rather than the whole-backend hash of mismatched
    // id sets; the backend hash is compared in the history-free case below.
    a.add_balls();
    b.add_balls();
    REQUIRE(a.balls.size() == b.balls.size());
    for (int i = 0; i < 480; ++i) {
        a.backend->step(1.0 / 240.0);
        b.backend->step(1.0 / 240.0);
    }
    std::uint64_t ha = 1469598103934665603ULL, hb = ha;
    for (std::size_t i = 0; i < a.balls.size(); ++i) {
        const Pose pa = a.backend->get_pose(a.balls[i]);
        const Pose pb = b.backend->get_pose(b.balls[i]);
        fold(ha, pa.position);
        fold(hb, pb.position);
        fold(ha, a.backend->get_motion(a.balls[i]).linear);
        fold(hb, b.backend->get_motion(b.balls[i]).linear);
        CHECK(pa.position.z < 60.0 - 5.0); // actually fell onto the tile
    }
    CHECK(ha == hb);
    CHECK(a.backend->debug_heightfield_digest(a.tile) == b.backend->debug_heightfield_digest(b.tile));

    b.backend->destroy_heightfield_spare(spare);
}

TEST_CASE("Heightfield spare swap gives the same backend state_hash as update_heightfield_body", "[backend][terrain]") {
    std::vector<real> target_h, prior_h;
    std::vector<SurfaceId> target_ids, prior_ids;
    make_tile(5, target_h, target_ids);
    make_tile(9, prior_h, prior_ids);

    Rig a;
    a.backend->update_heightfield_body(a.tile, target_h, target_ids);
    a.backend->set_pose(a.tile, a.tile_pose);

    Rig b;
    const HeightfieldSpareId spare = b.backend->create_heightfield_spare(placeholder_desc());
    auto ctx = b.backend->create_heightfield_build_context(kN);
    b.backend->build_heightfield_spare(spare, prior_h, prior_ids, *ctx); // different prior content
    std::thread builder([&] { b.backend->build_heightfield_spare(spare, target_h, target_ids, *ctx); });
    builder.join();
    b.backend->swap_heightfield_spare(b.tile, spare);
    b.backend->set_pose(b.tile, b.tile_pose);

    a.add_balls();
    b.add_balls();
    for (int i = 0; i < 480; ++i) {
        a.backend->step(1.0 / 240.0);
        b.backend->step(1.0 / 240.0);
        if (i % 60 == 59) REQUIRE(a.backend->compute_state_hash(0) == b.backend->compute_state_hash(0));
    }
    CHECK(a.backend->compute_state_hash(0) == b.backend->compute_state_hash(0));

    // Swap back: the body again carries the placeholder-created shape whose
    // content is the ORIGINAL placeholder (never written) - the old shape was
    // kept alive intact in the spare.
    Rig fresh;
    b.backend->swap_heightfield_spare(b.tile, spare);
    b.backend->set_pose(b.tile, b.tile_pose);
    CHECK(b.backend->debug_heightfield_digest(b.tile) == fresh.backend->debug_heightfield_digest(fresh.tile));
    CHECK(b.backend->debug_heightfield_spare_digest(spare) == a.backend->debug_heightfield_digest(a.tile));
    b.backend->destroy_heightfield_spare(spare);
}
