// Terrain tile-seam crossing (PLAN.md P8, vault PHYS-021 follow-up).
//
// Bodies and a car cross several TileManager heightfield tiles (synthetic
// flat / slope / wavy ITerrainSource, 16 m tiles x 16 samples) and are
// compared against a SEAMLESS single heightfield carrying the exact same
// samples (same spacing, same heights, one Jolt body), so any difference is
// the tile seam itself.
//
// The seam spike (pre-fix, release build): a sliding sphere/box on tiles
// took a one-tick vertical kick at every seam it crossed - peak |az|
// 115-662 m/s^2 for the sphere at 5-20 m/s, up to 1901 at 40 m/s; the box
// 360-3974 m/s^2 - against <= 36 m/s^2 on the seamless field (the original
// 256 m / 64-sample test_ground repro: 168 m/s^2 at 10 m/s, 616 at 20).
// Cause: every Jolt HeightFieldShape treats its own OUTER border edges as
// active (it cannot see the neighbour tile), and the speculative contact
// against the NEXT tile's border edge carries a backward-tilted edge normal.
// Fix: jolt_backend.cpp's SimCollideBodyVsBody hook collides a body against
// every overlapping heightfield tile through one internal-edge-removing
// collector (see TerrainSeamWeldingCollector there). Post-fix: tiles ==
// seamless within noise (every body case <= 77 m/s^2, the car unchanged).
//
// The vehicle path (a wheel shape_cast, not a contact) never showed the
// spike: peak chassis az 0.1-1.5 m/s^2 and peak wheel Fz within 2% of the
// seamless field, both before and after the fix.
//
// Bounds (chosen from measured values with margin, release build):
//  - body:  |az|_tiles <= 2*|az|_seamless + 30 m/s^2 AND <= 120 m/s^2
//           (post-fix worst on flat/wavy 50.4 tiles vs 31.1 seamless;
//           pre-fix the SMALLEST wavy spike was 177 m/s^2).
//  - car:   peak |az|_tiles <= 2*seamless + 0.5 m/s^2, peak pitch accel
//           <= 2*seamless + 0.5 rad/s^2, peak wheel Fz <= 1.05*seamless
//           (measured worst: az 0.42 vs 0.23 on the slope, Fz 1.020x).
//
// The [.][diag] cases print the full shape x body x speed table.

#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/terrain/terrain_source.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace ps;

namespace {

enum class TerrainShape { Flat, Slope, Wavy };
enum class Ground { Tiles, Seamless, Box, Mixed, MixedHole, MeshOnly };
enum class BodyKind { SlidingSphere, RollingSphere, SlidingBox, RollingCapsule };

const char* name_of(TerrainShape s) { return s == TerrainShape::Flat ? "flat" : s == TerrainShape::Slope ? "slope" : "wavy"; }
const char* name_of(Ground g) { return g == Ground::Tiles ? "tiles" : g == Ground::Seamless ? "seamless" : "box"; }
const char* name_of(BodyKind b) {
    switch (b) {
    case BodyKind::SlidingSphere: return "slide-sphere";
    case BodyKind::RollingSphere: return "roll-sphere";
    case BodyKind::SlidingBox: return "slide-box";
    case BodyKind::RollingCapsule: return "roll-capsule";
    }
    return "?";
}

constexpr double kTwoPi = 6.283185307179586;

bool env_flag(const char* name) {
#ifdef _WIN32
    char* v = nullptr;
    std::size_t n = 0;
    const bool set = _dupenv_s(&v, &n, name) == 0 && v != nullptr;
    std::free(v);
    return set;
#else
    return std::getenv(name) != nullptr;
#endif
}

real terrain_height(TerrainShape s, real x, real y) {
    switch (s) {
    case TerrainShape::Flat: return 0.0;
    case TerrainShape::Slope: return real(0.03) * x + real(0.02) * y;
    case TerrainShape::Wavy:
        return real(0.15) * static_cast<real>(std::sin(kTwoPi * x / 60.0)) * static_cast<real>(std::cos(kTwoPi * y / 47.0));
    }
    return 0.0;
}


class SyntheticTerrain final : public terrain::ITerrainSource {
public:
    SyntheticTerrain(TerrainShape shape, real tile_size, std::uint32_t samples) : shape_(shape), tile_(tile_size), n_(samples) {}
    real tile_size_m() const override { return tile_; }
    std::uint32_t samples_per_tile() const override { return n_; }
    terrain::TileKey key_for_world_xy(real x, real y) const override {
        return pack(static_cast<std::int64_t>(std::floor(x / tile_)), static_cast<std::int64_t>(std::floor(y / tile_)));
    }
    terrain::TileKey key_offset(terrain::TileKey key, std::int32_t dx, std::int32_t dy) const override {
        std::int64_t tx, ty;
        unpack(key, tx, ty);
        return pack(tx + dx, ty + dy);
    }
    Vec3 tile_origin_xy(terrain::TileKey key) const override {
        std::int64_t tx, ty;
        unpack(key, tx, ty);
        return Vec3{static_cast<real>(tx) * tile_, static_cast<real>(ty) * tile_, 0.0};
    }
    void fill_tile(terrain::TileKey key, terrain::TileSample& out) const override {
        const Vec3 o = tile_origin_xy(key);
        out.samples_per_tile = n_;
        out.tile_size_m = tile_;
        out.tile_origin_xy = o;
        const real sp = spacing();
        for (std::uint32_t iy = 0; iy < n_; ++iy)
            for (std::uint32_t ix = 0; ix < n_; ++ix)
                out.heights[static_cast<std::size_t>(iy) * n_ + ix] = terrain_height(shape_, o.x + ix * sp, o.y + iy * sp);
        std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), SurfaceId{0});
    }
    real spacing() const { return tile_ / static_cast<real>(n_ - 1); }

private:
    static terrain::TileKey pack(std::int64_t tx, std::int64_t ty) {
        const auto ux = static_cast<std::uint32_t>(static_cast<std::int32_t>(tx));
        const auto uy = static_cast<std::uint32_t>(static_cast<std::int32_t>(ty));
        return terrain::TileKey{static_cast<std::int64_t>((static_cast<std::uint64_t>(ux) << 32) | uy)};
    }
    static void unpack(terrain::TileKey k, std::int64_t& tx, std::int64_t& ty) {
        const auto u = static_cast<std::uint64_t>(k.packed);
        tx = static_cast<std::int32_t>(static_cast<std::uint32_t>(u >> 32));
        ty = static_cast<std::int32_t>(static_cast<std::uint32_t>(u & 0xFFFFFFFFu));
    }
    TerrainShape shape_;
    real tile_;
    std::uint32_t n_;
};

struct GroundParams {
    real tile = 16.0;
    std::uint32_t samples = 16;
    // Seamless heightfield extent: tiles [tx0, tx0+m) x [ty0, ty0+m), m odd so the sample count is even.
    int tx0 = -7, ty0 = -7, m = 15;
};

constexpr std::uint64_t kInterestId = 1;
constexpr real kStreamRadius = 48.0;

void install_ground(World& world, Ground g, TerrainShape shape, const GroundParams& gp, const Vec3& start) {
    if (g == Ground::Mixed || g == Ground::MixedHole || g == Ground::MeshOnly) {
        REQUIRE(shape == TerrainShape::Flat);
        HeightfieldShape hf;
        hf.sample_count = 16;
        hf.sample_spacing = 16.0 / 15.0;
        if (g == Ground::MixedHole) hf.sample_count = 32;
        hf.heights.assign(hf.sample_count * hf.sample_count, 0.0);
        hf.cell_surface_ids.assign((hf.sample_count-1) * (hf.sample_count-1), SurfaceId{0});
        if (g == Ground::MixedHole) {
            for (unsigned y = 0; y < hf.sample_count; ++y)
                for (unsigned x = 16; x < hf.sample_count; ++x)
                    hf.heights[y * hf.sample_count + x] = kHeightfieldNoCollision;
        }
        BodyDesc left;
        left.shape = hf;
        left.motion = BodyMotionType::Static;
        left.friction = 0.8;
        left.pose.position = Vec3{-16.0, 0.0, 0.0};
        if (g != Ground::MeshOnly) world.create_body(left);
        MeshShape mesh;
        const unsigned nx = g == Ground::MeshOnly ? 31u : 16u;
        const real origin_x = g == Ground::MeshOnly ? -16.0 : 0.0;
        for (int y = 0; y < 16; ++y)
            for (unsigned x = 0; x < nx; ++x)
                mesh.vertices.push_back(Vec3{origin_x + x * hf.sample_spacing, y * hf.sample_spacing, 0.0});
        for (std::uint32_t y = 0; y < 15; ++y)
            for (std::uint32_t x = 0; x + 1 < nx; ++x) {
                const auto a = y * nx + x;
                mesh.indices.insert(mesh.indices.end(), {a, a+1, a+nx+1, a, a+nx+1, a+nx});
                mesh.triangle_surface_ids.insert(mesh.triangle_surface_ids.end(), 2, SurfaceId{0});
            }
        BodyDesc right;
        right.shape = mesh;
        right.motion = BodyMotionType::Static;
        right.friction = 0.8;
        world.create_body(right);
    } else if (g == Ground::Tiles) {
        terrain::TerrainConfig tc;
        tc.max_resident_tiles = 512;
        tc.max_tile_fills_per_tick = 128;
        world.set_terrain_source(std::make_shared<SyntheticTerrain>(shape, gp.tile, gp.samples), tc);
        world.set_terrain_interest_point(kInterestId, start, kStreamRadius);
    } else if (g == Ground::Seamless) {
        const std::uint32_t per = gp.samples - 1;
        const std::uint32_t count = static_cast<std::uint32_t>(gp.m) * per + 1;
        REQUIRE(count % 2 == 0);
        const real sp = gp.tile / static_cast<real>(per);
        HeightfieldShape hf;
        hf.sample_spacing = sp;
        hf.sample_count = count;
        hf.heights.resize(static_cast<std::size_t>(count) * count);
        hf.cell_surface_ids.assign(static_cast<std::size_t>(count - 1) * (count - 1), SurfaceId{0});
        for (std::uint32_t gy = 0; gy < count; ++gy) {
            const std::uint32_t ty_local = std::min(gy / per, static_cast<std::uint32_t>(gp.m - 1));
            const real oy = static_cast<real>(gp.ty0 + static_cast<int>(ty_local)) * gp.tile;
            const real wy = oy + static_cast<real>(gy - ty_local * per) * sp;
            for (std::uint32_t gx = 0; gx < count; ++gx) {
                const std::uint32_t tx_local = std::min(gx / per, static_cast<std::uint32_t>(gp.m - 1));
                const real ox = static_cast<real>(gp.tx0 + static_cast<int>(tx_local)) * gp.tile;
                const real wx = ox + static_cast<real>(gx - tx_local * per) * sp;
                hf.heights[static_cast<std::size_t>(gy) * count + gx] = terrain_height(shape, wx, wy);
            }
        }
        BodyDesc d;
        d.shape = hf;
        d.motion = BodyMotionType::Static;
        d.friction = 0.8; // TileManager's own pool-body friction
        d.pose.position = Vec3{static_cast<real>(gp.tx0) * gp.tile, static_cast<real>(gp.ty0) * gp.tile, 0.0};
        world.create_body(d);
    } else {
        REQUIRE(shape == TerrainShape::Flat);
        BodyDesc d;
        d.shape = BoxShape{Vec3{2000.0, 2000.0, 0.5}};
        d.motion = BodyMotionType::Static;
        d.friction = 1.0;
        d.pose.position = Vec3{0.0, 0.0, -0.5};
        world.create_body(d);
    }
}

real nearest_seam_distance(real x, real tile) {
    const real k = std::round(x / tile);
    return std::abs(x - k * tile);
}

struct BodyRun {
    real peak_az = 0.0;
    real peak_az_x = 0.0;
    real peak_ax = 0.0;
    real min_clear = 1e9;
    real peak_contact_fz = 0.0; // net normal support from m*(az+g), NOT per-manifold solver impulse
    std::uint64_t hash = 1469598103934665603ull;
};

BodyRun run_body(Ground g, TerrainShape shape, BodyKind kind, real speed, const GroundParams& gp, real start_x,
                 real y, real distance, bool verbose, unsigned workers = 4) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.job_workers = workers;
    World world(cfg);
    install_ground(world, g, shape, gp, Vec3{start_x, y, 0.0});

    BodyDesc b;
    b.motion = BodyMotionType::Dynamic;
    b.allow_sleep = false;
    b.restitution = 0.0;
    b.max_angular_velocity = 1000.0; // Jolt's default 47 rad/s would clamp a 0.3 m sphere rolling at > 14 m/s
    real rest_h = 0.3;
    switch (kind) {
    case BodyKind::SlidingSphere:
        b.shape = SphereShape{0.3};
        b.mass = 5.0;
        b.friction = 0.0;
        break;
    case BodyKind::RollingSphere:
        b.shape = SphereShape{0.3};
        b.mass = 5.0;
        b.friction = 1.0;
        break;
    case BodyKind::SlidingBox:
        b.shape = BoxShape{Vec3{0.5, 0.4, 0.25}};
        b.mass = 50.0;
        b.friction = 0.0;
        rest_h = 0.25;
        break;
    case BodyKind::RollingCapsule:
        // Capsule axis is local Y (Jolt's convention) - lying across the travel direction.
        b.shape = CapsuleShape{0.4, 0.25};
        b.mass = 20.0;
        b.friction = 1.0;
        rest_h = 0.25;
        break;
    }
    b.pose.position = Vec3{start_x, y, terrain_height(shape, start_x, y) + rest_h + 0.005};
    const BodyId id = world.create_body(b);

    for (int i = 0; i < 120; ++i) {
        world.set_terrain_interest_point(kInterestId, world.get_pose(id).position, kStreamRadius);
        world.step();
    }
    {
        Motion m = world.get_motion(id);
        m.linear = Vec3{speed, 0.0, 0.0};
        if (kind == BodyKind::RollingSphere) m.angular = Vec3{0.0, speed / 0.3, 0.0};
        if (kind == BodyKind::RollingCapsule) m.angular = Vec3{0.0, speed / 0.25, 0.0};
        world.backend().set_motion(id, m);
    }
    BodyRun r;
    Motion prev = world.get_motion(id);
    const std::uint64_t n = static_cast<std::uint64_t>(distance / std::abs(speed) * cfg.tick_rate_hz);
    for (std::uint64_t t = 0; t < n; ++t) {
        world.set_terrain_interest_point(kInterestId, world.get_pose(id).position, kStreamRadius);
        world.step();
        const Pose p = world.get_pose(id);
        const Motion m = world.get_motion(id);
        r.hash = (r.hash ^ world.state_hash()) * 1099511628211ull;
        const real az = (m.linear.z - prev.linear.z) * cfg.tick_rate_hz;
        const real ax = (m.linear.x - prev.linear.x) * cfg.tick_rate_hz;
        prev = m;
        if (t < 2) continue; // launch tick
        if (std::abs(az) > std::abs(r.peak_az)) {
            r.peak_az = az;
            r.peak_az_x = p.position.x;
        }
        r.peak_contact_fz = std::max(r.peak_contact_fz, b.mass * (az + cfg.gravity.z * -1));
        r.peak_ax = std::max(r.peak_ax, std::abs(ax));
        r.min_clear = std::min(r.min_clear, p.position.z - terrain_height(shape, p.position.x, p.position.y));
        if (verbose && std::abs(az) > 5.0) {
            std::printf("    t=%llu x=%.4f seam_d=%.4f az=%.2f ax=%.2f vz=%.4f contacts:", static_cast<unsigned long long>(t),
                        p.position.x, nearest_seam_distance(p.position.x, gp.tile), az, ax, m.linear.z);
            for (const ContactEvent& ev : world.last_contact_events()) {
                if (ev.a != id && ev.b != id) continue;
                const BodyId other = ev.a == id ? ev.b : ev.a;
                std::printf(" [b%u n=(%.3f,%.3f,%.3f) p.x=%.3f]", other.index, ev.normal.x, ev.normal.y, ev.normal.z, ev.point.x);
            }
            std::printf("\n");
        }
    }
    REQUIRE(world.terrain_starved_tile_count() == 0);
    return r;
}

struct CarRun {
    real peak_az = 0.0, peak_az_x = 0.0;
    real peak_pitch_acc = 0.0;
    real peak_fz = 0.0, peak_fz_x = 0.0;
    std::uint64_t hash = 1469598103934665603ull;
    real wheel_peak_fz[4]{};
    real peak_dfz = 0.0; // largest tick-to-tick |dFz| on any wheel
};

// Holds the chassis's forward speed with a horizontal P-controlled force at
// its CoM (stateless, reads only ctx), so the car crosses the seams at the
// nominal speed even uphill with zero throttle (free-rolling wheels).
class SpeedHoldForce final : public IForceElement {
public:
    SpeedHoldForce(Frame frame, real mass, real target) : frame_(frame), mass_(mass), target_(target) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        return Wrench{Vec3{mass_ * real(3.0) * (target_ - ctx.frame_motion.linear.x), 0.0, 0.0}, Vec3::zero()};
    }

private:
    Frame frame_;
    real mass_;
    real target_;
};

CarRun run_car(Ground g, TerrainShape shape, real speed, const GroundParams& gp, real start_x, real y, real lead_s,
               real distance, bool verbose, unsigned workers = 4) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = workers;
    auto world = std::make_unique<World>(cfg);
    install_ground(*world, g, shape, gp, Vec3{start_x, y, 0.0});

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{start_x, y, terrain_height(shape, start_x, y) + 0.6};
    const BodyId ch = world->create_body(chassis);
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    world->set_surface_table(surfaces);
    const VehicleId veh = world->create_vehicle(io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"), ch);

    auto step = [&] {
        world->set_terrain_interest_point(kInterestId, world->get_pose(ch).position, kStreamRadius);
        world->step();
    };
    for (int i = 0; i < 360; ++i) step(); // settle 1.5 s
    world->backend().set_motion(ch, Motion{Vec3{speed, 0.0, 0.0}, Vec3::zero()});
    SpeedHoldForce hold(Frame(ch), chassis.mass, speed);
    world->add_force_element(&hold);
    for (int i = 0; i < static_cast<int>(lead_s * 240.0); ++i) step();

    CarRun r;
    Motion prev = world->get_motion(ch);
    std::vector<real> prev_fz(4);
    for (std::size_t w = 0; w < 4; ++w) prev_fz[w] = world->wheel_telemetry(veh, w).fz;
    const real x0 = world->get_pose(ch).position.x;
    std::uint64_t t = 0;
    while ((world->get_pose(ch).position.x - x0) * (speed > 0 ? 1 : -1) < distance && t < 100000) {
        step();
        ++t;
        const Pose p = world->get_pose(ch);
        const Motion m = world->get_motion(ch);
        r.hash = (r.hash ^ world->state_hash()) * 1099511628211ull;
        const real az = (m.linear.z - prev.linear.z) * 240.0;
        const real pitch_acc = (m.angular.y - prev.angular.y) * 240.0;
        prev = m;
        if (std::abs(az) > std::abs(r.peak_az)) {
            r.peak_az = az;
            r.peak_az_x = p.position.x;
        }
        r.peak_pitch_acc = std::max(r.peak_pitch_acc, std::abs(pitch_acc));
        real tick_max_dfz = 0.0;
        for (std::size_t w = 0; w < 4; ++w) {
            const real fz = world->wheel_telemetry(veh, w).fz;
            r.wheel_peak_fz[w] = std::max(r.wheel_peak_fz[w], fz);
            if (fz > r.peak_fz) {
                r.peak_fz = fz;
                r.peak_fz_x = p.position.x;
            }
            tick_max_dfz = std::max(tick_max_dfz, std::abs(fz - prev_fz[w]));
            prev_fz[w] = fz;
        }
        r.peak_dfz = std::max(r.peak_dfz, tick_max_dfz);
        if (verbose && (std::abs(az) > 1.0 || tick_max_dfz > 500.0)) {
            std::printf("    t=%llu x=%.3f az=%.2f pitch_acc=%.2f fz=[%.0f %.0f %.0f %.0f]\n", static_cast<unsigned long long>(t),
                        p.position.x, az, pitch_acc, world->wheel_telemetry(veh, 0).fz, world->wheel_telemetry(veh, 1).fz,
                        world->wheel_telemetry(veh, 2).fz, world->wheel_telemetry(veh, 3).fz);
        }
    }
    REQUIRE(world->terrain_starved_tile_count() == 0);
    return r;
}

real body_seam_bound(real seamless_peak_az) { return real(2.0) * std::abs(seamless_peak_az) + real(30.0); }
constexpr real kBodyAbsBound = 120.0;

} // namespace

TEST_CASE("terrain tile seams: sliding bodies cross seams without a contact spike", "[terrain][validation][long]") {
    GroundParams gp;
    for (TerrainShape shape : {TerrainShape::Flat, TerrainShape::Wavy}) {
        for (BodyKind kind : {BodyKind::SlidingSphere, BodyKind::SlidingBox}) {
            for (real v : {5.0, 20.0, 40.0}) {
                const real dist = std::max(40.0, 2.0 * v); // x = 8 -> >= 48: crosses the seams at 16, 32, 48
                const BodyRun tiles = run_body(Ground::Tiles, shape, kind, v, gp, 8.0, 8.0, dist, false);
                const BodyRun seamless = run_body(Ground::Seamless, shape, kind, v, gp, 8.0, 8.0, dist, false);
                CAPTURE(name_of(shape), name_of(kind), v, tiles.peak_az, tiles.peak_az_x, seamless.peak_az);
                CHECK(std::abs(tiles.peak_az) <= body_seam_bound(seamless.peak_az));
                CHECK(std::abs(tiles.peak_az) <= kBodyAbsBound);
            }
        }
    }
}

TEST_CASE("terrain tile seams: original 256 m / 64-sample repro at x = -512", "[terrain][validation]") {
    GroundParams gp;
    gp.tile = 256.0;
    gp.samples = 64;
    gp.tx0 = -3;
    gp.ty0 = -3;
    gp.m = 5;
    for (real v : {10.0, 20.0, 40.0}) {
        const BodyRun tiles = run_body(Ground::Tiles, TerrainShape::Flat, BodyKind::SlidingSphere, v, gp, -520.0, -150.0, 40.0, false);
        const BodyRun seamless =
            run_body(Ground::Seamless, TerrainShape::Flat, BodyKind::SlidingSphere, v, gp, -520.0, -150.0, 40.0, false);
        CAPTURE(v, tiles.peak_az, tiles.peak_az_x, seamless.peak_az);
        CHECK(std::abs(tiles.peak_az) <= body_seam_bound(seamless.peak_az));
        CHECK(std::abs(tiles.peak_az) <= kBodyAbsBound);
    }
}

TEST_CASE("terrain tile seams: a car crosses seams like a seamless heightfield", "[terrain][vehicle][validation][long]") {
    GroundParams gp;
    for (TerrainShape shape : {TerrainShape::Flat, TerrainShape::Slope, TerrainShape::Wavy}) {
        for (real v : {5.0, 20.0, 40.0}) {
            const real lead = 1.0;
            const real start_x = -40.0 - v * lead;
            const real dist = std::max(40.0, 1.5 * v); // from x ~ -40: crosses the seams at -32, -16, 0 (, 16, 32)
            const CarRun tiles = run_car(Ground::Tiles, shape, v, gp, start_x, 8.0, lead, dist, false);
            const CarRun seamless = run_car(Ground::Seamless, shape, v, gp, start_x, 8.0, lead, dist, false);
            CAPTURE(name_of(shape), v, tiles.peak_az, seamless.peak_az, tiles.peak_pitch_acc, seamless.peak_pitch_acc,
                    tiles.peak_fz, seamless.peak_fz);
            CHECK(std::abs(tiles.peak_az) <= real(2.0) * std::abs(seamless.peak_az) + real(0.5));
            CHECK(tiles.peak_pitch_acc <= real(2.0) * seamless.peak_pitch_acc + real(0.5));
            CHECK(tiles.peak_fz <= real(1.05) * seamless.peak_fz);
        }
    }
}

TEST_CASE("seam crossing diagnostics: dynamic bodies", "[.][diag][terrain]") {
    const bool verbose = env_flag("PS_SEAM_VERBOSE");
    GroundParams gp;
    for (TerrainShape shape : {TerrainShape::Flat, TerrainShape::Slope, TerrainShape::Wavy}) {
        for (BodyKind kind : {BodyKind::SlidingSphere, BodyKind::RollingSphere, BodyKind::SlidingBox, BodyKind::RollingCapsule}) {
            for (real v : {5.0, 10.0, 20.0, 40.0}) {
                const real dist = std::max(40.0, 2.0 * v);
                for (Ground g : {Ground::Tiles, Ground::Seamless}) {
                    const BodyRun r = run_body(g, shape, kind, v, gp, 8.0, 8.0, dist, verbose && g == Ground::Tiles);
                    std::printf("%-6s %-13s v=%4.0f %-8s peak|az|=%8.2f at x=%8.3f (seam_d=%.3f) peak|ax|=%7.2f min_clear=%.4f\n",
                                name_of(shape), name_of(kind), v, name_of(g), r.peak_az, r.peak_az_x,
                                nearest_seam_distance(r.peak_az_x, gp.tile), r.peak_ax, r.min_clear);
                }
            }
        }
    }
}

TEST_CASE("seam crossing diagnostics: original 256 m / 64-sample repro", "[.][diag][terrain]") {
    GroundParams gp;
    gp.tile = 256.0;
    gp.samples = 64;
    gp.tx0 = -3;
    gp.ty0 = -3;
    gp.m = 5;
    for (real v : {10.0, 20.0, 40.0}) {
        for (Ground g : {Ground::Tiles, Ground::Seamless}) {
            const BodyRun r = run_body(g, TerrainShape::Flat, BodyKind::SlidingSphere, v, gp, -520.0, -150.0, 40.0, true);
            std::printf("256/64 flat slide-sphere v=%4.0f %-8s peak|az|=%8.2f at x=%8.3f\n", v, name_of(g), r.peak_az, r.peak_az_x);
        }
    }
}

TEST_CASE("seam crossing diagnostics: car", "[.][diag][terrain][vehicle]") {
    const bool verbose = env_flag("PS_SEAM_VERBOSE");
    GroundParams gp;
    for (TerrainShape shape : {TerrainShape::Flat, TerrainShape::Slope, TerrainShape::Wavy}) {
        for (real v : {5.0, 20.0, 40.0}) {
            const real lead = 1.0;
            const real start_x = -40.0 - v * lead;
            const real dist = std::max(40.0, 1.5 * v);
            for (Ground g : {Ground::Tiles, Ground::Seamless, Ground::Box}) {
                if (g == Ground::Box && shape != TerrainShape::Flat) continue;
                const CarRun r = run_car(g, shape, v, gp, start_x, 8.0, lead, dist, verbose && g == Ground::Tiles);
                std::printf("car %-6s v=%4.0f %-8s peak|az|=%7.3f at x=%8.3f (seam_d=%.3f) pitch_acc=%7.3f peak_fz=%8.1f at x=%8.3f max|dFz|/tick=%7.1f\n",
                            name_of(shape), v, name_of(g), r.peak_az, r.peak_az_x, nearest_seam_distance(r.peak_az_x, gp.tile),
                            r.peak_pitch_acc, r.peak_fz, r.peak_fz_x, r.peak_dfz);
            }
        }
    }
}

// Equivalent to the test_ground boundary at x=-500: a flush grid edge
// shared by different static shape types. MixedHole also pins the case
// where the edge lies inside the heightfield AABB after sample suppression.
// The baseline includes BOTH continuous shape types: Jolt's stock standalone
// mesh/sphere path has tick-scale support oscillations (~g), unrelated to the
// boundary kick. Preserve that path; reject the extra O(100) m/s^2 edge kick.
// Rigid-body support force is inferred from vertical momentum plus gravity;
// ContactEvent::normal_impulse is currently unpopulated and must not be used.
TEST_CASE("Flush heightfield mesh boundary crosses without a contact kick", "[terrain][validation][mesh_boundary]") {
    GroundParams gp;
    gp.tx0 = -1; gp.ty0 = 0; gp.m = 3;
    for (const real speed : {10.0, -10.0, 20.0, -20.0, 40.0, -40.0}) {
        const real start = speed > 0 ? -8.0 : 8.0;
        for (const auto kind : {BodyKind::SlidingSphere, BodyKind::SlidingBox}) {
            const auto baseline = run_body(Ground::Seamless, TerrainShape::Flat, kind, speed, gp, start, 8.0, 16.0, false);
            const auto mesh_baseline = run_body(Ground::MeshOnly, TerrainShape::Flat, kind, speed, gp, start, 8.0, 16.0, false);
            for (auto ground : {Ground::Mixed, Ground::MixedHole}) {
                const auto mixed = run_body(ground, TerrainShape::Flat, kind, speed, gp, start, 8.0, 16.0, false);
                std::printf("MESH_BOUNDARY body=%s speed=%.0f baseline_az=%.6f mixed_az=%.6f peak_x=%.6f\n", name_of(kind), speed, baseline.peak_az, mixed.peak_az, mixed.peak_az_x);
                std::printf("  support_N baseline=%.6f mixed=%.6f hash=%llu\n", baseline.peak_contact_fz, mixed.peak_contact_fz, static_cast<unsigned long long>(mixed.hash));
                std::printf("  hole=%d mesh_baseline_az=%.6f mesh_support_N=%.6f\n", ground == Ground::MixedHole, mesh_baseline.peak_az, mesh_baseline.peak_contact_fz);
                CHECK(std::abs(mixed.peak_az) <= 2 * std::max(std::abs(baseline.peak_az), std::abs(mesh_baseline.peak_az)) + 5.0);
                for (unsigned workers : {1u, 24u}) {
                    const auto parallel = run_body(ground, TerrainShape::Flat, kind, speed, gp, start, 8.0, 16.0, false, workers);
                    CHECK(parallel.hash == mixed.hash);
                }
            }
        }
    }
}
TEST_CASE("Flush heightfield mesh wheel crossings and worker determinism", "[terrain][validation][mesh_boundary]") {
    GroundParams gp;
    gp.tx0 = -1; gp.ty0 = 0; gp.m = 3;
    for (const real speed : {10.0, -10.0, 20.0, -20.0}) {
        const real start = speed > 0 ? -10.0 : 10.0;
        const auto baseline = run_car(Ground::Seamless, TerrainShape::Flat, speed, gp, start, 8.0, 0.05, 16.0, false, 1);
        const auto reference = run_car(Ground::Mixed, TerrainShape::Flat, speed, gp, start, 8.0, 0.05, 16.0, false, 1);
        for (unsigned workers : {1u, 4u, 24u}) {
            const auto mixed = run_car(Ground::Mixed, TerrainShape::Flat, speed, gp, start, 8.0, 0.05, 16.0, false, workers);
            std::printf("MESH_BOUNDARY car speed=%.0f workers=%u baseline_az=%.6f mixed_az=%.6f baseline_fz=%.6f mixed_fz=%.6f delta_fz=%.6f\n", speed, workers, baseline.peak_az, mixed.peak_az, baseline.peak_fz, mixed.peak_fz, mixed.peak_dfz);
            CHECK(std::abs(mixed.peak_az) <= 2 * std::abs(baseline.peak_az) + 0.5);
            CHECK(mixed.peak_fz <= baseline.peak_fz * 1.05);
            CHECK(mixed.hash == reference.hash);
            for (unsigned wheel = 0; wheel < 4; ++wheel) {
                std::printf("  wheel=%u baseline_fz=%.6f mixed_fz=%.6f hash=%llu\n", wheel, baseline.wheel_peak_fz[wheel], mixed.wheel_peak_fz[wheel], static_cast<unsigned long long>(mixed.hash));
                CHECK(mixed.wheel_peak_fz[wheel] <= baseline.wheel_peak_fz[wheel] * 1.05);
            }
        }
    }
}
