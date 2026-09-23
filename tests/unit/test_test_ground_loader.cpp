// PLAN.md M1 step 4: data/tracks/test_ground's own static collision data
// (test_ground_collision.json + .bin), loaded via
// ps::io::load_test_ground_collision, must produce real Jolt static mesh
// bodies whose surface ids (ps::io::SurfaceTable, data/surfaces/
// surfaces.json) survive an actual collision - a dynamic body dropped onto
// asphalt and onto the grass patch must report the RIGHT surface id at
// contact (ContactEvent::surface_a/surface_b, PLAN.md section 6).
//
// The two drop points below were found empirically with a throwaway probe
// (ps::IRigidBackend::shape_cast straight down at a grid of candidate
// (x, y) - the same technique test_mesh_heightfield_shapes.cpp's
// orientation test uses) rather than guessed from the JSON's own AABBs:
// test_ground_collision.json's per-surface AABB spans a much larger area
// than that surface's actual triangles cover (it is a real winding track,
// not a filled rectangle), so most "clearly inside the AABB" points are
// actually off the mesh entirely and would make the ball fall forever.
// (-300, -1100) hits asphalt at z ~= 0.4715 m; (-300, -480) hits the grass
// patch at z ~= 0.05 m (both from an actual probe run against this exact
// file, not a formula) - see this file's own header comment history in
// out/handover_terrain.md for the probe's full output.

#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;

namespace {

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

// Drops a small dynamic sphere from `drop_z` straight down at (x, y) and
// returns the SurfaceId reported by the first contact event involving it
// (kInvalidSurfaceId if none was ever seen within `max_ticks`).
SurfaceId drop_and_find_contact_surface(World& world, real x, real y, real drop_z, int max_ticks) {
    BodyDesc ball;
    ball.shape = SphereShape{0.3};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{x, y, drop_z};
    ball.allow_sleep = false;
    const BodyId ball_id = world.create_body(ball);

    for (int i = 0; i < max_ticks; ++i) {
        world.step();
        for (const ContactEvent& ev : world.last_contact_events()) {
            if (ev.a != ball_id && ev.b != ball_id) continue;
            const SurfaceId s = (ev.a == ball_id) ? ev.surface_b : ev.surface_a;
            if (s != kInvalidSurfaceId) return s;
        }
    }
    return kInvalidSurfaceId;
}

} // namespace

TEST_CASE("load_test_ground_collision creates one static body per surface group", "[io][test_ground]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    const std::vector<BodyId> ids = io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    // test_ground_collision.json currently lists 5 surface groups (asphalt,
    // dirt, grass, kerb, low_mu) - not hardcoded elsewhere in the loader
    // itself, but this test's own expectation of that file's shape.
    REQUIRE(ids.size() == 5);
    for (BodyId id : ids) REQUIRE(id.valid());
}

TEST_CASE("A body dropped on the test_ground asphalt reports the asphalt surface id at contact", "[io][test_ground][contact]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    const SurfaceId expected = surfaces.id_for("asphalt");
    REQUIRE(expected != kInvalidSurfaceId);

    const SurfaceId found = drop_and_find_contact_surface(world, -300.0, -1100.0, /*drop_z=*/5.0, /*max_ticks=*/480);
    REQUIRE(found == expected);
}

TEST_CASE("A body dropped on the test_ground grass patch reports the grass surface id at contact", "[io][test_ground][contact]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    const SurfaceId expected = surfaces.id_for("grass");
    REQUIRE(expected != kInvalidSurfaceId);

    const SurfaceId found = drop_and_find_contact_surface(world, -300.0, -480.0, /*drop_z=*/2.0, /*max_ticks=*/480);
    REQUIRE(found == expected);
}

TEST_CASE("SurfaceTable assigns stable, distinct ids to every surfaces.json entry", "[io][surface_table]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    // surfaces.json currently defines asphalt, dirt, field, grass, kerb, low_mu.
    REQUIRE(surfaces.size() == 6);
    REQUIRE(surfaces.id_for("asphalt") != kInvalidSurfaceId);
    REQUIRE(surfaces.id_for("grass") != kInvalidSurfaceId);
    REQUIRE(surfaces.id_for("asphalt") != surfaces.id_for("grass"));
    REQUIRE(surfaces.id_for("does_not_exist") == kInvalidSurfaceId);
    REQUIRE(surfaces.name_for(surfaces.id_for("dirt")) == "dirt");
}

namespace {

// A throwaway track directory holding a synthetic test_ground_collision.json
// + .bin pair, so the loader's error paths can be exercised without touching
// data/tracks (read-only). Removed on destruction.
struct SyntheticTrackDir {
    std::filesystem::path dir;

    explicit SyntheticTrackDir(const std::string& tag) {
        std::random_device rd;
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_ground_loader_" + tag + "_" + std::to_string(rd()) + std::to_string(rd()));
        std::filesystem::create_directories(dir);
    }
    ~SyntheticTrackDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }

    void write_json(std::size_t vertex_offset, std::size_t vertex_count, std::size_t index_offset,
                    std::size_t index_count) const {
        std::ofstream out(dir / "test_ground_collision.json");
        out << R"({"surfaces":[{"surface":"asphalt","vertex_offset":)" << vertex_offset
            << R"(,"vertex_count":)" << vertex_count << R"(,"index_offset":)" << index_offset
            << R"(,"index_count":)" << index_count << "}]}";
    }
    void write_bin(const std::vector<unsigned char>& bytes) const {
        std::ofstream out(dir / "test_ground_collision.bin", std::ios::binary);
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
};

// One triangle: 3 vertices (36 bytes) followed by 3 indices (12 bytes).
std::vector<unsigned char> one_triangle_bin(std::uint32_t third_index = 2) {
    const float vertices[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    const std::uint32_t indices[3] = {0, 1, third_index};
    std::vector<unsigned char> bytes(sizeof(vertices) + sizeof(indices));
    std::memcpy(bytes.data(), vertices, sizeof(vertices));
    std::memcpy(bytes.data() + sizeof(vertices), indices, sizeof(indices));
    return bytes;
}

// The loader's exception message, or "" if it did not throw. Caught here
// rather than via REQUIRE_THROWS_WITH: on the clang-cl presets Catch2's own
// library is built without /EHsc (ps_options only reaches this project's
// targets), so Catch2 cannot translate an in-flight exception's message.
std::string load_error(World& world, const SyntheticTrackDir& track, const io::SurfaceTable& surfaces) {
    try {
        io::load_test_ground_collision(world, track.dir.string(), surfaces);
    } catch (const std::runtime_error& e) {
        return e.what();
    }
    return "";
}

} // namespace

TEST_CASE("load_test_ground_collision rejects a truncated or corrupt .bin loudly", "[io][test_ground]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    using Catch::Matchers::ContainsSubstring;

    SECTION("a well-formed synthetic file loads (baseline for the cases below)") {
        SyntheticTrackDir track("ok");
        track.write_json(0, 3, 36, 3);
        track.write_bin(one_triangle_bin());
        REQUIRE(io::load_test_ground_collision(world, track.dir.string(), surfaces).size() == 1);
    }
    SECTION("index block cut short") {
        SyntheticTrackDir track("truncated");
        track.write_json(0, 3, 36, 3);
        std::vector<unsigned char> bytes = one_triangle_bin();
        bytes.resize(44);
        track.write_bin(bytes);
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("index block") && ContainsSubstring("truncated or corrupt"));
    }
    SECTION("empty .bin") {
        SyntheticTrackDir track("empty");
        track.write_json(0, 3, 36, 3);
        track.write_bin({});
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("vertex block") && ContainsSubstring("truncated or corrupt"));
    }
    SECTION("vertex offset past the end of the file") {
        SyntheticTrackDir track("offset");
        track.write_json(4096, 3, 36, 3);
        track.write_bin(one_triangle_bin());
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("vertex block") && ContainsSubstring("truncated or corrupt"));
    }
    SECTION("a count so large that offset + count * size would overflow") {
        SyntheticTrackDir track("overflow");
        track.write_json(0, std::numeric_limits<std::size_t>::max() / 4, 36, 3);
        track.write_bin(one_triangle_bin());
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("vertex block") && ContainsSubstring("truncated or corrupt"));
    }
    SECTION("a triangle index past the group's own vertex block") {
        SyntheticTrackDir track("badindex");
        track.write_json(0, 3, 36, 3);
        track.write_bin(one_triangle_bin(3));
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("triangle index 3") && ContainsSubstring("corrupt"));
    }
    SECTION("missing .bin") {
        SyntheticTrackDir track("missing");
        track.write_json(0, 3, 36, 3);
        REQUIRE_THAT(load_error(world, track, surfaces),
                            ContainsSubstring("cannot open"));
    }
}
