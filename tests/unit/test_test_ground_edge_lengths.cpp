// tests/unit/test_test_ground_edge_lengths.cpp — PHYS-047 read-only exposure
// analysis of data/tracks/test_ground/test_ground_collision.{json,bin}
// (owned by a separate session, trackgen; READ-ONLY from here, never
// modified) against the large-triangle Jolt CastShape artefact
// tests/validation/test_powertrain_split_mu.cpp diagnosed and fixed first
// (vault ticket PHYS-047): Jolt's CastShape - the wheel's own suspension
// CylinderShape cast, WheelInstance::measure_contact - gives wrong results
// (phantom fraction-0 hits ~0.12 m above the plane, wrong-side surface ids,
// fraction errors up to 0.23) near the edge of a very large triangle, with
// the bad-cast count growing with edge length (0/6/80/276 bad casts out of
// 28471 probes for 4-triangle +-100/300/500/1000 m grounds; 0 for a 20 m
// tessellation).
//
// [.][diag] only: this is a REPORT, not a pass/fail gate - the task this
// file was written for is explicit that fixing anything under
// data/tracks/test_ground (a loader-side subdivision, or a trackgen-side
// retessellation) is out of scope; see the task's own final report for the
// recommendation between those two options.
//
// Parses the .json/.bin directly (duplicating test_ground_loader.cpp's own
// minimal little-endian reading, not calling it) because this analysis
// needs the raw per-triangle vertex geometry to compute edge lengths -
// load_test_ground_collision() only returns opaque BodyIds, and there is no
// public accessor to read a MeshShape's vertices back out of a live World.
// The bad-cast probing step DOES go through the real production loader
// (ps::io::load_test_ground_collision), so the cast results are exactly
// what a real World with this track loaded would see.

#include "ps/backend/i_rigid_backend.h"
#include "ps/backend/shape_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/world/world.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using json = nlohmann::json;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

struct Group {
    std::string surface;
    std::vector<Vec3> vertices;
    std::vector<std::uint32_t> indices;
};

std::vector<unsigned char> read_whole_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    REQUIRE(static_cast<bool>(in));
    const std::streamoff size = in.tellg();
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
    in.seekg(0, std::ios::beg);
    if (!bytes.empty()) in.read(reinterpret_cast<char*>(bytes.data()), size);
    REQUIRE(static_cast<bool>(in));
    return bytes;
}

std::vector<Group> load_groups(const std::string& track_dir) {
    std::ifstream json_in(track_dir + "/test_ground_collision.json");
    REQUIRE(static_cast<bool>(json_in));
    json j;
    json_in >> j;
    const std::vector<unsigned char> bin = read_whole_file(track_dir + "/test_ground_collision.bin");

    std::vector<Group> groups;
    for (const auto& sj : j.at("surfaces")) {
        Group g;
        g.surface = sj.at("surface").get<std::string>();
        const std::size_t vertex_offset = sj.at("vertex_offset").get<std::size_t>();
        const std::size_t vertex_count = sj.at("vertex_count").get<std::size_t>();
        const std::size_t index_offset = sj.at("index_offset").get<std::size_t>();
        const std::size_t index_count = sj.at("index_count").get<std::size_t>();

        g.vertices.reserve(vertex_count);
        for (std::size_t v = 0; v < vertex_count; ++v) {
            float xyz[3];
            std::memcpy(xyz, bin.data() + vertex_offset + v * sizeof(xyz), sizeof(xyz));
            g.vertices.push_back(Vec3{static_cast<real>(xyz[0]), static_cast<real>(xyz[1]), static_cast<real>(xyz[2])});
        }
        g.indices.resize(index_count);
        if (index_count > 0) std::memcpy(g.indices.data(), bin.data() + index_offset, index_count * sizeof(std::uint32_t));
        groups.push_back(std::move(g));
    }
    return groups;
}

struct TriRef {
    real max_edge = 0.0;
    std::array<Vec3, 3> v{};
};

struct EdgeStats {
    std::size_t triangle_count = 0;
    std::size_t over_50 = 0, over_100 = 0, over_500 = 0;
    real true_max_edge = 0.0;              // over EVERY triangle, not just those over 50 m
    std::vector<TriRef> worst; // up to 3 worst triangles over 50 m, descending by max_edge
};

void consider_worst(std::vector<TriRef>& worst, const TriRef& t, std::size_t keep = 3) {
    worst.push_back(t);
    std::sort(worst.begin(), worst.end(), [](const TriRef& a, const TriRef& b) { return a.max_edge > b.max_edge; });
    if (worst.size() > keep) worst.resize(keep);
}

EdgeStats analyze(const Group& g) {
    EdgeStats st;
    st.triangle_count = g.indices.size() / 3;
    for (std::size_t t = 0; t < st.triangle_count; ++t) {
        const Vec3& a = g.vertices[g.indices[t * 3 + 0]];
        const Vec3& b = g.vertices[g.indices[t * 3 + 1]];
        const Vec3& c = g.vertices[g.indices[t * 3 + 2]];
        const real m = std::max({distance(a, b), distance(b, c), distance(c, a)});
        st.true_max_edge = std::max(st.true_max_edge, m);
        if (m > 50.0) ++st.over_50;
        if (m > 100.0) ++st.over_100;
        if (m > 500.0) ++st.over_500;
        if (m > 50.0) consider_worst(st.worst, TriRef{m, {a, b, c}});
    }
    return st;
}

bool is_flat(const std::array<Vec3, 3>& v, real tol = 1e-3) {
    const real z0 = v[0].z;
    return std::abs(v[1].z - z0) < tol && std::abs(v[2].z - z0) < tol;
}

struct CastProbeStats {
    int probed = 0;
    int wrong = 0;
    real worst_dfraction = 0.0;
};

// Same wheel-cast geometry as test_powertrain_split_mu.cpp's
// probe_wheel_casts() (car_sedan.json's front wheel query). Grid-probes the
// 2D bounding box of `tri` (assumed flat, all vertices at `z_plane`) at
// `spacing` m, against the ALREADY-LOADED World `w` (the real production
// load_test_ground_collision() output, so this exercises exactly what a
// real World with this track sees) and counts casts disagreeing with the
// analytic flat-plane answer at `z_plane` with surface id `expected_surface`.
CastProbeStats probe_wheel_casts_over_triangle(const World& w, const std::array<Vec3, 3>& tri, SurfaceId expected_surface,
                                               real spacing, real margin) {
    const real z_plane = tri[0].z;
    real min_x = tri[0].x, max_x = tri[0].x, min_y = tri[0].y, max_y = tri[0].y;
    for (const Vec3& v : tri) {
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y);
        max_y = std::max(max_y, v.y);
    }
    min_x -= margin;
    min_y -= margin;
    max_x += margin;
    max_y += margin;

    const real radius = 0.317, half_width = 0.1125, sweep = 0.18;
    const real z0 = z_plane + 0.4374;
    const real expected_fraction = (z0 - z_plane - radius) / sweep;
    const Quat q = Quat::from_axis_angle(Vec3{0, 0, 1}, -0.2);
    const CylinderShape cyl{half_width, radius};

    CastProbeStats st;
    for (real x = min_x; x <= max_x; x += spacing) {
        for (real y = min_y; y <= max_y; y += spacing) {
            const Vec3 p{x, y, z0};
            const ShapeCastHit h = w.backend().shape_cast(cyl, Pose{p, q}, Vec3{0, 0, -sweep}, sweep);
            ++st.probed;
            const real d = h.hit ? std::abs(h.fraction - expected_fraction) : real(1.0);
            st.worst_dfraction = std::max(st.worst_dfraction, d);
            const bool ok = h.hit && !h.started_penetrating && d <= 1e-3 && h.surface == expected_surface;
            if (!ok) ++st.wrong;
        }
    }
    return st;
}

} // namespace

TEST_CASE("diagnostic: data/tracks/test_ground edge-length exposure to the large-triangle cast artefact",
          "[.][diag][terrain][vehicle]") {
    const std::string track_dir = std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground";
    const std::vector<Group> groups = load_groups(track_dir);

    // Load the SAME data through the real production loader, so the probe
    // step below exercises the exact bodies/surface ids a real World gets.
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    World w{WorldConfig{}};
    io::load_test_ground_collision(w, track_dir, *surfaces);

    std::printf("\n[test_ground edge-length exposure report]\n");
    for (const Group& g : groups) {
        const EdgeStats st = analyze(g);
        std::printf("  surface=%-8s triangles=%8zu max_edge=%9.2f m  over50=%6zu over100=%6zu over500=%6zu\n",
                    g.surface.c_str(), st.triangle_count, static_cast<double>(st.true_max_edge), st.over_50, st.over_100,
                    st.over_500);

        if (st.worst.empty()) continue; // no triangle over 50 m in this group

        const SurfaceId sid = surfaces->id_for(g.surface);
        for (std::size_t k = 0; k < st.worst.size(); ++k) {
            const TriRef& tri = st.worst[k];
            const Vec3 centroid = (tri.v[0] + tri.v[1] + tri.v[2]) / 3.0;
            std::printf("    worst[%zu]: max_edge=%.2f m centroid=(%.1f, %.1f, %.2f) flat=%d\n", k,
                        static_cast<double>(tri.max_edge), static_cast<double>(centroid.x),
                        static_cast<double>(centroid.y), static_cast<double>(centroid.z), is_flat(tri.v) ? 1 : 0);
            if (!is_flat(tri.v)) {
                std::printf("      skipped bad-cast probe: not flat (PHYS-047's analytic-answer method needs a "
                            "known-flat plane)\n");
                continue;
            }
            // Spacing matches test_powertrain_split_mu.cpp's/
            // test_vehicle_flat_surface_mesh.cpp's own established probe
            // density (0.01-0.02 m) rather than scaling with the triangle's
            // own edge length - PHYS-047's bad casts are a NEAR-SEAM
            // phenomenon, so a spacing that grows with a huge triangle's
            // edge (e.g. edge/300 for a 2828 m edge) would badly undersample
            // exactly where the artefact lives. Capped at 0.1 m so a
            // moderately large flat patch (dirt/grass here, max edge 174 m)
            // still finishes in a reasonable time over its whole footprint.
            const real spacing = std::min(std::max(tri.max_edge / 3000.0, 0.02), 0.1);
            const CastProbeStats ps = probe_wheel_casts_over_triangle(w, tri.v, sid, spacing, /*margin=*/0.0);
            std::printf("      probe: spacing=%.3f m probed=%d wrong=%d (%.2f%%) worst|dfraction|=%.4f\n",
                        static_cast<double>(spacing), ps.probed, ps.wrong,
                        ps.probed > 0 ? 100.0 * static_cast<double>(ps.wrong) / static_cast<double>(ps.probed) : 0.0,
                        static_cast<double>(ps.worst_dfraction));
        }
    }
    std::printf("[end test_ground edge-length exposure report]\n\n");
}
