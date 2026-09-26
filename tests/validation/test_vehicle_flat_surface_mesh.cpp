// tests/validation/test_vehicle_flat_surface_mesh.cpp — PHYS-047: pins
// tests/support/vehicle_test_helpers.h's flat_surface_mesh() (used by
// build_vehicle_rig_on_surface(), a consumer of which sits right next to
// this file - test_vehicle_accel_standing_start.cpp's grass/dirt case)
// against the large-triangle Jolt CastShape artefact test_powertrain_
// split_mu.cpp's split_surface_mesh() diagnosed and fixed first (vault
// PHYS-047).
//
// Method borrowed directly from test_powertrain_split_mu.cpp's
// probe_wheel_casts(): sweep the real front-wheel suspension CylinderShape
// cast (WheelInstance::measure_contact's own query - car_sedan.json's front
// wheel: half-width 0.1125, radius 0.317, yaw -0.2 rad, 0.18 m sweep from
// z0=0.4374, the only geometry any flat_surface_mesh() caller uses) over a
// grid straddling the ground's diagonal (the seam a 2-triangle square's CCW
// split used to put between (-he,-he) and (he,he) - a ~2828 m edge for the
// old default half_extent=1000 m, well into the range that produced bad
// casts for split_surface_mesh's 4-triangle ground) and counts every pose
// whose result disagrees with the analytic answer for an ideal flat plane
// at z=0: a hit, not started_penetrating, fraction within 1e-3 of the
// expected value, and the ground's single surface id.
//
// The legacy 2-triangle mesh is reproduced locally (not exported from
// vehicle_test_helpers.h any more) purely to document, via the [.][diag]
// case below, what the fix replaced - mirrors split_surface_mesh.cpp's own
// legacy_two_quad_split_mesh()/"legacy ground cast probe" pattern.

#include "vehicle_test_helpers.h"

#include "ps/io/surface_table.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using namespace ps;

namespace {

struct CastProbeStats {
    int probed = 0;
    int wrong = 0;
    real worst_dfraction = 0.0;
};

// The old 2-triangle flat_surface_mesh(), kept ONLY for the [.][diag] case
// below that documents why the rig no longer uses it.
MeshShape legacy_two_triangle_flat_mesh(real half_extent, SurfaceId surface) {
    MeshShape m;
    m.vertices = {
        Vec3{-half_extent, -half_extent, 0.0},
        Vec3{half_extent, -half_extent, 0.0},
        Vec3{half_extent, half_extent, 0.0},
        Vec3{-half_extent, half_extent, 0.0},
    };
    m.indices = {0, 1, 2, 0, 2, 3};
    m.triangle_surface_ids = {surface, surface};
    return m;
}

// Same wheel-cast geometry as test_powertrain_split_mu.cpp's
// probe_wheel_casts(): car_sedan.json's front wheel query
// (CylinderShape half-width 0.1125, r 0.317, yaw -0.2 rad, swept 0.18 m
// straight down from z0=0.4374). Sweeps a 0.01 m grid over x, y in
// [-2, 2] m - centred on the world origin, where build_vehicle_rig_on_
// surface() places the chassis, and therefore where the ground's diagonal
// seam (running through the origin) is actually exercised by a settled
// car's wheels.
CastProbeStats probe_wheel_casts(const MeshShape& ground, SurfaceId surface, bool print_first_wrong) {
    World w{WorldConfig{}};
    BodyDesc g;
    g.shape = ground;
    g.motion = BodyMotionType::Static;
    w.create_body(g);

    const real z0 = 0.4374, radius = 0.317, sweep = 0.18;
    const real expected_fraction = (z0 - radius) / sweep;
    const Quat q = Quat::from_axis_angle(Vec3{0, 0, 1}, -0.2);
    const CylinderShape cyl{0.1125, radius};
    CastProbeStats st;
    for (int ix = -200; ix <= 200; ++ix) {
        for (int iy = -200; iy <= 200; ++iy) {
            const Vec3 p{ix * 0.01, iy * 0.01, z0};
            const ShapeCastHit h = w.backend().shape_cast(cyl, Pose{p, q}, Vec3{0, 0, -sweep}, sweep);
            ++st.probed;
            const real d = h.hit ? std::abs(h.fraction - expected_fraction) : real(1.0);
            st.worst_dfraction = std::max(st.worst_dfraction, d);
            const bool ok = h.hit && !h.started_penetrating && d <= 1e-3 && h.surface == surface;
            if (!ok) {
                ++st.wrong;
                if (print_first_wrong && st.wrong <= 10) {
                    std::cout << "  wrong cast at (" << p.x << ", " << p.y << "): hit=" << h.hit
                              << " fraction=" << h.fraction << " (expected " << expected_fraction
                              << ") started_penetrating=" << h.started_penetrating << " surface=" << h.surface
                              << "\n";
                }
            }
        }
    }
    return st;
}

} // namespace

// Pins the fix: the tessellated flat_surface_mesh() must answer the front
// wheel's own shape cast exactly like an ideal plane over the region a
// settled car's wheels actually sweep near the world origin (see the
// [.][diag] case below for the 2-triangle ground's bad-cast count this
// replaced).
TEST_CASE("flat_surface_mesh ground validity: wheel shape casts near the diagonal seam match an ideal plane",
          "[validation][vehicle]") {
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId asphalt = surfaces->id_for("asphalt");
    const CastProbeStats st = probe_wheel_casts(ps_test::flat_surface_mesh(1000.0, asphalt), asphalt, true);
    INFO("probed=" << st.probed << " wrong=" << st.wrong << " worst|dfraction|=" << st.worst_dfraction);
    CHECK(st.probed == 401 * 401);
    CHECK(st.wrong == 0);
}

// Reproduces the old 2-triangle flat_surface_mesh()'s cast errors near its
// diagonal seam, for the record (mirrors test_powertrain_split_mu.cpp's
// "legacy ground cast probe" case).
TEST_CASE("flat_surface_mesh diagnostic: legacy 2-triangle ground cast probe", "[.][diag]") {
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId asphalt = surfaces->id_for("asphalt");
    std::cout << "legacy 2-triangle flat_surface_mesh, half-extent 1000 m:\n";
    const CastProbeStats legacy = probe_wheel_casts(legacy_two_triangle_flat_mesh(1000.0, asphalt), asphalt, true);
    std::cout << "  probed=" << legacy.probed << " wrong=" << legacy.wrong
              << " worst|dfraction|=" << legacy.worst_dfraction << "\n";
    const CastProbeStats fixed = probe_wheel_casts(ps_test::flat_surface_mesh(1000.0, asphalt), asphalt, false);
    std::cout << "tessellated flat_surface_mesh: probed=" << fixed.probed << " wrong=" << fixed.wrong
              << " worst|dfraction|=" << fixed.worst_dfraction << "\n";
}
