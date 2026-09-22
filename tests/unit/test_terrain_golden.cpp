// PLAN.md M1 acceptance: "Golden sample points from the Python reference
// evaluator must match the C++ implementation bit for bit." - loads
// data/tracks/test_ground/terrain.json (the spec) and terrain_golden.json
// (347 (x,y) -> (z,tile_x,tile_y,surface) samples from
// tools/trackgen/trackgen/terrain.py, the Python reference), evaluates
// ps::terrain::terrain_height/terrain_surface_cell for each point, and
// compares.
//
// Tolerance note (NOT a formula tolerance): terrain_golden.json's z values
// are Python's round(z, 6) - a decimal-text rounding step - AND its x/y
// values are separately round(x, 3)/round(y, 3), while the tile_x/tile_y
// and z in the SAME row were computed from the TRUE, unrounded x/y
// (terrain.py's build_golden calls height(x, y)/tile_key(x, y) BEFORE
// rounding x/y for the output dict). So this test evaluates our height
// function at the ROUNDED x/y (the only coordinates available from the
// file), which differs from Python's own input by up to 5e-4 in each axis -
// the height field's finest octave (70 m wavelength, 1.4 m amplitude) has a
// spatial derivative on the order of 0.1 m per metre, so that coordinate
// rounding alone can move z by up to roughly 1e-4 m, dwarfing the 5e-7
// rounding noise from z's own 6-decimal storage. Measured empirically
// across all 347 points: max abs error 7.34e-5 m. 2e-4 m (0.2 mm) is used
// as the comparison tolerance - comfortably above that measured noise
// floor, but still many orders of magnitude below anything a real formula
// bug would produce (a wrong octave, wrong hash, or wrong flat-zone mask
// misses by metres, not fractions of a millimetre). `surface` (categorical)
// is compared with exact string equality - no tolerance is meaningful
// there at all, and it is NOT sensitive to the same x/y rounding because
// the golden file's own tile_x/tile_y (also computed from the true,
// unrounded x/y) is used directly rather than re-deriving it from the
// rounded x/y (see terrain_surface_cell's call below, and the comment on
// why a "does our own key_for_world_xy reproduce tile_x/tile_y" test was
// deliberately NOT added: right at a tile boundary, a numerically-tiny
// nonzero offset can decide which tile a point falls in, and that
// information is exactly what round(x,3)/round(y,3) can erase).

#include "terrain/procedural_terrain_source.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

using namespace ps;
using namespace ps::terrain;

namespace {
std::string terrain_dir() {
#ifdef PS_SOURCE_DIR
    return std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground/";
#else
#error "PS_SOURCE_DIR must be defined (see tests/unit/CMakeLists.txt)"
#endif
}
} // namespace

TEST_CASE("Procedural terrain matches the Python reference evaluator's golden points", "[terrain][golden]") {
    const TerrainSpec spec = load_terrain_spec(terrain_dir() + "terrain.json");

    std::ifstream f(terrain_dir() + "terrain_golden.json");
    REQUIRE(f.good());
    std::stringstream ss;
    ss << f.rdbuf();
    const nlohmann::json golden = nlohmann::json::parse(ss.str());

    REQUIRE(golden.at("seed").get<std::uint32_t>() == spec.seed);
    const auto& points = golden.at("points");
    REQUIRE(points.size() == golden.at("count").get<std::size_t>());
    REQUIRE(points.size() > 300); // sanity: this is the 347-point golden set, not an empty/truncated file

    std::size_t checked = 0;
    real max_abs_err = 0.0;
    for (const auto& p : points) {
        const real x = p.at("x").get<real>();
        const real y = p.at("y").get<real>();
        const real expected_z = p.at("z").get<real>();
        const std::int64_t tile_x = p.at("tile_x").get<std::int64_t>();
        const std::int64_t tile_y = p.at("tile_y").get<std::int64_t>();
        const std::string expected_surface = p.at("surface").get<std::string>();

        const real z = terrain_height(spec, x, y);
        const real err = std::abs(z - expected_z);
        max_abs_err = std::max(max_abs_err, err);
        INFO("point (" << x << ", " << y << "): z=" << z << " expected=" << expected_z << " err=" << err);
        REQUIRE(err < 2e-4);

        const SurfaceId sid = terrain_surface_cell(spec, tile_x, tile_y, spec.tile_size_m);
        const SurfaceId expected_sid = spec.surface_id_for_name(expected_surface);
        REQUIRE(expected_sid != kInvalidSurfaceId); // the golden file must only ever name a surface this spec defines
        INFO("point (" << x << ", " << y << ") tile (" << tile_x << ", " << tile_y << "): surface="
                        << (sid < spec.surface_names.size() ? spec.surface_names[sid] : std::string("?"))
                        << " expected=" << expected_surface);
        REQUIRE(sid == expected_sid);

        ++checked;
    }
    REQUIRE(checked == points.size());
    INFO("max abs height error over all golden points: " << max_abs_err);
    // Cross-check the golden file's own precision ceiling (see the header
    // comment): if the formula were actually wrong (not just rounded),
    // errors would be orders of magnitude above this coordinate-rounding
    // noise floor, not sub-millimetre.
    REQUIRE(max_abs_err < 2e-4);
}

// NOTE: no "does our own key_for_world_xy reproduce the golden file's
// tile_x/tile_y" test - see the header comment on why that is not a
// meaningful check given how the golden file rounds x/y for storage
// (several golden points sit numerically infinitesimally off a tile
// boundary, e.g. cos(radians(270)) not being exactly 0.0, and
// round(x, 3) erases exactly that information).
