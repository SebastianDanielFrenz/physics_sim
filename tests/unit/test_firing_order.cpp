// tests/unit/test_firing_order.cpp — ps_audio A1 (out/audio_plan.md section 1):
// ps::engine_cycle::validate_firing_order / validate_firing_angles /
// resolve_firing_angles_deg (ps/engine_cycle/firing_order.h). Pure-function
// coverage only - no CycleModelDesc/loader involved (core/src/io/vehicle_io.cpp's
// read_cycle_block loader-rejection coverage lives in
// test_engine_map_generator.cpp's "C16a M5" cycle-block-rejections table, next
// to every other cycle-block key, plus a dedicated firing/banks section there).
//
// Every fixture firing_order/firing_angles_deg below is a TEST-ONLY, ASSUMED
// illustrative pattern (never a citation of a specific real engine's actual
// crank-pin geometry) - the pure derivation logic is what's under test here,
// not any real engine's phasing.

#include "ps/engine_cycle/firing_order.h"
#include "ps/types.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace ps;
namespace ec = ps::engine_cycle;
using Catch::Matchers::ContainsSubstring;

namespace {
bool close(real a, real b, real tol = 1e-9) { return std::abs(a - b) <= tol; }
}  // namespace

TEST_CASE("A1 firing_order: inline-4 default (1-3-4-2) even spacing", "[cycle][io]") {
    const std::vector<int> order = {1, 3, 4, 2};
    ec::validate_firing_order(order, 4);
    const std::vector<real> phi = ec::resolve_firing_angles_deg(order, {}, 4);
    REQUIRE(phi.size() == 4);
    // index 0 = cylinder 1 ... index 3 = cylinder 4
    CHECK(close(phi[0], 0.0));    // cylinder 1 fires first
    CHECK(close(phi[2], 180.0));  // cylinder 3 fires second
    CHECK(close(phi[3], 360.0));  // cylinder 4 fires third
    CHECK(close(phi[1], 540.0));  // cylinder 2 fires last

    // Sabotage: swap the last two entries (a plausible transcription slip,
    // 1-3-2-4 instead of 1-3-4-2) - the derived phasing must differ, proving
    // this test actually discriminates the correct order from a plausible
    // wrong one, not just from garbage input.
    const std::vector<int> sabotaged_order = {1, 3, 2, 4};
    const std::vector<real> sabotaged_phi = ec::resolve_firing_angles_deg(sabotaged_order, {}, 4);
    CHECK_FALSE(close(sabotaged_phi[1], phi[1]));
    CHECK_FALSE(close(sabotaged_phi[3], phi[3]));
}

TEST_CASE("A1 firing_order: cross-plane V8 (ASSUMED test fixture)", "[cycle][io]") {
    // LS-style cross-plane firing order, cylinders 1-8. TEST-ONLY ASSUMED
    // fixture, not a citation of a specific production engine.
    const std::vector<int> order = {1, 8, 7, 2, 6, 5, 4, 3};
    ec::validate_firing_order(order, 8);
    const std::vector<real> phi = ec::resolve_firing_angles_deg(order, {}, 8);
    REQUIRE(phi.size() == 8);
    constexpr real kStep = 90.0;  // 720 / 8
    for (std::size_t i = 0; i < order.size(); ++i) {
        const int cylinder = order[i];
        CHECK(close(phi[static_cast<std::size_t>(cylinder - 1)], real(i) * kStep));
    }

    // Sabotage: an off-by-one in the order (swap positions 4 and 5, i.e. the
    // 6th and 5th cylinders to fire) moves those two cylinders' derived
    // angles - the test must catch this, not just accept anything with the
    // right multiset of cylinder numbers.
    std::vector<int> sabotaged = order;
    std::swap(sabotaged[4], sabotaged[5]);
    const std::vector<real> sabotaged_phi = ec::resolve_firing_angles_deg(sabotaged, {}, 8);
    CHECK_FALSE(close(sabotaged_phi[5], phi[5]));  // cylinder 6 (index 5)
    CHECK_FALSE(close(sabotaged_phi[4], phi[4]));  // cylinder 5 (index 4)
}

TEST_CASE("A1 firing_order: flat-plane V8 (ASSUMED test fixture)", "[cycle][io]") {
    // A flat-plane-style firing order, cylinders 1-8 - a different
    // permutation from the cross-plane case above, still even 720/8 spacing
    // (a flat-plane V8's distinguishing trait is crank geometry/exhaust
    // routing, not the firing *interval* itself - out/audio_plan.md leaves
    // per-bank exhaust routing to a later increment, not this one). TEST-ONLY
    // ASSUMED fixture, not a citation of a specific production engine.
    const std::vector<int> order = {1, 5, 4, 8, 6, 3, 7, 2};
    ec::validate_firing_order(order, 8);
    const std::vector<real> phi = ec::resolve_firing_angles_deg(order, {}, 8);
    REQUIRE(phi.size() == 8);
    constexpr real kStep = 90.0;
    for (std::size_t i = 0; i < order.size(); ++i) {
        const int cylinder = order[i];
        CHECK(close(phi[static_cast<std::size_t>(cylinder - 1)], real(i) * kStep));
    }

    // Sabotage: an off-by-one (swap the first two positions) moves cylinder
    // 1 off phi=0, which a real engine's ignition timing absolutely depends
    // on getting right.
    std::vector<int> sabotaged = order;
    std::swap(sabotaged[0], sabotaged[1]);
    const std::vector<real> sabotaged_phi = ec::resolve_firing_angles_deg(sabotaged, {}, 8);
    CHECK_FALSE(close(sabotaged_phi[0], phi[0]));
}

TEST_CASE("A1 firing_order: explicit odd-fire angles (V-twin style, ASSUMED)", "[cycle][io]") {
    // A 45-degree V-twin-style odd-fire pattern: firing intervals 315/405 deg
    // instead of an even 360/360 split - the canonical case no firing_order-
    // derived even spacing can represent, requiring firing_angles_deg.
    // TEST-ONLY ASSUMED fixture, not a citation of a specific engine.
    const std::vector<int> order = {1, 2};
    const std::vector<real> explicit_angles = {0.0, 315.0};  // cylinder 1 @ 0, cylinder 2 @ 315
    ec::validate_firing_order(order, 2);
    ec::validate_firing_angles(explicit_angles, 2);
    const std::vector<real> phi = ec::resolve_firing_angles_deg(order, explicit_angles, 2);
    REQUIRE(phi.size() == 2);
    CHECK(close(phi[0], 0.0));
    CHECK(close(phi[1], 315.0));
    // An EMPTY firing_angles_deg would instead derive even 360/360 spacing -
    // confirm the explicit value actually took precedence (the odd-fire
    // interval is preserved), not silently falling back to the default.
    CHECK_FALSE(close(phi[1], 360.0));

    // Sabotage: swapping firing_order (1,2) -> (2,1) must NOT change the
    // result here - resolve_firing_angles_deg returns firing_angles_deg
    // as-is, indexed by cylinder number, once it is non-empty (this file's
    // own DESIGN NOTE). A caller wrongly re-deriving from firing_order once
    // explicit angles are given would fail this check.
    const std::vector<int> sabotaged_order = {2, 1};
    const std::vector<real> phi2 = ec::resolve_firing_angles_deg(sabotaged_order, explicit_angles, 2);
    CHECK(close(phi2[0], phi[0]));
    CHECK(close(phi2[1], phi[1]));
}

TEST_CASE("A1 validate_firing_order: rejections", "[cycle][io]") {
    SECTION("wrong length") {
        CHECK_THROWS_WITH(ec::validate_firing_order({1, 3, 4}, 4), ContainsSubstring("expected 4 entries, got 3"));
    }
    SECTION("out of range (too high)") {
        CHECK_THROWS_WITH(ec::validate_firing_order({1, 3, 4, 5}, 4), ContainsSubstring("out of range [1, 4]"));
    }
    SECTION("out of range (zero)") {
        CHECK_THROWS_WITH(ec::validate_firing_order({0, 3, 4, 2}, 4), ContainsSubstring("out of range [1, 4]"));
    }
    SECTION("duplicate cylinder") {
        CHECK_THROWS_WITH(ec::validate_firing_order({1, 3, 4, 3}, 4), ContainsSubstring("cylinder 3 appears more than once"));
    }
    SECTION("a valid permutation does not throw") { CHECK_NOTHROW(ec::validate_firing_order({1, 3, 4, 2}, 4)); }
}

TEST_CASE("A1 validate_firing_angles: rejections", "[cycle][io]") {
    SECTION("empty is always valid (derive the default)") { CHECK_NOTHROW(ec::validate_firing_angles({}, 4)); }
    SECTION("wrong length") {
        CHECK_THROWS_WITH(ec::validate_firing_angles({0.0, 180.0}, 4),
                          ContainsSubstring("expected 4 entries (one per cylinder) or none, got 2"));
    }
    SECTION("out of range (negative)") {
        CHECK_THROWS_WITH(ec::validate_firing_angles({-1.0, 180.0, 360.0, 540.0}, 4),
                          ContainsSubstring("must be finite and in [0, 720)"));
    }
    SECTION("out of range (>= 720)") {
        CHECK_THROWS_WITH(ec::validate_firing_angles({0.0, 180.0, 360.0, 720.0}, 4),
                          ContainsSubstring("must be finite and in [0, 720)"));
    }
    SECTION("non-finite") {
        CHECK_THROWS_WITH(ec::validate_firing_angles({0.0, 180.0, std::numeric_limits<real>::quiet_NaN(), 540.0}, 4),
                          ContainsSubstring("must be finite and in [0, 720)"));
    }
    SECTION("duplicate angle") {
        CHECK_THROWS_WITH(ec::validate_firing_angles({0.0, 180.0, 180.0, 540.0}, 4),
                          ContainsSubstring("share the same firing angle"));
    }
    SECTION("a valid explicit set does not throw") { CHECK_NOTHROW(ec::validate_firing_angles({0.0, 180.0, 360.0, 540.0}, 4)); }
}
