// Unit test: the articulation JSON scenario schema (PLAN.md D10/P1.5a
// scope item 9, "A JSON scenario for an articulation in data/scenarios,
// runnable by ps_run") loads, builds and steps without throwing. No
// existing test exercised ps::io::load_scenario/build_scenario at all
// before this (the pre-existing free_fall/pendulum/joint_chain scenario
// files were only ever run by hand via ps_run) - this covers the new
// articulation JSON path and, incidentally, the general scenario loader
// for the first time in CI.

#include "ps/io/scenario.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>

using namespace ps;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

TEST_CASE("data/scenarios/articulation_chain.json loads, builds and steps", "[unit][io][articulation]") {
    // PS_SOURCE_DIR (tests/unit/CMakeLists.txt, a compile definition) is
    // the repo root as an absolute path baked in at configure time -
    // walking up from the CTest working directory does NOT work here: the
    // WSL/Linux presets build under ~/build/physics_sim (PLAN.md section
    // 2, deliberately outside /mnt/s for speed), a wholly separate
    // directory tree from the source checkout, so no number of ".." steps
    // from the build tree ever reaches data/scenarios on those legs -
    // found by actually running the Linux CI legs (they failed with this
    // test unable to find the file), not by inspection.
    const std::filesystem::path found = std::filesystem::path(PS_SOURCE_DIR) / "data" / "scenarios" / "articulation_chain.json";
    REQUIRE(std::filesystem::exists(found));

    const io::Scenario scenario = io::load_scenario(found.string());
    REQUIRE(scenario.articulations.size() == 1);
    REQUIRE(scenario.articulations[0].desc.links.size() == 3);

    io::BuiltScenario built = io::build_scenario(scenario);
    REQUIRE(built.articulations_by_name.count("chain") == 1);

    const ArticulationId art = built.articulations_by_name.at("chain");
    for (int i = 0; i < 240; ++i) built.world->step();

    const real q0 = built.world->joint_position(art, 0);
    CHECK(std::isfinite(q0));
    CHECK(built.world->state_hash() != 0);
}
