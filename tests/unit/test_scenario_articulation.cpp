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

TEST_CASE("data/scenarios/articulation_chain.json loads, builds and steps", "[unit][io][articulation]") {
    // Resolve relative to this test binary's usual CTest working directory
    // (the build tree) up to the repo root - CTest runs each executable
    // with the build directory as CWD, so walk up looking for data/scenarios.
    std::filesystem::path p = std::filesystem::current_path();
    std::filesystem::path found;
    for (int i = 0; i < 8 && found.empty(); ++i) {
        const auto candidate = p / "data" / "scenarios" / "articulation_chain.json";
        if (std::filesystem::exists(candidate)) found = candidate;
        p = p.parent_path();
    }
    REQUIRE_FALSE(found.empty());

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
