// White-box test for core/src/articulation/model.cpp's Hinge-closure guard
// (PLAN.md P1.5a / vault PHYS-009 Step 1 rule 5): LoopClosureType::Hinge
// has no axis-alignment rows implemented yet (see loop_closure.h's file
// comment), so building an articulation with one must fail loudly instead
// of silently behaving like a Point closure.

#include "articulation/model.h"

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

using namespace ps;
using namespace ps::articulation;

namespace {

// A minimal single-revolute-link articulation with one loop closure back
// to the world, of the given type - just enough to reach build_model's
// loop-closure loop.
ArticulationDesc make_desc_with_closure(LoopClosureType type) {
    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose::identity();

    LinkDesc link;
    link.parent = -1;
    link.joint = JointType::Revolute;
    link.axis = Vec3::unit_y();
    link.mass = 1.0;
    link.com = Vec3{0, 0, -0.5};
    link.inertia = Mat3::diagonal(0.1, 0.1, 0.1);
    desc.links.push_back(link);

    LoopClosureDesc lc;
    lc.type = type;
    lc.link_a = 0;
    lc.anchor_a = Vec3{0, 0, -1.0};
    lc.link_b = -1; // world
    lc.anchor_b = Vec3{0, 0, -1.0};
    if (type == LoopClosureType::Hinge) {
        lc.axis_a = Vec3::unit_y();
        lc.axis_b = Vec3::unit_y();
    }
    desc.loop_closures.push_back(lc);
    return desc;
}

} // namespace

TEST_CASE("build_model rejects a Hinge loop closure", "[articulation][loop_closure]") {
    CHECK_THROWS_AS(build_model(make_desc_with_closure(LoopClosureType::Hinge)), std::invalid_argument);
}

TEST_CASE("build_model still accepts a Point loop closure", "[articulation][loop_closure]") {
    CHECK_NOTHROW(build_model(make_desc_with_closure(LoopClosureType::Point)));
}
