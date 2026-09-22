// core/src/articulation/gravity.h — gravity as a function of world
// position (internal).
//
// Mid-phase design change (Sebastian, via the orchestrator, after PLAN.md
// section 6 grew an IWorldShape bullet for the optional P11 curved-Earth
// world): do NOT bake gravity in as Featherstone's usual a_0 = -g "base
// acceleration" trick (ch. 3) - that trick only works because a uniform
// field is exactly equivalent to accelerating the whole tree's reference
// frame, which stops being true once gravity direction varies with
// position (a spherical planet's gravity points toward its centre, so two
// links far apart on a big machine would feel measurably different
// directions). Instead, gravity is applied per link as an ordinary
// external spatial force m_i * g(p_i) at that link's own COM, exactly
// like any other external wrench (ps::Wrench from a force element) - see
// instance.cpp's per-body ext_force assembly, which is the only caller of
// GravityField::at(). ABA/RNEA/CRBA themselves are untouched: they always
// took a fully general per-body ext_force, so there was nothing about
// their own math that needed to change, only what Instance puts into that
// array before calling them (their own `gravity` input field - the old
// a_0 trick - is simply not used by Instance any more, see
// instance.cpp; it stays in aba.h/rnea.h for now since
// tests/unit/test_articulation_core.cpp's ABA-vs-CRBA cross-check still
// exercises that code path directly, which is itself still correct - see
// that file - just no longer how gravity reaches the production stepping
// path).
//
// For now (P1.5a; P11 is optional/future and not otherwise touched) only
// a constant field is implemented - returns the same vector regardless of
// world position, i.e. numerically identical behaviour to the old a_0
// trick for every scenario in this phase (PLAN.md's instruction: "Make
// sure the energy and period tests still use the same constant g, so
// results stay comparable" - re-ran the full P1.5a validation suite after
// this change with no regressions, see the P1.5a report).

#pragma once

#include "ps/math/vec3.h"

namespace ps::articulation {

struct GravityField {
    Vec3 constant{0.0, 0.0, -9.81};

    [[nodiscard]] Vec3 at(const Vec3& /*world_position*/) const { return constant; }
};

} // namespace ps::articulation
