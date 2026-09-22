// core/src/articulation/contact_source.h — the seam Instance uses to ask
// for stiff-contact forces at SUBSTEP rate rather than once per 240 Hz
// tick (internal; PLAN.md P1.5b).
//
// Why this exists (found empirically, not designed up front): P1.5b's first
// cut computed the Compliant strategy's Hunt-Crossley contact force once
// per 240 Hz tick, in a World pipeline stage, exactly like every other
// external wrench (D3's "held constant across substeps" convention). That
// blew up (a 1 kg mass on a vertical rail, default material - contact.cpp's
// header comment - shot up to q=+36 instead of settling near q=-4.7): the
// contact's natural frequency is sqrt(k/m) =~ 1000 rad/s (159 Hz) with the
// default stiffness, and sampling/holding a spring that stiff at 240 Hz
// (1.5 samples per period) is a textbook-undersampled, unstable control
// loop, regardless of how good the ODE integrator driving the rest of the
// articulation is. Recomputing it once per 960 Hz substep instead (still
// held constant across one substep's RK4 stages only, the same size
// approximation D3 already makes for every other force at the coarser 240
// Hz grain) fixed it - see tests/unit/test_articulation_contact.cpp.
//
// A plain virtual interface + raw pointer, set once at construction
// (Instance::set_contact_source), not a std::function rebuilt per tick:
// PLAN.md D8 requires zero allocation inside a job, and step_tick runs
// inside one (System::step_all) - a std::function whose captured state
// does not fit a standard library's small-buffer optimisation allocates on
// every assignment, which a per-tick rebuild would do; a pointer set once
// at setup time never does.

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/math/pose.h"

#include <vector>

namespace ps::articulation {

class Instance;

class IContactSubstepSource {
public:
    virtual ~IContactSubstepSource() = default;

    // Called once per RK4 STAGE (up to 4x per 960 Hz substep - see below),
    // from inside Instance's stage evaluation, right after the caller has
    // pointed inst.link_world_pose/link_world_motion at THAT STAGE's own
    // (possibly mid-substep, RK4-offset) kinematic state. out_wrench is
    // sized model_.bodies.size() and pre-zeroed by the caller; implementations
    // add (+=) a world-frame wrench about each affected body's own origin
    // into out_wrench[body_index] - never resize or otherwise reallocate it
    // (PLAN.md D8: this runs inside a job). substep_dt is this substep's own
    // duration (needed for an energy-safety clamp on the normal impulse -
    // see contact.cpp); tick_dt is the full 240 Hz tick's duration, needed to
    // correctly time-average any reaction wrench applied directly to another
    // backend body via IRigidBackend::apply_wrench (which accumulates once
    // per backend.step(), not once per substep).
    //
    // Why per-STAGE, not just per-substep (found empirically - PHYS-011):
    // resampling a stiff Hunt-Crossley contact once per 960 Hz substep and
    // then freezing it across that substep's 4 RK4 stages is the exact same
    // "stiff force sampled/held too coarsely" failure the tick-rate ->
    // substep-rate fix above already fixed once, just one level down: a
    // fast impact can still move deep into (and, physically, most of the
    // way back out of) the compliant region within a single 960 Hz substep,
    // and applying the START-of-substep force at FULL strength for the
    // entire substep duration - rather than the true, much shorter,
    // still-penetrating fraction of it - manufactures energy out of
    // nowhere. Recomputing the force at each RK4 stage's own offset state
    // makes it track the within-substep evolution of the penetration
    // instead of freezing it, which is what a smooth ODE right-hand side is
    // supposed to look like to RK4 in the first place.
    //
    // apply_reaction_to_other: true only for the k1 (substep-start) stage,
    // false for k2/k3/k4. Implementations must gate any direct
    // IRigidBackend::apply_wrench call to the OTHER (Jolt) body on this
    // flag: that call already accounts for "once per substep" via its own
    // substep_dt/tick_dt scaling (see contact.cpp) - calling it from all 4
    // RK4 stages instead of once would apply 4x the intended reaction to
    // the other body every substep. The LINK's own contact force (the
    // return via out_wrench) has no such restriction and is always
    // evaluated fresh at every stage.
    virtual void evaluate_substep_contacts(const Instance& inst, IRigidBackend& backend, real substep_dt, real tick_dt,
                                            std::vector<Wrench>& out_wrench, bool apply_reaction_to_other) = 0;
};

} // namespace ps::articulation
