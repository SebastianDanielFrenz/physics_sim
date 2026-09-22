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

    // Called once per 960 Hz substep, from inside Instance::step_tick,
    // right after that substep's forward kinematics have been refreshed
    // (so inst.link_world_pose/link_world_motion reflect the CURRENT
    // substep's state, not the state at the start of the 240 Hz tick).
    // out_wrench is sized model_.bodies.size() and pre-zeroed by the
    // caller; implementations add (+=) a world-frame wrench about each
    // affected body's own origin into out_wrench[body_index] - never
    // resize or otherwise reallocate it (PLAN.md D8: this runs inside a
    // job). substep_dt is this substep's own duration (needed for an
    // energy-safety clamp on the normal impulse - see contact.cpp); tick_dt
    // is the full 240 Hz tick's duration, needed to correctly time-average
    // any reaction wrench applied directly to another backend body via
    // IRigidBackend::apply_wrench (which accumulates once per
    // backend.step(), not once per substep).
    virtual void evaluate_substep_contacts(const Instance& inst, IRigidBackend& backend, real substep_dt, real tick_dt,
                                            std::vector<Wrench>& out_wrench) = 0;
};

} // namespace ps::articulation
