// core/src/articulation/contact.h — articulation/Jolt contact coupling
// (internal; PLAN.md P1.5b).
//
// One ContactCoupling per articulation instance whose ArticulationDesc
// requested a contact_strategy != None. Owned by World (world.cpp),
// constructed right after the matching articulation::Instance so it can
// read the link's real initial world pose (needed to place a ProxyBody's
// Jolt body correctly on the very first tick).
//
// Both strategies are measured, side by side, in tests/validation's P1.5b
// scenarios and tests/bench/bench_contact_coupling.cpp; see contact.cpp's
// header comment for the exact force law (Compliant) and its sources.

#pragma once

#include "contact_source.h"
#include "instance.h"
#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/world/ids.h"

#include <vector>

namespace ps::articulation {

class System; // system.h; only System::accumulate_external_wrench is used here

// Compliant implements IContactSubstepSource itself (Instance calls it
// directly at substep rate, contact_source.h); ProxyBody does not need
// that seam at all (it only touches World-tick-rate hooks below).
class ContactCoupling : public IContactSubstepSource {
public:
    // desc must be the ArticulationDesc World::create_articulation built
    // `inst` from; inst must already be that articulation's freshly
    // constructed Instance (its constructor has run recompute_world_kinematics(),
    // so link_world_pose/link_world_motion are valid) - this articulation's
    // ArticulationId is not needed until pre_backend_step/post_backend_step
    // below (it addresses the *other* per-tick state, not construction).
    // backend outlives this object (World destroys articulation_contacts_
    // before its own backend_ member - see world.h's member order comment).
    ContactCoupling(const ArticulationDesc& desc, const Instance& inst, IRigidBackend& backend);
    ~ContactCoupling();
    ContactCoupling(const ContactCoupling&) = delete;
    ContactCoupling& operator=(const ContactCoupling&) = delete;

    [[nodiscard]] bool active() const { return strategy_ != ContactStrategy::None; }

    // ProxyBody only (Compliant is handled entirely by
    // evaluate_substep_contacts below, called from inside Instance::step_tick
    // at substep rate - contact_source.h). Called from World's
    // "contact_coupling_pre" stage, between stage_gather and backend_step:
    // pose/velocity-syncs every contact-enabled link's proxy Jolt body from
    // the link's current (post-substep) state, so backend_step below
    // solves the contact against it. A no-op for Compliant/None.
    void pre_backend_step(const Instance& inst, IRigidBackend& backend);

    // ProxyBody only. World's "contact_coupling_post" stage, right after
    // backend_step: reads back each proxy's velocity change, converts it to
    // an impulse/dt wrench, subtracts the proxy's own known gravity
    // contribution (see the constructor's gravity_enabled comment and
    // contact.cpp's header comment for why this subtraction is required),
    // and accumulates the remainder as the link's external wrench for NEXT
    // tick (same seam as every other force element). A no-op for
    // Compliant/None. `gravity` is World's own uniform gravity vector
    // (WorldConfig::gravity) - the same one the proxy body itself falls
    // under.
    void post_backend_step(System& sys, ArticulationId id, IRigidBackend& backend, real dt, const Vec3& gravity);

    // IContactSubstepSource (Compliant only - contact_source.h's header
    // comment on why this runs at substep, not tick, rate). A no-op for
    // ProxyBody/None (never actually called for them: Instance only calls
    // this when has_contact_source() is true, which only Compliant sets -
    // see the constructor). See contact_source.h for apply_reaction_to_other.
    void evaluate_substep_contacts(const Instance& inst, IRigidBackend& backend, real substep_dt, real tick_dt, std::vector<Wrench>& out_wrench,
                                    bool apply_reaction_to_other) override;

private:
    struct LinkContact {
        int link_index = -1;
        ShapeDesc shape;
        Pose shape_offset;
        ContactMaterial material;
        real link_mass = 1.0;                        // Compliant only: effective mass for the impulse safety clamp (contact.cpp)
        BodyId proxy_body;                          // ProxyBody only
        real proxy_mass = 0.0;                       // ProxyBody only
        Motion last_commanded_motion;                // ProxyBody only: what pre_backend_step commanded, for post's delta-v
        std::vector<ShapeContactHit> hits_scratch;    // Compliant only; reused every tick (PLAN.md D8: no per-tick allocation)
    };

    ContactStrategy strategy_ = ContactStrategy::None;
    std::vector<LinkContact> links_;
    IRigidBackend& backend_; // for ~ContactCoupling to destroy any ProxyBody bodies
};

} // namespace ps::articulation
