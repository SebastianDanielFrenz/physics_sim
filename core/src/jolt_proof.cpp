// Proves that ps_core actually links against JoltPhysics and can run its
// minimal startup sequence. This is the only translation unit in Phase 0
// that includes Jolt headers; ps/version.h (the public API) never does —
// see PLAN.md D2 ("no Jolt type appears in any public header").
//
// Real backend wiring (IRigidBackend, world stepping, ...) is P1 work; this
// is a placeholder self-check only.

#include "ps/version.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

namespace ps {

bool jolt_backend_self_check() {
    JPH::RegisterDefaultAllocator();

    JPH::Factory* factory = new JPH::Factory();
    JPH::Factory* previous = JPH::Factory::sInstance;
    JPH::Factory::sInstance = factory;

    JPH::RegisterTypes();
    const bool registered = JPH::Factory::sInstance != nullptr;

    JPH::UnregisterTypes();
    JPH::Factory::sInstance = previous;
    delete factory;

    return registered;
}

} // namespace ps
