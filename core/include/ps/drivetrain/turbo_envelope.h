// ps/drivetrain/turbo_envelope.h - Q5: a turbo engine's full-load envelope built OFF the thread that swaps the turbo.
//
// World::replace_turbo_configuration(id, component, desc) rebuilds the engine's full-load envelope (a ~30-point held-crank
// dyno run, 0.6 s for the hypercar on a memo miss) on the calling thread. To keep that out of a frame:
//   1. caller thread, between steps:  PreparedTurboEnvelope p = world.prepare_turbo_envelope(id, component, desc);
//      (cheap: copies the descriptor and holds the engine's current maps)
//   2. any thread outside a World tick job (a loader thread, a background-priority job; allocates, no World access):
//      build_turbo_envelope(p);
//   3. caller thread, between steps:  world.replace_turbo_configuration(id, component, p);
// The envelope is the same pure function of (maps, turbo hardware, ECU) that the synchronous path calls, through the same
// process-wide memo, so the result is bit-identical. If the engine's maps were replaced (World::replace_engine_maps applied)
// between step 1 and step 3, or p was never built, step 3 falls back to the synchronous build - always correct, just slow.
#pragma once

#include "ps/drivetrain/powertrain_desc.h"

#include <memory>

namespace ps::drivetrain {

struct FullLoadEnvelope;
struct SimulatedEngineMaps;

struct PreparedTurboEnvelope {
    TurboPairDesc turbo;                               // the hardware that will be installed
    std::shared_ptr<const SimulatedEngineDesc> desc;   // the engine with `turbo`; null: this engine builds no envelope
    std::shared_ptr<const SimulatedEngineMaps> maps;   // the maps the envelope is built for
    std::shared_ptr<const FullLoadEnvelope> envelope;  // null until built, and null after for engines without one
    bool built = false;
};

// Runs the (memoised) dyno build; thread-safe against other builds and the live World. Idempotent.
void build_turbo_envelope(PreparedTurboEnvelope& p);

} // namespace ps::drivetrain
