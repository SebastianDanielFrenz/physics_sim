// ps_audio/source.h - source list and calibration (ps_audio A8, out/audio_plan.md
// section 8): what each of a Voice's rendered channels IS (an exhaust outlet
// or the intake mouth), where it sits in the vehicle frame, and the
// calibration of the render path's float samples.
//
// API-compatibility note (A8): section 8 literally describes
// `Voice::render(frames, SourceBlock* out)`. Voice::render() keeps its
// existing A7 interleaved-float signature instead (out[f * source_count() +
// s], Pa at 1 m) - it is the signature Runner (A7) and A10's EngineRig
// already build on, and changing it would not be source-compatible. This
// file adds the SourceBlock shape and Calibration/SourceInfo ADDITIVELY on
// top of Voice's existing render()/source()/source_count(): voice_sources()
// turns Voice::source(k)'s ps::engine_cycle::RadiatingEnd into the spec's
// SourceInfo, and render_to_source_blocks() is a convenience wrapper that
// renders through the existing render() and re-packs the result into
// per-source SourceBlocks, for a caller that wants that exact shape (e.g. an
// offline multi-track exporter) without hand-deinterleaving.
//
// Deterministic, allocation-free (render_to_source_blocks; voice_sources
// itself allocates its returned vector, a one-time construction-time call by
// contract - see its own comment).
#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps_audio/voice.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps_audio {

using ps::real;

enum class SourceKind : std::uint8_t { ExhaustOutlet, IntakeMouth };

// PROVISIONAL (A8): omnidirectional only - out/audio_plan.md section 8
// mentions "an optional directivity hint" without specifying a pattern. A
// future increment may add a horn/cone pattern per source; every consumer
// must treat an unrecognised kind as Omni.
enum class Directivity : std::uint8_t { Omni };

struct SourceInfo {
    SourceKind kind = SourceKind::ExhaustOutlet;
    std::string name;                        // ps::engine_cycle::RadiatingEnd::name
    ps::Vec3 position_m = ps::Vec3::zero();  // vehicle frame (Vec3::zero() with no installation)
    Directivity directivity = Directivity::Omni;
};

// Calibration of Voice::render()'s / render_to_source_blocks()'s float
// samples: Pa at 1 m, full scale (a sample magnitude of 1.0) =
// `pa_full_scale` Pa at 1 m. Default 200 Pa at 1 m ~ 140 dB SPL [A]
// (out/audio_plan.md section 8's own figure). There is no AGC and no
// leveler anywhere in ps_audio (section 8) - a caller applying its own
// gain staging or a limiter reads this value first, once, at setup; nothing
// in ps_audio consults it at render time.
struct Calibration {
    real pa_full_scale = 200.0;
};

// One entry per Voice::source(k), same order/index (kind = IntakeMouth iff
// that RadiatingEnd::intake is set). Called once at setup (constructs a
// vector); not on the render hot path.
[[nodiscard]] std::vector<SourceInfo> voice_sources(const Voice& voice);

// audio_plan.md section 8's literal SourceBlock shape: source_id =
// Voice::source(k)'s index, kind from voice_sources(), samples Pa at 1 m
// (owned by the caller's `deinterleaved[source_id]` buffer, valid until the
// next render_to_source_blocks/render call).
struct SourceBlock {
    int source_id = 0;
    SourceKind kind = SourceKind::ExhaustOutlet;
    const float* samples = nullptr;  // `frames` entries
};

// Renders `frames` samples of every source through Voice::render()
// (interleaved into the caller-owned `interleaved` buffer, >= frames *
// voice.source_count() floats), de-interleaves each source into the
// caller-owned `deinterleaved[s]` buffer (>= frames floats each,
// source_count() of them), and fills the caller-owned `out` array
// (source_count() entries) with one SourceBlock per source pointing into
// `deinterleaved`. Allocation-free; consumer-thread contract identical to
// Voice::render()'s own.
void render_to_source_blocks(Voice& voice, int frames, float* interleaved, float* const* deinterleaved,
                             SourceBlock* out);

}  // namespace ps_audio
