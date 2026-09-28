// ps_audio/source.cpp - see source.h.
#include "ps_audio/source.h"

#include "ps/engine_cycle/cycle_engine.h"

namespace ps_audio {

std::vector<SourceInfo> voice_sources(const Voice& voice) {
    std::vector<SourceInfo> out;
    const int n = voice.source_count();
    out.reserve(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        const ps::engine_cycle::RadiatingEnd& r = voice.source(k);
        SourceInfo info;
        info.kind = r.intake ? SourceKind::IntakeMouth : SourceKind::ExhaustOutlet;
        info.name = r.name;
        info.position_m = r.position_m;
        info.directivity = Directivity::Omni;
        out.push_back(info);
    }
    return out;
}

void render_to_source_blocks(Voice& voice, int frames, float* interleaved, float* const* deinterleaved,
                             SourceBlock* out) {
    const int n = voice.source_count();
    voice.render(interleaved, frames);
    for (int s = 0; s < n; ++s) {
        float* d = deinterleaved[static_cast<std::size_t>(s)];
        for (int f = 0; f < frames; ++f) d[f] = interleaved[static_cast<std::size_t>(f) * static_cast<std::size_t>(n) + static_cast<std::size_t>(s)];
        const ps::engine_cycle::RadiatingEnd& r = voice.source(s);
        SourceBlock& b = out[static_cast<std::size_t>(s)];
        b.source_id = s;
        b.kind = r.intake ? SourceKind::IntakeMouth : SourceKind::ExhaustOutlet;
        b.samples = d;
    }
}

}  // namespace ps_audio
