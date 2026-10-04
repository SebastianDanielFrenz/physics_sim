#pragma once
// Engine audio bank generator (tools only, links ps_core + ps_audio): records a
// crank-synchronous PSAB0002 bank (ps_audio/audio_bank.h) from the live voice
// of a simulated cycle engine, cached under <cache_dir>/audio/<key>.psab.
#include "ps_audio/audio_bank.h"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
namespace ps_tools {

// Every generator input that is not in the engine/turbo files: version, rpm
// grid, pedals, settle/warm-up, loop length, crossfade, band-limit, live dx
// scale and plenum settings. Folded into the cache key.
extern const char* const kEngineAudioBankSettings;

struct EngineAudioBankStats {
    bool cache_hit = false;
    double seconds = 0.0;                          // wall time of prepare_engine_audio_bank
    std::size_t bytes = 0;                         // sample payload
    std::vector<double> removed_energy_fraction;   // per plane: energy the anti-alias band-limit removed
};

// FNV-1a over `settings` plus the engine file and every .json it references
// (recursively), plus the turbo configuration file if given.
std::uint64_t engine_audio_bank_key(const std::string& engine_path, const std::string& turbo_configuration_path,
                                    const std::string& settings = kEngineAudioBankSettings);

// Cache hit or generation. Any unreadable cache file (old PSAB0001, other key,
// damaged) is a silent miss and is regenerated. `cancelled` is polled before
// the engine load and between steps of every plane.
ps_audio::AudioBankPtr prepare_engine_audio_bank(const std::string& engine_path, const std::string& turbo_configuration_path,
                                                 const std::string& cache_dir, const std::function<bool()>& cancelled = {},
                                                 EngineAudioBankStats* stats = nullptr);
}  // namespace ps_tools
