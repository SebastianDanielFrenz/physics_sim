// ps_engine_tools/audio_bank_prep.h - engine garage P6 (design A14): bake an
// engine's sound bank (a crank-synchronous PSAB0002 ps_audio::AudioBank) from a
// RESOLVED engine description, with a disk cache, cancellation and progress.
//
// The library links ps_core and ps_audio; neither of those links it. It
// replaces tools/common/engine_bank_generator.* (same bank bytes, see below).
//
// What it bakes. For each of 5 rpm rows x 2 pedal positions (10 planes) the
// simulated engine is held at the row's rpm until its mean-value model has
// settled (1920 substeps at 960 Hz), then the live ps_audio::Voice is run at that
// operating point for a warm-up and a recording, and the recording is turned
// into a whole-cycle loop (ps_audio::build_crank_loop). Every plane is
// independent and deterministic: the bank bytes do not depend on the worker
// count. Heavy: seconds to a minute on a pipe engine (hyper V8 ~27 s [M], 5 of
// the machine's 10 threads); a call belongs on a background thread.
//
// The cache. <cache_dir>/audio/<key as 16 hex digits>.psab. The key is
// computed from the RESOLVED description, not from files: FNV-1a over the
// generator identity (kEngineAudioBankSettings), engine_map_cache_key() of the
// engine's cycle block with its first fuel (every acoustic/geometric input the
// voice reads - pipes, installation acoustics, audio cell sizes, firing, fuel;
// positions and free text are excluded there), and the 10 operating points the
// generator takes from the engine (manifold pressure, charge pressure and
// temperature, lambda, throttle position, spark advance - the only numbers the
// recording reads from the MVEM engine, so an ECU, cam, turbo or map change
// moves the key exactly when it reaches the sound, whichever part file or user
// edit it came from). A free-text field, a limiter or an idle gain does not
// move it. Any unreadable cache file (old format, other key, damaged) is a
// silent miss.
//
// Cancel / progress mirror the engine-map generator (ps/drivetrain/engine_maps.h,
// MapGenerationControl):
//   cancel / cancel_poll: read between work items - before each operating point,
//     before every 200-frame render tick of every plane and once more before the
//     result is published. Once either reads true the call throws
//     ps::drivetrain::GenerationCancelled (the same type the map loaders throw)
//     after the running ticks finished (latency: one tick, ~ms), and NOTHING is
//     written: no cache file, no temp file. The flag is a request: a call that
//     observed no set flag returns normally.
//   on_progress(done, total): once per FINISHED render tick of the recording
//     stage with done = 1, 2, ..., total (strictly rising by 1, ending at total
//     unless cancelled or failed; total = the sum of all planes' ticks, a pure
//     function of the settings, ~5400 [M] for the shipped grid), ALWAYS on the
//     thread that called prepare_engine_audio_bank - workers only bump an atomic
//     counter and the calling thread polls it (a burst of ticks finishing
//     between two polls arrives as consecutive calls). Not called at all for a
//     cache hit. The callback may allocate and be slow (it delays only the
//     reporting) and must not throw.
//   Both cancel members are polled on worker threads; cancel_poll must be
//   thread-safe.
#pragma once
#include "ps/drivetrain/powertrain_desc.h"
#include "ps_audio/audio_bank.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ps_engine_tools {

// Generator identity: every generator constant that is not part of the engine
// (version, rpm grid, pedals, settle/warm-up, loop length, crossfade,
// band-limit, live dx scale, plenum settings). Folded into the cache key.
extern const char* const kEngineAudioBankSettings;

struct AudioBankPrepOptions {
    // Cache directory (the bank lives in <cache_dir>/audio). Empty = no disk access: always
    // generate, store nothing.
    std::string cache_dir;
    // Threads that render planes. 0 = auto (half the hardware threads, at least 1: the game, its
    // sim thread and the live voice keep running while a bake goes on); 1 = the calling thread
    // renders everything (progress is then delivered inline). With more, the calling thread only
    // waits and delivers progress. The result does not depend on this number.
    int workers = 0;
    const std::atomic<bool>* cancel = nullptr;
    std::function<bool()> cancel_poll;
    std::function<void(std::size_t done, std::size_t total)> on_progress;
};

struct AudioBankPrepResult {
    ps_audio::AudioBankPtr bank;
    bool cache_hit = false;
    std::uint64_t key = 0;
    std::string path;                              // <cache_dir>/audio/<key>.psab, empty without a cache_dir
    double seconds = 0.0;                          // wall time of the call
    std::size_t bytes = 0;                         // sample payload
    std::vector<double> removed_energy_fraction;   // per plane (generated banks only): energy the anti-alias band-limit removed
};

// `engine` must be a simulated engine with a cycle block (the loaders' maps complete,
// maps_pending false). `forced_induction`: nullptr = use engine.turbo_pair as it is (including
// none); otherwise this turbo configuration replaces it for the bake. Throws std::runtime_error
// on an unusable engine, ps::drivetrain::GenerationCancelled when cancelled.
AudioBankPrepResult prepare_engine_audio_bank(const ps::drivetrain::SimulatedEngineDesc& engine,
                                              const ps::drivetrain::TurboPairDesc* forced_induction,
                                              const AudioBankPrepOptions& options = {});

// The cache key prepare_engine_audio_bank would use. Runs the 10 operating-point settles (about
// 0.1-0.3 s [M] on all workers), no recording; use it to decide whether an edit needs a new bake.
// options.cache_dir and on_progress are unused; cancel, cancel_poll and workers apply.
std::uint64_t engine_audio_bank_key(const ps::drivetrain::SimulatedEngineDesc& engine,
                                    const ps::drivetrain::TurboPairDesc* forced_induction,
                                    const AudioBankPrepOptions& options = {});

// <cache_dir>/audio/<key as 16 lowercase hex digits>.psab
std::string audio_bank_cache_path(const std::string& cache_dir, std::uint64_t key);

// Convenience for tools and the Godot adapter, which hold file paths: loads the engine with
// ps::io::load_engine_json (maps from / into options.cache_dir, generated on the calling thread when
// missing; options.cancel is forwarded to that generation, progress is not - load first yourself
// when the maps may still be missing and you want their progress) and the optional turbo
// configuration file (empty = the engine's own), then prepare_engine_audio_bank.
AudioBankPrepResult prepare_engine_audio_bank_from_files(const std::string& engine_path, const std::string& turbo_configuration_path,
                                                         const AudioBankPrepOptions& options = {});
}  // namespace ps_engine_tools
