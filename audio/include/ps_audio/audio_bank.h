// ps_audio/audio_bank.h - a baked engine-sound bank and its crank-synchronous
// playback (format PSAB0002).
//
// A bank is a grid of rpm rows x two load planes (low/high manifold pressure).
// Each plane is a LOOP of a whole number of engine cycles (720 deg crank),
// recorded from the live voice at a steady rpm chosen so one cycle is an
// integer number of frames: rpm = rate * 120 / frames_per_cycle exactly. The
// crank angle of the plane's frame 0 (crank0_deg, the cycle model's
// convention, 0 = cylinder 1 gas-exchange TDC) is stored with it, so frame n
// sits at crank0 + n * 720 / frames_per_cycle. build_crank_loop() turns a raw
// recording into such a loop: a zero-phase band-limit (so a plane played
// faster than it was recorded, up to the next rpm row, does not alias) and a
// cycle-aligned linear crossfade at the seam (the recording's own
// continuation past the loop end, at the same crank angle, fades into the
// loop start - cycle-to-cycle variation and mean drift never make a step).
//
// BankPlayback reads every plane at the LIVE crank angle (the voice's own
// engine crank, after its phase correction), not in sample time: all blended
// planes are phase-aligned with each other and with the engine, and a plane
// wraps exactly at its loop end (a cycle boundary). Planes are interpolated
// (4-point Catmull-Rom) at the same crank angle, then mixed linearly between
// the two load planes of a row and between the two bracketing rpm rows.
// Allocation-free and deterministic.
//
// The old sample-time playback (PSAB0001) survives only as a measurement /
// test reference: ps_audio/audio_bank_reference.h.
#pragma once
#include "ps/types.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace ps_audio {
inline constexpr int kBankMaxPlanes = 32;  // 16 rpm rows x 2 load planes

struct AudioBank {
    int rate = 48000, channels = 0;
    std::uint64_t key = 0;
    std::vector<ps::real> rpms;                    // per row, strictly increasing
    std::vector<ps::real> pressures;               // per row: low-load then high-load plane manifold pressure [Pa]
    std::vector<std::int32_t> frames_per_cycle;    // per row (rpms[r] == rate * 120 / frames_per_cycle[r])
    std::vector<std::int32_t> cycles;              // per row: loop length in engine cycles
    std::vector<ps::real> crank0_deg;              // per plane (row * 2 + load): crank angle of frame 0, [0, 720)
    // Planes in order row * 2 + load, each cycles[row] * frames_per_cycle[row]
    // frames, frame-major / channel-minor; pressure at 1 m [Pa].
    std::vector<float> samples;
    // false only for a PSAB0001 bank read through audio_bank_reference.h (no
    // crank data; playable by the sample-time reference only). Not stored.
    bool crank_synchronous = true;

    // Derived (finalize()): float offset of each plane in `samples`.
    std::vector<std::size_t> plane_offset;
    // Validates every field and fills plane_offset; throws std::runtime_error.
    void finalize();
    [[nodiscard]] int rows() const { return static_cast<int>(rpms.size()); }
    [[nodiscard]] int plane_frames(int plane) const { return cycles[static_cast<std::size_t>(plane / 2)] * frames_per_cycle[static_cast<std::size_t>(plane / 2)]; }
    [[nodiscard]] const float* plane_data(int plane) const { return samples.data() + plane_offset[static_cast<std::size_t>(plane)]; }
};
using AudioBankPtr = std::shared_ptr<const AudioBank>;

// PSAB0002. read: any mismatch (magic/version, key, dimensions, checksum,
// truncation) throws std::runtime_error - a cache caller treats that as a miss.
AudioBankPtr read_audio_bank(const std::string& path, std::uint64_t expected_key);
void write_audio_bank(const AudioBank& bank, const std::string& path);

// Raw recording -> one plane's loop. `recording` is interleaved
// (frame-major, `channels` per frame) and holds
//   filter_half_taps + cycles * frames_per_cycle + crossfade_frames + filter_half_taps
// frames; its loop frame 0 is recording frame filter_half_taps (the caller
// stores that frame's crank angle as crank0_deg). crossfade_frames must be a
// whole number of cycles in [frames_per_cycle, cycles * frames_per_cycle].
// The band-limit is a Blackman-windowed sinc of 2 * filter_half_taps + 1 taps,
// unity DC gain, -6 dB at cutoff_hz, applied zero-phase (no crank shift);
// filter_half_taps == 0 disables it. Returns cycles * frames_per_cycle
// interleaved frames.
struct CrankLoopSpec {
    int frames_per_cycle = 0;
    int cycles = 0;
    int crossfade_frames = 0;
    int filter_half_taps = 0;
    double cutoff_hz = 0.0;
    double rate_hz = 48000.0;
};
struct CrankLoopStats {
    double removed_energy_fraction = 0.0;  // band-limit: 1 - E(filtered) / E(raw) over the loop span
};
std::vector<float> build_crank_loop(const std::vector<float>& recording, int channels, const CrankLoopSpec& spec,
                                    CrankLoopStats* stats = nullptr);

// Crank-synchronous playback state (one per voice and bank).
class BankPlayback {
public:
    // One output frame (bank.channels values) at the live engine's crank angle
    // [deg, 0..720) and speed. rpm also scales the level to 0 below 400 rpm
    // (as before), pressure selects the load mix.
    void frame(const AudioBank& bank, double rpm, double pressure, double crank_deg, float* out);

private:
    double last_crank_ = -1.0;
    std::array<std::int32_t, kBankMaxPlanes / 2> cycle_{};  // per row: recorded cycle the live crank is in
};

// Catmull-Rom read of plane `plane`, channel `channel` at fractional loop
// frame `pos` in [0, plane_frames), wrapping modulo the loop.
float bank_plane_sample(const AudioBank& bank, int plane, int channel, double pos);
}  // namespace ps_audio
