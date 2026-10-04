// ps_audio/audio_bank_reference.h - MEASUREMENT / TEST REFERENCE ONLY.
//
// The engine-sound bank playback as shipped in 40923cf (PSAB0001): each plane
// advanced in SAMPLE TIME at rpm / plane_rpm and looped with a 25 ms
// (1200-frame) linear overlap. Loop joins are not crank-aligned and the
// blended planes have unrelated crank phases, which made audible periodic
// level steps (partial cancellation) under throttle. Replaced by the
// crank-synchronous BankPlayback (audio_bank.h); kept unchanged so the
// measurement harness (ps_engine_audio --audio-bank-v1 / --bank-reference-
// playback) can reproduce the defect and the [audio] tests can show each new
// guard failing on the old algorithm. Nothing in the shipped path uses it.
#pragma once
#include "ps_audio/audio_bank.h"

#include <array>
#include <string>

namespace ps_audio {

// Reads a PSAB0001 file (no key check) into an AudioBank with
// crank_synchronous = false: frames_per_cycle = the plane length, cycles = 1.
AudioBankPtr read_audio_bank_v1_reference(const std::string& path);

// The old algorithm, verbatim except that the plane length is per plane
// (bank.plane_frames) instead of one global `frames`.
class SampleTimeBankPlayback {
public:
    std::array<double, kBankMaxPlanes> phase{};
    void frame(const AudioBank& bank, double rpm, double pressure, float* out);
    static float sample(const AudioBank& bank, int plane, int channel, double phase);
};

}  // namespace ps_audio
