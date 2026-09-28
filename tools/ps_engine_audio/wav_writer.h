// wav_writer.h - ps_audio A10: a minimal, self-contained 16-bit PCM WAV
// writer for tools/ps_engine_audio (own code, no third-party library, per
// the brief: "header written by own code"). Little-endian only (this
// project's only build targets are x86_64, same assumption
// core/src/io/test_ground_loader.cpp already makes for its own binary
// format).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ps_tools {

// Writes a standard RIFF/WAVE file, PCM format tag 1, 16 bits/sample.
// `samples` is interleaved frame-major/channel-minor
// (samples[frame * channels + channel]), already clamped to the int16
// range by the caller (write_wav_pcm16 does not clip). Throws
// std::runtime_error if the file cannot be created.
void write_wav_pcm16(const std::string& path, int sample_rate_hz, int channels, const std::vector<int16_t>& samples);

} // namespace ps_tools
