// ps_audio/audio_bank_reference.cpp - see audio_bank_reference.h (reference only).
#include "ps_audio/audio_bank_reference.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace ps_audio {

AudioBankPtr read_audio_bank_v1_reference(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Missing audio bank");
    char magic[8]{};
    in.read(magic, 8);
    if (std::memcmp(magic, "PSAB0001", 8) != 0) throw std::runtime_error("Not a PSAB0001 audio bank");
    auto b = std::make_shared<AudioBank>();
    std::uint64_t sum = 0;
    std::uint32_t n = 0;
    int frames = 0;
    in.read(reinterpret_cast<char*>(&b->key), 8);
    in.read(reinterpret_cast<char*>(&sum), 8);
    in.read(reinterpret_cast<char*>(&b->rate), 4);
    in.read(reinterpret_cast<char*>(&b->channels), 4);
    in.read(reinterpret_cast<char*>(&frames), 4);
    in.read(reinterpret_cast<char*>(&n), 4);
    if (!in || n < 2 || n * 2 > static_cast<std::uint32_t>(kBankMaxPlanes) || frames < 4800 || frames > 96000 || b->channels < 1 ||
        b->channels > 8)
        throw std::runtime_error("Audio bank dimension mismatch");
    b->rpms.resize(n);
    b->pressures.resize(std::size_t{n} * 2);
    b->samples.resize(std::size_t{n} * 2 * static_cast<std::size_t>(frames) * static_cast<std::size_t>(b->channels));
    in.read(reinterpret_cast<char*>(b->rpms.data()), static_cast<std::streamsize>(b->rpms.size() * sizeof(ps::real)));
    in.read(reinterpret_cast<char*>(b->pressures.data()), static_cast<std::streamsize>(b->pressures.size() * sizeof(ps::real)));
    in.read(reinterpret_cast<char*>(b->samples.data()), static_cast<std::streamsize>(b->samples.size() * sizeof(float)));
    if (!in) throw std::runtime_error("Truncated audio bank");
    std::uint64_t h = 14695981039346656037ULL;
    auto fold = [&](const void* p, std::size_t len) {
        const auto* bytes = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < len; ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    };
    fold(b->rpms.data(), b->rpms.size() * sizeof(ps::real));
    fold(b->pressures.data(), b->pressures.size() * sizeof(ps::real));
    fold(b->samples.data(), b->samples.size() * sizeof(float));
    if (h != sum) throw std::runtime_error("Audio bank checksum mismatch");
    b->crank_synchronous = false;
    b->frames_per_cycle.assign(n, frames);
    b->cycles.assign(n, 1);
    b->crank0_deg.assign(std::size_t{n} * 2, 0.0);
    b->finalize();
    return b;
}

float SampleTimeBankPlayback::sample(const AudioBank& b, int plane, int channel, double p) {
    // 25 ms overlap hides loop joins; phase wraps to the end of the overlap.
    constexpr int overlap = 1200;
    const int frames = b.plane_frames(plane);
    const float* d = b.plane_data(plane);
    const int a = static_cast<int>(p);
    const int next = std::min(a + 1, frames - 1);
    const double f = p - a;
    auto at = [&](int frame) {
        return d[static_cast<std::size_t>(frame) * static_cast<std::size_t>(b.channels) + static_cast<std::size_t>(channel)];
    };
    float v = static_cast<float>(at(a) * (1 - f) + at(next) * f);
    if (a >= frames - overlap) {
        const double head = p - (frames - overlap);
        const int i = static_cast<int>(head);
        const double hf = head - i;
        const float hv = static_cast<float>(at(i) * (1 - hf) + at(std::min(i + 1, overlap)) * hf);
        const double blend = head / overlap;
        v = static_cast<float>(v * (1 - blend) + hv * blend);
    }
    return v;
}

void SampleTimeBankPlayback::frame(const AudioBank& b, double rpm, double pressure, float* out) {
    std::size_t lo = 0;
    while (lo + 1 < b.rpms.size() && b.rpms[lo + 1] < rpm) ++lo;
    const std::size_t hi = std::min(lo + 1, b.rpms.size() - 1);
    const double mix = hi == lo ? 0 : std::clamp((rpm - b.rpms[lo]) / (b.rpms[hi] - b.rpms[lo]), 0.0, 1.0);
    auto load = [&](std::size_t row) {
        return std::clamp((pressure - b.pressures[row * 2]) / (b.pressures[row * 2 + 1] - b.pressures[row * 2]), 0.0, 1.0);
    };
    const double a = load(lo), d = load(hi);
    for (int c = 0; c < b.channels; ++c) {
        const double l = sample(b, static_cast<int>(lo * 2), c, phase[lo * 2]) * (1 - a) +
                         sample(b, static_cast<int>(lo * 2 + 1), c, phase[lo * 2 + 1]) * a;
        const double h = sample(b, static_cast<int>(hi * 2), c, phase[hi * 2]) * (1 - d) +
                         sample(b, static_cast<int>(hi * 2 + 1), c, phase[hi * 2 + 1]) * d;
        out[c] = static_cast<float>((l * (1 - mix) + h * mix) * std::clamp(rpm / 400.0, 0.0, 1.0));
    }
    for (std::size_t i = 0; i < b.rpms.size() * 2; ++i) {
        const int frames = b.plane_frames(static_cast<int>(i));
        phase[i] += std::max(0.0, rpm) / b.rpms[i / 2];
        while (phase[i] >= frames) phase[i] -= frames - 1200;
    }
}

}  // namespace ps_audio
