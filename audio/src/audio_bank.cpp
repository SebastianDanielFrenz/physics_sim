// ps_audio/audio_bank.cpp - see audio_bank.h.
#include "ps_audio/audio_bank.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace ps_audio {
namespace {

constexpr char kMagicV2[8] = {'P', 'S', 'A', 'B', '0', '0', '0', '2'};
constexpr std::size_t kMaxBankFloats = std::size_t{64} * 1024 * 1024;  // 256 MB of samples
constexpr double kPi = 3.14159265358979323846264338327950288;

struct Fnv {
    std::uint64_t h = 14695981039346656037ULL;
    void fold(const void* p, std::size_t n) {
        const auto* bytes = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    template <class T>
    void fold_vec(const std::vector<T>& v) { fold(v.data(), v.size() * sizeof(T)); }
};

std::uint64_t checksum(const AudioBank& b) {
    Fnv f;
    f.fold_vec(b.rpms);
    f.fold_vec(b.pressures);
    f.fold_vec(b.frames_per_cycle);
    f.fold_vec(b.cycles);
    f.fold_vec(b.crank0_deg);
    f.fold_vec(b.samples);
    return f.h;
}

template <class T>
void read_vec(std::istream& in, std::vector<T>& v, std::size_t n) {
    v.resize(n);
    in.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(n * sizeof(T)));
}
template <class T>
void write_vec(std::ostream& out, const std::vector<T>& v) {
    out.write(reinterpret_cast<const char*>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(T)));
}

double sinc(double x) {
    if (x == 0.0) return 1.0;
    return ps::math::sin(kPi * x) / (kPi * x);
}

}  // namespace

void AudioBank::finalize() {
    const std::size_t n = rpms.size();
    if (rate < 8000 || rate > 192000 || channels < 1 || channels > 8 || n < 1 || n * 2 > static_cast<std::size_t>(kBankMaxPlanes) ||
        pressures.size() != n * 2 || frames_per_cycle.size() != n || cycles.size() != n || crank0_deg.size() != n * 2)
        throw std::runtime_error("Invalid audio bank dimensions");
    double previous = 0.0;
    for (std::size_t r = 0; r < n; ++r) {
        const double rpm = rpms[r];
        if (!std::isfinite(rpm) || rpm <= previous) throw std::runtime_error("Invalid audio bank RPM axis");
        previous = rpm;
        if (!std::isfinite(pressures[r * 2]) || !std::isfinite(pressures[r * 2 + 1]) || pressures[r * 2] <= 0.0 ||
            pressures[r * 2 + 1] <= pressures[r * 2])
            throw std::runtime_error("Invalid audio bank pressure axis");
        if (crank_synchronous) {
            if (frames_per_cycle[r] < 16 || cycles[r] < 1) throw std::runtime_error("Invalid audio bank cycle layout");
            const double expected = static_cast<double>(rate) * 120.0 / static_cast<double>(frames_per_cycle[r]);
            if (std::abs(rpm - expected) > 1e-9 * expected)
                throw std::runtime_error("Audio bank RPM is not rate*120/frames_per_cycle");
            for (int l = 0; l < 2; ++l) {
                const double c0 = crank0_deg[r * 2 + static_cast<std::size_t>(l)];
                if (!std::isfinite(c0) || c0 < 0.0 || c0 >= 720.0) throw std::runtime_error("Invalid audio bank crank origin");
            }
        } else if (frames_per_cycle[r] < 2400 || cycles[r] != 1) {
            throw std::runtime_error("Invalid sample-time audio bank layout");
        }
    }
    plane_offset.assign(n * 2, 0);
    std::size_t total = 0;
    for (std::size_t p = 0; p < n * 2; ++p) {
        plane_offset[p] = total;
        const std::size_t frames = static_cast<std::size_t>(cycles[p / 2]) * static_cast<std::size_t>(frames_per_cycle[p / 2]);
        total += frames * static_cast<std::size_t>(channels);
        if (total > kMaxBankFloats) throw std::runtime_error("Audio bank too large");
    }
    if (samples.size() != total) throw std::runtime_error("Invalid audio bank sample count");
    for (float v : samples)
        if (!std::isfinite(v)) throw std::runtime_error("Nonfinite audio bank sample");
}

AudioBankPtr read_audio_bank(const std::string& path, std::uint64_t expected_key) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Missing audio bank");
    char magic[8]{};
    in.read(magic, 8);
    if (!in || std::memcmp(magic, kMagicV2, 8) != 0) throw std::runtime_error("Audio bank version mismatch");
    auto b = std::make_shared<AudioBank>();
    std::uint64_t sum = 0;
    std::uint32_t n = 0, reserved = 0;
    in.read(reinterpret_cast<char*>(&b->key), 8);
    in.read(reinterpret_cast<char*>(&sum), 8);
    in.read(reinterpret_cast<char*>(&b->rate), 4);
    in.read(reinterpret_cast<char*>(&b->channels), 4);
    in.read(reinterpret_cast<char*>(&n), 4);
    in.read(reinterpret_cast<char*>(&reserved), 4);
    if (!in || b->key != expected_key || n < 1 || n * 2 > static_cast<std::uint32_t>(kBankMaxPlanes) || b->channels < 1 ||
        b->channels > 8 || reserved != 0)
        throw std::runtime_error("Audio bank key/dimension mismatch");
    read_vec(in, b->rpms, n);
    read_vec(in, b->pressures, std::size_t{n} * 2);
    read_vec(in, b->frames_per_cycle, n);
    read_vec(in, b->cycles, n);
    read_vec(in, b->crank0_deg, std::size_t{n} * 2);
    if (!in) throw std::runtime_error("Truncated audio bank");
    std::size_t total = 0;
    for (std::size_t r = 0; r < n; ++r) {
        if (b->frames_per_cycle[r] < 1 || b->cycles[r] < 1) throw std::runtime_error("Invalid audio bank cycle layout");
        total += 2 * static_cast<std::size_t>(b->frames_per_cycle[r]) * static_cast<std::size_t>(b->cycles[r]) *
                 static_cast<std::size_t>(b->channels);
        if (total > kMaxBankFloats) throw std::runtime_error("Audio bank too large");
    }
    read_vec(in, b->samples, total);
    if (!in || checksum(*b) != sum) throw std::runtime_error("Audio bank checksum mismatch");
    b->finalize();
    return b;
}

void write_audio_bank(const AudioBank& bank, const std::string& path) {
    AudioBank b = bank;
    b.finalize();
    if (!b.crank_synchronous) throw std::runtime_error("Only crank-synchronous banks are written");
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot save audio bank");
    const std::uint64_t sum = checksum(b);
    const auto n = static_cast<std::uint32_t>(b.rpms.size());
    const std::uint32_t reserved = 0;
    out.write(kMagicV2, 8);
    out.write(reinterpret_cast<const char*>(&b.key), 8);
    out.write(reinterpret_cast<const char*>(&sum), 8);
    out.write(reinterpret_cast<const char*>(&b.rate), 4);
    out.write(reinterpret_cast<const char*>(&b.channels), 4);
    out.write(reinterpret_cast<const char*>(&n), 4);
    out.write(reinterpret_cast<const char*>(&reserved), 4);
    write_vec(out, b.rpms);
    write_vec(out, b.pressures);
    write_vec(out, b.frames_per_cycle);
    write_vec(out, b.cycles);
    write_vec(out, b.crank0_deg);
    write_vec(out, b.samples);
    if (!out) throw std::runtime_error("Audio bank write failed");
}

std::vector<float> build_crank_loop(const std::vector<float>& rec, int channels, const CrankLoopSpec& s, CrankLoopStats* stats) {
    if (channels < 1 || s.frames_per_cycle < 1 || s.cycles < 1 || s.filter_half_taps < 0 ||
        s.crossfade_frames < s.frames_per_cycle || s.crossfade_frames % s.frames_per_cycle != 0 ||
        s.crossfade_frames > s.cycles * s.frames_per_cycle)
        throw std::invalid_argument("build_crank_loop: invalid loop spec");
    const std::size_t ch = static_cast<std::size_t>(channels);
    const std::size_t D = static_cast<std::size_t>(s.filter_half_taps);
    const std::size_t L = static_cast<std::size_t>(s.cycles) * static_cast<std::size_t>(s.frames_per_cycle);
    const std::size_t X = static_cast<std::size_t>(s.crossfade_frames);
    if (rec.size() != (2 * D + L + X) * ch) throw std::invalid_argument("build_crank_loop: recording length mismatch");

    // Zero-phase Blackman-windowed sinc, unity DC gain.
    std::vector<double> h(2 * D + 1, 0.0);
    if (D > 0) {
        if (!(s.cutoff_hz > 0.0 && s.cutoff_hz < 0.5 * s.rate_hz)) throw std::invalid_argument("build_crank_loop: invalid cutoff");
        const double fc = s.cutoff_hz / s.rate_hz;  // cycles per sample
        const double m = static_cast<double>(2 * D);
        double sum = 0.0;
        for (std::size_t k = 0; k <= 2 * D; ++k) {
            const double x = static_cast<double>(k);
            const double w = 0.42 - 0.5 * ps::math::cos(2.0 * kPi * x / m) + 0.08 * ps::math::cos(4.0 * kPi * x / m);
            h[k] = 2.0 * fc * sinc(2.0 * fc * (x - static_cast<double>(D))) * w;
            sum += h[k];
        }
        for (double& v : h) v /= sum;
    } else {
        h[0] = 1.0;
    }
    std::vector<double> filtered((L + X) * ch, 0.0);
    double e_raw = 0.0, e_filt = 0.0;
    for (std::size_t n = 0; n < L + X; ++n) {
        for (std::size_t c = 0; c < ch; ++c) {
            double acc = 0.0;
            for (std::size_t k = 0; k <= 2 * D; ++k) acc += h[k] * static_cast<double>(rec[(n + k) * ch + c]);
            filtered[n * ch + c] = acc;
            if (n < L) {
                const double raw = rec[(n + D) * ch + c];
                e_raw += raw * raw;
                e_filt += acc * acc;
            }
        }
    }
    if (stats != nullptr) stats->removed_energy_fraction = e_raw > 0.0 ? 1.0 - e_filt / e_raw : 0.0;
    // Cycle-aligned crossfade: frame x and frame x + L are at the same crank
    // angle; the loop start fades from the recording's own continuation past
    // the loop end (frame L follows frame L-1 seamlessly) into the start.
    std::vector<float> loop(L * ch);
    for (std::size_t x = 0; x < L; ++x) {
        const double w = x < X ? static_cast<double>(x) / static_cast<double>(X) : 1.0;
        for (std::size_t c = 0; c < ch; ++c) {
            const double v = x < X ? w * filtered[x * ch + c] + (1.0 - w) * filtered[(x + L) * ch + c] : filtered[x * ch + c];
            loop[x * ch + c] = static_cast<float>(v);
        }
    }
    return loop;
}

float bank_plane_sample(const AudioBank& b, int plane, int channel, double pos) {
    const int frames = b.plane_frames(plane);
    const float* d = b.plane_data(plane);
    const auto ch = static_cast<std::size_t>(b.channels);
    double fl = std::floor(pos);
    const double t = pos - fl;
    long long i = static_cast<long long>(fl) % frames;
    if (i < 0) i += frames;
    auto at = [&](long long k) {
        if (k < 0) k += frames;
        if (k >= frames) k -= frames;
        return static_cast<double>(d[static_cast<std::size_t>(k) * ch + static_cast<std::size_t>(channel)]);
    };
    const double p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2 >= frames ? i + 2 - frames : i + 2);
    return static_cast<float>(p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0))));
}

void BankPlayback::frame(const AudioBank& b, double rpm, double pressure, double crank, float* out) {
    const std::size_t rows = b.rpms.size();
    if (last_crank_ >= 0.0) {
        if (crank < last_crank_ - 360.0) {
            for (std::size_t r = 0; r < rows; ++r) cycle_[r] = cycle_[r] + 1 >= b.cycles[r] ? 0 : cycle_[r] + 1;
        } else if (crank > last_crank_ + 360.0) {
            for (std::size_t r = 0; r < rows; ++r) cycle_[r] = cycle_[r] == 0 ? b.cycles[r] - 1 : cycle_[r] - 1;
        }
    }
    last_crank_ = crank;
    std::size_t lo = 0;
    while (lo + 1 < rows && b.rpms[lo + 1] < rpm) ++lo;
    const std::size_t hi = std::min(lo + 1, rows - 1);
    const double mix = hi == lo ? 0.0 : std::clamp((rpm - b.rpms[lo]) / (b.rpms[hi] - b.rpms[lo]), 0.0, 1.0);
    auto load = [&](std::size_t row) {
        return std::clamp((pressure - b.pressures[row * 2]) / (b.pressures[row * 2 + 1] - b.pressures[row * 2]), 0.0, 1.0);
    };
    auto pos = [&](std::size_t plane) {
        const std::size_t r = plane / 2;
        const double span = static_cast<double>(b.cycles[r]) * 720.0;
        double a = static_cast<double>(cycle_[r]) * 720.0 + crank - b.crank0_deg[plane];
        while (a < 0.0) a += span;
        while (a >= span) a -= span;
        return a * static_cast<double>(b.frames_per_cycle[r]) / 720.0;
    };
    const double a = load(lo), d = load(hi);
    const double p_l0 = pos(lo * 2), p_l1 = pos(lo * 2 + 1), p_h0 = pos(hi * 2), p_h1 = pos(hi * 2 + 1);
    const double level = std::clamp(rpm / 400.0, 0.0, 1.0);
    const int lp = static_cast<int>(lo * 2), hp = static_cast<int>(hi * 2);
    for (int c = 0; c < b.channels; ++c) {
        const double l = bank_plane_sample(b, lp, c, p_l0) * (1.0 - a) + bank_plane_sample(b, lp + 1, c, p_l1) * a;
        const double h = bank_plane_sample(b, hp, c, p_h0) * (1.0 - d) + bank_plane_sample(b, hp + 1, c, p_h1) * d;
        out[c] = static_cast<float>((l * (1.0 - mix) + h * mix) * level);
    }
}

}  // namespace ps_audio
