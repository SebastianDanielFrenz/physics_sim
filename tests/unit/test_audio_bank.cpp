// Crank-synchronous engine audio bank (PSAB0002, ps_audio/audio_bank.h).
// Banks here are SYNTHETIC: each plane is "recorded" from a crank-periodic
// test signal (orders 2/4/8, optional smooth cycle-to-cycle variation and a
// DC drift) at the plane's exact rpm, with a known crank angle at frame 0,
// and turned into a loop by the production build_crank_loop(). Played at a
// live crank angle, the bank must reproduce the signal at that crank angle
// (all planes aligned, also across their seams), stay continuous at a seam,
// and carry no level modulation at the old loop period. Each guard is shown
// to fail with the old sample-time playback (audio_bank_reference.h) - the
// failing numbers are printed by the "sabotage" sections and recorded in
// out/audio_bank_crank_sync_progress.md.
#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"
#include "engine_bank_generator.h"

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps_audio/audio_bank.h"
#include "ps_audio/audio_bank_reference.h"
#include "ps_audio/runner.h"
#include "ps_audio/voice.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace {

using ps::real;
namespace test = ps::test;
constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr int kRate = 48000;

// ---- synthetic crank-periodic source --------------------------------------

struct Synth {
    std::array<int, 3> orders{2, 4, 8};
    std::array<double, 3> amp{1.0, 0.6, 0.3};
    std::array<double, 3> phase{0.3, 1.1, 2.0};
    double variation = 0.0;    // smooth per-cycle relative amplitude variation
    double drift_per_s = 0.0;  // DC drift over the recording [units/s]
};

double hash_unit(std::int64_t cycle, int k) {
    std::uint64_t z = static_cast<std::uint64_t>(cycle) * 0x9e3779b97f4a7c15ull + static_cast<std::uint64_t>(k) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    z ^= z >> 31;
    return static_cast<double>(z >> 11) / static_cast<double>(1ull << 53) * 2.0 - 1.0;
}

// phi: unwrapped crank angle [deg]; t: recording time [s].
double synth_value(const Synth& s, double phi, double t) {
    const double c = phi / 720.0;
    const double ci = std::floor(c);
    const double w = 0.5 - 0.5 * std::cos(kPi * (c - ci));
    double v = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double u0 = hash_unit(static_cast<std::int64_t>(ci), k), u1 = hash_unit(static_cast<std::int64_t>(ci) + 1, k);
        const double m = 1.0 + s.variation * (u0 + (u1 - u0) * w);
        v += s.amp[static_cast<std::size_t>(k)] * m * std::cos(2.0 * kPi * s.orders[static_cast<std::size_t>(k)] * phi / 720.0 + s.phase[static_cast<std::size_t>(k)]);
    }
    return v + s.drift_per_s * t;
}

double wrap720(double d) {
    d = std::fmod(d, 720.0);
    return d < 0.0 ? d + 720.0 : d;
}

struct SynthBankSpec {
    std::vector<int> fpc{1440, 960};  // 4000 / 6000 rpm
    std::vector<int> cycles{12, 18};
    int half_taps = 16;
    double cutoff_hz = 10000.0;
    std::array<double, 2> load_scale{0.5, 1.0};
    std::array<double, 2> pressure{30000.0, 100000.0};
    std::vector<double> crank_start{37.5, 211.25, 402.0, 655.75};
    Synth synth{};
    int channels = 1;
    bool no_crossfade = false;  // sabotage: the raw recording, no seam crossfade
};

double channel_scale(int c) { return 1.0 + 0.5 * c; }

ps_audio::AudioBankPtr make_bank(const SynthBankSpec& s) {
    auto b = std::make_shared<ps_audio::AudioBank>();
    b->rate = kRate;
    b->channels = s.channels;
    b->key = 77;
    const int rows = static_cast<int>(s.fpc.size());
    for (int r = 0; r < rows; ++r) {
        const int fpc = s.fpc[static_cast<std::size_t>(r)];
        const int cycles = s.cycles[static_cast<std::size_t>(r)];
        b->rpms.push_back(kRate * 120.0 / fpc);
        b->frames_per_cycle.push_back(fpc);
        b->cycles.push_back(cycles);
        for (int l = 0; l < 2; ++l) {
            const int plane = r * 2 + l;
            const double start = s.crank_start[static_cast<std::size_t>(plane) % s.crank_start.size()];
            const double dtheta = 720.0 / fpc;
            const int L = cycles * fpc;
            const int X = fpc * std::max(1, (2400 + fpc - 1) / fpc);
            const int D = s.half_taps;
            const int n = 2 * D + L + X;
            std::vector<float> rec(static_cast<std::size_t>(n * s.channels));
            for (int j = 0; j < n; ++j)
                for (int c = 0; c < s.channels; ++c)
                    rec[static_cast<std::size_t>(j * s.channels + c)] = static_cast<float>(
                        s.load_scale[static_cast<std::size_t>(l)] * channel_scale(c) * synth_value(s.synth, start + j * dtheta, j / static_cast<double>(kRate)));
            std::vector<float> loop;
            if (s.no_crossfade) {
                loop.assign(rec.begin() + static_cast<std::ptrdiff_t>(D * s.channels), rec.begin() + static_cast<std::ptrdiff_t>((D + L) * s.channels));
            } else {
                ps_audio::CrankLoopSpec spec;
                spec.frames_per_cycle = fpc;
                spec.cycles = cycles;
                spec.crossfade_frames = X;
                spec.filter_half_taps = D;
                spec.cutoff_hz = s.cutoff_hz;
                loop = ps_audio::build_crank_loop(rec, s.channels, spec);
            }
            b->samples.insert(b->samples.end(), loop.begin(), loop.end());
            b->crank0_deg.push_back(wrap720(start + D * dtheta));
            b->pressures.push_back(s.pressure[static_cast<std::size_t>(l)]);
        }
    }
    b->finalize();
    return b;
}

// Plays `frames` samples at a steady rpm with the live crank starting at
// theta0 (frame n at theta0 + (n+1)*dtheta, unwrapped in `phi`).
std::vector<double> play(const ps_audio::AudioBank& b, bool reference, double rpm, double pressure, double theta0, int frames,
                         std::vector<double>& phi) {
    ps_audio::BankPlayback crank_pb;
    ps_audio::SampleTimeBankPlayback ref_pb;
    std::vector<double> y(static_cast<std::size_t>(frames));
    phi.resize(static_cast<std::size_t>(frames));
    const double dtheta = rpm * 6.0 / kRate;
    float out[8]{};
    for (int n = 0; n < frames; ++n) {
        const double p = theta0 + (n + 1) * dtheta;
        phi[static_cast<std::size_t>(n)] = p;
        if (reference) ref_pb.frame(b, rpm, pressure, out);
        else crank_pb.frame(b, rpm, pressure, wrap720(p), out);
        y[static_cast<std::size_t>(n)] = out[0];
    }
    return y;
}

// Seam frames of row `row`'s planes: where (phi - crank0) crosses a multiple of the loop span.
std::vector<int> seam_frames(const ps_audio::AudioBank& b, int row, const std::vector<double>& phi) {
    std::vector<int> out;
    for (int l = 0; l < 2; ++l) {
        const double span = b.cycles[static_cast<std::size_t>(row)] * 720.0;
        const double c0 = b.crank0_deg[static_cast<std::size_t>(row * 2 + l)];
        for (std::size_t n = 1; n < phi.size(); ++n)
            if (std::floor((phi[n] - c0) / span) != std::floor((phi[n - 1] - c0) / span)) out.push_back(static_cast<int>(n));
    }
    return out;
}

// AC RMS of windows of `win` frames every `hop` frames.
std::vector<double> ac_rms(const std::vector<double>& y, int start, int win, int hop) {
    std::vector<double> out;
    for (int i = start; i + win <= static_cast<int>(y.size()); i += hop) {
        double m = 0.0;
        for (int k = 0; k < win; ++k) m += y[static_cast<std::size_t>(i + k)];
        m /= win;
        double e = 0.0;
        for (int k = 0; k < win; ++k) e += (y[static_cast<std::size_t>(i + k)] - m) * (y[static_cast<std::size_t>(i + k)] - m);
        out.push_back(std::sqrt(e / win));
    }
    return out;
}

// Relative amplitude of the component at `period_frames` in an envelope sampled every `hop` frames.
double envelope_component(const std::vector<double>& env, int hop, double period_frames) {
    double mean = 0.0;
    for (double v : env) mean += v;
    mean /= static_cast<double>(env.size());
    // whole periods only
    const double per = period_frames / hop;
    const std::size_t n = static_cast<std::size_t>(std::floor(static_cast<double>(env.size()) / per) * per);
    double re = 0.0, im = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double a = 2.0 * kPi * static_cast<double>(i) / per;
        re += (env[i] - mean) * std::cos(a);
        im += (env[i] - mean) * std::sin(a);
    }
    return 2.0 * std::sqrt(re * re + im * im) / static_cast<double>(n) / mean;
}

double sd_db(const std::vector<double>& env) {
    double m = 0.0;
    for (double v : env) m += 20.0 * std::log10(v);
    m /= static_cast<double>(env.size());
    double s = 0.0;
    for (double v : env) s += (20.0 * std::log10(v) - m) * (20.0 * std::log10(v) - m);
    return std::sqrt(s / static_cast<double>(env.size()));
}

struct AlignResult {
    double max_err_rel = 0.0, seam_err_rel = 0.0;
    int seams_row0 = 0, seams_row1 = 0;
};

// T1 measurement: live 5000 rpm (rpm mix 0.5) at 65 kPa (load 0.5): all four
// planes blend; expected output = 0.75 * g(live crank).
AlignResult align_run(bool reference) {
    SynthBankSpec spec;
    const auto bank = make_bank(spec);
    std::vector<double> phi;
    const int frames = 72000;  // 1.5 s
    const auto y = play(*bank, reference, 5000.0, 65000.0, 123.0, frames, phi);
    AlignResult r;
    double peak = 0.0;
    std::vector<double> err(y.size());
    for (std::size_t n = 0; n < y.size(); ++n) {
        const double e = 0.75 * synth_value(spec.synth, phi[n], 0.0);
        peak = std::max(peak, std::abs(e));
        err[n] = std::abs(y[n] - e);
    }
    for (double e : err) r.max_err_rel = std::max(r.max_err_rel, e / peak);
    const int cycle_frames = static_cast<int>(kRate * 120.0 / 5000.0);
    for (int row = 0; row < 2; ++row) {
        const auto seams = seam_frames(*bank, row, phi);
        (row == 0 ? r.seams_row0 : r.seams_row1) = static_cast<int>(seams.size());
        for (int s : seams)
            for (int n = std::max(0, s - cycle_frames); n < std::min(frames, s + cycle_frames); ++n)
                r.seam_err_rel = std::max(r.seam_err_rel, err[static_cast<std::size_t>(n)] / peak);
    }
    return r;
}

struct SeamResult {
    double jump_ratio = 0.0;            // max |dy| within 64 frames of a seam / max elsewhere
    double env_lo = 0.0, env_hi = 0.0;  // seam windows' AC RMS relative to the interior min / max
    int seams = 0;
};

// T2 measurement: one plane (row 0 at its own rpm, low load), signal with 5 %
// smooth cycle variation and a DC drift.
SeamResult seam_run(bool reference, bool no_crossfade) {
    SynthBankSpec spec;
    spec.synth.variation = 0.05;
    spec.synth.drift_per_s = 0.5;
    spec.no_crossfade = no_crossfade;
    const auto bank = make_bank(spec);
    std::vector<double> phi;
    const int frames = 96000;  // 2 s
    const double rpm = bank->rpms[0];
    const auto y = play(*bank, reference, rpm, 30000.0, 300.0, frames, phi);
    // Seams of the PLAYED plane (0): crank-sync wraps where (phi - crank0)
    // crosses the loop span; the reference wraps in sample time at
    // frames - 1200 (every L - 1200 frames at rate 1).
    std::vector<int> seams;
    if (reference) {
        const int L = bank->plane_frames(0);
        for (int n = L - 1; n < frames; n += L - 1200) seams.push_back(n);
    } else {
        const double span = bank->cycles[0] * 720.0;
        for (std::size_t n = 1; n < phi.size(); ++n)
            if (std::floor((phi[n] - bank->crank0_deg[0]) / span) != std::floor((phi[n - 1] - bank->crank0_deg[0]) / span))
                seams.push_back(static_cast<int>(n));
    }
    SeamResult r;
    r.seams = static_cast<int>(seams.size());
    auto near_seam = [&](int n, int margin) {
        for (int s : seams)
            if (std::abs(n - s) <= margin) return true;
        return false;
    };
    // The reference blends over 1200 frames before its wrap; the crank-sync
    // crossfade covers the 2 cycles after the seam.
    const int fpc = bank->frames_per_cycle[0];
    const int seam_lo = reference ? 1300 : 64, seam_hi = reference ? 64 : 2 * fpc;
    auto in_seam = [&](int n) {
        for (int s : seams)
            if (n >= s - seam_lo && n <= s + seam_hi) return true;
        return false;
    };
    double jump_seam = 0.0, jump_else = 0.0;
    for (int n = 1; n < frames; ++n) {
        const double d = std::abs(y[static_cast<std::size_t>(n)] - y[static_cast<std::size_t>(n - 1)]);
        if (near_seam(n, 64)) jump_seam = std::max(jump_seam, d);
        else jump_else = std::max(jump_else, d);
    }
    r.jump_ratio = jump_seam / jump_else;
    // Half-cycle windows (whole periods of orders 2/4/8), hop 1/8 cycle.
    const int win = fpc / 2, hop = fpc / 8;
    const auto env = ac_rms(y, 0, win, hop);
    double in_min = 1e9, in_max = 0.0;
    std::vector<double> seam_env;
    for (std::size_t i = 0; i < env.size(); ++i) {
        const int a = static_cast<int>(i) * hop, b = a + win;
        bool touches = false;
        for (int n = a; n < b && !touches; n += 8) touches = in_seam(n);
        if (touches) seam_env.push_back(env[i]);
        else {
            in_min = std::min(in_min, env[i]);
            in_max = std::max(in_max, env[i]);
        }
    }
    r.env_lo = 1e9;
    r.env_hi = 0.0;
    for (double v : seam_env) {
        r.env_lo = std::min(r.env_lo, v / in_min);
        r.env_hi = std::max(r.env_hi, v / in_max);
    }
    return r;
}

struct PeriodicResult {
    double sd_db = 0.0;
    double comp_row0 = 0.0, comp_row1 = 0.0;  // relative envelope component at each row's old wrap period
};

// T3 measurement: 6 s at 5000 rpm / 65 kPa, no variation; envelope = AC RMS
// of consecutive half cycles.
PeriodicResult periodic_run(bool reference) {
    SynthBankSpec spec;
    const auto bank = make_bank(spec);
    std::vector<double> phi;
    const int frames = 6 * kRate;
    const double rpm = 5000.0;
    const auto y = play(*bank, reference, rpm, 65000.0, 10.0, frames, phi);
    const int half_cycle = static_cast<int>(kRate * 60.0 / rpm);  // 360 deg
    const auto env = ac_rms(y, 0, half_cycle, half_cycle);
    PeriodicResult r;
    r.sd_db = sd_db(env);
    for (int row = 0; row < 2; ++row) {
        const double old_period = (bank->plane_frames(row * 2) - 1200) / (rpm / bank->rpms[static_cast<std::size_t>(row)]);
        (row == 0 ? r.comp_row0 : r.comp_row1) = envelope_component(env, half_cycle, old_period);
    }
    return r;
}

// A PSAB0001 file exactly as 40923cf wrote it (2 rows, constant planes).
void write_v1_file(const std::string& path, std::uint64_t key) {
    const int frames = 4800, channels = 1;
    std::vector<double> rpms{1000, 2000}, pressures{20000, 100000, 20000, 100000};
    std::vector<float> samples(4 * frames, 0.25f);
    std::uint64_t h = 14695981039346656037ULL;
    auto fold = [&](const void* p, std::size_t n) {
        const auto* bytes = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    };
    fold(rpms.data(), rpms.size() * 8);
    fold(pressures.data(), pressures.size() * 8);
    fold(samples.data(), samples.size() * 4);
    const int rate = 48000;
    const std::uint32_t n = 2;
    std::ofstream out(path, std::ios::binary);
    out.write("PSAB0001", 8);
    out.write(reinterpret_cast<const char*>(&key), 8);
    out.write(reinterpret_cast<const char*>(&h), 8);
    out.write(reinterpret_cast<const char*>(&rate), 4);
    out.write(reinterpret_cast<const char*>(&channels), 4);
    out.write(reinterpret_cast<const char*>(&frames), 4);
    out.write(reinterpret_cast<const char*>(&n), 4);
    out.write(reinterpret_cast<const char*>(rpms.data()), 16);
    out.write(reinterpret_cast<const char*>(pressures.data()), 32);
    out.write(reinterpret_cast<const char*>(samples.data()), static_cast<std::streamsize>(samples.size() * 4));
}

const ps::drivetrain::CycleModelDesc& fixture() {
    static const ps::drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    return d;
}

ps_audio::VoiceSettings coarse_settings(real rpm, real p_im) {
    ps_audio::VoiceSettings vs;
    vs.substeps = 1;
    vs.engine.dx_intake_m = 0.05;
    vs.engine.dx_exhaust_m = 0.05;
    vs.engine.initial_rpm = rpm;
    vs.engine.initial_p_intake_pa = p_im;
    return vs;
}

ps_audio::EngineInput steady_input(real t, real rpm, real p_im) {
    ps_audio::EngineInput in;
    in.t_sim_s = t;
    in.omega_rad_s = rpm * 2.0 * kPi / 60.0;
    in.p_intake_pa = p_im;
    return in;
}

}  // namespace

TEST_CASE("Audio bank B1: every plane is read at the live crank angle, also across its seam", "[audio]") {
    // Catches: playback indexed by sample time, a plane read without its
    // crank0 offset, a seam not at a cycle boundary, a wrong rpm/load blend.
    const AlignResult r = align_run(false);
    std::printf("Audio bank B1: max |out - 0.75 g(crank)| = %.3e of peak (near seams %.3e), seams row0 %d row1 %d\n",
                r.max_err_rel, r.seam_err_rel, r.seams_row0, r.seams_row1);
    CHECK(r.seams_row0 >= 6);  // both load planes of each row cross their seam several times
    CHECK(r.seams_row1 >= 4);
    CHECK(r.max_err_rel < 2e-3);
    CHECK(r.seam_err_rel < 2e-3);
    SECTION("sabotage: the old sample-time playback") {
        const AlignResult s = align_run(true);
        std::printf("Audio bank B1 sabotage (sample-time playback): max err %.3f of peak, near seams %.3f\n", s.max_err_rel,
                    s.seam_err_rel);
        CHECK(s.max_err_rel > 0.1);
    }
}

TEST_CASE("Audio bank B2: a plane's seam is continuous under cycle variation and drift", "[audio]") {
    // Catches: no seam crossfade (a step from the variation / DC drift), a
    // crossfade between different crank angles (level dip).
    const SeamResult r = seam_run(false, false);
    std::printf("Audio bank B2: %d seams, max jump near seam / elsewhere %.3f, seam window AC RMS %.4f x interior min, "
                "%.4f x interior max\n",
                r.seams, r.jump_ratio, r.env_lo, r.env_hi);
    CHECK(r.seams >= 4);
    CHECK(r.jump_ratio < 1.25);
    CHECK(r.env_lo > 0.97);
    CHECK(r.env_hi < 1.03);
    SECTION("sabotage: the old sample-time playback; no crossfade") {
        const SeamResult s = seam_run(true, false);
        std::printf("Audio bank B2 sabotage (sample-time playback): jump ratio %.3f, seam window %.4f x min / %.4f x max\n",
                    s.jump_ratio, s.env_lo, s.env_hi);
        CHECK((s.env_lo < 0.97 || s.env_hi > 1.03));
        const SeamResult x = seam_run(false, true);
        std::printf("Audio bank B2 sabotage (no crossfade): jump ratio %.3f, seam window %.4f x min / %.4f x max\n",
                    x.jump_ratio, x.env_lo, x.env_hi);
        CHECK(x.jump_ratio > 1.25);
    }
}

TEST_CASE("Audio bank B3: no level modulation at the old loop period", "[audio]") {
    // Catches: blended planes whose relative phase changes at each loop wrap
    // (the 40923cf steps: partial cancellation every ~0.5 s).
    const PeriodicResult r = periodic_run(false);
    std::printf("Audio bank B3: envelope sd %.4f dB, component at the old wrap periods %.2e / %.2e\n", r.sd_db, r.comp_row0,
                r.comp_row1);
    CHECK(r.sd_db < 0.02);
    CHECK(r.comp_row0 < 1e-3);
    CHECK(r.comp_row1 < 1e-3);
    SECTION("sabotage: the old sample-time playback") {
        const PeriodicResult s = periodic_run(true);
        std::printf("Audio bank B3 sabotage (sample-time playback): envelope sd %.3f dB, components %.3f / %.3f\n", s.sd_db,
                    s.comp_row0, s.comp_row1);
        CHECK(s.sd_db > 0.5);
        CHECK(std::max(s.comp_row0, s.comp_row1) > 0.02);
    }
}

TEST_CASE("Audio bank B4: PSAB0002 round trip, damage, PSAB0001 and cache key", "[audio][io]") {
    // Catches: a reader accepting an old or damaged file, a cache key that
    // does not move with the generator, an old cache file crashing the
    // generator instead of being a silent miss.
    SynthBankSpec spec;
    spec.channels = 2;
    const auto bank = make_bank(spec);
    const auto dir = std::filesystem::temp_directory_path() / "ps_audio_bank_b4";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "audio");
    const auto file = (dir / "bank.psab").string();
    ps_audio::write_audio_bank(*bank, file);
    const auto read = ps_audio::read_audio_bank(file, 77);
    CHECK(read->samples == bank->samples);
    CHECK(read->crank0_deg == bank->crank0_deg);
    CHECK(read->cycles == bank->cycles);
    CHECK(read->frames_per_cycle == bank->frames_per_cycle);
    CHECK(read->rpms == bank->rpms);
    CHECK_THROWS(ps_audio::read_audio_bank(file, 78));
    {
        std::fstream f(file, std::ios::binary | std::ios::in | std::ios::out);
        f.seekp(-3, std::ios::end);
        f.put('x');
    }
    CHECK_THROWS(ps_audio::read_audio_bank(file, 77));
    std::filesystem::resize_file(file, 100);
    CHECK_THROWS(ps_audio::read_audio_bank(file, 77));

    // A PSAB0001 file is not a PSAB0002 bank; only the reference reads it.
    const auto v1 = (dir / "old.psab").string();
    write_v1_file(v1, 77);
    CHECK_THROWS(ps_audio::read_audio_bank(v1, 77));
    const auto old = ps_audio::read_audio_bank_v1_reference(v1);
    CHECK_FALSE(old->crank_synchronous);
    CHECK(old->plane_frames(3) == 4800);

    // The key moves with the generator settings.
    const std::string engine = std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json";
    const std::uint64_t key = ps_tools::engine_audio_bank_key(engine, "");
    const std::uint64_t key_v1 = ps_tools::engine_audio_bank_key(engine, "", "engine-bank-v1:48000:950,2500,4500,6500,8000:0.12,1:24000:2s");
    std::printf("Audio bank B4: key %016llx (v1 settings %016llx)\n", static_cast<unsigned long long>(key),
                static_cast<unsigned long long>(key_v1));
    CHECK(key != key_v1);
    CHECK(std::string(ps_tools::kEngineAudioBankSettings).rfind("engine-bank-v2", 0) == 0);

    // An old file at the current key's cache path is a silent miss: the
    // generator goes on to generate (here: cancelled before the engine load).
    std::ostringstream hex;
    hex << std::hex << key;
    const auto cached = dir / "audio" / (hex.str() + ".psab");
    write_v1_file(cached.string(), key);
    bool polled = false;
    try {
        (void)ps_tools::prepare_engine_audio_bank(engine, "", dir.string(), [&] { return polled = true; });
        FAIL("an old PSAB0001 cache file was accepted");
    } catch (const std::runtime_error&) {
        // catch by type only (TOOL-019); the poll proves the miss path ran
    }
    CHECK(polled);
    // A valid PSAB0002 file with the key is a hit (no generation, no poll).
    auto hit_bank = std::make_shared<ps_audio::AudioBank>(*bank);
    hit_bank->key = key;
    ps_audio::write_audio_bank(*hit_bank, cached.string());
    polled = false;
    ps_tools::EngineAudioBankStats stats;
    const auto hit = ps_tools::prepare_engine_audio_bank(engine, "", dir.string(), [&] { return polled = true; }, &stats);
    CHECK_FALSE(polled);
    CHECK(stats.cache_hit);
    CHECK(hit->samples == bank->samples);
    std::filesystem::remove_all(dir);
}

TEST_CASE("Audio bank B5: the voice reads the bank at its own engine crank", "[audio][cycle][pipe]") {
    // Catches: the voice feeding the playback anything but the engine's
    // post-step crank angle (or sample time).
    auto run = [](bool reference) {
        ps_audio::VoiceSettings vs = coarse_settings(2000.0, 50000.0);
        vs.bank_sample_time_reference = reference;
        ps_audio::Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
        SynthBankSpec spec;
        spec.channels = v.source_count();
        const auto bank = make_bank(spec);
        v.set_audio_bank(bank);
        const int ns = v.source_count();
        std::vector<float> out(static_cast<std::size_t>(ns));
        double t = 0.0, worst = 0.0, peak = 0.0;
        // rpm 2000 is below row 0: row 0 only, level 1; load (50000-30000)/70000.
        const double load = 20000.0 / 70000.0;
        const double scale = 0.5 * (1.0 - load) + 1.0 * load;
        for (int n = 0; n < 24000; ++n) {
            if (n % 200 == 0) {
                v.push_input(steady_input(t, 2000.0, 50000.0));
                t += 1.0 / 240.0;
            }
            v.render(out.data(), 1);
            if (n < 14400) continue;  // bank fade-in (120 ms) and fuel-on weight settled
            const double crank = v.engine().crank_deg();
            for (int s = 0; s < ns; ++s) {
                const double e = scale * channel_scale(s) * synth_value(spec.synth, crank, 0.0);
                peak = std::max(peak, std::abs(e));
                worst = std::max(worst, std::abs(out[static_cast<std::size_t>(s)] - e));
            }
        }
        return worst / peak;
    };
    const double err = run(false);
    std::printf("Audio bank B5: voice output vs bank at the engine crank: %.3e of peak\n", err);
    CHECK(err < 2e-3);
    SECTION("sabotage: the old sample-time playback") {
        const double s = run(true);
        std::printf("Audio bank B5 sabotage (sample-time playback): %.3f of peak\n", s);
        CHECK(s > 0.1);
    }
}

TEST_CASE("Audio bank B6: rendering with a bank allocates nothing after construction", "[audio][cycle][pipe]") {
    // Catches: a heap allocation per sample/block in the crank playback or the
    // bank swap (crossfade from one bank to the next).
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    ps_audio::Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, coarse_settings(2000.0, 50000.0));
    SynthBankSpec spec;
    spec.channels = v.source_count();
    v.set_audio_bank(make_bank(spec));
    SynthBankSpec spec2 = spec;
    spec2.synth.variation = 0.1;
    const auto second = make_bank(spec2);
    ps_audio::Runner runner({&v});
    std::vector<float> buf(static_cast<std::size_t>(runner.block_frames() * runner.channel_count()));
    real t = 0.0;
    auto blocks = [&](int n) {
        for (int i = 0; i < n; ++i) {
            if (i % 2 == 0) {
                v.push_input(steady_input(t, 2000.0 + 4000.0 * std::fmod(t, 1.0), 50000.0));
                t += 1.0 / 240.0;
            }
            runner.render_block();
            runner.read(buf.data(), static_cast<std::size_t>(runner.block_frames()));
        }
    };
    blocks(20);
    v.set_audio_bank(second);  // control thread; the swap itself happens in render
    ps::jobs::JobSystem js(2);
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit([&] { blocks(60); }, &wg);
        js.wait(wg);
    }
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t n = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    std::printf("Audio bank B6: %llu allocations in 60 blocks with a bank swap\n", static_cast<unsigned long long>(n));
    CHECK(n == 0);
}

TEST_CASE("Audio bank B7: build_crank_loop band-limit and rejections", "[audio]") {
    // Catches: a filter that shifts the crank (non-zero-phase), passes
    // content above the stop band, or a loop spec that is not whole cycles.
    const int fpc = 960, cycles = 4, X = 2880, D = 48;
    const int n = 2 * D + cycles * fpc + X;
    auto tone = [&](double hz) {
        std::vector<float> rec(static_cast<std::size_t>(n));
        for (int j = 0; j < n; ++j) rec[static_cast<std::size_t>(j)] = static_cast<float>(std::cos(2.0 * kPi * hz * j / kRate));
        return rec;
    };
    ps_audio::CrankLoopSpec spec;
    spec.frames_per_cycle = fpc;
    spec.cycles = cycles;
    spec.crossfade_frames = X;
    spec.filter_half_taps = D;
    spec.cutoff_hz = 24000.0 / 2.0 - 1500.0;  // the generator's rule for ratio 2
    // 1 kHz (an integer number of periods per cycle at 6000 rpm: 20) passes,
    // in phase: the loop equals the recording at loop frame = recording frame D.
    ps_audio::CrankLoopStats st;
    const auto pass = ps_audio::build_crank_loop(tone(1000.0), 1, spec, &st);
    double err = 0.0;
    for (int x = 0; x < cycles * fpc; ++x) err = std::max(err, std::abs(pass[static_cast<std::size_t>(x)] - std::cos(2.0 * kPi * 1000.0 * (x + D) / kRate)));
    // 15 kHz (above the 12 kHz stop edge for ratio 2) is removed.
    const auto stop = ps_audio::build_crank_loop(tone(15000.0), 1, spec);
    double stop_peak = 0.0;
    for (float v : stop) stop_peak = std::max(stop_peak, static_cast<double>(std::abs(v)));
    std::printf("Audio bank B7: 1 kHz pass error %.2e, 15 kHz residual %.2e (%.1f dB), removed %.2e\n", err, stop_peak,
                20.0 * std::log10(stop_peak), st.removed_energy_fraction);
    CHECK(err < 1e-3);
    CHECK(stop_peak < 1e-3);
    spec.crossfade_frames = X + 1;
    CHECK_THROWS(ps_audio::build_crank_loop(tone(1000.0), 1, spec));
}
