#include "ps_engine_tools/audio_bank_prep.h"

#include "drivetrain/engine_simulated.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/io/vehicle_io.h"
#include "ps_audio/voice.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace ps_engine_tools {

// Generator identity (cache key). Planes: nominal rows 950/2500/4500/6500/8000
// rpm moved to rpm = 48000*120/round(48000*120/nominal) so one engine cycle is
// a whole number of frames; pedals 0.12/1.0; MVEM settle 1920 x 1/960 s; voice
// warm-up 240 ticks x 200 frames (1 s); loop = max(12, round(1.0 s * rpm/120))
// cycles; seam crossfade = whole cycles covering >= 2400 frames (50 ms);
// band-limit Blackman sinc 97 taps, -6 dB at 24000/ratio - 1500 Hz with ratio =
// next row / this row (top row 1.1); live dx x2, plenum PI 0.015 s / filter
// 0.003 s (the demo voice's settings). UNCHANGED by the move out of tools/common
// (the bank bytes are pinned by tests/unit/test_audio_bank_prep.cpp).
const char* const kEngineAudioBankSettings =
    "engine-bank-v2:48000:950,2500,4500,6500,8000@whole-cycle-frames:0.12,1:mvem1920/960:warm240x200:"
    "loop1.0s-min12:xfade2400-whole-cycles:fir48-blackman-stop24000/ratio-1500-top1.1:dx2:plenum0.015/0.003";

namespace {
constexpr int kRate = 48000;
constexpr int kTickFrames = 200;  // 48000 / 240
constexpr int kWarmTicks = 240;
constexpr int kMinCycles = 12;
constexpr double kLoopSeconds = 1.0;
constexpr int kCrossfadeMinFrames = 2400;
constexpr int kFilterHalfTaps = 48;
constexpr double kTopRowRatio = 1.1;
constexpr double kTransitionHz = 1500.0;
constexpr double kNominalRpm[] = {950, 2500, 4500, 6500, 8000};
constexpr double kPedals[] = {0.12, 1.0};
constexpr int kRows = static_cast<int>(std::size(kNominalRpm));
constexpr int kPlanes = kRows * 2;
constexpr double kTwoPi = 6.2831853071795864769;

constexpr std::uint64_t kFnvBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
void fold_bytes(std::uint64_t& h, const void* data, std::size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) { h ^= p[i]; h *= kFnvPrime; }
}
void fold_double(std::uint64_t& h, double v) { fold_bytes(h, &v, sizeof v); }
void fold_u64(std::uint64_t& h, std::uint64_t v) { fold_bytes(h, &v, sizeof v); }

double wrap720(double d) {
    d = std::fmod(d, 720.0);
    return d < 0.0 ? d + 720.0 : d;
}

// Row geometry: all a pure function of the constants above.
struct RowGeometry {
    int fpc = 0, cycles = 0, crossfade = 0, rec_ticks = 0;
    double rpm = 0.0, ratio = 0.0;
    std::size_t need = 0;
};
RowGeometry row_geometry(int row) {
    auto fpc_of = [](int r) { return static_cast<int>(std::lround(kRate * 120.0 / kNominalRpm[r])); };
    RowGeometry g;
    g.fpc = fpc_of(row);
    g.rpm = kRate * 120.0 / g.fpc;
    g.ratio = row + 1 < kRows ? (kRate * 120.0 / fpc_of(row + 1)) / g.rpm : kTopRowRatio;
    g.cycles = std::max(kMinCycles, static_cast<int>(std::lround(kLoopSeconds * g.rpm / 120.0)));
    g.crossfade = g.fpc * std::max(1, (kCrossfadeMinFrames + g.fpc - 1) / g.fpc);
    g.need = static_cast<std::size_t>(2 * kFilterHalfTaps + g.cycles * g.fpc + g.crossfade);
    g.rec_ticks = static_cast<int>((g.need + kTickFrames - 1) / kTickFrames);
    return g;
}

// ---- cancellation + the shared worker pool ----------------------------------

struct InternalAbort {};  // another item failed or was cancelled: stop quietly

class Control {
public:
    explicit Control(const AudioBankPrepOptions& o) : options_(o) {}
    bool user_cancelled() const {
        return (options_.cancel && options_.cancel->load(std::memory_order_relaxed)) ||
               (options_.cancel_poll && options_.cancel_poll());
    }
    // Called between work items, on any thread.
    void check() {
        if (stop_.load(std::memory_order_relaxed)) throw InternalAbort{};
        if (user_cancelled()) {
            stop_.store(true, std::memory_order_relaxed);
            throw ps::drivetrain::GenerationCancelled();
        }
    }
    void stop() { stop_.store(true, std::memory_order_relaxed); }

private:
    const AudioBankPrepOptions& options_;
    std::atomic<bool> stop_{false};
};

int resolve_workers(int requested, int items) {
    int w = requested;
    if (w <= 0) w = static_cast<int>(std::max(1u, std::thread::hardware_concurrency() / 2));
    return std::max(1, std::min(w, items));
}

// Runs body(item) for item = 0..count-1, dynamically scheduled over `workers` threads (1 = the
// calling thread). With workers > 1 the calling thread only calls `poll` (every ~1 ms and once
// after the join). The first real error by item index is rethrown; if there was none and any item
// was cancelled, GenerationCancelled is thrown.
template <class Body, class Poll>
void run_items(int count, int workers, Control& control, Body&& body, Poll&& poll) {
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(count));
    std::atomic<bool> cancelled{false};
    std::atomic<int> next{0};
    auto work = [&] {
        for (int i = next++; i < count; i = next++) {
            try {
                body(i);
            } catch (const InternalAbort&) {
                return;
            } catch (const ps::drivetrain::GenerationCancelled&) {
                cancelled = true;
                control.stop();
                return;
            } catch (...) {
                errors[static_cast<std::size_t>(i)] = std::current_exception();
                control.stop();
                return;
            }
        }
    };
    if (workers <= 1) {
        work();
    } else {
        std::vector<std::thread> pool;
        std::atomic<int> running{workers};
        for (int w = 0; w < workers; ++w)
            pool.emplace_back([&] {
                work();
                --running;
            });
        while (running.load() > 0) {
            poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for (auto& t : pool) t.join();
    }
    poll();
    for (const auto& e : errors)
        if (e) std::rethrow_exception(e);
    if (cancelled) throw ps::drivetrain::GenerationCancelled();
}

// ---- operating points and planes ---------------------------------------------

struct OperatingPoint {
    double omega = 0.0;
    double pedal = 0.0;
    ps::drivetrain::PreSolveResult pre;
};

// The engine held at the row's rpm and pedal until its mean-value model has settled.
OperatingPoint settle_operating_point(const ps::drivetrain::SimulatedEngineDesc& desc, int row, int load) {
    const RowGeometry g = row_geometry(row);
    OperatingPoint op;
    op.pedal = kPedals[load];
    op.omega = g.rpm * kTwoPi / 60.0;
    ps::drivetrain::SimulatedEngine engine(desc);
    ps::drivetrain::Sensors sensors;
    sensors.omega = op.omega;
    sensors.ignition = 1;
    // Below limiter while baking the high-speed texture; actual live cut flags remain authoritative.
    for (int i = 0; i < 1920; ++i) {
        op.pre = engine.pre_solve(sensors, {op.pedal, 0, 1}, {}, 1.0 / 960);
        (void)engine.post_solve(sensors.omega, 1.0 / 960);
    }
    return op;
}

struct PlaneResult {
    std::vector<float> loop;
    double pressure = 0.0, crank0 = 0.0, removed = 0.0;
    int channels = 0;
};

// One plane: a steady-state recording of the live voice at the row's exact
// rpm, turned into a crank-aligned loop. `tick_done` is called after each render tick.
template <class TickDone>
PlaneResult record_plane(const ps::drivetrain::SimulatedEngineDesc& desc, int row, const OperatingPoint& op, Control& control,
                         TickDone&& tick_done) {
    const RowGeometry g = row_geometry(row);
    const int fpc = g.fpc;
    const double rpm = g.rpm;
    const int cycles = g.cycles;
    const int crossfade = g.crossfade;
    const ps::drivetrain::PreSolveResult& pre = op.pre;

    PlaneResult out;
    out.pressure = pre.p_intake_pa;
    ps_audio::VoiceSettings settings;
    settings.engine.threads = 1;
    settings.engine.initial_rpm = rpm;
    settings.engine.initial_p_intake_pa = pre.p_intake_pa;
    settings.engine.plenum_pi_tau_s = 0.015;
    settings.engine.plenum_filter_tau_s = 0.003;
    auto cycle = *desc.cycle;
    cycle.resolution.audio_dx_intake_m *= 2;
    cycle.resolution.audio_dx_exhaust_m *= 2;
    ps_audio::Voice voice(cycle, desc.fuels[0].lhv_j_kg, desc.fuels[0].afr_stoich, settings);
    const int ch = voice.source_count();
    out.channels = ch;
    ps_audio::EngineInput input;
    input.omega_rad_s = op.omega;
    input.p_intake_pa = pre.p_intake_pa;
    input.p_charge_pa = pre.p_charge_pa;
    input.t_charge_k = pre.t_charge_k;
    input.lambda = pre.lambda;
    input.throttle = pre.throttle_position;
    input.spark_given = true;
    input.spark_deg_atdc = -pre.spark_advance_deg;
    // No crank_angle_deg: the voice runs open-loop at exactly omega, so the
    // crank of every recorded frame follows from the one read after warm-up.
    std::vector<float> block(static_cast<std::size_t>(kTickFrames * ch));
    const int rec_ticks = g.rec_ticks;
    std::vector<float> rec;
    rec.reserve(static_cast<std::size_t>(rec_ticks) * kTickFrames * static_cast<std::size_t>(ch));
    double crank_end_warm = 0.0;
    for (int tick = 0; tick < kWarmTicks + rec_ticks; ++tick) {
        control.check();
        input.t_sim_s = static_cast<double>(tick) / 240.0;
        voice.push_input(input);
        voice.render(block.data(), kTickFrames);
        if (tick >= kWarmTicks) rec.insert(rec.end(), block.begin(), block.end());
        if (tick == kWarmTicks - 1) crank_end_warm = voice.engine().crank_deg();
        tick_done();
    }
    const double deg_per_frame = 720.0 / fpc;
    const double predicted = wrap720(crank_end_warm + static_cast<double>(rec_ticks) * kTickFrames * deg_per_frame);
    double err = std::abs(predicted - voice.engine().crank_deg());
    err = std::min(err, 720.0 - err);
    if (err > 1e-3) throw std::runtime_error("Audio-bank recording crank drifted from omega*t (" + std::to_string(err) + " deg)");
    rec.resize(g.need * static_cast<std::size_t>(ch));
    ps_audio::CrankLoopSpec spec;
    spec.frames_per_cycle = fpc;
    spec.cycles = cycles;
    spec.crossfade_frames = crossfade;
    spec.filter_half_taps = kFilterHalfTaps;
    spec.cutoff_hz = 0.5 * kRate / g.ratio - kTransitionHz;
    spec.rate_hz = kRate;
    ps_audio::CrankLoopStats st;
    out.loop = ps_audio::build_crank_loop(rec, ch, spec, &st);
    out.removed = st.removed_energy_fraction;
    // Loop frame 0 = recording frame kFilterHalfTaps (frame j sits at crank_end_warm + (j+1)*deg_per_frame).
    out.crank0 = wrap720(crank_end_warm + (kFilterHalfTaps + 1) * deg_per_frame);
    return out;
}

// The resolved engine the bake uses: the given description with the forced-induction override.
ps::drivetrain::SimulatedEngineDesc effective_engine(const ps::drivetrain::SimulatedEngineDesc& engine,
                                                     const ps::drivetrain::TurboPairDesc* forced_induction) {
    if (engine.maps_pending) throw std::runtime_error("Audio bank requires complete engine maps (maps_pending)");
    if (!engine.cycle) throw std::runtime_error("Audio bank requires a simulated cycle engine");
    if (engine.fuels.empty()) throw std::runtime_error("Audio bank requires an engine with a fuel");
    // Engine garage P9: a cam-switching engine has two cylinder models; the bank records one engine. Refused until the bank
    // grows a profile axis (open point G10) - silently baking the low cam would mis-voice everything above the switch.
    if (engine.has_cam_profiles()) throw std::runtime_error("Audio bank does not support an engine with cam profiles yet (engine garage P9, open point G10)");
    // Engine garage P10: likewise a continuous intake lift family (one cylinder model per lift node; the bank has no u axis).
    if (engine.has_lift_family()) throw std::runtime_error("Audio bank does not support an engine with an intake lift family yet (engine garage P10, open point G10)");
    auto desc = engine;
    if (forced_induction) desc.turbo_pair = *forced_induction;
    return desc;
}

std::vector<OperatingPoint> settle_all(const ps::drivetrain::SimulatedEngineDesc& desc, int workers, Control& control) {
    std::vector<OperatingPoint> points(static_cast<std::size_t>(kPlanes));
    run_items(
        kPlanes, resolve_workers(workers, kPlanes), control,
        [&](int p) {
            control.check();
            points[static_cast<std::size_t>(p)] = settle_operating_point(desc, p / 2, p % 2);
        },
        [] {});
    return points;
}

// FNV-1a over: the key scheme tag and generator identity, engine_map_cache_key of the cycle block
// with its first fuel (the voice's whole acoustic/geometric input), and the operating points.
std::uint64_t compute_key(const ps::drivetrain::SimulatedEngineDesc& desc, const std::vector<OperatingPoint>& points) {
    std::uint64_t key = kFnvBasis;
    static const char tag[] = "ps-engine-tools/audio-bank-key/2:";
    fold_bytes(key, tag, sizeof tag - 1);
    fold_bytes(key, kEngineAudioBankSettings, std::char_traits<char>::length(kEngineAudioBankSettings));
    // engine_map_cache_key also folds two free-text strings (the cycle block's `source` and the fuel's
    // name); they cannot reach the sound, so they are blanked here (a citation edit must not trigger a rebake).
    auto cycle = *desc.cycle;
    cycle.source.clear();
    auto fuel = desc.fuels[0];
    fuel.name.clear();
    fold_u64(key, ps::drivetrain::engine_map_cache_key(cycle, fuel, ps::drivetrain::MapQuality::Full));
    for (const auto& op : points) {
        fold_double(key, op.pre.p_intake_pa);
        fold_double(key, op.pre.p_charge_pa);
        fold_double(key, op.pre.t_charge_k);
        fold_double(key, op.pre.lambda);
        fold_double(key, op.pre.throttle_position);
        fold_double(key, op.pre.spark_advance_deg);
    }
    return key;
}
}  // namespace

std::string audio_bank_cache_path(const std::string& cache_dir, std::uint64_t key) {
    char hex[17];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(key));
    return (std::filesystem::path(cache_dir) / "audio" / (std::string(hex) + ".psab")).string();
}

std::uint64_t engine_audio_bank_key(const ps::drivetrain::SimulatedEngineDesc& engine,
                                    const ps::drivetrain::TurboPairDesc* forced_induction, const AudioBankPrepOptions& options) {
    const auto desc = effective_engine(engine, forced_induction);
    Control control(options);
    control.check();
    return compute_key(desc, settle_all(desc, options.workers, control));
}

AudioBankPrepResult prepare_engine_audio_bank(const ps::drivetrain::SimulatedEngineDesc& engine,
                                              const ps::drivetrain::TurboPairDesc* forced_induction,
                                              const AudioBankPrepOptions& options) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto desc = effective_engine(engine, forced_induction);
    Control control(options);
    control.check();
    const auto points = settle_all(desc, options.workers, control);
    AudioBankPrepResult result;
    result.key = compute_key(desc, points);
    if (!options.cache_dir.empty()) result.path = audio_bank_cache_path(options.cache_dir, result.key);
    auto finish = [&](ps_audio::AudioBankPtr bank, bool hit) {
        result.bank = std::move(bank);
        result.cache_hit = hit;
        result.bytes = result.bank->samples.size() * sizeof(float);
        result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return result;
    };
    // Any unreadable file (PSAB0001, another key, damage) is a silent miss.
    if (!result.path.empty()) {
        try {
            return finish(ps_audio::read_audio_bank(result.path, result.key), true);
        } catch (...) {
        }
    }

    std::size_t total = 0;
    for (int p = 0; p < kPlanes; ++p) total += static_cast<std::size_t>(kWarmTicks + row_geometry(p / 2).rec_ticks);
    std::atomic<std::size_t> done{0};
    std::size_t reported = 0;
    auto deliver = [&] {
        if (!options.on_progress) return;
        const std::size_t d = done.load(std::memory_order_acquire);
        while (reported < d) options.on_progress(++reported, total);
    };
    const int workers = resolve_workers(options.workers, kPlanes);
    std::vector<PlaneResult> results(static_cast<std::size_t>(kPlanes));
    run_items(
        kPlanes, workers, control,
        [&](int p) {
            results[static_cast<std::size_t>(p)] = record_plane(desc, p / 2, points[static_cast<std::size_t>(p)], control, [&] {
                done.fetch_add(1, std::memory_order_release);
                if (workers <= 1) deliver();  // this IS the calling thread
            });
        },
        deliver);
    control.check();  // the last chance to cancel: nothing has been written yet

    auto bank = std::make_shared<ps_audio::AudioBank>();
    bank->key = result.key;
    bank->rate = kRate;
    bank->channels = results[0].channels;
    for (int r = 0; r < kRows; ++r) {
        const RowGeometry g = row_geometry(r);
        bank->rpms.push_back(kRate * 120.0 / g.fpc);
        bank->frames_per_cycle.push_back(g.fpc);
        bank->cycles.push_back(static_cast<std::int32_t>(results[static_cast<std::size_t>(r * 2)].loop.size() /
                                                          static_cast<std::size_t>(g.fpc * bank->channels)));
    }
    for (const auto& plane : results) {
        if (plane.channels != bank->channels) throw std::runtime_error("Audio bank channel topology changed");
        bank->pressures.push_back(plane.pressure);
        bank->crank0_deg.push_back(plane.crank0);
        bank->samples.insert(bank->samples.end(), plane.loop.begin(), plane.loop.end());
        result.removed_energy_fraction.push_back(plane.removed);
    }
    bank->finalize();
    if (!result.path.empty()) {
        const std::filesystem::path file(result.path);
        std::filesystem::create_directories(file.parent_path());
        std::ostringstream thread;
        thread << std::this_thread::get_id();
        const auto temp = file.string() + "." + thread.str() + ".tmp";
        std::error_code error;
        try {
            ps_audio::write_audio_bank(*bank, temp);
            std::filesystem::rename(temp, file, error);
            if (error) {
                std::filesystem::remove(file, error);
                error.clear();
                std::filesystem::rename(temp, file, error);
            }
        } catch (...) {
            std::filesystem::remove(temp, error);
            throw;
        }
        if (error) {
            const std::string message = error.message();
            std::filesystem::remove(temp, error);
            throw std::runtime_error("Cannot publish generated audio bank: " + message);
        }
    }
    return finish(std::move(bank), false);
}

AudioBankPrepResult prepare_engine_audio_bank_from_files(const std::string& engine_path, const std::string& turbo_configuration_path,
                                                         const AudioBankPrepOptions& options) {
    ps::io::EngineMapOptions map_options;
    map_options.cache_dir = options.cache_dir;
    map_options.cancel = options.cancel;
    auto loaded = ps::io::load_engine_json(engine_path, map_options);
    const auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&loaded);
    if (!sim) throw std::runtime_error("Audio bank requires a simulated cycle engine: " + engine_path);
    if (turbo_configuration_path.empty()) return prepare_engine_audio_bank(*sim, nullptr, options);
    const auto turbo = ps::io::load_turbo_configuration_json(turbo_configuration_path, options.cache_dir);
    return prepare_engine_audio_bank(*sim, &turbo, options);
}
}  // namespace ps_engine_tools
