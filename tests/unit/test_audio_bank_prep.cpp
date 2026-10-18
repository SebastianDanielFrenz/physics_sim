// Engine garage P6 (design A14): ps_engine_tools::prepare_engine_audio_bank - the audio bank baker as a
// library (engine_tools/), with a disk cache, cancel and progress.
//
// Fixtures. The quick cases use a SYNTHETIC engine: tests/support/mvem_fixtures.h's electronic MVEM engine
// carrying tests/support/cycle_pipe_fixtures.h's pipe I4 cycle block (the baker reads the MVEM engine only
// for the 10 operating points and the cycle block only for the voice, so any consistent pair works; no map
// generation). The [long] cases bake the shipped demo_i4_pipes and the turbo hyper V8 from their files (maps
// generated once into a shared temp cache).
//
// Pins. P1/P1b: the content hash of the baked bank (rate, channels, rows, pressures, frames per cycle,
// cycles, crank origins, every sample) recorded on the PRE-MOVE generator (tools/common/
// engine_bank_generator.cpp at master 131f3b3, two commits before the move): demo_i4_pipes
// 1d178f47cb870602 (4253824 bytes), hyper_v8_one1_audio 79cc5e377994856c (6380736 bytes). The cache KEY is
// deliberately not pinned against the old value: it changed (P6 point 3, see audio_bank_prep.h) - the old
// keys were ef8ced1e30d5013d / 4e8cb3985c3f746c.
// Engine garage P7d re-pinned demo_i4_pipes to 1b0b7ef18220095b (was 1d178f47cb870602, same 4253824 bytes; key
// 4c6a6ffa5db5cc23 -> 4109ceae432c96b0): its generated maps gained the wide-open-throttle top row (generator v4), and
// the bank records the runtime engine on those maps. The hyper (compressor-fed, generator v3) is unchanged.
//
// Each guard has a recorded sabotage (out/garage_p6_progress.md).
#include "cycle_pipe_fixtures.h"
#include "mvem_fixtures.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps_engine_tools/audio_bank_prep.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace tools = ps_engine_tools;
namespace dt = ps::drivetrain;
namespace fs = std::filesystem;

std::uint64_t fnv(std::uint64_t h, const void* data, std::size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}
template <class V> std::uint64_t fnv_vec(std::uint64_t h, const V& v) {
    return fnv(h, v.data(), v.size() * sizeof(typename V::value_type));
}
// Everything of a bank except its cache key.
std::uint64_t bank_content_hash(const ps_audio::AudioBank& b) {
    std::uint64_t h = 14695981039346656037ULL;
    h = fnv(h, &b.rate, sizeof b.rate);
    h = fnv(h, &b.channels, sizeof b.channels);
    h = fnv_vec(h, b.rpms);
    h = fnv_vec(h, b.pressures);
    h = fnv_vec(h, b.frames_per_cycle);
    h = fnv_vec(h, b.cycles);
    h = fnv_vec(h, b.crank0_deg);
    h = fnv_vec(h, b.samples);
    return h;
}

fs::path fresh_dir(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / ("ps_audio_bank_prep_" + name);
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}
// Maps of the shipped engines are generated once into this directory and reused by later runs.
fs::path shared_maps_dir() {
    const fs::path dir = fs::temp_directory_path() / "ps_audio_bank_prep_maps";
    fs::create_directories(dir);
    return dir;
}
std::size_t entries_under(const fs::path& dir) {
    std::size_t n = 0;
    if (!fs::exists(dir)) return 0;
    for (const auto& e : fs::recursive_directory_iterator(dir)) { (void)e; ++n; }
    return n;
}
std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), {});
}

// The synthetic engine of the quick cases.
dt::SimulatedEngineDesc fixture_engine() {
    dt::SimulatedEngineDesc d = ps::test::mvem::electronic_engine();
    d.cycle = ps::test::make_pipe_i4_cycle_desc();
    return d;
}

dt::SimulatedEngineDesc load_sim(const std::string& relative, const ps::io::EngineSlotOverrides& slots = {}) {
    ps::io::EngineMapOptions options;
    static ps::jobs::JobSystem pool(10);
    options.pool = &pool;
    options.cache_dir = shared_maps_dir().string();
    const auto params = ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/" + relative, options, nullptr, slots);
    return std::get<dt::SimulatedEngineDesc>(params);
}

// The progress total, derived from the generator's published constants (audio_bank_prep.cpp: rows at
// 950/2500/4500/6500/8000 rpm snapped to whole-cycle frame counts at 48 kHz, 240 warm-up ticks of 200
// frames, loop = max(12, round(rpm/120)) cycles, crossfade = whole cycles >= 2400 frames, 2 x 48 filter
// frames, ceil(need / 200) recording ticks) - an independent re-derivation, not the library's function.
std::size_t expected_total_ticks() {
    const double nominal[] = {950, 2500, 4500, 6500, 8000};
    std::size_t total = 0;
    for (double n : nominal) {
        const int fpc = static_cast<int>(std::lround(48000.0 * 120.0 / n));
        const double rpm = 48000.0 * 120.0 / fpc;
        const int cycles = std::max(12, static_cast<int>(std::lround(rpm / 120.0)));
        const int crossfade = fpc * std::max(1, (2400 + fpc - 1) / fpc);
        const std::size_t need = static_cast<std::size_t>(96 + cycles * fpc + crossfade);
        total += 2 * (240 + (need + 199) / 200);
    }
    return total;
}

// Records every on_progress call: values, the (constant) total, and whether each ran on the thread that made
// the prepare call.
struct ProgressLog {
    std::vector<std::size_t> done;
    std::size_t total = 0;
    std::thread::id caller = std::this_thread::get_id();
    std::atomic<bool> all_on_caller{true};
    bool total_constant = true;
    std::size_t cancel_at = 0;  // set the flag once this many calls arrived (0 = never)
    std::atomic<bool>* flag = nullptr;
    std::function<void(std::size_t, std::size_t)> hook() {
        return [this](std::size_t d, std::size_t t) {
            if (std::this_thread::get_id() != caller) all_on_caller = false;
            if (!done.empty() && t != total) total_constant = false;
            total = t;
            done.push_back(d);
            if (flag && cancel_at != 0 && d >= cancel_at) *flag = true;
        };
    }
    bool strictly_by_one() const {
        for (std::size_t i = 0; i < done.size(); ++i)
            if (done[i] != i + 1) return false;
        return true;
    }
};

// A minimal valid PSAB0002 bank (one 8000 rpm row, two 720-frame planes) under `key`.
ps_audio::AudioBank small_bank(std::uint64_t key) {
    ps_audio::AudioBank b;
    b.rate = 48000;
    b.channels = 1;
    b.key = key;
    b.rpms = {8000.0};
    b.pressures = {30000.0, 60000.0};
    b.frames_per_cycle = {720};
    b.cycles = {1};
    b.crank0_deg = {0.0, 0.0};
    b.samples.assign(2 * 720, 0.25f);
    b.finalize();
    return b;
}

std::uint64_t key_of(const dt::SimulatedEngineDesc& d, const dt::TurboPairDesc* fi = nullptr, int workers = 0) {
    tools::AudioBankPrepOptions o;
    o.workers = workers;
    return tools::engine_audio_bank_key(d, fi, o);
}
}  // namespace

TEST_CASE("Audio bank prep P1: demo_i4_pipes bank equals the pre-move generator; progress; cache hit; 1 vs 10 workers",
          "[audio][engine_tools][long]") {
    // Catches: any change of the baked bytes by the move out of tools/common (content pin), a worker-count dependence,
    // progress that is not 1..total on the calling thread, a cache hit that regenerates or reports progress.
    const auto desc = load_sim("data/engines/demo_i4_pipes.json");
    const auto dir10 = fresh_dir("p1_w10");
    const auto dir1 = fresh_dir("p1_w1");

    ProgressLog log10;
    tools::AudioBankPrepOptions o10;
    o10.cache_dir = dir10.string();
    o10.workers = 10;
    o10.on_progress = log10.hook();
    const auto r10 = tools::prepare_engine_audio_bank(desc, nullptr, o10);
    std::printf("P1: 10 workers %.2f s, key %016llx, content %016llx, %zu bytes, %zu progress calls (total %zu)\n", r10.seconds,
                static_cast<unsigned long long>(r10.key), static_cast<unsigned long long>(bank_content_hash(*r10.bank)), r10.bytes,
                log10.done.size(), log10.total);
    CHECK_FALSE(r10.cache_hit);
    CHECK(bank_content_hash(*r10.bank) == 0xc983bd04834bc403ULL /* re-pinned 2026-10-19, cause: knock model v3, was 0x5611596ff3be5702; 2026-10-18: knock opt-in of demo_i4_pipes, was 0x1b0b7ef18220095b */);  // pre-move generator; P7d maps (was 0x1d178f47cb870602)
    CHECK(r10.bytes == 4253824);
    CHECK(r10.bank->key == r10.key);
    CHECK(r10.path == tools::audio_bank_cache_path(dir10.string(), r10.key));
    CHECK(fs::exists(r10.path));
    CHECK(log10.total == expected_total_ticks());
    CHECK(log10.done.size() == log10.total);
    CHECK(log10.strictly_by_one());
    CHECK(log10.total_constant);
    CHECK(log10.all_on_caller);
    CHECK(r10.removed_energy_fraction.size() == 10);

    // The key function agrees with what the bake used.
    CHECK(key_of(desc) == r10.key);

    // A second call is a cache hit: same bank, no progress, much faster.
    ProgressLog hit_log;
    auto o_hit = o10;
    o_hit.on_progress = hit_log.hook();
    const auto hit = tools::prepare_engine_audio_bank(desc, nullptr, o_hit);
    std::printf("P1: cache hit %.3f s\n", hit.seconds);
    CHECK(hit.cache_hit);
    CHECK(hit.key == r10.key);
    CHECK(hit_log.done.empty());
    CHECK(hit.bank->samples == r10.bank->samples);
    CHECK(bank_content_hash(*hit.bank) == bank_content_hash(*r10.bank));
    CHECK(hit.seconds < 0.5 * r10.seconds);

    // One worker (the calling thread renders everything): the same bytes, the same progress sequence.
    ProgressLog log1;
    tools::AudioBankPrepOptions o1;
    o1.cache_dir = dir1.string();
    o1.workers = 1;
    o1.on_progress = log1.hook();
    const auto r1 = tools::prepare_engine_audio_bank(desc, nullptr, o1);
    std::printf("P1: 1 worker %.2f s\n", r1.seconds);
    CHECK_FALSE(r1.cache_hit);
    CHECK(r1.key == r10.key);
    CHECK(bank_content_hash(*r1.bank) == 0xc983bd04834bc403ULL /* re-pinned 2026-10-19, cause: knock model v3, was 0x5611596ff3be5702; 2026-10-18: knock opt-in of demo_i4_pipes, was 0x1b0b7ef18220095b */);
    CHECK(read_file(r1.path) == read_file(r10.path));  // the cache files are byte-identical
    CHECK(log1.done.size() == log10.done.size());
    CHECK(log1.strictly_by_one());
    CHECK(log1.all_on_caller);
}

TEST_CASE("Audio bank prep P1b: turbo hyper V8 bank equals the pre-move generator, from files", "[audio][engine_tools][long]") {
    // Catches: the forced-induction path (engine default and explicit file) changing the bytes or the key.
    const std::string root = std::string(PS_SOURCE_DIR) + "/";
    const auto dir = shared_maps_dir();
    fs::remove_all(dir / "audio");
    tools::AudioBankPrepOptions o;
    o.cache_dir = dir.string();
    const auto r = tools::prepare_engine_audio_bank_from_files(root + "data/engines/hyper_v8_one1_audio.json", "", o);
    std::printf("P1b: %.2f s, key %016llx, content %016llx, %zu bytes\n", r.seconds, static_cast<unsigned long long>(r.key),
                static_cast<unsigned long long>(bank_content_hash(*r.bank)), r.bytes);
    CHECK_FALSE(r.cache_hit);
    CHECK(bank_content_hash(*r.bank) == 0xaed7766ad5d31620ULL /* re-pinned 2026-10-19, cause: knock model v3, was 0xe1cd3181553397a5; 2026-10-18: knock opt-in + E85 fuel of the hyper; was 0x56711496c481c4a7 (knock), 0x79cc5e377994856c (before) */);  // pre-move generator
    CHECK(r.bytes == 6380736);
    // The engine's default forced-induction file given explicitly is the same engine: same key, a hit.
    const auto again = tools::prepare_engine_audio_bank_from_files(
        root + "data/engines/hyper_v8_one1_audio.json", root + "data/turbo_configurations/hyper_twin_default.json", o);
    CHECK(again.cache_hit);
    CHECK(again.key == r.key);
}

TEST_CASE("Audio bank prep P2: cancel throws GenerationCancelled and leaves nothing on disk", "[audio][engine_tools]") {
    // Catches: a cancel that is ignored, a cancel that leaves a cache or temp file, a cancel reported as another
    // error, progress off the calling thread.
    const auto desc = fixture_engine();

    SECTION("flag already set: nothing runs") {
        const auto dir = fresh_dir("p2_pre");
        std::atomic<bool> flag{true};
        ProgressLog log;
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.cancel = &flag;
        o.on_progress = log.hook();
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
        CHECK(log.done.empty());
        CHECK(entries_under(dir) == 0);
    }
    SECTION("one worker: cancelled from the progress callback, at the next tick") {
        const auto dir = fresh_dir("p2_w1");
        std::atomic<bool> flag{false};
        ProgressLog log;
        log.flag = &flag;
        log.cancel_at = 100;
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.workers = 1;
        o.cancel = &flag;
        o.on_progress = log.hook();
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
        CHECK(log.done.size() == 100);  // the tick after call 100 observes the flag
        CHECK(log.strictly_by_one());
        CHECK(entries_under(dir) == 0);
    }
    SECTION("four workers: cancelled from the progress callback") {
        const auto dir = fresh_dir("p2_w4");
        std::atomic<bool> flag{false};
        ProgressLog log;
        log.flag = &flag;
        log.cancel_at = 60;
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.workers = 4;
        o.cancel = &flag;
        o.on_progress = log.hook();
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
        CHECK(log.done.size() >= 60);
        CHECK(log.done.size() < 2000);  // far from the ~5400 ticks of a full bake
        CHECK(log.strictly_by_one());
        CHECK(log.all_on_caller);
        CHECK(entries_under(dir) == 0);
    }
    SECTION("cancel_poll, polled from the workers") {
        const auto dir = fresh_dir("p2_poll");
        std::atomic<int> polls{0};
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.workers = 3;
        o.cancel_poll = [&] { return ++polls > 400; };
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
        CHECK(polls.load() > 400);
        CHECK(entries_under(dir) == 0);
    }
    SECTION("no cache_dir: cancel is the same, no disk access") {
        std::atomic<bool> flag{false};
        ProgressLog log;
        log.flag = &flag;
        log.cancel_at = 10;
        tools::AudioBankPrepOptions o;
        o.workers = 2;
        o.cancel = &flag;
        o.on_progress = log.hook();
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
    }
    SECTION("an unusable engine is a runtime_error, not a cancellation") {
        auto no_cycle = desc;
        no_cycle.cycle.reset();
        bool cancelled_type = false;
        bool runtime = false;
        try {
            (void)tools::prepare_engine_audio_bank(no_cycle, nullptr, {});
        } catch (const dt::GenerationCancelled&) {
            cancelled_type = true;
        } catch (const std::runtime_error&) {
            runtime = true;
        }
        CHECK_FALSE(cancelled_type);
        CHECK(runtime);
        auto pending = desc;
        pending.maps_pending = true;
        CHECK_THROWS_AS(tools::prepare_engine_audio_bank(pending, nullptr, {}), std::runtime_error);
    }
}

TEST_CASE("Audio bank prep P3: a cancel after the last tick still publishes nothing", "[audio][engine_tools][long]") {
    // Catches: the final cancel check before the cache file is written (the flag is a request up to the
    // moment of publication).
    const auto desc = fixture_engine();
    const auto dir = fresh_dir("p3");
    std::atomic<bool> flag{false};
    ProgressLog log;
    log.flag = &flag;
    log.cancel_at = expected_total_ticks();  // set by the very last progress call
    tools::AudioBankPrepOptions o;
    o.cache_dir = dir.string();
    o.workers = 10;
    o.cancel = &flag;
    o.on_progress = log.hook();
    CHECK_THROWS_AS(tools::prepare_engine_audio_bank(desc, nullptr, o), dt::GenerationCancelled);
    CHECK(log.done.size() == expected_total_ticks());  // every tick ran
    CHECK(entries_under(dir) == 0);
}

TEST_CASE("Audio bank prep P4: cache files - hit, damaged, wrong key, old format", "[audio][engine_tools]") {
    // Catches: a valid file ignored, an invalid one accepted (or crashing the call instead of a silent miss).
    const auto desc = fixture_engine();
    const std::uint64_t key = key_of(desc);
    const auto dir = fresh_dir("p4");
    const std::string path = tools::audio_bank_cache_path(dir.string(), key);
    fs::create_directories(fs::path(path).parent_path());

    auto bake_started = [&](std::atomic<bool>& flag) {
        // A miss starts generating; the first progress call cancels it again (the point is the miss).
        flag = false;
        ProgressLog log;
        log.flag = &flag;
        log.cancel_at = 1;
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.workers = 1;
        o.cancel = &flag;
        o.on_progress = log.hook();
        bool cancelled = false;
        try {
            (void)tools::prepare_engine_audio_bank(desc, nullptr, o);
        } catch (const dt::GenerationCancelled&) {
            cancelled = true;
        }
        return cancelled && !log.done.empty();
    };
    std::atomic<bool> flag{false};

    {  // an old PSAB0001-looking file
        std::ofstream out(path, std::ios::binary);
        out << "PSAB0001" << std::string(200, 'x');
    }
    CHECK(bake_started(flag));
    {  // a valid bank stored under another key
        ps_audio::write_audio_bank(small_bank(key + 1), path);
    }
    CHECK(bake_started(flag));
    {  // a valid bank, damaged (truncated)
        ps_audio::write_audio_bank(small_bank(key), path);
        fs::resize_file(path, fs::file_size(path) - 7);
    }
    CHECK(bake_started(flag));
    {  // a valid bank under the right key: a hit, no bake, no progress
        ps_audio::write_audio_bank(small_bank(key), path);
        ProgressLog log;
        tools::AudioBankPrepOptions o;
        o.cache_dir = dir.string();
        o.on_progress = log.hook();
        const auto hit = tools::prepare_engine_audio_bank(desc, nullptr, o);
        CHECK(hit.cache_hit);
        CHECK(hit.key == key);
        CHECK(log.done.empty());
        CHECK(hit.bank->samples == small_bank(key).samples);
        CHECK(hit.path == path);
    }
}

TEST_CASE("Audio bank prep P5: the key follows what reaches the sound", "[audio][engine_tools]") {
    // Catches: a key blind to an ECU / map / throttle / cycle / fuel change (stale bank served), a key that moves
    // with free text, source positions or the worker count (needless 30 s rebakes).
    const auto base = fixture_engine();
    const std::uint64_t k0 = key_of(base);
    CHECK(key_of(base, nullptr, 1) == k0);  // worker count
    CHECK(key_of(base, nullptr, 4) == k0);

    SECTION("moves") {
        std::vector<std::pair<const char*, dt::SimulatedEngineDesc>> edits;
        auto add = [&](const char* name, auto&& edit) {
            auto d = base;
            edit(d);
            edits.emplace_back(name, std::move(d));
        };
        add("ecu spark", [](auto& d) { for (auto& v : d.ecu.spark_advance_deg.values) v += 3.0; });
        add("ecu lambda", [](auto& d) { for (auto& v : d.ecu.lambda_target.values) v *= 0.9; });
        add("maps ve", [](auto& d) { for (auto& v : d.maps.ve.values) v *= 1.05; });
        add("throttle body", [](auto& d) { d.throttle_body.diameter_m *= 1.2; });
        add("manifold volume", [](auto& d) { d.intake_manifold_volume_m3 *= 1.5; });
        add("cycle audio dx", [](auto& d) { d.cycle->resolution.audio_dx_exhaust_m *= 1.25; });
        add("cycle compression", [](auto& d) { d.cycle->compression_ratio += 0.5; });
        add("fuel lhv", [](auto& d) { d.fuels[0].lhv_j_kg *= 1.02; });
        std::vector<std::uint64_t> keys{k0};
        for (const auto& [name, d] : edits) {
            const std::uint64_t k = key_of(d);
            std::printf("P5: %-18s key %016llx\n", name, static_cast<unsigned long long>(k));
            for (std::uint64_t seen : keys) CHECK(k != seen);
            keys.push_back(k);
        }
    }
    SECTION("does not move") {
        auto d = base;
        d.cycle->source += " (edited text)";
        d.mass_source = "free text";
        d.mass_kg = 123.0;
        d.maps.source += " (edited text)";
        d.fuels[0].name += " (edited)";
        d.fuels[0].source += " (edited)";
        d.cycle->default_installation_path = "somewhere/else.json";
        d.cycle->installation_path = "somewhere/else.json";
        d.cycle->installation->exhaust.elements.front().position_m = ps::Vec3{1.0, 2.0, 3.0};
        CHECK(key_of(d) == k0);
    }
}

TEST_CASE("Audio bank prep P6: the key covers the ECU part file and the forced-induction file", "[audio][engine_tools][long]") {
    // Catches: a part file (engine garage P2 slot) whose content never reaches the key; free text in a part file
    // that does. Hyper V8 through the real loader with a slot override, as the garage does it.
    const std::string root = std::string(PS_SOURCE_DIR) + "/";
    const auto tmp = fresh_dir("p6");
    const std::string cache = shared_maps_dir().string();

    const auto base = load_sim("data/engines/hyper_v8_one1_audio.json");
    const std::uint64_t k0 = key_of(base);
    std::printf("P6: base key %016llx\n", static_cast<unsigned long long>(k0));

    auto ecu_variant = [&](const std::string& name, auto&& edit) {
        std::ifstream in(root + "data/ecus/hyper_v8_one1.json");
        auto j = nlohmann::json::parse(in);
        edit(j);
        const std::string path = (tmp / (name + ".json")).string();
        std::ofstream(path) << j.dump(2);
        return load_sim("data/engines/hyper_v8_one1_audio.json", {{"ecu", path}});
    };
    // The ECU's lambda table scaled: reaches the sound through the enrichment at the boosted operating points.
    const auto leaner = ecu_variant("ecu_lambda", [](nlohmann::json& j) {
        for (auto& row : j["ecu"]["lambda_target"]["values"])
            for (auto& v : row) v = v.get<double>() * 0.95;
    });
    CHECK(key_of(leaner) != k0);
    // The same file with only its free text changed: the same bank.
    const auto retitled = ecu_variant("ecu_text", [](nlohmann::json& j) { j["source"] = "only the text differs"; });
    CHECK(key_of(retitled) == k0);

    // The forced-induction file: another configuration moves the key, the engine's own default file does not.
    const auto n2o = ps::io::load_turbo_configuration_json(root + "data/turbo_configurations/hyper_twin_n2o.json", cache);
    const auto twin = ps::io::load_turbo_configuration_json(root + "data/turbo_configurations/hyper_twin_default.json", cache);
    CHECK(key_of(base, &n2o) != k0);
    CHECK(key_of(base, &twin) == k0);
}
