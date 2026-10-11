// PRE-MOVE PIN (engine garage P6): bank content of the pre-move generator.
#include "engine_bank_generator.h"

#include "ps_audio/audio_bank.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

namespace {
std::uint64_t fnv(std::uint64_t h, const void* data, std::size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}
template <class V> std::uint64_t fnv_vec(std::uint64_t h, const V& v) {
    return fnv(h, v.data(), v.size() * sizeof(typename V::value_type));
}
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
}  // namespace

TEST_CASE("Audio bank prep P0: pre-move pin", "[audio][engine_tools][long]") {
    const std::string engine = std::string(PS_SOURCE_DIR) + "/data/engines/demo_i4_pipes.json";
    const auto maps = std::filesystem::temp_directory_path() / "ps_audio_bank_prep_p0";
    std::filesystem::remove_all(maps / "audio");
    ps_tools::EngineAudioBankStats stats;
    const auto bank = ps_tools::prepare_engine_audio_bank(engine, "", maps.string(), {}, &stats);
    // Recorded on the pre-move generator (tools/common/engine_bank_generator.cpp, master 131f3b3):
    // key ef8ced1e30d5013d, content 1d178f47cb870602, 4253824 bytes (release, clang-cl, 4.7 s on this machine).
    CHECK(bank_content_hash(*bank) == 0x1d178f47cb870602ULL);
    CHECK(ps_tools::engine_audio_bank_key(engine, "") == 0xef8ced1e30d5013dULL);
    std::printf("P0 key %016llx content %016llx bytes %zu seconds %.2f\n",
                static_cast<unsigned long long>(ps_tools::engine_audio_bank_key(engine, "")),
                static_cast<unsigned long long>(bank_content_hash(*bank)), stats.bytes, stats.seconds);
    CHECK(bank->channels > 0);
}
