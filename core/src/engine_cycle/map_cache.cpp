#include "map_cache.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

namespace ps::drivetrain {

namespace {

// --- FNV-1a folding for the cache KEY (mirrors engine_simulated.cpp's and
// map_generator.cpp's own file-local Fnv helper - no shared header for it
// exists in this codebase yet, and adding one is out of C16b's scope). ---
struct KeyFnv {
    std::uint64_t h;
    template <typename T>
    void pod(const T& v) {
        unsigned char bytes[sizeof(T)];
        std::memcpy(bytes, &v, sizeof(T));
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    void vec(const std::vector<real>& v) {
        pod(static_cast<std::uint64_t>(v.size()));
        for (real x : v) pod(x);
    }
    void curve(const vehicle::Curve1D& c) {
        vec(c.x);
        vec(c.y);
    }
    void table(const Table2D& t) {
        vec(t.x);
        vec(t.y);
        vec(t.values);
    }
    void str(const std::string& s) {
        pod(static_cast<std::uint64_t>(s.size()));
        for (char c : s) pod(c);
    }
};

// --- Plain little-endian byte-buffer (de)serialisation for the PAYLOAD. ---
// A payload mismatch (wrong size, truncated) is always caught by the
// caller's re-computed content_hash check, not by anything in here - these
// helpers never throw; a malformed/short buffer just reads zeros/garbage
// that the hash check then rejects (H2).
struct Writer {
    std::vector<unsigned char> buf;
    template <typename T>
    void pod(const T& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
        buf.insert(buf.end(), bytes, bytes + sizeof(T));
    }
    void vec(const std::vector<real>& v) {
        pod(static_cast<std::uint64_t>(v.size()));
        for (real x : v) pod(x);
    }
    void curve(const vehicle::Curve1D& c) {
        vec(c.x);
        vec(c.y);
    }
    void table(const Table2D& t) {
        vec(t.x);
        vec(t.y);
        vec(t.values);
    }
    void str(const std::string& s) {
        pod(static_cast<std::uint64_t>(s.size()));
        buf.insert(buf.end(), s.begin(), s.end());
    }
};

// Returns false (leaving *out untouched-or-partial) on ANY underrun - the
// caller never trusts a Reader's output without the content_hash check.
class Reader {
public:
    Reader(const unsigned char* data, std::size_t size) : data_(data), size_(size) {}

    template <typename T>
    [[nodiscard]] bool pod(T& v) {
        if (pos_ + sizeof(T) > size_) return false;
        std::memcpy(&v, data_ + pos_, sizeof(T));
        pos_ += sizeof(T);
        return true;
    }
    [[nodiscard]] bool vec(std::vector<real>& v) {
        std::uint64_t n = 0;
        if (!pod(n) || n > size_) return false; // n > size_: a corrupt/huge count can never fit this buffer
        v.resize(static_cast<std::size_t>(n));
        for (auto& x : v) {
            if (!pod(x)) return false;
        }
        return true;
    }
    [[nodiscard]] bool curve(vehicle::Curve1D& c) { return vec(c.x) && vec(c.y); }
    [[nodiscard]] bool table(Table2D& t) { return vec(t.x) && vec(t.y) && vec(t.values); }
    [[nodiscard]] bool str(std::string& s) {
        std::uint64_t n = 0;
        if (!pod(n) || n > size_ || pos_ + n > size_) return false;
        s.assign(reinterpret_cast<const char*>(data_ + pos_), static_cast<std::size_t>(n));
        pos_ += n;
        return true;
    }
    [[nodiscard]] bool at_end() const { return pos_ == size_; }

private:
    const unsigned char* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
};

void write_maps_payload(Writer& w, const EngineMaps& m) {
    w.table(m.maps.ve);
    w.table(m.maps.eta_i);
    w.curve(m.maps.f_lambda);
    w.curve(m.maps.eta_spark);
    w.table(m.maps.theta_mbt_deg);
    w.curve(m.maps.friction_torque_nm);
    w.pod<std::uint8_t>(m.maps.has_pmep_excess ? 1 : 0);
    w.table(m.maps.pmep_excess_pa);
    w.pod<std::uint8_t>(m.maps.has_theta_kl ? 1 : 0);
    w.table(m.maps.theta_kl_deg);
    w.str(m.maps.source);
    w.table(m.t_exh_k);
    w.table(m.h_exh_w);
    w.pod(m.eta_spark_spread);
}

// Fills every field of *out except generator_version/quality/content_hash
// (the caller sets those from the header it already validated). Returns
// false on any underrun.
[[nodiscard]] bool read_maps_payload(Reader& r, EngineMaps& out) {
    std::uint8_t has_pmep = 0, has_theta_kl = 0;
    const bool ok = r.table(out.maps.ve) && r.table(out.maps.eta_i) && r.curve(out.maps.f_lambda) &&
                     r.curve(out.maps.eta_spark) && r.table(out.maps.theta_mbt_deg) && r.curve(out.maps.friction_torque_nm) &&
                     r.pod(has_pmep) && r.table(out.maps.pmep_excess_pa) && r.pod(has_theta_kl) && r.table(out.maps.theta_kl_deg) &&
                     r.str(out.maps.source) && r.table(out.t_exh_k) && r.table(out.h_exh_w) && r.pod(out.eta_spark_spread);
    if (!ok) return false;
    out.maps.has_pmep_excess = has_pmep != 0;
    out.maps.has_theta_kl = has_theta_kl != 0;
    return true;
}

constexpr char kMagic[8] = {'P', 'S', 'M', 'A', 'P', 'S', '0', '1'};

[[nodiscard]] std::string cache_file_path(const std::string& cache_dir, std::uint64_t key) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase; // placeholder, overwritten below - kept for clang-tidy hygiene only
    char hex[17];
    std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(key));
    return (std::filesystem::path(cache_dir) / (std::string(hex) + ".psmaps")).string();
}

} // namespace

std::uint64_t engine_map_cache_key(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality) {
    KeyFnv f{14695981039346656037ULL};
    // CycleModelDesc, every field incl. source (the generated maps carry it
    // as EngineMaps::source - see map_cache.h's PROVISIONAL DECISION).
    f.str(desc.source);
    f.pod(desc.bore_m);
    f.pod(desc.stroke_m);
    f.pod(desc.rod_m);
    f.pod(desc.pin_offset_m);
    f.pod(desc.compression_ratio);
    f.pod(desc.cylinders);
    f.pod(desc.cycle);
    f.curve(desc.intake_lift.lift_m_vs_deg);
    f.pod(desc.intake_lift.phase_ref_deg);
    f.curve(desc.exhaust_lift.lift_m_vs_deg);
    f.pod(desc.exhaust_lift.phase_ref_deg);
    f.pod(desc.intake_valves);
    f.pod(desc.exhaust_valves);
    f.pod(desc.intake_valve_d_m);
    f.pod(desc.exhaust_valve_d_m);
    f.pod(desc.runner_length_m);
    f.pod(desc.runner_diameter_m);
    f.pod(desc.intake_port_m3);
    f.pod(desc.exhaust_port_m3);
    f.table(desc.cam.intake_advance_deg);
    f.table(desc.cam.exhaust_retard_deg);
    f.curve(desc.back_pressure_pa_vs_exh_kg_s);
    f.pod(desc.wiebe_a);
    f.pod(desc.wiebe_m);
    f.pod(desc.burn_duration_deg);
    f.pod(desc.ca50_mbt_deg_atdc);
    f.pod(desc.woschni_c1_gx);
    f.pod(desc.woschni_c1_cc);
    f.pod(desc.woschni_c2);
    f.pod(desc.wall_temperature_k);
    f.curve(desc.cd_vs_l_over_d);
    f.pod(desc.head_area_factor);
    f.pod(desc.combustion_eff_lean);
    f.pod(desc.gas.fresh.cv_a_j_kgk);
    f.pod(desc.gas.fresh.cv_b_j_kgk2);
    f.pod(desc.gas.fresh.r_j_kgk);
    f.pod(desc.gas.fresh.gamma_flow);
    f.pod(desc.gas.burned.cv_a_j_kgk);
    f.pod(desc.gas.burned.cv_b_j_kgk2);
    f.pod(desc.gas.burned.r_j_kgk);
    f.pod(desc.gas.burned.gamma_flow);
    f.pod(static_cast<int>(desc.friction.model));
    f.pod(desc.friction.a_pa);
    f.pod(desc.friction.b_pa);
    f.pod(desc.friction.c_pa);
    f.pod(desc.exhaust_backflow_temperature_k);
    f.vec(desc.grid_rpm);
    f.vec(desc.grid_p_im);
    // FuelDesc (source excluded - the generator never reads it)
    f.str(fuel.name);
    f.pod(static_cast<int>(fuel.phase));
    f.pod(fuel.lhv_j_kg);
    f.pod(fuel.afr_stoich);
    f.pod(fuel.density_kg_m3);
    // generator + quality
    f.pod(kGeneratorVersion);
    f.pod(static_cast<int>(quality));
    return f.h;
}

EngineMapsPtr load_cached_engine_maps(const std::string& cache_dir, std::uint64_t key) {
    if (cache_dir.empty()) return nullptr;
    std::error_code ec;
    const std::string path = cache_file_path(cache_dir, key);
    if (!std::filesystem::is_regular_file(path, ec) || ec) return nullptr;

    std::ifstream in(path, std::ios::binary);
    if (!in) return nullptr;
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    // Header: magic[8] + format_version(u32) + key(u64) + content_hash(u64) + quality(u32) + payload_size(u64) = 40 bytes.
    constexpr std::size_t kHeaderSize = 8 + 4 + 8 + 8 + 4 + 8;
    if (bytes.size() < kHeaderSize) return nullptr; // truncated (H2)

    Reader hdr(bytes.data(), kHeaderSize);
    char magic[8];
    std::uint32_t format_version = 0;
    std::uint64_t file_key = 0, content_hash = 0, payload_size = 0;
    std::uint32_t quality_raw = 0;
    for (char& c : magic) {
        if (!hdr.pod(c)) return nullptr;
    }
    if (!hdr.pod(format_version) || !hdr.pod(file_key) || !hdr.pod(content_hash) || !hdr.pod(quality_raw) || !hdr.pod(payload_size)) {
        return nullptr;
    }
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) return nullptr; // wrong magic (H2)
    if (format_version != kPsMapsFormatVersion) return nullptr;          // wrong format version (H2)
    if (file_key != key) return nullptr;                                 // wrong key (H2)
    if (bytes.size() - kHeaderSize != payload_size) return nullptr;      // size mismatch (H2, catches a truncated file too)

    auto out = std::make_shared<EngineMaps>();
    Reader payload(bytes.data() + kHeaderSize, static_cast<std::size_t>(payload_size));
    if (!read_maps_payload(payload, *out) || !payload.at_end()) return nullptr; // truncated/malformed payload (H2)

    out->generator_version = kGeneratorVersion; // a stale generator version could never share this KEY (see map_cache.h)
    out->quality = static_cast<MapQuality>(quality_raw);
    out->content_hash = engine_maps_content_hash(*out);
    if (out->content_hash != content_hash) return nullptr; // a flipped payload byte changes this (H2)
    return out;
}

void store_cached_engine_maps(const std::string& cache_dir, std::uint64_t key, const EngineMaps& maps) {
    if (cache_dir.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(cache_dir, ec); // best-effort; a failure here just falls through to the write failing below
    (void)ec;

    Writer w;
    write_maps_payload(w, maps);

    std::vector<unsigned char> file_bytes;
    file_bytes.reserve(40 + w.buf.size());
    const auto append_pod = [&](const auto& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
        file_bytes.insert(file_bytes.end(), bytes, bytes + sizeof(v));
    };
    file_bytes.insert(file_bytes.end(), std::begin(kMagic), std::end(kMagic));
    append_pod(kPsMapsFormatVersion);
    append_pod(key);
    append_pod(maps.content_hash);
    append_pod(static_cast<std::uint32_t>(maps.quality));
    append_pod(static_cast<std::uint64_t>(w.buf.size()));
    file_bytes.insert(file_bytes.end(), w.buf.begin(), w.buf.end());

    // Atomic write (H4): a process+thread-unique temp name in the same
    // directory, then std::filesystem::rename.
    std::ostringstream tmp_name;
    tmp_name << cache_file_path(cache_dir, key) << "."
             << std::hash<std::thread::id>{}(std::this_thread::get_id()) << "." << static_cast<const void*>(&maps) << ".tmp";
    const std::string tmp_path = tmp_name.str();
    {
        std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
        if (!out) return; // caching is a pure optimisation - never throw
        out.write(reinterpret_cast<const char*>(file_bytes.data()), static_cast<std::streamsize>(file_bytes.size()));
        if (!out) {
            std::filesystem::remove(tmp_path, ec);
            return;
        }
    }
    std::filesystem::rename(tmp_path, cache_file_path(cache_dir, key), ec);
    if (ec) std::filesystem::remove(tmp_path, ec); // best-effort cleanup on a failed rename
}

} // namespace ps::drivetrain
