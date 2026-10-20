#include "map_cache.h"

#include "ps/engine_cycle/knock.h"

#include <atomic>
#include <bit>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h> // _getpid
#else
#include <unistd.h> // getpid
#endif

namespace ps::drivetrain {

namespace {

// The payload and header are raw host-order bytes (memcpy of doubles and
// fixed-width integers) and documented as little-endian (map_cache.h). Every
// build target is x86_64; a big-endian port must add byte swapping here.
static_assert(std::endian::native == std::endian::little, "the .psmaps format is little-endian; add byte swapping for this host");

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
    void ints(const std::vector<int>& v) {
        pod(static_cast<std::uint64_t>(v.size()));
        for (int x : v) pod(x);
    }
};

// ---- ps_audio A4: acoustic-field folding for CycleModelDesc's optional
// intake/exhaust pipe pair and the resolved InstallationDesc. Positions
// (RunnerPipeDesc has none; ExhaustElementDesc::position_m,
// InstallationIntakeDesc::inlet_position_m) and every path/source string
// (default_installation_path, installation_path, and the *.source citation
// fields on IntakeCycleDesc/ExhaustCycleDesc/InstallationDesc - mirrors
// FuelDesc::source's own exclusion above: provenance, never read by the
// generator) are deliberately NOT folded - out/audio_plan.md section 3.3
// "positions are excluded from the key" plus the same reasoning extended to
// every other purely-descriptive string. Everything that could change the
// finite-volume geometry A5 will build from these structs IS folded.
void fold_exhaust_elements(KeyFnv& f, const std::vector<ExhaustElementDesc>& elems) {
    f.pod(static_cast<std::uint64_t>(elems.size()));
    for (const auto& e : elems) {
        f.pod(static_cast<int>(e.type));
        f.pod(e.length_m);
        f.pod(e.d_m);
        f.pod(e.k_loss);
        // position_m intentionally excluded (audio-only, see comment above).
        f.pod(static_cast<std::uint64_t>(e.branches.size()));
        for (const auto& chain : e.branches) fold_exhaust_elements(f, chain);
    }
}

void fold_installation(KeyFnv& f, const InstallationDesc& inst) {
    f.pod(inst.intake.airbox_m3);
    f.pod(static_cast<std::uint8_t>(inst.intake.has_snorkel ? 1 : 0));
    f.pod(inst.intake.snorkel.length_m);
    f.pod(inst.intake.snorkel.d_m);
    // inlet_position_m intentionally excluded (audio-only).
    fold_exhaust_elements(f, inst.exhaust.elements);
    // source intentionally excluded (provenance-only).
}

void fold_pipe_pair(KeyFnv& f, const CycleModelDesc& desc) {
    // Presence itself must move the key so a pipe-bearing engine can never
    // collide with a lumped one that happens to share every other field
    // (in practice impossible since runner_length_m/runner_diameter_m are
    // forced to 0 for a pipe-bearing engine - kept anyway for clarity/safety).
    f.str("audio_a4.pipes");
    const auto& intake = *desc.intake;
    f.pod(intake.throttle_bore_m);
    f.pod(intake.plenum_m3);
    f.pod(static_cast<std::uint64_t>(intake.runners.size()));
    for (const auto& r : intake.runners) {
        f.ints(r.cylinders);
        f.pod(r.length_m);
        f.pod(r.d_in_m);
        f.pod(r.d_out_m);
    }
    f.str(intake.interface); // a topology tag, not a citation - folded

    const auto& exhaust = *desc.exhaust;
    f.pod(static_cast<std::uint64_t>(exhaust.primaries.size()));
    for (const auto& p : exhaust.primaries) {
        f.ints(p.cylinders);
        f.pod(p.length_m);
        f.pod(p.d_m);
    }
    f.pod(static_cast<std::uint64_t>(exhaust.merges.size()));
    for (const auto& m : exhaust.merges) {
        f.pod(static_cast<std::uint64_t>(m.from.size()));
        for (const auto& src : m.from) {
            f.pod(static_cast<std::uint8_t>(src.is_cylinder ? 1 : 0));
            f.pod(src.cylinder);
            f.str(src.name);
        }
        f.str(m.into); // a topology tag (referenced by a later merge/interface), not a citation - folded
        f.pod(m.length_m);
        f.pod(m.d_m);
    }
    f.str(exhaust.interface);

    f.pod(desc.resolution.power_dx_m);
    f.pod(desc.resolution.audio_dx_exhaust_m);
    f.pod(desc.resolution.audio_dx_intake_m);

    if (desc.installation.has_value()) {
        f.str("audio_a4.installation");
        fold_installation(f, *desc.installation);
    }

    // ps_audio A5: the generator v2 (CycleEngine) phases the cylinders by the
    // firing order/angles, so they are generator inputs for a pipe engine.
    // banks/bank_angle_deg are not read by the generator and not folded.
    f.str("audio_a5.firing");
    f.ints(desc.firing_order);
    f.vec(desc.firing_angles_deg);
}

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
        if (!pod(n) || n > (size_ - pos_) / sizeof(real)) return false; // a corrupt/huge count can never fit the rest of this buffer
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

// Engine garage P7d: marks the trailing wide-open-throttle block (an arbitrary 64-bit marker, never a knock count).
constexpr std::uint64_t kWotBlockTag = 0x31765424F5744F37ULL;

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
    // Engine garage P7: the per-fuel knock tables, as an optional TRAILING block written only when present - a file
    // without it is byte-identical to before P7 (the reader takes the block only when bytes remain).
    if (!m.maps.theta_kl_by_fuel.empty()) {
        w.pod(static_cast<std::uint64_t>(m.maps.theta_kl_by_fuel.size()));
        for (const Table2D& t : m.maps.theta_kl_by_fuel) w.table(t);
    }
    // Engine garage P7d: the wide-open-throttle curve, a TAGGED trailing block after the knock block, written only when
    // present (a file without it is byte-identical to before P7d). The tag cannot be a knock count (those are 1..64).
    if (!m.maps.wot_p_im_pa.x.empty()) {
        w.pod(kWotBlockTag);
        w.curve(m.maps.wot_p_im_pa);
    }
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
    if (r.at_end()) return true;
    // P7 trailing knock block, then the P7d tagged WOT block (see write_maps_payload). A count that cannot fit, an
    // unknown tag or bytes after the WOT block are a malformed file.
    std::uint64_t n = 0;
    if (!r.pod(n)) return false;
    if (n != kWotBlockTag) {
        if (n == 0 || n > 64) return false;
        out.maps.theta_kl_by_fuel.resize(static_cast<std::size_t>(n));
        for (Table2D& t : out.maps.theta_kl_by_fuel) {
            if (!r.table(t)) return false;
        }
        if (r.at_end()) return true;
        if (!r.pod(n) || n != kWotBlockTag) return false;
    }
    return r.curve(out.maps.wot_p_im_pa) && !out.maps.wot_p_im_pa.x.empty() &&
           out.maps.wot_p_im_pa.x.size() == out.maps.wot_p_im_pa.y.size();
}

constexpr char kMagic[8] = {'P', 'S', 'M', 'A', 'P', 'S', '0', '1'};

[[nodiscard]] std::string cache_file_path(const std::string& cache_dir, std::uint64_t key) {
    char hex[17];
    std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(key));
    return (std::filesystem::path(cache_dir) / (std::string(hex) + ".psmaps")).string();
}

} // namespace

std::uint64_t engine_map_cache_key(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality) {
    return engine_map_cache_key(desc, fuel, quality, engine_map_generator_version(desc));
}

std::vector<std::uint64_t> engine_map_profile_cache_keys(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality) {
    // P9 cam profiles in profile order; P10 lift nodes u = 1 first, then ascending u (engine_map_set_descs).
    std::vector<std::uint64_t> keys;
    for (const CycleModelDesc& d : engine_map_set_descs(desc)) keys.push_back(engine_map_cache_key(d, fuel, quality));
    return keys;
}

std::uint64_t engine_map_cache_key(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                   std::uint32_t generator_version) {
    // Engine garage P9: a cam-profile desc has one key PER PROFILE (engine_map_profile_cache_keys); keying the list
    // itself would make the profile id part of the key and lose the per-profile cache reuse.
    if (!desc.cam_profiles.empty()) {
        throw std::runtime_error("engine_map_cache_key: the cycle desc has cam profiles - key each profile "
                                 "(engine_map_profile_cache_keys / cycle_desc_for_cam_profile)");
    }
    // Engine garage P10: likewise one key per lift node (cycle_desc_for_lift_node), never the family itself.
    if (!desc.lift_family.empty()) {
        throw std::runtime_error("engine_map_cache_key: the cycle desc has an intake lift family - key each node "
                                 "(engine_map_profile_cache_keys / cycle_desc_for_lift_node)");
    }
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
    // Legacy keys remain identical when adaptive sampling is disabled.
    if (desc.pressure_sampling.enabled) {
        f.pod(desc.pressure_sampling.min_pa); f.pod(desc.pressure_sampling.max_pa);
        f.pod(desc.pressure_sampling.relative_error); f.pod(desc.pressure_sampling.max_points);
        f.pod(desc.pressure_sampling.max_passes);
        f.pod(desc.pressure_sampling.allow_budget_limit);
    }
    // C17a follow-up: the Tier 1b spark schedule and the Tier 2 ignition
    // delay, folded (behind a tag) only when present / non-zero so every
    // pre-existing engine's key - and its .psmaps file name - is unchanged.
    // The generator's output for such an engine is bit-identical as well.
    if (!desc.spark_advance_deg.x.empty() || !desc.spark_advance_deg.y.empty() ||
        !desc.spark_advance_deg.values.empty() || desc.ignition_delay_deg != real(0.0)) {
        f.str("c17a2.spark_schedule");
        f.table(desc.spark_advance_deg);
        f.pod(desc.ignition_delay_deg);
    }
    // Engine garage P7: the knock limit, folded (behind a tag, with the knock
    // model's own version) ONLY for an engine that opts in - every other
    // engine's key is unchanged and a fuel's octane never reaches it. The
    // opt-in flag alone moves the key; each listed fuel's name, RON and MON
    // (in fuels order) and the margin do too.
    if (desc.knock.enabled) {
        f.str("p7.knock_limit");
        f.pod(engine_cycle::kKnockModelVersion);
        f.pod(desc.knock.margin_deg);
        f.pod(static_cast<std::uint64_t>(desc.knock.fuels.size()));
        for (const CycleModelDesc::KnockFuel& kf : desc.knock.fuels) {
            f.str(kf.name);
            f.pod(kf.ron);
            f.pod(kf.mon);
        }
        f.table(desc.knock.lambda_target); // P7b (knock model v2): the replay's lambda
        // Knock model v3: the replay's Douaud-Eyzat parameter set.
        f.pod(desc.knock.model.prefactor_scale);
        f.pod(desc.knock.model.pressure_exponent);
        f.pod(desc.knock.model.activation_k);
        f.pod(desc.knock.model.octane_index_k);
        f.pod(desc.knock.model.dilution_exponent);
        f.pod(desc.knock.model.end_burn_fraction); // knock model v5 (burn law phase 2a)
    }
    // Burn law (design C, vault PHYS-079): a law engine folds the law's version and every parameter (H3 rows per
    // field); a Fixed desc folds nothing here, so its key is the pre-law key (its generator version is too).
    if (desc.burn_law == CycleModelDesc::BurnLawMode::Standard) {
        const engine_cycle::BurnLawParams& b = desc.burn_law_params;
        f.str("phys079.burn_law");
        f.pod(engine_cycle::kBurnLawVersion);
        for (real v : {b.reference_rpm, b.reference_unburned_temperature_k, b.reference_unburned_pressure_pa,
                       b.reference_residual_fraction, b.development_reference_deg, b.rapid_reference_deg, b.scale,
                       b.rpm_exponent, b.flame_speed_exponent, b.flame_speed_pressure_factor, b.mk_phi_m, b.mk_b_m_m_s, b.mk_b_2_m_s, b.mk_alpha_0,
                       b.mk_alpha_phi, b.mk_beta_0, b.mk_beta_phi, b.mk_t0_k, b.mk_p0_pa, b.mk_phi_min, b.mk_phi_max, b.dilution_k,
                       b.dilution_exponent, b.min_dilution_factor, b.min_flame_speed_m_s, b.compression_exponent,
                       b.pressure_exponent}) {
            f.pod(v);
        }
    }
    // ps_audio A4: the intake/exhaust pipe pair + resolved installation,
    // folded only when present (both absent - the pre-A4 default for every
    // shipped engine - leaves the key bit-identical to before this field
    // existed; see fold_pipe_pair's own comment for what is/isn't folded).
    if (desc.intake.has_value()) {
        fold_pipe_pair(f, desc);
        // Folded only when set, so every engine without a turbo keeps its key.
        if (desc.intake->compressor_supply) f.str("boost.compressor_supply");
    }
    // FuelDesc (source excluded - the generator never reads it)
    f.str(fuel.name);
    f.pod(static_cast<int>(fuel.phase));
    f.pod(fuel.lhv_j_kg);
    f.pod(fuel.afr_stoich);
    f.pod(fuel.density_kg_m3);
    // generator + quality. ps_audio A5: the version of the path this desc
    // generates through (1 lumped - every pre-A5 key unchanged, 2 pipe), so
    // v1 and v2 maps can never share a file.
    f.pod(generator_version);
    f.pod(static_cast<int>(quality));
    return f.h;
}

namespace {

EngineMapsPtr load_cached_engine_maps_impl(const std::string& cache_dir, std::uint64_t key, std::uint32_t generator_version) {
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
    // An unknown quality value (a file from a newer build, or corruption) is a miss, never a cast of
    // an out-of-range integer into the enum (engine garage P1).
    if (quality_raw != static_cast<std::uint32_t>(MapQuality::Preview) && quality_raw != static_cast<std::uint32_t>(MapQuality::Full) &&
        quality_raw != static_cast<std::uint32_t>(MapQuality::Sketch)) {
        return nullptr;
    }

    auto out = std::make_shared<EngineMaps>();
    Reader payload(bytes.data() + kHeaderSize, static_cast<std::size_t>(payload_size));
    if (!read_maps_payload(payload, *out) || !payload.at_end()) return nullptr; // truncated/malformed payload (H2)

    out->generator_version = generator_version; // the key folds the version, so a stale file can never share it (map_cache.h)
    out->quality = static_cast<MapQuality>(quality_raw);
    out->content_hash = engine_maps_content_hash(*out);
    if (out->content_hash != content_hash) return nullptr; // a flipped payload byte changes this (H2)
    return out;
}

// Process id plus a process-wide counter make every temp name unique across
// concurrent writers: the thread id alone is not (on Linux std::thread::id
// is a pthread_t, an address two processes can share), and neither is the
// maps object's address.
[[nodiscard]] unsigned long long process_id() {
#if defined(_WIN32)
    return static_cast<unsigned long long>(_getpid());
#else
    return static_cast<unsigned long long>(getpid());
#endif
}

std::atomic<std::uint64_t> g_temp_counter{0};

void store_cached_engine_maps_impl(const std::string& cache_dir, std::uint64_t key, const EngineMaps& maps) {
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

    // Atomic write (H4): a temp name unique per process, thread and call in
    // the same directory, then std::filesystem::rename (MoveFileExW with
    // MOVEFILE_REPLACE_EXISTING on Windows; if a reader holds the target open
    // the rename fails, ec is set and the temp is removed - the existing file
    // stays whole).
    char suffix[96];
    std::snprintf(suffix, sizeof(suffix), ".%llu.%llu.%llu.tmp", process_id(),
                  static_cast<unsigned long long>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                  static_cast<unsigned long long>(g_temp_counter.fetch_add(1, std::memory_order_relaxed)));
    const std::string tmp_path = cache_file_path(cache_dir, key) + suffix;
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

} // namespace

// Both public entry points are documented as never throwing. The filesystem
// calls use error_code overloads, but path/string/vector construction can
// still throw (bad_alloc, a path conversion error); any such exception is a
// cache miss / a skipped write. Caught by type only (vault TOOL-019).

EngineMapsPtr load_cached_engine_maps(const std::string& cache_dir, std::uint64_t key, std::uint32_t generator_version) {
    if (cache_dir.empty()) return nullptr;
    try {
        return load_cached_engine_maps_impl(cache_dir, key, generator_version);
    } catch (...) {
        return nullptr;
    }
}

void store_cached_engine_maps(const std::string& cache_dir, std::uint64_t key, const EngineMaps& maps) {
    if (cache_dir.empty()) return;
    try {
        store_cached_engine_maps_impl(cache_dir, key, maps);
    } catch (...) {
    }
}

} // namespace ps::drivetrain
