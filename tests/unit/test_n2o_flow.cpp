// tests/unit/test_n2o_flow.cpp - N2O plan N0: the NIST oxidizer data file
// (data/oxidizers/nitrous_oxide.json), its loader (ps::io::load_oxidizer_json)
// and the internal flow tables (core/src/drivetrain/nitrous_flow.h:
// OxidizerTables - saturation lookups, HEM critical flux, Dyer NHNE).
//
// F1 the loaded file equals the NIST rows (pinned literals + the plan's own
//    section 1.1 table within its interpolation precision)
// F2 G_HEM / choke pressure / SPI at 900/950/1000 psig against a brute-force
//    re-derivation written independently in this file (1e-9) AND against
//    literals from an independent Python implementation (out/hem_check.py of
//    the N0 session); the plan's hand numbers are reported, not asserted tight
// F3 Dyer = 0.5 SPI + 0.5 HEM exactly
// F4 loader rejections incl. the O2 mass fraction cross-check
// F5 zero allocation of every runtime lookup (debug_alloc-armed job), with a
//    negative control proving the hook sees an allocation in the same job
//
// Windows ASan caveat (CLAUDE.md): throw checks use CHECK_THROWS_WITH only.

#include "drivetrain/nitrous_flow.h"
#include "ps/io/nitrous_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ps;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;

namespace {

const std::string kOxidizerPath = std::string(PS_SOURCE_DIR) + "/data/oxidizers/nitrous_oxide.json";

struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const std::string& contents, const char* tag) {
        path = std::filesystem::temp_directory_path() /
               (std::string("ps_test_n2o_flow_") + tag + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << contents;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

json file_json() {
    std::ifstream in(kOxidizerPath);
    json j;
    in >> j;
    return j;
}

// psig -> Pa abs, as the plan's section 1.1 does: (psig + 14.696) * 0.0689476 bar.
double psig_to_pa(double psig) { return (psig + 14.696) * 0.0689476 * 1.0e5; }

// Independent brute-force HEM: for a bottle state taken at row i, loop every
// lower saturation row as the downstream state. Written from the physics, not
// from nitrous_flow.cpp: specific volume of the two-phase mixture, enthalpy
// drop, flux = sqrt(2*dh)/v2.
struct BruteHem {
    std::vector<double> p_pa, g, choke;
};

BruteHem brute_hem(const drivetrain::OxidizerDesc& d) {
    const auto& s = d.saturation;
    const std::size_t n = s.t_k.size();
    BruteHem r;
    r.p_pa.resize(n);
    r.g.assign(n, 0.0);
    r.choke.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        r.p_pa[i] = s.p_bar[i] * 1.0e5;
        r.choke[i] = r.p_pa[i];
        for (std::size_t j = 0; j < i; ++j) {
            const double x = (s.s_liquid_j_gk[i] - s.s_liquid_j_gk[j]) / (s.s_vapour_j_gk[j] - s.s_liquid_j_gk[j]);
            if (x < 0.0 || x > 1.0) continue;
            const double v2 = (1.0 - x) / s.rho_liquid[j] + x / s.rho_vapour[j];
            const double dh_j_kg = 1000.0 * (s.h_liquid_kj_kg[i] - (s.h_liquid_kj_kg[j] + x * (s.h_vapour_kj_kg[j] - s.h_liquid_kj_kg[j])));
            if (dh_j_kg <= 0.0) continue;
            const double flux = std::sqrt(2.0 * dh_j_kg) / v2;
            if (flux > r.g[i]) {
                r.g[i] = flux;
                r.choke[i] = r.p_pa[j];
            }
        }
    }
    return r;
}

double lerp_table(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    std::size_t hi = 1;
    while (xs[hi] < x) ++hi;
    const double t = (x - xs[hi - 1]) / (xs[hi] - xs[hi - 1]);
    return ys[hi - 1] + t * (ys[hi] - ys[hi - 1]);
}

std::size_t row_of(const drivetrain::OxidizerDesc& d, double t_k) {
    for (std::size_t i = 0; i < d.saturation.t_k.size(); ++i) {
        if (d.saturation.t_k[i] == t_k) return i;
    }
    FAIL("no saturation row at " << t_k << " K");
    return 0;
}

} // namespace

TEST_CASE("F1 the oxidizer file equals the NIST rows", "[n2o][io][drivetrain]") {
    const drivetrain::OxidizerDesc d = io::load_oxidizer_json(kOxidizerPath);
    CHECK(d.molar_mass_g_mol == 44.0128);
    CHECK(d.formation_enthalpy_kj_mol == 82.05);
    CHECK(d.oxygen_mass_fraction == 0.3635);
    CHECK(d.saturation.t_k.front() == 183.0);
    CHECK(d.saturation.t_k.back() == 309.5);
    CHECK(d.source.find("NIST") != std::string::npos);
    CHECK(d.source.find("Chase 1998") != std::string::npos);

    struct Row {
        double t, p, rho_l, h_l, s_l, rho_v, h_v, s_v;
    };
    // Literals as printed by the NIST WebBook download (RefState DEF), see the file's "source".
    const Row nist[] = {
        {293.13, 50.502, 785.26, 214.34, 0.87892, 157.88, 384.34, 1.4589},
        {303.37, 63.378, 685.18, 247.59, 0.98444, 235.16, 364.47, 1.3697},
        {305.41, 66.250, 654.29, 256.53, 1.0124, 261.75, 357.11, 1.3417},
        {307.36, 69.111, 613.48, 267.45, 1.0466, 298.44, 346.77, 1.3047},
    };
    // The plan's section 1.1 table (hand-interpolated from the 1 K NIST rows): agrees to its own rounding.
    const Row plan[] = {
        {293.13, 50.501, 785.26, 214.34, 0.87891, 157.88, 384.35, 1.4589},
        {303.37, 63.384, 685.12, 247.61, 0.98450, 235.21, 364.46, 1.3697},
        {305.41, 66.247, 654.32, 256.52, 1.0124, 261.72, 357.12, 1.3418},
        {307.36, 69.110, 613.50, 267.45, 1.0466, 298.42, 346.78, 1.3047},
    };
    for (int k = 0; k < 4; ++k) {
        const std::size_t i = row_of(d, nist[k].t);
        const auto& s = d.saturation;
        CAPTURE(nist[k].t);
        CHECK(s.p_bar[i] == nist[k].p);
        CHECK(s.rho_liquid[i] == nist[k].rho_l);
        CHECK(s.h_liquid_kj_kg[i] == nist[k].h_l);
        CHECK(s.s_liquid_j_gk[i] == nist[k].s_l);
        CHECK(s.rho_vapour[i] == nist[k].rho_v);
        CHECK(s.h_vapour_kj_kg[i] == nist[k].h_v);
        CHECK(s.s_vapour_j_gk[i] == nist[k].s_v);
        const double rel = 3.0e-4; // the plan's hand-interpolated rho_v at 303.37 K (235.21) is 2.1e-4 above the NIST row (235.16)
        CHECK(s.p_bar[i] == Approx(plan[k].p).epsilon(rel));
        CHECK(s.rho_liquid[i] == Approx(plan[k].rho_l).epsilon(rel));
        CHECK(s.h_liquid_kj_kg[i] == Approx(plan[k].h_l).epsilon(rel));
        CHECK(s.s_liquid_j_gk[i] == Approx(plan[k].s_l).epsilon(rel));
        CHECK(s.rho_vapour[i] == Approx(plan[k].rho_v).epsilon(rel));
        CHECK(s.h_vapour_kj_kg[i] == Approx(plan[k].h_v).epsilon(rel));
        CHECK(s.s_vapour_j_gk[i] == Approx(plan[k].s_v).epsilon(rel));
    }
    // 1 bar gas isobar: the plan's two spot enthalpies.
    const auto& g = d.gas_1bar;
    const auto at = [&](double t) {
        for (std::size_t i = 0; i < g.t_k.size(); ++i) {
            if (g.t_k[i] == t) return g.h_kj_kg[i];
        }
        FAIL("no gas row at " << t);
        return 0.0;
    };
    CHECK(at(280.0) == 452.70);
    CHECK(at(300.0) == 470.20);

    // The tables built from it: decomposition heat dHf/M and the table's own lookups.
    const drivetrain::OxidizerTables tab(d);
    CHECK(tab.decomposition_j_per_kg() == Approx(82.05e3 / 44.0128e-3).epsilon(1e-12)); // 1.8643 MJ/kg
    CHECK(tab.decomposition_j_per_kg() == Approx(1.8643e6).epsilon(1e-4));
    const auto liq = tab.liquid_at_temperature(293.13);
    CHECK(liq.p_pa == Approx(50.502e5).epsilon(1e-12));
    CHECK(liq.rho_l == Approx(785.26).epsilon(1e-12));
    CHECK(liq.h_l_j_kg == Approx(214.34e3).epsilon(1e-12));
    CHECK(tab.gas_enthalpy_j_kg(290.0) == Approx(461.40e3).epsilon(1e-12));
    // Saturated liquid at 63.07 bar abs (900 psig): T_sat about 303.1 K (plan section 1.1).
    CHECK(tab.liquid_at_pressure(psig_to_pa(900.0)).t_k == Approx(303.14).margin(0.05));
    CHECK(tab.liquid_at_pressure(psig_to_pa(950.0)).t_k == Approx(305.59).margin(0.05));
}

TEST_CASE("F2 HEM flux, choke pressure and SPI at 900/950/1000 psig", "[n2o][drivetrain]") {
    const drivetrain::OxidizerDesc d = io::load_oxidizer_json(kOxidizerPath);
    const drivetrain::OxidizerTables tab(d);
    const BruteHem bh = brute_hem(d);

    struct Case {
        double psig;
        double g_literal;      // independent Python implementation, same file
        double choke_literal;  // bar
        double spi_literal;    // flux to 1 bar abs
        double plan_g, plan_choke_bar, plan_spi;
    };
    const Case cases[] = {
        {900.0, 32293.892223813982, 43.842, 92425.96351722824, 32327.0, 43.8, 92427.0},
        {950.0, 32749.352722160387, 45.936, 92356.14582868946, 32782.0, 45.7, 92363.0},
        {1000.0, 32771.3770899475, 47.97346866492493, 90738.42106276346, 32788.0, 47.4, 90756.0},
    };
    for (const Case& c : cases) {
        SECTION("psig " + std::to_string(static_cast<int>(c.psig))) {
            const double p1 = psig_to_pa(c.psig);
            const double g_bf = lerp_table(bh.p_pa, bh.g, p1);
            const double ch_bf = lerp_table(bh.p_pa, bh.choke, p1);
            CHECK(tab.g_hem(p1) == Approx(g_bf).epsilon(1e-9));
            CHECK(tab.choke_pressure_pa(p1) == Approx(ch_bf).epsilon(1e-9));
            CHECK(tab.g_hem(p1) == Approx(c.g_literal).epsilon(1e-9));
            CHECK(tab.choke_pressure_pa(p1) == Approx(c.choke_literal * 1.0e5).epsilon(1e-9));
            // SPI to 1 bar absolute with the saturated-liquid density at p1.
            const double rho_l = lerp_table(bh.p_pa, [&] {
                std::vector<double> v(d.saturation.rho_liquid.begin(), d.saturation.rho_liquid.end());
                return v;
            }(), p1);
            CHECK(tab.g_spi(p1, 1.0e5) == Approx(std::sqrt(2.0 * rho_l * (p1 - 1.0e5))).epsilon(1e-12));
            CHECK(tab.g_spi(p1, 1.0e5) == Approx(c.spi_literal).epsilon(1e-9));
            // Plan hand numbers (section 2.2 table): same quantity, within 0.2 % (flux) /
            // one 1 K table row (choke pressure, a table-row argmax: about 1.5-2 bar per row).
            INFO("G_HEM " << tab.g_hem(p1) << " vs plan " << c.plan_g << ", choke "
                          << tab.choke_pressure_pa(p1) / 1.0e5 << " vs plan " << c.plan_choke_bar << " bar");
            CHECK(tab.g_hem(p1) == Approx(c.plan_g).epsilon(2.0e-3));
            CHECK(tab.choke_pressure_pa(p1) / 1.0e5 == Approx(c.plan_choke_bar).margin(0.8));
            CHECK(tab.g_spi(p1, 1.0e5) == Approx(c.plan_spi).epsilon(5.0e-4));
        }
    }
    // The HEM flux never exceeds the single-phase flux at the same pressure (flashing chokes the jet).
    const double p = psig_to_pa(950.0);
    CHECK(tab.g_hem(p) < tab.g_spi(p, 1.0e5));
    // Clamped outside the table, zero SPI without a pressure difference.
    CHECK(tab.g_hem(1.0) == tab.g_hem(tab.p_min_pa()));
    CHECK(tab.g_spi(p, p) == 0.0);
    CHECK(tab.g_spi(p, 2.0 * p) == 0.0);
}

TEST_CASE("F3 Dyer NHNE is half SPI plus half HEM", "[n2o][drivetrain]") {
    const drivetrain::OxidizerDesc d = io::load_oxidizer_json(kOxidizerPath);
    const drivetrain::OxidizerTables tab(d);
    for (const double psig : {900.0, 950.0, 1000.0}) {
        const double p1 = psig_to_pa(psig);
        for (const double p_down : {0.5e5, 1.0e5, 2.5e5}) {
            const double expect = 0.5 * tab.g_spi(p1, p_down) + 0.5 * tab.g_hem(p1);
            CHECK(tab.g_dyer(p1, p_down) == Approx(expect).epsilon(1e-15));
        }
    }
    // The plan's observation: Dyer gives about 1.9x the HEM flux.
    const double p1 = psig_to_pa(950.0);
    const double ratio = tab.g_dyer(p1, 1.0e5) / tab.g_hem(p1);
    INFO("Dyer/HEM = " << ratio);
    CHECK(ratio > 1.8);
    CHECK(ratio < 2.1);
}

TEST_CASE("F4 oxidizer loader rejections", "[n2o][io]") {
    const json base = file_json();

    SECTION("the shipped file loads") { CHECK_NOTHROW(io::load_oxidizer_json(kOxidizerPath)); }
    SECTION("unreadable file") { CHECK_THROWS_WITH(io::load_oxidizer_json("no/such/file.json"), ContainsSubstring("cannot open file")); }
    SECTION("unknown format") {
        json j = base;
        j["format"] = "physics_sim.oxidizer/2";
        TempJsonFile f(j.dump(), "fmt");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("missing format") {
        json j = base;
        j.erase("format");
        TempJsonFile f(j.dump(), "nofmt");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("missing required 'format'"));
    }
    SECTION("unknown top-level key") {
        json j = base;
        j["p_critical_bar"] = 72.4;
        TempJsonFile f(j.dump(), "unk");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("unknown key 'p_critical_bar'"));
    }
    SECTION("missing source") {
        json j = base;
        j.erase("source");
        TempJsonFile f(j.dump(), "nosrc");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'source' is required"));
    }
    SECTION("missing saturation key") {
        json j = base;
        j["saturation"].erase("h_liquid_kj_kg");
        TempJsonFile f(j.dump(), "nokey");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'h_liquid_kj_kg' is required"));
    }
    SECTION("non-monotonic t_k") {
        json j = base;
        std::swap(j["saturation"]["t_k"][10], j["saturation"]["t_k"][11]);
        TempJsonFile f(j.dump(), "tmono");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'t_k' must be strictly ascending"));
    }
    SECTION("non-monotonic p_bar") {
        json j = base;
        std::swap(j["saturation"]["p_bar"][20], j["saturation"]["p_bar"][21]);
        TempJsonFile f(j.dump(), "pmono");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'p_bar' must be strictly ascending"));
    }
    SECTION("table length mismatch") {
        json j = base;
        j["saturation"]["rho_vapour"].erase(j["saturation"]["rho_vapour"].begin());
        TempJsonFile f(j.dump(), "len");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'rho_vapour' has"));
    }
    SECTION("gas table length mismatch") {
        json j = base;
        j["gas_1bar"]["h_kj_kg"].erase(j["gas_1bar"]["h_kj_kg"].begin());
        TempJsonFile f(j.dump(), "glen");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'h_kj_kg' has"));
    }
    SECTION("a non-number in a column") {
        json j = base;
        j["saturation"]["p_bar"][5] = "fifty";
        TempJsonFile f(j.dump(), "nan");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("only numbers"));
    }
    SECTION("non-finite number") {
        // JSON has no NaN/Inf literal; a 1e999 overflow must not load as +inf.
        std::string text = base.dump();
        const std::string key = "\"molar_mass_g_mol\":44.0128";
        const auto pos = text.find(key);
        REQUIRE(pos != std::string::npos);
        text.replace(pos, key.size(), "\"molar_mass_g_mol\":1e999");
        TempJsonFile f(text, "inf");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("number overflow parsing '1e999'")); // nlohmann itself refuses the overflowing literal
    }
    SECTION("oxygen mass fraction out of range") {
        json j = base;
        j["oxygen_mass_fraction"] = 1.5;
        TempJsonFile f(j.dump(), "o2range");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'oxygen_mass_fraction' must be in (0, 1)"));
    }
    SECTION("oxygen mass fraction typo") {
        json j = base;
        j["oxygen_mass_fraction"] = 0.3535; // one digit off
        TempJsonFile f(j.dump(), "o2typo");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("disagrees with 0.5*31.998/molar_mass"));
    }
    SECTION("oxygen mass fraction inconsistent with the molar mass") {
        json j = base;
        j["molar_mass_g_mol"] = 30.0; // the O2 fraction 0.3635 no longer fits
        TempJsonFile f(j.dump(), "o2mm");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("disagrees with"));
    }
    SECTION("vapour entropy not above liquid entropy") {
        json j = base;
        j["saturation"]["s_vapour_j_gk"][7] = j["saturation"]["s_liquid_j_gk"][7];
        TempJsonFile f(j.dump(), "sv");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'s_vapour_j_gk' must exceed"));
    }
    SECTION("non-positive density") {
        json j = base;
        j["saturation"]["rho_vapour"][3] = 0.0;
        TempJsonFile f(j.dump(), "rho");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("'rho_vapour' must be > 0"));
    }
    SECTION("too few saturation rows") {
        json j = base;
        for (auto& kv : j["saturation"].items()) kv.value() = json::array({kv.value()[0], kv.value()[1]});
        TempJsonFile f(j.dump(), "few");
        CHECK_THROWS_WITH(io::load_oxidizer_json(f.path.string()), ContainsSubstring("at least 3 rows"));
    }
}

TEST_CASE("F5 the runtime lookups allocate nothing", "[n2o][drivetrain][debug_alloc]") {
    const drivetrain::OxidizerDesc d = io::load_oxidizer_json(kOxidizerPath);
    const drivetrain::OxidizerTables tab(d);
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        return;
    }
    jobs::JobSystem js(2);
    namespace da = ps::jobs::debug_alloc;
    std::vector<double> sink(64, 0.0);
    da::reset();
    da::set_armed(true);
    js.parallel_batches(sink.size(), [&](std::size_t i) {
        const double p = 5.0e6 + 1.0e5 * static_cast<double>(i % 20);
        const auto liq = tab.liquid_at_pressure(p);
        const auto liq_t = tab.liquid_at_temperature(liq.t_k);
        sink[i] = tab.g_hem(p) + tab.choke_pressure_pa(p) + tab.g_spi(p, 1.0e5) + tab.g_dyer(p, 1.5e5) +
                  tab.gas_enthalpy_j_kg(liq_t.t_k) + tab.saturation_pressure_pa(liq.t_k) + liq_t.h_l_j_kg;
    });
    da::set_armed(false);
    CHECK(da::own_violation_count() == 0);
    CHECK(std::all_of(sink.begin(), sink.end(), [](double v) { return std::isfinite(v) && v > 0.0; }));

    // Negative control: the same hook must see an allocation made in the same kind of job.
    da::reset();
    da::set_armed(true);
    js.parallel_batches(4, [&](std::size_t i) {
        std::vector<double> v(256 + i, 1.0);
        sink[i] = v.back();
    });
    da::set_armed(false);
    CHECK(da::own_violation_count() > 0);
    da::reset();
}
