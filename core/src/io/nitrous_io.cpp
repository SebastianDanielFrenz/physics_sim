#include "ps/io/nitrous_io.h"

#include "nitrous_components.h"
#include "ps/drivetrain/isa_air.h"
#include <filesystem>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps::io {
namespace {

using json = nlohmann::json;
using ps::drivetrain::OxidizerDesc;

json read_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    json j;
    try {
        in >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + path + ": " + e.what());
    }
    return j;
}

// Same hard-error rule as every other loader (no legacy/missing/unknown format).
void require_format(const json& j, const std::string& kind, int major, const std::string& path) {
    const std::string expected = "physics_sim." + kind + "/" + std::to_string(major);
    if (!j.is_object()) throw std::runtime_error(path + ": top level must be a JSON object");
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        throw std::runtime_error(path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
    }
}

// New-block rule: unknown keys are errors, not silently ignored.
void reject_unknown_keys(const json& j, const std::vector<std::string>& allowed, const std::string& ctx) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        bool known = false;
        for (const std::string& a : allowed) known = known || a == it.key();
        if (!known) throw std::runtime_error(ctx + ": unknown key '" + it.key() + "'");
    }
}

real require_number(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_number()) {
        throw std::runtime_error(ctx + ": '" + key + "' is required and must be a number");
    }
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v)) throw std::runtime_error(ctx + ": '" + key + "' must be finite");
    return v;
}

std::string require_string(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_string() || j.at(key).get<std::string>().empty()) {
        throw std::runtime_error(ctx + ": '" + key + "' is required and must be a non-empty string");
    }
    return j.at(key).get<std::string>();
}

std::vector<real> require_array(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_array()) {
        throw std::runtime_error(ctx + ": '" + key + "' is required and must be an array of numbers");
    }
    std::vector<real> out;
    out.reserve(j.at(key).size());
    for (const auto& e : j.at(key)) {
        if (!e.is_number()) throw std::runtime_error(ctx + ": '" + key + "' must contain only numbers");
        const real v = e.get<real>();
        if (!std::isfinite(v)) throw std::runtime_error(ctx + ": '" + key + "' must contain only finite numbers");
        out.push_back(v);
    }
    return out;
}

void require_strictly_ascending(const std::vector<real>& v, const char* key, const std::string& ctx) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        if (!(v[i] > v[i - 1])) {
            throw std::runtime_error(ctx + ": '" + key + "' must be strictly ascending (row " + std::to_string(i) + ")");
        }
    }
}

void require_all_positive(const std::vector<real>& v, const char* key, const std::string& ctx) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!(v[i] > real(0.0))) throw std::runtime_error(ctx + ": '" + key + "' must be > 0 (row " + std::to_string(i) + ")");
    }
}

void require_length(const std::vector<real>& v, std::size_t n, const char* key, const std::string& ctx) {
    if (v.size() != n) {
        throw std::runtime_error(ctx + ": '" + key + "' has " + std::to_string(v.size()) + " rows, expected " +
                                  std::to_string(n) + " (every column must have as many rows as 't_k')");
    }
}

// Mass of O2 per mole of O2, for the cross-check of oxygen_mass_fraction only.
constexpr real kO2MolarMassG = 31.998;


// ---- N1 part-file helpers ---------------------------------------------------

using ps::drivetrain::N2oBottleDesc;
using ps::drivetrain::N2oHeaterDesc;
using ps::drivetrain::N2oPressureDesc;
using ps::drivetrain::N2oPressureMode;
using ps::drivetrain::NitrousControllerDesc;
using ps::drivetrain::NitrousKitDesc;

constexpr real kIsaGaugeReferenceBar = ps::drivetrain::kIsaAirPressurePa * real(1e-5); // kit gauges are read at sea level

std::string resolve_path(const std::string& base_dir, const std::string& p) {
    const std::filesystem::path fp(p);
    if (fp.is_absolute()) return p;
    return (std::filesystem::path(base_dir) / fp).string();
}

std::string dir_of(const std::string& path) { return std::filesystem::path(path).parent_path().string(); }

// The part files are in bar (like turbo.md): a pascal or psi key is a unit slip, not an alias.
void reject_unit_keys(const json& j, const std::string& ctx) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        const bool pa = k.size() >= 3 && k.compare(k.size() - 3, 3, "_pa") == 0;
        const bool psi = k.size() >= 4 && k.compare(k.size() - 4, 4, "_psi") == 0;
        if (pa || psi) throw std::runtime_error(ctx + ": key '" + k + "' is not accepted - pressures are in bar (e.g. 'gauge_bar')");
    }
}

real optional_number(const json& j, const char* key, real fallback, const std::string& ctx) {
    if (!j.contains(key)) return fallback;
    return require_number(j, key, ctx);
}

int require_int(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_number_integer()) {
        throw std::runtime_error(ctx + ": '" + key + "' is required and must be an integer");
    }
    return j.at(key).get<int>();
}

int optional_int(const json& j, const char* key, int fallback, const std::string& ctx) {
    if (!j.contains(key)) return fallback;
    return require_int(j, key, ctx);
}

const json& require_object(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_object()) throw std::runtime_error(ctx + ": '" + key + "' is required and must be an object");
    return j.at(key);
}

// A saturated bottle state must lie inside the oxidizer table (triple point to 0.98 of the last row's pressure).
void check_abs_bar(real p_bar_abs, const OxidizerDesc& ox, const std::string& what) {
    const real lo = ox.saturation.p_bar.front();
    const real hi = real(0.98) * ox.saturation.p_bar.back();
    if (!(p_bar_abs >= lo && p_bar_abs <= hi)) {
        throw std::runtime_error(what + ": absolute pressure " + std::to_string(p_bar_abs) + " bar is outside the oxidizer table's usable range [" +
                                  std::to_string(lo) + ", " + std::to_string(hi) + "] bar (triple point to 0.98 of the last row)");
    }
}

void check_temperature_k(real t_k, const OxidizerDesc& ox, const std::string& what) {
    const auto& tk = ox.saturation.t_k;
    if (!(t_k >= tk.front() && t_k <= tk.back())) {
        throw std::runtime_error(what + ": temperature " + std::to_string(t_k) + " K is outside the oxidizer table [" +
                                  std::to_string(tk.front()) + ", " + std::to_string(tk.back()) + "] K");
    }
    // p_sat(T), linear in the table, must also be in the usable pressure window.
    std::size_t hi = 1;
    while (hi + 1 < tk.size() && tk[hi] < t_k) ++hi;
    const real f = (t_k - tk[hi - 1]) / (tk[hi] - tk[hi - 1]);
    const real p = ox.saturation.p_bar[hi - 1] + f * (ox.saturation.p_bar[hi] - ox.saturation.p_bar[hi - 1]);
    check_abs_bar(p, ox, what);
}

N2oPressureDesc read_pressure_block(const json& pj, const OxidizerDesc& ox, const std::string& ctx) {
    if (!pj.is_object()) throw std::runtime_error(ctx + ": 'pressure' must be an object");
    reject_unit_keys(pj, ctx);
    reject_unknown_keys(pj, {"mode", "gauge_bar", "fill_fraction", "temperature_k"}, ctx);
    const std::string mode = require_string(pj, "mode", ctx);
    N2oPressureDesc d;
    if (mode == "constant") {
        d.mode = N2oPressureMode::Constant;
        if (pj.contains("fill_fraction") || pj.contains("temperature_k")) {
            throw std::runtime_error(ctx + ": mode \"constant\" takes only 'gauge_bar'");
        }
        d.gauge_bar = require_number(pj, "gauge_bar", ctx);
        check_abs_bar(d.gauge_bar + kIsaGaugeReferenceBar, ox, ctx + ".gauge_bar");
    } else if (mode == "vs_fill") {
        d.mode = N2oPressureMode::VsFill;
        if (pj.contains("temperature_k")) throw std::runtime_error(ctx + ": mode \"vs_fill\" takes only 'fill_fraction' and 'gauge_bar'");
        d.gauge_vs_fill.x = require_array(pj, "fill_fraction", ctx);
        d.gauge_vs_fill.y = require_array(pj, "gauge_bar", ctx);
        if (d.gauge_vs_fill.x.size() < 2 || d.gauge_vs_fill.x.size() != d.gauge_vs_fill.y.size()) {
            throw std::runtime_error(ctx + ": 'fill_fraction' and 'gauge_bar' must be arrays of the same length (at least 2)");
        }
        require_strictly_ascending(d.gauge_vs_fill.x, "fill_fraction", ctx);
        if (!(d.gauge_vs_fill.x.front() == real(0.0) && d.gauge_vs_fill.x.back() == real(1.0))) {
            throw std::runtime_error(ctx + ": 'fill_fraction' must span exactly 0 to 1");
        }
        for (std::size_t i = 0; i < d.gauge_vs_fill.y.size(); ++i) {
            check_abs_bar(d.gauge_vs_fill.y[i] + kIsaGaugeReferenceBar, ox, ctx + ".gauge_bar[" + std::to_string(i) + "]");
        }
    } else if (mode == "temperature") {
        d.mode = N2oPressureMode::Temperature;
        if (pj.contains("fill_fraction") || pj.contains("gauge_bar")) {
            throw std::runtime_error(ctx + ": mode \"temperature\" takes only 'temperature_k'");
        }
        d.temperature_k = require_number(pj, "temperature_k", ctx);
        check_temperature_k(d.temperature_k, ox, ctx + ".temperature_k");
    } else {
        throw std::runtime_error(ctx + ": unknown pressure mode \"" + mode + "\" (expected \"constant\", \"vs_fill\" or \"temperature\")");
    }
    return d;
}

N2oHeaterDesc read_heater_block(const json& hj, const OxidizerDesc& ox, const std::string& ctx) {
    if (!hj.is_object()) throw std::runtime_error(ctx + ": 'heater' must be an object");
    reject_unit_keys(hj, ctx);
    reject_unknown_keys(hj, {"channel", "default_on", "set_point_gauge_bar", "set_point_k"}, ctx);
    N2oHeaterDesc h;
    h.channel = require_string(hj, "channel", ctx);
    h.default_on = optional_number(hj, "default_on", real(0.0), ctx);
    const bool has_bar = hj.contains("set_point_gauge_bar");
    const bool has_k = hj.contains("set_point_k");
    if (has_bar == has_k) throw std::runtime_error(ctx + ": exactly one of 'set_point_gauge_bar' and 'set_point_k' is required");
    if (has_bar) {
        h.set_point_gauge_bar = require_number(hj, "set_point_gauge_bar", ctx);
        check_abs_bar(h.set_point_gauge_bar + kIsaGaugeReferenceBar, ox, ctx + ".set_point_gauge_bar");
    } else {
        h.set_point_is_temperature = true;
        h.set_point_k = require_number(hj, "set_point_k", ctx);
        check_temperature_k(h.set_point_k, ox, ctx + ".set_point_k");
    }
    return h;
}

void check_positive_mm_and_cd(real d_mm, real cd, const std::string& ctx, const char* d_key, const char* cd_key) {
    if (!(d_mm > real(0.0))) throw std::runtime_error(ctx + ": '" + d_key + "' must be > 0");
    if (!(cd > real(0.0) && cd <= real(1.0))) throw std::runtime_error(ctx + ": '" + cd_key + "' must be in (0, 1]");
}

N2oBottleDesc read_bottle_object(const json& j, const std::string& path) {
    const std::string ctx = "n2o_bottle " + path;
    reject_unit_keys(j, ctx);
    reject_unknown_keys(j, {"format", "oxidizer", "capacity_kg", "pressure", "heater", "outlet", "source"}, ctx);
    N2oBottleDesc b;
    b.oxidizer = ps::io::load_oxidizer_json(resolve_path(dir_of(path), require_string(j, "oxidizer", ctx)));
    b.capacity_kg = require_number(j, "capacity_kg", ctx);
    if (!(b.capacity_kg > real(0.0))) throw std::runtime_error(ctx + ": 'capacity_kg' must be > 0");
    b.initial_kg = b.capacity_kg;
    b.pressure = read_pressure_block(require_object(j, "pressure", ctx), b.oxidizer, ctx + ".pressure");
    if (j.contains("heater")) {
        b.has_heater = true;
        b.heater = read_heater_block(j.at("heater"), b.oxidizer, ctx + ".heater");
    }
    if (j.contains("outlet")) {
        const std::string outlet = require_string(j, "outlet", ctx);
        if (outlet != "siphon") throw std::runtime_error(ctx + ": outlet \"" + outlet + "\" is not supported (only \"siphon\", a liquid draw)");
    }
    b.source = require_string(j, "source", ctx);
    return b;
}

NitrousControllerDesc read_controller_object(const json& j, const std::string& path) {
    const std::string ctx = "nitrous_controller " + path;
    reject_unit_keys(j, ctx);
    reject_unknown_keys(j, {"format", "arm_channel", "arm_default", "min_throttle_command", "min_rpm", "max_rpm", "min_gear", "max_gear",
                             "min_speed_mps", "progressive", "spark_retard_deg", "safety", "purge", "source"}, ctx);
    NitrousControllerDesc c;
    c.arm_channel = require_string(j, "arm_channel", ctx);
    c.arm_default = optional_number(j, "arm_default", real(0.0), ctx);
    c.min_throttle_command = optional_number(j, "min_throttle_command", c.min_throttle_command, ctx);
    if (!(c.min_throttle_command >= real(0.0) && c.min_throttle_command <= real(1.0))) {
        throw std::runtime_error(ctx + ": 'min_throttle_command' must be in [0, 1]");
    }
    c.min_rpm = require_number(j, "min_rpm", ctx);
    c.max_rpm = require_number(j, "max_rpm", ctx);
    if (!(c.min_rpm >= real(0.0))) throw std::runtime_error(ctx + ": 'min_rpm' must be >= 0");
    if (!(c.max_rpm > c.min_rpm)) throw std::runtime_error(ctx + ": 'max_rpm' must be above 'min_rpm'");
    c.min_gear = optional_int(j, "min_gear", c.min_gear, ctx);
    c.max_gear = optional_int(j, "max_gear", c.max_gear, ctx);
    if (c.max_gear < c.min_gear) throw std::runtime_error(ctx + ": 'max_gear' must be >= 'min_gear'");
    c.min_speed_mps = optional_number(j, "min_speed_mps", real(0.0), ctx);
    if (!(c.min_speed_mps >= real(0.0))) throw std::runtime_error(ctx + ": 'min_speed_mps' must be >= 0");

    if (j.contains("progressive")) {
        const json& pj = j.at("progressive");
        const std::string pctx = ctx + ".progressive";
        if (!pj.is_object()) throw std::runtime_error(pctx + ": must be an object");
        reject_unknown_keys(pj, {"start_fraction", "delay_s", "ramp_s"}, pctx);
        c.progressive.start_fraction = optional_number(pj, "start_fraction", real(1.0), pctx);
        c.progressive.delay_s = optional_number(pj, "delay_s", real(0.0), pctx);
        c.progressive.ramp_s = optional_number(pj, "ramp_s", real(0.0), pctx);
        if (!(c.progressive.start_fraction > real(0.0) && c.progressive.start_fraction <= real(1.0))) {
            throw std::runtime_error(pctx + ": 'start_fraction' must be in (0, 1]");
        }
        if (!(c.progressive.delay_s >= real(0.0))) throw std::runtime_error(pctx + ": 'delay_s' must be >= 0");
        if (!(c.progressive.ramp_s >= real(0.0))) throw std::runtime_error(pctx + ": 'ramp_s' must be >= 0");
    }

    if (j.contains("spark_retard_deg")) {
        const json& rj = j.at("spark_retard_deg");
        const std::string rctx = ctx + ".spark_retard_deg";
        if (rj.is_number()) {
            c.spark_retard_deg = rj.get<real>();
            if (!std::isfinite(c.spark_retard_deg) || c.spark_retard_deg < real(0.0)) {
                throw std::runtime_error(rctx + ": must be finite and >= 0");
            }
        } else if (rj.is_object()) {
            reject_unknown_keys(rj, {"fraction", "deg"}, rctx);
            c.retard_is_curve = true;
            c.retard_vs_fraction.x = require_array(rj, "fraction", rctx);
            c.retard_vs_fraction.y = require_array(rj, "deg", rctx);
            if (c.retard_vs_fraction.x.size() < 2 || c.retard_vs_fraction.x.size() != c.retard_vs_fraction.y.size()) {
                throw std::runtime_error(rctx + ": 'fraction' and 'deg' must be arrays of the same length (at least 2)");
            }
            require_strictly_ascending(c.retard_vs_fraction.x, "fraction", rctx);
            if (c.retard_vs_fraction.x.front() < real(0.0) || c.retard_vs_fraction.x.back() > real(1.0)) {
                throw std::runtime_error(rctx + ": 'fraction' must lie in [0, 1]");
            }
            for (real v : c.retard_vs_fraction.y) {
                if (v < real(0.0)) throw std::runtime_error(rctx + ": 'deg' must be >= 0");
            }
        } else {
            throw std::runtime_error(rctx + ": must be a number or a {\"fraction\": [...], \"deg\": [...]} curve");
        }
    }

    if (j.contains("safety")) {
        const json& sj = j.at("safety");
        const std::string sctx = ctx + ".safety";
        if (!sj.is_object()) throw std::runtime_error(sctx + ": must be an object");
        reject_unknown_keys(sj, {"min_bottle_gauge_bar", "cut_on_fuel_starve", "max_active_s"}, sctx);
        c.safety.min_bottle_gauge_bar = optional_number(sj, "min_bottle_gauge_bar", real(0.0), sctx);
        if (!(c.safety.min_bottle_gauge_bar >= real(0.0))) throw std::runtime_error(sctx + ": 'min_bottle_gauge_bar' must be >= 0");
        if (sj.contains("cut_on_fuel_starve")) {
            const json& fj = sj.at("cut_on_fuel_starve");
            if (fj.is_boolean()) c.safety.cut_on_fuel_starve = fj.get<bool>();
            else if (fj.is_number()) c.safety.cut_on_fuel_starve = fj.get<real>() != real(0.0);
            else throw std::runtime_error(sctx + ": 'cut_on_fuel_starve' must be 0/1 or a boolean");
        }
        if (sj.contains("max_active_s")) {
            c.safety.max_active_s = require_number(sj, "max_active_s", sctx);
            if (!(c.safety.max_active_s > real(0.0))) throw std::runtime_error(sctx + ": 'max_active_s' must be > 0 (omit it for no limit)");
        }
    }

    if (j.contains("purge")) {
        const json& pj = j.at("purge");
        const std::string pctx = ctx + ".purge";
        if (!pj.is_object()) throw std::runtime_error(pctx + ": must be an object");
        reject_unknown_keys(pj, {"channel", "jet_diameter_mm", "cd"}, pctx);
        c.has_purge = true;
        c.purge.channel = require_string(pj, "channel", pctx);
        c.purge.jet_diameter_mm = require_number(pj, "jet_diameter_mm", pctx);
        c.purge.cd = require_number(pj, "cd", pctx);
        check_positive_mm_and_cd(c.purge.jet_diameter_mm, c.purge.cd, pctx, "jet_diameter_mm", "cd");
        if (c.purge.channel == c.arm_channel) throw std::runtime_error(pctx + ": the purge channel must differ from the arm channel");
    }

    c.source = require_string(j, "source", ctx);
    return c;
}

NitrousKitDesc read_kit_object(const json& j, const std::string& path, const std::string& controller_override_path) {
    const std::string ctx = "nitrous_kit " + path;
    reject_unit_keys(j, ctx);
    reject_unknown_keys(j, {"format", "type", "injection", "nozzle_count", "nitrous_jet_diameter_mm", "nitrous_jet_cd", "flow_model",
                             "fuel_jet_diameter_mm", "fuel_jet_cd", "fuel_pressure", "rating", "default_controller", "source"}, ctx);
    NitrousKitDesc k;
    const std::string type = require_string(j, "type", ctx);
    if (type == "dry") throw std::runtime_error(ctx + ": type \"dry\" is not supported yet (only \"wet\")");
    if (type != "wet") throw std::runtime_error(ctx + ": unknown type \"" + type + "\" (expected \"wet\")");
    k.type = drivetrain::NitrousKitType::Wet;

    const std::string injection = require_string(j, "injection", ctx);
    if (injection == "plenum") k.injection = drivetrain::NitrousInjection::Plenum;
    else if (injection == "direct_port") k.injection = drivetrain::NitrousInjection::DirectPort;
    else throw std::runtime_error(ctx + ": unknown injection \"" + injection + "\" (expected \"plenum\" or \"direct_port\")");

    k.nozzle_count = optional_int(j, "nozzle_count", 1, ctx);
    if (k.nozzle_count < 1) throw std::runtime_error(ctx + ": 'nozzle_count' must be >= 1");

    k.nitrous_jet_diameter_mm = require_number(j, "nitrous_jet_diameter_mm", ctx);
    k.nitrous_jet_cd = require_number(j, "nitrous_jet_cd", ctx);
    check_positive_mm_and_cd(k.nitrous_jet_diameter_mm, k.nitrous_jet_cd, ctx, "nitrous_jet_diameter_mm", "nitrous_jet_cd");
    if (j.contains("flow_model")) {
        const std::string fm = require_string(j, "flow_model", ctx);
        if (fm == "hem") k.flow_model = drivetrain::NitrousFlowModel::Hem;
        else if (fm == "dyer") k.flow_model = drivetrain::NitrousFlowModel::Dyer;
        else throw std::runtime_error(ctx + ": unknown flow_model \"" + fm + "\" (expected \"hem\" or \"dyer\")");
    }

    k.fuel_jet_diameter_mm = require_number(j, "fuel_jet_diameter_mm", ctx);
    k.fuel_jet_cd = require_number(j, "fuel_jet_cd", ctx);
    check_positive_mm_and_cd(k.fuel_jet_diameter_mm, k.fuel_jet_cd, ctx, "fuel_jet_diameter_mm", "fuel_jet_cd");
    {
        const json& fj = require_object(j, "fuel_pressure", ctx);
        const std::string fctx = ctx + ".fuel_pressure";
        reject_unit_keys(fj, fctx);
        reject_unknown_keys(fj, {"gauge_bar", "reference"}, fctx);
        k.fuel_pressure_gauge_bar = require_number(fj, "gauge_bar", fctx);
        if (!(k.fuel_pressure_gauge_bar >= real(0.0))) throw std::runtime_error(fctx + ": 'gauge_bar' must be >= 0");
        const std::string ref = require_string(fj, "reference", fctx);
        if (ref == "atmosphere") k.fuel_reference = drivetrain::NitrousFuelReference::Atmosphere;
        else if (ref == "manifold") k.fuel_reference = drivetrain::NitrousFuelReference::Manifold;
        else throw std::runtime_error(fctx + ": unknown reference \"" + ref + "\" (expected \"atmosphere\" or \"manifold\")");
    }

    if (j.contains("rating")) {
        const json& rj = j.at("rating");
        const std::string rctx = ctx + ".rating";
        if (!rj.is_object()) throw std::runtime_error(rctx + ": must be an object");
        reject_unit_keys(rj, rctx);
        reject_unknown_keys(rj, {"power_kw", "bottle_gauge_bar", "basis", "source"}, rctx);
        k.rating.power_kw = require_number(rj, "power_kw", rctx);
        k.rating.bottle_gauge_bar = require_number(rj, "bottle_gauge_bar", rctx);
        k.rating.basis = require_string(rj, "basis", rctx);
        k.rating.source = require_string(rj, "source", rctx);
        if (!(k.rating.power_kw > real(0.0))) throw std::runtime_error(rctx + ": 'power_kw' must be > 0");
        if (!(k.rating.bottle_gauge_bar > real(0.0))) throw std::runtime_error(rctx + ": 'bottle_gauge_bar' must be > 0");
        if (k.rating.basis != "crank" && k.rating.basis != "wheel") {
            throw std::runtime_error(rctx + ": 'basis' must be \"crank\" or \"wheel\"");
        }
    }

    std::string controller_path = controller_override_path;
    if (controller_path.empty() && j.contains("default_controller")) {
        controller_path = resolve_path(dir_of(path), require_string(j, "default_controller", ctx));
    }
    if (controller_path.empty()) {
        throw std::runtime_error(ctx + ": no controller - the kit has no 'default_controller' and the vehicle component gave no 'controller'");
    }
    k.controller = ps::io::load_nitrous_controller_json(controller_path);
    k.source = require_string(j, "source", ctx);
    return k;
}

// Vehicle component key whitelist: a ref'd part plus the documented overrides.
void check_component_keys(const json& cj, const std::vector<std::string>& allowed, const std::string& ctx) {
    reject_unknown_keys(cj, allowed, ctx);
    if (!cj.contains("ref") || !cj.at("ref").is_string() || cj.at("ref").get<std::string>().empty()) {
        throw std::runtime_error(ctx + ": 'ref' is required and must be a non-empty path string");
    }
}

} // namespace

drivetrain::OxidizerDesc load_oxidizer_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "oxidizer", 1, path);
    const std::string ctx = "oxidizer " + path;
    reject_unknown_keys(j, {"format", "name", "molar_mass_g_mol", "formation_enthalpy_kj_mol", "oxygen_mass_fraction",
                             "saturation", "gas_1bar", "source"}, ctx);

    OxidizerDesc d;
    d.name = require_string(j, "name", ctx);
    d.molar_mass_g_mol = require_number(j, "molar_mass_g_mol", ctx);
    if (!(d.molar_mass_g_mol > real(0.0))) throw std::runtime_error(ctx + ": 'molar_mass_g_mol' must be > 0");
    d.formation_enthalpy_kj_mol = require_number(j, "formation_enthalpy_kj_mol", ctx);
    d.oxygen_mass_fraction = require_number(j, "oxygen_mass_fraction", ctx);
    if (!(d.oxygen_mass_fraction > real(0.0) && d.oxygen_mass_fraction < real(1.0))) {
        throw std::runtime_error(ctx + ": 'oxygen_mass_fraction' must be in (0, 1), got " + std::to_string(d.oxygen_mass_fraction));
    }
    // Typo guard: 2 N2O -> 2 N2 + O2 fixes the O2 mass fraction at 0.5*M(O2)/M(N2O).
    const real o2_check = real(0.5) * kO2MolarMassG / d.molar_mass_g_mol;
    if (std::fabs(d.oxygen_mass_fraction - o2_check) > real(1e-4)) {
        throw std::runtime_error(ctx + ": 'oxygen_mass_fraction' " + std::to_string(d.oxygen_mass_fraction) +
                                  " disagrees with 0.5*31.998/molar_mass = " + std::to_string(o2_check) + " (tolerance 1e-4)");
    }

    if (!j.contains("saturation") || !j.at("saturation").is_object()) {
        throw std::runtime_error(ctx + ": 'saturation' is required and must be an object");
    }
    const json& sj = j.at("saturation");
    const std::string sctx = ctx + ".saturation";
    reject_unknown_keys(sj, {"t_k", "p_bar", "rho_liquid", "rho_vapour", "h_liquid_kj_kg", "h_vapour_kj_kg",
                              "s_liquid_j_gk", "s_vapour_j_gk"}, sctx);
    auto& s = d.saturation;
    s.t_k = require_array(sj, "t_k", sctx);
    if (s.t_k.size() < 3) throw std::runtime_error(sctx + ": needs at least 3 rows");
    const std::size_t n = s.t_k.size();
    s.p_bar = require_array(sj, "p_bar", sctx);
    s.rho_liquid = require_array(sj, "rho_liquid", sctx);
    s.rho_vapour = require_array(sj, "rho_vapour", sctx);
    s.h_liquid_kj_kg = require_array(sj, "h_liquid_kj_kg", sctx);
    s.h_vapour_kj_kg = require_array(sj, "h_vapour_kj_kg", sctx);
    s.s_liquid_j_gk = require_array(sj, "s_liquid_j_gk", sctx);
    s.s_vapour_j_gk = require_array(sj, "s_vapour_j_gk", sctx);
    require_length(s.p_bar, n, "p_bar", sctx);
    require_length(s.rho_liquid, n, "rho_liquid", sctx);
    require_length(s.rho_vapour, n, "rho_vapour", sctx);
    require_length(s.h_liquid_kj_kg, n, "h_liquid_kj_kg", sctx);
    require_length(s.h_vapour_kj_kg, n, "h_vapour_kj_kg", sctx);
    require_length(s.s_liquid_j_gk, n, "s_liquid_j_gk", sctx);
    require_length(s.s_vapour_j_gk, n, "s_vapour_j_gk", sctx);
    require_strictly_ascending(s.t_k, "t_k", sctx);
    require_strictly_ascending(s.p_bar, "p_bar", sctx);
    require_all_positive(s.t_k, "t_k", sctx);
    require_all_positive(s.p_bar, "p_bar", sctx);
    require_all_positive(s.rho_liquid, "rho_liquid", sctx);
    require_all_positive(s.rho_vapour, "rho_vapour", sctx);
    for (std::size_t i = 0; i < n; ++i) {
        // The HEM quality (s - s_l)/(s_v - s_l) and the latent heat need s_v > s_l and h_v > h_l.
        if (!(s.s_vapour_j_gk[i] > s.s_liquid_j_gk[i])) {
            throw std::runtime_error(sctx + ": 's_vapour_j_gk' must exceed 's_liquid_j_gk' (row " + std::to_string(i) + ")");
        }
        if (!(s.h_vapour_kj_kg[i] > s.h_liquid_kj_kg[i])) {
            throw std::runtime_error(sctx + ": 'h_vapour_kj_kg' must exceed 'h_liquid_kj_kg' (row " + std::to_string(i) + ")");
        }
    }

    if (!j.contains("gas_1bar") || !j.at("gas_1bar").is_object()) {
        throw std::runtime_error(ctx + ": 'gas_1bar' is required and must be an object");
    }
    const json& gj = j.at("gas_1bar");
    const std::string gctx = ctx + ".gas_1bar";
    reject_unknown_keys(gj, {"t_k", "h_kj_kg"}, gctx);
    d.gas_1bar.t_k = require_array(gj, "t_k", gctx);
    d.gas_1bar.h_kj_kg = require_array(gj, "h_kj_kg", gctx);
    if (d.gas_1bar.t_k.size() < 2) throw std::runtime_error(gctx + ": needs at least 2 rows");
    require_length(d.gas_1bar.h_kj_kg, d.gas_1bar.t_k.size(), "h_kj_kg", gctx);
    require_strictly_ascending(d.gas_1bar.t_k, "t_k", gctx);
    require_strictly_ascending(d.gas_1bar.h_kj_kg, "h_kj_kg", gctx); // enthalpy of a gas rises with T

    d.source = require_string(j, "source", ctx);
    return d;
}


drivetrain::N2oBottleDesc load_n2o_bottle_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "n2o_bottle", 1, path);
    return read_bottle_object(j, path);
}

drivetrain::NitrousControllerDesc load_nitrous_controller_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "nitrous_controller", 1, path);
    return read_controller_object(j, path);
}

drivetrain::NitrousKitDesc load_nitrous_kit_json(const std::string& path, const std::string& controller_override_path) {
    const json j = read_json_file(path);
    require_format(j, "nitrous_kit", 1, path);
    return read_kit_object(j, path, controller_override_path);
}

namespace detail {

drivetrain::N2oBottleDesc build_n2o_bottle_component(const json& cj, const std::string& base_dir, const std::string& ctx) {
    check_component_keys(cj, {"id", "type", "ref", "initial_kg", "pressure"}, ctx);
    const std::string path = resolve_path(base_dir, cj.at("ref").get<std::string>());
    N2oBottleDesc b = load_n2o_bottle_json(path);
    if (cj.contains("initial_kg")) {
        b.initial_kg = require_number(cj, "initial_kg", ctx);
        if (b.initial_kg < real(0.0)) throw std::runtime_error(ctx + ": 'initial_kg' must not be negative");
        if (b.initial_kg > b.capacity_kg) {
            throw std::runtime_error(ctx + ": 'initial_kg' (" + std::to_string(b.initial_kg) + ") must not exceed the bottle's capacity_kg (" +
                                      std::to_string(b.capacity_kg) + ")");
        }
    }
    if (cj.contains("pressure")) b.pressure = read_pressure_block(cj.at("pressure"), b.oxidizer, ctx + ".pressure");
    return b;
}

drivetrain::NitrousKitDesc build_nitrous_kit_component(const json& cj, const std::string& base_dir, const std::string& ctx) {
    check_component_keys(cj, {"id", "type", "ref", "controller"}, ctx);
    std::string controller_path;
    if (cj.contains("controller")) controller_path = resolve_path(base_dir, require_string(cj, "controller", ctx));
    return load_nitrous_kit_json(resolve_path(base_dir, cj.at("ref").get<std::string>()), controller_path);
}

} // namespace detail

} // namespace ps::io
