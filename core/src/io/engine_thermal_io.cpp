// Loaders of physics_sim.engine_thermal/1 and physics_sim.cooling_system/1
// (engine thermal plan T1 sections 9.2/9.3). Every rejection message names
// the file. No exception object is caught here (clang-cl ASan catch caveat).

#include "ps/io/engine_thermal_io.h"

#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ps::io {
namespace {

using json = nlohmann::json;
using ps::drivetrain::CoolingSystemDesc;
using ps::drivetrain::EngineThermalDesc;
using ps::drivetrain::kCelsiusToKelvin;

class Reader {
public:
    Reader(const json& obj, std::string ctx, const std::string& origin) : obj_(obj), ctx_(std::move(ctx)), origin_(origin) {
        if (!obj.is_object()) fail("must be an object");
    }
    [[noreturn]] void fail(const std::string& msg) const {
        throw std::runtime_error(origin_ + ": " + (ctx_.empty() ? "" : ctx_ + ": ") + msg);
    }
    bool has(const char* key) const { return obj_.contains(key); }
    void mark(const char* key) { used_.push_back(key); }

    real number(const char* key, bool required, real fallback) {
        mark(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_number()) fail(std::string("\"") + key + "\" must be a number");
        const real v = it->get<real>();
        if (!std::isfinite(v)) fail(std::string("\"") + key + "\" must be finite");
        return v;
    }
    std::string string(const char* key, bool required) {
        mark(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return {};
        }
        if (!it->is_string()) fail(std::string("\"") + key + "\" must be a string");
        return it->get<std::string>();
    }
    const json* child(const char* key) {
        mark(key);
        auto it = obj_.find(key);
        return it == obj_.end() ? nullptr : &*it;
    }
    const json& object(const char* key) {
        const json* c = child(key);
        if (c == nullptr) fail(std::string("missing required field \"") + key + "\"");
        if (!c->is_object()) fail(std::string("\"") + key + "\" must be an object");
        return *c;
    }
    std::vector<real> numbers(const char* key) {
        mark(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) fail(std::string("missing required field \"") + key + "\"");
        if (!it->is_array()) fail(std::string("\"") + key + "\" must be an array of numbers");
        std::vector<real> out;
        for (const auto& e : *it) {
            if (!e.is_number()) fail(std::string("\"") + key + "\" must contain only numbers");
            const real v = e.get<real>();
            if (!std::isfinite(v)) fail(std::string("\"") + key + "\" must contain only finite numbers");
            out.push_back(v);
        }
        return out;
    }
    void finish() const {
        for (auto it = obj_.begin(); it != obj_.end(); ++it) {
            bool known = false;
            for (const auto& u : used_) known = known || u == it.key();
            if (!known) fail("unknown key \"" + it.key() + "\"");
        }
    }

private:
    const json& obj_;
    std::string ctx_;
    const std::string& origin_;
    std::vector<std::string> used_;
};

void require_format(const json& j, const char* kind, const std::string& origin) {
    const std::string expected = std::string("physics_sim.") + kind + "/1";
    if (!j.is_object()) throw std::runtime_error(origin + ": top level must be a JSON object");
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(origin + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        throw std::runtime_error(origin + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
    }
}

// Exactly one of `absolute_key` / `per_litre_key` (an object of numbers each).
struct Scaled {
    const json* obj = nullptr;
    real factor = 1.0;
    bool per_litre = false;
};

Scaled pick_scaled(Reader& r, const char* absolute_key, const char* per_litre_key, real displacement_l) {
    const bool a = r.has(absolute_key), p = r.has(per_litre_key);
    if (a == p) {
        r.fail(std::string("exactly one of \"") + absolute_key + "\" and \"" + per_litre_key + "\" is required");
    }
    Scaled s;
    s.per_litre = p;
    s.obj = r.child(p ? per_litre_key : absolute_key);
    r.mark(p ? absolute_key : per_litre_key);
    if (!s.obj->is_object()) r.fail(std::string("\"") + (p ? per_litre_key : absolute_key) + "\" must be an object");
    if (p) {
        if (!(displacement_l > 0.0)) r.fail(std::string("\"") + per_litre_key + "\" requires \"displacement_l\" > 0");
        s.factor = displacement_l;
    }
    return s;
}

} // namespace

drivetrain::ThermalStartKind parse_thermal_start(const std::string& value, const std::string& context) {
    if (value == "warm") return drivetrain::ThermalStartKind::Warm;
    if (value == "ambient") return drivetrain::ThermalStartKind::Ambient;
    throw std::runtime_error(context + ": 'engine_thermal_start' must be \"warm\" or \"ambient\", got \"" + value + "\"");
}

EngineThermalDesc load_engine_thermal_json(const std::string& path) {
    const json root = load_json_with_extends(std::filesystem::path(path));
    require_format(root, "engine_thermal", path);
    Reader r(root, "", path);
    r.mark("format");

    EngineThermalDesc d;
    d.source = r.string("source", true);
    if (d.source.empty()) r.fail("'source' must not be empty (PLAN.md section 8)");
    r.mark("ecu");
    if (r.has("ecu")) r.fail("\"ecu\" is reserved for engine thermal increment T2 and is not implemented yet; remove it");
    d.displacement_l = r.number("displacement_l", false, 0.0);
    if (r.has("displacement_l") && !(d.displacement_l > 0.0)) r.fail("\"displacement_l\" must be > 0");

    {
        const Scaled s = pick_scaled(r, "capacity", "capacity_per_l", d.displacement_l);
        Reader c(*s.obj, s.per_litre ? "capacity_per_l" : "capacity", path);
        d.engine_capacity_j_k = c.number("engine_j_k", true, 0.0) * s.factor;
        d.oil_capacity_j_k = c.number("oil_j_k", true, 0.0) * s.factor;
        c.finish();
        if (!(d.engine_capacity_j_k > 0.0) || !(d.oil_capacity_j_k > 0.0)) r.fail("capacities must be > 0");
    }

    d.wall_heat_fraction = r.number("wall_heat_fraction", true, 0.0);
    if (!(d.wall_heat_fraction > 0.0 && d.wall_heat_fraction < 0.5)) r.fail("\"wall_heat_fraction\" must be in (0, 0.5)");
    d.friction_to_oil_fraction = r.number("friction_to_oil_fraction", true, 0.0);
    if (!(d.friction_to_oil_fraction >= 0.0 && d.friction_to_oil_fraction <= 1.0)) {
        r.fail("\"friction_to_oil_fraction\" must be in [0, 1]");
    }
    d.motoring_friction_fraction = r.number("motoring_friction_fraction", false, 1.0);
    if (!(d.motoring_friction_fraction > 0.0 && d.motoring_friction_fraction <= 1.0)) {
        r.fail("\"motoring_friction_fraction\" must be in (0, 1]");
    }

    {
        Reader o(r.object("oil"), "oil", path);
        d.oil.kv40_mm2_s = o.number("kv40_mm2_s", true, 0.0);
        d.oil.kv100_mm2_s = o.number("kv100_mm2_s", true, 0.0);
        d.oil.density_15c_kg_m3 = o.number("density_15c_kg_m3", true, 0.0);
        const real t_ref_c = o.number("reference_temperature_c", true, 0.0);
        d.oil.hydrodynamic_fraction = o.number("hydrodynamic_fraction", true, 0.0);
        o.mark("friction_scale_max");
        if (o.has("friction_scale_max")) {
            o.fail("\"friction_scale_max\" is rejected: the owner ruled on 2026-10-05 that there is NO friction cap (the "
                   "floor \"friction_scale_min\" stays); test I9 guards cold cranking instead");
        }
        d.oil.friction_scale_min = o.number("friction_scale_min", false, 1.0);
        o.finish();
        if (!(d.oil.kv100_mm2_s > 0.0 && d.oil.kv40_mm2_s > d.oil.kv100_mm2_s)) {
            o.fail("requires kv40_mm2_s > kv100_mm2_s > 0");
        }
        if (!(d.oil.density_15c_kg_m3 > 0.0)) o.fail("\"density_15c_kg_m3\" must be > 0");
        if (!(t_ref_c >= 60.0 && t_ref_c <= 140.0)) o.fail("\"reference_temperature_c\" must be in [60, 140]");
        d.oil.reference_temperature_k = t_ref_c + kCelsiusToKelvin;
        if (!(d.oil.hydrodynamic_fraction > 0.0 && d.oil.hydrodynamic_fraction <= 1.0)) {
            o.fail("\"hydrodynamic_fraction\" must be in (0, 1]");
        }
        if (!(d.oil.friction_scale_min >= 1.0)) o.fail("\"friction_scale_min\" must be >= 1");
    }

    {
        const Scaled s = pick_scaled(r, "conductance", "conductance_per_l", d.displacement_l);
        Reader c(*s.obj, s.per_litre ? "conductance_per_l" : "conductance", path);
        // The time constant is not a per-litre quantity: G_oc = C_oil / tau already scales with C_oil.
        d.oil_coolant_time_constant_s = c.number("oil_coolant_time_constant_s", true, 0.0);
        d.block_air_w_k = c.number("block_air_w_k", true, 0.0) * s.factor;
        d.block_air_w_k_per_m_s = c.number("block_air_w_k_per_m_s", true, 0.0) * s.factor;
        d.oil_air_w_k = c.number("oil_air_w_k", true, 0.0) * s.factor;
        d.oil_air_w_k_per_m_s = c.number("oil_air_w_k_per_m_s", true, 0.0) * s.factor;
        c.finish();
        if (!(d.oil_coolant_time_constant_s > 0.0)) r.fail("\"oil_coolant_time_constant_s\" must be > 0");
        if (d.block_air_w_k < 0.0 || d.block_air_w_k_per_m_s < 0.0 || d.oil_air_w_k < 0.0 || d.oil_air_w_k_per_m_s < 0.0) {
            r.fail("conductances must be >= 0");
        }
    }

    {
        Reader c(r.object("coolant"), "coolant", path);
        d.coolant_cp_j_kg_k = c.number("cp_j_kg_k", true, 0.0);
        d.coolant_density_kg_m3 = c.number("density_kg_m3", true, 0.0);
        c.finish();
        if (!(d.coolant_cp_j_kg_k > 0.0) || !(d.coolant_density_kg_m3 > 0.0)) r.fail("coolant cp and density must be > 0");
    }

    {
        Reader p(r.object("pump"), "pump", path);
        const bool a = p.has("l_min_per_krpm"), l = p.has("l_min_per_krpm_per_l");
        if (a == l) p.fail("exactly one of \"l_min_per_krpm\" and \"l_min_per_krpm_per_l\" is required");
        if (l) {
            if (!(d.displacement_l > 0.0)) p.fail("\"l_min_per_krpm_per_l\" requires \"displacement_l\" > 0");
            d.pump_l_min_per_krpm = p.number("l_min_per_krpm_per_l", true, 0.0) * d.displacement_l;
            p.mark("l_min_per_krpm");
        } else {
            d.pump_l_min_per_krpm = p.number("l_min_per_krpm", true, 0.0);
            p.mark("l_min_per_krpm_per_l");
        }
        p.finish();
        if (!(d.pump_l_min_per_krpm > 0.0)) p.fail("pump flow must be > 0");
    }

    {
        Reader t(r.object("thermostat"), "thermostat", path);
        const real open_c = t.number("open_c", true, 0.0);
        const real full_c = t.number("full_open_c", true, 0.0);
        d.thermostat.hysteresis_k = t.number("hysteresis_k", true, 0.0);
        d.thermostat.time_constant_s = t.number("time_constant_s", true, 0.0);
        t.finish();
        if (!(open_c < full_c)) t.fail("requires open_c < full_open_c");
        if (!(d.thermostat.hysteresis_k >= 0.0)) t.fail("\"hysteresis_k\" must be >= 0");
        if (!(d.thermostat.time_constant_s > 0.0)) t.fail("\"time_constant_s\" must be > 0");
        d.thermostat.open_k = open_c + kCelsiusToKelvin;
        d.thermostat.full_open_k = full_c + kCelsiusToKelvin;
    }

    r.finish();
    return d;
}

CoolingSystemDesc load_cooling_system_json(const std::string& path) {
    const json root = load_json_with_extends(std::filesystem::path(path));
    require_format(root, "cooling_system", path);
    Reader r(root, "", path);
    r.mark("format");

    CoolingSystemDesc d;
    d.source = r.string("source", true);
    if (d.source.empty()) r.fail("'source' must not be empty (PLAN.md section 8)");
    r.mark("heater_core");
    if (r.has("heater_core")) r.fail("\"heater_core\" is reserved for engine thermal increment T7 and is not implemented yet; remove it");

    {
        Reader c(r.object("radiator"), "radiator", path);
        d.radiator.frontal_area_m2 = c.number("frontal_area_m2", true, 0.0);
        d.radiator.ram_coefficient = c.number("ram_coefficient", true, 0.0);
        d.radiator.ua_ref_w_k = c.number("ua_ref_w_k", true, 0.0);
        d.radiator.air_mass_flow_ref_kg_s = c.number("air_mass_flow_ref_kg_s", true, 0.0);
        d.radiator.ua_exponent = c.number("ua_exponent", false, 0.6);
        c.finish();
        if (!(d.radiator.frontal_area_m2 > 0.0)) c.fail("\"frontal_area_m2\" must be > 0");
        if (!(d.radiator.ram_coefficient >= 0.0 && d.radiator.ram_coefficient <= 1.0)) {
            c.fail("\"ram_coefficient\" must be in [0, 1]");
        }
        if (!(d.radiator.ua_ref_w_k > 0.0) || !(d.radiator.air_mass_flow_ref_kg_s > 0.0)) {
            c.fail("\"ua_ref_w_k\" and \"air_mass_flow_ref_kg_s\" must be > 0");
        }
        if (!(d.radiator.ua_exponent > 0.0 && d.radiator.ua_exponent <= 1.0)) c.fail("\"ua_exponent\" must be in (0, 1]");
    }
    {
        Reader c(r.object("fan"), "fan", path);
        d.fan.face_velocity_m_s = c.number("face_velocity_m_s", true, 0.0);
        const real on_c = c.number("on_c", true, 0.0);
        const real off_c = c.number("off_c", true, 0.0);
        d.fan.power_w = c.number("power_w", true, 0.0);
        c.finish();
        if (!(off_c < on_c)) c.fail("requires off_c < on_c");
        if (!(d.fan.face_velocity_m_s >= 0.0) || !(d.fan.power_w >= 0.0)) c.fail("fan velocity and power must be >= 0");
        d.fan.on_k = on_c + kCelsiusToKelvin;
        d.fan.off_k = off_c + kCelsiusToKelvin;
    }
    if (const json* oc = r.child("oil_cooler")) {
        Reader c(*oc, "oil_cooler", path);
        d.oil_cooler_air_w_k_per_m_s = c.number("air_w_k_per_m_s", false, 0.0);
        c.finish();
        if (!(d.oil_cooler_air_w_k_per_m_s >= 0.0)) c.fail("\"air_w_k_per_m_s\" must be >= 0");
    }
    {
        Reader c(r.object("gauge"), "gauge", path);
        const std::vector<real> xc = c.numbers("x_c");
        const std::vector<real> y = c.numbers("y");
        c.finish();
        if (xc.size() < 2 || xc.size() != y.size()) c.fail("\"x_c\" and \"y\" must have the same length >= 2");
        for (std::size_t i = 0; i < xc.size(); ++i) {
            if (i > 0 && !(xc[i] > xc[i - 1])) c.fail("\"x_c\" must be strictly increasing");
            if (!(y[i] >= 0.0 && y[i] <= 1.0)) c.fail("\"y\" must be within [0, 1]");
            if (i > 0 && y[i] < y[i - 1]) c.fail("\"y\" must be non-decreasing");
            d.gauge_x_k_y.x.push_back(xc[i] + kCelsiusToKelvin);
            d.gauge_x_k_y.y.push_back(y[i]);
        }
    }
    {
        Reader c(r.object("warnings"), "warnings", path);
        const real over_c = c.number("overheat_c", true, 0.0);
        const real cold_c = c.number("cold_c", true, 0.0);
        c.finish();
        if (!(cold_c < over_c)) c.fail("requires cold_c < overheat_c");
        d.warnings.overheat_k = over_c + kCelsiusToKelvin;
        d.warnings.cold_k = cold_c + kCelsiusToKelvin;
    }

    r.finish();
    return d;
}

} // namespace ps::io
