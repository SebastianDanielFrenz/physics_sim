#include "ps/io/electronics_io.h"

#include "engine_parts.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

[[nodiscard]] bool is_assist_name(const std::string& n) {
    for (const char* a : kElectronicsAssistNames) {
        if (n == a) return true;
    }
    return false;
}

void reject_unknown(const json& j, std::initializer_list<const char*> allowed, const std::string& path) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (std::none_of(allowed.begin(), allowed.end(), [&](const char* a) { return it.key() == a; })) {
            throw std::runtime_error(path + ": unknown electronics field '" + it.key() + "'");
        }
    }
}

[[nodiscard]] real require_positive(const json& j, const char* key, const std::string& path) {
    if (!j.contains(key) || !j.at(key).is_number()) throw std::runtime_error(path + ": '" + key + "' is required and must be a number");
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v) || !(v > real(0.0))) throw std::runtime_error(path + ": '" + key + "' must be finite and > 0");
    return v;
}

[[nodiscard]] int traction_mode_from_name(const std::string& n) {
    for (int i = 0; i < vehicle::kTractionModeCount; ++i) {
        if (n == vehicle::kTractionModeNames[i]) return i;
    }
    return -1;
}

[[nodiscard]] real optional_positive(const json& j, const char* key, const std::string& ctx) {
    if (!j.at(key).is_number()) throw std::runtime_error(ctx + ": '" + key + "' must be a number");
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v) || !(v > real(0.0))) throw std::runtime_error(ctx + ": '" + key + "' must be finite and > 0");
    return v;
}

void read_assist_unit(const json& j, const std::string& path, ElectronicsPart& part) {
    reject_unknown(j, {"format", "kind", "name", "source", "extends", "grade", "traction_control", "launch_control", "aero_control"}, path);
    part.kind = ElectronicsKind::AssistUnit;
    if (!j.contains("grade") || !j.at("grade").is_string()) {
        throw std::runtime_error(path + ": 'grade' is required (\"street\", \"sport\" or \"racing\")");
    }
    const std::string grade = j.at("grade").get<std::string>();
    int gi = -1;
    for (int i = 0; i < 3; ++i) {
        if (grade == vehicle::kAssistGrades[i].id) gi = i;
    }
    if (gi < 0) throw std::runtime_error(path + ": unknown grade \"" + grade + "\" (expected \"street\", \"sport\" or \"racing\")");
    part.unit.grade = static_cast<vehicle::AssistGrade>(gi);
    const vehicle::AssistGradeInfo& info = vehicle::assist_grade_info(part.unit.grade);
    for (const char* blk : {"launch_control", "aero_control"}) {
        if (!j.contains(blk)) continue;
        const bool allowed = std::string(blk) == "launch_control" ? info.allows_launch_control : info.allows_aero_control;
        if (!allowed) throw std::runtime_error(path + ": grade '" + grade + "' does not allow '" + blk + "'");
        throw std::runtime_error(path + ": '" + blk + "' in an assist_unit is not implemented yet (assists plan I4/I5)");
    }
    if (!j.contains("traction_control")) return;
    const json& tc = j.at("traction_control");
    const std::string ctx = path + ": traction_control";
    if (!tc.is_object()) throw std::runtime_error(ctx + " must be an object");
    for (auto it = tc.begin(); it != tc.end(); ++it) {
        if (it.key() != "modes" && it.key() != "default_mode" && it.key() != "mode_params") {
            throw std::runtime_error(ctx + ": unknown field '" + it.key() + "'");
        }
    }
    part.unit.has_traction_control = true;
    vehicle::AssistTractionDesc& t = part.unit.traction_control;
    if (!tc.contains("modes") || !tc.at("modes").is_array() || tc.at("modes").empty()) {
        throw std::runtime_error(ctx + ": 'modes' is required and must be a non-empty array of mode names");
    }
    t.modes_bits = 0;
    for (const json& m : tc.at("modes")) {
        const int mi = m.is_string() ? traction_mode_from_name(m.get<std::string>()) : -1;
        if (mi < 0) throw std::runtime_error(ctx + ": 'modes' entries must be one of off, on, relaxed, sport, drift");
        const std::uint8_t bit = vehicle::traction_mode_bit(static_cast<vehicle::TractionMode>(mi));
        if (t.modes_bits & bit) throw std::runtime_error(ctx + ": duplicate mode '" + m.get<std::string>() + "' in 'modes'");
        t.modes_bits = static_cast<std::uint8_t>(t.modes_bits | bit);
    }
    if (t.modes_bits == vehicle::traction_mode_bit(vehicle::TractionMode::Off)) {
        throw std::runtime_error(ctx + ": 'modes' must contain at least one mode other than 'off'");
    }
    if (!tc.contains("default_mode") || !tc.at("default_mode").is_string()) {
        throw std::runtime_error(ctx + ": 'default_mode' is required (a mode name listed in 'modes')");
    }
    const int dm = traction_mode_from_name(tc.at("default_mode").get<std::string>());
    if (dm < 0 || !(t.modes_bits & vehicle::traction_mode_bit(static_cast<vehicle::TractionMode>(dm)))) {
        throw std::runtime_error(ctx + ": 'default_mode' \"" + tc.at("default_mode").get<std::string>() + "\" is not one of the listed 'modes'");
    }
    t.default_mode = static_cast<vehicle::TractionMode>(dm);
    if (!tc.contains("mode_params")) return;
    const json& mp = tc.at("mode_params");
    if (!mp.is_object()) throw std::runtime_error(ctx + ": 'mode_params' must be an object");
    for (auto it = mp.begin(); it != mp.end(); ++it) {
        const int mi = traction_mode_from_name(it.key());
        if (mi < 0) throw std::runtime_error(ctx + ": mode_params: unknown mode '" + it.key() + "'");
        if (mi == 0 || mi == 1) {
            throw std::runtime_error(ctx + ": mode_params: '" + it.key() + "' takes no parameters (mode 'on' is the car's own traction_control block)");
        }
        if (!(t.modes_bits & vehicle::traction_mode_bit(static_cast<vehicle::TractionMode>(mi)))) {
            throw std::runtime_error(ctx + ": mode_params for '" + it.key() + "' but '" + it.key() + "' is not in 'modes'");
        }
        const std::string mctx = ctx + ".mode_params." + it.key();
        const json& o = it.value();
        if (!o.is_object()) throw std::runtime_error(mctx + " must be an object");
        vehicle::TractionModeParams& p = t.params[static_cast<std::size_t>(mi)];
        for (auto f = o.begin(); f != o.end(); ++f) {
            const std::string& k = f.key();
            if (k == "target_slip") p.target_slip = optional_positive(o, "target_slip", mctx);
            else if (k == "kp") p.kp = optional_positive(o, "kp", mctx);
            else if (k == "ki_per_s") p.ki_per_s = optional_positive(o, "ki_per_s", mctx);
            else if (k == "attack_rate_per_s") p.attack_rate_per_s = optional_positive(o, "attack_rate_per_s", mctx);
            else if (k == "recovery_rate_per_s") p.recovery_rate_per_s = optional_positive(o, "recovery_rate_per_s", mctx);
            else if (k == "min_fraction") {
                if (!f.value().is_number()) throw std::runtime_error(mctx + ": 'min_fraction' must be a number");
                const real v = f.value().get<real>();
                if (!std::isfinite(v) || v < real(0.0) || v >= real(1.0)) throw std::runtime_error(mctx + ": 'min_fraction' must be in [0, 1)");
                p.min_fraction = v;
            } else if (k == "ignition_cut") {
                if (!f.value().is_boolean()) throw std::runtime_error(mctx + ": 'ignition_cut' must be a boolean");
                p.ignition_cut = f.value().get<bool>();
            } else {
                throw std::runtime_error(mctx + ": unknown field '" + k + "'");
            }
        }
        if (p.target_slip && *p.target_slip >= real(1.0)) throw std::runtime_error(mctx + ": 'target_slip' must be below 1");
    }
}

} // namespace

ElectronicsPart load_electronics_json(const std::string& path) {
    const json j = parts::load_part(path);
    if (!j.is_object()) throw std::runtime_error(path + ": an electronics part must be a JSON object");
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"physics_sim.electronics/1\")");
    }
    if (j.at("format").get<std::string>() != "physics_sim.electronics/1") {
        throw std::runtime_error(path + ": unsupported 'format' \"" + j.at("format").get<std::string>() +
                                 "\" (expected \"physics_sim.electronics/1\")");
    }
    if (!j.contains("kind") || !j.at("kind").is_string()) {
        throw std::runtime_error(path + ": 'kind' is required (\"launch_control\", \"assist_fitment\" or \"assist_unit\")");
    }
    const std::string kind = j.at("kind").get<std::string>();

    ElectronicsPart part;
    if (j.contains("name") && j.at("name").is_string()) part.name = j.at("name").get<std::string>();
    if (j.contains("source") && j.at("source").is_string()) part.source = j.at("source").get<std::string>();

    if (kind == "launch_control") {
        reject_unknown(j, {"format", "kind", "name", "source", "extends", "rpm", "max_speed_mps", "channel", "fitted"}, path);
        part.kind = ElectronicsKind::LaunchControl;
        part.launch.rpm = require_positive(j, "rpm", path);
        part.launch.max_speed_mps = require_positive(j, "max_speed_mps", path);
        if (j.contains("channel")) {
            if (!j.at("channel").is_string() || j.at("channel").get<std::string>().empty()) {
                throw std::runtime_error(path + ": 'channel' must be a non-empty string");
            }
            part.launch.channel = j.at("channel").get<std::string>();
        }
        if (j.contains("fitted")) {
            if (!j.at("fitted").is_boolean()) throw std::runtime_error(path + ": 'fitted' must be a boolean");
            part.launch.fitted = j.at("fitted").get<bool>();
        }
    } else if (kind == "assist_fitment") {
        reject_unknown(j, {"format", "kind", "name", "source", "extends", "fits"}, path);
        part.kind = ElectronicsKind::AssistFitment;
        if (!j.contains("fits") || !j.at("fits").is_array() || j.at("fits").empty()) {
            throw std::runtime_error(path + ": 'fits' is required and must be a non-empty array of assist names");
        }
        for (const json& f : j.at("fits")) {
            if (!f.is_string() || !is_assist_name(f.get<std::string>())) {
                throw std::runtime_error(path + ": 'fits' entries must be one of auto_clutch, auto_blip, auto_shift, traction_control, anti_lag");
            }
            const std::string n = f.get<std::string>();
            if (std::find(part.fits.begin(), part.fits.end(), n) != part.fits.end()) {
                throw std::runtime_error(path + ": duplicate assist '" + n + "' in 'fits'");
            }
            part.fits.push_back(n);
        }
    } else if (kind == "assist_unit") {
        read_assist_unit(j, path, part);
    } else {
        throw std::runtime_error(path + ": unknown electronics kind \"" + kind + "\" (expected \"launch_control\", \"assist_fitment\" or \"assist_unit\")");
    }
    return part;
}

} // namespace ps::io
