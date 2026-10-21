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
// Q3 / I8: "park_only" | "handbrake".
[[nodiscard]] vehicle::EpbMode epb_mode_from(const json& v, const std::string& ctx) {
    if (v.is_string()) {
        for (int i = 0; i < 2; ++i) {
            if (v.get<std::string>() == vehicle::kEpbModeNames[i]) return static_cast<vehicle::EpbMode>(i);
        }
    }
    throw std::runtime_error(ctx + ": 'epb_mode' must be \"park_only\" or \"handbrake\"");
}

// Assists I5: the aero_control block. Ranges copy racing_game's loader (environment.cpp: cruise_wing_offset_deg
// [-8, 55], airbrake_wing_offset_deg [0, 55], airbrake_min_speed_m_s [0, 150], airbrake_threshold [0, 0.99]);
// the ramp, brake_full and launch_offset_deg ranges are this port's own [AS].
void read_aero_control(const json& a, const std::string& path, vehicle::AssistUnitDesc& unit) {
    const std::string ctx = path + ": aero_control";
    if (!a.is_object()) throw std::runtime_error(ctx + " must be an object");
    vehicle::AeroControlDesc c;
    const auto number = [&](const char* key, real lo, real hi) {
        const json& v = a.at(key);
        if (!v.is_number()) throw std::runtime_error(ctx + ": '" + key + "' must be a number");
        const real x = v.get<real>();
        if (!std::isfinite(x) || x < lo || x > hi) {
            throw std::runtime_error(ctx + ": '" + key + "' must be in [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
        }
        return x;
    };
    for (auto it = a.begin(); it != a.end(); ++it) {
        const std::string& k = it.key();
        if (k == "surface") {
            if (!it.value().is_string() || it.value().get<std::string>().empty()) throw std::runtime_error(ctx + ": 'surface' must be a non-empty string");
            c.surface = it.value().get<std::string>();
        } else if (k == "default_enabled") {
            if (!it.value().is_boolean() && !(it.value().is_number() && (it.value().get<real>() == real(0) || it.value().get<real>() == real(1)))) {
                throw std::runtime_error(ctx + ": 'default_enabled' must be a boolean or 0/1");
            }
            c.default_enabled = it.value().is_boolean() ? it.value().get<bool>() : it.value().get<real>() == real(1);
        } else if (k == "cruise_offset_deg") c.cruise_offset_deg = number("cruise_offset_deg", -8, 55);
        else if (k == "cruise_ramp_start_m_s") c.cruise_ramp_start_m_s = number("cruise_ramp_start_m_s", 0, 150);
        else if (k == "cruise_ramp_end_m_s") c.cruise_ramp_end_m_s = number("cruise_ramp_end_m_s", 0, 150);
        else if (k == "airbrake_offset_deg") c.airbrake_offset_deg = number("airbrake_offset_deg", 0, 55);
        else if (k == "airbrake_min_speed_m_s") c.airbrake_min_speed_m_s = number("airbrake_min_speed_m_s", 0, 150);
        else if (k == "airbrake_brake_start") c.airbrake_brake_start = number("airbrake_brake_start", 0, 0.99);
        else if (k == "airbrake_brake_full") c.airbrake_brake_full = number("airbrake_brake_full", 0.01, 1);
        else if (k == "launch_offset_deg") c.launch_offset_deg = number("launch_offset_deg", -8, 55);
        else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
    }
    if (!(c.cruise_ramp_end_m_s > c.cruise_ramp_start_m_s)) {
        throw std::runtime_error(ctx + ": 'cruise_ramp_end_m_s' must be greater than 'cruise_ramp_start_m_s'");
    }
    if (!(c.airbrake_brake_full > c.airbrake_brake_start)) {
        throw std::runtime_error(ctx + ": 'airbrake_brake_full' must be greater than 'airbrake_brake_start'");
    }
    unit.aero_control = c;
}

// Assists I4: the launch_control block (D-LC1). Ranges are validate_launch_assist's; the loader adds key-specific
// type checks and the "exactly one of launch_rpm / launch_rpm_fraction" rule.
void read_launch_control(const json& a, const std::string& path, vehicle::AssistUnitDesc& unit) {
    const std::string ctx = path + ": launch_control";
    if (!a.is_object()) throw std::runtime_error(ctx + " must be an object");
    vehicle::LaunchAssistDesc d;
    const auto number = [&](const json& v, const std::string& key) {
        if (!v.is_number()) throw std::runtime_error(ctx + ": '" + key + "' must be a number");
        const real x = v.get<real>();
        if (!std::isfinite(x)) throw std::runtime_error(ctx + ": '" + key + "' must be finite");
        return x;
    };
    for (auto it = a.begin(); it != a.end(); ++it) {
        const std::string& k = it.key();
        const json& v = it.value();
        if (k == "launch_rpm") d.launch_rpm = number(v, k);
        else if (k == "launch_rpm_fraction") d.launch_rpm_fraction = number(v, k);
        else if (k == "default_enabled") {
            if (!v.is_boolean() && !(v.is_number() && (v.get<real>() == real(0) || v.get<real>() == real(1)))) {
                throw std::runtime_error(ctx + ": 'default_enabled' must be a boolean or 0/1");
            }
            d.default_enabled = v.is_boolean() ? v.get<bool>() : v.get<real>() == real(1);
        } else if (k == "arm_throttle") d.arm_throttle = number(v, k);
        else if (k == "arm_brake") d.arm_brake = number(v, k);
        else if (k == "release_brake") d.release_brake = number(v, k);
        else if (k == "arm_max_speed_mps") d.arm_max_speed_mps = number(v, k);
        else if (k == "handover_speed_mps") d.handover_speed_mps = number(v, k);
        else if (k == "max_launch_s") d.max_launch_s = number(v, k);
        else if (k == "clutch_ramp_s") d.clutch_ramp_s = number(v, k);
        else if (k == "clutch_up_per_s") d.clutch_up_per_s = number(v, k);
        else if (k == "clutch_down_per_s") d.clutch_down_per_s = number(v, k);
        else if (k == "slip_floor_mps") d.slip_floor_mps = number(v, k);
        else if (k == "kp") d.kp = number(v, k);
        else if (k == "ki_per_s") d.ki_per_s = number(v, k);
        else if (k == "attack_rate_per_s") d.attack_rate_per_s = number(v, k);
        else if (k == "recovery_rate_per_s") d.recovery_rate_per_s = number(v, k);
        else if (k == "min_fraction") d.min_fraction = number(v, k);
        else if (k == "max_limiter_fraction") d.max_limiter_fraction = number(v, k); // G1
        else if (k == "slip_target") {
            if (!v.is_object()) throw std::runtime_error(ctx + ": 'slip_target' must be an object keyed by traction mode name");
            for (auto m = v.begin(); m != v.end(); ++m) {
                const int mi = traction_mode_from_name(m.key());
                if (mi < 0) throw std::runtime_error(ctx + ": slip_target: unknown mode '" + m.key() + "'");
                d.slip_target[static_cast<std::size_t>(mi)] = number(m.value(), "slip_target." + m.key());
            }
            if (!v.contains("off") && v.contains("on")) d.slip_target[0] = d.slip_target[1];
        } else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
    }
    if (!((d.launch_rpm > real(0)) != (d.launch_rpm_fraction > real(0)))) {
        throw std::runtime_error(ctx + ": exactly one of 'launch_rpm' (> 0) and 'launch_rpm_fraction' (in (0, 1)) is required");
    }
    try {
        vehicle::validate_launch_assist(d);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
    unit.launch_control = d;
}

[[nodiscard]] int esc_mode_from_name(const std::string& n) {
    for (int i = 0; i < vehicle::kEscModeCount; ++i) {
        if (n == vehicle::kEscModeNames[i]) return i;
    }
    return -1;
}

// Assists I7: the drive_modes part (kind "drive_modes"). Keys per mode: pedal_bend {x, y}, auto_shift {the four rpm
// points, each optional}, auto_clutch {close_time_s}, traction_control_default (a traction mode name),
// launch_control_default (bool, I7b), launch_control_allowed, nitrous_arm_allowed. A mode is offered iff it has an entry in "modes" (an empty object is
// a valid "no overrides" mode). Ranges are validate_drive_modes'.
void read_drive_modes(const json& j, const std::string& path, ElectronicsPart& part) {
    reject_unknown(j, {"format", "kind", "name", "source", "extends", "default_mode", "modes"}, path);
    part.kind = ElectronicsKind::DriveModes;
    vehicle::DriveModesDesc& d = part.drive_modes;
    const auto mode_index = [](const std::string& n) {
        for (int i = 0; i < vehicle::kDriveModeCount; ++i)
            if (n == vehicle::kDriveModeNames[i]) return i;
        return -1;
    };
    if (!j.contains("modes") || !j.at("modes").is_object() || j.at("modes").empty()) {
        throw std::runtime_error(path + ": 'modes' is required and must be a non-empty object keyed by mode name (eco, normal, sport, racing, electric)");
    }
    const auto number = [&](const json& v, const std::string& ctx, const std::string& key) {
        if (!v.is_number()) throw std::runtime_error(ctx + ": '" + key + "' must be a number");
        const real x = v.get<real>();
        if (!std::isfinite(x)) throw std::runtime_error(ctx + ": '" + key + "' must be finite");
        return x;
    };
    const auto boolean = [&](const json& v, const std::string& ctx, const std::string& key) {
        if (!v.is_boolean()) throw std::runtime_error(ctx + ": '" + key + "' must be a boolean");
        return v.get<bool>();
    };
    for (auto it = j.at("modes").begin(); it != j.at("modes").end(); ++it) {
        const int mi = mode_index(it.key());
        if (mi < 0) throw std::runtime_error(path + ": modes: unknown mode '" + it.key() + "' (expected eco, normal, sport, racing or electric)");
        const std::string ctx = path + ": modes." + it.key();
        if (!it.value().is_object()) throw std::runtime_error(ctx + " must be an object");
        d.modes_bits = static_cast<std::uint8_t>(d.modes_bits | vehicle::drive_mode_bit(static_cast<vehicle::DriveMode>(mi)));
        vehicle::DriveModeParams& p = d.params[static_cast<std::size_t>(mi)];
        for (auto f = it.value().begin(); f != it.value().end(); ++f) {
            const std::string& k = f.key();
            const json& v = f.value();
            if (k == "pedal_bend") {
                if (!v.is_object()) throw std::runtime_error(ctx + ": 'pedal_bend' must be an object {x, y}");
                vehicle::PedalBend b;
                bool hx = false, hy = false;
                for (auto bf = v.begin(); bf != v.end(); ++bf) {
                    if (bf.key() == "x") { b.x = number(bf.value(), ctx + ".pedal_bend", "x"); hx = true; }
                    else if (bf.key() == "y") { b.y = number(bf.value(), ctx + ".pedal_bend", "y"); hy = true; }
                    else throw std::runtime_error(ctx + ".pedal_bend: unknown field '" + bf.key() + "'");
                }
                if (!hx || !hy) throw std::runtime_error(ctx + ": 'pedal_bend' needs both 'x' and 'y'");
                p.pedal_bend = b;
            } else if (k == "auto_shift") {
                if (!v.is_object()) throw std::runtime_error(ctx + ": 'auto_shift' must be an object");
                for (auto sf = v.begin(); sf != v.end(); ++sf) {
                    const std::string& sk = sf.key();
                    const real x = number(sf.value(), ctx + ".auto_shift", sk);
                    if (sk == "up_rpm_at_zero_throttle") p.up_rpm_at_zero_throttle = x;
                    else if (sk == "up_rpm_at_full_throttle") p.up_rpm_at_full_throttle = x;
                    else if (sk == "down_rpm_at_zero_throttle") p.down_rpm_at_zero_throttle = x;
                    else if (sk == "down_rpm_at_full_throttle") p.down_rpm_at_full_throttle = x;
                    else throw std::runtime_error(ctx + ".auto_shift: unknown field '" + sk + "'");
                }
            } else if (k == "auto_clutch") {
                if (!v.is_object()) throw std::runtime_error(ctx + ": 'auto_clutch' must be an object");
                for (auto cf = v.begin(); cf != v.end(); ++cf) {
                    if (cf.key() == "close_time_s") p.clutch_close_time_s = number(cf.value(), ctx + ".auto_clutch", "close_time_s");
                    else throw std::runtime_error(ctx + ".auto_clutch: unknown field '" + cf.key() + "'");
                }
            } else if (k == "traction_control_default") {
                const int tm = v.is_string() ? traction_mode_from_name(v.get<std::string>()) : -1;
                if (tm < 0) throw std::runtime_error(ctx + ": 'traction_control_default' must be one of off, on, relaxed, sport, drift");
                p.traction_control_default = static_cast<vehicle::TractionMode>(tm);
            } else if (k == "launch_control_default") p.launch_control_default = boolean(v, ctx, k);
            else if (k == "launch_control_allowed") p.launch_control_allowed = boolean(v, ctx, k);
            else if (k == "nitrous_arm_allowed") p.nitrous_arm_allowed = boolean(v, ctx, k);
            else if (k == "epb_mode") p.epb_mode = epb_mode_from(v, ctx);
            else if (k == "abs_default") p.abs_default = boolean(v, ctx, k);
            else if (k == "esc_default") {
                const int em = v.is_string() ? esc_mode_from_name(v.get<std::string>()) : -1;
                if (em < 0) throw std::runtime_error(ctx + ": 'esc_default' must be one of off, on, sport");
                p.esc_default = static_cast<vehicle::EscMode>(em);
            }
            else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
        }
    }
    if (!j.contains("default_mode") || !j.at("default_mode").is_string()) {
        throw std::runtime_error(path + ": 'default_mode' is required (a mode name listed in 'modes')");
    }
    const int dm = mode_index(j.at("default_mode").get<std::string>());
    if (dm < 0 || !(d.modes_bits & vehicle::drive_mode_bit(static_cast<vehicle::DriveMode>(dm)))) {
        throw std::runtime_error(path + ": 'default_mode' \"" + j.at("default_mode").get<std::string>() + "\" is not one of the listed 'modes'");
    }
    d.default_mode = static_cast<vehicle::DriveMode>(dm);
    try {
        vehicle::validate_drive_modes(d);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
}

// ABS / ESC (assists I5 part 2 / I6): the abs and esc blocks. Every key is optional (the AbsDesc / EscDesc defaults);
// ranges are validate_abs / validate_esc's.
void read_abs(const json& a, const std::string& path, vehicle::AssistUnitDesc& unit) {
    const std::string ctx = path + ": abs";
    if (!a.is_object()) throw std::runtime_error(ctx + " must be an object");
    vehicle::AbsDesc d;
    for (auto it = a.begin(); it != a.end(); ++it) {
        const std::string& k = it.key();
        if (k == "default_enabled") {
            if (!it.value().is_boolean()) throw std::runtime_error(ctx + ": 'default_enabled' must be a boolean");
            d.default_enabled = it.value().get<bool>();
        } else if (k == "reference_speed") {
            if (!it.value().is_string()) throw std::runtime_error(ctx + ": 'reference_speed' must be \"ground_truth\" or \"estimated\"");
            const std::string v = it.value().get<std::string>();
            if (v == "ground_truth") d.reference_speed = vehicle::AbsReferenceSpeed::GroundTruth;
            else if (v == "estimated") d.reference_speed = vehicle::AbsReferenceSpeed::Estimated;
            else throw std::runtime_error(ctx + ": 'reference_speed' must be \"ground_truth\" or \"estimated\", got \"" + v + "\"");
        } else if (k == "rear_select_low") {
            if (!it.value().is_boolean()) throw std::runtime_error(ctx + ": 'rear_select_low' must be a boolean");
            d.rear_select_low = it.value().get<bool>();
        } else if (k == "target_slip") d.target_slip = optional_positive(a, "target_slip", ctx);
        else if (k == "min_speed_mps") d.min_speed_mps = optional_positive(a, "min_speed_mps", ctx);
        else if (k == "slip_floor_mps") d.slip_floor_mps = optional_positive(a, "slip_floor_mps", ctx);
        else if (k == "kp") d.kp = optional_positive(a, "kp", ctx);
        else if (k == "ki_per_s") d.ki_per_s = optional_positive(a, "ki_per_s", ctx);
        else if (k == "release_rate_per_s") d.release_rate_per_s = optional_positive(a, "release_rate_per_s", ctx);
        else if (k == "apply_rate_per_s") d.apply_rate_per_s = optional_positive(a, "apply_rate_per_s", ctx);
        else if (k == "reanchor_rate_per_s") d.reanchor_rate_per_s = optional_positive(a, "reanchor_rate_per_s", ctx);
        else if (k == "reanchor_max_decel_mps2") d.reanchor_max_decel_mps2 = optional_positive(a, "reanchor_max_decel_mps2", ctx);
        else if (k == "front_split_rate_per_s") d.front_split_rate_per_s = optional_positive(a, "front_split_rate_per_s", ctx);
        else if (k == "reanchor_scale") {
            if (!it.value().is_number()) throw std::runtime_error(ctx + ": 'reanchor_scale' must be a number");
            d.reanchor_scale = it.value().get<real>();
        } else if (k == "front_split_max") {
            if (!it.value().is_number()) throw std::runtime_error(ctx + ": 'front_split_max' must be a number");
            d.front_split_max = it.value().get<real>();
        } else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
    }
    try {
        vehicle::validate_abs(d);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
    unit.abs = d;
}

void read_esc_mode_params(const json& o, const std::string& ctx, vehicle::EscModeParams& p) {
    if (!o.is_object()) throw std::runtime_error(ctx + " must be an object");
    for (auto it = o.begin(); it != o.end(); ++it) {
        const std::string& k = it.key();
        if (!it.value().is_number()) throw std::runtime_error(ctx + ": '" + k + "' must be a number");
        const real v = it.value().get<real>();
        if (k == "deadband_rad_s") p.deadband_rad_s = v;
        else if (k == "deadband_fraction") p.deadband_fraction = v;
        else if (k == "brake_gain_per_rad_s") p.brake_gain_per_rad_s = v;
        else if (k == "max_brake") p.max_brake = v;
        else if (k == "torque_gain_per_rad_s") p.torque_gain_per_rad_s = v;
        else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
    }
}

// esc: modes (array of off / on / sport, "on" required), default_mode, the vehicle-level keys, the mode parameters of
// mode on at the top level and "mode_params": {"sport": {overrides}} (sport starts from on's values).
void read_esc(const json& a, const std::string& path, vehicle::AssistUnitDesc& unit) {
    const std::string ctx = path + ": esc";
    if (!a.is_object()) throw std::runtime_error(ctx + " must be an object");
    vehicle::EscDesc d;
    vehicle::EscModeParams base;
    json base_params = json::object();
    for (auto it = a.begin(); it != a.end(); ++it) {
        const std::string& k = it.key();
        if (k == "modes") {
            if (!it.value().is_array() || it.value().empty()) throw std::runtime_error(ctx + ": 'modes' must be a non-empty array of mode names");
            d.modes_bits = 0;
            for (const json& m : it.value()) {
                const int mi = m.is_string() ? esc_mode_from_name(m.get<std::string>()) : -1;
                if (mi < 0) throw std::runtime_error(ctx + ": 'modes' entries must be one of off, on, sport");
                const std::uint8_t bit = vehicle::esc_mode_bit(static_cast<vehicle::EscMode>(mi));
                if (d.modes_bits & bit) throw std::runtime_error(ctx + ": duplicate mode '" + m.get<std::string>() + "' in 'modes'");
                d.modes_bits = static_cast<std::uint8_t>(d.modes_bits | bit);
            }
        } else if (k == "default_mode" || k == "mode_params") {
            // read below, after 'modes'
        } else if (k == "characteristic_speed_mps") d.characteristic_speed_mps = optional_positive(a, "characteristic_speed_mps", ctx);
        else if (k == "max_lateral_accel_mps2") d.max_lateral_accel_mps2 = optional_positive(a, "max_lateral_accel_mps2", ctx);
        else if (k == "min_speed_mps") d.min_speed_mps = optional_positive(a, "min_speed_mps", ctx);
        else if (k == "brake_rate_per_s") d.brake_rate_per_s = optional_positive(a, "brake_rate_per_s", ctx);
        else if (k == "deadband_rad_s" || k == "deadband_fraction" || k == "brake_gain_per_rad_s" || k == "max_brake" || k == "torque_gain_per_rad_s") {
            base_params[k] = it.value();
        } else throw std::runtime_error(ctx + ": unknown field '" + k + "'");
    }
    if (!(d.modes_bits & vehicle::esc_mode_bit(vehicle::EscMode::On))) throw std::runtime_error(ctx + ": 'modes' must contain 'on'");
    read_esc_mode_params(base_params, ctx, base);
    for (auto& p : d.params) p = base;
    if (a.contains("default_mode")) {
        const json& v = a.at("default_mode");
        const int mi = v.is_string() ? esc_mode_from_name(v.get<std::string>()) : -1;
        if (mi < 0 || !(d.modes_bits & vehicle::esc_mode_bit(static_cast<vehicle::EscMode>(mi)))) {
            throw std::runtime_error(ctx + ": 'default_mode' must be one of the listed 'modes'");
        }
        d.default_mode = static_cast<vehicle::EscMode>(mi);
    }
    if (a.contains("mode_params")) {
        const json& mp = a.at("mode_params");
        if (!mp.is_object()) throw std::runtime_error(ctx + ": 'mode_params' must be an object");
        for (auto it = mp.begin(); it != mp.end(); ++it) {
            const int mi = esc_mode_from_name(it.key());
            if (mi != static_cast<int>(vehicle::EscMode::Sport)) {
                throw std::runtime_error(ctx + ": mode_params: only 'sport' takes parameters (mode 'on' is the block's own keys), got '" + it.key() + "'");
            }
            if (!(d.modes_bits & vehicle::esc_mode_bit(vehicle::EscMode::Sport))) {
                throw std::runtime_error(ctx + ": mode_params for 'sport' but 'sport' is not in 'modes'");
            }
            read_esc_mode_params(it.value(), ctx + ".mode_params.sport", d.params[static_cast<std::size_t>(vehicle::EscMode::Sport)]);
        }
    }
    try {
        vehicle::validate_esc(d);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
    unit.esc = d;
}

void read_assist_unit(const json& j, const std::string& path, ElectronicsPart& part) {
    reject_unknown(j, {"format", "kind", "name", "source", "extends", "grade", "traction_control", "launch_control", "aero_control", "abs", "esc"}, path);
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
    }
    if (j.contains("launch_control")) read_launch_control(j.at("launch_control"), path, part.unit);
    if (j.contains("aero_control")) read_aero_control(j.at("aero_control"), path, part.unit);
    if (j.contains("esc") && !j.contains("abs")) throw std::runtime_error(path + ": 'esc' needs an 'abs' block (ESC brakes single wheels through the ABS modulation)");
    if (j.contains("abs")) read_abs(j.at("abs"), path, part.unit);
    if (j.contains("esc")) read_esc(j.at("esc"), path, part.unit);
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

// Q3 / I8: the electronic parking brake part (kind "epb"): max_engage_speed_mps (required, finite > 0), epb_mode
// (optional, default park_only). Drives the car's handbrake rows; the vehicle loader requires them.
void read_epb(const json& j, const std::string& path, ElectronicsPart& part) {
    reject_unknown(j, {"format", "kind", "name", "source", "extends", "max_engage_speed_mps", "epb_mode"}, path);
    part.kind = ElectronicsKind::Epb;
    part.epb.max_engage_speed_mps = require_positive(j, "max_engage_speed_mps", path);
    if (j.contains("epb_mode")) part.epb.epb_mode = epb_mode_from(j.at("epb_mode"), path);
    try {
        vehicle::validate_epb(part.epb);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(path + ": " + e.what());
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
        throw std::runtime_error(path + ": 'kind' is required (\"launch_control\", \"assist_fitment\", \"assist_unit\", \"drive_modes\" or \"epb\")");
    }
    const std::string kind = j.at("kind").get<std::string>();

    ElectronicsPart part;
    if (j.contains("name") && j.at("name").is_string()) part.name = j.at("name").get<std::string>();
    if (j.contains("source") && j.at("source").is_string()) part.source = j.at("source").get<std::string>();

    if (kind == "launch_control") {
        reject_unknown(j, {"format", "kind", "name", "source", "extends", "rpm", "max_speed_mps", "channel", "fitted", "max_limiter_fraction"}, path);
        part.kind = ElectronicsKind::LaunchControl;
        part.launch.rpm = require_positive(j, "rpm", path);
        part.launch.max_speed_mps = require_positive(j, "max_speed_mps", path);
        if (j.contains("max_limiter_fraction")) { // G1: the clamp of rpm against a low engine limiter
            const json& f = j.at("max_limiter_fraction");
            if (!f.is_number() || !std::isfinite(f.get<real>()) || !(f.get<real>() > real(0.0)) || !(f.get<real>() < real(1.0))) {
                throw std::runtime_error(path + ": 'max_limiter_fraction' must be a number in (0, 1)");
            }
            part.launch.max_limiter_fraction = f.get<real>();
        }
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
    } else if (kind == "drive_modes") {
        read_drive_modes(j, path, part);
    } else if (kind == "epb") {
        read_epb(j, path, part);
    } else {
        throw std::runtime_error(path + ": unknown electronics kind \"" + kind + "\" (expected \"launch_control\", \"assist_fitment\", \"assist_unit\", \"drive_modes\" or \"epb\")");
    }
    return part;
}

} // namespace ps::io
