// ps/vehicle/assist_unit_desc.h - the assist unit (assists plan I1, vault PHYS-074, docs/data_formats/electronics.md):
// a car's driver-assist electronics part, graded Street / Sport / Racing. I1 carries the traction-control mode table
// only plus (assists I5) the aero automation block; launch control (assists I4) is declared in the grade table but
// not yet loaded.
#pragma once

#include "ps/aero/aerodynamics.h"
#include "ps/types.h"
#include "ps/vehicle/traction_control.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ps::vehicle {

// The grade decides which BLOCKS a unit may carry (owner 2026-10-12: "Street / Sport / Racing Assists"). The modes a
// unit offers are its own data (the `modes` list), so the owner's grade-to-mode mapping is a data edit.
enum class AssistGrade : std::uint8_t { Street = 0, Sport = 1, Racing = 2 };

struct AssistGradeInfo {
    std::string_view id;
    bool allows_launch_control; // [AS] proposal: Sport and Racing
    bool allows_aero_control;   // [AS] proposal: Racing
};
inline constexpr AssistGradeInfo kAssistGrades[] = {
    {"street", false, false},
    {"sport", true, false},
    {"racing", true, true},
};
[[nodiscard]] inline const AssistGradeInfo& assist_grade_info(AssistGrade g) { return kAssistGrades[static_cast<int>(g)]; }

// Traction-control mode CODES of the single channel `assist.traction_control`. Append-only; the numbers are NOT
// ordered by intervention (see traction_mode_rank).
enum class TractionMode : std::uint8_t { Off = 0, On = 1, Relaxed = 2, Sport = 3, Drift = 4 };
inline constexpr int kTractionModeCount = 5;
inline constexpr const char* kTractionModeNames[kTractionModeCount] = {"off", "on", "relaxed", "sport", "drift"};

[[nodiscard]] constexpr std::uint8_t traction_mode_bit(TractionMode m) { return static_cast<std::uint8_t>(1u << static_cast<unsigned>(m)); }
// Intervention rank: On 4 > Relaxed 3 > Sport 2 > Drift 1 > Off 0.
[[nodiscard]] constexpr int traction_mode_rank(TractionMode m) {
    constexpr int r[kTractionModeCount] = {0, 4, 3, 2, 1};
    return r[static_cast<int>(m)];
}
inline constexpr TractionMode kTractionModesByRank[5] = {TractionMode::Off, TractionMode::Drift, TractionMode::Sport,
                                                         TractionMode::Relaxed, TractionMode::On};
inline constexpr std::uint8_t kTractionModesBaseBits = 0b00011; // a car without a unit offers Off and On

// Overrides of the car's own TractionControlDesc for one mode. Unset = the car's value (min_fraction 0, ignition_cut true).
struct TractionModeParams {
    std::optional<real> target_slip, kp, ki_per_s, attack_rate_per_s, recovery_rate_per_s, min_fraction;
    std::optional<bool> ignition_cut;
};

struct AssistTractionDesc {
    std::uint8_t modes_bits = kTractionModesBaseBits; // bit per TractionMode code
    TractionMode default_mode = TractionMode::On;     // the channel's default value
    // Index by mode code. Index 1 (On) is never stored: mode On is the car's own traction_control block.
    std::array<TractionModeParams, kTractionModeCount> params{};
};

// Assists I5 (D-AE1, vault PHYS-074): rear-wing automation, a port of racing_game's law (core/src/session.cpp, the
// "automatic_rear_wing" block). Every default below is racing_game's value [D] (rg/environment.h, environment.cpp);
// launch_offset_deg is an [AS] (plan B4: the car_hyper maximum-downforce offset, 9-11 deg [D]).
struct AeroControlDesc {
    std::string surface = "rear_wing"; // the aero surface this controller owns (looked up by name in the car's aero block)
    bool default_enabled = false;      // the channel assist.aero_control's default (0/1)
    real cruise_offset_deg = 4;        // [D] racing_game cruise_wing_offset_deg
    real cruise_ramp_start_m_s = 10;   // [D] racing_game session.cpp literal: cruise ramp (v - 10) / 20
    real cruise_ramp_end_m_s = 30;     // [D] ... 10 + 20
    real airbrake_offset_deg = 45;     // [D] racing_game airbrake_wing_offset_deg
    real airbrake_min_speed_m_s = 20;  // [D] racing_game airbrake_min_speed_m_s
    real airbrake_brake_start = 0.45;  // [D] racing_game airbrake_threshold
    real airbrake_brake_full = 1.0;    // [D] racing_game's literal 1
    real launch_offset_deg = 10;       // [AS] the offset a launch control (assists I4) asks for
};

struct AssistUnitDesc {
    AssistGrade grade = AssistGrade::Street;
    bool has_traction_control = false; // false: TC stays as the car's base desc defines it (modes Off/On)
    AssistTractionDesc traction_control;
    std::optional<AeroControlDesc> aero_control; // present: the unit carries rear-wing automation (grade racing)
};

// Fitment (assists I5, decision recorded in docs/data_formats/electronics.md): the controller is fitted iff the unit
// carries the block AND the car's aero block has a surface of that name. A unit naming a surface the car lacks is NOT
// a load error - a generic Racing unit stays attachable to a wingless car - it is simply not fitted (bit 6 of
// assist_fitted_bits 0, no channel). Returns the surface index.
[[nodiscard]] inline std::optional<std::size_t> find_aero_control_surface(const aero::Desc& aero, const std::optional<AssistUnitDesc>& unit) {
    if (!unit || !unit->aero_control) return std::nullopt;
    for (std::size_t i = 0; i < aero.surfaces.size(); ++i) {
        if (aero.surfaces[i].name == unit->aero_control->surface) return i;
    }
    return std::nullopt;
}

// The law. forward_speed = chassis-frame x of (chassis velocity - wind) [m/s]; brake in [0, 1] (clamped here).
// `launch` is the hook a launch control (assists I4) sets: it makes target = launch_offset_deg and yields over the
// airbrake and cruise terms (order LC > airbrake > cruise). Returns the surface command in [-1, 1] and the target
// offset in degrees. Stateless, only + - * / and comparisons (no libm).
// Implemented in core/src/vehicle/assist_unit.cpp.
[[nodiscard]] real aero_control_command(const AeroControlDesc& c, real max_offset_rad, real min_offset_rad, real forward_speed, real brake, bool launch,
                                        real* target_deg_out = nullptr);

// Channel value -> requested mode code. Non-finite or <= 0.5 is Off (the pre-assist-unit rule `value > 0.5`), otherwise
// clamp(round(value), 1, 4).
[[nodiscard]] inline TractionMode traction_mode_requested(real channel_value) {
    if (!std::isfinite(channel_value) || !(channel_value > real(0.5))) return TractionMode::Off;
    const real r = std::round(channel_value);
    return static_cast<TractionMode>(r >= real(4) ? 4 : (r < real(1) ? 1 : static_cast<int>(r)));
}

// The mode actually run. A requested mode the unit does not offer maps, statelessly, to the nearest available mode
// with MORE intervention (On > Relaxed > Sport > Drift); if none is more intervening, to the most intervening
// available mode. Off is never a substitute. If Off itself is not offered, Off maps to the LEAST intervening
// available mode. No available mode at all (assist not fitted) is Off.
[[nodiscard]] constexpr TractionMode resolve_traction_mode(std::uint8_t available_bits, TractionMode requested) {
    if (available_bits & traction_mode_bit(requested)) return requested;
    if (requested == TractionMode::Off) {
        for (int r = 1; r <= 4; ++r)
            if (available_bits & traction_mode_bit(kTractionModesByRank[r])) return kTractionModesByRank[r];
        return TractionMode::Off;
    }
    for (int r = traction_mode_rank(requested) + 1; r <= 4; ++r)
        if (available_bits & traction_mode_bit(kTractionModesByRank[r])) return kTractionModesByRank[r];
    for (int r = 4; r >= 1; --r)
        if (available_bits & traction_mode_bit(kTractionModesByRank[r])) return kTractionModesByRank[r];
    return TractionMode::Off;
}

// Modes a car offers: none when TC is not fitted, the unit's list when it has a TC block, else Off and On.
[[nodiscard]] inline std::uint8_t traction_modes_available_bits(const std::optional<AssistUnitDesc>& unit, bool fitted) {
    if (!fitted) return 0;
    if (unit && unit->has_traction_control) return unit->traction_control.modes_bits;
    return kTractionModesBaseBits;
}

// The channel's default value: 0 when not fitted, the unit's default mode with a TC block, else the base `enabled`.
[[nodiscard]] inline real traction_default_channel_value(const TractionControlDesc& base, const std::optional<AssistUnitDesc>& unit) {
    if (!base.fitted) return real(0);
    if (unit && unit->has_traction_control) return static_cast<real>(static_cast<int>(unit->traction_control.default_mode));
    return base.enabled ? real(1) : real(0);
}

// One TractionControlDesc per mode code: the car's own desc with the mode's overrides applied. Index 0 (Off) and 1 (On)
// are the car's desc unchanged (mode On is bit-identical to a car without a unit). Every OFFERED mode is validated;
// throws std::invalid_argument naming the mode.
[[nodiscard]] inline std::array<TractionControlDesc, kTractionModeCount> resolve_traction_mode_descs(
    const TractionControlDesc& base, const std::optional<AssistUnitDesc>& unit) {
    std::array<TractionControlDesc, kTractionModeCount> out;
    out.fill(base);
    if (!unit || !unit->has_traction_control) return out;
    for (int m = 2; m < kTractionModeCount; ++m) {
        const TractionModeParams& p = unit->traction_control.params[static_cast<std::size_t>(m)];
        TractionControlDesc& d = out[static_cast<std::size_t>(m)];
        if (p.target_slip) d.target_slip = *p.target_slip;
        if (p.kp) d.kp = *p.kp;
        if (p.ki_per_s) d.ki_per_s = *p.ki_per_s;
        if (p.attack_rate_per_s) d.attack_rate_per_s = *p.attack_rate_per_s;
        if (p.recovery_rate_per_s) d.recovery_rate_per_s = *p.recovery_rate_per_s;
        if (p.min_fraction) d.min_fraction = *p.min_fraction;
        if (p.ignition_cut) d.ignition_cut = *p.ignition_cut;
        if (unit->traction_control.modes_bits & traction_mode_bit(static_cast<TractionMode>(m))) {
            try {
                validate_traction_control(d);
            } catch (const std::invalid_argument& e) {
                throw std::invalid_argument(std::string("traction mode '") + kTractionModeNames[m] + "': " + e.what());
            }
        }
    }
    return out;
}

} // namespace ps::vehicle
