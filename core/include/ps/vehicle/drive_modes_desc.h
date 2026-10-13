// ps/vehicle/drive_modes_desc.h - drive modes ECO / Normal / Sport (assists plan I7, vault PHYS-074,
// docs/data_formats/electronics.md "drive_modes"): a CAR-SPECIFIC electronics part (kind `drive_modes`) that bundles,
// per mode, the pedal bend, the auto-shift rpm points, the auto-clutch close time after a shift, the traction-control
// default and the launch-control / nitrous gates. Mirrors the traction-mode pattern of assist_unit_desc.h (channel,
// available bits, requested/effective in the snapshot).
#pragma once

#include "ps/types.h"
#include "ps/vehicle/assist_unit_desc.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace ps::vehicle {

// Mode CODES of the channel `assist.drive_mode`. Append-only (owner: Eco, Normal, Sport).
enum class DriveMode : std::uint8_t { Eco = 0, Normal = 1, Sport = 2 };
inline constexpr int kDriveModeCount = 3;
inline constexpr const char* kDriveModeNames[kDriveModeCount] = {"eco", "normal", "sport"};
[[nodiscard]] constexpr std::uint8_t drive_mode_bit(DriveMode m) { return static_cast<std::uint8_t>(1u << static_cast<unsigned>(m)); }

// One bend point: pedal 0 -> 0, bend_x -> bend_y, 1 -> 1, piecewise linear. x in (0, 1), y in [0, 1] (owner).
struct PedalBend {
    real x = 0, y = 0;
};

// out = bend(in), in clamped to [0, 1]. Only + - * / and comparisons (determinism rule: no libm).
[[nodiscard]] constexpr real pedal_bend_apply(const PedalBend& b, real in) {
    const real v = in < real(0) ? real(0) : (in > real(1) ? real(1) : in);
    if (v >= real(1)) return real(1); // the end points are exact (full pedal stays exactly 1)
    return v <= b.x ? v * (b.y / b.x) : b.y + (v - b.x) * ((real(1) - b.y) / (real(1) - b.x));
}

// Everything a mode overrides. An unset optional keeps the car's own value (pedal curve, TCU thresholds, TC channel).
struct DriveModeParams {
    std::optional<PedalBend> pedal_bend;
    // Auto-shift points (rpm) at zero / full throttle command; each overrides the matching AutoShiftDesc field.
    std::optional<real> up_rpm_at_zero_throttle, up_rpm_at_full_throttle, down_rpm_at_zero_throttle, down_rpm_at_full_throttle;
    // Overrides AutoClutchDesc::close_time_s (the post-shift e_auto close rate 1/close_time_s).
    std::optional<real> clutch_close_time_s;
    // The traction mode the car switches to when this drive mode becomes effective (until the driver changes the
    // assist.traction_control channel). Resolved through the assist-unit availability fallback.
    std::optional<TractionMode> traction_control_default;
    // Assists I7b: the launch-control ENABLE the car switches to when this drive mode becomes effective (until the driver
    // changes the assist.launch_control channel value) - the same rule as traction_control_default. Unset = this mode
    // does not touch the enable. launch_control_allowed still gates on top: a mode that does not allow LC never arms.
    std::optional<bool> launch_control_default;
    bool launch_control_allowed = true;
    bool nitrous_arm_allowed = true;
};

struct DriveModesDesc {
    std::uint8_t modes_bits = 0;
    DriveMode default_mode = DriveMode::Normal;
    std::array<DriveModeParams, kDriveModeCount> params{};
};

// Channel value -> requested mode code: round to nearest, clamp to 0..2; non-finite is the default mode.
[[nodiscard]] inline DriveMode drive_mode_requested(const DriveModesDesc& d, real channel_value) {
    if (!std::isfinite(channel_value)) return d.default_mode;
    const real r = std::round(channel_value);
    return static_cast<DriveMode>(r <= real(0) ? 0 : (r >= real(2) ? 2 : static_cast<int>(r)));
}

// The mode actually run. A mode the part does not offer maps, statelessly, to the part's default mode (drive modes
// have no intervention order, so "the nearest" is not defined; the default is the one mode the owner approved).
[[nodiscard]] constexpr DriveMode resolve_drive_mode(const DriveModesDesc& d, DriveMode requested) {
    return (d.modes_bits & drive_mode_bit(requested)) ? requested : d.default_mode;
}

// The TC channel's default value with drive modes: the default drive mode's traction_control_default code when it has
// one and TC is fitted, else the base rule.
[[nodiscard]] inline real traction_default_channel_value(const TractionControlDesc& base, const std::optional<AssistUnitDesc>& unit,
                                                         const std::optional<DriveModesDesc>& modes) {
    if (base.fitted && modes) {
        const auto& tc = modes->params[static_cast<std::size_t>(modes->default_mode)].traction_control_default;
        if (tc) return static_cast<real>(static_cast<int>(*tc));
    }
    return traction_default_channel_value(base, unit);
}

// Assists I7b: true iff any OFFERED mode sets launch_control_default. Gates the extra state and hash fold, so a part that
// sets none (and every car without the part) hashes exactly as before I7b.
[[nodiscard]] inline bool any_launch_control_default(const DriveModesDesc& d) {
    for (int m = 0; m < kDriveModeCount; ++m) {
        if ((d.modes_bits & drive_mode_bit(static_cast<DriveMode>(m))) && d.params[static_cast<std::size_t>(m)].launch_control_default) return true;
    }
    return false;
}

// The assist.launch_control channel's default with drive modes: the default drive mode's launch_control_default when it
// has one, else `base` (the unit block's default_enabled, or 0).
[[nodiscard]] inline real launch_control_default_channel_value(real base, const std::optional<DriveModesDesc>& modes) {
    if (modes) {
        const auto& lc = modes->params[static_cast<std::size_t>(modes->default_mode)].launch_control_default;
        if (lc) return *lc ? real(1) : real(0);
    }
    return base;
}

// Range checks shared by the loader and VehicleInstance (code-built descs). Throws std::invalid_argument.
inline void validate_drive_modes(const DriveModesDesc& d) {
    const auto fin = [](real v) { return std::isfinite(v); };
    if (d.modes_bits == 0 || (d.modes_bits & ~std::uint8_t(0b111)) != 0) throw std::invalid_argument("drive_modes: modes must be a non-empty subset of eco, normal, sport");
    if (!(d.modes_bits & drive_mode_bit(d.default_mode))) throw std::invalid_argument("drive_modes: default_mode is not one of the offered modes");
    for (int m = 0; m < kDriveModeCount; ++m) {
        const DriveModeParams& p = d.params[static_cast<std::size_t>(m)];
        const std::string ctx = std::string("drive_modes: mode '") + kDriveModeNames[m] + "': ";
        if (p.pedal_bend) {
            if (!fin(p.pedal_bend->x) || !(p.pedal_bend->x > real(0)) || !(p.pedal_bend->x < real(1))) throw std::invalid_argument(ctx + "pedal_bend x must be in (0, 1)");
            if (!fin(p.pedal_bend->y) || p.pedal_bend->y < real(0) || p.pedal_bend->y > real(1)) throw std::invalid_argument(ctx + "pedal_bend y must be in [0, 1]");
        }
        if (p.launch_control_default && *p.launch_control_default && !p.launch_control_allowed) {
            throw std::invalid_argument(ctx + "launch_control_default true contradicts launch_control_allowed false");
        }
        for (const std::optional<real>* r : {&p.up_rpm_at_zero_throttle, &p.up_rpm_at_full_throttle, &p.down_rpm_at_zero_throttle, &p.down_rpm_at_full_throttle}) {
            if (*r && (!fin(**r) || !(**r > real(0)))) throw std::invalid_argument(ctx + "auto-shift rpm values must be finite and > 0");
        }
        if (p.clutch_close_time_s && (!fin(*p.clutch_close_time_s) || !(*p.clutch_close_time_s > real(0)))) throw std::invalid_argument(ctx + "clutch_close_time_s must be finite and > 0");
    }
}

} // namespace ps::vehicle
