// ps/vehicle/drive_modes_desc.h - drive modes ECO / Normal / Sport (assists plan I7, vault PHYS-074,
// docs/data_formats/electronics.md "drive_modes"): a CAR-SPECIFIC electronics part (kind `drive_modes`) that bundles,
// per mode, the pedal bend, the auto-shift rpm points, the auto-clutch close time after a shift, the traction-control
// default and the launch-control / nitrous gates. Mirrors the traction-mode pattern of assist_unit_desc.h (channel,
// available bits, requested/effective in the snapshot).
#pragma once

#include "ps/types.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/vehicle/epb_desc.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace ps::vehicle {

// Mode CODES of the channel `assist.drive_mode`. Append-only (owner: Eco, Normal, Sport; Racing added by Q3 / I8).
enum class DriveMode : std::uint8_t { Eco = 0, Normal = 1, Sport = 2, Racing = 3 };
inline constexpr int kDriveModeCount = 4;
inline constexpr const char* kDriveModeNames[kDriveModeCount] = {"eco", "normal", "sport", "racing"};
inline constexpr std::uint8_t kDriveModeAllBits = static_cast<std::uint8_t>((1u << kDriveModeCount) - 1u);
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
    // The traction mode the car switches to when this drive mode becomes effective: the physics WRITES that code into
    // the assist.traction_control channel (assists I7c; applied by World::step after the tick, so a caller sees it via
    // get_control), and the driver's next write of any value wins. Resolved through the assist-unit availability fallback.
    std::optional<TractionMode> traction_control_default;
    // Assists I7b/I7c: the launch-control ENABLE the car switches to when this drive mode becomes effective: written into
    // the assist.launch_control channel (1 / 0) exactly like traction_control_default. Unset = this mode does not touch
    // the enable. launch_control_allowed still gates on top: a mode that does not allow LC never arms.
    std::optional<bool> launch_control_default;
    bool launch_control_allowed = true;
    bool nitrous_arm_allowed = true;
    // Q3 / I8: the EPB mode while this drive mode runs (unset = the epb part's own epb_mode). Ignored without an epb part.
    std::optional<EpbMode> epb_mode;
    // ABS / ESC: the assist.abs enable and the assist.esc mode the car switches to when this drive mode becomes effective,
    // written into the channels like traction_control_default. Ignored on a car whose unit lacks the block.
    std::optional<bool> abs_default;
    std::optional<EscMode> esc_default;
};

struct DriveModesDesc {
    std::uint8_t modes_bits = 0;
    DriveMode default_mode = DriveMode::Normal;
    std::array<DriveModeParams, kDriveModeCount> params{};
};

// Channel value -> requested mode code: round to nearest, clamp to 0..kDriveModeCount-1; non-finite is the default mode.
// (Q3: a value of 3 used to clamp to Sport; it now requests Racing, which an unoffered part maps to its default mode.)
[[nodiscard]] inline DriveMode drive_mode_requested(const DriveModesDesc& d, real channel_value) {
    if (!std::isfinite(channel_value)) return d.default_mode;
    const real r = std::round(channel_value);
    return static_cast<DriveMode>(r <= real(0) ? 0 : (r >= real(kDriveModeCount - 1) ? kDriveModeCount - 1 : static_cast<int>(r)));
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

// The assist.abs / assist.esc channel defaults: the default drive mode's value when it sets one, else the unit block's.
[[nodiscard]] inline real abs_default_channel_value(const AbsDesc& abs, const std::optional<DriveModesDesc>& modes) {
    if (modes) {
        const auto& v = modes->params[static_cast<std::size_t>(modes->default_mode)].abs_default;
        if (v) return *v ? real(1) : real(0);
    }
    return abs.default_enabled ? real(1) : real(0);
}
[[nodiscard]] inline real esc_default_channel_value(const EscDesc& esc, const std::optional<DriveModesDesc>& modes) {
    if (modes) {
        const auto& v = modes->params[static_cast<std::size_t>(modes->default_mode)].esc_default;
        if (v) return static_cast<real>(static_cast<int>(*v));
    }
    return static_cast<real>(static_cast<int>(esc.default_mode));
}

// Range checks shared by the loader and VehicleInstance (code-built descs). Throws std::invalid_argument.
inline void validate_drive_modes(const DriveModesDesc& d) {
    const auto fin = [](real v) { return std::isfinite(v); };
    if (d.modes_bits == 0 || (d.modes_bits & static_cast<std::uint8_t>(~kDriveModeAllBits)) != 0) throw std::invalid_argument("drive_modes: modes must be a non-empty subset of eco, normal, sport, racing");
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
        if (p.esc_default && static_cast<int>(*p.esc_default) >= kEscModeCount) throw std::invalid_argument(ctx + "unknown esc_default");
        if (p.epb_mode && static_cast<unsigned>(*p.epb_mode) > 1u) throw std::invalid_argument(ctx + "unknown epb_mode");
        if (p.clutch_close_time_s && (!fin(*p.clutch_close_time_s) || !(*p.clutch_close_time_s > real(0)))) throw std::invalid_argument(ctx + "clutch_close_time_s must be finite and > 0");
    }
}

} // namespace ps::vehicle
