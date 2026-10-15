// ps/vehicle/epb_desc.h - electronic parking brake (queue item Q3 / assists I8, docs/data_formats/electronics.md "epb"):
// electronics kind `epb`, channel `epb` (a switch level 0/1), per drive mode `epb_mode` park_only | handbrake.
//
// Owner rule, verbatim: "No, unless special modes are used (racing, drift), the parking brake should refuse to engage
// while speed is significant."
//
// The EPB drives the car's existing handbrake rows (BrakeSystemDesc::handbrake) - no rows of its own. park_only engages
// only below max_engage_speed_mps (a refused request is dropped, never pended: it engages only on a NEW rising edge);
// handbrake applies the rows at any speed. park_lock (the gearbox pawl) stays a separate channel.
#pragma once

#include "ps/types.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace ps::vehicle {

// Append-only codes.
enum class EpbMode : std::uint8_t { ParkOnly = 0, Handbrake = 1 };
inline constexpr const char* kEpbModeNames[2] = {"park_only", "handbrake"};

// Snapshot code of PowertrainSnapshot::epb_state (racing_game: 0 released / 1 latched / 2 handbrake).
enum class EpbState : std::uint8_t { Released = 0, Latched = 1, Handbrake = 2 };

struct EpbDesc {
    // park_only gate on |chassis velocity| at the tick start, m/s. [AS] 2.0 (walking-pace-and-a-bit; config).
    real max_engage_speed_mps = 2.0;
    // The mode when no drive mode sets one (a car without a drive_modes part, or a mode without epb_mode).
    EpbMode epb_mode = EpbMode::ParkOnly;
};

// Shared by the loader and VehicleInstance (code-built descs). Throws std::invalid_argument.
inline void validate_epb(const EpbDesc& d) {
    if (!std::isfinite(d.max_engage_speed_mps) || !(d.max_engage_speed_mps > real(0))) {
        throw std::invalid_argument("epb: max_engage_speed_mps must be finite and > 0");
    }
    if (static_cast<unsigned>(d.epb_mode) > 1u) throw std::invalid_argument("epb: unknown epb_mode");
}

} // namespace ps::vehicle
