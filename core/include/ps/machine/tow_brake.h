// ps/machine/tow_brake.h — what drives a towed vehicle's service brakes
// (the "tow_brake" block of a vehicle file, docs/data_formats/vehicle.md).
//
// A trailer has no driver. Either its brakes follow a channel of the towing
// vehicle, the air line of a tractor or a truck, or the trailer presses them
// itself when it runs up on the hitch, the overrun brake of a car trailer. Both
// act as the one brake channel a vehicle already has, through a first-order lag:
// a line takes time to fill, an overrun damper to stroke. Without a tow_brake
// block a towed vehicle's own brake channel is all it has, and a heavy trailer
// pushes the tow vehicle round as soon as anything breaks the symmetry.

#pragma once

#include "ps/types.h"

#include <stdexcept>
#include <string>

namespace ps::machine {

enum class TowBrakeKind : unsigned char { Channel, Overrun };

struct TowBrakeDesc {
    TowBrakeKind kind = TowBrakeKind::Channel;
    real lag_s = 0.25;

    // Channel: this channel of the towing vehicle, read in that vehicle's scope, times gain. A towing vehicle that is
    // itself towed passes its own demand on, so a chain brakes from the front.
    std::string channel = "brake";
    real gain = 1.0;

    // Overrun: no braking below deadband_n of hitch compression, full at full_force_n. Reversing faster than
    // reverse_release_speed_m_s releases it, as the reverse latch of a real coupling does.
    real deadband_n = 300.0;
    real full_force_n = 6000.0;
    real reverse_release_speed_m_s = 0.5;
};

inline void validate_tow_brake(const TowBrakeDesc& d) {
    if (!(d.lag_s >= 0.0)) throw std::invalid_argument("tow_brake: lag_s must not be negative");
    if (d.kind == TowBrakeKind::Channel) {
        if (d.channel.empty()) throw std::invalid_argument("tow_brake: channel is empty");
        if (!(d.gain > 0.0)) throw std::invalid_argument("tow_brake: gain must be positive");
    } else {
        if (!(d.deadband_n >= 0.0)) throw std::invalid_argument("tow_brake: deadband_n must not be negative");
        if (!(d.full_force_n > d.deadband_n)) throw std::invalid_argument("tow_brake: full_force_n must exceed deadband_n");
        if (!(d.reverse_release_speed_m_s >= 0.0)) throw std::invalid_argument("tow_brake: reverse_release_speed_m_s must not be negative");
    }
}

} // namespace ps::machine
