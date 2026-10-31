// ps/machine/hitch_class.h — what a hitch class holds and what it leaves free
// (physics_sim.hitch_class/1, data/hitch_classes/*.json, docs/data_formats/hitch.md).
//
// A hitch always holds the two sockets together. Rotation is described by the
// Cardan angles of the towed hitch frame in the towing one, in the order yaw
// about the towing z, pitch about the resulting y, roll about the towed x, and
// each of the three is free, limited to a range or locked at zero.

#pragma once

#include "ps/types.h"

#include <string>

namespace ps::machine {

enum class HitchAxisMode : unsigned char { Free, Limited, Locked };

struct HitchAxis {
    HitchAxisMode mode = HitchAxisMode::Free;
    real min_rad = 0.0, max_rad = 0.0; // Limited only
};

struct HitchClassDesc {
    std::string name;
    std::string source;
    HitchAxis yaw, pitch, roll;

    // Every row of the hitch rings at this frequency whatever the two masses are: the stiffness is omega^2 over the
    // inverse mass seen by the rows (core/src/machine/coupling_solver.h).
    real frequency_hz = 0.0;
    real damping_ratio = 1.0;

    // Attaching: the sockets may be this far apart, the frames this far turned (locked axes) and the sockets this fast
    // apart. Within them, the target starts at the real error and rate and reaches zero after latch_time_s.
    real latch_time_s = 0.0;
    real max_latch_error_m = 0.0;
    real max_latch_angle_rad = 0.0;
    real max_latch_speed_m_s = 0.0;
};

[[nodiscard]] inline bool has_rotational_rows(const HitchClassDesc& d) {
    return d.yaw.mode != HitchAxisMode::Free || d.pitch.mode != HitchAxisMode::Free || d.roll.mode != HitchAxisMode::Free;
}

// Throws std::invalid_argument naming the class and the field.
void validate_hitch_class(const HitchClassDesc& d);

} // namespace ps::machine
