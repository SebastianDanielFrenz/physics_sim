// ps/machine/soil_source.h — what a tool asks of the ground.
//
// ps_core does not link ps_field: the world holds one of these and the game
// implements it on whatever keeps its soil, the farm on ps::field::Field's views.
// sample() is called from the vehicle jobs, several at once and once per tool per
// tick, so it reads, it never allocates and it never touches a lock a tick could
// wait on. What a tool changes in the soil reaches it only after the game has
// applied the drained work, between ticks.

#pragma once

#include "ps/types.h"

namespace ps::machine {

enum class SoilTexture : unsigned char { Fine = 0, Medium = 1, Coarse = 2 };

struct SoilSample {
    bool workable = false; // false: a road, a yard, anything a tool scrapes over without working it
    SoilTexture texture = SoilTexture::Medium;
    real wetness = 0.0;    // root-zone water: 0 wilting point, 1 field capacity, above 1 wetter
    real tilled = 0.0;     // 0 untilled, 1 freshly worked
    real compaction = 0.0; // 0 none, 1 the worst the game's soil gets
};

class ISoilSource {
public:
    virtual ~ISoilSource() = default;
    [[nodiscard]] virtual SoilSample sample(real x, real y) const noexcept = 0; // world metres
};

} // namespace ps::machine
