// ps/io/test_ground_loader.h — PLAN.md M1 step 4: static test-ground
// collision. Loads a track directory's `test_ground_collision.json` +
// `test_ground_collision.bin` (see that JSON file's own "layout" field for
// the exact binary format - a small data-driven format, not specific to
// data/tracks/test_ground; any directory following the same two-file
// convention works) into `world` as one static MeshShape body per surface
// group, each tagged with that group's SurfaceId from `surfaces` (PLAN.md
// section 6: "surface ids must survive collision queries and contact
// events, so the tyre/contact code can look up a surface").
//
// No hardcoded path beyond the two filenames the format itself is named
// after (`<track_dir>/test_ground_collision.json` / `.bin`) - `track_dir`
// is the only path this function needs from its caller, so it works for
// any track directory built the same way, not only test_ground.

#pragma once

#include "ps/io/surface_table.h"
#include "ps/world/world.h"

#include <string>
#include <vector>

namespace ps::io {

// Returns one BodyId per surface group actually created, in the same
// order as the JSON's own "surfaces" array. Throws std::runtime_error on a
// malformed/missing/short file, or if a surface name named in the JSON has
// no id in `surfaces` (SurfaceTable::id_for returning kInvalidSurfaceId).
std::vector<BodyId> load_test_ground_collision(World& world, const std::string& track_dir, const SurfaceTable& surfaces);

} // namespace ps::io
