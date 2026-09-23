#include "ps/terrain/procedural_terrain.h"

#include "procedural_terrain_source.h"

namespace ps::terrain {

std::shared_ptr<ITerrainSource> load_procedural_terrain_source(const std::string& terrain_json_path, std::uint32_t samples_per_tile) {
    return std::make_shared<ProceduralTerrainSource>(load_terrain_spec(terrain_json_path), samples_per_tile);
}

} // namespace ps::terrain
