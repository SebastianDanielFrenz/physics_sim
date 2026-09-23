#include "ps/io/test_ground_loader.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

// Little-endian float32/uint32 reads at an absolute byte offset - this
// project's own CI legs (Windows/WSL, both x86_64, PLAN.md's CMakePresets)
// are little-endian, matching test_ground_collision.json's own documented
// "Little-endian, 4-byte aligned throughout" layout, so a raw read is
// enough; not a general-purpose portable binary reader.
float read_f32_at(std::ifstream& in, std::size_t byte_offset) {
    static_assert(sizeof(float) == 4, "expects IEEE-754 binary32 float");
    in.seekg(static_cast<std::streamoff>(byte_offset));
    float value = 0.0f;
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!in) throw std::runtime_error("test_ground_collision.bin: unexpected EOF reading a float32");
    return value;
}

std::uint32_t read_u32_at(std::ifstream& in, std::size_t byte_offset) {
    in.seekg(static_cast<std::streamoff>(byte_offset));
    std::uint32_t value = 0;
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!in) throw std::runtime_error("test_ground_collision.bin: unexpected EOF reading a uint32");
    return value;
}

struct SurfaceGroup {
    std::string surface;
    std::size_t vertex_offset = 0; // bytes into the .bin file
    std::size_t vertex_count = 0;  // vertices (each 3 float32s)
    std::size_t index_offset = 0;  // bytes into the .bin file
    std::size_t index_count = 0;   // uint32 indices (triangle_count * 3)
};

std::vector<SurfaceGroup> read_groups(const json& j) {
    std::vector<SurfaceGroup> groups;
    for (const auto& sj : j.at("surfaces")) {
        SurfaceGroup g;
        g.surface = sj.at("surface").get<std::string>();
        g.vertex_offset = sj.at("vertex_offset").get<std::size_t>();
        g.vertex_count = sj.at("vertex_count").get<std::size_t>();
        g.index_offset = sj.at("index_offset").get<std::size_t>();
        g.index_count = sj.at("index_count").get<std::size_t>();
        groups.push_back(std::move(g));
    }
    return groups;
}

} // namespace

std::vector<BodyId> load_test_ground_collision(World& world, const std::string& track_dir, const SurfaceTable& surfaces) {
    const std::filesystem::path dir(track_dir);
    const std::filesystem::path json_path = dir / "test_ground_collision.json";
    const std::filesystem::path bin_path = dir / "test_ground_collision.bin";

    std::ifstream json_in(json_path);
    if (!json_in) throw std::runtime_error("cannot open " + json_path.string());
    json j;
    try {
        json_in >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + json_path.string() + ": " + e.what());
    }

    std::ifstream bin_in(bin_path, std::ios::binary);
    if (!bin_in) throw std::runtime_error("cannot open " + bin_path.string());

    std::vector<BodyId> body_ids;
    for (const SurfaceGroup& g : read_groups(j)) {
        const SurfaceId surface_id = surfaces.id_for(g.surface);
        if (surface_id == kInvalidSurfaceId) {
            throw std::runtime_error("test_ground_collision.json surface '" + g.surface +
                                      "' has no id in the given SurfaceTable");
        }

        MeshShape mesh;
        mesh.vertices.reserve(g.vertex_count);
        for (std::size_t v = 0; v < g.vertex_count; ++v) {
            const std::size_t base = g.vertex_offset + v * 3 * sizeof(float);
            const real x = static_cast<real>(read_f32_at(bin_in, base));
            const real y = static_cast<real>(read_f32_at(bin_in, base + sizeof(float)));
            const real z = static_cast<real>(read_f32_at(bin_in, base + 2 * sizeof(float)));
            mesh.vertices.push_back(Vec3{x, y, z});
        }

        mesh.indices.reserve(g.index_count);
        for (std::size_t i = 0; i < g.index_count; ++i) {
            mesh.indices.push_back(read_u32_at(bin_in, g.index_offset + i * sizeof(std::uint32_t)));
        }

        if (mesh.indices.size() % 3 != 0) {
            throw std::runtime_error("test_ground_collision.json surface '" + g.surface + "' has a non-multiple-of-3 index_count");
        }
        mesh.triangle_surface_ids.assign(mesh.indices.size() / 3, surface_id);

        BodyDesc desc;
        desc.shape = std::move(mesh);
        desc.motion = BodyMotionType::Static;
        body_ids.push_back(world.create_body(desc));
    }
    return body_ids;
}

} // namespace ps::io
