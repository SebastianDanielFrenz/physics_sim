#include "ps/io/test_ground_loader.h"

#include <nlohmann/json.hpp>

#include <bit>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace ps::io {
namespace {

using json = nlohmann::json;

// The .bin is read into memory in ONE bulk read and parsed from there.
// The previous per-scalar seekg+read cost ~608k syscall pairs for the asphalt
// group alone, which dominated load time on WSL's 9p /mnt/s mount.
//
// Byte order: test_ground_collision.json documents the .bin as
// "Little-endian, 4-byte aligned throughout", and scalars are copied
// verbatim (std::memcpy, so alignment is never assumed), which is only
// correct on a little-endian host. This project's only build targets
// (Windows/WSL x86_64) are; the static_assert makes the assumption explicit
// instead of silently misreading on anything else. Not a general-purpose
// portable binary reader.
static_assert(std::endian::native == std::endian::little,
              "test_ground_collision.bin is little-endian and is parsed by raw copy");
static_assert(sizeof(float) == 4, "expects IEEE-754 binary32 float");
static_assert(std::numeric_limits<float>::is_iec559, "expects IEEE-754 binary32 float");

std::vector<unsigned char> read_whole_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) throw std::runtime_error("cannot open " + path.string());
    const std::streamoff size = in.tellg();
    if (size < 0) throw std::runtime_error("cannot determine the size of " + path.string());
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
    in.seekg(0, std::ios::beg);
    if (!bytes.empty()) in.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!in || in.gcount() != size) {
        throw std::runtime_error("short read of " + path.string() + ": expected " + std::to_string(size) +
                                 " bytes, got " + std::to_string(in.gcount()));
    }
    return bytes;
}

// Returns a pointer to `count` elements of `elem_size` bytes starting at
// byte `offset`, after checking the whole block lies inside the file
// (overflow-safe: never computes offset + count * elem_size unchecked).
// Throws naming the surface/field on a truncated or corrupt file.
const unsigned char* checked_block(const std::vector<unsigned char>& bin, std::size_t offset, std::size_t count,
                                   std::size_t elem_size, const std::string& surface, const char* what) {
    const std::size_t file_size = bin.size();
    const bool fits = offset <= file_size && count <= (file_size - offset) / elem_size;
    if (!fits) {
        throw std::runtime_error("test_ground_collision.bin: surface '" + surface + "' " + what + " block (offset " +
                                 std::to_string(offset) + ", " + std::to_string(count) + " x " +
                                 std::to_string(elem_size) + " bytes) runs past the end of the " +
                                 std::to_string(file_size) + "-byte file (truncated or corrupt)");
    }
    return bin.data() + offset;
}

struct SurfaceGroup {
    std::string surface;
    std::size_t vertex_offset = 0; // bytes into the .bin file
    std::size_t vertex_count = 0;  // vertices (each 3 float32s)
    std::size_t index_offset = 0;  // bytes into the .bin file
    std::size_t index_count = 0;   // uint32 indices (triangle_count * 3)
};

// PLAN.md P4 I0 (PHYS-033 section 9 / owner ruling Q7: strict check, no
// legacy support). data/tracks/test_ground/test_ground_collision.json is
// owned by a separate session (trackgen) and read-only from here; it
// already carries "physics_sim.collision/1" (checked, not edited).
void require_format(const json& j, const std::string& json_path) {
    const std::string expected = "physics_sim.collision/1";
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(json_path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        throw std::runtime_error(json_path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
    }
}

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
    require_format(j, json_path.string());

    const std::vector<unsigned char> bin = read_whole_file(bin_path);

    std::vector<BodyId> body_ids;
    for (const SurfaceGroup& g : read_groups(j)) {
        const SurfaceId surface_id = surfaces.id_for(g.surface);
        if (surface_id == kInvalidSurfaceId) {
            throw std::runtime_error("test_ground_collision.json surface '" + g.surface +
                                      "' has no id in the given SurfaceTable");
        }

        if (g.index_count % 3 != 0) {
            throw std::runtime_error("test_ground_collision.json surface '" + g.surface + "' has a non-multiple-of-3 index_count");
        }
        const unsigned char* vertex_bytes =
            checked_block(bin, g.vertex_offset, g.vertex_count, 3 * sizeof(float), g.surface, "vertex");
        const unsigned char* index_bytes =
            checked_block(bin, g.index_offset, g.index_count, sizeof(std::uint32_t), g.surface, "index");

        MeshShape mesh;
        mesh.vertices.reserve(g.vertex_count);
        for (std::size_t v = 0; v < g.vertex_count; ++v) {
            float xyz[3];
            std::memcpy(xyz, vertex_bytes + v * sizeof(xyz), sizeof(xyz));
            mesh.vertices.push_back(Vec3{static_cast<real>(xyz[0]), static_cast<real>(xyz[1]), static_cast<real>(xyz[2])});
        }

        mesh.indices.resize(g.index_count);
        if (g.index_count > 0) std::memcpy(mesh.indices.data(), index_bytes, g.index_count * sizeof(std::uint32_t));
        for (const std::uint32_t index : mesh.indices) {
            if (index >= g.vertex_count) {
                throw std::runtime_error("test_ground_collision.bin: surface '" + g.surface + "' has triangle index " +
                                         std::to_string(index) + " but only " + std::to_string(g.vertex_count) +
                                         " vertices (corrupt file)");
            }
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
