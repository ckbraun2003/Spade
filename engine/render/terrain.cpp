#include "render/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "render/shadow.hpp"

namespace spade::render {

namespace {

constexpr uint32_t kMaxSamplesAcross = 16384;  // a bound on what a caller can ask this function to hold

[[nodiscard]] Error refuse(std::string why) { return Error{Code::invalid_argument, "terrain: " + std::move(why)}; }

// The source grid at (east, north), bilinear between the sample centres and clamped to the first and last ones: the
// map's edge takes its nearest sample.
class HeightSampler {
public:
    explicit HeightSampler(const world::TerrainInput& map)
        : map_(map),
          width_(map.extent.east_max - map.extent.east_min),
          height_(map.extent.north_max - map.extent.north_min) {}

    [[nodiscard]] double at(double east, double north) const {
        double fx = (east - map_.extent.east_min) / width_ * map_.nx - 0.5;
        double fy = (map_.extent.north_max - north) / height_ * map_.ny - 0.5;
        fx = std::clamp(fx, 0.0, static_cast<double>(map_.nx - 1u));
        fy = std::clamp(fy, 0.0, static_cast<double>(map_.ny - 1u));
        const uint32_t i0 = static_cast<uint32_t>(fx);  // fx >= 0, so the cast is the floor
        const uint32_t j0 = static_cast<uint32_t>(fy);
        const uint32_t i1 = std::min(i0 + 1u, map_.nx - 1u);
        const uint32_t j1 = std::min(j0 + 1u, map_.ny - 1u);
        const double tx = fx - static_cast<double>(i0);
        const double ty = fy - static_cast<double>(j0);
        const auto h = [&](uint32_t i, uint32_t j) {
            return static_cast<double>(map_.heights[static_cast<size_t>(j) * map_.nx + i]);
        };
        const double top = h(i0, j0) * (1.0 - tx) + h(i1, j0) * tx;
        const double bottom = h(i0, j1) * (1.0 - tx) + h(i1, j1) * tx;
        return top * (1.0 - ty) + bottom * ty;
    }

private:
    const world::TerrainInput& map_;
    double width_, height_;
};

[[nodiscard]] Result<void> validate(const world::TerrainInput& map, const TerrainSpec& spec) {
    if (spec.max_cells == 0u) return std::unexpected(refuse("max_cells is 0"));
    if (map.nx < 2u || map.ny < 2u) {
        return std::unexpected(refuse(std::format("the height grid is {} x {}; it needs at least 2 x 2", map.nx, map.ny)));
    }
    if (map.nx > kMaxSamplesAcross || map.ny > kMaxSamplesAcross) {
        return std::unexpected(refuse(std::format("the height grid is {} x {}; the most a side holds is {}", map.nx, map.ny,
                                                  kMaxSamplesAcross)));
    }
    if (map.heights.size() != static_cast<size_t>(map.nx) * map.ny) {
        return std::unexpected(refuse(std::format("the height array holds {} values, not {} x {}", map.heights.size(),
                                                  map.nx, map.ny)));
    }
    if (map.image.width < 2u || map.image.height < 2u) {
        return std::unexpected(refuse(std::format("the image is {} x {}; it needs at least 2 x 2", map.image.width,
                                                  map.image.height)));
    }
    if (map.image.width > kMaxSamplesAcross || map.image.height > kMaxSamplesAcross ||
        map.image.rgb.size() != static_cast<size_t>(map.image.width) * map.image.height * 3u) {
        return std::unexpected(refuse(std::format("the image array holds {} bytes, not {} x {} x 3", map.image.rgb.size(),
                                                  map.image.width, map.image.height)));
    }
    const world::TerrainExtent& e = map.extent;
    if (!std::isfinite(e.east_min) || !std::isfinite(e.east_max) || !std::isfinite(e.north_min) ||
        !std::isfinite(e.north_max)) {
        return std::unexpected(refuse("the extent is not finite"));
    }
    if (!(e.east_max > e.east_min) || !(e.north_max > e.north_min)) {
        return std::unexpected(refuse("the extent is empty or reversed (east_min < east_max and north_min < north_max)"));
    }
    // REQ-G07: a side of at most 1 km, with one source cell of tolerance.
    const double side_x = e.east_max - e.east_min, side_z = e.north_max - e.north_min;
    if (side_x > kTerrainMaxSideM + side_x / map.nx || side_z > kTerrainMaxSideM + side_z / map.ny) {
        return std::unexpected(refuse(std::format("the area is {:.1f} x {:.1f} m; a side is at most 1000 m (REQ-G07)",
                                                  side_x, side_z)));
    }
    for (const float h : map.heights) {
        if (!std::isfinite(h)) return std::unexpected(refuse("the height grid holds a value that is not finite"));
    }
    return {};
}

}  // namespace

Result<TerrainStats> attach_terrain(RenderScene& scene, const world::TerrainInput& map, const TerrainSpec& spec) {
    if (auto ok = validate(map, spec); !ok) return std::unexpected(ok.error());

    const world::TerrainExtent& ext = map.extent;
    const uint32_t cells_x = std::min(map.nx, spec.max_cells);
    const uint32_t cells_z = std::min(map.ny, spec.max_cells);
    const uint32_t verts_x = cells_x + 1u;
    const uint32_t verts_z = cells_z + 1u;
    const double width = ext.east_max - ext.east_min;
    const double depth = ext.north_max - ext.north_min;
    const HeightSampler source(map);

    MeshData mesh;
    mesh.positions.resize(static_cast<size_t>(verts_x) * verts_z);
    for (uint32_t j = 0; j < verts_z; ++j) {
        const double north = ext.north_max - depth * (static_cast<double>(j) / cells_z);
        for (uint32_t i = 0; i < verts_x; ++i) {
            const double east = ext.east_min + width * (static_cast<double>(i) / cells_x);
            mesh.positions[static_cast<size_t>(j) * verts_x + i] =
                glm::vec3(static_cast<float>(east), static_cast<float>(source.at(east, north)),
                          static_cast<float>(-north));
        }
    }

    // Normals: the slope by central differences of the vertex heights (one-sided at the edge), as the surface (-gx, 1, -gz).
    mesh.normals.resize(mesh.positions.size());
    const auto vertex = [&](uint32_t i, uint32_t j) -> const glm::vec3& {
        return mesh.positions[static_cast<size_t>(j) * verts_x + i];
    };
    for (uint32_t j = 0; j < verts_z; ++j) {
        const uint32_t j0 = j == 0u ? 0u : j - 1u, j1 = std::min(j + 1u, cells_z);
        for (uint32_t i = 0; i < verts_x; ++i) {
            const uint32_t i0 = i == 0u ? 0u : i - 1u, i1 = std::min(i + 1u, cells_x);
            const float gx = (vertex(i1, j).y - vertex(i0, j).y) / (vertex(i1, j).x - vertex(i0, j).x);
            const float gz = (vertex(i, j1).y - vertex(i, j0).y) / (vertex(i, j1).z - vertex(i, j0).z);
            mesh.normals[static_cast<size_t>(j) * verts_x + i] = glm::normalize(glm::vec3(0.0f - gx, 1.0f, 0.0f - gz));
        }
    }

    // Two triangles a cell, both wound to face +y: (v(i,j), v(i,j+1), v(i+1,j)) and (v(i+1,j), v(i,j+1), v(i+1,j+1)).
    mesh.indices.reserve(static_cast<size_t>(cells_x) * cells_z * 6u);
    for (uint32_t j = 0; j < cells_z; ++j) {
        for (uint32_t i = 0; i < cells_x; ++i) {
            const uint32_t a = j * verts_x + i, b = (j + 1u) * verts_x + i;
            const uint32_t c = j * verts_x + i + 1u, d = (j + 1u) * verts_x + i + 1u;
            mesh.indices.insert(mesh.indices.end(), {a, b, c, c, b, d});
        }
    }

    TerrainStats stats;
    stats.cells_x = cells_x;
    stats.cells_z = cells_z;
    stats.triangles = 2u * cells_x * cells_z;
    stats.lowest_y_m = std::numeric_limits<float>::infinity();
    for (const glm::vec3& p : mesh.positions) stats.lowest_y_m = std::min(stats.lowest_y_m, p.y);
    // The gap to the source, at each cell's centre: the mesh there is the mean of the diagonal's ends (b and c).
    for (uint32_t j = 0; j < cells_z; ++j) {
        for (uint32_t i = 0; i < cells_x; ++i) {
            const glm::vec3& b = vertex(i, j + 1u);
            const glm::vec3& c = vertex(i + 1u, j);
            const double mesh_y = 0.5 * (static_cast<double>(b.y) + static_cast<double>(c.y));
            const double east = ext.east_min + width * ((static_cast<double>(i) + 0.5) / cells_x);
            const double north = ext.north_max - depth * ((static_cast<double>(j) + 0.5) / cells_z);
            stats.max_gap_m = std::max(stats.max_gap_m, static_cast<float>(std::abs(mesh_y - source.at(east, north))));
        }
    }

    // ---- from here the scene changes; nothing above can fail ----

    const uint32_t mesh_index = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(std::move(mesh));
    const uint32_t material = static_cast<uint32_t>(scene.materials.size());
    scene.materials.push_back(Material{glm::vec4(spec.base_color, 1.0f), spec.lit ? 0u : 1u});
    scene.statics.push_back(DrawItem{.mesh_index = mesh_index,
                                     .material_override = material,
                                     .object_label = intern_label(scene, "terrain"),
                                     .casts_shadow = false});

    // The floor rule: the render copy of each floor under (or level with) the map's lowest vertex goes just below it.
    const float target = stats.lowest_y_m - kTerrainGroundMargin;
    for (GroundPlane& plane : scene.ground_planes) {
        if (!(plane.normal.y > 0.999f)) continue;  // a wall or a slope is no floor
        const float y_plane = plane.offset / plane.normal.y;
        if (!(y_plane > target)) continue;
        const float delta = y_plane - target;
        plane.offset -= delta * plane.normal.y;
        if (plane.item < scene.statics.size()) {
            DrawItem& item = scene.statics[plane.item];
            item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -delta, 0.0f)) * item.local_to_world;
        }
        stats.ground_lowered_m = std::max(stats.ground_lowered_m, delta);
        ++stats.planes_lowered;
    }
    // A floor is one of the sun's static casters: rebuild the map for the lowered one (the terrain casts none).
    if (stats.planes_lowered > 0u && scene.static_shadow.has_value()) {
        Result<ShadowMap> rebuilt = build_static_shadow_map(scene);
        if (!rebuilt) return std::unexpected(rebuilt.error());
        scene.static_shadow = std::move(*rebuilt);
    }
    return stats;
}

}  // namespace spade::render
