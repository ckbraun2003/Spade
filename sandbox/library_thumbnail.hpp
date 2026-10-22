// The library's thumbnails (INT-14, N14b): each entry drawn alone on the CPU
// renderer, kThumbnailSize square, from one fixed three-quarter view that
// frames everything it draws. An object is composed into an empty world as a
// scene would place it (its materials, its collider and visual); a drone's
// parts are bound as the editor draws them (design frame at the origin); a
// part (Core's L4) is its own geometry, one primitive.
// tests/test_library_thumbnails.cpp writes the committed PNGs with
// SPADE_LIBRARY_WRITE set and checks them otherwise. Display-free.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "drone_view.hpp"
#include "editor_library.hpp"
#include "png_reader.hpp"
#include "render/mesh_resolver.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render_parts/render_parts.hpp"
#include "scene/compose.hpp"
#include "scene/part_file.hpp"
#include "scene/scene_file.hpp"
#include "vehicle_draw.hpp"
#include "vehicles/airframe_compile.hpp"
#include "world/builder.hpp"

namespace spade::sandbox::editor {

namespace detail {

// The view: from the front-right and above, the item's box filling the frame.
inline constexpr float kThumbnailYaw = 0.6108652f;    // 35 degrees
inline constexpr float kThumbnailPitch = 0.4363323f;  // 25 degrees
inline constexpr float kThumbnailFov = 0.6108652f;    // 35 degrees, vertical

[[nodiscard]] inline std::unexpected<Error> thumbnail_refusal(const LibraryItem& item, const std::string& why) {
    return std::unexpected(Error{Code::invalid_argument, "thumbnail '" + item.name + "': " + why});
}

// The box around every static's mesh, in the world.
[[nodiscard]] inline std::optional<render::Aabb> drawn_bounds(const render::RenderScene& scene) {
    glm::vec3 lo(std::numeric_limits<float>::infinity());
    glm::vec3 hi(-std::numeric_limits<float>::infinity());
    for (const render::DrawItem& item : scene.statics) {
        for (const glm::vec3& p : scene.meshes.at(item.mesh_index).positions) {
            const glm::vec3 w(item.local_to_world * glm::vec4(p, 1.0f));
            lo = glm::min(lo, w);
            hi = glm::max(hi, w);
        }
    }
    if (!(lo.x <= hi.x)) return std::nullopt;
    return render::Aabb{lo, hi};
}

// A part's own geometry as one primitive, as its file's shape sizes it
// (vehicles::BlockShape, part-local: a sphere's size.x the radius; a box's
// full extents; a cylinder or tube along +Y, size.x the radius and size.y the
// length). A prop with no shape is its disc: its diameter, a hundredth as
// thick (2 mm at least). None for a point mass or a tensor: no geometry.
[[nodiscard]] inline std::optional<scene::SceneAsset> part_asset(const scene::PartFile& part) {
    const auto* prop = std::get_if<scene::PropPart>(&part);
    if (prop && !prop->block.shape) {
        const float r = static_cast<float>(0.5 * prop->block.diameter);
        if (!(r > 0.0f)) return std::nullopt;
        return primitive_asset("part", SdfPrim::cylinder, glm::vec4(r, std::max(0.01f * r, 0.001f), 0.0f, 0.0f),
                               SdfPose{});
    }
    const std::optional<vehicles::BlockShape> shape =
        std::visit([](const auto& p) { return p.block.shape; }, part);
    if (!shape) return std::nullopt;
    const glm::vec3 size(shape->size);
    switch (shape->shape) {
        case vehicles::PartShape::sphere:
            return primitive_asset("part", SdfPrim::sphere, glm::vec4(size.x, 0.0f, 0.0f, 0.0f), SdfPose{});
        case vehicles::PartShape::box:
            return primitive_asset("part", SdfPrim::box, glm::vec4(0.5f * size, 0.0f), SdfPose{});
        case vehicles::PartShape::cylinder:
        case vehicles::PartShape::tube:
            return primitive_asset("part", SdfPrim::cylinder, glm::vec4(size.x, 0.5f * size.y, 0.0f, 0.0f),
                                   SdfPose{});
        case vehicles::PartShape::point:
        case vehicles::PartShape::tensor:
            break;
    }
    return std::nullopt;
}

}  // namespace detail

// `item` drawn alone, `size` x `size`, top row first. Meshes resolve from
// `mesh_roots` (the library's meshes/ folder). invalid_argument naming the item
// for a part with no geometry (detail::part_asset()), a model without parts, a
// mesh that resolves nowhere, or an item that draws nothing; whatever compose(),
// scene_from_world(), prepare_parts() or render() refuse, prefixed the same.
[[nodiscard]] inline Result<png::Image> render_thumbnail(const LibraryItem& item,
                                                         std::span<const std::filesystem::path> mesh_roots,
                                                         uint32_t size = kThumbnailSize) {
    std::optional<scene::SceneAsset> part;
    if (item.kind == LibraryItem::Kind::part) {
        if (item.part) part = detail::part_asset(*item.part);
        if (!part) {
            const std::string kind =
                item.part ? std::string(scene::part_kind_name(scene::part_kind(*item.part))) : std::string("part");
            return detail::thumbnail_refusal(item, "its " + kind +
                                                       " file has no shape, or a tensor one: no geometry to draw");
        }
    } else if (!placeable(item)) {
        return detail::thumbnail_refusal(item, "nothing to draw");
    }

    // An empty world: no ground, so the item is drawn alone.
    WorldBuilder base_builder;
    base_builder.name("thumbnail").environment(Environment{}).capacities(Capacities{1, 2, 3, 4});
    Result<WorldDesc> base = base_builder.build();
    if (!base) return detail::thumbnail_refusal(item, base.error().context);
    Result<uint64_t> hash = scene::world_hash(*base);
    if (!hash) return detail::thumbnail_refusal(item, hash.error().context);

    scene::SceneDesc desc;
    desc.name = "thumbnail";
    desc.world.file = "thumbnail.world.yaml";
    desc.world.hash = *hash;
    if (item.kind == LibraryItem::Kind::asset) {
        desc.materials = item.materials;
        scene::SceneAsset a = *item.asset;
        a.name = item.name.empty() ? std::string("thumbnail") : item.name;
        desc.assets.push_back(std::move(a));
    } else if (part) {
        desc.assets.push_back(std::move(*part));
    }
    Result<scene::ComposedScene> composed = scene::compose(desc, *base);
    if (!composed) return detail::thumbnail_refusal(item, composed.error().context);

    const render::ResolvedMeshes meshes = render::resolve_meshes(composed->world, mesh_roots);
    if (!meshes.unresolved.empty()) {
        return detail::thumbnail_refusal(item, "mesh '" + meshes.unresolved.front().ref +
                                                   "' resolves nowhere: " + meshes.unresolved.front().why);
    }
    Result<render::RenderScene> frame = render::scene_from_world(composed->world, meshes.meshes);
    if (!frame) return detail::thumbnail_refusal(item, frame.error().context);

    if (item.kind == LibraryItem::Kind::model) {
        if (item.model->parts.empty()) return detail::thumbnail_refusal(item, "a model without parts");
        Result<render_parts::PreparedParts> prepared = render_parts::prepare_parts(*item.model);
        if (!prepared) return detail::thumbnail_refusal(item, prepared.error().context);
        render_parts::PartsLook look;
        look.body = static_cast<uint32_t>(frame->materials.size());
        frame->materials.push_back(sandbox::detail::lambert(0.35f, 0.37f, 0.40f));
        look.rotor = static_cast<uint32_t>(frame->materials.size());
        frame->materials.push_back(sandbox::detail::lambert(0.12f, 0.12f, 0.14f));
        look.porous = look.body;
        look.object_name = item.model->name;
        if (Result<void> bound = append_vehicle_parts(*frame, *prepared, *item.model, glm::vec3(0.0f),
                                                      glm::quat(1.0f, 0.0f, 0.0f, 0.0f), look);
            !bound) {
            return detail::thumbnail_refusal(item, bound.error().context);
        }
    }

    const std::optional<render::Aabb> box = detail::drawn_bounds(*frame);
    if (!box) return detail::thumbnail_refusal(item, "it draws nothing");
    const glm::vec3 centre = 0.5f * (box->min + box->max);
    const float radius = std::max(0.5f * glm::length(box->max - box->min), 1e-3f);
    const glm::vec3 toward(std::sin(detail::kThumbnailYaw) * std::cos(detail::kThumbnailPitch),
                           std::sin(detail::kThumbnailPitch),
                           std::cos(detail::kThumbnailYaw) * std::cos(detail::kThumbnailPitch));
    const float distance = radius / std::sin(0.5f * detail::kThumbnailFov);
    render::Camera camera;
    camera.position = centre + toward * distance;
    camera.orientation = glm::quatLookAt(-toward, glm::vec3(0.0f, 1.0f, 0.0f));
    camera.fov_y_radians = detail::kThumbnailFov;
    camera.near_plane = std::max(distance - 2.0f * radius, 1e-3f);
    camera.far_plane = distance + 2.0f * radius;

    render::RenderOptions options;
    options.overlays = false;
    options.ground_grid = false;
    options.spawn_markers = false;
    std::vector<uint8_t> bgrx(static_cast<std::size_t>(size) * size * 4u);
    render::RenderTarget target;
    target.pixels = bgrx;
    target.width = size;
    target.height = size;
    target.stride = size * 4u;
    if (Result<void> drawn = render::render(*frame, camera, options, target); !drawn) {
        return detail::thumbnail_refusal(item, drawn.error().context);
    }

    png::Image image;
    image.width = size;
    image.height = size;
    image.rgba.resize(bgrx.size());
    for (std::size_t i = 0; i < bgrx.size(); i += 4) {
        image.rgba[i] = bgrx[i + 2];
        image.rgba[i + 1] = bgrx[i + 1];
        image.rgba[i + 2] = bgrx[i];
        image.rgba[i + 3] = 255u;
    }
    return image;
}

}  // namespace spade::sandbox::editor
