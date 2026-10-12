// The editor's library: the built-in items, and placing a copy of one into a
// scene. Display-free (SL15b).
//
// The built-in items ship with the editor: the quadrotor template (the
// sandbox stand's airframe), and the primitive assets -- box, sphere, cylinder
// and capsule -- at unit size, as the builder makes them. Library files, and
// Save to library, come with Core's model and asset files (Task 13).
//
// Placing copies (EDT-012): a model goes in once per placed vehicle, under its
// own unique name, with the vehicle that flies it; an asset goes in as an
// asset. The scene holds the copy itself and never refers to the library, so
// it stays whole when the library changes. A placement is one edit, so one
// undo takes it back.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §4;
// 2026-10-05-editor-plan.md Task 9.

#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "drone_sim.hpp"
#include "editor_document.hpp"
#include "editor_scene_edits.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

struct LibraryItem {
    enum class Kind { model, asset };
    Kind kind = Kind::asset;
    std::string name;
    std::optional<vehicles::ModelType> model;  // set for a model
    std::optional<scene::SceneAsset> asset;    // set for an asset
};

// One primitive at its collider's origin, as WorldBuilder lays out its
// parameters (sdf.hpp's packing): the library's primitives, and the edit
// text's short form for them.
[[nodiscard]] inline scene::SceneAsset primitive_asset(std::string name, SdfPrim kind, glm::vec4 params,
                                                       const SdfPose& pose) {
    scene::SceneAsset a;
    a.name = std::move(name);
    a.pose = pose;
    a.collider.transforms.push_back(SdfTransform{});
    SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(SdfOp::none);
    n.params = params;
    a.collider.nodes.push_back(n);
    return a;
}

// The kind a vehicle's model makes it, for the library panel's grouping. The
// kind comes from the model, so a new kind needs no change to the panel
// (editor spec §4). ModelType carries no kind of its own yet, so it is read
// from the rotors: four make a quadrotor, other counts a multirotor, and none
// a plain vehicle. Drones are the only kind today.
[[nodiscard]] inline std::string vehicle_kind_of(const vehicles::ModelType& model) {
    if (model.rotors.size() == vehicles::kQuadrotorRotorCount) return "quadrotor";
    if (!model.rotors.empty()) return "multirotor";
    return "vehicle";
}

// The built-in items: the quadrotor, then the box, sphere, cylinder and
// capsule. The quadrotor is make_quadrotor() over the stand's airframe, which
// the sandbox flies every run; the library tests check it is present and valid.
[[nodiscard]] inline std::vector<LibraryItem> builtin_library() {
    std::vector<LibraryItem> out;
    vehicles::QuadrotorParams params = drone_stand_params();
    params.name = "quadrotor";
    if (const Result<vehicles::ModelType> quad = vehicles::make_quadrotor(params)) {
        out.push_back(LibraryItem{LibraryItem::Kind::model, "quadrotor", *quad, std::nullopt});
    }
    const auto primitive = [&](std::string name, SdfPrim kind, glm::vec4 p) {
        scene::SceneAsset a = primitive_asset(name, kind, p, SdfPose{});
        out.push_back(LibraryItem{LibraryItem::Kind::asset, std::move(name), std::nullopt, std::move(a)});
    };
    // Unit size, as the builder's: a 1 m cube, a 1 m ball, a cylinder and a
    // capsule 1 m across and 1 m tall.
    primitive("box", SdfPrim::box, glm::vec4(0.5f, 0.5f, 0.5f, 0.0f));
    primitive("sphere", SdfPrim::sphere, glm::vec4(0.5f, 0.0f, 0.0f, 0.0f));
    primitive("cylinder", SdfPrim::cylinder, glm::vec4(0.5f, 0.5f, 0.0f, 0.0f));
    primitive("capsule", SdfPrim::capsule, glm::vec4(0.25f, 0.25f, 0.0f, 0.0f));
    return out;
}

// ---- the scene picker (SL13; editor plan Task 11) ----

// A scene the editor can open: a built-in scene file (assets/scenes/), a
// test scenario once composer task 5 converts them to scene files (none
// until then), or a scene the user saved.
struct SceneSource {
    enum class Kind { builtin, test_scenario, saved };
    Kind kind = Kind::builtin;
    std::string name;  // the file's name without ".scene.yaml"
    std::filesystem::path file;
};

namespace detail {

// The *.scene.yaml files directly in `dir`, by name; none for a folder that
// does not exist (a saved folder is made by the first save).
inline void add_scene_files(std::vector<SceneSource>& out, const std::filesystem::path& dir, SceneSource::Kind kind) {
    constexpr std::string_view kSuffix = ".scene.yaml";
    std::vector<SceneSource> found;
    std::error_code ec;
    for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string file = it->path().filename().string();
        if (!it->is_regular_file(ec) || !file.ends_with(kSuffix)) continue;
        found.push_back(SceneSource{kind, file.substr(0, file.size() - kSuffix.size()), it->path()});
    }
    std::sort(found.begin(), found.end(), [](const SceneSource& a, const SceneSource& b) { return a.name < b.name; });
    out.insert(out.end(), found.begin(), found.end());
}

}  // namespace detail

// The picker's list: the built-in scenes of `assets`/scenes/, then the
// scenes saved in `saved_dir`, each by name.
[[nodiscard]] inline std::vector<SceneSource> scene_sources(const std::filesystem::path& assets,
                                                            const std::filesystem::path& saved_dir) {
    std::vector<SceneSource> out;
    detail::add_scene_files(out, assets / "scenes", SceneSource::Kind::builtin);
    if (!saved_dir.empty()) detail::add_scene_files(out, saved_dir, SceneSource::Kind::saved);
    return out;
}

// Places a copy of `item` at `at` and returns the name it was given, made
// unique in the scene. A model is placed as a vehicle at rest at the pose,
// flying its own copy of the model; a vehicle has no scale, so a pose whose
// scale is not 1 is refused rather than dropped (L6). An asset takes the whole
// pose. Nothing changes on a refusal.
[[nodiscard]] inline Result<std::string> place(SceneDocument& doc, const LibraryItem& item, const SdfPose& at) {
    if (item.kind == LibraryItem::Kind::asset) {
        if (!item.asset) {
            return std::unexpected(
                Error{Code::invalid_argument, "place '" + item.name + "': the library item has no asset"});
        }
        scene::SceneAsset a = *item.asset;
        a.pose = at;
        if (const Result<void> r = apply(doc, AddAsset{std::move(a)}); !r) return std::unexpected(r.error());
        return doc.desc().assets.back().name;
    }
    if (!item.model) {
        return std::unexpected(
            Error{Code::invalid_argument, "place '" + item.name + "': the library item has no model"});
    }
    if (at.scale != 1.0f) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "place '" + item.name + "' at scale " + std::to_string(at.scale) +
                                         ": a vehicle has no scale; place it at scale 1"});
    }
    scene::SceneVehicle v;
    v.name = item.name;
    v.start.pos = at.position;
    v.start.orient = at.rotation;
    if (const Result<void> r = apply(doc, AddVehicle{std::move(v), *item.model}); !r) {
        return std::unexpected(r.error());
    }
    return doc.desc().vehicles.back().name;
}

}  // namespace spade::sandbox::editor
