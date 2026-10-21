// The editor's library: the default library's items, a file's, the built-in
// ones, and placing a copy of one into a scene. Display-free (SL15b).
//
// THE PANEL lists Core's default library (assets/library/library.yaml,
// scene::load_library()) in its index's order, grouped by category (INT-14).
// An entry that won't load is named, and the rest still load. Load from file
// reads a *.model.yaml or *.object.yaml by its extension, as the index does.
// The built-in items -- the quadrotor template (the sandbox stand's airframe)
// and the unit primitives, as the builder makes them -- left the panel; the
// tests keep them.
//
// A FILE'S ITEM is placed through Core (TD-9): AddModelFile and AddObjectFile
// read its file and go through scene::add_model(), which keeps the chain, and
// scene::add_object(), which adds the materials. A built-in is placed as it
// is, by AddVehicle and AddAsset.
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
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "drone_sim.hpp"
#include "drone_view.hpp"
#include "editor_document.hpp"
#include "editor_scene_edits.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/aabb.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

struct LibraryItem {
    enum class Kind { model, asset };
    Kind kind = Kind::asset;
    std::string name;                          // what the panel shows
    std::optional<vehicles::ModelType> model;  // set for a model: a model file's, its chain included
    std::optional<scene::SceneAsset> asset;    // set for an asset: an object file's object
    std::vector<MaterialDesc> materials{};     // an object file's own materials
    std::string category{};                    // the index's, or "from file"; empty for a built-in
    std::string file{};                        // as the index spells it, or the path loaded from
    std::string source{};                      // the path placing reads; empty for a built-in
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

// The panel's group for an item: its category, else a built-in's kind (a
// vehicle's, by vehicle_kind_of()) or "assets".
[[nodiscard]] inline std::string panel_category(const LibraryItem& item) {
    if (!item.category.empty()) return item.category;
    if (item.kind == LibraryItem::Kind::model && item.model) return vehicle_kind_of(*item.model);
    return "assets";
}

// A library's items in its index's order, and what would not load, verbatim:
// the index (then there are no items), or each entry that won't (Core's
// refusal names it: "library entry '<name>': ...").
struct LibraryLoad {
    std::vector<LibraryItem> items;
    std::vector<std::string> problems;
};

namespace detail {

[[nodiscard]] inline LibraryItem model_item(const scene::ModelFile& f) {
    LibraryItem item;
    item.kind = LibraryItem::Kind::model;
    item.model = f.model;
    return item;
}

[[nodiscard]] inline LibraryItem object_item(const scene::ObjectFile& f) {
    LibraryItem item;
    item.kind = LibraryItem::Kind::asset;
    item.asset = f.object;
    item.materials = f.materials;
    return item;
}

// The lowest point of `model`'s parts, its body at the origin, level and
// still: each part's box (part_bounds()) through its posed frame. None when
// no part can be posed and bounded.
[[nodiscard]] inline std::optional<float> lowest_part_point(const vehicles::ModelType& model) {
    BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<float> still(model.rotors.size(), 0.0f);
    const Result<vehicles::PosedParts> posed =
        vehicles::posed_parts(model, body, still, still, vehicles::PartFilter::all);
    if (!posed) return std::nullopt;
    std::optional<float> low;
    for (const vehicles::PosedPart& p : posed->parts) {
        const Result<Aabb> box = vehicles::part_bounds(model.parts[p.part]);
        if (!box) continue;
        for (uint32_t c = 0; c < 8; ++c) {
            const glm::vec3 corner((c & 1u) != 0 ? box->max.x : box->min.x, (c & 2u) != 0 ? box->max.y : box->min.y,
                                   (c & 4u) != 0 ? box->max.z : box->min.z);
            const float y = (p.origin + p.orient * corner).y;
            low = low ? std::min(*low, y) : y;
        }
    }
    return low;
}

}  // namespace detail

// Every entry of the index at `index`, loaded (load_library_model() or
// load_library_object()).
[[nodiscard]] inline LibraryLoad load_library_items(const std::filesystem::path& index) {
    LibraryLoad out;
    const Result<scene::Library> library = scene::load_library(index);
    if (!library) {
        out.problems.push_back(library.error().context);
        return out;
    }
    for (const scene::LibraryEntry& e : library->entries) {
        LibraryItem item;
        if (e.kind == scene::LibraryKind::model) {
            const Result<scene::ModelFile> f = scene::load_library_model(*library, e);
            if (!f) {
                out.problems.push_back(f.error().context);
                continue;
            }
            item = detail::model_item(*f);
        } else {
            const Result<scene::ObjectFile> f = scene::load_library_object(*library, e);
            if (!f) {
                out.problems.push_back(f.error().context);
                continue;
            }
            item = detail::object_item(*f);
        }
        item.name = e.name;
        item.category = e.category;
        item.file = e.file;
        item.source = (library->dir / e.file).generic_string();
        out.items.push_back(std::move(item));
    }
    return out;
}

// Load from file: a model or object file, by its extension as the index
// reads one, as an item in the "from file" category, named as the file's
// model or object is.
[[nodiscard]] inline Result<LibraryItem> item_from_file(const std::filesystem::path& path) {
    const std::string file = path.generic_string();
    const std::string leaf = path.filename().generic_string();
    const auto ends = [&](std::string_view suffix) { return leaf.size() > suffix.size() && leaf.ends_with(suffix); };
    LibraryItem item;
    if (ends(".model.yaml")) {
        const Result<scene::ModelFile> f = scene::load_model_file(path);
        if (!f) return std::unexpected(f.error());
        item = detail::model_item(*f);
        item.name = f->model.name;
    } else if (ends(".object.yaml")) {
        const Result<scene::ObjectFile> f = scene::load_object_file(path);
        if (!f) return std::unexpected(f.error());
        item = detail::object_item(*f);
        item.name = f->object.name;
    } else {
        return std::unexpected(Error{Code::invalid_argument,
                                     "load '" + file + "': a library file is a *.model.yaml (a drone) or a "
                                     "*.object.yaml (anything else)"});
    }
    item.category = "from file";
    item.file = file;
    item.source = file;
    return item;
}

// How far above the ground an item's origin sits when it rests there
// (Test/Docs' placed-at): a drone on its own lowest posed part, or on the
// built-in drone's body when it has no parts; an asset on its collider's box
// (sdf_bounds()), or at its origin when the box can't be found.
[[nodiscard]] inline float rest_height(const LibraryItem& item) {
    if (item.kind == LibraryItem::Kind::model) {
        if (!item.model || item.model->parts.empty()) return kDroneBodyHalf.y;
        const std::optional<float> low = detail::lowest_part_point(*item.model);
        return low ? -*low : kDroneBodyHalf.y;
    }
    if (!item.asset) return 0.0f;
    const Result<Aabb> box = sdf_bounds(item.asset->collider);
    return box ? -box->min.y : 0.0f;
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
// pose. A file's item goes through Core (AddModelFile, AddObjectFile), named
// as its file's model or object is. Nothing changes on a refusal.
[[nodiscard]] inline Result<std::string> place(SceneDocument& doc, const LibraryItem& item, const SdfPose& at) {
    if (item.kind == LibraryItem::Kind::asset) {
        if (!item.asset) {
            return std::unexpected(
                Error{Code::invalid_argument, "place '" + item.name + "': the library item has no asset"});
        }
        if (!item.source.empty()) {
            if (const Result<void> r = apply(doc, AddObjectFile{item.source, item.asset->name, at}); !r) {
                return std::unexpected(r.error());
            }
            return doc.desc().assets.back().name;
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
    v.start.pos = at.position;
    v.start.orient = at.rotation;
    if (!item.source.empty()) {
        if (const Result<void> r = apply(doc, AddModelFile{item.source, v}); !r) return std::unexpected(r.error());
        return doc.desc().vehicles.back().name;
    }
    v.name = item.name;
    if (const Result<void> r = apply(doc, AddVehicle{std::move(v), *item.model}); !r) {
        return std::unexpected(r.error());
    }
    return doc.desc().vehicles.back().name;
}

}  // namespace spade::sandbox::editor
