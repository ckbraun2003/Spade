// The editor's scene edits. Display-free (SL15b).
//
// Each edit changes a copy of the scene and commits it through the document,
// so validate_scene() judges it (EDT-003) and it can be undone (EDT-004).
// Objects are addressed by name, never index (EDT-008): an asset and a vehicle
// share one namespace (SCN-004), models and materials each have their own. A
// vehicle the editor adds gets its own copy of its model, so an edit to one
// vehicle's model never reaches another (the user's Q1, EDT-019).
//
// A model or object FILE goes in through Core's one function for it (TD-9):
// scene::add_model(), which keeps the model's chain, and scene::add_object(),
// which adds the object's materials. Core never renames, so the editor makes
// the names unique first. The edit carries the file's path, the form its text
// has (EDT-002), and reads the file when it is applied; undo restores the
// scene, never the file (INT-14).
//
// docs/design/interface/plans/2026-10-05-editor-design.md §3-§4;
// 2026-10-05-editor-plan.md Tasks 2 and 3.

#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "editor_document.hpp"
#include "editor_import.hpp"
#include "editor_model_params.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"

namespace spade::sandbox::editor {

struct AddAsset {
    scene::SceneAsset asset;  // its name is made unique
};
struct AddVehicle {
    scene::SceneVehicle vehicle;  // its name is made unique
    vehicles::ModelType model;    // copied for this vehicle alone, under a unique model name
};
// A model file, placed as a vehicle: the file's model goes in under its own
// name made unique, and the vehicle (its name made unique too, the model's
// when empty) flies it.
struct AddModelFile {
    std::string file;             // the model file's path, as given
    scene::SceneVehicle vehicle;  // `model` is set to the copy's name
};
// An object file, placed as an asset at `pose`, under `name` made unique
// (the object's own when empty).
struct AddObjectFile {
    std::string file;  // the object file's path, as given
    std::string name;
    SdfPose pose;
};
struct RemoveObject {
    std::string name;  // an asset or a vehicle; a vehicle's model goes too when nothing else uses it
};
struct RenameObject {
    std::string from;
    std::string to;
};
struct DuplicateObject {
    std::string name;  // the copy goes at the end of its list, as "<name>_<n>", n the lowest free
};
struct MoveObject {
    std::string name;
    std::size_t to;  // its new place in its own list (SCN-005)
};
struct SetAssetPose {
    std::string asset;
    SdfPose pose;
};
struct SetAssetMaterial {
    std::string asset;
    std::string material;  // the visual's material, by name
};
// The collider's material: what an asset without a visual draws with (the
// composed CSG shades each node by its material). Every node takes it; an
// empty name puts them back on the palette's default.
struct SetColliderMaterial {
    std::string asset;
    std::string material;  // by name, or empty for the default
};
struct SetVehicleStart {
    std::string vehicle;
    VehicleSpawn start;  // in the model's design frame (DBE-013)
};
struct AddMaterial {
    MaterialDesc material;
};
struct RepointWorld {
    std::string file;  // relative to the scene file (SCN-001)
    uint64_t hash = 0;
};
// One parameter of the model a vehicle flies (editor_model_params.hpp). It
// reaches that vehicle alone: a model another vehicle shares is copied for it
// first (EDT-019). A value ModelType::issues() objects to is refused, naming
// each problem's part and field (EDT-014).
struct SetModelParam {
    std::string vehicle;
    vehicles::ModelIssue::Element element = vehicles::ModelIssue::Element::model;
    std::size_t index = 0;
    std::string field;
    ParamValue value;
};
// N15: meshes staged from a glTF (editor_import.hpp's import_gltf_parts()),
// added to the model a vehicle flies as imported_part() makes them, the model
// copied first as SetModelParam's is. A mesh the edit doesn't hold (its text
// form carries none) is read from its sidecar. A name the model's parts
// already have is refused.
struct AddMeshParts {
    std::string vehicle;
    std::vector<ImportedMesh> meshes;
};
// N15: what an imported part's rows edit (D4): drawn only or a solid, its
// declared smallest feature (m), and its mount's turn. An authored part is
// refused.
struct SetImportedPart {
    std::string vehicle;
    std::string part;
    bool visual_only = false;
    float min_feature = 0.0f;
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};
};

// The editor's airflow run (INT-15, T1): a one-way follow region around a
// drone, named <vehicle>_air, with the plan's §4 defaults (kAirCells^3
// cubic cells, dx its largest rotor's radius over 3 floored to the
// millimetre, scene v5's fluid); one region per drone (M2).
struct AddAirflowRegion {
    std::string vehicle;
};
struct RemoveAirflowRegion {
    std::string name;
};

inline constexpr uint32_t kAirCells = 32;  // §4: the live grid the tour flies

using SceneEdit = std::variant<AddAsset, AddVehicle, RemoveObject, RenameObject, DuplicateObject, MoveObject,
                               SetAssetPose, SetAssetMaterial, SetColliderMaterial, SetVehicleStart, AddMaterial,
                               RepointWorld, SetModelParam, AddModelFile, AddObjectFile, AddMeshParts,
                               SetImportedPart, AddAirflowRegion, RemoveAirflowRegion>;

namespace detail {

[[nodiscard]] inline bool object_taken(const scene::SceneDesc& s, std::string_view name) {
    return std::any_of(s.assets.begin(), s.assets.end(), [&](const auto& a) { return a.name == name; }) ||
           std::any_of(s.vehicles.begin(), s.vehicles.end(), [&](const auto& v) { return v.name == name; });
}

[[nodiscard]] inline bool model_taken(const scene::SceneDesc& s, std::string_view name) {
    return std::any_of(s.models.begin(), s.models.end(), [&](const auto& m) { return m.name == name; });
}

// `base` if free, else "<base>_<n>" for the lowest n >= 2 that is free.
template <class Taken>
[[nodiscard]] std::string lowest_free(std::string_view base, const Taken& taken) {
    if (!taken(base)) return std::string(base);
    for (uint64_t n = 2;; ++n) {
        std::string candidate = std::string(base) + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

[[nodiscard]] inline Error no_object(std::string_view what, std::string_view name) {
    return Error{Code::invalid_argument, std::string(what) + ": no asset or vehicle is named '" + std::string(name) +
                                             "' in this scene; check the name in the hierarchy"};
}

template <class T>
[[nodiscard]] T* find_named(std::vector<T>& list, std::string_view name) {
    for (T& item : list) {
        if (item.name == name) return &item;
    }
    return nullptr;
}

template <class T>
[[nodiscard]] Result<void> move_within(std::vector<T>& list, std::string_view name, std::size_t to) {
    const auto it = std::find_if(list.begin(), list.end(), [&](const T& t) { return t.name == name; });
    if (to >= list.size()) {
        return std::unexpected(Error{Code::invalid_argument, "move '" + std::string(name) + "' to place " +
                                                                 std::to_string(to) + ": its list holds " +
                                                                 std::to_string(list.size()) + ", so the last place is " +
                                                                 std::to_string(list.size() - 1)});
    }
    T item = std::move(*it);
    list.erase(it);
    list.insert(list.begin() + static_cast<std::ptrdiff_t>(to), std::move(item));
    return {};
}

// Each change edits `s` in place and says whether anything changed; the caller
// commits only a change.
inline Result<bool> change(scene::SceneDesc& s, const AddAsset& e) {
    scene::SceneAsset a = e.asset;
    a.name = lowest_free(a.name.empty() ? "asset" : a.name, [&](std::string_view n) { return object_taken(s, n); });
    s.assets.push_back(std::move(a));
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const AddVehicle& e) {
    vehicles::ModelType m = e.model;
    m.name = lowest_free(m.name.empty() ? "model" : m.name, [&](std::string_view n) { return model_taken(s, n); });
    scene::SceneVehicle v = e.vehicle;
    v.name = lowest_free(v.name.empty() ? "vehicle" : v.name, [&](std::string_view n) { return object_taken(s, n); });
    v.model = m.name;
    s.models.push_back(std::move(m));
    s.vehicles.push_back(std::move(v));
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const AddModelFile& e) {
    const Result<scene::ModelFile> file = scene::load_model_file(e.file);
    if (!file) return std::unexpected(Error{file.error().code, "add-model-file: " + file.error().context});
    const std::string model = lowest_free(file->model.name.empty() ? "model" : file->model.name,
                                          [&](std::string_view n) { return model_taken(s, n); });
    if (const Result<void> added = scene::add_model(s, *file, model); !added) return std::unexpected(added.error());
    scene::SceneVehicle v = e.vehicle;
    v.name = lowest_free(v.name.empty() ? model : v.name, [&](std::string_view n) { return object_taken(s, n); });
    v.model = model;
    s.vehicles.push_back(std::move(v));
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const AddObjectFile& e) {
    const Result<scene::ObjectFile> file = scene::load_object_file(e.file);
    if (!file) return std::unexpected(Error{file.error().code, "add-object-file: " + file.error().context});
    const std::string& own = e.name.empty() ? file->object.name : e.name;
    const std::string name =
        lowest_free(own.empty() ? "object" : own, [&](std::string_view n) { return object_taken(s, n); });
    if (const Result<void> added = scene::add_object(s, *file, name, e.pose); !added) {
        return std::unexpected(added.error());
    }
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const RemoveObject& e) {
    if (const auto it = std::find_if(s.assets.begin(), s.assets.end(), [&](const auto& a) { return a.name == e.name; });
        it != s.assets.end()) {
        s.assets.erase(it);
        return true;
    }
    const auto it = std::find_if(s.vehicles.begin(), s.vehicles.end(), [&](const auto& v) { return v.name == e.name; });
    if (it == s.vehicles.end()) return std::unexpected(no_object("remove", e.name));
    // A region follows its vehicle by name (N5b): not removed from under it.
    for (const scene::SceneAirflowRegion& r : s.airflow) {
        if (r.follows == e.name) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "remove '" + e.name + "': airflow region '" + r.name +
                                             "' follows it; remove the region or point it at another vehicle first"});
        }
    }
    const std::string model = it->model;
    s.vehicles.erase(it);
    const bool still_used =
        std::any_of(s.vehicles.begin(), s.vehicles.end(), [&](const auto& v) { return v.model == model; });
    if (!still_used) {
        std::erase_if(s.models, [&](const auto& m) { return m.name == model; });
    }
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const AddAirflowRegion& e) {
    const scene::SceneVehicle* v = find_named(s.vehicles, e.vehicle);
    if (v == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "add an airflow region: no vehicle is named '" + e.vehicle +
                                                                 "' in this scene; check the name in the hierarchy"});
    }
    for (const scene::SceneAirflowRegion& r : s.airflow) {
        if (r.kind == scene::AirflowKind::follow && r.follows == e.vehicle) {
            return std::unexpected(Error{Code::invalid_argument, "add an airflow region around '" + e.vehicle +
                                                                     "': airflow region '" + r.name +
                                                                     "' already follows it; one region holds one drone"});
        }
    }
    float radius = 0.0f;
    if (const vehicles::ModelType* m = find_named(s.models, v->model)) {
        for (const vehicles::RotorDesc& rotor : m->rotors) radius = std::max(radius, rotor.radius);
    }
    if (!(radius > 0.0f)) {
        return std::unexpected(Error{Code::invalid_argument, "add an airflow region around '" + e.vehicle +
                                                                 "': its model '" + v->model +
                                                                 "' has no rotors, so there is no air to drive"});
    }
    scene::SceneAirflowRegion r;
    r.name = lowest_free(e.vehicle + "_air", [&](std::string_view n) {
        return std::any_of(s.airflow.begin(), s.airflow.end(), [&](const auto& x) { return x.name == n; });
    });
    r.kind = scene::AirflowKind::follow;
    r.follows = e.vehicle;
    r.cells = physics::airflow::GridShape{kAirCells, kAirCells, kAirCells, std::floor(radius / 3.0f * 1000.0f) / 1000.0f};
    r.density = 1.225f;
    r.kinematic_viscosity = 0.0f;
    r.coupling = scene::AirflowCoupling::one_way;
    s.airflow.push_back(std::move(r));
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const RemoveAirflowRegion& e) {
    const auto it = std::find_if(s.airflow.begin(), s.airflow.end(), [&](const auto& r) { return r.name == e.name; });
    if (it == s.airflow.end()) {
        return std::unexpected(Error{Code::invalid_argument, "remove an airflow region: no airflow region is named '" +
                                                                 e.name + "' in this scene"});
    }
    s.airflow.erase(it);
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const RenameObject& e) {
    if (!object_taken(s, e.from)) return std::unexpected(no_object("rename", e.from));
    if (e.from == e.to) return false;
    if (e.to.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "rename '" + e.from + "': a name cannot be empty"});
    }
    if (object_taken(s, e.to)) {
        return std::unexpected(Error{Code::invalid_argument, "rename '" + e.from + "' to '" + e.to + "': '" + e.to +
                                                                 "' is already an asset or vehicle in this scene; "
                                                                 "pick another name"});
    }
    if (scene::SceneAsset* a = find_named(s.assets, e.from)) {
        a->name = e.to;
    } else {
        find_named(s.vehicles, e.from)->name = e.to;
        // The regions that follow it (N5b) follow it under its new name.
        for (scene::SceneAirflowRegion& r : s.airflow) {
            if (r.follows == e.from) r.follows = e.to;
        }
    }
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const DuplicateObject& e) {
    if (const scene::SceneAsset* a = find_named(s.assets, e.name)) {
        scene::SceneAsset copy = *a;
        copy.name = lowest_free(e.name, [&](std::string_view n) { return object_taken(s, n); });
        s.assets.push_back(std::move(copy));
        return true;
    }
    const scene::SceneVehicle* v = find_named(s.vehicles, e.name);
    if (v == nullptr) return std::unexpected(no_object("duplicate", e.name));
    const vehicles::ModelType* m = find_named(s.models, v->model);
    if (m == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "duplicate '" + e.name + "': its model '" + v->model +
                                                                 "' is not in this scene"});
    }
    const AddVehicle add{*v, *m};
    return change(s, add);
}

inline Result<bool> change(scene::SceneDesc& s, const MoveObject& e) {
    if (find_named(s.assets, e.name) != nullptr) {
        if (Result<void> moved = move_within(s.assets, e.name, e.to); !moved) return std::unexpected(moved.error());
        return true;
    }
    if (find_named(s.vehicles, e.name) == nullptr) return std::unexpected(no_object("move", e.name));
    if (Result<void> moved = move_within(s.vehicles, e.name, e.to); !moved) return std::unexpected(moved.error());
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const SetAssetPose& e) {
    scene::SceneAsset* a = find_named(s.assets, e.asset);
    if (a == nullptr) return std::unexpected(no_object("set the pose", e.asset));
    a->pose = e.pose;
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const SetAssetMaterial& e) {
    scene::SceneAsset* a = find_named(s.assets, e.asset);
    if (a == nullptr) return std::unexpected(no_object("set the material", e.asset));
    if (a->visual.mesh_ref.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "set the material of '" + e.asset +
                                                                 "': it has no visual, and a material belongs to a "
                                                                 "visual; give it a mesh first"});
    }
    a->visual.material = e.material;
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const SetColliderMaterial& e) {
    scene::SceneAsset* a = find_named(s.assets, e.asset);
    if (a == nullptr) return std::unexpected(no_object("set the collider material", e.asset));
    if (a->collider.nodes.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "set the collider material of '" + e.asset +
                                                                 "': it has no collider to colour"});
    }
    if (e.material.empty()) {
        a->collider_materials.clear();
    } else {
        a->collider_materials.assign(a->collider.nodes.size(), e.material);
    }
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const SetVehicleStart& e) {
    scene::SceneVehicle* v = find_named(s.vehicles, e.vehicle);
    if (v == nullptr) return std::unexpected(no_object("set the start", e.vehicle));
    v->start = e.start;
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const AddMaterial& e) {
    s.materials.push_back(e.material);
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const RepointWorld& e) {
    s.world.file = e.file;
    s.world.hash = e.hash;
    return true;
}

// Bit for bit, so -0 and 0 differ and a NaN equals itself: "no change" must
// mean the saved file would not change.
[[nodiscard]] inline bool same_value(const ParamValue& a, const ParamValue& b) {
    if (a.index() != b.index()) return false;
    const auto bits = [](float f) { return std::bit_cast<uint32_t>(f); };
    if (const float* x = std::get_if<float>(&a)) return bits(*x) == bits(std::get<float>(b));
    if (const glm::vec3* x = std::get_if<glm::vec3>(&a)) {
        const glm::vec3& y = std::get<glm::vec3>(b);
        return bits(x->x) == bits(y.x) && bits(x->y) == bits(y.y) && bits(x->z) == bits(y.z);
    }
    if (const glm::quat* x = std::get_if<glm::quat>(&a)) {
        const glm::quat& y = std::get<glm::quat>(b);
        return bits(x->w) == bits(y.w) && bits(x->x) == bits(y.x) && bits(x->y) == bits(y.y) && bits(x->z) == bits(y.z);
    }
    return a == b;  // uint32_t, std::string
}

// Puts `edited` in for the model `v` flies, if ModelType::issues() has no
// objection (EDT-014): in place when no other vehicle flies it, else as this
// vehicle's own copy, the others keeping theirs (copy-on-first-edit, EDT-019).
// `refused` ends the refusal.
inline Result<bool> commit_model(scene::SceneDesc& s, scene::SceneVehicle& v, vehicles::ModelType edited,
                                 std::string_view refused) {
    if (const std::string problems = describe_issues(edited); !problems.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "model '" + edited.name + "' of '" + v.name +
                                                                 "' would not be valid: " + problems + "; " +
                                                                 std::string(refused)});
    }
    const std::string model = v.model;
    const bool shared = std::any_of(s.vehicles.begin(), s.vehicles.end(),
                                    [&](const auto& other) { return other.name != v.name && other.model == model; });
    vehicles::ModelType* m = find_named(s.models, model);
    if (!shared && m != nullptr) {
        *m = std::move(edited);
        return true;
    }
    edited.name = lowest_free(model, [&](std::string_view n) { return model_taken(s, n); });
    v.model = edited.name;
    s.models.push_back(std::move(edited));  // pointers into s.models may dangle from here
    return true;
}

inline Result<bool> change(scene::SceneDesc& s, const SetModelParam& e) {
    scene::SceneVehicle* v = find_named(s.vehicles, e.vehicle);
    if (v == nullptr) return std::unexpected(no_object("set a model parameter", e.vehicle));
    vehicles::ModelType* m = find_named(s.models, v->model);
    if (m == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "set a model parameter of '" + e.vehicle +
                                                                 "': its model '" + v->model + "' is not in this scene"});
    }
    const Result<ParamValue> now = get_model_param(*m, e.element, e.index, e.field);
    if (!now) return std::unexpected(now.error());
    if (same_value(*now, e.value)) return false;

    vehicles::ModelType edited = *m;
    if (Result<void> set = set_model_param(edited, e.element, e.index, e.field, e.value); !set) {
        return std::unexpected(set.error());
    }
    return commit_model(s, *v, std::move(edited), "the edit was not made");
}

// The vehicle `e` names and the model it flies, or why not, for `what`.
template <class Edit>
[[nodiscard]] Result<std::pair<scene::SceneVehicle*, vehicles::ModelType*>> flown(scene::SceneDesc& s, const Edit& e,
                                                                                 std::string_view what) {
    scene::SceneVehicle* v = find_named(s.vehicles, e.vehicle);
    if (v == nullptr) return std::unexpected(no_object(what, e.vehicle));
    vehicles::ModelType* m = find_named(s.models, v->model);
    if (m == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, std::string(what) + " '" + e.vehicle + "': its model '" +
                                                                 v->model + "' is not in this scene"});
    }
    return std::pair{v, m};
}

inline Result<bool> change(scene::SceneDesc& s, const AddMeshParts& e) {
    const auto at = flown(s, e, "import parts onto");
    if (!at) return std::unexpected(at.error());
    const auto [v, m] = *at;
    if (e.meshes.empty()) return false;
    vehicles::ModelType edited = *m;
    for (const ImportedMesh& staged : e.meshes) {
        if (find_named(edited.parts, staged.name) != nullptr) {
            return std::unexpected(Error{Code::invalid_argument, "import parts onto '" + e.vehicle + "': model '" +
                                                                     m->name + "' has a part '" + staged.name +
                                                                     "' already; the parts were not added"});
        }
        ImportedMesh held = staged;
        if (!held.mesh) {
            Result<LoadedMesh> read = load_mesh_file(held.sidecar);
            if (!read) {
                return std::unexpected(Error{read.error().code, "import parts onto '" + e.vehicle + "': part '" +
                                                                    held.name + "''s mesh could not be read from " +
                                                                    path_text(held.sidecar) + ": " +
                                                                    read.error().context +
                                                                    "; the parts were not added"});
            }
            held.mesh = std::move(read->mesh);
        }
        edited.parts.push_back(imported_part(held));
    }
    return commit_model(s, *v, std::move(edited), "the parts were not added");
}

inline Result<bool> change(scene::SceneDesc& s, const SetImportedPart& e) {
    const auto at = flown(s, e, "set an imported part of");
    if (!at) return std::unexpected(at.error());
    const auto [v, m] = *at;
    const vehicles::PartDesc* now = find_named(m->parts, e.part);
    if (now == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "set an imported part of '" + e.vehicle + "': model '" +
                                                                 m->name + "' has no part '" + e.part + "'"});
    }
    if (!is_imported_part(*now)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "set an imported part of '" + e.vehicle + "': part '" + e.part +
                                         "' is not imported; only an imported part's role, min_feature and "
                                         "mount are set here"});
    }
    vehicles::ModelType edited = *m;
    vehicles::PartDesc& p = *find_named(edited.parts, e.part);
    p.role = std::string(e.visual_only ? vehicles::kDrawnOnlyRole : analysis::kImportedPartRole);
    p.min_feature = e.min_feature;
    p.mount_orient = e.mount_orient;
    if (p.role == now->role && p.min_feature == now->min_feature && p.mount_orient == now->mount_orient) return false;
    return commit_model(s, *v, std::move(edited), "the edit was not made");
}

}  // namespace detail

// A name for a new asset or vehicle: `base` if no asset or vehicle has it,
// else "<base>_<n>" for the lowest free n >= 2. An empty base is "object".
[[nodiscard]] inline std::string unique_name(const scene::SceneDesc& s, std::string_view base) {
    return detail::lowest_free(base.empty() ? "object" : base,
                               [&](std::string_view n) { return detail::object_taken(s, n); });
}

// The same, among the scene's models. An empty base is "model".
[[nodiscard]] inline std::string unique_model_name(const scene::SceneDesc& s, std::string_view base) {
    return detail::lowest_free(base.empty() ? "model" : base,
                               [&](std::string_view n) { return detail::model_taken(s, n); });
}

// Applies `edit` to the document's scene and commits it, or with `amend`
// amends the current state in place (Document::amend, a drag's later poses).
// A refusal, from the edit or from validate_scene(), leaves the document
// unchanged and says why; an edit that changes nothing (a rename to the same
// name) commits nothing.
[[nodiscard]] inline Result<void> apply(SceneDocument& doc, const SceneEdit& edit, bool amend = false) {
    scene::SceneDesc next = doc.desc();
    const Result<bool> changed = std::visit([&](const auto& e) { return detail::change(next, e); }, edit);
    if (!changed) return std::unexpected(changed.error());
    if (!*changed) return {};
    return amend ? doc.amend(std::move(next)) : doc.commit(std::move(next));
}

}  // namespace spade::sandbox::editor
