// The editor's library: the built-in items and placement by copy (editor plan
// Task 9; EDT-012), and the default
// library's files (N14a, INT-14).
//
// The built-in items ship with the editor: the quadrotor template, and the
// primitive assets (box, sphere, cylinder, capsule). Placing an item copies it
// into the scene: a model once per placed vehicle, under its own name, and an
// asset as an asset. A scene never refers to a library item.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_library.hpp"
#include "editor_test_support.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"
#include "world/aabb.hpp"
#include "world/sdf.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;
using spade::SdfPose;

// A copy: the library is often a temporary.
[[nodiscard]] ed::LibraryItem item_named(const std::vector<ed::LibraryItem>& lib, std::string_view name) {
    for (const ed::LibraryItem& i : lib) {
        if (i.name == name) return i;
    }
    ADD_FAILURE() << "no library item named " << name;
    return lib.front();
}

[[nodiscard]] SdfPose pose_at(glm::vec3 p, float yaw = 0.0f, float scale = 1.0f) {
    SdfPose at;
    at.position = p;
    at.rotation = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    at.scale = scale;
    return at;
}

[[nodiscard]] ed::SceneDocument hover_document() {
    return ed::make_scene_document(support::sample_scene(), {});
}

}  // namespace

TEST(EditorLibrary, TheBuiltInItemsAreTheQuadrotorAndFourPrimitives) {
    const std::vector<ed::LibraryItem> lib = ed::builtin_library();
    std::vector<std::string> names;
    for (const ed::LibraryItem& i : lib) names.push_back(i.name);
    EXPECT_EQ(names, (std::vector<std::string>{"quadrotor", "box", "sphere", "cylinder", "capsule"}));
    for (const ed::LibraryItem& i : lib) {
        if (i.kind == ed::LibraryItem::Kind::model) {
            ASSERT_TRUE(i.model.has_value()) << i.name;
            EXPECT_FALSE(i.asset.has_value()) << i.name;
            EXPECT_TRUE(i.model->validate().has_value()) << i.name;
        } else {
            ASSERT_TRUE(i.asset.has_value()) << i.name;
            EXPECT_FALSE(i.model.has_value()) << i.name;
        }
    }
}

TEST(EditorLibrary, PlacingAnAssetTwiceGivesTwoUniquelyNamedCopiesHoldingNoPath) {
    ed::SceneDocument doc = hover_document();
    const ed::LibraryItem& box = item_named(ed::builtin_library(), "box");
    const Result<std::string> first = ed::place(doc, box, pose_at(glm::vec3(1.0f, 0.5f, 0.0f)));
    const Result<std::string> second = ed::place(doc, box, pose_at(glm::vec3(-1.0f, 0.5f, 0.0f)));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(*first, "box");
    EXPECT_EQ(*second, "box_2");
    ASSERT_EQ(doc.desc().assets.size(), 2u);
    for (const spade::scene::SceneAsset& a : doc.desc().assets) {
        EXPECT_TRUE(a.visual.mesh_ref.empty()) << a.name;   // no file behind it
        EXPECT_FALSE(a.collider.nodes.empty()) << a.name;   // the shape itself, copied in
    }
    EXPECT_EQ(doc.desc().assets[1].pose.position, glm::vec3(-1.0f, 0.5f, 0.0f));
}

TEST(EditorLibrary, PlacingTheQuadrotorTwiceGivesTwoVehiclesAndTwoModels) {
    ed::SceneDocument doc = hover_document();
    const std::size_t vehicles = doc.desc().vehicles.size();
    const std::size_t models = doc.desc().models.size();
    const ed::LibraryItem& quad = item_named(ed::builtin_library(), "quadrotor");
    const Result<std::string> first = ed::place(doc, quad, pose_at(glm::vec3(2.0f, 0.0f, 0.0f)));
    const Result<std::string> second = ed::place(doc, quad, pose_at(glm::vec3(4.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_NE(*first, *second);
    ASSERT_EQ(doc.desc().vehicles.size(), vehicles + 2);
    ASSERT_EQ(doc.desc().models.size(), models + 2);
    const auto& a = doc.desc().vehicles[vehicles];
    const auto& b = doc.desc().vehicles[vehicles + 1];
    EXPECT_EQ(a.name, *first);
    EXPECT_EQ(b.name, *second);
    EXPECT_NE(a.model, b.model);  // a model once per placed vehicle (EDT-012)
}

TEST(EditorLibrary, APlacedModelEqualsTheLibrarysExceptForItsName) {
    ed::SceneDocument doc = hover_document();
    const ed::LibraryItem& quad = item_named(ed::builtin_library(), "quadrotor");
    ASSERT_TRUE(ed::place(doc, quad, pose_at(glm::vec3(2.0f, 0.0f, 0.0f))).has_value());
    ASSERT_TRUE(ed::place(doc, quad, pose_at(glm::vec3(4.0f, 0.0f, 0.0f))).has_value());
    for (std::size_t i = doc.desc().vehicles.size() - 2; i < doc.desc().vehicles.size(); ++i) {
        spade::vehicles::ModelType placed = support::model_named(doc.desc(), doc.desc().vehicles[i].model);
        spade::vehicles::ModelType library = *quad.model;
        placed.name = "same";
        library.name = "same";
        EXPECT_EQ(support::text_of(support::scene_with(placed)), support::text_of(support::scene_with(library)));
    }
}

TEST(EditorLibrary, APlacedVehicleStartsAtThePose) {
    ed::SceneDocument doc = hover_document();
    const SdfPose at = pose_at(glm::vec3(3.0f, 0.25f, -2.0f), 0.7f);
    const Result<std::string> name = ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), at);
    ASSERT_TRUE(name.has_value()) << name.error().context;
    const spade::scene::SceneVehicle& v = doc.desc().vehicles.back();
    EXPECT_EQ(v.start.pos, at.position);
    EXPECT_EQ(v.start.orient, at.rotation);
    EXPECT_EQ(v.start.vel, glm::vec3(0.0f));
}

// A vehicle has no scale; one is refused rather than dropped (L6).
TEST(EditorLibrary, AScaledVehicleIsRefusedAndNothingIsPlaced) {
    ed::SceneDocument doc = hover_document();
    const Result<std::string> r =
        ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), pose_at(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f, 2.0f));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("scale"), std::string::npos) << r.error().context;
    EXPECT_FALSE(doc.can_undo());
}

TEST(EditorLibrary, APlacementIsOneUndo) {
    ed::SceneDocument doc = hover_document();
    const std::string before = support::text_of(doc.desc());
    ASSERT_TRUE(ed::place(doc, item_named(ed::builtin_library(), "quadrotor"), pose_at(glm::vec3(2.0f, 0, 0))).has_value());
    ASSERT_TRUE(doc.undo());
    EXPECT_EQ(support::text_of(doc.desc()), before);  // the vehicle and its model, both gone
    EXPECT_FALSE(doc.can_undo());
}

// The kind comes from the model, so a new kind needs no change to the panel
// (editor spec §4). Drones are the only kind today.
TEST(EditorLibrary, TheTemplatesVehicleKindIsQuadrotor) {
    const spade::vehicles::ModelType quad = *item_named(ed::builtin_library(), "quadrotor").model;
    EXPECT_EQ(ed::vehicle_kind_of(quad), "quadrotor");
    spade::vehicles::ModelType six = quad;
    six.rotors.push_back(six.rotors.front());
    six.rotors.push_back(six.rotors.front());
    EXPECT_EQ(ed::vehicle_kind_of(six), "multirotor");
}

// ---- the scene picker (SL13; editor plan Task 11) ----

// Every scene file in assets/scenes/ is a built-in source, by name, in order.
TEST(EditorSceneSources, EveryBuiltInSceneFileIsListed) {
    namespace fs = std::filesystem;
    const fs::path assets(SPADE_ASSETS_DIR);
    const std::vector<ed::SceneSource> sources = ed::scene_sources(assets, fs::path());
    std::vector<std::string> expected;
    for (const auto& entry : fs::directory_iterator(assets / "scenes")) {
        const std::string file = entry.path().filename().string();
        if (file.ends_with(".scene.yaml")) expected.push_back(file.substr(0, file.size() - 11));
    }
    std::sort(expected.begin(), expected.end());
    std::vector<std::string> names;
    for (const ed::SceneSource& s : sources) {
        EXPECT_EQ(s.kind, ed::SceneSource::Kind::builtin) << s.name;
        names.push_back(s.name);
    }
    EXPECT_EQ(names, expected);
    EXPECT_FALSE(names.empty());
}

// A scene saved in the saved folder is listed after the built-ins; anything
// that is not a scene file is not.
TEST(EditorSceneSources, ASavedSceneIsListedAsSaved) {
    const support::TempDir tmp;
    const std::filesystem::path copy = support::mirror({std::filesystem::path(SPADE_ASSETS_DIR), support::hover_scene()},
                                                       tmp.path());
    support::write_bytes(copy.parent_path() / "notes.txt", "not a scene");
    const std::vector<ed::SceneSource> sources =
        ed::scene_sources(std::filesystem::path(SPADE_ASSETS_DIR), copy.parent_path());
    ASSERT_FALSE(sources.empty());
    const ed::SceneSource& last = sources.back();
    EXPECT_EQ(last.kind, ed::SceneSource::Kind::saved);
    EXPECT_EQ(last.name, "hover");
    EXPECT_EQ(last.file, copy);
    for (const ed::SceneSource& s : sources) EXPECT_NE(s.name, "notes") << s.file;
}

TEST(EditorSceneSources, EachListedSceneOpens) {
    for (const ed::SceneSource& s : ed::scene_sources(std::filesystem::path(SPADE_ASSETS_DIR), {})) {
        const spade::Result<ed::OpenedScene> opened = ed::open_scene(s.file);
        EXPECT_TRUE(opened.has_value()) << s.file << ": " << (opened ? "" : opened.error().context);
    }
}

// A saved folder that does not exist yet lists nothing for it, and no error.
TEST(EditorSceneSources, AMissingSavedFolderListsNoSavedScenes) {
    const std::vector<ed::SceneSource> sources =
        ed::scene_sources(std::filesystem::path(SPADE_ASSETS_DIR), std::filesystem::path("no_such_folder_here"));
    for (const ed::SceneSource& s : sources) EXPECT_NE(s.kind, ed::SceneSource::Kind::saved) << s.file;
}

// ===========================================================================
// THE DEFAULT LIBRARY (N14a, INT-14). The panel lists Core's library.yaml
// (scene::load_library()) in its order, and placing goes through Core:
// a model entry through scene::add_model(), which keeps its chain, and an
// object entry through scene::add_object(), which adds its materials. Load
// from file reads a model or object file by its extension, as the index does.
// A drone rests on its own lowest part, an object on its collider's box.
// ===========================================================================

namespace {

namespace sc = spade::scene;
namespace vh = spade::vehicles;

[[nodiscard]] std::filesystem::path library_index() {
    return std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "library.yaml";
}

[[nodiscard]] std::filesystem::path library_file(std::string_view relative) {
    return std::filesystem::path(SPADE_ASSETS_DIR) / "library" / relative;
}

[[nodiscard]] ed::LibraryItem library_item(std::string_view name) {
    const ed::LibraryLoad load = ed::load_library_items(library_index());
    EXPECT_TRUE(load.problems.empty()) << load.problems.front();
    return item_named(load.items, name);
}

// The lowest point of `m`'s posed parts, its body at the origin and level and
// its rotors still: each part's box (part_bounds()) through its posed frame,
// as Test/Docs' placed-at measures it.
[[nodiscard]] float lowest_part_point(const vh::ModelType& m) {
    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<float> still(m.rotors.size(), 0.0f);
    const Result<vh::PosedParts> posed = vh::posed_parts(m, body, still, still, vh::PartFilter::all);
    EXPECT_TRUE(posed.has_value()) << (posed ? "" : posed.error().context);
    float low = std::numeric_limits<float>::infinity();
    if (!posed) return low;
    for (const vh::PosedPart& p : posed->parts) {
        const Result<spade::Aabb> box = vh::part_bounds(m.parts[p.part]);
        EXPECT_TRUE(box.has_value()) << (box ? "" : box.error().context);
        if (!box) continue;
        for (uint32_t c = 0; c < 8; ++c) {
            const glm::vec3 corner((c & 1u) != 0 ? box->max.x : box->min.x, (c & 2u) != 0 ? box->max.y : box->min.y,
                                   (c & 4u) != 0 ? box->max.z : box->min.z);
            low = std::min(low, (p.origin + p.orient * corner).y);
        }
    }
    return low;
}

}  // namespace

TEST(EditorLibraryFiles, TheDefaultLibraryIsListedInItsIndexsOrder) {
    const Result<sc::Library> index = sc::load_library(library_index());
    ASSERT_TRUE(index.has_value()) << index.error().context;
    const ed::LibraryLoad load = ed::load_library_items(library_index());
    EXPECT_TRUE(load.problems.empty()) << load.problems.front();
    ASSERT_EQ(load.items.size(), index->entries.size());
    ASSERT_GE(load.items.size(), 9u);
    for (std::size_t i = 0; i < load.items.size(); ++i) {
        const ed::LibraryItem& item = load.items[i];
        const sc::LibraryEntry& entry = index->entries[i];
        EXPECT_EQ(item.name, entry.name);
        EXPECT_EQ(item.category, entry.category) << entry.name;
        EXPECT_EQ(item.file, entry.file) << entry.name;
        EXPECT_EQ(item.kind == ed::LibraryItem::Kind::model, entry.kind == sc::LibraryKind::model) << entry.name;
        EXPECT_EQ(item.model.has_value(), entry.kind == sc::LibraryKind::model) << entry.name;
        EXPECT_EQ(item.asset.has_value(), entry.kind == sc::LibraryKind::object) << entry.name;
    }
}

TEST(EditorLibraryFiles, AnEntryThatWontLoadIsNamedAndTheRestStillLoad) {
    support::TempDir tmp;
    std::filesystem::copy_file(library_file("primitives/box.object.yaml"), tmp.path() / "box.object.yaml");
    support::write_bytes(tmp.path() / "bad.object.yaml", "object_version: 1\nnonsense: 1\n");
    support::write_bytes(tmp.path() / "library.yaml",
                         "library_version: 1\n"
                         "entries:\n"
                         "  - {name: \"Bad\", category: \"primitives\", file: \"bad.object.yaml\"}\n"
                         "  - {name: \"Box\", category: \"primitives\", file: \"box.object.yaml\"}\n");
    const ed::LibraryLoad load = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(load.items.size(), 1u);
    EXPECT_EQ(load.items[0].name, "Box");
    ASSERT_EQ(load.problems.size(), 1u);
    EXPECT_NE(load.problems[0].find("'Bad'"), std::string::npos) << load.problems[0];
}

TEST(EditorLibraryFiles, AnIndexThatWontLoadIsNamed) {
    support::TempDir tmp;
    const std::filesystem::path missing = tmp.path() / "library.yaml";
    const ed::LibraryLoad load = ed::load_library_items(missing);
    EXPECT_TRUE(load.items.empty());
    ASSERT_EQ(load.problems.size(), 1u);
    EXPECT_NE(load.problems[0].find("library.yaml"), std::string::npos) << load.problems[0];
}

// propulsion-kept: the placed model's three identities are the file's. The
// second copy is renamed, so its model identity (which folds the name) is
// the renamed file's; its parts and its chain fold no name.
TEST(EditorLibraryFiles, ThePlacedLibraryDroneKeepsItsThreeIdentities) {
    const Result<sc::ModelFile> file = sc::load_model_file(library_file("drones/reference_quad.model.yaml"));
    ASSERT_TRUE(file.has_value()) << file.error().context;
    ASSERT_TRUE(file->model.propulsion.has_value());
    const ed::LibraryItem quad = library_item("Reference quad");
    ed::SceneDocument doc = hover_document();
    const std::size_t before = doc.desc().vehicles.size();
    ASSERT_TRUE(ed::place(doc, quad, pose_at(glm::vec3(2.0f, 0.0f, 0.0f))).has_value());
    const Result<std::string> second = ed::place(doc, quad, pose_at(glm::vec3(4.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(second.has_value()) << second.error().context;
    ASSERT_EQ(doc.desc().vehicles.size(), before + 2);
    const vh::ModelType& one = support::model_named(doc.desc(), doc.desc().vehicles[before].model);
    const vh::ModelType& two = support::model_named(doc.desc(), doc.desc().vehicles[before + 1].model);
    EXPECT_EQ(vh::model_identity(one), vh::model_identity(file->model));
    vh::ModelType renamed = file->model;
    renamed.name = two.name;
    EXPECT_EQ(vh::model_identity(two), vh::model_identity(renamed));
    for (const vh::ModelType* m : {&one, &two}) {
        EXPECT_EQ(vh::parts_identity(*m), vh::parts_identity(file->model)) << m->name;
        ASSERT_TRUE(m->propulsion.has_value()) << m->name << ": the copy dropped the chain";
        EXPECT_EQ(vh::propulsion_identity(*m->propulsion), vh::propulsion_identity(*file->model.propulsion)) << m->name;
    }
    EXPECT_EQ(support::text_of(doc.desc()).find("library"), std::string::npos) << "the scene names the library";
}

// An object goes in through add_object(): its materials come with it.
TEST(EditorLibraryFiles, APlacedObjectBringsItsMaterials) {
    const ed::LibraryItem gate = library_item("Gate");
    ed::SceneDocument doc = hover_document();
    const Result<std::string> placed = ed::place(doc, gate, pose_at(glm::vec3(1.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(placed.has_value()) << placed.error().context;
    EXPECT_EQ(*placed, "gate");
    const auto& materials = doc.desc().materials;
    EXPECT_TRUE(std::any_of(materials.begin(), materials.end(),
                            [](const spade::MaterialDesc& m) { return m.name == "library_gate"; }));
    ASSERT_TRUE(doc.undo());
    EXPECT_EQ(support::text_of(doc.desc()), support::text_of(hover_document().desc())) << "one undo takes it back";
}

// A scene material of the same name and another colour is add_object()'s
// refusal, shown verbatim; nothing changes.
TEST(EditorLibraryFiles, AnObjectWhoseMaterialConflictsIsRefusedInCoresWordsAndNothingChanges) {
    const ed::LibraryItem gate = library_item("Gate");
    ed::SceneDocument doc = hover_document();
    spade::MaterialDesc other;
    other.name = "library_gate";
    other.base_color = glm::vec4(0.1f, 0.2f, 0.9f, 1.0f);
    ASSERT_TRUE(ed::apply(doc, ed::AddMaterial{other}).has_value());
    const std::string before = support::text_of(doc.desc());
    const Result<std::string> placed = ed::place(doc, gate, pose_at(glm::vec3(1.0f, 0.0f, 0.0f)));
    ASSERT_FALSE(placed.has_value());
    EXPECT_NE(placed.error().context.find("material 'library_gate': base_color differs"), std::string::npos)
        << placed.error().context;
    EXPECT_EQ(support::text_of(doc.desc()), before);
}

TEST(EditorLibraryFiles, LoadFromFileReadsAModelOrAnObjectByItsExtension) {
    const std::filesystem::path drone = library_file("drones/reference_quad.model.yaml");
    const Result<ed::LibraryItem> model = ed::item_from_file(drone);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    EXPECT_EQ(model->kind, ed::LibraryItem::Kind::model);
    ASSERT_TRUE(model->model.has_value());
    EXPECT_TRUE(model->model->propulsion.has_value());
    EXPECT_EQ(model->file, drone.generic_string());

    const Result<ed::LibraryItem> object = ed::item_from_file(library_file("obstacles/gate.object.yaml"));
    ASSERT_TRUE(object.has_value()) << object.error().context;
    EXPECT_EQ(object->kind, ed::LibraryItem::Kind::asset);
    ASSERT_TRUE(object->asset.has_value());
    EXPECT_FALSE(object->materials.empty());

    support::TempDir tmp;
    support::write_bytes(tmp.path() / "notes.txt", "a drone\n");
    const Result<ed::LibraryItem> other = ed::item_from_file(tmp.path() / "notes.txt");
    ASSERT_FALSE(other.has_value());
    EXPECT_NE(other.error().context.find(".model.yaml"), std::string::npos) << other.error().context;
    EXPECT_NE(other.error().context.find(".object.yaml"), std::string::npos) << other.error().context;
}

// placed-at's rest (Test/Docs' expected red): the reference quad's battery
// hangs below the built-in drone's 25 mm, so a drone rests on its own lowest
// posed part, not on kDroneBodyHalf.
TEST(EditorLibraryFiles, ALibraryDroneRestsOnItsOwnLowestPart) {
    const ed::LibraryItem quad = library_item("Reference quad");
    ASSERT_TRUE(quad.model.has_value());
    ASSERT_FALSE(quad.model->parts.empty());
    const float low = lowest_part_point(*quad.model);
    ASSERT_TRUE(std::isfinite(low));
    EXPECT_NEAR(ed::rest_height(quad), -low, 1e-6f);
}

// An object rests on its collider's box, whatever its primitives.
TEST(EditorLibraryFiles, ALibraryObjectRestsOnItsCollidersBox) {
    for (const ed::LibraryItem& item : ed::load_library_items(library_index()).items) {
        if (item.kind != ed::LibraryItem::Kind::asset) continue;
        ASSERT_TRUE(item.asset.has_value()) << item.name;
        const Result<spade::Aabb> box = spade::sdf_bounds(item.asset->collider);
        ASSERT_TRUE(box.has_value()) << item.name << ": " << box.error().context;
        EXPECT_NEAR(ed::rest_height(item), -box->min.y, 1e-6f) << item.name;
    }
}

// The edits read their file when applied: one that won't read is refused
// naming it, and nothing changes.
TEST(EditorLibraryFiles, AFileThatWontReadIsRefusedNamingItAndNothingChanges) {
    ed::SceneDocument doc = hover_document();
    const std::string before = support::text_of(doc.desc());
    const Result<void> object = ed::apply(doc, ed::AddObjectFile{"no/such.object.yaml", "", SdfPose{}});
    ASSERT_FALSE(object.has_value());
    EXPECT_NE(object.error().context.find("no/such.object.yaml"), std::string::npos) << object.error().context;
    const Result<void> model =
        ed::apply(doc, ed::AddModelFile{"no/such.model.yaml", spade::scene::SceneVehicle{"quad", "", {}}});
    ASSERT_FALSE(model.has_value());
    EXPECT_NE(model.error().context.find("no/such.model.yaml"), std::string::npos) << model.error().context;
    EXPECT_EQ(support::text_of(doc.desc()), before);
}
