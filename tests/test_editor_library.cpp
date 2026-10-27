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
#include <fstream>
#include <iterator>
#include <limits>
#include <variant>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_files.hpp"
#include "../sandbox/editor_library.hpp"
#include "../sandbox/editor_panels.hpp"
#include "../sandbox/text_fit.hpp"
#include "editor_test_support.hpp"
#include "scene/library.hpp"
#include "scene/part_file.hpp"
#include "scene/model_file.hpp"
#include "scene/model_sources.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_part.hpp"
#include "sim/design_frame.hpp"
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

// The lowest point of `m`'s posed parts, its DESIGN frame at the origin and
// level (a scene's start pose places the design frame; the body is
// to_body_state() of it, as the sim spawns it) and its rotors still: each
// part's box (part_bounds()) through its posed frame, as Test/Docs' placed-at
// measures it.
[[nodiscard]] float lowest_part_point(const vh::ModelType& m) {
    const spade::FrameState b =
        spade::to_body_state(spade::FrameState{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.0f),
                                               glm::vec3(0.0f)},
                             glm::normalize(m.design_to_principal), m.com_offset);
    spade::BodyState body{};
    body.pos = b.pos;
    body.orient = b.orient;
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

// THUMBNAILS (N14b): each item carries the image its entry names, decoded,
// for the panel's tile.
TEST(EditorLibraryFiles, TheDefaultLibrarysItemsCarryTheirThumbnails) {
    const ed::LibraryLoad load = ed::load_library_items(library_index());
    EXPECT_TRUE(load.problems.empty()) << load.problems.front();
    ASSERT_FALSE(load.items.empty());
    for (const ed::LibraryItem& item : load.items) {
        SCOPED_TRACE(item.name);
        ASSERT_TRUE(item.thumbnail.has_value());
        EXPECT_EQ(item.thumbnail->width, 96u);
        EXPECT_EQ(item.thumbnail->height, 96u);
        EXPECT_EQ(item.thumbnail->rgba.size(), 96u * 96u * 4u);
    }
}

// A thumbnail that won't decode is named, in the reader's words, and its
// entry is still listed, as a name without a tile.
TEST(EditorLibraryFiles, AThumbnailThatWontDecodeIsNamedAndItsEntryIsStillListed) {
    support::TempDir tmp;
    std::filesystem::copy_file(library_file("primitives/box.object.yaml"), tmp.path() / "box.object.yaml");
    support::write_bytes(tmp.path() / "box.png", "not an image");
    support::write_bytes(tmp.path() / "library.yaml",
                         "library_version: 1\n"
                         "entries:\n"
                         "  - {name: \"Box\", category: \"primitives\", file: \"box.object.yaml\", thumbnail: "
                         "\"box.png\"}\n");
    const ed::LibraryLoad load = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(load.items.size(), 1u);
    EXPECT_EQ(load.items[0].name, "Box");
    EXPECT_FALSE(load.items[0].thumbnail.has_value());
    ASSERT_EQ(load.problems.size(), 1u);
    EXPECT_NE(load.problems[0].find("'Box'"), std::string::npos) << load.problems[0];
    EXPECT_NE(load.problems[0].find("box.png"), std::string::npos) << load.problems[0];
    EXPECT_NE(load.problems[0].find("not a PNG"), std::string::npos) << load.problems[0];
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
    const ed::LibraryItem quad = library_item("Reference quad (test fixture)");
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
    const ed::LibraryItem quad = library_item("Reference quad (test fixture)");
    ASSERT_TRUE(quad.model.has_value());
    ASSERT_FALSE(quad.model->parts.empty());
    const float low = lowest_part_point(*quad.model);
    ASSERT_TRUE(std::isfinite(low));
    EXPECT_NEAR(ed::rest_height(quad), -low, 1e-6f);
}

// The rest is the design frame's: a centre of mass 50 mm higher in the design
// frame carries the parts 50 mm higher, so the design origin rests 50 mm
// lower.
TEST(EditorLibraryFiles, ADronesRestIsItsDesignFramesNotItsBodys) {
    const ed::LibraryItem quad = library_item("Reference quad (test fixture)");
    ASSERT_TRUE(quad.model.has_value());
    ed::LibraryItem raised = quad;
    raised.model->com_offset += glm::vec3(0.0f, 0.05f, 0.0f);
    EXPECT_NEAR(ed::rest_height(raised), ed::rest_height(quad) - 0.05f, 1e-5f);
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

// ===========================================================================
// PARTS (library index v2, Core's L2; INT-14). A motor, prop, ESC or battery
// entry is read through load_library_part() against the library's sources
// registry, listed in its own category, and inspect-only: it is built into a
// drone, never placed on its own.
// ===========================================================================

namespace {

// Test/Docs' motor (tests/test_part_file.cpp), citing `source` for its kv.
[[nodiscard]] std::string motor_text(std::string_view source) {
    return "part_version: 1\n"
           "motor:\n"
           "  kv: 960\n"
           "  resistance: 0.1\n"
           "  no_load_current: 0.5\n"
           "  no_load_voltage: 10\n"
           "  current_max: 20\n"
           "  pole_pairs: 7\n"
           "  inductance: 0\n"
           "  rotor_inertia: 1e-05\n"
           "  stator: \"2312\"\n"
           "  mass: 0.055\n"
           "  shape: {kind: cylinder, size: [0.014, 0.025, 0]}\n"
           "sources:\n"
           "  kv: {source: \"" + std::string(source) + "\", grade: retailer}\n"
           "  resistance: {grade: assumed, note: \"typical of a 2312 motor\"}\n"
           "  no_load_current: {grade: assumed, note: \"typical\"}\n"
           "  no_load_voltage: {grade: assumed, note: \"typical\"}\n"
           "  current_max: {grade: assumed, note: \"typical\"}\n"
           "  pole_pairs: {grade: assumed, note: \"12N14P\"}\n"
           "  inductance: {grade: assumed, note: \"dropped\"}\n"
           "  rotor_inertia: {grade: assumed, note: \"typical\"}\n"
           "  mass: {source: \"retail_phantom3_parts\", grade: retailer, note: \"55 g\"}\n"
           "  shape: {grade: assumed, note: \"a 28 mm bell\"}\n";
}

// A version-2 library in `dir`: the box, then a motor citing `source`.
void write_parts_library(const std::filesystem::path& dir, std::string_view source) {
    std::filesystem::copy_file(library_file("primitives/box.object.yaml"), dir / "box.object.yaml");
    std::filesystem::copy_file(library_file("sources.yaml"), dir / "sources.yaml");
    support::write_bytes(dir / "m2312.motor.yaml", motor_text(source));
    support::write_bytes(dir / "library.yaml",
                         "library_version: 2\n"
                         "entries:\n"
                         "  - {name: \"Box\", category: \"primitives\", file: \"box.object.yaml\"}\n"
                         "  - {name: \"2312 motor\", category: \"motors\", file: \"m2312.motor.yaml\"}\n");
}

}  // namespace

TEST(EditorLibraryFiles, APartIsListedInItsCategoryAndIsNeverPlaced) {
    support::TempDir tmp;
    write_parts_library(tmp.path(), "retail_phantom3_parts");
    const ed::LibraryLoad load = ed::load_library_items(tmp.path() / "library.yaml");
    EXPECT_TRUE(load.problems.empty()) << load.problems.front();
    ASSERT_EQ(load.items.size(), 2u);
    const ed::LibraryItem& motor = load.items[1];
    EXPECT_EQ(motor.name, "2312 motor");
    EXPECT_EQ(motor.kind, ed::LibraryItem::Kind::part);
    EXPECT_EQ(ed::panel_category(motor), "motors");
    ASSERT_TRUE(motor.part.has_value());
    EXPECT_TRUE(std::holds_alternative<sc::MotorPart>(*motor.part));
    EXPECT_FALSE(ed::placeable(motor));
    EXPECT_TRUE(ed::placeable(load.items[0]));

    ed::SceneDocument doc = hover_document();
    const std::string before = support::text_of(doc.desc());
    const Result<std::string> placed = ed::place(doc, motor, pose_at(glm::vec3(1.0f, 0.0f, 0.0f)));
    ASSERT_FALSE(placed.has_value());
    EXPECT_NE(placed.error().context.find("built into a drone"), std::string::npos) << placed.error().context;
    EXPECT_EQ(support::text_of(doc.desc()), before);
}

// A part citing a source the registry doesn't list is load_library_part()'s
// refusal, named; the rest still load.
TEST(EditorLibraryFiles, APartCitingAnUnlistedSourceIsNamedAndTheRestStillLoad) {
    support::TempDir tmp;
    write_parts_library(tmp.path(), "nowhere_listed");
    const ed::LibraryLoad load = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(load.items.size(), 1u);
    EXPECT_EQ(load.items[0].name, "Box");
    ASSERT_EQ(load.problems.size(), 1u);
    EXPECT_NE(load.problems[0].find("'2312 motor'"), std::string::npos) << load.problems[0];
}
// ===========================================================================
// GRADES (INT-14's assumed marks; the lead's plan, 2026-10-22). A part's
// inspector rows are its cited fields in its own order, each with its value as
// the part's file writes it and its citation's grade, source and note. A part
// with any field graded assumed carries the mark; an object never does.
// Clicking a part's tile inspects it: the scene's selection clears and
// nothing is armed.
// ===========================================================================

namespace {

// motor_text() with every assumed citation cited to `source` as a retailer's.
[[nodiscard]] std::string motor_text_all_cited(std::string_view source) {
    std::string text = motor_text(source);
    for (std::size_t i = text.find("{grade: assumed"); i != std::string::npos; i = text.find("{grade: assumed")) {
        const std::size_t end = text.find('}', i);
        text.replace(i, end - i + 1, "{source: \"" + std::string(source) + "\", grade: retailer}");
    }
    return text;
}

[[nodiscard]] const ed::PartRow* row_of(const std::vector<ed::PartRow>& rows, std::string_view field) {
    for (const ed::PartRow& r : rows) {
        if (r.field == field) return &r;
    }
    ADD_FAILURE() << "no row for '" << field << "'";
    return nullptr;
}

}  // namespace

TEST(EditorLibraryGrades, APartsRowsAreItsCitedFieldsWithTheirValuesAndGrades) {
    support::TempDir tmp;
    write_parts_library(tmp.path(), "retail_phantom3_parts");
    const ed::LibraryLoad load = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(load.items.size(), 2u);
    ASSERT_TRUE(load.items[1].part.has_value());
    const Result<std::vector<ed::PartRow>> rows = ed::part_rows(*load.items[1].part);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    std::vector<std::string> fields;
    for (const ed::PartRow& r : *rows) fields.push_back(r.field);
    EXPECT_EQ(fields, sc::cited_fields(*load.items[1].part));
    ASSERT_EQ(rows->size(), 10u);
    const ed::PartRow& kv = (*rows)[0];
    EXPECT_EQ(kv.field, "kv");
    EXPECT_EQ(kv.value, "960");
    EXPECT_EQ(kv.grade, sc::Grade::retailer);
    EXPECT_EQ(kv.source, "retail_phantom3_parts");
    const ed::PartRow& resistance = (*rows)[1];
    EXPECT_EQ(resistance.field, "resistance");
    EXPECT_EQ(resistance.value, "0.1");
    EXPECT_EQ(resistance.grade, sc::Grade::assumed);
    EXPECT_EQ(resistance.source, "");
    EXPECT_EQ(resistance.note, "typical of a 2312 motor");
    const ed::PartRow& mass = (*rows)[8];
    EXPECT_EQ(mass.field, "mass");
    EXPECT_EQ(mass.value, "0.055");
    EXPECT_EQ(mass.note, "55 g");
    const ed::PartRow& shape = rows->back();
    EXPECT_EQ(shape.field, "shape");
    EXPECT_EQ(shape.value.rfind("{kind: cylinder", 0), 0u) << shape.value;
}

// The library's own prop: its tables and its blade's lists show as counts,
// its airfoil without quotes, and an assumed value may still cite a source.
TEST(EditorLibraryGrades, APropsTablesAndBladeShowAsCountsAndAnAssumedValueMayCiteASource) {
    const ed::LibraryItem prop = library_item("6-inch three-blade (scaled stand-in)");
    ASSERT_TRUE(prop.part.has_value());
    const Result<std::vector<ed::PartRow>> rows = ed::part_rows(*prop.part);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    if (const ed::PartRow* r = row_of(*rows, "diameter")) {
        EXPECT_EQ(r->value, "0.1524");
        EXPECT_EQ(r->grade, sc::Grade::retailer);
        EXPECT_EQ(r->source, "retail_fpv_parts");
        EXPECT_EQ(r->note, "HQProp DP 6x3.5x3");
    }
    if (const ed::PartRow* r = row_of(*rows, "ct_table")) {
        EXPECT_EQ(r->value, "table, 19 points");
        EXPECT_EQ(r->grade, sc::Grade::assumed);
        EXPECT_EQ(r->source, "uiuc_propdb");
    }
    if (const ed::PartRow* r = row_of(*rows, "blade.r_over_r")) {
        EXPECT_EQ(r->value, "list, 18 values");
    }
    if (const ed::PartRow* r = row_of(*rows, "blade.airfoil")) {
        EXPECT_EQ(r->value, "unknown");
        EXPECT_EQ(r->grade, sc::Grade::assumed);
    }
    EXPECT_TRUE(ed::any_assumed(prop));
}

TEST(EditorLibraryGrades, APartWithNoAssumedFieldCarriesNoMarkAndNeitherDoesAnObject) {
    support::TempDir tmp;
    write_parts_library(tmp.path(), "retail_phantom3_parts");
    const ed::LibraryLoad graded = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(graded.items.size(), 2u);
    EXPECT_TRUE(ed::any_assumed(graded.items[1]));
    EXPECT_FALSE(ed::any_assumed(graded.items[0]));  // the Box

    support::write_bytes(tmp.path() / "m2312.motor.yaml", motor_text_all_cited("retail_phantom3_parts"));
    const ed::LibraryLoad cited = ed::load_library_items(tmp.path() / "library.yaml");
    ASSERT_EQ(cited.items.size(), 2u) << (cited.problems.empty() ? "" : cited.problems.front());
    EXPECT_FALSE(ed::any_assumed(cited.items[1]));
}

TEST(EditorLibraryGrades, InspectingAPartClearsTheSelectionAndArmsNothing) {
    support::TempDir tmp;
    write_parts_library(tmp.path(), "retail_phantom3_parts");
    ed::EditorPanels p;
    p.library = ed::load_library_items(tmp.path() / "library.yaml").items;
    ASSERT_EQ(p.library.size(), 2u);
    p.selection = ed::Selection{ed::Selection::Kind::object, "box"};
    EXPECT_TRUE(ed::inspect_item(p, 1));
    EXPECT_EQ(p.inspected, std::optional<std::size_t>(1));
    EXPECT_EQ(p.selection.kind, ed::Selection::Kind::none);
    EXPECT_FALSE(p.armed.has_value());

    // An object's tile arms, as it always has; it is never inspected.
    p.selection = ed::Selection{ed::Selection::Kind::object, "box"};
    EXPECT_FALSE(ed::inspect_item(p, 0));
    EXPECT_FALSE(ed::inspect_item(p, 2));
    EXPECT_EQ(p.inspected, std::optional<std::size_t>(1));
    EXPECT_EQ(p.selection.kind, ed::Selection::Kind::object);
}

// A cell shows a fractional number to 4 significant figures (the lead,
// 2026-10-22; the hover keeps the whole value). An integer, a count, a word
// or a shape stays as written.
TEST(EditorLibraryGrades, ACellShowsAFractionalNumberToFourSignificantFigures) {
    EXPECT_EQ(ed::short_value("0.09635567375886525"), "0.09636");
    EXPECT_EQ(ed::short_value("0.005869995503936827"), "0.00587");
    EXPECT_EQ(ed::short_value("0.18147428896905915"), "0.1815");
    EXPECT_EQ(ed::short_value("0.2388"), "0.2388");
    EXPECT_EQ(ed::short_value("0.127"), "0.127");
    EXPECT_EQ(ed::short_value("1e-05"), "1e-05");
    EXPECT_EQ(ed::short_value("1234.5678"), "1235");
    EXPECT_EQ(ed::short_value("960"), "960");
    EXPECT_EQ(ed::short_value("0"), "0");
    EXPECT_EQ(ed::short_value("table, 12 points"), "table, 12 points");
    EXPECT_EQ(ed::short_value("unknown"), "unknown");
    EXPECT_EQ(ed::short_value("{kind: cylinder, size: [0.014, 0.025, 0]}"), "{kind: cylinder, size: [0.014, 0.025, 0]}");
    EXPECT_EQ(ed::short_value(""), "");
}

// A library drone's grades are its L5 sidecar's (scene/model_sources.hpp;
// the lead's Q2, deferred to L5): one row per input, in the sidecar's order,
// the value as the sidecar writes it (a vector as a flow sequence), then its
// grade, source and note. The sidecar sits beside the model file.
TEST(EditorLibraryGrades, ADronesRowsAreItsSidecarsInputsWithTheirValuesAndGrades) {
    EXPECT_EQ(ed::sidecar_file("drones/crazyflie_class.model.yaml"), "drones/crazyflie_class.sources.yaml");
    EXPECT_EQ(ed::sidecar_file("obstacles/crate.object.yaml"), "");
    sc::ModelSources s;
    s.inputs.push_back(
        sc::ModelInput{"mass", 0.0271, sc::SourceRef{"bitcraze_cf21", sc::Grade::measured, ""}, std::nullopt});
    s.inputs.push_back(sc::ModelInput{"rotors[0].hub.orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0},
                                      sc::SourceRef{"", sc::Grade::assumed, "identity"}, std::nullopt});
    const std::vector<ed::PartRow> rows = ed::model_rows(s);
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_EQ(rows[0].field, "mass");
    EXPECT_EQ(rows[0].value, "0.0271");
    EXPECT_EQ(rows[0].grade, sc::Grade::measured);
    EXPECT_EQ(rows[0].source, "bitcraze_cf21");
    EXPECT_EQ(rows[0].note, "");
    EXPECT_EQ(rows[1].field, "rotors[0].hub.orientation");
    EXPECT_EQ(rows[1].value, "[1, 0, 0, 0]");
    EXPECT_EQ(rows[1].grade, sc::Grade::assumed);
    EXPECT_EQ(rows[1].source, "");
    EXPECT_EQ(rows[1].note, "identity");
}

// A drone with an input graded assumed is marked as a part is, and inspected
// as one, staying armed to place (a drone is placeable; a part is not). A
// drone without a sidecar (a project's own, the reference quad) has no
// grades to show, and one citing every input carries no mark.
TEST(EditorLibraryGrades, ADroneWithAnAssumedInputIsMarkedAndInspectedLikeAPart) {
    ed::LibraryItem drone;
    drone.kind = ed::LibraryItem::Kind::model;
    drone.name = "drone";
    EXPECT_FALSE(ed::has_grades(drone));
    EXPECT_FALSE(ed::any_assumed(drone));
    sc::ModelSources s;
    s.inputs.push_back(
        sc::ModelInput{"mass", 0.0271, sc::SourceRef{"bitcraze_cf21", sc::Grade::measured, ""}, std::nullopt});
    drone.sources = s;
    EXPECT_TRUE(ed::has_grades(drone));
    EXPECT_FALSE(ed::any_assumed(drone));
    s.inputs.push_back(
        sc::ModelInput{"gravity", 9.80665, sc::SourceRef{"", sc::Grade::assumed, "standard gravity"}, std::nullopt});
    drone.sources = s;
    EXPECT_TRUE(ed::any_assumed(drone));

    ed::EditorPanels p;
    p.library = {drone};
    p.selection = ed::Selection{ed::Selection::Kind::object, "box"};
    p.armed = 0;
    EXPECT_TRUE(ed::inspect_item(p, 0));
    EXPECT_EQ(p.inspected, std::optional<std::size_t>(0));
    EXPECT_EQ(p.selection.kind, ed::Selection::Kind::none);
    EXPECT_EQ(p.armed, std::optional<std::size_t>(0)) << "an inspected drone stays armed to place";
    p.library[0].sources.reset();
    p.inspected.reset();
    EXPECT_FALSE(ed::inspect_item(p, 0));
    EXPECT_FALSE(p.inspected.has_value());
}

// The default library's drones carry their sidecars, checked against the
// library as their own tests check them (check_model_sources()). Every one
// has a sidecar, the reference quad's too since L6 (a drone without one
// loads without grades: AStaleSidecarIsNamedAndItsDroneLoadsWithoutGrades).
// Each has an input assumed (the compile's operating point, or the fixture's
// prop ramp), so each tile is marked.
TEST(EditorLibraryGrades, TheDefaultLibrarysDronesCarryTheirSidecars) {
    const std::filesystem::path dir = std::filesystem::path(SPADE_ASSETS_DIR) / "library";
    const ed::LibraryLoad load = ed::load_library_items(dir / "library.yaml");
    std::string problems;
    for (const std::string& s : load.problems) problems += "\n  " + s;
    EXPECT_TRUE(load.problems.empty()) << problems;
    std::size_t with = 0;
    std::size_t without = 0;
    for (const ed::LibraryItem& item : load.items) {
        if (item.kind != ed::LibraryItem::Kind::model) continue;
        SCOPED_TRACE(item.name);
        const std::string sidecar = ed::sidecar_file(item.file);
        EXPECT_EQ(item.sources.has_value(), !sidecar.empty() && std::filesystem::exists(dir / sidecar));
        if (!item.sources) {
            ++without;
            continue;
        }
        ++with;
        EXPECT_FALSE(item.sources->inputs.empty());
        EXPECT_TRUE(ed::any_assumed(item));
    }
    EXPECT_GE(with, 6u) << "L5's library drones and the reference quad (L6)";
    EXPECT_EQ(without, 0u) << "every library drone carries its sidecar";
}

// A sidecar its model no longer matches (a stale hash) is named, and the
// drone still loads, without grades: the editor shows no citation the
// library's own check would refuse.
TEST(EditorLibraryGrades, AStaleSidecarIsNamedAndItsDroneLoadsWithoutGrades) {
    support::TempDir tmp;
    const std::filesystem::path dir = tmp.path() / "library";
    std::filesystem::copy(std::filesystem::path(SPADE_ASSETS_DIR) / "library", dir,
                          std::filesystem::copy_options::recursive);
    const std::filesystem::path sidecar = dir / "drones" / "crazyflie_class.sources.yaml";
    std::string text;
    {
        std::ifstream in(sidecar, std::ios::binary);
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string key = "model: {file: \"drones/crazyflie_class.model.yaml\", hash: \"";
    const std::size_t at = text.find(key);
    ASSERT_NE(at, std::string::npos);
    text.replace(at + key.size(), 16, "0000000000000000");
    {
        std::ofstream out(sidecar, std::ios::binary | std::ios::trunc);
        out << text;
    }
    const ed::LibraryLoad load = ed::load_library_items(dir / "library.yaml");
    ASSERT_EQ(load.problems.size(), 1u);
    EXPECT_NE(load.problems[0].find("crazyflie_class.sources.yaml"), std::string::npos) << load.problems[0];
    const auto drone = std::find_if(load.items.begin(), load.items.end(), [](const ed::LibraryItem& i) {
        return i.file == "drones/crazyflie_class.model.yaml";
    });
    ASSERT_NE(drone, load.items.end()) << "the drone still loads";
    EXPECT_FALSE(drone->sources.has_value());
    EXPECT_FALSE(ed::has_grades(*drone));
}

// A field path too long for its cell is cut from the LEFT (the lead,
// 2026-10-26): its end tells rows apart (".x", ".y", ".z"), so the ellipsis
// goes first and the longest tail that fits after it is kept. Here every
// byte is 1 wide and the ellipsis 3.
TEST(EditorLibraryGrades, AFieldPathTooLongForItsCellIsCutFromTheLeft) {
    const auto unit = [](std::string_view s) { return static_cast<float>(s.size()); };
    const std::string_view path = "rotors[0].hub.position.x";  // 24 wide
    EXPECT_EQ(spade::sandbox::ui::left_cut(path, 24.0f, 3.0f, unit), 0u) << "it fits: nothing is cut";
    EXPECT_EQ(spade::sandbox::ui::left_cut(path, 30.0f, 3.0f, unit), 0u);
    // 20 wide: the ellipsis and 17 of it, "0].hub.position.x".
    EXPECT_EQ(path.substr(spade::sandbox::ui::left_cut(path, 20.0f, 3.0f, unit)), "0].hub.position.x");
    // Too narrow for anything but the last character: it is kept.
    EXPECT_EQ(path.substr(spade::sandbox::ui::left_cut(path, 2.0f, 3.0f, unit)), "x");
    EXPECT_EQ(spade::sandbox::ui::left_cut("", 0.0f, 3.0f, unit), 0u);
    // A cut never splits a UTF-8 sequence: "a\xc2\xb0z" is "a", a 2-byte degree sign, "z", 4 wide. With
    // an ellipsis 0.5 wide, 3 wide would fit "\xb0z" if a cut could start inside the sign; it keeps "z".
    const std::string_view degrees = "a\xc2\xb0z";
    EXPECT_EQ(degrees.substr(spade::sandbox::ui::left_cut(degrees, 3.0f, 0.5f, unit)), "z");
    EXPECT_EQ(degrees.substr(spade::sandbox::ui::left_cut(degrees, 3.5f, 0.5f, unit)), "\xc2\xb0z");
}
