// Save to library (EDT-020; INT-14 Q1a, the lead 2026-10-22): a scene's
// vehicle (its model, chain included) or asset (with the materials it names)
// is saved into the PROJECT library, <the scene's folder>/library/, created on
// the first save. The default library (assets/library) is never written, so a
// built-in scene has no project library. A taken name becomes <name>_2, a
// file already there is never overwritten, and the index is replaced
// atomically with Core's library_to_yaml() text. A refusal leaves the folder
// as it was.

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_library.hpp"
#include "../sandbox/editor_library_save.hpp"
#include "editor_test_support.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"

namespace {

namespace fs = std::filesystem;
namespace ed = spade::sandbox::editor;
namespace sc = spade::scene;
namespace support = spade::sandbox::editor::test;
using spade::Result;

const std::vector<fs::path> kNoMeshRoots{};

// Every file under `dir`, by its path relative to it, with its bytes: what a
// refusal must leave as it was.
[[nodiscard]] std::map<std::string, std::string> contents(const fs::path& dir) {
    std::map<std::string, std::string> out;
    std::error_code ec;
    for (fs::recursive_directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (it->is_regular_file()) out[fs::relative(it->path(), dir).generic_string()] = support::read_bytes(it->path());
    }
    return out;
}

[[nodiscard]] ed::LibraryItem default_item(std::string_view name) {
    const ed::LibraryLoad load =
        ed::load_library_items(fs::path(SPADE_ASSETS_DIR) / "library" / "library.yaml");
    for (const ed::LibraryItem& i : load.items) {
        if (i.name == name) return i;
    }
    ADD_FAILURE() << "no default library item named " << name;
    return ed::LibraryItem{};
}

// The hover scene with `item` placed in it at x = 1; `placed` is its name there.
[[nodiscard]] ed::SceneDocument hover_with(const ed::LibraryItem& item, std::string& placed) {
    ed::SceneDocument doc = ed::make_scene_document(support::sample_scene(), {});
    spade::SdfPose at;
    at.position = glm::vec3(1.0f, 0.0f, 0.0f);
    const Result<std::string> name = ed::place(doc, item, at);
    EXPECT_TRUE(name.has_value()) << (name ? "" : name.error().context);
    placed = name ? *name : std::string();
    return doc;
}

[[nodiscard]] const sc::SceneVehicle& vehicle_named(const sc::SceneDesc& s, std::string_view name) {
    for (const sc::SceneVehicle& v : s.vehicles) {
        if (v.name == name) return v;
    }
    ADD_FAILURE() << "no vehicle named " << name;
    return s.vehicles.at(0);
}

[[nodiscard]] const sc::SceneAsset& asset_named(const sc::SceneDesc& s, std::string_view name) {
    for (const sc::SceneAsset& a : s.assets) {
        if (a.name == name) return a;
    }
    ADD_FAILURE() << "no asset named " << name;
    return s.assets.at(0);
}

[[nodiscard]] std::string model_text(const spade::vehicles::ModelType& m) {
    const Result<std::string> t = sc::model_to_yaml(sc::ModelFile{m});
    EXPECT_TRUE(t.has_value()) << (t ? "" : t.error().context);
    return t ? *t : std::string();
}

}  // namespace

// The project library is the scene's own folder's library/; a built-in
// scene's folder is the assets folder, and an unsaved scene has none.
TEST(EditorLibrarySave, TheProjectLibraryIsTheScenesOwnFoldersAndABuiltInSceneHasNone) {
    const support::TempDir tmp;
    const Result<fs::path> own = ed::project_library_dir(tmp.path() / "mine.scene.yaml");
    ASSERT_TRUE(own.has_value()) << own.error().context;
    EXPECT_EQ(*own, tmp.path() / "library");

    const Result<fs::path> builtin = ed::project_library_dir(support::hover_scene());
    ASSERT_FALSE(builtin.has_value());
    EXPECT_NE(builtin.error().context.find("built-in"), std::string::npos) << builtin.error().context;
    EXPECT_NE(builtin.error().context.find("never written"), std::string::npos) << builtin.error().context;

    const Result<fs::path> unsaved = ed::project_library_dir(fs::path());
    ASSERT_FALSE(unsaved.has_value());
    EXPECT_NE(unsaved.error().context.find("save the scene"), std::string::npos) << unsaved.error().context;
}

// A vehicle is saved as a model file, its chain included, in a library made
// on the first save; it loads back as the scene's model, with its thumbnail.
TEST(EditorLibrarySave, AVehicleIsSavedAsAModelFileThatLoadsBackEqual) {
    std::string quad;
    const ed::SceneDocument doc = hover_with(default_item("Reference quad (test fixture)"), quad);
    ASSERT_FALSE(quad.empty());
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    ASSERT_FALSE(fs::exists(dir));

    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, doc.desc(), quad, kNoMeshRoots);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    EXPECT_EQ(saved->name, quad);
    EXPECT_EQ(saved->file.parent_path(), dir);
    EXPECT_TRUE(saved->file.filename().string().ends_with(".model.yaml")) << saved->file;
    EXPECT_FALSE(saved->thumbnail.empty()) << saved->no_thumbnail;

    const ed::LibraryLoad load = ed::load_library_items(dir / "library.yaml");
    ASSERT_TRUE(load.problems.empty()) << load.problems.front();
    ASSERT_EQ(load.items.size(), 1u);
    const ed::LibraryItem& item = load.items[0];
    EXPECT_EQ(item.name, quad);
    EXPECT_EQ(item.category, "drones");
    ASSERT_EQ(item.kind, ed::LibraryItem::Kind::model);
    ASSERT_TRUE(item.model.has_value());
    ASSERT_TRUE(item.model->propulsion.has_value()) << "the chain is part of the model";
    const sc::SceneVehicle& v = vehicle_named(doc.desc(), quad);
    EXPECT_EQ(model_text(*item.model), model_text(support::model_named(doc.desc(), v.model)));
    ASSERT_TRUE(item.thumbnail.has_value());
    EXPECT_EQ(item.thumbnail->width, ed::kThumbnailSize);
    EXPECT_EQ(item.thumbnail->height, ed::kThumbnailSize);
}

// An asset is saved as an object file with the scene materials it names; it
// loads back as the scene's asset.
TEST(EditorLibrarySave, AnAssetIsSavedAsAnObjectFileWithItsMaterials) {
    std::string gate;
    const ed::SceneDocument doc = hover_with(default_item("Gate"), gate);
    ASSERT_FALSE(gate.empty());
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";

    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, doc.desc(), gate, kNoMeshRoots);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    EXPECT_TRUE(saved->file.filename().string().ends_with(".object.yaml")) << saved->file;

    const Result<sc::Library> library = sc::load_library(dir / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    ASSERT_EQ(library->entries.size(), 1u);
    EXPECT_EQ(library->entries[0].kind, sc::LibraryKind::object);
    EXPECT_EQ(library->entries[0].category, "objects");
    const Result<sc::ObjectFile> back = sc::load_library_object(*library, library->entries[0]);
    ASSERT_TRUE(back.has_value()) << back.error().context;

    sc::ObjectFile want;
    want.object = asset_named(doc.desc(), gate);
    for (const spade::MaterialDesc& m : doc.desc().materials) {
        if (m.name == "library_gate") want.materials.push_back(m);
    }
    ASSERT_EQ(want.materials.size(), 1u) << "the gate names its own material";
    const Result<std::string> want_text = sc::object_to_yaml(want);
    const Result<std::string> back_text = sc::object_to_yaml(*back);
    ASSERT_TRUE(want_text.has_value()) << want_text.error().context;
    ASSERT_TRUE(back_text.has_value()) << back_text.error().context;
    EXPECT_EQ(*back_text, *want_text);
}

// Saving the same vehicle again gives <name>_2, beside the first, whose file
// and index entry are unchanged.
TEST(EditorLibrarySave, ATakenNameGoesToUnderscore2AndTheExistingBytesAreUnchanged) {
    std::string quad;
    const ed::SceneDocument doc = hover_with(default_item("Reference quad (test fixture)"), quad);
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    const Result<ed::LibrarySaved> first = ed::save_to_library(dir, doc.desc(), quad, kNoMeshRoots);
    ASSERT_TRUE(first.has_value()) << first.error().context;
    const std::string first_bytes = support::read_bytes(first->file);

    const Result<ed::LibrarySaved> second = ed::save_to_library(dir, doc.desc(), quad, kNoMeshRoots);
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(second->name, quad + "_2");
    EXPECT_NE(second->file, first->file);
    EXPECT_EQ(support::read_bytes(first->file), first_bytes) << "never overwritten";

    const Result<sc::Library> library = sc::load_library(dir / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    ASSERT_EQ(library->entries.size(), 2u);
    EXPECT_EQ(library->entries[0].name, quad);
    EXPECT_EQ(library->entries[0].file, first->file.filename().generic_string());
    EXPECT_EQ(library->entries[1].name, quad + "_2");
    const ed::LibraryLoad load = ed::load_library_items(dir / "library.yaml");
    EXPECT_TRUE(load.problems.empty()) << load.problems.front();
    EXPECT_EQ(load.items.size(), 2u);
}

// A file already where the item would go, even one the index doesn't list,
// is never overwritten: the item takes the next name.
TEST(EditorLibrarySave, AFileAlreadyThereIsNeverOverwritten) {
    std::string quad;
    const ed::SceneDocument doc = hover_with(default_item("Reference quad (test fixture)"), quad);
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    fs::create_directories(dir);
    const fs::path stray = dir / (quad + ".model.yaml");
    support::write_bytes(stray, "someone else's\n");

    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, doc.desc(), quad, kNoMeshRoots);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    EXPECT_EQ(saved->name, quad + "_2");
    EXPECT_NE(saved->file, stray);
    EXPECT_EQ(support::read_bytes(stray), "someone else's\n");
}

// A drone the CPU renderer can't draw (the hover quad has no parts) is still
// saved, without a thumbnail, and says why; its tile shows its name.
TEST(EditorLibrarySave, AnItemThatCantBeDrawnIsSavedWithoutAThumbnailAndSaysWhy) {
    const sc::SceneDesc scene = support::sample_scene();
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, scene, "hover_quad_0", kNoMeshRoots);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    EXPECT_TRUE(saved->thumbnail.empty());
    EXPECT_FALSE(saved->no_thumbnail.empty());

    const Result<sc::Library> library = sc::load_library(dir / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    ASSERT_EQ(library->entries.size(), 1u);
    EXPECT_TRUE(library->entries[0].thumbnail.empty());
}

// The project's index won't load: refused naming it, and nothing is written.
TEST(EditorLibrarySave, AnIndexThatWontLoadIsNamedAndNothingIsWritten) {
    std::string quad;
    const ed::SceneDocument doc = hover_with(default_item("Reference quad (test fixture)"), quad);
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    fs::create_directories(dir);
    support::write_bytes(dir / "library.yaml", "library_version: 1\nnonsense: 1\n");
    const std::map<std::string, std::string> before = contents(dir);

    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, doc.desc(), quad, kNoMeshRoots);
    ASSERT_FALSE(saved.has_value());
    EXPECT_NE(saved.error().context.find("library.yaml"), std::string::npos) << saved.error().context;
    EXPECT_EQ(contents(dir), before);
}

// A name the scene has no vehicle or asset of is refused by name, and no
// library is made.
TEST(EditorLibrarySave, ANameTheSceneDoesntHaveIsRefusedByName) {
    const sc::SceneDesc scene = support::sample_scene();
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";
    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, scene, "no_such", kNoMeshRoots);
    ASSERT_FALSE(saved.has_value());
    EXPECT_NE(saved.error().context.find("'no_such'"), std::string::npos) << saved.error().context;
    EXPECT_FALSE(fs::exists(dir));
}

// A drone with mesh parts, the library's Phantom with L7c1's props, is saved
// with its sidecars (the lead, 2026-10-27): each beside the model file under
// the content-named path its part gives, so the entry loads back equal, its
// meshes checked by their hashes, and places back.
TEST(EditorLibrarySave, ADroneWithMeshPartsIsSavedWithItsSidecarsAndLoadsBackEqual) {
    std::string phantom;
    const ed::SceneDocument doc = hover_with(default_item("Phantom 3-class quad"), phantom);
    ASSERT_FALSE(phantom.empty());
    const spade::vehicles::ModelType& placed =
        support::model_named(doc.desc(), vehicle_named(doc.desc(), phantom).model);
    std::vector<std::string> sidecars;
    for (const spade::vehicles::PartDesc& part : placed.parts) {
        if (part.mesh) sidecars.push_back(part.mesh_file);
    }
    ASSERT_FALSE(sidecars.empty()) << "L7c1's props are mesh parts";
    const support::TempDir tmp;
    const fs::path dir = tmp.path() / "library";

    const Result<ed::LibrarySaved> saved = ed::save_to_library(dir, doc.desc(), phantom, kNoMeshRoots);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    for (const std::string& s : sidecars) EXPECT_TRUE(fs::exists(dir / s)) << s;
    const ed::LibraryLoad load = ed::load_library_items(dir / "library.yaml");
    ASSERT_TRUE(load.problems.empty()) << load.problems.front();
    ASSERT_EQ(load.items.size(), 1u);
    ASSERT_TRUE(load.items[0].model.has_value());
    EXPECT_EQ(model_text(*load.items[0].model), model_text(placed));
    std::string again;
    (void)hover_with(load.items[0], again);
    EXPECT_FALSE(again.empty()) << "it places back";
}
