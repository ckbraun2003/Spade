// N15: the editor's glTF import (sandbox/editor_import.hpp; the plan,
// tasks/n15-plan.md, approved by the lead 2026-10-22). Core reads the file
// (read_gltf) and writes the sidecar (save_mesh_file); the editor mints the
// mesh ref, refuses a clash, and stages the sidecar in its own folder until
// Save copies it beside the scene (D1).
//
// THE REF RULE (the lead, 2026-10-20): imported/<stem>, each character outside
// [A-Za-z0-9_-] (a UTF-8 code point at a time) mapped to '_', then
// world::mesh_ref_problem() before anything is written; a stem with no letter
// or digit is refused; a clash is refused naming the existing ref, never
// renamed (D3).
//
// Every fixture is built here, as a .gltf with its buffer inline in base64:
// cubes whose corners are numbered x + 2y + 4z, with test_mesh_import.cpp's
// outward triangles.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "../sandbox/editor_import.hpp"
#include "../sandbox/editor_workspace.hpp"
#include "../sandbox/end_goal_checks.hpp"
#include "core/path_text.hpp"
#include "editor_test_support.hpp"
#include "render/mesh_resolver.hpp"
#include "render/scene.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_part.hpp"
#include "world/builder.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"
#include "world/mesh_ref.hpp"

namespace {

namespace fs = std::filesystem;
namespace ed = spade::sandbox::editor;
namespace eg = spade::sandbox::end_goal;
using spade::test::TempDir;

// One cube of the fixture: its mesh's name (empty: none), where its corner 0
// sits, its side, and how many of its 12 triangles are left off the end (an
// open solid).
struct Cube {
    std::string name;
    glm::vec3 at{0.0f};
    float size = 1.0f;
    int drop = 0;
};

constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
    {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
};

void put(std::string& b, const void* p, std::size_t n) { b.append(static_cast<const char*>(p), n); }

[[nodiscard]] std::string base64(std::string_view bytes) {
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        const uint32_t a = static_cast<unsigned char>(bytes[i]);
        const uint32_t b = i + 1 < bytes.size() ? static_cast<unsigned char>(bytes[i + 1]) : 0u;
        const uint32_t c = i + 2 < bytes.size() ? static_cast<unsigned char>(bytes[i + 2]) : 0u;
        const uint32_t v = (a << 16) | (b << 8) | c;
        out += kAlphabet[(v >> 18) & 63u];
        out += kAlphabet[(v >> 12) & 63u];
        out += i + 1 < bytes.size() ? kAlphabet[(v >> 6) & 63u] : '=';
        out += i + 2 < bytes.size() ? kAlphabet[v & 63u] : '=';
    }
    return out;
}

[[nodiscard]] std::string num(float v) {
    char text[32];
    std::snprintf(text, sizeof text, "%.9g", static_cast<double>(v));
    return text;
}

// A .gltf at `path`: one node per cube, in order, under one scene.
void write_gltf(const fs::path& path, const std::vector<Cube>& cubes) {
    std::string bin;
    std::string views;
    std::string accessors;
    std::string meshes;
    std::string nodes;
    std::string roots;
    for (std::size_t k = 0; k < cubes.size(); ++k) {
        const Cube& c = cubes[k];
        const std::size_t positions_at = bin.size();
        for (uint32_t v = 0; v < 8; ++v) {
            const float p[3] = {c.at.x + c.size * static_cast<float>(v & 1u),
                                c.at.y + c.size * static_cast<float>((v >> 1) & 1u),
                                c.at.z + c.size * static_cast<float>((v >> 2) & 1u)};
            put(bin, p, sizeof p);
        }
        const std::size_t indices_at = bin.size();
        const int triangles = 12 - c.drop;
        for (int t = 0; t < triangles; ++t) put(bin, kCubeTriangles[t], sizeof kCubeTriangles[t]);
        const std::string sep = k == 0 ? "" : ",";
        views += sep + "{\"buffer\":0,\"byteOffset\":" + std::to_string(positions_at) + ",\"byteLength\":96}," +
                 "{\"buffer\":0,\"byteOffset\":" + std::to_string(indices_at) +
                 ",\"byteLength\":" + std::to_string(triangles * 12) + "}";
        accessors += sep + "{\"bufferView\":" + std::to_string(2 * k) +
                     ",\"componentType\":5126,\"count\":8,\"type\":\"VEC3\",\"min\":[" + num(c.at.x) + "," +
                     num(c.at.y) + "," + num(c.at.z) + "],\"max\":[" + num(c.at.x + c.size) + "," +
                     num(c.at.y + c.size) + "," + num(c.at.z + c.size) + "]}," + "{\"bufferView\":" +
                     std::to_string(2 * k + 1) + ",\"componentType\":5125,\"count\":" +
                     std::to_string(triangles * 3) + ",\"type\":\"SCALAR\"}";
        meshes += sep + "{" + (c.name.empty() ? std::string() : "\"name\":\"" + c.name + "\",") +
                  "\"primitives\":[{\"attributes\":{\"POSITION\":" + std::to_string(2 * k) +
                  "},\"indices\":" + std::to_string(2 * k + 1) + "}]}";
        nodes += sep + "{\"mesh\":" + std::to_string(k) + "}";
        roots += sep + std::to_string(k);
    }
    const std::string doc = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[" + roots +
                            "]}],\"nodes\":[" + nodes + "],\"meshes\":[" + meshes + "],\"accessors\":[" +
                            accessors + "],\"bufferViews\":[" + views + "],\"buffers\":[{\"byteLength\":" +
                            std::to_string(bin.size()) + ",\"uri\":\"data:application/octet-stream;base64," +
                            base64(bin) + "\"}]}";
    std::ofstream out(path, std::ios::binary);
    out << doc;
}

// Every file under `dir`, as text.
[[nodiscard]] std::vector<std::string> files_under(const fs::path& dir) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::exists(dir, ec)) return out;
    for (const fs::directory_entry& e : fs::recursive_directory_iterator(dir)) {
        if (e.is_regular_file()) out.push_back(e.path().string());
    }
    return out;
}

// A temp folder with a scene folder and a staging folder in it.
struct Bench {
    TempDir dir{"editor-import"};
    fs::path scene_dir = dir.path() / "scene";
    fs::path staging = dir.path() / "staging";
    spade::scene::SceneDesc scene{};
    spade::WorldDesc world{};
    Bench() {
        fs::create_directories(scene_dir);
        fs::create_directories(staging);
    }
    [[nodiscard]] fs::path gltf(std::string_view name, const std::vector<Cube>& cubes) const {
        const fs::path p = dir.path() / fs::path(std::u8string(name.begin(), name.end()));
        write_gltf(p, cubes);
        return p;
    }
    [[nodiscard]] spade::Result<ed::ImportedMesh> import(const fs::path& file) const {
        return ed::import_gltf_object(file, staging, scene, world, scene_dir);
    }
};

}  // namespace

TEST(EditorImport, ARefIsImportedSlashTheStemWithEveryOtherCharacterAnUnderscore) {
    const auto ref = [](std::string_view file) {
        const spade::Result<std::string> r = ed::mint_ref(fs::path(std::u8string(file.begin(), file.end())));
        return r ? *r : "<refused: " + r.error().context + ">";
    };
    EXPECT_EQ(ref("C:/models/Duck.gltf"), "imported/Duck");
    EXPECT_EQ(ref("my model.v2.glb"), "imported/my_model_v2");
    EXPECT_EQ(ref("guard-ring_3.gltf"), "imported/guard-ring_3");
    EXPECT_EQ(ref("w\xC3\xBCrfel.gltf"), "imported/w_rfel") << "one '_' per code point, not per byte";
}

TEST(EditorImport, ADeviceNameOrAStemWithNoLetterOrDigitIsRefusedAndNothingIsWritten) {
    Bench b;
    const spade::Result<ed::ImportedMesh> con = b.import(b.gltf("con.gltf", {{"crate"}}));
    ASSERT_FALSE(con.has_value());
    EXPECT_NE(con.error().context.find(std::string(spade::mesh_ref_problem("imported/con"))), std::string::npos)
        << "in the rule's own words: " << con.error().context;
    const spade::Result<ed::ImportedMesh> marks = b.import(b.gltf("%%%.gltf", {{"crate"}}));
    ASSERT_FALSE(marks.has_value());
    EXPECT_NE(marks.error().context.find("no letter or digit"), std::string::npos) << marks.error().context;
    EXPECT_TRUE(files_under(b.staging).empty());
}

// The sidecar is staged under the editor's folder, never beside the scene
// (D1), and loads back holding what read_gltf read; import-named holds.
TEST(EditorImport, AnObjectIsStagedAsASidecarThatLoadsBackWithTheReadersCounts) {
    Bench b;
    const fs::path file = b.gltf("crate.gltf", {{"crate", glm::vec3(-0.5f, 0.0f, -0.5f), 1.0f}});
    const spade::Result<ed::ImportedMesh> got = b.import(file);
    ASSERT_TRUE(got.has_value()) << got.error().context;
    EXPECT_EQ(got->name, "crate");
    EXPECT_EQ(got->ref, "imported/crate");
    EXPECT_EQ(got->sidecar, b.staging / "imported" / "crate.spmesh");
    EXPECT_EQ(got->source, spade::path_text(file));
    EXPECT_TRUE(files_under(b.scene_dir).empty()) << "nothing beside the scene until Save";

    const spade::Result<std::vector<spade::MeshObject>> read = spade::read_gltf(file);
    ASSERT_TRUE(read.has_value()) << read.error().context;
    ASSERT_EQ(read->size(), 1u);
    const spade::Result<spade::LoadedMesh> back = spade::load_mesh_file(got->sidecar, got->mesh->content_hash());
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->provenance.source, got->source);

    eg::ImportShown shown;
    shown.shown_name = got->name;
    shown.mesh_name = "crate";
    shown.file = spade::path_text(file);
    shown.shown_source = got->source;
    shown.sidecar_loaded = true;
    shown.gltf_vertices = read->front().mesh->positions().size();
    shown.gltf_triangles = read->front().mesh->triangle_count();
    shown.sidecar_vertices = back->mesh->positions().size();
    shown.sidecar_triangles = back->mesh->triangle_count();
    const eg::Verdict named = eg::import_named(shown);
    EXPECT_TRUE(named.ok) << named.detail;
}

// read_gltf names an unnamed mesh "node[0]"; the editor names it by the file.
TEST(EditorImport, AnUnnamedMeshIsNamedByTheFilesStem) {
    Bench b;
    const spade::Result<ed::ImportedMesh> got = b.import(b.gltf("box.gltf", {{""}}));
    ASSERT_TRUE(got.has_value()) << got.error().context;
    EXPECT_EQ(got->name, "box");
    EXPECT_EQ(got->ref, "imported/box");
}

// make_mesh() refuses an open solid; the refusal is the reader's, prefixed
// by the import, and nothing is staged.
TEST(EditorImport, AnOpenSolidIsRefusedInTheReadersWordsAndNothingIsStaged) {
    Bench b;
    const fs::path file = b.gltf("lid_off.gltf", {{"lid_off", glm::vec3(0.0f), 1.0f, 1}});
    const spade::Result<ed::ImportedMesh> got = b.import(file);
    ASSERT_FALSE(got.has_value());
    const spade::Result<std::vector<spade::MeshObject>> read = spade::read_gltf(file);
    ASSERT_FALSE(read.has_value());
    EXPECT_NE(got.error().context.find(read.error().context), std::string::npos) << got.error().context;
    EXPECT_TRUE(files_under(b.staging).empty());
}

// An object is one mesh (D2): a file holding several is refused naming the
// count, pointing at a drone's parts.
TEST(EditorImport, AFileOfSeveralMeshesIsRefusedAsAnObjectNamingTheCount) {
    Bench b;
    const spade::Result<ed::ImportedMesh> got =
        b.import(b.gltf("pair.gltf", {{"left"}, {"right", glm::vec3(2.0f, 0.0f, 0.0f)}}));
    ASSERT_FALSE(got.has_value());
    EXPECT_NE(got.error().context.find("holds 2 meshes"), std::string::npos) << got.error().context;
    EXPECT_NE(got.error().context.find("parts"), std::string::npos) << got.error().context;
    EXPECT_TRUE(files_under(b.staging).empty());
}

// D3: a ref already in use is refused by name, never renamed: an asset's
// visual, a world prop's, a part's sidecar, a file beside the scene, or an
// earlier import staged this session. Nothing is written.
TEST(EditorImport, ARefAlreadyInUseIsRefusedNamingItAndNeverRenamed) {
    const auto expect_clash = [](const Bench& b, const fs::path& file, std::string_view holder) {
        const spade::Result<ed::ImportedMesh> got = b.import(file);
        ASSERT_FALSE(got.has_value()) << "imported as " << got->ref;
        EXPECT_NE(got.error().context.find("'imported/crate'"), std::string::npos) << got.error().context;
        EXPECT_NE(got.error().context.find(holder), std::string::npos) << got.error().context;
    };
    {
        Bench b;
        spade::scene::SceneAsset a;
        a.name = "old crate";
        a.visual.mesh_ref = "imported/crate";
        b.scene.assets.push_back(a);
        expect_clash(b, b.gltf("crate.gltf", {{"crate"}}), "asset 'old crate'");
        EXPECT_TRUE(files_under(b.staging).empty());
    }
    {
        Bench b;
        spade::PropDesc prop;
        prop.mesh_ref = "imported/crate";
        b.world.props.push_back(prop);
        expect_clash(b, b.gltf("crate.gltf", {{"crate"}}), "the world");
    }
    {
        Bench b;
        spade::vehicles::ModelType m;
        m.name = "quad";
        spade::vehicles::PartDesc part;
        part.name = "shell";
        part.mesh_file = "imported/crate.spmesh";
        m.parts.push_back(part);
        b.scene.models.push_back(m);
        expect_clash(b, b.gltf("crate.gltf", {{"crate"}}), "part 'shell'");
    }
    {
        Bench b;
        fs::create_directories(b.scene_dir / "imported");
        std::ofstream(b.scene_dir / "imported" / "crate.spmesh") << "x";
        expect_clash(b, b.gltf("crate.gltf", {{"crate"}}), "beside the scene");
    }
    {
        Bench b;
        const fs::path file = b.gltf("crate.gltf", {{"crate"}});
        ASSERT_TRUE(b.import(file).has_value());
        expect_clash(b, file, "imported already");
        EXPECT_EQ(files_under(b.staging).size(), 1u);
    }
}

// ===========================================================================
// I2: AN OBJECT IMPORT IN THE WORKSPACE. Load from file takes a .gltf or .glb
// as an "imported" library item, armed; a click places it resting on the
// picked surface by its mesh's bounds, as one undoable asset with a visual
// (the world's first material) and no collider. The view finds its sidecar in
// the staging folder, the resolver's second root (D1), so nothing is written
// beside the scene; the inspector shows the path imported.
// ===========================================================================

namespace {

namespace support = spade::sandbox::editor::test;

[[nodiscard]] std::unique_ptr<ed::EditorWorkspace> hover_workspace() {
    spade::Result<ed::OpenedScene> opened = ed::open_scene(support::hover_scene());
    EXPECT_TRUE(opened.has_value()) << (opened ? "" : opened.error().context);
    spade::Result<std::unique_ptr<ed::EditorWorkspace>> w = ed::EditorWorkspace::create(std::move(*opened));
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? std::move(*w) : nullptr;
}

void run(ed::EditorWorkspace& w, ed::Command c) {
    w.panels().commands.push_back(c);
    w.apply_panels();
}

// The view's roots, as the session passes them: the scene's folder, then the
// staging folder.
[[nodiscard]] spade::render::ResolvedMeshes resolved(const ed::EditorWorkspace& w) {
    const std::vector<fs::path> roots{w.documents().scene.path.parent_path(), w.import_staging()};
    return spade::render::resolve_meshes(w.composed().world, roots);
}

[[nodiscard]] std::size_t statics_drawn(const ed::EditorWorkspace& w) {
    const spade::render::ResolvedMeshes m = resolved(w);
    const spade::Result<spade::render::RenderScene> s = spade::render::scene_from_world(w.composed().world, m.meshes);
    EXPECT_TRUE(s.has_value()) << (s ? "" : s.error().context);
    return s ? s->statics.size() : 0u;
}

}  // namespace

TEST(EditorImport, LoadFromFileImportsAGltfAsAnArmedItemThatRestsWhereTheClickMeetsTheGround) {
    TempDir t{"editor-import-place"};
    const fs::path file = t.path() / "crate.gltf";
    write_gltf(file, {{"crate", glm::vec3(-0.25f, -0.1f, -0.25f), 0.5f}});
    auto w = hover_workspace();
    ASSERT_NE(w, nullptr);
    const fs::path scene_dir = w->documents().scene.path.parent_path();
    const std::size_t drawn_before = statics_drawn(*w);

    w->panels().item_path = spade::path_text(file);
    run(*w, ed::Command::load_item);
    ASSERT_TRUE(w->panels().armed.has_value()) << w->panels().status;
    const ed::LibraryItem& item = w->panels().library.at(*w->panels().armed);
    EXPECT_EQ(item.name, "crate");
    EXPECT_EQ(item.category, "imported");
    EXPECT_TRUE(fs::exists(w->import_staging() / "imported" / "crate.spmesh"));
    EXPECT_FALSE(fs::exists(scene_dir / "imported")) << "nothing beside the scene until Save";

    w->click(glm::vec3(4.0f, 5.0f, 4.0f), glm::vec3(0.0f, -1.0f, 0.0f));
    ASSERT_EQ(w->documents().scene.desc().assets.size(), 1u) << w->panels().status;
    const spade::scene::SceneAsset& a = w->documents().scene.desc().assets[0];
    EXPECT_EQ(a.name, "crate");
    EXPECT_EQ(a.visual.mesh_ref, "imported/crate");
    EXPECT_EQ(a.visual.material, w->documents().world.desc().materials.front().name);
    EXPECT_TRUE(a.collider.nodes.empty()) << "a mesh obstacle's collider waits for N9";
    EXPECT_NEAR(a.pose.position.x, 4.0f, 1e-4f);
    EXPECT_NEAR(a.pose.position.z, 4.0f, 1e-4f);
    EXPECT_NEAR(a.pose.position.y, 0.1f, 1e-3f) << "its lowest vertex, 0.1 m below its origin, on the ground";
    EXPECT_EQ(w->panels().selection, (ed::Selection{ed::Selection::Kind::object, "crate"}));
    w->refresh_panels();
    EXPECT_EQ(w->panels().selected_source, spade::path_text(file));

    const spade::render::ResolvedMeshes m = resolved(*w);
    EXPECT_TRUE(m.unresolved.empty()) << m.unresolved.front().why;
    ASSERT_EQ(m.from_fallback.size(), 1u) << "found in the staging folder, root 1";
    EXPECT_EQ(m.from_fallback.front().ref, "imported/crate");
    EXPECT_EQ(m.from_fallback.front().root, 1u);
    EXPECT_EQ(statics_drawn(*w), drawn_before + 1) << "one drawable for the import";

    run(*w, ed::Command::undo);
    EXPECT_TRUE(w->documents().scene.desc().assets.empty());
}

TEST(EditorImport, ARefusedImportArmsNothingAndSaysWhyInTheReadersWords) {
    TempDir t{"editor-import-refused"};
    const fs::path file = t.path() / "lid_off.gltf";
    write_gltf(file, {{"lid_off", glm::vec3(0.0f), 1.0f, 1}});
    auto w = hover_workspace();
    ASSERT_NE(w, nullptr);
    w->refresh_panels();
    const std::size_t items = w->panels().library.size();
    w->panels().item_path = spade::path_text(file);
    run(*w, ed::Command::load_item);
    EXPECT_FALSE(w->panels().armed.has_value());
    EXPECT_EQ(w->panels().library.size(), items);
    EXPECT_NE(w->panels().status.find("object 'lid_off'"), std::string::npos) << w->panels().status;
}

// The staging folder is the workspace's: made at the first import, removed
// with the workspace, so an unsaved import leaves nothing behind.
TEST(EditorImport, TheStagingFolderGoesWithTheWorkspace) {
    TempDir t{"editor-import-staging"};
    const fs::path file = t.path() / "crate.gltf";
    write_gltf(file, {{"crate"}});
    auto w = hover_workspace();
    ASSERT_NE(w, nullptr);
    w->panels().item_path = spade::path_text(file);
    run(*w, ed::Command::load_item);
    const fs::path staging = w->import_staging();
    ASSERT_TRUE(fs::exists(staging / "imported" / "crate.spmesh")) << w->panels().status;
    auto other = hover_workspace();
    ASSERT_NE(other, nullptr);
    EXPECT_NE(other->import_staging(), staging) << "each workspace stages in its own folder";
    w.reset();
    EXPECT_FALSE(fs::exists(staging));
}
