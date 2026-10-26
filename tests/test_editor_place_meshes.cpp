// Placing a drone with mesh parts in the editor (L7b0 of the realistic drones
// plan, docs/design/core/plans/2026-10-26-l7-realistic-drones-draft.md).
//
// The library drones are about to wear meshes (L7b2), and the editor places a
// library drone through AddModelFile. That edit now goes through
// scene::add_model_with_meshes(), which names each part's sidecar under the
// scene's imported/ folder by its mesh. Before Save, the workspace writes each
// of those sidecars that no folder holds yet into its staging folder, from
// the mesh in memory, so Save As carries them beside the scene (N15's copy)
// and the saved scene opens with them. The command line's --save writes them
// beside its target, through the same function (scene::write_placed_meshes()).

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <string>

#include <glm/glm.hpp>

#include "../sandbox/editor_cli.hpp"
#include "../sandbox/editor_workspace.hpp"
#include "core/path_text.hpp"
#include "editor_test_support.hpp"
#include "scene/model_file.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace fs = std::filesystem;

using MeshPtr = std::shared_ptr<const spade::TriangleMesh>;

// A cube of side `a` centred on the origin, counter-clockwise from outside.
[[nodiscard]] MeshPtr cube(double a) {
    const double h = 0.5 * a;
    spade::MeshInput m;
    m.unit = spade::MeshUnit::metre;
    for (int i = 0; i < 8; ++i) m.positions.emplace_back((i & 1) ? h : -h, (i & 2) ? h : -h, (i & 4) ? h : -h);
    m.indices = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
    const auto r = spade::make_mesh(m, "cube");
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().context);
    return r ? *r : nullptr;
}

// The reference quad's model file with one mesh part, "shell", its sidecar
// beside it as meshes/shell.spmesh: a drone file as L7b2's will be.
[[nodiscard]] fs::path write_meshed_drone(const fs::path& dir, const MeshPtr& mesh) {
    const auto reference =
        spade::scene::load_model_file(fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml");
    EXPECT_TRUE(reference.has_value()) << (reference ? "" : reference.error().context);
    if (!reference) return {};
    spade::scene::ModelFile file = *reference;
    spade::vehicles::PartDesc shell{};
    shell.name = "shell";
    shell.role = "body";
    shell.mount_pos = {0.0f, 0.05f, 0.0f};
    shell.mesh = mesh;
    shell.mesh_file = "meshes/shell.spmesh";
    shell.min_feature = 0.004f;
    file.model.parts.push_back(shell);
    const auto text = spade::scene::model_to_yaml(file);
    EXPECT_TRUE(text.has_value()) << (text ? "" : text.error().context);
    if (!text) return {};
    fs::create_directories(dir / "meshes");
    const auto saved = spade::save_mesh_file(dir / "meshes" / "shell.spmesh", *mesh);
    EXPECT_TRUE(saved.has_value()) << (saved ? "" : saved.error().context);
    const fs::path path = dir / "meshed.model.yaml";
    support::write_bytes(path, *text);
    return path;
}

[[nodiscard]] std::unique_ptr<ed::EditorWorkspace> hover_workspace() {
    spade::Result<ed::OpenedScene> opened = ed::open_scene(support::hover_scene());
    EXPECT_TRUE(opened.has_value()) << (opened ? "" : opened.error().context);
    if (!opened) return nullptr;
    spade::Result<std::unique_ptr<ed::EditorWorkspace>> w = ed::EditorWorkspace::create(std::move(*opened));
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? std::move(*w) : nullptr;
}

void run(ed::EditorWorkspace& w, ed::Command c) {
    w.panels().commands.push_back(c);
    w.apply_panels();
}

[[nodiscard]] const spade::vehicles::PartDesc* part_named(const spade::vehicles::ModelType& m, std::string_view n) {
    for (const spade::vehicles::PartDesc& p : m.parts) {
        if (p.name == n) return &p;
    }
    return nullptr;
}

}  // namespace

TEST(EditorPlaceMeshes, PlacingALibraryDroneWithMeshPartsStagesItsSidecarsAndSaveCopiesThem) {
    support::TempDir t;
    const MeshPtr shell = cube(0.08);
    ASSERT_NE(shell, nullptr);
    const fs::path drone = write_meshed_drone(t.path(), shell);
    ASSERT_FALSE(drone.empty());
    auto w = hover_workspace();
    ASSERT_NE(w, nullptr);

    // Placed: the model's part names its sidecar under the scene's imported/.
    const std::size_t vehicles = w->documents().scene.desc().vehicles.size();
    w->panels().item_path = spade::path_text(drone);
    run(*w, ed::Command::load_item);
    w->click(glm::vec3(4.0f, 5.0f, 4.0f), glm::vec3(0.0f, -1.0f, 0.0f));
    ASSERT_EQ(w->documents().scene.desc().vehicles.size(), vehicles + 1) << w->panels().status;
    const std::string sidecar = spade::scene::placed_mesh_file(*shell);
    {
        const spade::scene::SceneDesc& s = w->documents().scene.desc();
        const spade::vehicles::PartDesc* p = part_named(support::model_named(s, s.vehicles.back().model), "shell");
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->mesh_file, sidecar);
        ASSERT_NE(p->mesh, nullptr);
        EXPECT_EQ(p->mesh->content_hash(), shell->content_hash()) << "the mesh the drone's file loaded";
    }

    // Save As carries the sidecar beside the scene.
    const fs::path mine = t.path() / "out" / "mine.scene.yaml";
    fs::create_directories(mine.parent_path());
    w->panels().path = spade::path_text(mine);
    run(*w, ed::Command::save_as);
    ASSERT_TRUE(fs::exists(mine)) << w->panels().status;
    const fs::path beside = mine.parent_path() / fs::path(sidecar);
    ASSERT_TRUE(fs::exists(beside)) << spade::path_text(beside);
    const spade::Result<spade::LoadedMesh> carried = spade::load_mesh_file(beside, shell->content_hash());
    EXPECT_TRUE(carried.has_value()) << (carried ? "" : carried.error().context);

    // The saved scene opens with the drone's mesh.
    const spade::Result<ed::OpenedScene> reopened = ed::open_scene(mine);
    ASSERT_TRUE(reopened.has_value()) << reopened.error().context;
    const spade::scene::SceneDesc& back = reopened->scene.desc();
    const spade::vehicles::PartDesc* p = part_named(support::model_named(back, back.vehicles.back().model), "shell");
    ASSERT_NE(p, nullptr);
    ASSERT_NE(p->mesh, nullptr);
    EXPECT_EQ(p->mesh->content_hash(), shell->content_hash());
}

TEST(EditorPlaceMeshes, TheCommandLinesSaveWritesAPlacedDronesSidecarsBesideTheScene) {
    support::TempDir t;
    const MeshPtr shell = cube(0.08);
    ASSERT_NE(shell, nullptr);
    const fs::path drone = write_meshed_drone(t.path() / "drone", shell);
    ASSERT_FALSE(drone.empty());
    const fs::path copy = support::mirror({fs::path(SPADE_ASSETS_DIR), support::hover_scene()}, t.path());
    const spade::Result<spade::scene::SceneDesc> base = spade::scene::load_scene_file(copy);
    ASSERT_TRUE(base.has_value()) << base.error().context;
    ASSERT_FALSE(base->vehicles.empty());

    // --apply's add-model-file places it; --save writes the scene elsewhere.
    spade::scene::SceneVehicle placed{"meshed drone", "", base->vehicles.front().start};
    placed.start.pos.x += 2.0f;
    const spade::Result<std::string> line = ed::format_edit(ed::AddModelFile{drone.generic_string(), placed});
    ASSERT_TRUE(line.has_value()) << line.error().context;
    const fs::path edits = t.path() / "edits.txt";
    support::write_bytes(edits, *line + "\n");
    const fs::path mine = t.path() / "out" / "mine.scene.yaml";
    fs::create_directories(mine.parent_path());
    std::ostringstream out;
    std::ostringstream err;
    const int code = ed::edit_cli({copy.string(), "--apply", edits.string(), "--save", mine.string()}, out, err);
    ASSERT_EQ(code, 0) << err.str();

    // The sidecar is beside the scene, and the scene opens with the mesh.
    const fs::path beside = mine.parent_path() / fs::path(spade::scene::placed_mesh_file(*shell));
    ASSERT_TRUE(fs::exists(beside)) << spade::path_text(beside);
    const spade::Result<ed::OpenedScene> reopened = ed::open_scene(mine);
    ASSERT_TRUE(reopened.has_value()) << reopened.error().context;
    const spade::scene::SceneDesc& back = reopened->scene.desc();
    const spade::vehicles::PartDesc* p = part_named(support::model_named(back, back.vehicles.back().model), "shell");
    ASSERT_NE(p, nullptr);
    ASSERT_NE(p->mesh, nullptr);
    EXPECT_EQ(p->mesh->content_hash(), shell->content_hash());
}
