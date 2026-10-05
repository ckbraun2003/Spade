// Shared helpers for the editor's tests (docs/design/interface/plans/
// 2026-10-05-editor-plan.md names them). Each task adds the helpers it needs.

#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scene/scene_file.hpp"
#include "world/world_file.hpp"

namespace spade::sandbox::editor::test {

// assets/scenes/hover.scene.yaml: one quadrotor, "hover_quad_0", no assets.
[[nodiscard]] inline std::filesystem::path hover_scene() {
    return std::filesystem::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml";
}

[[nodiscard]] inline std::filesystem::path hover_world() {
    return std::filesystem::path(SPADE_ASSETS_DIR) / "worlds" / "hover.world.yaml";
}

// The hover scene, loaded. Fails the calling test if it does not load.
[[nodiscard]] inline scene::SceneDesc sample_scene() {
    Result<scene::SceneDesc> s = scene::load_scene_file(hover_scene());
    EXPECT_TRUE(s.has_value()) << (s ? "" : s.error().context);
    return s ? *s : scene::SceneDesc{};
}

[[nodiscard]] inline WorldDesc sample_world() {
    Result<WorldDesc> w = load_world_file(hover_world());
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? *w : WorldDesc{};
}

// The canonical text of a scene: equal text is an equal scene, bit for bit.
[[nodiscard]] inline std::string text_of(const scene::SceneDesc& s) {
    Result<std::string> t = scene::scene_to_yaml(s);
    return t ? *t : std::string("<scene_to_yaml failed: ") + t.error().context + ">";
}

// The canonical text of a world, as text_of() is for a scene.
[[nodiscard]] inline std::string world_text_of(const WorldDesc& w) {
    Result<std::string> t = world_to_yaml(w);
    return t ? *t : std::string("<world_to_yaml failed: ") + t.error().context + ">";
}

// A unit box asset (half extents 0.5) at the identity pose, colliding, with no visual.
[[nodiscard]] inline scene::SceneAsset asset_named(std::string name) {
    scene::SceneAsset a;
    a.name = std::move(name);
    a.collider.transforms.push_back(SdfTransform{});
    SdfNode box{};
    box.kind = static_cast<uint32_t>(SdfPrim::box);
    box.op = static_cast<uint32_t>(SdfOp::none);
    box.params = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f);
    a.collider.nodes.push_back(box);
    return a;
}

[[nodiscard]] inline scene::SceneDesc with_asset(scene::SceneDesc s, std::string name) {
    s.assets.push_back(asset_named(std::move(name)));
    return s;
}

// The hover scene's quadrotor model, renamed "quad": four rotors, a drag body
// and an IMU mount, so every part kind has an index 0.
[[nodiscard]] inline vehicles::ModelType quad_model() {
    vehicles::ModelType m = sample_scene().models.at(0);
    m.name = "quad";
    return m;
}

// The hover scene with `m` as its only model and its one vehicle, "quad_0", flying it.
[[nodiscard]] inline scene::SceneDesc scene_with(const vehicles::ModelType& m) {
    scene::SceneDesc s = sample_scene();
    s.models = {m};
    s.vehicles.at(0).name = "quad_0";
    s.vehicles.at(0).model = m.name;
    return s;
}

// Two vehicles, "quad_0" and "quad_1", sharing one model named `name`, as a
// scene from a file may (the editor never makes one).
[[nodiscard]] inline scene::SceneDesc two_vehicles_sharing(std::string name) {
    vehicles::ModelType m = quad_model();
    m.name = std::move(name);
    scene::SceneDesc s = scene_with(m);
    scene::SceneVehicle twin = s.vehicles.at(0);
    twin.name = "quad_1";
    twin.start.pos.x += 2.0f;
    s.vehicles.push_back(twin);
    return s;
}

[[nodiscard]] inline const vehicles::ModelType& model_named(const scene::SceneDesc& s, std::string_view name) {
    for (const auto& m : s.models) {
        if (m.name == name) return m;
    }
    ADD_FAILURE() << "no model named " << name;
    return s.models.at(0);
}

// ---- files (Task 5 onward) ----

// A fresh folder for one test, named after it and removed when it goes out of
// scope. Tests may run in parallel processes, so the name is the test's own.
class TempDir {
  public:
    TempDir() {
        const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
        path_ = std::filesystem::temp_directory_path() /
                ("spade_editor_" + std::string(info->test_suite_name()) + "_" + info->name());
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

  private:
    std::filesystem::path path_;
};

[[nodiscard]] inline std::string read_bytes(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

inline void write_bytes(const std::filesystem::path& p, std::string_view text) {
    std::filesystem::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

inline void make_read_only(const std::filesystem::path& p) {
    std::filesystem::permissions(p, std::filesystem::perms::owner_write | std::filesystem::perms::group_write |
                                        std::filesystem::perms::others_write,
                                 std::filesystem::perm_options::remove);
}

inline void make_writable(const std::filesystem::path& p) {
    std::filesystem::permissions(p, std::filesystem::perms::owner_write, std::filesystem::perm_options::add);
}

// A file's bytes without its "# provenance:" lines, which a golden carries
// above the generated text and scene_to_yaml() never writes.
[[nodiscard]] inline std::string canonical_bytes(const std::filesystem::path& p) {
    const std::string all = read_bytes(p);
    std::string out;
    std::size_t at = 0;
    while (at < all.size()) {
        std::size_t end = all.find('\n', at);
        end = end == std::string::npos ? all.size() : end + 1;
        if (all.compare(at, 13, "# provenance:") != 0) out.append(all, at, end - at);
        at = end;
    }
    return out;
}

// A committed scene file and the folder it lives under (assets/ or
// tests/golden/), which also holds the world it names.
struct CommittedScene {
    std::filesystem::path root;
    std::filesystem::path file;
};

// Every committed scene file: assets/scenes/ and tests/golden/scenes/.
[[nodiscard]] inline std::vector<CommittedScene> committed_scene_files() {
    std::vector<CommittedScene> out;
    for (const std::filesystem::path& root : {std::filesystem::path(SPADE_ASSETS_DIR),
                                             std::filesystem::path(SPADE_GOLDEN_DIR)}) {
        for (const auto& entry : std::filesystem::directory_iterator(root / "scenes")) {
            const std::string name = entry.path().filename().string();
            if (name.size() > 11 && name.ends_with(".scene.yaml")) out.push_back({root, entry.path()});
        }
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.file < b.file; });
    return out;
}

// Copies a committed scene and the world it names into `to`, keeping their
// places relative to their root, so the scene's world path still resolves.
// Returns the copy of the scene.
[[nodiscard]] inline std::filesystem::path mirror(const CommittedScene& c, const std::filesystem::path& to) {
    const Result<scene::SceneDesc> s = scene::load_scene_file(c.file);
    EXPECT_TRUE(s.has_value()) << c.file;
    const std::filesystem::path world = (c.file.parent_path() / (s ? s->world.file : "")).lexically_normal();
    const std::filesystem::path scene_copy = to / c.file.lexically_relative(c.root);
    const std::filesystem::path world_copy = to / world.lexically_relative(c.root);
    write_bytes(scene_copy, read_bytes(c.file));
    write_bytes(world_copy, read_bytes(world));
    return scene_copy;
}

}  // namespace spade::sandbox::editor::test
