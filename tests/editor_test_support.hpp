// Shared helpers for the editor's tests (docs/design/interface/plans/
// 2026-10-05-editor-plan.md names them). Each task adds the helpers it needs.

#pragma once

#include <filesystem>
#include <cstdint>
#include <string>
#include <utility>

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

}  // namespace spade::sandbox::editor::test
