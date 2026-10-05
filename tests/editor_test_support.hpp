// Shared helpers for the editor's tests (docs/design/interface/plans/
// 2026-10-05-editor-plan.md names them). Each task adds the helpers it needs.

#pragma once

#include <filesystem>
#include <string>

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

}  // namespace spade::sandbox::editor::test
