// Shared helpers for the editor's tests (docs/design/interface/plans/
// 2026-10-05-editor-plan.md names them). Each task adds the helpers it needs.

#pragma once

#include <filesystem>
#include <cstdint>
#include <string>
#include <string_view>
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

}  // namespace spade::sandbox::editor::test
