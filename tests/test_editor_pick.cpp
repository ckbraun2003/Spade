// What a click in the view selects (editor plan Task 10, step b).
//
// The ray marches the composed world's SDF. A hit belongs to the asset whose
// posed collider is at the hit point; a hit on the world's own terrain belongs
// to nothing. A vehicle is picked within its reach: its proxy radius, or its
// rotors' reach when that is larger, so a click on an arm picks the drone.
// The nearest of the two wins. While the scene runs, a vehicle is picked where
// the run has it, not where it started.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "../sandbox/editor_library.hpp"
#include "../sandbox/editor_pick.hpp"
#include "../sandbox/editor_run.hpp"
#include "editor_test_support.hpp"
#include "scene/compose.hpp"
#include "world/sdf.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::scene::ComposedScene;
using spade::scene::SceneDesc;

const glm::vec3 kForward(0.0f, 0.0f, -1.0f);

// The hover scene with a unit box, named "box", at `at`.
[[nodiscard]] SceneDesc with_box(glm::vec3 at) {
    ed::SceneDocument doc = ed::make_scene_document(support::sample_scene(), {});
    spade::SdfPose pose;
    pose.position = at;
    for (const ed::LibraryItem& item : ed::builtin_library()) {
        if (item.name == "box") {
            EXPECT_TRUE(ed::place(doc, item, pose).has_value());
        }
    }
    return doc.desc();
}

[[nodiscard]] ComposedScene composed(const SceneDesc& s) {
    const spade::Result<ComposedScene> c = spade::scene::compose(s, support::sample_world());
    EXPECT_TRUE(c.has_value()) << (c ? "" : c.error().context);
    return c ? *c : ComposedScene{};
}

[[nodiscard]] glm::vec3 quad_start(const SceneDesc& s) { return s.vehicles.at(0).start.pos; }

const std::optional<std::string> kNothing = std::nullopt;

}  // namespace

TEST(EditorPick, ARayAtABoxPicksIt) {
    const SceneDesc s = with_box(glm::vec3(3.0f, 0.5f, 0.0f));
    const ComposedScene c = composed(s);
    EXPECT_EQ(ed::pick(s, c, nullptr, glm::vec3(3.0f, 0.5f, 6.0f), kForward), std::optional<std::string>("box"));
    // Its edge, a little in from the corner: still the box.
    EXPECT_EQ(ed::pick(s, c, nullptr, glm::vec3(3.45f, 0.95f, 6.0f), kForward), std::optional<std::string>("box"));
}

TEST(EditorPick, ARayAtADronePicksItsVehicle) {
    const SceneDesc s = support::sample_scene();
    const ComposedScene c = composed(s);
    const glm::vec3 q = quad_start(s);
    const std::optional<std::string> quad = s.vehicles.at(0).name;
    EXPECT_EQ(ed::pick(s, c, nullptr, q + glm::vec3(0.0f, 0.0f, 6.0f), kForward), quad);
    // Out along an arm, past any proxy sphere: the rotors are part of the drone.
    EXPECT_EQ(ed::pick(s, c, nullptr, q + glm::vec3(0.25f, 0.0f, 6.0f), kForward), quad);
    // Clear of its reach: not the drone.
    EXPECT_NE(ed::pick(s, c, nullptr, q + glm::vec3(0.6f, 0.0f, 6.0f), kForward), quad);
}

TEST(EditorPick, TheSkyAndTheBareGroundPickNothing) {
    const SceneDesc s = with_box(glm::vec3(3.0f, 0.5f, 0.0f));
    const ComposedScene c = composed(s);
    EXPECT_EQ(ed::pick(s, c, nullptr, glm::vec3(0.0f, 2.0f, 6.0f), glm::normalize(glm::vec3(0.0f, 1.0f, -0.2f))),
              kNothing);
    EXPECT_EQ(ed::pick(s, c, nullptr, glm::vec3(-10.0f, 5.0f, -10.0f), glm::vec3(0.0f, -1.0f, 0.0f)), kNothing);
    // A degenerate direction picks nothing rather than marching nowhere.
    EXPECT_EQ(ed::pick(s, c, nullptr, glm::vec3(3.0f, 0.5f, 6.0f), glm::vec3(0.0f)), kNothing);
}

TEST(EditorPick, TheNearestObjectWins) {
    const SceneDesc bare = support::sample_scene();
    const glm::vec3 q = quad_start(bare);
    const SceneDesc s = with_box(q + glm::vec3(0.0f, 0.0f, 3.0f));  // between the camera and the drone
    const ComposedScene c = composed(s);
    EXPECT_EQ(ed::pick(s, c, nullptr, q + glm::vec3(0.0f, 0.0f, 8.0f), kForward), std::optional<std::string>("box"));
    // From behind the drone, the drone is nearer.
    EXPECT_EQ(ed::pick(s, c, nullptr, q - glm::vec3(0.0f, 0.0f, 8.0f), -kForward),
              std::optional<std::string>(s.vehicles.at(0).name));
}

TEST(EditorPick, ARunningVehicleIsPickedWhereTheRunHasIt) {
    SceneDesc s = support::sample_scene();
    s.vehicles.at(0).start.vel = glm::vec3(4.0f, 0.0f, 0.0f);
    const std::string name = s.vehicles.at(0).name;
    spade::Result<ed::EditorRun> run =
        ed::EditorRun::start(s, support::sample_world(), ed::records_for(support::hover_scene()));
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const glm::vec3 start = quad_start(s);
    for (int i = 0; i < 50 && run->state_of(name)->pos.x - start.x < 1.5f; ++i) {
        ASSERT_TRUE(run->step(40).has_value());
    }
    const glm::vec3 now = run->state_of(name)->pos;
    ASSERT_GT(now.x - start.x, 1.5f);
    const ComposedScene c = composed(s);
    EXPECT_EQ(ed::pick(s, c, &*run, now + glm::vec3(0.0f, 0.0f, 6.0f), kForward), std::optional<std::string>(name));
    EXPECT_EQ(ed::pick(s, c, &*run, start + glm::vec3(0.0f, 0.0f, 6.0f), kForward), kNothing);
}

// The hit is on the surface, not short of it, so what a click places rests
// there. N15's editor.add_objects measured objects the tour placed from its
// camera, about 8 m off, floating 7.5 to 8.9 mm: the march stopped within
// 1e-3 of the distance travelled. From 10 m, and from 30 m at a grazing
// 4 degrees, the hit point is within 0.1 mm of the surface by the SDF itself.
TEST(EditorPick, TheRayMeetsTheSurfaceWithinATenthOfAMillimetreFromAfar) {
    const ComposedScene c = composed(support::sample_scene());
    for (const auto& [eye, target] : {std::pair{glm::vec3(0.0f, 3.42f, 9.4f), glm::vec3(0.0f, 0.0f, 0.0f)},
                                      std::pair{glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 0.0f, -30.0f)}}) {
        const glm::vec3 dir = glm::normalize(target - eye);
        const std::optional<float> t = ed::detail::march(c.world.sdf, eye, dir);
        ASSERT_TRUE(t.has_value());
        EXPECT_LT(std::abs(spade::eval(c.world.sdf, eye + dir * *t)), 1e-4f) << "from " << glm::length(target - eye) << " m";
    }
}
