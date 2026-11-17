// flow_movie's role-to-material table (tools/flow_movie/part_materials.hpp; the lead, 2026-11-17, Cameron: "the drone should look
// fairly realistic"). Today every non-rotor part of a library model draws one light grey: render_parts::bind_parts has three
// materials, by KIND (body, rotor, porous), and never reads a part's role. The tool reads the role as DISPLAY data after the parts
// are bound: each draw item's part ("<object>/<part name>") is looked up in the model, and a known role gets its own flat colour.
// Nothing about the flow or the physics reads a role; an unknown or empty role keeps the item's material.
//
//   FlowMovieMaterials.Table*     the roles the table knows, each a colour in [0, 1], distinct where the look must tell parts apart
//   FlowMovieMaterials.Apply*     items from `first_item` on get the material of their part's role, once a role; others untouched
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "../tools/flow_movie/part_materials.hpp"
#include "render/scene.hpp"

namespace {

using Roles = std::vector<std::pair<std::string, std::string>>;  // part name, role

// A scene whose statics are items for the named parts of an object, each on material `start` (a hand-made palette of 2).
spade::render::RenderScene scene_of(const std::vector<std::string>& part_names, uint32_t start = 0u) {
    spade::render::RenderScene scene;
    scene.materials.push_back(spade::render::Material{.base_color = glm::vec4(0.8f, 0.8f, 0.84f, 1.0f)});
    scene.materials.push_back(spade::render::Material{.base_color = glm::vec4(0.3f, 0.32f, 0.36f, 1.0f)});
    const uint32_t object = spade::render::intern_label(scene, "quad");
    for (const std::string& name : part_names) {
        spade::render::DrawItem item;
        item.object_label = object;
        item.part_label = spade::render::intern_label(scene, "quad/" + name);
        item.material_override = start;
        scene.statics.push_back(item);
    }
    return scene;
}

glm::vec3 colour_of(const spade::render::RenderScene& scene, std::size_t item) {
    return glm::vec3(scene.materials[scene.statics[item].material_override].base_color);
}

}  // namespace

TEST(FlowMovieMaterials, TheTableKnowsTheRolesTheAirframesUseAndEachColourIsInRange) {
    for (const char* role : {"body", "plate", "arm", "motor", "prop", "battery", "strap", "camera", "gimbal", "lens", "gear", "esc",
                             "gps", "antenna"}) {
        const auto colour = flow_movie::role_colour(role);
        ASSERT_TRUE(colour.has_value()) << role;
        for (int a = 0; a < 3; ++a) {
            EXPECT_GE((*colour)[a], 0.0f) << role;
            EXPECT_LE((*colour)[a], 1.0f) << role;
        }
    }
    EXPECT_FALSE(flow_movie::role_colour("").has_value());
    EXPECT_FALSE(flow_movie::role_colour("wing").has_value());
    EXPECT_FALSE(flow_movie::role_colour("Body").has_value()) << "exact, lower case: the model's label is data, not a guess";
    auto luma = [](glm::vec3 c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; };
    EXPECT_GT(luma(*flow_movie::role_colour("body")), luma(*flow_movie::role_colour("battery")) + 0.2f)
        << "the shell is pale, the battery darker";
    EXPECT_GT(luma(*flow_movie::role_colour("motor")), luma(*flow_movie::role_colour("prop"))) << "metal over carbon";
    EXPECT_LT(luma(*flow_movie::role_colour("camera")), 0.12f) << "the camera is near-black";
}

TEST(FlowMovieMaterials, AppliedItemsTakeTheirPartsRoleAndAPartKeepsOneMaterialPerRole) {
    auto scene = scene_of({"body", "motor[0]", "motor[1]", "battery", "mystery", "prop[0]"});
    const Roles roles{{"body", "body"}, {"motor[0]", "motor"}, {"motor[1]", "motor"}, {"battery", "battery"},
                      {"mystery", "wing"}, {"prop[0]", ""}};
    const std::size_t before = scene.materials.size();
    EXPECT_EQ(flow_movie::apply_role_materials(scene, 0u, roles), 4u) << "body, two motors, battery";
    EXPECT_EQ(colour_of(scene, 0), *flow_movie::role_colour("body"));
    EXPECT_EQ(colour_of(scene, 1), *flow_movie::role_colour("motor"));
    EXPECT_EQ(scene.statics[1].material_override, scene.statics[2].material_override) << "one material for a role";
    EXPECT_EQ(colour_of(scene, 3), *flow_movie::role_colour("battery"));
    EXPECT_EQ(scene.statics[4].material_override, 0u) << "an unknown role keeps the item's material";
    EXPECT_EQ(scene.statics[5].material_override, 0u) << "an empty role keeps it";
    EXPECT_EQ(scene.materials.size(), before + 3u) << "three roles used: body, motor, battery";
}

TEST(FlowMovieMaterials, OnlyItemsFromTheFirstOneOnAreTouchedAndNothingOutsideTheObjectsPartsIs) {
    auto scene = scene_of({"body", "body", "body"});
    const Roles roles{{"body", "body"}};
    EXPECT_EQ(flow_movie::apply_role_materials(scene, 2u, roles), 1u);
    EXPECT_EQ(scene.statics[0].material_override, 0u);
    EXPECT_EQ(scene.statics[1].material_override, 0u);
    EXPECT_NE(scene.statics[2].material_override, 0u);
    // an item whose part is not in the model's list (the ground, say) is left alone
    auto other = scene_of({"unlisted"});
    EXPECT_EQ(flow_movie::apply_role_materials(other, 0u, roles), 0u);
    EXPECT_EQ(other.statics[0].material_override, 0u);
    EXPECT_EQ(other.materials.size(), 2u) << "no material is added for nothing";
    EXPECT_EQ(flow_movie::apply_role_materials(other, 5u, roles), 0u) << "a first item past the end is a no-op";
}

TEST(FlowMovieMaterials, ADrawItemWithNoPartLabelOrABadLabelIndexIsLeftAlone) {
    auto scene = scene_of({"body"});
    scene.statics[0].part_label = 9999u;
    const Roles roles{{"body", "body"}};
    EXPECT_EQ(flow_movie::apply_role_materials(scene, 0u, roles), 0u);
    EXPECT_EQ(scene.statics[0].material_override, 0u);
}
