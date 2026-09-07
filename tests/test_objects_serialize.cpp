// test_objects_serialize.cpp -- Plan A Task 6 (24th spec SL4): the object
// graph's JSON form.
//
// The format's two load-bearing choices are pinned here rather than described
// anywhere else: component TYPES travel by registered name (a fixed, unique
// vocabulary), and PARENT LINKS travel by array index (object names are user
// data and are not unique). DuplicateNamesStillReparentCorrectly is the case
// that separates those two decisions -- it passes under indices and cannot
// pass under names.

#include <gtest/gtest.h>

#include <string>

#include "objects/serialize.hpp"

using namespace spade::objects;

namespace {

[[nodiscard]] std::string dumped(const ObjectGraph& g) {
    const spade::Result<std::string> text = to_json(g);
    EXPECT_TRUE(text.has_value())
        << (text ? std::string{} : text.error().context);
    return text ? *text : std::string{};
}

}  // namespace

TEST(ObjectGraphSerialize, RoundTripsObjectsAndComponents) {
    ObjectGraph g;
    const auto drone = g.create("drone");
    g.get(drone)->position = glm::vec3(1.0f, 2.0f, 3.0f);
    g.attach<BodyComponent>(drone, BodyComponent{.world_index = 1, .body_slot = 4});
    g.attach<CameraComponent>(drone, CameraComponent{.fov_degrees = 60.0f, .active = false});

    const auto restored = from_json(dumped(g));
    ASSERT_TRUE(restored.has_value());

    ASSERT_EQ(restored->size(), 1u);
    // Handles are NOT preserved across a round trip -- indices are an
    // in-memory fact. Look the object up by name, which is its stable identity
    // for a reader.
    const ObjectId id = find_by_name(*restored, "drone");
    ASSERT_FALSE(id.is_null());
    EXPECT_EQ(restored->get(id)->position, glm::vec3(1.0f, 2.0f, 3.0f));
    ASSERT_NE(restored->component<BodyComponent>(id), nullptr);
    EXPECT_EQ(restored->component<BodyComponent>(id)->body_slot, 4u);
    ASSERT_NE(restored->component<CameraComponent>(id), nullptr);
    EXPECT_FALSE(restored->component<CameraComponent>(id)->active);
    EXPECT_EQ(restored->component<MeshComponent>(id), nullptr);
}

// Every component type at once, so a type whose fields this file forgets to
// write cannot hide behind the four the other cases happen to exercise.
TEST(ObjectGraphSerialize, RoundTripsEveryComponentType) {
    ObjectGraph g;
    const auto id = g.create("everything");
    g.attach<TransformComponent>(id, TransformComponent{});
    g.attach<BodyComponent>(id, BodyComponent{.world_index = 3, .body_slot = 7});
    g.attach<MeshComponent>(id, MeshComponent{.draw_item = 11});
    g.attach<MaterialComponent>(id, MaterialComponent{.material_index = 13});
    g.attach<ColliderComponent>(id, ColliderComponent{.sphere_radius = 0.5f});
    g.attach<SensorComponent>(id, SensorComponent{.sensor_slot = 17});
    g.attach<ForceElementComponent>(id, ForceElementComponent{.element_slot = 19});
    g.attach<CameraComponent>(id, CameraComponent{
                                      .fov_degrees = 42.0f, .near_plane = 0.5f,
                                      .far_plane = 900.0f, .active = false});
    g.attach<BehaviorComponent>(id, BehaviorComponent{.behavior_index = 23});
    g.attach<FluidComponent>(
        id, FluidComponent{.rest_density = 998.0f, .stiffness = 3.0f, .viscosity = 0.25f});

    const auto restored = from_json(dumped(g));
    ASSERT_TRUE(restored.has_value());
    const ObjectId r = find_by_name(*restored, "everything");
    ASSERT_FALSE(r.is_null());

    // The mask compares in one read -- every type present, none invented.
    EXPECT_EQ(restored->component_mask(r), g.component_mask(id));

    ASSERT_NE(restored->component<BodyComponent>(r), nullptr);
    EXPECT_EQ(restored->component<BodyComponent>(r)->world_index, 3u);
    ASSERT_NE(restored->component<MeshComponent>(r), nullptr);
    EXPECT_EQ(restored->component<MeshComponent>(r)->draw_item, 11u);
    ASSERT_NE(restored->component<MaterialComponent>(r), nullptr);
    EXPECT_EQ(restored->component<MaterialComponent>(r)->material_index, 13u);
    ASSERT_NE(restored->component<ColliderComponent>(r), nullptr);
    EXPECT_FLOAT_EQ(restored->component<ColliderComponent>(r)->sphere_radius, 0.5f);
    ASSERT_NE(restored->component<SensorComponent>(r), nullptr);
    EXPECT_EQ(restored->component<SensorComponent>(r)->sensor_slot, 17u);
    ASSERT_NE(restored->component<ForceElementComponent>(r), nullptr);
    EXPECT_EQ(restored->component<ForceElementComponent>(r)->element_slot, 19u);
    ASSERT_NE(restored->component<CameraComponent>(r), nullptr);
    EXPECT_FLOAT_EQ(restored->component<CameraComponent>(r)->far_plane, 900.0f);
    ASSERT_NE(restored->component<BehaviorComponent>(r), nullptr);
    EXPECT_EQ(restored->component<BehaviorComponent>(r)->behavior_index, 23u);
    ASSERT_NE(restored->component<FluidComponent>(r), nullptr);
    EXPECT_FLOAT_EQ(restored->component<FluidComponent>(r)->viscosity, 0.25f);
}

TEST(ObjectGraphSerialize, RejectsAnUnknownComponentName) {
    // A saved graph naming a component this build does not have is an ERROR,
    // not a silent skip: silently dropping it would load a scene that is
    // quietly missing behaviour the author put there.
    const auto bad = from_json(R"({"objects":[{
        "name":"x","parent":-1,"position":[0,0,0],"orientation":[1,0,0,0],
        "scale":[1,1,1],"components":{"no_such":{}}}]})");
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("no_such"), std::string::npos) << bad.error().context;
}

TEST(ObjectGraphSerialize, PreservesParentLinks) {
    ObjectGraph g;
    const auto parent = g.create("hub");
    const auto child = g.create("arm");
    g.get(child)->parent = parent;

    const auto restored = from_json(dumped(g));
    ASSERT_TRUE(restored.has_value());
    const ObjectId h = find_by_name(*restored, "hub");
    const ObjectId a = find_by_name(*restored, "arm");
    ASSERT_FALSE(h.is_null());
    ASSERT_FALSE(a.is_null());
    EXPECT_EQ(restored->get(a)->parent, h);
    EXPECT_TRUE(restored->get(h)->parent.is_null());
}

// THE CASE THAT DECIDES THE FORMAT. Object names are user data and nothing
// makes them unique. Resolving a parent by name -- which the plan specified --
// attaches this child to whichever "hub" the loader reaches first, silently
// producing a different hierarchy than the one saved. Indices cannot be
// ambiguous, so this passes.
TEST(ObjectGraphSerialize, DuplicateNamesStillReparentCorrectly) {
    ObjectGraph g;
    const auto first_hub = g.create("hub");
    const auto second_hub = g.create("hub");
    const auto arm = g.create("arm");
    g.get(arm)->parent = second_hub;  // the SECOND one, deliberately
    g.get(first_hub)->position = glm::vec3(1.0f, 0.0f, 0.0f);
    g.get(second_hub)->position = glm::vec3(2.0f, 0.0f, 0.0f);

    const auto restored = from_json(dumped(g));
    ASSERT_TRUE(restored.has_value());
    ASSERT_EQ(restored->size(), 3u);

    const ObjectId r_arm = find_by_name(*restored, "arm");
    ASSERT_FALSE(r_arm.is_null());
    const ObjectId r_parent = restored->get(r_arm)->parent;
    ASSERT_FALSE(r_parent.is_null());
    // Identified by the field that distinguishes them, not by the name they
    // share.
    EXPECT_EQ(restored->get(r_parent)->position, glm::vec3(2.0f, 0.0f, 0.0f));
}

// A parent appearing AFTER its child in the array must still resolve -- the
// reason loading is two passes.
TEST(ObjectGraphSerialize, ResolvesAForwardParentReference) {
    const auto g = from_json(R"({"objects":[
        {"name":"child","parent":1,"position":[0,0,0],"orientation":[1,0,0,0],
         "scale":[1,1,1],"components":{}},
        {"name":"parent","parent":-1,"position":[0,0,0],"orientation":[1,0,0,0],
         "scale":[1,1,1],"components":{}}]})");
    ASSERT_TRUE(g.has_value());
    const ObjectId child = find_by_name(*g, "child");
    const ObjectId parent = find_by_name(*g, "parent");
    ASSERT_FALSE(child.is_null());
    EXPECT_EQ(g->get(child)->parent, parent);
}

// Saving a graph whose child names a destroyed parent REFUSES rather than
// quietly writing the child as a root. See serialize.hpp: nothing owns
// hierarchy, so this is where the dangling state has to be answered.
TEST(ObjectGraphSerialize, RefusesToSaveADanglingParent) {
    ObjectGraph g;
    const auto parent = g.create("hub");
    const auto child = g.create("arm");
    g.get(child)->parent = parent;
    ASSERT_TRUE(g.destroy(parent));

    const spade::Result<std::string> text = to_json(g);
    ASSERT_FALSE(text.has_value());
    EXPECT_NE(text.error().context.find("arm"), std::string::npos) << text.error().context;
}

TEST(ObjectGraphSerialize, RejectsMalformedDocuments) {
    EXPECT_FALSE(from_json("not json at all").has_value());
    EXPECT_FALSE(from_json(R"({"nope":[]})").has_value());
    // A vector of the wrong length is rejected rather than partly applied --
    // [1,2] as a position would otherwise leave z at whatever the default was.
    EXPECT_FALSE(from_json(R"({"objects":[{
        "name":"x","parent":-1,"position":[1,2],"orientation":[1,0,0,0],
        "scale":[1,1,1],"components":{}}]})")
                     .has_value());
    // A parent index past the end of the array.
    EXPECT_FALSE(from_json(R"({"objects":[{
        "name":"x","parent":7,"position":[0,0,0],"orientation":[1,0,0,0],
        "scale":[1,1,1],"components":{}}]})")
                     .has_value());
    // An object parented to itself.
    EXPECT_FALSE(from_json(R"({"objects":[{
        "name":"x","parent":0,"position":[0,0,0],"orientation":[1,0,0,0],
        "scale":[1,1,1],"components":{}}]})")
                     .has_value());
}

// Output is a function of the GRAPH, not of the order objects were created or
// recycled in -- which is what makes a saved scene diffable across sessions.
TEST(ObjectGraphSerialize, OutputIsDeterministicForAGivenGraph) {
    ObjectGraph g;
    const auto a = g.create("a");
    const auto scratch = g.create("scratch");
    g.attach<MeshComponent>(scratch, MeshComponent{.draw_item = 99});
    ASSERT_TRUE(g.destroy(scratch));
    const auto b = g.create("b");  // recycles scratch's slot
    g.attach<MeshComponent>(a, MeshComponent{.draw_item = 1});
    g.attach<MeshComponent>(b, MeshComponent{.draw_item = 2});

    const std::string once = dumped(g);
    const std::string twice = dumped(g);
    EXPECT_EQ(once, twice);
    // And the recycled object carries none of the dead one's components.
    EXPECT_EQ(once.find("99"), std::string::npos) << once;
}
