// test_objects_component.cpp -- Plan A Task 2 (24th spec SL4): component type
// ids are compile-time registered, monotonic from zero, and dense.
//
// These are SERIALIZATION KEYS, which is the whole reason they are hand-
// assigned rather than derived. typeid().hash_code() is not stable across
// compilers, is not stable across builds under some ABIs, and carries no
// ordering -- all three break a saved graph. The ordering pinned below is
// therefore frozen: appending a type is legal, renumbering one silently
// invalidates every scene ever saved.

#include <gtest/gtest.h>

#include <iterator>

#include "objects/component.hpp"
#include "objects/graph.hpp"

using namespace spade::objects;

TEST(ComponentTypeId, IsMonotonicFromZeroAndDense) {
    // Small, dense and ordered, so a saved graph reads back without a lookup
    // table.
    EXPECT_EQ(component_type_id<TransformComponent>(), ComponentTypeId{0});
    EXPECT_EQ(component_type_id<BodyComponent>(), ComponentTypeId{1});
    EXPECT_EQ(kComponentTypeCount, 10u);
}

TEST(ComponentTypeId, EveryTypeHasADistinctId) {
    const ComponentTypeId ids[] = {
        component_type_id<TransformComponent>(),    component_type_id<BodyComponent>(),
        component_type_id<MeshComponent>(),         component_type_id<MaterialComponent>(),
        component_type_id<ColliderComponent>(),     component_type_id<SensorComponent>(),
        component_type_id<ForceElementComponent>(), component_type_id<CameraComponent>(),
        component_type_id<BehaviorComponent>(),     component_type_id<FluidComponent>(),
    };
    for (std::size_t i = 0; i < std::size(ids); ++i) {
        EXPECT_EQ(ids[i], ComponentTypeId{static_cast<uint32_t>(i)});
    }
}

TEST(ComponentTypeId, NameRoundTripsForSerialization) {
    EXPECT_EQ(component_type_name(component_type_id<MeshComponent>()), "mesh");
    EXPECT_EQ(component_type_id_from_name("mesh"), component_type_id<MeshComponent>());
    EXPECT_FALSE(component_type_id_from_name("no_such_component").has_value());
}

// The name table is indexed BY id, so a table that drifts out of step with the
// enum would hand back the wrong name for every type after the drift point --
// silently, and only visibly in a saved file. Walking the whole range makes
// that a test failure instead.
TEST(ComponentTypeId, EveryIdInRangeHasANonEmptyNameThatRoundTrips) {
    for (uint32_t i = 0; i < kComponentTypeCount; ++i) {
        const auto id = static_cast<ComponentTypeId>(i);
        const std::string_view name = component_type_name(id);
        EXPECT_FALSE(name.empty()) << "id " << i << " has no name";
        EXPECT_EQ(component_type_id_from_name(name), id) << "id " << i << " does not round-trip";
    }
}

// Out-of-range must be inert rather than reading past the table.
TEST(ComponentTypeId, OutOfRangeNameIsEmptyNotUndefined) {
    EXPECT_TRUE(component_type_name(static_cast<ComponentTypeId>(kComponentTypeCount)).empty());
    EXPECT_TRUE(component_type_name(static_cast<ComponentTypeId>(9999u)).empty());
}

// ---------------------------------------------------------------------------
// Task 4: attachment. Storage is one parallel vector per type indexed by
// object slot, plus a per-object uint32 bitmask over the ten type ids. The
// mask is what makes "which components does this object have" a single read,
// which is what Plan C's inspector iterates.
// ---------------------------------------------------------------------------

TEST(ComponentAttach, AttachThenReadBack) {
    ObjectGraph g;
    const auto id = g.create("drone");
    EXPECT_TRUE(g.attach<BodyComponent>(id, BodyComponent{.world_index = 2, .body_slot = 41}));
    ASSERT_NE(g.component<BodyComponent>(id), nullptr);
    EXPECT_EQ(g.component<BodyComponent>(id)->body_slot, 41u);
    EXPECT_TRUE(g.has(id, component_type_id<BodyComponent>()));
    EXPECT_FALSE(g.has(id, component_type_id<MeshComponent>()));
    EXPECT_EQ(g.component<MeshComponent>(id), nullptr);
}

TEST(ComponentAttach, DetachRemovesOnlyThatType) {
    ObjectGraph g;
    const auto id = g.create("drone");
    g.attach<BodyComponent>(id, BodyComponent{});
    g.attach<MeshComponent>(id, MeshComponent{.draw_item = 5});
    EXPECT_TRUE(g.detach<BodyComponent>(id));
    EXPECT_EQ(g.component<BodyComponent>(id), nullptr);
    ASSERT_NE(g.component<MeshComponent>(id), nullptr);
    EXPECT_EQ(g.component<MeshComponent>(id)->draw_item, 5u);
    EXPECT_FALSE(g.detach<BodyComponent>(id));  // detaching twice is rejected
}

// The leak this catches: a recycled slot inheriting the previous occupant's
// components, which would make a fresh object silently reference another
// object's body slot. create() clearing the mask is the single line that
// prevents it -- deleting that line is what takes this case red.
TEST(ComponentAttach, DestroyingAnObjectClearsItsComponents) {
    ObjectGraph g;
    const auto first = g.create("a");
    g.attach<MeshComponent>(first, MeshComponent{.draw_item = 9});
    ASSERT_TRUE(g.destroy(first));
    const auto second = g.create("b");
    ASSERT_EQ(first.index, second.index);
    EXPECT_EQ(g.component<MeshComponent>(second), nullptr);
    EXPECT_EQ(g.component_mask(second), 0u);
    EXPECT_FALSE(g.has(second, component_type_id<MeshComponent>()));
}

TEST(ComponentAttach, RejectsADeadObject) {
    ObjectGraph g;
    const auto id = g.create("a");
    ASSERT_TRUE(g.destroy(id));
    EXPECT_FALSE(g.attach<MeshComponent>(id, MeshComponent{}));
    EXPECT_EQ(g.component<MeshComponent>(id), nullptr);
    EXPECT_FALSE(g.detach<MeshComponent>(id));
    EXPECT_FALSE(g.has(id, component_type_id<MeshComponent>()));
    EXPECT_EQ(g.component_mask(id), 0u);
}

// The mask's BIT LAYOUT is the type id, not an arbitrary enumeration order --
// bit N is type id N. Plan C's inspector and any future save format read it
// that way, so a mask built from a different mapping would be a silent
// reinterpretation rather than a visible error.
TEST(ComponentAttach, MaskBitIsTheTypeId) {
    ObjectGraph g;
    const auto id = g.create("drone");
    g.attach<TransformComponent>(id, TransformComponent{});   // id 0 -> bit 0
    g.attach<MeshComponent>(id, MeshComponent{});             // id 2 -> bit 2
    g.attach<FluidComponent>(id, FluidComponent{});           // id 9 -> bit 9
    EXPECT_EQ(g.component_mask(id), (1u << 0) | (1u << 2) | (1u << 9));
    ASSERT_TRUE(g.detach<MeshComponent>(id));
    EXPECT_EQ(g.component_mask(id), (1u << 0) | (1u << 9));
}

// Two objects sharing a component TYPE must not share its VALUE: the stores
// are indexed by object slot, and an implementation that indexed by anything
// else (a push_back, say) would still pass every single-object case above.
TEST(ComponentAttach, StoresAreIndexedPerObjectNotShared) {
    ObjectGraph g;
    const auto a = g.create("a");
    const auto b = g.create("b");
    g.attach<MeshComponent>(a, MeshComponent{.draw_item = 1});
    g.attach<MeshComponent>(b, MeshComponent{.draw_item = 2});
    ASSERT_NE(g.component<MeshComponent>(a), nullptr);
    ASSERT_NE(g.component<MeshComponent>(b), nullptr);
    EXPECT_EQ(g.component<MeshComponent>(a)->draw_item, 1u);
    EXPECT_EQ(g.component<MeshComponent>(b)->draw_item, 2u);
    // Detaching one must not disturb the other's bit or value.
    ASSERT_TRUE(g.detach<MeshComponent>(a));
    ASSERT_NE(g.component<MeshComponent>(b), nullptr);
    EXPECT_EQ(g.component<MeshComponent>(b)->draw_item, 2u);
}
