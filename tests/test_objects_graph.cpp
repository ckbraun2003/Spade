// test_objects_graph.cpp -- Plan A Tasks 1 and 3 (24th spec SL3): ObjectId
// identity, and the ObjectGraph lifecycle that issues those ids.
//
// ObjectId is a generational handle, NOT a bare index. The distinction is the
// whole point: a stale id whose slot has since been reused must compare
// unequal to the id now living there, and comparing only `index` would make
// those two indistinguishable. These cases pin that.

#include <gtest/gtest.h>

#include "objects/graph.hpp"
#include "objects/object.hpp"

using spade::objects::ObjectGraph;
using spade::objects::ObjectId;

TEST(ObjectId, DefaultConstructedIsNull) {
    ObjectId id{};
    EXPECT_TRUE(id.is_null());
    EXPECT_EQ(id, spade::kNull<spade::objects::ObjectTag>);
}

TEST(ObjectId, DiffersByGenerationNotOnlyIndex) {
    ObjectId a{.index = 7, .generation = 1};
    ObjectId b{.index = 7, .generation = 3};
    EXPECT_NE(a, b);
}

// SL3's load-bearing claim in miniature: an Object carries identity and
// placement and nothing else. If simulation state ever leaks into this struct,
// the object graph stops being reconstructible-from-a-description and the
// determinism-neutrality argument the whole plan rests on stops holding.
TEST(Object, DefaultsToAnUnparentedIdentityPlacement) {
    const spade::objects::Object o{};
    EXPECT_TRUE(o.parent.is_null());
    EXPECT_TRUE(o.name.empty());
    EXPECT_EQ(o.position, glm::vec3(0.0f));
    EXPECT_EQ(o.scale, glm::vec3(1.0f));
    // Identity rotation, spelled (w, x, y, z) as glm's quat constructor takes
    // it -- not the (x, y, z, w) storage order, which is a standing trap.
    EXPECT_EQ(o.orientation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
}

// ---------------------------------------------------------------------------
// Task 3: ObjectGraph lifecycle.
//
// Generation parity follows BodyRef's convention deliberately (even == dead,
// odd == live, 0 == never issued) so there is one rule to learn across the
// engine rather than two that look alike and differ.
// ---------------------------------------------------------------------------

TEST(ObjectGraph, CreateReturnsALiveHandle) {
    ObjectGraph g;
    const ObjectId id = g.create("drone");
    EXPECT_FALSE(id.is_null());
    EXPECT_TRUE(g.alive(id));
    ASSERT_NE(g.get(id), nullptr);
    EXPECT_EQ(g.get(id)->name, "drone");
    EXPECT_EQ(g.size(), 1u);
}

TEST(ObjectGraph, DestroyInvalidatesTheHandle) {
    ObjectGraph g;
    const ObjectId id = g.create("gate");
    EXPECT_TRUE(g.destroy(id));
    EXPECT_FALSE(g.alive(id));
    EXPECT_EQ(g.get(id), nullptr);
    EXPECT_EQ(g.size(), 0u);
    EXPECT_FALSE(g.destroy(id));  // double-destroy is rejected, not silent
}

// The failure this exists to catch: a handle captured before its slot was
// reused must not silently address the new occupant. Measured, not assumed:
// dropping the generation comparison from alive() takes THREE cases red --
// this one, DestroyInvalidatesTheHandle and NullAndOutOfRange... -- because a
// live-parity handle then matches any slot in range. This is the only one of
// the three that pins the recycle specifically.
TEST(ObjectGraph, RecycledSlotDoesNotAnswerToTheStaleHandle) {
    ObjectGraph g;
    const ObjectId first = g.create("a");
    ASSERT_TRUE(g.destroy(first));
    const ObjectId second = g.create("b");
    EXPECT_EQ(first.index, second.index);  // slot genuinely reused
    EXPECT_NE(first, second);              // handle still distinguishes them
    EXPECT_FALSE(g.alive(first));
    EXPECT_EQ(g.get(first), nullptr);
    ASSERT_NE(g.get(second), nullptr);
    EXPECT_EQ(g.get(second)->name, "b");
}

// A null handle is what every default-constructed ObjectId is -- including the
// `parent` of every root object. If it read as alive, slot 0 would answer for
// "no parent" and the whole parent-null convention would collapse.
TEST(ObjectGraph, NullAndOutOfRangeHandlesAreNeverAlive) {
    ObjectGraph g;
    const ObjectId real = g.create("root");
    EXPECT_FALSE(g.alive(ObjectId{}));
    EXPECT_EQ(g.get(ObjectId{}), nullptr);
    EXPECT_FALSE(g.destroy(ObjectId{}));
    // Past the end of the pool entirely.
    EXPECT_FALSE(g.alive(ObjectId{.index = 4096, .generation = 1}));
    EXPECT_EQ(g.get(ObjectId{.index = 4096, .generation = 1}), nullptr);
    // In range, but the wrong generation for that slot.
    EXPECT_FALSE(g.alive(ObjectId{.index = real.index, .generation = real.generation + 2u}));

    // In range AND matching the slot's counter exactly -- but that counter is
    // now EVEN, because the slot is destroyed. Only the parity half of the
    // rule rejects this one; a generation-equality check alone would call it
    // alive. This is the case that makes the parity test falsifiable rather
    // than decorative.
    ASSERT_TRUE(g.destroy(real));
    EXPECT_FALSE(g.alive(ObjectId{.index = real.index, .generation = real.generation + 1u}));
}

// size() counts LIVE objects, not slots ever allocated -- the distinction that
// makes a recycling pool's bookkeeping observable rather than merely assumed.
TEST(ObjectGraph, SizeCountsLiveObjectsNotSlots) {
    ObjectGraph g;
    EXPECT_EQ(g.size(), 0u);
    const ObjectId a = g.create("a");
    const ObjectId b = g.create("b");
    const ObjectId c = g.create("c");
    EXPECT_EQ(g.size(), 3u);
    ASSERT_TRUE(g.destroy(b));
    EXPECT_EQ(g.size(), 2u);
    const ObjectId d = g.create("d");  // reuses b's slot
    EXPECT_EQ(d.index, b.index);
    EXPECT_EQ(g.size(), 3u);
    EXPECT_TRUE(g.alive(a));
    EXPECT_TRUE(g.alive(c));
    EXPECT_TRUE(g.alive(d));
}

// A recycled slot must not hand back the previous occupant's fields. Name is
// the one a create() overwrites unconditionally, so the trap is the REST of
// the struct: a create that only assigned `name` would leave the dead
// object's placement in place, and a freshly created object would silently
// inherit a position nobody authored.
TEST(ObjectGraph, RecycledSlotStartsFromADefaultObject) {
    ObjectGraph g;
    const ObjectId first = g.create("moved");
    spade::objects::Object* o = g.get(first);
    ASSERT_NE(o, nullptr);
    o->position = glm::vec3(3.0f, 4.0f, 5.0f);
    o->scale = glm::vec3(2.0f);
    o->parent = ObjectId{.index = 9, .generation = 1};
    ASSERT_TRUE(g.destroy(first));

    const ObjectId second = g.create("fresh");
    ASSERT_EQ(second.index, first.index);
    const spade::objects::Object* fresh = g.get(second);
    ASSERT_NE(fresh, nullptr);
    EXPECT_EQ(fresh->name, "fresh");
    EXPECT_EQ(fresh->position, glm::vec3(0.0f));
    EXPECT_EQ(fresh->scale, glm::vec3(1.0f));
    EXPECT_TRUE(fresh->parent.is_null());
}
