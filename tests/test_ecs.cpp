#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "ecs/handle.hpp"
#include "ecs/pool.hpp"
#include "ecs/registry.hpp"

namespace {

struct Position {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Velocity {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

}  // namespace

// ---------------------------------------------------------------------------
// Pool<T>: slot stability, free-list reuse, tombstone-skipping iteration.
// ---------------------------------------------------------------------------

TEST(Pool, CreateAssignsAscendingSlots) {
    spade::Pool<int> pool;
    EXPECT_EQ(pool.create(10), 0u);
    EXPECT_EQ(pool.create(20), 1u);
    EXPECT_EQ(pool.create(30), 2u);
    EXPECT_EQ(pool.size(), 3u);
}

TEST(Pool, DestroyDoesNotMoveOtherSlots) {
    // The core "no swap-and-pop" guarantee: destroying the middle slot must
    // not relocate the slot after it, which is exactly what an
    // erase-by-swap-with-last implementation would do.
    spade::Pool<int> pool;
    uint32_t a = pool.create(10);
    uint32_t b = pool.create(20);
    uint32_t c = pool.create(30);

    EXPECT_TRUE(pool.destroy(b));

    ASSERT_NE(pool.get(a), nullptr);
    EXPECT_EQ(*pool.get(a), 10);
    ASSERT_NE(pool.get(c), nullptr);
    EXPECT_EQ(*pool.get(c), 30);
    EXPECT_EQ(a, 0u);
    EXPECT_EQ(c, 2u);  // c's slot number is unchanged by b's removal
    EXPECT_EQ(pool.get(b), nullptr);
}

TEST(Pool, DestroyIsIdempotentAndReportsWhetherAnythingWasRemoved) {
    spade::Pool<int> pool;
    uint32_t a = pool.create(1);

    EXPECT_TRUE(pool.destroy(a));
    EXPECT_FALSE(pool.destroy(a));    // already tombstoned
    EXPECT_FALSE(pool.destroy(999));  // out of range
}

TEST(Pool, FreedSlotIsRecycledByNextCreate) {
    spade::Pool<int> pool;
    uint32_t a = pool.create(1);
    uint32_t b = pool.create(2);
    pool.destroy(a);

    uint32_t c = pool.create(3);
    EXPECT_EQ(c, a);  // free-list reuse, not a fresh append
    EXPECT_EQ(pool.capacity(), 2u);
    ASSERT_NE(pool.get(c), nullptr);
    EXPECT_EQ(*pool.get(c), 3);
    ASSERT_NE(pool.get(b), nullptr);
    EXPECT_EQ(*pool.get(b), 2);  // untouched by a's destroy/recycle
}

TEST(Pool, IterationIsAscendingSlotOrderSkippingTombstones) {
    spade::Pool<int> pool;
    pool.create(10);              // slot 0
    uint32_t b = pool.create(20);  // slot 1
    pool.create(30);              // slot 2
    pool.destroy(b);

    std::vector<std::pair<uint32_t, int>> seen;
    pool.for_each([&](uint32_t slot, int& value) { seen.emplace_back(slot, value); });

    ASSERT_EQ(seen.size(), 2u);
    EXPECT_EQ(seen[0].first, 0u);
    EXPECT_EQ(seen[0].second, 10);
    EXPECT_EQ(seen[1].first, 2u);
    EXPECT_EQ(seen[1].second, 30);
}

// ---------------------------------------------------------------------------
// spade::component_id<T>(): monotonic, stable per type, never typeid.
// ---------------------------------------------------------------------------

TEST(ComponentId, StablePerTypeAndDistinctAcrossTypes) {
    auto p1 = spade::component_id<Position>();
    auto v1 = spade::component_id<Velocity>();
    auto p2 = spade::component_id<Position>();

    EXPECT_EQ(p1, p2);
    EXPECT_NE(p1, v1);
}

// ---------------------------------------------------------------------------
// Component concept: must require move-assignment, not just
// move-construction. Registry::add<T>()'s overwrite-in-place path
// (re-adding a component to an entity that already has one) move-assigns
// into the existing pool slot -- a type that is move-constructible but NOT
// move-assignable must be rejected by the concept at compile time, rather
// than compiling for a first add() and only failing on a second one.
// ---------------------------------------------------------------------------

namespace {

// Move-constructible, deliberately NOT move-assignable (a const member
// blocks the implicit move-assignment operator, and none is declared).
struct MoveOnlyNotAssignable {
    const int tag;
    explicit MoveOnlyNotAssignable(int t) : tag(t) {}
    MoveOnlyNotAssignable(MoveOnlyNotAssignable&&) = default;
    MoveOnlyNotAssignable(const MoveOnlyNotAssignable&) = delete;
};

static_assert(std::is_move_constructible_v<MoveOnlyNotAssignable>);
static_assert(!std::is_move_assignable_v<MoveOnlyNotAssignable>);
static_assert(!spade::Component<MoveOnlyNotAssignable>,
              "Component must require move-assignment (add()'s overwrite path move-assigns "
              "into the existing pool slot), not just move-construction");
static_assert(spade::Component<Position>);
static_assert(spade::Component<Velocity>);

}  // namespace

TEST(Component, ConceptExcludesMoveConstructibleOnlyTypes) {
    // The static_asserts above are the actual regression check (a concept
    // violation is a compile error, not a runtime one); this test exists so
    // the property shows up as a named, discoverable test case too.
    SUCCEED();
}

// ---------------------------------------------------------------------------
// Registry: entity create/destroy/reuse + generation invalidation.
// ---------------------------------------------------------------------------

TEST(Registry, FirstHandleHasGenerationOne) {
    // Generation 0 means "never issued" (Handle<Tag>::is_null()), so the
    // very first handle for a fresh slot must start at 1, not 0.
    spade::Registry reg;
    auto h = reg.create();
    EXPECT_EQ(h.index, 0u);
    EXPECT_EQ(h.generation, 1u);
    EXPECT_FALSE(h.is_null());
    EXPECT_TRUE(reg.is_valid(h));
}

TEST(Registry, DestroyThenCreateReusesSlotAndBumpsGeneration) {
    spade::Registry reg;
    auto h1 = reg.create();

    ASSERT_TRUE(reg.destroy(h1).has_value());
    EXPECT_FALSE(reg.is_valid(h1));

    auto h2 = reg.create();
    EXPECT_EQ(h2.index, h1.index);        // slot reused
    EXPECT_EQ(h2.generation, 2u);         // generation bumped
    EXPECT_NE(h1, h2);
    EXPECT_TRUE(reg.is_valid(h2));
    EXPECT_FALSE(reg.is_valid(h1));       // stale handle still invalid post-reuse
}

TEST(Registry, StaleHandleDestroyReturnsNotFound) {
    spade::Registry reg;
    auto h = reg.create();
    ASSERT_TRUE(reg.destroy(h).has_value());

    auto result = reg.destroy(h);  // already dead
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::not_found);
}

TEST(Registry, StaleHandleGetReturnsNotFound) {
    spade::Registry reg;
    auto h = reg.create();
    ASSERT_TRUE(reg.add<Position>(h, Position{1, 2, 3}).has_value());
    ASSERT_TRUE(reg.destroy(h).has_value());

    auto result = reg.get<Position>(h);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::not_found);
}

TEST(Registry, NullHandleIsNeverValid) {
    spade::Registry reg;
    EXPECT_FALSE(reg.is_valid(spade::EntityHandle{}));
    auto result = reg.destroy(spade::EntityHandle{});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::not_found);
}

TEST(Registry, EntitySlotsStableAcrossDestroys) {
    spade::Registry reg;
    auto a = reg.create();  // slot 0
    auto b = reg.create();  // slot 1
    auto c = reg.create();  // slot 2

    ASSERT_TRUE(reg.destroy(b).has_value());

    EXPECT_TRUE(reg.is_valid(a));
    EXPECT_TRUE(reg.is_valid(c));
    EXPECT_EQ(a.index, 0u);
    EXPECT_EQ(c.index, 2u);  // c's slot is unaffected by b's removal
}

// ---------------------------------------------------------------------------
// Registry: component add/get/remove, and component cleanup on destroy.
// ---------------------------------------------------------------------------

TEST(Registry, ComponentAddGetRemoveRoundTrip) {
    spade::Registry reg;
    auto e = reg.create();

    ASSERT_TRUE(reg.add<Position>(e, Position{1, 2, 3}).has_value());
    auto got = reg.get<Position>(e);
    ASSERT_TRUE(got.has_value());
    EXPECT_FLOAT_EQ((*got)->x, 1.0f);
    EXPECT_FLOAT_EQ((*got)->y, 2.0f);
    EXPECT_FLOAT_EQ((*got)->z, 3.0f);

    ASSERT_TRUE(reg.remove<Position>(e).has_value());
    EXPECT_FALSE(reg.get<Position>(e).has_value());
}

TEST(Registry, AddTwiceOverwritesInPlace) {
    spade::Registry reg;
    auto e = reg.create();

    ASSERT_TRUE(reg.add<Position>(e, Position{1, 1, 1}).has_value());
    ASSERT_TRUE(reg.add<Position>(e, Position{9, 9, 9}).has_value());

    auto got = reg.get<Position>(e);
    ASSERT_TRUE(got.has_value());
    EXPECT_FLOAT_EQ((*got)->x, 9.0f);
}

TEST(Registry, RemoveWithoutAddReturnsNotFound) {
    spade::Registry reg;
    auto e = reg.create();
    auto result = reg.remove<Position>(e);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::not_found);
}

TEST(Registry, ComponentsAreIndependentAcrossEntities) {
    // Removing one entity's component must not disturb another entity's
    // component of the same type (this is Pool<T>'s no-swap-and-pop
    // guarantee showing through Registry's public surface).
    spade::Registry reg;
    auto e1 = reg.create();
    auto e2 = reg.create();
    auto e3 = reg.create();
    ASSERT_TRUE(reg.add<Position>(e1, Position{1, 1, 1}).has_value());
    ASSERT_TRUE(reg.add<Position>(e2, Position{2, 2, 2}).has_value());
    ASSERT_TRUE(reg.add<Position>(e3, Position{3, 3, 3}).has_value());

    ASSERT_TRUE(reg.remove<Position>(e2).has_value());

    auto p1 = reg.get<Position>(e1);
    auto p3 = reg.get<Position>(e3);
    ASSERT_TRUE(p1.has_value());
    ASSERT_TRUE(p3.has_value());
    EXPECT_FLOAT_EQ((*p1)->x, 1.0f);
    EXPECT_FLOAT_EQ((*p3)->x, 3.0f);
    EXPECT_FALSE(reg.get<Position>(e2).has_value());
}

TEST(Registry, DestroyingEntityRemovesItsComponentsAndDoesNotLeakIntoReusedSlot) {
    spade::Registry reg;
    auto e1 = reg.create();
    auto e2 = reg.create();
    ASSERT_TRUE(reg.add<Position>(e1, Position{1, 1, 1}).has_value());
    ASSERT_TRUE(reg.add<Position>(e2, Position{2, 2, 2}).has_value());

    ASSERT_TRUE(reg.destroy(e1).has_value());

    // e2's component is untouched.
    auto e2pos = reg.get<Position>(e2);
    ASSERT_TRUE(e2pos.has_value());
    EXPECT_FLOAT_EQ((*e2pos)->x, 2.0f);

    // A new entity that reuses e1's slot must not see e1's stale Position --
    // apply_destroy() must have stripped it, not just orphaned it.
    auto e3 = reg.create();
    EXPECT_EQ(e3.index, e1.index);
    EXPECT_FALSE(reg.get<Position>(e3).has_value());
}

// ---------------------------------------------------------------------------
// Registry: begin_step()/end_step() queued-destroy protocol.
// ---------------------------------------------------------------------------

TEST(Registry, DestroyAppliesImmediatelyOutsideAStep) {
    spade::Registry reg;
    auto h = reg.create();
    ASSERT_TRUE(reg.destroy(h).has_value());
    EXPECT_FALSE(reg.is_valid(h));
}

TEST(Registry, DestroyQueuesDuringAStepAndHandleStaysValidUntilFlush) {
    spade::Registry reg;
    auto h = reg.create();
    ASSERT_TRUE(reg.add<Position>(h, Position{5, 5, 5}).has_value());

    reg.begin_step();
    auto queued = reg.destroy(h);
    ASSERT_TRUE(queued.has_value());   // queuing itself reports success
    EXPECT_TRUE(reg.is_valid(h));      // still alive mid-step
    EXPECT_TRUE(reg.get<Position>(h).has_value());

    reg.end_step();
    EXPECT_FALSE(reg.is_valid(h));     // flushed on end_step()
    EXPECT_FALSE(reg.get<Position>(h).has_value());
}

TEST(Registry, QueuedDestroysApplyInQueueOrderWithGenerationBump) {
    spade::Registry reg;
    auto a = reg.create();  // slot 0, gen 1
    auto b = reg.create();  // slot 1, gen 1
    auto c = reg.create();  // slot 2, gen 1

    reg.begin_step();
    ASSERT_TRUE(reg.destroy(a).has_value());
    ASSERT_TRUE(reg.destroy(b).has_value());
    reg.end_step();

    EXPECT_FALSE(reg.is_valid(a));
    EXPECT_FALSE(reg.is_valid(b));
    EXPECT_TRUE(reg.is_valid(c));

    // Free-list is LIFO: applying the queue in call order (a then b) pushes
    // a's slot onto the free-list first and b's slot second, so the next
    // create() reuses b's slot.
    auto d = reg.create();
    EXPECT_EQ(d.index, b.index);
    EXPECT_EQ(d.generation, 2u);
}

TEST(Registry, DoubleQueueingSameHandleWithinAStepIsIdempotent) {
    spade::Registry reg;
    auto h = reg.create();

    reg.begin_step();
    ASSERT_TRUE(reg.destroy(h).has_value());
    ASSERT_TRUE(reg.destroy(h).has_value());  // queued again; must not double-bump on flush
    reg.end_step();

    EXPECT_FALSE(reg.is_valid(h));

    auto h2 = reg.create();
    EXPECT_EQ(h2.index, h.index);
    EXPECT_EQ(h2.generation, 2u);  // exactly one bump, not two
}

TEST(Registry, DestroyDuringStepOnStaleHandleStillReturnsNotFound) {
    spade::Registry reg;
    auto h = reg.create();
    ASSERT_TRUE(reg.destroy(h).has_value());  // already dead, outside a step

    reg.begin_step();
    auto result = reg.destroy(h);
    reg.end_step();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::not_found);
}

// ---------------------------------------------------------------------------
// Determinism: two identical 10k-entity create/destroy sequences must
// produce identical slot layouts.
// ---------------------------------------------------------------------------

namespace {

struct ChurnResult {
    std::vector<spade::EntityHandle> created;       // full creation history, in order
    std::vector<spade::EntityHandle> final_alive;    // whatever's left at the end
};

// Runs a fixed-seed pseudo-random create/destroy sequence against a fresh
// Registry until exactly 10,000 entities have been created. The seed is
// fixed so the *sequence of operations* -- not Registry's behavior -- is
// what's reproduced between the two calls in the test below; this is a
// test-local scenario generator, not engine runtime randomness, so it does
// not need spade::rng (Task 8), which governs determinism of gameplay/sim
// randomness under spade/engine/, not test scaffolding under spade/tests/.
ChurnResult run_churn_sequence() {
    spade::Registry reg;
    std::mt19937 rng(0xC0FFEEu);
    std::vector<spade::EntityHandle> alive;
    ChurnResult result;
    result.created.reserve(10000);

    while (result.created.size() < 10000) {
        bool do_destroy = alive.size() > 4 && (rng() % 3 == 0);
        if (do_destroy) {
            std::uniform_int_distribution<size_t> pick(0, alive.size() - 1);
            size_t idx = pick(rng);
            EXPECT_TRUE(reg.destroy(alive[idx]).has_value());
            alive.erase(alive.begin() + static_cast<std::ptrdiff_t>(idx));
        } else {
            auto h = reg.create();
            alive.push_back(h);
            result.created.push_back(h);
        }
    }

    result.final_alive = std::move(alive);
    return result;
}

}  // namespace

TEST(Registry, TenThousandEntityChurnProducesDeterministicSlotLayout) {
    ChurnResult run1 = run_churn_sequence();
    ChurnResult run2 = run_churn_sequence();

    ASSERT_EQ(run1.created.size(), run2.created.size());
    for (size_t i = 0; i < run1.created.size(); ++i) {
        EXPECT_EQ(run1.created[i].index, run2.created[i].index) << "creation #" << i << " index diverged";
        EXPECT_EQ(run1.created[i].generation, run2.created[i].generation)
            << "creation #" << i << " generation diverged";
    }

    ASSERT_EQ(run1.final_alive.size(), run2.final_alive.size());
    for (size_t i = 0; i < run1.final_alive.size(); ++i) {
        EXPECT_EQ(run1.final_alive[i], run2.final_alive[i]) << "final alive entity #" << i << " diverged";
    }
}
