#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"

namespace {

// A caller-defined array element -- the "probe" the registry canary
// registers. Deliberately not one of layout.hpp's structs, so the canary
// proves the registry walks whatever was registered rather than a hardcoded
// list of engine arrays.
struct ProbeState {
    uint32_t a;
    uint32_t b;
    uint64_t c;
};
static_assert(sizeof(ProbeState) == 16);

// Names seen by a registry walk, in walk order.
std::vector<std::string> walk_names(const spade::StateRegistry& registry) {
    std::vector<std::string> names;
    registry.for_each_array([&names](const spade::RegisteredArray& array) { names.push_back(array.name); });
    return names;
}

}  // namespace

// ---------------------------------------------------------------------------
// layout.hpp -- the std430 contract.
//
// Sizes, alignments and per-field offsets are static_asserted in the header
// itself, so they are verified by the mere fact that this file compiles. The
// tests below cover the parts a static_assert cannot reach.
// ---------------------------------------------------------------------------

TEST(StateLayout, StaticAssertedSizesAreVisibleAtRuntime) {
    // Restates the header's compile-time contract so a layout regression
    // shows up as a named test failure, not only as a wall of compiler
    // errors, and so the numbers appear in test output.
    EXPECT_EQ(sizeof(spade::BodyState), 128u);
    EXPECT_EQ(alignof(spade::BodyState), 16u);
    EXPECT_EQ(sizeof(spade::WorldParams), 64u);
    EXPECT_EQ(alignof(spade::WorldParams), 16u);
    EXPECT_EQ(spade::kWorldParamsNamedBytes, 56u);
}

TEST(StateLayout, GlmQuatMemoryOrderIsXyzw) {
    // The engine design spec names "the GLM quaternion-order coincidence" as
    // one of v1's two worst layout hazards. glm::quat's CONSTRUCTOR is
    // (w, x, y, z) but with glm 1.0.1 and no GLM_FORCE_QUAT_DATA_WXYZ its
    // MEMORY order is x, y, z, w -- and BodyState::orient is mirrored to the
    // GPU byte-wise, so the memory order is the contract. The components
    // live in an anonymous union, which is why this is a runtime test rather
    // than a static_assert.
    const glm::quat q(1.0f, 2.0f, 3.0f, 4.0f);
    ASSERT_EQ(q.w, 1.0f);
    ASSERT_EQ(q.x, 2.0f);
    ASSERT_EQ(q.y, 3.0f);
    ASSERT_EQ(q.z, 4.0f);

    std::array<float, 4> words{};
    std::memcpy(words.data(), &q, sizeof(q));
    EXPECT_EQ(words[0], 2.0f) << "word 0 must be x";
    EXPECT_EQ(words[1], 3.0f) << "word 1 must be y";
    EXPECT_EQ(words[2], 4.0f) << "word 2 must be z";
    EXPECT_EQ(words[3], 1.0f) << "word 3 must be w";
}

TEST(StateLayout, ArenaStorageIsSixteenByteAligned) {
    spade::ArenaSet arenas(4);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 8);
    ASSERT_TRUE(bodies.has_value());
    const auto span = arenas.array(*bodies);
    ASSERT_TRUE(span.has_value());
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(span->data()) % spade::kStd430StructAlignment, 0u);
}

TEST(StateLayout, WorldParamsTailPaddingIsZeroInAnArena) {
    // WorldParams' named fields stop at byte 56 and alignas(16) pads the
    // struct to the std430 stride of 64. Those 8 bytes are the only bytes in
    // layout.hpp not covered by a named field; ArenaSet zero-fills its
    // storage precisely so they are deterministic in a snapshot blob rather
    // than heap garbage.
    spade::ArenaSet arenas(1);
    const auto params = arenas.register_array<spade::WorldParams>("world_params", 1);
    ASSERT_TRUE(params.has_value());
    const auto slot = arenas.alloc_slot(*params, 0);
    ASSERT_TRUE(slot.has_value());

    auto span = arenas.array(*params);
    ASSERT_TRUE(span.has_value());
    spade::WorldParams& p = (*span)[*slot];
    p.gravity = glm::vec3(0.0f, 0.0f, -9.81f);
    p.air_density = 1.225f;
    p.wind = glm::vec3(3.0f, -1.0f, 0.5f);
    p._p = 0.0f;
    p.body_capacity = 8;
    p.body_count = 3;
    p.seed = 0x0123456789abcdefULL;
    p._reserved0 = 0;

    std::array<std::byte, sizeof(spade::WorldParams)> raw{};
    std::memcpy(raw.data(), &p, sizeof(p));
    for (std::size_t i = spade::kWorldParamsNamedBytes; i < sizeof(spade::WorldParams); ++i) {
        EXPECT_EQ(std::to_integer<int>(raw[i]), 0) << "tail padding byte " << i << " is not zero";
    }
}

// ---------------------------------------------------------------------------
// Partition arithmetic: world w owns slots [w * capacity, (w+1) * capacity).
// ---------------------------------------------------------------------------

TEST(ArenaSetPartition, RangesTileTheWholeArrayWithoutOverlap) {
    constexpr uint32_t kWorlds = 4;
    constexpr uint32_t kCapacity = 6;
    spade::ArenaSet arenas(kWorlds);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", kCapacity);
    ASSERT_TRUE(bodies.has_value());

    const auto span = arenas.array(*bodies);
    ASSERT_TRUE(span.has_value());
    EXPECT_EQ(span->size(), std::size_t{kWorlds} * kCapacity);

    uint32_t expected_begin = 0;
    for (uint32_t world = 0; world < kWorlds; ++world) {
        const auto range = arenas.range(*bodies, world);
        ASSERT_TRUE(range.has_value()) << "world " << world;
        EXPECT_EQ(range->begin, expected_begin);
        EXPECT_EQ(range->count, kCapacity);
        expected_begin = range->begin + range->count;
    }
    EXPECT_EQ(expected_begin, kWorlds * kCapacity) << "partitions must tile the array exactly";
}

TEST(ArenaSetPartition, WorldSliceAliasesTheGlobalArrayAtItsRange) {
    spade::ArenaSet arenas(3);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 5);
    ASSERT_TRUE(bodies.has_value());

    auto slice = arenas.world_slice(*bodies, 2);
    ASSERT_TRUE(slice.has_value());
    EXPECT_EQ(slice->size(), 5u);
    (*slice)[1].mass = 7.5f;

    const auto range = arenas.range(*bodies, 2);
    ASSERT_TRUE(range.has_value());
    const auto all = arenas.array(*bodies);
    ASSERT_TRUE(all.has_value());
    EXPECT_EQ((*all)[range->begin + 1].mass, 7.5f);
    EXPECT_EQ(slice->data(), all->data() + range->begin);
}

TEST(ArenaSetPartition, AllocatedSlotsStayInsideTheirWorldAndMapBackToIt) {
    constexpr uint32_t kWorlds = 3;
    constexpr uint32_t kCapacity = 4;
    spade::ArenaSet arenas(kWorlds);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", kCapacity);
    ASSERT_TRUE(bodies.has_value());

    for (uint32_t world = 0; world < kWorlds; ++world) {
        const auto range = arenas.range(*bodies, world);
        ASSERT_TRUE(range.has_value());
        for (uint32_t i = 0; i < kCapacity; ++i) {
            const auto slot = arenas.alloc_slot(*bodies, world);
            ASSERT_TRUE(slot.has_value()) << "world " << world << " alloc " << i;
            EXPECT_GE(*slot, range->begin);
            EXPECT_LT(*slot, range->begin + range->count);
        }
        const auto live = arenas.live_count(*bodies, world);
        ASSERT_TRUE(live.has_value());
        EXPECT_EQ(*live, kCapacity);
    }

    const auto map = arenas.slot_to_world(*bodies);
    ASSERT_TRUE(map.has_value());
    ASSERT_EQ(map->size(), std::size_t{kWorlds} * kCapacity);
    for (uint32_t slot = 0; slot < map->size(); ++slot) {
        EXPECT_EQ((*map)[slot], slot / kCapacity) << "slot " << slot;
    }
}

TEST(ArenaSetPartition, FreeSlotsReadAsInvalidWorldInTheMap) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 3);
    ASSERT_TRUE(bodies.has_value());

    const auto map = arenas.slot_to_world(*bodies);
    ASSERT_TRUE(map.has_value());
    for (uint32_t world : *map) {
        EXPECT_EQ(world, spade::kInvalidWorld);
    }

    ASSERT_TRUE(arenas.alloc_slot(*bodies, 1).has_value());
    const auto after = arenas.slot_to_world(*bodies);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ((*after)[3], 1u) << "world 1's first slot is global slot 3";
    EXPECT_EQ((*after)[0], spade::kInvalidWorld);
}

TEST(ArenaSetPartition, AWorldlessSetHoldsEmptyArraysAndRefusesAllocation) {
    // Degenerate but reachable from config: zero worlds means zero slots.
    // Pinned so it stays a clean empty arena rather than an allocation of
    // unspecified size or a crash.
    spade::ArenaSet arenas(0);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 4);
    ASSERT_TRUE(bodies.has_value());
    EXPECT_EQ(arenas.array(*bodies)->size(), 0u);
    EXPECT_EQ(arenas.registry().find("bodies")->element_count(), 0u);
    EXPECT_EQ(arenas.registry().total_bytes(), 0u);

    const auto alloc = arenas.alloc_slot(*bodies, 0);
    ASSERT_FALSE(alloc.has_value());
    EXPECT_EQ(alloc.error().code, spade::Code::invalid_argument);
}

TEST(ArenaSetPartition, RangeRejectsUnknownArraysAndWorlds) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 2);
    ASSERT_TRUE(bodies.has_value());

    const auto out_of_range_world = arenas.range(*bodies, 2);
    ASSERT_FALSE(out_of_range_world.has_value());
    EXPECT_EQ(out_of_range_world.error().code, spade::Code::invalid_argument);

    const auto unknown = arenas.range(spade::ArrayIndex{99}, 0);
    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error().code, spade::Code::not_found);
}

// ---------------------------------------------------------------------------
// Fixed capacity: overflow is reported, never silently absorbed, and it is
// per world rather than global.
// ---------------------------------------------------------------------------

TEST(ArenaSetCapacity, OverflowingAWorldReturnsCapacityExceeded) {
    constexpr uint32_t kCapacity = 3;
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", kCapacity);
    ASSERT_TRUE(bodies.has_value());

    for (uint32_t i = 0; i < kCapacity; ++i) {
        ASSERT_TRUE(arenas.alloc_slot(*bodies, 0).has_value()) << "alloc " << i;
    }
    const auto overflow = arenas.alloc_slot(*bodies, 0);
    ASSERT_FALSE(overflow.has_value());
    EXPECT_EQ(overflow.error().code, spade::Code::capacity_exceeded);

    // World 1's partition is untouched by world 0 filling up.
    const auto other = arenas.alloc_slot(*bodies, 1);
    ASSERT_TRUE(other.has_value());
    EXPECT_EQ(*other, kCapacity);
}

TEST(ArenaSetCapacity, FreeingMakesRoomAgainInThatWorldOnly) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 2);
    ASSERT_TRUE(bodies.has_value());

    ASSERT_TRUE(arenas.alloc_slot(*bodies, 0).has_value());
    ASSERT_TRUE(arenas.alloc_slot(*bodies, 0).has_value());
    ASSERT_FALSE(arenas.alloc_slot(*bodies, 0).has_value());

    ASSERT_TRUE(arenas.free_slot(*bodies, 0).has_value());
    const auto reused = arenas.alloc_slot(*bodies, 0);
    ASSERT_TRUE(reused.has_value());
    EXPECT_EQ(*reused, 0u);

    const auto live = arenas.live_count(*bodies, 0);
    ASSERT_TRUE(live.has_value());
    EXPECT_EQ(*live, 2u);
}

TEST(ArenaSetCapacity, RegisterArrayRejectsZeroCapacityAndOversizedRequests) {
    spade::ArenaSet arenas(4);

    const auto zero = arenas.register_array<spade::BodyState>("zero", 0);
    ASSERT_FALSE(zero.has_value());
    EXPECT_EQ(zero.error().code, spade::Code::invalid_argument);

    // 4 worlds * 2^30 slots overflows the uint32 slot index space.
    const auto huge = arenas.register_array<spade::BodyState>("huge", 1u << 30);
    ASSERT_FALSE(huge.has_value());
    EXPECT_EQ(huge.error().code, spade::Code::capacity_exceeded);

    // A rejected registration must leave nothing behind in the walk.
    EXPECT_EQ(arenas.registry().size(), 0u);
}

// ---------------------------------------------------------------------------
// Ordered (ascending) free list.
// ---------------------------------------------------------------------------

TEST(ArenaSetFreeList, AllocAlwaysReturnsTheLowestFreeSlotInThePartition) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 5);
    ASSERT_TRUE(bodies.has_value());

    // World 1's partition is global slots 5..9.
    for (uint32_t expected = 5; expected <= 8; ++expected) {
        const auto slot = arenas.alloc_slot(*bodies, 1);
        ASSERT_TRUE(slot.has_value());
        EXPECT_EQ(*slot, expected);
    }

    // Free out of order; the free list must still hand slots back ascending.
    ASSERT_TRUE(arenas.free_slot(*bodies, 7).has_value());
    ASSERT_TRUE(arenas.free_slot(*bodies, 5).has_value());
    ASSERT_TRUE(arenas.free_slot(*bodies, 6).has_value());

    for (uint32_t expected : {5u, 6u, 7u}) {
        const auto slot = arenas.alloc_slot(*bodies, 1);
        ASSERT_TRUE(slot.has_value());
        EXPECT_EQ(*slot, expected);
    }
    // Free list drained; the bump cursor continues where it left off.
    const auto next = arenas.alloc_slot(*bodies, 1);
    ASSERT_TRUE(next.has_value());
    EXPECT_EQ(*next, 9u);
}

TEST(ArenaSetFreeList, DoubleFreeAndOutOfRangeFreeAreReported) {
    spade::ArenaSet arenas(1);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 2);
    ASSERT_TRUE(bodies.has_value());

    const auto slot = arenas.alloc_slot(*bodies, 0);
    ASSERT_TRUE(slot.has_value());
    ASSERT_TRUE(arenas.free_slot(*bodies, *slot).has_value());

    const auto again = arenas.free_slot(*bodies, *slot);
    ASSERT_FALSE(again.has_value());
    EXPECT_EQ(again.error().code, spade::Code::not_found);

    const auto never = arenas.free_slot(*bodies, 1);
    ASSERT_FALSE(never.has_value());
    EXPECT_EQ(never.error().code, spade::Code::not_found);

    const auto out_of_range = arenas.free_slot(*bodies, 2);
    ASSERT_FALSE(out_of_range.has_value());
    EXPECT_EQ(out_of_range.error().code, spade::Code::invalid_argument);

    const auto live = arenas.live_count(*bodies, 0);
    ASSERT_TRUE(live.has_value());
    EXPECT_EQ(*live, 0u) << "a rejected free must not decrement the live count";
}

TEST(ArenaSetFreeList, FreeingZeroesTheSlotBytes) {
    spade::ArenaSet arenas(1);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 2);
    ASSERT_TRUE(bodies.has_value());

    const auto slot = arenas.alloc_slot(*bodies, 0);
    ASSERT_TRUE(slot.has_value());
    {
        auto span = arenas.array(*bodies);
        ASSERT_TRUE(span.has_value());
        spade::BodyState& body = (*span)[*slot];
        body.pos = glm::vec3(1.0f, 2.0f, 3.0f);
        body.mass = 4.0f;
        body.flags = 0xdeadbeefU;
        body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    ASSERT_TRUE(arenas.free_slot(*bodies, *slot).has_value());

    const auto span = arenas.array(*bodies);
    ASSERT_TRUE(span.has_value());
    std::array<std::byte, sizeof(spade::BodyState)> raw{};
    std::memcpy(raw.data(), &(*span)[*slot], sizeof(spade::BodyState));
    for (std::size_t i = 0; i < raw.size(); ++i) {
        EXPECT_EQ(std::to_integer<int>(raw[i]), 0) << "byte " << i << " of a freed slot is not zero";
    }
}

// ---------------------------------------------------------------------------
// Determinism: identical op sequences produce identical slot assignments.
// ---------------------------------------------------------------------------

namespace {

// Runs a fixed alloc/free script and returns a transcript: every slot index
// alloc_slot handed out, then the whole slot_to_world map. Any divergence in
// allocation order or partition bookkeeping shows up as a transcript
// mismatch.
std::vector<uint32_t> run_alloc_script() {
    spade::ArenaSet arenas(3);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 4);
    std::vector<uint32_t> transcript;
    if (!bodies) return transcript;

    const auto alloc = [&](uint32_t world) {
        const auto slot = arenas.alloc_slot(*bodies, world);
        transcript.push_back(slot.has_value() ? *slot : 0xFFFF0000U);
    };
    const auto release = [&](uint32_t slot) {
        const auto freed = arenas.free_slot(*bodies, slot);
        transcript.push_back(freed.has_value() ? 1U : 0xFFFF0001U);
    };

    alloc(0); alloc(0); alloc(0); alloc(0);
    alloc(1); alloc(1);
    alloc(2);
    release(2); release(0); release(5);
    alloc(0); alloc(1); alloc(0);
    alloc(0);  // world 0 is full here -- the failure sentinel is part of the transcript
    release(3);
    alloc(0);

    if (const auto map = arenas.slot_to_world(*bodies); map) {
        transcript.insert(transcript.end(), map->begin(), map->end());
    }
    return transcript;
}

}  // namespace

TEST(ArenaSetDeterminism, IdenticalOpSequencesProduceIdenticalSlotAssignments) {
    const std::vector<uint32_t> first = run_alloc_script();
    const std::vector<uint32_t> second = run_alloc_script();
    EXPECT_EQ(first, second);
}

TEST(ArenaSetDeterminism, TheSlotAssignmentSequenceIsPinnedNotMerelyStable) {
    // Stability alone would still be satisfied by a different-but-consistent
    // policy (LIFO reuse, say). Pin the actual lowest-index-first sequence so
    // a policy change is a visible, deliberate edit -- snapshot replay
    // corpora are recorded against these exact numbers.
    const std::vector<uint32_t> transcript = run_alloc_script();
    const std::vector<uint32_t> expected_allocs = {
        0, 1, 2, 3,  // world 0 fills slots 0..3
        4, 5,        // world 1 starts at slot 4
        8,           // world 2 starts at slot 8
        1, 1, 1,     // three successful frees (slots 2, 0, 5)
        0, 5, 2,     // world 0 -> 0 then 2 (ascending), world 1 -> 5
        0xFFFF0000U, // world 0 is full
        1,           // free slot 3
        3,           // world 0 reuses slot 3
    };
    ASSERT_GE(transcript.size(), expected_allocs.size());
    const std::vector<uint32_t> head(transcript.begin(), transcript.begin() + static_cast<std::ptrdiff_t>(expected_allocs.size()));
    EXPECT_EQ(head, expected_allocs);
}

// ---------------------------------------------------------------------------
// "No unregistered state": the registry canary.
// ---------------------------------------------------------------------------

TEST(StateRegistryCanary, AProbeArrayRegisteredInTheArenaAppearsInTheWalk) {
    spade::ArenaSet arenas(2);
    ASSERT_TRUE(arenas.register_array<spade::BodyState>("bodies", 4).has_value());
    ASSERT_TRUE(arenas.register_array<spade::WorldParams>("world_params", 1).has_value());

    // The canary: an array the engine's own layout header knows nothing
    // about. If a future ArenaSet gains a second way to obtain storage, this
    // is the test that notices the walk no longer covers everything.
    const auto probe = arenas.register_array<ProbeState>("probe", 3);
    ASSERT_TRUE(probe.has_value());

    // Every arena contributes two entries: its elements and its
    // slot_to_world map (the liveness record, which a restore needs and
    // which nothing else in the walk encodes).
    std::vector<std::string> names = walk_names(arenas.registry());
    EXPECT_EQ(arenas.registry().size(), 6u);
    std::sort(names.begin(), names.end());
    EXPECT_EQ(names, (std::vector<std::string>{"bodies", "bodies.slot_to_world", "probe", "probe.slot_to_world",
                                               "world_params", "world_params.slot_to_world"}));

    const spade::RegisteredArray* entry = arenas.registry().find("probe");
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->elem_size, sizeof(ProbeState));
    EXPECT_EQ(entry->world_count, 2u);
    EXPECT_EQ(entry->capacity_per_world, 3u);
    EXPECT_EQ(entry->element_count(), 6u);
    EXPECT_EQ(entry->byte_size(), 6u * sizeof(ProbeState));

    const auto span = arenas.array(*probe);
    ASSERT_TRUE(span.has_value());
    EXPECT_EQ(entry->data, reinterpret_cast<std::byte*>(span->data()))
        << "the walk must point at the same bytes the typed accessor exposes";
    EXPECT_EQ(entry->byte_size(), span->size_bytes());

    const spade::RegisteredArray* map_entry = arenas.registry().find("probe.slot_to_world");
    ASSERT_NE(map_entry, nullptr);
    EXPECT_EQ(map_entry->elem_size, sizeof(uint32_t));
    EXPECT_EQ(map_entry->world_count, 2u);
    EXPECT_EQ(map_entry->capacity_per_world, 3u);
    const auto map = arenas.slot_to_world(*probe);
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(static_cast<const std::byte*>(map_entry->data), reinterpret_cast<const std::byte*>(map->data()));
    EXPECT_EQ(map_entry->byte_size(), map->size_bytes());
}

TEST(StateRegistryCanary, TheWalkAccountsForEveryByteOfEveryArenaArray) {
    spade::ArenaSet arenas(3);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 8);
    const auto params = arenas.register_array<spade::WorldParams>("world_params", 1);
    const auto probe = arenas.register_array<ProbeState>("probe", 2);
    ASSERT_TRUE(bodies.has_value());
    ASSERT_TRUE(params.has_value());
    ASSERT_TRUE(probe.has_value());

    const std::size_t expected = arenas.array(*bodies)->size_bytes() + arenas.slot_to_world(*bodies)->size_bytes() +
                                 arenas.array(*params)->size_bytes() + arenas.slot_to_world(*params)->size_bytes() +
                                 arenas.array(*probe)->size_bytes() + arenas.slot_to_world(*probe)->size_bytes();
    EXPECT_EQ(arenas.registry().total_bytes(), expected);

    // Walk order is registration order and is deterministic.
    EXPECT_EQ(walk_names(arenas.registry()),
              (std::vector<std::string>{"bodies", "bodies.slot_to_world", "world_params", "world_params.slot_to_world",
                                        "probe", "probe.slot_to_world"}));

    std::size_t visited = 0;
    arenas.registry().for_each_array([&visited](const spade::RegisteredArray& array) {
        EXPECT_NE(array.data, nullptr);
        EXPECT_EQ(array.element_count(), array.world_count * array.capacity_per_world);
        ++visited;
    });
    EXPECT_EQ(visited, arenas.registry().size());
}

TEST(StateRegistryCanary, NonArenaStateCanJoinTheSameWalk) {
    // The invariant is one-directional: everything in an arena is
    // registered, but the registry also accepts state it does not own --
    // Task 7's snapshot must fold rng stream states into the same walk.
    std::vector<uint64_t> rng_streams(4, 0);
    spade::StateRegistry registry;
    ASSERT_TRUE(registry
                    .register_array(spade::RegisteredArray{.name = "rng_streams",
                                                           .elem_size = sizeof(uint64_t),
                                                           .world_count = 1,
                                                           .capacity_per_world = 4,
                                                           .data = reinterpret_cast<std::byte*>(rng_streams.data())})
                    .has_value());
    EXPECT_EQ(registry.size(), 1u);
    EXPECT_EQ(registry.total_bytes(), 4u * sizeof(uint64_t));
    EXPECT_EQ(walk_names(registry), (std::vector<std::string>{"rng_streams"}));
}

TEST(StateRegistry, RejectsMalformedAndDuplicateRegistrations) {
    std::array<std::byte, 16> storage{};
    spade::StateRegistry registry;
    const spade::RegisteredArray good{
        .name = "a", .elem_size = 4, .world_count = 1, .capacity_per_world = 4, .data = storage.data()};

    ASSERT_TRUE(registry.register_array(good).has_value());

    auto duplicate = registry.register_array(good);
    ASSERT_FALSE(duplicate.has_value());
    EXPECT_EQ(duplicate.error().code, spade::Code::invalid_argument);

    spade::RegisteredArray unnamed = good;
    unnamed.name.clear();
    EXPECT_FALSE(registry.register_array(unnamed).has_value());

    spade::RegisteredArray zero_stride = good;
    zero_stride.name = "b";
    zero_stride.elem_size = 0;
    EXPECT_FALSE(registry.register_array(zero_stride).has_value());

    spade::RegisteredArray null_data = good;
    null_data.name = "c";
    null_data.data = nullptr;
    EXPECT_FALSE(registry.register_array(null_data).has_value());

    EXPECT_EQ(registry.size(), 1u);
    EXPECT_EQ(registry.find("missing"), nullptr);
}

TEST(StateRegistryCanary, ADuplicateArrayNameLeavesNoOrphanedArena) {
    spade::ArenaSet arenas(2);
    const auto first = arenas.register_array<spade::BodyState>("bodies", 4);
    ASSERT_TRUE(first.has_value());

    const auto clash = arenas.register_array<spade::BodyState>("bodies", 4);
    ASSERT_FALSE(clash.has_value());
    EXPECT_EQ(clash.error().code, spade::Code::invalid_argument);

    // The rejected arena must not linger: an unregistered arena would be
    // exactly the state a snapshot walk misses. Nor may it leave a
    // half-registered pair -- the two entries go in atomically.
    EXPECT_EQ(arenas.registry().size(), 2u);
    EXPECT_EQ(walk_names(arenas.registry()), (std::vector<std::string>{"bodies", "bodies.slot_to_world"}));
    const auto still_works = arenas.alloc_slot(*first, 0);
    ASSERT_TRUE(still_works.has_value());
    EXPECT_EQ(*still_works, 0u);
}

TEST(StateRegistryCanary, AClashWithAGeneratedMapNameRegistersNeitherEntry) {
    spade::ArenaSet arenas(1);
    // Occupy the name ArenaSet would synthesize for "x"'s map.
    ASSERT_TRUE(arenas.register_array<ProbeState>("x.slot_to_world", 2).has_value());
    const std::size_t before = arenas.registry().size();

    const auto clash = arenas.register_array<spade::BodyState>("x", 2);
    ASSERT_FALSE(clash.has_value());
    EXPECT_EQ(clash.error().code, spade::Code::invalid_argument);

    // "x" itself must not have slipped in on its own.
    EXPECT_EQ(arenas.registry().size(), before);
    EXPECT_EQ(arenas.registry().find("x"), nullptr);
}

// ---------------------------------------------------------------------------
// The completeness claim: a registry walk carries everything a restore needs.
// ---------------------------------------------------------------------------

TEST(StateSnapshotWalk, TheWalkAloneCarriesEverythingNeededToRestoreAnArena) {
    constexpr uint32_t kWorlds = 3;
    constexpr uint32_t kCapacity = 4;

    spade::ArenaSet saved(kWorlds);
    const auto saved_bodies = saved.register_array<spade::BodyState>("bodies", kCapacity);
    ASSERT_TRUE(saved_bodies.has_value());

    // Churn so the free lists are non-trivial: world 0 full then holed below
    // its bump cursor, world 1 partially used with a hole, world 2 untouched.
    for (uint32_t world : {0u, 0u, 0u, 0u, 1u, 1u, 1u}) {
        ASSERT_TRUE(saved.alloc_slot(*saved_bodies, world).has_value());
    }
    ASSERT_TRUE(saved.free_slot(*saved_bodies, 1).has_value());
    ASSERT_TRUE(saved.free_slot(*saved_bodies, 2).has_value());
    ASSERT_TRUE(saved.free_slot(*saved_bodies, 5).has_value());
    (*saved.array(*saved_bodies))[3].mass = 11.0f;

    // Snapshot == registry walk, and nothing else.
    std::map<std::string, std::vector<std::byte>> blob;
    saved.registry().for_each_array([&blob](const spade::RegisteredArray& array) {
        blob[array.name].assign(array.data, array.data + array.byte_size());
    });
    EXPECT_EQ(blob.size(), 2u) << "one arena contributes its elements and its slot_to_world map";

    // Restore into a fresh, identically-shaped set by writing through that
    // same walk -- RegisteredArray::data is mutable for exactly this reason.
    spade::ArenaSet restored(kWorlds);
    const auto restored_bodies = restored.register_array<spade::BodyState>("bodies", kCapacity);
    ASSERT_TRUE(restored_bodies.has_value());
    restored.registry().for_each_array([&blob](const spade::RegisteredArray& array) {
        const auto it = blob.find(array.name);
        ASSERT_NE(it, blob.end());
        ASSERT_EQ(it->second.size(), array.byte_size());
        std::memcpy(array.data, it->second.data(), it->second.size());
    });
    ASSERT_TRUE(restored.resync_from_slot_to_world(*restored_bodies).has_value());

    EXPECT_EQ((*restored.array(*restored_bodies))[3].mass, 11.0f);
    for (uint32_t world = 0; world < kWorlds; ++world) {
        const auto restored_live = restored.live_count(*restored_bodies, world);
        const auto saved_live = saved.live_count(*saved_bodies, world);
        ASSERT_TRUE(restored_live.has_value());
        ASSERT_TRUE(saved_live.has_value());
        EXPECT_EQ(*restored_live, *saved_live) << "world " << world;
    }

    // The real claim: every FUTURE allocation matches too, including the
    // capacity_exceeded ones. If the free lists held any information the
    // walk did not carry, this is where it would show.
    for (uint32_t world : {0u, 0u, 1u, 2u, 0u, 1u, 2u, 1u}) {
        const auto from_saved = saved.alloc_slot(*saved_bodies, world);
        const auto from_restored = restored.alloc_slot(*restored_bodies, world);
        ASSERT_EQ(from_saved.has_value(), from_restored.has_value()) << "world " << world;
        if (from_saved.has_value()) {
            EXPECT_EQ(*from_saved, *from_restored) << "world " << world;
        } else {
            EXPECT_EQ(from_saved.error().code, from_restored.error().code);
        }
    }
}

TEST(StateSnapshotWalk, ResyncRejectsAMapThatDisagreesWithThePartitionShape) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 3);
    ASSERT_TRUE(bodies.has_value());

    // Corrupt the map through the walk: slot 0 belongs to world 0, so
    // claiming world 1 owns it is not self-consistent.
    const spade::RegisteredArray* entry = arenas.registry().find("bodies.slot_to_world");
    ASSERT_NE(entry, nullptr);
    const uint32_t wrong_world = 1;
    std::memcpy(entry->data, &wrong_world, sizeof(wrong_world));

    const auto resynced = arenas.resync_from_slot_to_world(*bodies);
    ASSERT_FALSE(resynced.has_value());
    EXPECT_EQ(resynced.error().code, spade::Code::invalid_argument);
}

// ---------------------------------------------------------------------------
// Typed ids and move semantics.
// ---------------------------------------------------------------------------

TEST(ArenaSetTypedIds, ElementSizeMismatchIsRejected) {
    spade::ArenaSet arenas(1);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 2);
    ASSERT_TRUE(bodies.has_value());

    // An ArrayId<WorldParams> pointing at the bodies array -- what an id
    // minted by a different ArenaSet would look like.
    const spade::ArrayId<spade::WorldParams> wrong{bodies->index};
    const auto span = arenas.array(wrong);
    ASSERT_FALSE(span.has_value());
    EXPECT_EQ(span.error().code, spade::Code::invalid_argument);
}

TEST(ArenaSetTypedIds, MovingAnArenaSetKeepsRegistryPointersValid) {
    spade::ArenaSet source(2);
    const auto bodies = source.register_array<spade::BodyState>("bodies", 4);
    ASSERT_TRUE(bodies.has_value());
    const auto slot = source.alloc_slot(*bodies, 1);
    ASSERT_TRUE(slot.has_value());
    (*source.array(*bodies))[*slot].mass = 2.5f;
    std::byte* before = source.registry().find("bodies")->data;

    spade::ArenaSet moved = std::move(source);
    const spade::RegisteredArray* entry = moved.registry().find("bodies");
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->data, before) << "arena bytes are heap-stable across a move";
    EXPECT_EQ(entry->data, reinterpret_cast<std::byte*>(moved.array(*bodies)->data()));
    EXPECT_EQ((*moved.array(*bodies))[*slot].mass, 2.5f);

    const auto live = moved.live_count(*bodies, 1);
    ASSERT_TRUE(live.has_value());
    EXPECT_EQ(*live, 1u);
}

TEST(ArenaSetTypedIds, LaterRegistrationsDoNotInvalidateEarlierArrayPointers) {
    spade::ArenaSet arenas(2);
    const auto bodies = arenas.register_array<spade::BodyState>("bodies", 4);
    ASSERT_TRUE(bodies.has_value());
    spade::BodyState* first = arenas.array(*bodies)->data();

    for (int i = 0; i < 8; ++i) {
        ASSERT_TRUE(arenas.register_array<ProbeState>("probe" + std::to_string(i), 2).has_value());
    }

    EXPECT_EQ(arenas.array(*bodies)->data(), first) << "growing the arena vector must not move arena bytes";
    EXPECT_EQ(arenas.registry().find("bodies")->data, reinterpret_cast<std::byte*>(first));
}
