#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "core/time.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "state/snapshot.hpp"

// ---------------------------------------------------------------------------
// Snapshot save/restore -- the replay/determinism contract.
//
// The claim these tests defend is not "the bytes come back" (that is a memcpy)
// but "a run resumed from a snapshot is indistinguishable from the run that
// was never interrupted". That has three parts, each with its own section
// below: the walk carries every authoritative byte; the DERIVED state that
// the walk deliberately does not carry is rebuilt (the resync proof); and a
// blob that does not describe this state -- or is not a well-formed blob at
// all -- is refused without touching a single byte of the target.
// ---------------------------------------------------------------------------

namespace {

constexpr uint32_t kWorlds = 3;
constexpr uint32_t kBodyCapacity = 4;
constexpr uint32_t kParamCapacity = 1;

// Stand-in for the kind of state that lives outside any arena -- Task 8's rng
// streams are the real instance. The snapshot has never heard of this type;
// it is carried because it is in the walk, and for no other reason.
struct StreamState {
    uint64_t counter;
    uint64_t key;
};

// An arena set in the engine's shape, plus its ids.
struct Sim {
    spade::ArenaSet arenas;
    spade::ArrayId<spade::BodyState> bodies{};
    spade::ArrayId<spade::WorldParams> params{};

    explicit Sim(uint32_t worlds = kWorlds, uint32_t body_capacity = kBodyCapacity,
                 const std::string& body_name = "bodies")
        : arenas(worlds) {
        bodies = arenas.register_array<spade::BodyState>(body_name, body_capacity).value();
        params = arenas.register_array<spade::WorldParams>("world_params", kParamCapacity).value();
    }
};

// Puts a set into a state whose free lists are non-trivial: world 0 filled and
// then holed below its bump cursor, world 1 partially used with a hole, world
// 2 untouched. A restore that only copies bytes looks identical to a correct
// one until exactly this shape is allocated from again.
void churn(Sim& sim) {
    for (uint32_t world : {0u, 0u, 0u, 0u, 1u, 1u, 1u}) {
        ASSERT_TRUE(sim.arenas.alloc_slot(sim.bodies, world).has_value());
    }
    ASSERT_TRUE(sim.arenas.free_slot(sim.bodies, 1).has_value());
    ASSERT_TRUE(sim.arenas.free_slot(sim.bodies, 2).has_value());
    ASSERT_TRUE(sim.arenas.free_slot(sim.bodies, 5).has_value());

    const auto bodies = sim.arenas.array(sim.bodies);
    ASSERT_TRUE(bodies.has_value());
    (*bodies)[0].pos = glm::vec3(1.0f, 2.0f, 3.0f);
    (*bodies)[0].mass = 1.5f;
    (*bodies)[3].orient = glm::quat(0.5f, 0.5f, 0.5f, 0.5f);
    (*bodies)[3].mass = 2.25f;
    (*bodies)[4].specific_force = glm::vec3(0.0f, 0.0f, -9.81f);
    (*bodies)[6].omega_body = glm::vec3(0.1f, -0.2f, 0.3f);

    for (uint32_t world = 0; world < kWorlds; ++world) {
        ASSERT_TRUE(sim.arenas.alloc_slot(sim.params, world).has_value());
    }
    const auto params = sim.arenas.array(sim.params);
    ASSERT_TRUE(params.has_value());
    (*params)[0].gravity = glm::vec3(0.0f, 0.0f, -9.81f);
    (*params)[0].air_density = 1.225f;
    (*params)[0].body_capacity = kBodyCapacity;
    (*params)[0].body_count = 2;
    (*params)[0].seed = 0x0123456789abcdefULL;
    (*params)[2].wind = glm::vec3(3.0f, 0.0f, 0.0f);
}

// One registered array's identity and bytes, as the walk sees it.
struct ArrayImage {
    std::string name;
    std::vector<std::byte> bytes;
};

std::vector<ArrayImage> walk_images(const spade::StateRegistry& registry) {
    std::vector<ArrayImage> images;
    registry.for_each_array([&images](const spade::RegisteredArray& array) {
        ArrayImage image;
        image.name = array.name;
        if (array.byte_size() != 0) {
            image.bytes.assign(array.data, array.data + array.byte_size());
        }
        images.push_back(std::move(image));
    });
    return images;
}

// The bit-identity check the brief asks for: memcmp, per registered array.
void expect_images_identical(const std::vector<ArrayImage>& expected, const std::vector<ArrayImage>& actual) {
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(expected[i].name, actual[i].name) << "walk position " << i;
        ASSERT_EQ(expected[i].bytes.size(), actual[i].bytes.size()) << "array '" << expected[i].name << "'";
        EXPECT_EQ(0, std::memcmp(expected[i].bytes.data(), actual[i].bytes.data(), expected[i].bytes.size()))
            << "array '" << expected[i].name << "' differs byte-wise";
    }
}

std::vector<std::byte> blob_bytes(const spade::SnapshotBlob& blob) {
    const std::span<const std::byte> span = blob.bytes();
    return std::vector<std::byte>(span.begin(), span.end());
}

template <class T>
void poke(std::vector<std::byte>& bytes, std::size_t offset, const T& value) {
    ASSERT_LE(offset + sizeof(T), bytes.size());
    std::memcpy(bytes.data() + offset, &value, sizeof(T));
}

template <class T>
T peek(const std::vector<std::byte>& bytes, std::size_t offset) {
    T value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    return value;
}

// Where each section lives inside a blob. A second, independent reader of the
// format: if snapshot.cpp's writer and this walk ever disagree the corruption
// tests below stop finding what they aim at, which is itself a signal.
struct BlobSection {
    std::string name;
    std::size_t header_offset = 0;
    std::size_t payload_offset = 0;
    std::size_t payload_bytes = 0;
};

std::vector<BlobSection> blob_sections(const spade::SnapshotBlob& blob) {
    std::vector<BlobSection> sections;
    const std::span<const std::byte> bytes = blob.bytes();
    std::size_t offset = sizeof(spade::SnapshotHeader);
    for (uint32_t i = 0; i < blob.array_count(); ++i) {
        if (bytes.size() - offset < sizeof(spade::SnapshotSectionHeader)) return {};
        spade::SnapshotSectionHeader head{};
        std::memcpy(&head, bytes.data() + offset, sizeof(head));

        BlobSection section;
        section.header_offset = offset;
        offset += sizeof(head);
        section.name.assign(reinterpret_cast<const char*>(bytes.data() + offset), head.name_length);
        offset += head.name_length;
        section.payload_offset = offset;
        section.payload_bytes = static_cast<std::size_t>(head.byte_length);
        offset += section.payload_bytes;
        sections.push_back(std::move(section));
    }
    return sections;
}

const BlobSection* find_section(const std::vector<BlobSection>& sections, std::string_view name) {
    for (const BlobSection& section : sections) {
        if (section.name == name) return &section;
    }
    return nullptr;
}

std::filesystem::path temp_blob_path(const std::string& stem) {
    std::error_code ec;
    std::filesystem::path path = std::filesystem::temp_directory_path(ec) / ("spade_snapshot_" + stem + ".bin");
    return path;
}

void remove_quietly(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

}  // namespace

// ---------------------------------------------------------------------------
// Round trip: the walk carries every authoritative byte.
// ---------------------------------------------------------------------------

TEST(SnapshotRoundTrip, EveryRegisteredArrayComesBackByteIdentical) {
    Sim source;
    churn(source);
    const std::vector<ArrayImage> expected = walk_images(source.arenas.registry());
    ASSERT_EQ(expected.size(), 4u) << "two arenas, each contributing its elements and its slot_to_world map";

    const auto blob = spade::save(source.arenas, spade::Tick{1234});
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    Sim target;
    ASSERT_TRUE(spade::restore(target.arenas, *blob).has_value());
    expect_images_identical(expected, walk_images(target.arenas.registry()));
}

TEST(SnapshotRoundTrip, HeaderCarriesTickSchemaAndWorldSetShape) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{1234});
    ASSERT_TRUE(blob.has_value());

    const spade::SnapshotHeader head = blob->header();
    EXPECT_EQ(head.magic, spade::kSnapshotMagic);
    EXPECT_EQ(head.version, spade::kSnapshotVersion);
    EXPECT_EQ(blob->tick(), spade::Tick{1234});
    EXPECT_EQ(blob->world_count(), kWorlds);
    EXPECT_EQ(blob->array_count(), 4u);
    EXPECT_EQ(blob->schema_hash(), spade::schema_hash(source.arenas.registry()));
    EXPECT_EQ(spade::world_set_size(source.arenas.registry()), source.arenas.world_count());

    // A hex dump of the first four bytes reads SPSN on this (little-endian)
    // host, which is the point of spelling the magic that way.
    const std::vector<std::byte> bytes = blob_bytes(*blob);
    EXPECT_EQ(static_cast<char>(bytes[0]), 'S');
    EXPECT_EQ(static_cast<char>(bytes[1]), 'P');
    EXPECT_EQ(static_cast<char>(bytes[2]), 'S');
    EXPECT_EQ(static_cast<char>(bytes[3]), 'N');

    // The whole format, restated: header + per-section (fixed prefix, name,
    // payload). Computed from the registry rather than hardcoded, so it pins
    // the layout without pinning today's array sizes.
    std::size_t expected_size = sizeof(spade::SnapshotHeader);
    source.arenas.registry().for_each_array([&expected_size](const spade::RegisteredArray& array) {
        expected_size += sizeof(spade::SnapshotSectionHeader) + array.name.size() + array.byte_size();
    });
    EXPECT_EQ(blob->size(), expected_size);
}

TEST(SnapshotRoundTrip, RestoringOverAMutatedSetReturnsItToTheSnapshotState) {
    Sim sim;
    churn(sim);
    const std::vector<ArrayImage> at_snapshot = walk_images(sim.arenas.registry());
    const auto blob = spade::save(sim.arenas, spade::Tick{7});
    ASSERT_TRUE(blob.has_value());

    // Move on: allocate, free, and overwrite values.
    ASSERT_TRUE(sim.arenas.alloc_slot(sim.bodies, 2).has_value());
    ASSERT_TRUE(sim.arenas.free_slot(sim.bodies, 0).has_value());
    (*sim.arenas.array(sim.bodies))[3].mass = -1.0f;
    (*sim.arenas.array(sim.params))[0].seed = 0;
    EXPECT_NE(0, std::memcmp(at_snapshot[0].bytes.data(), sim.arenas.registry().find("bodies")->data,
                             at_snapshot[0].bytes.size()));

    ASSERT_TRUE(spade::restore(sim.arenas, *blob).has_value());
    expect_images_identical(at_snapshot, walk_images(sim.arenas.registry()));
}

TEST(SnapshotRoundTrip, AnEmptyRegistryRoundTrips) {
    const spade::StateRegistry empty;
    const auto blob = spade::save(empty, spade::Tick{99});
    ASSERT_TRUE(blob.has_value());
    EXPECT_EQ(blob->size(), sizeof(spade::SnapshotHeader));
    EXPECT_EQ(blob->array_count(), 0u);
    EXPECT_EQ(blob->world_count(), 0u);
    EXPECT_EQ(blob->tick(), spade::Tick{99});

    const spade::StateRegistry also_empty;
    EXPECT_TRUE(spade::restore(also_empty, *blob).has_value());

    // ...but not into a registry that has arrays.
    Sim sim;
    const auto rejected = spade::restore(sim.arenas, *blob);
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error().code, spade::Code::schema_mismatch);
}

TEST(SnapshotWalk, NonArenaRegisteredStateIsCarriedWithoutASpecialCase) {
    // Exactly the shape Task 8's rng streams register with: state that no
    // arena owns, joining the same walk. Nothing in snapshot.cpp mentions it.
    std::vector<StreamState> source_streams{{1, 2}, {3, 4}, {5, 6}, {7, 8}};
    spade::StateRegistry source;
    ASSERT_TRUE(source
                    .register_array({.name = "rng.streams",
                                     .elem_size = sizeof(StreamState),
                                     .world_count = 1,
                                     .capacity_per_world = 4,
                                     .data = reinterpret_cast<std::byte*>(source_streams.data())})
                    .has_value());

    const auto blob = spade::save(source, spade::Tick{5});
    ASSERT_TRUE(blob.has_value());

    std::vector<StreamState> target_streams(4, StreamState{0, 0});
    spade::StateRegistry target;
    ASSERT_TRUE(target
                    .register_array({.name = "rng.streams",
                                     .elem_size = sizeof(StreamState),
                                     .world_count = 1,
                                     .capacity_per_world = 4,
                                     .data = reinterpret_cast<std::byte*>(target_streams.data())})
                    .has_value());

    ASSERT_TRUE(spade::restore(target, *blob).has_value());
    EXPECT_EQ(0, std::memcmp(source_streams.data(), target_streams.data(), source_streams.size() * sizeof(StreamState)));
}

// ---------------------------------------------------------------------------
// The resync proof: derived state the walk does not carry is rebuilt, and a
// resume allocates exactly what the uninterrupted run would have.
// ---------------------------------------------------------------------------

TEST(SnapshotResync, AllocationsAfterARestoreMatchTheUninterruptedRun) {
    Sim uninterrupted;
    churn(uninterrupted);

    const auto blob = spade::save(uninterrupted.arenas, spade::Tick{100});
    ASSERT_TRUE(blob.has_value());

    Sim resumed;
    ASSERT_TRUE(spade::restore(resumed.arenas, *blob).has_value());

    for (uint32_t world = 0; world < kWorlds; ++world) {
        const auto live_there = uninterrupted.arenas.live_count(uninterrupted.bodies, world);
        const auto live_here = resumed.arenas.live_count(resumed.bodies, world);
        ASSERT_TRUE(live_there.has_value());
        ASSERT_TRUE(live_here.has_value());
        EXPECT_EQ(*live_there, *live_here) << "world " << world;
    }

    // The claim in full: every FUTURE allocation agrees, including the ones
    // that fail with capacity_exceeded. Free lists are not in the blob, so if
    // they carried any information the walk did not, this is where the two
    // runs would part company.
    for (uint32_t world : {0u, 0u, 1u, 2u, 0u, 1u, 2u, 1u, 0u}) {
        const auto there = uninterrupted.arenas.alloc_slot(uninterrupted.bodies, world);
        const auto here = resumed.arenas.alloc_slot(resumed.bodies, world);
        ASSERT_EQ(there.has_value(), here.has_value()) << "world " << world;
        if (there.has_value()) {
            EXPECT_EQ(*there, *here) << "world " << world;
        } else {
            EXPECT_EQ(there.error().code, here.error().code) << "world " << world;
        }
    }

    // And the bytes still agree afterwards, so the two sets really are one
    // state, not two states that happen to allocate alike.
    expect_images_identical(walk_images(uninterrupted.arenas.registry()), walk_images(resumed.arenas.registry()));
}

TEST(SnapshotResync, ARegistryLevelRestoreLeavesDerivedStateStale) {
    // Pins WHY restore(ArenaSet&) exists. The registry-level overload is a
    // byte copy and nothing more: correct for a registry that owns no arenas,
    // silently divergent for one that does. This test asserts the divergence
    // so that "restore then resync" can never be quietly dropped to "restore".
    Sim uninterrupted;
    churn(uninterrupted);
    const auto blob = spade::save(uninterrupted.arenas, spade::Tick{100});
    ASSERT_TRUE(blob.has_value());

    Sim resumed;
    ASSERT_TRUE(spade::restore(resumed.arenas.registry(), *blob).has_value());

    // Bytes: identical. Derived state: untouched, therefore wrong.
    expect_images_identical(walk_images(uninterrupted.arenas.registry()), walk_images(resumed.arenas.registry()));
    EXPECT_EQ(*uninterrupted.arenas.live_count(uninterrupted.bodies, 0), 2u);
    EXPECT_EQ(*resumed.arenas.live_count(resumed.bodies, 0), 0u) << "the free lists were not rebuilt";

    const auto there = uninterrupted.arenas.alloc_slot(uninterrupted.bodies, 0);
    const auto here = resumed.arenas.alloc_slot(resumed.bodies, 0);
    ASSERT_TRUE(there.has_value());
    ASSERT_TRUE(here.has_value());
    EXPECT_EQ(*there, 1u) << "lowest free slot in world 0";
    EXPECT_EQ(*here, 0u) << "a set with an empty free list hands out a slot that is live in the restored map";
    EXPECT_NE(*there, *here);
}

TEST(SnapshotResync, AnInconsistentSlotMapIsRejectedBeforeAnythingIsWritten) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{3});
    ASSERT_TRUE(blob.has_value());

    const std::vector<BlobSection> sections = blob_sections(*blob);
    const BlobSection* map = find_section(sections, "bodies.slot_to_world");
    ASSERT_NE(map, nullptr);

    // Slot 0 lives in world 0's partition; claiming world 1 owns it is not a
    // state any ArenaSet can be in.
    std::vector<std::byte> corrupt = blob_bytes(*blob);
    poke<uint32_t>(corrupt, map->payload_offset, 1u);
    const auto bad = spade::SnapshotBlob::from_bytes(std::move(corrupt));
    ASSERT_TRUE(bad.has_value()) << "the header is intact; only a payload changed";

    Sim target;
    const std::vector<ArrayImage> before = walk_images(target.arenas.registry());
    const auto restored = spade::restore(target.arenas, *bad);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::io_error) << restored.error().context;
    expect_images_identical(before, walk_images(target.arenas.registry()));

    // The registry-level overload has no arena semantics, so the same blob is
    // merely bytes to it. That asymmetry is deliberate: the map invariant
    // belongs to whoever owns the free lists.
    Sim permissive;
    EXPECT_TRUE(spade::restore(permissive.arenas.registry(), *bad).has_value());
}

// ---------------------------------------------------------------------------
// Schema identity.
// ---------------------------------------------------------------------------

TEST(SnapshotSchema, MutatingTheSchemaHashIsRejected) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    constexpr std::size_t kHashAt = offsetof(spade::SnapshotHeader, schema_hash);
    std::vector<std::byte> bytes = blob_bytes(*blob);
    poke<uint64_t>(bytes, kHashAt, peek<uint64_t>(bytes, kHashAt) ^ uint64_t{1});
    const auto mutated = spade::SnapshotBlob::from_bytes(std::move(bytes));
    ASSERT_TRUE(mutated.has_value());

    Sim target;
    const std::vector<ArrayImage> before = walk_images(target.arenas.registry());
    const auto restored = spade::restore(target.arenas, *mutated);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::schema_mismatch);
    expect_images_identical(before, walk_images(target.arenas.registry()));
}

TEST(SnapshotSchema, AWrongWorldSetShapeIsRejected) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    // Fewer worlds.
    Sim narrower(kWorlds - 1, kBodyCapacity);
    const auto by_worlds = spade::restore(narrower.arenas, *blob);
    ASSERT_FALSE(by_worlds.has_value());
    EXPECT_EQ(by_worlds.error().code, spade::Code::schema_mismatch);

    // Same worlds, bigger per-world capacity.
    Sim roomier(kWorlds, kBodyCapacity * 2);
    const auto by_capacity = spade::restore(roomier.arenas, *blob);
    ASSERT_FALSE(by_capacity.has_value());
    EXPECT_EQ(by_capacity.error().code, spade::Code::schema_mismatch);

    // Same shape, different names.
    Sim renamed(kWorlds, kBodyCapacity, "drones");
    const auto by_name = spade::restore(renamed.arenas, *blob);
    ASSERT_FALSE(by_name.has_value());
    EXPECT_EQ(by_name.error().code, spade::Code::schema_mismatch);

    // Same arrays plus one more.
    Sim extra;
    ASSERT_TRUE(extra.arenas.register_array<spade::BodyState>("debris", 2).has_value());
    const auto by_count = spade::restore(extra.arenas, *blob);
    ASSERT_FALSE(by_count.has_value());
    EXPECT_EQ(by_count.error().code, spade::Code::schema_mismatch);
}

TEST(SnapshotSchema, TheHashSeparatesNamesElementSizesAndExtents) {
    static std::byte storage[256];

    const auto hash_of = [](const std::string& name, uint32_t elem_size, uint32_t worlds, uint32_t capacity) {
        spade::StateRegistry registry;
        EXPECT_TRUE(registry
                        .register_array({.name = name,
                                         .elem_size = elem_size,
                                         .world_count = worlds,
                                         .capacity_per_world = capacity,
                                         .data = storage})
                        .has_value());
        return spade::schema_hash(registry);
    };

    const uint64_t base = hash_of("bodies", 16, 2, 4);
    EXPECT_EQ(base, hash_of("bodies", 16, 2, 4)) << "the hash is a function of shape alone";
    EXPECT_NE(base, hash_of("bodie3", 16, 2, 4));
    EXPECT_NE(base, hash_of("bodies2", 16, 2, 4));
    EXPECT_NE(base, hash_of("bodies", 8, 2, 4));
    EXPECT_NE(base, hash_of("bodies", 16, 4, 4));
    EXPECT_NE(base, hash_of("bodies", 16, 2, 8));

    // Array count is part of the identity, and so is order.
    spade::StateRegistry two;
    ASSERT_TRUE(two.register_array({.name = "a", .elem_size = 4, .world_count = 1, .capacity_per_world = 1, .data = storage})
                    .has_value());
    ASSERT_TRUE(two.register_array({.name = "b", .elem_size = 4, .world_count = 1, .capacity_per_world = 1, .data = storage})
                    .has_value());
    spade::StateRegistry reversed;
    ASSERT_TRUE(reversed
                    .register_array({.name = "b", .elem_size = 4, .world_count = 1, .capacity_per_world = 1, .data = storage})
                    .has_value());
    ASSERT_TRUE(reversed
                    .register_array({.name = "a", .elem_size = 4, .world_count = 1, .capacity_per_world = 1, .data = storage})
                    .has_value());
    EXPECT_NE(spade::schema_hash(two), hash_of("a", 4, 1, 1));
    EXPECT_NE(spade::schema_hash(two), spade::schema_hash(reversed));
}

// ---------------------------------------------------------------------------
// Blob parsing: nothing a blob declares is believed.
// ---------------------------------------------------------------------------

TEST(SnapshotBlobParsing, TruncationAtEveryLengthIsRejectedAndWritesNothing) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());
    const std::vector<std::byte> bytes = blob_bytes(*blob);
    ASSERT_GT(bytes.size(), sizeof(spade::SnapshotHeader));

    Sim target;
    const std::vector<ArrayImage> before = walk_images(target.arenas.registry());

    // Every proper prefix of a valid blob. Cheap enough to do exhaustively,
    // and exhaustive is the point: this is the overread hunt.
    for (std::size_t length = 0; length < bytes.size(); ++length) {
        std::vector<std::byte> truncated(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(length));
        const auto parsed = spade::SnapshotBlob::from_bytes(std::move(truncated));
        if (!parsed) {
            ASSERT_EQ(parsed.error().code, spade::Code::io_error) << "length " << length;
            continue;  // shorter than a header: rejected at the door
        }
        const auto restored = spade::restore(target.arenas, *parsed);
        ASSERT_FALSE(restored.has_value()) << "length " << length;
        EXPECT_EQ(restored.error().code, spade::Code::io_error) << "length " << length << ": " << restored.error().context;
    }

    expect_images_identical(before, walk_images(target.arenas.registry()));
}

TEST(SnapshotBlobParsing, TrailingBytesAreRejected) {
    Sim source;
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    std::vector<std::byte> bytes = blob_bytes(*blob);
    bytes.push_back(std::byte{0});
    const auto parsed = spade::SnapshotBlob::from_bytes(std::move(bytes));
    ASSERT_TRUE(parsed.has_value());

    Sim target;
    const auto restored = spade::restore(target.arenas, *parsed);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::io_error) << restored.error().context;
}

TEST(SnapshotBlobParsing, BadMagicAndUnknownVersionAreDistinguished) {
    Sim source;
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    std::vector<std::byte> wrong_magic = blob_bytes(*blob);
    poke<uint32_t>(wrong_magic, offsetof(spade::SnapshotHeader, magic), 0xdeadbeefU);
    const auto not_ours = spade::SnapshotBlob::from_bytes(std::move(wrong_magic));
    ASSERT_FALSE(not_ours.has_value());
    EXPECT_EQ(not_ours.error().code, spade::Code::io_error);

    std::vector<std::byte> wrong_version = blob_bytes(*blob);
    poke<uint32_t>(wrong_version, offsetof(spade::SnapshotHeader, version), spade::kSnapshotVersion + 1);
    const auto future = spade::SnapshotBlob::from_bytes(std::move(wrong_version));
    ASSERT_FALSE(future.has_value());
    EXPECT_EQ(future.error().code, spade::Code::schema_mismatch) << "our bytes, a format this build does not read";

    // An empty buffer is not a snapshot either.
    const auto nothing = spade::SnapshotBlob::from_bytes({});
    ASSERT_FALSE(nothing.has_value());
    EXPECT_EQ(nothing.error().code, spade::Code::io_error);
}

TEST(SnapshotBlobParsing, AMovedFromBlobIsInertRatherThanUndefined) {
    // A moved-from blob is the one state that does not satisfy the class
    // invariant, and its accessors must still not read off the end of an empty
    // buffer. Deliberate use-after-move: that is the case under test.
    Sim source;
    auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());
    const spade::SnapshotBlob moved_to = std::move(*blob);
    const spade::SnapshotBlob& moved_from = *blob;  // NOLINT(bugprone-use-after-move)

    // std::vector's moved-from state is "valid but unspecified"; because move
    // construction must be O(1) the buffer is stolen and the source left empty
    // everywhere in practice. Pinned rather than assumed: if a standard
    // library ever does otherwise, this line says so instead of the
    // expectations below failing mysteriously.
    ASSERT_LT(moved_from.size(), sizeof(spade::SnapshotHeader))
        << "this standard library does not empty a moved-from vector";

    EXPECT_EQ(moved_from.header().magic, 0u);
    EXPECT_EQ(moved_from.version(), 0u);
    EXPECT_EQ(moved_from.array_count(), 0u);

    Sim target;
    const auto restored = spade::restore(target.arenas, moved_from);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::schema_mismatch) << restored.error().context;

    // The blob that received the bytes is unaffected.
    EXPECT_EQ(moved_to.array_count(), 4u);
    EXPECT_TRUE(spade::restore(target.arenas, moved_to).has_value());
}

TEST(SnapshotBlobParsing, ASectionLengthPastTheEndIsRejected) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());
    const std::vector<BlobSection> sections = blob_sections(*blob);
    const BlobSection* bodies = find_section(sections, "bodies");
    ASSERT_NE(bodies, nullptr);

    Sim target;
    const std::vector<ArrayImage> before = walk_images(target.arenas.registry());

    // (a) A declared length that no longer matches the declared shape.
    {
        std::vector<std::byte> bytes = blob_bytes(*blob);
        poke<uint64_t>(bytes, bodies->header_offset + offsetof(spade::SnapshotSectionHeader, byte_length),
                       uint64_t{1} << 40);
        const auto parsed = spade::SnapshotBlob::from_bytes(std::move(bytes));
        ASSERT_TRUE(parsed.has_value());
        const auto restored = spade::restore(target.arenas, *parsed);
        ASSERT_FALSE(restored.has_value());
        EXPECT_EQ(restored.error().code, spade::Code::io_error) << restored.error().context;
    }

    // (b) A self-consistent but oversized shape: capacity and length doubled
    //     together, so only the bounds check can catch it.
    {
        std::vector<std::byte> bytes = blob_bytes(*blob);
        const std::size_t shape_at = bodies->header_offset + offsetof(spade::SnapshotSectionHeader, capacity_per_world);
        const std::size_t length_at = bodies->header_offset + offsetof(spade::SnapshotSectionHeader, byte_length);
        poke<uint32_t>(bytes, shape_at, peek<uint32_t>(bytes, shape_at) * 2);
        poke<uint64_t>(bytes, length_at, peek<uint64_t>(bytes, length_at) * 2);
        const auto parsed = spade::SnapshotBlob::from_bytes(std::move(bytes));
        ASSERT_TRUE(parsed.has_value());
        const auto restored = spade::restore(target.arenas, *parsed);
        ASSERT_FALSE(restored.has_value());
        EXPECT_EQ(restored.error().code, spade::Code::io_error) << restored.error().context;
    }

    expect_images_identical(before, walk_images(target.arenas.registry()));
}

TEST(SnapshotBlobParsing, AnInflatedArrayCountIsRejectedWithoutAllocating) {
    Sim source;
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    std::vector<std::byte> bytes = blob_bytes(*blob);
    poke<uint32_t>(bytes, offsetof(spade::SnapshotHeader, array_count), 0xffffffffU);
    const auto parsed = spade::SnapshotBlob::from_bytes(std::move(bytes));
    ASSERT_TRUE(parsed.has_value());

    // If the count were used to size a reservation this would try for tens of
    // gigabytes before failing.
    Sim target;
    const auto restored = spade::restore(target.arenas, *parsed);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::io_error) << restored.error().context;
}

// ---------------------------------------------------------------------------
// File IO.
// ---------------------------------------------------------------------------

TEST(SnapshotFile, WriteThenReadReproducesTheBlobAndRestores) {
    Sim source;
    churn(source);
    const std::vector<ArrayImage> expected = walk_images(source.arenas.registry());
    const auto blob = spade::save(source.arenas, spade::Tick{88});
    ASSERT_TRUE(blob.has_value());

    const std::filesystem::path path = temp_blob_path("roundtrip");
    remove_quietly(path);
    ASSERT_TRUE(blob->write_file(path).has_value());

    const auto reloaded = spade::SnapshotBlob::read_file(path);
    ASSERT_TRUE(reloaded.has_value()) << (reloaded ? "" : reloaded.error().context);
    ASSERT_EQ(reloaded->size(), blob->size());
    EXPECT_EQ(0, std::memcmp(reloaded->bytes().data(), blob->bytes().data(), blob->size()));
    EXPECT_EQ(reloaded->tick(), spade::Tick{88});

    Sim target;
    ASSERT_TRUE(spade::restore(target.arenas, *reloaded).has_value());
    expect_images_identical(expected, walk_images(target.arenas.registry()));

    // Overwriting an existing file must truncate it, not leave a longer old
    // blob's tail behind.
    const spade::StateRegistry empty;
    const auto small = spade::save(empty, spade::Tick{0});
    ASSERT_TRUE(small.has_value());
    ASSERT_TRUE(small->write_file(path).has_value());
    const auto reread = spade::SnapshotBlob::read_file(path);
    ASSERT_TRUE(reread.has_value());
    EXPECT_EQ(reread->size(), sizeof(spade::SnapshotHeader));

    remove_quietly(path);
}

TEST(SnapshotFile, FilesystemFailuresAreIoErrors) {
    const std::filesystem::path missing = temp_blob_path("does_not_exist");
    remove_quietly(missing);
    const auto absent = spade::SnapshotBlob::read_file(missing);
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, spade::Code::io_error);

    Sim source;
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());
    const auto unwritable = blob->write_file(temp_blob_path("no_such_dir") / "nested" / "blob.bin");
    ASSERT_FALSE(unwritable.has_value());
    EXPECT_EQ(unwritable.error().code, spade::Code::io_error);

    // A file that is not a snapshot at all.
    const std::filesystem::path junk = temp_blob_path("junk");
    {
        std::ofstream out(junk, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        const char noise[] = "this is not a snapshot blob, not even close";
        out.write(noise, static_cast<std::streamsize>(sizeof(noise)));
    }
    const auto garbage = spade::SnapshotBlob::read_file(junk);
    ASSERT_FALSE(garbage.has_value());
    EXPECT_EQ(garbage.error().code, spade::Code::io_error);
    remove_quietly(junk);
}

// ---------------------------------------------------------------------------
// find_section: reading ONE named section of a blob without restoring it.
//
// Its reason to exist is that a caller above this layer (Simulation::restore,
// which must read the `replay_config` row BEFORE it decides to restore at all)
// would otherwise write a second walk over the section table -- duplicating
// this file's whole trust boundary in a layer that has no business owning it.
// So the burden here is that it is the SAME parse: it must accept exactly the
// blobs restore() accepts and refuse exactly the ones restore() refuses.
//
// NOTE the name shadowing: this file already has an anonymous-namespace
// BlobSection/find_section pair -- a deliberately independent reader of the
// format, used by the corruption tests. Every reference below to the ENGINE's
// is qualified, and the two are compared against each other on purpose.
// ---------------------------------------------------------------------------

TEST(SnapshotFindSection, ReturnsTheExtentsAndBytesOfANamedArray) {
    Sim source;
    churn(source);
    const auto blob = spade::save(source.arenas, spade::Tick{77});
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    const std::vector<BlobSection> independent = blob_sections(*blob);
    ASSERT_FALSE(independent.empty());

    // Every section the independent reader sees, the engine's finder must find
    // -- with the same payload, at the same place.
    for (const BlobSection& expected : independent) {
        const auto found = spade::find_section(*blob, expected.name);
        ASSERT_TRUE(found.has_value()) << expected.name << ": " << found.error().context;
        EXPECT_EQ(found->payload.size(), expected.payload_bytes) << expected.name;
        EXPECT_EQ(found->payload.data(), blob->bytes().data() + expected.payload_offset) << expected.name;
        EXPECT_EQ(static_cast<std::size_t>(found->elem_size) * found->world_count * found->capacity_per_world,
                  expected.payload_bytes)
            << expected.name << ": declared shape must agree with the payload length";
    }

    // The extents are the registry's, field for field.
    const spade::RegisteredArray* registered = source.arenas.registry().find("bodies");
    ASSERT_NE(registered, nullptr);
    const auto bodies = spade::find_section(*blob, "bodies");
    ASSERT_TRUE(bodies.has_value()) << bodies.error().context;
    EXPECT_EQ(bodies->elem_size, registered->elem_size);
    EXPECT_EQ(bodies->world_count, registered->world_count);
    EXPECT_EQ(bodies->capacity_per_world, registered->capacity_per_world);
    ASSERT_EQ(bodies->payload.size(), registered->byte_size());
    EXPECT_EQ(std::memcmp(bodies->payload.data(), registered->data, bodies->payload.size()), 0)
        << "the located payload must be the array's bytes";
}

TEST(SnapshotFindSection, ReportsAMissingNameAndAMalformedBlobDifferently) {
    Sim source;
    const auto blob = spade::save(source.arenas, spade::Tick{1});
    ASSERT_TRUE(blob.has_value());

    // Present, absent, and near-miss names. A section name is matched whole:
    // "bodie" and "bodiess" are not "bodies".
    EXPECT_TRUE(spade::find_section(*blob, "bodies").has_value());
    for (const char* absent : {"", "bodie", "bodiess", "replay_config"}) {
        const auto missing = spade::find_section(*blob, absent);
        ASSERT_FALSE(missing.has_value()) << "'" << absent << "'";
        EXPECT_EQ(missing.error().code, spade::Code::not_found) << "'" << absent << "'";
    }

    // A TRUNCATED blob is io_error, not "not found" -- the same verdict
    // restore() reaches on the same bytes, which is the point of sharing one
    // parser. Truncating inside the last section's payload leaves the FIRST
    // section perfectly readable, so a finder that scanned to its first hit and
    // stopped would happily hand out a section from a blob that does not parse.
    std::vector<std::byte> raw(blob->bytes().begin(), blob->bytes().end() - 1);
    const auto truncated = spade::SnapshotBlob::from_bytes(std::move(raw));
    ASSERT_TRUE(truncated.has_value());
    const auto found = spade::find_section(*truncated, "bodies");
    ASSERT_FALSE(found.has_value());
    EXPECT_EQ(found.error().code, spade::Code::io_error);

    Sim target;
    const auto rejected = spade::restore(target.arenas, *truncated);
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error().code, found.error().code)
        << "find_section and restore must agree about whether a blob parses";
}

// A blob written before the module API is format version 1. It is refused
// with the version message, never misread as a 40-byte header. (A v2 blob with
// its version field set back to 1 stands in for one.)
TEST(SnapshotFormat, AVersionOneBlobIsRefusedWithTheVersionMessage) {
    spade::StateRegistry registry;
    const auto blob = spade::save(registry, spade::Tick{1});
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    std::vector<std::byte> bytes(blob->bytes().begin(), blob->bytes().end());
    const uint32_t v1 = 1;
    std::memcpy(bytes.data() + offsetof(spade::SnapshotHeader, version), &v1, sizeof(v1));
    const auto reread = spade::SnapshotBlob::from_bytes(std::move(bytes));
    ASSERT_FALSE(reread.has_value());
    EXPECT_EQ(reread.error().code, spade::Code::schema_mismatch);
    EXPECT_NE(reread.error().context.find("format version 1"), std::string::npos) << reread.error().context;
}

TEST(SnapshotFormat, TheConfigurationIdentityRoundTripsThroughTheHeader) {
    spade::StateRegistry registry;
    const auto blob = spade::save(registry, spade::Tick{7}, 0x1234'5678'9ABC'DEF0ULL);
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    EXPECT_EQ(blob->version(), 2u);
    EXPECT_EQ(blob->configuration_identity(), 0x1234'5678'9ABC'DEF0ULL);
    const auto reread = spade::SnapshotBlob::from_bytes(
        std::vector<std::byte>(blob->bytes().begin(), blob->bytes().end()));
    ASSERT_TRUE(reread.has_value()) << reread.error().context;
    EXPECT_EQ(reread->configuration_identity(), 0x1234'5678'9ABC'DEF0ULL);
}
