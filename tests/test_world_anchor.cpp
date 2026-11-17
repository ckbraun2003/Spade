// SP-2 of the geo plan (L85, docs/design/core/2026-11-17-geo-g1-design.md): a world's optional geodetic ANCHOR.
//
// What is pinned: the anchor is part of the world and its file ONLY when set. An anchor-less world is written as `world_version: 2`
// with no `anchor` key, exactly as before; an anchored one is written as `world_version: 3` (v3 is v2 plus optional keys, and the
// writer writes the lowest version that holds the world), with its doubles bit for bit; a v2 file with an `anchor` key is an unknown
// key; a v3 file without one loads as a plain world. The anchor folds into config_hash and into the replay row's anchor identity
// only when set, and a restore under another anchor is refused by name (Code::invalid_argument, "world anchor differs").
#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/geodesy.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

namespace {

namespace geo = spade::geo;

[[nodiscard]] spade::WorldDesc base_world(std::optional<geo::Geodetic> anchor = std::nullopt) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 1;
    caps.sensors = 1;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> built =
        spade::WorldBuilder().name("anchored").environment(env).capacities(caps).plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).build();
    EXPECT_TRUE(built.has_value()) << (built ? "" : built.error().context);
    spade::WorldDesc w = *built;
    w.anchor = anchor;
    return w;
}

[[nodiscard]] spade::WorldSetDesc set_of(std::optional<geo::Geodetic> anchor) {
    spade::WorldInstanceDesc inst;
    inst.world = base_world(anchor);
    inst.seed = 0x5EEDu;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.grid.cell_size = 1.0f;
    return spade::WorldSetDesc{{inst}};
}

[[nodiscard]] uint64_t bits(double v) { return std::bit_cast<uint64_t>(v); }

[[nodiscard]] bool contains(const std::string& text, const std::string& what) { return text.find(what) != std::string::npos; }

const geo::Geodetic kBonneville{40.76, -113.9, 1286.5};

}  // namespace

TEST(WorldAnchorFile, AnAnchorlessWorldIsVersion2WithNoAnchorKey) {
    const spade::Result<std::string> text = spade::world_to_yaml(base_world());
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_TRUE(contains(*text, "\nworld_version: 2\n")) << "the lowest version that holds the world";
    EXPECT_FALSE(contains(*text, "anchor")) << "no key, no comment about one: the bytes are what they always were";
}

TEST(WorldAnchorFile, AnAnchoredWorldIsVersion3WithItsAnchorBlock) {
    const spade::Result<std::string> text = spade::world_to_yaml(base_world(kBonneville));
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_TRUE(contains(*text, "\nworld_version: 3\n"));
    EXPECT_TRUE(contains(*text, "\nanchor:\n  lat_deg: 40.76\n  lon_deg: -113.9\n  height_m: 1286.5\n")) << *text;
}

TEST(WorldAnchorFile, TheAnchorsDoublesRoundTripBitForBit) {
    const geo::Geodetic cases[] = {kBonneville,
                                   {1.0 / 3.0, -2.0 / 3.0, 0.1 + 0.2},
                                   {-0.0, -0.0, -0.0},
                                   {89.0, 180.0, -10000.0},
                                   {-89.0, -180.0, 100000.0},
                                   {1e-300, -1e-300, 4.9e-324},
                                   {53.80939444444444, 2.12955, 73.0}};
    for (const geo::Geodetic& a : cases) {
        const spade::Result<std::string> text = spade::world_to_yaml(base_world(a));
        ASSERT_TRUE(text.has_value()) << text.error().context;
        const spade::Result<spade::WorldDesc> back = spade::world_from_yaml(*text);
        ASSERT_TRUE(back.has_value()) << back.error().context;
        ASSERT_TRUE(back->anchor.has_value());
        EXPECT_EQ(bits(back->anchor->lat_deg), bits(a.lat_deg)) << a.lat_deg;
        EXPECT_EQ(bits(back->anchor->lon_deg), bits(a.lon_deg)) << a.lon_deg;
        EXPECT_EQ(bits(back->anchor->height_m), bits(a.height_m)) << a.height_m;
        const spade::Result<std::string> again = spade::world_to_yaml(*back);
        ASSERT_TRUE(again.has_value());
        EXPECT_EQ(*again, *text) << "canonical: the saved text is a function of the world";
    }
}

TEST(WorldAnchorFile, AV2FileWithAnAnchorKeyIsAnUnknownKey) {
    const spade::Result<std::string> text = spade::world_to_yaml(base_world(kBonneville));
    ASSERT_TRUE(text.has_value());
    std::string v2 = *text;
    v2.replace(v2.find("world_version: 3"), 16, "world_version: 2");
    const spade::Result<spade::WorldDesc> world = spade::world_from_yaml(v2);
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(world.error().context, "unknown key 'anchor'")) << world.error().context;
    EXPECT_TRUE(contains(world.error().context, "schema v2")) << world.error().context;
}

TEST(WorldAnchorFile, AV3FileWithoutAnAnchorLoadsAsAPlainWorld) {
    const spade::Result<std::string> text = spade::world_to_yaml(base_world());
    ASSERT_TRUE(text.has_value());
    std::string v3 = *text;
    v3.replace(v3.find("world_version: 2"), 16, "world_version: 3");
    const spade::Result<spade::WorldDesc> world = spade::world_from_yaml(v3);
    ASSERT_TRUE(world.has_value()) << world.error().context;
    EXPECT_FALSE(world->anchor.has_value());
}

TEST(WorldAnchorFile, AnAnchorOutsideItsLimitsOrMalformedIsRefusedByField) {
    // Through validation (the builder and the writer share it).
    const struct {
        geo::Geodetic g;
        const char* field;
    } bad[] = {{{89.5, 0.0, 0.0}, "lat_deg"}, {{0.0, 181.0, 0.0}, "lon_deg"}, {{0.0, 0.0, 100001.0}, "height_m"}};
    for (const auto& row : bad) {
        const spade::Result<std::string> text = spade::world_to_yaml(base_world(row.g));
        ASSERT_FALSE(text.has_value()) << row.field;
        EXPECT_EQ(text.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(text.error().context, row.field)) << text.error().context;
    }
    // Through the file: take a good v3 text and spoil the block.
    const spade::Result<std::string> good = spade::world_to_yaml(base_world(kBonneville));
    ASSERT_TRUE(good.has_value());
    const auto spoiled = [&](const std::string& from, const std::string& to) {
        std::string t = *good;
        const std::size_t at = t.find(from);
        EXPECT_NE(at, std::string::npos) << from;
        t.replace(at, from.size(), to);
        return spade::world_from_yaml(t);
    };
    const spade::Result<spade::WorldDesc> far = spoiled("lat_deg: 40.76", "lat_deg: 95");
    ASSERT_FALSE(far.has_value());
    EXPECT_EQ(far.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(far.error().context, "lat_deg")) << far.error().context;
    const spade::Result<spade::WorldDesc> missing = spoiled("  lon_deg: -113.9\n", "");
    ASSERT_FALSE(missing.has_value());
    EXPECT_TRUE(contains(missing.error().context, "lon_deg")) << missing.error().context;
    const spade::Result<spade::WorldDesc> extra = spoiled("  height_m: 1286.5\n", "  height_m: 1286.5\n  alt_m: 3\n");
    ASSERT_FALSE(extra.has_value());
    EXPECT_TRUE(contains(extra.error().context, "unknown key 'alt_m'")) << extra.error().context;
    const spade::Result<spade::WorldDesc> text = spoiled("lon_deg: -113.9", "lon_deg: \"west\"");
    ASSERT_FALSE(text.has_value());
    EXPECT_TRUE(contains(text.error().context, "lon_deg")) << text.error().context;
}

TEST(WorldAnchorIdentity, TheAnchorFoldsIntoConfigHashOnlyWhenSet) {
    const uint64_t none = spade::config_hash(set_of(std::nullopt));
    const uint64_t a = spade::config_hash(set_of(kBonneville));
    EXPECT_NE(a, none);
    EXPECT_EQ(a, spade::config_hash(set_of(kBonneville)));
    EXPECT_NE(a, spade::config_hash(set_of(geo::Geodetic{40.7600001, -113.9, 1286.5})));
    EXPECT_NE(a, spade::config_hash(set_of(geo::Geodetic{40.76, -113.9, 1286.6})));
    EXPECT_EQ(spade::config_hash(set_of(geo::Geodetic{0.0, 0.0, 0.0})), spade::config_hash(set_of(geo::Geodetic{-0.0, -0.0, -0.0})))
        << "the sign of a zero is not a different place";
    EXPECT_NE(spade::config_hash(set_of(geo::Geodetic{0.0, 0.0, 0.0})), none) << "an anchor at (0, 0, 0) is an anchor";
}

TEST(WorldAnchorSimulation, TheSimulationReportsItsWorldsAnchor) {
    spade::Result<spade::Simulation> anchored = spade::Simulation::create(set_of(kBonneville), 1'000'000, 1);
    ASSERT_TRUE(anchored.has_value()) << anchored.error().context;
    const spade::Result<std::optional<geo::Geodetic>> got = anchored->world_anchor(0);
    ASSERT_TRUE(got.has_value());
    ASSERT_TRUE(got->has_value());
    EXPECT_EQ(**got, kBonneville);
    spade::Result<spade::Simulation> plain = spade::Simulation::create(set_of(std::nullopt), 1'000'000, 1);
    ASSERT_TRUE(plain.has_value());
    const spade::Result<std::optional<geo::Geodetic>> none = plain->world_anchor(0);
    ASSERT_TRUE(none.has_value());
    EXPECT_FALSE(none->has_value());
    const spade::Result<std::optional<geo::Geodetic>> past = anchored->world_anchor(1);
    ASSERT_FALSE(past.has_value());
    EXPECT_EQ(past.error().code, spade::Code::invalid_argument) << "as the other per-world accessors";
}

TEST(WorldAnchorSimulation, ABadAnchorIsRefusedAtCreate) {
    spade::Result<spade::Simulation> sim = spade::Simulation::create(set_of(geo::Geodetic{95.0, 0.0, 0.0}), 1'000'000, 1);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(sim.error().context, "lat_deg")) << sim.error().context;
}

namespace {

[[nodiscard]] spade::Result<void> restore_into(const spade::SnapshotBlob& blob, std::optional<geo::Geodetic> anchor) {
    spade::Result<spade::Simulation> target = spade::Simulation::create(set_of(anchor), 1'000'000, 1);
    EXPECT_TRUE(target.has_value()) << (target ? "" : target.error().context);
    return target->restore(blob);
}

[[nodiscard]] spade::SnapshotBlob blob_under(std::optional<geo::Geodetic> anchor) {
    spade::Result<spade::Simulation> sim = spade::Simulation::create(set_of(anchor), 1'000'000, 1);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    EXPECT_TRUE(sim->step(3).has_value());
    spade::Result<spade::SnapshotBlob> blob = sim->snapshot();
    EXPECT_TRUE(blob.has_value());
    return std::move(*blob);
}

}  // namespace

TEST(WorldAnchorRestore, ABlobRestoresUnderItsOwnAnchorAndIsRefusedByNameUnderAnyOther) {
    const spade::SnapshotBlob anchored = blob_under(kBonneville);
    EXPECT_TRUE(restore_into(anchored, kBonneville).has_value()) << "the same anchor";

    const auto refused = [](const spade::Result<void>& r, const char* what) {
        ASSERT_FALSE(r.has_value()) << what;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
        EXPECT_TRUE(contains(r.error().context, "world anchor differs")) << what << ": " << r.error().context;
        EXPECT_TRUE(contains(r.error().context, "0 means no anchor")) << what << ": " << r.error().context;
    };
    refused(restore_into(anchored, geo::Geodetic{40.7600001, -113.9, 1286.5}), "another anchor");
    refused(restore_into(anchored, std::nullopt), "an anchored blob into a plain world");
    refused(restore_into(blob_under(std::nullopt), kBonneville), "a plain blob into an anchored world");
}

TEST(WorldAnchorRestore, APlainBlobStillRestoresIntoAPlainWorld) {
    const spade::SnapshotBlob plain = blob_under(std::nullopt);
    EXPECT_TRUE(restore_into(plain, std::nullopt).has_value());
}
