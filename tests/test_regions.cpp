// test_regions.cpp -- regions (module API stage 7, Task D2; the stage 7 plan, batch D).
//
// A module declares prioritised regions (ModuleDesc::regions); a field provider bound to one (FieldDecl::region + `sample`)
// answers Simulation::sample_medium inside it, on the CPU; a world places regions (WorldInstanceDesc::regions, boxes). Outside
// every region the answer is today's. Where placed regions overlap the highest priority wins, and two overlapping placements
// of equal priority are refused at create(), naming both. A world with no regions hashes as it always did.
// The device half (sample_medium refusing on Vulkan inside a region) is GpuRegions.* below.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>

#include <glm/vec3.hpp>

#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/medium.hpp"

namespace {

using spade::MediumSample;
using spade::RegionPlacement;
using spade::WorldSetDesc;
using spade::modules::FieldDecl;
using spade::modules::FieldKind;
using spade::modules::ModuleDesc;
using spade::modules::RegionDecl;

// A uniform gust and a position-dependent plume: the second is the SPH-shaped provider (the answer is a function of where).
MediumSample gust_sample(glm::vec3) noexcept { return MediumSample{1.1f, glm::vec3(1.0f, 0.0f, 0.0f)}; }
MediumSample plume_sample(glm::vec3 p) noexcept { return MediumSample{0.9f, glm::vec3(p.x, 2.0f, 0.0f)}; }

constexpr RegionDecl kGustRegion[] = {{.name = "gust_box", .priority = 1}};
constexpr FieldDecl kGustWind[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s", .region = "gust_box", .sample = &gust_sample}};
constexpr RegionDecl kPlumeRegion[] = {{.name = "plume_box", .priority = 2}};
constexpr FieldDecl kPlumeWind[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s", .region = "plume_box", .sample = &plume_sample}};

[[nodiscard]] spade::modules::ModuleSet with_regions() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "gusts", .fields = kGustWind, .regions = kGustRegion});
    set.push_back(ModuleDesc{.name = "plume", .fields = kPlumeWind, .regions = kPlumeRegion});
    return set;
}

[[nodiscard]] WorldSetDesc world() {
    spade::Result<WorldSetDesc> w = spade::test::floor_world_set(2);
    EXPECT_TRUE(w.has_value()) << w.error().context;
    return *w;
}

[[nodiscard]] spade::Result<spade::Simulation> make(const WorldSetDesc& w, const spade::modules::ModuleSet& set,
                                                    spade::compute::BackendKind kind = spade::compute::BackendKind::cpu) {
    return spade::Simulation::create(w, 1'000'000, 2, spade::compute::BackendDesc{.kind = kind}, set);
}

}  // namespace

// PIN. A world with no regions hashes as today: an empty list folds nothing, so the hash is the one the goldens pin; a placed
// region is a different configuration (name, box and order each fold). (The literal is pinned from the RED head's print, as
// batch C's Snapshot literal was; the committed goldens already hold the empty case.)
TEST(Regions, AWorldWithoutRegionsHashesAsToday) {
    const WorldSetDesc base = world();
    const uint64_t reference = spade::config_hash(base);
    WorldSetDesc empty = base;
    empty.worlds[0].regions.clear();
    EXPECT_EQ(spade::config_hash(empty), reference);

    WorldSetDesc placed = base;
    placed.worlds[0].regions.push_back({"gust_box", glm::vec3(0.0f), glm::vec3(1.0f)});
    const uint64_t with_one = spade::config_hash(placed);
    EXPECT_NE(with_one, reference);
    WorldSetDesc moved = placed;
    moved.worlds[0].regions[0].hi.x = 2.0f;
    EXPECT_NE(spade::config_hash(moved), with_one) << "the box folds";
    WorldSetDesc renamed = placed;
    renamed.worlds[0].regions[0].region = "plume_box";
    EXPECT_NE(spade::config_hash(renamed), with_one) << "the name folds";
    WorldSetDesc two = placed;
    two.worlds[0].regions.push_back({"plume_box", glm::vec3(0.0f), glm::vec3(1.0f)});
    WorldSetDesc swapped = two;
    std::swap(swapped.worlds[0].regions[0], swapped.worlds[0].regions[1]);
    EXPECT_NE(spade::config_hash(swapped), spade::config_hash(two)) << "the order folds";
}

TEST(Regions, EqualPriorityOverlapIsRefusedNamingBoth) {
    // Two regions of one priority: declare gust_box at priority 2 too, by a second module with the same priority as plume.
    static constexpr RegionDecl kEqual[] = {{.name = "twin_box", .priority = 2}};
    static constexpr FieldDecl kTwinWind[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s", .region = "twin_box", .sample = &gust_sample}};
    spade::modules::ModuleSet set = with_regions();
    set.push_back(ModuleDesc{.name = "twin", .fields = kTwinWind, .regions = kEqual});

    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"plume_box", glm::vec3(0.0f), glm::vec3(4.0f)});
    w.worlds[0].regions.push_back({"twin_box", glm::vec3(3.0f), glm::vec3(8.0f)});  // overlaps at [3,4]^3
    const auto refused = make(w, set);
    ASSERT_FALSE(refused.has_value()) << "equal priority, overlapping";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("plume_box"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("twin_box"), std::string::npos) << refused.error().context;

    // Equal priority that does not overlap is fine, and so is an overlap of distinct priorities.
    w.worlds[0].regions[1] = {"twin_box", glm::vec3(10.0f), glm::vec3(12.0f)};
    EXPECT_TRUE(make(w, set).has_value());
}

TEST(Regions, APlacementNamingNoRegionOrAnInvertedBoxIsRefusedNamingIt) {
    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"no_such_region", glm::vec3(0.0f), glm::vec3(1.0f)});
    const auto unknown = make(w, with_regions());
    ASSERT_FALSE(unknown.has_value());
    EXPECT_NE(unknown.error().context.find("no_such_region"), std::string::npos) << unknown.error().context;

    w.worlds[0].regions[0] = {"gust_box", glm::vec3(2.0f), glm::vec3(1.0f)};  // lo > hi
    const auto inverted = make(w, with_regions());
    ASSERT_FALSE(inverted.has_value());
    EXPECT_NE(inverted.error().context.find("gust_box"), std::string::npos) << inverted.error().context;
}

TEST(Regions, PriorityResolvesTheHighestRegionPerPoint) {
    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"gust_box", glm::vec3(0.0f), glm::vec3(10.0f)});
    w.worlds[0].regions.push_back({"plume_box", glm::vec3(4.0f), glm::vec3(6.0f)});
    auto sim = make(w, with_regions());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto inner = sim->sample_medium(0, glm::vec3(5.0f));  // both contain it; plume has priority 2
    ASSERT_TRUE(inner.has_value()) << inner.error().context;
    EXPECT_EQ(inner->density, 0.9f);
    EXPECT_EQ(inner->wind.x, 5.0f);
    EXPECT_EQ(inner->wind.y, 2.0f);
    const auto outer = sim->sample_medium(0, glm::vec3(1.0f));  // only the gust box
    ASSERT_TRUE(outer.has_value());
    EXPECT_EQ(outer->density, 1.1f);
    EXPECT_EQ(outer->wind.x, 1.0f);
}

// THE REFUSAL THAT SURVIVES the one-provider rule's rewrite: a field still has exactly one WHOLE-WORLD provider. A second
// provider with no region is refused naming both modules (ModuleFields.TwoProvidersOfOneFieldAreRefused holds the same for a
// custom field); a region provider with no sample, a region its module does not declare, or no whole-world provider beside it,
// is refused naming the module.
TEST(Regions, TwoWholeWorldProvidersOfOneFieldAreStillRefusedNamingBoth) {
    static constexpr FieldDecl kPlainWind[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s"}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "rival", .fields = kPlainWind});
    const auto refused = spade::modules::compile_schedule(set);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("wind"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("dryden"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("rival"), std::string::npos) << refused.error().context;
}

TEST(Regions, AProviderBoundToARegionNeedsASampleAFittingRegionAndAWholeWorldProviderBesideIt) {
    EXPECT_TRUE(spade::modules::compile_schedule(with_regions()).has_value());

    static constexpr FieldDecl kNoSample[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s", .region = "gust_box"}};
    spade::modules::ModuleSet no_sample = spade::modules::standard_modules();
    no_sample.push_back(ModuleDesc{.name = "gusts", .fields = kNoSample, .regions = kGustRegion});
    const auto a = spade::modules::compile_schedule(no_sample);
    ASSERT_FALSE(a.has_value()) << "a region provider with no sample function";
    EXPECT_NE(a.error().context.find("gusts"), std::string::npos) << a.error().context;

    static constexpr FieldDecl kWrongRegion[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s", .region = "elsewhere", .sample = &gust_sample}};
    spade::modules::ModuleSet wrong = spade::modules::standard_modules();
    wrong.push_back(ModuleDesc{.name = "gusts", .fields = kWrongRegion, .regions = kGustRegion});
    const auto b = spade::modules::compile_schedule(wrong);
    ASSERT_FALSE(b.has_value()) << "a region its module does not declare";
    EXPECT_NE(b.error().context.find("elsewhere"), std::string::npos) << b.error().context;

    static constexpr FieldDecl kAlone[] = {{.name = "hum", .kind = FieldKind::scalar, .unit = "dB", .region = "gust_box", .sample = &gust_sample}};
    spade::modules::ModuleSet alone = spade::modules::standard_modules();
    alone.push_back(ModuleDesc{.name = "gusts", .fields = kAlone, .regions = kGustRegion});
    const auto c = spade::modules::compile_schedule(alone);
    ASSERT_FALSE(c.has_value()) << "a region provider of a field no whole-world provider answers";
    EXPECT_NE(c.error().context.find("hum"), std::string::npos) << c.error().context;
}

// Outside every region: today's answer, the whole-world provider's -- the same bits a set with no regions gives.
TEST(Regions, SampleMediumIsRoutedAndOutsideEveryRegionIsTodaysAnswer) {
    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"gust_box", glm::vec3(0.0f), glm::vec3(10.0f)});
    auto with = make(w, with_regions());
    auto without = make(world(), spade::modules::standard_modules());
    ASSERT_TRUE(with.has_value() && without.has_value());
    const glm::vec3 far(50.0f, 3.0f, -2.0f);
    const auto a = with->sample_medium(0, far);
    const auto b = without->sample_medium(0, far);
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(a->density, b->density);
    EXPECT_EQ(a->wind.x, b->wind.x);
    EXPECT_EQ(a->wind.y, b->wind.y);
    EXPECT_EQ(a->wind.z, b->wind.z);
    // Another world with no placed region is unaffected by this world's.
}

// The SPH-shaped second consumer (design 6.2): a CPU-only provider whose answer is a function of position needs no Core change
// beyond the declarations: its sample function is all the engine calls.
TEST(Regions, ACpuOnlyPositionDependentProviderNeedsNoCoreChange) {
    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"plume_box", glm::vec3(0.0f), glm::vec3(10.0f)});
    auto sim = make(w, with_regions());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    for (const float x : {1.0f, 2.5f, 9.0f}) {
        const auto s = sim->sample_medium(0, glm::vec3(x, 1.0f, 1.0f));
        ASSERT_TRUE(s.has_value());
        EXPECT_EQ(s->wind.x, x) << "the provider's own function of position";
    }
    ASSERT_TRUE(sim->step(2).has_value()) << "a region provider has no pass: the step path is the standard one";
}

// The device half: on Vulkan sample_medium refuses inside a region, naming it (there is no device region row in D2), and
// answers outside as today.
TEST(GpuRegions, SampleMediumRefusesOnVulkanInsideARegionNamingIt) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    WorldSetDesc w = world();
    w.worlds[0].regions.push_back({"gust_box", glm::vec3(0.0f), glm::vec3(10.0f)});
    auto sim = make(w, with_regions(), spade::compute::BackendKind::vulkan);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto inside = sim->sample_medium(0, glm::vec3(5.0f));
    ASSERT_FALSE(inside.has_value());
    EXPECT_EQ(inside.error().code, spade::Code::unavailable);
    EXPECT_NE(inside.error().context.find("gust_box"), std::string::npos) << inside.error().context;
    EXPECT_TRUE(sim->sample_medium(0, glm::vec3(50.0f)).has_value());
}
