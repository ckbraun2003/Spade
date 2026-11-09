// test_module_config.cpp -- hashed module configuration (module API stage 7, Task C1; the stage 7 plan, C1, and L35).
//
// ModuleDesc::config is the module's canonical configuration image. compile_schedule spells it into the schedule identity
// right after the module's version (0x02, the length as a u32, the bytes), so a blob restores only under the same
// configuration; a module with no image folds nothing, so every set today keeps the identity it has.
//
// Tests that are green at red are PINS and say so: the standard identity, the empty-image case and the padding rule.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "core/rng.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"

namespace {

using spade::modules::compile_schedule;
using spade::modules::ModuleDesc;
using spade::modules::ModuleSet;

// A configuration image has no padding: four u32 lanes.
struct GridConfig {
    uint32_t nx, ny, nz, cell_mm;
};
static_assert(sizeof(GridConfig) == 16);

struct Padded {
    uint8_t a;
    uint32_t b;
};
// PIN, green at red: a type with padding is not a configuration image (it does not compile).
template <class T>
concept CanBeAnImage = requires(const T& t) { spade::modules::config_image(t); };
static_assert(!CanBeAnImage<Padded>);
static_assert(CanBeAnImage<GridConfig>);

// compile_schedule takes a span; a braced list of modules is a vector first.
[[nodiscard]] spade::Result<spade::modules::CompiledSchedule> schedule_of(std::vector<ModuleDesc> modules) {
    return compile_schedule(modules);
}

constexpr uint64_t kStandardIdentity = 0x3303cfc86821f502ULL;

[[nodiscard]] uint64_t fold(uint64_t h, uint8_t b) { return (h ^ b) * spade::rng::kFnv1aPrime; }
[[nodiscard]] uint64_t fold_u32(uint64_t h, uint32_t v) {
    for (int i = 0; i < 4; ++i) h = fold(h, static_cast<uint8_t>(v >> (8 * i)));
    return h;
}

// The identity of a set of passless modules, spelt by hand: per module the name, 0x00, the version as a u32, and (only when
// it has an image) 0x02, the length as a u32 and the bytes; then 0x01 (no passes follow).
struct Spec {
    std::string name;
    uint32_t version;
    std::vector<std::byte> image;
};
[[nodiscard]] uint64_t spelt(const std::vector<Spec>& specs) {
    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    for (const Spec& s : specs) {
        for (const char c : s.name) h = fold(h, static_cast<uint8_t>(c));
        h = fold(h, 0);
        h = fold_u32(h, s.version);
        if (!s.image.empty()) {
            h = fold(h, 0x02);
            h = fold_u32(h, static_cast<uint32_t>(s.image.size()));
            for (const std::byte b : s.image) h = fold(h, static_cast<uint8_t>(b));
        }
    }
    return fold(h, 1);
}

[[nodiscard]] std::vector<std::byte> bytes_of(const GridConfig& g) {
    const auto s = spade::modules::config_image(g);
    return std::vector<std::byte>(s.begin(), s.end());
}

[[nodiscard]] uint64_t fnv_of(std::span<const std::byte> image) {
    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    for (const std::byte b : image) h = fold(h, static_cast<uint8_t>(b));
    return h;
}

constexpr GridConfig kGrid16{16, 16, 16, 500};
constexpr GridConfig kGrid32{32, 32, 32, 250};

}  // namespace

// PIN, green at red: the standard set has no configuration image and keeps its identity.
TEST(ModuleConfig, AModuleWithNoConfigKeepsItsIdentity) {
    const auto standard = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(standard.has_value()) << standard.error().context;
    EXPECT_EQ(standard->identity, kStandardIdentity);

    ModuleSet with_empty = spade::modules::standard_modules();
    with_empty.push_back(ModuleDesc{.name = "plain", .version = 3, .config = {}});
    ModuleSet without = spade::modules::standard_modules();
    without.push_back(ModuleDesc{.name = "plain", .version = 3});
    EXPECT_EQ(compile_schedule(with_empty)->identity, compile_schedule(without)->identity);
}

TEST(ModuleConfig, AConfigImageIsSpeltAfterTheVersionWith0x02AndItsLength) {
    const GridConfig grid = kGrid16;
    const auto s = schedule_of({ModuleDesc{.name = "m", .version = 7, .config = spade::modules::config_image(grid)}});
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, spelt({{"m", 7, bytes_of(grid)}}));
    EXPECT_NE(s->identity, spelt({{"m", 7, {}}})) << "the image is part of the identity";
}

TEST(ModuleConfig, TwoConfigsGiveTwoIdentities) {
    const GridConfig a = kGrid16;
    const GridConfig b = kGrid32;
    const auto ia = schedule_of({ModuleDesc{.name = "m", .config = spade::modules::config_image(a)}});
    const auto ib = schedule_of({ModuleDesc{.name = "m", .config = spade::modules::config_image(b)}});
    ASSERT_TRUE(ia.has_value() && ib.has_value());
    EXPECT_NE(ia->identity, ib->identity);

    // The same bytes in a different module are a different schedule.
    const auto first = schedule_of({ModuleDesc{.name = "a", .config = spade::modules::config_image(a)}, ModuleDesc{.name = "b"}});
    const auto second = schedule_of({ModuleDesc{.name = "a"}, ModuleDesc{.name = "b", .config = spade::modules::config_image(a)}});
    ASSERT_TRUE(first.has_value() && second.has_value());
    EXPECT_NE(first->identity, second->identity);
}

TEST(ModuleConfig, ABindingThatIsNotTheConfigOneIsRefusedNamingTheModule) {
    const GridConfig grid = kGrid16;
    const auto wrong = schedule_of(
        {ModuleDesc{.name = "m", .config = spade::modules::config_image(grid),
                    .config_binding = spade::compute::gen::kBinding_airflow_u}});
    ASSERT_FALSE(wrong.has_value());
    EXPECT_EQ(wrong.error().code, spade::Code::invalid_argument);
    EXPECT_NE(wrong.error().context.find("'m'"), std::string::npos) << wrong.error().context;

    const auto empty = schedule_of(
        {ModuleDesc{.name = "m", .config_binding = spade::compute::gen::kBinding_airflow_config}});
    ASSERT_FALSE(empty.has_value()) << "a binding with no image to upload";
    EXPECT_NE(empty.error().context.find("'m'"), std::string::npos) << empty.error().context;
}

TEST(ModuleConfig, TwoModulesCannotBindTheConfigBufferNamingBoth) {
    const GridConfig grid = kGrid16;
    const auto two = schedule_of(
        {ModuleDesc{.name = "first", .config = spade::modules::config_image(grid),
                    .config_binding = spade::compute::gen::kBinding_airflow_config},
         ModuleDesc{.name = "second", .config = spade::modules::config_image(grid),
                    .config_binding = spade::compute::gen::kBinding_airflow_config}});
    ASSERT_FALSE(two.has_value());
    EXPECT_NE(two.error().context.find("first"), std::string::npos) << two.error().context;
    EXPECT_NE(two.error().context.find("second"), std::string::npos) << two.error().context;
}

TEST(ModuleConfig, TheConfigBindingIsDeclaredAtThirtyThree) {
    EXPECT_EQ(spade::compute::gen::kBinding_airflow_config, 33u) << "the plan's decision 3: appended, never renumbered";
}

namespace {

[[nodiscard]] spade::Simulation make(const ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, 2, spade::compute::BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] ModuleSet standard_with(const GridConfig& grid) {
    ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "gridded", .version = 2, .config = spade::modules::config_image(grid)});
    return set;
}

}  // namespace

TEST(ModuleConfig, ARestoreUnderAnotherGridIsRefusedByIdentity) {
    static const GridConfig a = kGrid16;
    static const GridConfig b = kGrid32;
    spade::Simulation first = make(standard_with(a));
    ASSERT_TRUE(first.step(2).has_value());
    const auto blob = first.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    spade::Simulation same = make(standard_with(a));
    EXPECT_TRUE(same.restore(*blob).has_value()) << "the same configuration restores";

    spade::Simulation other = make(standard_with(b));
    const auto refused = other.restore(*blob);
    ASSERT_FALSE(refused.has_value()) << "another grid is another schedule identity";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("identity"), std::string::npos) << refused.error().context;
}

TEST(ModuleConfig, ConfigurationReportListsEachModuleNameVersionAndHash) {
    static const GridConfig grid = kGrid16;
    spade::Simulation sim = make(standard_with(grid));
    const auto report = sim.configuration_report();
    ASSERT_EQ(report.size(), sim.schedule().modules.size()) << "one entry per module, in set order";
    for (std::size_t i = 0; i < report.size(); ++i) {
        EXPECT_EQ(report[i].module, sim.schedule().modules[i].name);
        EXPECT_EQ(report[i].version, sim.schedule().modules[i].version);
    }
    const auto& last = report.back();
    EXPECT_EQ(last.module, "gridded");
    EXPECT_EQ(last.version, 2u);
    EXPECT_EQ(last.bytes, sizeof(GridConfig));
    EXPECT_EQ(last.hash, fnv_of(spade::modules::config_image(grid)));
    EXPECT_EQ(report.front().bytes, 0u) << "a module with no image reports none";
    EXPECT_EQ(report.front().hash, spade::rng::kFnv1aOffsetBasis);
}
