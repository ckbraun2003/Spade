// test_point_samples.cpp -- point samples, the host half (module API stage 7, Task D1; the stage 7 plan, batch D).
//
// A module with a position-dependent field declares the points it is sampled at (ModuleDesc::points). The set then gets the
// internal per-world array `points.airflow_points` (one PointRow per point: bodies, then force elements x k, then sensors, then the
// host's rows) appended after the registry walk, and the positions pass `points.positions` after behaviors.kinematic.
// Pins, green at red and green for good: a set with no such module has neither, and keeps its identity, its walk and its
// pass list. The device half is test_gpu_point_samples.cpp.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/state_view.hpp"

namespace {

using spade::modules::Access;
using spade::modules::CompiledSchedule;
using spade::modules::compile_schedule;
using spade::modules::ModuleDesc;
using spade::modules::PassDecl;
using spade::modules::Phase;
using spade::modules::PointRow;
using spade::modules::PointsDecl;
using spade::modules::QuantityAccess;

constexpr uint32_t kK = 2;             // points per force-element slot
constexpr uint32_t kHostRows = 3;      // rows the host fills
constexpr uint32_t kBodies = 4;        // floor_world_set(4): body slots
constexpr uint32_t kElements = 4;      // its force-element slots
constexpr uint32_t kSensors = 2;       // its sensor slots
constexpr uint32_t kRowsPerWorld = kBodies + kElements * kK + kSensors + kHostRows;

// The provider: it places its own element rows (Core's positions pass writes body rows only: L43) at (i, i, i) for row i, then
// for every point row sets wind.x = 2 * pos.y and density = pos.x -- a function of WHERE the point is, so a provider that
// saw a stale or zero position would write something else.
void provide(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 2) return;  // state[i] views the pass's i-th access: field.swirl has no view, the points array is state[1]
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<PointRow> rows = spade::world_rows<PointRow>(ctx.state[1], w);
        for (uint32_t i = 0; i < rows.size(); ++i) {
            PointRow& row = rows[i];
            if (i >= kBodies && i < kBodies + kElements * kK) row.pos[0] = row.pos[1] = row.pos[2] = static_cast<float>(i);
            row.wind[0] = 2.0f * row.pos[1];
            row.density = row.pos[0];
        }
    }
}

constexpr spade::modules::FieldDecl kSwirl[] = {{.name = "swirl", .kind = spade::modules::FieldKind::scalar, .unit = "1"}};
constexpr QuantityAccess kProvideAccess[] = {{"field.swirl", Access::write}, {"points.airflow_points", Access::write}};
constexpr std::string_view kAfterPositions[] = {"points.positions"};
constexpr PassDecl kProvidePasses[] = {{.name = "sample", .phase = Phase::fields, .access = kProvideAccess,
                                        .after = kAfterPositions, .cpu = &provide}};
// A provider that declares no write of points.airflow_points: what a module with position_dependent == false looks like (the array
// does not exist for it to name), and what the missing-writer refusal is tested with.
constexpr QuantityAccess kNoPointsAccess[] = {{"field.swirl", Access::write}};
constexpr PassDecl kNoPointsPasses[] = {{.name = "sample", .phase = Phase::fields, .access = kNoPointsAccess, .cpu = &provide}};

[[nodiscard]] spade::modules::ModuleSet with_points(bool position_dependent = true, bool writes_points = true, uint32_t k = kK) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "gusts", .passes = writes_points ? std::span<const PassDecl>(kProvidePasses)
                                                                       : std::span<const PassDecl>(kNoPointsPasses),
                             .fields = kSwirl,
                             .points = PointsDecl{.k = k, .host_requests = kHostRows, .position_dependent = position_dependent}});
    return set;
}

[[nodiscard]] std::vector<std::string> names(const CompiledSchedule& s) {
    std::vector<std::string> out;
    for (const auto& p : s.passes) out.push_back(p.module + "." + p.pass);
    return out;
}

[[nodiscard]] spade::Simulation make(const spade::modules::ModuleSet& set, uint32_t substeps = 2) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(kBodies);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, substeps, spade::compute::BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

// PIN. A standard set allocates nothing new: no points array, no positions pass, the identity and the 11-entry walk it has (3 head arrays, drag_bodies, dryden, imu x2, rotors, replay_config, gnss x2).
TEST(PointSamples, AStandardSetAllocatesNothingNew) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_FALSE(s->points.present);
    EXPECT_EQ(s->identity, 0x3303cfc86821f502ULL);
    EXPECT_EQ(spade::modules::walk_order(*s).size(), 11u);
    EXPECT_FALSE(std::ranges::any_of(s->arrays, [](const auto& a) { return a.name == "airflow_points"; }));
    spade::Simulation sim = make(spade::modules::standard_modules());
    EXPECT_FALSE(sim.module_array("airflow_points").has_value()) << "no points array in a standard set";
}

// PIN. No standard pass is added or renamed, so the identity holds: the chain has no `points.*` pass.
TEST(PointSamples, TheStandardChainIsUnchanged) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const std::vector<std::string> order = names(*s);
    EXPECT_FALSE(std::ranges::any_of(order, [](const std::string& n) { return n.starts_with("points."); }));
    EXPECT_NE(std::ranges::find(order, "behaviors.kinematic"), order.end());
}

TEST(PointSamples, APositionDependentProviderGetsThePositionsPassAndTheArrayAfterTheWalk) {
    const auto s = compile_schedule(with_points());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_TRUE(s->points.present);
    EXPECT_EQ(s->points.k, kK);
    EXPECT_EQ(s->points.host_requests, kHostRows);
    EXPECT_EQ(s->points.module, "gusts");
    const std::vector<std::string> order = names(*s);
    const auto kinematic = std::ranges::find(order, "behaviors.kinematic");
    const auto positions = std::ranges::find(order, "points.positions");
    const auto sample = std::ranges::find(order, "gusts.sample");
    ASSERT_NE(positions, order.end()) << "compile_schedule injects the positions pass";
    ASSERT_NE(kinematic, order.end());
    ASSERT_NE(sample, order.end());
    EXPECT_LT(kinematic, positions);
    EXPECT_LT(positions, sample);
    ASSERT_FALSE(s->arrays.empty());
    EXPECT_EQ(s->arrays.back().name, "airflow_points") << "appended after the walk";
    EXPECT_EQ(s->arrays.back().elem_size, sizeof(PointRow));
    EXPECT_NE(s->identity, 0x3303cfc86821f502ULL);
    // The reader is drag.forces: it runs after both writers and its view of the array is bound.
    const auto drag = std::ranges::find(order, "drag.forces");
    ASSERT_NE(drag, order.end());
    EXPECT_LT(sample, drag);
    const auto& pass = s->passes[static_cast<std::size_t>(drag - order.begin())];
    ASSERT_FALSE(pass.state.empty());
    EXPECT_EQ(pass.state.back().kind, spade::modules::BindingKind::array) << "drag.forces binds points.airflow_points";
    EXPECT_EQ(s->arrays[pass.state.back().index].name, "airflow_points");
}

// THE GUARD (lead's ruling L43): an unfilled row must never be read as a real point at the origin. The positions pass writes
// body rows only, so a set with element rows (k > 0) needs a pass of another module that declares a write of points.airflow_points;
// without one compile_schedule refuses, naming the array and the missing writer.
TEST(PointSamples, ElementRowsWithNoWriterAreRefusedNamingTheArrayAndTheMissingWriter) {
    const auto refused = compile_schedule(with_points(true, false, kK));
    ASSERT_FALSE(refused.has_value()) << "k > 0, and no pass declares a write of points.airflow_points";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("points.airflow_points"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("write"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("gusts"), std::string::npos) << "names the declaring module: " << refused.error().context;
    // k == 0: no element rows, so the body rows (the positions pass's) are all Core sizes, and nothing is refused.
    EXPECT_TRUE(compile_schedule(with_points(true, false, 0)).has_value());
}

// The declaration without a position-dependent provider adds nothing: no array, no pass.
TEST(PointSamples, APointsDeclarationThatIsNotPositionDependentAddsNothing) {
    const auto s = compile_schedule(with_points(false, false));
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_FALSE(s->points.present);
    EXPECT_FALSE(std::ranges::any_of(s->arrays, [](const auto& a) { return a.name == "airflow_points"; }));
    const std::vector<std::string> order = names(*s);
    EXPECT_EQ(std::ranges::find(order, "points.positions"), order.end());
}

TEST(PointSamples, TwoModulesDeclaringPointsAreRefusedNamingBoth) {
    spade::modules::ModuleSet set = with_points();
    static constexpr spade::modules::FieldDecl other[] = {{.name = "eddy", .kind = spade::modules::FieldKind::scalar, .unit = "1"}};
    static constexpr QuantityAccess eddy_access[] = {{"field.eddy", Access::write}};
    static constexpr PassDecl eddy_passes[] = {{.name = "sample", .phase = Phase::fields, .access = eddy_access, .cpu = &provide}};
    set.push_back(ModuleDesc{.name = "eddies", .passes = eddy_passes, .fields = other,
                             .points = PointsDecl{.k = 1, .host_requests = 0, .position_dependent = true}});
    const auto refused = compile_schedule(set);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("gusts"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("eddies"), std::string::npos) << refused.error().context;
}

// The provider sees each point's position: bodies first (slot order), then the host's rows. The array holds
// bodies + elements x k + sensors + host rows per world, 8 words each. ONE substep, so the positions pass (which runs before
// Integrate in the substep) sees exactly the spawn positions; with more substeps a row holds where the body was at the start
// of the LAST substep, not where it ends the step.
TEST(PointSamples, APositionDependentProviderSeesEachPointsPosition) {
    const spade::modules::ModuleSet set = with_points();
    spade::Simulation sim = make(set, 1);
    const auto first = sim.spawn(0, spade::test::bare_body(glm::vec3(1.0f, 2.0f, 3.0f)));
    const auto second = sim.spawn(0, spade::test::bare_body(glm::vec3(-4.0f, 5.0f, 6.0f)));
    ASSERT_TRUE(first.has_value() && second.has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    const auto rows = sim.module_rows<PointRow>("airflow_points", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    ASSERT_EQ(rows->size(), kRowsPerWorld) << "per world: bodies + elements x k + sensors + host rows";
    EXPECT_EQ((*rows)[0].pos[0], 1.0f);
    EXPECT_EQ((*rows)[0].pos[1], 2.0f);
    EXPECT_EQ((*rows)[0].pos[2], 3.0f);
    EXPECT_EQ((*rows)[1].pos[0], -4.0f);
    EXPECT_EQ((*rows)[1].pos[1], 5.0f);
    EXPECT_EQ((*rows)[1].pos[2], 6.0f);
    EXPECT_EQ((*rows)[0].reserved, 0.0f);
    // The provider wrote a function of each position, so it ran after the positions pass.
    EXPECT_EQ((*rows)[0].wind[0], 4.0f);
    EXPECT_EQ((*rows)[0].density, 1.0f);
    EXPECT_EQ((*rows)[1].wind[0], 10.0f);
    EXPECT_EQ((*rows)[1].density, -4.0f);
    // The module's own pass filled its element rows (L43), and the provider saw them.
    EXPECT_EQ((*rows)[kBodies].pos[1], static_cast<float>(kBodies));
    EXPECT_EQ((*rows)[kBodies].wind[0], 2.0f * static_cast<float>(kBodies));
}

// Host requests: the last kHostRows rows are the host's. The positions pass leaves them alone; the provider sees what the
// host wrote.
TEST(PointSamples, HostRequestRowsAreTheHostsAndThePositionsPassLeavesThemAlone) {
    const spade::modules::ModuleSet set = with_points();
    spade::Simulation sim = make(set);
    // write_module_rows takes a whole world's rows: read them, change the host's, write them back.
    const auto current = sim.module_rows<PointRow>("airflow_points", 0);
    ASSERT_TRUE(current.has_value()) << current.error().context;
    ASSERT_EQ(current->size(), kRowsPerWorld);
    std::vector<PointRow> all(current->begin(), current->end());
    for (uint32_t i = kRowsPerWorld - kHostRows; i < kRowsPerWorld; ++i) {
        all[i].pos[0] = 7.0f;
        all[i].pos[1] = 8.0f;
        all[i].pos[2] = 9.0f;
    }
    ASSERT_TRUE(sim.write_module_rows("airflow_points", 0, std::as_bytes(std::span<const PointRow>(all))).has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    const auto rows = sim.module_rows<PointRow>("airflow_points", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    ASSERT_EQ(rows->size(), kRowsPerWorld);
    const PointRow& r = (*rows)[kRowsPerWorld - kHostRows];
    EXPECT_EQ(r.pos[0], 7.0f);
    EXPECT_EQ(r.pos[1], 8.0f);
    EXPECT_EQ(r.wind[0], 16.0f);
    EXPECT_EQ(r.density, 7.0f);
}
