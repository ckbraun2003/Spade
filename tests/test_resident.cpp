// test_resident.cpp -- device-resident arrays, the host half (module API stage 7, Task C2; the stage 7 plan, C2).
//
// ArrayDecl::residency = device keeps a per_world array on the device between steps. compile_schedule accepts it only on a
// per_world array of a module that claims a CPU `reference` (the CPU run is the array's meaning); on the cpu backend a device
// array is an ordinary host array. The device half is test_gpu_resident.cpp.
//
// Green at red (pins, named): a device array is an ordinary host array on the cpu backend.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "resident_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"

namespace {

using spade::modules::ArrayDecl;
using spade::modules::Extent;
using spade::modules::ModuleDesc;
using spade::modules::Residency;
using spade::test::ResidentModule;
using spade::test::ResidentRow;

[[nodiscard]] spade::Simulation make(const spade::modules::ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, 2, spade::compute::BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] spade::Result<spade::modules::CompiledSchedule> schedule_of(std::vector<ModuleDesc> modules) {
    return spade::modules::compile_schedule(modules);
}

}  // namespace

// PIN, green at red: on the cpu backend a device array is an ordinary host array -- stepped, snapshotted, digested as one.
TEST(Resident, ADeviceArrayIsAnOrdinaryHostArrayOnTheCpu) {
    const ResidentModule device(Residency::device);
    const ResidentModule host(Residency::host);
    spade::Simulation a = make(device.set);
    spade::Simulation b = make(host.set);
    ASSERT_TRUE(a.step(3).has_value());
    ASSERT_TRUE(b.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b)) << "residency is not part of the state";

    const auto rows = a.module_rows<ResidentRow>("airflow_u", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    EXPECT_EQ((*rows)[0].cell[0], 6.0f) << "3 steps x 2 substeps";
    EXPECT_TRUE(a.resident_current()) << "nothing is stale on the cpu backend";

    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value());
    spade::Simulation c = make(device.set);
    ASSERT_TRUE(c.restore(*blob).has_value());
    EXPECT_EQ(spade::testing::state_digest(c), spade::testing::state_digest(a));
}

// PIN, green at red: on the cpu backend nothing is ever stale, so the world digest through every form -- the const one, the
// non-const one that syncs, the checked one that refuses -- is one number, and the device-declared run equals the host-declared.
TEST(Resident, TheWorldDigestOnTheCpuIsOneNumberThroughEveryForm) {
    const ResidentModule device(Residency::device);
    const ResidentModule host(Residency::host);
    spade::Simulation a = make(device.set);
    spade::Simulation b = make(host.set);
    ASSERT_TRUE(a.step(3).has_value());
    ASSERT_TRUE(b.step(3).has_value());
    const spade::Simulation& view = a;
    for (uint32_t w = 0; w < 1; ++w) {
        const uint64_t plain = spade::testing::world_digest(view, w);
        const auto checked = spade::testing::checked_world_digest(view, w);
        ASSERT_TRUE(checked.has_value()) << checked.error().context;
        EXPECT_EQ(*checked, plain);
        EXPECT_EQ(spade::testing::world_digest(a, w), plain);
        EXPECT_EQ(spade::testing::world_digest(b, w), plain) << "residency is not part of the state";
    }
}

TEST(Resident, OnlyPerWorldMayBeDevice) {
    static const ArrayDecl per_body[] = {{.name = "grid", .elem_size = 16, .extent = Extent::per_body,
                                           .spawn_size = 4, .init = [](const spade::modules::RowInit&) noexcept {},
                                           .residency = Residency::device}};
    const auto refused = schedule_of({ModuleDesc{.name = "cells", .state = per_body,
                                                 .grade = {.cpu = spade::Grade::reference, .vulkan = spade::Grade::banded}}});
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("cells"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("grid"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("device-resident"), std::string::npos) << "the residency rule's own text: " << refused.error().context;
}

TEST(Resident, RequiresACpuReference) {
    static const ArrayDecl per_world[] = {{.name = "grid", .elem_size = 16, .extent = Extent::per_world,
                                           .residency = Residency::device}};
    for (const spade::Grade cpu : {spade::Grade::best_effort, spade::Grade::banded}) {
        // vulkan best_effort: L4 (a banded Vulkan grade needs a CPU reference) would refuse first and hide the residency rule.
        const auto refused = schedule_of({ModuleDesc{.name = "cells", .state = per_world,
                                                     .grade = {.cpu = cpu, .vulkan = spade::Grade::best_effort}}});
        ASSERT_FALSE(refused.has_value()) << "cpu grade " << static_cast<int>(cpu);
        EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
        EXPECT_NE(refused.error().context.find("cells"), std::string::npos) << refused.error().context;
        EXPECT_NE(refused.error().context.find("grid"), std::string::npos) << refused.error().context;
        EXPECT_NE(refused.error().context.find("device-resident"), std::string::npos)
            << "the residency rule's own text, not L4's: " << refused.error().context;
        EXPECT_NE(refused.error().context.find("reference"), std::string::npos) << refused.error().context;
    }
    const auto accepted = schedule_of({ModuleDesc{.name = "cells", .state = per_world,
                                                  .grade = {.cpu = spade::Grade::reference, .vulkan = spade::Grade::banded}}});
    EXPECT_TRUE(accepted.has_value()) << accepted.error().context;
}

TEST(Resident, TheScheduleRecordsWhichArraysAreResident) {
    const ResidentModule device(Residency::device);
    const auto s = spade::modules::compile_schedule(device.set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    int resident = 0;
    for (const spade::modules::CompiledArray& a : s->arrays) {
        if (a.residency == Residency::device) {
            ++resident;
            EXPECT_EQ(a.name, "airflow_u");
        }
    }
    EXPECT_EQ(resident, 1);
}
