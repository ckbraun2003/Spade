// rebuild_module_support.hpp -- test modules with a rebuild hook (module API stage 7, Task C3).
//
// CPU module "derive": a per_world state array "region" (one u32, bumped by the pass each substep) and a per_world SCRATCH
// "derived" (one u32, not state: snapshots do not carry it). Its rebuild hook sets derived = region * 2 + k for each world, and
// counts its own calls, so a test sees WHEN it ran and WHAT it saw. A hook that did not run after a restore leaves the scratch
// at what the new simulation had.
//
// GRID module "gridded" (GPU tests): the resident grid of resident_module_support.hpp ("airflow_u") plus a derived scratch
// ("airflow_v") that the rebuild fills with the configuration's constant k and the pass adds to the grid every substep.
// Built in place and never copied.
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <span>

#include "core/grade.hpp"
#include "physics/schedule.hpp"
#include "resident_module_support.hpp"
#include "sim/module.hpp"
#include "state/state_view.hpp"

namespace spade::test {

struct DeriveConfig {
    uint32_t k;
};
inline constexpr DeriveConfig kDeriveConfig{7};

inline int g_rebuild_calls = 0;
inline uint32_t g_rebuild_derived[2] = {0, 0};  // the last derived value per world

inline void derive_bump(const physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 1) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<uint32_t> region = world_rows<uint32_t>(ctx.state[0], w);
        if (!region.empty()) region[0] += 1;
    }
}

inline void derive_rebuild(const physics::SubstepContext& ctx) noexcept {
    ++g_rebuild_calls;
    if (ctx.state.size() != 2) return;
    for (uint32_t w = 0; w < ctx.worlds.size() && w < 2; ++w) {
        const std::span<const uint32_t> region = world_rows<const uint32_t>(ctx.state[0], w);
        const std::span<uint32_t> derived = world_rows<uint32_t>(ctx.state[1], w);
        if (region.empty() || derived.empty()) continue;
        derived[0] = region[0] * 2u + kDeriveConfig.k;
        g_rebuild_derived[w] = derived[0];
    }
}

inline constexpr modules::QuantityAccess kDeriveBumpAccess[] = {{"derive.region", modules::Access::write}};
inline constexpr modules::QuantityAccess kDeriveRebuildAccess[] = {{"derive.region", modules::Access::read},
                                                                   {"derive.derived", modules::Access::write}};
inline constexpr modules::ArrayDecl kDeriveArrays[] = {
    {.name = "region", .elem_size = sizeof(uint32_t), .extent = modules::Extent::per_world}};
inline constexpr modules::ScratchDecl kDeriveScratch[] = {
    {.name = "derived", .elem_size = sizeof(uint32_t), .extent = modules::Extent::per_world}};

struct DeriveModule {
    std::array<modules::PassDecl, 1> passes;
    modules::ModuleSet set;

    explicit DeriveModule(bool with_hook = true) {
        passes[0] = {.name = "bump", .phase = modules::Phase::sensors, .access = kDeriveBumpAccess, .cpu = &derive_bump};
        set = modules::standard_modules();
        modules::ModuleDesc m{.name = "derive",
                              .passes = passes,
                              .state = kDeriveArrays,
                              .scratch = kDeriveScratch,
                              .grade = {.cpu = Grade::reference, .vulkan = Grade::banded},
                              .config = std::as_bytes(std::span<const DeriveConfig, 1>(&kDeriveConfig, 1))};
        if (with_hook) {
            m.rebuild_access = kDeriveRebuildAccess;
            m.rebuild_cpu = &derive_rebuild;
        }
        set.push_back(m);
    }
    DeriveModule(const DeriveModule&) = delete;
    DeriveModule& operator=(const DeriveModule&) = delete;
};

}  // namespace spade::test
