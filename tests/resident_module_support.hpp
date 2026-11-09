// resident_module_support.hpp -- a test module with one DEVICE-RESIDENT grid (module API stage 7, Task C2).
//
// Module "cells" owns one per_world array "airflow_u" (the set-1 binding of that name, decision 2: names are the contract) of
// kResidentCells floats a world, declared Residency::device (or host, for the comparison runs). Its one pass adds 1.0 to every
// cell in every substep: on the cpu backend as a CPU function, on the device through `recipe` (a test recipe the GPU tests
// supply; null means the module has no GPU kernel and Vulkan refuses its set, as it does a developer's pass). Both are exact in
// fp32, so a CPU run, a Vulkan run and a restore across them agree bit for bit. Built in place and never copied.
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "compute/recipe.hpp"
#include "core/grade.hpp"
#include "physics/schedule.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "state/state_view.hpp"

namespace spade::test {

inline constexpr uint32_t kResidentCells = 4096;  // 16^3
struct ResidentRow {
    float cell[kResidentCells];
};
static_assert(sizeof(ResidentRow) == 16384);

inline void resident_cpu(const physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 1) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<ResidentRow> rows = world_rows<ResidentRow>(ctx.state[0], w);
        if (rows.empty()) continue;
        for (float& c : rows[0].cell) c += 1.0f;
    }
}

inline constexpr modules::QuantityAccess kResidentAccess[] = {{"cells.airflow_u", modules::Access::write}};
inline constexpr modules::ArrayDecl kResidentOnDevice[] = {
    {.name = "airflow_u", .elem_size = sizeof(ResidentRow), .extent = modules::Extent::per_world,
     .residency = modules::Residency::device}};
inline constexpr modules::ArrayDecl kResidentOnHost[] = {
    {.name = "airflow_u", .elem_size = sizeof(ResidentRow), .extent = modules::Extent::per_world}};

// The resident grid of every world of `a` equals `b`'s, bit for bit (the non-const module_rows syncs a device-resident array).
// What a cross-backend test compares: the whole-set digest also folds the dryden state, which the CPU and the device
// advance in their own bands (GpuParity), so it is no exact cross-backend check beyond a few steps.
[[nodiscard]] inline bool grids_equal(Simulation& a, Simulation& b, uint32_t worlds) {
    for (uint32_t w = 0; w < worlds; ++w) {
        const auto ra = a.module_rows<ResidentRow>("airflow_u", w);
        const auto rb = b.module_rows<ResidentRow>("airflow_u", w);
        if (!ra || !rb || std::memcmp(ra->data(), rb->data(), sizeof(ResidentRow)) != 0) return false;
    }
    return true;
}

struct ResidentModule {
    std::array<modules::PassDecl, 1> passes;
    modules::ModuleSet set;

    explicit ResidentModule(modules::Residency residency = modules::Residency::device,
                            const compute::RecipeDesc* recipe = nullptr) {
        passes[0] = {.name = "add_one", .phase = modules::Phase::sensors, .access = kResidentAccess, .cpu = &resident_cpu,
                     .recipe = recipe};
        set = modules::standard_modules();
        set.push_back(modules::ModuleDesc{
            .name = "cells",
            .passes = passes,
            .state = residency == modules::Residency::device ? std::span<const modules::ArrayDecl>(kResidentOnDevice)
                                                              : std::span<const modules::ArrayDecl>(kResidentOnHost),
            .grade = {.cpu = Grade::reference, .vulkan = Grade::banded}});
    }
    ResidentModule(const ResidentModule&) = delete;
    ResidentModule& operator=(const ResidentModule&) = delete;
};

}  // namespace spade::test
