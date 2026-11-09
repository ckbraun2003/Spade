// points_proof_support.hpp -- Review Focus 2 of the stage 7 plan for point samples (module API stage 7, Task D1): a set with a
// position-dependent provider, fed the WORLD's own sample in every point row, steps a drag body to the very bits the standard set
// does, on the CPU and on the device. The provider's pass does nothing (on the device it dispatches the recipe_probe kernel):
// the rows are written once by the host (calm air, the world's density), and Core's positions pass leaves wind and density alone.
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "probe_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace spade::test {

inline constexpr uint32_t kProofBodies = 4;

inline void proof_noop(const physics::SubstepContext&) noexcept {}
inline constexpr modules::QuantityAccess kProofAccess[] = {{"points.airflow_points", modules::Access::write}};
inline constexpr std::string_view kProofAfter[] = {"points.positions"};

struct ProofModule {
    std::array<modules::PassDecl, 1> passes;
    modules::ModuleSet set;
    explicit ProofModule(const compute::RecipeDesc* recipe) {
        passes[0] = {.name = "sample", .phase = modules::Phase::fields, .access = kProofAccess, .after = kProofAfter,
                     .cpu = &proof_noop, .recipe = recipe};
        set = modules::standard_modules();
        set.push_back(modules::ModuleDesc{.name = "calm",
                                          .passes = passes,
                                          .points = modules::PointsDecl{.k = 0, .host_requests = 1, .position_dependent = true},
                                          .grade = {.cpu = Grade::reference, .vulkan = Grade::banded}});
    }
    ProofModule(const ProofModule&) = delete;
    ProofModule& operator=(const ProofModule&) = delete;
};

// One run: a body with a drag element and a velocity, `steps` steps; the body row's bytes afterwards. `rows` true feeds every
// point row the world's sample (no wind, the floor world's 1.225 kg/m^3) before the first step.
[[nodiscard]] inline std::vector<std::byte> proof_run(const modules::ModuleSet& set, compute::BackendKind kind, bool rows, uint32_t steps) {
    const Result<WorldSetDesc> world = floor_world_set(kProofBodies);
    EXPECT_TRUE(world.has_value());
    Result<Simulation> made = Simulation::create(*world, 1'000'000, 2, compute::BackendDesc{.kind = kind}, set);
    EXPECT_TRUE(made.has_value()) << made.error().context;
    Simulation sim = std::move(*made);
    BodySpawn spawn = bare_body(glm::vec3(0.0f, 2.0f, 0.0f));
    spawn.vel = glm::vec3(3.0f, 0.0f, -1.0f);
    const Result<BodyRef> body = sim.spawn(0, spawn);
    EXPECT_TRUE(body.has_value());
    DragElementSpawn drag;
    drag.mode = physics::drag_mode::quadratic;
    drag.area = 0.5f;
    drag.coeffs = glm::vec3(1.0f, 0.0f, 0.0f);
    EXPECT_TRUE(sim.add_drag_element(*body, drag).has_value());
    if (rows) {
        const auto current = sim.module_rows<physics::PointRow>("airflow_points", 0);
        EXPECT_TRUE(current.has_value());
        std::vector<physics::PointRow> all(current->begin(), current->end());
        for (physics::PointRow& r : all) {
            r.wind[0] = r.wind[1] = r.wind[2] = 0.0f;
            r.density = 1.225f;
        }
        EXPECT_TRUE(sim.write_module_rows("airflow_points", 0, std::as_bytes(std::span<const physics::PointRow>(all))).has_value());
    }
    EXPECT_TRUE(sim.step(steps).has_value());
    const BodyState row = row_of(sim, *body);
    std::vector<std::byte> bytes(sizeof(BodyState));
    std::memcpy(bytes.data(), &row, sizeof(BodyState));
    return bytes;
}

}  // namespace spade::test
