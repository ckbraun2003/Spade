// N17's headless rig: the default library's reference quad alone in a bare
// world, in open air, stepped as the editor's Run steps it (4 ms, 4
// substeps). The flight mixer's and controller's tests fly it.
#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "scene/model_file.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace spade::sandbox::flight::test {

inline constexpr uint64_t kStepNs = 4'000'000;  // the editor's Run (builtin_records.hpp)
inline constexpr uint32_t kSubsteps = 4;
inline constexpr float kStepS = 1.0e-9f * static_cast<float>(kStepNs);

struct FlightRig {
    Simulation sim;
    VehicleRef ref{};
    vehicles::ModelType model;  // the library's file's, chain included
};

[[nodiscard]] inline std::filesystem::path reference_quad_file() {
    return std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
}

// Every rotor's speed for a hover: the weight shared evenly along +Y.
[[nodiscard]] inline float hover_omega(const vehicles::ModelType& m) {
    const float weight = m.body.mass * 9.80665f;
    float k = 0.0f;
    for (const vehicles::RotorDesc& r : m.rotors) k += r.thrust_coeff;
    return std::sqrt(weight / k);
}

// The reference quad level at `at`, still, its rotors at `rotor_omega` (none:
// at hover_omega()), in `world` (none: a bare one, open air), flushed so it
// takes rotor commands at once. Fails the calling test, and returns none,
// when anything refuses.
[[nodiscard]] inline std::optional<FlightRig> reference_quad_rig(glm::vec3 at,
                                                                 std::optional<float> rotor_omega = std::nullopt,
                                                                 std::optional<WorldDesc> world = std::nullopt) {
    const Result<scene::ModelFile> file = scene::load_model_file(reference_quad_file());
    if (!file) {
        ADD_FAILURE() << file.error().context;
        return std::nullopt;
    }
    VehicleSpawn start;
    start.pos = at;
    start.rotor_omega = rotor_omega.value_or(hover_omega(file->model));
    if (!world) {
        const Result<WorldDesc> bare = WorldBuilder()
                                           .name("flight")
                                           .environment(Environment{})
                                           .capacities(Capacities{4, 64, 16, 8})
                                           .build();
        if (!bare) {
            ADD_FAILURE() << bare.error().context;
            return std::nullopt;
        }
        world = *bare;
    }
    WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = dryden_params(TurbulenceLevel::none);
    Result<Simulation> sim = Simulation::create(WorldSetDesc{{inst}}, kStepNs, kSubsteps);
    if (!sim) {
        ADD_FAILURE() << sim.error().context;
        return std::nullopt;
    }
    const Result<ModelTypeId> id = sim->register_model(file->model);
    if (!id) {
        ADD_FAILURE() << id.error().context;
        return std::nullopt;
    }
    const Result<VehicleRef> ref = sim->spawn(0, *id, start);
    if (!ref) {
        ADD_FAILURE() << ref.error().context;
        return std::nullopt;
    }
    if (Result<void> flushed = sim->flush_structural(); !flushed) {
        ADD_FAILURE() << flushed.error().context;
        return std::nullopt;
    }
    return FlightRig{std::move(*sim), *ref, file->model};
}

}  // namespace spade::sandbox::flight::test
