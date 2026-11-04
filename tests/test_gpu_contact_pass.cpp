// test_gpu_contact_pass.cpp -- L9 on the device: collision_static.slang's set
// path, the CPU's contacts.cpp mirrored. Static contact is banded, not bit-equal
// (its SDF walk and normalize are the device's), so the device's Phantom,
// placed at its lowest point and left disarmed for 1 s, rests where the CPU's
// does within the band below and, like it, stays put. Device-executing, so
// Gpu* (the gpu label), skipped without a device; compiled only with
// SPADE_VULKAN.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "library_drones.hpp"
#include "physics/contact_sets.hpp"
#include "sim/simulation.hpp"
#include "state/layout.hpp"
#include "vehicles/airframe_compile.hpp"
#include "world/builder.hpp"

namespace {

namespace ld = spade::test::library_drones;
namespace ph = spade::physics;
using spade::BodyState;
using spade::Simulation;

[[nodiscard]] std::optional<spade::vehicles::ModelType> phantom() {
    const spade::Result<ld::Committed> lib = ld::committed_library();
    if (!lib) return std::nullopt;
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, "phantom3_class");
    if (!drone) return std::nullopt;
    const auto compiled = spade::vehicles::compile_airframe(drone->spec);
    if (!compiled) return std::nullopt;
    return compiled->model;
}

[[nodiscard]] spade::WorldSetDesc ground() {
    spade::WorldBuilder b;
    b.name("contact_pass").environment(spade::Environment{}).capacities(spade::Capacities{4, 16, 4, 4});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    auto built = b.build();
    EXPECT_TRUE(built.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

// The Phantom placed at its lowest point on `backend`, then 1 s at 2 ms; the
// body at the start and at the end.
struct Rest {
    BodyState start{};
    BodyState end{};
};

[[nodiscard]] std::optional<Rest> rest(const spade::vehicles::ModelType& m,
                                       const spade::compute::BackendDesc& backend) {
    auto sim = Simulation::create(ground(), 2'000'000, 2, backend);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    if (!sim) return std::nullopt;
    const auto id = sim->register_model(m);
    EXPECT_TRUE(id.has_value());
    if (!id) return std::nullopt;
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 1.0f, 0.0f);
    const auto v = sim->spawn(0, *id, where);
    EXPECT_TRUE(v.has_value());
    if (!v || !sim->flush_structural()) return std::nullopt;
    const BodyState b = **sim->body(v->body);
    const std::optional<float> low = ph::lowest_contact_point(m, b);
    if (!low) return std::nullopt;
    const glm::vec3 pos = b.pos - glm::vec3(0.0f, *low, 0.0f);
    if (!sim->set_body_state(v->body, pos, b.orient, glm::vec3(0.0f), glm::vec3(0.0f))) return std::nullopt;
    Rest out;
    out.start = **sim->body(v->body);
    EXPECT_TRUE(sim->step(500).has_value());
    out.end = **sim->body(v->body);
    return out;
}

TEST(GpuContactPass, ThePhantomRestsOnTheDeviceWhereItRestsOnTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const std::optional<spade::vehicles::ModelType> m = phantom();
    ASSERT_TRUE(m.has_value());
    const std::optional<Rest> cpu = rest(*m, {});
    const std::optional<Rest> gpu = rest(*m, {.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(cpu.has_value());
    ASSERT_TRUE(gpu.has_value());
    const glm::vec3 d = gpu->end.pos - cpu->end.pos;
    const float apart = std::sqrt(glm::dot(d, d));
    const glm::vec3 moved = gpu->end.pos - gpu->start.pos;
    std::printf("[ L9 ] the Phantom at rest, 1 s: the device %.3e m from the CPU; it moved %.3f mm\n",
                static_cast<double>(apart), 1000.0 * std::sqrt(static_cast<double>(glm::dot(moved, moved))));
    EXPECT_LE(apart, 1.0e-4f) << "the device rests where the CPU does";
    EXPECT_LE(std::sqrt(glm::dot(moved, moved)), 0.010f) << "and stays put";
    const float low = ph::lowest_contact_point(*m, gpu->end).value_or(1.0e9f);
    EXPECT_GE(low, -0.002f);
    EXPECT_LE(low, 0.005f);
}

}  // namespace
