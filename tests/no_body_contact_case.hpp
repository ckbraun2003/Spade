#pragma once

// L87: a body flagged "no body contact" is flown through. The case both tests (CPU and device) run: an interceptor
// (a bare 1 kg body at 10 m/s) flies at a heavy target (1000 kg, at rest) on the gate world, with the world's sphere
// proxy 0.2 m for both (a contact distance of 0.4 m), in the air with nothing else near it. Without the flag the pair
// meets (the interceptor loses most of its speed, the target is pushed); with the flag on the TARGET the interceptor
// keeps its 10 m/s and the target is not touched. Both bodies fall together, so the pass is along +x.

#include <cstdint>
#include <filesystem>

#include <glm/glm.hpp>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/medium.hpp"
#include "world/world_ref.hpp"

namespace spade::testing {

struct FlyThrough {
    glm::vec3 interceptor_vel{0.0f};
    glm::vec3 interceptor_pos{0.0f};
    glm::vec3 target_vel{0.0f};
    glm::vec3 target_pos{0.0f};
};

// 400 steps of 1 ms: the interceptor, from x = -2, is past the target (x = 0) at 0.2 s and 2 m beyond it at the end.
[[nodiscard]] inline Result<FlyThrough> fly_through(compute::BackendKind kind, bool flag_the_target) {
    WorldInstanceDesc prototype;
    prototype.turbulence = dryden_params(TurbulenceLevel::none);
    prototype.contacts.restitution_e = 0.2f;
    prototype.contacts.friction_mu = 0.3f;
    prototype.contacts.baumgarte_beta = 0.2f;
    prototype.contacts.slop = 1.0e-3f;
    prototype.contacts.proxy_radius = 0.2f;
    prototype.grid.cell_size = 0.5f;
    const std::filesystem::path gate = std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / "gate.world.yaml";
    const Result<WorldSetDesc> set = world_set_from(WorldRef{gate}, 1, 0x1087B0D1ULL, prototype);
    if (!set) return std::unexpected(set.error());
    Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, compute::BackendDesc{.kind = kind});
    if (!sim) return std::unexpected(sim.error());

    BodySpawn interceptor;
    interceptor.pos = {-2.0f, 5.0f, 0.0f};
    interceptor.vel = {10.0f, 0.0f, 0.0f};
    interceptor.mass = 1.0f;
    BodySpawn target;
    target.pos = {0.0f, 5.0f, 0.0f};
    target.mass = 1000.0f;
    target.no_body_contact = flag_the_target;
    const Result<BodyRef> a = sim->spawn(0, interceptor);
    if (!a) return std::unexpected(a.error());
    const Result<BodyRef> b = sim->spawn(0, target);
    if (!b) return std::unexpected(b.error());
    if (const Result<void> stepped = sim->step(400); !stepped) return std::unexpected(stepped.error());
    if (const Result<void> synced = sim->sync_resident(); !synced) return std::unexpected(synced.error());
    const Result<const BodyState*> ia = sim->body(*a);
    if (!ia) return std::unexpected(ia.error());
    const Result<const BodyState*> ib = sim->body(*b);
    if (!ib) return std::unexpected(ib.error());
    return FlyThrough{(*ia)->vel, (*ia)->pos, (*ib)->vel, (*ib)->pos};
}

}  // namespace spade::testing
