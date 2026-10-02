// spawn_helpers.hpp -- bulk body placement (24th spec SL9e, Plan A Task 11).
//
// Closes the v1 "spawn helpers" row of the SL7 transfer register: v1's sandbox
// scattered thousands of bodies through a sphere or a cube, and v2 had only
// one-body-at-a-time spawn().
//
// SEEDED, NEVER AMBIENT. Placement is drawn from core/rng.hpp's splitmix64
// streams with an explicit domain tag, as the stream-derivation rule requires -- never
// std::random_device, never a global generator, never a clock. v1's own
// RandomizeVelocity/RandomizeColor seeded mt19937 from std::random_device on
// every call, which is precisely why the v1 baseline captures can never be a
// numerical reference. The same helper here is reproducible by construction: a
// sandbox scene reopens identically, and a saved scene means the same thing
// tomorrow.
//
// ALL OR NOTHING. Capacity is checked BEFORE any body is spawned, and a spawn
// that fails part-way despawns what this call already placed. A partial spawn
// leaves the caller a half-built scene with no clean way back -- worse than a
// refusal, because it looks like it worked.

#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "sim/simulation.hpp"

namespace spade::objects {

// `velocity_radius_mps` covers v1's SetVelocity/RandomizeVelocity half of the
// SL9e row: 0 spawns everything at rest, and a positive value draws each body's
// initial velocity uniformly from a ball of that radius -- from the SAME seeded
// stream as the positions, so a scene stays reproducible. v1 drew these from a
// std::random_device-seeded mt19937 on every call, which is the specific thing
// that makes its captures unreproducible.
//
// There is no colour here, and that is the disposition rather than an omission:
// in v2 colour is a RenderScene Material, not a property of a body. v1's
// SetColor/RandomizeColor have no body-side equivalent to transfer to.
struct SphereSpawn {
    glm::vec3 center{0.0f};
    float radius = 1.0f;
    uint32_t count = 0;
    uint64_t seed = 0;
    float mass = 1.0f;
    float velocity_radius_mps = 0.0f;
};

struct CubeSpawn {
    glm::vec3 center{0.0f};
    float half_extent = 1.0f;
    uint32_t count = 0;
    uint64_t seed = 0;
    float mass = 1.0f;
    float velocity_radius_mps = 0.0f;
};

// Uniform inside the SOLID sphere (not on its surface), by rejection sampling
// in the unit cube -- which keeps the distribution uniform without a cube root,
// and keeps the draw sequence trivially reproducible.
//
// Errors: capacity_exceeded when `count` exceeds the world's remaining declared
// capacity; invalid_argument for a non-positive radius; whatever spawn() itself
// returns, after despawning this call's own bodies.
[[nodiscard]] Result<std::vector<BodyRef>> spawn_in_sphere(Simulation& sim, uint32_t world_index,
                                                           const SphereSpawn& params);

// Uniform inside an axis-aligned cube.
[[nodiscard]] Result<std::vector<BodyRef>> spawn_in_cube(Simulation& sim, uint32_t world_index,
                                                         const CubeSpawn& params);

}  // namespace spade::objects
