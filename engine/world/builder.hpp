#pragma once

// ---------------------------------------------------------------------------
// WorldBuilder and WorldDesc (engine design D7, spec section 8 "Worlds & the
// file format").
//
// WorldBuilder is the authoring path for a static world: fluent SDF adds, named
// spawn points, the environment, and per-world capacities. build() validates
// and returns an immutable-by-convention WorldDesc -- a plain value struct with
// no engine state and no I/O attached. The versioned YAML world file (S5)
// serializes exactly this product and loads back through the same validation.
//
// FLUENT + ERRORS: the adders return *this so a world reads as one expression,
// which leaves nowhere to return an Error mid-chain. The builder therefore
// records the FIRST authoring error it sees (a degenerate rotation, a
// non-positive scale) and every validation failure is reported by build().
// Nothing is thrown -- no exceptions cross a Spade boundary.
//
// SDF ADDS ARE POSTFIX: a primitive adder pushes one node; an operator adder
// pops two and pushes one. A gate is
//     .torus(1.5f, 0.15f, ring_pose).box(post, left).union_().box(post, right).union_()
// and build() rejects any sequence that does not reduce to exactly one value.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "world/sdf.hpp"

namespace spade {

// Authoring-side pose for an SDF node: rigid + UNIFORM scale. Non-uniform
// scale is deliberately unrepresentable -- it destroys the metric of a distance
// field (see SdfTransform in sdf.hpp).
struct SdfPose {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};  // (w, x, y, z); normalized by the builder
    float scale = 1.0f;
};

// A named pose in the world. Spawn points are how a scenario says "put the
// vehicle here" without hard-coding coordinates in the caller.
struct SpawnPoint {
    std::string name;
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Per-world environment (spec D6). Y-up, metres, SI -- the same frame the SDF
// primitives' local axes assume. The Dryden turbulence state that rides on
// `seed` lives in the Medium implementation (S4), not here.
struct Environment {
    glm::vec3 gravity{0.0f, -9.80665f, 0.0f};
    glm::vec3 wind{0.0f};
    float air_density = 1.225f;    // kg/m^3
    float temperature_k = 288.15f; // K
    uint64_t seed = 0;
};

// Fixed per-world capacities (spec section 4). Allocation is done once at world
// creation and never grows mid-run, so these must all be stated up front and
// must all be > 0.
struct Capacities {
    uint32_t bodies = 0;
    uint32_t force_elements = 0;
    uint32_t sensors = 0;
    uint32_t contacts = 0;
};

// The builder's product: everything needed to instantiate one world, and
// nothing else. Pure value semantics -- copyable, movable, no owned resources
// (so rule-of-zero, not rule-of-five).
struct WorldDesc {
    std::string name;
    SdfProgram sdf;
    std::vector<SpawnPoint> spawns;
    Environment environment{};
    Capacities capacities{};

    // Spawn names are unique in a validated WorldDesc, so this is unambiguous.
    // Returns nullptr when there is no such spawn point.
    [[nodiscard]] const SpawnPoint* find_spawn(std::string_view spawn_name) const noexcept;
};

// ---------------------------------------------------------------------------
// WorldBuilder
// ---------------------------------------------------------------------------
class WorldBuilder {
public:
    WorldBuilder();

    // --- world-level -------------------------------------------------------
    WorldBuilder& name(std::string world_name);
    WorldBuilder& environment(const Environment& env);
    WorldBuilder& capacities(const Capacities& caps);
    WorldBuilder& spawn(std::string spawn_name, glm::vec3 position,
                        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // --- SDF primitives (each pushes one node) -----------------------------
    // `normal` is normalized here; the plane is the half-space dot(p,n) <= offset.
    WorldBuilder& plane(glm::vec3 normal, float offset, const SdfPose& pose = {});
    WorldBuilder& sphere(float radius, const SdfPose& pose = {});
    WorldBuilder& box(glm::vec3 half_extents, const SdfPose& pose = {});
    WorldBuilder& cylinder(float radius, float half_height, const SdfPose& pose = {});
    WorldBuilder& capsule(float radius, float half_height, const SdfPose& pose = {});
    WorldBuilder& torus(float major_radius, float minor_radius, const SdfPose& pose = {});
    WorldBuilder& heightfield(float amplitude, glm::vec2 frequency, float base_height,
                              const SdfPose& pose = {});

    // --- SDF operators (each pops two nodes, pushes one) -------------------
    WorldBuilder& union_();      // trailing underscore: `union` is a keyword
    WorldBuilder& intersect();
    WorldBuilder& subtract();    // "a minus b": the operand added SECOND is removed
    WorldBuilder& smooth_union(float k);

    // --- product -----------------------------------------------------------
    // Validates and returns the world. Codes:
    //   invalid_argument  -- unnamed/duplicate spawn point, capacity == 0,
    //                        non-finite environment, degenerate pose, malformed
    //                        SDF postfix, bad node parameters
    //   capacity_exceeded -- SDF program deeper than kMaxSdfDepth
    [[nodiscard]] Result<WorldDesc> build() const;

private:
    uint32_t add_transform(const SdfPose& pose);
    void push_primitive(SdfPrim kind, const glm::vec4& params, const SdfPose& pose);
    void push_op(SdfOp op, const glm::vec4& params);
    void fail(std::string context);

    WorldDesc desc_{};
    std::optional<Error> error_{};
};

}  // namespace spade
