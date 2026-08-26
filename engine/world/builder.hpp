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
#include <glm/vec4.hpp>

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

// ---------------------------------------------------------------------------
// Materials, lighting and props (schema v2, S7a task W1). All three are
// RENDER-ONLY -- like visual_refs above, physics never reads them, no pass,
// no digest (sim/world_set.cpp's config_hash), no snapshot touches them, and
// two worlds differing only here replay a restored blob identically.
// ---------------------------------------------------------------------------

// Named shading model for a material, so a consumer (S7a task R6) branches on
// a name rather than inventing its own spelling for an integer. Underlying
// type fixed at uint32_t for the same reason SdfPrim/SdfOp are: it round-trips
// through a file and a validated-enum check rather than being UB to cast back.
enum class MaterialShading : uint32_t {
    lambert = 0,   // diffuse, lit by LightingDesc's sun + ambient
    unlit = 1,     // base_color verbatim, no lighting applied
    emissive = 2,  // base_color treated as emitted radiance
};
inline constexpr uint32_t kMaterialShadingCount = 3;

// One named surface appearance. `base_color`'s alpha lane is carried for a
// future transparency pass; today's renderer (R6) is opaque-only and simply
// ignores it. materials[0] -- the default -- always exists once a WorldDesc
// has passed validate_world_desc() (WorldBuilder::build() and the world-file
// loader both guarantee it, inserting this very default when the caller never
// added one), so `node_materials`/`PropDesc::material`'s default value of 0
// always resolves to something.
struct MaterialDesc {
    std::string name = "default";
    glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
    MaterialShading shading = MaterialShading::lambert;
};

// Per-world lighting: one directional sun plus a flat ambient term and a two-
// colour sky gradient (R6 draws the sky as a vertical zenith->horizon blend
// and an analytic ground from these fields; a lighting struct without them
// would force R6 to hardcode a look no world file could ever change).
// `sun_direction` points FROM the scene TOWARD the sun (the direction light
// arrives from, not the direction it travels); need not be pre-normalized --
// R6 normalizes it, the same convention SdfPose's rotation already uses for
// authoring convenience.
struct LightingDesc {
    glm::vec3 sun_direction{0.0f, 1.0f, 0.0f};   // overhead, by default
    glm::vec3 sun_color{1.0f, 1.0f, 1.0f};
    float sun_intensity = 1.0f;
    glm::vec3 ambient_color{0.100000001f, 0.100000001f, 0.100000001f};
    glm::vec3 sky_zenith{0.300000012f, 0.5f, 0.800000012f};
    glm::vec3 sky_horizon{0.800000012f, 0.850000024f, 0.899999976f};
};

// A render-only mesh placement: not part of the collision SDF (spawns and SDF
// nodes cover physics), just a mesh reference, a pose (SdfPose -- the same
// authoring-side pose type an SDF primitive uses) and a material index.
struct PropDesc {
    std::string mesh_ref;
    SdfPose pose;
    uint32_t material = 0;
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

    // RENDER-ONLY references (mesh/material ids the presentation layer
    // resolves). Physics never reads these -- no pass, no digest, no snapshot
    // touches them -- and the only thing validation asks is that each one is
    // non-empty. They live here rather than in a parallel side table because
    // the world file is one document: a world's visuals travel with the world
    // that owns them, and a loader that dropped them would not round-trip.
    std::vector<std::string> visual_refs;

    // Material palette, world lighting and mesh props (schema v2). See the
    // struct comments above; all three are render-only, same as visual_refs.
    // validate_world_desc() guarantees materials is never empty once this
    // WorldDesc has been validated.
    std::vector<MaterialDesc> materials;
    LightingDesc lighting{};
    std::vector<PropDesc> props;

    // Spawn names are unique in a validated WorldDesc, so this is unambiguous.
    // Returns nullptr when there is no such spawn point.
    [[nodiscard]] const SpawnPoint* find_spawn(std::string_view spawn_name) const noexcept;
};

// ---------------------------------------------------------------------------
// THE ONE VALIDATION. Everything that can produce a WorldDesc runs exactly
// this function -- WorldBuilder::build() (construct, then validate, then move
// out) and the YAML loader (world/world_file.hpp: parse, then validate). The
// engine design's "parse -> same validation -> WorldDesc" is a fact about this
// call, not a promise two code paths make separately and drift apart on.
//
// A THIRD call site exists for the same reason: world/world_ref.hpp's
// resolve_world() accepts a WorldRef whose desc alternative may hold a
// WorldDesc that reached the caller some OTHER way -- a test fixture, a
// generated scene, a hand-assembled aggregate -- and did not necessarily
// pass through either producer above. resolve_world() validates that copy
// itself (S5 final-review fix wave, I5) before handing it on, which is what
// keeps "one validation" true for every WorldDesc an engine consumer can
// actually reach through the sanctioned WorldRef path, not only the two that
// build one from scratch.
//
// On success returns the SDF program's peak evaluation-stack depth, exactly as
// SdfProgram::validate() does (0 for a program with no nodes).
//
// Codes:
//   invalid_argument  -- capacity == 0, unnamed/duplicate spawn point,
//                        non-finite or non-unit spawn pose, non-finite
//                        environment, empty visual reference, malformed SDF
//                        postfix, bad node parameters, an empty materials
//                        palette, an unknown material shading value, a
//                        non-finite material/lighting value, a prop with an
//                        empty mesh_ref or a non-finite/non-unit/non-positive
//                        pose, or a materials/node_materials/prop material
//                        index out of range
//   capacity_exceeded -- SDF program deeper than kMaxSdfDepth
//
// It does NOT normalize anything. A loader that re-normalized a spawn
// quaternion would perturb its last bit and break the file's round trip; the
// builder normalizes at authoring time instead, and this only checks.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<uint32_t> validate_world_desc(const WorldDesc& desc);

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

    // A render-only reference carried with the world (WorldDesc::visual_refs).
    // Rejected empty; otherwise opaque to the engine.
    WorldBuilder& visual(std::string ref);

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

    // --- materials, lighting, props (schema v2) -----------------------------
    // Appends a material and returns *this; its index is materials().size()-1
    // on the desc build() would produce (i.e. one past every material() call
    // so far), which is what material_for_last_node()/prop() take.
    WorldBuilder& material(MaterialDesc m);
    WorldBuilder& lighting(const LightingDesc& light);
    // Rejected empty mesh_ref; `pose` is normalized the same way an SDF
    // primitive's pose is (see plane()/sphere()/... above).
    WorldBuilder& prop(std::string mesh_ref, const SdfPose& pose, uint32_t material = 0);
    // Sets the material index of the SDF node most recently pushed by a
    // primitive or operator adder above. Growing node_materials to
    // nodes.size() lazily (filling earlier slots with the default, index 0)
    // is this call's job, not build()'s -- see sdf.hpp's node_materials note.
    WorldBuilder& material_for_last_node(uint32_t material_index);

    // --- product -----------------------------------------------------------
    // Construct, then validate_world_desc(), then hand the world over. The
    // first recorded authoring error (if any) is reported ahead of validation,
    // because it names the offending CALL, which a check on the finished
    // product no longer can. See validate_world_desc() above for the codes.
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
