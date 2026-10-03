#include "world/builder.hpp"

#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/validate.hpp"

namespace spade {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

}  // namespace

const SpawnPoint* WorldDesc::find_spawn(std::string_view spawn_name) const noexcept {
    for (const SpawnPoint& s : spawns) {
        if (s.name == spawn_name) {
            return &s;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Construction / world-level setters
// ---------------------------------------------------------------------------

WorldBuilder::WorldBuilder() {
    // transforms[0] is the identity by convention (sdf.hpp): every node with an
    // identity pose indexes it, so the common case costs no extra storage.
    desc_.sdf.transforms.push_back(SdfTransform{});
}

void WorldBuilder::fail(std::string context) {
    if (!error_) {  // first error wins; later adds are recorded but never reported
        error_ = invalid(std::move(context));
    }
}

WorldBuilder& WorldBuilder::name(std::string world_name) {
    desc_.name = std::move(world_name);
    return *this;
}

WorldBuilder& WorldBuilder::environment(const Environment& env) {
    desc_.environment = env;
    return *this;
}

WorldBuilder& WorldBuilder::capacities(const Capacities& caps) {
    desc_.capacities = caps;
    return *this;
}

WorldBuilder& WorldBuilder::spawn(std::string spawn_name, glm::vec3 position,
                                  glm::quat orientation) {
    const float qlen = glm::length(orientation);
    if (!finite(position) || !finite(qlen) || !(qlen > 0.0f)) {
        fail("spawn point '" + spawn_name + "' has a non-finite position or degenerate rotation");
        return *this;
    }
    desc_.spawns.push_back(SpawnPoint{std::move(spawn_name), position, orientation / qlen});
    return *this;
}

WorldBuilder& WorldBuilder::visual(std::string ref) {
    if (ref.empty()) {
        fail("visual reference must not be empty");
        return *this;
    }
    desc_.visual_refs.push_back(std::move(ref));
    return *this;
}

// ---------------------------------------------------------------------------
// SDF nodes
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] bool is_identity_pose(const SdfPose& pose) noexcept {
    return pose.position == glm::vec3(0.0f) && pose.scale == 1.0f &&
           pose.rotation == glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
}

}  // namespace

Result<SdfTransform> transform_of(const SdfPose& pose) {
    if (is_identity_pose(pose)) {
        return SdfTransform{};
    }

    const float qlen = glm::length(pose.rotation);
    if (!finite(pose.position) || !finite(pose.scale) || !finite(qlen)) {
        return std::unexpected(invalid("SDF node pose has non-finite components"));
    }
    if (!(pose.scale > 0.0f)) {
        return std::unexpected(invalid("SDF node pose scale must be > 0 (uniform scale only)"));
    }
    if (!(qlen > 0.0f)) {
        return std::unexpected(invalid("SDF node pose rotation is a degenerate quaternion"));
    }

    // Stored pre-inverted (see SdfTransform). Built analytically rather than via
    // glm::inverse: inv(T * R * S) = (1/s)R^T * translate(-position), which is
    // both exact and cheaper than a general 4x4 inversion.
    const glm::mat3 m = glm::transpose(glm::mat3_cast(pose.rotation / qlen)) / pose.scale;
    const glm::vec3 c = -(m * pose.position);

    SdfTransform t{};
    t.world_to_local = glm::mat4(m);
    t.world_to_local[3] = glm::vec4(c, 1.0f);
    t.scale = pose.scale;
    return t;
}

uint32_t WorldBuilder::add_transform(const SdfPose& pose) {
    // The identity shares transform 0 (pushed by the constructor) rather than
    // pushing a copy, so every unposed primitive points at one row.
    if (is_identity_pose(pose)) {
        return 0;
    }
    const Result<SdfTransform> t = transform_of(pose);
    if (!t) {
        fail(t.error().context);
        return 0;
    }
    desc_.sdf.transforms.push_back(*t);
    return static_cast<uint32_t>(desc_.sdf.transforms.size() - 1);
}

void WorldBuilder::push_primitive(SdfPrim kind, const glm::vec4& params, const SdfPose& pose) {
    const uint32_t transform = add_transform(pose);
    SdfNode node{};
    node.kind = static_cast<uint32_t>(kind);
    node.op = static_cast<uint32_t>(SdfOp::none);
    node.transform = transform;
    node.params = params;
    desc_.sdf.nodes.push_back(node);
    // Keep node_materials in lockstep once ANY material_for_last_node() call
    // has activated it (sdf.hpp's invariant: empty, or exactly nodes.size()).
    // Without this, a node pushed AFTER an earlier material_for_last_node()
    // call would silently desync the two arrays until the next such call --
    // this way, calling it once mid-chain and never again still leaves a
    // fully-sized, valid array.
    if (!desc_.sdf.node_materials.empty()) {
        desc_.sdf.node_materials.push_back(0);
    }
}

void WorldBuilder::push_op(SdfOp op, const glm::vec4& params) {
    SdfNode node{};
    node.kind = 0;
    node.op = static_cast<uint32_t>(op);
    node.transform = 0;
    node.params = params;
    desc_.sdf.nodes.push_back(node);
    if (!desc_.sdf.node_materials.empty()) {
        desc_.sdf.node_materials.push_back(0);
    }
}

WorldBuilder& WorldBuilder::plane(glm::vec3 normal, float offset, const SdfPose& pose) {
    const float len = glm::length(normal);
    if (!finite(len) || !(len > 0.0f)) {
        fail("plane normal must be a non-zero, finite vector");
        return *this;
    }
    // Normalized here so the stored node is metric by construction; validate()
    // re-checks it for hand-built programs.
    const glm::vec3 n = normal / len;
    push_primitive(SdfPrim::plane, glm::vec4(n, offset), pose);
    return *this;
}

WorldBuilder& WorldBuilder::sphere(float radius, const SdfPose& pose) {
    push_primitive(SdfPrim::sphere, glm::vec4(radius, 0.0f, 0.0f, 0.0f), pose);
    return *this;
}

WorldBuilder& WorldBuilder::box(glm::vec3 half_extents, const SdfPose& pose) {
    push_primitive(SdfPrim::box, glm::vec4(half_extents, 0.0f), pose);
    return *this;
}

WorldBuilder& WorldBuilder::cylinder(float radius, float half_height, const SdfPose& pose) {
    push_primitive(SdfPrim::cylinder, glm::vec4(radius, half_height, 0.0f, 0.0f), pose);
    return *this;
}

WorldBuilder& WorldBuilder::capsule(float radius, float half_height, const SdfPose& pose) {
    push_primitive(SdfPrim::capsule, glm::vec4(radius, half_height, 0.0f, 0.0f), pose);
    return *this;
}

WorldBuilder& WorldBuilder::torus(float major_radius, float minor_radius, const SdfPose& pose) {
    push_primitive(SdfPrim::torus, glm::vec4(major_radius, minor_radius, 0.0f, 0.0f), pose);
    return *this;
}

WorldBuilder& WorldBuilder::heightfield(float amplitude, glm::vec2 frequency, float base_height,
                                        const SdfPose& pose) {
    push_primitive(SdfPrim::heightfield,
                   glm::vec4(amplitude, frequency.x, frequency.y, base_height), pose);
    return *this;
}

WorldBuilder& WorldBuilder::union_() {
    push_op(SdfOp::union_, glm::vec4(0.0f));
    return *this;
}

WorldBuilder& WorldBuilder::intersect() {
    push_op(SdfOp::intersect, glm::vec4(0.0f));
    return *this;
}

WorldBuilder& WorldBuilder::subtract() {
    push_op(SdfOp::subtract, glm::vec4(0.0f));
    return *this;
}

WorldBuilder& WorldBuilder::smooth_union(float k) {
    push_op(SdfOp::smooth_union, glm::vec4(k, 0.0f, 0.0f, 0.0f));
    return *this;
}

// ---------------------------------------------------------------------------
// Materials, lighting, props (schema v2)
// ---------------------------------------------------------------------------

WorldBuilder& WorldBuilder::material(MaterialDesc m) {
    desc_.materials.push_back(std::move(m));
    return *this;
}

WorldBuilder& WorldBuilder::lighting(const LightingDesc& light) {
    desc_.lighting = light;
    return *this;
}

WorldBuilder& WorldBuilder::prop(std::string mesh_ref, const SdfPose& pose, uint32_t material) {
    if (mesh_ref.empty()) {
        fail("prop mesh reference must not be empty");
        return *this;
    }
    const float qlen = glm::length(pose.rotation);
    if (!finite(pose.position) || !finite(pose.scale) || !finite(qlen)) {
        fail("prop '" + mesh_ref + "' has a non-finite pose");
        return *this;
    }
    if (!(pose.scale > 0.0f)) {
        fail("prop '" + mesh_ref + "' pose scale must be > 0 (uniform scale only)");
        return *this;
    }
    if (!(qlen > 0.0f)) {
        fail("prop '" + mesh_ref + "' pose rotation is a degenerate quaternion");
        return *this;
    }

    PropDesc p;
    p.mesh_ref = std::move(mesh_ref);
    p.pose = pose;
    p.pose.rotation = pose.rotation / qlen;  // same normalize-at-authoring-time as spawn()
    p.material = material;
    desc_.props.push_back(std::move(p));
    return *this;
}

WorldBuilder& WorldBuilder::material_for_last_node(uint32_t material_index) {
    if (desc_.sdf.nodes.empty()) {
        fail("material_for_last_node() called before any SDF node was pushed");
        return *this;
    }
    // Grow lazily to nodes.size() (sdf.hpp's node_materials contract: empty or
    // exactly nodes.size()), defaulting every not-yet-assigned slot to 0.
    if (desc_.sdf.node_materials.size() != desc_.sdf.nodes.size()) {
        desc_.sdf.node_materials.resize(desc_.sdf.nodes.size(), 0);
    }
    desc_.sdf.node_materials.back() = material_index;
    return *this;
}

// ---------------------------------------------------------------------------
// validate_material() -- one material's rules, shared by validate_world_desc()
// and the scene file (TD-9). See the contract in builder.hpp.
// ---------------------------------------------------------------------------

Result<void> validate_material(const MaterialDesc& material, std::string_view label) {
    // Non-empty, same rule as visual_refs and PropDesc::mesh_ref.
    if (material.name.empty()) {
        return std::unexpected(invalid(std::string(label) + " has an empty name"));
    }
    if (!finite(material.base_color)) {
        return std::unexpected(invalid(std::string(label) + " has a non-finite base_color"));
    }
    if (static_cast<uint32_t>(material.shading) >= kMaterialShadingCount) {
        return std::unexpected(invalid(std::string(label) + " has an unknown shading value"));
    }
    return {};
}

// ---------------------------------------------------------------------------
// validate_world_desc() -- the one validation, shared by build() and by the
// world-file loader. See the contract in builder.hpp.
// ---------------------------------------------------------------------------

Result<uint32_t> validate_world_desc(const WorldDesc& desc) {
    // Capacities: fixed at creation and never grown, so a zero is always an
    // authoring mistake rather than "unlimited".
    const Capacities& caps = desc.capacities;
    if (caps.bodies == 0) {
        return std::unexpected(invalid("world capacity 'bodies' must be > 0"));
    }
    if (caps.force_elements == 0) {
        return std::unexpected(invalid("world capacity 'force_elements' must be > 0"));
    }
    if (caps.sensors == 0) {
        return std::unexpected(invalid("world capacity 'sensors' must be > 0"));
    }
    if (caps.contacts == 0) {
        return std::unexpected(invalid("world capacity 'contacts' must be > 0"));
    }

    // Spawn points: named, uniquely named (they are looked up by name), and
    // carrying a usable pose. The builder's spawn() already rejects a
    // non-finite position and normalizes the rotation, so for a builder-made
    // world these re-check what is true by construction -- they exist for the
    // OTHER producer, the world file, whose numbers came out of a text editor.
    //
    // The unit-quaternion tolerance is SdfProgram::validate()'s plane-normal
    // tolerance (1e-3 on the squared length), deliberately the same number:
    // both answer "is this direction close enough to unit that the geometry it
    // describes is still metric", and having two answers to that in one
    // validator would be a defect waiting to be found.
    for (size_t i = 0; i < desc.spawns.size(); ++i) {
        const SpawnPoint& s = desc.spawns[i];
        if (s.name.empty()) {
            return std::unexpected(invalid("spawn point " + std::to_string(i) + " has no name"));
        }
        for (size_t j = 0; j < i; ++j) {
            if (desc.spawns[j].name == s.name) {
                return std::unexpected(invalid("duplicate spawn point name '" + s.name + "'"));
            }
        }
        if (!finite(s.position)) {
            return std::unexpected(
                invalid("spawn point '" + s.name + "' has a non-finite position"));
        }
        if (!finite(s.orientation)) {
            return std::unexpected(
                invalid("spawn point '" + s.name + "' has a non-finite orientation"));
        }
        const glm::quat& q = s.orientation;
        const float q2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (!(glm::abs(q2 - 1.0f) <= 1e-3f)) {
            return std::unexpected(
                invalid("spawn point '" + s.name + "' orientation must be a unit quaternion"));
        }
    }

    const Environment& env = desc.environment;
    if (!finite(env.gravity) || !finite(env.wind) || !finite(env.air_density) ||
        !finite(env.temperature_k)) {
        return std::unexpected(invalid("environment has non-finite values"));
    }

    // Render-only, and the engine's only claim about one is that it names
    // something. An empty string names nothing and is always a mistake.
    for (size_t i = 0; i < desc.visual_refs.size(); ++i) {
        if (desc.visual_refs[i].empty()) {
            return std::unexpected(
                invalid("visual reference " + std::to_string(i) + " is empty"));
        }
    }

    // --- materials, lighting, props (schema v2) -----------------------------
    //
    // materials must never be empty: index 0 is the default every
    // node_materials/PropDesc::material entry falls back to, and both
    // producers of a WorldDesc (WorldBuilder::build(), world_from_yaml()'s v1
    // upgrade path and its v2 parse) guarantee at least one entry before
    // calling this function -- so an empty palette here can only mean a
    // hand-assembled WorldDesc (world/world_ref.hpp's resolve_world() desc
    // alternative) that skipped that step.
    if (desc.materials.empty()) {
        return std::unexpected(invalid("world must have at least one material (index 0)"));
    }
    for (size_t i = 0; i < desc.materials.size(); ++i) {
        // Not uniqueness -- materials are referenced by index, never by name.
        if (Result<void> r = validate_material(desc.materials[i], "material " + std::to_string(i)); !r) {
            return std::unexpected(r.error());
        }
    }

    const LightingDesc& light = desc.lighting;
    if (!finite(light.sun_direction) || !finite(light.sun_color) || !finite(light.sun_intensity) ||
        !finite(light.ambient_color) || !finite(light.sky_zenith) || !finite(light.sky_horizon)) {
        return std::unexpected(invalid("lighting has non-finite values"));
    }
    // sun_direction need not be PRE-normalized (builder.hpp's own note -- R6
    // normalizes it), but it must be normalizABLE: a zero vector has no
    // direction, and normalizing one hands R6 a NaN. Same shape as
    // plane()'s own normal check just above (finite, then non-zero), the
    // only difference being that this is enforced here rather than at an
    // adder, because WorldBuilder::lighting() stores the struct verbatim
    // (there is no per-field setter to normalize at) and the file loader
    // reaches this same check by construction, both producers included.
    if (!(glm::length(light.sun_direction) > 0.0f)) {
        return std::unexpected(invalid("lighting.sun_direction must be a non-zero vector"));
    }

    for (size_t i = 0; i < desc.props.size(); ++i) {
        const PropDesc& p = desc.props[i];
        if (p.mesh_ref.empty()) {
            return std::unexpected(invalid("prop " + std::to_string(i) + " has an empty mesh_ref"));
        }
        const float qlen = glm::length(p.pose.rotation);
        if (!finite(p.pose.position) || !finite(p.pose.scale) || !finite(qlen)) {
            return std::unexpected(
                invalid("prop '" + p.mesh_ref + "' has a non-finite pose"));
        }
        if (!(p.pose.scale > 0.0f)) {
            return std::unexpected(
                invalid("prop '" + p.mesh_ref + "' pose scale must be > 0 (uniform scale only)"));
        }
        // Same 1e-3 tolerance as a spawn point's orientation above -- both
        // answer "is this direction close enough to unit that the pose it
        // describes is metric", and this is NOT re-normalized for the same
        // reason a spawn's is not (see validate_world_desc's own note above):
        // re-normalizing here would perturb a file-loaded pose's last bit and
        // break its round trip.
        const float q2 = p.pose.rotation.w * p.pose.rotation.w + p.pose.rotation.x * p.pose.rotation.x +
                         p.pose.rotation.y * p.pose.rotation.y + p.pose.rotation.z * p.pose.rotation.z;
        if (!(glm::abs(q2 - 1.0f) <= 1e-3f)) {
            return std::unexpected(
                invalid("prop '" + p.mesh_ref + "' pose rotation must be a unit quaternion"));
        }
        if (p.material >= desc.materials.size()) {
            return std::unexpected(
                invalid("prop '" + p.mesh_ref + "' material index out of range"));
        }
    }

    // node_materials' LENGTH is SdfProgram::validate()'s own job (it has the
    // node count; it does not have the materials palette); each entry's VALUE
    // is this function's job, since only here is desc.materials in scope.
    for (size_t i = 0; i < desc.sdf.node_materials.size(); ++i) {
        if (desc.sdf.node_materials[i] >= desc.materials.size()) {
            return std::unexpected(invalid("SDF node " + std::to_string(i) +
                                           "'s material index out of range"));
        }
    }

    // Structural + parameter validation of the SDF program, including the
    // kMaxSdfDepth bound (reported as capacity_exceeded). Returns the peak
    // evaluation-stack depth, which is this function's own result.
    return desc.sdf.validate();
}

// ---------------------------------------------------------------------------
// build()
// ---------------------------------------------------------------------------

Result<WorldDesc> WorldBuilder::build() const {
    if (error_) {
        return std::unexpected(*error_);
    }
    // build() is const (the fluent chain never mutates desc_ from here), so
    // the default-material fallback works on a COPY: a caller who never calls
    // .material() still gets a valid, non-empty palette (validate_world_desc's
    // "materials must have at least one entry" rule), without .material()
    // itself needing to pre-seed index 0 the way the constructor pre-seeds
    // transforms[0] -- that would make a caller's FIRST .material() call land
    // at index 1, not 0.
    WorldDesc desc = desc_;
    if (desc.materials.empty()) {
        desc.materials.push_back(MaterialDesc{});
    }
    const Result<uint32_t> depth = validate_world_desc(desc);
    if (!depth) {
        return std::unexpected(depth.error());
    }
    return desc;
}

}  // namespace spade
