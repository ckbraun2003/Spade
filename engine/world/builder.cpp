#include "world/builder.hpp"

#include <cmath>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spade {
namespace {

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }

[[nodiscard]] bool finite(const glm::vec3& v) noexcept {
    return finite(v.x) && finite(v.y) && finite(v.z);
}

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

// ---------------------------------------------------------------------------
// SDF nodes
// ---------------------------------------------------------------------------

uint32_t WorldBuilder::add_transform(const SdfPose& pose) {
    const bool is_identity = pose.position == glm::vec3(0.0f) && pose.scale == 1.0f &&
                             pose.rotation == glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (is_identity) {
        return 0;
    }

    const float qlen = glm::length(pose.rotation);
    if (!finite(pose.position) || !finite(pose.scale) || !finite(qlen)) {
        fail("SDF node pose has non-finite components");
        return 0;
    }
    if (!(pose.scale > 0.0f)) {
        fail("SDF node pose scale must be > 0 (uniform scale only)");
        return 0;
    }
    if (!(qlen > 0.0f)) {
        fail("SDF node pose rotation is a degenerate quaternion");
        return 0;
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

    desc_.sdf.transforms.push_back(t);
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
}

void WorldBuilder::push_op(SdfOp op, const glm::vec4& params) {
    SdfNode node{};
    node.kind = 0;
    node.op = static_cast<uint32_t>(op);
    node.transform = 0;
    node.params = params;
    desc_.sdf.nodes.push_back(node);
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
// build()
// ---------------------------------------------------------------------------

Result<WorldDesc> WorldBuilder::build() const {
    if (error_) {
        return std::unexpected(*error_);
    }

    // Capacities: fixed at creation and never grown, so a zero is always an
    // authoring mistake rather than "unlimited".
    const Capacities& caps = desc_.capacities;
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

    // Spawn points: named, and uniquely named -- they are looked up by name.
    for (size_t i = 0; i < desc_.spawns.size(); ++i) {
        const SpawnPoint& s = desc_.spawns[i];
        if (s.name.empty()) {
            return std::unexpected(invalid("spawn point " + std::to_string(i) + " has no name"));
        }
        for (size_t j = 0; j < i; ++j) {
            if (desc_.spawns[j].name == s.name) {
                return std::unexpected(invalid("duplicate spawn point name '" + s.name + "'"));
            }
        }
    }

    const Environment& env = desc_.environment;
    if (!finite(env.gravity) || !finite(env.wind) || !finite(env.air_density) ||
        !finite(env.temperature_k)) {
        return std::unexpected(invalid("environment has non-finite values"));
    }

    // Structural + parameter validation of the SDF program, including the
    // kMaxSdfDepth bound (reported as capacity_exceeded).
    const Result<uint32_t> depth = desc_.sdf.validate();
    if (!depth) {
        return std::unexpected(depth.error());
    }

    return desc_;
}

}  // namespace spade
