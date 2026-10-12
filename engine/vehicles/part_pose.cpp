// ---------------------------------------------------------------------------
// vehicles/part_pose.cpp -- see part_pose.hpp.
//
// Compiled under spade_fp_strict, like every vehicles/ file: a posed
// transform lands in a world-frame SdfProgram, and stage 7's raster mirrors
// these expressions, so gcc must not contract them into FMA. Every expression
// below is written in the order it is evaluated; the GPU twin keeps it.
// ---------------------------------------------------------------------------
#include "vehicles/part_pose.hpp"

#include <cassert>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/fp32_math.hpp"  // sin32, cos32: the azimuth's turn (TD-3)
#include "core/validate.hpp"   // finite
#include "world/builder.hpp"   // transform_of, compose_transform, SdfPose

namespace spade::vehicles {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// A part as every message names it: its index and its name.
[[nodiscard]] std::string part_label(const PartDesc& part, std::size_t index) {
    return "part " + std::to_string(index) + " '" + part.name + "'";
}

// One rotor span (rotor_omega or psi): empty, or one finite entry per rotor.
[[nodiscard]] Result<void> check_span(std::span<const float> span, const char* name, std::size_t rotors,
                                      const std::string& where) {
    if (!span.empty() && span.size() != rotors) {
        return std::unexpected(invalid(where + name + " has " + std::to_string(span.size()) +
                                       " entries, but the model has " + std::to_string(rotors) +
                                       " rotors; give one per rotor, or none when no rotor part is posed"));
    }
    for (std::size_t k = 0; k < span.size(); ++k) {
        if (!finite(span[k])) {
            return std::unexpected(invalid(where + name + "[" + std::to_string(k) + "] (rotor " +
                                           std::to_string(k) + ") is not finite"));
        }
    }
    return {};
}

}  // namespace

Result<PosedParts> posed_parts(const ModelType& model, const BodyState& body, std::span<const float> rotor_omega,
                               std::span<const float> psi, PartFilter filter) {
    const std::string where = "posed_parts: model '" + model.name + "': ";

    // The body: an orientation that normalizes (ModelType's own rule for a
    // rotation), and a finite position and motion.
    if (!finite(body.orient) || !(glm::dot(body.orient, body.orient) > 0.0f)) {
        return std::unexpected(invalid(where + "the body orientation is not finite and non-zero"));
    }
    if (!finite(body.pos)) return std::unexpected(invalid(where + "the body's pos is not finite"));
    if (!finite(body.vel)) return std::unexpected(invalid(where + "the body's vel is not finite"));
    if (!finite(body.omega_body)) return std::unexpected(invalid(where + "the body's omega_body is not finite"));
    if (filter != PartFilter::all && filter != PartFilter::rigid_only) {
        return std::unexpected(invalid(where + "filter " + std::to_string(static_cast<uint32_t>(filter)) +
                                       " is neither all (0) nor rigid_only (1)"));
    }
    const std::size_t rotors = model.rotors.size();
    if (Result<void> r = check_span(rotor_omega, "rotor_omega", rotors, where); !r) return std::unexpected(r.error());
    if (Result<void> r = check_span(psi, "psi", rotors, where); !r) return std::unexpected(r.error());

    const glm::quat q_body = glm::normalize(body.orient);
    PosedParts out{.com = body.pos, .vel = body.vel, .omega = q_body * body.omega_body, .parts = {}};
    out.parts.reserve(model.parts.size());

    for (std::size_t i = 0; i < model.parts.size(); ++i) {
        const PartDesc& part = model.parts[i];
        const bool on_rotor = part.attach == PartAttach::rotor;
        if (on_rotor && filter == PartFilter::rigid_only) {
            continue;
        }
        const glm::quat q_mount = glm::normalize(part.mount_orient);

        PosedPart posed{.part = static_cast<uint32_t>(i),
                        .world_to_part = SdfTransform{},
                        .origin = glm::vec3(0.0f),
                        .orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                        .hub = glm::vec3(0.0f),
                        .spin = glm::vec3(0.0f)};
        if (!on_rotor) {
            // X = B . M
            posed.origin = body.pos + q_body * part.mount_pos;
            posed.orient = q_body * q_mount;
        } else {
            // X = B . H_k . R_y(s_k psi_k) . M
            if (part.rotor >= rotors) {
                return std::unexpected(invalid(where + part_label(part, i) + ": rotor " + std::to_string(part.rotor) +
                                               " is out of range: the model has " + std::to_string(rotors) +
                                               " rotors (a model that does not validate)"));
            }
            if (rotor_omega.empty() || psi.empty()) {
                const char* empty = rotor_omega.empty() ? (psi.empty() ? "rotor_omega and psi are" : "rotor_omega is")
                                                        : "psi is";
                return std::unexpected(invalid(where + part_label(part, i) + " turns with rotor " +
                                               std::to_string(part.rotor) + ", but " + empty +
                                               " empty; give one entry per rotor"));
            }
            const RotorDesc& rotor = model.rotors[part.rotor];
            const glm::quat q_hub = q_body * glm::normalize(rotor.local_orient);
            const float half = rotor.spin_dir * psi[part.rotor] * 0.5f;
            const glm::quat turn(math::cos32(half), 0.0f, math::sin32(half), 0.0f);  // (w, x, y, z): about +Y
            const glm::quat q_turned = q_hub * turn;
            posed.hub = body.pos + q_body * rotor.local_pos;
            posed.origin = posed.hub + q_turned * part.mount_pos;
            posed.orient = q_turned * q_mount;
            const glm::vec3 axis = q_hub * glm::vec3(0.0f, 1.0f, 0.0f);  // the world thrust axis
            posed.spin = axis * (rotor.spin_dir * rotor_omega[part.rotor]);
        }

        // The builder's own transform, so a part is posed bit for bit as
        // WorldBuilder poses a primitive at X.
        const Result<SdfTransform> t =
            transform_of(SdfPose{.position = posed.origin, .rotation = posed.orient, .scale = 1.0f});
        if (!t) {
            return std::unexpected(Error{t.error().code, where + part_label(part, i) + ": " + t.error().context});
        }
        posed.world_to_part = *t;
        out.parts.push_back(posed);
    }
    return out;
}

Result<SdfProgram> posed_program(const ModelType& model, const PosedParts& posed) {
    const std::string where = "posed_program: model '" + model.name + "': ";
    SdfProgram out{};
    out.transforms.push_back(SdfTransform{});  // transforms[0] is the identity (sdf.hpp)

    for (const PosedPart& p : posed.parts) {
        if (p.part >= model.parts.size()) {
            return std::unexpected(invalid(where + "posed part " + std::to_string(p.part) + " is not one of the model's " +
                                           std::to_string(model.parts.size()) +
                                           " parts; pose the model whose program this is"));
        }
        const PartDesc& part = model.parts[p.part];
        if (part.surface.kind != PartSurface::Kind::wall) {
            continue;  // a porous part is the grid's sink, not a wall
        }
        const std::string label = where + part_label(part, p.part) + ": ";

        // The depth: the first part's own, and each later part's one more,
        // because the program so far holds one value beneath it until its
        // union.
        const Result<uint32_t> depth = part.shape.validate();
        if (!depth) {
            return std::unexpected(Error{depth.error().code, label + "shape: " + depth.error().context});
        }
        const bool joins = !out.nodes.empty();
        const uint64_t needed = static_cast<uint64_t>(*depth) + (joins ? 1u : 0u);
        if (needed > kMaxSdfDepth) {
            return std::unexpected(Error{Code::capacity_exceeded,
                                         label + "the posed wall program would need " + std::to_string(needed) +
                                             " evaluation slots, more than kMaxSdfDepth (" +
                                             std::to_string(kMaxSdfDepth) + ")"});
        }
        const std::size_t base = out.transforms.size();
        if (base + part.shape.transforms.size() > std::numeric_limits<uint32_t>::max()) {
            return std::unexpected(Error{Code::capacity_exceeded,
                                         label + "the posed wall program would hold more than 2^32 - 1 transforms"});
        }

        // compose()'s join (scene/compose.cpp), without materials.
        for (const SdfTransform& t : part.shape.transforms) {
            out.transforms.push_back(compose_transform(t, p.world_to_part));
        }
        for (SdfNode node : part.shape.nodes) {
            // Only a primitive indexes the transforms; an operator's stays 0.
            if (node.op == static_cast<uint32_t>(SdfOp::none)) {
                node.transform += static_cast<uint32_t>(base);
            }
            out.nodes.push_back(node);
        }
        if (joins) {
            SdfNode join{};  // WorldBuilder::union_()'s node, byte for byte
            join.op = static_cast<uint32_t>(SdfOp::union_);
            out.nodes.push_back(join);
        }
    }
    return out;
}

glm::vec3 part_wall_velocity(const PosedParts& posed, std::size_t i, glm::vec3 x) noexcept {
    assert(i < posed.parts.size());
    const PosedPart& p = posed.parts[i];
    return posed.vel + glm::cross(posed.omega, x - posed.com) + glm::cross(p.spin, x - p.hub);
}

SdfSample part_sample(const PartDesc& part, glm::vec3 p_part) noexcept {
    return sample(part.shape, p_part);
}

PartSample sample_parts(const ModelType& model, const PosedParts& posed, glm::vec3 x) noexcept {
    PartSample out{};
    std::size_t winner = posed.parts.size();
    glm::vec3 part_gradient(0.0f);

    // Declaration order. Strictly smaller replaces the winner, so a tie keeps
    // the lower part index (CORE-8 section 5.2), and a NaN never wins.
    for (std::size_t j = 0; j < posed.parts.size(); ++j) {
        const PosedPart& p = posed.parts[j];
        assert(p.part < model.parts.size());
        const SdfTransform& t = p.world_to_part;
        const glm::vec3 local(t.world_to_local * glm::vec4(x, 1.0f));
        const SdfSample s = part_sample(model.parts[p.part], local);
        const float distance = s.distance * t.scale;
        if (distance < out.distance) {
            out.distance = distance;
            part_gradient = s.gradient;
            winner = j;
        }
    }
    if (winner == posed.parts.size()) {
        return out;
    }

    // eval()'s chain rule through a transform (sdf.cpp): for a rigid pose it
    // is the rotation to world.
    const PosedPart& p = posed.parts[winner];
    out.gradient = (glm::transpose(glm::mat3(p.world_to_part.world_to_local)) * part_gradient) * p.world_to_part.scale;
    out.owner = p.part;
    out.surface = model.parts[p.part].surface.kind;
    out.velocity = part_wall_velocity(posed, winner, x);
    return out;
}

}  // namespace spade::vehicles
