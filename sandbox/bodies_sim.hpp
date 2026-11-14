// The smoke tour's chapter 4, "Bodies" (the lead's audit, 2026-11-13): L68's contact tumbling, through Core's public API
// only: a Simulation whose module set holds rigid_contact_module(), and set_body_shape() on each body. A row of four
// shapes dropped together onto the ground, spaced so that none can touch another (body-body contact is not in L68):
//
//   a sphere   thrown along the ground: it slides until v = omega r, then rolls on at 5/7 of the throw;
//   a cube     dropped on a corner at a generic tilt: it spins up and settles on a face;
//   a capsule  dropped a few degrees off flat: it lies down on both ends;
//   a hull     drone-sized (0.45 m across the arm tips, a body and four feet), dropped tilted: it lands flat.
//
// Each scenario is one that tests/test_rigid_contact.cpp proves alone; here they run together in one world, and the
// verdicts below are those tests' own (their tolerances). No window: tests/test_sandbox_bodies.cpp runs it, and the tour
// (bodies_session.hpp) draws it. 1 kHz, one substep, the CPU backend; the same bytes every run.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "physics/rigid_contact.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace spade::sandbox::bodies {

enum class Kind { sphere, box, capsule, hull };

struct Piece {
    Kind kind = Kind::box;
    std::string name;
    glm::vec3 start{0.0f};  // the body's centre at the start, m
    glm::quat tilt{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 vel{0.0f};
    float mass = 1.0f;
    glm::vec3 inv_inertia{1.0f};
    physics::ShapeDesc shape;
    float bound = 0.0f;  // a sphere about the piece's centre that holds all of it, m
};

struct Verdict {
    bool ok = false;
    std::string detail;
};

inline constexpr float kGravity = 9.80665f;
inline constexpr float kSphereRadius = 0.1f;
inline constexpr float kSphereThrow = 0.6f;  // m/s along +z
inline constexpr float kCubeHalf = 0.1f;
inline constexpr float kCapsuleRadius = 0.05f;
inline constexpr float kCapsuleHalfLength = 0.3f;
inline constexpr float kHullFoot = 0.03f;  // the hull's half height

// The drone-sized hull's points: the body's eight corners, and four feet out at the arm tips, a diagonal of 0.32 m.
[[nodiscard]] inline std::vector<glm::vec3> drone_hull_points() {
    std::vector<glm::vec3> p;
    for (int c = 0; c < 8; ++c) {
        p.emplace_back((c & 1) ? 0.08f : -0.08f, (c & 2) ? kHullFoot : -kHullFoot, (c & 4) ? 0.08f : -0.08f);
    }
    constexpr float tip = 0.2263f;  // 0.32 / sqrt 2
    for (int c = 0; c < 4; ++c) p.emplace_back((c & 1) ? tip : -tip, -kHullFoot, (c & 2) ? tip : -tip);
    return p;
}

// The row, left to right along x. Each piece's line is its own x, so the gaps below hold for the whole run.
[[nodiscard]] inline std::vector<Piece> row() {
    std::vector<Piece> out;
    {
        Piece p;
        p.kind = Kind::sphere;
        p.name = "sphere";
        p.start = glm::vec3(-1.8f, kSphereRadius, -1.7f);
        p.vel = glm::vec3(0.0f, 0.0f, kSphereThrow);
        p.mass = 1.0f;
        p.inv_inertia = glm::vec3(1.0f / (0.4f * kSphereRadius * kSphereRadius));
        p.shape = physics::sphere_shape(kSphereRadius);
        p.shape.friction = 0.5f;
        p.shape.restitution = 0.0f;
        p.bound = kSphereRadius;
        out.push_back(p);
    }
    {
        Piece p;
        p.kind = Kind::box;
        p.name = "cube";
        p.tilt = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 0.4f, 0.2f)));
        // The centre height that puts the lowest corner 1 m up at that tilt.
        float low = 1e9f;
        for (int c = 0; c < 8; ++c) {
            const glm::vec3 corner((c & 1) ? kCubeHalf : -kCubeHalf, (c & 2) ? kCubeHalf : -kCubeHalf,
                                   (c & 4) ? kCubeHalf : -kCubeHalf);
            low = std::min(low, (p.tilt * corner).y);
        }
        p.start = glm::vec3(-0.9f, 1.0f - low, 0.0f);
        p.mass = 1.0f;
        p.inv_inertia = physics::box_inv_inertia(1.0f, glm::vec3(kCubeHalf));
        p.shape = physics::box_shape(glm::vec3(kCubeHalf));
        p.shape.restitution = 0.2f;
        p.shape.friction = 0.5f;
        p.bound = std::sqrt(3.0f) * kCubeHalf;
        out.push_back(p);
    }
    {
        Piece p;
        p.kind = Kind::capsule;
        p.name = "capsule";
        p.start = glm::vec3(0.2f, 0.5f, 0.0f);
        p.tilt = glm::angleAxis(glm::radians(5.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        p.mass = 1.0f;
        p.inv_inertia = glm::vec3(400.0f, 30.0f, 30.0f);
        p.shape = physics::capsule_shape(glm::vec3(-kCapsuleHalfLength, 0.0f, 0.0f),
                                         glm::vec3(kCapsuleHalfLength, 0.0f, 0.0f), kCapsuleRadius);
        p.shape.friction = 0.3f;
        p.shape.restitution = 0.1f;
        p.bound = kCapsuleHalfLength + kCapsuleRadius;
        out.push_back(p);
    }
    {
        Piece p;
        p.kind = Kind::hull;
        p.name = "drone hull";
        p.start = glm::vec3(1.4f, 0.5f, 0.0f);
        p.tilt = glm::angleAxis(glm::radians(8.0f), glm::normalize(glm::vec3(1.0f, 0.0f, 0.3f)));
        p.mass = 2.0f;
        p.inv_inertia = physics::box_inv_inertia(2.0f, glm::vec3(0.2263f, kHullFoot, 0.2263f));
        p.shape = physics::hull_shape(drone_hull_points());
        p.shape.friction = 0.5f;
        p.shape.restitution = 0.1f;
        p.bound = 0.33f;
        out.push_back(p);
    }
    return out;
}

class BodiesSim {
  public:
    static constexpr uint32_t kWindow = 500;  // steps: the half second a verdict looks back over

    [[nodiscard]] static Result<BodiesSim> create(compute::BackendKind kind = compute::BackendKind::cpu) {
        Environment env;
        env.gravity = glm::vec3(0.0f, -kGravity, 0.0f);
        env.wind = glm::vec3(0.0f);
        env.air_density = 1.225f;
        WorldBuilder b;
        b.name("bodies").environment(env).capacities(Capacities{8, 12, 4, 1});
        b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
        const auto built = b.build();
        if (!built) return std::unexpected(built.error());
        WorldInstanceDesc inst;
        inst.world = *built;
        inst.seed = 7;
        inst.turbulence = dryden_params(TurbulenceLevel::none);
        inst.contacts.restitution_e = 0.0f;
        inst.contacts.friction_mu = 0.0f;
        inst.contacts.proxy_radius = 0.0f;
        modules::ModuleSet set = modules::standard_modules();
        set.push_back(modules::rigid_contact_module());
        Result<Simulation> sim = Simulation::create(WorldSetDesc{{inst}}, 1'000'000, 1, compute::BackendDesc{.kind = kind}, set);
        if (!sim) return std::unexpected(sim.error());
        BodiesSim out;
        out.sim_.emplace(std::move(*sim));
        out.pieces_ = row();
        for (const Piece& p : out.pieces_) {
            BodySpawn s;
            s.pos = p.start;
            s.vel = p.vel;
            s.orient = p.tilt;
            s.mass = p.mass;
            s.inv_inertia_diag = p.inv_inertia;
            const Result<BodyRef> ref = out.sim_->spawn(0, s);
            if (!ref) return std::unexpected(ref.error());
            if (const Result<void> f = out.sim_->flush_structural(); !f) return std::unexpected(f.error());
            if (const Result<void> shaped = out.sim_->set_body_shape(*ref, p.shape); !shaped) {
                return std::unexpected(shaped.error());
            }
            out.refs_.push_back(*ref);
        }
        out.speed_.assign(out.pieces_.size(), {});
        out.spin_.assign(out.pieces_.size(), {});
        return out;
    }

    // n steps of 1 ms.
    [[nodiscard]] Result<void> step(uint32_t n) {
        for (uint32_t i = 0; i < n; ++i) {
            if (const Result<void> stepped = sim_->step(1); !stepped) return stepped;
            ++steps_;
            const std::size_t slot = static_cast<std::size_t>(steps_ % kWindow);
            for (std::size_t k = 0; k < refs_.size(); ++k) {
                const BodyState b = state(k);
                speed_[k][slot] = glm::length(b.vel);
                spin_[k][slot] = glm::length(b.orient * b.omega_body);
            }
        }
        return {};
    }

    // A frame of `dt` seconds: whole milliseconds of the sim, the rest carried to the next frame.
    [[nodiscard]] Result<void> advance(float dt) {
        carry_ms_ += static_cast<double>(dt) * 1000.0;
        const double whole = std::floor(carry_ms_);
        carry_ms_ -= whole;
        return step(static_cast<uint32_t>(whole));
    }

    [[nodiscard]] uint64_t steps() const noexcept { return steps_; }
    [[nodiscard]] const std::vector<Piece>& pieces() const noexcept { return pieces_; }

    [[nodiscard]] BodyState state(std::size_t i) const {
        const Result<const BodyState*> row = sim_->body(refs_[i]);
        return row ? **row : BodyState{};
    }

    // What this piece's shape says it does, at the current state. Not ok before a half second has passed: nothing is
    // at rest on the first frame.
    [[nodiscard]] Verdict verdict(std::size_t i) const {
        if (i >= pieces_.size()) return {false, "no such piece"};
        if (steps_ < kWindow) return {false, std::format("{} s of the run: too early to say", static_cast<double>(steps_) / 1000.0)};
        const Piece& p = pieces_[i];
        const BodyState b = state(i);
        const float v = *std::max_element(speed_[i].begin(), speed_[i].end());
        const float w = *std::max_element(spin_[i].begin(), spin_[i].end());
        const glm::vec3 up_in_body = glm::conjugate(b.orient) * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 axis = b.orient * glm::vec3(1.0f, 0.0f, 0.0f);
        switch (p.kind) {
            case Kind::sphere: {
                const float rolling = kSphereThrow * 5.0f / 7.0f;
                const float speed = glm::length(b.vel);
                const float omega_r = glm::length(b.orient * b.omega_body) * kSphereRadius;
                const bool ok = std::abs(speed - rolling) <= 0.03f * rolling && std::abs(omega_r - speed) <= 0.03f * rolling &&
                                std::abs(b.pos.y - kSphereRadius) <= 0.01f;
                return {ok, std::format("rolling on at {:.3f} m/s (5/7 of the throw is {:.3f}), omega r {:.3f}, {:.3f} m up",
                                        speed, rolling, omega_r, b.pos.y)};
            }
            case Kind::box: {
                const float flat = std::max({std::abs(up_in_body.x), std::abs(up_in_body.y), std::abs(up_in_body.z)});
                const bool ok = v < 0.02f && w < 0.1f && flat >= std::cos(glm::radians(2.0f)) &&
                                std::abs(b.pos.y - kCubeHalf) <= 0.01f;
                return {ok, std::format("at rest on a face (a face is {:.4f} of flat, at most 1), still within {:.4f} m/s and {:.4f} rad/s, {:.4f} m up",
                                        flat, v, w, b.pos.y)};
            }
            case Kind::capsule: {
                const bool ok = v < 0.02f && std::abs(axis.y) < 0.02f && std::abs(b.pos.y - kCapsuleRadius) <= 0.01f;
                return {ok, std::format("lying on both ends (its axis is {:.4f} off flat), still within {:.4f} m/s, {:.4f} m up",
                                        axis.y, v, b.pos.y)};
            }
            case Kind::hull: {
                const bool ok = v < 0.02f && w < 0.1f && std::abs(up_in_body.y) >= std::cos(glm::radians(2.0f)) &&
                                std::abs(b.pos.y - kHullFoot) <= 0.01f;
                return {ok, std::format("landed flat ({:.4f} of upright), still within {:.4f} m/s and {:.4f} rad/s, {:.4f} m up",
                                        std::abs(up_in_body.y), v, w, b.pos.y)};
            }
        }
        return {false, "unknown shape"};
    }

    // Every piece's pose and motion, field by field: the bytes two runs must share.
    [[nodiscard]] std::vector<std::byte> bytes() const {
        std::vector<std::byte> out;
        const auto put = [&out](const float* f, std::size_t n) {
            const std::size_t at = out.size();
            out.resize(at + n * sizeof(float));
            std::memcpy(out.data() + at, f, n * sizeof(float));
        };
        for (std::size_t i = 0; i < refs_.size(); ++i) {
            const BodyState b = state(i);
            put(&b.pos.x, 3);
            put(&b.vel.x, 3);
            const float q[4] = {b.orient.w, b.orient.x, b.orient.y, b.orient.z};
            put(q, 4);
            put(&b.omega_body.x, 3);
        }
        return out;
    }

  private:
    BodiesSim() = default;

    std::optional<Simulation> sim_;
    std::vector<Piece> pieces_;
    std::vector<BodyRef> refs_;
    std::vector<std::array<float, kWindow>> speed_, spin_;  // the last kWindow steps, per piece
    uint64_t steps_ = 0;
    double carry_ms_ = 0.0;
};

}  // namespace spade::sandbox::bodies
