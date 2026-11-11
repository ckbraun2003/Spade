// rigid_contact_support.hpp -- the world, the module set and the cube scenario the L68 tests share (test_rigid_contact.cpp, the CPU half;
// test_gpu_rigid_contact.cpp, the device half). Header-only so the two translation units agree on the scenario to the bit.
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "physics/rigid_contact.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace spade::test::rigid {


namespace ph = spade::physics;
using spade::BodyRef;
using spade::BodyState;
using spade::Simulation;

inline constexpr float kG = 9.80665f;

struct WorldOpts {
    float wall_x = std::numeric_limits<float>::infinity();  // a wall at x >= wall_x when finite
    bool bar = false;                                        // a 0.1 x 0.1 m bar along x at height 0.5 (the library gate's frame section)
    float mu = 0.0f;                                         // the world's ContactParams
    float e = 0.0f;
    float proxy_radius = 0.0f;
};

[[nodiscard]] inline spade::WorldSetDesc make_world(const WorldOpts& o = {}) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::WorldBuilder b;
    b.name("rigid").environment(env).capacities(spade::Capacities{4, 12, 4, 1});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    if (std::isfinite(o.wall_x)) b.plane(glm::vec3(-1.0f, 0.0f, 0.0f), -o.wall_x).union_();
    if (o.bar) b.box(glm::vec3(0.5f, 0.05f, 0.05f), spade::SdfPose{.position = glm::vec3(0.0f, 0.5f, 0.0f)}).union_();
    const auto built = b.build();
    EXPECT_TRUE(built.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.seed = 7;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.contacts.restitution_e = o.e;
    inst.contacts.friction_mu = o.mu;
    inst.contacts.proxy_radius = o.proxy_radius;
    return spade::WorldSetDesc{{inst}};
}

[[nodiscard]] inline spade::modules::ModuleSet tumbling_set() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::rigid_contact_module());
    return set;
}

[[nodiscard]] inline Simulation make(const spade::modules::ModuleSet& set, const WorldOpts& o = {},
                              spade::compute::BackendKind kind = spade::compute::BackendKind::cpu) {
    auto sim = Simulation::create(make_world(o), 1'000'000, 1, spade::compute::BackendDesc{.kind = kind}, set);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    return std::move(*sim);
}

[[nodiscard]] inline BodyRef put(Simulation& sim, glm::vec3 pos, glm::vec3 vel, glm::quat orient, glm::vec3 inv_inertia, float mass = 1.0f,
                          glm::vec3 omega_body = glm::vec3(0.0f)) {
    spade::BodySpawn s;
    s.pos = pos;
    s.vel = vel;
    s.orient = orient;
    s.mass = mass;
    s.inv_inertia_diag = inv_inertia;
    s.omega_body = omega_body;
    const auto ref = sim.spawn(0, s);
    EXPECT_TRUE(ref.has_value()) << (ref ? "" : ref.error().context);
    EXPECT_TRUE(sim.flush_structural().has_value());
    return ref.value_or(BodyRef{});
}

[[nodiscard]] inline BodyState body(const Simulation& sim, BodyRef ref) {
    const auto row = sim.body(ref);
    EXPECT_TRUE(row.has_value());
    return row ? **row : BodyState{};
}

[[nodiscard]] inline bool shaped(Simulation& sim, BodyRef ref, const ph::ShapeDesc& d) {
    const spade::Result<void> r = sim.set_body_shape(ref, d);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().context);
    return r.has_value();
}

// A generic tilt: no body axis near the vertical, so a corner leads.
[[nodiscard]] inline glm::quat tilt() { return glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 0.4f, 0.2f))); }

// Centre height that puts the lowest of a cube's corners (half extent h) at `lowest` for orientation q.
[[nodiscard]] inline float centre_for_lowest(glm::quat q, float h, float lowest) {
    float low = 1e9f;
    for (int c = 0; c < 8; ++c) {
        const glm::vec3 corner((c & 1) ? h : -h, (c & 2) ? h : -h, (c & 4) ? h : -h);
        low = std::min(low, (q * corner).y);
    }
    return lowest - low;
}

[[nodiscard]] inline std::vector<glm::vec3> cube_corners(float h) {
    std::vector<glm::vec3> out;
    for (int c = 0; c < 8; ++c) out.emplace_back((c & 1) ? h : -h, (c & 2) ? h : -h, (c & 4) ? h : -h);
    return out;
}

inline constexpr float kCubeHalf = 0.1f;
[[nodiscard]] inline glm::vec3 cube_inv() { return ph::box_inv_inertia(1.0f, glm::vec3(kCubeHalf)); }

// The settled state of the tumbling cube: how still it is over the last half second, and which of its axes is up.
struct Settle {
    BodyState end{};
    float last_half_max_v = 0.0f;
    float last_half_max_w = 0.0f;
    float up_alignment = 0.0f;  // the largest |component| of world-up in the body frame: 1 when a face is flat
    float max_w_after_first_contact = 0.0f;
};

[[nodiscard]] inline Settle drop_cube(Simulation& sim, BodyRef ref, int steps) {
    Settle s;
    bool contact = false;
    for (int i = 0; i < steps; ++i) {
        EXPECT_TRUE(sim.step(1).has_value());
        const BodyState b = body(sim, ref);
        if (!contact && b.pos.y < 0.5f) contact = true;
        const float w = glm::length(b.orient * b.omega_body);
        if (contact && i < 1500) s.max_w_after_first_contact = std::max(s.max_w_after_first_contact, w);
        if (i >= steps - 500) {
            s.last_half_max_v = std::max(s.last_half_max_v, glm::length(b.vel));
            s.last_half_max_w = std::max(s.last_half_max_w, w);
        }
    }
    s.end = body(sim, ref);
    const glm::vec3 up_in_body = glm::conjugate(s.end.orient) * glm::vec3(0.0f, 1.0f, 0.0f);
    s.up_alignment = std::max({std::abs(up_in_body.x), std::abs(up_in_body.y), std::abs(up_in_body.z)});
    return s;
}

[[nodiscard]] inline ph::ShapeDesc cube(float e = 0.2f, float mu = 0.5f) {
    ph::ShapeDesc d = ph::box_shape(glm::vec3(kCubeHalf));
    d.restitution = e;
    d.friction = mu;
    return d;
}


// ---- the device-band scenario (docs/design/test-docs/2026-11-11-l68-device-band.md) ------------------------------------------------------

// The scenario: a 0.2 m cube (half extent 0.1, 1 kg, mu 0.5, e 0.3), lowest corner 1 m up at a generic tilt about (1, 2, 3)/sqrt(14), onto a
// plane floor, 1 kHz, 6 s. The tilt is the first of the record's list (20, 30, 40 degrees) whose nine-run CPU ensemble settles on one face.
inline constexpr float kScenarioTiltDeg = 20.0f;
inline constexpr int kScenarioSteps = 6000;
// The band constants. Floors now; the ensemble's measured spreads are printed beside them and the constants are filled in after the CPU pass.
inline constexpr float kLateralBand = 0.010f;  // m, device against CPU: max(0.010, 2 x the ensemble's largest pairwise horizontal spread)
inline constexpr float kYawBandDeg = 2.0f;     // degrees: max(2, 2 x the ensemble's largest pairwise yaw spread), up to 1 degree
inline constexpr int kImpactBand = 1;          // impacts: max(1, the ensemble's largest pairwise difference)
inline constexpr float kHeightBand = 1e-4f;    // m, device against CPU
inline constexpr float kHeightLow = -0.002f;   // m, each run, about the half extent (the static contact band)
inline constexpr float kHeightHigh = 0.005f;
inline constexpr float kTiltBandDeg = 1.0f;    // each run on its own

struct Outcome {
    bool settled = false;
    int face = -1;            // 0..5: the body axis closest to world up (+x, +y, +z, -x, -y, -z)
    float tilt_deg = 0.0f;    // that axis from world up
    float centre_above = 0.0f;  // centre height minus the half extent
    glm::vec2 lateral{0.0f};  // x, z
    float yaw_deg = 0.0f;     // heading of the next body axis, about the vertical
    int impacts = 0;          // steps where the centre's vertical velocity jumped up by more than 0.1 m/s (a real impact, not resting)
    BodyState end{};
};

[[nodiscard]] inline Outcome run_cube(spade::compute::BackendKind kind, float tilt_deg, float dh = 0.0f, float dtilt = 0.0f,
                                      int steps = kScenarioSteps) {
    Outcome out;
    Simulation sim = make(tumbling_set(), WorldOpts{}, kind);
    const glm::vec3 axis = glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));
    const glm::quat q = glm::angleAxis(glm::radians(tilt_deg) + dtilt, axis);
    const BodyRef ref = put(sim, glm::vec3(0.0f, centre_for_lowest(q, kCubeHalf, 1.0f) + dh, 0.0f), glm::vec3(0.0f), q, cube_inv());
    ph::ShapeDesc d = ph::box_shape(glm::vec3(kCubeHalf));
    d.restitution = 0.3f;
    d.friction = 0.5f;
    if (!shaped(sim, ref, d)) return out;
    float max_v = 0.0f;
    float max_w = 0.0f;
    float vy_prev = 0.0f;
    for (int i = 0; i < steps; ++i) {
        if (!sim.step(1).has_value()) return out;
        const BodyState b = body(sim, ref);
        if (b.vel.y - vy_prev > 0.1f) ++out.impacts;
        vy_prev = b.vel.y;
        if (i >= steps - 500) {
            max_v = std::max(max_v, glm::length(b.vel));
            max_w = std::max(max_w, glm::length(b.orient * b.omega_body));
        }
    }
    out.end = body(sim, ref);
    out.settled = max_v < 0.02f && max_w < 0.1f;
    const glm::vec3 up_in_body = glm::conjugate(out.end.orient) * glm::vec3(0.0f, 1.0f, 0.0f);
    const float comp[3] = {up_in_body.x, up_in_body.y, up_in_body.z};
    int axis_index = 0;
    for (int i = 1; i < 3; ++i) {
        if (std::abs(comp[i]) > std::abs(comp[axis_index])) axis_index = i;
    }
    out.face = axis_index + (comp[axis_index] < 0.0f ? 3 : 0);
    out.tilt_deg = glm::degrees(std::acos(std::min(1.0f, std::abs(comp[axis_index]))));
    out.centre_above = out.end.pos.y - kCubeHalf;
    out.lateral = glm::vec2(out.end.pos.x, out.end.pos.z);
    glm::vec3 in_plane(0.0f);
    in_plane[(axis_index + 1) % 3] = 1.0f;
    const glm::vec3 heading = out.end.orient * in_plane;
    out.yaw_deg = glm::degrees(std::atan2(heading.z, heading.x));
    return out;
}

// The difference of two headings, wrapped to [-180, 180].
[[nodiscard]] inline float heading_difference(float a_deg, float b_deg) {
    float d = std::fmod(a_deg - b_deg, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

}  // namespace spade::test::rigid
