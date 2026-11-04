#pragma once

// contact_set_fakes.hpp -- fake contact-set builds for L10's plumbing tests
// (test_contact_sets.cpp, test_gpu_contact_sets.cpp). Physics' derivation
// (physics/contact_sets.cpp) is not Core's to test, and until it lands it
// builds nothing. So these tests swap static_contact's table build for a fake
// that writes physics/contact_sets.hpp's row layout from a model's mass alone:
// a model of 2 kg or more has a set, a lighter one has none.

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "physics/contact_sets.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace contact_set_fakes {

using spade::modules::ConfigBuildFn;
using spade::modules::ConfigTableDecl;
using spade::modules::ModuleSet;
using spade::physics::kContactSetRowFloats;
using spade::physics::kContactSetsCapacityRows;
using spade::physics::kContactSetsTable;

// The agreed layout from one point list per model: a range row per model,
// then the points; nothing at all when no model has a point.
inline void write_layout(const std::vector<std::vector<glm::vec3>>& sets, std::vector<float>& out) {
    bool any = false;
    for (const auto& s : sets) any = any || !s.empty();
    if (!any) return;
    uint32_t next = static_cast<uint32_t>(sets.size());
    for (const auto& s : sets) {
        const auto count = static_cast<uint32_t>(s.size());
        out.insert(out.end(), {s.empty() ? 0.0f : static_cast<float>(next), static_cast<float>(count), 0.0f, 0.0f});
        next += count;
    }
    for (const auto& s : sets) {
        for (const glm::vec3& p : s) out.insert(out.end(), {p.x, p.y, p.z, 0.0f});
    }
}

// Set A: a model of mass m >= 2 kg has the two points (m, 0, 0) and (0, m, 0).
inline void build_a(std::span<const spade::vehicles::ModelType> models, std::vector<float>& out) {
    std::vector<std::vector<glm::vec3>> sets;
    for (const auto& m : models) {
        const float k = m.body.mass;
        sets.push_back(k >= 2.0f ? std::vector<glm::vec3>{{k, 0.0f, 0.0f}, {0.0f, k, 0.0f}}
                                 : std::vector<glm::vec3>{});
    }
    write_layout(sets, out);
}

// Set B: set A with its second point moved, so the same models build a table
// that differs from A's in one float.
inline void build_b(std::span<const spade::vehicles::ModelType> models, std::vector<float>& out) {
    std::vector<std::vector<glm::vec3>> sets;
    for (const auto& m : models) {
        const float k = m.body.mass;
        sets.push_back(k >= 2.0f ? std::vector<glm::vec3>{{k, 0.0f, 0.0f}, {0.0f, k + 0.5f, 0.0f}}
                                 : std::vector<glm::vec3>{});
    }
    write_layout(sets, out);
}

// Set A, except that a model of 3 kg or more has the table's whole capacity
// in points, so registering one takes the table past it.
inline void build_huge(std::span<const spade::vehicles::ModelType> models, std::vector<float>& out) {
    std::vector<std::vector<glm::vec3>> sets;
    for (const auto& m : models) {
        const float k = m.body.mass;
        if (k >= 3.0f) {
            sets.emplace_back(kContactSetsCapacityRows, glm::vec3(k, 0.0f, 0.0f));
        } else {
            sets.push_back(k >= 2.0f ? std::vector<glm::vec3>{{k, 0.0f, 0.0f}, {0.0f, k, 0.0f}}
                                     : std::vector<glm::vec3>{});
        }
    }
    write_layout(sets, out);
}

// A table that breaks the layout: model 1's range runs past the last row.
inline void build_malformed(std::span<const spade::vehicles::ModelType> models, std::vector<float>& out) {
    if (models.empty() || models[0].body.mass < 2.0f) return;
    for (std::size_t i = 0; i < models.size(); ++i) {
        out.insert(out.end(), {i == 0 ? static_cast<float>(models.size()) : 0.0f, i == 0 ? 3.0f : 0.0f, 0.0f, 0.0f});
    }
    out.insert(out.end(), {1.0f, 0.0f, 0.0f, 0.0f});  // one point row, where three were promised
}

// No set for any model: what static_contact's table holds before Physics'
// derivation, and what master's standard set amounts to.
inline void build_none(std::span<const spade::vehicles::ModelType>, std::vector<float>&) {}

// static_contact's own declaration in the standard set.
[[nodiscard]] inline ConfigTableDecl standard_contact_table() {
    for (const spade::modules::ModuleDesc& m : spade::modules::standard_modules()) {
        if (m.name == "static_contact" && m.tables.size() == 1) return m.tables[0];
    }
    ADD_FAILURE() << "the standard set's static_contact declares no single table";
    return {};
}

// The standard set with static_contact's table built by `Build` instead: the
// same name, binding and capacity.
template <ConfigBuildFn Build>
[[nodiscard]] ModuleSet standard_set_with() {
    static const ConfigTableDecl table = [] {
        ConfigTableDecl t = standard_contact_table();
        t.build = Build;
        return t;
    }();
    ModuleSet set = spade::modules::standard_modules();
    for (spade::modules::ModuleDesc& m : set) {
        if (m.name == "static_contact") m.tables = std::span<const ConfigTableDecl>(&table, 1);
    }
    return set;
}

// A quadrotor of the given mass: 4 rotors, a drag body and an IMU.
[[nodiscard]] inline spade::vehicles::ModelType quad_of_mass(float mass) {
    spade::vehicles::QuadrotorParams p;
    p.mass = mass;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f) * mass;
    p.arm_length = 0.19f;
    for (spade::vehicles::RotorParams& rotor : p.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.13f;
        rotor.thrust_coeff = 1.2e-5f;
        rotor.torque_coeff = 1.9e-7f;
    }
    p.drag.mode = spade::physics::drag_mode::quadratic;
    p.drag.area = 0.05f;
    p.drag.coeffs = glm::vec3(1.6f, 0.0f, 0.0f);
    p.imu.rate_divider = 1;
    p.imu.sigma_a = 0.05f;
    p.imu.sigma_g = 0.01f;
    return spade::vehicles::make_quadrotor(p).value();
}

// One world, room for three quads, a 0.2 m contact sphere; a ground plane at
// y = 0 when `ground`.
[[nodiscard]] inline spade::WorldSetDesc world(bool ground) {
    spade::WorldBuilder b;
    b.name("contact_sets").environment(spade::Environment{}).capacities(spade::Capacities{4, 16, 4, 4});
    if (ground) b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    auto built = b.build();
    EXPECT_TRUE(built.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.contacts.proxy_radius = 0.2f;
    return spade::WorldSetDesc{{inst}};
}

// A vehicle pose at (x, y, 0), at rest, rotors stopped.
[[nodiscard]] inline spade::VehicleSpawn at(float x, float y) {
    spade::VehicleSpawn where;
    where.pos = glm::vec3(x, y, 0.0f);
    return where;
}

}  // namespace contact_set_fakes
