// test_part_pose.cpp -- the posing rule for a model's parts (CORE-7; the
// parts and meshes plan, Task A2; vehicles/part_pose.hpp). A body part is
// posed by the body and then its mount, a rotor part by the body, its hub and
// the azimuth, and the wall velocity is v + omega x r plus the rotor's spin.
// posed_program() gives the wall parts as one world-frame SdfProgram, and
// sample_parts() samples every part in its own frame, the smallest distance
// winning and a tie going to the lower part index (CORE-8 section 5.2).
// Expectations are worked by hand from the pose, not read from the code
// (TD-4).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "state/layout.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace {

using spade::vehicles::ModelType;
using spade::vehicles::PartAttach;
using spade::vehicles::PartDesc;
using spade::vehicles::PartFilter;
using spade::vehicles::PartSample;
using spade::vehicles::PartSurface;
using spade::vehicles::PosedPart;
using spade::vehicles::PosedParts;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
const glm::quat kIdentity(1.0f, 0.0f, 0.0f, 0.0f);

template <class T>
[[nodiscard]] bool same_bytes(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// The postfix pieces of a part's shape.
[[nodiscard]] spade::SdfNode leaf(spade::SdfPrim kind, glm::vec4 params, uint32_t transform = 0) {
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.transform = transform;
    n.params = params;
    return n;
}

[[nodiscard]] spade::SdfNode op(spade::SdfOp which) {
    spade::SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

[[nodiscard]] spade::SdfProgram program(std::vector<spade::SdfNode> nodes) {
    spade::SdfProgram p;
    p.transforms.push_back(spade::SdfTransform{});
    p.nodes = std::move(nodes);
    return p;
}

[[nodiscard]] spade::SdfProgram sphere(float radius) {
    return program({leaf(spade::SdfPrim::sphere, glm::vec4(radius, 0.0f, 0.0f, 0.0f))});
}
[[nodiscard]] spade::SdfProgram box(glm::vec3 half) {
    return program({leaf(spade::SdfPrim::box, glm::vec4(half, 0.0f))});
}

// A wall part on the body, declaring a 1 mm smallest feature.
[[nodiscard]] PartDesc part(std::string name, spade::SdfProgram shape, glm::vec3 mount_pos = glm::vec3(0.0f),
                            glm::quat mount_orient = kIdentity) {
    return PartDesc{.name = std::move(name),
                    .role = "test",
                    .attach = PartAttach::body,
                    .rotor = 0,
                    .mount_pos = mount_pos,
                    .mount_orient = mount_orient,
                    .shape = std::move(shape),
                    .surface = PartSurface{.kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.001f};
}

[[nodiscard]] PartDesc on_rotor(PartDesc p, uint32_t rotor) {
    p.attach = PartAttach::rotor;
    p.rotor = rotor;
    return p;
}

[[nodiscard]] PartDesc made_porous(PartDesc p) {
    p.surface = PartSurface{.kind = PartSurface::Kind::porous, .inv_permeability = 1.0e6f, .inertial_coeff = 20.0f};
    return p;
}

// bench_sim.cpp's 1 kg airframe (four rotors), with these parts appended.
[[nodiscard]] ModelType quad_with(std::vector<PartDesc> parts) {
    spade::vehicles::QuadrotorParams p;
    p.name = "pose_quad";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    p.rotor_height = 0.02f;
    p.proxy_radius = 0.2f;
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
    ModelType m = spade::vehicles::make_quadrotor(p).value();
    m.parts = std::move(parts);
    return m;
}

[[nodiscard]] spade::BodyState body_at(glm::vec3 pos, glm::quat orient, glm::vec3 vel = glm::vec3(0.0f),
                                       glm::vec3 omega_body = glm::vec3(0.0f)) {
    spade::BodyState b{};
    b.pos = pos;
    b.orient = orient;
    b.vel = vel;
    b.omega_body = omega_body;
    return b;
}

// A drone of wall parts in every form the rule has: a hull, a turned arm, a
// motor cylinder, a part that is a CSG of two posed primitives, and a blade on
// rotor 0. Tilted, moving and turning.
[[nodiscard]] ModelType walled_drone() {
    spade::SdfProgram cut = program({leaf(spade::SdfPrim::box, glm::vec4(0.04f, 0.04f, 0.04f, 0.0f)),
                                     leaf(spade::SdfPrim::sphere, glm::vec4(0.03f, 0.0f, 0.0f, 0.0f), 1),
                                     op(spade::SdfOp::subtract)});
    cut.transforms.push_back(spade::transform_of(spade::SdfPose{.position = glm::vec3(0.02f, 0.01f, 0.0f),
                                                                .rotation = kIdentity,
                                                                .scale = 1.0f})
                                 .value());
    return quad_with({
        part("hull", box(glm::vec3(0.08f, 0.03f, 0.05f))),
        part("arm0", box(glm::vec3(0.09f, 0.008f, 0.008f)), glm::vec3(0.12f, 0.0f, 0.12f),
             glm::angleAxis(-glm::quarter_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f))),
        part("motor0",
             program({leaf(spade::SdfPrim::cylinder, glm::vec4(0.014f, 0.012f, 0.0f, 0.0f))}),
             glm::vec3(0.18f, 0.02f, -0.18f)),
        part("cut", std::move(cut), glm::vec3(-0.1f, 0.0f, 0.0f),
             glm::angleAxis(0.5f, glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f)))),
        on_rotor(part("blade", box(glm::vec3(0.06f, 0.001f, 0.008f)), glm::vec3(0.065f, 0.0f, 0.0f)), 0),
    });
}

[[nodiscard]] spade::BodyState walled_drone_body() {
    return body_at(glm::vec3(0.5f, 1.5f, -0.25f), glm::angleAxis(0.6f, glm::normalize(glm::vec3(0.2f, 1.0f, -0.3f))),
                   glm::vec3(2.0f, 0.0f, 1.0f), glm::vec3(0.3f, -1.0f, 2.0f));
}

constexpr std::array<float, 4> kDroneOmega{900.0f, 910.0f, 920.0f, 930.0f};
constexpr std::array<float, 4> kDronePsi{0.7f, 1.1f, -2.0f, 3.0f};

}  // namespace

TEST(PartPose, ARigidPartIsPosedByTheBodyThenItsMount) {
    // A 0.01 m sphere mounted 0.1 m along body +X, on a body at (0, 5, 0) turned
    // +90 degrees about +Y, so body +X is world -Z: its centre is (0, 5, -0.1).
    const ModelType m = quad_with({part("pod", sphere(0.01f), glm::vec3(0.1f, 0.0f, 0.0f))});
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b =
        body_at(glm::vec3(0.0f, 5.0f, 0.0f), glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f)));
    const auto posed = spade::vehicles::posed_parts(m, b, {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    const auto program = spade::vehicles::posed_program(m, *posed);
    ASSERT_TRUE(program.has_value()) << program.error().context;
    EXPECT_NEAR(spade::eval(*program, glm::vec3(0.0f, 5.0f, -0.1f)), -0.01f, 1e-6f);
    EXPECT_NEAR(spade::eval(*program, glm::vec3(0.0f, 5.0f, -0.16f)), 0.05f, 1e-6f);
    // sqrt(0.16^2 + 0.1^2) - 0.01 = sqrt(0.0356) - 0.01; where body +X was, it would be 0.05.
    EXPECT_NEAR(spade::eval(*program, glm::vec3(0.16f, 5.0f, 0.0f)), 0.1786796f, 1e-6f) << "not where body +X was";

    // The sampler: the same distance, the outward normal -Z, the one wall part.
    const PartSample s = spade::vehicles::sample_parts(m, *posed, glm::vec3(0.0f, 5.0f, -0.16f));
    EXPECT_NEAR(s.distance, 0.05f, 1e-6f);
    EXPECT_NEAR(s.gradient.x, 0.0f, 1e-6f);
    EXPECT_NEAR(s.gradient.y, 0.0f, 1e-6f);
    EXPECT_NEAR(s.gradient.z, -1.0f, 1e-6f);
    EXPECT_EQ(s.owner, 0u);
    EXPECT_EQ(s.surface, PartSurface::Kind::wall);
}

TEST(PartPose, TheWallVelocityIsTheBodysPlusTheRotorsSpin) {
    // Body at the origin, unrotated: v = (1, 0, 0), omega = (0, 2, 0). Rotor 0's hub is
    // at (0.1, 0, 0), thrust +Y, spin +1, omega_0 = 100 rad/s. At x = hub + (0, 0, 0.05):
    //   omega x (x - com) = (0, 2, 0) x (0.1, 0, 0.05) = (0.1, 0, -0.2)
    //   spin x (x - hub)  = (0, 100, 0) x (0, 0, 0.05) = (5, 0, 0)
    ModelType m = quad_with({on_rotor(part("blade", sphere(0.01f)), 0)});
    m.rotors[0].local_pos = glm::vec3(0.1f, 0.0f, 0.0f);
    m.rotors[0].local_orient = kIdentity;
    m.rotors[0].spin_dir = 1.0f;
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b =
        body_at(glm::vec3(0.0f), kIdentity, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 2.0f, 0.0f));
    const std::array<float, 4> omega{100.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 4> psi{0.0f, 0.0f, 0.0f, 0.0f};
    const auto posed = spade::vehicles::posed_parts(m, b, omega, psi, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    ASSERT_EQ(posed->parts.size(), 1u);
    const glm::vec3 x(0.1f, 0.0f, 0.05f);
    const glm::vec3 v = spade::vehicles::part_wall_velocity(*posed, 0, x);
    EXPECT_NEAR(v.x, 6.1f, 1e-5f);
    EXPECT_NEAR(v.y, 0.0f, 1e-6f);
    EXPECT_NEAR(v.z, -0.2f, 1e-6f);

    // The sampler reports the owner's wall velocity, the same bits.
    const PartSample s = spade::vehicles::sample_parts(m, *posed, x);
    EXPECT_EQ(s.owner, 0u);
    EXPECT_NEAR(s.distance, 0.04f, 1e-6f);
    EXPECT_TRUE(same_bytes(s.velocity, v));
}

TEST(PartPose, ARotorPartTurnsWithItsAzimuthAboutTheHubAxis) {
    // The body at (1, 2, 3), unrotated; rotor 2's hub at body (0.2, 0.03, -0.1),
    // thrust +Y, so the hub is at world (1.2, 2.03, 2.9). A 0.05 x 0.002 x 0.01 m
    // box blade (half extents 0.025, 0.001, 0.005) is mounted 0.06 m along hub +X.
    //   psi = 0:                  centre hub + (0.06, 0, 0);
    //   psi = pi/2, spin +1:      R_y(+90 deg) takes +X to -Z: hub + (0, 0, -0.06),
    //                             the blade's long axis now along world Z;
    //   psi = pi/2, spin -1:      hub + (0, 0, +0.06);
    //   psi = pi/2, spin 0:       never turns: hub + (0.06, 0, 0).
    const glm::vec3 hub(1.2f, 2.03f, 2.9f);
    const auto centre_at = [&](float spin_dir, float psi2) -> glm::vec3 {
        ModelType m = quad_with({on_rotor(part("blade", box(glm::vec3(0.025f, 0.001f, 0.005f)),
                                                glm::vec3(0.06f, 0.0f, 0.0f)),
                                           2)});
        m.rotors[2].local_pos = glm::vec3(0.2f, 0.03f, -0.1f);
        m.rotors[2].local_orient = kIdentity;
        m.rotors[2].spin_dir = spin_dir;
        EXPECT_TRUE(m.validate().has_value());
        const std::array<float, 4> omega{0.0f, 0.0f, 0.0f, 0.0f};
        const std::array<float, 4> psi{0.0f, 0.0f, psi2, 0.0f};
        const auto posed = spade::vehicles::posed_parts(m, body_at(glm::vec3(1.0f, 2.0f, 3.0f), kIdentity), omega,
                                                        psi, PartFilter::all);
        EXPECT_TRUE(posed.has_value());
        if (!posed || posed->parts.size() != 1u) {
            ADD_FAILURE() << "spin " << spin_dir << ", psi " << psi2 << ": not one posed part";
            return glm::vec3(kNaN);
        }
        // The program agrees: the centre is the blade's deepest point (-0.001).
        const auto program = spade::vehicles::posed_program(m, *posed);
        EXPECT_TRUE(program.has_value());
        if (program) {
            EXPECT_NEAR(spade::eval(*program, posed->parts[0].origin), -0.001f, 1e-6f);
        }
        return posed->parts[0].origin;
    };
    const auto expect_at = [](glm::vec3 got, glm::vec3 want, const char* why) {
        EXPECT_NEAR(got.x, want.x, 1e-6f) << why;
        EXPECT_NEAR(got.y, want.y, 1e-6f) << why;
        EXPECT_NEAR(got.z, want.z, 1e-6f) << why;
    };
    expect_at(centre_at(1.0f, 0.0f), hub + glm::vec3(0.06f, 0.0f, 0.0f), "psi 0");
    expect_at(centre_at(1.0f, glm::half_pi<float>()), hub + glm::vec3(0.0f, 0.0f, -0.06f), "psi pi/2, spin +1");
    expect_at(centre_at(-1.0f, glm::half_pi<float>()), hub + glm::vec3(0.0f, 0.0f, 0.06f), "psi pi/2, spin -1");
    expect_at(centre_at(0.0f, glm::half_pi<float>()), hub + glm::vec3(0.06f, 0.0f, 0.0f), "psi pi/2, spin 0");

    // The shape turned too: at psi = pi/2, spin +1, 0.035 m past the centre along
    // world -Z is 0.01 m beyond the blade's 0.025 m half length.
    ModelType m = quad_with(
        {on_rotor(part("blade", box(glm::vec3(0.025f, 0.001f, 0.005f)), glm::vec3(0.06f, 0.0f, 0.0f)), 2)});
    m.rotors[2].local_pos = glm::vec3(0.2f, 0.03f, -0.1f);
    m.rotors[2].local_orient = kIdentity;
    m.rotors[2].spin_dir = 1.0f;
    const std::array<float, 4> omega{0.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 4> psi{0.0f, 0.0f, glm::half_pi<float>(), 0.0f};
    const auto posed =
        spade::vehicles::posed_parts(m, body_at(glm::vec3(1.0f, 2.0f, 3.0f), kIdentity), omega, psi, PartFilter::all);
    ASSERT_TRUE(posed.has_value());
    const auto program = spade::vehicles::posed_program(m, *posed);
    ASSERT_TRUE(program.has_value());
    EXPECT_NEAR(spade::eval(*program, hub + glm::vec3(0.0f, 0.0f, -0.095f)), 0.01f, 1e-6f);
}

TEST(PartPose, ARigidPartPosesAsTheBuilderPosesAPrimitive) {
    const ModelType m = quad_with({part("pod", sphere(0.02f), glm::vec3(0.1f, -0.05f, 0.02f),
                                        glm::angleAxis(0.3f, glm::normalize(glm::vec3(0.0f, 1.0f, 1.0f))))});
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b =
        body_at(glm::vec3(0.3f, 1.2f, -0.7f), glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))));
    const auto posed = spade::vehicles::posed_parts(m, b, {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    ASSERT_EQ(posed->parts.size(), 1u);
    const PosedPart& pp = posed->parts[0];

    // WorldBuilder places the same sphere at the part's pose X.
    const auto world = spade::WorldBuilder()
                           .name("x")
                           .environment(spade::Environment{})
                           .capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1})
                           .sphere(0.02f, spade::SdfPose{.position = pp.origin, .rotation = pp.orient, .scale = 1.0f})
                           .build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->sdf.transforms.size(), 2u);
    ASSERT_EQ(world->sdf.nodes.size(), 1u);
    EXPECT_TRUE(same_bytes(pp.world_to_part, world->sdf.transforms[1]));

    const auto program = spade::vehicles::posed_program(m, *posed);
    ASSERT_TRUE(program.has_value()) << program.error().context;
    ASSERT_EQ(program->transforms.size(), 2u);
    ASSERT_EQ(program->nodes.size(), 1u);
    EXPECT_TRUE(same_bytes(program->transforms[0], world->sdf.transforms[0]));
    EXPECT_TRUE(same_bytes(program->transforms[1], world->sdf.transforms[1]));
    EXPECT_TRUE(same_bytes(program->nodes[0], world->sdf.nodes[0]));
    EXPECT_TRUE(program->node_materials.empty());
}

TEST(PartPose, RigidOnlyDropsRotorPartsAndTheProgramHoldsOnlyWalls) {
    // A body box (wall), an arm (porous) at body (0.2, 0, 0), and a blade on
    // rotor 1, whose hub is at body (0, 0.05, 0.3). The body is at the origin,
    // unrotated, and psi = 0.
    ModelType m = quad_with({
        part("body", box(glm::vec3(0.05f, 0.05f, 0.05f))),
        made_porous(part("arm", box(glm::vec3(0.08f, 0.01f, 0.01f)), glm::vec3(0.2f, 0.0f, 0.0f))),
        on_rotor(part("blade", box(glm::vec3(0.025f, 0.001f, 0.005f)), glm::vec3(0.06f, 0.0f, 0.0f)), 1),
    });
    m.rotors[1].local_pos = glm::vec3(0.0f, 0.05f, 0.3f);
    m.rotors[1].local_orient = kIdentity;
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b = body_at(glm::vec3(0.0f), kIdentity);
    const std::array<float, 4> omega{0.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 4> psi{0.0f, 0.0f, 0.0f, 0.0f};
    const glm::vec3 body_centre(0.0f);
    const glm::vec3 arm_centre(0.2f, 0.0f, 0.0f);
    const glm::vec3 blade_centre(0.06f, 0.05f, 0.3f);

    const auto rigid = spade::vehicles::posed_parts(m, b, {}, {}, PartFilter::rigid_only);
    ASSERT_TRUE(rigid.has_value()) << rigid.error().context;
    ASSERT_EQ(rigid->parts.size(), 2u);
    EXPECT_EQ(rigid->parts[0].part, 0u);
    EXPECT_EQ(rigid->parts[1].part, 1u);
    const auto every = spade::vehicles::posed_parts(m, b, omega, psi, PartFilter::all);
    ASSERT_TRUE(every.has_value()) << every.error().context;
    ASSERT_EQ(every->parts.size(), 3u);
    EXPECT_EQ(every->parts[2].part, 2u);

    // rigid_only: the program is the body box alone; the porous arm is not a wall.
    const auto rigid_program = spade::vehicles::posed_program(m, *rigid);
    ASSERT_TRUE(rigid_program.has_value()) << rigid_program.error().context;
    EXPECT_EQ(rigid_program->nodes.size(), 1u);
    EXPECT_NEAR(spade::eval(*rigid_program, body_centre), -0.05f, 1e-6f);
    EXPECT_NEAR(spade::eval(*rigid_program, arm_centre), 0.15f, 1e-6f);

    // all: the body and the blade, joined by one union.
    const auto all_program = spade::vehicles::posed_program(m, *every);
    ASSERT_TRUE(all_program.has_value()) << all_program.error().context;
    ASSERT_EQ(all_program->nodes.size(), 3u);
    EXPECT_EQ(all_program->nodes[2].op, static_cast<uint32_t>(spade::SdfOp::union_));
    EXPECT_NEAR(spade::eval(*all_program, blade_centre), -0.001f, 1e-6f);
    EXPECT_NEAR(spade::eval(*all_program, arm_centre), 0.15f, 1e-6f);

    // The sampler sees the porous arm, and reports it, inside it, with its kind.
    for (const PosedParts* posed : {&*rigid, &*every}) {
        const PartSample in_arm = spade::vehicles::sample_parts(m, *posed, arm_centre);
        EXPECT_EQ(in_arm.owner, 1u);
        EXPECT_EQ(in_arm.surface, PartSurface::Kind::porous);
        EXPECT_NEAR(in_arm.distance, -0.01f, 1e-6f);
        const PartSample in_body = spade::vehicles::sample_parts(m, *posed, body_centre);
        EXPECT_EQ(in_body.owner, 0u);
        EXPECT_EQ(in_body.surface, PartSurface::Kind::wall);
    }
    const PartSample in_blade = spade::vehicles::sample_parts(m, *every, blade_centre);
    EXPECT_EQ(in_blade.owner, 2u);
    EXPECT_NEAR(in_blade.distance, -0.001f, 1e-6f);
}

TEST(PartPose, SampleAgreesWithTheProgramAndATieGoesToTheLowerPart) {
    const ModelType m = walled_drone();
    ASSERT_TRUE(m.validate().has_value()) << m.validate().error().context;
    const spade::BodyState b = walled_drone_body();
    const auto posed = spade::vehicles::posed_parts(m, b, kDroneOmega, kDronePsi, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    ASSERT_EQ(posed->parts.size(), 5u);
    const auto program = spade::vehicles::posed_program(m, *posed);
    ASSERT_TRUE(program.has_value()) << program.error().context;
    ASSERT_FALSE(program->nodes.empty());

    // 1000 seeded points in a 1 m cube about the body.
    spade::rng::Stream rng{.state = 0x5EED0A2ULL, .cached_gauss = 0.0f, .has_cached = 0u};
    int disagreements = 0;
    for (int n = 0; n < 1000; ++n) {
        const float u = rng.next_float();
        const float v = rng.next_float();
        const float w = rng.next_float();
        const glm::vec3 x = b.pos + (glm::vec3(u, v, w) * 2.0f - 1.0f) * 0.5f;
        const PartSample s = spade::vehicles::sample_parts(m, *posed, x);
        const float d = spade::eval(*program, x);
        if (!(std::fabs(s.distance - d) <= 1e-6f) || s.owner == spade::vehicles::kNoPart) {
            if (++disagreements <= 5) {
                ADD_FAILURE() << "point " << n << ": sample_parts " << s.distance << " (owner " << s.owner
                              << "), eval " << d;
            }
        }
    }
    EXPECT_EQ(disagreements, 0);

    // Two identical parts tie everywhere: the lower index owns every point,
    // whatever its surface (a porous part 0 still wins against a wall part 1).
    for (const bool porous_first : {false, true}) {
        PartDesc first = part("a", sphere(0.03f), glm::vec3(0.05f, 0.0f, 0.0f));
        if (porous_first) first = made_porous(std::move(first));
        const ModelType twins = quad_with({first, part("b", sphere(0.03f), glm::vec3(0.05f, 0.0f, 0.0f))});
        ASSERT_TRUE(twins.validate().has_value());
        const auto pair = spade::vehicles::posed_parts(twins, b, {}, {}, PartFilter::all);
        ASSERT_TRUE(pair.has_value()) << pair.error().context;
        for (const glm::vec3 offset : {glm::vec3(0.0f), glm::vec3(0.2f, -0.1f, 0.05f), glm::vec3(0.0f, 0.03f, 0.0f)}) {
            const PartSample s = spade::vehicles::sample_parts(twins, *pair, b.pos + offset);
            EXPECT_EQ(s.owner, 0u) << "porous first: " << porous_first;
            EXPECT_EQ(s.surface, porous_first ? PartSurface::Kind::porous : PartSurface::Kind::wall);
        }
    }
}

TEST(PartPose, BadInputsAreRefusedByName) {
    ModelType m = quad_with({
        part("hull", box(glm::vec3(0.05f, 0.05f, 0.05f))),
        on_rotor(part("blade", box(glm::vec3(0.025f, 0.001f, 0.005f)), glm::vec3(0.06f, 0.0f, 0.0f)), 1),
    });
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b = body_at(glm::vec3(0.0f), kIdentity);
    const std::array<float, 4> omega{100.0f, 100.0f, 100.0f, 100.0f};
    const std::array<float, 4> psi{0.0f, 0.0f, 0.0f, 0.0f};
    const std::array<float, 3> psi3{0.0f, 0.0f, 0.0f};
    const std::array<float, 2> omega2{100.0f, 100.0f};
    std::array<float, 4> psi_nan = psi;
    psi_nan[2] = kNaN;
    spade::BodyState no_turn = b;
    no_turn.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    spade::BodyState nan_vel = b;
    nan_vel.vel.y = kNaN;
    ModelType rotor9 = m;
    rotor9.parts[1].rotor = 9;  // a model that does not validate

    struct Case {
        const char* why;
        spade::Result<PosedParts> got;
        std::vector<const char*> needles;
    };
    std::vector<Case> cases;
    cases.push_back(Case{.why = "psi of length 3",
                         .got = spade::vehicles::posed_parts(m, b, omega, psi3, PartFilter::all),
                         .needles = {"psi", "3 entries", "4 rotors"}});
    cases.push_back(Case{.why = "psi empty while a rotor part is posed",
                         .got = spade::vehicles::posed_parts(m, b, omega, {}, PartFilter::all),
                         .needles = {"psi", "part 1 'blade'", "rotor 1"}});
    cases.push_back(Case{.why = "rotor_omega of length 2",
                         .got = spade::vehicles::posed_parts(m, b, omega2, psi, PartFilter::all),
                         .needles = {"rotor_omega", "2 entries"}});
    cases.push_back(Case{.why = "a NaN psi",
                         .got = spade::vehicles::posed_parts(m, b, omega, psi_nan, PartFilter::all),
                         .needles = {"psi[2]", "rotor 2"}});
    cases.push_back(Case{.why = "a zero body orientation",
                         .got = spade::vehicles::posed_parts(m, no_turn, omega, psi, PartFilter::all),
                         .needles = {"orientation"}});
    cases.push_back(Case{.why = "a NaN body velocity",
                         .got = spade::vehicles::posed_parts(m, nan_vel, omega, psi, PartFilter::all),
                         .needles = {"vel"}});
    cases.push_back(Case{.why = "an unknown filter",
                         .got = spade::vehicles::posed_parts(m, b, omega, psi, static_cast<PartFilter>(7)),
                         .needles = {"filter", "7"}});
    cases.push_back(Case{.why = "a rotor out of range",
                         .got = spade::vehicles::posed_parts(rotor9, b, omega, psi, PartFilter::all),
                         .needles = {"part 1 'blade'", "rotor 9"}});
    for (const Case& c : cases) {
        ASSERT_FALSE(c.got.has_value()) << c.why;
        EXPECT_EQ(c.got.error().code, spade::Code::invalid_argument) << c.why;
        EXPECT_NE(c.got.error().context.find("model 'pose_quad'"), std::string::npos) << c.got.error().context;
        for (const char* needle : c.needles) {
            EXPECT_NE(c.got.error().context.find(needle), std::string::npos)
                << c.why << ": no '" << needle << "' in: " << c.got.error().context;
        }
    }

    // Empty spans are fine while no rotor part is posed.
    EXPECT_TRUE(spade::vehicles::posed_parts(m, b, {}, {}, PartFilter::rigid_only).has_value());

    // posed_program: a posed part the model does not have.
    const auto posed = spade::vehicles::posed_parts(m, b, omega, psi, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    ModelType fewer = m;
    fewer.parts.pop_back();
    const auto stranger = spade::vehicles::posed_program(fewer, *posed);
    ASSERT_FALSE(stranger.has_value());
    EXPECT_EQ(stranger.error().code, spade::Code::invalid_argument);
    EXPECT_NE(stranger.error().context.find("part 1"), std::string::npos) << stranger.error().context;

    // posed_program: two parts of depth kMaxSdfDepth each. The first fits; the
    // second, joined above it, needs one slot more.
    std::vector<spade::SdfNode> deep;
    for (uint32_t i = 0; i < spade::kMaxSdfDepth; ++i) {
        deep.push_back(leaf(spade::SdfPrim::sphere, glm::vec4(0.01f + 0.001f * static_cast<float>(i), 0.0f, 0.0f, 0.0f)));
    }
    for (uint32_t i = 1; i < spade::kMaxSdfDepth; ++i) deep.push_back(op(spade::SdfOp::union_));
    const ModelType deep_pair = quad_with({part("deep0", program(deep)), part("deep1", program(deep))});
    ASSERT_TRUE(deep_pair.validate().has_value()) << deep_pair.validate().error().context;
    const auto deep_posed = spade::vehicles::posed_parts(deep_pair, b, {}, {}, PartFilter::all);
    ASSERT_TRUE(deep_posed.has_value()) << deep_posed.error().context;
    const auto too_deep = spade::vehicles::posed_program(deep_pair, *deep_posed);
    ASSERT_FALSE(too_deep.has_value());
    EXPECT_EQ(too_deep.error().code, spade::Code::capacity_exceeded);
    EXPECT_NE(too_deep.error().context.find("part 1 'deep1'"), std::string::npos) << too_deep.error().context;
}

TEST(PartPose, PosingIsDeterministic) {
    const ModelType m = walled_drone();
    ASSERT_TRUE(m.validate().has_value());
    const spade::BodyState b = walled_drone_body();
    const auto first = spade::vehicles::posed_parts(m, b, kDroneOmega, kDronePsi, PartFilter::all);
    const auto second = spade::vehicles::posed_parts(m, b, kDroneOmega, kDronePsi, PartFilter::all);
    ASSERT_TRUE(first.has_value() && second.has_value());
    ASSERT_EQ(first->parts.size(), 5u);
    ASSERT_EQ(second->parts.size(), first->parts.size());
    EXPECT_TRUE(same_bytes(first->com, second->com));
    EXPECT_TRUE(same_bytes(first->vel, second->vel));
    EXPECT_TRUE(same_bytes(first->omega, second->omega));
    for (std::size_t i = 0; i < first->parts.size(); ++i) {
        const PosedPart& p = first->parts[i];
        const PosedPart& q = second->parts[i];
        EXPECT_EQ(p.part, q.part);
        EXPECT_TRUE(same_bytes(p.world_to_part, q.world_to_part)) << i;
        EXPECT_TRUE(same_bytes(p.origin, q.origin)) << i;
        EXPECT_TRUE(same_bytes(p.orient, q.orient)) << i;
        EXPECT_TRUE(same_bytes(p.hub, q.hub)) << i;
        EXPECT_TRUE(same_bytes(p.spin, q.spin)) << i;
    }
    const auto a = spade::vehicles::posed_program(m, *first);
    const auto c = spade::vehicles::posed_program(m, *second);
    ASSERT_TRUE(a.has_value() && c.has_value());
    ASSERT_EQ(a->nodes.size(), c->nodes.size());
    ASSERT_EQ(a->transforms.size(), c->transforms.size());
    ASSERT_FALSE(a->nodes.empty());
    EXPECT_EQ(std::memcmp(a->nodes.data(), c->nodes.data(), a->nodes.size() * sizeof(spade::SdfNode)), 0);
    EXPECT_EQ(std::memcmp(a->transforms.data(), c->transforms.data(), a->transforms.size() * sizeof(spade::SdfTransform)),
              0);
    const glm::vec3 x = b.pos + glm::vec3(0.07f, -0.02f, 0.11f);
    const PartSample s1 = spade::vehicles::sample_parts(m, *first, x);
    const PartSample s2 = spade::vehicles::sample_parts(m, *second, x);
    EXPECT_TRUE(same_bytes(s1.distance, s2.distance));
    EXPECT_TRUE(same_bytes(s1.gradient, s2.gradient));
    EXPECT_TRUE(same_bytes(s1.velocity, s2.velocity));
    EXPECT_EQ(s1.owner, s2.owner);
}

// The layering the posing rule depends on (the plan's Global Constraints):
// vehicles/ includes world/ and state/, never scene/, sim/ or render/; world/
// includes nothing from vehicles/, scene/, sim/ or render/. Green at red: a pin.
TEST(PartPose, VehiclesAndWorldIncludeNothingAboveThem) {
    namespace fs = std::filesystem;
    const fs::path engine(SPADE_ENGINE_DIR);
    struct Rule {
        const char* dir;
        std::vector<std::string> forbidden;
    };
    const Rule rules[] = {
        Rule{.dir = "vehicles", .forbidden = {"#include \"scene/", "#include \"sim/", "#include \"render/"}},
        Rule{.dir = "world",
             .forbidden = {"#include \"vehicles/", "#include \"scene/", "#include \"sim/", "#include \"render/"}},
    };
    for (const Rule& rule : rules) {
        const fs::path dir = engine / rule.dir;
        ASSERT_TRUE(fs::is_directory(dir)) << dir.string();
        int scanned = 0;
        for (const fs::directory_entry& entry : fs::recursive_directory_iterator(dir)) {
            const std::string ext = entry.path().extension().string();
            if (!entry.is_regular_file() || (ext != ".hpp" && ext != ".cpp")) continue;
            ++scanned;
            std::ifstream in(entry.path());
            std::string line;
            int number = 0;
            while (std::getline(in, line)) {
                ++number;
                const std::size_t start = line.find_first_not_of(" \t");
                if (start == std::string::npos) continue;
                for (const std::string& f : rule.forbidden) {
                    EXPECT_NE(line.compare(start, f.size(), f), 0)
                        << entry.path().string() << ":" << number << ": " << line;
                }
            }
        }
        EXPECT_GT(scanned, 0) << dir.string();
    }
}
