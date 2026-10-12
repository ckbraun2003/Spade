// test_model_parts.cpp -- a model's solid geometry, part by part (CORE-7; the
// parts and meshes plan, Task A1). ModelType::parts holds named, mounted,
// exact SDF parts with a wall or porous surface. issues() refuses a bad part
// by its index, its name and the field at fault, and validate() returns the
// first issue's own code. Parts are not folded into the model identity until a
// step reads them (stage 7).

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

namespace {

using spade::vehicles::ModelIssue;
using spade::vehicles::ModelType;
using spade::vehicles::PartAttach;
using spade::vehicles::PartDesc;
using spade::vehicles::PartSurface;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// The postfix pieces of a part's shape. Every leaf uses transform 0, the
// identity.
[[nodiscard]] spade::SdfNode leaf(spade::SdfPrim kind, glm::vec4 params) {
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = params;
    return n;
}

[[nodiscard]] spade::SdfNode op(spade::SdfOp which) {
    spade::SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

[[nodiscard]] spade::SdfNode sphere_node(float radius) {
    return leaf(spade::SdfPrim::sphere, glm::vec4(radius, 0.0f, 0.0f, 0.0f));
}
[[nodiscard]] spade::SdfNode box_node(float half) {
    return leaf(spade::SdfPrim::box, glm::vec4(half, half, half, 0.0f));
}
// The half-space below y = 0.
[[nodiscard]] spade::SdfNode plane_node() {
    return leaf(spade::SdfPrim::plane, glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
}

[[nodiscard]] spade::SdfProgram program(std::vector<spade::SdfNode> nodes) {
    spade::SdfProgram p;
    p.transforms.push_back(spade::SdfTransform{});
    p.nodes = std::move(nodes);
    return p;
}

[[nodiscard]] spade::SdfProgram sphere(float radius) { return program({sphere_node(radius)}); }

// A valid wall part: a 1 cm sphere at the centre of mass, declaring a 5 mm
// smallest feature.
[[nodiscard]] PartDesc part(std::string name) {
    return PartDesc{.name = std::move(name),
                    .role = "arm",
                    .attach = PartAttach::body,
                    .rotor = 0,
                    .mount_pos = glm::vec3(0.0f),
                    .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                    .shape = sphere(0.01f),
                    .surface = PartSurface{
                        .kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.005f};
}

[[nodiscard]] PartSurface porous(float inv_permeability, float inertial_coeff) {
    return PartSurface{
        .kind = PartSurface::Kind::porous, .inv_permeability = inv_permeability, .inertial_coeff = inertial_coeff};
}

// bench_sim.cpp's 1 kg airframe (four rotors, one drag body, one IMU), with
// these parts appended.
[[nodiscard]] ModelType quad_with(std::vector<PartDesc> parts) {
    spade::vehicles::QuadrotorParams p;
    p.name = "parts_quad";
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

[[nodiscard]] spade::Simulation one_world() {
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("parts")
                                       .environment(spade::Environment{})
                                       .capacities(spade::Capacities{2, 16, 4, 1})
                                       .build()
                                       .value();
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2).value();
}

}  // namespace

TEST(ModelParts, EachPartRefusalNamesThePartAndItsField) {
    // The baseline: a body part and a part on the last rotor are both fine.
    PartDesc spinner = part("bell");
    spinner.attach = PartAttach::rotor;
    spinner.rotor = 3;
    EXPECT_TRUE(quad_with({part("arm0"), spinner}).issues().empty());

    struct Case {
        const char* why;
        PartDesc bad;
        const char* field;
    };
    std::vector<Case> cases;
    cases.push_back({"a duplicate name", part("arm0"), "name"});
    cases.push_back({"an empty name", part(""), "name"});
    {
        PartDesc p = part("prop");
        p.attach = PartAttach::rotor;
        p.rotor = 4;
        cases.push_back({"rotor 4 of 4", p, "rotor"});
    }
    {
        PartDesc p = part("x");
        p.rotor = 1;
        cases.push_back({"a rotor index on a body part", p, "rotor"});
    }
    {
        PartDesc p = part("y");
        p.attach = static_cast<PartAttach>(7);
        cases.push_back({"an unknown attach", p, "attach"});
    }
    {
        PartDesc p = part("z");
        p.mount_pos.x = kNaN;
        cases.push_back({"a NaN mount", p, "mount_pos"});
    }
    {
        PartDesc p = part("w");
        p.mount_orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
        cases.push_back({"a zero rotation", p, "mount_orient"});
    }
    {
        PartDesc p = part("v");
        p.shape = {};
        cases.push_back({"no shape", p, "shape"});
    }
    {
        PartDesc p = part("v2");
        p.shape.nodes.push_back(op(spade::SdfOp::union_));
        cases.push_back({"a malformed shape", p, "shape"});
    }
    {
        PartDesc p = part("u");
        p.shape.node_materials = {0u};
        cases.push_back({"node materials", p, "shape"});
    }
    {
        PartDesc p = part("t");
        p.surface.inv_permeability = 1.0f;
        cases.push_back({"a coefficient on a wall", p, "surface"});
    }
    {
        PartDesc p = part("s");
        p.surface = porous(0.0f, -1.0f);
        cases.push_back({"a negative coefficient", p, "surface"});
    }
    {
        PartDesc p = part("r");
        p.surface.kind = static_cast<PartSurface::Kind>(9);
        cases.push_back({"an unknown kind", p, "surface"});
    }
    {
        PartDesc p = part("q");
        p.min_feature = -0.001f;
        cases.push_back({"a negative feature", p, "min_feature"});
    }
    {
        PartDesc p = part("q2");
        p.min_feature = 0.0f;
        cases.push_back({"an undeclared feature", p, "min_feature"});
    }
    {
        PartDesc p = part("q3");
        p.min_feature = kNaN;
        cases.push_back({"a NaN feature", p, "min_feature"});
    }
    for (const Case& c : cases) {
        const ModelType m = quad_with({part("arm0"), c.bad});
        const std::vector<ModelIssue> issues = m.issues();
        ASSERT_EQ(issues.size(), 1u) << c.why;
        EXPECT_EQ(issues[0].element, ModelIssue::Element::part) << c.why;
        EXPECT_EQ(issues[0].index, 1u) << c.why;
        EXPECT_EQ(issues[0].field, c.field) << c.why;
        EXPECT_EQ(issues[0].code, spade::Code::invalid_argument) << c.why;
        const std::string named = "model type 'parts_quad': part 1 '" + c.bad.name + "': ";
        EXPECT_EQ(issues[0].message.rfind(named, 0), 0u) << c.why << ": " << issues[0].message;
        const spade::Result<void> v = m.validate();
        ASSERT_FALSE(v.has_value()) << c.why;
        EXPECT_EQ(v.error().code, spade::Code::invalid_argument) << c.why;
        EXPECT_EQ(v.error().context, issues[0].message) << c.why;
    }
}

TEST(ModelParts, PartsLeaveTheModelIdentityAlone) {
    // CORE-7: not folded until a step reads them (stage 7). Green at red: a pin.
    const ModelType bare = quad_with({});
    PartDesc canopy = part("canopy");
    canopy.surface = porous(1.0e6f, 20.0f);
    PartDesc bell = part("bell");
    bell.attach = PartAttach::rotor;
    bell.rotor = 2;
    const ModelType dressed = quad_with({part("arm0"), canopy, bell});
    EXPECT_EQ(spade::vehicles::model_identity(dressed), spade::vehicles::model_identity(bare));
    EXPECT_EQ(spade::vehicles::model_registry_identity(std::vector<ModelType>{dressed}),
              spade::vehicles::model_registry_identity(std::vector<ModelType>{bare}));
}

TEST(ModelParts, AShapeDeeperThanTheStackIsCapacityExceeded) {
    // kMaxSdfDepth + 1 spheres pushed before any union: well formed, but one
    // slot deeper than eval()'s stack. The code is SdfProgram::validate()'s.
    std::vector<spade::SdfNode> nodes;
    for (uint32_t i = 0; i <= spade::kMaxSdfDepth; ++i) nodes.push_back(sphere_node(0.01f));
    for (uint32_t i = 0; i < spade::kMaxSdfDepth; ++i) nodes.push_back(op(spade::SdfOp::union_));
    PartDesc deep = part("deep");
    deep.shape = program(std::move(nodes));
    const ModelType m = quad_with({deep});

    const std::vector<ModelIssue> issues = m.issues();
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_EQ(issues[0].element, ModelIssue::Element::part);
    EXPECT_EQ(issues[0].index, 0u);
    EXPECT_EQ(issues[0].field, "shape");
    EXPECT_EQ(issues[0].code, spade::Code::capacity_exceeded);
    EXPECT_NE(issues[0].message.find("part 0 'deep'"), std::string::npos) << issues[0].message;
    const spade::Result<void> v = m.validate();
    ASSERT_FALSE(v.has_value());
    EXPECT_EQ(v.error().code, spade::Code::capacity_exceeded);
    EXPECT_EQ(v.error().context, issues[0].message);
}

TEST(ModelParts, AnUnboundedShapeIsRefused) {
    struct Case {
        const char* why;
        std::vector<spade::SdfNode> nodes;
        bool bounded;
    };
    const std::vector<Case> cases = {
        {"a plane alone", {plane_node()}, false},
        {"a heightfield alone",
         {leaf(spade::SdfPrim::heightfield, glm::vec4(0.1f, 1.0f, 1.0f, 0.0f))},
         false},
        {"a plane intersected with a box", {plane_node(), box_node(0.1f), op(spade::SdfOp::intersect)}, true},
        {"a box minus a plane", {box_node(0.1f), plane_node(), op(spade::SdfOp::subtract)}, true},
        {"a plane minus a box", {plane_node(), box_node(0.1f), op(spade::SdfOp::subtract)}, false},
        {"a box united with a plane", {box_node(0.1f), plane_node(), op(spade::SdfOp::union_)}, false},
        {"a box smoothly united with a sphere",
         {box_node(0.1f), sphere_node(0.1f), op(spade::SdfOp::smooth_union)},
         true},
    };
    for (const Case& c : cases) {
        PartDesc p = part("hull");
        p.shape = program(c.nodes);
        const std::vector<ModelIssue> issues = quad_with({p}).issues();
        if (c.bounded) {
            EXPECT_TRUE(issues.empty()) << c.why << ": " << (issues.empty() ? "" : issues[0].message);
            continue;
        }
        ASSERT_EQ(issues.size(), 1u) << c.why;
        EXPECT_EQ(issues[0].element, ModelIssue::Element::part) << c.why;
        EXPECT_EQ(issues[0].field, "shape") << c.why;
        EXPECT_NE(issues[0].message.find("unbounded"), std::string::npos) << issues[0].message;
    }
}

TEST(ModelParts, APorousPartTakesNonNegativeCoefficients) {
    // Accepted: a transparent porous part, one coefficient, both, and a plain wall.
    for (const PartSurface& ok : {porous(0.0f, 0.0f), porous(1.0e6f, 0.0f), porous(0.0f, 20.0f),
                                  porous(1.0e6f, 20.0f), PartSurface{}}) {
        PartDesc p = part("canopy");
        p.surface = ok;
        const std::vector<ModelIssue> issues = quad_with({p}).issues();
        EXPECT_TRUE(issues.empty()) << (issues.empty() ? "" : issues[0].message);
    }
    // Refused, naming the coefficient: negative or non-finite, on a porous part.
    struct Case {
        PartSurface bad;
        const char* coefficient;
    };
    for (const Case& c : {Case{porous(-1.0f, 0.0f), "inv_permeability"}, Case{porous(kNaN, 0.0f), "inv_permeability"},
                          Case{porous(0.0f, kInf), "inertial_coeff"}}) {
        PartDesc p = part("canopy");
        p.surface = c.bad;
        const std::vector<ModelIssue> issues = quad_with({p}).issues();
        ASSERT_EQ(issues.size(), 1u) << c.coefficient;
        EXPECT_EQ(issues[0].field, "surface");
        EXPECT_NE(issues[0].message.find(c.coefficient), std::string::npos) << issues[0].message;
    }
}

TEST(ModelParts, RegisterModelRefusesABadPart) {
    const ModelType twice = quad_with({part("arm0"), part("arm0")});
    const std::vector<ModelIssue> issues = twice.issues();
    ASSERT_FALSE(issues.empty()) << "a duplicate part name must be an issue";

    spade::Simulation sim = one_world();
    const spade::Result<spade::ModelTypeId> refused = sim.register_model(twice);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(refused.error().context, issues[0].message) << "the message issues() gives, word for word";

    const spade::Result<spade::ModelTypeId> once = sim.register_model(quad_with({part("arm0"), part("arm1")}));
    EXPECT_TRUE(once.has_value()) << (once ? "" : once.error().context);
}
