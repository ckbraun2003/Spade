// test_model_parts.cpp -- a model's solid geometry, part by part (CORE-7; the
// parts and meshes plan, Task A1). ModelType::parts holds named, mounted,
// exact SDF parts with a wall or porous surface. issues() refuses a bad part
// by its index, its name and the field at fault, and validate() returns the
// first issue's own code. Parts are never folded into the model identity.
//
// Task B4 (CORE-6, import only): a part's geometry is an SdfProgram OR a
// triangle mesh, exactly one, and part_sample() reads either through the
// same {distance, gradient} contract. The mesh expectations are the boxes'
// own coordinates and the SDF box beside them (TD-4).
//
// parts_identity() (vehicles/model_identity.hpp) is the parts' own identity,
// the one an Interface CFD case pins beside model_identity.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

namespace {

using MeshPtr = std::shared_ptr<const spade::TriangleMesh>;
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
                    .mesh = nullptr,
                    .surface = PartSurface{
                        .kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.005f};
}

// A box's twelve triangles, counter-clockwise seen from outside, its corners
// numbered x + 2y + 4z (tests/test_mesh_field.cpp's cube).
constexpr uint32_t kBoxTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // -z
    {4, 5, 6}, {5, 7, 6},  // +z
    {0, 1, 4}, {1, 5, 4},  // -y
    {2, 6, 3}, {3, 6, 7},  // +y
    {0, 4, 2}, {2, 4, 6},  // -x
    {1, 3, 5}, {3, 7, 5},  // +x
};

// A box `size` long on each axis about `centre`, both given in `unit`, built by
// make_mesh(), which scales it to metres once.
[[nodiscard]] MeshPtr box_mesh(glm::dvec3 centre, glm::dvec3 size, spade::MeshUnit unit) {
    std::vector<glm::dvec3> positions;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{(v & 1u) != 0 ? 0.5 : -0.5, ((v >> 1) & 1u) != 0 ? 0.5 : -0.5,
                                ((v >> 2) & 1u) != 0 ? 0.5 : -0.5};
        positions.push_back(centre + size * corner);
    }
    std::vector<uint32_t> indices;
    for (const auto& t : kBoxTriangles) indices.insert(indices.end(), {t[0], t[1], t[2]});
    const spade::MeshInput input{.positions = std::move(positions), .indices = std::move(indices), .unit = unit};
    return spade::make_mesh(input, "box").value();
}

// The 0.3 x 0.1 x 0.2 m hull about its part origin, in metres.
[[nodiscard]] MeshPtr hull_mesh() {
    return box_mesh(glm::dvec3(0.0), glm::dvec3(0.3, 0.1, 0.2), spade::MeshUnit::metre);
}

// A valid wall part whose geometry is `mesh` (no SdfProgram), declaring a
// 4 mm smallest feature.
[[nodiscard]] PartDesc mesh_part(std::string name, MeshPtr mesh) {
    return PartDesc{.name = std::move(name),
                    .role = "body",
                    .attach = PartAttach::body,
                    .rotor = 0,
                    .mount_pos = glm::vec3(0.0f),
                    .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                    .shape = spade::SdfProgram{},
                    .mesh = std::move(mesh),
                    .surface = PartSurface{
                        .kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.004f};
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
    // CORE-7: model_identity never folds parts (parts_identity() does), and
    // the registry identity does not until stage 7. Green at red: a pin.
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

// --- Task B4: a mesh as a part's geometry (CORE-6) --------------------------

TEST(ModelParts, APartHasExactlyOneOfShapeOrMesh) {
    const MeshPtr hull = hull_mesh();

    // Either form alone is a part.
    for (const PartDesc& ok : {part("sdf"), mesh_part("mesh", hull)}) {
        const ModelType m = quad_with({ok});
        const std::vector<ModelIssue> issues = m.issues();
        EXPECT_TRUE(issues.empty()) << ok.name << ": " << (issues.empty() ? "" : issues[0].message);
        EXPECT_TRUE(m.validate().has_value()) << ok.name;
    }

    // Both, or neither, is refused at `shape`, naming the part and the mesh.
    PartDesc both = part("both");
    both.mesh = hull;
    struct Case {
        const char* why;
        PartDesc bad;
        const char* says;
    };
    for (const Case& c : {Case{"both", both, "both shape and mesh"},
                          Case{"neither", mesh_part("neither", nullptr), "neither shape nor mesh"}}) {
        const ModelType m = quad_with({part("arm0"), c.bad});
        const std::vector<ModelIssue> issues = m.issues();
        ASSERT_EQ(issues.size(), 1u) << c.why;
        EXPECT_EQ(issues[0].element, ModelIssue::Element::part) << c.why;
        EXPECT_EQ(issues[0].index, 1u) << c.why;
        EXPECT_EQ(issues[0].field, "shape") << c.why;
        EXPECT_EQ(issues[0].code, spade::Code::invalid_argument) << c.why;
        const std::string named = "model type 'parts_quad': part 1 '" + c.bad.name + "': ";
        EXPECT_EQ(issues[0].message.rfind(named, 0), 0u) << c.why << ": " << issues[0].message;
        EXPECT_NE(issues[0].message.find(c.says), std::string::npos) << c.why << ": " << issues[0].message;
        const spade::Result<void> v = m.validate();
        ASSERT_FALSE(v.has_value()) << c.why;
        EXPECT_EQ(v.error().context, issues[0].message) << c.why;
    }
}

TEST(ModelParts, AMeshPartTakesEveryOtherPartCheck) {
    // A mesh replaces the SdfProgram and nothing else: min_feature is still
    // declared, finite and > 0 (CORE-8 D5), and the mount, the attachment and
    // the surface are checked as on any part. Each is the one issue, at its
    // own field.
    const MeshPtr hull = hull_mesh();
    struct Case {
        const char* why;
        PartDesc bad;
        const char* field;
    };
    std::vector<Case> cases;
    for (const float feature : {0.0f, -0.001f, kNaN, kInf}) {
        PartDesc p = mesh_part("hull", hull);
        p.min_feature = feature;
        cases.push_back({"min_feature", p, "min_feature"});
    }
    {
        PartDesc p = mesh_part("hull", hull);
        p.mount_pos.y = kNaN;
        cases.push_back({"a NaN mount", p, "mount_pos"});
    }
    {
        PartDesc p = mesh_part("hull", hull);
        p.attach = PartAttach::rotor;
        p.rotor = 9;
        cases.push_back({"rotor 9 of 4", p, "rotor"});
    }
    {
        PartDesc p = mesh_part("hull", hull);
        p.surface.inertial_coeff = 20.0f;
        cases.push_back({"a coefficient on a wall", p, "surface"});
    }
    for (const Case& c : cases) {
        const std::vector<ModelIssue> issues = quad_with({part("arm0"), c.bad}).issues();
        ASSERT_EQ(issues.size(), 1u) << c.why << ": " << (issues.empty() ? "" : issues[0].message);
        EXPECT_EQ(issues[0].element, ModelIssue::Element::part) << c.why;
        EXPECT_EQ(issues[0].index, 1u) << c.why;
        EXPECT_EQ(issues[0].field, c.field) << c.why << ": " << issues[0].message;
    }

    // A porous mesh part, and a mesh part on a rotor, are parts.
    PartDesc canopy = mesh_part("canopy", hull);
    canopy.surface = porous(1.0e6f, 20.0f);
    PartDesc bell = mesh_part("bell", hull);
    bell.attach = PartAttach::rotor;
    bell.rotor = 3;
    const std::vector<ModelIssue> fine = quad_with({canopy, bell}).issues();
    EXPECT_TRUE(fine.empty()) << (fine.empty() ? "" : fine[0].message);
}

TEST(ModelParts, AMeshPartSamplesAsTheEquivalentSdfPart) {
    // part_sample() is a host call that never throws, for either form.
    static_assert(
        noexcept(spade::vehicles::part_sample(std::declval<const PartDesc&>(), std::declval<const glm::vec3&>())));

    // One 0.3 x 0.1 x 0.2 m box, centred at (0.02, -0.01, 0.005) m in its part
    // frame, twice: as an SdfProgram, and as a mesh authored in MILLIMETRES,
    // which make_mesh() turned into metres once. The part frame is the mesh's
    // own frame; posing (the mount) is the caller's.
    const glm::vec3 half(0.15f, 0.05f, 0.1f);
    const glm::vec3 centre(0.02f, -0.01f, 0.005f);
    PartDesc sdf = part("sdf");
    sdf.shape = program({leaf(spade::SdfPrim::box, glm::vec4(half, 0.0f))});
    sdf.shape.transforms.push_back(
        spade::transform_of(spade::SdfPose{
                                .position = centre, .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f), .scale = 1.0f})
            .value());
    sdf.shape.nodes[0].transform = 1;
    ASSERT_TRUE(sdf.shape.validate().has_value());
    const PartDesc mesh = mesh_part(
        "mesh", box_mesh(glm::dvec3(20.0, -10.0, 5.0), glm::dvec3(300.0, 100.0, 200.0), spade::MeshUnit::millimetre));

    // 1000 points in a 1 m cube about the part origin, then 1000 in a box
    // 0.4 x 0.2 x 0.3 m about the hull's centre, a quarter of them inside.
    spade::rng::Stream s{.state = 0xb4a5e5b0c5ULL, .cached_gauss = 0.0f, .has_cached = 0u};
    int gradients = 0;
    int inside = 0;
    for (int i = 0; i < 2000; ++i) {
        const glm::vec3 r{s.next_float() - 0.5f, s.next_float() - 0.5f, s.next_float() - 0.5f};
        const glm::vec3 p = i < 1000 ? r : centre + glm::vec3(0.4f, 0.2f, 0.3f) * r;
        const spade::SdfSample want = spade::vehicles::part_sample(sdf, p);
        const spade::SdfSample got = spade::vehicles::part_sample(mesh, p);
        ASSERT_EQ(want.distance, spade::sample(sdf.shape, p).distance) << "an SDF part is sample() itself";

        // The distance: exact for both, so they agree within fp32 rounding.
        ASSERT_NEAR(got.distance, want.distance, 2e-6f)
            << "point " << i << " (" << p.x << ", " << p.y << ", " << p.z << ")";
        if (std::abs(want.distance) > 2e-6f) {
            ASSERT_EQ(got.distance < 0.0f, want.distance < 0.0f) << "point " << i;
        }

        // The gradient, a unit vector out of the solid, where it is defined
        // for both: a centimetre off the surface and, inside, a millimetre
        // off the box's medial planes, so the nearest face is not in doubt.
        const glm::dvec3 q = glm::abs(glm::dvec3(p) - glm::dvec3(centre)) - glm::dvec3(half);
        double depth[3] = {q.x, q.y, q.z};
        std::sort(depth, depth + 3);
        const bool clear = std::abs(want.distance) >= 1e-2f && (want.distance > 0.0f || depth[2] - depth[1] >= 1e-3);
        if (!clear) continue;
        ++gradients;
        inside += want.distance < 0.0f ? 1 : 0;
        EXPECT_NEAR(got.gradient.x, want.gradient.x, 1e-4f) << "point " << i;
        EXPECT_NEAR(got.gradient.y, want.gradient.y, 1e-4f) << "point " << i;
        EXPECT_NEAR(got.gradient.z, want.gradient.z, 1e-4f) << "point " << i;
    }
    EXPECT_GT(gradients, 1000) << "too few points compared the gradient";
    EXPECT_GT(inside, 100) << "too few points compared the gradient inside";
}

TEST(ModelParts, AMeshPartLeavesTheIdentityAlone) {
    // CORE-7: model_identity never folds parts, a mesh part included
    // (parts_identity() folds its content_hash()), and the registry identity
    // does not fold them until stage 7. Green at red: a pin.
    const ModelType bare = quad_with({});
    const ModelType dressed = quad_with({part("arm0"), mesh_part("hull", hull_mesh())});
    EXPECT_EQ(spade::vehicles::model_identity(dressed), spade::vehicles::model_identity(bare));
    EXPECT_EQ(spade::vehicles::model_registry_identity(std::vector<ModelType>{dressed}),
              spade::vehicles::model_registry_identity(std::vector<ModelType>{bare}));
}

// --- parts_identity(): the parts' own identity (TD-9: the one rule) ---------

namespace {

using spade::vehicles::kNoPartsIdentity;
using spade::vehicles::parts_identity;

// A small fixed model: two SdfProgram parts and one mesh part. Every literal
// is fixed, so its parts_identity is the same on every toolchain.
//   arm0  a 1 cm sphere on the body, 9 cm out along +X, turned 90 degrees
//         about +Z (sqrt(1/2) as a float literal, not a libm call);
//   bell  on rotor 2, porous: a sphere smooth-unioned with a box 2 cm along
//         +X (a second transform, written out: world_to_local's translation
//         column is (-0.02, 0, 0, 1));
//   hull  the 0.3 x 0.1 x 0.2 m box mesh, 1 cm below the body origin.
[[nodiscard]] ModelType fixed_parts_model() {
    PartDesc arm = part("arm0");
    arm.mount_pos = glm::vec3(0.09f, 0.0f, 0.0f);
    arm.mount_orient = glm::quat(0.70710677f, 0.0f, 0.0f, 0.70710677f);

    PartDesc bell = part("bell");
    bell.role = "motor";
    bell.attach = PartAttach::rotor;
    bell.rotor = 2;
    bell.mount_pos = glm::vec3(0.0f, 0.01f, 0.0f);
    spade::SdfTransform along_x{};
    along_x.world_to_local[3] = glm::vec4(-0.02f, 0.0f, 0.0f, 1.0f);
    bell.shape = program({sphere_node(0.01f), box_node(0.008f), op(spade::SdfOp::smooth_union)});
    bell.shape.transforms.push_back(along_x);
    bell.shape.nodes[1].transform = 1;
    bell.shape.nodes[2].params = glm::vec4(0.002f, 0.0f, 0.0f, 0.0f);
    bell.surface = porous(1.0e6f, 20.0f);
    bell.min_feature = 0.002f;

    PartDesc hull = mesh_part("hull", hull_mesh());
    hull.mount_pos = glm::vec3(0.0f, -0.01f, 0.0f);

    return quad_with({arm, bell, hull});
}

// fixed_parts_model()'s parts_identity: PINNED, and the cross-toolchain
// check (the gcc leg must reproduce it). Taken from an independent Python
// port of the fold, written byte by byte from model_identity.hpp's comment
// before the C++ fold existed (TD-4: not the code's own output read back).
constexpr uint64_t kFixedPartsIdentity = 0xb493dade88ddc89aull;

}  // namespace

TEST(PartsIdentity, NoPartsIsTheDocumentedConstant) {
    // kNoPartsIdentity, spelt here independently: FNV-1a 64 over the domain
    // tag (0xFF, then "parts_identity") and a zero part count (u32 LE).
    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    const auto fold = [&h](uint8_t b) {
        h ^= b;
        h *= spade::rng::kFnv1aPrime;
    };
    fold(0xFF);
    for (const char c : std::string_view("parts_identity")) fold(static_cast<uint8_t>(c));
    for (int i = 0; i < 4; ++i) fold(0);
    EXPECT_EQ(kNoPartsIdentity, h) << std::hex << h;
    EXPECT_NE(kNoPartsIdentity, 0u) << "0 would read as never computed";

    // Any model with no parts, whatever else it holds: parts alone are folded.
    ModelType bare = quad_with({});
    EXPECT_EQ(parts_identity(bare), kNoPartsIdentity) << std::hex << parts_identity(bare);
    bare.name = "another";
    bare.body.mass = 2.0f;
    EXPECT_EQ(parts_identity(bare), kNoPartsIdentity);
}

TEST(PartsIdentity, AFixedModelHasAPinnedValue) {
    const ModelType m = fixed_parts_model();
    ASSERT_TRUE(m.validate().has_value()) << "hash a valid model";
    EXPECT_EQ(parts_identity(m), kFixedPartsIdentity) << std::hex << parts_identity(m);
    EXPECT_EQ(parts_identity(m), parts_identity(fixed_parts_model())) << "built twice";
}

TEST(PartsIdentity, EveryFoldedFieldMovesIt) {
    const MeshPtr longer = box_mesh(glm::dvec3(0.0), glm::dvec3(0.3, 0.1, 0.21), spade::MeshUnit::metre);
    const std::vector<std::pair<const char*, std::function<void(ModelType&)>>> changes = {
        {"a mount shift by one ulp",
         [](ModelType& m) { m.parts[0].mount_pos.x = std::nextafter(m.parts[0].mount_pos.x, 1.0f); }},
        {"-0 for +0 in a mount", [](ModelType& m) { m.parts[0].mount_pos.y = -0.0f; }},
        {"a mount turn by one ulp",
         [](ModelType& m) { m.parts[0].mount_orient.w = std::nextafter(m.parts[0].mount_orient.w, 0.0f); }},
        {"a shape parameter", [](ModelType& m) { m.parts[0].shape.nodes[0].params.x = 0.011f; }},
        {"a node's kind",
         [](ModelType& m) { m.parts[1].shape.nodes[1].kind = static_cast<uint32_t>(spade::SdfPrim::sphere); }},
        {"a node's operator",
         [](ModelType& m) { m.parts[1].shape.nodes[2].op = static_cast<uint32_t>(spade::SdfOp::union_); }},
        {"a node's transform index", [](ModelType& m) { m.parts[1].shape.nodes[1].transform = 0; }},
        {"a transform's matrix", [](ModelType& m) { m.parts[1].shape.transforms[1].world_to_local[3][0] = -0.03f; }},
        {"a transform's scale", [](ModelType& m) { m.parts[1].shape.transforms[1].scale = 2.0f; }},
        {"a surface coefficient", [](ModelType& m) { m.parts[1].surface.inv_permeability = 2.0e6f; }},
        {"the other surface coefficient", [](ModelType& m) { m.parts[1].surface.inertial_coeff = 21.0f; }},
        {"the surface kind", [](ModelType& m) { m.parts[1].surface.kind = PartSurface::Kind::wall; }},
        {"min_feature", [](ModelType& m) { m.parts[2].min_feature = 0.003f; }},
        {"a name", [](ModelType& m) { m.parts[0].name = "arm1"; }},
        {"the role", [](ModelType& m) { m.parts[0].role = "strut"; }},
        {"the name and role split elsewhere (lengths are folded)",
         [](ModelType& m) {
             m.parts[0].name = "arm0a";  // "arm0" + "arm" -> "arm0a" + "rm": the same bytes
             m.parts[0].role = "rm";
         }},
        {"the attach", [](ModelType& m) { m.parts[1].attach = PartAttach::body; }},
        {"the rotor", [](ModelType& m) { m.parts[1].rotor = 3; }},
        {"a different mesh", [longer](ModelType& m) { m.parts[2].mesh = longer; }},
        {"an SDF part made a mesh part",
         [](ModelType& m) {
             m.parts[0].shape = spade::SdfProgram{};
             m.parts[0].mesh = hull_mesh();
         }},
        {"two parts swapped", [](ModelType& m) { std::swap(m.parts[0], m.parts[1]); }},
        {"a part dropped", [](ModelType& m) { m.parts.pop_back(); }},
    };
    const uint64_t base = parts_identity(fixed_parts_model());
    const uint64_t model = spade::vehicles::model_identity(fixed_parts_model());
    for (const auto& [why, change] : changes) {
        ModelType m = fixed_parts_model();
        change(m);
        EXPECT_NE(parts_identity(m), base) << why;
        EXPECT_EQ(spade::vehicles::model_identity(m), model) << why << ": model_identity never folds parts";
    }
}

TEST(PartsIdentity, ACopyKeepsIt) {
    const ModelType m = fixed_parts_model();
    const ModelType copied = m;  // shares the mesh, copies the programs
    ModelType assigned = quad_with({part("other")});
    assigned = m;
    ModelType rebuilt = quad_with({});
    rebuilt.parts = std::vector<PartDesc>(m.parts.begin(), m.parts.end());
    for (const ModelType* c : std::vector<const ModelType*>{&m, &copied, &assigned, &rebuilt}) {
        EXPECT_EQ(parts_identity(*c), kFixedPartsIdentity) << std::hex << parts_identity(*c);
    }
}

TEST(PartsIdentity, AMeshIsFoldedByItsContentHash) {
    // The same triangles, made again by another make_mesh() call: another
    // TriangleMesh, the same content_hash, the same identity. The same box
    // authored in millimetres stores the same fp32 metres (make_mesh()
    // scales once), so it is the same geometry: the unit is not folded.
    const ModelType m = fixed_parts_model();
    const MeshPtr again = hull_mesh();
    const MeshPtr in_mm = box_mesh(glm::dvec3(0.0), glm::dvec3(300.0, 100.0, 200.0), spade::MeshUnit::millimetre);
    ASSERT_NE(again.get(), m.parts[2].mesh.get());
    ASSERT_EQ(again->content_hash(), m.parts[2].mesh->content_hash());
    ASSERT_EQ(in_mm->content_hash(), m.parts[2].mesh->content_hash()) << "the same stored metres";
    for (const MeshPtr& same : {again, in_mm}) {
        ModelType remade = m;
        remade.parts[2].mesh = same;
        EXPECT_EQ(parts_identity(remade), kFixedPartsIdentity) << std::hex << parts_identity(remade);
    }

    // Different triangles move it: a box moved by a millimetre, and one a
    // centimetre longer.
    for (const MeshPtr& other :
         {box_mesh(glm::dvec3(0.001, 0.0, 0.0), glm::dvec3(0.3, 0.1, 0.2), spade::MeshUnit::metre),
          box_mesh(glm::dvec3(0.0), glm::dvec3(0.31, 0.1, 0.2), spade::MeshUnit::metre)}) {
        ASSERT_NE(other->content_hash(), m.parts[2].mesh->content_hash());
        ModelType moved = m;
        moved.parts[2].mesh = other;
        EXPECT_NE(parts_identity(moved), parts_identity(m));
    }
}

// --- model_reach(): parts join it (CORE-7; agreed with Interface) ------------
//
// The reach from the centre of mass is the largest of the contact radius, the
// farthest rotor tip and the farthest point of each part's box (part_bounds())
// through its mount; a rotor part's from its hub, whose distance it adds to,
// since it turns about the hub. quad_with()'s farthest rotor tip is about
// 0.311 m, so each part below reaches past it.

using spade::vehicles::model_reach;

// The farthest of a box's eight corners from the origin, in double.
[[nodiscard]] double farthest_corner(glm::dvec3 centre, glm::dvec3 half) {
    double best = 0.0;
    for (uint32_t c = 0; c < 8; ++c) {
        const glm::dvec3 corner = centre + glm::dvec3((c & 1u) != 0 ? half.x : -half.x,
                                                      (c & 2u) != 0 ? half.y : -half.y,
                                                      (c & 4u) != 0 ? half.z : -half.z);
        best = std::max(best, glm::length(corner));
    }
    return best;
}

[[nodiscard]] spade::SdfProgram box_shape(glm::vec3 half) {
    return program({leaf(spade::SdfPrim::box, glm::vec4(half, 0.0f))});
}

// part_bounds() pads an SDF part's box by 2^-16 of its largest coordinate, so
// the reach may exceed the exact corner by about that, never fall short.
void expect_reach(float reach, double exact, const char* what) {
    EXPECT_GE(static_cast<double>(reach), exact * (1.0 - 1e-7)) << what;
    EXPECT_LE(static_cast<double>(reach), exact * (1.0 + 1e-4)) << what;
}

TEST(ModelReach, PartsLeaveItAloneInsideTheRotorsReach) {
    const ModelType bare = quad_with({});
    float tip = 0.0f;
    for (const spade::vehicles::RotorDesc& r : bare.rotors) tip = std::max(tip, glm::length(r.local_pos) + r.radius);
    EXPECT_EQ(model_reach(bare), std::max(tip, bare.proxy_radius));
    EXPECT_EQ(model_reach(quad_with({part("hub")})), model_reach(bare)) << "a 1 cm sphere at the centre of mass";
}

TEST(ModelReach, ABodyPartReachesByItsFarthestCorner) {
    PartDesc boom = part("boom");
    boom.shape = box_shape(glm::vec3(0.05f));
    boom.mount_pos = glm::vec3(0.5f, 0.0f, 0.0f);
    const ModelType m = quad_with({boom});
    expect_reach(model_reach(m), farthest_corner(glm::dvec3(0.5, 0.0, 0.0), glm::dvec3(double(0.05f))), "a boom");
}

TEST(ModelReach, ATurnedPartReachesByItsTurnedBox) {
    // A 0.4 m bar along its own x, 0.3 m out along the body's x. Turned a
    // quarter about +Z it lies across the body's y instead: a shorter reach.
    PartDesc bar = part("bar");
    bar.shape = box_shape(glm::vec3(0.2f, 0.01f, 0.01f));
    bar.mount_pos = glm::vec3(0.3f, 0.0f, 0.0f);
    const glm::dvec3 half(double(0.2f), double(0.01f), double(0.01f));
    expect_reach(model_reach(quad_with({bar})), farthest_corner(glm::dvec3(0.3, 0.0, 0.0), half), "along x");

    bar.mount_orient = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::dvec3 turned(half.y, half.x, half.z);
    expect_reach(model_reach(quad_with({bar})), farthest_corner(glm::dvec3(0.3, 0.0, 0.0), turned), "across y");
}

TEST(ModelReach, ARotorPartReachesFromItsHub) {
    // A 0.1 m box 0.1 m above rotor 0's hub, in the hub frame: it turns with
    // the blades, so its reach is the hub's distance plus its own.
    PartDesc guard = part("guard");
    guard.attach = PartAttach::rotor;
    guard.rotor = 0;
    guard.shape = box_shape(glm::vec3(0.05f));
    guard.mount_pos = glm::vec3(0.0f, 0.1f, 0.0f);
    const ModelType m = quad_with({guard});
    const double hub = glm::length(glm::dvec3(m.rotors[0].local_pos));
    expect_reach(model_reach(m), hub + farthest_corner(glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(double(0.05f))),
                 "a rotor guard");
}

TEST(ModelReach, AMeshPartReachesByItsVertices) {
    // The 0.3 x 0.1 x 0.2 m hull, 0.3 m out along x. A mesh part's box is its
    // vertices' extremes exactly, so the reach is the farthest vertex, in float.
    PartDesc hull = mesh_part("hull", hull_mesh());
    hull.mount_pos = glm::vec3(0.3f, 0.0f, 0.0f);
    float want = 0.0f;
    for (const glm::vec3& v : hull.mesh->positions()) want = std::max(want, glm::length(hull.mount_pos + v));
    EXPECT_EQ(model_reach(quad_with({hull})), want);
}
