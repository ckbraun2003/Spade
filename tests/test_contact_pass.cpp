// test_contact_pass.cpp -- L9, part contact (Physics' note, accepted by the
// lead 2026-11-03), on Core's table (L10, L11): the contact sets' derivation
// (physics/contact_sets.cpp) and static_contact.resolve's set path
// (physics/contacts.cpp; collision_static.slang on the device). A body with a
// set rests on its most penetrated point at radius 0; a body without one keeps
// the sphere, byte for byte (the existing contact suite and goldens hold that,
// and the first test here pins it directly).
//
// Mutations, each failing a test below: the set ignored (every body on the
// sphere); the rims dropped; a tie broken last-first (`<=`).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "contact_set_fakes.hpp"
#include "physics/contact_sets.hpp"
#include "physics/contacts.hpp"
#include "physics/integrator.hpp"
#include "library_drones.hpp"
#include "vehicles/airframe_compile.hpp"
#include "sim/simulation.hpp"
#include "state/layout.hpp"
#include "vehicles/airflow_solids.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace {

namespace ph = spade::physics;
using spade::BodyState;
using spade::Simulation;
using spade::vehicles::ModelType;

namespace ld = spade::test::library_drones;

// A library drone as Run flies it: built from the committed library, compiled.
[[nodiscard]] ModelType library_drone(const char* stem) {
    const spade::Result<ld::Committed> lib = ld::committed_library();
    EXPECT_TRUE(lib.has_value()) << (lib ? "" : lib.error().context);
    if (!lib) return ModelType{};
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, stem);
    EXPECT_TRUE(drone.has_value()) << stem << ": " << (drone ? "" : drone.error().context);
    if (!drone) return ModelType{};
    const auto compiled = spade::vehicles::compile_airframe(drone->spec);
    EXPECT_TRUE(compiled.has_value()) << stem << ": " << (compiled ? "" : compiled.error().context);
    return compiled ? compiled->model : ModelType{};
}

[[nodiscard]] std::size_t body_parts(const ModelType& m) {
    return static_cast<std::size_t>(std::count_if(m.parts.begin(), m.parts.end(), [](const auto& p) {
        return p.attach == spade::vehicles::PartAttach::body;
    }));
}

[[nodiscard]] float length(glm::vec3 v) { return std::sqrt(glm::dot(v, v)); }

// ===========================================================================
// The derivation.
// ===========================================================================

TEST(ContactPass, AModelWithoutABodyPartHasNoSetAndTheTableStaysEmpty) {
    const ModelType quad = contact_set_fakes::quad_of_mass(1.0f);
    ASSERT_TRUE(quad.parts.empty());
    EXPECT_TRUE(ph::contact_points(quad).empty());
    std::vector<float> table;
    const ModelType models[] = {quad, quad};
    ph::build_contact_sets(models, table);
    EXPECT_TRUE(table.empty()) << "a part-less registry keeps master's identity";
    BodyState b{};
    b.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(ph::lowest_contact_point(quad, b).has_value());
}

// The Phantom: its 11 body parts' corners (drawn-only included: nothing here
// filters by role), then 8 rim points per rotor at radius R in the hub plane.
// Mutation: the rims dropped.
TEST(ContactPass, ThePhantomsPointsAreItsCornersThenItsRims) {
    const ModelType m = library_drone("phantom3_class");
    const std::vector<glm::vec3> pts = ph::contact_points(m);
    ASSERT_EQ(body_parts(m), 11u);
    ASSERT_EQ(m.rotors.size(), 4u);
    ASSERT_EQ(pts.size(), 8u * 11u + 8u * 4u) << "120 points";
    const std::size_t rims = 8u * 11u;
    for (std::size_t r = 0; r < m.rotors.size(); ++r) {
        const auto& rotor = m.rotors[r];
        const glm::vec3 x_hub = rotor.local_orient * glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 y_hub = rotor.local_orient * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 z_hub = rotor.local_orient * glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3* rim = &pts[rims + 8u * r];
        EXPECT_EQ(rim[0], rotor.local_pos + rotor.radius * (1.0f * x_hub + 0.0f * z_hub)) << r;
        EXPECT_EQ(rim[2], rotor.local_pos + rotor.radius * (0.0f * x_hub + 1.0f * z_hub)) << r;
        for (uint32_t k = 0; k < 8u; ++k) {
            const glm::vec3 d = rim[k] - rotor.local_pos;
            EXPECT_NEAR(length(d), rotor.radius, 1.0e-6f) << r << " " << k;
            EXPECT_NEAR(glm::dot(d, y_hub), 0.0f, 1.0e-6f) << "in the hub plane: " << r << " " << k;
        }
    }
}

// The agreed layout (L10): a range row per model in registration order, a
// part-less model's {0, 0, 0, 0}, first absolute; then the points.
TEST(ContactPass, TheTableIsTheRangeRowsThenThePoints) {
    const ModelType quad = contact_set_fakes::quad_of_mass(1.0f);
    const ModelType phantom = library_drone("phantom3_class");
    const ModelType models[] = {quad, phantom};
    std::vector<float> t;
    ph::build_contact_sets(models, t);
    const std::vector<glm::vec3> pts = ph::contact_points(phantom);
    ASSERT_EQ(t.size(), (2u + pts.size()) * ph::kContactSetRowFloats);
    EXPECT_EQ(t[0], 0.0f);
    EXPECT_EQ(t[1], 0.0f);
    EXPECT_EQ(t[4], 2.0f) << "first: absolute, after the two range rows";
    EXPECT_EQ(t[5], static_cast<float>(pts.size()));
    EXPECT_EQ(t[6], 0.0f);
    EXPECT_EQ(t[7], 0.0f);
    for (std::size_t k = 0; k < pts.size(); ++k) {
        const float* row = &t[(2u + k) * ph::kContactSetRowFloats];
        EXPECT_EQ(std::memcmp(row, &pts[k], 3 * sizeof(float)), 0) << k;
        EXPECT_EQ(row[3], 0.0f) << k;
    }
}

// L9's one source for rest: the lowest world y over the set, by the pass's
// own expression (contact_point_world()).
TEST(ContactPass, TheLowestPointIsTheSetsLowestAtThePose) {
    const ModelType m = library_drone("phantom3_class");
    BodyState b{};
    b.pos = glm::vec3(0.3f, 1.25f, -0.2f);
    b.orient = glm::normalize(glm::quat(0.98f, 0.1f, 0.15f, 0.05f));
    float low = 1.0e9f;
    for (const glm::vec3& p : ph::contact_points(m)) low = std::min(low, ph::contact_point_world(b, p).y);
    const std::optional<float> got = ph::lowest_contact_point(m, b);
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(*got, low);
}

// ===========================================================================
// The pass, on hand-made bodies and tables.
// ===========================================================================

[[nodiscard]] spade::SdfNode plane_node(glm::vec3 n, float offset) {
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::plane);
    node.op = static_cast<uint32_t>(spade::SdfOp::none);
    node.transform = 0;
    node.params = glm::vec4(n, offset);
    return node;
}

// The floor y <= 0 union the wall x <= 0.
[[nodiscard]] spade::SdfProgram corner() {
    spade::SdfProgram prog;
    prog.transforms.push_back(spade::SdfTransform{});
    prog.nodes.push_back(plane_node(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f));
    prog.nodes.push_back(plane_node(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f));
    spade::SdfNode u{};
    u.op = static_cast<uint32_t>(spade::SdfOp::union_);
    prog.nodes.push_back(u);
    EXPECT_TRUE(prog.validate().has_value());
    return prog;
}

[[nodiscard]] ph::ContactParams still_material() {
    ph::ContactParams p{};
    p.restitution_e = 0.0f;
    p.friction_mu = 0.0f;
    p.baumgarte_beta = 0.5f;
    p.slop = 0.0f;
    p.proxy_radius = 0.05f;
    return p;
}

[[nodiscard]] BodyState body_at(glm::vec3 pos, uint32_t lane) {
    BodyState b{};
    b.pos = pos;
    b.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.mass = 1.0f;
    b.flags = ph::body_flags::active;
    b.contact_set = lane;
    return b;
}

// A one-model table: its range row, then `points`.
[[nodiscard]] std::vector<float> table_of(const std::vector<glm::vec3>& points) {
    std::vector<float> t = {1.0f, static_cast<float>(points.size()), 0.0f, 0.0f};
    for (const glm::vec3& p : points) t.insert(t.end(), {p.x, p.y, p.z, 0.0f});
    return t;
}

// A lane of 0, or a lane whose range names no points, takes the sphere byte
// for byte; so does a set lane beside an empty table.
TEST(ContactPass, ABodyWithoutASetTakesTheSphereByteForByte) {
    const spade::SdfProgram world = corner();
    const ph::ContactParams mat = still_material();
    const std::vector<float> table = table_of({{0.5f, -0.2f, 0.0f}});
    const std::vector<float> empty_range = {0.0f, 0.0f, 0.0f, 0.0f};
    for (const glm::vec3 pos : {glm::vec3(0.5f, 0.02f, 0.0f), glm::vec3(0.03f, 0.5f, 0.0f)}) {
        BodyState sphere[] = {body_at(pos, 0u)};
        ph::resolve_static_contacts(sphere, world, mat, 0.001f);
        BodyState lane0[] = {body_at(pos, 0u)};
        ph::resolve_static_contacts(lane0, world, mat, 0.001f, {}, table);
        BodyState no_table[] = {body_at(pos, 1u)};
        ph::resolve_static_contacts(no_table, world, mat, 0.001f, {}, {});
        BodyState no_points[] = {body_at(pos, 1u)};
        ph::resolve_static_contacts(no_points, world, mat, 0.001f, {}, empty_range);
        for (const BodyState* other : {&lane0[0], &no_table[0], &no_points[0]}) {
            BodyState a = sphere[0];
            BodyState b = *other;
            a.contact_set = 0u;
            b.contact_set = 0u;
            EXPECT_EQ(std::memcmp(&a, &b, sizeof(BodyState)), 0);
        }
    }
}

// The most penetrated point wins, and the first wins a tie: two points 1 cm
// into the floor and the wall respectively. First the floor's: the body is
// pushed up (+y) only. Then the wall's first: pushed along +x only. And a
// deeper wall point beats a shallower floor point listed first. Mutation: a
// tie broken last-first (`<=`).
TEST(ContactPass, TheMostPenetratedPointWinsAndTheFirstWinsATie) {
    const spade::SdfProgram world = corner();
    const ph::ContactParams mat = still_material();
    const glm::vec3 in_floor(0.5f, -0.01f, 0.0f);
    const glm::vec3 in_wall(-0.01f, 0.5f, 0.0f);
    struct Case {
        std::vector<glm::vec3> points;
        glm::vec3 push;  // the correction's direction
    };
    const Case cases[] = {{{in_floor, in_wall}, {0.0f, 1.0f, 0.0f}},
                          {{in_wall, in_floor}, {1.0f, 0.0f, 0.0f}},
                          {{in_floor, glm::vec3(-0.02f, 0.5f, 0.0f)}, {1.0f, 0.0f, 0.0f}}};
    for (const Case& c : cases) {
        const std::vector<float> table = table_of(c.points);
        BodyState b[] = {body_at(glm::vec3(0.0f), 1u)};
        ph::resolve_static_contacts(b, world, mat, 0.001f, {}, table);
        const glm::vec3 moved = b[0].pos;
        EXPECT_GT(glm::dot(moved, c.push), 0.0f) << moved.x << " " << moved.y;
        EXPECT_EQ(glm::dot(moved, glm::vec3(1.0f) - c.push), 0.0f) << moved.x << " " << moved.y;
    }
}

// ===========================================================================
// Drones in a world.
// ===========================================================================

constexpr uint64_t kDtNs = 2'000'000;
constexpr uint32_t kSubsteps = 2;

// One world with a ground at y = 0 and, when `wall_x` is finite, a wall
// filling x >= wall_x; default material.
[[nodiscard]] spade::WorldSetDesc ground_world(float wall_x = INFINITY) {
    spade::WorldBuilder b;
    b.name("contact_pass").environment(spade::Environment{}).capacities(spade::Capacities{4, 16, 4, 4});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    if (std::isfinite(wall_x)) b.plane(glm::vec3(-1.0f, 0.0f, 0.0f), -wall_x).union_();
    auto built = b.build();
    EXPECT_TRUE(built.has_value()) << (built ? "" : built.error().context);
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

struct Placed {
    Simulation sim;
    spade::VehicleRef v;
    ModelType model;
};

// `m` spawned level in the design frame and then lowered, by set_body_state(),
// so its lowest contact point is at y = `lift` (the editor's placement);
// rotors stopped, nothing commanded (disarmed).
[[nodiscard]] std::optional<Placed> placed(const ModelType& m, float lift = 0.0f, float wall_x = INFINITY) {
    auto sim = Simulation::create(ground_world(wall_x), kDtNs, kSubsteps);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    if (!sim) return std::nullopt;
    const auto id = sim->register_model(m);
    EXPECT_TRUE(id.has_value()) << (id ? "" : id.error().context);
    if (!id) return std::nullopt;
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 1.0f, 0.0f);
    const auto v = sim->spawn(0, *id, where);
    EXPECT_TRUE(v.has_value()) << (v ? "" : v.error().context);
    if (!v || !sim->flush_structural()) return std::nullopt;
    const BodyState b = **sim->body(v->body);
    EXPECT_NE(b.contact_set, 0u) << m.name << " has a set";
    const std::optional<float> low = ph::lowest_contact_point(m, b);
    EXPECT_TRUE(low.has_value());
    if (!low) return std::nullopt;
    const glm::vec3 pos = b.pos - glm::vec3(0.0f, *low - lift, 0.0f);
    const auto set = sim->set_body_state(v->body, pos, b.orient, glm::vec3(0.0f), glm::vec3(0.0f));
    EXPECT_TRUE(set.has_value()) << (set ? "" : set.error().context);
    return Placed{std::move(*sim), *v, m};
}

[[nodiscard]] float tilt_degrees(glm::quat a, glm::quat b) {
    const float d = std::min(1.0f, std::fabs(glm::dot(a, b)));
    return 2.0f * std::acos(d) * 57.29578f;
}

// Test/Docs' placed-at, its stillness half (L9): a drone placed at its lowest
// point, disarmed, for 1 s of Run moves at most 10 mm and does not tilt, and
// its lowest point settles in [-2, +5] mm. On the sphere the Phantom would be
// lifted to its proxy radius, 0.296 m, about 266 mm. Mutation: the set ignored.
TEST(ContactPass, ADronePlacedAtItsLowestPointStaysPut) {
    for (const char* file : {"phantom3_class", "iris_plus_class"}) {
        SCOPED_TRACE(file);
        std::optional<Placed> p = placed(library_drone(file));
        ASSERT_TRUE(p.has_value());
        const BodyState start = **p->sim.body(p->v.body);
        ASSERT_TRUE(p->sim.step(500).has_value()) << "1 s at 2 ms";
        const BodyState end = **p->sim.body(p->v.body);
        const float moved = length(end.pos - start.pos);
        const float low = ph::lowest_contact_point(p->model, end).value_or(1.0e9f);
        std::printf("[ L9 ] %s placed at its lowest point, 1 s disarmed: moved %.2f mm, lowest point %+.2f mm, "
                    "tilt %.4f deg\n",
                    file, 1000.0 * moved, 1000.0 * low, static_cast<double>(tilt_degrees(start.orient, end.orient)));
        EXPECT_LE(moved, 0.010f);
        EXPECT_LT(tilt_degrees(start.orient, end.orient), 0.01f);
        EXPECT_GE(low, -0.002f);
        EXPECT_LE(low, 0.005f);
    }
}

// A drone flown level into a wall stops with a contact point at the wall:
// the reach its own points give, not the sphere's. The Phantom from 0.5 m
// short of a wall at 2 m/s; its body's centre never comes nearer the wall than
// its farthest point toward it, less the slop's few millimetres.
TEST(ContactPass, ADroneFlownLevelIntoAWallStopsAtItsOwnPoints) {
    const ModelType m = library_drone("phantom3_class");
    std::optional<Placed> p = placed(m, 0.5f, 1.0f);
    ASSERT_TRUE(p.has_value());
    BodyState b = **p->sim.body(p->v.body);
    float reach = -1.0e9f;  // the farthest point toward +x from the centre, at this pose
    for (const glm::vec3& q : ph::contact_points(m)) reach = std::max(reach, (ph::contact_point_world(b, q) - b.pos).x);
    const glm::vec3 start(1.0f - reach - 0.5f, b.pos.y, b.pos.z);
    ASSERT_TRUE(p->sim.set_body_state(p->v.body, start, b.orient, glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(0.0f)));
    float nearest = -1.0e9f;
    for (int s = 0; s < 250; ++s) {  // 0.5 s: the drone reaches the wall after 0.25 s
        ASSERT_TRUE(p->sim.step(1).has_value());
        b = **p->sim.body(p->v.body);
        nearest = std::max(nearest, b.pos.x);
    }
    std::printf("[ L9 ] the Phantom into a wall at x = 1 m: its centre reached x = %.4f m; its own reach %.4f m "
                "(the sphere's: 0.296 m)\n",
                static_cast<double>(nearest), static_cast<double>(reach));
    EXPECT_GT(nearest, 1.0f - reach - 0.005f) << "it reached the wall";
    EXPECT_LT(nearest, 1.0f - reach + 0.005f) << "a point of its own stopped it";
}

// A drone standing on a drawn-only skid: L9 counts every body part, the
// air's drawn-only mark included. The Phantom with a 4 mm-thick skid well under
// its lowest part (its mount is in the design frame, the points in the body
// frame, so the margin covers the centre of mass's offset) rests on the skid.
TEST(ContactPass, ADroneStandsOnADrawnOnlySkid) {
    ModelType m = library_drone("phantom3_class");
    BodyState level{};
    level.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const float bottom = ph::lowest_contact_point(m, level).value();
    spade::vehicles::PartDesc skid;
    skid.name = "skid";
    skid.role = std::string(spade::vehicles::kDrawnOnlyRole);
    skid.attach = spade::vehicles::PartAttach::body;
    skid.mount_pos = glm::vec3(0.0f, bottom - 0.1f, 0.0f);
    skid.mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.op = static_cast<uint32_t>(spade::SdfOp::none);
    box.params = glm::vec4(0.08f, 0.002f, 0.08f, 0.0f);
    skid.shape.transforms.push_back(spade::SdfTransform{});
    skid.shape.nodes = {box};
    skid.surface = spade::vehicles::PartSurface{.kind = spade::vehicles::PartSurface::Kind::wall,
                                                .inv_permeability = 0.0f, .inertial_coeff = 0.0f};
    skid.min_feature = 0.004f;
    m.parts.push_back(skid);
    const std::vector<glm::vec3> pts = ph::contact_points(m);
    ASSERT_EQ(pts.size(), 8u * 12u + 8u * 4u) << "the skid's 8 corners join the set";
    float skid_level = 1.0e9f;
    float rest_level = 1.0e9f;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        float& lowest = k >= 8u * 11u && k < 8u * 12u ? skid_level : rest_level;
        lowest = std::min(lowest, pts[k].y);
    }
    ASSERT_LT(skid_level, rest_level - 0.05f) << "the skid is the drone's lowest part";
    std::optional<Placed> p = placed(m);
    ASSERT_TRUE(p.has_value());
    ASSERT_TRUE(p->sim.step(500).has_value());
    const BodyState end = **p->sim.body(p->v.body);
    float skid_low = 1.0e9f;
    for (std::size_t k = 8u * 11u; k < 8u * 12u; ++k) {
        skid_low = std::min(skid_low, ph::contact_point_world(end, pts[k]).y);
    }
    std::printf("[ L9 ] the Phantom on a drawn-only skid, 1 s: the skid's underside at %+.2f mm\n", 1000.0 * skid_low);
    EXPECT_GE(skid_low, -0.002f);
    EXPECT_LE(skid_low, 0.005f);
}

}  // namespace
