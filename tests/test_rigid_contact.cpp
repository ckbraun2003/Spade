// test_rigid_contact.cpp -- contact tumbling on the CPU (L68; docs/design/physics/plans/2026-11-11-l68-contact-tumbling.md).
//
// A body with a SHAPE (sphere, capsule, box, hull) meets the world SDF at several points and each contact's impulse acts at its point, so a
// corner hit spins the body up and it rolls and settles on a face; friction stops a slide, restitution bounces. The module is opt-in
// (modules::rigid_contact_module()): a standard set, and a body with no shape, are untouched. The device half is test_gpu_rigid_contact.cpp.
//
// Pins, green at red and for good: the standard set has no rigid_contact array, pass or identity change; a module set with no shaped body evolves
// every body exactly as the standard set does.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "physics/rigid_contact.hpp"
#include "rigid_contact_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

using namespace spade::test::rigid;


// ---- pins, green at red --------------------------------------------------------------------------------------------------------------------

// PIN. A standard set has no rigid_contact array or pass and keeps the identity and the 11-entry walk it has.
TEST(RigidContact, AStandardSetHasNoRigidContactAndKeepsItsIdentity) {
    const auto s = spade::modules::compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, 0x3303cfc86821f502ULL);
    EXPECT_EQ(spade::modules::walk_order(*s).size(), 11u);
    EXPECT_FALSE(std::ranges::any_of(s->arrays, [](const auto& a) { return a.module == "rigid_contact"; }));
    EXPECT_FALSE(std::ranges::any_of(s->passes, [](const auto& p) { return p.module == "rigid_contact"; }));
    Simulation sim = make(spade::modules::standard_modules());
    EXPECT_FALSE(sim.module_array("shape").has_value()) << "no shape array in a standard set";
}

// PIN. The module adds one array and one pass between the two contact passes, and nothing else.
TEST(RigidContact, TheModuleAddsOneArrayAndOnePassBetweenTheContactPasses) {
    const auto s = spade::modules::compile_schedule(tumbling_set());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    std::vector<std::string> order;
    for (const auto& p : s->passes) order.push_back(p.module + "." + p.pass);
    const auto stat = std::ranges::find(order, "static_contact.resolve");
    const auto rigid = std::ranges::find(order, "rigid_contact.resolve");
    const auto dyn = std::ranges::find(order, "dynamic_contact.resolve");
    ASSERT_NE(rigid, order.end());
    EXPECT_LT(stat, rigid);
    EXPECT_LT(rigid, dyn);
    const auto arrays = std::ranges::count_if(s->arrays, [](const auto& a) { return a.module == "rigid_contact"; });
    EXPECT_EQ(arrays, 1);
    EXPECT_EQ(sizeof(ph::ShapeRow), 288u);
}

// PIN. A module set with no shaped body evolves every body exactly as the standard set does: bodies, bit for bit.
TEST(RigidContact, ASetWithNoShapedBodyEvolvesBodiesAsTheStandardSetDoes) {
    const WorldOpts world{.mu = 0.4f, .e = 0.2f, .proxy_radius = 0.1f};
    Simulation standard = make(spade::modules::standard_modules(), world);
    Simulation module = make(tumbling_set(), world);
    const glm::quat q = tilt();
    const BodyRef a = put(standard, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), q, glm::vec3(5.0f));
    const BodyRef b = put(module, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), q, glm::vec3(5.0f));
    ASSERT_TRUE(standard.step(1500).has_value());
    ASSERT_TRUE(module.step(1500).has_value());
    const BodyState sa = body(standard, a);
    const BodyState sb = body(module, b);
    EXPECT_EQ(std::memcmp(&sa, &sb, sizeof(BodyState)), 0) << "an unshaped body is untouched by the module";
    EXPECT_GT(sa.pos.y, 0.0f);
}

// ---- the helper and the shapes ----------------------------------------------------------------------------------------------------------

TEST(RigidContact, BoxInverseInertiaIsTheSolidBoxs) {
    const glm::vec3 half(0.1f, 0.2f, 0.3f);
    const glm::vec3 inv = ph::box_inv_inertia(2.0f, half);
    EXPECT_FLOAT_EQ(inv.x, 3.0f / (2.0f * (0.2f * 0.2f + 0.3f * 0.3f)));
    EXPECT_FLOAT_EQ(inv.y, 3.0f / (2.0f * (0.1f * 0.1f + 0.3f * 0.3f)));
    EXPECT_FLOAT_EQ(inv.z, 3.0f / (2.0f * (0.1f * 0.1f + 0.2f * 0.2f)));
}

TEST(RigidContact, EveryShapeCompilesToAPointListAndARadius) {
    const auto sphere = ph::compile_shape(ph::sphere_shape(0.25f));
    ASSERT_TRUE(sphere.has_value()) << sphere.error().context;
    EXPECT_EQ(sphere->kind, static_cast<uint32_t>(ph::ShapeKind::sphere));
    EXPECT_EQ(sphere->count, 1u);
    EXPECT_EQ(sphere->radius, 0.25f);
    EXPECT_EQ(sphere->points[0][0], 0.0f);

    const auto box = ph::compile_shape(ph::box_shape(glm::vec3(0.1f, 0.2f, 0.3f), 0.01f));
    ASSERT_TRUE(box.has_value()) << box.error().context;
    EXPECT_EQ(box->count, 8u);
    EXPECT_EQ(box->radius, 0.01f);
    for (uint32_t c = 0; c < 8; ++c) {  // corner c: +h.x if c & 1, +h.y if c & 2, +h.z if c & 4
        EXPECT_EQ(box->points[c][0], (c & 1) ? 0.1f : -0.1f) << c;
        EXPECT_EQ(box->points[c][1], (c & 2) ? 0.2f : -0.2f) << c;
        EXPECT_EQ(box->points[c][2], (c & 4) ? 0.3f : -0.3f) << c;
    }

    const auto capsule = ph::compile_shape(ph::capsule_shape(glm::vec3(-0.3f, 0.0f, 0.0f), glm::vec3(0.3f, 0.0f, 0.0f), 0.05f));
    ASSERT_TRUE(capsule.has_value()) << capsule.error().context;
    EXPECT_EQ(capsule->count, 13u) << "0.6 m at no more than 0.05 m apart, ends included";
    EXPECT_EQ(capsule->points[0][0], -0.3f);
    EXPECT_EQ(capsule->points[12][0], 0.3f);

    const auto hull = ph::compile_shape(ph::hull_shape({glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)}));
    ASSERT_TRUE(hull.has_value()) << hull.error().context;
    EXPECT_EQ(hull->count, 3u);
    EXPECT_EQ(hull->points[1][1], 1.0f);
}

// ---- refusals by name -----------------------------------------------------------------------------------------------------------------------

TEST(RigidContact, ASetWithoutTheModuleRefusesAShapeByName) {
    Simulation sim = make(spade::modules::standard_modules());
    const BodyRef ref = put(sim, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f));
    const spade::Result<void> r = sim.set_body_shape(ref, cube());
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::unavailable);
    EXPECT_NE(r.error().context.find("rigid_contact"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("set_body_shape"), std::string::npos) << r.error().context;
}

TEST(RigidContact, ABadDescriptionIsRefusedNamingItAndChangesNothing) {
    Simulation sim = make(tumbling_set());
    const BodyRef ref = put(sim, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f));
    const uint64_t before = spade::testing::state_digest(sim);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<glm::vec3> seventeen;
    for (int i = 0; i < 17; ++i) seventeen.emplace_back(static_cast<float>(i), 0.0f, 0.0f);
    struct Case {
        ph::ShapeDesc desc;
        const char* names;
    };
    ph::ShapeDesc bad_restitution = cube();
    bad_restitution.restitution = 1.5f;
    ph::ShapeDesc nan_friction = cube();
    nan_friction.friction = nan;
    ph::ShapeDesc negative_rest = cube();
    negative_rest.rest_speed = -1.0f;
    const Case cases[] = {
        {ph::sphere_shape(0.0f), "radius"},
        {ph::sphere_shape(-1.0f), "radius"},
        {ph::capsule_shape(glm::vec3(0.0f), glm::vec3(0.0f), 0.05f), "capsule"},
        {ph::capsule_shape(glm::vec3(-0.5f, 0.0f, 0.0f), glm::vec3(0.5f, 0.0f, 0.0f), 0.05f), "16"},  // 1 m at 0.05 m needs 21 points
        {ph::box_shape(glm::vec3(0.1f, 0.0f, 0.1f)), "half_extents"},
        {ph::box_shape(glm::vec3(0.1f), -0.1f), "radius"},
        {ph::hull_shape({}), "hull"},
        {ph::hull_shape(seventeen), "16"},
        {ph::hull_shape({glm::vec3(0.0f), glm::vec3(nan, 0.0f, 0.0f)}), "finite"},
        {bad_restitution, "restitution"},
        {nan_friction, "friction"},
        {negative_rest, "rest_speed"},
    };
    for (const Case& c : cases) {
        const spade::Result<void> r = sim.set_body_shape(ref, c.desc);
        ASSERT_FALSE(r.has_value()) << c.names;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << c.names << ": " << r.error().context;
        EXPECT_NE(r.error().context.find("set_body_shape"), std::string::npos) << r.error().context;
        EXPECT_NE(r.error().context.find(c.names), std::string::npos) << r.error().context;
        EXPECT_EQ(spade::testing::state_digest(sim), before) << c.names << ": a refusal changes nothing";
    }
    const spade::Result<void> stale = sim.set_body_shape(BodyRef{0, 3, 99}, cube());
    ASSERT_FALSE(stale.has_value());
    EXPECT_EQ(stale.error().code, spade::Code::not_found);
}

TEST(RigidContact, AShapeIsStateTheDigestFoldsAndNoShapeClearsIt) {
    Simulation sim = make(tumbling_set());
    const BodyRef ref = put(sim, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f));
    const uint64_t bare = spade::testing::state_digest(sim);
    ASSERT_TRUE(shaped(sim, ref, cube()));
    EXPECT_NE(spade::testing::state_digest(sim), bare) << "the row is in the digest";
    const auto row = sim.body_shape_row(ref);
    ASSERT_TRUE(row.has_value()) << row.error().context;
    EXPECT_EQ(row->count, 8u);
    ASSERT_TRUE(shaped(sim, ref, ph::ShapeDesc{}));
    EXPECT_EQ(spade::testing::state_digest(sim), bare) << "ShapeDesc{} clears the row to zero";
}

// ---- the physics --------------------------------------------------------------------------------------------------------------------------------

// Restitution: a sphere dropped 1 m rebounds to e^2 of the height, and comes to rest (the rest speed stops the micro-bounces).
TEST(RigidContact, ASphereBouncesToERestitutionSquaredAndRests) {
    Simulation sim = make(tumbling_set());
    const float r = 0.1f;
    const BodyRef ref = put(sim, glm::vec3(0.0f, 1.0f + r, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f / (0.4f * r * r)));
    ph::ShapeDesc d = ph::sphere_shape(r);
    d.restitution = 0.5f;
    d.friction = 0.0f;
    ASSERT_TRUE(shaped(sim, ref, d));
    bool rebounding = false;
    float apex = 0.0f;
    for (int i = 0; i < 1500; ++i) {
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState b = body(sim, ref);
        if (b.vel.y > 0.5f) rebounding = true;
        if (rebounding) apex = std::max(apex, b.pos.y - r);
    }
    EXPECT_NEAR(apex, 0.25f, 0.03f) << "e^2 * 1 m";
    ASSERT_TRUE(sim.step(4000).has_value());
    const BodyState end = body(sim, ref);
    EXPECT_LT(std::abs(end.vel.y), 0.05f);
    EXPECT_NEAR(end.pos.y, r, 0.005f);
}

// Friction: a box sliding level stops at v^2 / (2 mu g), and stays stopped.
TEST(RigidContact, ALevelSlideStopsAtVSquaredOverTwoMuG) {
    Simulation sim = make(tumbling_set());
    const glm::vec3 half(0.2f, 0.05f, 0.2f);
    const BodyRef ref = put(sim, glm::vec3(0.0f, half.y, 0.0f), glm::vec3(3.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            ph::box_inv_inertia(1.0f, half));
    ph::ShapeDesc d = ph::box_shape(half);
    d.friction = 0.5f;
    d.restitution = 0.0f;
    ASSERT_TRUE(shaped(sim, ref, d));
    ASSERT_TRUE(sim.step(3000).has_value());
    const BodyState end = body(sim, ref);
    EXPECT_NEAR(end.pos.x, 9.0f / (2.0f * 0.5f * kG), 0.05f * 9.0f / (2.0f * 0.5f * kG));
    EXPECT_LT(glm::length(end.vel), 1e-3f);
    EXPECT_NEAR(end.pos.y, half.y, 0.005f);
}

// The corner hit: a cube dropped on a corner spins up at the first impact, then rolls and comes to rest on a face.
TEST(RigidContact, ACubeDroppedOnACornerSpinsUpAndSettlesOnAFace) {
    Simulation sim = make(tumbling_set());
    const glm::quat q = tilt();
    const BodyRef ref = put(sim, glm::vec3(0.0f, centre_for_lowest(q, kCubeHalf, 1.0f), 0.0f), glm::vec3(0.0f), q, cube_inv());
    ASSERT_TRUE(shaped(sim, ref, cube()));
    const Settle s = drop_cube(sim, ref, 8000);
    EXPECT_GT(s.max_w_after_first_contact, 0.5f) << "r x J spins the body up";
    EXPECT_LT(s.last_half_max_v, 0.02f);
    EXPECT_LT(s.last_half_max_w, 0.1f);
    EXPECT_GE(s.up_alignment, std::cos(glm::radians(2.0f))) << "a face is down";
    EXPECT_NEAR(s.end.pos.y, kCubeHalf, 0.01f) << "resting on that face, within the depth tolerance";
}

// A hull of the box's 8 corners in the box's order is the box, bit for bit.
TEST(RigidContact, AHullOfTheBoxsCornersIsTheBoxBitForBit) {
    BodyState ends[2];
    for (int variant = 0; variant < 2; ++variant) {
        Simulation sim = make(tumbling_set());
        const glm::quat q = tilt();
        const BodyRef ref = put(sim, glm::vec3(0.0f, centre_for_lowest(q, kCubeHalf, 1.0f), 0.0f), glm::vec3(0.0f), q, cube_inv());
        ph::ShapeDesc d = variant == 0 ? ph::box_shape(glm::vec3(kCubeHalf)) : ph::hull_shape(cube_corners(kCubeHalf));
        d.restitution = 0.2f;
        d.friction = 0.5f;
        ASSERT_TRUE(shaped(sim, ref, d));
        ASSERT_TRUE(sim.step(3000).has_value());
        ends[variant] = body(sim, ref);
    }
    EXPECT_EQ(std::memcmp(&ends[0], &ends[1], sizeof(BodyState)), 0);
    EXPECT_GT(ends[0].pos.y, 0.0f);
}

// A sphere on a friction floor slides, then rolls (omega r = v, v = 5/7 of the launch), and ROLLS ON: no rolling resistance.
TEST(RigidContact, ASphereSlidesThenRollsAndRollsOn) {
    Simulation sim = make(tumbling_set());
    const float r = 0.1f;
    const BodyRef ref = put(sim, glm::vec3(0.0f, r, 0.0f), glm::vec3(3.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            glm::vec3(1.0f / (0.4f * r * r)));
    ph::ShapeDesc d = ph::sphere_shape(r);
    d.friction = 0.5f;
    d.restitution = 0.0f;
    ASSERT_TRUE(shaped(sim, ref, d));
    ASSERT_TRUE(sim.step(1000).has_value());
    BodyState b = body(sim, ref);
    const float rolling = 3.0f * 5.0f / 7.0f;
    EXPECT_NEAR(glm::length(b.vel), rolling, 0.02f * rolling);
    EXPECT_NEAR(glm::length(b.orient * b.omega_body) * r, glm::length(b.vel), 0.02f * rolling) << "omega r = v";
    ASSERT_TRUE(sim.step(3000).has_value());
    b = body(sim, ref);
    EXPECT_NEAR(glm::length(b.vel), rolling, 0.03f * rolling) << "a sphere rolls on";
}

// A capsule dropped slightly tilted lies down on both ends.
TEST(RigidContact, ACapsuleLiesDownOnBothEnds) {
    Simulation sim = make(tumbling_set());
    const glm::quat q = glm::angleAxis(glm::radians(5.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    const BodyRef ref = put(sim, glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(0.0f), q, glm::vec3(400.0f, 30.0f, 30.0f));
    ph::ShapeDesc d = ph::capsule_shape(glm::vec3(-0.3f, 0.0f, 0.0f), glm::vec3(0.3f, 0.0f, 0.0f), 0.05f);
    d.friction = 0.3f;
    d.restitution = 0.1f;
    ASSERT_TRUE(shaped(sim, ref, d));
    ASSERT_TRUE(sim.step(5000).has_value());
    const BodyState end = body(sim, ref);
    const glm::vec3 axis = end.orient * glm::vec3(1.0f, 0.0f, 0.0f);
    EXPECT_LT(std::abs(axis.y), 0.02f) << "lying flat on both ends";
    EXPECT_NEAR(end.pos.y, 0.05f, 0.01f);
    EXPECT_LT(glm::length(end.vel), 0.02f);
}

// The wall: a sphere thrown at a wall rebounds along the wall's normal at e of its speed; its tangential speed is untouched.
TEST(RigidContact, ASphereBouncesOffAWallAlongItsNormal) {
    Simulation sim = make(tumbling_set(), WorldOpts{.wall_x = 2.0f});
    const float r = 0.1f;
    const BodyRef ref = put(sim, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(4.0f, 0.0f, 1.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            glm::vec3(1.0f / (0.4f * r * r)));
    ph::ShapeDesc d = ph::sphere_shape(r);
    d.restitution = 0.6f;
    d.friction = 0.0f;
    ASSERT_TRUE(shaped(sim, ref, d));
    float rebound = 0.0f;
    for (int i = 0; i < 600 && rebound == 0.0f; ++i) {
        ASSERT_TRUE(sim.step(1).has_value());
        const BodyState b = body(sim, ref);
        if (b.vel.x < 0.0f) rebound = b.vel.x;
    }
    EXPECT_NEAR(rebound, -2.4f, 0.12f) << "e * 4 m/s back along the wall's normal";
    EXPECT_NEAR(body(sim, ref).vel.z, 1.5f, 1e-3f) << "frictionless: the tangential speed is untouched";
}

// The material precedence: a NEGATIVE row value takes the world's ContactParams, and a row value beats the world's.
TEST(RigidContact, TheRowsMaterialBeatsTheWorldsAndANegativeTakesTheWorlds) {
    const glm::vec3 half(0.2f, 0.05f, 0.2f);
    BodyState ends[3];
    const float row_mu[3] = {0.5f, -1.0f, 0.0f};
    const float world_mu[3] = {0.0f, 0.5f, 0.5f};
    for (int v = 0; v < 3; ++v) {
        Simulation sim = make(tumbling_set(), WorldOpts{.mu = world_mu[v], .e = 0.0f});
        const BodyRef ref = put(sim, glm::vec3(0.0f, half.y, 0.0f), glm::vec3(3.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                ph::box_inv_inertia(1.0f, half));
        ph::ShapeDesc d = ph::box_shape(half);
        d.friction = row_mu[v];
        d.restitution = -1.0f;
        ASSERT_TRUE(shaped(sim, ref, d));
        ASSERT_TRUE(sim.step(2000).has_value());
        ends[v] = body(sim, ref);
    }
    EXPECT_EQ(std::memcmp(&ends[0], &ends[1], sizeof(BodyState)), 0) << "row mu 0.5 on a frictionless world == row -1 on a mu 0.5 world";
    EXPECT_GT(ends[2].pos.x, 5.0f * ends[0].pos.x) << "a frictionless row on a mu 0.5 world slides on: the row wins";
}

// Determinism and the snapshot: two runs are bit-equal, and a snapshot taken mid-tumble restores to the same digest.
TEST(RigidContact, TwoRunsAreBitEqualAndASnapshotMidTumbleRestoresTheSameDigest) {
    uint64_t digests[2];
    for (int v = 0; v < 2; ++v) {
        Simulation sim = make(tumbling_set());
        const glm::quat q = tilt();
        const BodyRef ref = put(sim, glm::vec3(0.0f, centre_for_lowest(q, kCubeHalf, 1.0f), 0.0f), glm::vec3(0.0f), q, cube_inv());
        ASSERT_TRUE(shaped(sim, ref, cube()));
        ASSERT_TRUE(sim.step(3000).has_value());
        digests[v] = spade::testing::state_digest(sim);
    }
    EXPECT_EQ(digests[0], digests[1]);

    Simulation a = make(tumbling_set());
    const glm::quat q = tilt();
    const BodyRef ra = put(a, glm::vec3(0.0f, centre_for_lowest(q, kCubeHalf, 1.0f), 0.0f), glm::vec3(0.0f), q, cube_inv());
    ASSERT_TRUE(shaped(a, ra, cube()));
    ASSERT_TRUE(a.step(520).has_value());  // mid-tumble: the first impact is at about 0.45 s
    EXPECT_GT(glm::length(body(a, ra).omega_body), 0.1f);
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    Simulation b = make(tumbling_set());
    const BodyRef rb = put(b, glm::vec3(5.0f, 5.0f, 5.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f));
    (void)rb;
    ASSERT_TRUE(b.restore(*blob).has_value());
    ASSERT_TRUE(a.step(2000).has_value());
    ASSERT_TRUE(b.step(2000).has_value());
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b));
}

// A vehicle model with a shape: its body's row is written at spawn; a model without one is unchanged.
TEST(RigidContact, AModelsShapeIsWrittenToItsBodyAtSpawn) {
    spade::vehicles::QuadrotorParams p;
    p.name = "shaped";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.19f;
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
    spade::vehicles::ModelType with_shape = spade::vehicles::make_quadrotor(p).value();
    spade::vehicles::ModelType plain = with_shape;
    plain.name = "plain";
    with_shape.shape = ph::box_shape(glm::vec3(0.2f, 0.05f, 0.2f));

    Simulation sim = make(tumbling_set());
    const auto shaped_id = sim.register_model(with_shape);
    const auto plain_id = sim.register_model(plain);
    ASSERT_TRUE(shaped_id.has_value() && plain_id.has_value());
    spade::VehicleSpawn at;
    at.pos = glm::vec3(0.0f, 2.0f, 0.0f);
    const auto a = sim.spawn(0, *shaped_id, at);
    at.pos = glm::vec3(1.0f, 2.0f, 0.0f);
    const auto b = sim.spawn(0, *plain_id, at);
    ASSERT_TRUE(a.has_value() && b.has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    const auto ra = sim.body_shape_row(a->body);
    const auto rb = sim.body_shape_row(b->body);
    ASSERT_TRUE(ra.has_value()) << ra.error().context;
    ASSERT_TRUE(rb.has_value()) << rb.error().context;
    EXPECT_EQ(ra->kind, static_cast<uint32_t>(ph::ShapeKind::box));
    EXPECT_EQ(ra->count, 8u);
    EXPECT_EQ(rb->count, 0u) << "a model without a shape has none";
}

// ---- the band's CPU ensemble (docs/design/test-docs/2026-11-11-l68-device-band.md) ---------------------------------------------------------

// Nine CPU runs of the device-band scenario: the nominal and eight starts perturbed by (height, tilt) offsets of (+-1e-6 m, 0), (0, +-1e-6 rad) and
// (+-1e-6 m, +-1e-6 rad). The scenario must pass first: all nine settle on the same face, each within the tilt and height bands. If not, the tilt
// sits near a basin edge: use the next of the record's list (20, 30, 40 degrees) and say which. Then the sensitivity guard: each spread is at most half
// its band, so a change that makes the tumble more chaotic fails here, on the CPU.
TEST(RigidContact, TheDeviceBandScenarioSettlesOnOneFaceAcrossTheNineRunEnsemble) {
    struct Offset {
        float dh;
        float dtilt;
    };
    constexpr float e = 1e-6f;
    const Offset offsets[9] = {{0, 0}, {e, 0}, {-e, 0}, {0, e}, {0, -e}, {e, e}, {e, -e}, {-e, e}, {-e, -e}};
    Outcome runs[9];
    for (int i = 0; i < 9; ++i) runs[i] = run_cube(spade::compute::BackendKind::cpu, kScenarioTiltDeg, offsets[i].dh, offsets[i].dtilt);
    for (int i = 0; i < 9; ++i) {
        EXPECT_TRUE(runs[i].settled) << "run " << i << " did not settle at 6 s";
        EXPECT_EQ(runs[i].face, runs[0].face) << "run " << i << ": the tilt is near a basin edge; use the next of 20, 30, 40 degrees";
        EXPECT_LE(runs[i].tilt_deg, kTiltBandDeg) << "run " << i;
        EXPECT_GE(runs[i].centre_above, kHeightLow) << "run " << i;
        EXPECT_LE(runs[i].centre_above, kHeightHigh) << "run " << i;
    }
    float lateral = 0.0f;
    float yaw = 0.0f;
    int impacts = 0;
    for (int i = 0; i < 9; ++i) {
        for (int j = i + 1; j < 9; ++j) {
            lateral = std::max(lateral, glm::length(runs[i].lateral - runs[j].lateral));
            yaw = std::max(yaw, std::abs(heading_difference(runs[i].yaw_deg, runs[j].yaw_deg)));
            impacts = std::max(impacts, std::abs(runs[i].impacts - runs[j].impacts));
        }
    }
    std::printf("L68 ensemble (tilt %.0f deg): lateral spread %.5f m (band %.3f), yaw spread %.3f deg (band %.1f), impact spread %d (band %d), face %d\n",
                kScenarioTiltDeg, lateral, kLateralBand, yaw, kYawBandDeg, impacts, kImpactBand, runs[0].face);
    EXPECT_LE(lateral, 0.5f * kLateralBand) << "the sensitivity guard: the tumble has become more chaotic";
    EXPECT_LE(yaw, 0.5f * kYawBandDeg);
    EXPECT_LE(static_cast<float>(impacts), 0.5f * static_cast<float>(kImpactBand));
}

// CPU against itself: two runs of the scenario are memcmp-equal.
TEST(RigidContact, TheDeviceBandScenarioIsBitIdenticalOnTheCpuRunToRun) {
    const Outcome a = run_cube(spade::compute::BackendKind::cpu, kScenarioTiltDeg);
    const Outcome b = run_cube(spade::compute::BackendKind::cpu, kScenarioTiltDeg);
    EXPECT_EQ(std::memcmp(&a.end, &b.end, sizeof(BodyState)), 0);
    EXPECT_GT(a.impacts, 0) << "the cube hit the floor";
}

// The thin-bar limit (Spade's READY ask): eight corners at radius 0 miss a bar that falls between them. The library's gate frame is a 0.10 m
// bar (outer half 0.85, inner half 0.75); a cube of half extent 0.1 has corners 0.2 m apart, so a bar centred between them leaves each corner
// (0.2 - 0.1) / 2 = 0.05 m clear. Radius 0 falls through; a skin above that gap (0.06 m) is caught.
TEST(RigidContact, ACubeFallsThroughAThinBarAtRadiusZeroAndASkinAboveTheGapCatchesIt) {
    float centre_y[2];
    const float skins[2] = {0.0f, 0.06f};
    for (int v = 0; v < 2; ++v) {
        Simulation sim = make(tumbling_set(), WorldOpts{.bar = true});
        const BodyRef ref = put(sim, glm::vec3(0.0f, 0.9f, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), cube_inv());
        ph::ShapeDesc d = ph::box_shape(glm::vec3(kCubeHalf), skins[v]);
        d.restitution = 0.0f;
        d.friction = 0.5f;
        ASSERT_TRUE(shaped(sim, ref, d));
        ASSERT_TRUE(sim.step(2500).has_value());
        centre_y[v] = body(sim, ref).pos.y;
    }
    EXPECT_LT(centre_y[0], 0.2f) << "radius 0: the cube passed the bar and lies on the floor";
    EXPECT_GT(centre_y[1], 0.6f) << "skin 0.06 m > the 0.05 m gap: the bar holds it up";
}
