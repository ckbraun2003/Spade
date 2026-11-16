// The smoke tour's chapter 4, "Bodies": L68's contact tumbling, through Core's public API only (Simulation with
// rigid_contact_module() in the set, set_body_shape()). sandbox/bodies_sim.hpp is a row of four shapes dropped together
// onto the ground, spaced so that none touches another (body-body contact is not in L68): a sphere that is thrown and
// rolls on, a cube that tumbles and settles on a face, a capsule that lies down, and a drone-sized hull that lands flat.
// The tour draws them (bodies_session) and checks what these tests check, without a window: each shape does what its
// shape says, and two runs of the row are the same bytes.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/geometric.hpp>

#include "../sandbox/bodies_sim.hpp"

namespace {

namespace bodies = spade::sandbox::bodies;

constexpr uint32_t kEightSeconds = 8000u;  // 1 kHz: the cube's settling in Core's own test takes this long

bodies::BodiesSim made() {
    auto sim = bodies::BodiesSim::create();
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    return std::move(*sim);
}

TEST(BodiesSim, TheRowIsFourDistinctShapesSpacedSoNoneCanTouchAnother) {
    const std::vector<bodies::Piece> row = bodies::row();
    ASSERT_EQ(row.size(), 4u);
    EXPECT_EQ(row[0].kind, bodies::Kind::sphere);
    EXPECT_EQ(row[1].kind, bodies::Kind::box);
    EXPECT_EQ(row[2].kind, bodies::Kind::capsule);
    EXPECT_EQ(row[3].kind, bodies::Kind::hull);
    for (std::size_t i = 0; i < row.size(); ++i) {
        EXPECT_FALSE(row[i].name.empty());
        for (std::size_t j = i + 1; j < row.size(); ++j) {
            EXPECT_NE(row[i].name, row[j].name);
            // Every piece falls (or rolls) along its own line x = const, so the gap in x between two pieces, less their
            // bounding radii, is the gap between them for good.
            const float gap = std::abs(row[i].start.x - row[j].start.x) - row[i].bound - row[j].bound;
            EXPECT_GE(gap, 0.15f) << row[i].name << " and " << row[j].name << " must never touch";
        }
    }
    // The sphere's path is its own line too: thrown along z, not along the row.
    EXPECT_EQ(row[0].vel.x, 0.0f);
    EXPECT_GT(glm::length(row[0].vel), 0.5f);
}

// What each shape says. The sphere is thrown and rolls on at 5/7 of its throw (it slides until v = omega r); the cube
// settles on a face; the capsule lies down on both ends; the hull lands flat on its feet.
TEST(BodiesSim, AfterEightSecondsEachShapeIsAtRestOrRollingAsItsShapeSays) {
    bodies::BodiesSim sim = made();
    ASSERT_EQ(sim.pieces().size(), 4u);
    ASSERT_TRUE(sim.step(kEightSeconds).has_value());
    for (std::size_t i = 0; i < sim.pieces().size(); ++i) {
        const bodies::Verdict v = sim.verdict(i);
        EXPECT_TRUE(v.ok) << sim.pieces()[i].name << ": " << v.detail;
        EXPECT_FALSE(v.detail.empty());
    }
}

// A check that cannot fail checks nothing: before the pieces have landed, they are not at rest on anything.
TEST(BodiesSim, TheVerdictsAreNotVacuousBeforeThePiecesHaveLanded) {
    bodies::BodiesSim sim = made();
    ASSERT_EQ(sim.pieces().size(), 4u);
    EXPECT_FALSE(sim.verdict(1).ok) << "the cube, at the start";
    ASSERT_TRUE(sim.step(100u).has_value());
    for (const std::size_t i : {std::size_t{1}, std::size_t{2}, std::size_t{3}}) {
        EXPECT_FALSE(sim.verdict(i).ok) << sim.pieces()[i].name << " is still falling at 0.1 s: " << sim.verdict(i).detail;
    }
}

TEST(BodiesSim, TwoRunsOfTheRowAreTheSameBytes) {
    bodies::BodiesSim a = made();
    bodies::BodiesSim b = made();
    ASSERT_TRUE(a.step(kEightSeconds).has_value());
    ASSERT_TRUE(b.step(kEightSeconds).has_value());
    EXPECT_EQ(a.bytes(), b.bytes());
    EXPECT_FALSE(a.bytes().empty());
    bodies::BodiesSim c = made();
    ASSERT_TRUE(c.step(kEightSeconds - 1u).has_value());
    EXPECT_NE(a.bytes(), c.bytes()) << "one step less is a different state: the bytes see the run";
}

// The window advances by frames: whole milliseconds of the sim, the rest carried to the next frame, so 480 frames of a
// sixtieth are the same 8000 steps as stepping them at once.
TEST(BodiesSim, AdvancingByFramesIsSteppingByMilliseconds) {
    bodies::BodiesSim framed = made();
    for (int frame = 0; frame < 480; ++frame) ASSERT_TRUE(framed.advance(1.0f / 60.0f).has_value());
    EXPECT_NEAR(static_cast<double>(framed.steps()), 8000.0, 2.0);
    bodies::BodiesSim stepped = made();
    ASSERT_TRUE(stepped.step(static_cast<uint32_t>(framed.steps())).has_value());
    EXPECT_EQ(framed.bytes(), stepped.bytes());
}

}  // namespace
