// ---------------------------------------------------------------------------
// test_d1_case.cpp -- tests/d1_case.hpp, D1's one builder (the lead,
// 2026-10-27): what flow_movie's D1 film, Physics' B7 and Test/Docs' harness
// all fly. The case is the record's (tests/data/airflow/records/d1-hover.md):
// the library quad with its ten rigid parts in the flow, armed at its hover
// throttle from the start, in a 3 m/s wind, stepped every 0.38 ms on
// 240 x 177 x 168 at 4 mm.
//
//   D1Case  the case as built and as resolved on kD1Grid; a grid past the cap
//           refused by name; without its parts in the flow (B7's GPU film),
//           the same case with the parts drawn only.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <string>

#include "d1_case.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
using spade::testing::kD1Grid;

TEST(D1Case, TheCaseIsTheRecordsAndResolvesOnKD1Grid) {
    const auto q = spade::testing::d1_case(0.6);
    ASSERT_TRUE(q.has_value()) << q.error().context;
    EXPECT_EQ(q->run_model.parts.size(), 10u) << "the ten rigid parts are in the flow";
    EXPECT_EQ(q->draw_model.parts.size(), 10u);
    EXPECT_EQ(q->c.time.step, 0.00038f);
    EXPECT_EQ(q->c.flight.kind, an::FlightKind::hover_in_wind);
    EXPECT_EQ(q->c.flight.speed, 3.0f);
    EXPECT_EQ(q->program_name, "held at hover");
    ASSERT_EQ(q->program.points.size(), 1u) << "armed at hover from the start";
    EXPECT_TRUE(q->program.points[0].controls.armed);
    EXPECT_EQ(q->program.points[0].controls.throttle, q->trim.hover);
    ASSERT_TRUE(q->grid.has_value());
    EXPECT_EQ(q->grid->nx, 240u);
    EXPECT_EQ(q->grid->ny, 177u);
    EXPECT_EQ(q->grid->nz, 168u);
    EXPECT_EQ(q->c.grid.spacing, 0.004f);
    EXPECT_EQ(q->c.grid.box, 0.004f * 240.0f) << "the box names the grid's longest side, 240 x 4 mm";

    const auto resolved = spade::testing::resolve_with_grid(q->c, q->run_model, q->grid);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->base.nx, kD1Grid.nx);
    EXPECT_EQ(resolved->base.ny, kD1Grid.ny);
    EXPECT_EQ(resolved->base.nz, kD1Grid.nz);
    EXPECT_EQ(resolved->base.dx, kD1Grid.dx);
    EXPECT_LE(resolved->base_bytes, af::kGridByteCap);
}

TEST(D1Case, AGridPastTheCapIsRefusedByName) {
    const auto q = spade::testing::d1_case(0.1);
    ASSERT_TRUE(q.has_value()) << q.error().context;
    const af::GridShape huge{2000, 2000, 2000, 0.004f};
    const auto resolved = spade::testing::resolve_with_grid(q->c, q->run_model, huge);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_NE(resolved.error().context.find("past kGridByteCap"), std::string::npos) << resolved.error().context;
}

TEST(D1Case, WithoutItsPartsTheyAreDrawnOnly) {
    const auto with = spade::testing::d1_case(0.6);
    const auto without = spade::testing::d1_case(0.6, kD1Grid, false);
    ASSERT_TRUE(with.has_value()) << with.error().context;
    ASSERT_TRUE(without.has_value()) << without.error().context;
    EXPECT_TRUE(without->run_model.parts.empty()) << "no part in the flow";
    EXPECT_EQ(without->draw_model.parts.size(), 10u) << "all ten drawn";
    EXPECT_EQ(without->run_model.rotors.size(), with->run_model.rotors.size());
    EXPECT_EQ(without->trim.hover, with->trim.hover);
    ASSERT_EQ(without->program.points.size(), 1u);
    EXPECT_EQ(without->program.points[0].controls.throttle, with->program.points[0].controls.throttle);
    const auto resolved = spade::testing::resolve_with_grid(without->c, without->run_model, without->grid);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->base.nx, kD1Grid.nx);
}

}  // namespace
