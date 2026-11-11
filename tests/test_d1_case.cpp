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

#include <cstdio>
#include <string>
#include <vector>

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

TEST(D1Case, TheDeepBoxHoldsTheDroneOffCentreOnTheCellAlignment) {
    // L63 (Cameron, 2026-11-11): 1.214 m below the drone, the same 0.354 m above it, in 240 x 392 x 168 at 4 mm.
    using spade::testing::kD1DeepGrid;
    EXPECT_EQ(spade::testing::d1_hold(kD1Grid), glm::vec3(0.0f)) << "kD1Grid keeps the centre, exactly";
    const auto q = spade::testing::d1_case(0.6, kD1DeepGrid);
    ASSERT_TRUE(q.has_value()) << q.error().context;
    ASSERT_TRUE(q->grid.has_value());
    EXPECT_EQ(q->grid->ny, 392u);
    EXPECT_EQ(q->hold, spade::testing::d1_hold(kD1DeepGrid));
    const auto resolved = spade::testing::resolve_with_grid(q->c, q->run_model, q->grid, q->hold);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->hold_from_centre, q->hold);
    EXPECT_EQ(resolved->base.ny, 392u);
    EXPECT_LT(resolved->base_bytes, af::kGridByteCap);

    // LiveRun's origin (live_run.cpp): -0.5 dx n - hold; the drone, the world origin, in the grid's cells.
    const af::GridShape& g = resolved->base;
    const glm::vec3 origin = -0.5f * g.dx *
                                 glm::vec3(static_cast<float>(g.nx), static_cast<float>(g.ny), static_cast<float>(g.nz)) -
                             resolved->hold_from_centre;
    const glm::vec3 cells = -origin / g.dx;
    EXPECT_NEAR(cells.x, 120.0f, 1e-3f) << "an x face";
    EXPECT_NEAR(cells.y, 303.5f, 1e-3f) << "a y cell centre, 1.214 m above the floor";
    EXPECT_NEAR(cells.z, 84.0f, 1e-3f) << "a z face";
    EXPECT_NEAR(origin.y + static_cast<float>(g.ny) * g.dx, 0.354f, 1e-5f) << "0.354 m to the top face, as kD1Grid";

    const auto d1 = spade::testing::d1_case(0.6);
    ASSERT_TRUE(d1.has_value());
    EXPECT_EQ(d1->hold, glm::vec3(0.0f)) << "the D1 grid keeps the centre";
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

// The heading is the wind's direction about +Y from the nose (+X): 0 a headwind moving -x, +90 from the left (-Z) so
// the air moves +z, -90 the other way. A heading that did nothing would put a wrong label on a clip, so this reads it
// back three ways: the case, the resolved ambient, and the air itself after a few steps on a small grid (the CPU
// step, no solids): the open faces carry the ambient in, and the mean velocity leans to the wind's side at every
// heading. Mutation: the builder not passing the heading on turns it red (both side headings read a headwind).
TEST(D1Case, TheHeadingPutsTheWindAlongTheBodysSidewaysAxisWithTheStatedSign) {
    struct Row {
        float heading;
        glm::vec3 air;  // the direction the air moves, unit
    };
    const Row rows[] = {{0.0f, {-1.0f, 0.0f, 0.0f}}, {90.0f, {0.0f, 0.0f, 1.0f}}, {-90.0f, {0.0f, 0.0f, -1.0f}}};
    const af::GridShape small{24, 24, 24, 0.02f};
    for (const Row& row : rows) {
        const auto q = spade::testing::d1_case(0.1, small, false, row.heading);
        ASSERT_TRUE(q.has_value()) << q.error().context;
        EXPECT_EQ(q->c.flight.heading_deg, row.heading) << "the heading reached the case";
        const auto resolved = spade::testing::resolve_with_grid(q->c, q->run_model, q->grid);
        ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
        const glm::vec3 want = 3.0f * row.air;
        EXPECT_NEAR(resolved->domain.ambient.x, want.x, 1e-5f) << "heading " << row.heading;
        EXPECT_NEAR(resolved->domain.ambient.y, want.y, 1e-5f) << "heading " << row.heading;
        EXPECT_NEAR(resolved->domain.ambient.z, want.z, 1e-5f) << "heading " << row.heading;

        auto grid = af::make_grid(small, resolved->domain);
        ASSERT_TRUE(grid.has_value()) << grid.error().context;
        const af::FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = 2, .threads = 1};
        auto scratch = af::make_fluid_scratch(*grid, resolved->domain, cfg);
        ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
        for (int t = 0; t < 24; ++t) {
            const af::FluidHealth health = af::fluid_step(*grid, resolved->domain, cfg, *scratch);
            ASSERT_EQ(health.fault, af::FluidFault::none) << "step " << t;
        }
        const auto mean = [](const std::vector<float>& f) {
            double s = 0.0;
            for (const float v : f) s += v;
            return static_cast<float>(s / static_cast<double>(f.size()));
        };
        const glm::vec3 m(mean(grid->u), mean(grid->v), mean(grid->w));
        const float along = glm::dot(m, row.air);
        const float across = glm::length(m - along * row.air);
        std::printf("[D1Case] heading %+.0f: mean air (%.4f, %.4f, %.4f) m/s, along the wind %.4f, across %.4f\n",
                    row.heading, m.x, m.y, m.z, along, across);
        EXPECT_GT(along, 1.0f) << "the air moves with the wind, at heading " << row.heading;
        // Across the wind the mean is the open faces' entrainment (0.27 against 1.18 at the side headings).
        EXPECT_LT(across, 0.5f * along) << "and not across it, at heading " << row.heading;
    }
}

}  // namespace
