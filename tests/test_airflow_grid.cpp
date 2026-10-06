// The airflow solver core's grid (physics plan 2026-10-05-airflow-solver-core-plan.md,
// Task 1): a MAC grid, its staggered arrays and the domain faces' configuration.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "physics/airflow/mac_grid.hpp"

namespace {

using spade::Code;
using spade::physics::airflow::DomainBc;
using spade::physics::airflow::FaceBc;
using spade::physics::airflow::GridShape;
using spade::physics::airflow::make_grid;

TEST(AirflowGrid, SizesItsArraysByTheStaggeredLayout) {
    const auto g = make_grid(GridShape{4, 3, 2, 0.1f}, DomainBc{});
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->u.size(), 5u * 3u * 2u) << "x-faces: (nx + 1) ny nz";
    EXPECT_EQ(g->v.size(), 4u * 4u * 2u) << "y-faces: nx (ny + 1) nz";
    EXPECT_EQ(g->w.size(), 4u * 3u * 3u) << "z-faces: nx ny (nz + 1)";
    EXPECT_EQ(g->p.size(), 24u);
    for (float x : g->u) EXPECT_EQ(x, 0.0f);
    for (float x : g->p) EXPECT_EQ(x, 0.0f);
}

TEST(AirflowGrid, IndexesEveryArrayWithIInnermost) {
    const GridShape s{4, 3, 2, 0.1f};
    using namespace spade::physics::airflow;
    EXPECT_EQ(u_index(s, 4, 2, 1), 4u + 5u * (2u + 3u * 1u));
    EXPECT_EQ(v_index(s, 3, 3, 1), 3u + 4u * (3u + 4u * 1u));
    EXPECT_EQ(w_index(s, 3, 2, 2), 3u + 4u * (2u + 3u * 2u));
    EXPECT_EQ(cell_index(s, 3, 2, 1), 3u + 4u * (2u + 3u * 1u));
}

TEST(AirflowGrid, VisitsEachComponentsSamplesAtTheirPositions) {
    const GridShape s{2, 1, 1, 0.5f};
    std::vector<float> xs;
    spade::physics::airflow::for_each_sample(s, 0, [&](std::size_t idx, glm::vec3 x) {
        EXPECT_EQ(idx, xs.size());
        xs.push_back(x.x);
        EXPECT_EQ(x.y, 0.25f) << "a u-face sits mid-cell in y";
        EXPECT_EQ(x.z, 0.25f);
    });
    EXPECT_EQ(xs, (std::vector<float>{0.0f, 0.5f, 1.0f})) << "u-faces at i dx, i = 0..nx";
}

TEST(AirflowGrid, RefusesAShapeItCannotStep) {
    const auto refused = [](GridShape s, DomainBc bc, const char* what) {
        const auto g = make_grid(s, bc);
        if (g.has_value()) return testing::AssertionFailure() << "accepted";
        if (g.error().code != Code::invalid_argument) return testing::AssertionFailure() << "code";
        if (g.error().context.find(what) == std::string::npos) {
            return testing::AssertionFailure() << g.error().context;
        }
        return testing::AssertionSuccess();
    };
    EXPECT_TRUE(refused({0, 4, 4, 0.1f}, {}, "dimension"));
    EXPECT_TRUE(refused({4, 4, 4, 0.0f}, {}, "dx"));
    EXPECT_TRUE(refused({4, 4, 4, std::numeric_limits<float>::quiet_NaN()}, {}, "dx"));
    DomainBc unpaired;
    unpaired.face[0] = FaceBc::periodic;
    EXPECT_TRUE(refused({4, 4, 4, 0.1f}, unpaired, "x+"));
    DomainBc odd;
    odd.face[0] = odd.face[1] = FaceBc::periodic;
    EXPECT_TRUE(refused({3, 4, 4, 0.1f}, odd, "even"));
    DomainBc slab;
    slab.face[4] = slab.face[5] = FaceBc::periodic;
    EXPECT_TRUE(make_grid({4, 4, 1, 0.1f}, slab).has_value()) << "a periodic axis of size 1 is a quasi-2D slab";
}

}  // namespace
