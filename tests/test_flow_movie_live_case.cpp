// ---------------------------------------------------------------------------
// test_flow_movie_live_case.cpp -- tools/flow_movie/live_case.hpp's D1 films carry the drone's hold in the box (L63).
// Interface's ResolvedCase::hold_from_centre puts the drone at the world origin and the box's centre at -hold; the
// deeper website box (tests/d1_case.hpp's kD1DeepGrid, the floor 1.214 m below the drone) needs it, and a film that
// resolves its case through resolve_live() must pass it on, or the box is centred on the drone and the cameras' floor
// (web_masters.hpp's kDeepFloorY) is not where the solve puts it.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <string>

#include <glm/glm.hpp>

#include "../tools/flow_movie/live_case.hpp"
#include "../tools/flow_movie/web_masters.hpp"

namespace {

using spade::testing::kD1DeepGrid;
using spade::testing::kD1Grid;

// The grid's minimum corner in the world frame, with LiveRun's own operations (engine/analysis/live_run.cpp).
glm::vec3 grid_origin(const spade::analysis::ResolvedCase& r) {
    const spade::physics::airflow::GridShape& g = r.base;
    return -0.5f * g.dx * glm::vec3(static_cast<float>(g.nx), static_cast<float>(g.ny), static_cast<float>(g.nz)) -
           r.hold_from_centre;
}

}  // namespace

TEST(FlowMovieLiveCase, TheDeepBoxFilmResolvesWithTheDronesHold) {
    const auto lc = flow_movie::d1_still_air_live_case(0.01, kD1DeepGrid);
    ASSERT_TRUE(lc.has_value()) << lc.error().context;
    const auto resolved = flow_movie::resolve_live(*lc);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->hold_from_centre, spade::testing::d1_hold(kD1DeepGrid));
    EXPECT_EQ(resolved->base.ny, 392u);
    // Where the solve puts the box: the floor 1.214 m below the drone (the cameras' kDeepFloorY) and the top face 0.354 m above.
    const glm::vec3 origin = grid_origin(*resolved);
    EXPECT_NEAR(origin.y, flow_movie::web::kDeepFloorY, 1e-4f);
    EXPECT_NEAR(origin.y + resolved->base.dx * static_cast<float>(resolved->base.ny), 0.354f, 1e-4f);
    EXPECT_NEAR(origin.x, -flow_movie::web::kBoxHalfX, 1e-4f);
    EXPECT_NEAR(origin.z, -flow_movie::web::kBoxHalfZ, 1e-4f);
}

TEST(FlowMovieLiveCase, TheOtherD1FilmsOnTheDeepBoxCarryItToo) {
    // The web case (the clips' own builder) and the crosswind one go through the same hold.
    const auto web_still = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f);
    ASSERT_TRUE(web_still.has_value()) << web_still.error().context;
    const auto side = flow_movie::d1_live_case(0.01, kD1DeepGrid, true, 90.0f);
    ASSERT_TRUE(side.has_value()) << side.error().context;
    for (const flow_movie::LiveCase* lc : {&*web_still, &*side}) {
        const auto resolved = flow_movie::resolve_live(*lc);
        ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
        EXPECT_EQ(resolved->hold_from_centre, spade::testing::d1_hold(kD1DeepGrid)) << lc->name;
    }
}

TEST(FlowMovieLiveCase, AHoldOfZeroIsTheCentredRunBitForBit) {
    const auto lc = flow_movie::d1_still_air_live_case(0.01, kD1Grid);
    ASSERT_TRUE(lc.has_value()) << lc.error().context;
    const auto resolved = flow_movie::resolve_live(*lc);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->hold_from_centre, glm::vec3(0.0f));
    // LiveRun's origin with the hold subtracted is exactly the centred one (subtracting 0 is exact).
    const spade::physics::airflow::GridShape& g = resolved->base;
    const glm::vec3 centred =
        -0.5f * g.dx * glm::vec3(static_cast<float>(g.nx), static_cast<float>(g.ny), static_cast<float>(g.nz));
    const glm::vec3 origin = grid_origin(*resolved);
    EXPECT_EQ(origin.x, centred.x);
    EXPECT_EQ(origin.y, centred.y);
    EXPECT_EQ(origin.z, centred.z);
    // A film with no grid of its own (the cube's) holds at zero too.
    const auto quad = flow_movie::quad_live_case(0.01);
    ASSERT_TRUE(quad.has_value()) << quad.error().context;
    const auto cube = flow_movie::resolve_live(*quad);
    ASSERT_TRUE(cube.has_value()) << cube.error().context;
    EXPECT_EQ(cube->hold_from_centre, glm::vec3(0.0f));
}

TEST(FlowMovieLiveCase, TheCachesMarkNamesTheHoldOnlyWhenThereIsOne) {
    // A cache made with the drone off centre is not the centred box's, even on the same grid: the mark names the hold. A
    // centred film's mark is as it was before the hold existed (no new words), so the caches already on disk still match.
    const auto deep = flow_movie::d1_still_air_live_case(0.01, kD1DeepGrid);
    ASSERT_TRUE(deep.has_value()) << deep.error().context;
    EXPECT_NE(deep->identity.find("hold"), std::string::npos) << deep->identity;
    const auto centred = flow_movie::d1_still_air_live_case(0.01, kD1Grid);
    ASSERT_TRUE(centred.has_value()) << centred.error().context;
    EXPECT_EQ(centred->identity.find("hold"), std::string::npos) << centred->identity;
    const auto web_deep = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f);
    ASSERT_TRUE(web_deep.has_value()) << web_deep.error().context;
    EXPECT_NE(web_deep->identity.find("hold"), std::string::npos) << web_deep->identity;
}
