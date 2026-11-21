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

TEST(FlowMovieLiveCase, ABoxShorterThanD1sStaysCentredAsAGridFlagAlwaysDid) {
    // d1_hold() is relative to kD1Grid's 177 rows, so a shorter box would get a NEGATIVE hold: the gate's 112 x 80 x 112
    // box (GpuFlowMovie.GpuD1RendersTwoFrames, --grid 112x80x112) put the drone 0.194 m below the grid's centre, outside
    // the box ("rotor 0: actuator disc: its kernel support leaves the grid"). Only a deeper box moves the drone.
    for (const spade::physics::airflow::GridShape grid :
         {spade::physics::airflow::GridShape{112, 80, 112, 0.004f}, spade::physics::airflow::GridShape{224, 160, 168, 0.004f}}) {
        const auto lc = flow_movie::d1_live_case(0.01, grid, false);
        ASSERT_TRUE(lc.has_value()) << lc.error().context;
        const auto resolved = flow_movie::resolve_live(*lc);
        ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
        EXPECT_EQ(resolved->hold_from_centre, glm::vec3(0.0f)) << grid.ny << " rows";
        EXPECT_EQ(lc->identity.find("hold"), std::string::npos) << lc->identity;
    }
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

// L86c (the lead, from Physics' harness box that carried D1's own 3 m/s ambient without saying so): a still-air clip's
// log, run header and caption name the ambient, the faces, the grid, h and Cs, so a stray wind cannot slip in, and a still
// clip whose resolved case has any wind is refused by name.
TEST(FlowMovieLiveCase, TheStillAirClipsResolvedDomainHasNoWindAndItsLineSaysSo) {
    const auto lc = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f);
    ASSERT_TRUE(lc.has_value()) << lc.error().context;
    const auto resolved = flow_movie::resolve_live(*lc);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->domain.ambient, glm::vec3(0.0f)) << "still air: no ambient wind";
    EXPECT_EQ(flow_movie::still_air_refusal(*resolved), "");
    const std::string line = flow_movie::domain_line(*resolved, 0.0002f, 0.17f);
    EXPECT_NE(line.find("ambient (0, 0, 0) m/s"), std::string::npos) << line;
    EXPECT_NE(line.find("x- open, x+ open, y- open, y+ open, z- open, z+ open"), std::string::npos) << line;
    EXPECT_NE(line.find("240 x 392 x 168 cells at 4 mm"), std::string::npos) << line;
    EXPECT_NE(line.find("step 0.2 ms"), std::string::npos) << line;
    EXPECT_NE(line.find("Cs 0.17"), std::string::npos) << line;
    EXPECT_NE(lc->domain.find("ambient (0, 0, 0) m/s"), std::string::npos) << "the case carries it for the caption";
    EXPECT_NE(lc->notes.find("ambient (0, 0, 0) m/s"), std::string::npos) << lc->notes;
}

TEST(FlowMovieLiveCase, ASideWindClipSaysItsWindAndAStillClipWithAWindIsRefused) {
    const auto side = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 90.0f, 0.0002f, 0.17f);
    ASSERT_TRUE(side.has_value()) << side.error().context;
    const auto resolved = flow_movie::resolve_live(*side);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_NEAR(resolved->domain.ambient.x, 0.0f, 1e-5f);  // heading 90: the air moves +z at 3 m/s (cos32 leaves ~3e-7)
    EXPECT_NEAR(resolved->domain.ambient.y, 0.0f, 1e-5f);
    EXPECT_NEAR(resolved->domain.ambient.z, 3.0f, 1e-5f);
    EXPECT_NE(side->domain.find("ambient (0, 0, 3) m/s"), std::string::npos) << side->domain;
    EXPECT_EQ(flow_movie::still_air_refusal(*resolved).empty(), false) << "the side wind is not still air";
    // The refusal names the wind it found.
    spade::analysis::ResolvedCase windy = *resolved;
    windy.domain.ambient = glm::vec3(-3.0f, 0.0f, 0.0f);  // Physics' harness box carried this
    const std::string why = flow_movie::still_air_refusal(windy);
    EXPECT_NE(why.find("(-3, 0, 0) m/s"), std::string::npos) << why;
    EXPECT_NE(why.find("still"), std::string::npos) << why;
}

// The hover throttle is the same in still air and in D1's 3 m/s case: the trim finds the hover from the drone's mass, the
// air's density and the charge (analysis/live_run.cpp's hover_point), never from the case's wind. So S1, S8 and S2 hover at
// the still-air trim whatever d1_case() set its wind to when it trimmed (a characterization: green from the start).
TEST(FlowMovieLiveCase, TheHoverTrimDoesNotDependOnTheCasesWind) {
    const auto windy = spade::testing::d1_case(0.01, kD1Grid);  // flight.kind hover_in_wind, 3 m/s
    ASSERT_TRUE(windy.has_value()) << windy.error().context;
    ASSERT_EQ(windy->c.flight.speed, 3.0f);
    const auto still = flow_movie::d1_still_air_live_case(0.01, kD1Grid);
    ASSERT_TRUE(still.has_value()) << still.error().context;
    ASSERT_EQ(still->c.flight.speed, 0.0f);
    const auto in_still = spade::analysis::showcase_trim(still->c, still->run_model);
    ASSERT_TRUE(in_still.has_value()) << in_still.error().context;
    EXPECT_EQ(windy->trim.hover, in_still->hover) << "the same hover throttle, bit for bit";
    EXPECT_EQ(windy->trim.climb, in_still->climb);
    ASSERT_EQ(still->program.points.size(), 1u);
    EXPECT_EQ(still->program.points.front().controls.throttle, in_still->hover) << "the film flies the still-air trim";
}

// L63 made the deeper box the still-air clips' own (S1, S2, S8: the cameras' floor is 1.214 m under the drone), so a master
// run without --grid must not fall back to the shallow box and show the floor (the lead, 2026-11-17). The side-wind clips
// (S3, S4) keep the shallow box; an explicit --grid is always what it says.
TEST(FlowMovieLiveCase, TheStillAirClipsDefaultToTheDeepBoxAndAnExplicitGridWins) {
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt).ny, kD1DeepGrid.ny);
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt).nx, kD1DeepGrid.nx);
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt).nz, kD1DeepGrid.nz);
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt).dx, kD1DeepGrid.dx);
    EXPECT_EQ(flow_movie::d1_web_grid(false, std::nullopt).ny, kD1Grid.ny) << "the side wind keeps the shallow box";
    const spade::physics::airflow::GridShape small{112, 80, 112, 0.004f};
    EXPECT_EQ(flow_movie::d1_web_grid(true, small).ny, 80u) << "--grid wins on a still-air clip";
    EXPECT_EQ(flow_movie::d1_web_grid(false, small).ny, 80u) << "--grid wins on a side-wind clip";
}

// L106c (the lead, 2026-11-19): the website film at the L106 config, the X500's 2.5 mm box (270 x 237 x 270) with the drone held where Physics'
// harness holds it (d1_x500_hold()). --cell-mm 2.5 selects both; with no --cell-mm everything above is as it was.
TEST(FlowMovieLiveCase, TheCellOptionSelectsTheX500sBoxAndItsHoldAndNothingElseChanges) {
    const auto box = flow_movie::d1_web_grid(true, std::nullopt, 2.5f);
    EXPECT_EQ(box.nx, spade::testing::kD1X500Grid.nx);
    EXPECT_EQ(box.ny, spade::testing::kD1X500Grid.ny);
    EXPECT_EQ(box.nz, spade::testing::kD1X500Grid.nz);
    EXPECT_EQ(box.dx, spade::testing::kD1X500Grid.dx);
    const auto hold = flow_movie::d1_web_hold(2.5f);
    ASSERT_TRUE(hold.has_value());
    EXPECT_EQ(*hold, spade::testing::d1_x500_hold());
    EXPECT_FALSE(flow_movie::d1_web_hold(std::nullopt).has_value()) << "no --cell-mm: the case's own hold";
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt, std::nullopt).dx, kD1DeepGrid.dx) << "the default box is the deep one";
    EXPECT_EQ(flow_movie::d1_web_grid(true, std::nullopt, std::nullopt).ny, kD1DeepGrid.ny);
    const spade::physics::airflow::GridShape small{112, 80, 112, 0.004f};
    EXPECT_EQ(flow_movie::d1_web_grid(true, small, std::nullopt).ny, 80u) << "--grid still wins when there is no --cell-mm";
}

// The OpenFrame-5F's box (lead, 2026-11-21): the FPV harness' own 1.25 mm box, 252 x 237 x 284 (x along the kit's forward, z across), held where d1_fpv_hold(hub height)
// holds it: the drone entry declares the cell and the hub height above its design origin and the box and hold follow; the X500's stay exactly as above.
TEST(FlowMovieLiveCase, TheFpvCellSelectsTheFpvBoxAndItsHoldFromTheHubHeight) {
    const auto box = flow_movie::d1_web_grid(true, std::nullopt, 1.25f);
    EXPECT_EQ(box.nx, spade::testing::kD1FpvGrid.nx);
    EXPECT_EQ(box.ny, spade::testing::kD1FpvGrid.ny);
    EXPECT_EQ(box.nz, spade::testing::kD1FpvGrid.nz);
    EXPECT_EQ(box.dx, spade::testing::kD1FpvGrid.dx);
    EXPECT_EQ(box.nx, 252u);
    EXPECT_EQ(box.nz, 284u) << "x is the kit's forward, z across";
    const auto at_origin = flow_movie::d1_web_hold(1.25f, 0.0f);
    ASSERT_TRUE(at_origin.has_value());
    EXPECT_EQ(*at_origin, spade::testing::d1_fpv_hold(0.0f));
    EXPECT_NEAR(at_origin->y, 0.0525f, 1.0e-6f) << "a hub at the origin: Physics' 42 rows";
    const auto raised = flow_movie::d1_web_hold(1.25f, 0.031f);
    ASSERT_TRUE(raised.has_value());
    EXPECT_EQ(*raised, spade::testing::d1_fpv_hold(0.031f)) << "the hold follows the entry's hub height";
    // the X500's hold takes no hub height: whatever is passed, it is d1_x500_hold()
    ASSERT_TRUE(flow_movie::d1_web_hold(2.5f, 0.031f).has_value());
    EXPECT_EQ(*flow_movie::d1_web_hold(2.5f, 0.031f), spade::testing::d1_x500_hold());
}

// The model-file parameter of D1's builder (the lead, 2026-11-17: Cameron wants a real drone model in the web clips). The
// default is today's reference quad, so every existing D1 film and test is the film it was; another library drone is built
// the same way, its parts in the flow, held at its own hover throttle.
TEST(FlowMovieLiveCase, TheDefaultD1CaseIsTheReferenceQuadAndAnotherLibraryDroneIsBuiltTheSameWay) {
    const auto plain = spade::testing::d1_case(0.01, kD1Grid);
    const auto named = spade::testing::d1_case(0.01, kD1Grid, true, 0.0f, spade::testing::kLibraryQuad);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    ASSERT_TRUE(named.has_value()) << named.error().context;
    EXPECT_EQ(plain->c.model.file, named->c.model.file) << "the default is the reference quad";
    EXPECT_NE(plain->c.model.file.find("reference_quad"), std::string::npos) << plain->c.model.file;
    EXPECT_EQ(plain->trim.hover, named->trim.hover);
    EXPECT_EQ(plain->run_model.rotors.size(), named->run_model.rotors.size());

    const auto phantom =
        spade::testing::d1_case(0.01, kD1Grid, true, 0.0f, "library/drones/phantom3_class.model.yaml");
    ASSERT_TRUE(phantom.has_value()) << phantom.error().context;
    EXPECT_NE(phantom->c.model.file.find("phantom3_class"), std::string::npos) << phantom->c.model.file;
    EXPECT_EQ(phantom->run_model.rotors.size(), 4u);
    EXPECT_NEAR(phantom->run_model.rotors.front().radius, 0.1194f, 1e-4f);
    EXPECT_FALSE(phantom->run_model.parts.empty()) << "the parts are in the flow";
    EXPECT_GT(phantom->trim.hover, 0.0f);
    EXPECT_LT(phantom->trim.hover, 1.0f);
    EXPECT_NE(phantom->trim.hover, plain->trim.hover) << "a 1.28 kg drone hovers at its own throttle";
}

TEST(FlowMovieLiveCase, TheWebCaseTakesTheDroneAndANamelessFileIsRefused) {
    const auto phantom = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f,
                                                      "library/drones/phantom3_class.model.yaml");
    ASSERT_TRUE(phantom.has_value()) << phantom.error().context;
    EXPECT_EQ(phantom->run_model.rotors.size(), 4u);
    const auto plain = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_NE(phantom->identity, plain->identity) << "the cache's mark tells the two drones apart (model and parts hashes)";
    const auto missing = flow_movie::d1_web_live_case(0.01, kD1DeepGrid, 0.0f, 0.0002f, 0.17f,
                                                      "library/drones/no_such_drone.model.yaml");
    EXPECT_FALSE(missing.has_value());
}
