// check_grid() (the N5b agreement with Core, 2026-10-20): one rule, shared by
// make_grid(), make_fluid_scratch(), set_solids() and the scene validator,
// for whether a grid of a shape and its domain faces can be made and stepped,
// with or without solids (TD-9). Allocation-free, so a validator can ask it of
// a grid it never builds.
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::Code;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

[[nodiscard]] DomainBc walls() {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    return bc;
}

// make_grid()'s refusals are check_shape()'s, word for word, and
// check_grid() gives the same with or without solids: one rule. Mutation: a
// rule only make_grid() keeps.
TEST(AirflowCheckGrid, TheShapeRulesAreMakeGridsWordForWord) {
    DomainBc unpaired;
    unpaired.face[kXMinus] = FaceBc::periodic;
    DomainBc odd;
    odd.face[kXMinus] = odd.face[kXPlus] = FaceBc::periodic;
    const struct Case {
        GridShape shape;
        DomainBc bc;
        Code code;
        std::string what;
    } cases[] = {
        {{0, 4, 4, 0.1f}, {}, Code::invalid_argument, "dimension"},
        {{4, 4, 4, 0.0f}, {}, Code::invalid_argument, "dx"},
        {{4, 4, 4, std::numeric_limits<float>::quiet_NaN()}, {}, Code::invalid_argument, "dx"},
        {{4, 4, 4, std::numeric_limits<float>::infinity()}, {}, Code::invalid_argument, "dx"},
        {{2048, 2048, 1024, 0.01f}, {}, Code::capacity_exceeded, "2^31"},
        {{1024, 1024, 1024, 0.01f}, {}, Code::capacity_exceeded, std::to_string(kGridByteCap)},
        {{4, 4, 4, 0.1f}, unpaired, Code::invalid_argument, "x+"},
        {{3, 4, 4, 0.1f}, odd, Code::invalid_argument, "even"},
    };
    for (const Case& c : cases) {
        const auto made = make_grid(c.shape, c.bc);
        ASSERT_FALSE(made.has_value()) << c.what;
        const spade::Result<void> shape_ok = check_shape(c.shape, c.bc);
        ASSERT_FALSE(shape_ok.has_value()) << c.what;
        EXPECT_EQ(shape_ok.error().code, c.code) << c.what;
        EXPECT_EQ(shape_ok.error().context, made.error().context) << c.what;
        for (const bool with_solids : {false, true}) {
            const spade::Result<void> grid_ok = check_grid(c.shape, c.bc, with_solids);
            ASSERT_FALSE(grid_ok.has_value()) << c.what;
            EXPECT_EQ(grid_ok.error().context, made.error().context) << c.what;
        }
    }
    DomainBc slab;
    slab.face[kZMinus] = slab.face[kZPlus] = FaceBc::periodic;
    EXPECT_TRUE(check_shape({4, 4, 1, 0.1f}, slab).has_value()) << "a periodic axis of size 1 is a quasi-2D slab";
    EXPECT_TRUE(check_grid({4, 4, 1, 0.1f}, slab, true).has_value());
}

// A closed box whose -x wall moves in takes a net flux: check_domain()'s
// refusal, by face, from check_grid() and make_fluid_scratch() alike.
TEST(AirflowCheckGrid, AClosedDomainWithANetFluxIsRefusedByName) {
    const GridShape shape{8, 8, 8, 0.1f};
    DomainBc bc = walls();
    bc.wall_velocity[kXMinus] = glm::vec3(1.0f, 0.0f, 0.0f);
    const spade::Result<void> ok = check_grid(shape, bc, true);
    ASSERT_FALSE(ok.has_value());
    EXPECT_EQ(ok.error().code, Code::invalid_argument);
    EXPECT_NE(ok.error().context.find("x-"), std::string::npos) << ok.error().context;
    const MacGrid g = *make_grid(shape, bc);
    const auto scratch = make_fluid_scratch(g, bc, FluidConfig{.h = 0.001f});
    ASSERT_FALSE(scratch.has_value());
    EXPECT_EQ(scratch.error().context, ok.error().context);
}

// A 320^3 grid fits the cap as plan A (about 3.4 GB) but not with solids
// (about 6.6 GB), so a scene validator asks for the grid it will run.
// Mutation: with_solids ignored.
TEST(AirflowCheckGrid, SolidsMoveTheCap) {
    const GridShape shape{320, 320, 320, 0.01f};
    const DomainBc bc;
    const uint64_t plain = fluid_bytes(shape, bc);
    const uint64_t solid = plain + solid_bytes(shape, bc);
    ASSERT_LE(plain, kGridByteCap);
    ASSERT_GT(solid, kGridByteCap);
    EXPECT_TRUE(check_grid(shape, bc, false).has_value());
    const spade::Result<void> ok = check_grid(shape, bc, true);
    ASSERT_FALSE(ok.has_value());
    EXPECT_EQ(ok.error().code, Code::capacity_exceeded);
    const std::string& why = ok.error().context;
    EXPECT_NE(why.find(std::to_string(solid)), std::string::npos) << why;
    EXPECT_NE(why.find(std::to_string(kGridByteCap)), std::string::npos) << why;
    EXPECT_NE(why.find("solids"), std::string::npos) << why;
}

// solid_bytes() is what set_solids() allocates for the shape: the solid
// field, the regions' per-cell arrays (and the flood fill's queue, held while
// it labels), the PCG's vectors and every multigrid level's face weights;
// measured from a scratch after set_solids(), walled and periodic. Not
// counted: what grows with the solid surface (the wall links, the fill lists)
// or with the region count. Mutation: a term left out.
TEST(AirflowCheckGrid, TheSolidBytesAreWhatSetSolidsAllocates) {
    DomainBc periodic_x = walls();
    periodic_x.face[kXMinus] = periodic_x.face[kXPlus] = FaceBc::periodic;
    for (const DomainBc& bc : {walls(), periodic_x}) {
        const GridShape shape{16, 10, 12, 0.1f};
        MacGrid g = *make_grid(shape, bc);
        FluidScratch s = *make_fluid_scratch(g, bc, FluidConfig{.h = 0.001f});
        Scene scene;
        scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {0.8f, 0.5f, 0.6f},
                                     .radius = 0.3f, .min_feature = 0.6f});
        ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
        ASSERT_TRUE(s.solids.any);
        uint64_t held = 0;
        for (uint32_t c = 0; c < 3; ++c) {
            held += 4u * (s.solids.theta[c].size() + s.solids.wall[c].size() + s.solids.phi[c].size() +
                          s.solids.link[c].size());
            held += s.solids.layer[c].size();
        }
        held += 4u * s.regions.label.size() + s.regions.solid.size() + 4u * s.regions.touched.size();
        const PcgScratch& pcg = s.projection.pcg;
        held += 4u * (pcg.x.size() + pcg.b.size() + pcg.r.size() + pcg.z.size() + pcg.d.size() + pcg.q.size());
        for (const PoissonLevel& level : s.projection.mg.levels) {
            for (uint32_t a = 0; a < 3; ++a) held += 4u * level.w[a].size();
        }
        const uint64_t queue = 4u * g.p.size();
        EXPECT_EQ(solid_bytes(shape, bc), held + queue);
    }
}

}  // namespace
