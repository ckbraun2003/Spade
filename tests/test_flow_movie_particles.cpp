// flow_movie's particle look (tools/flow_movie/particles.hpp; Cameron, 2026-11-17: "the airflow needs to surround the drone
// blades", "it can feel particle-y"). Points carried through the stored frames in order, one midpoint step a frame, seeded
// from a fixed pool, recycled by reason. Pure functions of the frame, the pool and the options: no RNG, no clock.
//
//   FlowMovieParticles.Moves*      a uniform field moves a point by exactly u dt; a solid-body rotation keeps its radius
//   FlowMovieParticles.Seeds*      a frame emits its slice of the pool, drawn at the seed with no streak
//   FlowMovieParticles.Retires*    leaving the region, meeting no-data and age each retire a point, counted by reason
//   FlowMovieParticles.Drops*      an emission past capacity is dropped and counted, never a silent grow
//   FlowMovieParticles.Refuses*    what it cannot run is refused by name before anything changes
//   FlowMovieParticles.TheSame*    the same frames, pool and options give the same state, bit for bit
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "../tools/flow_movie/particles.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace {

using spade::FlowVolume;
namespace fp16 = spade::fp16;

// A cube of n cells a side whose cell centre at world p holds the velocity f(p).
template <typename F>
[[nodiscard]] FlowVolume field(uint32_t n, float cell, glm::vec3 origin, F f) {
    FlowVolume v;
    v.header.origin = origin;
    v.header.cell = cell;
    v.header.nx = v.header.ny = v.header.nz = n;
    const std::size_t cells = std::size_t{n} * n * n;
    v.v.assign(cells * spade::kFlowHalvesPerCell, 0u);
    v.s.assign(cells * spade::kFlowHalvesPerCell, 0u);
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) {
                const glm::vec3 p =
                    origin + (glm::vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) + 0.5f) * cell;
                const glm::vec3 u = f(p);
                const std::size_t c = spade::flow_cell_index(v.header, i, j, k) * spade::kFlowHalvesPerCell;
                v.v[c + spade::kFlowU] = fp16::from_f32(u.x).bits;
                v.v[c + spade::kFlowV] = fp16::from_f32(u.y).bits;
                v.v[c + spade::kFlowW] = fp16::from_f32(u.z).bits;
            }
        }
    }
    for (std::size_t ch = 0; ch < spade::kFlowChannelCount; ++ch) {
        v.header.channel_min[ch] = -20.0f;
        v.header.channel_max[ch] = 20.0f;
    }
    return v;
}

flow_movie::ParticleOptions options() {
    flow_movie::ParticleOptions o;
    o.capacity = 64u;
    o.max_age_s = 1.0f;
    o.per_frame = 1u;
    o.region_lo = glm::vec3(-0.5f);
    o.region_hi = glm::vec3(0.5f);
    return o;
}

}  // namespace

TEST(FlowMovieParticles, MovesAUniformFieldByExactlyUDtAndKeepsWhereItWas) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, -1.0f, 0.5f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.0f, 0.1f, 0.0f)};
    flow_movie::ParticleField f;
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, options()));  // frame 0: emits the seed, moves nothing
    ASSERT_EQ(f.position.size(), 1u);
    EXPECT_EQ(f.position[0], pool[0]);
    EXPECT_EQ(f.previous[0], pool[0]) << "a new point has no streak";
    auto o = options();
    o.per_frame = 0u;
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, o));  // frame 1: moves by u dt
    ASSERT_EQ(f.position.size(), 1u);
    EXPECT_EQ(f.previous[0], pool[0]);
    EXPECT_NEAR(f.position[0].x, 0.02f, 1e-6f);
    EXPECT_NEAR(f.position[0].y, 0.09f, 1e-6f);
    EXPECT_NEAR(f.position[0].z, 0.005f, 1e-6f);
    EXPECT_NEAR(f.age[0], 0.01f, 1e-9f);
}

TEST(FlowMovieParticles, MovesASolidBodyRotationRoundTheAxisKeepingItsRadius) {
    const float omega = 50.0f;  // rad/s: 10 m/s at 0.2 m
    const FlowVolume v =
        field(32, 0.04f, glm::vec3(-0.64f), [omega](glm::vec3 p) { return glm::vec3(-omega * p.z, 0.0f, omega * p.x); });
    const std::vector<glm::vec3> pool{glm::vec3(0.2f, 0.0f, 0.0f)};
    flow_movie::ParticleField f;
    auto o = options();
    o.region_lo = glm::vec3(-0.6f);
    o.region_hi = glm::vec3(0.6f);
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.001f, o));
    o.per_frame = 0u;
    for (int i = 0; i < 100; ++i) ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.001f, o));
    ASSERT_EQ(f.position.size(), 1u);
    const glm::vec3 p = f.position[0];
    EXPECT_NEAR(std::sqrt(p.x * p.x + p.z * p.z), 0.2f, 0.2f * 0.01f) << "radius kept to 1% over 100 steps";
    const float angle = std::atan2(p.z, p.x);
    EXPECT_NEAR(angle, omega * 0.1f - 2.0f * 3.14159265f, 0.02f) << "0.1 s at omega is 5 rad, wrapped to (-pi, pi]";
}

TEST(FlowMovieParticles, SeedsTakeTheFramesSliceOfThePoolAndWrapAround) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 5; ++i) pool.emplace_back(0.05f * static_cast<float>(i), 0.0f, 0.0f);
    flow_movie::ParticleField f;
    auto o = options();
    o.per_frame = 2u;
    for (int frame = 0; frame < 3; ++frame) ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, o));
    ASSERT_EQ(f.position.size(), 6u);
    const int expect[6] = {0, 1, 2, 3, 4, 0};  // frame 0: 0,1; frame 1: 2,3; frame 2: 4 and 0 again
    for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(f.position[static_cast<std::size_t>(i)], pool[static_cast<std::size_t>(expect[i])]) << i;
    }
    EXPECT_EQ(f.frames, 3u);
}

TEST(FlowMovieParticles, RetiresAPointThatLeavesTheRegionMeetsNoDataOrGrowsOldAndCountsEach) {
    FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(10.0f, 0.0f, 0.0f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.45f, 0.0f, 0.0f), glm::vec3(-0.45f, 0.3f, 0.0f),
                                      glm::vec3(0.0f, -0.3f, 0.0f)};
    flow_movie::ParticleField f;
    auto o = options();
    o.per_frame = 3u;
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.001f, o));
    ASSERT_EQ(f.position.size(), 3u);
    o.per_frame = 0u;
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, o));  // 0.1 m: the first leaves x = 0.5
    EXPECT_EQ(f.position.size(), 2u);
    EXPECT_EQ(f.retired_left_region, 1u);
    // the second point is at (-0.35, 0.3, 0): the column of cells i = 5 (centre x = -0.25) becomes no-data, and its midpoint stage reads it
    for (uint32_t j = 0; j < 16u; ++j) {
        for (uint32_t k = 0; k < 16u; ++k) {
            const std::size_t c = spade::flow_cell_index(v.header, 5u, j, k) * spade::kFlowHalvesPerCell;
            for (std::size_t ch = 0; ch < 4; ++ch) v.v[c + ch] = fp16::kNoDataBits;
            v.header.no_data += 4u;
        }
    }
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, o));
    EXPECT_EQ(f.retired_no_data, 1u);
    EXPECT_EQ(f.position.size(), 1u);
    o.max_age_s = 0.02f;
    const FlowVolume still = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    ASSERT_TRUE(flow_movie::advance_particles(f, still, pool, 0.02f, o));
    EXPECT_EQ(f.retired_aged, 1u);
    EXPECT_TRUE(f.position.empty());
}

TEST(FlowMovieParticles, DropsAnEmissionPastCapacityAndCountsItNeverGrowing) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 10; ++i) pool.emplace_back(0.01f * static_cast<float>(i), 0.0f, 0.0f);
    flow_movie::ParticleField f;
    auto o = options();
    o.capacity = 4u;
    o.per_frame = 10u;
    ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.01f, o));
    EXPECT_EQ(f.position.size(), 4u);
    EXPECT_EQ(f.dropped, 6u);
}

TEST(FlowMovieParticles, RefusesWhatItCannotRunByNameBeforeChangingAnything) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.0f)};
    flow_movie::ParticleField f;
    auto o = options();
    EXPECT_FALSE(flow_movie::advance_particles(f, v, pool, 0.0f, o)) << "dt must be > 0";
    EXPECT_FALSE(flow_movie::advance_particles(f, v, {}, 0.01f, o)) << "an empty pool with per_frame > 0";
    o.capacity = 0u;
    EXPECT_FALSE(flow_movie::advance_particles(f, v, pool, 0.01f, o)) << "capacity 0";
    EXPECT_TRUE(f.position.empty());
    EXPECT_EQ(f.frames, 0u);
}

TEST(FlowMovieParticles, TheSameFramesPoolAndOptionsGiveTheSameStateBitForBit) {
    const FlowVolume v =
        field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3 p) { return glm::vec3(-3.0f * p.z, 0.4f, 3.0f * p.x); });
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 20; ++i) {
        pool.emplace_back(0.04f * static_cast<float>(i) - 0.4f, 0.05f, 0.03f * static_cast<float>(i % 7) - 0.1f);
    }
    auto run = [&] {
        flow_movie::ParticleField f;
        auto o = options();
        o.per_frame = 3u;
        for (int frame = 0; frame < 30; ++frame) EXPECT_TRUE(flow_movie::advance_particles(f, v, pool, 0.005f, o));
        return f;
    };
    const auto a = run();
    const auto b = run();
    EXPECT_EQ(a.position, b.position);
    EXPECT_EQ(a.previous, b.previous);
    EXPECT_EQ(a.age, b.age);
    EXPECT_EQ(a.retired_left_region, b.retired_left_region);
}

// The driver advances a field through frames IN ORDER (a stream hands them over once, 0 first), keeps a copy at the frames
// a still will be drawn from (the still is drawn after the solve, when the live field is long past it), and says which field
// shows frame k.
TEST(FlowMovieParticles, ADriverAdvancesInOrderAndKeepsACopyAtTheFramesAStillReads) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    auto o = options();
    o.per_frame = 2u;
    flow_movie::ParticleDriver driver(o, {2u});
    EXPECT_FALSE(driver.has_pool());
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 8; ++i) pool.emplace_back(0.05f * static_cast<float>(i) - 0.2f, 0.0f, 0.0f);
    driver.set_pool(pool);
    EXPECT_TRUE(driver.has_pool());
    EXPECT_FALSE(driver.advance(1u, v, 0.0)) << "frame 0 first";
    for (uint32_t k = 0; k < 5u; ++k) ASSERT_TRUE(driver.advance(k, v, 0.01 * k)) << k;
    EXPECT_FALSE(driver.advance(4u, v, 0.04)) << "a frame is advanced once";
    ASSERT_NE(driver.at(2u), nullptr);
    EXPECT_EQ(driver.at(2u)->frames, 3u) << "the copy is the field as frame 2 left it";
    ASSERT_NE(driver.at(4u), nullptr);
    EXPECT_EQ(driver.at(4u)->frames, 5u) << "the live field shows the last frame";
    EXPECT_EQ(driver.at(3u), nullptr) << "neither a copy nor the live field's frame";
    EXPECT_EQ(driver.at(9u), nullptr);
}

TEST(FlowMovieParticles, ADriverStepsByTheTimeBetweenFramesAndTheFirstFrameMovesNothing) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    auto o = options();
    o.per_frame = 1u;
    flow_movie::ParticleDriver driver(o, {});
    driver.set_pool(std::vector<glm::vec3>{glm::vec3(0.0f)});
    ASSERT_TRUE(driver.advance(0u, v, 0.100));
    ASSERT_TRUE(driver.advance(1u, v, 0.105));  // 5 ms later: the first point moves by 2 m/s x 5 ms = 1 cm
    ASSERT_NE(driver.at(1u), nullptr);
    ASSERT_EQ(driver.at(1u)->position.size(), 2u);
    EXPECT_NEAR(driver.at(1u)->position[0].x, 0.01f, 1e-6f);
    EXPECT_NEAR(driver.at(1u)->position[1].x, 0.0f, 1e-9f) << "the new seed stays where it is";
    EXPECT_FALSE(driver.advance(2u, v, 0.105)) << "time must go forward between frames";
}

// The region the points live in, when the options leave it open, is the first volume's box less one cell: a point at the
// box's edge reads clamped air, not the flow.
TEST(FlowMovieParticles, ADriverWithAnOpenRegionKeepsPointsInsideTheVolumeLessOneCell) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(10.0f, 0.0f, 0.0f); });
    flow_movie::ParticleOptions o;  // the region left at its default (open)
    o.capacity = 16u;
    o.per_frame = 1u;
    o.max_age_s = 5.0f;
    flow_movie::ParticleDriver driver(o, {});
    driver.set_pool(std::vector<glm::vec3>{glm::vec3(0.6f, 0.0f, 0.0f)});  // the box runs to x = 0.8; less a cell, 0.7
    ASSERT_TRUE(driver.advance(0u, v, 0.0));
    ASSERT_TRUE(driver.advance(1u, v, 0.02));  // 0.2 m on: x = 0.8
    EXPECT_EQ(driver.at(1u)->retired_left_region, 1u);
}

