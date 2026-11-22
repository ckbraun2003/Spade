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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

#include "../tools/flow_movie/particles.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "unique_temp.hpp"

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

// A pool whose first half is the rotor shells and second half the far field would emit the shells for the first half of its
// cycle and the far field for the second: spread_pool() permutes it by a fixed stride coprime to its length, so any run of
// frames takes from both. Same points, deterministic, no RNG.
TEST(FlowMovieParticles, ASpreadPoolHoldsTheSamePointsAndMixesTheGroupsInAnyShortRun) {
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 1000; ++i) pool.emplace_back(0.0f, static_cast<float>(i), 0.0f);    // group A: y < 500
    for (int i = 0; i < 1000; ++i) pool.emplace_back(1.0f, static_cast<float>(i), 0.0f);    // group B: x = 1
    const auto spread = flow_movie::spread_pool(pool);
    ASSERT_EQ(spread.size(), pool.size());
    auto sorted = [](std::vector<glm::vec3> v) {
        std::sort(v.begin(), v.end(), [](const glm::vec3& a, const glm::vec3& b) {
            return a.x != b.x ? a.x < b.x : (a.y != b.y ? a.y < b.y : a.z < b.z);
        });
        return v;
    };
    EXPECT_EQ(sorted(spread), sorted(pool)) << "a permutation";
    EXPECT_EQ(spread, flow_movie::spread_pool(pool)) << "deterministic";
    for (std::size_t from = 0; from + 100u <= spread.size(); from += 100u) {
        int b = 0;
        for (std::size_t i = from; i < from + 100u; ++i) b += spread[i].x > 0.5f ? 1 : 0;
        EXPECT_GE(b, 25) << "frames from " << from;
        EXPECT_LE(b, 75) << "frames from " << from;
    }
    EXPECT_TRUE(flow_movie::spread_pool({}).empty());
    EXPECT_EQ(flow_movie::spread_pool({glm::vec3(1.0f)}).size(), 1u);
}

// The drawn points: one per live particle, bright where it was seeded and dimmer and bluer as it ages, never outside [0, 1].
TEST(FlowMovieParticles, DrawnPointsAreOnePerParticleBrightestWhenNewAndNeverOutsideZeroToOne) {
    flow_movie::ParticleField f;
    f.position = {glm::vec3(0.0f), glm::vec3(1.0f), glm::vec3(2.0f)};
    f.previous = f.position;
    f.age = {0.0f, 0.25f, 0.5f};
    const spade::render::FlowPoints p = flow_movie::particle_points(f, 0.5f, 4u);
    ASSERT_EQ(p.positions.size(), 3u);
    ASSERT_EQ(p.colours.size(), 3u);
    EXPECT_EQ(p.size_px, 4u);
    EXPECT_EQ(p.positions[1], glm::vec3(1.0f));
    auto brightness = [](glm::vec3 c) { return c.x + c.y + c.z; };
    EXPECT_GT(brightness(p.colours[0]), brightness(p.colours[1]));
    EXPECT_GT(brightness(p.colours[1]), brightness(p.colours[2]));
    EXPECT_GT(p.colours[2].z, p.colours[2].x) << "an old point is blue";
    for (const glm::vec3& c : p.colours) {
        for (int a = 0; a < 3; ++a) {
            EXPECT_GE(c[a], 0.0f);
            EXPECT_LE(c[a], 1.0f);
        }
    }
    EXPECT_TRUE(spade::render::validate_flow_points(p));
}

// THE DRAWN STREAKS (the lead, 2026-11-17: "square dust isn't air"; Cameron: "particle-y"). One two-point polyline per particle
// for the GL capsule path with additive light: from where the point was `tail_s` ago along its own velocity (so the streak is as
// long as the air is fast), to where it is; alpha nothing at the tail, the head's alpha at the head; colour by speed, deep blue
// and dim when slow, warm white and bright when fast. A point is born invisible and fades in, and fades out toward max age, so
// recycling never pops.
TEST(FlowMovieParticles, AStreakRunsBackFromTheHeadAlongTheVelocityAndFadesFromNothingAtItsTail) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    flow_movie::ParticleField f;
    f.position = {glm::vec3(0.0f, 0.1f, 0.0f)};
    f.previous = f.position;
    f.age = {0.2f};
    flow_movie::StreakLook look;
    look.tail_s = 0.005f;
    look.width_px = 3.0f;
    const auto streaks = flow_movie::particle_streaks(f, v, 0.5f, look);
    ASSERT_EQ(streaks.size(), 1u);
    const spade::render::FlowPolyline& s = streaks[0];
    ASSERT_EQ(s.points.size(), 2u);
    EXPECT_NEAR(s.points[1].x, 0.0f, 1e-7f) << "the head is the point";
    EXPECT_NEAR(s.points[0].x, -0.01f, 1e-6f) << "the tail: 2 m/s for 5 ms behind it";
    EXPECT_NEAR(s.points[0].y, 0.1f, 1e-7f);
    ASSERT_EQ(s.alphas.size(), 2u);
    EXPECT_EQ(s.alphas[0], 0.0f) << "nothing at the tail";
    EXPECT_GT(s.alphas[1], 0.5f);
    EXPECT_LE(s.alphas[1], 1.0f);
    EXPECT_EQ(s.width_px, 3.0f);
    ASSERT_EQ(s.colours.size(), 2u);
    EXPECT_EQ(s.colours[0], s.colours[1]);
    EXPECT_TRUE(spade::render::validate_flow_polyline(s).has_value());
}

TEST(FlowMovieParticles, AStreakIsBluerAndDimmerSlowAndWarmerAndBrighterFastAndNeverOutsideZeroToOne) {
    auto colour_at = [](float speed) {
        const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [speed](glm::vec3) { return glm::vec3(speed, 0.0f, 0.0f); });
        flow_movie::ParticleField f;
        f.position = {glm::vec3(0.0f)};
        f.previous = f.position;
        f.age = {0.2f};
        const auto streaks = flow_movie::particle_streaks(f, v, 0.5f);
        return streaks.at(0).colours.at(1);
    };
    const glm::vec3 slow = colour_at(1.0f), fast = colour_at(10.0f), faster = colour_at(40.0f);
    auto luma = [](glm::vec3 c) { return c.x + c.y + c.z; };
    EXPECT_GT(luma(fast), luma(slow));
    EXPECT_GT(fast.x / fast.z, slow.x / slow.z) << "warmer: more red against blue";
    EXPECT_GT(slow.z, slow.x) << "slow is blue";
    EXPECT_EQ(faster, fast) << "the ramp ends at fast_mps";
    for (const glm::vec3 c : {slow, fast, faster}) {
        for (int a = 0; a < 3; ++a) {
            EXPECT_GE(c[a], 0.0f);
            EXPECT_LE(c[a], 1.0f);
        }
    }
}

TEST(FlowMovieParticles, AStreakIsBornInvisibleFadesInAndFadesOutTowardMaxAgeSoRecyclingNeverPops) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    auto head_alpha = [&v](float age) {
        flow_movie::ParticleField f;
        f.position = {glm::vec3(0.0f)};
        f.previous = f.position;
        f.age = {age};
        flow_movie::StreakLook look;
        look.fade_in_s = 0.04f;
        const auto streaks = flow_movie::particle_streaks(f, v, 0.5f, look);
        return streaks.at(0).alphas.at(1);
    };
    EXPECT_EQ(head_alpha(0.0f), 0.0f);
    EXPECT_GT(head_alpha(0.02f), 0.0f);
    EXPECT_LT(head_alpha(0.02f), head_alpha(0.04f));
    EXPECT_NEAR(head_alpha(0.04f), head_alpha(0.1f), 1e-6f) << "full from the fade-in on";
    EXPECT_NEAR(head_alpha(0.1f), head_alpha(0.29f), 1e-6f) << "and until 0.6 of the maximum age";
    EXPECT_GT(head_alpha(0.3f), head_alpha(0.4f));
    EXPECT_GT(head_alpha(0.4f), head_alpha(0.49f));
    EXPECT_NEAR(head_alpha(0.5f), 0.0f, 1e-7f) << "gone at the maximum age";
}

TEST(FlowMovieParticles, AStreakOfAParticleInNoDataIsLeftOutAndAStillOneIsAPointNotAHole) {
    FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3 p) { return p.x < 0.0f ? glm::vec3(0.0f) : glm::vec3(2.0f, 0.0f, 0.0f); });
    // the cells with x index 12 onward are a solid
    for (uint32_t k = 0; k < 16u; ++k) {
        for (uint32_t j = 0; j < 16u; ++j) {
            for (uint32_t i = 12u; i < 16u; ++i) {
                const std::size_t c = spade::flow_cell_index(v.header, i, j, k) * spade::kFlowHalvesPerCell;
                for (std::size_t ch = 0; ch < 4; ++ch) v.v[c + ch] = fp16::kNoDataBits;
                v.header.no_data += 4u;
            }
        }
    }
    flow_movie::ParticleField f;
    f.position = {glm::vec3(-0.4f, 0.0f, 0.0f), glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.3f, 0.0f, 0.0f)};
    f.previous = f.position;
    f.age = {0.2f, 0.2f, 0.2f};
    const auto streaks = flow_movie::particle_streaks(f, v, 0.5f);
    ASSERT_EQ(streaks.size(), 2u) << "the one inside the solid has no velocity: no streak";
    EXPECT_EQ(streaks[0].points[0], streaks[0].points[1]) << "no flow there: the streak is a point";
    EXPECT_NEAR(streaks[1].points[0].x, 0.3f - 2.0f * flow_movie::StreakLook{}.tail_s, 1e-6f) << "the third, in the order given";
}

TEST(FlowMovieParticles, TheStreaksAreTheSameEveryTimeAndOnePerDrawnParticleInOrder) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3 p) { return glm::vec3(-3.0f * p.z, 0.4f, 3.0f * p.x); });
    flow_movie::ParticleField f;
    for (int i = 0; i < 40; ++i) {
        f.position.emplace_back(0.03f * static_cast<float>(i) - 0.5f, 0.05f, 0.02f * static_cast<float>(i % 9) - 0.1f);
        f.age.push_back(0.01f * static_cast<float>(i));
    }
    f.previous = f.position;
    const auto a = flow_movie::particle_streaks(f, v, 0.5f);
    const auto b = flow_movie::particle_streaks(f, v, 0.5f);
    ASSERT_EQ(a.size(), 40u);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].points, b[i].points);
        EXPECT_EQ(a[i].alphas, b[i].alphas);
        EXPECT_EQ(a[i].colours, b[i].colours);
        EXPECT_EQ(a[i].points[1], f.position[i]) << "in the field's order";
    }
}

// THE PARTICLE SNAPSHOT (the lead's "days of looks work", 2026-11-17): a live run costs 11 GPU-minutes to carry the points to the
// still's frame, and every change of look (width, glow, colour, count) would pay it again. The field at the still's frame is small
// (about 25,000 points), so the run saves it beside the kept frame and a still is then redrawn from the two in seconds.
// A file of the field's three arrays and counters; a bad one is refused by name, never half-read.
namespace {
[[nodiscard]] std::filesystem::path snapshot_path(const char* name) {
    return spade::test::unique_temp_path(std::string("particle_snapshot_") + name + ".bin");  // TestHygiene: temp paths come from unique_temp.hpp
}
}  // namespace

TEST(FlowMovieParticles, ASnapshotWrittenAndReadBackIsTheSameFieldBitForBit) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3 p) { return glm::vec3(-3.0f * p.z, 0.4f, 3.0f * p.x); });
    std::vector<glm::vec3> pool;
    for (int i = 0; i < 30; ++i) pool.emplace_back(0.03f * static_cast<float>(i) - 0.4f, 0.0f, 0.02f * static_cast<float>(i % 5));
    flow_movie::ParticleField f;
    auto o = options();
    o.per_frame = 4u;
    o.capacity = 50u;
    for (int frame = 0; frame < 25; ++frame) ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.004f, o));
    ASSERT_GT(f.position.size(), 10u);
    ASSERT_GT(f.dropped + f.retired_left_region + f.retired_aged + f.retired_no_data, 0u) << "counters worth keeping";
    const auto path = snapshot_path("roundtrip");
    ASSERT_TRUE(flow_movie::write_particle_field(path, f).has_value());
    const auto back = flow_movie::read_particle_field(path);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->position, f.position);
    EXPECT_EQ(back->previous, f.previous);
    EXPECT_EQ(back->age, f.age);
    EXPECT_EQ(back->frames, f.frames);
    EXPECT_EQ(back->emitted, f.emitted);
    EXPECT_EQ(back->dropped, f.dropped);
    EXPECT_EQ(back->retired_left_region, f.retired_left_region);
    EXPECT_EQ(back->retired_no_data, f.retired_no_data);
    EXPECT_EQ(back->retired_aged, f.retired_aged);
    std::filesystem::remove(path);
}

TEST(FlowMovieParticles, AnEmptyFieldRoundTripsAndABadSnapshotIsRefusedByNameNeverHalfRead) {
    const auto path = snapshot_path("bad");
    const flow_movie::ParticleField empty;
    ASSERT_TRUE(flow_movie::write_particle_field(path, empty).has_value());
    const auto none = flow_movie::read_particle_field(path);
    ASSERT_TRUE(none.has_value());
    EXPECT_TRUE(none->position.empty());

    flow_movie::ParticleField f;
    f.position = {glm::vec3(1.0f), glm::vec3(2.0f)};
    f.previous = f.position;
    f.age = {0.1f, 0.2f};
    ASSERT_TRUE(flow_movie::write_particle_field(path, f).has_value());
    std::ifstream in(path, std::ios::binary);
    const std::string whole((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    ASSERT_GT(whole.size(), 40u);
    auto write_bytes = [&path](const std::string& bytes) {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    };
    write_bytes(whole.substr(0, whole.size() - 3));
    auto truncated = flow_movie::read_particle_field(path);
    ASSERT_FALSE(truncated.has_value());
    EXPECT_NE(truncated.error().context.find("particle"), std::string::npos) << truncated.error().context;
    std::string wrong = whole;
    wrong[0] = 'X';
    write_bytes(wrong);
    auto magic = flow_movie::read_particle_field(path);
    ASSERT_FALSE(magic.has_value());
    EXPECT_NE(magic.error().context.find("not a particle snapshot"), std::string::npos) << magic.error().context;
    write_bytes(whole + "extra");
    EXPECT_FALSE(flow_movie::read_particle_field(path).has_value()) << "trailing bytes: not the file that was written";
    std::filesystem::remove(path);
    EXPECT_FALSE(flow_movie::read_particle_field(path).has_value()) << "a missing file";
}

TEST(FlowMovieParticles, ADriverTakesASnapshotAsTheFieldThatShowsItsFrame) {
    flow_movie::ParticleOptions o = options();
    flow_movie::ParticleDriver driver(o, {});
    flow_movie::ParticleField f;
    f.position = {glm::vec3(0.1f)};
    f.previous = f.position;
    f.age = {0.1f};
    f.frames = 201u;
    EXPECT_EQ(driver.at(200u), nullptr);
    driver.add_snapshot(200u, f);
    ASSERT_NE(driver.at(200u), nullptr);
    EXPECT_EQ(driver.at(200u)->position, f.position);
    ASSERT_TRUE(driver.snapshot_at(200u) != nullptr) << "and it can be written out";
    EXPECT_EQ(driver.snapshot_at(199u), nullptr);
}

// THE LOOK'S NUMBERS, ON THE COMMAND LINE (--particle-look KEY=VALUE,...): a redraw from the snapshot takes seconds, so one job can
// draw several variants of the look; each variant is a string. Keys: tail (seconds), min_len (metres: a slow streak is never
// shorter), width and glow (pixels), strength (the glow's alpha), alpha (the head's, when full), fade_in (seconds), fast (m/s, the
// warm end of the colour ramp). An empty string is the defaults; an unknown key, a value that is not a number, or one out of its
// range is refused naming it.
TEST(FlowMovieParticles, AParticleLookStringSetsTheNamedNumbersAndLeavesTheRest) {
    const flow_movie::StreakLook defaults;
    const auto none = flow_movie::parse_particle_look("");
    ASSERT_TRUE(none.has_value()) << none.error().context;
    EXPECT_EQ(none->tail_s, defaults.tail_s);
    EXPECT_EQ(none->width_px, defaults.width_px);
    const auto some = flow_movie::parse_particle_look("tail=0.004, width=1.5,alpha=0.5,glow=3,strength=0.2,min_len=0.01,fade_in=0.02,fast=8");
    ASSERT_TRUE(some.has_value()) << some.error().context;
    EXPECT_FLOAT_EQ(some->tail_s, 0.004f);
    EXPECT_FLOAT_EQ(some->width_px, 1.5f);
    EXPECT_FLOAT_EQ(some->peak_alpha, 0.5f);
    EXPECT_FLOAT_EQ(some->glow_px, 3.0f);
    EXPECT_FLOAT_EQ(some->glow_strength, 0.2f);
    EXPECT_FLOAT_EQ(some->min_len_m, 0.01f);
    EXPECT_FLOAT_EQ(some->fade_in_s, 0.02f);
    EXPECT_FLOAT_EQ(some->fast_mps, 8.0f);
    const auto one = flow_movie::parse_particle_look("width=3");
    ASSERT_TRUE(one.has_value());
    EXPECT_FLOAT_EQ(one->width_px, 3.0f);
    EXPECT_EQ(one->tail_s, defaults.tail_s) << "the rest keep their defaults";
}

TEST(FlowMovieParticles, AParticleLookStringThatCannotBeRunIsRefusedByNameNotSkipped) {
    for (const char* bad : {"colour=red", "width", "width=", "width=abc", "width=0.1", "width=100", "alpha=1.5", "alpha=-0.1", "tail=0",
                            "tail=-1", "glow=-1", "strength=2", "fast=0", "fade_in=0", "min_len=-0.01", "width=2,,tail=0.004", "=3"}) {
        const auto r = flow_movie::parse_particle_look(bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(r.error().context.find("particle look"), std::string::npos) << bad << ": " << r.error().context;
    }
    const auto unknown = flow_movie::parse_particle_look("width=2,sparkle=3");
    ASSERT_FALSE(unknown.has_value());
    EXPECT_NE(unknown.error().context.find("sparkle"), std::string::npos) << "the key is named: " << unknown.error().context;
}

TEST(FlowMovieParticles, ASlowStreakIsAtLeastMinLenLongAlongItsVelocityAndAFastOneAndStillAirAreUnchanged) {
    auto tail_of = [](float speed, float min_len) {
        const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [speed](glm::vec3) { return glm::vec3(speed, 0.0f, 0.0f); });
        flow_movie::ParticleField f;
        f.position = {glm::vec3(0.0f)};
        f.previous = f.position;
        f.age = {0.2f};
        flow_movie::StreakLook look;
        look.tail_s = 0.005f;
        look.min_len_m = min_len;
        return flow_movie::particle_streaks(f, v, 0.5f, look).at(0).points.at(0);
    };
    EXPECT_NEAR(tail_of(1.0f, 0.0f).x, -0.005f, 1e-7f) << "no minimum: u times tail_s";
    EXPECT_NEAR(tail_of(1.0f, 0.02f).x, -0.02f, 1e-7f) << "1 m/s for 5 ms is 5 mm: lengthened to the minimum, along the velocity";
    EXPECT_NEAR(tail_of(10.0f, 0.02f).x, -0.05f, 1e-7f) << "a fast one is already longer";
    EXPECT_EQ(tail_of(0.0f, 0.02f), glm::vec3(0.0f)) << "still air has no direction: a point";
    EXPECT_NEAR(tail_of(-1.0f, 0.02f).x, 0.02f, 1e-7f) << "behind it, whichever way it goes";
}

// THE DRAWN STREAKS NEAR THE DRONE (the lead, 2026-11-17: "the streaks must not wash the drone; the drone must read clearly"): the
// streak's alpha is dimmed by `dim` (0 to 1, off at 0) at the drone's centre, falling smoothly to nothing at `dim_r` metres from
// it. Additive light in front of the airframe lifts its colour; dimmed, the body and the rotors read through.
TEST(FlowMovieParticles, StreaksNearTheDroneAreDimmedByTheStatedAmountAndFarOnesAreNot) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    auto head_alpha = [&v](glm::vec3 at, float dim) {
        flow_movie::ParticleField f;
        f.position = {at};
        f.previous = f.position;
        f.age = {0.2f};
        flow_movie::StreakLook look;
        look.dim_centre = glm::vec3(0.0f);
        look.dim_r = 0.2f;
        look.dim = dim;
        return flow_movie::particle_streaks(f, v, 0.5f, look).at(0).alphas.at(1);
    };
    const float full = head_alpha(glm::vec3(0.0f, 0.0f, 0.0f), 0.0f);
    EXPECT_GT(full, 0.5f);
    EXPECT_NEAR(head_alpha(glm::vec3(0.0f), 0.6f), full * 0.4f, 1e-6f) << "at the centre: 1 - dim";
    EXPECT_NEAR(head_alpha(glm::vec3(0.25f, 0.0f, 0.0f), 0.6f), full, 1e-6f) << "past dim_r: untouched";
    const float half_way = head_alpha(glm::vec3(0.1f, 0.0f, 0.0f), 0.6f);
    EXPECT_GT(half_way, full * 0.4f);
    EXPECT_LT(half_way, full) << "in between: smoothly";
    EXPECT_NEAR(head_alpha(glm::vec3(0.0f), 0.0f), full, 1e-7f) << "dim 0 is off";
}

TEST(FlowMovieParticles, TheDimmingKeysAreInTheLookStringAndRangeChecked) {
    const auto look = flow_movie::parse_particle_look("dim=0.7,dim_r=0.15");
    ASSERT_TRUE(look.has_value()) << look.error().context;
    EXPECT_FLOAT_EQ(look->dim, 0.7f);
    EXPECT_FLOAT_EQ(look->dim_r, 0.15f);
    EXPECT_FALSE(flow_movie::parse_particle_look("dim=1.5").has_value());
    EXPECT_FALSE(flow_movie::parse_particle_look("dim=-0.1").has_value());
    EXPECT_FALSE(flow_movie::parse_particle_look("dim_r=0").has_value());
}


// L106b: a still of the particle look is drawn from a run's kept frames with no solve: advance_through_frames() carries the driver through frames 0 to `last` in
// order, each frame read once, the pool taken from frame 0's log, each step the stored frames' own time difference. A uniform 2 m/s flow read from 1 ms-apart
// frames moves the first seed by exactly v x n x dt after n advances (the same step the streamed run takes), a snapshot is kept at each asked frame, and a frame
// that cannot be read stops the pass naming it.
TEST(FlowMovieParticles, AdvancingThroughStoredFramesMovesAPointByExactlyVNDtAndKeepsTheAskedSnapshots) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    auto o = options();
    o.per_frame = 1u;
    flow_movie::ParticleDriver driver(o, {2u, 5u});
    uint32_t reads = 0, pools = 0;
    const auto read = [&](uint32_t k, FlowVolume& volume, double& time_s) -> spade::Result<void> {
        EXPECT_EQ(k, reads) << "each frame once, in order";
        ++reads;
        volume = v;
        time_s = 0.100 + 0.001 * static_cast<double>(k);  // 1 ms apart, as Physics' runs write them
        return {};
    };
    const auto pool_of = [&]() -> std::vector<glm::vec3> {
        ++pools;
        EXPECT_EQ(reads, 1u) << "the pool is taken once, right after frame 0 is read";
        return std::vector<glm::vec3>{glm::vec3(0.0f), glm::vec3(0.0f, 0.1f, 0.0f)};
    };
    ASSERT_TRUE(flow_movie::advance_through_frames(driver, 5u, read, pool_of));
    EXPECT_EQ(reads, 6u);
    EXPECT_EQ(pools, 1u);
    const flow_movie::ParticleField* five = driver.snapshot_at(5u);
    ASSERT_NE(five, nullptr);
    ASSERT_EQ(five->position.size(), 6u) << "one seed a frame";
    EXPECT_NEAR(five->position[0].x, 5.0f * 2.0f * 0.001f, 1e-6f) << "v x n x dt = 2 m/s x 5 x 1 ms";
    EXPECT_NEAR(five->position[0].y, 0.0f, 1e-9f);
    EXPECT_NEAR(five->position[1].x, 4.0f * 2.0f * 0.001f, 1e-6f) << "the second seed, one frame younger";
    ASSERT_NE(driver.snapshot_at(2u), nullptr);
    EXPECT_EQ(driver.snapshot_at(2u)->frames, 3u);
    EXPECT_EQ(driver.snapshot_at(3u), nullptr);

    flow_movie::ParticleDriver again(o, {});
    const auto broken = [&](uint32_t k, FlowVolume& volume, double& time_s) -> spade::Result<void> {
        if (k == 2u) return std::unexpected(spade::Error{spade::Code::io_error, "frame 2 is not on disk"});
        volume = v;
        time_s = 0.001 * static_cast<double>(k);
        return {};
    };
    const spade::Result<void> stopped = flow_movie::advance_through_frames(again, 4u, broken, [&]() { return std::vector<glm::vec3>{glm::vec3(0.0f)}; });
    ASSERT_FALSE(stopped.has_value());
    EXPECT_NE(stopped.error().context.find("frame 2"), std::string::npos) << stopped.error().context;
}

// L106b: one RK2 step across 5 to 8 cells of a frame's frozen velocity is coarse in a tip vortex's core, so a frame can be advanced in `substeps` midpoint steps of
// dt / substeps through the same frame (ParticleOptions::substeps, --particle-look substeps=N). Default 1: the film as it was, bit for bit.
TEST(FlowMovieParticles, SubStepsDefaultToOneAndAFrameIsAdvancedInTheSameWayAsBefore) {
    EXPECT_EQ(flow_movie::ParticleOptions{}.substeps, 1u);
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.0f)};
    for (uint32_t substeps : {1u, 4u}) {
        flow_movie::ParticleField f;
        auto o = options();
        o.substeps = substeps;
        ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.005f, o)) << substeps;
        o.per_frame = 0u;
        ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.005f, o)) << substeps;
        ASSERT_EQ(f.position.size(), 1u);
        EXPECT_NEAR(f.position[0].x, 0.01f, 1e-6f) << "a uniform flow moves a point by exactly u dt however it is sub-stepped (substeps " << substeps << ")";
        EXPECT_NEAR(f.previous[0].x, 0.0f, 1e-9f) << "previous is where it was a FRAME ago, not a sub-step ago";
    }
}

TEST(FlowMovieParticles, MoreSubStepsHoldASolidBodyRotationsRadiusTighter) {
    const float omega = 125.0f;  // rad/s: 25 m/s at 0.2 m, so omega x dt = 0.5 a frame (the held frame's rotor region moves 5 to 8 cells a frame)
    const FlowVolume v =
        field(32, 0.04f, glm::vec3(-0.64f), [omega](glm::vec3 p) { return glm::vec3(-omega * p.z, 0.0f, omega * p.x); });
    const std::vector<glm::vec3> pool{glm::vec3(0.2f, 0.0f, 0.0f)};
    float radius[2] = {0.0f, 0.0f};
    int at = 0;
    for (uint32_t substeps : {1u, 4u}) {
        flow_movie::ParticleField f;
        auto o = options();
        o.region_lo = glm::vec3(-0.6f);
        o.region_hi = glm::vec3(0.6f);
        o.substeps = substeps;
        ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.004f, o)) << substeps;
        o.per_frame = 0u;
        for (int i = 0; i < 20; ++i) ASSERT_TRUE(flow_movie::advance_particles(f, v, pool, 0.004f, o)) << substeps;
        ASSERT_EQ(f.position.size(), 1u) << substeps;
        radius[at++] = std::sqrt(f.position[0].x * f.position[0].x + f.position[0].z * f.position[0].z);
    }
    EXPECT_GT(radius[0], 0.2f * 1.10f) << "one midpoint step a frame lets the radius grow more than 10% in 20 frames at omega dt = 0.5";
    EXPECT_LT(radius[1], 0.2f * 1.02f) << "four sub-steps hold it to 2%";
    EXPECT_LT(radius[1] - 0.2f, 0.5f * (radius[0] - 0.2f)) << "and to less than half of the drift";
}

TEST(FlowMovieParticles, ASubStepCountOutsideOneToSixtyFourIsRefusedByNameBeforeAnyChange) {
    const FlowVolume v = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.0f)};
    for (uint32_t substeps : {0u, 65u}) {
        flow_movie::ParticleField f;
        auto o = options();
        o.substeps = substeps;
        const spade::Result<void> refused = flow_movie::advance_particles(f, v, pool, 0.001f, o);
        ASSERT_FALSE(refused.has_value()) << substeps;
        EXPECT_NE(refused.error().context.find("substeps"), std::string::npos) << refused.error().context;
        EXPECT_EQ(f.frames, 0u) << "nothing changed";
    }
}

TEST(FlowMovieParticles, TheParticleLookStringSetsSubStepsAsAWholeNumberOneToSixtyFour) {
    EXPECT_EQ(flow_movie::parse_particle_look("").value().substeps, 1u) << "the film's default";
    const auto four = flow_movie::parse_particle_look("substeps=4,width=3");
    ASSERT_TRUE(four.has_value());
    EXPECT_EQ(four->substeps, 4u);
    EXPECT_NEAR(four->width_px, 3.0f, 1e-6f) << "the other keys still work beside it";
    for (const char* text : {"substeps=0", "substeps=65", "substeps=2.5", "substeps=-1"}) {
        const auto refused = flow_movie::parse_particle_look(text);
        ASSERT_FALSE(refused.has_value()) << text;
        EXPECT_NE(refused.error().context.find("substeps"), std::string::npos) << refused.error().context;
    }
}

// ---- B1 (O170, audit #12): the points' time blend. Between two cached frames the flow is not steady: with the NEXT frame in hand, each midpoint step reads the velocity blended between the
// drawn frame and the next at the time of the step, instead of holding the drawn frame's velocity for the whole interval. Without the next frame (a streamed run) nothing changes, bit for bit.
namespace {
[[nodiscard]] flow_movie::ParticleField after_one_step(const FlowVolume& a, const FlowVolume* next, uint32_t substeps, float dt = 0.01f) {
    const std::vector<glm::vec3> pool{glm::vec3(0.0f, 0.1f, 0.0f)};
    flow_movie::ParticleField f;
    auto o = options();
    o.substeps = substeps;
    EXPECT_TRUE(flow_movie::advance_particles(f, a, next, pool, dt, o));  // frame 0 emits the seed
    o.per_frame = 0u;
    EXPECT_TRUE(flow_movie::advance_particles(f, a, next, pool, dt, o));  // frame 1 moves it
    return f;
}
}  // namespace

TEST(FlowMovieParticles, WithoutANextFrameOrWithTheSameOneNothingChangesBitForBit) {
    const auto shear = [](glm::vec3 p) { return glm::vec3(2.0f + 4.0f * p.y, -1.0f, 0.5f * p.x); };
    const FlowVolume a = field(16, 0.1f, glm::vec3(-0.8f), shear);
    const FlowVolume same = a;
    for (const uint32_t n : {1u, 4u, 8u}) {
        const flow_movie::ParticleField plain = [&] {
            const std::vector<glm::vec3> pool{glm::vec3(0.0f, 0.1f, 0.0f)};
            flow_movie::ParticleField f;
            auto o = options();
            o.substeps = n;
            EXPECT_TRUE(flow_movie::advance_particles(f, a, pool, 0.01f, o));
            o.per_frame = 0u;
            EXPECT_TRUE(flow_movie::advance_particles(f, a, pool, 0.01f, o));
            return f;
        }();
        const auto none = after_one_step(a, nullptr, n);
        const auto twin = after_one_step(a, &same, n);
        ASSERT_EQ(plain.position.size(), 1u);
        EXPECT_EQ(none.position[0], plain.position[0]) << "no next frame: the old step, " << n << " substeps";
        EXPECT_EQ(twin.position[0], plain.position[0]) << "a next frame equal to the drawn one blends nothing, " << n << " substeps";
    }
}

// u runs linearly in time from 0 (drawn frame) to U = 2 m/s (next frame): the midpoint rule is exact for it, whatever the substeps: the displacement is U dt / 2.
TEST(FlowMovieParticles, AFlowThatGrowsLinearlyBetweenTheFramesMovesHalfTheNextFramesStepForAnySubsteps) {
    const FlowVolume a = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    const FlowVolume b = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    for (const uint32_t n : {1u, 4u, 8u}) {
        const auto held = after_one_step(a, nullptr, n);
        const auto blended = after_one_step(a, &b, n);
        ASSERT_EQ(blended.position.size(), 1u);
        EXPECT_NEAR(held.position[0].x, 0.0f, 1e-7f) << "the drawn frame alone holds u = 0 for the whole interval: " << n;
        EXPECT_NEAR(blended.position[0].x, 0.5f * 2.0f * 0.01f, 1e-6f) << "U dt / 2: " << n << " substeps";
        EXPECT_NEAR(blended.position[0].y, 0.1f, 1e-7f);
    }
    // two steady-ish frames, u = 1 then 3: the interval's mean is 2
    const FlowVolume one = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    const FlowVolume three = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(3.0f, 0.0f, 0.0f); });
    const auto mean = after_one_step(one, &three, 4);
    EXPECT_NEAR(mean.position[0].x, 2.0f * 0.01f, 1e-6f);
}

TEST(FlowMovieParticles, ANextFrameOfAnotherShapeIsRefusedByNameAndNoDataInItRetiresThePoint) {
    const FlowVolume a = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    const FlowVolume small = field(8, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(1.0f, 0.0f, 0.0f); });
    const std::vector<glm::vec3> pool{glm::vec3(0.0f, 0.1f, 0.0f)};
    flow_movie::ParticleField f;
    const auto refused = flow_movie::advance_particles(f, a, &small, pool, 0.01f, options());
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("next"), std::string::npos) << refused.error().context;
    EXPECT_TRUE(f.position.empty()) << "refused before any change";

    FlowVolume hole = a;
    for (std::size_t c = 0; c < hole.v.size(); ++c) hole.v[c] = 0x7E00u;  // no-data everywhere in the next frame
    flow_movie::ParticleField g;
    auto o = options();
    ASSERT_TRUE(flow_movie::advance_particles(g, a, &hole, pool, 0.01f, o));  // frame 0: emits, moves nothing
    o.per_frame = 0u;
    ASSERT_TRUE(flow_movie::advance_particles(g, a, &hole, pool, 0.01f, o));
    EXPECT_TRUE(g.position.empty());
    EXPECT_EQ(g.retired_no_data, 1u);
}

// B1, at the driver: the run's own driver passes the next stored frame to the step, so the points follow the velocity between frames (linear in time from the drawn frame to the next)
// and not the drawn frame's held for the whole interval. A streamed run has no next frame and steps as before.
TEST(FlowMovieParticles, ADriverGivenTheNextFrameBlendsTowardItAndWithoutItStepsAsBefore) {
    const FlowVolume drawn = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(0.0f); });
    const FlowVolume next = field(16, 0.1f, glm::vec3(-0.8f), [](glm::vec3) { return glm::vec3(2.0f, 0.0f, 0.0f); });
    const auto run = [&](const FlowVolume* next_frame) {
        auto o = options();
        o.per_frame = 1u;
        flow_movie::ParticleDriver driver(o, {});
        driver.set_pool(std::vector<glm::vec3>{glm::vec3(0.0f)});
        EXPECT_TRUE(driver.advance(0u, drawn, 0.100, next_frame));
        EXPECT_TRUE(driver.advance(1u, drawn, 0.110, next_frame));  // 10 ms
        return driver.at(1u)->position[0].x;
    };
    EXPECT_NEAR(run(nullptr), 0.0f, 1e-9f) << "no next frame: the drawn frame's still air moves nothing";
    EXPECT_NEAR(run(&next), 0.5f * 2.0f * 0.010f, 1e-6f) << "u runs 0 to 2 m/s over the step: the midpoint rule moves it U dt / 2 = 1 cm";
}
