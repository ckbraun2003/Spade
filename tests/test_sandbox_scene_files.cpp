// The built-in scene files the live smoke used to open, headless.
//
// The tour opened five scene files of assets/scenes/ in a window (shower,
// bounce, gate, hover, wind) and checked what each is for. The restructured
// tour (docs/design/interface/plans/2026-10-13-live-tour-storyboard.md, §4)
// keeps those checks here, without a window: each file composes and
// instantiates with the sandbox's built-in records, places the bodies it
// should, and does what it is for, stepped as the tour stepped it (60 Hz
// frames of whole 4 ms steps, the remainder carried to the next frame).
// gate's lobs are test_viewer_gate.cpp's.
//
// The shower steps 1000 bodies for 12.75 s, about 33 s on release and far
// past the debug cap, so it runs where the viewer's full trajectories run:
// with SPADE_FULL_VIEWER_TRAJECTORIES=1, which the Docker leg sets
// (test_viewer_trajectories.cpp gates its shower the same way).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/builtin_records.hpp"
#include "../sandbox/scene_checks.hpp"
#include "scene/compose.hpp"

namespace {

namespace fs = std::filesystem;
namespace checks = spade::sandbox::checks;
using spade::sandbox::BuiltinRecords;

struct Lane {
    spade::scene::ComposedScene composed;
    spade::scene::SceneRun run;
    BuiltinRecords records;
};

// A scene file composed and instantiated with its built-in records, as the
// tour's scene session did.
[[nodiscard]] spade::Result<Lane> open_lane(const std::string& stem) {
    const std::optional<BuiltinRecords> records = spade::sandbox::builtin_records(stem);
    if (!records) return std::unexpected(spade::Error{spade::Code::not_found, stem + " has no built-in records"});
    const fs::path file = fs::path(SPADE_ASSETS_DIR) / "scenes" / (stem + ".scene.yaml");
    spade::Result<spade::scene::ComposedScene> composed = spade::scene::compose_file(file);
    if (!composed) return std::unexpected(spade::Error{composed.error().code, stem + ": " + composed.error().context});
    spade::Result<spade::scene::SceneRun> run =
        spade::scene::instantiate(*composed, spade::sandbox::instance_of(*records), spade::sandbox::kBuiltinStepNs,
                                  spade::sandbox::kBuiltinSubsteps);
    if (!run) return std::unexpected(spade::Error{run.error().code, stem + ": " + run.error().context});
    return Lane{std::move(*composed), std::move(*run), *records};
}

// The tour's clock: frames of 1/60 s, each running the whole built-in steps
// it covers, the remainder carried (SceneFileSession::frame()).
class FrameClock {
  public:
    [[nodiscard]] uint64_t steps_in_next_frame() {
        carry_ns_ += static_cast<uint64_t>(std::llround(static_cast<double>(1.0f / 60.0f) * 1e9));
        const uint64_t n = carry_ns_ / spade::sandbox::kBuiltinStepNs;
        carry_ns_ -= n * spade::sandbox::kBuiltinStepNs;
        return n;
    }

  private:
    uint64_t carry_ns_ = 0;
};

// One frame of every lane. Void, so a failed step stops the test.
void frame(std::vector<Lane>& lanes, FrameClock& clock) {
    const uint64_t n = clock.steps_in_next_frame();
    for (Lane& lane : lanes) {
        for (uint64_t i = 0; i < n; ++i) {
            const spade::Result<void> stepped = lane.run.sim.step(1);
            ASSERT_TRUE(stepped.has_value()) << stepped.error().context;
        }
    }
}

void frames(std::vector<Lane>& lanes, FrameClock& clock, int count) {
    for (int f = 0; f < count; ++f) ASSERT_NO_FATAL_FAILURE(frame(lanes, clock));
}

[[nodiscard]] std::optional<glm::vec3> position(const Lane& lane, std::size_t vehicle) {
    const auto body = lane.run.sim.body(lane.run.vehicles.at(vehicle).body);
    if (!body) return std::nullopt;
    return (*body)->pos;
}

[[nodiscard]] std::optional<glm::quat> orientation(const Lane& lane, std::size_t vehicle) {
    const auto body = lane.run.sim.body(lane.run.vehicles.at(vehicle).body);
    if (!body) return std::nullopt;
    return (*body)->orient;
}

// Read once, before the suite starts any thread.
[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] bool finite(const glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// The angle a body has tilted from level, radians.
[[nodiscard]] float tilt(const glm::quat& q) {
    const glm::vec3 up = q * glm::vec3(0.0f, 1.0f, 0.0f);
    return std::acos(std::clamp(up.y, -1.0f, 1.0f));
}

// Opens `stems` as lanes and runs the tour's "open" step: 45 frames.
void open_scene(const std::vector<std::string>& stems, std::vector<Lane>& lanes, FrameClock& clock) {
    for (const std::string& stem : stems) {
        spade::Result<Lane> lane = open_lane(stem);
        ASSERT_TRUE(lane.has_value()) << lane.error().context;
        lanes.push_back(std::move(*lane));
    }
    ASSERT_NO_FATAL_FAILURE(frames(lanes, clock, 45));
}

}  // namespace

// The live smoke's <scene>.open/composed and /bodies: each file composes and
// instantiates with its built-in records, and places the bodies it should.
TEST(SandboxSceneFiles, EachTourSceneComposesAndPlacesItsBodies) {
    const std::vector<std::pair<std::string, std::size_t>> expected = {
        {"shower", 1000},       {"bounce_lane_0", 1}, {"bounce_lane_1", 1}, {"bounce_lane_2", 1},
        {"bounce_lane_3", 1},   {"gate", 5},          {"hover", 1},         {"wind", 1}};
    for (const auto& [stem, bodies] : expected) {
        const spade::Result<Lane> lane = open_lane(stem);
        ASSERT_TRUE(lane.has_value()) << lane.error().context;
        EXPECT_EQ(lane->run.vehicles.size(), bodies) << stem;
    }
}

// shower.pour: 1000 bodies poured into a CSG bowl (a floor whose top is y = 0
// and walls whose inner faces are |x|, |z| = 4), which holds them.
TEST(SandboxSceneFiles, TheShowerPoursIntoTheBowlAndStaysThere) {
    if (!env_gate_is_set("SPADE_FULL_VIEWER_TRAJECTORIES")) {
        GTEST_SKIP() << "the shower's 12.75 s of 1000 bodies runs with SPADE_FULL_VIEWER_TRAJECTORIES=1 (the Docker leg)";
    }
    std::vector<Lane> lanes;
    FrameClock clock;
    ASSERT_NO_FATAL_FAILURE(open_scene({"shower"}, lanes, clock));
    const Lane& shower = lanes[0];
    const auto mean_height = [&] {
        double sum = 0.0;
        for (std::size_t i = 0; i < shower.run.vehicles.size(); ++i) sum += position(shower, i).value_or(glm::vec3(0.0f)).y;
        return static_cast<float>(sum / static_cast<double>(shower.run.vehicles.size()));
    };
    const float before = mean_height();
    ASSERT_NO_FATAL_FAILURE(frames(lanes, clock, 720));

    std::size_t not_finite = 0;
    std::size_t outside = 0;
    for (std::size_t i = 0; i < shower.run.vehicles.size(); ++i) {
        const std::optional<glm::vec3> p = position(shower, i);
        if (!p || !finite(*p)) {
            ++not_finite;
            continue;
        }
        if (std::abs(p->x) > 4.0f || std::abs(p->z) > 4.0f || p->y < 0.0f) ++outside;
    }
    EXPECT_EQ(not_finite, 0u) << "bodies not finite";
    EXPECT_EQ(outside, 0u) << "of 1000 outside the bowl";
    const float after = mean_height();
    EXPECT_LT(after, before - 0.5f) << "mean height " << before << " -> " << after << " m";
}

// bounce.drops: four lanes, one body each, dropped from 6 m at restitution 0,
// 0.25, 0.5 and 0.75. Each lands, and each rebounds higher than the last.
TEST(SandboxSceneFiles, TheBounceReboundsRiseWithRestitution) {
    std::vector<Lane> lanes;
    FrameClock clock;
    ASSERT_NO_FATAL_FAILURE(
        open_scene({"bounce_lane_0", "bounce_lane_1", "bounce_lane_2", "bounce_lane_3"}, lanes, clock));
    std::vector<std::vector<float>> heights(lanes.size());
    for (int f = 0; f < 420; ++f) {
        ASSERT_NO_FATAL_FAILURE(frame(lanes, clock));
        for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
            heights[lane].push_back(position(lanes[lane], 0).value_or(glm::vec3(0.0f)).y);
        }
    }
    // Rebound: the highest point after first contact, its centre within one
    // contact radius and a margin of the ground.
    std::vector<float> rebounds;
    for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
        const float contact_y = lanes[lane].records.contacts.proxy_radius + 0.05f;
        const std::optional<float> r = checks::rebound_height(heights[lane], contact_y);
        EXPECT_TRUE(r.has_value()) << "lane " << lane << " (e " << lanes[lane].records.contacts.restitution_e
                                   << ") never landed";
        rebounds.push_back(r.value_or(0.0f));
    }
    EXPECT_TRUE(checks::strictly_increasing(rebounds))
        << "rebounds " << ::testing::PrintToString(rebounds) << " m do not rise with restitution";
}

// hover.holds: a quadrotor at its hover rotor speed, open loop, in calm air,
// holds its height, level.
TEST(SandboxSceneFiles, TheHoverHoldsItsHeightLevel) {
    std::vector<Lane> lanes;
    FrameClock clock;
    ASSERT_NO_FATAL_FAILURE(open_scene({"hover"}, lanes, clock));
    const float y0 = position(lanes[0], 0).value_or(glm::vec3(0.0f)).y;
    float worst = 0.0f;
    for (int f = 0; f < 360; ++f) {
        ASSERT_NO_FATAL_FAILURE(frame(lanes, clock));
        worst = std::max(worst, std::abs(position(lanes[0], 0).value_or(glm::vec3(0.0f)).y - y0));
    }
    EXPECT_LT(worst, 0.25f) << "largest change " << worst << " m from " << y0 << " m";
    const float tilted = tilt(orientation(lanes[0], 0).value_or(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)));
    EXPECT_LT(tilted, 0.05f) << "tilted " << tilted << " rad";
}

// wind.gusts: the same open-loop hover in moderate turbulence, which carries
// it metres (the viewer scene's own account of it), and it stays finite.
TEST(SandboxSceneFiles, TheWindCarriesTheHover) {
    std::vector<Lane> lanes;
    FrameClock clock;
    ASSERT_NO_FATAL_FAILURE(open_scene({"wind"}, lanes, clock));
    const glm::vec3 p0 = position(lanes[0], 0).value_or(glm::vec3(0.0f));
    float worst = 0.0f;
    bool all_finite = true;
    for (int f = 0; f < 360; ++f) {
        ASSERT_NO_FATAL_FAILURE(frame(lanes, clock));
        const glm::vec3 p = position(lanes[0], 0).value_or(glm::vec3(0.0f));
        all_finite = all_finite && finite(p);
        worst = std::max(worst, std::abs(p.y - p0.y));
    }
    EXPECT_TRUE(all_finite) << "the drone went non-finite";
    EXPECT_GT(worst, 0.5f) << "largest height change " << worst << " m";
}
