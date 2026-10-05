// ---------------------------------------------------------------------------
// test_viewer_gate.cpp -- the gate scene throws its lobs through the ring.
//
// spade_viewer's gate scene (engine/tools/viewer/scenes.cpp) lobs five bodies
// at a torus gate, each launched so it is back at ring height as it reaches
// the gate's plane. The live smoke found it was not: the launch speed put
// every lob at the top of its arc there, 4.9 to 14.3 m up, over a ring whose
// centre is 1.8 m. This pins what the scene is for, on the viewer's own
// setup: every lob crosses z = 0 inside the ring's clear radius less its own
// contact radius, so it passes without touching.
// ---------------------------------------------------------------------------
#include <cstddef>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "../sandbox/scene_checks.hpp"
#include "bridge.hpp"
#include "setup.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"

namespace {

namespace checks = spade::sandbox::checks;

// The ring: a torus of radius 1.5 m and tube 0.15 m centred 1.8 m up
// (build_gate_world in scenes.cpp), so its clear opening is 1.35 m.
constexpr float kRingCentreY = 1.8f;
constexpr float kRingClearRadius = 1.5f - 0.15f;

// Body `row`'s position in world 0, read from the registered body rows.
[[nodiscard]] std::optional<glm::vec3> body_position(const spade::Simulation& sim, uint32_t row) {
    std::optional<glm::vec3> out;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& a) {
        if (a.name != "bodies" || row >= a.capacity_per_world || a.elem_size != sizeof(spade::BodyState)) return;
        glm::vec3 pos;
        std::memcpy(&pos, a.data + static_cast<std::size_t>(row) * a.elem_size + offsetof(spade::BodyState, pos),
                    sizeof pos);
        out = pos;
    });
    return out;
}

}  // namespace

TEST(ViewerGate, EveryLobPassesThroughTheRing) {
    std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene("gate");
    ASSERT_TRUE(scene.has_value());
    ASSERT_EQ(scene->bodies.size(), 5u);
    const float contact_radius = scene->worlds.worlds[0].contacts.proxy_radius;
    spade::viewer::SceneRun run = spade::viewer::make_simulation(*scene, spade::compute::BackendDesc{});

    const std::size_t n = scene->bodies.size();
    std::vector<glm::vec3> last(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto p = body_position(run.sim, static_cast<uint32_t>(i));
        ASSERT_TRUE(p.has_value()) << "no body row " << i;
        last[i] = *p;
    }
    std::vector<std::optional<checks::GateCrossing>> crossed(n);
    // 2 s at the viewer's 4 ms step; the slowest lob reaches the gate at 1.6 s.
    for (int tick = 0; tick < 500; ++tick) {
        ASSERT_TRUE(run.sim.step(1).has_value()) << "step failed at tick " << tick;
        for (std::size_t i = 0; i < n; ++i) {
            const glm::vec3 now = body_position(run.sim, static_cast<uint32_t>(i)).value_or(last[i]);
            if (!crossed[i]) crossed[i] = checks::crossing_z0(last[i], now);
            last[i] = now;
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        ASSERT_TRUE(crossed[i].has_value()) << "lob " << i << " never reached the gate's plane";
        EXPECT_TRUE(checks::through_ring(*crossed[i], kRingCentreY, kRingClearRadius - contact_radius))
            << "lob " << i << " crossed the gate at x " << crossed[i]->x << ", y " << crossed[i]->y
            << ", not inside the ring (centre y " << kRingCentreY << ", clear radius " << kRingClearRadius
            << " less the body's " << contact_radius << ")";
    }
}
