// Airflow visualization plan step 6: plan A's own fields through the views.
// The solver's MAC grid goes through Physics' publish_render_volume() (Task
// 8b) into the flow volume the views read, and the views are checked
// against what the field is known to be. The Taylor-Green vortex is
// analytic at t = 0, and decays at the viscous step's own rate at low
// Reynolds number. The actuator disc (Task 9) drives a wake whose rotor
// seeds must move down as momentum theory says, through every view.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "flow_synthetic.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/publish.hpp"
#include "render/flow_lines.hpp"
#include "render/flow_sample.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::FlowVolume;
using spade::render::Camera;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kN = 32;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr float kDx = static_cast<float>(kTwoPi / kN);

// The Taylor-Green vortex of amplitude 1 on the periodic box [0, 2 pi)^3:
// u = sin x cos y cos z, v = -cos x sin y cos z, w = 0.
[[nodiscard]] glm::dvec3 tg_velocity(glm::dvec3 p) {
    return glm::dvec3(std::sin(p.x) * std::cos(p.y) * std::cos(p.z), -std::cos(p.x) * std::sin(p.y) * std::cos(p.z),
                      0.0);
}

// Q = |omega|^2 / 4 - |S|^2 / 2 = -tr(J^2) / 2 = cos^2 z (sin^2 x sin^2 y - cos^2 x cos^2 y).
[[nodiscard]] double tg_q(glm::dvec3 p) {
    const double sx = std::sin(p.x), cx = std::cos(p.x), sy = std::sin(p.y), cy = std::cos(p.y);
    const double cz = std::cos(p.z);
    return cz * cz * (sx * sx * sy * sy - cx * cx * cy * cy);
}

// omega = (-cos x sin y sin z, -sin x cos y sin z, 2 sin x sin y cos z).
[[nodiscard]] double tg_vorticity(glm::dvec3 p) {
    const double sx = std::sin(p.x), cx = std::cos(p.x), sy = std::sin(p.y), cy = std::cos(p.y);
    const double sz = std::sin(p.z), cz = std::cos(p.z);
    const glm::dvec3 w(-cx * sy * sz, -sx * cy * sz, 2.0 * sx * sy * cz);
    return std::sqrt(glm::dot(w, w));
}

[[nodiscard]] DomainBc periodic() {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    return bc;
}

// The vortex on plan A's grid, each face sample at its own position.
[[nodiscard]] MacGrid taylor_green_grid() {
    MacGrid g = *make_grid({kN, kN, kN, kDx}, periodic());
    for_each_sample(g.shape, 0, [&](std::size_t i, glm::vec3 p) { g.u[i] = static_cast<float>(tg_velocity(p).x); });
    for_each_sample(g.shape, 1, [&](std::size_t i, glm::vec3 p) { g.v[i] = static_cast<float>(tg_velocity(p).y); });
    return g;
}

[[nodiscard]] FlowVolume publish(const MacGrid& g) {
    FlowVolume out;
    PublishInput in;
    in.periodic = {true, true, true};  // the box wraps on every axis
    const auto counts = publish_render_volume(g, in, out);
    EXPECT_TRUE(counts.has_value());
    return out;
}

[[nodiscard]] glm::dvec3 cell_centre(uint32_t i, uint32_t j, uint32_t k) {
    return glm::dvec3((i + 0.5) * kDx, (j + 0.5) * kDx, (k + 0.5) * kDx);
}

[[nodiscard]] float channel(const FlowVolume& v, std::size_t cell, spade::FlowChannel c) {
    return spade::fp16::to_f32(spade::flow_half(v, cell, c));
}

constexpr uint32_t kW = 160, kH = 120;

[[nodiscard]] std::vector<uint8_t> draw(const RenderScene& scene, const Camera& camera) {
    std::vector<uint8_t> px(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kW, .height = kH, .stride = kW * 4u};
    RenderOptions options;
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    EXPECT_TRUE(r.has_value()) << r.error().context;
    return px;
}

// 8b's exact-field test: at t = 0 the published Q (q-hat |q-hat|) and |omega|
// at every cell centre against the analytic field. Publish differences the
// staggered faces, so the error is the discretisation's, O(dx^2), plus f16.
TEST(RenderFlowFeed, TaylorGreenPublishesItsAnalyticQAndVorticity) {
    const FlowVolume v = publish(taylor_green_grid());
    ASSERT_TRUE(spade::validate_flow_volume(v).has_value());
    EXPECT_EQ(v.header.no_data, 0u);
    // [0] interior cells, [1] cells on the grid's edge. Told the axes are
    // periodic, Publish differences the edge cells across the wrap.
    double q_err[2] = {0.0, 0.0}, w_err[2] = {0.0, 0.0};
    for (uint32_t k = 0; k < kN; ++k) {
        for (uint32_t j = 0; j < kN; ++j) {
            for (uint32_t i = 0; i < kN; ++i) {
                const bool edge = std::min({i, j, k}) == 0u || std::max({i, j, k}) == kN - 1u;
                const std::size_t cell = spade::flow_cell_index(v.header, i, j, k);
                const glm::dvec3 p = cell_centre(i, j, k);
                const double q_hat = channel(v, cell, spade::kFlowQHat);
                q_err[edge] = std::max(q_err[edge], std::abs(q_hat * std::abs(q_hat) - tg_q(p)));
                w_err[edge] =
                    std::max(w_err[edge], std::abs(channel(v, cell, spade::kFlowVorticity) - tg_vorticity(p)));
            }
        }
    }
    RecordProperty("q_err_interior_x1e6", static_cast<int>(q_err[0] * 1e6));
    RecordProperty("vorticity_err_interior_x1e6", static_cast<int>(w_err[0] * 1e6));
    RecordProperty("q_err_edge_x1e6", static_cast<int>(q_err[1] * 1e6));
    RecordProperty("vorticity_err_edge_x1e6", static_cast<int>(w_err[1] * 1e6));
    std::printf("[ tg t=0 ] interior: Q max error %.5f, |omega| %.5f; edge: Q %.5f, |omega| %.5f (peaks 1 and 2)\n",
                q_err[0], w_err[0], q_err[1], w_err[1]);
    // Measured 2026-10-12: inside, Q 0.0215 of its peak 1 and |omega| 0.0221
    // of its peak 2, the O(dx^2) of differences at dx = 2 pi / 32. The edge
    // cells hold the interior's bound now that Publish wraps periodic axes
    // (PublishInput::periodic). One-sided, they were 0.129 off in |omega|.
    EXPECT_LE(q_err[0], 0.022);
    EXPECT_LE(w_err[0], 0.023);
    EXPECT_LE(q_err[1], 0.022);
    EXPECT_LE(w_err[1], 0.023);
}

// The Q iso of the published field draws where the analytic field's does:
// the same scene, once from Publish and once from the analytic channels at
// the cell centres, through the CPU raster. They differ only where the
// discretisation moves the surface.
TEST(RenderFlowFeed, TheQIsoOfThePublishedTaylorGreenDrawsLikeTheAnalyticOne) {
    const FlowVolume published = publish(taylor_green_grid());
    const FlowVolume analytic = spade::test::make_flow_volume(glm::vec3(0.0f), kDx, kN, kN, kN, [](glm::vec3 p) {
        const glm::dvec3 d(p);
        const glm::dvec3 u = tg_velocity(d);
        const double q = tg_q(d);
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = static_cast<float>(u.x);
        c[spade::kFlowV] = static_cast<float>(u.y);
        c[spade::kFlowW] = static_cast<float>(u.z);
        c[spade::kFlowVorticity] = static_cast<float>(tg_vorticity(d));
        c[spade::kFlowQHat] = static_cast<float>(q < 0.0 ? -std::sqrt(-q) : std::sqrt(q));
        return c;
    });
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 1.0f;
    spade::render::IsoSurface iso;
    iso.quantity = spade::render::FlowQuantity::q_hat;
    iso.value = 0.5f;  // Q = 0.25, a quarter of the peak
    iso.colour_by = speed;
    const auto scene_of = [&](const FlowVolume& v) {
        RenderScene scene;
        scene.flow_layers.push_back(spade::render::FlowLayer{&v, {}, {iso}});
        return scene;
    };
    Camera camera;
    camera.position = glm::vec3(static_cast<float>(kTwoPi / 2.0), static_cast<float>(kTwoPi / 2.0), 13.0f);
    const std::vector<uint8_t> a = draw(scene_of(analytic), camera);
    const std::vector<uint8_t> b = draw(scene_of(published), camera);
    const std::vector<uint8_t> bare = draw(RenderScene{}, camera);
    size_t drawn = 0, differing = 0, over_two = 0;
    int worst = 0;
    for (size_t p = 0; p < static_cast<size_t>(kW) * kH; ++p) {
        int m = 0, m_bare = 0;
        for (size_t c = 0; c < 3; ++c) {
            m = std::max(m, std::abs(int{a[p * 4u + c]} - int{b[p * 4u + c]}));
            m_bare = std::max(m_bare, std::abs(int{a[p * 4u + c]} - int{bare[p * 4u + c]}));
        }
        drawn += m_bare > 0 ? 1u : 0u;
        differing += m > 0 ? 1u : 0u;
        over_two += m > 2 ? 1u : 0u;
        worst = std::max(worst, m);
    }
    ASSERT_GT(drawn, static_cast<size_t>(kW) * kH / 10u) << "the analytic iso must fill a good part of the frame";
    RecordProperty("drawn", static_cast<int>(drawn));
    RecordProperty("differing", static_cast<int>(differing));
    RecordProperty("over_two", static_cast<int>(over_two));
    std::printf("[ tg iso ] %zu px drawn; published against analytic: %zu differ, %zu over two, worst %d levels\n",
                drawn, differing, over_two, worst);
    // Measured 2026-10-12 at 160x120: of 3404 pixels the iso draws, 2202
    // differ, 259 by more than two levels. Looked at: the two frames show
    // the same four lobes, and the 259 lie on the lobes' silhouettes, where
    // a surface moved by the differences' O(dx^2) uncovers or covers a pixel.
    // The rest are a level or two of colour: the published velocity is the
    // faces' average, not the analytic value at the centre.
    EXPECT_LE(over_two, 259u);
    EXPECT_LE(differing, 2202u);
}

// At low Reynolds number the vortex decays as the viscous step damps its
// mode: velocity and peak |omega| by exp(-3 nu t) in the continuum, here by
// the backward-Euler step's own factor 1 / (1 + nu h k^2) per step, with the
// discrete k^2 = 3 (4 / dx^2) sin^2(dx / 2). The nonlinear transfer enters at
// O(t^2). The published peak |omega| after ten steps against that, at two
// sweep counts, so the band is the step's and not an unconverged solve's.
TEST(RenderFlowFeed, TaylorGreenDecaysAtTheViscousStepsRate) {
    const float nu = 1.0f, h = 0.01f;
    const int steps = 10;
    const double k2 = 3.0 * 4.0 / (static_cast<double>(kDx) * kDx) * std::pow(std::sin(kDx / 2.0), 2.0);
    const double expected = std::pow(1.0 / (1.0 + static_cast<double>(nu) * h * k2), steps);
    double ratio[2] = {0.0, 0.0};
    const uint32_t sweeps[2] = {8u, 32u};
    for (int s = 0; s < 2; ++s) {
        MacGrid g = taylor_green_grid();
        const float start = publish(g).header.channel_max[spade::kFlowVorticity];
        const FluidConfig cfg{.nu = nu, .h = h, .visc_sweeps = sweeps[s]};
        FluidScratch scratch = *make_fluid_scratch(g, periodic(), cfg);
        for (int n = 0; n < steps; ++n) {
            const FluidHealth health = fluid_step(g, periodic(), cfg, scratch);
            ASSERT_EQ(health.fault, FluidFault::none);
        }
        ratio[s] = publish(g).header.channel_max[spade::kFlowVorticity] / start;
    }
    RecordProperty("ratio_8_sweeps_x1e6", static_cast<int>(ratio[0] * 1e6));
    RecordProperty("ratio_32_sweeps_x1e6", static_cast<int>(ratio[1] * 1e6));
    std::printf("[ tg decay ] peak |omega| ratio after %d steps: %.5f (8 sweeps), %.5f (32 sweeps); step rate %.5f, "
                "continuum %.5f\n",
                steps, ratio[0], ratio[1], expected, std::exp(-3.0 * nu * h * steps));
    // Measured 2026-10-12: 0.74399 at both sweep counts, against the step's
    // 0.74479 (the continuum's exp(-0.3) is 0.74082). The 0.0008 left is the
    // nonlinear transfer and advection's own damping.
    EXPECT_LE(std::abs(ratio[0] - expected), 0.001);
    EXPECT_LE(std::abs(ratio[1] - ratio[0]), 1e-5);
}

// The actuator disc's wake (Task 9): a disc of radius R = 0.2 m, 0.9 m up an
// open 1.2 m box (24^3), pushing down with 1.2 N, for 70 steps of 5 ms: the
// fewest at which the wake reaches the bottom, kept small because debug
// runs this under ctest's 60 s limit (first sized at 32^3 and 80 steps, which
// took 65 s in debug; found by Core 2026-10-12). Plan
// section 4's rotor-seed check: seeds half a radius below the hub, on a ring
// of half the radius, move down faster than half the hover induced velocity
// v_h = sqrt(T / (2 rho pi R^2)), about 2 m/s here, and their streamlines
// leave the box through its bottom. The volume and Q iso views draw the
// wake from the published field, as smoke tests.
TEST(RenderFlowFeed, TheActuatorDiscsWakeCarriesTheRotorSeedsDown) {
    constexpr uint32_t n = 24;
    constexpr float dx = 0.05f;
    const DomainBc bc;  // open on every face, still air outside
    MacGrid g = *make_grid({n, n, n, dx}, bc);
    const ActuatorDisc disc{.centre = glm::vec3(0.6f, 0.9f, 0.6f),
                            .axis = glm::vec3(0.0f, -1.0f, 0.0f),
                            .radius = 0.2f,
                            .thrust = 1.2f};
    ASSERT_TRUE(check_disc(disc, g.shape).has_value());
    const FluidConfig cfg{.nu = 1.5e-5f, .h = 0.005f};
    FluidScratch scratch = *make_fluid_scratch(g, bc, cfg);
    for (int step = 0; step < 70; ++step) {
        spread_actuator(disc, g.shape, cfg.rho, cfg.h, scratch.du, scratch.dv, scratch.dw);
        ASSERT_EQ(fluid_step(g, bc, cfg, scratch).fault, FluidFault::none) << "step " << step;
    }
    const FlowVolume v = publish(g);
    ASSERT_TRUE(spade::validate_flow_volume(v).has_value());
    EXPECT_EQ(v.header.no_data, 0u);

    const double kPi = kTwoPi / 2.0;
    const float v_h = static_cast<float>(std::sqrt(disc.thrust / (2.0 * cfg.rho * kPi * disc.radius * disc.radius)));
    const std::vector<glm::vec3> seeds =
        spade::render::seed_ring(disc.centre + disc.axis * (0.5f * disc.radius), disc.axis, 0.5f * disc.radius, 12);
    const spade::render::FlowSampler sampler = spade::render::make_flow_sampler(v);
    float slowest = std::numeric_limits<float>::infinity();
    for (const glm::vec3& seed : seeds) {
        const spade::render::FlowVelocity u = spade::render::sample_velocity(sampler, seed);
        ASSERT_FALSE(u.no_data);
        slowest = std::min(slowest, glm::dot(u.u, disc.axis));
    }
    spade::render::StreamlineOptions trace;
    trace.backward = false;
    const auto lines = spade::render::trace_streamlines(v, seeds, trace);
    ASSERT_TRUE(lines.has_value());
    size_t out_the_bottom = 0;
    for (const spade::render::Streamline& line : *lines) {
        const bool left = line.forward_end == spade::render::LineEnd::left_box;
        out_the_bottom += left && line.points.back().y < 0.1f ? 1u : 0u;
    }
    RecordProperty("slowest_over_v_h_x1000", static_cast<int>(slowest / v_h * 1000.0f));
    std::printf("[ disc ] v_h %.3f m/s; slowest rotor seed moves down at %.3f m/s (%.2f v_h); %zu of %zu "
                "streamlines leave through the bottom\n",
                v_h, slowest, slowest / v_h, out_the_bottom, lines->size());
    EXPECT_GT(slowest, 0.5f * v_h) << "plan section 4: rotor seeds move down faster than half of v_h";
    EXPECT_EQ(out_the_bottom, lines->size()) << "every rotor seed's streamline leaves through the bottom";

    // Smoke tests: the published wake draws in the volume and Q iso views.
    spade::render::TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 2.0f * v_h;
    speed.extinction = {{0.25f * v_h, 0.0f}, {2.0f * v_h, 6.0f}};
    spade::render::IsoSurface q_iso;
    q_iso.quantity = spade::render::FlowQuantity::q_hat;
    q_iso.value = 0.3f * v.header.channel_max[spade::kFlowQHat];
    q_iso.colour_by = speed;
    ASSERT_GT(q_iso.value, 0.0f);
    Camera camera;
    camera.position = glm::vec3(0.6f, 0.6f, 2.6f);
    const std::vector<uint8_t> bare = draw(RenderScene{}, camera);
    for (const bool iso : {false, true}) {
        RenderScene scene;
        scene.flow_layers.push_back(iso ? spade::render::FlowLayer{&v, {}, {q_iso}}
                                        : spade::render::FlowLayer{&v, {speed}, {}});
        const std::vector<uint8_t> frame = draw(scene, camera);
        size_t drawn = 0;
        for (size_t p = 0; p < static_cast<size_t>(kW) * kH; ++p) {
            drawn += frame[p * 4u] != bare[p * 4u] || frame[p * 4u + 1u] != bare[p * 4u + 1u] ||
                             frame[p * 4u + 2u] != bare[p * 4u + 2u]
                         ? 1u
                         : 0u;
        }
        std::printf("[ disc ] %s view: %zu px drawn\n", iso ? "Q iso" : "volume", drawn);
        // Measured 2026-10-12: the volume draws 1578 pixels, an hourglass of
        // fast air through the disc; the Q iso 362, the disc's edge ring and
        // the starting vortex below it, seen edge on. Looked at. The floor
        // only asks that each view draw the wake at all.
        EXPECT_GT(drawn, 100u) << (iso ? "the Q iso must draw the wake" : "the volume must draw the wake");
    }
}

}  // namespace
