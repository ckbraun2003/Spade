// P10: the passive dye (engine/physics/airflow/dye.hpp, advect.hpp's advect_scalar, publish's V channel 3) on the CPU, and the
// proof that drew it. See .superpowers/sdd/2026-11-10-wake/p10-dye-sizing.md for the scheme.
//
// The cheap tests run in the normal suite: the advection's maximum principle, a uniform dye staying uniform to the bit, a
// one-cell-per-step translation, the emitter's bounds and ring, and the publish into channel 3. The proof itself is opt-in:
// SPADE_DYE_PROOF_OUT=<folder> runs the reference quad's hover case (tools/flow_movie/quad_case.cpp: four swirling discs,
// 80 x 128 x 80 at 1 cm, h 0.5 ms, Cs 0.17, still air) for SPADE_DYE_STEPS steps (default 1200 = 0.6 s) with two dyes beside
// each other (a full-disc release and an r/R 0.85-1.0 ring release), and writes stills (PPM) at the web S2 camera on pure black.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/fp16.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/rotor_discs.hpp"
#include "render/flow_look.hpp"
#include "render/flow_render.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace rd = spade::render;

// -- the cheap tests --------------------------------------------------------------------------------------------------

[[nodiscard]] af::MacGrid small_grid(uint32_t n, float dx) {
    auto g = af::make_grid(af::GridShape{n, n, n, dx}, af::DomainBc{});
    EXPECT_TRUE(g.has_value());
    return std::move(*g);
}

struct Lcg {
    uint32_t state = 12345u;
    float next() {  // [0, 1)
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 16777216.0f;
    }
};

TEST(AirflowDye, AUniformDyeStaysUniformToTheBitWhateverTheFlow) {
    af::MacGrid g = small_grid(12, 0.01f);
    Lcg rng;
    for (std::vector<float>* a : {&g.u, &g.v, &g.w}) {
        for (float& x : *a) x = 6.0f * (rng.next() - 0.5f);
    }
    const std::size_t cells = 12u * 12u * 12u;
    std::vector<float> in(cells, 0.7f), out(cells, -1.0f);
    af::ScalarScratch s = af::make_scalar_scratch(g.shape);
    af::advect_scalar(g, in, out, af::DomainBc{}, 0.002f, s);
    for (const float v : out) ASSERT_EQ(v, 0.7f);
}

TEST(AirflowDye, TheAdvectedDyeNeverLeavesTheRangeOfTheOldFieldNorGoesNegative) {
    af::MacGrid g = small_grid(16, 0.01f);
    Lcg rng;
    for (std::vector<float>* a : {&g.u, &g.v, &g.w}) {
        for (float& x : *a) x = 8.0f * (rng.next() - 0.5f);
    }
    const std::size_t cells = 16u * 16u * 16u;
    std::vector<float> a(cells), b(cells);
    for (float& x : a) x = rng.next() < 0.3f ? rng.next() : 0.0f;  // a ragged cloud with clear air in it
    const float top = *std::max_element(a.begin(), a.end());
    af::ScalarScratch s = af::make_scalar_scratch(g.shape);
    for (int step = 0; step < 20; ++step) {
        af::advect_scalar(g, a, b, af::DomainBc{}, 0.002f, s);
        a.swap(b);
        for (const float v : a) {
            ASSERT_GE(v, 0.0f) << "step " << step;
            ASSERT_LE(v, top) << "step " << step;
        }
    }
}

TEST(AirflowDye, ACourantNumberOfOneMovesTheDyeExactlyOneCell) {
    af::MacGrid g = small_grid(16, 0.01f);
    std::fill(g.u.begin(), g.u.end(), 0.5f);  // dx / h
    Lcg rng;
    const std::size_t cells = 16u * 16u * 16u;
    std::vector<float> in(cells), out(cells);
    for (float& x : in) x = rng.next();
    af::ScalarScratch s = af::make_scalar_scratch(g.shape);
    af::advect_scalar(g, in, out, af::DomainBc{}, 0.02f, s);
    for (uint32_t k = 0; k < 16; ++k) {
        for (uint32_t j = 0; j < 16; ++j) {
            for (uint32_t i = 2; i < 15; ++i) {  // the last cell's forward trace leaves the box and clamps: bounded, not exact
                ASSERT_NEAR(out[i + 16u * (j + 16u * k)], in[(i - 1) + 16u * (j + 16u * k)], 2e-5f) << i << " " << j << " " << k;
            }
        }
    }
}

TEST(AirflowDye, TheReleaseStaysInZeroToOneIsOneSidedAndTheRingLeavesTheMiddleClear) {
    const af::GridShape shape{40, 40, 40, 0.005f};
    af::ActuatorDisc disc;
    disc.centre = glm::vec3(0.1f, 0.1f, 0.1f);
    disc.axis = glm::vec3(0.0f, -1.0f, 0.0f);  // pushes the air down: downstream is -y
    disc.radius = 0.06f;
    const af::DyeRelease whole{120.0f, 0.0f, 1.0f}, ring{120.0f, 0.85f, 1.0f};
    ASSERT_TRUE(af::check_dye_release(whole).has_value());
    ASSERT_TRUE(af::check_dye_release(ring).has_value());
    std::vector<float> full(40u * 40u * 40u, 0.0f), rim = full;
    for (int step = 0; step < 200; ++step) {
        af::emit_dye(full, shape, std::span<const af::ActuatorDisc>(&disc, 1), 0.0005f, whole);
        af::emit_dye(rim, shape, std::span<const af::ActuatorDisc>(&disc, 1), 0.0005f, ring);
    }
    float full_max = 0.0f, ring_max = 0.0f, ring_middle = 0.0f, outside = 0.0f, upstream = 0.0f;
    for (uint32_t k = 0; k < 40; ++k) {
        for (uint32_t j = 0; j < 40; ++j) {
            for (uint32_t i = 0; i < 40; ++i) {
                const std::size_t idx = i + 40u * (j + 40u * k);
                const glm::vec3 p((static_cast<float>(i) + 0.5f) * 0.005f, (static_cast<float>(j) + 0.5f) * 0.005f,
                                  (static_cast<float>(k) + 0.5f) * 0.005f);
                const float r = std::hypot(p.x - 0.1f, p.z - 0.1f);
                const float below = 0.1f - p.y;  // along the axis
                EXPECT_GE(full[idx], 0.0f);
                EXPECT_LE(full[idx], 1.0f);
                EXPECT_LE(rim[idx], 1.0f);
                full_max = std::max(full_max, full[idx]);
                ring_max = std::max(ring_max, rim[idx]);
                if (r < 0.5f * 0.06f) ring_middle = std::max(ring_middle, rim[idx]);
                if (below < 0.0f) upstream = std::max({upstream, full[idx], rim[idx]});
                if (below >= 2.0f * 0.005f || r > 0.06f + 0.005f) outside = std::max({outside, full[idx], rim[idx]});
            }
        }
    }
    EXPECT_GT(full_max, 0.5f);
    EXPECT_GT(ring_max, 0.5f);
    EXPECT_EQ(ring_middle, 0.0f) << "an r/R 0.85-1 ring leaves the disc's middle clear";
    EXPECT_EQ(upstream, 0.0f) << "no dye upstream of the plane: no halo above the disc";
    EXPECT_EQ(outside, 0.0f) << "nothing beyond the sheet's two cells or the rim";
    EXPECT_FALSE(af::check_dye_release(af::DyeRelease{-1.0f, 0.0f, 1.0f}).has_value());
    EXPECT_FALSE(af::check_dye_release(af::DyeRelease{1.0f, 0.9f, 0.9f}).has_value());
}

// Passive: the dye never acts on the flow. The same discs and steps with the dye off and with it on and released leave u, v, w and
// p the same bits.
TEST(AirflowDye, TheFlowIsTheSameBitsWithTheDyeOnOrOff) {
    const af::GridShape shape{24, 32, 24, 0.01f};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.0005f, .n_v = 2, .sgs_cs = 0.17f};
    af::ActuatorDisc disc;
    disc.centre = glm::vec3(0.12f, 0.2f, 0.12f);
    disc.axis = glm::vec3(0.0f, -1.0f, 0.0f);
    disc.radius = 0.05f;
    disc.thrust = 0.5f;
    disc.torque = 0.003f;
    const af::DyeRelease release{};
    std::array<std::array<std::vector<float>, 4>, 2> fields;  // u, v, w, p of each run
    std::array<std::vector<float>, 2> dyes;
    for (int with_dye = 0; with_dye < 2; ++with_dye) {
        auto g = af::make_grid(shape, bc);
        ASSERT_TRUE(g.has_value());
        auto scratch = af::make_fluid_scratch(*g, bc, cfg);
        ASSERT_TRUE(scratch.has_value());
        if (with_dye != 0) {
            ASSERT_TRUE(af::enable_dye(*scratch, *g).has_value());
        }
        for (int step = 0; step < 30; ++step) {
            af::spread_actuators(std::span<const af::ActuatorDisc>(&disc, 1), shape, cfg.rho, cfg.h, *scratch);
            ASSERT_EQ(af::fluid_step(*g, bc, cfg, *scratch).fault, af::FluidFault::none);
            if (with_dye != 0) af::emit_dye(scratch->dye, shape, std::span<const af::ActuatorDisc>(&disc, 1), cfg.h, release);
        }
        dyes[static_cast<std::size_t>(with_dye)] = scratch->dye;
        fields[static_cast<std::size_t>(with_dye)] = {g->u, g->v, g->w, g->p};
    }
    const auto same = [](const std::vector<float>& a, const std::vector<float>& b) {
        return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };
    EXPECT_TRUE(same(fields[0][0], fields[1][0])) << "u differs with the dye on";
    EXPECT_TRUE(same(fields[0][1], fields[1][1])) << "v differs with the dye on";
    EXPECT_TRUE(same(fields[0][2], fields[1][2])) << "w differs with the dye on";
    EXPECT_TRUE(same(fields[0][3], fields[1][3])) << "p differs with the dye on";
    EXPECT_TRUE(dyes[0].empty());
    EXPECT_GT(*std::max_element(dyes[1].begin(), dyes[1].end()), 0.0f) << "the dye is not vacuous";
    EXPECT_LE(*std::max_element(dyes[1].begin(), dyes[1].end()), 1.0f);
    EXPECT_GE(*std::min_element(dyes[1].begin(), dyes[1].end()), 0.0f);
    // A second enable is refused by name.
    auto g2 = af::make_grid(shape, bc);
    auto s2 = af::make_fluid_scratch(*g2, bc, cfg);
    ASSERT_TRUE(af::enable_dye(*s2, *g2).has_value());
    EXPECT_FALSE(af::enable_dye(*s2, *g2).has_value());
}

TEST(AirflowDye, ThePublishPutsTheDyeInChannelThreeAndItsRangeInTheHeader) {
    af::MacGrid g = small_grid(8, 0.01f);
    std::vector<float> dye(8u * 8u * 8u, 0.0f);
    dye[3 + 8u * (4 + 8u * 5)] = 0.75f;
    dye[0] = 0.25f;
    af::PublishInput in;
    in.dye = dye;
    spade::FlowVolume volume;
    ASSERT_TRUE(af::publish_render_volume(g, in, volume).has_value());
    EXPECT_EQ(spade::fp16::to_f32(volume.v[(3 + 8u * (4 + 8u * 5)) * 4u + spade::kFlowDye]), 0.75f);
    EXPECT_EQ(spade::fp16::to_f32(volume.v[0 * 4u + spade::kFlowDye]), 0.25f);
    EXPECT_EQ(volume.header.channel_max[spade::kFlowDye], 0.75f);
    EXPECT_EQ(volume.header.channel_min[spade::kFlowDye], 0.0f);
    // No dye given: channel 3 is 0 as it always was.
    spade::FlowVolume plain;
    ASSERT_TRUE(af::publish_render_volume(g, af::PublishInput{}, plain).has_value());
    EXPECT_EQ(plain.header.channel_max[spade::kFlowDye], 0.0f);
    // A dye of the wrong size is refused by name.
    std::vector<float> wrong(7, 0.0f);
    in.dye = wrong;
    const auto refused = af::publish_render_volume(g, in, volume);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("dye"), std::string::npos);
}

// -- the proof --------------------------------------------------------------------------------------------------------

[[nodiscard]] const char* env_cstr(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

void write_ppm(const std::string& path, const std::vector<uint8_t>& bgrx, uint32_t w, uint32_t h) {
    std::ofstream f(path, std::ios::binary);
    f << "P6" << '\n' << w << ' ' << h << '\n' << "255" << '\n';
    std::vector<char> row(static_cast<std::size_t>(w) * 3u);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 4u;
            row[3u * x + 0] = static_cast<char>(bgrx[i + 2]);
            row[3u * x + 1] = static_cast<char>(bgrx[i + 1]);
            row[3u * x + 2] = static_cast<char>(bgrx[i + 0]);
        }
        f.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
}

// views.cpp's orbit(): the camera `distance` from `target` at an azimuth about +y (0 on +z) and an elevation.
[[nodiscard]] rd::Camera orbit(glm::vec3 target, float distance, float azimuth_deg, float elevation_deg) {
    const float az = glm::radians(azimuth_deg), el = glm::radians(elevation_deg);
    const glm::vec3 from(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az));
    rd::Camera camera;
    camera.position = target + distance * from;
    camera.orientation = glm::quatLookAt(-from, glm::vec3(0.0f, 1.0f, 0.0f));
    camera.near_plane = 0.01f * distance;
    camera.far_plane = 100.0f * distance;
    return camera;
}

struct Look {
    const char* name;
    rd::TransferFunction tf;
};

[[nodiscard]] float env_float(const char* name, float fallback) {
    const char* v = env_cstr(name);
    return v != nullptr ? std::strtof(v, nullptr) : fallback;
}

// The engine's dye transfer function (inferno), and a smoke one: a constant light grey, extinction rising with the dye.
// SPADE_DYE_SIGMA is the extinction at dye 1 in 1/m (default 12), SPADE_DYE_ONSET where it starts (default 0.03).
[[nodiscard]] std::vector<Look> dye_looks() {
    const float sigma = env_float("SPADE_DYE_SIGMA", 12.0f);
    const float onset = env_float("SPADE_DYE_ONSET", 0.03f);
    std::vector<Look> v;
    v.push_back({"dye", rd::volume_transfer(rd::FlowQuantity::dye, rd::ColourMapId::inferno, 1.0f, onset, sigma)});
    rd::TransferFunction smoke;
    smoke.quantity = rd::FlowQuantity::dye;
    smoke.constant_colour = glm::vec3(0.90f, 0.92f, 0.95f);
    smoke.range_min = 0.0f;
    smoke.range_max = 1.0f;
    smoke.extinction = {{0.0f, 0.0f}, {onset, 0.0f}, {1.0f, sigma}};
    v.push_back({"smoke", smoke});
    return v;
}

// One still of `dye` published into a volume over `grid` (its velocities are irrelevant to a dye transfer function) and drawn at
// the web S2 camera (web_masters.hpp: the aim 0.08 m below the COM, azimuth 25, 8 degrees down, 60 degrees fov), black sky.
void draw_still(const af::MacGrid& grid, glm::vec3 origin, const std::vector<float>& dye, const std::string& path,
                const Look& look, float radius_m) {
    const af::GridShape& shape = grid.shape;
    af::PublishInput in;
    in.origin = origin;
    in.dye = dye;
    spade::FlowVolume volume;
    const auto published = af::publish_render_volume(grid, in, volume);
    ASSERT_TRUE(published.has_value()) << published.error().context;
    rd::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    scene.bounds = rd::Aabb{.min = origin, .max = origin + glm::vec3(shape.nx, shape.ny, shape.nz) * shape.dx};
    scene.flow_layers.push_back(rd::FlowLayer{&volume, {look.tf}, {}});
    const float distance = radius_m / std::sin(glm::radians(30.0f)) * 1.1f;
    const rd::Camera camera = orbit(glm::vec3(0.0f, -0.08f, 0.0f), distance, 25.0f, 8.0f);
    constexpr uint32_t kW = 1280, kH = 720;
    std::vector<uint8_t> px(static_cast<std::size_t>(kW) * kH * 4u, 0u);
    rd::RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kW, .height = kH, .stride = kW * 4u};
    rd::RenderOptions options;
    options.overlays = false;
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = rd::render(scene, camera, options, target);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    write_ppm(path, px, kW, kH);
    std::printf("still %s: dye max %.3f, drawn in %.1f s\n", path.c_str(), volume.header.channel_max[spade::kFlowDye],
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

void save_floats(const std::string& path, const std::vector<float>& v) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(float)));
}

[[nodiscard]] std::vector<float> load_floats(const std::string& path, std::size_t n) {
    std::vector<float> v(n, 0.0f);
    std::ifstream f(path, std::ios::binary);
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(n * sizeof(float)));
    return v;
}

TEST(AirflowDye, TheQuadsDyeOnTheCpuAndItsStills) {
    const char* out_dir = env_cstr("SPADE_DYE_PROOF_OUT");
    if (out_dir == nullptr) GTEST_SKIP() << "opt-in: SPADE_DYE_PROOF_OUT=<folder> (CPU, minutes)";
    const char* steps_env = env_cstr("SPADE_DYE_STEPS");
    const uint32_t steps = steps_env != nullptr ? static_cast<uint32_t>(std::strtoul(steps_env, nullptr, 10)) : 1200u;

    // The reference quad's hover case, tools/flow_movie/quad_case.cpp's numbers.
    constexpr float kThrust = 1.28957453f, kTorque = 0.0131021f, kRadius = 0.0762f, kHubY = 0.0241444856f;
    const std::array<glm::vec3, 4> hubs{glm::vec3(0.11f, kHubY, 0.0f), glm::vec3(0.0f, kHubY, 0.11f),
                                        glm::vec3(-0.11f, kHubY, 0.0f), glm::vec3(0.0f, kHubY, -0.11f)};
    const std::array<float, 4> spins{+1.0f, -1.0f, +1.0f, -1.0f};
    const glm::vec3 com(0.40f, 0.96f - kHubY, 0.40f);
    const af::GridShape shape{80, 128, 80, 0.01f};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.0005f, .n_v = 2, .sgs_cs = 0.17f};
    const glm::vec3 origin = -com;
    std::array<af::DiscRotor, 4> rotors{};
    for (std::size_t i = 0; i < 4; ++i) {
        rotors[i] = af::DiscRotor{.hub_body = hubs[i], .radius = kRadius, .spin_dir = spins[i], .thrust = kThrust,
                                  .torque = kTorque};
    }
    std::vector<af::ActuatorDisc> discs(4);
    ASSERT_TRUE(af::discs_from_rotors(rotors, af::DiscBody{}, origin, shape, discs).has_value());

    auto grid = af::make_grid(shape, bc);
    ASSERT_TRUE(grid.has_value()) << grid.error().context;
    auto scratch = af::make_fluid_scratch(*grid, bc, cfg);
    ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
    const std::size_t cells = std::size_t{shape.nx} * shape.ny * shape.nz;
    std::vector<float> full(cells, 0.0f), ring(cells, 0.0f), next(cells, 0.0f);
    af::ScalarScratch dye_scratch = af::make_scalar_scratch(shape);
    const float rate = env_float("SPADE_DYE_RATE", 120.0f);  // 1/s
    const af::DyeRelease full_emit{rate, 0.0f, 1.0f}, ring_emit{rate, 0.85f, 1.0f};

    const auto draw = [&](const std::vector<float>& dye, const std::string& stem, const Look& look, float radius_m) {
        draw_still(*grid, origin, dye, std::string(out_dir) + "/" + stem + ".ppm", look, radius_m);
    };
    const std::vector<Look> looks = dye_looks();

    const auto wall0 = std::chrono::steady_clock::now();
    for (uint32_t step = 1; step <= steps; ++step) {
        af::spread_actuators(discs, shape, cfg.rho, cfg.h, *scratch);
        const af::FluidHealth health = af::fluid_step(*grid, bc, cfg, *scratch);
        ASSERT_EQ(health.fault, af::FluidFault::none) << "step " << step;
        for (std::vector<float>* dye : {&full, &ring}) {
            af::advect_scalar(*grid, *dye, next, bc, cfg.h, dye_scratch);
            dye->swap(next);
        }
        af::emit_dye(full, shape, discs, cfg.h, full_emit);
        af::emit_dye(ring, shape, discs, cfg.h, ring_emit);
        if (step % 200u == 0u || step == steps) {
            float fmax = 0.0f, rmax = 0.0f, fmin = 1.0f;
            double fsum = 0.0, rsum = 0.0;
            for (std::size_t c = 0; c < cells; ++c) {
                fmax = std::max(fmax, full[c]);
                rmax = std::max(rmax, ring[c]);
                fmin = std::min(fmin, full[c]);
                fsum += full[c];
                rsum += ring[c];
            }
            std::printf("step %u (t %.3f s): full dye max %.3f min %.3g, ring max %.3f; dye volume %.1f / %.1f cells; "
                        "residual %.2e; %.0f s wall\n",
                        step, step * cfg.h, fmax, fmin, rmax, fsum, rsum, health.pressure_residual,
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count());
            EXPECT_GE(fmin, 0.0f);
            EXPECT_LE(fmax, 1.0f);
            EXPECT_LE(rmax, 1.0f);
        }
        if (step == steps / 2u) {
            save_floats(std::string(out_dir) + "/mid-full.f32", full);
            save_floats(std::string(out_dir) + "/mid-ring.f32", ring);
            draw(full, "mid-full-smoke", looks[1], 0.375f);
            draw(ring, "mid-ring-smoke", looks[1], 0.375f);
        }
    }
    save_floats(std::string(out_dir) + "/final-full.f32", full);
    save_floats(std::string(out_dir) + "/final-ring.f32", ring);
    for (const Look& look : looks) {
        draw(full, std::string("full-") + look.name, look, 0.375f);
        draw(ring, std::string("ring-") + look.name, look, 0.375f);
    }
    draw(full, "full-smoke-wide", looks[1], 0.60f);
    draw(ring, "ring-smoke-wide", looks[1], 0.60f);
}

// Opt-in: the saved dye fields drawn again with other transfer-function numbers (SPADE_DYE_SIGMA, SPADE_DYE_ONSET), no solve.
// SPADE_DYE_RENDER_IN=<folder with the proof's .f32 files>, SPADE_DYE_RENDER_OUT=<folder>, SPADE_DYE_RENDER_TAG=<name suffix>.
TEST(AirflowDye, TheSavedDyeDrawnAgainWithOtherNumbers) {
    const char* in_dir = env_cstr("SPADE_DYE_RENDER_IN");
    const char* out_dir = env_cstr("SPADE_DYE_RENDER_OUT");
    if (in_dir == nullptr || out_dir == nullptr) GTEST_SKIP() << "opt-in: SPADE_DYE_RENDER_IN and SPADE_DYE_RENDER_OUT";
    const char* tag_env = env_cstr("SPADE_DYE_RENDER_TAG");
    const std::string tag = tag_env != nullptr ? tag_env : "again";
    const af::GridShape shape{80, 128, 80, 0.01f};
    const glm::vec3 origin = -glm::vec3(0.40f, 0.96f - 0.0241444856f, 0.40f);
    auto grid = af::make_grid(shape, af::DomainBc{});
    ASSERT_TRUE(grid.has_value());
    const std::size_t cells = std::size_t{shape.nx} * shape.ny * shape.nz;
    const std::vector<Look> looks = dye_looks();
    for (const char* name : {"final-full", "final-ring", "mid-full", "mid-ring"}) {
        const std::vector<float> dye = load_floats(std::string(in_dir) + "/" + name + ".f32", cells);
        for (const Look& look : looks) {
            draw_still(*grid, origin, dye, std::string(out_dir) + "/" + name + "-" + look.name + "-" + tag + ".ppm", look, 0.375f);
        }
    }
}

}  // namespace
