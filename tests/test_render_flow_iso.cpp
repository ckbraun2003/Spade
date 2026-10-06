// render/flow_render.hpp's iso-surfaces, against analytic fields (airflow
// visualization plan section 3):
// - a Lamb-Oseen vortex, whose Q-criterion iso-surface is a cylinder of a
//   radius known in closed form, Q(r) = (u_theta / r) du_theta / dr;
// - the Taylor-Green vortex at t = 0, Q = cos^2 z (sin^2 x sin^2 y -
//   cos^2 x cos^2 y), against a ray-cast of the analytic Q in double.
// Plus colour by another quantity, translucency, depth, and refusals.
// Test-only double and std::exp/sin/cos; this file feeds no golden.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_render.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"

namespace {

using spade::render::Camera;
using spade::render::FlowLayer;
using spade::render::FlowQuantity;
using spade::render::IsoSurface;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::TransferFunction;

constexpr double kPi = 3.14159265358979323846;

// --- Lamb-Oseen: a straight vortex along y, core a, circulation G. ------
constexpr double kA = 0.25, kG = 1.0;

[[nodiscard]] double lo_u_theta(double r) { return kG / (2.0 * kPi * r) * (1.0 - std::exp(-r * r / (kA * kA))); }

[[nodiscard]] double lo_q(double r) {
    if (r < 1e-9) return std::pow(kG / (2.0 * kPi * kA * kA), 2.0);
    const double g = std::exp(-r * r / (kA * kA));
    const double du = kG / (2.0 * kPi) * (-(1.0 - g) / (r * r) + 2.0 * g / (kA * kA));
    return lo_u_theta(r) / r * du;
}

[[nodiscard]] float q_hat(double q) { return static_cast<float>(q < 0.0 ? -std::sqrt(-q) : std::sqrt(q)); }

// The radius where Q falls to q0, by bisection (Q decreases from the axis).
[[nodiscard]] double lo_radius_of(double q0) {
    double lo = 0.0, hi = 1.12 * kA;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (lo_q(mid) > q0 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

// A box of half-extent 0.6 m about the origin, n cells a side.
[[nodiscard]] spade::FlowVolume lamb_oseen_volume(uint32_t n = 64) {
    const float half = 0.6f;
    return spade::test::make_flow_volume(glm::vec3(-half), 2.0f * half / static_cast<float>(n), n, n, n,
                                         [](glm::vec3 p) {
                                             spade::test::FlowChannels c{};
                                             const double r = std::sqrt(double{p.x} * p.x + double{p.z} * p.z);
                                             const double ut = r < 1e-9 ? 0.0 : lo_u_theta(r);
                                             if (r > 1e-9) {
                                                 c[spade::kFlowU] = static_cast<float>(-ut * p.z / r);
                                                 c[spade::kFlowW] = static_cast<float>(ut * p.x / r);
                                             }
                                             c[spade::kFlowVorticity] = static_cast<float>(
                                                 kG / (kPi * kA * kA) * std::exp(-r * r / (kA * kA)));
                                             c[spade::kFlowQHat] = q_hat(lo_q(r));
                                             return c;
                                         });
}

constexpr uint32_t kW = 97, kH = 97;

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::vec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::vec3(px[i + 2], px[i + 1], px[i + 0]);
    }
};

[[nodiscard]] Frame draw(const RenderScene& scene, const Camera& camera, spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    RenderOptions options;
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return f;
}

// Pixels the iso-surface covers: those that differ from the bare frame.
[[nodiscard]] std::vector<bool> covered(const Frame& with, const Frame& bare) {
    std::vector<bool> mask(static_cast<size_t>(kW) * kH);
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::vec3 d = glm::abs(with.rgb(x, y) - bare.rgb(x, y));
            mask[static_cast<size_t>(y) * kW + x] = std::max({d.x, d.y, d.z}) > 20.0f;
        }
    }
    return mask;
}

[[nodiscard]] Camera camera_at(glm::vec3 position) {
    Camera c;
    c.position = position;
    c.near_plane = 0.05f;
    c.far_plane = 100.0f;
    return c;
}

[[nodiscard]] IsoSurface red_iso(FlowQuantity q, float value) {
    IsoSurface iso;
    iso.quantity = q;
    iso.value = value;
    iso.colour = glm::vec3(1.0f, 0.1f, 0.1f);
    return iso;
}

// The Q iso-surface of a Lamb-Oseen vortex is a cylinder of radius r0 with
// Q(r0) = Q0. Seen from the side, a pixel is covered exactly when its ray
// passes within r0 of the axis; each row may differ at its two edges only.
TEST(RenderFlowIso, ALambOseenQIsoIsACylinderOfTheAnalyticRadius) {
    const spade::FlowVolume volume = lamb_oseen_volume();
    const double q0 = 0.5 * lo_q(0.0);
    const double r0 = lo_radius_of(q0);
    const Camera camera = camera_at(glm::vec3(0.0f, 0.0f, 1.0f));
    RenderScene scene;
    scene.flow_layers.push_back(FlowLayer{&volume, {}, {red_iso(FlowQuantity::q_hat, q_hat(q0))}});
    const std::vector<bool> got = covered(draw(scene, camera), draw(RenderScene{}, camera));
    const spade::render::RayGrid grid = spade::render::ray_grid(camera, kW, kH);
    uint32_t widest = 0;
    for (uint32_t y = 40; y <= 56; ++y) {
        uint32_t mismatched = 0, width = 0;
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::vec3 d = spade::render::camera_ray(grid, x, y).dir_world;
            // Distance from the y axis to the ray, in the xz-plane.
            const double dist = std::fabs(double{camera.position.z} * d.x) / std::sqrt(double{d.x} * d.x + double{d.z} * d.z);
            const bool want = dist < r0;
            width += want ? 1u : 0u;
            mismatched += (got[static_cast<size_t>(y) * kW + x] != want) ? 1u : 0u;
        }
        widest = std::max(widest, width);
        EXPECT_LE(mismatched, 2u) << "row " << y << ": at most one pixel at each edge";
    }
    EXPECT_GE(widest, 20u) << "the cylinder must be wide enough for the edges to mean something";
}

// |w| iso-surfaces are cylinders too, of radius a sqrt(ln(w_max / w0)).
TEST(RenderFlowIso, ALambOseenVorticityIsoIsACylinderOfTheAnalyticRadius) {
    const spade::FlowVolume volume = lamb_oseen_volume();
    const double w_max = kG / (kPi * kA * kA);
    const double w0 = 0.3 * w_max;
    const double r0 = kA * std::sqrt(std::log(w_max / w0));
    const Camera camera = camera_at(glm::vec3(0.0f, 0.0f, 1.0f));
    RenderScene scene;
    scene.flow_layers.push_back(FlowLayer{&volume, {}, {red_iso(FlowQuantity::vorticity, static_cast<float>(w0))}});
    const std::vector<bool> got = covered(draw(scene, camera), draw(RenderScene{}, camera));
    const spade::render::RayGrid grid = spade::render::ray_grid(camera, kW, kH);
    for (uint32_t y = 40; y <= 56; ++y) {
        uint32_t mismatched = 0;
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::vec3 d = spade::render::camera_ray(grid, x, y).dir_world;
            const double dist = std::fabs(double{camera.position.z} * d.x) / std::sqrt(double{d.x} * d.x + double{d.z} * d.z);
            mismatched += (got[static_cast<size_t>(y) * kW + x] != (dist < r0)) ? 1u : 0u;
        }
        EXPECT_LE(mismatched, 2u) << "row " << y;
    }
}

// --- Taylor-Green at t = 0 -----------------------------------------------
[[nodiscard]] double tgv_q(double x, double y, double z) {
    const double cz = std::cos(z);
    return cz * cz * (std::pow(std::sin(x) * std::sin(y), 2.0) - std::pow(std::cos(x) * std::cos(y), 2.0));
}

// The box [0, pi]^2 x [-pi/2, 3 pi/2]: Q <= 0 on every face, so Q's positive
// level sets are closed, two blobs about z = 0 and z = pi.
[[nodiscard]] spade::FlowVolume tgv_volume() {
    const float cell = static_cast<float>(kPi / 32.0);
    return spade::test::make_flow_volume(glm::vec3(0.0f, 0.0f, static_cast<float>(-kPi / 2.0)), cell, 32, 32, 64,
                                         [](glm::vec3 p) {
                                             spade::test::FlowChannels c{};
                                             c[spade::kFlowU] =
                                                 static_cast<float>(std::sin(p.x) * std::cos(p.y) * std::cos(p.z));
                                             c[spade::kFlowV] =
                                                 static_cast<float>(-std::cos(p.x) * std::sin(p.y) * std::cos(p.z));
                                             c[spade::kFlowQHat] = q_hat(tgv_q(p.x, p.y, p.z));
                                             return c;
                                         });
}

// The reference: each pixel's ray through the analytic Q in double, at a
// sixteenth of a cell, covered when Q crosses q0 inside the box.
[[nodiscard]] std::vector<bool> tgv_reference(const Camera& camera, double q0) {
    const spade::render::RayGrid grid = spade::render::ray_grid(camera, kW, kH);
    const glm::dvec3 lo(0.0, 0.0, -kPi / 2.0), hi(kPi, kPi, 3.0 * kPi / 2.0);
    const double h = kPi / 32.0 / 16.0;
    std::vector<bool> mask(static_cast<size_t>(kW) * kH);
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::dvec3 o(camera.position);
            const glm::dvec3 d(spade::render::camera_ray(grid, x, y).dir_world);
            double t0 = 0.0, t1 = 1e30;
            for (int a = 0; a < 3; ++a) {
                double ta = (lo[a] - o[a]) / d[a], tb = (hi[a] - o[a]) / d[a];
                if (ta > tb) std::swap(ta, tb);
                t0 = std::max(t0, ta);
                t1 = std::min(t1, tb);
            }
            bool hit = false;
            double prev = std::numeric_limits<double>::quiet_NaN();
            for (double t = t0; t <= t1 && !hit; t += h) {
                const glm::dvec3 p = o + d * t;
                const double f = tgv_q(p.x, p.y, p.z) - q0;
                hit = !std::isnan(prev) && (prev < 0.0) != (f < 0.0);
                prev = f;
            }
            mask[static_cast<size_t>(y) * kW + x] = hit;
        }
    }
    return mask;
}

// A closed 3D topology against the analytic ray-cast. The band is measured
// (MSVC release, 2026-10-06) and recorded in the assertion.
TEST(RenderFlowIso, ATaylorGreenQIsoMatchesTheAnalyticRayCast) {
    const spade::FlowVolume volume = tgv_volume();
    const double q0 = 0.1;
    const glm::vec3 centre(static_cast<float>(kPi / 2.0));
    const glm::vec3 eye = centre + 6.5f * glm::normalize(glm::vec3(0.9f, 0.35f, 0.3f));
    Camera camera = camera_at(eye);
    camera.orientation = glm::quatLookAt(glm::normalize(centre - eye), glm::vec3(0.0f, 1.0f, 0.0f));
    RenderScene scene;
    scene.flow_layers.push_back(FlowLayer{&volume, {}, {red_iso(FlowQuantity::q_hat, q_hat(q0))}});
    const std::vector<bool> got = covered(draw(scene, camera), draw(RenderScene{}, camera));
    const std::vector<bool> want = tgv_reference(camera, q0);
    size_t hits = 0, mismatched = 0;
    for (size_t i = 0; i < want.size(); ++i) {
        hits += want[i] ? 1u : 0u;
        mismatched += got[i] != want[i] ? 1u : 0u;
    }
    std::printf("[ TGV iso ] analytic hits %zu, mismatched %zu\n", hits, mismatched);
    EXPECT_GE(hits, 500u) << "the blobs must fill enough of the frame to mean something";
    EXPECT_LE(mismatched, hits / 50u) << "within 2% of the analytic coverage";
}

// --- Colour, translucency, depth -----------------------------------------

// Colour by speed: the hit takes the speed map's colour at the hit (unlit).
TEST(RenderFlowIso, AnIsoColouredBySpeedTakesTheSpeedColourAtTheHit) {
    const spade::FlowVolume volume = lamb_oseen_volume();
    const double q0 = 0.5 * lo_q(0.0);
    const double r0 = lo_radius_of(q0);
    IsoSurface iso = red_iso(FlowQuantity::q_hat, q_hat(q0));
    TransferFunction speed;
    speed.range_min = 0.0f;
    speed.range_max = 1.0f;
    iso.colour_by = speed;
    const auto prepared = spade::render::prepare_flow_layer(FlowLayer{&volume, {}, {iso}});
    const auto r = spade::render::march_flow_ray(prepared, glm::vec3(0.0f, 0.0f, 1.0f),
                                                 glm::vec3(0.0f, 0.0f, -1.0f), 0.0f, 2.0f);
    const glm::vec3 want = spade::render::colour_map_at(spade::render::ColourMapId::viridis,
                                                        static_cast<float>(lo_u_theta(r0)));
    EXPECT_NEAR(r.colour.x, want.x, 0.01f);
    EXPECT_NEAR(r.colour.y, want.y, 0.01f);
    EXPECT_NEAR(r.colour.z, want.z, 0.01f);
    EXPECT_EQ(r.transmittance, 0.0f);
    EXPECT_NEAR(r.opaque_t, 1.0 - r0, 0.004) << "the hit is the cylinder's near side";
}

// A translucent iso composites at each crossing and lets the ray go on; an
// opaque one ends it.
TEST(RenderFlowIso, TranslucentIsosCompositeAndOpaqueOnesEndTheRay) {
    const spade::FlowVolume volume = lamb_oseen_volume();
    IsoSurface iso = red_iso(FlowQuantity::q_hat, q_hat(0.5 * lo_q(0.0)));
    iso.colour = glm::vec3(0.4f, 0.8f, 0.2f);
    iso.opacity = 0.5f;
    const auto prepared = spade::render::prepare_flow_layer(FlowLayer{&volume, {}, {iso}});
    const auto r = spade::render::march_flow_ray(prepared, glm::vec3(0.0f, 0.0f, 1.0f),
                                                 glm::vec3(0.0f, 0.0f, -1.0f), 0.0f, 2.0f);
    // Two crossings on the axis, into the cylinder and out: each passes half.
    EXPECT_NEAR(r.transmittance, 0.25f, 1e-6f);
    EXPECT_NEAR(r.colour.y, 0.75f * 0.8f, 1e-6f);
    EXPECT_EQ(r.opaque_t, std::numeric_limits<float>::infinity());
}

// Depth both ways: an opaque surface in front hides the iso; the iso hides
// a dense absorber behind it, because an opaque hit writes depth.
TEST(RenderFlowIso, AnOpaqueIsoIsDepthTestedAndWritesDepth) {
    const spade::FlowVolume volume = lamb_oseen_volume();
    const Camera camera = camera_at(glm::vec3(0.0f, 0.0f, 1.0f));
    const IsoSurface iso = red_iso(FlowQuantity::q_hat, q_hat(0.5 * lo_q(0.0)));
    RenderScene alone;
    alone.flow_layers.push_back(FlowLayer{&volume, {}, {iso}});
    const glm::vec3 iso_px = draw(alone, camera).rgb(48, 48);

    spade::render::FieldLayer wall;
    wall.center = glm::vec3(0.0f, 0.0f, 0.8f);
    wall.values = {0.0f};
    RenderScene hidden = alone;
    hidden.field_layers.push_back(wall);
    RenderScene wall_only;
    wall_only.field_layers.push_back(wall);
    EXPECT_EQ(draw(hidden, camera).rgb(48, 48), draw(wall_only, camera).rgb(48, 48)) << "a surface in front hides the iso";

    const spade::FlowVolume slab = spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.5f), 0.1f, 40, 40, 5,
                                                                 spade::test::uniform_flow({1.0f, 0.0f, 0.0f}));
    TransferFunction black;
    black.constant_colour = glm::vec3(0.0f);
    black.extinction = {{0.0f, 50.0f}, {1.0f, 50.0f}};
    RenderScene behind = alone;
    behind.flow_layers.push_back(FlowLayer{&slab, {black}, {}});
    EXPECT_EQ(draw(behind, camera).rgb(48, 48), iso_px) << "air behind an opaque iso does not reach it";
}

// L6: a malformed iso-surface, or a layer that draws nothing, is refused.
TEST(RenderFlowIso, AMalformedIsoIsRefused) {
    const spade::FlowVolume volume = lamb_oseen_volume(8);
    const auto refused = [&](const FlowLayer& layer, const char* what) {
        const auto r = spade::render::validate_flow_layer(layer);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
    };
    const IsoSurface good = red_iso(FlowQuantity::q_hat, 1.0f);
    ASSERT_TRUE(spade::render::validate_flow_layer(FlowLayer{&volume, {}, {good}}).has_value());
    refused(FlowLayer{&volume, {}, {}}, "a layer with neither transfer functions nor isos");
    refused(FlowLayer{&volume, {}, {good, good, good, good, good}}, "five isos");
    IsoSurface bad = good;
    bad.value = std::numeric_limits<float>::quiet_NaN();
    refused(FlowLayer{&volume, {}, {bad}}, "a NaN level");
    bad = good;
    bad.opacity = 0.0f;
    refused(FlowLayer{&volume, {}, {bad}}, "zero opacity");
    bad.opacity = 1.5f;
    refused(FlowLayer{&volume, {}, {bad}}, "opacity past 1");
    bad = good;
    bad.colour = glm::vec3(-0.1f, 0.0f, 0.0f);
    refused(FlowLayer{&volume, {}, {bad}}, "a colour below 0");
    bad = good;
    TransferFunction broken;
    broken.range_max = broken.range_min;
    bad.colour_by = broken;
    refused(FlowLayer{&volume, {}, {bad}}, "a colour_by that fails validation");
}

}  // namespace
