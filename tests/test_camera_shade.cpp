// test_camera_shade.cpp -- the camera's shading: one ray to a linear colour, and a linear colour to bytes (camera sensor Phase 1, P1.3; design section 5, "one pixel").
//
// WHAT IS PINNED. The CPU reference camera shades with the scene's ONE definition of a lit pixel, ported not reimplemented: shade_vertex_color (Lambert + ambient,
// unlit and emissive echo the base colour), sky_gradient_color on a miss, and horizon_blend (SR-17a, off at strength 0, which is exact) over the surface at range. A
// back face is lit by the normal that faces the ray. The pixel then takes the exposure gain (linear, before the clamp), the sensor mode (colour: 3 channels; grey: 1,
// luminance Y = 0.2126 R + 0.7152 G + 0.0722 B of the exposed linear colour) and the sRGB transfer curve to a byte, rounded to nearest. The engine has no pow: the
// encode is the exact round-to-nearest of the curve through a table of thresholds (render needs no interpolation to be exact). THIS FILE's references use libm on
// purpose, as an independent check.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "render/camera_shade.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"

namespace rd = spade::render;

namespace {

// A quad in the z = 0 plane facing +z, with one lambert material and a sun straight down +z.
struct Fixture {
    rd::RenderScene scene;
    Fixture() {
        rd::MeshData m;
        m.positions = {{-5.0f, -5.0f, 0.0f}, {5.0f, -5.0f, 0.0f}, {5.0f, 5.0f, 0.0f}, {-5.0f, 5.0f, 0.0f}};
        m.normals = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
        m.indices = {0, 1, 2, 0, 2, 3};
        m.submesh_first_index = {0};
        m.submesh_index_count = {6};
        m.submesh_material = {0};
        scene.meshes.push_back(m);
        rd::DrawItem d;
        d.mesh_index = 0;
        scene.statics.push_back(d);
        scene.materials.resize(1);
        scene.materials[0].base_color = glm::vec4(0.5f, 0.25f, 0.125f, 1.0f);
        scene.lighting.sun_direction = glm::vec3(0.0f, 0.0f, 1.0f);
        scene.lighting.sun_color = glm::vec3(1.0f);
        scene.lighting.sun_intensity = 0.6f;
        scene.lighting.ambient_color = glm::vec3(0.1f);
    }
    [[nodiscard]] rd::RayScene rays() const {
        spade::Result<rd::RayScene> r = rd::RayScene::build(scene);
        EXPECT_TRUE(r.has_value());
        return r ? std::move(*r) : rd::RayScene{};
    }
};

[[nodiscard]] rd::Ray ray(const glm::dvec3& o, const glm::dvec3& d) {
    rd::Ray r;
    r.origin = o;
    r.direction = glm::normalize(d);
    return r;
}

// the exact sRGB byte of a linear value, by libm
[[nodiscard]] double srgb_exact(double linear) {
    const double c = std::clamp(linear, 0.0, 1.0);
    const double e = c <= 0.0031308 ? 12.92 * c : 1.055 * std::pow(c, 1.0 / 2.4) - 0.055;
    return 255.0 * e;
}

}  // namespace

// A lit hit is the shared Lambert + ambient; the back of a face is lit by the normal facing the ray; unlit and emissive echo the base colour; a miss is the sky
// gradient by the ray's elevation.
TEST(CameraShade, AHitIsTheScenesOwnLambertAndAMissIsItsSky) {
    Fixture f;
    const rd::RayScene rays = f.rays();

    const glm::vec3 lit = rd::shade_ray(f.scene, rays, ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0}));
    EXPECT_NEAR(lit.x, 0.5f * (0.6f + 0.1f), 1e-6f);
    EXPECT_NEAR(lit.y, 0.25f * (0.6f + 0.1f), 1e-6f);
    EXPECT_NEAR(lit.z, 0.125f * (0.6f + 0.1f), 1e-6f);

    const glm::vec3 back = rd::shade_ray(f.scene, rays, ray({0.0, 0.0, -5.0}, {0.0, 0.0, 1.0}));
    EXPECT_NEAR(back.x, 0.5f * 0.1f, 1e-6f) << "the back face: the normal faces the ray, away from the sun, so ambient only";
    EXPECT_NEAR(back.y, 0.25f * 0.1f, 1e-6f);

    f.scene.materials[0].shading = 1;  // unlit
    EXPECT_EQ(rd::shade_ray(f.scene, f.rays(), ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0})), glm::vec3(0.5f, 0.25f, 0.125f));
    f.scene.materials[0].shading = 2;  // emissive
    EXPECT_EQ(rd::shade_ray(f.scene, f.rays(), ray({0.0, 0.0, 5.0}, {0.0, 0.0, -1.0})), glm::vec3(0.5f, 0.25f, 0.125f));
    f.scene.materials[0].shading = 0;

    const rd::RayScene again = f.rays();
    EXPECT_EQ(rd::shade_ray(f.scene, again, ray({0.0, 0.0, 5.0}, {0.0, 1.0, 0.0})), f.scene.lighting.sky_zenith) << "straight up: the zenith";
    EXPECT_EQ(rd::shade_ray(f.scene, again, ray({0.0, 0.0, 5.0}, {1.0, 0.0, 0.0})), f.scene.lighting.sky_horizon) << "on the horizon";
    const glm::vec3 half = rd::shade_ray(f.scene, again, ray({0.0, 0.0, 5.0}, {1.0, 1.0, 0.0}));
    EXPECT_EQ(half, rd::sky_gradient_color(f.scene.lighting, glm::vec3(1.0f, 1.0f, 0.0f))) << "in between: the scene's own gradient";
}

// SR-17a: at strength 0 the surface is exactly as without it; above 0 it blends toward the sky by the rational term, by the distance along the ray.
TEST(CameraShade, TheAtmosphereIsExactlyOffAtZeroAndBlendsTowardTheSkyAtRange) {
    Fixture f;
    const rd::RayScene rays = f.rays();
    const rd::Ray r = ray({0.0, 0.0, 10.0}, {0.0, 0.0, -1.0});  // the quad at t = 10
    const glm::vec3 plain = rd::shade_ray(f.scene, rays, r);
    rd::CameraShading off;
    off.atmosphere_strength = 0.0f;
    off.atmosphere_onset = 10.0f;
    EXPECT_EQ(rd::shade_ray(f.scene, rays, r, off), plain) << "strength 0: bit-exact";

    rd::CameraShading on;
    on.atmosphere_strength = 1.0f;
    on.atmosphere_onset = 10.0f;  // r = 1: t = 1/2, so half way to the sky
    const glm::vec3 sky = rd::sky_gradient_color(f.scene.lighting, glm::vec3(0.0f, 0.0f, -1.0f));
    const glm::vec3 want = plain + (sky - plain) * 0.5f;
    const glm::vec3 got = rd::shade_ray(f.scene, rays, r, on);
    EXPECT_NEAR(got.x, want.x, 1e-6f);
    EXPECT_NEAR(got.y, want.y, 1e-6f);
    EXPECT_NEAR(got.z, want.z, 1e-6f);
}

// The byte is the exact round-to-nearest of the sRGB curve, with no libm in the engine: 20,000 linear values against libm, none off by a level except exactly at a
// rounding tie; the ends and the clamp; the exposure gain acts on the linear colour before the clamp.
TEST(CameraShade, TheByteIsTheExactSrgbRoundToNearest) {
    int wrong = 0;
    for (int i = 0; i <= 20000; ++i) {
        const double x = static_cast<double>(i) / 20000.0;
        const double want = srgb_exact(static_cast<double>(static_cast<float>(x)));
        const double frac = want - std::floor(want);
        if (std::abs(frac - 0.5) < 1e-6) continue;  // a tie: either neighbour is the nearest
        if (static_cast<double>(rd::srgb_byte(static_cast<float>(x))) != std::floor(want + 0.5)) ++wrong;
    }
    EXPECT_EQ(wrong, 0);
    EXPECT_EQ(rd::srgb_byte(0.0f), 0u);
    EXPECT_EQ(rd::srgb_byte(1.0f), 255u);
    EXPECT_EQ(rd::srgb_byte(-3.0f), 0u) << "clamped";
    EXPECT_EQ(rd::srgb_byte(7.0f), 255u) << "clamped";
    EXPECT_EQ(rd::srgb_byte(0.2140f), 128u) << "linear 0.2140 is sRGB 0.5";

    uint8_t px[3] = {9, 9, 9};
    rd::CameraShading s;
    EXPECT_EQ(rd::encode_pixel(glm::vec3(0.1f, 0.2f, 0.3f), s, px), 3u);
    EXPECT_EQ(px[0], rd::srgb_byte(0.1f));
    EXPECT_EQ(px[1], rd::srgb_byte(0.2f));
    EXPECT_EQ(px[2], rd::srgb_byte(0.3f));
    s.exposure = 2.0f;
    EXPECT_EQ(rd::encode_pixel(glm::vec3(0.1f, 0.2f, 0.3f), s, px), 3u);
    EXPECT_EQ(px[0], rd::srgb_byte(0.2f)) << "the gain is linear";
    EXPECT_EQ(px[1], rd::srgb_byte(0.4f));
    EXPECT_EQ(px[2], rd::srgb_byte(0.6f));
    s.exposure = 100.0f;
    EXPECT_EQ(rd::encode_pixel(glm::vec3(0.1f, 0.2f, 0.3f), s, px), 3u);
    EXPECT_EQ(px[0], 255u) << "clamped after the gain";
}

// Grey is one channel: the luminance of the exposed linear colour, then the same curve.
TEST(CameraShade, GreyIsTheLuminanceOfTheExposedLinearColourThenTheSameCurve) {
    rd::CameraShading s;
    s.grey = true;
    uint8_t px[3] = {9, 9, 9};
    EXPECT_EQ(rd::encode_pixel(glm::vec3(0.5f, 0.25f, 0.125f), s, px), 1u);
    const float y = 0.2126f * 0.5f + 0.7152f * 0.25f + 0.0722f * 0.125f;
    EXPECT_EQ(px[0], rd::srgb_byte(y));
    EXPECT_EQ(px[1], 9u) << "only one byte is written";
    s.exposure = 0.5f;
    EXPECT_EQ(rd::encode_pixel(glm::vec3(0.5f, 0.25f, 0.125f), s, px), 1u);
    EXPECT_EQ(px[0], rd::srgb_byte(0.5f * y));
    EXPECT_EQ(rd::channel_count(s), 1u);
    s.grey = false;
    EXPECT_EQ(rd::channel_count(s), 3u);
}
