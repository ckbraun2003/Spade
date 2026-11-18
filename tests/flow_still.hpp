#pragma once

// flow_still.hpp -- a still of a FlowVolume's dye on black, for the opt-in harnesses that draw the dye (test_gpu_airflow_line_stills.cpp):
// the PPM writer, views.cpp's orbit camera, the dye and smoke transfer functions, and one draw. test_airflow_dye.cpp keeps its own
// copies of the pieces it had before this header (it draws a MacGrid it publishes itself).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_look.hpp"
#include "render/flow_render.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"

namespace flow_still {

namespace rd = spade::render;

[[nodiscard]] inline const char* env_cstr(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] inline float env_float(const char* name, float fallback) {
    const char* v = env_cstr(name);
    return v != nullptr ? std::strtof(v, nullptr) : fallback;
}

inline void write_ppm(const std::string& path, const std::vector<uint8_t>& bgrx, uint32_t w, uint32_t h) {
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
[[nodiscard]] inline rd::Camera orbit(glm::vec3 target, float distance, float azimuth_deg, float elevation_deg) {
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

// The engine's dye transfer function (inferno), and a smoke one: a constant light grey, extinction rising with the dye.
// SPADE_DYE_SIGMA is the extinction at dye 1 in 1/m (default 25), SPADE_DYE_ONSET where it starts (default 0.03).
[[nodiscard]] inline std::vector<Look> dye_looks() {
    const float sigma = env_float("SPADE_DYE_SIGMA", 25.0f);
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

// One still of `volume`'s dye through `look` at `camera`, black sky, as a PPM; false and why on a refusal.
[[nodiscard]] inline spade::Result<void> draw_volume(const spade::FlowVolume& volume, const rd::Camera& camera,
                                                     const Look& look, const std::string& path, uint32_t w = 1280,
                                                     uint32_t h = 720) {
    rd::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    const spade::FlowVolumeHeader& hd = volume.header;
    scene.bounds = rd::Aabb{.min = hd.origin,
                            .max = hd.origin + glm::vec3(static_cast<float>(hd.nx), static_cast<float>(hd.ny),
                                                         static_cast<float>(hd.nz)) * hd.cell};
    scene.flow_layers.push_back(rd::FlowLayer{&volume, {look.tf}, {}});
    std::vector<uint8_t> px(static_cast<std::size_t>(w) * h * 4u, 0u);
    rd::RenderTarget target{.pixels = std::span<uint8_t>(px), .width = w, .height = h, .stride = w * 4u};
    rd::RenderOptions options;
    options.overlays = false;
    const auto r = rd::render(scene, camera, options, target);
    if (!r) return std::unexpected(r.error());
    write_ppm(path, px, w, h);
    return {};
}

}  // namespace flow_still
