#pragma once

// The C_p view's sphere (RND-8, R2.4): potential flow past a sphere,
// published as Physics' P6 rule writes a solid, the sphere's mesh, and a
// frame's C_p error against the analytic surface value. Shared by the CPU
// and GL tests, so both judge a frame by one rule.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/transfer_function.hpp"

namespace spade::test {

// Potential flow past a sphere of radius a at the origin, the stream U = 1
// along +x, rho = 1: C_p = 1 - |u|^2 / U^2 off the sphere, 1 - 9/4 sin^2(theta)
// on it. As pressure, p = C_p / 2: q_ref = 1/2 and p_ref = 0.
inline constexpr double kCpSphereRadius = 0.5;

[[nodiscard]] inline double potential_sphere_cp(double x, double y, double z) {
    const double r2 = x * x + y * y + z * z;
    const double r = std::sqrt(r2);
    const double c = x / r;  // cos(theta) from the stream axis
    const double a3 = kCpSphereRadius * kCpSphereRadius * kCpSphereRadius / (r2 * r);
    const double ur = c * (1.0 - a3);
    const double ut2 = (1.0 - c * c) * (1.0 + 0.5 * a3) * (1.0 + 0.5 * a3);
    return 1.0 - (ur * ur + ut2);
}

// The box [-1.5, 1.5]^3 m at `cells_per_radius` cells per a. Fluid cells hold
// the field; a solid cell with a fluid face-neighbour holds their mean, and
// deeper solid cells are no-data.
[[nodiscard]] inline FlowVolume potential_sphere_volume(uint32_t cells_per_radius) {
    const double a = kCpSphereRadius;
    const float cell = static_cast<float>(a) / static_cast<float>(cells_per_radius);
    const uint32_t n = 6u * cells_per_radius;
    const glm::vec3 origin(-1.5f);
    const auto centre = [&](uint32_t i) { return origin.x + (static_cast<float>(i) + 0.5f) * cell; };
    const auto at = [n](uint32_t i, uint32_t j, uint32_t k) { return (static_cast<std::size_t>(k) * n + j) * n + i; };
    std::vector<float> p(static_cast<std::size_t>(n) * n * n);
    std::vector<uint8_t> solid(p.size());
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) {
                const double x = centre(i), y = centre(j), z = centre(k);
                solid[at(i, j, k)] = x * x + y * y + z * z < a * a ? 1u : 0u;
                p[at(i, j, k)] = solid[at(i, j, k)] ? 0.0f : static_cast<float>(0.5 * potential_sphere_cp(x, y, z));
            }
        }
    }
    std::vector<float> published = p;
    for (uint32_t k = 1; k + 1 < n; ++k) {
        for (uint32_t j = 1; j + 1 < n; ++j) {
            for (uint32_t i = 1; i + 1 < n; ++i) {
                if (!solid[at(i, j, k)]) continue;
                double sum = 0.0;
                int fluid = 0;
                const std::size_t nb[6] = {at(i - 1, j, k), at(i + 1, j, k), at(i, j - 1, k),
                                           at(i, j + 1, k), at(i, j, k - 1), at(i, j, k + 1)};
                for (const std::size_t m : nb) {
                    if (!solid[m]) {
                        sum += p[m];
                        ++fluid;
                    }
                }
                published[at(i, j, k)] =
                    fluid > 0 ? static_cast<float>(sum / fluid) : std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return make_flow_volume(origin, cell, n, n, n, [&](glm::vec3 q) {
        FlowChannels c{};
        const auto index = [&](float v) {
            return static_cast<uint32_t>(std::clamp((v - origin.x) / cell, 0.0f, static_cast<float>(n - 1)));
        };
        c[kFlowPressure] = published[at(index(q.x), index(q.y), index(q.z))];
        return c;
    });
}

// A UV sphere of radius a with outward normals, wound counter-clockwise seen
// from outside. 96 x 192 puts its facets within 0.1% of a of the sphere.
[[nodiscard]] inline render::MeshData potential_sphere_mesh(uint32_t rings = 96, uint32_t segments = 192) {
    constexpr double kPi = 3.14159265358979323846;
    render::MeshData mesh;
    for (uint32_t r = 0; r <= rings; ++r) {
        const double v = kPi * static_cast<double>(r) / rings;
        for (uint32_t sgm = 0; sgm <= segments; ++sgm) {
            const double u = 2.0 * kPi * static_cast<double>(sgm) / segments;
            const glm::vec3 d(static_cast<float>(std::sin(v) * std::cos(u)), static_cast<float>(std::cos(v)),
                              static_cast<float>(std::sin(v) * std::sin(u)));
            mesh.positions.push_back(static_cast<float>(kCpSphereRadius) * d);
            mesh.normals.push_back(d);
        }
    }
    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t sgm = 0; sgm < segments; ++sgm) {
            const uint32_t a = r * (segments + 1) + sgm, b = a + segments + 1;
            mesh.indices.insert(mesh.indices.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
    }
    return mesh;
}

// A frame's C_p against 1 - 9/4 sin^2(theta) on the true sphere. Each pixel
// whose ray meets the sphere at least 0.3 face-on is measured: its colour
// turned back into C_p by the cool-warm map's nearest entry over [cp_min,
// cp_max]. A colour no entry is within 3 levels of is a pixel left shaded
// (no-data), counted and not measured.
struct CpError {
    double worst = 0.0;
    double worst_sin2 = 0.0;  // sin^2(theta) where the worst is
    double mean = 0.0;
    std::size_t count = 0;
    std::size_t unmapped = 0;
};

// `rgb_at(x, y)` gives a pixel's 0-255 RGB, y counted from the top row.
template <class RgbAt>
[[nodiscard]] CpError potential_sphere_cp_error(const render::Camera& camera, uint32_t width, uint32_t height,
                                                float cp_min, float cp_max, RgbAt&& rgb_at) {
    const double a = kCpSphereRadius;
    std::array<glm::vec3, 256> map{};
    for (int e = 0; e < 256; ++e) {
        map[e] = 255.0f * render::colour_map_at(render::ColourMapId::cool_warm, static_cast<float>(e) / 255.0f);
    }
    const render::RayGrid grid = render::ray_grid(camera, width, height);
    const glm::dvec3 eye(camera.position);
    CpError out;
    double sum = 0.0;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const glm::dvec3 d(render::camera_ray(grid, x, y).dir_world);
            const double b = glm::dot(eye, d);
            const double disc = b * b - (glm::dot(eye, eye) - a * a);
            if (disc <= 0.0) continue;
            const glm::dvec3 hit = eye + d * (-b - std::sqrt(disc));
            if (glm::dot(glm::normalize(hit), -d) < 0.3) continue;
            const glm::vec3 rgb = rgb_at(x, y);
            int best = 0;
            float best_d = std::numeric_limits<float>::max();
            for (int e = 0; e < 256; ++e) {
                const glm::vec3 diff = rgb - map[e];
                const float d2 = glm::dot(diff, diff);
                if (d2 < best_d) {
                    best_d = d2;
                    best = e;
                }
            }
            if (best_d > 27.0f) {
                ++out.unmapped;
                continue;
            }
            const double got = cp_min + (cp_max - cp_min) * static_cast<double>(best) / 255.0;
            const double sin2 = 1.0 - (hit.x / a) * (hit.x / a);
            const double err = std::abs(got - (1.0 - 2.25 * sin2));
            if (err > out.worst) {
                out.worst = err;
                out.worst_sin2 = sin2;
            }
            sum += err;
            ++out.count;
        }
    }
    out.mean = out.count > 0 ? sum / static_cast<double>(out.count) : 0.0;
    return out;
}

}  // namespace spade::test
