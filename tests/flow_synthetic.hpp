#pragma once

// Test-side synthetic flow volumes: analytic fields written through the
// publish format (core/flow_volume.hpp, core/fp16.hpp), so a view's test
// pins rendering alone and never moves with the solver (airflow spec
// section 5). Each cell holds its field's value at the cell's centre.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <glm/vec3.hpp>

#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "core/fp32_math.hpp"

namespace spade::test {

using FlowChannels = std::array<float, kFlowChannelCount>;

// `field(p)` gives every channel at world point p. Ranges are taken over the
// decoded halves, as the volume holds them; no-data samples are counted and
// left out of the ranges.
template <class Field>
[[nodiscard]] FlowVolume make_flow_volume(glm::vec3 origin, float cell, uint32_t nx, uint32_t ny, uint32_t nz,
                                          Field&& field) {
    FlowVolume volume;
    FlowVolumeHeader& h = volume.header;
    h.origin = origin;
    h.cell = cell;
    h.nx = nx;
    h.ny = ny;
    h.nz = nz;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    volume.v.resize(cells * kFlowHalvesPerCell);
    volume.s.resize(cells * kFlowHalvesPerCell);
    std::array<bool, kFlowChannelCount> seen{};
    for (uint32_t k = 0; k < nz; ++k) {
        for (uint32_t j = 0; j < ny; ++j) {
            for (uint32_t i = 0; i < nx; ++i) {
                const glm::vec3 p = origin + (glm::vec3(static_cast<float>(i), static_cast<float>(j),
                                                        static_cast<float>(k)) +
                                              glm::vec3(0.5f)) *
                                                 cell;
                const FlowChannels c = field(p);
                const std::size_t base = flow_cell_index(h, i, j, k) * kFlowHalvesPerCell;
                for (uint32_t ch = 0; ch < kFlowChannelCount; ++ch) {
                    const fp16::Half half = fp16::from_f32(c[ch]);
                    (ch < kFlowHalvesPerCell ? volume.v : volume.s)[base + ch % kFlowHalvesPerCell] = half.bits;
                    if (half.no_data) {
                        ++h.no_data;
                        continue;
                    }
                    const float x = fp16::to_f32(half.bits);
                    h.channel_min[ch] = seen[ch] ? std::min(h.channel_min[ch], x) : x;
                    h.channel_max[ch] = seen[ch] ? std::max(h.channel_max[ch], x) : x;
                    seen[ch] = true;
                }
            }
        }
    }
    return volume;
}

// Uniform flow (u, v, w), everything else zero.
[[nodiscard]] inline auto uniform_flow(glm::vec3 velocity) {
    return [velocity](glm::vec3) {
        FlowChannels c{};
        c[kFlowU] = velocity.x;
        c[kFlowV] = velocity.y;
        c[kFlowW] = velocity.z;
        return c;
    };
}

// A vortex ring with a dye blob, for frame goldens. GOLDEN-FEEDING: only
// sqrt and math::exp32 (TD-3), and the tilt is fixed constants, not trig.
// The ring: radius R about its axis, a Lamb-Oseen core of radius a and
// circulation G, so at distance d from the core circle
//   |w| = G / (pi a^2) exp(-d^2 / a^2)
//   |u| = G / (2 pi d) (1 - exp(-d^2 / a^2)), swirling about the core.
// The axis is z tilted 30 degrees about x, so the camera sees the ring as
// a ring with depth. The dye is a Gaussian blob of radius b on the core.
[[nodiscard]] inline auto vortex_ring_with_dye() {
    return [](glm::vec3 p) {
        constexpr float kR = 0.55f, kA = 0.12f, kG = 1.0f, kB = 0.2f;
        constexpr float kPi = 3.14159265f;
        constexpr float kC = 0.8660254f, kS = 0.5f;  // cos 30, sin 30
        // World to ring frame (rotation about x by -30 degrees), and back.
        const glm::vec3 q(p.x, kC * p.y + kS * p.z, -kS * p.y + kC * p.z);
        const float rho = std::sqrt(q.x * q.x + q.y * q.y);
        FlowChannels c{};
        if (rho > 1.0e-6f) {
            const glm::vec3 core(kR * q.x / rho, kR * q.y / rho, 0.0f);
            const glm::vec3 r = q - core;
            const float d2 = r.x * r.x + r.y * r.y + r.z * r.z;
            const float d = std::sqrt(d2);
            const float g = spade::math::exp32(-d2 / (kA * kA));
            c[kFlowVorticity] = kG / (kPi * kA * kA) * g;
            if (d > 1.0e-6f) {
                const glm::vec3 t(-q.y / rho, q.x / rho, 0.0f);  // the ring's tangent
                const glm::vec3 swirl(t.y * r.z - t.z * r.y, t.z * r.x - t.x * r.z, t.x * r.y - t.y * r.x);
                const float speed = kG / (2.0f * kPi * d) * (1.0f - g);
                const glm::vec3 u = swirl * (speed / d);  // |t x r| = d, since t is normal to r
                c[kFlowU] = u.x;
                c[kFlowV] = kC * u.y - kS * u.z;
                c[kFlowW] = kS * u.y + kC * u.z;
            }
            const glm::vec3 blob = q - glm::vec3(kR, 0.0f, 0.0f);
            c[kFlowDye] = spade::math::exp32(-(blob.x * blob.x + blob.y * blob.y + blob.z * blob.z) / (kB * kB));
        }
        return c;
    };
}

}  // namespace spade::test
