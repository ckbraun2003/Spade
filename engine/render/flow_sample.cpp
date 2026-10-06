#include "render/flow_sample.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "core/fp16.hpp"

namespace spade::render {
namespace {

// One axis of the trilinear stencil: the two cells and the far cell's weight.
struct Axis {
    uint32_t i0 = 0;
    uint32_t i1 = 0;
    float f = 0.0f;
};

[[nodiscard]] Axis axis_of(float coordinate, float origin, float inv_cell, uint32_t n) {
    const float top = static_cast<float>(n - 1u);
    const float g = std::clamp((coordinate - origin) * inv_cell - 0.5f, 0.0f, top);  // a NaN stays NaN
    if (!(g >= 0.0f)) return Axis{};                                                 // NaN: the first cell
    const float base = std::floor(g);
    Axis a;
    a.i0 = static_cast<uint32_t>(base);
    a.i1 = std::min(a.i0 + 1u, n - 1u);
    a.f = g - base;
    return a;
}

// The stencil: one Axis per axis.
struct Stencil {
    Axis x, y, z;
};

[[nodiscard]] Stencil stencil_at(const FlowSampler& s, glm::vec3 p) {
    const FlowVolumeHeader& h = s.volume->header;
    return Stencil{axis_of(p.x, s.origin.x, s.inv_cell, h.nx), axis_of(p.y, s.origin.y, s.inv_cell, h.ny),
                   axis_of(p.z, s.origin.z, s.inv_cell, h.nz)};
}

// a + (b - a) f: exact at f = 0 and on a constant, so a uniform field
// samples to itself.
[[nodiscard]] float lerp(float a, float b, float f) { return a + (b - a) * f; }

// Nested lerps along x, then y, then z. A corner is read only when its axis
// fraction is above 0 (its weight is then above 0, since every fraction is
// below 1); an unread corner stays 0, and a lerp at f = 0 returns its first
// argument exactly.
[[nodiscard]] FlowValue interpolate(const FlowSampler& s, const Stencil& st, FlowChannel channel) {
    const FlowVolumeHeader& h = s.volume->header;
    float c[2][2][2] = {};
    for (uint32_t dz = 0; dz <= (st.z.f > 0.0f ? 1u : 0u); ++dz) {
        for (uint32_t dy = 0; dy <= (st.y.f > 0.0f ? 1u : 0u); ++dy) {
            for (uint32_t dx = 0; dx <= (st.x.f > 0.0f ? 1u : 0u); ++dx) {
                const std::size_t cell = flow_cell_index(h, dx ? st.x.i1 : st.x.i0, dy ? st.y.i1 : st.y.i0,
                                                         dz ? st.z.i1 : st.z.i0);
                const float x = fp16::to_f32(flow_half(*s.volume, cell, channel));
                if (!std::isfinite(x)) return FlowValue{0.0f, true};
                c[dz][dy][dx] = x;
            }
        }
    }
    const float y0 = lerp(lerp(c[0][0][0], c[0][0][1], st.x.f), lerp(c[0][1][0], c[0][1][1], st.x.f), st.y.f);
    const float y1 = lerp(lerp(c[1][0][0], c[1][0][1], st.x.f), lerp(c[1][1][0], c[1][1][1], st.x.f), st.y.f);
    return FlowValue{lerp(y0, y1, st.z.f), false};
}

}  // namespace

FlowSampler make_flow_sampler(const FlowVolume& volume) {
    return FlowSampler{&volume, volume.header.origin, 1.0f / volume.header.cell};
}

FlowValue sample_channel(const FlowSampler& sampler, FlowChannel channel, glm::vec3 p) {
    return interpolate(sampler, stencil_at(sampler, p), channel);
}

FlowVelocity sample_velocity(const FlowSampler& sampler, glm::vec3 p) {
    const Stencil st = stencil_at(sampler, p);
    const FlowValue u = interpolate(sampler, st, kFlowU);
    const FlowValue v = interpolate(sampler, st, kFlowV);
    const FlowValue w = interpolate(sampler, st, kFlowW);
    if (u.no_data || v.no_data || w.no_data) return FlowVelocity{glm::vec3(0.0f), true};
    return FlowVelocity{glm::vec3(u.value, v.value, w.value), false};
}

FlowValue sample_quantity(const FlowSampler& sampler, FlowQuantity quantity, glm::vec3 p) {
    switch (quantity) {
        case FlowQuantity::speed: {
            const FlowVelocity vel = sample_velocity(sampler, p);
            if (vel.no_data) return FlowValue{0.0f, true};
            return FlowValue{std::sqrt(vel.u.x * vel.u.x + vel.u.y * vel.u.y + vel.u.z * vel.u.z), false};
        }
        case FlowQuantity::u:
            return sample_channel(sampler, kFlowU, p);
        case FlowQuantity::v:
            return sample_channel(sampler, kFlowV, p);
        case FlowQuantity::w:
            return sample_channel(sampler, kFlowW, p);
        case FlowQuantity::dye:
            return sample_channel(sampler, kFlowDye, p);
        case FlowQuantity::pressure:
            return sample_channel(sampler, kFlowPressure, p);
        case FlowQuantity::vorticity:
            return sample_channel(sampler, kFlowVorticity, p);
        case FlowQuantity::q_hat:
            return sample_channel(sampler, kFlowQHat, p);
    }
    return FlowValue{0.0f, true};
}

}  // namespace spade::render
