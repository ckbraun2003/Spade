#include "physics/airflow/advect.hpp"

#include <algorithm>
#include <cmath>

namespace spade::physics::airflow {

namespace {

struct Axis {
    uint32_t i0;
    uint32_t i1;
    float t;
};

// A fractional sample coordinate f along an axis of n cells holding `count`
// samples (n + 1 on the component's own axis, n on the others).
[[nodiscard]] Axis locate(float f, uint32_t n, uint32_t count, bool periodic) noexcept {
    if (periodic) {
        if (n == 1u) return {0u, 0u, 0.0f};
        const float nf = static_cast<float>(n);
        float w = f - nf * std::floor(f / nf);
        if (!(w >= 0.0f) || !(w < nf)) w = 0.0f;  // NaN, or rounding onto the period's end
        const auto i0 = static_cast<uint32_t>(w);
        return {i0, i0 + 1u == n ? 0u : i0 + 1u, w - static_cast<float>(i0)};
    }
    if (count == 1u) return {0u, 0u, 0.0f};
    const float top = static_cast<float>(count - 1u);
    const float c = f > top ? top : (f >= 0.0f ? f : 0.0f);  // NaN clamps to 0
    uint32_t i0 = static_cast<uint32_t>(c);
    if (i0 + 1u >= count) i0 = count - 2u;
    return {i0, i0 + 1u, c - static_cast<float>(i0)};
}

[[nodiscard]] inline float lerp(float a, float b, float t) noexcept { return a + t * (b - a); }

}  // namespace

Sample sample_component(const GridShape& s, std::span<const float> field, uint32_t c, glm::vec3 x,
                        const DomainBc& bc) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const float inv_dx = 1.0f / s.dx;
    Axis ax[3];
    for (uint32_t a = 0; a < 3; ++a) {
        const bool own = a == c;
        const float f = x[static_cast<glm::length_t>(a)] * inv_dx - (own ? 0.0f : 0.5f);
        ax[a] = locate(f, n[a], own ? n[a] + 1u : n[a], bc.face[2u * a] == FaceBc::periodic);
    }
    const std::size_t sx = c == 0u ? s.nx + 1u : s.nx;
    const std::size_t sy = c == 1u ? s.ny + 1u : s.ny;
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) { return field[i + sx * (j + sy * k)]; };
    const float a000 = at(ax[0].i0, ax[1].i0, ax[2].i0);
    const float a100 = at(ax[0].i1, ax[1].i0, ax[2].i0);
    const float a010 = at(ax[0].i0, ax[1].i1, ax[2].i0);
    const float a110 = at(ax[0].i1, ax[1].i1, ax[2].i0);
    const float a001 = at(ax[0].i0, ax[1].i0, ax[2].i1);
    const float a101 = at(ax[0].i1, ax[1].i0, ax[2].i1);
    const float a011 = at(ax[0].i0, ax[1].i1, ax[2].i1);
    const float a111 = at(ax[0].i1, ax[1].i1, ax[2].i1);
    const float c0 = lerp(lerp(a000, a100, ax[0].t), lerp(a010, a110, ax[0].t), ax[1].t);
    const float c1 = lerp(lerp(a001, a101, ax[0].t), lerp(a011, a111, ax[0].t), ax[1].t);
    Sample out{lerp(c0, c1, ax[2].t), a000, a000};
    for (const float v : {a100, a010, a110, a001, a101, a011, a111}) {
        out.lo = std::min(out.lo, v);
        out.hi = std::max(out.hi, v);
    }
    return out;
}

glm::vec3 velocity_at(const MacGrid& g, glm::vec3 x, const DomainBc& bc) noexcept {
    return glm::vec3(sample_component(g.shape, g.u, 0, x, bc).value, sample_component(g.shape, g.v, 1, x, bc).value,
                     sample_component(g.shape, g.w, 2, x, bc).value);
}

AdvectScratch make_advect_scratch(const GridShape& shape) {
    AdvectScratch s;
    const std::size_t sizes[3] = {std::size_t{shape.nx + 1u} * shape.ny * shape.nz,
                                  std::size_t{shape.nx} * (shape.ny + 1u) * shape.nz,
                                  std::size_t{shape.nx} * shape.ny * (shape.nz + 1u)};
    for (uint32_t c = 0; c < 3; ++c) {
        s.hat[c].assign(sizes[c], 0.0f);
        s.lo[c].assign(sizes[c], 0.0f);
        s.hi[c].assign(sizes[c], 0.0f);
    }
    return s;
}

void advect_velocity(const MacGrid& in, MacGrid& out, const DomainBc& bc, float h, AdvectScratch& s) noexcept {
    const float half_h = 0.5f * h;
    const auto back_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x - half_h * velocity_at(in, x, bc);
        return x - h * velocity_at(in, mid, bc);
    };
    const auto forward_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x + half_h * velocity_at(in, x, bc);
        return x + h * velocity_at(in, mid, bc);
    };
    const std::vector<float>* src[3] = {&in.u, &in.v, &in.w};
    std::vector<float>* dst[3] = {&out.u, &out.v, &out.w};
    // 1. The forward semi-Lagrangian estimate and its stencil's range.
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const Sample f = sample_component(in.shape, *src[c], c, back_trace(x), bc);
            s.hat[c][idx] = f.value;
            s.lo[c][idx] = f.lo;
            s.hi[c][idx] = f.hi;
        });
    }
    // 2. The backward estimate from the forward one, and the clamped correction.
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const float back = sample_component(in.shape, s.hat[c], c, forward_trace(x), bc).value;
            float v = s.hat[c][idx] + 0.5f * ((*src[c])[idx] - back);
            v = v < s.lo[c][idx] ? s.lo[c][idx] : (v > s.hi[c][idx] ? s.hi[c][idx] : v);
            (*dst[c])[idx] = v;
        });
    }
}

}  // namespace spade::physics::airflow
