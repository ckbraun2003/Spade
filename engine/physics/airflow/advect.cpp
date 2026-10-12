#include "physics/airflow/advect.hpp"

#include <algorithm>
#include <cmath>

namespace spade::physics::airflow {

namespace {

// The sampler is the solver's hot loop: MacCormack's two RK2 traces take 14
// trilinear samples per face per step. Everything below is forced inline (an
// out-of-line locate() with its stack checks cost about half the time;
// measured, the airflow CPU-performance task, 2026-10-12). Every value is the
// expression the earlier sampler computed, in the same order, so the samples'
// bits are unchanged.
#if defined(_MSC_VER)
#define SPADE_AIRFLOW_HOT __forceinline
#else
#define SPADE_AIRFLOW_HOT inline __attribute__((always_inline))
#endif

struct Axis {
    uint32_t i0;
    uint32_t i1;
    float t;
};

// A fractional sample coordinate f along an axis of n cells holding `count`
// samples (n + 1 on the component's own axis, n on the others).
SPADE_AIRFLOW_HOT Axis locate(float f, uint32_t n, uint32_t count, bool periodic) noexcept {
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

SPADE_AIRFLOW_HOT float lerp(float a, float b, float t) noexcept { return a + t * (b - a); }

// The grid's sampling geometry, shared by the three components. A point's
// coordinate along an axis is located twice at most: against the faces normal
// to that axis (the axis' own component: offset 0, n + 1 samples) and against
// the cell centres (the other two: offset 0.5, n samples).
struct Geometry {
    uint32_t n[3];
    bool periodic[3];
    std::size_t sx[3];  // per component, the samples along x and y
    std::size_t sy[3];
    float inv_dx;
};

[[nodiscard]] Geometry geometry(const GridShape& s, const DomainBc& bc) noexcept {
    Geometry g{};
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    for (uint32_t a = 0; a < 3; ++a) {
        g.n[a] = n[a];
        g.periodic[a] = bc.face[2u * a] == FaceBc::periodic;
        g.sx[a] = a == 0u ? s.nx + 1u : s.nx;
        g.sy[a] = a == 1u ? s.ny + 1u : s.ny;
    }
    g.inv_dx = 1.0f / s.dx;
    return g;
}

// The coordinate along axis a, located for component c.
SPADE_AIRFLOW_HOT Axis locate_for(const Geometry& g, glm::vec3 x, uint32_t a, uint32_t c) noexcept {
    const bool own = a == c;
    const float f = x[static_cast<glm::length_t>(a)] * g.inv_dx - (own ? 0.0f : 0.5f);
    return locate(f, g.n[a], own ? g.n[a] + 1u : g.n[a], g.periodic[a]);
}

// The trilinear sample of component c's field on located axes; with kRange,
// also its stencil's range (the MacCormack clamp's bounds), which only the
// first pass's estimate needs.
template <bool kRange>
SPADE_AIRFLOW_HOT Sample sample_on(const Geometry& g, uint32_t c, const float* field, const Axis& x0, const Axis& x1,
                                   const Axis& x2) noexcept {
    const std::size_t sx = g.sx[c];
    const std::size_t sy = g.sy[c];
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) { return field[i + sx * (j + sy * k)]; };
    const float a000 = at(x0.i0, x1.i0, x2.i0);
    const float a100 = at(x0.i1, x1.i0, x2.i0);
    const float a010 = at(x0.i0, x1.i1, x2.i0);
    const float a110 = at(x0.i1, x1.i1, x2.i0);
    const float a001 = at(x0.i0, x1.i0, x2.i1);
    const float a101 = at(x0.i1, x1.i0, x2.i1);
    const float a011 = at(x0.i0, x1.i1, x2.i1);
    const float a111 = at(x0.i1, x1.i1, x2.i1);
    const float c0 = lerp(lerp(a000, a100, x0.t), lerp(a010, a110, x0.t), x1.t);
    const float c1 = lerp(lerp(a001, a101, x0.t), lerp(a011, a111, x0.t), x1.t);
    Sample out{lerp(c0, c1, x2.t), a000, a000};
    if constexpr (kRange) {
        for (const float v : {a100, a010, a110, a001, a101, a011, a111}) {
            out.lo = std::min(out.lo, v);
            out.hi = std::max(out.hi, v);
        }
    }
    return out;
}

// One component's sample at x.
template <bool kRange>
SPADE_AIRFLOW_HOT Sample sample(const Geometry& g, uint32_t c, const float* field, glm::vec3 x) noexcept {
    return sample_on<kRange>(g, c, field, locate_for(g, x, 0, c), locate_for(g, x, 1, c), locate_for(g, x, 2, c));
}

struct Velocity {
    Geometry geo;
    const float* field[3];

    // The three components at x: each axis is located once against its faces
    // and once against the centres, and each component takes its own mix.
    SPADE_AIRFLOW_HOT glm::vec3 at(glm::vec3 x) const noexcept {
        const Axis face0 = locate_for(geo, x, 0, 0);
        const Axis face1 = locate_for(geo, x, 1, 1);
        const Axis face2 = locate_for(geo, x, 2, 2);
        const Axis cent0 = locate_for(geo, x, 0, 1);
        const Axis cent1 = locate_for(geo, x, 1, 0);
        const Axis cent2 = locate_for(geo, x, 2, 0);
        return glm::vec3(sample_on<false>(geo, 0, field[0], face0, cent1, cent2).value,
                         sample_on<false>(geo, 1, field[1], cent0, face1, cent2).value,
                         sample_on<false>(geo, 2, field[2], cent0, cent1, face2).value);
    }
};

[[nodiscard]] Velocity velocity_sampler(const MacGrid& g, const DomainBc& bc) noexcept {
    return {geometry(g.shape, bc), {g.u.data(), g.v.data(), g.w.data()}};
}

}  // namespace

Sample sample_component(const GridShape& s, std::span<const float> field, uint32_t c, glm::vec3 x,
                        const DomainBc& bc) noexcept {
    return sample<true>(geometry(s, bc), c, field.data(), x);
}

glm::vec3 velocity_at(const MacGrid& g, glm::vec3 x, const DomainBc& bc) noexcept {
    return velocity_sampler(g, bc).at(x);
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
    const Velocity vel = velocity_sampler(in, bc);
    const auto back_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x - half_h * vel.at(x);
        return x - h * vel.at(mid);
    };
    const auto forward_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x + half_h * vel.at(x);
        return x + h * vel.at(mid);
    };
    const std::vector<float>* src[3] = {&in.u, &in.v, &in.w};
    std::vector<float>* dst[3] = {&out.u, &out.v, &out.w};
    // 1. The forward semi-Lagrangian estimate and its stencil's range.
    for (uint32_t c = 0; c < 3; ++c) {
        const float* field = src[c]->data();
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const Sample f = sample<true>(vel.geo, c, field, back_trace(x));
            s.hat[c][idx] = f.value;
            s.lo[c][idx] = f.lo;
            s.hi[c][idx] = f.hi;
        });
    }
    // 2. The backward estimate from the forward one, and the clamped correction.
    for (uint32_t c = 0; c < 3; ++c) {
        const float* hat = s.hat[c].data();
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const float back = sample<false>(vel.geo, c, hat, forward_trace(x)).value;
            float v = s.hat[c][idx] + 0.5f * ((*src[c])[idx] - back);
            v = v < s.lo[c][idx] ? s.lo[c][idx] : (v > s.hi[c][idx] ? s.hi[c][idx] : v);
            (*dst[c])[idx] = v;
        });
    }
}

}  // namespace spade::physics::airflow
