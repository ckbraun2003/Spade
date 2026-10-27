#include "physics/airflow/solids.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] std::string mm(float metres) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.1f mm", static_cast<double>(metres) * 1000.0);
    return buf;
}

[[nodiscard]] std::size_t sample_count(const GridShape& s, uint32_t c) noexcept {
    return std::size_t{c == 0 ? s.nx + 1u : s.nx} * (c == 1 ? s.ny + 1u : s.ny) * (c == 2 ? s.nz + 1u : s.nz);
}

[[nodiscard]] std::array<uint32_t, 3> sample_dims(const GridShape& s, uint32_t c) noexcept {
    return {c == 0 ? s.nx + 1u : s.nx, c == 1 ? s.ny + 1u : s.ny, c == 2 ? s.nz + 1u : s.nz};
}

void open_everything(SolidField& f) {
    for (uint32_t c = 0; c < 3; ++c) {
        std::fill(f.theta[c].begin(), f.theta[c].end(), 1.0f);
        std::fill(f.wall[c].begin(), f.wall[c].end(), 0.0f);
        std::fill(f.phi[c].begin(), f.phi[c].end(), std::numeric_limits<float>::max());
        std::fill(f.link[c].begin(), f.link[c].end(), kNoWallLinks);
        f.fill1[c].clear();
        f.fill2[c].clear();
        std::fill(f.layer[c].begin(), f.layer[c].end(), uint8_t{0});
        f.range[c] = SolidRange{};
    }
    f.links.clear();
    f.classes.clear();
    f.any = false;
}

// Where a sheet's mid-surface crosses the line through a sample along an axis,
// as a signed offset from the sample: the sheet lies against the gradient,
// which points away from it. False when the line runs along the sheet.
[[nodiscard]] bool sheet_offset(float d_mid, float g_axis, float& s) noexcept {
    if (g_axis == 0.0f) return false;
    s = (g_axis > 0.0f ? -d_mid : d_mid) / std::fabs(g_axis);
    return true;
}

[[nodiscard]] bool finite3(glm::vec3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// A sheet sample's distance to the sheet's TRUE mid-surface (P2 Task 11, the
// final review's finding 1). The class thickness can exceed the thickness
// here (a cambered plate's box, a curved shell's grazing crossing); a
// mid-surface placed from it sat too far, and faces it crossed stayed open.
// So the thickness is measured along the normal: from the near surface point
// x - q.distance g, a probe 2 dx deep reads the far surface, t = 2 dx -
// d(probe), clamped to [0, 2 dx]. Exact for a slab or a radial shell; where the
// probe leaves the sheet (a rim) t only shrinks, which closes more, never less.
[[nodiscard]] float sheet_mid_distance(const SolidSampler& sample, uint32_t part, const SolidQuery& q, glm::vec3 x,
                                       float dx) noexcept {
    const float reach = 2.0f * dx;
    const glm::vec3 probe = x - (q.distance + reach) * q.gradient;
    const SolidQuery far = sample.part(sample.ctx, part, probe);
    const float t = std::isfinite(far.distance) ? std::min(std::max(reach - far.distance, 0.0f), reach) : 0.0f;
    return std::max(q.distance + 0.5f * t, 0.0f);
}

// Each component's sample index range over the box [lo, hi] (m), inclusive
// and clipped to the grid; empty where the box misses it.
[[nodiscard]] std::array<SolidRange, 3> sampled_range(const GridShape& s, glm::vec3 lo, glm::vec3 hi) noexcept {
    const float inv_dx = 1.0f / s.dx;
    std::array<SolidRange, 3> range{};
    for (uint32_t c = 0; c < 3; ++c) {
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        range[c].empty = false;
        for (uint32_t a = 0; a < 3; ++a) {
            const float off = a == c ? 0.0f : 0.5f;
            const float q0 = -std::floor(-(lo[static_cast<glm::length_t>(a)] * inv_dx - off));  // ceil
            const float q1 = std::floor(hi[static_cast<glm::length_t>(a)] * inv_dx - off);
            const float top = static_cast<float>(dims[a] - 1u);
            const float c0 = std::max(q0, 0.0f);
            const float c1 = std::min(q1, top);
            if (c1 < c0) {
                range[c].empty = true;
                break;
            }
            range[c].lo[a] = static_cast<uint32_t>(c0);
            range[c].hi[a] = static_cast<uint32_t>(c1);
        }
    }
    return range;
}

// Pass 1 (theta, the wall velocity and phi in range) and the periodic copy.
// closes[p] is set when a sample part p wins has theta = 0. Every sample in
// range is reset to open first, so a second pass leaves nothing of the first.
[[nodiscard]] Result<void> sample_pass(SolidField& f, const GridShape& s, std::array<bool, 3> periodic,
                                       std::span<const SolidPart> parts, const std::vector<SolidClass>& classes,
                                       const SolidSampler& sample, const std::array<SolidRange, 3>& range,
                                       std::vector<uint8_t>& closes) {
    const auto fail = [](const std::string& why) -> Result<void> {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: " + why});
    };
    const float inv_dx = 1.0f / s.dx;
    f.links.clear();
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const glm::vec3 off(c == 0 ? 0.0f : 0.5f, c == 1 ? 0.0f : 0.5f, c == 2 ? 0.0f : 0.5f);
        for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
            for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const glm::vec3 x = (glm::vec3(static_cast<float>(i), static_cast<float>(j),
                                                   static_cast<float>(k)) +
                                         off) *
                                        s.dx;
                    f.theta[c][idx] = 1.0f;  // open, until a part says otherwise
                    f.wall[c][idx] = 0.0f;
                    f.phi[c][idx] = std::numeric_limits<float>::max();
                    f.link[c][idx] = kNoWallLinks;
                    const SolidQuery q = sample.nearest(sample.ctx, x);
                    if (q.part == kNoSolidPart && q.distance == std::numeric_limits<float>::max()) continue;
                    if (!std::isfinite(q.distance) || !finite3(q.gradient)) {
                        return fail("the sampler reported a non-finite distance or gradient at (" +
                                      std::to_string(x.x) + ", " + std::to_string(x.y) + ", " + std::to_string(x.z) +
                                      ") m");
                    }
                    if (q.part >= parts.size()) {
                        return fail("the sampler reported part " + std::to_string(q.part) + " of " +
                                      std::to_string(parts.size()));
                    }
                    if (!finite3(q.velocity)) {
                        return fail("the sampler reported a non-finite velocity for part '" + parts[q.part].name +
                                      "'");
                    }
                    const SolidClass& cl = classes[q.part];
                    f.wall[c][idx] = q.velocity[static_cast<glm::length_t>(c)];
                    if (cl.role == SolidRole::wall) {
                        f.phi[c][idx] = q.distance;
                        f.theta[c][idx] = std::min(std::max(0.5f + q.distance * inv_dx, 0.0f), 1.0f);
                    } else {
                        const float d_mid = sheet_mid_distance(sample, q.part, q, x, s.dx);
                        f.phi[c][idx] = d_mid;
                        float off_c = 0.0f;
                        const bool crosses = sheet_offset(d_mid, q.gradient[static_cast<glm::length_t>(c)], off_c) &&
                                             off_c > -0.5f * s.dx && off_c <= 0.5f * s.dx;
                        f.theta[c][idx] = crosses ? 0.0f : 1.0f;
                        // The sheet's links (Task 8): along each axis the
                        // mid-surface lies against the gradient, d_mid / |g_a| away.
                        for (uint32_t a = 0; a < 3; ++a) {
                            const float g_a = q.gradient[static_cast<glm::length_t>(a)];
                            if (g_a == 0.0f) continue;
                            const float dist = d_mid / std::fabs(g_a);
                            if (dist > s.dx) continue;
                            const uint32_t dir = g_a > 0.0f ? 0u : 1u;  // the sheet's side: -a when the gradient points +a
                            const uint32_t qa = a == 0 ? i : (a == 1 ? j : k);
                            if (dir == 0u ? qa == 0u : qa + 1u == dims[a]) continue;  // no neighbour there
                            const float frac = std::max(dist / s.dx, kMinCrossing);
                            if (f.link[c][idx] == kNoWallLinks) {
                                f.link[c][idx] = static_cast<uint32_t>(f.links.size());
                                f.links.push_back(WallLinks{});
                            }
                            WallLinks& wl = f.links[f.link[c][idx]];
                            wl.inv_f[2u * a + dir] = 1.0f / frac;
                            wl.wall[2u * a + dir] = q.velocity[static_cast<glm::length_t>(c)];
                        }
                    }
                    // A sheet meeting a wall (the final review's finding 2): theta
                    // comes from the nearest part, so near the join the wall wins
                    // faces the sheet crosses, and its ramp left a slot open along
                    // it. Each sheet whose box (2 dx out) holds x is asked as well,
                    // and its mid-surface crossing closes the face.
                    if (cl.role == SolidRole::wall && f.theta[c][idx] > 0.0f) {
                        for (uint32_t p = 0; p < parts.size(); ++p) {
                            if (p == q.part || classes[p].role != SolidRole::sheet) continue;
                            const glm::vec3 lo = parts[p].box_min - glm::vec3(2.0f * s.dx);
                            const glm::vec3 hi = parts[p].box_max + glm::vec3(2.0f * s.dx);
                            if (glm::any(glm::lessThan(x, lo)) || glm::any(glm::greaterThan(x, hi))) continue;
                            const SolidQuery qp = sample.part(sample.ctx, p, x);
                            if (!std::isfinite(qp.distance) || !finite3(qp.gradient) || !finite3(qp.velocity)) continue;
                            float off_p = 0.0f;
                            const float d_p = sheet_mid_distance(sample, p, qp, x, s.dx);
                            if (sheet_offset(d_p, qp.gradient[static_cast<glm::length_t>(c)], off_p) &&
                                off_p > -0.5f * s.dx && off_p <= 0.5f * s.dx) {
                                f.theta[c][idx] = 0.0f;
                                f.wall[c][idx] = qp.velocity[static_cast<glm::length_t>(c)];
                                closes[p] = 1u;
                                break;
                            }
                        }
                    }
                    if (f.theta[c][idx] == 0.0f) closes[q.part] = 1u;
                }
            }
        }
    }

    // A periodic axis: the last face plane is the first.
    for (uint32_t c = 0; c < 3; ++c) {
        if (!periodic[c]) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const uint32_t n = dims[c] - 1u;
        for (uint32_t k = 0; k < dims[2]; ++k) {
            for (uint32_t j = 0; j < dims[1]; ++j) {
                for (uint32_t i = 0; i < dims[0]; ++i) {
                    const uint32_t q[3] = {i, j, k};
                    if (q[c] != n) continue;
                    uint32_t first[3] = {i, j, k};
                    first[c] = 0u;
                    const std::size_t to = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const std::size_t from = first[0] + std::size_t{dims[0]} * (first[1] + std::size_t{dims[1]} * first[2]);
                    f.theta[c][to] = f.theta[c][from];
                    f.wall[c][to] = f.wall[c][from];
                    f.phi[c][to] = f.phi[c][from];
                }
            }
        }
    }
    return {};
}

// The 4-cell reach and the 2% bound (design section 4).
constexpr int kEdgeReachCells = 4;
constexpr float kEdgeAreaFraction = 0.02f;
constexpr int kTraceSteps = 32;
constexpr float kTraceHit = 1.0e-4f;  // times dx
constexpr float kSheetWidthCells = 2.0f;

// Whether a thin crossing's material is a sheet's there (the lead, 2026-10-13:
// the criterion is local): along each tangent of its normal n, a probe
// kSheetWidthCells dx from its mid-point, one way or the other, lands within
// dx/2 of the part. A sheet, flat or curved, is wide (a shell of radius R
// leaves its tangent plane by w^2 / 2R); a wire is narrow across itself, a
// bead every way.
[[nodiscard]] bool wide(const SolidSampler& sample, uint32_t p, glm::vec3 mid, glm::vec3 n, float dx) noexcept {
    const float ax = std::fabs(n.x);
    const float ay = std::fabs(n.y);
    const float az = std::fabs(n.z);
    const glm::vec3 pick = ax <= ay && ax <= az ? glm::vec3(1.0f, 0.0f, 0.0f)
                           : ay <= az           ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                : glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 t1 = glm::cross(n, pick);
    t1 /= std::sqrt(glm::dot(t1, t1));
    const glm::vec3 t2 = glm::cross(n, t1);
    const float w = kSheetWidthCells * dx;
    const float near = 0.5f * dx;
    for (const glm::vec3& t : {t1, t2}) {
        const float ahead = sample.part(sample.ctx, p, mid + w * t).distance;
        const float behind = sample.part(sample.ctx, p, mid - w * t).distance;
        if (!(ahead <= near) && !(behind <= near)) return false;
    }
    return true;
}

struct Trace {
    float t = 0.0f;           // from the start, m
    glm::vec3 normal{0.0f};   // the part's gradient at the last point
    bool hit = false;
    bool finite = true;
};

// Sphere-traces part `p` from `from` along `dir` (unit), up to `length`: t
// advances by phi, a hit is phi <= kTraceHit dx, at most kTraceSteps steps.
[[nodiscard]] Trace trace(const SolidSampler& sample, uint32_t p, glm::vec3 from, glm::vec3 dir, float length,
                          float dx) noexcept {
    Trace out;
    const float eps = kTraceHit * dx;
    SolidQuery q = sample.part(sample.ctx, p, from);
    for (int step = 0; step < kTraceSteps; ++step) {
        if (!std::isfinite(q.distance) || !finite3(q.gradient)) {
            out.finite = false;
            return out;
        }
        out.normal = q.gradient;
        if (q.distance <= eps) {
            out.hit = true;
            return out;
        }
        out.t += q.distance;
        if (out.t >= length) {
            out.t = length;
            return out;
        }
        q = sample.part(sample.ctx, p, from + out.t * dir);
    }
    out.hit = std::isfinite(q.distance) && q.distance <= eps;
    if (out.hit) out.normal = q.gradient;
    return out;
}

struct Leaks {
    uint32_t runs = 0;          // leaking runs
    uint32_t off_edge = 0;      // of them, outside the edge reach
    float area = 0.0f;          // their open area, m^2
    glm::vec3 first{0.0f};      // the first one's entry point
    uint32_t thin = 0;          // thin crossings
    float thin_tau_max = 0.0f;  // the largest thin crossing's, m
    bool narrow = false;        // a thin crossing narrower than kSheetWidthCells along a tangent
};

// Cells with a closed face, grown by kEdgeReachCells (Chebyshev).
[[nodiscard]] std::vector<uint8_t> near_closed(const SolidField& f, const GridShape& s) {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    std::vector<uint8_t> mask(cells, 0u);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const bool any_closed = closed(f.theta[0][u_index(s, i, j, k)]) ||
                                        closed(f.theta[0][u_index(s, i + 1u, j, k)]) ||
                                        closed(f.theta[1][v_index(s, i, j, k)]) ||
                                        closed(f.theta[1][v_index(s, i, j + 1u, k)]) ||
                                        closed(f.theta[2][w_index(s, i, j, k)]) ||
                                        closed(f.theta[2][w_index(s, i, j, k + 1u)]);
                mask[cell_index(s, i, j, k)] = any_closed ? 1u : 0u;
            }
        }
    }
    std::vector<uint8_t> tmp(cells, 0u);
    const std::size_t stride[3] = {1u, s.nx, std::size_t{s.nx} * s.ny};
    for (uint32_t a = 0; a < 3; ++a) {  // a separable max filter, axis by axis
        for (uint32_t k = 0; k < s.nz; ++k) {
            for (uint32_t j = 0; j < s.ny; ++j) {
                for (uint32_t i = 0; i < s.nx; ++i) {
                    const uint32_t q[3] = {i, j, k};
                    const std::size_t c = cell_index(s, i, j, k);
                    uint8_t v = 0u;
                    for (int d = -kEdgeReachCells; d <= kEdgeReachCells && v == 0u; ++d) {
                        const int64_t at = int64_t{q[a]} + d;
                        if (at < 0 || at >= int64_t{n[a]}) continue;
                        v = mask[static_cast<std::size_t>(int64_t(c) + d * int64_t(stride[a]))];
                    }
                    tmp[c] = v;
                }
            }
        }
        mask.swap(tmp);
    }
    return mask;
}

// Part p's cell box: its posed box grown by 2 dx, as inclusive cell indices.
struct CellBox {
    std::array<uint32_t, 3> lo{};
    std::array<uint32_t, 3> hi{};
};

[[nodiscard]] CellBox cell_box(const GridShape& s, const SolidPart& p) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const float inv_dx = 1.0f / s.dx;
    CellBox b;
    for (uint32_t a = 0; a < 3; ++a) {
        const auto al = static_cast<glm::length_t>(a);
        const float q0 = -std::floor(-((p.box_min[al] - 2.0f * s.dx) * inv_dx - 0.5f));
        const float q1 = std::floor((p.box_max[al] + 2.0f * s.dx) * inv_dx - 0.5f);
        b.lo[a] = static_cast<uint32_t>(std::min(std::max(q0, 0.0f), static_cast<float>(n[a] - 1u)));
        b.hi[a] = static_cast<uint32_t>(std::min(std::max(q1, 0.0f), static_cast<float>(n[a] - 1u)));
    }
    return b;
}

// Part p's leaks: every run along every axis through its cell box.
[[nodiscard]] Result<Leaks> find_leaks(const SolidField& f, const GridShape& s, const SolidSampler& sample,
                                       uint32_t p, const SolidPart& part, const std::vector<uint8_t>& near,
                                       std::vector<float>& cache) {
    Leaks out;
    const CellBox b = cell_box(s, part);
    const uint32_t ext[3] = {b.hi[0] - b.lo[0] + 1u, b.hi[1] - b.lo[1] + 1u, b.hi[2] - b.lo[2] + 1u};
    cache.assign(std::size_t{ext[0]} * ext[1] * ext[2], 0.0f);
    const auto centre = [&](uint32_t i, uint32_t j, uint32_t k) {
        return (glm::vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) + 0.5f) * s.dx;
    };
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) {
        return (i - b.lo[0]) + std::size_t{ext[0]} * ((j - b.lo[1]) + std::size_t{ext[1]} * (k - b.lo[2]));
    };
    for (uint32_t k = b.lo[2]; k <= b.hi[2]; ++k) {
        for (uint32_t j = b.lo[1]; j <= b.hi[1]; ++j) {
            for (uint32_t i = b.lo[0]; i <= b.hi[0]; ++i) {
                const SolidQuery q = sample.part(sample.ctx, p, centre(i, j, k));
                if (!std::isfinite(q.distance)) {
                    return std::unexpected(Error{Code::invalid_argument, "airflow solids: the sampler reported a "
                                                                         "non-finite distance for part '" +
                                                                             part.name + "'"});
                }
                cache[at(i, j, k)] = q.distance;
            }
        }
    }
    const float dx = s.dx;
    for (uint32_t a = 0; a < 3; ++a) {
        const uint32_t a1 = (a + 1u) % 3u;
        const uint32_t a2 = (a + 2u) % 3u;
        const uint32_t outer = a1 > a2 ? a1 : a2;  // k-like, then j-like, then along the line
        const uint32_t inner = a1 > a2 ? a2 : a1;
        glm::vec3 e(0.0f);
        e[static_cast<glm::length_t>(a)] = 1.0f;
        for (uint32_t u = b.lo[outer]; u <= b.hi[outer]; ++u) {
            for (uint32_t v = b.lo[inner]; v <= b.hi[inner]; ++v) {
                int64_t o1 = -1;
                for (uint32_t q = b.lo[a]; q <= b.hi[a]; ++q) {
                    uint32_t ijk[3];
                    ijk[a] = q;
                    ijk[outer] = u;
                    ijk[inner] = v;
                    const float phi = cache[at(ijk[0], ijk[1], ijk[2])];
                    if (!(phi > 0.0f)) continue;  // inside: the run goes on
                    if (o1 >= 0) {
                        // A run from o1 to q.
                        uint32_t c1[3] = {ijk[0], ijk[1], ijk[2]};
                        c1[a] = static_cast<uint32_t>(o1);
                        const uint32_t k_inside = q - static_cast<uint32_t>(o1) - 1u;
                        const float phi1 = cache[at(c1[0], c1[1], c1[2])];
                        const float length = static_cast<float>(q - static_cast<uint32_t>(o1)) * dx;
                        const glm::vec3 x1 = centre(c1[0], c1[1], c1[2]);
                        const glm::vec3 x2 = centre(ijk[0], ijk[1], ijk[2]);
                        if (k_inside > 0u || phi1 + phi <= dx) {
                            const Trace in = trace(sample, p, x1, e, length, dx);
                            const Trace out_t = trace(sample, p, x2, -e, length, dx);
                            if (!in.finite || !out_t.finite) {
                                return std::unexpected(Error{Code::invalid_argument,
                                                             "airflow solids: the sampler reported a non-finite "
                                                             "distance for part '" + part.name + "'"});
                            }
                            if (k_inside > 0u || in.hit) {
                                const float chord = std::max(length - in.t - out_t.t, 0.0f);
                                const float tau = chord * std::fabs(in.normal[static_cast<glm::length_t>(a)]);
                                const bool thin = glm::dot(in.normal, out_t.normal) <= -0.5f && tau < 2.0f * dx;
                                if (thin) {
                                    const glm::vec3 mid = 0.5f * ((x1 + in.t * e) + (x2 - out_t.t * e));
                                    if (!wide(sample, p, mid, in.normal, dx)) out.narrow = true;
                                    ++out.thin;
                                    out.thin_tau_max = std::max(out.thin_tau_max, tau);
                                    float smallest = 1.0f;
                                    bool all_open = true;
                                    for (uint32_t face = static_cast<uint32_t>(o1) + 1u; face <= q; ++face) {
                                        uint32_t fi[3] = {ijk[0], ijk[1], ijk[2]};
                                        fi[a] = face;
                                        const std::size_t idx = a == 0 ? u_index(s, fi[0], fi[1], fi[2])
                                                                : (a == 1 ? v_index(s, fi[0], fi[1], fi[2])
                                                                          : w_index(s, fi[0], fi[1], fi[2]));
                                        const float th = f.theta[a][idx];
                                        if (closed(th)) {
                                            all_open = false;
                                            break;
                                        }
                                        smallest = std::min(smallest, th);
                                    }
                                    if (all_open) {
                                        if (out.runs == 0u) out.first = x1 + in.t * e;
                                        ++out.runs;
                                        out.area += smallest * dx * dx;
                                        for (uint32_t cq = static_cast<uint32_t>(o1); cq <= q; ++cq) {
                                            uint32_t ci[3] = {ijk[0], ijk[1], ijk[2]};
                                            ci[a] = cq;
                                            if (near[cell_index(s, ci[0], ci[1], ci[2])] == 0u) {
                                                ++out.off_edge;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    o1 = q;
                }
            }
        }
    }
    return out;
}

// A blob's cover (the lead's ruling, 2026-10-27): over part p's sampled range
// (its box grown by 2 dx), the samples p alone would block (its own phi <=
// dx/2, theta < 1), and of them those closed (theta = 0) by another part, the
// sample's winner. `by` marks, per part, a winner of a covered sample.
struct Cover {
    uint32_t blocking = 0;
    uint32_t covered = 0;
    std::vector<uint8_t> by;
};

[[nodiscard]] Result<Cover> cover_of(const SolidField& f, const GridShape& s, const SolidSampler& sample, uint32_t p,
                                     std::span<const SolidPart> parts) {
    Cover out;
    out.by.assign(parts.size(), 0u);
    const glm::vec3 grow(2.0f * s.dx);
    const std::array<SolidRange, 3> range = sampled_range(s, parts[p].box_min - grow, parts[p].box_max + grow);
    const float blocks = 0.5f * s.dx;
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const glm::vec3 off(c == 0 ? 0.0f : 0.5f, c == 1 ? 0.0f : 0.5f, c == 2 ? 0.0f : 0.5f);
        for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
            for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const glm::vec3 x = (glm::vec3(static_cast<float>(i), static_cast<float>(j),
                                                   static_cast<float>(k)) +
                                         off) *
                                        s.dx;
                    const SolidQuery own = sample.part(sample.ctx, p, x);
                    if (!std::isfinite(own.distance)) {
                        return std::unexpected(Error{Code::invalid_argument, "airflow solids: the sampler reported a "
                                                                             "non-finite distance for part '" +
                                                                                 parts[p].name + "'"});
                    }
                    if (!(own.distance <= blocks)) continue;  // p alone leaves it open
                    ++out.blocking;
                    if (!closed(f.theta[c][idx])) continue;
                    const uint32_t winner = sample.nearest(sample.ctx, x).part;
                    if (winner == p || winner >= parts.size()) continue;
                    ++out.covered;
                    out.by[winner] = 1u;
                }
            }
        }
    }
    return out;
}

// The parts `by` marks, quoted, in part order: 'a'; 'a' and 'b'; 'a', 'b'
// and 'c'.
[[nodiscard]] std::string quoted_parts(std::span<const SolidPart> parts, const std::vector<uint8_t>& by) {
    std::vector<std::string> names;
    for (std::size_t q = 0; q < parts.size(); ++q) {
        if (by[q] != 0u) names.push_back("'" + parts[q].name + "'");
    }
    std::string out;
    for (std::size_t n = 0; n < names.size(); ++n) {
        if (n > 0u) out += n + 1u == names.size() ? " and " : ", ";
        out += names[n];
    }
    return out;
}

[[nodiscard]] std::string fmt(const char* format, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, format, v);
    return buf;
}

}  // namespace

Result<SolidClass> classify_part(const SolidPart& p, float dx) {
    const auto refuse = [&](const std::string& why) -> Result<SolidClass> {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: part '" + p.name + "' " + why});
    };
    if (!(dx > 0.0f) || !std::isfinite(dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: dx must be positive and finite"});
    }
    if (p.porous) return refuse("is porous; porous parts arrive with the porous task (the solids design, section 5)");
    if (!(p.min_feature >= 0.0f) || !std::isfinite(p.min_feature)) {
        return refuse("has a min_feature that is not finite and non-negative");
    }
    std::array<float, 3> e{p.extent.x, p.extent.y, p.extent.z};
    for (const float v : e) {
        if (!(v >= 0.0f) || !std::isfinite(v)) return refuse("has a bounding box that is not finite and non-negative");
    }
    std::sort(e.begin(), e.end());
    const float feature = p.min_feature > 0.0f ? std::min(p.min_feature, e[0]) : e[0];
    const float two_dx = 2.0f * dx;
    if (feature >= two_dx) return SolidClass{SolidRole::wall, 0.0f};
    if (e[1] < two_dx) {
        return refuse("is below 2 dx = " + mm(two_dx) + " in two extents (smallest feature " + mm(feature) +
                      ", bounding box " + mm(e[0]) + " x " + mm(e[1]) + " x " + mm(e[2]) +
                      "): a wire or a fastener is neither a wall nor a sheet; it needs dx <= " + mm(0.5f * feature) +
                      " (refused, never dropped)");
    }
    return SolidClass{SolidRole::sheet, feature};
}

uint64_t solid_field_bytes(const GridShape& s) noexcept {
    uint64_t faces = 0;
    for (uint32_t c = 0; c < 3; ++c) faces += sample_count(s, c);
    return faces * (3u * sizeof(float) + sizeof(uint32_t) + sizeof(uint8_t));
}

Result<SolidField> make_solid_field(const GridShape& s) {
    if (s.nx == 0 || s.ny == 0 || s.nz == 0 || !(s.dx > 0.0f) || !std::isfinite(s.dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: a grid shape make_grid() refuses"});
    }
    if (const uint64_t bytes = solid_field_bytes(s); bytes > kGridByteCap) {
        return std::unexpected(Error{Code::capacity_exceeded, "airflow solids: " + std::to_string(bytes) +
                                                                   " bytes of solid field, past the cap of " +
                                                                   std::to_string(kGridByteCap)});
    }
    SolidField f;
    for (uint32_t c = 0; c < 3; ++c) {
        const std::size_t n = sample_count(s, c);
        f.theta[c].assign(n, 1.0f);
        f.wall[c].assign(n, 0.0f);
        f.phi[c].assign(n, std::numeric_limits<float>::max());
        f.link[c].assign(n, kNoWallLinks);
        f.layer[c].assign(n, uint8_t{0});
    }
    f.run_line = "solids: none";
    return f;
}

Result<void> build_solids(SolidField& f, const GridShape& s, std::array<bool, 3> periodic,
                          std::span<const SolidPart> parts, const SolidSampler& sample) {
    open_everything(f);
    f.run_line = "solids: none";
    if (parts.empty()) return {};
    const auto refuse = [&](const std::string& why) -> Result<void> {
        open_everything(f);
        f.run_line = "solids: none";
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: " + why});
    };
    if (sample.nearest == nullptr || sample.part == nullptr) return refuse("the sampler is empty");

    // 1. Classify every part, and check each grown box against the grid.
    std::vector<SolidClass> classes;
    classes.reserve(parts.size());
    const float n_metres[3] = {static_cast<float>(s.nx) * s.dx, static_cast<float>(s.ny) * s.dx,
                               static_cast<float>(s.nz) * s.dx};
    const float grow = 2.0f * s.dx;
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(-std::numeric_limits<float>::max());
    for (const SolidPart& p : parts) {
        Result<SolidClass> c = classify_part(p, s.dx);
        if (!c) {
            open_everything(f);
            f.run_line = "solids: none";
            return std::unexpected(c.error());
        }
        classes.push_back(*c);
        // A part reaching past the grid is clipped to it: validation geometry
        // fills to the domain's edge (S2's tube), and the drone box's own
        // placement check is P2b's. A part wholly outside it is refused.
        bool meets = true;
        for (uint32_t a = 0; a < 3; ++a) {
            const float a0 = p.box_min[static_cast<glm::length_t>(a)] - grow;
            const float a1 = p.box_max[static_cast<glm::length_t>(a)] + grow;
            if (!std::isfinite(a0) || !std::isfinite(a1)) return refuse("part '" + p.name + "' has a non-finite box");
            if (a1 < 0.0f || a0 > n_metres[a]) meets = false;
        }
        if (!meets) return refuse("part '" + p.name + "' lies wholly outside the grid");
        // The sampler does not wrap a periodic seam, so a part within reach of
        // one periodic face but not the other would be cut there, its walls'
        // links and faces stopping at the seam (P2 Task 11, the final review's
        // finding 5). A part spanning the whole axis (S3's sheet, S2's
        // cylinder) is the same on both sides, and is allowed.
        for (uint32_t a = 0; a < 3; ++a) {
            if (!periodic[a]) continue;
            const bool past_lo = p.box_min[static_cast<glm::length_t>(a)] - grow < 0.0f;
            const bool past_hi = p.box_max[static_cast<glm::length_t>(a)] + grow > n_metres[a];
            if (past_lo != past_hi) {
                return refuse("part '" + p.name + "' reaches across the periodic " + std::string(1, "xyz"[a]) +
                              " face: the grid does not wrap a part across a seam, so it must stay 2 dx clear of "
                              "both periodic faces or span the whole axis");
            }
        }
        lo = glm::min(lo, p.box_min - glm::vec3(grow));
        hi = glm::max(hi, p.box_max + glm::vec3(grow));
    }

    // 2. Each component's sampled index range.
    const std::array<SolidRange, 3> range = sampled_range(s, lo, hi);

    // 3-4. Pass 1 and the periodic copy.
    std::vector<uint8_t> closes(parts.size(), 0u);
    if (Result<void> ok = sample_pass(f, s, periodic, parts, classes, sample, range, closes); !ok) {
        open_everything(f);
        f.run_line = "solids: none";
        return std::unexpected(ok.error());
    }

    std::vector<uint8_t> measured(parts.size(), 0u);  // sheeted by the safeguard, which writes its note
    // 5. The measured safeguard (design section 4).
    std::string notes;
    {
        const std::vector<uint8_t> near = near_closed(f, s);
        std::vector<float> cache;
        bool resheeted = false;
        for (uint32_t p = 0; p < parts.size(); ++p) {
            if (classes[p].role != SolidRole::wall) continue;
            Result<Leaks> found = find_leaks(f, s, sample, p, parts[p], near, cache);
            if (!found) {
                open_everything(f);
                f.run_line = "solids: none";
                return std::unexpected(found.error());
            }
            const Leaks& lk = *found;
            const bool blob = closes[p] == 0u;
            if (!blob && lk.runs == 0u) continue;  // resolved
            // Covered (the lead's ruling, 2026-10-27): a blob whose every sample
            // it would block another part closes lies inside that part. It
            // won no closed sample, so it added no boundary: the field stays as
            // the pass made it, and it never reaches the sheet path below.
            Cover cover;
            if (blob) {
                Result<Cover> cv = cover_of(f, s, sample, p, parts);
                if (!cv) {
                    open_everything(f);
                    f.run_line = "solids: none";
                    return std::unexpected(cv.error());
                }
                cover = std::move(*cv);
                if (cover.blocking > 0u && cover.covered == cover.blocking) {
                    notes += "; " + parts[p].name + ": covered by " + quoted_parts(parts, cover.by);
                    continue;
                }
            }
            std::array<float, 3> e{parts[p].extent.x, parts[p].extent.y, parts[p].extent.z};
            std::sort(e.begin(), e.end());
            const float largest_face = e[1] * e[2];
            if (!blob && lk.off_edge == 0u && lk.area <= kEdgeAreaFraction * largest_face) {
                notes += "; " + parts[p].name + ": leaks at an edge (" + std::to_string(lk.runs) + " runs, open area " +
                         fmt("%.3f", static_cast<double>(lk.area / (s.dx * s.dx))) + " dx^2, " +
                         fmt("%.2f", 100.0 * static_cast<double>(lk.area / largest_face)) + "% of its largest face)";
                continue;
            }
            if (blob && lk.thin > 0u && !lk.narrow) {
                classes[p] = SolidClass{SolidRole::sheet, lk.thin_tau_max};
                resheeted = true;
                measured[p] = 1u;
                notes += "; " + parts[p].name + ": sheet (measured " + mm(lk.thin_tau_max) + " across " +
                         std::to_string(lk.thin) + " thin crossings, no closed face; declared min_feature " +
                         mm(parts[p].min_feature) + ")";
                continue;
            }
            const std::string not_sheet = lk.thin > 0u && lk.narrow ? " and is thin in more than one direction "
                                                                      "(a wire or a bead, not a sheet)"
                                                                    : "";
            // A blob no other part closes any of: no face centre lies dx/2
            // inside it. One partly inside others loses those faces to them,
            // and is exposed where they do not cover it.
            const auto covering = static_cast<std::size_t>(std::count(cover.by.begin(), cover.by.end(), uint8_t{1}));
            const std::string where =
                !blob ? "a grid line crosses it at (" + fmt("%.4f", lk.first.x) + ", " + fmt("%.4f", lk.first.y) +
                            ", " + fmt("%.4f", lk.first.z) + ") m without meeting a closed face (" +
                            std::to_string(lk.runs) + " leaking runs, " + std::to_string(lk.off_edge) +
                            " beyond the edge reach)"
                : cover.covered == 0u
                    ? "it closes no face at dx = " + mm(s.dx) + " (no face centre lies dx/2 inside it)" + not_sheet
                    : "it closes no face of its own at dx = " + mm(s.dx) + " and is exposed where it is not covered (" +
                          quoted_parts(parts, cover.by) + (covering == 1u ? " closes " : " close ") +
                          std::to_string(cover.covered) + " of the " + std::to_string(cover.blocking) +
                          " faces it would block; " + std::to_string(cover.blocking - cover.covered) +
                          " stay open)" + not_sheet;
            open_everything(f);
            f.run_line = "solids: none";
            return std::unexpected(Error{Code::invalid_argument, "airflow solids: part '" + parts[p].name +
                                                                     "' leaks: " + where +
                                                                     "; declare min_feature or refine dx"});
        }
        if (resheeted) {
            std::fill(closes.begin(), closes.end(), uint8_t{0});
            if (Result<void> again = sample_pass(f, s, periodic, parts, classes, sample, range, closes); !again) {
                open_everything(f);
                f.run_line = "solids: none";
                return std::unexpected(again.error());
            }
        }
    }

    // 6. The free-slip layers (Task 7).
    // The free-slip layers (Task 7): closed samples with an open neighbour of
    // their own component, then closed samples with a first-layer neighbour.
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        for (uint32_t layer = 1; layer <= 2; ++layer) {
            for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
                for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                    for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                        const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                        if (!closed(f.theta[c][idx]) || f.layer[c][idx] != 0u) continue;
                        const uint32_t q[3] = {i, j, k};
                        bool next_to = false;
                        for (uint32_t a = 0; a < 3 && !next_to; ++a) {
                            for (int d = -1; d <= 1; d += 2) {
                                if ((d < 0 && q[a] == 0u) || (d > 0 && q[a] + 1u == dims[a])) continue;
                                const std::size_t nb = d < 0 ? idx - stride[a] : idx + stride[a];
                                if (layer == 1u ? !closed(f.theta[c][nb]) : f.layer[c][nb] == 1u) {
                                    next_to = true;
                                    break;
                                }
                            }
                        }
                        if (!next_to) continue;
                        (layer == 1u ? f.fill1[c] : f.fill2[c]).push_back(static_cast<uint32_t>(idx));
                    }
                }
            }
            for (const uint32_t idx : (layer == 1u ? f.fill1[c] : f.fill2[c])) f.layer[c][idx] = static_cast<uint8_t>(layer);
        }
    }

    // The wall links (Task 8): a solved sample (theta > 0, phi >= 0) next to
    // a sample inside a wall (phi < 0) crosses the wall at f = phi_s / (phi_s
    // - phi_n) of the link (Gibou, Fedkiw, Cheng and Kang 2002), floored at
    // kMinCrossing; the wall's velocity there blends the two samples' rigid
    // velocities, exact for one rigid owner. A sheet link already nearer wins.
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
            for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const float phi_s = f.phi[c][idx];
                    if (closed(f.theta[c][idx]) || phi_s < 0.0f) continue;  // fixed: not solved
                    const uint32_t q[3] = {i, j, k};
                    for (uint32_t a = 0; a < 3; ++a) {
                        for (uint32_t dir = 0; dir < 2; ++dir) {
                            if (dir == 0u ? q[a] == 0u : q[a] + 1u == dims[a]) continue;
                            const std::size_t nb = dir == 0u ? idx - stride[a] : idx + stride[a];
                            const float phi_n = f.phi[c][nb];
                            if (!(phi_n < 0.0f)) continue;
                            const float frac = std::max(phi_s / (phi_s - phi_n), kMinCrossing);
                            const float inv = 1.0f / frac;
                            if (f.link[c][idx] == kNoWallLinks) {
                                f.link[c][idx] = static_cast<uint32_t>(f.links.size());
                                f.links.push_back(WallLinks{});
                            }
                            WallLinks& wl = f.links[f.link[c][idx]];
                            if (wl.inv_f[2u * a + dir] >= inv) continue;
                            wl.inv_f[2u * a + dir] = inv;
                            wl.wall[2u * a + dir] = f.wall[c][idx] + frac * (f.wall[c][nb] - f.wall[c][idx]);
                        }
                    }
                }
            }
        }
    }

    // 7. The run line.
    uint32_t walls = 0;
    uint32_t sheets = 0;
    std::string sheet_text;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        if (classes[p].role == SolidRole::wall) {
            ++walls;
            continue;
        }
        ++sheets;
        if (measured[p] == 0u) sheet_text += "; " + parts[p].name + ": sheet (" + mm(classes[p].thickness) + " < 2 dx = " + mm(2.0f * s.dx) + ")";
    }
    f.run_line = "solids: " + std::to_string(parts.size()) + " parts, " + std::to_string(walls) + " walls, " +
                 std::to_string(sheets) + " sheets" + sheet_text + notes;
    f.classes = std::move(classes);
    f.range = range;
    f.any = true;
    return {};
}

void apply_solid_velocities(MacGrid& g, const SolidField& f, SolidInterior mode) noexcept {
    if (!f.any) return;
    std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        if (f.range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(g.shape, c);
        const SolidRange& rg = f.range[c];
        for (uint32_t k = rg.lo[2]; k <= rg.hi[2]; ++k) {
            for (uint32_t j = rg.lo[1]; j <= rg.hi[1]; ++j) {
                for (uint32_t i = rg.lo[0]; i <= rg.hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    if (closed(f.theta[c][idx])) (*vel[c])[idx] = f.wall[c][idx];
                }
            }
        }
    }
    if (mode != SolidInterior::extrapolate) return;
    for (uint32_t c = 0; c < 3; ++c) {
        const std::array<uint32_t, 3> dims = sample_dims(g.shape, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        std::vector<float>& v = *vel[c];
        for (uint32_t layer = 1; layer <= 2; ++layer) {
            for (const uint32_t idx : (layer == 1u ? f.fill1[c] : f.fill2[c])) {
                const uint32_t q[3] = {static_cast<uint32_t>(idx % dims[0]),
                                       static_cast<uint32_t>((idx / dims[0]) % dims[1]),
                                       static_cast<uint32_t>(idx / (std::size_t{dims[0]} * dims[1]))};
                float sum = 0.0f;
                uint32_t count = 0;
                for (uint32_t a = 0; a < 3; ++a) {
                    for (int d = -1; d <= 1; d += 2) {
                        if ((d < 0 && q[a] == 0u) || (d > 0 && q[a] + 1u == dims[a])) continue;
                        const std::size_t nb = d < 0 ? idx - stride[a] : idx + stride[a];
                        const bool source = layer == 1u ? !closed(f.theta[c][nb]) : f.layer[c][nb] == 1u;
                        if (!source) continue;
                        sum += v[nb];
                        ++count;
                    }
                }
                v[idx] = sum / static_cast<float>(count);  // a deliberate per-sample division: count is 1 to 6
            }
        }
    }
}

}  // namespace spade::physics::airflow
