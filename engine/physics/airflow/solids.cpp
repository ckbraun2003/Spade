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
                        const float d_mid = std::max(q.distance + 0.5f * cl.thickness, 0.0f);
                        f.phi[c][idx] = d_mid;
                        float off_c = 0.0f;
                        const bool crosses = sheet_offset(d_mid, q.gradient[static_cast<glm::length_t>(c)], off_c) &&
                                             off_c > -0.5f * s.dx && off_c <= 0.5f * s.dx;
                        f.theta[c][idx] = crosses ? 0.0f : 1.0f;
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

Result<SolidField> make_solid_field(const GridShape& s) {
    if (s.nx == 0 || s.ny == 0 || s.nz == 0 || !(s.dx > 0.0f) || !std::isfinite(s.dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: a grid shape make_grid() refuses"});
    }
    uint64_t faces = 0;
    for (uint32_t c = 0; c < 3; ++c) faces += sample_count(s, c);
    if (const uint64_t bytes = faces * 16u; bytes > kGridByteCap) {
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
        lo = glm::min(lo, p.box_min - glm::vec3(grow));
        hi = glm::max(hi, p.box_max + glm::vec3(grow));
    }

    // 2. Each component's sampled index range.
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
            const std::string where = blob ? "it closes no face at dx = " + mm(s.dx) +
                                                 " (no face centre lies dx/2 inside it)" + not_sheet
                                           : "a grid line crosses it at (" + fmt("%.4f", lk.first.x) + ", " +
                                                 fmt("%.4f", lk.first.y) + ", " + fmt("%.4f", lk.first.z) +
                                                 ") m without meeting a closed face (" + std::to_string(lk.runs) +
                                                 " leaking runs, " + std::to_string(lk.off_edge) +
                                                 " beyond the edge reach)";
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

    // 6. The free-slip layers (Task 7) and the wall links (Task 8) follow here.

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
    (void)mode;  // extrapolate: Task 7
}

}  // namespace spade::physics::airflow
