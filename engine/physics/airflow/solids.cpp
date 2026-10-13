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

    // 3. Pass 1: theta, the wall velocity and phi at every sample in range.
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
                    const SolidQuery q = sample.nearest(sample.ctx, x);
                    if (q.part == kNoSolidPart && q.distance == std::numeric_limits<float>::max()) continue;
                    if (!std::isfinite(q.distance) || !finite3(q.gradient)) {
                        return refuse("the sampler reported a non-finite distance or gradient at (" +
                                      std::to_string(x.x) + ", " + std::to_string(x.y) + ", " + std::to_string(x.z) +
                                      ") m");
                    }
                    if (q.part >= parts.size()) {
                        return refuse("the sampler reported part " + std::to_string(q.part) + " of " +
                                      std::to_string(parts.size()));
                    }
                    if (!finite3(q.velocity)) {
                        return refuse("the sampler reported a non-finite velocity for part '" + parts[q.part].name +
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
                }
            }
        }
    }

    // 4. A periodic axis: the last face plane is the first.
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

    // 5. The measured safeguard (Task 2), the free-slip layers (Task 7) and
    //    the wall links (Task 8) follow here.

    // 6. The run line.
    uint32_t walls = 0;
    uint32_t sheets = 0;
    std::string sheet_text;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        if (classes[p].role == SolidRole::wall) {
            ++walls;
            continue;
        }
        ++sheets;
        sheet_text += "; " + parts[p].name + ": sheet (" + mm(classes[p].thickness) + " < 2 dx = " + mm(2.0f * s.dx) + ")";
    }
    f.run_line = "solids: " + std::to_string(parts.size()) + " parts, " + std::to_string(walls) + " walls, " +
                 std::to_string(sheets) + " sheets" + sheet_text;
    f.classes = std::move(classes);
    f.range = range;
    f.any = true;
    return {};
}

}  // namespace spade::physics::airflow
