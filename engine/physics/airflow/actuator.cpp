#include "physics/airflow/actuator.hpp"

#include <array>
#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <vector>

#include "core/detail/flush_subnormals.hpp"

namespace spade::physics::airflow {

namespace {

// cos of 2 pi m / 64, m = 0..63: literals, not libm (TD-3), each the float
// nearest the double cos, as triage 2's ring_points() took it; its three
// residues of about 1e-16 (m = 16, 48 and sin's 32) are 0. sin(2 pi m / 64) is
// kCos[(m + 48) % 64], a quarter turn back. A disc of 16 or 32 angles steps
// through it by 4 or 2: at 16 these are master's kCos[16] and kSin[16] to the
// bit, so a 3 x 16 disc spreads as it did before the dense disc.
constexpr float kCos[kDiscAngles] = {
    1.0f,          0.9951847f,    0.98078525f,   0.95694035f,
    0.9238795f,    0.8819213f,    0.8314696f,    0.77301043f,
    0.70710677f,   0.6343933f,    0.55557024f,   0.47139674f,
    0.38268343f,   0.29028466f,   0.19509032f,   0.09801714f,
    0.0f,          -0.09801714f,  -0.19509032f,  -0.29028466f,
    -0.38268343f,  -0.47139674f,  -0.55557024f,  -0.6343933f,
    -0.70710677f,  -0.77301043f,  -0.8314696f,   -0.8819213f,
    -0.9238795f,   -0.95694035f,  -0.98078525f,  -0.9951847f,
    -1.0f,         -0.9951847f,   -0.98078525f,  -0.95694035f,
    -0.9238795f,   -0.8819213f,   -0.8314696f,   -0.77301043f,
    -0.70710677f,  -0.6343933f,   -0.55557024f,  -0.47139674f,
    -0.38268343f,  -0.29028466f,  -0.19509032f,  -0.09801714f,
    0.0f,          0.09801714f,   0.19509032f,   0.29028466f,
    0.38268343f,   0.47139674f,   0.55557024f,   0.6343933f,
    0.70710677f,   0.77301043f,   0.8314696f,    0.8819213f,
    0.9238795f,    0.95694035f,   0.98078525f,   0.9951847f,
};
static_assert(kDiscAngles % 4 == 0, "sin is cos a quarter turn back");

[[nodiscard]] constexpr float sin_at(uint32_t m) noexcept {
    return kCos[(m + 3 * kDiscAngles / 4) % kDiscAngles];
}

// The disc's plane: e1, e2 and the axis, right-handed (e1 x e2 = axis).
struct DiscFrame {
    glm::vec3 e1;
    glm::vec3 e2;
};

[[nodiscard]] DiscFrame disc_frame(const ActuatorDisc& d) noexcept {
    const glm::vec3 helper = std::fabs(d.axis.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 c1 = glm::cross(d.axis, helper);
    const glm::vec3 e1 = c1 / std::sqrt(glm::dot(c1, c1));
    return {e1, glm::cross(d.axis, e1)};
}

[[nodiscard]] float ring_radius(const ActuatorDisc& d, uint32_t ring, uint32_t rings) noexcept {
    return d.radius * std::sqrt((static_cast<float>(ring) + 0.5f) / static_cast<float>(rings));
}

// The density's points, ring by ring; the slots past n.count stay 0.
[[nodiscard]] std::array<glm::vec3, kDiscPoints> disc_points(const ActuatorDisc& d, const DiscDensity& n) noexcept {
    const DiscFrame f = disc_frame(d);
    const uint32_t step = kDiscAngles / n.angles;
    std::array<glm::vec3, kDiscPoints> out{};
    for (uint32_t ring = 0; ring < n.rings; ++ring) {
        const float r = ring_radius(d, ring, n.rings);
        for (uint32_t m = 0; m < n.angles; ++m) {
            out[ring * n.angles + m] = d.centre + r * (kCos[m * step] * f.e1 + sin_at(m * step) * f.e2);
        }
    }
    return out;
}

// Each point's swirl direction over its radius, t / r: t = axis x (cos e1 +
// sin e2) = cos e2 - sin e1, the unit tangent about the axis.
[[nodiscard]] std::array<glm::vec3, kDiscPoints> swirl_over_radius(const ActuatorDisc& d,
                                                                    const DiscDensity& n) noexcept {
    const DiscFrame f = disc_frame(d);
    const uint32_t step = kDiscAngles / n.angles;
    std::array<glm::vec3, kDiscPoints> out{};
    for (uint32_t ring = 0; ring < n.rings; ++ring) {
        const float inv_r = 1.0f / ring_radius(d, ring, n.rings);
        for (uint32_t m = 0; m < n.angles; ++m) {
            out[ring * n.angles + m] = inv_r * (kCos[m * step] * f.e2 - sin_at(m * step) * f.e1);
        }
    }
    return out;
}

// The larger of the outer ring's arc and the rings' radial pitch, in dx.
[[nodiscard]] float spacing_cells(float radius, float dx) noexcept {
    const DiscDensity n = disc_density(radius, dx);
    const float cells = radius / dx;
    const float rings = static_cast<float>(n.rings);
    const float outer = cells * std::sqrt((rings - 0.5f) / rings);
    const float arc = 2.0f * std::numbers::pi_v<float> * outer / static_cast<float>(n.angles);
    const float pitch = cells / rings;
    return arc > pitch ? arc : pitch;
}

// "disc 2", "discs 0-3" or "discs 0, 2".
[[nodiscard]] std::string disc_names(const std::vector<std::size_t>& at) {
    if (at.size() == 1u) return std::format("disc {}", at[0]);
    if (at.back() - at.front() + 1u == at.size()) return std::format("discs {}-{}", at.front(), at.back());
    std::string out = "discs ";
    for (std::size_t i = 0; i < at.size(); ++i) out += (i == 0u ? "" : ", ") + std::to_string(at[i]);
    return out;
}

}  // namespace

float peskin4(float r) noexcept {
    const float a = r < 0.0f ? -r : r;
    if (a < 1.0f) return 0.125f * (3.0f - 2.0f * a + std::sqrt(1.0f + 4.0f * a - 4.0f * a * a));
    if (a < 2.0f) return 0.125f * (5.0f - 2.0f * a - std::sqrt(-7.0f + 12.0f * a - 4.0f * a * a));
    return 0.0f;
}

DiscDensity disc_density(float radius, float dx) noexcept {
    static_assert(kDiscRings == 10 && kDiscAngles == 64, "the rule's cap: 10 rings of 64");
    const float cells = radius / dx;  // R/dx
    uint32_t rings = 3;               // clamp(ceil(R / 2dx), 3, 10)
    while (rings < kDiscRings && static_cast<float>(rings) < 0.5f * cells) ++rings;
    uint32_t angles = 16;  // the smallest of 16, 32 and 64 at or above pi R/dx; 64 if none is
    while (angles < kDiscAngles && static_cast<float>(angles) < std::numbers::pi_v<float> * cells) angles *= 2u;
    return {.rings = rings, .angles = angles, .count = rings * angles};
}

std::string disc_spacing_note(std::span<const float> radii, float dx) {
    std::string out;
    std::vector<bool> named(radii.size(), false);
    for (std::size_t i = 0; i < radii.size(); ++i) {
        const float spacing = spacing_cells(radii[i], dx);
        if (named[i] || !(spacing > 2.0f)) continue;
        std::vector<std::size_t> same;
        for (std::size_t j = i; j < radii.size(); ++j) {
            if (radii[j] == radii[i]) {
                same.push_back(j);
                named[j] = true;
            }
        }
        const bool one = same.size() == 1u;
        out += std::format("{}{} at R/dx {:.1f} {} {} points {:.1f} dx apart (over 2 dx)", out.empty() ? "" : "; ",
                           disc_names(same), radii[i] / dx, one ? "spaces" : "space", one ? "its" : "their", spacing);
    }
    return out;
}

Result<void> check_disc(const ActuatorDisc& d, const GridShape& s) {
    const auto refuse = [](const char* what) {
        return std::unexpected(Error{Code::invalid_argument, std::string("actuator disc: ") + what});
    };
    if (!(d.radius > 0.0f) || !std::isfinite(d.radius)) return refuse("radius must be positive and finite");
    if (!std::isfinite(d.thrust)) return refuse("thrust must be finite");
    if (!std::isfinite(d.torque)) return refuse("torque must be finite");
    if (!(std::fabs(glm::dot(d.axis, d.axis) - 1.0f) <= 1.0e-5f)) return refuse("axis must be a unit vector");
    const float reach = d.radius + 2.0f * s.dx;
    const float extent[3] = {static_cast<float>(s.nx) * s.dx, static_cast<float>(s.ny) * s.dx,
                             static_cast<float>(s.nz) * s.dx};
    for (uint32_t a = 0; a < 3; ++a) {
        const float c = d.centre[static_cast<glm::length_t>(a)];
        if (!(c - reach >= 0.0f) || !(c + reach <= extent[a])) return refuse("its kernel support leaves the grid");
    }
    return {};
}

DiscSpread disc_spread(const ActuatorDisc& d, const GridShape& s, float rho, float h) noexcept {
    DiscSpread out;
    const DiscDensity n = disc_density(d.radius, s.dx);
    out.rings = n.rings;
    out.angles = n.angles;
    out.count = n.count;
    out.points = disc_points(d, n);
    out.scale = h * d.thrust / (static_cast<float>(n.count) * rho * s.dx * s.dx * s.dx);
    out.reach = d.radius + 2.0f * s.dx;
    // Without a torque this is the thrust-only spread, bit for bit (both goldens' discs).
    out.swirl = d.torque != 0.0f;
    if (out.swirl) out.tangent = swirl_over_radius(d, n);
    out.swirl_scale = h * d.torque / (static_cast<float>(n.count) * rho * s.dx * s.dx * s.dx);
    return out;
}

namespace {

// spread_actuator()'s body, run under its wrapper's scope (the wrapper rule,
// core/detail/flush_subnormals.hpp); disc_spread() runs inside it.
SPADE_NOINLINE void spread_flushed(const ActuatorDisc& d, const GridShape& s, float rho, float h, std::span<float> du,
                                   std::span<float> dv, std::span<float> dw) noexcept {
    const DiscSpread spread = disc_spread(d, s, rho, h);
    const std::array<glm::vec3, kDiscPoints>& pts = spread.points;
    const std::array<glm::vec3, kDiscPoints>& tangent = spread.tangent;
    const float inv_dx = 1.0f / s.dx;
    const float scale = spread.scale;
    const float reach = spread.reach;
    const bool swirl = spread.swirl;
    const float swirl_scale = spread.swirl_scale;
    const uint32_t count = spread.count;
    const std::span<float> out[3] = {du, dv, dw};
    for (uint32_t c = 0; c < 3; ++c) {
        const auto cl = static_cast<glm::length_t>(c);
        const float axis_c = d.axis[cl];
        if (axis_c == 0.0f && !swirl) continue;
        for_each_sample(s, c, [&](std::size_t idx, glm::vec3 x) {
            // Outside this box every point's weight is exactly 0, so skipping it changes nothing.
            if (std::fabs(x.x - d.centre.x) > reach || std::fabs(x.y - d.centre.y) > reach ||
                std::fabs(x.z - d.centre.z) > reach) {
                return;
            }
            float sum = 0.0f;
            float swirl_sum = 0.0f;
            for (uint32_t i = 0; i < count; ++i) {
                const glm::vec3 f = (x - pts[i]) * inv_dx;
                const float w = peskin4(f.x) * peskin4(f.y) * peskin4(f.z);
                sum += w;
                if (swirl) swirl_sum += tangent[i][cl] * w;
            }
            out[c][idx] += swirl ? scale * axis_c * sum + swirl_scale * swirl_sum : scale * axis_c * sum;
        });
    }
}

}  // namespace

// Subnormals flushed as fluid_step() flushes them (spec section 4.5 as amended
// 2026-10-22; the GPU's twin is B4's actuator_gather).
void spread_actuator(const ActuatorDisc& d, const GridShape& s, float rho, float h, std::span<float> du,
                     std::span<float> dv, std::span<float> dw) noexcept {
    const spade::math::ScopedFlushSubnormals flush;
    spread_flushed(d, s, rho, h, du, dv, dw);
}

}  // namespace spade::physics::airflow
