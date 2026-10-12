#include "physics/airflow/actuator.hpp"

#include <array>
#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {

// cos and sin of 2 pi m / 16, m = 0..15: literals, not libm (TD-3).
constexpr float kCos[kDiscAngles] = {1.0f,         0.92387953f,  0.70710678f,  0.38268343f,
                                     0.0f,         -0.38268343f, -0.70710678f, -0.92387953f,
                                     -1.0f,        -0.92387953f, -0.70710678f, -0.38268343f,
                                     0.0f,         0.38268343f,  0.70710678f,  0.92387953f};
constexpr float kSin[kDiscAngles] = {0.0f,         0.38268343f,  0.70710678f,  0.92387953f,
                                     1.0f,         0.92387953f,  0.70710678f,  0.38268343f,
                                     0.0f,         -0.38268343f, -0.70710678f, -0.92387953f,
                                     -1.0f,        -0.92387953f, -0.70710678f, -0.38268343f};

[[nodiscard]] std::array<glm::vec3, kDiscPoints> disc_points(const ActuatorDisc& d) noexcept {
    const glm::vec3 helper = std::fabs(d.axis.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 c1 = glm::cross(d.axis, helper);
    const glm::vec3 e1 = c1 / std::sqrt(glm::dot(c1, c1));
    const glm::vec3 e2 = glm::cross(d.axis, e1);
    std::array<glm::vec3, kDiscPoints> out{};
    for (uint32_t ring = 0; ring < kDiscRings; ++ring) {
        const float r = d.radius * std::sqrt((static_cast<float>(ring) + 0.5f) / static_cast<float>(kDiscRings));
        for (uint32_t m = 0; m < kDiscAngles; ++m) {
            out[ring * kDiscAngles + m] = d.centre + r * (kCos[m] * e1 + kSin[m] * e2);
        }
    }
    return out;
}

}  // namespace

float peskin4(float r) noexcept {
    const float a = r < 0.0f ? -r : r;
    if (a < 1.0f) return 0.125f * (3.0f - 2.0f * a + std::sqrt(1.0f + 4.0f * a - 4.0f * a * a));
    if (a < 2.0f) return 0.125f * (5.0f - 2.0f * a - std::sqrt(-7.0f + 12.0f * a - 4.0f * a * a));
    return 0.0f;
}

Result<void> check_disc(const ActuatorDisc& d, const GridShape& s) {
    const auto refuse = [](const char* what) {
        return std::unexpected(Error{Code::invalid_argument, std::string("actuator disc: ") + what});
    };
    if (!(d.radius > 0.0f) || !std::isfinite(d.radius)) return refuse("radius must be positive and finite");
    if (!std::isfinite(d.thrust)) return refuse("thrust must be finite");
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

void spread_actuator(const ActuatorDisc& d, const GridShape& s, float rho, float h, std::span<float> du,
                     std::span<float> dv, std::span<float> dw) noexcept {
    const std::array<glm::vec3, kDiscPoints> pts = disc_points(d);
    const float inv_dx = 1.0f / s.dx;
    const float scale = h * d.thrust / (static_cast<float>(kDiscPoints) * rho * s.dx * s.dx * s.dx);
    const float reach = d.radius + 2.0f * s.dx;
    const std::span<float> out[3] = {du, dv, dw};
    for (uint32_t c = 0; c < 3; ++c) {
        const float axis_c = d.axis[static_cast<glm::length_t>(c)];
        if (axis_c == 0.0f) continue;
        for_each_sample(s, c, [&](std::size_t idx, glm::vec3 x) {
            // Outside this box every point's weight is exactly 0, so skipping it changes nothing.
            if (std::fabs(x.x - d.centre.x) > reach || std::fabs(x.y - d.centre.y) > reach ||
                std::fabs(x.z - d.centre.z) > reach) {
                return;
            }
            float sum = 0.0f;
            for (const glm::vec3& p : pts) {
                const glm::vec3 f = (x - p) * inv_dx;
                sum += peskin4(f.x) * peskin4(f.y) * peskin4(f.z);
            }
            out[c][idx] += scale * axis_c * sum;
        });
    }
}

}  // namespace spade::physics::airflow
