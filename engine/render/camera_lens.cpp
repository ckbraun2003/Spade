#include "render/camera_lens.hpp"

#include <cmath>
#include <cstddef>
#include <format>

#include "core/detail/math_ops_f64.hpp"

namespace spade::render {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;
constexpr int kNewtonIterations = 16;      // fixed: no data-dependent exit
constexpr int kFieldOfViewSamples = 4096;  // the monotone scan of lens_issue()

// theta_d(theta), and its slope, for the KB4 coefficients k1..k4.
[[nodiscard]] double theta_d(const double* k, double t) {
    const double u = t * t;
    return t * (1.0 + u * (k[0] + u * (k[1] + u * (k[2] + u * k[3]))));
}

[[nodiscard]] double theta_d_slope(const double* k, double t) {
    const double u = t * t;
    return 1.0 + u * (3.0 * k[0] + u * (5.0 * k[1] + u * (7.0 * k[2] + u * 9.0 * k[3])));
}

// sin and cos of an angle in radians, from the engine's degree kernel.
void sincos_rad(double t, double& s, double& c) { math::sincos_deg_arith(t * (180.0 / kPi), s, c); }

// The widest normalised image radius the pixels ask for: the farthest of the four image corners (the pixel grid spans [0, width] x [0, height]).
[[nodiscard]] double corner_radius(const LensSpec& s) {
    double worst = 0.0;
    for (const double x : {0.0, static_cast<double>(s.width)}) {
        for (const double y : {0.0, static_cast<double>(s.height)}) {
            const double a = (x - s.cx) / s.fx;
            const double b = (y - s.cy) / s.fy;
            const double r = std::sqrt(a * a + b * b);
            worst = r > worst ? r : worst;
        }
    }
    return worst;
}

[[nodiscard]] bool finite(double v) { return std::isfinite(v); }

}  // namespace

std::string lens_issue(const LensSpec& s) {
    if (s.kind > kLensKb4) return std::format("lens kind {} is not 0 (pinhole) or 1 (KB4): above KB4 is reserved", s.kind);
    if (s.width == 0u) return "lens width must be at least 1";
    if (s.height == 0u) return "lens height must be at least 1";
    if (!(finite(s.fx) && s.fx > 0.0)) return "lens fx must be finite and above 0";
    if (!(finite(s.fy) && s.fy > 0.0)) return "lens fy must be finite and above 0";
    if (!finite(s.cx)) return "lens cx must be finite";
    if (!finite(s.cy)) return "lens cy must be finite";
    for (int i = 0; i < 4; ++i) {
        if (!finite(s.k[i])) return std::format("lens k{} must be finite", i + 1);
        if (s.kind == kLensPinhole && s.k[i] != 0.0) return std::format("lens k{} must be 0 for a pinhole lens", i + 1);
    }
    if (s.kind == kLensPinhole) return {};

    // KB4: walk the angle from the axis out to 90 degrees. theta_d must grow (slope above 0) until it reaches the widest corner's radius, which is the
    // field of view the pixels ask for; a fold before that has no unique unprojection. Never reaching it by 90 degrees is a lens past 180 degrees.
    const double r_max = corner_radius(s);
    for (int i = 1; i <= kFieldOfViewSamples; ++i) {
        const double t = kHalfPi * static_cast<double>(i) / static_cast<double>(kFieldOfViewSamples);
        if (!(theta_d_slope(s.k, t) > 0.0)) {
            // the coefficient whose term pulls the slope down the most at this angle: 3 k1 u, 5 k2 u^2, 7 k3 u^3, 9 k4 u^4
            const double u = t * t;
            const double term[4] = {3.0 * s.k[0] * u, 5.0 * s.k[1] * u * u, 7.0 * s.k[2] * u * u * u, 9.0 * s.k[3] * u * u * u * u};
            int worst = 0;
            for (int j = 1; j < 4; ++j) worst = term[j] < term[worst] ? j : worst;
            return std::format("lens theta_d(theta) is not monotone: it stops growing at {:.3f} rad off the axis, before the corners' radius {:.3f} is reached; "
                               "the coefficient k{} turns it over",
                               t, r_max, worst + 1);
        }
        if (theta_d(s.k, t) >= r_max) return {};
    }
    return std::format("lens corners look beyond 90 degrees off the axis (a field of view past 180 degrees): theta_d at 90 degrees is {:.3f}, the corners' radius is {:.3f}",
                       theta_d(s.k, kHalfPi), r_max);
}

glm::dvec3 lens_ray(const LensSpec& s, double px, double py) {
    const double a = (px - s.cx) / s.fx;
    const double b = (py - s.cy) / s.fy;
    if (s.kind == kLensPinhole) {
        const double n = std::sqrt(a * a + b * b + 1.0);
        return {a / n, b / n, 1.0 / n};
    }
    const double r = std::sqrt(a * a + b * b);
    if (r == 0.0) return {0.0, 0.0, 1.0};
    // theta_d(theta) = r by Newton from theta = r, a fixed number of steps, kept inside [0, 90 degrees]
    double t = r < kHalfPi ? r : kHalfPi;
    for (int i = 0; i < kNewtonIterations; ++i) {
        const double slope = theta_d_slope(s.k, t);
        if (slope > 1e-12) t -= (theta_d(s.k, t) - r) / slope;
        t = t < 0.0 ? 0.0 : (t > kHalfPi ? kHalfPi : t);
    }
    double sn = 0.0, cs = 1.0;
    sincos_rad(t, sn, cs);
    return {sn * a / r, sn * b / r, cs};
}

std::optional<glm::dvec2> lens_project(const LensSpec& s, const glm::dvec3& p) {
    if (!(p.z > 0.0)) return std::nullopt;  // at or behind the image plane, or a NaN
    const double rho = std::sqrt(p.x * p.x + p.y * p.y);
    if (rho == 0.0) return glm::dvec2(s.cx, s.cy);
    if (s.kind == kLensPinhole) return glm::dvec2(s.fx * (p.x / p.z) + s.cx, s.fy * (p.y / p.z) + s.cy);
    const double td = theta_d(s.k, math::atan2_arith(rho, p.z));
    return glm::dvec2(s.fx * td * (p.x / rho) + s.cx, s.fy * td * (p.y / rho) + s.cy);
}

std::vector<glm::vec3> lens_table(const LensSpec& s) {
    std::vector<glm::vec3> table;
    table.reserve(static_cast<std::size_t>(s.width) * s.height);
    for (uint32_t j = 0; j < s.height; ++j) {
        for (uint32_t i = 0; i < s.width; ++i) {
            const glm::dvec3 d = lens_ray(s, static_cast<double>(i) + 0.5, static_cast<double>(j) + 0.5);
            table.emplace_back(static_cast<float>(d.x), static_cast<float>(d.y), static_cast<float>(d.z));
        }
    }
    return table;
}

}  // namespace spade::render
