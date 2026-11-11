#include "physics/airflow/blade.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "core/fp32_math.hpp"

namespace spade::physics::airflow {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kDeg = kPi / 180.0f;

Error refuse(std::string why) { return Error{Code::invalid_argument, std::move(why)}; }

bool finite(float v) noexcept { return std::isfinite(v); }

// The estimated polar's pieces (blade.hpp, THE POLAR), for a shape: zero lift at alpha_0, attached 14 degrees either side of
// it, blended over 14 degrees more to the flat plate. Drag scaled by cd_scale throughout (1 = as estimated).
constexpr float kSlope = 5.5f;   // per radian
constexpr float kAttached = 14.0f;  // deg: the attached range either side of alpha_0
constexpr float kBlend = 14.0f;     // deg: the linear blend to the flat plate

float attached_cl(const EstimatedPolarShape& s, float alpha_deg) noexcept {
    return kSlope * (alpha_deg - s.alpha0_deg) * kDeg;
}
float attached_cd(const EstimatedPolarShape& s, float alpha_deg) noexcept {
    const float c = attached_cl(s, alpha_deg);
    return s.cd_scale * (0.03f + 0.02f * c * c);
}
float flat_cl(float alpha_deg) noexcept { return 0.9f * math::sin32(2.0f * alpha_deg * kDeg); }
float flat_cd(const EstimatedPolarShape& s, float alpha_deg) noexcept {
    const float sn = math::sin32(alpha_deg * kDeg);
    return s.cd_scale * (0.03f + 1.77f * sn * sn);
}

struct Coefficients {
    float cl;
    float cd;
};

Coefficients estimated_at(const EstimatedPolarShape& s, float alpha_deg) noexcept {
    const float up = s.alpha0_deg + kAttached;
    const float down = s.alpha0_deg - kAttached;
    if (alpha_deg >= down && alpha_deg <= up) return {attached_cl(s, alpha_deg), attached_cd(s, alpha_deg)};
    const float edge = alpha_deg > up ? up : down;
    const float far = alpha_deg > up ? up + kBlend : down - kBlend;
    const float t = (alpha_deg - edge) / (far - edge);  // 0 at the edge, 1 at the flat plate's start
    if (t >= 1.0f) return {flat_cl(alpha_deg), flat_cd(s, alpha_deg)};
    const Coefficients a{attached_cl(s, edge), attached_cd(s, edge)};
    const Coefficients f{flat_cl(far), flat_cd(s, far)};
    return {a.cl + (f.cl - a.cl) * t, a.cd + (f.cd - a.cd) * t};
}

// The diamond key of an angle in degrees: +-180 are the two ends, as the angle's jump is.
float key_of(float alpha_deg) noexcept {
    if (alpha_deg >= 180.0f) return 2.0f;
    if (alpha_deg <= -180.0f) return -2.0f;
    const float r = alpha_deg * kDeg;
    return diamond_angle(math::cos32(r), math::sin32(r));
}

}  // namespace

float diamond_angle(float c, float s) noexcept {
    if (!finite(c) || !finite(s)) return std::numeric_limits<float>::quiet_NaN();
    const float ac = c < 0.0f ? -c : c;
    const float as = s < 0.0f ? -s : s;
    const float d = ac + as;
    if (d == 0.0f) return 0.0f;
    const float t = s / d;
    if (c >= 0.0f) return t;
    const float m = 2.0f - as / d;
    return s < 0.0f ? -m : m;
}

Result<PolarTable> make_polar(std::span<const float> alpha_deg, std::span<const float> cl, std::span<const float> cd) {
    const std::size_t n = alpha_deg.size();
    if (n < 2) return std::unexpected(refuse("polar: needs at least 2 points, got " + std::to_string(n)));
    if (cl.size() != n || cd.size() != n) {
        return std::unexpected(refuse("polar: alpha_deg, cl and cd differ in length (" + std::to_string(n) + ", " +
                                      std::to_string(cl.size()) + ", " + std::to_string(cd.size()) + ")"));
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (!finite(alpha_deg[i])) return std::unexpected(refuse("polar: alpha_deg[" + std::to_string(i) + "] is not finite"));
        if (!finite(cl[i])) return std::unexpected(refuse("polar: cl[" + std::to_string(i) + "] is not finite"));
        if (!finite(cd[i])) return std::unexpected(refuse("polar: cd[" + std::to_string(i) + "] is not finite"));
    }
    PolarTable out;
    out.key.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (alpha_deg[i] < -180.0f || alpha_deg[i] > 180.0f) {
            return std::unexpected(refuse("polar: alpha_deg[" + std::to_string(i) + "] is outside [-180, 180]"));
        }
        if (i > 0 && !(alpha_deg[i] > alpha_deg[i - 1])) {
            return std::unexpected(refuse("polar: alpha_deg[" + std::to_string(i) + "] must be greater than alpha_deg[" +
                                          std::to_string(i - 1) + "]"));
        }
        out.key.push_back(key_of(alpha_deg[i]));
        if (i > 0 && !(out.key[i] > out.key[i - 1])) {
            return std::unexpected(refuse("polar: alpha_deg[" + std::to_string(i) + "] is too close to alpha_deg[" +
                                          std::to_string(i - 1) + "] to tell apart"));
        }
    }
    out.cl.assign(cl.begin(), cl.end());
    out.cd.assign(cd.begin(), cd.end());
    return out;
}

PolarTable estimated_polar(const EstimatedPolarShape& shape) {
    std::vector<float> alpha;
    for (int d = -180; d < -20; d += 5) alpha.push_back(static_cast<float>(d));
    for (int d = -20; d <= 20; ++d) alpha.push_back(static_cast<float>(d));
    for (int d = 25; d <= 180; d += 5) alpha.push_back(static_cast<float>(d));
    std::vector<float> cl;
    std::vector<float> cd;
    for (const float a : alpha) {
        const Coefficients c = estimated_at(shape, a);
        cl.push_back(c.cl);
        cd.push_back(c.cd);
    }
    // Built from its own nodes: well-formed by construction.
    return *make_polar(alpha, cl, cd);
}

SectionCoefficients polar_at(const PolarTable& polar, float cos_alpha, float sin_alpha) noexcept {
    const std::size_t n = polar.key.size();
    if (n == 0) return {};
    const float k = diamond_angle(cos_alpha, sin_alpha);
    if (!finite(k)) return {k, k};
    if (k <= polar.key.front()) return {polar.cl.front(), polar.cd.front()};
    if (k >= polar.key.back()) return {polar.cl.back(), polar.cd.back()};
    const std::size_t hi = static_cast<std::size_t>(std::upper_bound(polar.key.begin(), polar.key.end(), k) -
                                                    polar.key.begin());
    const std::size_t lo = hi - 1;
    const float t = (k - polar.key[lo]) / (polar.key[hi] - polar.key[lo]);
    return {polar.cl[lo] + (polar.cl[hi] - polar.cl[lo]) * t, polar.cd[lo] + (polar.cd[hi] - polar.cd[lo]) * t};
}

SectionLoad section_load(const PolarTable& polar, float chord, float sin_beta, float cos_beta, float ua, float wt,
                         float rho) noexcept {
    const float w2 = ua * ua + wt * wt;
    if (w2 == 0.0f) return {};
    const float w = std::sqrt(w2);
    const float cphi = wt / w;
    const float sphi = ua / w;
    const float sin_alpha = sin_beta * cphi - cos_beta * sphi;
    const float cos_alpha = cos_beta * cphi + sin_beta * sphi;
    const SectionCoefficients c = polar_at(polar, cos_alpha, sin_alpha);
    const float q = 0.5f * rho * w2 * chord;
    const float lift = q * c.cl;
    const float drag = q * c.cd;
    return {lift * cphi - drag * sphi, lift * sphi + drag * cphi};
}

Result<void> check_blade(const BladeShape& blade) {
    if (blade.blades == 0) return std::unexpected(refuse("blade: blades must be at least 1"));
    if (!(blade.radius > 0.0f) || !finite(blade.radius)) {
        return std::unexpected(refuse("blade: radius must be positive and finite"));
    }
    if (blade.stations.size() < 2) return std::unexpected(refuse("blade: stations needs at least 2, got " +
                                                                  std::to_string(blade.stations.size())));
    for (std::size_t i = 0; i < blade.stations.size(); ++i) {
        const BladeStation& s = blade.stations[i];
        const std::string at = "blade: stations[" + std::to_string(i) + "].";
        if (!(s.r >= 0.0f) || !(s.r <= blade.radius)) {
            return std::unexpected(refuse(at + "r must be within [0, radius]"));
        }
        if (i > 0 && !(s.r > blade.stations[i - 1].r)) return std::unexpected(refuse(at + "r must increase"));
        if (!(s.chord > 0.0f) || !finite(s.chord)) {
            return std::unexpected(refuse(at + "chord must be positive and finite"));
        }
        if (!finite(s.beta_deg)) return std::unexpected(refuse(at + "beta_deg must be finite"));
    }
    return {};
}

}  // namespace spade::physics::airflow
