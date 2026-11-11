#pragma once

// ---------------------------------------------------------------------------
// bemt_reference.hpp -- the grid-free blade-element-momentum reference of the actuator line's validation (P4a, L61; V1 and
// the V2 comparator, docs/design/test-docs/2026-11-11-l61-actuator-line-bands.md): a host solve of each annulus's induced
// speed from the momentum balance, with Prandtl's tip loss on request. Test-side because Prandtl's factor needs acos, which
// is libm and barred under engine/; the sections it loads are the engine's (physics/airflow/blade.hpp), so the line and the
// reference share them.
//
// Per annulus at radius r, the air's axial speed through the rotor is climb + v, v the induced speed solved from
//     B axial(v) = 4 pi rho r (climb + v) v F        (per unit span),
// F = 1, or Prandtl's (2/pi) acos(exp(-f)), f = (B/2) (R - r) / (r sin phi), sin phi = (climb + v) / |W|. By bisection of a
// fixed count, so the same bits on every run. The air's swirl is not fed back (wt = omega r). Thrust, torque and the
// thrust-weighted mean induced speed are the sums over annuli.
// ---------------------------------------------------------------------------

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"

namespace spade::testing {

struct BemtInput {
    const physics::airflow::BladeShape* blade = nullptr;
    const physics::airflow::PolarTable* polar = nullptr;
    float omega = 0.0f;    // rad/s, >= 0
    float rho = 1.225f;    // kg/m3
    float climb = 0.0f;    // m/s, the air's axial approach speed (V = J n D); >= 0
    uint32_t annuli = 64;  // from the first station to the tip
    bool tip_loss = false;  // Prandtl's
};

struct RotorLoads {
    float thrust = 0.0f;       // N, all blades
    float torque = 0.0f;       // N m about the axis
    float power = 0.0f;        // W, torque * omega
    float inflow_mean = 0.0f;  // m/s, the thrust-weighted mean induced speed
};

[[nodiscard]] inline Result<RotorLoads> bemt(const BemtInput& in) {
    namespace af = physics::airflow;
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "bemt: " + why});
    };
    if (in.blade == nullptr) return refuse("blade is null");
    if (in.polar == nullptr) return refuse("polar is null");
    if (!(in.omega >= 0.0f) || !std::isfinite(in.omega)) return refuse("omega must be >= 0 and finite");
    if (!(in.rho > 0.0f) || !std::isfinite(in.rho)) return refuse("rho must be positive and finite");
    if (!(in.climb >= 0.0f) || !std::isfinite(in.climb)) return refuse("climb must be >= 0 and finite");
    if (in.annuli == 0) return refuse("annuli must be at least 1");
    if (Result<void> ok = af::check_blade(*in.blade); !ok) return std::unexpected(ok.error());

    const af::BladeShape& b = *in.blade;
    const double pi = std::numbers::pi;
    const double r0 = b.stations.front().r;
    const double R = b.radius;
    const double dr = (R - r0) / static_cast<double>(in.annuli);
    const auto interpolate = [&](double r, auto pick) {
        const auto& s = b.stations;
        if (r <= s.front().r) return static_cast<double>(pick(s.front()));
        if (r >= s.back().r) return static_cast<double>(pick(s.back()));
        std::size_t i = 1;
        while (s[i].r < r) ++i;
        const double t = (r - s[i - 1].r) / (static_cast<double>(s[i].r) - s[i - 1].r);
        return static_cast<double>(pick(s[i - 1])) + t * (static_cast<double>(pick(s[i])) - pick(s[i - 1]));
    };

    double thrust = 0.0;
    double torque = 0.0;
    double weighted = 0.0;
    for (uint32_t k = 0; k < in.annuli; ++k) {
        const double r = r0 + dr * (static_cast<double>(k) + 0.5);
        const float chord = static_cast<float>(interpolate(r, [](const af::BladeStation& s) { return s.chord; }));
        const double beta = interpolate(r, [](const af::BladeStation& s) { return s.beta_deg; }) * pi / 180.0;
        const float sin_b = static_cast<float>(std::sin(beta));
        const float cos_b = static_cast<float>(std::cos(beta));
        const float wt = in.omega * static_cast<float>(r);
        const auto load = [&](double v) { return af::section_load(*in.polar, chord, sin_b, cos_b,
                                                                  static_cast<float>(in.climb + v), wt, in.rho); };
        const auto residual = [&](double v) {
            const af::SectionLoad l = load(v);
            double f = 1.0;
            if (in.tip_loss) {
                const double ua = in.climb + v;
                const double w = std::sqrt(ua * ua + static_cast<double>(wt) * wt);
                const double sphi = w > 0.0 ? ua / w : 0.0;
                if (sphi > 0.0) {
                    const double e = (static_cast<double>(b.blades) / 2.0) * (R - r) / (r * sphi);
                    f = (2.0 / pi) * std::acos(std::exp(-e));
                }
            }
            return static_cast<double>(b.blades) * l.axial - 4.0 * pi * in.rho * r * (in.climb + v) * v * f;
        };
        double lo = 0.0;
        double hi = std::max(1.0, static_cast<double>(in.omega) * R);
        double v = 0.0;
        if (residual(0.0) > 0.0) {
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi);
                (residual(mid) > 0.0 ? lo : hi) = mid;
            }
            v = 0.5 * (lo + hi);
        }
        const af::SectionLoad l = load(v);
        const double dt = static_cast<double>(b.blades) * l.axial * dr;
        thrust += dt;
        torque += static_cast<double>(b.blades) * l.tangent * r * dr;
        weighted += v * dt;
    }
    RotorLoads out;
    out.thrust = static_cast<float>(thrust);
    out.torque = static_cast<float>(torque);
    out.power = static_cast<float>(torque * in.omega);
    out.inflow_mean = thrust > 0.0 ? static_cast<float>(weighted / thrust) : 0.0f;
    return out;
}

}  // namespace spade::testing
