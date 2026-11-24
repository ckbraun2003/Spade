#pragma once
// The hover trim's controller and its stopping rule (the FPV film's one trim run, O158 and L-S31), as pure functions of the window series so a fake
// plant can test them offline. The driver owns the solve; this owns what the next speed is and when the series has settled.
//
// A window is one controller period: the rotor speed held through it, and the mean over it of (the lines' thrust + the air's force on the frame) as a
// share of the weight. The next speed is a Newton step in omega for a thrust that varies as omega squared, damped by a gain; the series has settled
// when the last two windows agree on speed and share and the latest share is at one. A run that does not settle inside the cap reports UNSETTLED and
// its speed is never used for the film.
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace spade::test::fpv_trim {

struct Window {
    double omega = 0.0;  // rad/s, held through the window
    double share = 0.0;  // mean (line thrust + air force on the frame) / weight over the window
};

struct Rule {
    double omega_tol = 0.0025;    // the last two windows' speeds agree within this, relative
    double share_tol = 0.005;     // their shares agree within this, absolute
    double near_one = 0.01;       // and the latest share is within this of 1
    std::size_t min_windows = 12;  // never settled before this many windows: 12 x 516 = 6,192 steps, the wake's development (50 to 100 ms)
};

struct Config {
    double gain = 0.25;                   // chosen against fake plants with lag and dead time (the RED's table)
    std::size_t window_steps = 516;       // two revolutions at the hover speed
    std::size_t cap_steps = 16000;        // covers a 100 ms plant lag; ends earlier at the rule
    Rule rule{};
};

enum class Outcome { settled, unsettled, refused };

struct Result {
    Outcome outcome = Outcome::refused;
    std::vector<Window> windows;  // every window run, in order
    double omega = std::numeric_limits<double>::quiet_NaN();  // the settled speed (the last window's); NaN unless settled
    std::size_t steps = 0;        // steps run
};

// A Newton step in omega with gain k, for a thrust that varies as omega squared: omega * (1 + k (1 - share) / (2 share)). NaN for a non-finite or
// non-positive omega or share, or a gain outside (0, 1].
[[nodiscard]] inline double next_omega(double omega, double share, double gain) noexcept {
    if (!(omega > 0.0) || !std::isfinite(omega) || !(share > 0.0) || !std::isfinite(share) || !(gain > 0.0 && gain <= 1.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return omega * (1.0 + gain * 0.5 * (1.0 - share) / share);
}

[[nodiscard]] inline bool settled(std::span<const Window> w, const Rule& rule = {}) noexcept {
    const std::size_t n = w.size();
    if (n < rule.min_windows || n < 2) return false;
    const Window& a = w[n - 2];
    const Window& b = w[n - 1];
    for (const double v : {a.omega, a.share, b.omega, b.share}) {
        if (!std::isfinite(v)) return false;
    }
    return std::fabs(b.omega - a.omega) <= rule.omega_tol * b.omega && std::fabs(b.share - a.share) <= rule.share_tol &&
           std::fabs(b.share - 1.0) <= rule.near_one;
}

// Runs the controller against a plant: plant(omega, steps) advances the plant `steps` steps with the rotor at omega and returns the window's share.
// Stops at the rule, at the cap (whole windows only), or refused on a bad configuration or a non-finite or non-positive share.
template <class Plant>
[[nodiscard]] Result run(Plant&& plant, double omega0, const Config& cfg = {}) {
    Result r;
    if (!(omega0 > 0.0) || !std::isfinite(omega0) || !(cfg.gain > 0.0 && cfg.gain <= 1.0) || cfg.window_steps == 0 ||
        cfg.cap_steps < cfg.window_steps) {
        return r;
    }
    double omega = omega0;
    for (std::size_t steps = 0; steps + cfg.window_steps <= cfg.cap_steps; steps += cfg.window_steps) {
        const double share = plant(omega, cfg.window_steps);
        r.steps = steps + cfg.window_steps;
        r.windows.push_back({omega, share});
        if (!(share > 0.0) || !std::isfinite(share)) return r;  // refused
        if (settled(r.windows, cfg.rule)) {
            r.outcome = Outcome::settled;
            r.omega = omega;
            return r;
        }
        omega = next_omega(omega, share, cfg.gain);
    }
    r.outcome = Outcome::unsettled;
    return r;
}

}  // namespace spade::test::fpv_trim
