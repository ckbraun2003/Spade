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
[[nodiscard]] inline double next_omega(double, double, double) noexcept { return std::numeric_limits<double>::quiet_NaN(); }

[[nodiscard]] inline bool settled(std::span<const Window>, const Rule& = {}) noexcept { return false; }

template <class Plant>
[[nodiscard]] Result run(Plant&&, double, const Config& = {}) {
    return {};
}

}  // namespace spade::test::fpv_trim
