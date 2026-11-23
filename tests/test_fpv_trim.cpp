// The hover trim's controller and its stopping rule (tests/fpv_trim.hpp), against fake plants. The gain, the window, the floor and the cap were
// CHOSEN by this table, offline, before the one GPU trim run: a first-order lag of the share toward (omega / omega*)^2 with a plant lag of 430 or
// 2,000 steps (the gate's own settling, and 0.05 s of simulated time), a pure dead time of 0, 500, 1,000 or 2,000 steps in front of it (the wake's
// flight to the frame is about 200 to 600 steps; the longer ones are stress rows), and starts at 0.93 and 1.09 of the trimmed speed.
//
// Why 0.25: the share the controller reads is stale by the plant's lag and delay, so a full Newton step overshoots. Measured on these plants at
// 516-step windows: a gain of 0.4 stops settling at 1,000 steps of delay on the short lag, and 0.3 settles the 2,000-step lag with 500 steps of
// delay only on the cap's last window; 0.25 settles every row the plant can plausibly be (up to 500 steps of delay) with margin, and the rows
// that do not settle by the cap decay.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "fpv_trim.hpp"

namespace ft = spade::test::fpv_trim;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The fake plant: the share relaxes toward (the omega held `delay` steps ago)^2 with a first-order lag, sampled once per window.
struct FakePlant {
    double lag_steps;
    double share;
    std::deque<double> history;
    FakePlant(double lag, int delay_steps, double start, std::size_t window_steps)
        : lag_steps(lag),
          share(start * start),
          history(static_cast<std::size_t>(std::lround(static_cast<double>(delay_steps) / static_cast<double>(window_steps))), start) {}
    double operator()(double omega, std::size_t steps) {
        history.push_back(omega);
        const double effective = history.front();
        history.pop_front();
        const double a = 1.0 - std::exp(-static_cast<double>(steps) / lag_steps);
        share += a * (effective * effective - share);
        return share;
    }
};

}  // namespace

TEST(FpvTrim, TheDefaultsAreWhatTheTableChose) {
    const ft::Config c;
    EXPECT_EQ(c.gain, 0.25);
    EXPECT_EQ(c.window_steps, 516u) << "two revolutions at the hover speed";
    EXPECT_EQ(c.cap_steps, 16000u) << "covers a 100 ms plant lag; the run ends earlier at the rule";
    EXPECT_EQ(c.rule.min_windows, 12u) << "12 x 516 = 6,192 steps, the same simulated time as six windows of 1,032";
    EXPECT_EQ(c.rule.omega_tol, 0.0025);
    EXPECT_EQ(c.rule.share_tol, 0.005);
    EXPECT_EQ(c.rule.near_one, 0.01);
}

TEST(FpvTrim, TheNextSpeedIsANewtonStepWithTheGainAndRefusesWhatItCannotStep) {
    EXPECT_EQ(ft::next_omega(100.0, 1.0, 0.25), 100.0) << "at the weight the speed stays";
    EXPECT_NEAR(ft::next_omega(100.0, 0.9, 1.0), 100.0 * (1.0 + 0.5 * 0.1 / 0.9), 1e-12);
    EXPECT_NEAR(ft::next_omega(100.0, 0.9, 0.25), 100.0 * (1.0 + 0.25 * 0.5 * 0.1 / 0.9), 1e-12);
    EXPECT_NEAR(ft::next_omega(100.0, 1.2, 0.25), 100.0 * (1.0 - 0.25 * 0.5 * 0.2 / 1.2), 1e-12);
    EXPECT_GT(ft::next_omega(100.0, 0.5, 0.25), 100.0) << "short of the weight: faster";
    EXPECT_LT(ft::next_omega(100.0, 2.0, 0.25), 100.0) << "over the weight: slower";
    for (const double bad : {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
        EXPECT_TRUE(std::isnan(ft::next_omega(bad, 1.0, 0.25))) << "omega " << bad;
        EXPECT_TRUE(std::isnan(ft::next_omega(100.0, bad, 0.25))) << "share " << bad;
    }
    for (const double bad : {0.0, -0.1, 1.0001, kNaN}) {
        EXPECT_TRUE(std::isnan(ft::next_omega(100.0, 1.0, bad))) << "gain " << bad;
    }
    EXPECT_FALSE(std::isnan(ft::next_omega(100.0, 1.0, 1.0))) << "a gain of exactly 1 is the full step and is allowed";
}

TEST(FpvTrim, TheRuleSettlesOnlyOnAgreeingWindowsAtTheWeightAfterTheFloor) {
    const ft::Rule rule;
    const auto series = [](std::size_t n, double omega, double share) { return std::vector<ft::Window>(n, ft::Window{omega, share}); };
    EXPECT_TRUE(ft::settled(series(12, 100.0, 1.0), rule));
    EXPECT_FALSE(ft::settled(series(11, 100.0, 1.0), rule)) << "never before 12 windows, however well they agree";
    EXPECT_FALSE(ft::settled(series(0, 100.0, 1.0), rule));
    ft::Rule one;
    one.min_windows = 1;
    EXPECT_FALSE(ft::settled(series(1, 100.0, 1.0), one)) << "two windows are the least that can agree";
    // the three conditions, each failing alone
    auto w = series(12, 100.0, 1.0);
    w.back().omega = 100.0 * 1.003;
    EXPECT_FALSE(ft::settled(w, rule)) << "speeds 0.3% apart";
    w.back().omega = 100.0 * 1.002;
    EXPECT_TRUE(ft::settled(w, rule)) << "speeds 0.2% apart";
    w = series(12, 100.0, 1.0);
    w.back().share = 1.006;
    EXPECT_FALSE(ft::settled(w, rule)) << "shares 0.6 points apart";
    w = series(12, 100.0, 1.012);
    EXPECT_FALSE(ft::settled(w, rule)) << "agreeing, but 1.2% off the weight";
    w = series(12, 100.0, 0.992);
    EXPECT_TRUE(ft::settled(w, rule)) << "agreeing and within 1%";
    // an oscillation that crosses the weight every window never settles
    w.clear();
    for (std::size_t i = 0; i < 20; ++i) {
        w.push_back({100.0 * (i % 2 == 0 ? 0.95 : 1.05), i % 2 == 0 ? 0.9 : 1.1});
    }
    EXPECT_FALSE(ft::settled(w, rule));
    // a non-finite window is never settled
    w = series(12, 100.0, 1.0);
    w.back().share = kNaN;
    EXPECT_FALSE(ft::settled(w, rule));
    w = series(12, 100.0, 1.0);
    w.back().omega = kNaN;
    EXPECT_FALSE(ft::settled(w, rule));
}

TEST(FpvTrim, TheRunRefusesABadConfigurationAndAPlantThatReturnsNonsense) {
    const auto flat = [](double, std::size_t) { return 1.0; };
    const double w0 = 100.0;
    ft::Config c;
    EXPECT_EQ(ft::run(flat, w0, c).outcome, ft::Outcome::settled) << "a plant already at the weight settles at the floor";
    EXPECT_EQ(ft::run(flat, w0, c).steps, 12u * 516u);
    for (const double bad : {0.0, -5.0, kNaN}) {
        EXPECT_EQ(ft::run(flat, bad, c).outcome, ft::Outcome::refused) << bad;
    }
    c.gain = 0.0;
    EXPECT_EQ(ft::run(flat, w0, c).outcome, ft::Outcome::refused);
    c = ft::Config{};
    c.gain = 1.5;
    EXPECT_EQ(ft::run(flat, w0, c).outcome, ft::Outcome::refused);
    c = ft::Config{};
    c.window_steps = 0;
    EXPECT_EQ(ft::run(flat, w0, c).outcome, ft::Outcome::refused);
    c = ft::Config{};
    c.cap_steps = 100;
    EXPECT_EQ(ft::run(flat, w0, c).outcome, ft::Outcome::refused) << "a cap shorter than one window";
    c = ft::Config{};
    const auto nan_plant = [](double, std::size_t) { return kNaN; };
    const ft::Result r = ft::run(nan_plant, w0, c);
    EXPECT_EQ(r.outcome, ft::Outcome::refused) << "a non-finite share stops the run, it is not stepped on";
    EXPECT_EQ(r.windows.size(), 1u);
    const auto zero_plant = [](double, std::size_t) { return 0.0; };
    EXPECT_EQ(ft::run(zero_plant, w0, c).outcome, ft::Outcome::refused);
}

TEST(FpvTrim, ThePlantTableEverySettledRowIsInsideItsBoundAndEveryOtherRowDecays) {
    struct Row {
        double lag;
        int delay;
        std::size_t bound_steps;  // 0: not expected to settle by the cap; it must decay
    };
    // the worst of the two starts, measured with the defaults (516-step windows, floor 12 windows, cap 16,000 steps = 31 windows = 15,996 steps)
    const Row rows[] = {
        {430.0, 0, 6192},   {430.0, 500, 6192},   {430.0, 1000, 9288}, {430.0, 2000, 0},
        {2000.0, 0, 11352}, {2000.0, 500, 12900}, {2000.0, 1000, 0},   {2000.0, 2000, 0},
    };
    const ft::Config cfg;
    for (const Row& row : rows) {
        for (const double start : {0.93, 1.09}) {
            FakePlant plant(row.lag, row.delay, start, cfg.window_steps);
            const ft::Result r = ft::run(plant, start, cfg);
            const std::string label = "lag " + std::to_string(static_cast<int>(row.lag)) + " delay " + std::to_string(row.delay) + " start " +
                                      std::to_string(start);
            ASSERT_FALSE(r.windows.empty()) << label;
            if (row.bound_steps != 0) {
                ASSERT_EQ(r.outcome, ft::Outcome::settled) << label;
                EXPECT_LE(r.steps, row.bound_steps) << label;
                EXPECT_GE(r.steps, 12u * 516u) << label << ": never before the floor";
                EXPECT_NEAR(r.omega, 1.0, 0.015) << label << ": the speed that carries the weight is 1 in these units";
                EXPECT_NEAR(r.windows.back().share, 1.0, 0.01) << label;
            } else {
                ASSERT_EQ(r.outcome, ft::Outcome::unsettled) << label << ": a row meant to hit the cap";
                EXPECT_TRUE(std::isnan(r.omega)) << label << ": an unsettled speed is never reported";
                EXPECT_LE(r.steps, cfg.cap_steps) << label;
                EXPECT_GT(r.steps, cfg.cap_steps - cfg.window_steps) << label << ": it ran to the last whole window";
                // decaying, not growing: the last quarter's peak of |share - 1| is under 0.7 of the first window's deviation (measured 0.19 to 0.61, the slowest being lag 2,000 with 2,000 steps of delay from 0.93)
                const std::size_t n = r.windows.size();
                double late = 0.0;
                for (std::size_t i = n - n / 4; i < n; ++i) {
                    late = std::fmax(late, std::fabs(r.windows[i].share - 1.0));
                }
                EXPECT_LT(late, 0.7 * std::fabs(r.windows.front().share - 1.0)) << label;
            }
        }
    }
}

TEST(FpvTrim, AGainAboveWhatTheTableChoseRingsAndStopsSettlingOnTheSameDeadTimeRows) {
    // the reason for 0.25, kept as a test: on the short lag a gain of 0.4 crosses the weight more often with 500 steps of delay than 0.25 does, and
    // does not settle with 1,000 steps where 0.25 does
    int crossings_025 = 0;
    int crossings_040 = 0;
    ft::Outcome outcome_040_1000 = ft::Outcome::settled;
    ft::Outcome outcome_025_1000 = ft::Outcome::unsettled;
    for (const double gain : {0.25, 0.4}) {
        for (const int delay : {500, 1000}) {
            ft::Config cfg;
            cfg.gain = gain;
            FakePlant plant(430.0, delay, 1.09, cfg.window_steps);
            const ft::Result r = ft::run(plant, 1.09, cfg);
            int crossings = 0;
            int prev = 0;
            for (const ft::Window& w : r.windows) {
                const int sg = (w.share > 1.0) - (w.share < 1.0);
                if (prev != 0 && sg != 0 && sg != prev) {
                    ++crossings;
                }
                if (sg != 0) {
                    prev = sg;
                }
            }
            if (delay == 500) {
                (gain == 0.25 ? crossings_025 : crossings_040) = crossings;
            } else {
                (gain == 0.25 ? outcome_025_1000 : outcome_040_1000) = r.outcome;
            }
        }
    }
    EXPECT_LT(crossings_025, crossings_040);
    EXPECT_EQ(outcome_025_1000, ft::Outcome::settled);
    EXPECT_EQ(outcome_040_1000, ft::Outcome::unsettled);
}
