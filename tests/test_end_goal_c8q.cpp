// C8q (docs/design/test-docs/2026-10-19-end-goal-checks.md §5, "C8's band
// where it holds"): the library's reference quad held level in still air at
// C8's R/dx 10, its parts cleared, each rotor's downwash over a 0.5R disc 2R
// below against momentum theory. The window is past the wake's horizon, so
// the verdict is a four-run one-ulp ensemble's: run 0 as it is, three with
// 1 ulp up on 110 of the discs' increments, through LiveRun's test nudge.
//
// - The nudge (LiveRun::set_test_increment_nudge): unset, a run is the same
//   to the bit; it moves a run; it is never saved. In the gate.
// - The ensemble judge (end_goal::c8q()) on made-up means. In the gate.
// - SPADE_C8Q_TIME=1: start and two steps timed at 128^3, to size the run.
// - SPADE_C8Q=1: the four runs and the record. Release, in a quiet window.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "core/flow_volume.hpp"
#include "scene/model_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_type.hpp"
#include "../sandbox/airflow_live.hpp"
#include "../sandbox/end_goal_checks.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::sandbox::airflow;
namespace eg = spade::sandbox::end_goal;
namespace fs = std::filesystem;

[[nodiscard]] bool env_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// "CPU reference", the compiler and the preset, as the V1 records name them.
[[nodiscard]] std::string path_line() {
#if defined(NDEBUG)
    const char* preset = "release";
#else
    const char* preset = "debug";
#endif
#if defined(_MSC_VER)
    return "CPU reference, MSVC " + std::to_string(_MSC_VER) + ", " + preset;
#else
    return std::string("CPU reference, ") + __VERSION__ + ", " + preset;
#endif
}

// splitmix64, as B4's and C8's: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
};

// ---------------------------------------------------------------------------
// The nudge
// ---------------------------------------------------------------------------

inline constexpr std::size_t kC8qNudged = 110;

// C8's nudge, through LiveRun's hook: kC8qNudged picks, with replacement,
// from the faces the discs touch on the first step that touches any, each
// put 1 ulp up on every step. A pick whose increment is 0 that step is left
// alone: an ulp up from 0 is a subnormal, which a flush-to-zero mode would
// erase (Interface).
struct Nudge {
    uint64_t seed = 0;
    std::vector<std::size_t> picks;  // into du, then dv, then dw
    uint64_t moved = 0;              // increments moved, over every step
};

void nudge_increments(void* context, std::span<float> du, std::span<float> dv, std::span<float> dw) {
    Nudge& n = *static_cast<Nudge*>(context);
    const auto at = [&](std::size_t i) -> float& {
        if (i < du.size()) return du[i];
        if (i < du.size() + dv.size()) return dv[i - du.size()];
        return dw[i - du.size() - dv.size()];
    };
    if (n.picks.empty()) {
        std::vector<std::size_t> touched;
        const std::size_t all = du.size() + dv.size() + dw.size();
        for (std::size_t i = 0; i < all; ++i) {
            if (at(i) != 0.0f) touched.push_back(i);
        }
        if (touched.empty()) return;
        SplitMix rng{n.seed};
        for (std::size_t m = 0; m < kC8qNudged; ++m) n.picks.push_back(touched[rng.next() % touched.size()]);
    }
    for (const std::size_t i : n.picks) {
        float& x = at(i);
        if (x == 0.0f) continue;
        x = std::nextafter(x, std::numeric_limits<float>::infinity());
        ++n.moved;
    }
}

// A nudge that changes nothing: it counts its calls and the nonzero increments.
struct Count {
    uint64_t calls = 0;
    uint64_t touched = 0;
};

void count_increments(void* context, std::span<float> du, std::span<float> dv, std::span<float> dw) {
    Count& c = *static_cast<Count*>(context);
    ++c.calls;
    for (const std::span<float> s : {du, dv, dw}) {
        for (const float x : s) c.touched += x != 0.0f ? 1u : 0u;
    }
}

// LiveRun's own tests' tiny case: the powered quad, parts cleared, in an 8^3
// box of 0.8 m, stepped every 4 ms.
[[nodiscard]] an::AnalysisCase tiny_case(const an::test::PoweredQuad& p) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", p.model);
    c.rotors.assign(p.model.rotors.size(), an::RotorSetup{});
    c.grid.box = 0.8f;
    c.grid.spacing = 0.1f;
    c.time.duration = 1.0f;
    c.time.step = 0.004f;
    return an::test::with_power(c, p);
}

[[nodiscard]] an::ResolvedCase resolved(const an::AnalysisCase& c, const spade::vehicles::ModelType& m) {
    const auto r = an::resolve(c, m);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

[[nodiscard]] an::LiveRun started(const an::ResolvedCase& r, const spade::vehicles::ModelType& m) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(r, m);
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    return std::move(*run);
}

[[nodiscard]] std::vector<char> bytes_of(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// The run's whole state, as its save holds it (Physics' grid among it): the
// published volume is fp16, too coarse to show an ulp.
[[nodiscard]] std::vector<char> state_of(const an::LiveRun& run, const fs::path& file) {
    EXPECT_TRUE(run.save(file).has_value());
    return bytes_of(file);
}

const an::Controls kHalf{true, 0.5f, 0.0f, 0.0f, 0.0f};

void advance(an::LiveRun& run, int frames) {
    spade::FlowVolume volume;
    for (int f = 0; f < frames; ++f) ASSERT_TRUE(run.advance_frame(kHalf, volume).has_value());
}

TEST(EndGoalC8qNudge, UnsetAndANoOpNudgeGiveTheSameRunToTheBit) {
    const an::test::PoweredQuad p = an::test::powered_quad();
    const an::ResolvedCase r = resolved(tiny_case(p), p.model);
    an::LiveRun a = started(r, p.model);
    an::LiveRun b = started(r, p.model);
    Count count;
    b.set_test_increment_nudge(&count_increments, &count);
    advance(a, 6);
    advance(b, 6);
    EXPECT_EQ(count.calls, b.state().steps) << "the nudge is called once a step";
    EXPECT_GT(count.touched, 0u) << "the discs touch no face";
    const spade::test::TempDir dir("c8q-nudge");
    EXPECT_EQ(state_of(a, dir.path() / "a.live"), state_of(b, dir.path() / "b.live"));
}

TEST(EndGoalC8qNudge, OneUlpOnAFewIncrementsMovesTheRun) {
    const an::test::PoweredQuad p = an::test::powered_quad();
    const an::ResolvedCase r = resolved(tiny_case(p), p.model);
    an::LiveRun a = started(r, p.model);
    an::LiveRun b = started(r, p.model);
    Nudge nudge{0x1C8, {}, 0};
    b.set_test_increment_nudge(&nudge_increments, &nudge);
    advance(a, 6);
    advance(b, 6);
    EXPECT_EQ(nudge.picks.size(), kC8qNudged);
    EXPECT_GT(nudge.moved, 0u);
    const spade::test::TempDir dir("c8q-nudge");
    EXPECT_NE(state_of(a, dir.path() / "a.live"), state_of(b, dir.path() / "b.live"));
}

// Nudged for three frames and saved, then restored: it goes on as the same
// run unset after those frames, to the bit.
TEST(EndGoalC8qNudge, IsNeverSavedSoARestoredRunStartsUnset) {
    const an::test::PoweredQuad p = an::test::powered_quad();
    const an::ResolvedCase r = resolved(tiny_case(p), p.model);
    const spade::test::TempDir dir("c8q-nudge");
    an::LiveRun x = started(r, p.model);
    Nudge nx{0x1C8, {}, 0};
    x.set_test_increment_nudge(&nudge_increments, &nx);
    advance(x, 3);
    ASSERT_TRUE(x.save(dir.path() / "x.live").has_value());
    spade::Result<an::LiveRun> restored = an::LiveRun::restore(dir.path() / "x.live", r, p.model);
    ASSERT_TRUE(restored.has_value()) << restored.error().context;
    advance(*restored, 3);

    an::LiveRun y = started(r, p.model);
    Nudge ny{0x1C8, {}, 0};
    y.set_test_increment_nudge(&nudge_increments, &ny);
    advance(y, 3);
    y.set_test_increment_nudge(nullptr, nullptr);
    advance(y, 3);
    EXPECT_GT(ny.moved, 0u);
    EXPECT_EQ(state_of(*restored, dir.path() / "restored.live"), state_of(y, dir.path() / "y.live"));
}

// ---------------------------------------------------------------------------
// The ensemble judge, on made-up means
// ---------------------------------------------------------------------------

// A run whose rotors read `ratio` times a 10 m/s reference.
[[nodiscard]] eg::DownwashMeans means(const std::vector<double>& ratio) {
    eg::DownwashMeans m;
    for (const double r : ratio) {
        m.u.push_back(10.0 * r);
        m.v.push_back(10.0);
    }
    m.count = 7;
    m.window = "[0.3, 0.5] s";
    m.r_over_dx = 10.0f;
    return m;
}

TEST(EndGoalC8qJudge, PassesFourRunsInsideBothBands) {
    const std::vector<eg::DownwashMeans> runs{means({1.05, 1.06, 1.04, 1.05}), means({1.06, 1.05, 1.05, 1.04}),
                                              means({1.04, 1.05, 1.06, 1.06}), means({1.05, 1.04, 1.05, 1.06})};
    const eg::Verdict v = eg::c8q(runs);
    EXPECT_TRUE(v.ok) << v.detail;
    EXPECT_EQ(v.detail.find("(OUT)"), std::string::npos) << v.detail;
}

// The mean of the four is inside, but one run is out: the worst run decides.
TEST(EndGoalC8qJudge, TheWorstRunDecidesWhenTheMeanIsInside) {
    const std::vector<eg::DownwashMeans> runs{means({1.05, 1.05, 1.05, 1.05}), means({1.05, 1.05, 1.05, 1.05}),
                                              means({1.05, 1.05, 1.05, 1.05}), means({1.12, 1.05, 1.05, 1.05})};
    ASSERT_TRUE(eg::c8q_bands(eg::ensemble_mean(runs).u, eg::ensemble_mean(runs).v).ok);
    const eg::Verdict v = eg::c8q(runs);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_NE(v.detail.find("the worst run by the rotor band, run 3"), std::string::npos) << v.detail;
}

TEST(EndGoalC8qJudge, FailsARotorOutsideTenPercent) {
    const std::vector<eg::DownwashMeans> runs(4, means({0.85, 0.95, 0.95, 0.95}));
    EXPECT_FALSE(eg::c8q(runs).ok);
}

// Each rotor within 10% of momentum theory, but one 13% off the four's mean.
TEST(EndGoalC8qJudge, FailsTheMutualBandInsideTheRotorBand) {
    const std::vector<eg::DownwashMeans> runs(4, means({0.91, 1.09, 1.09, 1.09}));
    const eg::C8qBands b = eg::c8q_bands(runs[0].u, runs[0].v);
    EXPECT_NEAR(b.mutual_off, (10.45 - 9.1) / 10.45, 1e-12);
    EXPECT_FALSE(b.ok);
    EXPECT_FALSE(eg::c8q(runs).ok);
}

// Each worst run is marked OUT by its own band only. C8q's first ensemble
// (2026-10-22) marked run 2 OUT by the mutual band at 0.00885, inside it: the
// mark read the run's verdict on both bands.
TEST(EndGoalC8qJudge, MarksEachWorstRunOutByItsOwnBandOnly) {
    const auto marks = [](const std::vector<eg::DownwashMeans>& runs) {
        const std::string d = eg::c8q(runs).detail;
        const std::size_t rotor = d.find("the worst run by the rotor band");
        const std::size_t mutual = d.find("; by the mutual band");
        const std::size_t end = d.find("; each rotor's spread");
        if (!(rotor < mutual && mutual < end && end != std::string::npos)) {
            ADD_FAILURE() << d;
            return std::pair{false, false};
        }
        return std::pair{d.substr(rotor, mutual - rotor).find("(OUT)") != std::string::npos,
                         d.substr(mutual, end - mutual).find("(OUT)") != std::string::npos};
    };
    // Outside the rotor band, inside the mutual band; then the other way round.
    EXPECT_EQ(marks(std::vector<eg::DownwashMeans>(4, means({0.82, 0.81, 0.82, 0.82}))), std::pair(true, false));
    EXPECT_EQ(marks(std::vector<eg::DownwashMeans>(4, means({0.91, 1.09, 1.09, 1.09}))), std::pair(false, true));
}

TEST(EndGoalC8qJudge, ReportsTheMutualSpreadAndEachRotorsSpreadAcrossTheRuns) {
    const std::vector<eg::DownwashMeans> runs{means({1.0, 1.0, 1.0, 1.0}), means({1.02, 1.0, 1.0, 1.0}),
                                              means({1.0, 1.0, 1.0, 1.0}), means({1.0, 1.0, 1.0, 1.04})};
    const eg::C8qBands b = eg::c8q_bands(runs[3].u, runs[3].v);
    EXPECT_NEAR(b.mutual_spread, 0.4 / 10.1, 1e-12);
    const eg::Verdict v = eg::c8q(runs);
    EXPECT_TRUE(v.ok) << v.detail;
    // Rotor 0: (10.2 - 10) / 10.05; rotor 3: (10.4 - 10) / 10.1.
    EXPECT_NE(v.detail.find("each rotor's spread across the runs: 0.0199 0 0 0.0396"), std::string::npos) << v.detail;
}

TEST(EndGoalC8qJudge, IsAbsentWithoutRunsOrWhenTheRunsDisagreeOnTheRotors) {
    EXPECT_FALSE(eg::c8q({}).ok);
    const std::vector<eg::DownwashMeans> runs{means({1.0, 1.0, 1.0, 1.0}), means({1.0, 1.0, 1.0})};
    const eg::Verdict v = eg::c8q(runs);
    EXPECT_FALSE(v.ok);
    EXPECT_EQ(v.detail, "c8q: absent: the runs disagree on the rotors");
}

TEST(EndGoalC8qJudge, AReferenceOfZeroIsNeverInside) {
    eg::DownwashMeans m = means({1.0, 1.0, 1.0, 1.0});
    m.v[2] = 0.0;
    EXPECT_FALSE(eg::c8q_bands(m.u, m.v).ok);
}

// ---------------------------------------------------------------------------
// C8q's case, timed and run (opt-in)
// ---------------------------------------------------------------------------

inline constexpr float kC8qBoxM = 0.975f;
inline constexpr uint32_t kC8qCells = 128;
inline constexpr uint32_t kC8qFrames = 15;  // to 0.50 s: t_dev 0.30 s plus the 0.20 s window
inline constexpr uint64_t kC8qSeeds[] = {0, 0x1C8, 0x1C9, 0x1CA};

// The library's reference quad, its parts cleared as the editor's airflow
// chapter flies it, held level in still air: a 0.975 m box at 128^3 (R/dx
// 10.0), h = dx/8 as C8's, air's viscosity (Physics, 2026-10-22), no floor,
// armed at the hover throttle from t = 0.
struct C8qCase {
    spade::vehicles::ModelType model;
    an::ResolvedCase resolved;
    an::Controls hover;
    eg::DownwashCase judged;
};

[[nodiscard]] std::optional<C8qCase> c8q_case() {
    const fs::path file = fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
    spade::Result<spade::scene::ModelFile> loaded = spade::scene::load_model_file(file);
    EXPECT_TRUE(loaded.has_value()) << (loaded ? "" : loaded.error().context);
    if (!loaded) return std::nullopt;
    C8qCase q;
    q.model = loaded->model;
    q.model.parts.clear();
    an::AnalysisCase c;
    c.model = an::pin_model(file.generic_string(), q.model);
    c.power = an::Power{.idle = static_cast<float>(af::kIdleDuty),
                        .state_of_charge = static_cast<float>(af::kStateOfCharge)};
    c.rotors.assign(q.model.rotors.size(), an::RotorSetup{});
    c.grid.box = kC8qBoxM;
    c.grid.spacing = kC8qBoxM / static_cast<float>(kC8qCells);
    c.time.step = *c.grid.spacing / 8.0f;
    c.time.duration = static_cast<float>(kC8qFrames) / static_cast<float>(an::kFramesPerSecond);
    const spade::Result<an::ShowcaseTrim> trim = an::showcase_trim(c, q.model);
    EXPECT_TRUE(trim.has_value()) << (trim ? "" : trim.error().context);
    if (!trim) return std::nullopt;
    q.hover = an::Controls{true, trim->hover, 0.0f, 0.0f, 0.0f};
    q.resolved = resolved(c, q.model);
    q.judged = eg::DownwashCase{c.fluid.density, q.model.body.mass, 9.80665f};
    return q;
}

TEST(EndGoalC8q, TimesStartAndTwoStepsAtRDxTen) {
    if (!env_set("SPADE_C8Q_TIME")) GTEST_SKIP() << "set SPADE_C8Q_TIME=1 to time C8q's start and two steps at 128^3";
    const std::optional<C8qCase> q = c8q_case();
    ASSERT_TRUE(q.has_value());
    using clock = std::chrono::steady_clock;
    const auto seconds = [](clock::duration d) { return std::chrono::duration<double>(d).count(); };
    const clock::time_point t0 = clock::now();
    an::LiveRun run = started(q->resolved, q->model);
    const clock::time_point t1 = clock::now();
    ASSERT_TRUE(run.step(q->hover).has_value());
    const clock::time_point t2 = clock::now();
    ASSERT_TRUE(run.step(q->hover).has_value());
    const clock::time_point t3 = clock::now();
    const double h = run.step_s();
    const double steps = std::ceil(static_cast<double>(kC8qFrames) / static_cast<double>(an::kFramesPerSecond) / h);
    std::printf("c8q-time: %s; %s; h %.6f s; start %.1f s; step 1 %.2f s; step 2 %.2f s; %.0f steps to %.2f s, "
                "so one run is about %.0f min on one thread\n",
                path_line().c_str(), run.run_label().c_str(), h, seconds(t1 - t0), seconds(t2 - t1),
                seconds(t3 - t2), steps, static_cast<double>(kC8qFrames) / static_cast<double>(an::kFramesPerSecond),
                steps * seconds(t3 - t2) / 60.0);
}

// One run's frames: the published volume read after every frame, as the
// downwash check reads it, timed from arming (t = 0).
struct C8qRun {
    std::vector<eg::DownwashFrame> frames;
    std::string fault;
    std::string label;  // run_label()
    uint64_t moved = 0;
};

[[nodiscard]] C8qRun c8q_run(const C8qCase& q, uint64_t seed) {
    C8qRun out;
    spade::Result<an::LiveRun> run = an::LiveRun::start(q.resolved, q.model);
    if (!run) {
        out.fault = run.error().context;
        return out;
    }
    Nudge nudge{seed, {}, 0};
    if (seed != 0) run->set_test_increment_nudge(&nudge_increments, &nudge);
    spade::FlowVolume volume;
    for (uint32_t f = 0; f < kC8qFrames; ++f) {
        if (spade::Result<an::LiveFrame> frame = run->advance_frame(q.hover, volume); !frame) {
            out.fault = frame.error().context;
            return out;
        }
        out.frames.push_back(eg::sample_downwash(volume, af::probes_of(*run), run->state().t_s,
                                                 eg::tilt_of(run->body().orient), true));
    }
    out.moved = nudge.moved;
    out.label = run->run_label();
    return out;
}

[[nodiscard]] const char* verdict_word(bool ok) { return ok ? "within" : "MISS"; }

[[nodiscard]] bool in_band(double ratio) {
    return ratio >= static_cast<double>(eg::kC8q.lo) && ratio <= static_cast<double>(eg::kC8q.hi);
}

TEST(EndGoalC8q, ReferenceQuadAgainstMomentumTheory) {
    if (!env_set("SPADE_C8Q")) {
        GTEST_SKIP() << "set SPADE_C8Q=1 to run C8q's four runs at 128^3 (release, a quiet window)";
    }
    const std::optional<C8qCase> q = c8q_case();
    ASSERT_TRUE(q.has_value());
    constexpr std::size_t kRuns = std::size(kC8qSeeds);
    std::vector<C8qRun> runs(kRuns);
    {
        // Each run is single-threaded, so its bits are the same however many run beside it.
        std::vector<std::thread> pool;
        for (std::size_t m = 0; m < kRuns; ++m) pool.emplace_back([&, m] { runs[m] = c8q_run(*q, kC8qSeeds[m]); });
        for (std::thread& t : pool) t.join();
    }
    std::vector<eg::DownwashMeans> means;
    for (std::size_t m = 0; m < kRuns; ++m) {
        ASSERT_TRUE(runs[m].fault.empty()) << "run " << m << ": " << runs[m].fault;
        if (kC8qSeeds[m] != 0) {
            EXPECT_GT(runs[m].moved, 0u) << "run " << m << "'s nudge moved nothing";
        }
        const auto judged = eg::downwash_means(runs[m].frames, q->judged, eg::kC8q);
        ASSERT_TRUE(judged.has_value()) << "run " << m << ": " << judged.error().detail;
        means.push_back(*judged);
        std::printf("c8q-run: run %zu seed 0x%llX:", m, static_cast<unsigned long long>(kC8qSeeds[m]));
        for (std::size_t i = 0; i < judged->u.size(); ++i) {
            std::printf(" rotor %zu %.4f / %.4f m/s;", i, judged->u[i], judged->v[i]);
        }
        std::printf("\n");
    }
    const eg::Verdict v = eg::c8q(means);
    std::printf("c8q-verdict: %s %s\n", verdict_word(v.ok), v.detail.c_str());

    // The record's numbers: the bands on the runs' mean and on the worst run.
    const eg::DownwashMeans mean = eg::ensemble_mean(means);
    const eg::C8qBands all = eg::c8q_bands(mean.u, mean.v);
    std::vector<eg::C8qBands> each;
    std::size_t worst_ratio = 0;
    std::size_t worst_mutual = 0;
    for (std::size_t m = 0; m < kRuns; ++m) {
        each.push_back(eg::c8q_bands(means[m].u, means[m].v));
        if (each[m].ratio_off > each[worst_ratio].ratio_off) worst_ratio = m;
        if (each[m].mutual_off > each[worst_mutual].mutual_off) worst_mutual = m;
    }
    const auto ratio_ok = [](const eg::C8qBands& b) {
        bool ok = true;
        for (const double r : b.ratio) ok = ok && in_band(r);
        return ok;
    };
    std::vector<double> spreads;
    for (std::size_t i = 0; i < mean.u.size(); ++i) {
        double lo = means[0].u[i];
        double hi = means[0].u[i];
        for (const eg::DownwashMeans& m : means) {
            lo = std::min(lo, m.u[i]);
            hi = std::max(hi, m.u[i]);
        }
        spreads.push_back((hi - lo) / mean.u[i]);
    }
    const float dx = q->resolved.base.dx;
    const double r_over_dx = static_cast<double>(mean.r_over_dx);
    std::printf("=== validation record: C8q reference quad ===\n"
                "path: %s\n"
                "config: the library's reference quad (assets/library/drones/reference_quad.model.yaml), its parts "
                "cleared as the editor's airflow chapter flies it, held level at the box's centre in still air, no "
                "floor; %s; a %.3f m box, dx %.2f mm, R/dx %.2f, h = dx/8 = %.6f s, air's kinematic viscosity "
                "%.4g m2/s (C8 ran inviscid: the difference was ruled immaterial before the run, Physics 2026-10-22: "
                "viscous spreading about 0.05R by 0.5 s, the probe disc's edge about 0.23R inside the slipstream); "
                "armed at the hover throttle %.4f from t = 0; the window %s, %zu frames a run. Four runs that differ "
                "only by a nudge in the test, 1 ulp up on %zu of the discs' increments, seed s: run 0 as it is, runs "
                "1 to 3 seeds 0x1C8, 0x1C9, 0x1CA. So the spread is the measure's sensitivity to rounding, not a "
                "physical uncertainty\n",
                path_line().c_str(), runs[0].label.c_str(), static_cast<double>(kC8qBoxM),
                1000.0 * static_cast<double>(dx), r_over_dx,
                static_cast<double>(q->resolved.spec.time.step.value_or(0.0f)),
                static_cast<double>(q->resolved.spec.fluid.kinematic_viscosity),
                static_cast<double>(q->hover.throttle), mean.window.c_str(), means[0].count, kC8qNudged);
    for (std::size_t i = 0; i < mean.u.size(); ++i) {
        const bool in = in_band(all.ratio[i]);
        std::printf("measure: rotor %zu's mean downwash over the 0.5R disc 2R below, the four runs' mean = %.4f m/s\n"
                    "reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, the four runs' mean = "
                    "%.4f m/s (tests/data/airflow/README.md, C8 and C8q)\n"
                    "tolerance: within 10%% (C8's, plan A Task 11; C8q's bands and its four runs fixed before the "
                    "first run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)\n"
                    "verdict: %s\n",
                    i, mean.u[i], mean.v[i], verdict_word(in));
    }
    std::printf("measure: the four rotors' means, the four runs' mean, off their mean by at most = %.4f\n"
                "reference: the four rotors are equal statistically (quarter-turn symmetry; Physics, 2026-10-19)\n"
                "tolerance: <= 0.1 (as above)\n"
                "verdict: %s\n"
                "measure: the worst run by the rotor band, run %zu, off momentum theory by at most = %.4f\n"
                "reference: momentum theory, as above\n"
                "tolerance: <= 0.1 (as above: the verdict holds across the spread)\n"
                "verdict: %s\n"
                "measure: the worst run by the mutual band, run %zu, the four off their mean by at most = %.4f\n"
                "reference: the four rotors are equal statistically, as above\n"
                "tolerance: <= 0.1 (as above: the verdict holds across the spread)\n"
                "verdict: %s\n"
                "measure: the mutual spread, (max - min) / mean of the four rotors' means, the four runs' mean = "
                "%.4f\n"
                "reference: none (evidence for any later tightening, set in advance)\n"
                "tolerance: none\n"
                "verdict: report only\n",
                all.mutual_off, verdict_word(all.mutual_off <= eg::kC8qMutual), worst_ratio,
                each[worst_ratio].ratio_off, verdict_word(ratio_ok(each[worst_ratio])), worst_mutual,
                each[worst_mutual].mutual_off, verdict_word(each[worst_mutual].mutual_off <= eg::kC8qMutual),
                all.mutual_spread);
    // One measure a rotor: a measure's value is a single number.
    for (std::size_t i = 0; i < spreads.size(); ++i) {
        std::printf("measure: rotor %zu's spread across the four runs, (max - min) / mean = %.4f\n"
                    "reference: none (the measure's sensitivity to rounding)\n"
                    "tolerance: none\n"
                    "verdict: report only\n",
                    i, spreads[i]);
    }
    EXPECT_TRUE(v.ok) << v.detail;
}

}  // namespace
