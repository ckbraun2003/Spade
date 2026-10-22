// C8q (docs/design/test-docs/2026-10-19-end-goal-checks.md §5, "C8's band
// where it holds"): the library's reference quad held level in still air at
// C8's R/dx 10, its parts cleared, a rotor's downwash over a 0.5R disc 2R
// below against momentum theory. As changed on 2026-10-22, before its banded
// run: rotor 0 alone in the quad's case is banded by C8's 10%, and the
// four-rotor quad by its rotors' symmetry, its ratios reported only. The
// window is past the wake's horizon, so each is a four-run one-ulp ensemble's:
// run 0 as it is, three with 1 ulp up on 110 of the discs' increments, through
// LiveRun's test nudge.
//
// - The nudge (LiveRun::set_test_increment_nudge): unset, a run is the same
//   to the bit; it moves a run; it is never saved. In the gate.
// - The judge (end_goal::c8q()) on made-up means, and the record's lines
//   through the records' reader. In the gate.
// - SPADE_C8Q_TIME=1: start and two steps timed at 128^3, to size the run.
// - SPADE_C8Q=1: the two ensembles and the record. Release, in a quiet window.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
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
#include "analysis/validation.hpp"
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

// Rotor 0 alone's runs: one rotor each, at `ratio` of a 10 m/s reference.
[[nodiscard]] std::vector<eg::DownwashMeans> alone(const std::vector<double>& ratios) {
    std::vector<eg::DownwashMeans> runs;
    for (const double r : ratios) runs.push_back(means({r}));
    return runs;
}

// The quad as its first ensemble read it (2026-10-22): every rotor about 0.816
// of momentum theory, within 0.1% of the four's mean.
[[nodiscard]] std::vector<eg::DownwashMeans> quad_as_read() {
    return std::vector<eg::DownwashMeans>(4, means({0.8167, 0.8156, 0.8161, 0.8170}));
}

// Rotor 0 alone's part of a verdict's detail, and the quad's.
[[nodiscard]] std::pair<std::string, std::string> parts(const std::string& d) {
    const std::size_t quad = d.find(". The quad");
    if (quad == std::string::npos) {
        ADD_FAILURE() << d;
        return {};
    }
    return {d.substr(0, quad), d.substr(quad)};
}

TEST(EndGoalC8qJudge, PassesRotorZeroAloneInsideTheBandWithTheQuadsSymmetry) {
    const eg::Verdict v = eg::c8q(alone({1.05, 1.06, 1.04, 1.05}), quad_as_read());
    EXPECT_TRUE(v.ok) << v.detail;
    EXPECT_EQ(v.detail.find("(OUT)"), std::string::npos) << v.detail;
}

// At 0.816 of momentum theory, the first ensemble's MISS, the quad now passes
// on its symmetry: its ratios are reported, never judged (§5, 2026-10-22).
TEST(EndGoalC8qJudge, ReportsTheQuadsRatiosWithoutJudgingThem) {
    const eg::Verdict v = eg::c8q(alone({1.05, 1.05, 1.05, 1.05}), quad_as_read());
    EXPECT_TRUE(v.ok) << v.detail;
    const auto [rotor0, quad] = parts(v.detail);
    EXPECT_NE(quad.find("report only: rotor 0 "), std::string::npos) << v.detail;
    EXPECT_NE(quad.find("rotor 3 "), std::string::npos) << v.detail;
    EXPECT_EQ(quad.find("(OUT)"), std::string::npos) << v.detail;
}

// The mean of rotor 0's four runs is inside, but one run is out.
TEST(EndGoalC8qJudge, TheWorstRotorZeroRunDecidesWhenTheMeanIsInside) {
    const std::vector<eg::DownwashMeans> runs = alone({1.05, 1.05, 1.05, 1.12});
    ASSERT_TRUE(eg::c8q_bands(eg::ensemble_mean(runs).u, eg::ensemble_mean(runs).v).ratio_ok);
    const eg::Verdict v = eg::c8q(runs, quad_as_read());
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_NE(v.detail.find("the worst run, run 3"), std::string::npos) << v.detail;
}

TEST(EndGoalC8qJudge, FailsRotorZeroOutsideTenPercent) {
    EXPECT_FALSE(eg::c8q(alone({0.85, 0.85, 0.85, 0.85}), quad_as_read()).ok);
}

// Each quad rotor within 10% of momentum theory, but one 13% off the four's mean.
TEST(EndGoalC8qJudge, FailsTheQuadsSymmetryBand) {
    const std::vector<eg::DownwashMeans> quad(4, means({0.91, 1.09, 1.09, 1.09}));
    const eg::C8qBands b = eg::c8q_bands(quad[0].u, quad[0].v);
    EXPECT_NEAR(b.mutual_off, (10.45 - 9.1) / 10.45, 1e-12);
    EXPECT_FALSE(b.mutual_ok);
    EXPECT_FALSE(eg::c8q(alone({1.05, 1.05, 1.05, 1.05}), quad).ok);
}

// The quad's four runs' mean is inside the symmetry band, but run 3 is 13% out.
TEST(EndGoalC8qJudge, TheWorstQuadRunDecidesTheSymmetryBand) {
    std::vector<eg::DownwashMeans> quad(3, means({1.0, 1.0, 1.0, 1.0}));
    quad.push_back(means({0.90, 1.08, 1.08, 1.08}));
    ASSERT_TRUE(eg::c8q_bands(eg::ensemble_mean(quad).u, eg::ensemble_mean(quad).v).mutual_ok);
    const eg::Verdict v = eg::c8q(alone({1.05, 1.05, 1.05, 1.05}), quad);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_NE(v.detail.find("the worst run by the mutual band, run 3"), std::string::npos) << v.detail;
}

// Each part is marked OUT by its own band only.
TEST(EndGoalC8qJudge, MarksEachPartOutByItsOwnBandOnly) {
    const eg::Verdict rotor0_out = eg::c8q(alone({0.85, 0.85, 0.85, 0.85}), quad_as_read());
    const auto [a0, aq] = parts(rotor0_out.detail);
    EXPECT_NE(a0.find("(OUT)"), std::string::npos) << rotor0_out.detail;
    EXPECT_EQ(aq.find("(OUT)"), std::string::npos) << rotor0_out.detail;
    const eg::Verdict quad_out =
        eg::c8q(alone({1.05, 1.05, 1.05, 1.05}), std::vector<eg::DownwashMeans>(4, means({0.91, 1.09, 1.09, 1.09})));
    const auto [b0, bq] = parts(quad_out.detail);
    EXPECT_EQ(b0.find("(OUT)"), std::string::npos) << quad_out.detail;
    EXPECT_NE(bq.find("(OUT)"), std::string::npos) << quad_out.detail;
}

TEST(EndGoalC8qJudge, ReportsTheQuadsSpreadAndEachRotorsSpreadAcrossTheRuns) {
    std::vector<eg::DownwashMeans> quad(3, means({1.0, 1.0, 1.0, 1.0}));
    quad.push_back(means({1.0, 1.0, 1.0, 1.04}));
    const eg::Verdict v = eg::c8q(alone({1.0, 1.02, 1.0, 1.0}), quad);
    EXPECT_TRUE(v.ok) << v.detail;
    // Rotor 0 alone: (10.2 - 10) / 10.05; the quad's rotor 3: (10.4 - 10) / 10.1.
    EXPECT_NE(v.detail.find("its spread across the runs 0.0199"), std::string::npos) << v.detail;
    EXPECT_NE(v.detail.find("each rotor's spread across the runs: 0 0 0 0.0396"), std::string::npos) << v.detail;
}

TEST(EndGoalC8qJudge, IsAbsentWithoutEitherEnsembleOrWithTheWrongRotors) {
    const std::vector<eg::DownwashMeans> ok0 = alone({1.0, 1.0, 1.0, 1.0});
    const std::vector<eg::DownwashMeans> disagree{means({1.0, 1.0, 1.0, 1.0}), means({1.0, 1.0, 1.0})};
    const std::pair<eg::Verdict, std::string> cases[] = {
        {eg::c8q({}, quad_as_read()), "c8q: absent: no runs of rotor 0 alone"},
        {eg::c8q(ok0, {}), "c8q: absent: no runs of the quad"},
        {eg::c8q(quad_as_read(), quad_as_read()), "c8q: absent: a run of rotor 0 alone holds 4 rotors"},
        {eg::c8q(ok0, disagree), "c8q: absent: the quad's runs disagree on the rotors"},
        {eg::c8q(ok0, ok0), "c8q: absent: the quad's runs disagree on the rotors"},
    };
    for (const auto& [v, detail] : cases) {
        EXPECT_FALSE(v.ok) << v.detail;
        EXPECT_EQ(v.detail, detail);
    }
}

TEST(EndGoalC8qJudge, AReferenceOfZeroIsNeverInside) {
    eg::DownwashMeans m = means({1.0, 1.0, 1.0, 1.0});
    m.v[2] = 0.0;
    EXPECT_FALSE(eg::c8q_bands(m.u, m.v).ok);
    eg::DownwashMeans m0 = means({1.0});
    m0.v[0] = 0.0;
    EXPECT_FALSE(eg::c8q(std::vector<eg::DownwashMeans>(4, m0), quad_as_read()).ok);
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
//
// Its variants:
// - rotor_0_alone, C8q's banded measure (§5, 2026-10-22): rotors 1 to 3 push
//   nothing and spread no swirl. Their thrust coefficient is the least normal
//   float, about 1e-32 N at hover, since a model refuses 0, and their torque
//   coefficient is 0. Rotor 0 keeps its own torque, as it flies. The hover
//   throttle is trimmed on the four-rotor model first, so rotor 0's speed, and
//   with it its reference, are the quad's. Only rotor 0 is judged, against a
//   quarter of the weight, so the loaded premise reads its share as it reads
//   the quad's.
// Physics' discriminating runs for the first ensemble's miss, one run each:
// - no_torque: every rotor's torque coefficient is 0, so the discs spread no
//   swirl;
// - rotor_0_alone_no_torque: rotor 0 alone, as above, with its torque at 0 too.
enum class C8qVariant { as_flown, rotor_0_alone, no_torque, rotor_0_alone_no_torque };

struct C8qCase {
    spade::vehicles::ModelType model;
    an::ResolvedCase resolved;
    an::Controls hover;
    eg::DownwashCase judged;
    bool rotor_0_only = false;  // judge rotor 0's probe alone
};

[[nodiscard]] std::optional<C8qCase> c8q_case(C8qVariant variant = C8qVariant::as_flown) {
    const fs::path file = fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
    spade::Result<spade::scene::ModelFile> loaded = spade::scene::load_model_file(file);
    EXPECT_TRUE(loaded.has_value()) << (loaded ? "" : loaded.error().context);
    if (!loaded) return std::nullopt;
    C8qCase q;
    q.model = loaded->model;
    q.model.parts.clear();
    if (variant == C8qVariant::no_torque || variant == C8qVariant::rotor_0_alone_no_torque) {
        for (auto& rotor : q.model.rotors) rotor.torque_coeff = 0.0f;
    }
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
    float judged_mass = q.model.body.mass;
    if (variant == C8qVariant::rotor_0_alone || variant == C8qVariant::rotor_0_alone_no_torque) {
        for (std::size_t i = 1; i < q.model.rotors.size(); ++i) {
            q.model.rotors[i].thrust_coeff = std::numeric_limits<float>::min();
            q.model.rotors[i].torque_coeff = 0.0f;
        }
        c.model = an::pin_model(file.generic_string(), q.model);
        judged_mass /= static_cast<float>(q.model.rotors.size());
        q.rotor_0_only = true;
    }
    q.resolved = resolved(c, q.model);
    q.judged = eg::DownwashCase{c.fluid.density, judged_mass, 9.80665f};
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
        std::vector<eg::RotorProbe> probes = af::probes_of(*run);
        if (q.rotor_0_only) probes.resize(1);
        out.frames.push_back(
            eg::sample_downwash(volume, probes, run->state().t_s, eg::tilt_of(run->body().orient), true));
    }
    out.moved = nudge.moved;
    out.label = run->run_label();
    return out;
}

[[nodiscard]] const char* verdict_word(bool ok) { return ok ? "within" : "MISS"; }

[[nodiscard]] std::string printed(const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list size_args;
    va_copy(size_args, args);
    const int n = std::vsnprintf(nullptr, 0, format, size_args);
    va_end(size_args);
    std::string s(static_cast<std::size_t>(std::max(n, 0)), '\0');
    std::vsnprintf(s.data(), s.size() + 1, format, args);
    va_end(args);
    return s;
}

// What C8q's record says of its case, besides the two ensembles' means.
struct C8qRecordCase {
    std::string path;          // path_line()
    std::string rotor0_label;  // run_label() of rotor 0 alone's run 0
    std::string quad_label;    // and of the quad's
    double dx_mm = 0.0;
    double r_over_dx = 0.0;
    double h_s = 0.0;
    double nu = 0.0;
    double throttle = 0.0;
    std::string window;
    std::size_t frames = 0;
};

// Rotor i's (max - min) / mean of its u across the runs.
[[nodiscard]] double spread_of(std::span<const eg::DownwashMeans> runs, std::size_t i, double mean) {
    double lo = runs[0].u[i];
    double hi = runs[0].u[i];
    for (const eg::DownwashMeans& m : runs) {
        lo = std::min(lo, m.u[i]);
        hi = std::max(hi, m.u[i]);
    }
    return (hi - lo) / mean;
}

// C8q's record, as its run prints it: the opening line, then path, config and
// the measures, one key a line, for the paste step to add its commit and date
// after the opening (01-verification, "Validation records"). The banded
// measures are the judge's (end_goal::c8q()); the quad's ratios and every
// spread are report only. Precondition: both ensembles nonempty, rotor 0's
// runs one rotor each and the quad's agreeing on theirs, as c8q() checks.
[[nodiscard]] std::string c8q_record(const C8qRecordCase& c, std::span<const eg::DownwashMeans> rotor0,
                                     std::span<const eg::DownwashMeans> quad) {
    const eg::DownwashMeans m0 = eg::ensemble_mean(rotor0);
    const eg::C8qBands b0 = eg::c8q_bands(m0.u, m0.v);
    std::size_t worst0 = 0;
    std::vector<eg::C8qBands> each0;
    for (std::size_t k = 0; k < rotor0.size(); ++k) {
        each0.push_back(eg::c8q_bands(rotor0[k].u, rotor0[k].v));
        if (each0[k].ratio_off > each0[worst0].ratio_off) worst0 = k;
    }
    const eg::DownwashMeans mq = eg::ensemble_mean(quad);
    const eg::C8qBands bq = eg::c8q_bands(mq.u, mq.v);
    std::size_t worstq = 0;
    std::vector<eg::C8qBands> eachq;
    for (std::size_t k = 0; k < quad.size(); ++k) {
        eachq.push_back(eg::c8q_bands(quad[k].u, quad[k].v));
        if (eachq[k].mutual_off > eachq[worstq].mutual_off) worstq = k;
    }

    std::string r = "=== validation record: C8q reference quad ===\n";
    r += "path: " + c.path + "\n";
    r += printed(
        "config: the library's reference quad (assets/library/drones/reference_quad.model.yaml), its parts cleared as "
        "the editor's airflow chapter flies it, held level at the box's centre in still air, no floor; a %.3f m box, "
        "dx %.2f mm, R/dx %.2f, h = dx/8 = %.6f s, air's kinematic viscosity %.4g m2/s (C8 ran inviscid: the "
        "difference was ruled immaterial before the run, Physics 2026-10-22: viscous spreading about 0.05R by 0.5 s, "
        "the probe disc's edge about 0.23R inside the slipstream); armed at the hover throttle %.4f from t = 0, "
        "trimmed on the four-rotor model; the window %s, %zu frames a run. Two ensembles, as C8q's bands were changed "
        "on 2026-10-22 before this run (docs/design/test-docs/2026-10-19-end-goal-checks.md section 5): rotor 0 "
        "alone, %s, with its own torque, rotors 1 to 3 pushing nothing (thrust coefficient the least normal float, "
        "torque 0); and the quad as flown, %s. Each is four runs that differ only by a nudge in the test, 1 ulp up "
        "on %zu of the discs' increments, seed s: run 0 as it is, runs 1 to 3 seeds 0x1C8, 0x1C9, 0x1CA. So each "
        "spread is the measure's sensitivity to rounding, not a physical uncertainty\n",
        static_cast<double>(kC8qBoxM), c.dx_mm, c.r_over_dx, c.h_s, c.nu, c.throttle, c.window.c_str(), c.frames,
        c.rotor0_label.c_str(), c.quad_label.c_str(), kC8qNudged);
    r += printed(
        "measure: rotor 0 alone's mean downwash over the 0.5R disc 2R below, its four runs' mean = %.4f m/s\n"
        "reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, its four runs' mean = %.4f m/s "
        "(tests/data/airflow/README.md, C8 and C8q)\n"
        "tolerance: within 10%% (C8's, plan A Task 11; C8q's bands as changed on 2026-10-22, fixed before this run, "
        "docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)\n"
        "verdict: %s\n"
        "measure: rotor 0 alone's worst run, run %zu, off momentum theory by = %.4f\n"
        "reference: momentum theory, as above\n"
        "tolerance: <= 0.1 (as above: the verdict holds across the spread)\n"
        "verdict: %s\n"
        "measure: rotor 0 alone's spread across its four runs, (max - min) / mean = %.4f\n"
        "reference: none (the measure's sensitivity to rounding)\n"
        "tolerance: none\n"
        "verdict: report only\n",
        m0.u[0], m0.v[0], verdict_word(b0.ratio_ok), worst0, each0[worst0].ratio_off,
        verdict_word(each0[worst0].ratio_ok), spread_of(rotor0, 0, m0.u[0]));
    // One measure a rotor: a measure's value is a single number.
    for (std::size_t i = 0; i < mq.u.size(); ++i) {
        r += printed(
            "measure: the quad's rotor %zu, its downwash over momentum theory's, its four runs' mean = %.4f\n"
            "reference: none (the four rotors together: their wakes' interaction and the open box's response to four "
            "times the flux, not separated; no closed-form reference, Physics 2026-10-22)\n"
            "tolerance: none\n"
            "verdict: report only\n",
            i, bq.ratio[i]);
    }
    r += printed(
        "measure: the quad's rotors' means, its four runs' mean, off their mean by at most = %.4f\n"
        "reference: the four rotors are equal statistically (quarter-turn symmetry; Physics, 2026-10-19)\n"
        "tolerance: <= 0.1 (C8q's mutual band, fixed before the first run, as above)\n"
        "verdict: %s\n"
        "measure: the quad's worst run by the mutual band, run %zu, its rotors off their mean by at most = %.4f\n"
        "reference: the four rotors are equal statistically, as above\n"
        "tolerance: <= 0.1 (as above: the verdict holds across the spread)\n"
        "verdict: %s\n"
        "measure: the quad's mutual spread, (max - min) / mean of its rotors' means, its four runs' mean = %.4f\n"
        "reference: none (evidence for any later tightening, set in advance)\n"
        "tolerance: none\n"
        "verdict: report only\n",
        bq.mutual_off, verdict_word(bq.mutual_ok), worstq, eachq[worstq].mutual_off,
        verdict_word(eachq[worstq].mutual_ok), bq.mutual_spread);
    for (std::size_t i = 0; i < mq.u.size(); ++i) {
        r += printed(
            "measure: the quad's rotor %zu's spread across its four runs, (max - min) / mean = %.4f\n"
            "reference: none (the measure's sensitivity to rounding)\n"
            "tolerance: none\n"
            "verdict: report only\n",
            i, spread_of(quad, i, mq.u[i]));
    }
    return r;
}

// The record's lines read as one validation record, with the paste step's
// commit and date, before the run they wait for: each banded measure judged as
// the judge judges it, every other report only.
TEST(EndGoalC8qJudge, TheRecordReadsAsAValidationRecord) {
    const C8qRecordCase c{"CPU reference, a test", "run A", "run B", 7.62, 10.0, 0.000952, 1.46e-5, 0.4321,
                          "[0.3, 0.5] s", 7};
    const auto read = [&](std::span<const eg::DownwashMeans> rotor0, std::span<const eg::DownwashMeans> quad) {
        const std::string block = c8q_record(c, rotor0, quad);
        const std::size_t first = block.find('\n') + 1;
        const std::string text = "```text\n" + block.substr(0, first) + "commit: 0000000 (a test)\ndate: 2026-10-22\n" +
                                 block.substr(first) + "```\n";
        return an::validation_records(text, "c8q-test.md");
    };
    const auto within = read(alone({1.05, 1.06, 1.04, 1.05}), quad_as_read());
    ASSERT_TRUE(within.has_value()) << within.error().context;
    ASSERT_EQ(within->size(), 1u);
    const std::vector<an::ValidationMeasure>& m = within->at(0).measures;
    ASSERT_EQ(m.size(), 14u);  // rotor 0: 3; the quad: 4 ratios, 3, 4 spreads
    for (std::size_t k = 0; k < m.size(); ++k) {
        const bool banded = k == 0 || k == 1 || k == 7 || k == 8;
        EXPECT_EQ(m[k].verdict, banded ? an::Verdict::within : an::Verdict::report_only) << k << ": " << m[k].quantity;
        EXPECT_EQ(m[k].tolerance.has_value(), banded) << k << ": " << m[k].quantity;
    }
    EXPECT_EQ(m[0].unit, "m/s");
    EXPECT_NEAR(m[0].number, 10.5, 1e-9);
    EXPECT_NEAR(m[3].number, 0.8167, 1e-9);  // the quad's rotor 0, reported

    const auto miss =
        read(alone({0.85, 0.85, 0.85, 0.85}), std::vector<eg::DownwashMeans>(4, means({0.91, 1.09, 1.09, 1.09})));
    ASSERT_TRUE(miss.has_value()) << miss.error().context;
    for (const std::size_t k : {std::size_t{0}, std::size_t{1}, std::size_t{7}, std::size_t{8}}) {
        EXPECT_EQ(miss->at(0).measures.at(k).verdict, an::Verdict::miss) << k;
    }
}

// One ensemble's means: its four runs, side by side. Each run is
// single-threaded, so its bits are the same however many run beside it.
[[nodiscard]] std::vector<eg::DownwashMeans> c8q_ensemble(const C8qCase& q, const char* tag, std::string& label) {
    constexpr std::size_t kRuns = std::size(kC8qSeeds);
    std::vector<C8qRun> runs(kRuns);
    {
        std::vector<std::thread> pool;
        for (std::size_t m = 0; m < kRuns; ++m) pool.emplace_back([&, m] { runs[m] = c8q_run(q, kC8qSeeds[m]); });
        for (std::thread& t : pool) t.join();
    }
    std::vector<eg::DownwashMeans> out;
    for (std::size_t m = 0; m < kRuns; ++m) {
        EXPECT_TRUE(runs[m].fault.empty()) << tag << " run " << m << ": " << runs[m].fault;
        if (!runs[m].fault.empty()) return {};
        if (kC8qSeeds[m] != 0) {
            EXPECT_GT(runs[m].moved, 0u) << tag << " run " << m << "'s nudge moved nothing";
        }
        const auto judged = eg::downwash_means(runs[m].frames, q.judged, eg::kC8q);
        EXPECT_TRUE(judged.has_value()) << tag << " run " << m << ": " << (judged ? "" : judged.error().detail);
        if (!judged) return {};
        out.push_back(*judged);
        std::printf("%s: run %zu seed 0x%llX:", tag, m, static_cast<unsigned long long>(kC8qSeeds[m]));
        for (std::size_t i = 0; i < judged->u.size(); ++i) {
            std::printf(" rotor %zu %.4f / %.4f m/s;", i, judged->u[i], judged->v[i]);
        }
        std::printf("\n");
    }
    label = runs[0].label;
    return out;
}

TEST(EndGoalC8q, ReferenceQuadAgainstMomentumTheory) {
    if (!env_set("SPADE_C8Q")) {
        GTEST_SKIP() << "set SPADE_C8Q=1 to run C8q's two ensembles of four at 128^3 (release, a quiet window)";
    }
    const std::optional<C8qCase> q0 = c8q_case(C8qVariant::rotor_0_alone);
    const std::optional<C8qCase> qq = c8q_case(C8qVariant::as_flown);
    ASSERT_TRUE(q0.has_value() && qq.has_value());
    ASSERT_EQ(q0->hover.throttle, qq->hover.throttle);  // both trimmed on the four-rotor model
    C8qRecordCase c;
    const std::vector<eg::DownwashMeans> rotor0 = c8q_ensemble(*q0, "c8q-rotor0-run", c.rotor0_label);
    ASSERT_EQ(rotor0.size(), std::size(kC8qSeeds));
    const std::vector<eg::DownwashMeans> quad = c8q_ensemble(*qq, "c8q-quad-run", c.quad_label);
    ASSERT_EQ(quad.size(), std::size(kC8qSeeds));
    const eg::Verdict v = eg::c8q(rotor0, quad);
    std::printf("c8q-verdict: %s %s\n", verdict_word(v.ok), v.detail.c_str());

    c.path = path_line();
    c.dx_mm = 1000.0 * static_cast<double>(qq->resolved.base.dx);
    c.r_over_dx = static_cast<double>(quad[0].r_over_dx);
    c.h_s = static_cast<double>(qq->resolved.spec.time.step.value_or(0.0f));
    c.nu = static_cast<double>(qq->resolved.spec.fluid.kinematic_viscosity);
    c.throttle = static_cast<double>(qq->hover.throttle);
    c.window = quad[0].window;
    c.frames = quad[0].count;
    std::fputs(c8q_record(c, rotor0, quad).c_str(), stdout);
    EXPECT_TRUE(v.ok) << v.detail;
}

// One run of a discriminating variant, seed 0, report only: each judged
// rotor's ratio, tagged, for Physics. No band is judged here.
void report_variant(C8qVariant variant, const char* tag) {
    const std::optional<C8qCase> q = c8q_case(variant);
    ASSERT_TRUE(q.has_value());
    const C8qRun run = c8q_run(*q, 0);
    ASSERT_TRUE(run.fault.empty()) << run.fault;
    const auto judged = eg::downwash_means(run.frames, q->judged, eg::kC8q);
    ASSERT_TRUE(judged.has_value()) << judged.error().detail;
    const eg::C8qBands b = eg::c8q_bands(judged->u, judged->v);
    std::printf("%s: %s; %s; hover throttle %.4f; the window %s, %zu frames\n", tag, path_line().c_str(),
                run.label.c_str(), static_cast<double>(q->hover.throttle), judged->window.c_str(), judged->count);
    for (std::size_t i = 0; i < judged->u.size(); ++i) {
        std::printf("%s: rotor %zu %.4f / %.4f m/s = %.4f\n", tag, i, judged->u[i], judged->v[i], b.ratio[i]);
    }
    if (judged->u.size() > 1) std::printf("%s: the rotors off their mean by %.4f at most\n", tag, b.mutual_off);
}

// Physics' discriminating runs for C8q's first miss (every rotor 0.816 of
// momentum theory, 2026-10-22), each the lead's grant on Physics' word. First,
// the discs' torque at 0, so they spread no swirl, as C8's disc spread none.
TEST(EndGoalC8q, ReferenceQuadWithoutTheDiscsSwirl) {
    if (!env_set("SPADE_C8Q_NO_TORQUE")) {
        GTEST_SKIP() << "set SPADE_C8Q_NO_TORQUE=1 for Physics' one C8q run with the discs' torque at 0";
    }
    report_variant(C8qVariant::no_torque, "c8q-no-torque");
}

// Then rotor 0 alone, as placed, still without swirl: the others push nothing.
// Near C8's +5.7%, the deficit is the four slipstreams together; still near
// 0.85, it is LiveRun's case itself (Physics, 2026-10-22).
TEST(EndGoalC8q, RotorZeroAloneWithoutSwirl) {
    if (!env_set("SPADE_C8Q_ROTOR0")) {
        GTEST_SKIP() << "set SPADE_C8Q_ROTOR0=1 for Physics' one C8q run with rotor 0 alone";
    }
    report_variant(C8qVariant::rotor_0_alone_no_torque, "c8q-rotor0");
}

}  // namespace
