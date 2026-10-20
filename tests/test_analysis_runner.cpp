// The analysis's runner loop (W2b; INT-7; the CFD spec §5), driven by a
// stand-in step and a fake clock: it runs a case to its duration, samples
// its history, stops for a cancel flag between steps, and names a fault.
// The real solver plugs in at W1.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/run.hpp"
#include "analysis/runner.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

// A queued run of a 0.1 s hover case.
[[nodiscard]] fs::path queued_run(const fs::path& runs) {
    const auto s = spade::scene::load_scene_file(fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value());
    const spade::vehicles::ModelType m = s ? s->models.at(0) : spade::vehicles::ModelType{};
    an::AnalysisCase c;
    c.model = an::ModelRef{"quad.model.yaml", spade::vehicles::model_identity(m), spade::vehicles::parts_identity(m)};
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    c.time.duration = 0.1f;
    const auto r = an::resolve(c, m);
    if (!r) {
        ADD_FAILURE() << "the case did not resolve";
        return {};
    }
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs, "hover", *r, "2026-10-13", "abc", an::RunKind::dry_run);
    EXPECT_TRUE(folder.has_value()) << (folder ? "" : folder.error().context);
    return folder.value_or(fs::path());
}

// 10 ms of simulated time per step; the loads are the step's number.
[[nodiscard]] an::StepFn ticking() {
    return [](uint64_t step) -> spade::Result<an::StepSample> {
        return an::StepSample{0.01 * static_cast<double>(step + 1), {static_cast<double>(step), 2.0}};
    };
}

// A clock that moves 50 ms on every read.
[[nodiscard]] an::Clock fake_clock() {
    auto t = std::make_shared<double>(0.0);
    return [t] { return *t += 0.05; };
}

[[nodiscard]] an::RunnerSetup setup() {
    an::RunnerSetup s;
    s.columns = {"thrust_N", "torque_N_m"};
    s.memory_bytes = 4096;
    return s;
}

[[nodiscard]] std::vector<std::string> lines_of(const fs::path& file) {
    std::ifstream in(file);
    std::vector<std::string> out;
    for (std::string line; std::getline(in, line);) out.push_back(line);
    return out;
}

}  // namespace

TEST(AnalysisRunner, RunsToTheCasesDurationAndSaysDone) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    const spade::Result<an::RunProgress> end = an::run_case(run, setup(), ticking(), fake_clock());
    ASSERT_TRUE(end.has_value()) << end.error().context;
    EXPECT_EQ(end->state, an::RunState::done);
    EXPECT_EQ(end->steps, 10u);
    EXPECT_NEAR(end->simulated_s, 0.1, 1e-12);
    EXPECT_EQ(end->memory_bytes, 4096u);
    EXPECT_GT(end->elapsed_s, 0.0);
    EXPECT_NEAR(end->ms_per_step, end->elapsed_s * 1000.0 / 10.0, 1e-9);
    // What a host reads is what the loop ended with.
    const spade::Result<an::RunProgress> read = an::read_progress(run);
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(read->state, an::RunState::done);
    EXPECT_EQ(read->steps, 10u);
}

// Every step's sample, with the units in the header (DBC-035).
TEST(AnalysisRunner, WritesItsHistoryWithUnits) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    ASSERT_TRUE(an::run_case(run, setup(), ticking(), fake_clock()).has_value());
    const std::vector<std::string> rows = lines_of(run / "history.csv");
    ASSERT_EQ(rows.size(), 11u);
    EXPECT_EQ(rows.front(), "time_s,thrust_N,torque_N_m");
    EXPECT_EQ(rows.at(1), "0.01,0,2");
    EXPECT_EQ(rows.back(), "0.1,9,2");
}

TEST(AnalysisRunner, SamplesItsHistoryEveryInterval) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    an::RunnerSetup s = setup();
    s.history_every_s = 0.05;
    ASSERT_TRUE(an::run_case(run, s, ticking(), fake_clock()).has_value());
    const std::vector<std::string> rows = lines_of(run / "history.csv");
    ASSERT_EQ(rows.size(), 3u) << "the header, then 0.05 and 0.1";
    EXPECT_EQ(rows.at(1).substr(0, 5), "0.05,");
    EXPECT_EQ(rows.at(2).substr(0, 4), "0.1,");
}

// The host asks by making the cancel file; the loop sees it between steps.
TEST(AnalysisRunner, ACancelFlagStopsItBetweenSteps) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    const an::StepFn step = [&](uint64_t k) -> spade::Result<an::StepSample> {
        if (k == 2) std::ofstream(run / an::kCancelFile).put('\n');
        return an::StepSample{0.01 * static_cast<double>(k + 1), {0.0, 0.0}};
    };
    const spade::Result<an::RunProgress> end = an::run_case(run, setup(), step, fake_clock());
    ASSERT_TRUE(end.has_value()) << end.error().context;
    EXPECT_EQ(end->state, an::RunState::cancelled);
    EXPECT_EQ(end->steps, 3u);
    EXPECT_NE(end->message.find("partial"), std::string::npos) << end->message;
    EXPECT_EQ(lines_of(run / "history.csv").size(), 4u);  // what it wrote stays
}

TEST(AnalysisRunner, AFaultStopsItAndIsNamed) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    const an::StepFn step = [](uint64_t k) -> spade::Result<an::StepSample> {
        if (k == 4) return std::unexpected(spade::Error{spade::Code::internal, "non-finite pressure in cell (3, 4, 5)"});
        return an::StepSample{0.01 * static_cast<double>(k + 1), {0.0, 0.0}};
    };
    const spade::Result<an::RunProgress> end = an::run_case(run, setup(), step, fake_clock());
    ASSERT_TRUE(end.has_value()) << end.error().context;
    EXPECT_EQ(end->state, an::RunState::faulted);
    EXPECT_EQ(end->steps, 4u);
    EXPECT_NE(end->message.find("non-finite pressure in cell (3, 4, 5)"), std::string::npos) << end->message;
    const std::vector<std::string> log = lines_of(run / "log.txt");
    ASSERT_FALSE(log.empty());
    EXPECT_NE(log.back().find("non-finite pressure"), std::string::npos) << log.back();
}

// A sample whose width differs from the header's is refused: a history
// column never shifts under its name.
TEST(AnalysisRunner, ASampleOfTheWrongWidthIsAFault) {
    const spade::test::TempDir runs("runner");
    const fs::path run = queued_run(runs.path());
    const an::StepFn step = [](uint64_t k) -> spade::Result<an::StepSample> {
        return an::StepSample{0.01 * static_cast<double>(k + 1), {1.0}};
    };
    const spade::Result<an::RunProgress> end = an::run_case(run, setup(), step, fake_clock());
    ASSERT_TRUE(end.has_value());
    EXPECT_EQ(end->state, an::RunState::faulted);
    EXPECT_NE(end->message.find("1 values for 2 columns"), std::string::npos) << end->message;
}

TEST(AnalysisRunner, AFolderWithoutItsCaseIsRefused) {
    const spade::test::TempDir runs("runner");
    const spade::Result<an::RunProgress> end = an::run_case(runs.path(), setup(), ticking(), fake_clock());
    ASSERT_FALSE(end.has_value());
    EXPECT_EQ(end.error().code, spade::Code::io_error);
}
