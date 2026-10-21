// The analysis's queued runs as their own processes (W2c; INT-7; the CFD
// spec §5): launch_run() starts the installed runner tool on a run folder
// by its explicit path with an argument array, detached, and records its
// PID in run.yaml. A missing runner, a relative path and a real run (no
// airflow solver until W1) are refused by name. A dry run (the stand-in
// step) says so in run.yaml, its progress, its log and its history.
//
// The end-to-end tests run the real spade_analysis_runner and a host probe
// (analysis_host_probe.cpp) that launches it and exits at once.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/detail/process.hpp"
#include "analysis/launch.hpp"
#include "analysis/run.hpp"
#include "core/path_text.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

// A queued run of a 0.1 s hover case: the dry run's 100 steps of 1 ms.
[[nodiscard]] fs::path queued_run(const fs::path& runs, an::RunKind kind = an::RunKind::dry_run) {
    const auto s = spade::scene::load_scene_file(fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value());
    const spade::vehicles::ModelType m = s ? s->models.at(0) : spade::vehicles::ModelType{};
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", m);
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    c.time.duration = 0.1f;
    const auto r = an::resolve(c, m);
    if (!r) {
        ADD_FAILURE() << "the case did not resolve";
        return {};
    }
    const spade::Result<fs::path> folder = an::create_run_folder(runs, "hover", *r, "2026-10-13", "abc", kind);
    EXPECT_TRUE(folder.has_value()) << (folder ? "" : folder.error().context);
    return folder.value_or(fs::path());
}

[[nodiscard]] std::string read_all(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

[[nodiscard]] std::string first_line(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    std::string line;
    std::getline(in, line);
    return line;
}

// A stand-in runner file: launch_run() checks the runner exists before the
// launcher is asked.
[[nodiscard]] fs::path fake_runner(const fs::path& dir) {
    const fs::path exe = dir / "fake_runner.exe";
    std::ofstream(exe).put('\n');
    return exe;
}

// Polls `done` every 10 ms for up to 30 s (a debug build on a loaded box).
[[nodiscard]] bool eventually(const std::function<bool()>& done) {
    for (int i = 0; i < 3000; ++i) {
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return done();
}

[[nodiscard]] bool progress_is(const fs::path& run, an::RunState state) {
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    return p && p->state == state;
}

an::LaunchOptions dry_run(uint32_t pace_ms = 0) {
    an::LaunchOptions o;
    o.kind = an::RunKind::dry_run;
    o.pace_ms = pace_ms;
    return o;
}

}  // namespace

// ---- the command line Windows hands the runner ----------------------------

// CreateProcessW takes one string; the child's C runtime splits it back
// into arguments by CommandLineToArgvW's rules. Every argument must come
// back exactly as given, whatever spaces, quotes and backslashes it holds.
TEST(AnalysisLaunchCommandLine, QuotesOnlyWhatTheChildWouldSplit) {
    using an::detail::windows_command_line;
    EXPECT_EQ(windows_command_line({"runner.exe", "plain"}), "runner.exe plain");
    EXPECT_EQ(windows_command_line({"C:\\Program Files\\Spade\\runner.exe", "a b"}),
              "\"C:\\Program Files\\Spade\\runner.exe\" \"a b\"");
    EXPECT_EQ(windows_command_line({"r", ""}), "r \"\"");
    EXPECT_EQ(windows_command_line({"r", "say \"hi\""}), "r \"say \\\"hi\\\"\"");
    // Backslashes are literal except before a quote, so a folder's trailing
    // backslash is doubled once the argument is quoted.
    EXPECT_EQ(windows_command_line({"r", "C:\\my runs\\"}), "r \"C:\\my runs\\\\\"");
    EXPECT_EQ(windows_command_line({"r", "a\\\\b"}), "r a\\\\b");
    EXPECT_EQ(windows_command_line({"r", "a\\\"b"}), "r \"a\\\\\\\"b\"");
    EXPECT_EQ(windows_command_line({"r", "tab\there"}), "r \"tab\there\"");
}

// ---- run.yaml and the progress's kind --------------------------------------

TEST(AnalysisRunRecord, RoundTripsThroughItsFile) {
    const spade::test::TempDir dir("record");
    an::RunRecord r;
    r.kind = an::RunKind::dry_run;
    r.pid = 4242;
    r.runner = "C:/Spade/bin/spade_analysis_runner.exe";
    ASSERT_TRUE(an::write_run_record(dir.path(), r).has_value());
    const std::string text = read_all(dir.path() / "run.yaml");
    EXPECT_NE(text.find("kind: \"dry-run\""), std::string::npos) << text;
    const spade::Result<an::RunRecord> back = an::read_run_record(dir.path());
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->kind, an::RunKind::dry_run);
    EXPECT_EQ(back->pid, 4242u);
    EXPECT_EQ(back->runner, r.runner);
}

TEST(AnalysisRunRecord, TheProgressSaysItsKind) {
    const spade::test::TempDir dir("record");
    an::RunProgress p;
    p.kind = an::RunKind::dry_run;
    ASSERT_TRUE(an::write_progress(dir.path(), p).has_value());
    EXPECT_NE(read_all(dir.path() / "progress.yaml").find("kind: \"dry-run\""), std::string::npos);
    const spade::Result<an::RunProgress> back = an::read_progress(dir.path());
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->kind, an::RunKind::dry_run);
    p.kind = an::RunKind::cfd;
    ASSERT_TRUE(an::write_progress(dir.path(), p).has_value());
    EXPECT_EQ(an::read_progress(dir.path())->kind, an::RunKind::cfd);
}

// ---- launch_run's refusals and what it asks the launcher for ---------------

TEST(AnalysisLaunch, AMissingRunnerIsRefusedByName) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    const fs::path missing = runs.path() / "no_such_runner.exe";
    bool asked = false;
    const an::Launcher launcher = [&](const an::ProcessSpec&) -> spade::Result<uint64_t> {
        asked = true;
        return 1u;
    };
    const spade::Result<an::RunRecord> r = an::launch_run(missing, run, dry_run(), launcher);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::not_found);
    EXPECT_NE(r.error().context.find("no_such_runner.exe"), std::string::npos) << r.error().context;
    EXPECT_FALSE(asked);
    EXPECT_FALSE(fs::exists(run / "run.yaml"));
    // The real launcher refuses it by name too, before any process exists.
    const spade::Result<uint64_t> direct = an::detail::launch_detached({missing, {}, run});
    ASSERT_FALSE(direct.has_value());
    EXPECT_NE(direct.error().context.find("no_such_runner.exe"), std::string::npos) << direct.error().context;
}

// An explicit path, never a search: a bare name is refused.
TEST(AnalysisLaunch, ARelativeRunnerPathIsRefused) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    const spade::Result<an::RunRecord> r =
        an::launch_run("spade_analysis_runner", run, dry_run(), [](const an::ProcessSpec&) -> spade::Result<uint64_t> {
            return 1u;
        });
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_NE(r.error().context.find("absolute"), std::string::npos) << r.error().context;
}

TEST(AnalysisLaunch, ARealRunIsRefusedUntilTheSolverExists) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    bool asked = false;
    const spade::Result<an::RunRecord> r =
        an::launch_run(fake_runner(runs.path()), run, an::LaunchOptions{},
                       [&](const an::ProcessSpec&) -> spade::Result<uint64_t> {
                           asked = true;
                           return 1u;
                       });
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::unavailable);
    EXPECT_NE(r.error().context.find("no airflow solver yet (W1)"), std::string::npos) << r.error().context;
    EXPECT_FALSE(asked);
}

// A folder runs as the kind it was made as: a real run's folder isn't
// launched as a dry run (its name and need are a real run's).
TEST(AnalysisLaunch, ARunLaunchesAsTheKindItWasMadeAs) {
    const spade::test::TempDir runs("launch");
    const fs::path real = queued_run(runs.path(), an::RunKind::cfd);
    bool asked = false;
    const spade::Result<an::RunRecord> r = an::launch_run(fake_runner(runs.path()), real, dry_run(),
                                                          [&](const an::ProcessSpec&) -> spade::Result<uint64_t> {
                                                              asked = true;
                                                              return 1u;
                                                          });
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_NE(r.error().context.find("made as a cfd run"), std::string::npos) << r.error().context;
    EXPECT_FALSE(asked);
}

// cancel_run() writes its flag only into an unfinished run's folder.
TEST(AnalysisLaunch, CancelRefusesWhatIsntAnUnfinishedRun) {
    const spade::test::TempDir runs("launch");
    const spade::Result<void> not_a_run = an::cancel_run(runs.path());
    ASSERT_FALSE(not_a_run.has_value());
    EXPECT_NE(not_a_run.error().context.find("not a run folder"), std::string::npos) << not_a_run.error().context;
    EXPECT_FALSE(fs::exists(runs.path() / "cancel"));

    const fs::path run = queued_run(runs.path());
    an::RunProgress done = an::read_progress(run).value_or(an::RunProgress{});
    done.state = an::RunState::done;
    ASSERT_TRUE(an::write_progress(run, done).has_value());
    const spade::Result<void> finished = an::cancel_run(run);
    ASSERT_FALSE(finished.has_value());
    EXPECT_NE(finished.error().context.find("finished"), std::string::npos) << finished.error().context;
    EXPECT_FALSE(fs::exists(run / "cancel"));

    const fs::path queued = queued_run(runs.path());
    EXPECT_TRUE(an::cancel_run(queued).has_value());
}

TEST(AnalysisLaunch, AFolderWithoutItsCaseIsRefused) {
    const spade::test::TempDir runs("launch");
    const spade::Result<an::RunRecord> r = an::launch_run(
        fake_runner(runs.path()), runs.path(), dry_run(),
        [](const an::ProcessSpec&) -> spade::Result<uint64_t> { return 1u; });
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("case.yaml"), std::string::npos) << r.error().context;
}

// The runner by its explicit path, its arguments as an array, in the run's
// folder; then run.yaml holds the PID and the progress the kind.
TEST(AnalysisLaunch, AsksForTheRunnerByPathWithAnArgumentArray) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    const fs::path runner = fake_runner(runs.path());
    an::ProcessSpec seen;
    const spade::Result<an::RunRecord> r =
        an::launch_run(runner, run, dry_run(20), [&](const an::ProcessSpec& spec) -> spade::Result<uint64_t> {
            seen = spec;
            return 4242u;
        });
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(seen.executable, runner);
    EXPECT_EQ(seen.args, (std::vector<std::string>{spade::path_text(fs::absolute(run)), "--dry-run", "--pace-ms", "20"}));
    EXPECT_EQ(seen.working_dir, fs::absolute(run));
    EXPECT_EQ(r->pid, 4242u);
    EXPECT_EQ(r->kind, an::RunKind::dry_run);
    const spade::Result<an::RunRecord> record = an::read_run_record(run);
    ASSERT_TRUE(record.has_value()) << record.error().context;
    EXPECT_EQ(record->pid, 4242u);
    EXPECT_EQ(record->runner, spade::path_text(runner));
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->kind, an::RunKind::dry_run);
    EXPECT_EQ(p->state, an::RunState::queued);
}

// A folder runs once: a second launch names the first's PID.
TEST(AnalysisLaunch, AFolderIsLaunchedOnce) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    const fs::path runner = fake_runner(runs.path());
    const an::Launcher launcher = [](const an::ProcessSpec&) -> spade::Result<uint64_t> { return 4242u; };
    ASSERT_TRUE(an::launch_run(runner, run, dry_run(), launcher).has_value());
    const spade::Result<an::RunRecord> again = an::launch_run(runner, run, dry_run(), launcher);
    ASSERT_FALSE(again.has_value());
    EXPECT_NE(again.error().context.find("4242"), std::string::npos) << again.error().context;
}

// A launcher's own refusal reaches the host, and nothing is recorded.
TEST(AnalysisLaunch, ALauncherRefusalRecordsNothing) {
    const spade::test::TempDir runs("launch");
    const fs::path run = queued_run(runs.path());
    const spade::Result<an::RunRecord> r = an::launch_run(
        fake_runner(runs.path()), run, dry_run(), [](const an::ProcessSpec&) -> spade::Result<uint64_t> {
            return std::unexpected(spade::Error{spade::Code::io_error, "CreateProcessW failed: access denied"});
        });
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("access denied"), std::string::npos);
    EXPECT_FALSE(fs::exists(run / "run.yaml"));
}

// ---- end to end: the real runner in its own process ------------------------

TEST(AnalysisLaunchEndToEnd, ADryRunRunsToDoneInItsOwnProcess) {
    const spade::test::TempDir runs("e2e");
    const fs::path run = queued_run(runs.path());
    const spade::Result<an::RunRecord> r = an::launch_run(SPADE_ANALYSIS_RUNNER, run, dry_run());
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_GT(r->pid, 0u);
    ASSERT_TRUE(eventually([&] { return progress_is(run, an::RunState::done); })) << read_all(run / "progress.yaml");
    ASSERT_TRUE(eventually([&] { return !an::process_running(r->pid); }));
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->kind, an::RunKind::dry_run);
    EXPECT_EQ(p->steps, 100u);
    // Its output says what it is wherever a reader looks.
    EXPECT_EQ(first_line(run / "history.csv").rfind("# kind: dry-run", 0), 0u) << first_line(run / "history.csv");
    EXPECT_NE(read_all(run / "log.txt").find("dry-run"), std::string::npos) << read_all(run / "log.txt");
    EXPECT_NE(read_all(run / "run.yaml").find("kind: \"dry-run\""), std::string::npos);
}

// The host asks with cancel_run(); the runner stops between steps and its
// process ends.
TEST(AnalysisLaunchEndToEnd, ACancelledRunStopsBetweenSteps) {
    const spade::test::TempDir runs("e2e");
    const fs::path run = queued_run(runs.path());
    const spade::Result<an::RunRecord> r = an::launch_run(SPADE_ANALYSIS_RUNNER, run, dry_run(20));
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_TRUE(eventually([&] {
        const spade::Result<an::RunProgress> p = an::read_progress(run);
        return p && p->state == an::RunState::running && p->steps > 0;
    }));
    ASSERT_TRUE(an::cancel_run(run).has_value());
    ASSERT_TRUE(eventually([&] { return progress_is(run, an::RunState::cancelled); })) << read_all(run / "progress.yaml");
    ASSERT_TRUE(eventually([&] { return !an::process_running(r->pid); }));
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    ASSERT_TRUE(p.has_value());
    EXPECT_LT(p->steps, 100u);
    EXPECT_NE(p->message.find("partial"), std::string::npos) << p->message;
}

// The host closes and the run goes on: the probe launches the runner and
// exits at once; the runner, by the PID run.yaml holds, is still running,
// then finishes.
TEST(AnalysisLaunchEndToEnd, TheRunOutlivesTheHostThatLaunchedIt) {
    const spade::test::TempDir runs("e2e");
    const fs::path run = queued_run(runs.path());
    const spade::Result<uint64_t> host = an::detail::launch_detached(
        {SPADE_ANALYSIS_HOST_PROBE, {spade::path_text(fs::path(SPADE_ANALYSIS_RUNNER)), spade::path_text(run)}, run});
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(eventually([&] { return !an::process_running(*host); }));
    ASSERT_TRUE(eventually([&] { return an::read_run_record(run).has_value(); }));
    const spade::Result<an::RunRecord> record = an::read_run_record(run);
    ASSERT_TRUE(record.has_value());
    EXPECT_NE(record->pid, *host);
    // The host is gone; the run is not done, and its process lives.
    EXPECT_FALSE(progress_is(run, an::RunState::done));
    EXPECT_TRUE(an::process_running(record->pid));
    ASSERT_TRUE(eventually([&] { return progress_is(run, an::RunState::done); })) << read_all(run / "progress.yaml");
    EXPECT_TRUE(eventually([&] { return !an::process_running(record->pid); }));
}

// Run straight, without --dry-run, the runner refuses in the folder itself,
// where the host reads it.
TEST(AnalysisLaunchEndToEnd, TheRunnerRefusesARealRun) {
    const spade::test::TempDir runs("e2e");
    const fs::path run = queued_run(runs.path());
    const spade::Result<uint64_t> pid =
        an::detail::launch_detached({SPADE_ANALYSIS_RUNNER, {spade::path_text(run)}, run});
    ASSERT_TRUE(pid.has_value()) << pid.error().context;
    ASSERT_TRUE(eventually([&] { return progress_is(run, an::RunState::refused); })) << read_all(run / "progress.yaml");
    EXPECT_TRUE(eventually([&] { return !an::process_running(*pid); }));
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    ASSERT_TRUE(p.has_value());
    EXPECT_NE(p->message.find("no airflow solver yet (W1)"), std::string::npos) << p->message;
    EXPECT_FALSE(fs::exists(run / "history.csv"));
}
