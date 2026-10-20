// The analysis's run queue (W2c; INT-7; the CFD spec §5 "Queued"): one heavy
// run at a time, started only when its memory need plus the box's margin is
// free, and otherwise waiting and saying why. Its order is runs/queue.txt
// and each run's state its own progress.yaml, so a host that closes and
// reopens finds the queue as it left it. A runner that dies without
// finishing is a fault; a queued run that is cancelled never starts.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/launch.hpp"
#include "analysis/queue.hpp"
#include "analysis/run.hpp"
#include "core/path_text.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

constexpr uint64_t kGiB = 1ull << 30;

// A queued 0.1 s hover run named `name` under `runs`, a dry run unless said.
[[nodiscard]] fs::path queued_run(const fs::path& runs, const std::string& name,
                                  an::RunKind kind = an::RunKind::dry_run) {
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
    const spade::Result<fs::path> folder = an::create_run_folder(runs, name, *r, "2026-10-13", "abc", kind);
    EXPECT_TRUE(folder.has_value()) << (folder ? "" : folder.error().context);
    return folder.value_or(fs::path());
}

[[nodiscard]] std::string read_all(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

[[nodiscard]] an::RunProgress progress(const fs::path& run) {
    const spade::Result<an::RunProgress> p = an::read_progress(run);
    EXPECT_TRUE(p.has_value()) << (p ? "" : p.error().context);
    return p.value_or(an::RunProgress{});
}

// A machine in a test: launches are recorded and given PIDs from 101; a
// PID runs until the test ends it; free memory is what the test says.
struct FakeBox {
    std::vector<an::ProcessSpec> launched;
    std::set<uint64_t> alive;
    uint64_t free = 64 * kGiB;

    [[nodiscard]] an::QueueProbes probes() {
        an::QueueProbes p;
        p.launch = [this](const an::ProcessSpec& spec) -> spade::Result<uint64_t> {
            launched.push_back(spec);
            const uint64_t pid = 100 + launched.size();
            alive.insert(pid);
            return pid;
        };
        p.running = [this](uint64_t pid) { return alive.count(pid) > 0; };
        p.free_bytes = [this] { return free; };
        return p;
    }
};

struct Queue {
    spade::test::TempDir dir{"queue"};
    fs::path runner;
    an::QueueSetup setup;

    Queue() {
        runner = dir.path() / "fake_runner.exe";
        std::ofstream(runner).put('\n');
        setup.runs_dir = dir.path() / "runs";
        setup.runner = runner;
        setup.options.kind = an::RunKind::dry_run;
    }
    [[nodiscard]] fs::path add(const std::string& name, an::RunKind kind = an::RunKind::dry_run) {
        const fs::path run = queued_run(setup.runs_dir, name, kind);
        const spade::Result<void> r = an::enqueue_run(setup, run);
        EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().context);
        return run;
    }
};

// The run a launch was for: its folder is the first argument.
[[nodiscard]] fs::path folder_of(const an::ProcessSpec& spec) { return fs::path(spec.args.at(0)); }

}  // namespace

// ---- what a queued run needs ----------------------------------------------------

// A real run's queued progress carries its memory need, the resolved base
// grid; a dry run holds no grid, so it needs none.
TEST(AnalysisQueue, AQueuedRunCarriesItsMemoryNeed) {
    Queue q;
    EXPECT_GT(progress(q.add("real", an::RunKind::cfd)).memory_bytes, 0u);
    EXPECT_EQ(progress(q.add("dry")).memory_bytes, 0u);
}

// ---- the order, kept on disk ----------------------------------------------------

TEST(AnalysisQueue, EnqueueKeepsTheOrderOnDisk) {
    Queue q;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    const std::string list = read_all(q.setup.runs_dir / "queue.txt");
    EXPECT_LT(list.find("a/"), list.find("b/")) << list;
    // Twice, outside the runs folder, or not queued: refused by name.
    EXPECT_FALSE(an::enqueue_run(q.setup, a).has_value());
    const spade::test::TempDir elsewhere("elsewhere");
    const fs::path outside = queued_run(elsewhere.path(), "c");
    const spade::Result<void> r = an::enqueue_run(q.setup, outside);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("runs"), std::string::npos) << r.error().context;
    (void)b;
}

// ---- one at a time --------------------------------------------------------------

TEST(AnalysisQueue, StartsTheFirstRunWhenNothingRuns) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(box.launched.size(), 1u);
    EXPECT_EQ(fs::equivalent(folder_of(box.launched[0]), a), true);
    EXPECT_TRUE(fs::equivalent(s->started, a));
    EXPECT_TRUE(fs::equivalent(s->active, a));
    ASSERT_EQ(s->waiting.size(), 1u);
    EXPECT_TRUE(fs::equivalent(s->waiting[0], b));
    EXPECT_EQ(an::read_run_record(a)->pid, 101u);
}

TEST(AnalysisQueue, RunsOneAtATime) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    ASSERT_TRUE(an::step_queue(q.setup, box.probes()).has_value());
    // a's runner is still going: b waits for it, and says so.
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(box.launched.size(), 1u);
    EXPECT_TRUE(s->started.empty());
    EXPECT_TRUE(fs::equivalent(s->active, a));
    EXPECT_NE(s->why.find("a/"), std::string::npos) << s->why;
    // a finishes: b starts.
    an::RunProgress done = progress(a);
    done.state = an::RunState::done;
    ASSERT_TRUE(an::write_progress(a, done).has_value());
    box.alive.erase(101);
    const spade::Result<an::QueueStep> next = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(next.has_value()) << next.error().context;
    ASSERT_EQ(box.launched.size(), 2u);
    EXPECT_TRUE(fs::equivalent(next->started, b));
    // A finished run leaves the queue.
    EXPECT_EQ(read_all(q.setup.runs_dir / "queue.txt").find("a/"), std::string::npos);
}

// ---- memory -----------------------------------------------------------------------

// A real run waits for its need plus the margin. With no solver yet (W1), a
// real run let through the gate is then refused at its launch, so that is how
// this test sees the gate open.
TEST(AnalysisQueue, ARunWaitsForItsMemoryAndSaysWhy) {
    Queue q;
    q.setup.options.kind = an::RunKind::cfd;
    FakeBox box;
    const fs::path a = q.add("a", an::RunKind::cfd);
    const uint64_t need = progress(a).memory_bytes;
    box.free = need + q.setup.margin_bytes - 1;
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_TRUE(box.launched.empty());
    EXPECT_TRUE(s->active.empty());
    EXPECT_NE(s->why.find("GiB"), std::string::npos) << s->why;
    EXPECT_NE(s->why.find("margin"), std::string::npos) << s->why;
    const an::RunProgress waiting = progress(a);
    EXPECT_EQ(waiting.state, an::RunState::queued);
    EXPECT_EQ(waiting.message, s->why);
    // Enough frees up: the gate opens, and the launch is the next refusal.
    box.free = need + q.setup.margin_bytes;
    const spade::Result<an::QueueStep> next = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(next.has_value());
    EXPECT_TRUE(box.launched.empty());
    EXPECT_EQ(progress(a).state, an::RunState::refused);
    EXPECT_NE(progress(a).message.find("no airflow solver yet (W1)"), std::string::npos) << progress(a).message;
}

// A dry run holds no grid, so no memory gate: it starts at once, however
// little is free, and its progress and log say so (the lead's ruling).
TEST(AnalysisQueue, ADryRunSkipsTheMemoryGate) {
    Queue q;
    FakeBox box;
    box.free = 0;
    const fs::path a = q.add("a");
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(box.launched.size(), 1u);
    EXPECT_TRUE(fs::equivalent(s->started, a));
    EXPECT_TRUE(s->why.empty()) << s->why;
    EXPECT_EQ(progress(a).message, an::kDryRunNoMemoryGate);
    EXPECT_NE(read_all(a / "log.txt").find(an::kDryRunNoMemoryGate), std::string::npos) << read_all(a / "log.txt");
}

// ---- runs that end without finishing ---------------------------------------------

TEST(AnalysisQueue, ARunnerThatDiesIsAFault) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    ASSERT_TRUE(an::step_queue(q.setup, box.probes()).has_value());
    an::RunProgress running = progress(a);
    running.state = an::RunState::running;
    running.steps = 12;
    ASSERT_TRUE(an::write_progress(a, running).has_value());
    box.alive.erase(101);  // gone, its progress still "running"
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const an::RunProgress faulted = progress(a);
    EXPECT_EQ(faulted.state, an::RunState::faulted);
    EXPECT_NE(faulted.message.find("pid 101"), std::string::npos) << faulted.message;
    EXPECT_NE(faulted.message.find("partial"), std::string::npos) << faulted.message;
    EXPECT_EQ(faulted.steps, 12u);
    // The queue goes on.
    EXPECT_TRUE(fs::equivalent(s->started, b));
}

TEST(AnalysisQueue, ACancelledQueuedRunNeverStarts) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    ASSERT_TRUE(an::cancel_run(a).has_value());
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(progress(a).state, an::RunState::cancelled);
    EXPECT_FALSE(fs::exists(a / "run.yaml"));
    ASSERT_EQ(box.launched.size(), 1u);
    EXPECT_TRUE(fs::equivalent(s->started, b));
}

// A run folder the host deleted leaves the queue, named, so the queue never
// wedges on it.
TEST(AnalysisQueue, AMissingRunLeavesTheQueueNamed) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    fs::remove_all(a);
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, box.probes());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(s->dropped.size(), 1u);
    EXPECT_NE(spade::path_text(s->dropped[0]).find("a"), std::string::npos);
    EXPECT_TRUE(fs::equivalent(s->started, b));
}

// A launch the queue can't make (the runner missing) is the run's refusal,
// named in its progress, and the queue goes on.
TEST(AnalysisQueue, ARefusedLaunchIsNamedAndTheQueueGoesOn) {
    Queue q;
    FakeBox box;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    an::QueueProbes probes = box.probes();
    bool first = true;
    probes.launch = [&](const an::ProcessSpec& spec) -> spade::Result<uint64_t> {
        if (first) {
            first = false;
            return std::unexpected(spade::Error{spade::Code::io_error, "CreateProcessW failed: access denied"});
        }
        box.launched.push_back(spec);
        box.alive.insert(102);
        return 102u;
    };
    const spade::Result<an::QueueStep> s = an::step_queue(q.setup, probes);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const an::RunProgress refused = progress(a);
    EXPECT_EQ(refused.state, an::RunState::refused);
    EXPECT_NE(refused.message.find("access denied"), std::string::npos) << refused.message;
    EXPECT_TRUE(fs::equivalent(s->started, b));
}

// ---- end to end: the real runner, two dry runs, one after the other ---------------

TEST(AnalysisQueueEndToEnd, TwoDryRunsRunOneAfterTheOther) {
    Queue q;
    q.setup.runner = SPADE_ANALYSIS_RUNNER;
    q.setup.options.pace_ms = 5;
    const fs::path a = q.add("a");
    const fs::path b = q.add("b");
    an::QueueProbes probes;
    probes.free_bytes = [] { return 64 * kGiB; };  // the box's own memory isn't this test's question
    bool b_started_after_a_done = false;
    for (int i = 0; i < 3000; ++i) {
        const spade::Result<an::QueueStep> s = an::step_queue(q.setup, probes);
        ASSERT_TRUE(s.has_value()) << s.error().context;
        if (!s->started.empty() && fs::equivalent(s->started, b)) {
            b_started_after_a_done = progress(a).state == an::RunState::done;
        }
        if (progress(b).state == an::RunState::done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(progress(a).state, an::RunState::done) << read_all(a / "progress.yaml");
    EXPECT_EQ(progress(b).state, an::RunState::done) << read_all(b / "progress.yaml");
    EXPECT_TRUE(b_started_after_a_done);
    for (const fs::path& run : {a, b}) {
        const spade::Result<an::RunRecord> record = an::read_run_record(run);
        ASSERT_TRUE(record.has_value());
        for (int i = 0; i < 3000 && an::process_running(record->pid); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_FALSE(an::process_running(record->pid));
    }
}
