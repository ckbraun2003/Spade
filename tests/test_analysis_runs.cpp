// The analysis's runs (W2; INT-7; the CFD spec §5): the run folder, its
// progress file replaced atomically so a host reading it never sees a torn
// one, and its name.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <thread>

#include "analysis/case.hpp"
#include "analysis/run.hpp"
#include "core/build_identity.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

[[nodiscard]] spade::vehicles::ModelType quad() {
    const auto s = spade::scene::load_scene_file(fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value()) << (s ? "" : s.error().context);
    return s ? s->models.at(0) : spade::vehicles::ModelType{};
}

// A hover case with discs, resolved against the quad.
[[nodiscard]] an::ResolvedCase resolved() {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c;
    c.model = an::ModelRef{"quad.model.yaml", spade::vehicles::model_identity(m), spade::vehicles::parts_identity(m)};
    c.controls.armed = true;
    c.controls.throttle = 0.5f;
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, m);
    EXPECT_TRUE(r.has_value());
    return r ? *r : an::ResolvedCase{};
}

[[nodiscard]] std::string read_all(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(AnalysisRunProgress, RoundTripsThroughItsFile) {
    const spade::test::TempDir dir("progress");
    an::RunProgress p;
    p.state = an::RunState::running;
    p.simulated_s = 0.5;
    p.target_s = 2.0;
    p.steps = 500;
    p.elapsed_s = 12.25;
    p.ms_per_step = 24.5;
    p.memory_bytes = 203423744;
    p.message = "line one: \"quoted\"";
    ASSERT_TRUE(an::write_progress(dir.path(), p).has_value());
    const spade::Result<an::RunProgress> back = an::read_progress(dir.path());
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->state, an::RunState::running);
    EXPECT_EQ(back->simulated_s, 0.5);
    EXPECT_EQ(back->target_s, 2.0);
    EXPECT_EQ(back->steps, 500u);
    EXPECT_EQ(back->elapsed_s, 12.25);
    EXPECT_EQ(back->ms_per_step, 24.5);
    EXPECT_EQ(back->memory_bytes, 203423744u);
    EXPECT_EQ(back->message, p.message);
}

TEST(AnalysisRunProgress, AMissingFileIsAnIoError) {
    const spade::test::TempDir dir("noprogress");
    const spade::Result<an::RunProgress> p = an::read_progress(dir.path());
    ASSERT_FALSE(p.has_value());
    EXPECT_EQ(p.error().code, spade::Code::io_error);
}

// The writer replaces the file while a reader reads it, hundreds of times:
// every read is one whole progress, the old or the new, never a mix.
TEST(AnalysisRunProgress, AReaderNeverSeesATornFile) {
    const spade::test::TempDir dir("torn");
    const auto big = [](char c) {
        an::RunProgress p;
        p.state = an::RunState::running;
        p.message = std::string(64 * 1024, c);
        return p;
    };
    ASSERT_TRUE(an::write_progress(dir.path(), big('a')).has_value());
    std::atomic<bool> done{false};
    std::atomic<int> torn{0};
    std::atomic<int> reads{0};
    std::thread reader([&] {
        while (!done.load()) {
            const spade::Result<an::RunProgress> p = an::read_progress(dir.path());
            if (!p || p->message.size() != 64u * 1024u ||
                p->message.find_first_not_of(p->message.front()) != std::string::npos) {
                ++torn;
            }
            ++reads;
        }
    });
    int failed_writes = 0;
    for (int i = 0; i < 300; ++i) {
        if (!an::write_progress(dir.path(), big(i % 2 == 0 ? 'b' : 'a')).has_value()) ++failed_writes;
    }
    done = true;
    reader.join();
    EXPECT_EQ(failed_writes, 0);
    EXPECT_EQ(torn.load(), 0) << "of " << reads.load() << " reads";
    EXPECT_GT(reads.load(), 0);
    // Nothing of the writer's is left beside it.
    int files = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(dir.path())) {
        (void)e;
        ++files;
    }
    EXPECT_EQ(files, 1);
}

// runs/<case>/<date>_<hash16>_<commit12>/: the case it ran, by its hash,
// at the commit that ran it.
TEST(AnalysisRunFolder, IsNamedByDateHashAndCommit) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const fs::path folder = an::run_folder(runs.path(), "hover", r, "2026-10-13",
                                           "36f8ca59f717c9545373d59c8cd2764b38a6b021", an::RunKind::cfd);
    char hash[17];
    std::snprintf(hash, sizeof hash, "%016llx", static_cast<unsigned long long>(r.hash));
    EXPECT_EQ(folder, runs.path() / "hover" / ("2026-10-13_" + std::string(hash) + "_36f8ca59f717"));
    // A dry run's folder says so, so it never shares a name with a real run.
    EXPECT_EQ(an::run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca59f717", an::RunKind::dry_run),
              runs.path() / "hover" / ("2026-10-13_" + std::string(hash) + "_36f8ca59f717_dry"));
}

TEST(AnalysisRunFolder, StartsQueuedWithTheCaseItRuns) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca5", an::RunKind::cfd);
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    EXPECT_EQ(read_all(*folder / "case.yaml"), an::case_to_yaml(r.spec).value_or("-"));
    const spade::Result<an::RunProgress> p = an::read_progress(*folder);
    ASSERT_TRUE(p.has_value()) << p.error().context;
    EXPECT_EQ(p->state, an::RunState::queued);
    EXPECT_EQ(p->kind, an::RunKind::cfd);
    EXPECT_EQ(p->target_s, static_cast<double>(r.spec.time.duration));
    EXPECT_EQ(p->memory_bytes, r.base_bytes);  // a real run's need, which the queue waits for
    EXPECT_EQ(p->message, "");
}

// A dry run holds no grid: its need is 0, and its progress says it skips the
// memory gate (the lead's ruling, 2026-10-19).
TEST(AnalysisRunFolder, ADryRunIsQueuedWithoutAMemoryNeed) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca5", an::RunKind::dry_run);
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    EXPECT_TRUE(folder->filename().string().ends_with("_dry")) << folder->filename().string();
    const spade::Result<an::RunProgress> p = an::read_progress(*folder);
    ASSERT_TRUE(p.has_value()) << p.error().context;
    EXPECT_EQ(p->kind, an::RunKind::dry_run);
    EXPECT_EQ(p->memory_bytes, 0u);
    EXPECT_EQ(p->message, an::kDryRunNoMemoryGate);
}

// A re-run the same day keeps the earlier run's folder and takes the lowest
// free suffix; a folder already there, even an empty one, is never shared.
TEST(AnalysisRunFolder, AReRunTakesTheLowestFreeSuffix) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const auto make = [&] {
        return an::create_run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca5", an::RunKind::cfd);
    };
    const spade::Result<fs::path> first = make();
    ASSERT_TRUE(first.has_value()) << first.error().context;
    const spade::Result<fs::path> second = make();
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(second->filename().string(), first->filename().string() + "_2");
    EXPECT_TRUE(fs::exists(*first / "case.yaml")) << "the earlier run was disturbed";
    fs::create_directory(first->parent_path() / (first->filename().string() + "_3"));  // another host's, say
    const spade::Result<fs::path> fourth = make();
    ASSERT_TRUE(fourth.has_value()) << fourth.error().context;
    EXPECT_EQ(fourth->filename().string(), first->filename().string() + "_4");
    EXPECT_FALSE(fs::exists(first->parent_path() / (first->filename().string() + "_3") / "case.yaml"));
}

// All or nothing: a run that can't be made leaves no folder behind.
TEST(AnalysisRunFolder, ARefusedRunLeavesNoFolder) {
    const spade::test::TempDir runs("runs");
    an::ResolvedCase bad = resolved();
    bad.spec.time.duration = -1.0f;  // case_to_yaml refuses it, after the folder's name is chosen
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", bad, "2026-10-13", "36f8ca5", an::RunKind::cfd);
    ASSERT_FALSE(folder.has_value());
    EXPECT_FALSE(fs::exists(runs.path() / "hover")) << "a refused run left its folder";
}

// The case name is UTF-8, on every platform.
TEST(AnalysisRunFolder, ACaseNameIsUtf8) {
    const spade::test::TempDir runs("runs");
    const std::string name = "h\xc3\xb6ver";  // "hover" with an o-umlaut
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), name, resolved(), "2026-10-13", "36f8ca5", an::RunKind::cfd);
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    const std::u8string got = folder->parent_path().filename().u8string();
    EXPECT_EQ(std::string(got.begin(), got.end()), name);
}

// A name that can't be one folder is refused by name, and nothing is made.
TEST(AnalysisRunFolder, RefusesANameDateOrCommitThatCantNameAFolder) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    for (const std::string& bad : {std::string(""), std::string("."), std::string(".."), std::string("a/b"),
                                   std::string("a\\b"), std::string("a:b"), std::string("a?b"), std::string("a*b"),
                                   std::string("a<b"), std::string("a|b"), std::string("tab\there"), std::string("con"),
                                   std::string("NUL.txt"), std::string("Com1"), std::string("lpt9"),
                                   std::string("trail."), std::string("trail "), std::string("\xff\xfe")}) {
        const spade::Result<fs::path> f =
            an::create_run_folder(runs.path(), bad, r, "2026-10-13", "36f8ca5", an::RunKind::cfd);
        ASSERT_FALSE(f.has_value()) << "'" << bad << "' was made: " << f->string();
        EXPECT_EQ(f.error().code, spade::Code::invalid_argument) << bad;
        EXPECT_NE(f.error().context.find("case name"), std::string::npos) << f.error().context;
    }
    for (const std::string& date : {"2026/10/13", "20261013", "2026-10-1", "../10-13", ""}) {
        const spade::Result<fs::path> f =
            an::create_run_folder(runs.path(), "hover", r, date, "36f8ca5", an::RunKind::cfd);
        ASSERT_FALSE(f.has_value()) << date;
        EXPECT_NE(f.error().context.find("date"), std::string::npos) << f.error().context;
    }
    for (const std::string& commit : {"", "../x", "36F8CA5", "36f8ca5 ", "unknown"}) {
        const spade::Result<fs::path> f =
            an::create_run_folder(runs.path(), "hover", r, "2026-10-13", commit, an::RunKind::cfd);
        ASSERT_FALSE(f.has_value()) << commit;
        EXPECT_NE(f.error().context.find("commit"), std::string::npos) << f.error().context;
    }
    std::size_t made = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(runs.path())) {
        (void)e;
        ++made;
    }
    EXPECT_EQ(made, 0u) << "a refused name left something behind";
}

constexpr std::string_view kBuiltCommit = "0123456789abcdef0123456789abcdef01234567";

// A build's identity names the run: the folder at its commit's first 12.
TEST(AnalysisRunFolder, ABuildsIdentityNamesTheRun) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", r, "2026-10-13", spade::BuildIdentity{kBuiltCommit, false}, an::RunKind::cfd);
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    EXPECT_EQ(*folder, an::run_folder(runs.path(), "hover", r, "2026-10-13", kBuiltCommit, an::RunKind::cfd));
    EXPECT_TRUE(folder->filename().string().ends_with("_0123456789ab")) << folder->filename().string();
    EXPECT_TRUE(fs::exists(*folder / "case.yaml"));
}

// A build with uncommitted changes is marked as git describe marks it, so its
// runs are never taken for the clean commit's.
TEST(AnalysisRunFolder, ADirtyBuildsRunIsMarkedDirty) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", r, "2026-10-13", spade::BuildIdentity{kBuiltCommit, true}, an::RunKind::cfd);
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    const fs::path clean = an::run_folder(runs.path(), "hover", r, "2026-10-13", kBuiltCommit, an::RunKind::cfd);
    EXPECT_EQ(folder->parent_path(), clean.parent_path());
    EXPECT_EQ(folder->filename().string(), clean.filename().string() + "-dirty");
    EXPECT_TRUE(fs::exists(*folder / "case.yaml"));
    EXPECT_FALSE(fs::exists(clean));
}

// A build that cannot name its commit is refused: its runs could not be traced.
TEST(AnalysisRunFolder, AnUnknownCommitIsRefused) {
    const spade::test::TempDir runs("runs");
    const spade::Result<fs::path> folder =
        an::create_run_folder(runs.path(), "hover", resolved(), "2026-10-13", spade::BuildIdentity{"unknown", false}, an::RunKind::cfd);
    ASSERT_FALSE(folder.has_value());
    EXPECT_EQ(folder.error().code, spade::Code::unavailable);
    EXPECT_NE(folder.error().context.find("can't name its commit"), std::string::npos) << folder.error().context;
    EXPECT_NE(folder.error().context.find("pass the commit"), std::string::npos) << folder.error().context;
    EXPECT_FALSE(fs::exists(runs.path() / "hover"));
}

// Without a commit, a run is named by the build this engine is
// (core/build_identity.hpp), so a host needs no git.
TEST(AnalysisRunFolder, WithoutACommitIsNamedByTheBuildsCommit) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::BuildIdentity built = spade::build_identity();
    const spade::Result<fs::path> folder = an::create_run_folder(runs.path(), "hover", r, "2026-10-13", an::RunKind::cfd);
    if (built.commit == "unknown") {
        ASSERT_FALSE(folder.has_value());
        EXPECT_EQ(folder.error().code, spade::Code::unavailable);
        return;
    }
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    const std::string clean =
        an::run_folder(runs.path(), "hover", r, "2026-10-13", built.commit, an::RunKind::cfd).filename().string();
    EXPECT_EQ(folder->filename().string(), built.dirty ? clean + "-dirty" : clean) << "commit " << built.commit;
    EXPECT_TRUE(fs::exists(*folder / "case.yaml"));
}
