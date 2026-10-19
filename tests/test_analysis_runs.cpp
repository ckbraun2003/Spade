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
    const fs::path folder =
        an::run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca59f717c9545373d59c8cd2764b38a6b021");
    char hash[17];
    std::snprintf(hash, sizeof hash, "%016llx", static_cast<unsigned long long>(r.hash));
    EXPECT_EQ(folder, runs.path() / "hover" / ("2026-10-13_" + std::string(hash) + "_36f8ca59f717"));
}

TEST(AnalysisRunFolder, StartsQueuedWithTheCaseItRuns) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const spade::Result<fs::path> folder = an::create_run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca5");
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    EXPECT_EQ(read_all(*folder / "case.yaml"), an::case_to_yaml(r.spec).value_or("-"));
    const spade::Result<an::RunProgress> p = an::read_progress(*folder);
    ASSERT_TRUE(p.has_value()) << p.error().context;
    EXPECT_EQ(p->state, an::RunState::queued);
    EXPECT_EQ(p->target_s, static_cast<double>(r.spec.time.duration));
    // A second run of the same case at the same commit and date is the same
    // folder: refused, not overwritten.
    const spade::Result<fs::path> again = an::create_run_folder(runs.path(), "hover", r, "2026-10-13", "36f8ca5");
    ASSERT_FALSE(again.has_value());
    EXPECT_NE(again.error().context.find("exists"), std::string::npos) << again.error().context;
}

// Without a commit, a run is named by the one this engine was built from
// (core/build_identity.hpp), so a host needs no git.
TEST(AnalysisRunFolder, WithoutACommitIsNamedByTheBuildsCommit) {
    const spade::test::TempDir runs("runs");
    const an::ResolvedCase r = resolved();
    const std::string_view commit = spade::build_identity().commit;
    ASSERT_FALSE(commit.empty());
    const spade::Result<fs::path> folder = an::create_run_folder(runs.path(), "hover", r, "2026-10-13");
    ASSERT_TRUE(folder.has_value()) << folder.error().context;
    const std::string name = folder->filename().string();
    EXPECT_TRUE(name.ends_with("_" + std::string(commit.substr(0, 12)))) << name << ", commit " << commit;
    EXPECT_EQ(*folder, an::run_folder(runs.path(), "hover", r, "2026-10-13", commit));
    EXPECT_TRUE(fs::exists(*folder / "case.yaml"));
}
