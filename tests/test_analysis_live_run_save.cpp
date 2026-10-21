// LiveRun's save and restore (approved by the lead, 2026-10-13): a long run
// (a D1 film, a queued run) stops and resumes in foreground chunks, and a
// restored run gives exactly the frames an unbroken one does. The file holds
// Physics' grid (u, v, w, p), Core's Simulation snapshot, the hold, the
// counters and the recorded controls, under one checksum. It refuses another
// case, model, propulsion or version, and a corrupted file, by name.

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "core/flow_volume.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

// The quad as a body alone: rotors wait for P7a, and parts for P2.
[[nodiscard]] spade::vehicles::ModelType body_only() {
    const auto s = spade::scene::load_scene_file(fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value());
    spade::vehicles::ModelType m = s ? s->models.at(0) : spade::vehicles::ModelType{};
    m.parts.clear();
    m.rotors.clear();
    return m;
}

// Wind through an 8^3 box, so the fluid has state worth saving.
[[nodiscard]] an::ResolvedCase windy(const spade::vehicles::ModelType& m, float speed = 3.0f) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", m);
    c.grid.box = 0.8f;
    c.grid.spacing = 0.1f;
    c.time.duration = 1.0f;
    c.time.step = 0.004f;
    c.flight.kind = an::FlightKind::forward;
    c.flight.speed = speed;
    c.flight.heading_deg = 20.0f;
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, m);
    EXPECT_TRUE(r.has_value());
    return r ? *r : an::ResolvedCase{};
}

// A turn, so the hold's angles and the recorded controls are state too.
[[nodiscard]] an::ControlProgram turning() {
    return an::ControlProgram{{{0.0f, an::Controls{}},
                               {0.05f, an::Controls{false, 0.0f, 20.0f, 0.0f, 0.0f}},
                               {0.12f, an::Controls{false, 0.0f, 20.0f, -15.0f, 10.0f}}}};
}

// Armed and turning at half throttle, so the rotors' speeds and the air they
// push are state too.
[[nodiscard]] an::ControlProgram powered_turn() {
    return an::ControlProgram{{{0.0f, an::Controls{true, 0.5f, 0.0f, 0.0f, 0.0f}},
                               {0.05f, an::Controls{true, 0.6f, 20.0f, 0.0f, 0.0f}},
                               {0.12f, an::Controls{true, 0.4f, 20.0f, -15.0f, 10.0f}}}};
}

[[nodiscard]] an::LiveRun started(const an::ResolvedCase& r, const spade::vehicles::ModelType& m,
                                  const std::optional<an::LivePropulsion>& propulsion = std::nullopt) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(r, m, propulsion);
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    return std::move(*run);
}

// Flies `frames` frames of `program`, returning each frame's volume.
[[nodiscard]] std::vector<spade::FlowVolume> fly(an::LiveRun& run, int frames,
                                                 const an::ControlProgram& program = turning()) {
    std::vector<spade::FlowVolume> out;
    for (int i = 0; i < frames; ++i) {
        spade::FlowVolume v;
        const spade::Result<an::LiveFrame> f =
            run.advance_frame(an::controls_at(program, static_cast<float>(run.state().t_s)), v);
        EXPECT_TRUE(f.has_value()) << (f ? "" : f.error().context);
        out.push_back(std::move(v));
    }
    return out;
}

void expect_same_frames(const std::vector<spade::FlowVolume>& a, const std::vector<spade::FlowVolume>& b) {
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].v, b[i].v) << "frame " << i;
        EXPECT_EQ(a[i].s, b[i].s) << "frame " << i;
        for (std::size_t c = 0; c < 8; ++c) {
            EXPECT_EQ(a[i].header.channel_min[c], b[i].header.channel_min[c]) << "frame " << i << ", channel " << c;
            EXPECT_EQ(a[i].header.channel_max[c], b[i].header.channel_max[c]) << "frame " << i << ", channel " << c;
        }
    }
}

[[nodiscard]] std::vector<char> bytes_of(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::vector<char>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void write_bytes(const fs::path& file, const std::vector<char>& bytes) {
    std::ofstream(file, std::ios::binary | std::ios::trunc).write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

}  // namespace

// The A/B: stopped after 3 frames, saved and restored, the run gives frames
// 4-6 bit for bit as the unbroken run does, and ends in the same state.
TEST(AnalysisLiveRunSave, ARestoredRunMatchesAnUnbrokenOne) {
    const spade::vehicles::ModelType m = body_only();
    const an::ResolvedCase r = windy(m);
    an::LiveRun unbroken = started(r, m);
    std::vector<spade::FlowVolume> all = fly(unbroken, 6);
    const std::vector<spade::FlowVolume> last(all.begin() + 3, all.end());

    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "run.livesave";
    an::LiveRun first = started(r, m);
    (void)fly(first, 3);
    ASSERT_TRUE(first.save(file).has_value());
    spade::Result<an::LiveRun> resumed = an::LiveRun::restore(file, r, m);
    ASSERT_TRUE(resumed.has_value()) << resumed.error().context;
    EXPECT_EQ(resumed->state().frames, 3u);
    EXPECT_EQ(resumed->state().steps, first.state().steps);
    expect_same_frames(fly(*resumed, 3), last);
    EXPECT_EQ(resumed->state().t_s, unbroken.state().t_s);
    EXPECT_EQ(resumed->state().attitude, unbroken.state().attitude);
    EXPECT_EQ(resumed->state().omega_body, unbroken.state().omega_body);
    EXPECT_EQ(an::program_to_yaml(resumed->recorded_controls()).value_or("a"),
              an::program_to_yaml(unbroken.recorded_controls()).value_or("b"));
}

// Saved before its first step, a run restores to its start.
TEST(AnalysisLiveRunSave, ARunSavedAtItsStartRestoresToIt) {
    const spade::vehicles::ModelType m = body_only();
    const an::ResolvedCase r = windy(m);
    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "start.livesave";
    ASSERT_TRUE(started(r, m).save(file).has_value());
    spade::Result<an::LiveRun> resumed = an::LiveRun::restore(file, r, m);
    ASSERT_TRUE(resumed.has_value()) << resumed.error().context;
    an::LiveRun fresh = started(r, m);
    expect_same_frames(fly(*resumed, 2), fly(fresh, 2));
}

TEST(AnalysisLiveRunSave, RefusesAnotherCaseOrModel) {
    const spade::vehicles::ModelType m = body_only();
    const an::ResolvedCase r = windy(m);
    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "run.livesave";
    an::LiveRun run = started(r, m);
    (void)fly(run, 1);
    ASSERT_TRUE(run.save(file).has_value());
    // Another case: a different wind.
    const spade::Result<an::LiveRun> other_case = an::LiveRun::restore(file, windy(m, 4.0f), m);
    ASSERT_FALSE(other_case.has_value());
    EXPECT_NE(other_case.error().context.find("case"), std::string::npos) << other_case.error().context;
    // Another model under the same case.
    spade::vehicles::ModelType heavier = m;
    heavier.body.mass *= 2.0f;
    const spade::Result<an::LiveRun> other_model = an::LiveRun::restore(file, r, heavier);
    ASSERT_FALSE(other_model.has_value());
    EXPECT_NE(other_model.error().context.find("model"), std::string::npos) << other_model.error().context;
}

TEST(AnalysisLiveRunSave, RefusesACorruptedFileByName) {
    const spade::vehicles::ModelType m = body_only();
    const an::ResolvedCase r = windy(m);
    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "run.livesave";
    an::LiveRun run = started(r, m);
    (void)fly(run, 1);
    ASSERT_TRUE(run.save(file).has_value());
    const std::vector<char> good = bytes_of(file);
    ASSERT_GT(good.size(), 64u);

    // One flipped byte in the payload: the checksum.
    std::vector<char> flipped = good;
    flipped[good.size() / 2] = static_cast<char>(flipped[good.size() / 2] ^ 0x40);
    write_bytes(file, flipped);
    const spade::Result<an::LiveRun> f = an::LiveRun::restore(file, r, m);
    ASSERT_FALSE(f.has_value());
    EXPECT_NE(f.error().context.find("checksum"), std::string::npos) << f.error().context;

    // Cut short.
    write_bytes(file, std::vector<char>(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(good.size() / 3)));
    EXPECT_FALSE(an::LiveRun::restore(file, r, m).has_value());

    // Another version: a format this build doesn't speak (the version follows
    // the 8-byte magic).
    std::vector<char> v2 = good;
    v2[8] = static_cast<char>(v2[8] + 1);
    write_bytes(file, v2);
    const spade::Result<an::LiveRun> v = an::LiveRun::restore(file, r, m);
    ASSERT_FALSE(v.has_value());
    EXPECT_EQ(v.error().code, spade::Code::schema_mismatch) << v.error().context;

    // Not ours at all, and missing.
    write_bytes(file, std::vector<char>(100, 'x'));
    EXPECT_FALSE(an::LiveRun::restore(file, r, m).has_value());
    const spade::Result<an::LiveRun> missing = an::LiveRun::restore(dir.path() / "none.livesave", r, m);
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, spade::Code::io_error);
}

// The A/B with the rotors turning: the restored run's rotors, and the air they
// push, go on exactly as the unbroken run's.
TEST(AnalysisLiveRunSave, ARestoredPoweredRunMatchesAnUnbrokenOne) {
    const an::test::PoweredQuad p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p.model);
    an::LiveRun unbroken = started(r, p.model, p.propulsion);
    std::vector<spade::FlowVolume> all = fly(unbroken, 6, powered_turn());
    const std::vector<spade::FlowVolume> last(all.begin() + 3, all.end());

    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "powered.livesave";
    an::LiveRun first = started(r, p.model, p.propulsion);
    (void)fly(first, 3, powered_turn());
    ASSERT_TRUE(first.save(file).has_value());
    spade::Result<an::LiveRun> resumed = an::LiveRun::restore(file, r, p.model, p.propulsion);
    ASSERT_TRUE(resumed.has_value()) << resumed.error().context;
    EXPECT_EQ(resumed->state().rotor_omega, first.state().rotor_omega);
    expect_same_frames(fly(*resumed, 3, powered_turn()), last);
    EXPECT_EQ(resumed->state().rotor_omega, unbroken.state().rotor_omega);
    EXPECT_EQ(resumed->state().propulsion_flags, unbroken.state().propulsion_flags);
}

// A save pins the propulsion it flew: another chain, idle or state of charge
// is refused by name, and so is none.
TEST(AnalysisLiveRunSave, RefusesAnotherPropulsion) {
    const an::test::PoweredQuad p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p.model);
    const spade::test::TempDir dir("live_save");
    const fs::path file = dir.path() / "powered.livesave";
    an::LiveRun run = started(r, p.model, p.propulsion);
    (void)fly(run, 1, powered_turn());
    ASSERT_TRUE(run.save(file).has_value());

    an::LivePropulsion chain = p.propulsion;
    chain.chain.current_max *= 0.5;
    an::LivePropulsion idle = p.propulsion;
    idle.idle = 0.08;
    an::LivePropulsion soc = p.propulsion;
    soc.soc = 0.5;
    for (const auto& [other, word] : {std::pair{chain, "propulsion chain"}, std::pair{idle, "idle"},
                                      std::pair{soc, "state of charge"}}) {
        const spade::Result<an::LiveRun> refused = an::LiveRun::restore(file, r, p.model, other);
        ASSERT_FALSE(refused.has_value()) << word;
        EXPECT_NE(refused.error().context.find(word), std::string::npos) << refused.error().context;
    }
    EXPECT_FALSE(an::LiveRun::restore(file, r, p.model).has_value()) << "restored without its propulsion";
}
