// SL10's same-path invariant across Plan C task C1's TargetSink seam.
//
// SL10: the frame the sandbox presents is the frame spade::render produced for
// the same (scene, camera, options).
//
// ⚠⚠ WHY THIS FILE EXISTS AT ALL, AND WHY ITS SHAPE IS NOT THE OBVIOUS ONE.
// Before C1 that invariant was true BY CONSTRUCTION: spade_sandbox rendered
// into a buffer and wrote its PPM from THAT SAME BUFFER, so there was no seam
// for anything to diverge across. C1 introduces the first real opportunity for
// divergence, and this file is what must notice it.
//
// ⛔ THE OBVIOUS TEST CANNOT FAIL. A sink that holds a REFERENCE to the
// caller's buffer makes "what the sink got" and "what render produced"
// trivially equal, and the assertion then passes for ANY sink, correct or not.
// That is two copies that move together reported as agreement -- this realm's
// competition-700 finding, where the same arithmetic held to the last digit in
// two files FOR OPPOSITE REASONS and could not fail in either.
//
// So the subject here is not the pointer. It is WHAT THE SANDBOX EMITS: the
// PPM it writes. That round-trips through a real channel reorder and a real
// file format, both of which can be wrong.
//
// ⚠ AND THE CHANNEL MAPPING IS NECESSARILY RESTATED BY ANY TEST OF IT -- a
// pure transform can only be checked by expressing the transform. That is why
// the MUTATION CONTROL below is not optional: it is the only thing that proves
// these assertions are capable of failing at all. Two further cases assert
// properties that are NOT restatements (determinism, and discrimination
// between different frames), so the file does not rest on the mapping alone.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "target_sink.hpp"

using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::sandbox::HeadlessTargetSink;
using spade::sandbox::TargetSink;

namespace {

constexpr uint32_t kW = 96, kH = 64;

[[nodiscard]] RenderScene scene_or_fail(const WorldDesc& world) {
    const spade::Result<RenderScene> scene = spade::render::scene_from_world(world, {});
    EXPECT_TRUE(scene.has_value());
    return scene.has_value() ? *scene : RenderScene{};
}

[[nodiscard]] WorldDesc plane_world(float offset) {
    // `offset` shifts the ground plane so two calls give genuinely DIFFERENT
    // frames -- the discrimination case below needs a second frame that is not
    // merely a second copy of the first.
    WorldBuilder b;
    b.name("sink_fixture")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{4, 4, 1, 1})
        .material(spade::MaterialDesc{.name = "ground", .base_color = {0.8f, 0.75f, 0.7f, 1.0f}})
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), offset);
    const spade::Result<WorldDesc> world = b.build();
    EXPECT_TRUE(world.has_value());
    return world.has_value() ? *world : WorldDesc{};
}

[[nodiscard]] Camera fixture_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 3.0f, 5.0f);
    camera.orientation = glm::angleAxis(glm::radians(-20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

// Renders into `storage` and hands the SAME target to `sink`, which is exactly
// what spade_sandbox's main() does -- the production path, not a re-creation
// of it.
void render_through(TargetSink& sink, const RenderScene& scene, std::vector<uint8_t>& storage) {
    storage.assign(static_cast<size_t>(kW) * kH * 4u, 0u);
    RenderTarget target{
        .pixels = storage,
        .width = kW,
        .height = kH,
        .stride = kW * 4u,
        .format = PixelFormat::bgrx8,
    };
    RenderOptions options;
    options.mode = DrawMode::shaded;
    const spade::Result<void> ok = spade::render::render(scene, fixture_camera(), options, target);
    ASSERT_TRUE(ok.has_value());
    sink.accept(target);
}

// THE MUTATION CONTROL, and it is the reason the assertions above it mean
// anything. One byte, deterministically chosen. If the same-path assertion
// still passes against this, the assertion is measuring nothing.
class OneByteWrongSink final : public TargetSink {
  public:
    void accept(const spade::render::RenderTarget& target) override {
        inner_.accept(target);
        rgb_ = inner_.emitted_rgb();
        ASSERT_FALSE(rgb_.empty());
        rgb_[rgb_.size() / 2] = static_cast<uint8_t>(rgb_[rgb_.size() / 2] ^ 0x01u);
    }
    [[nodiscard]] const std::vector<uint8_t>& emitted_rgb() const noexcept { return rgb_; }

  private:
    HeadlessTargetSink inner_{std::string{}};
    std::vector<uint8_t> rgb_;
};

// The expected RGB for a BGRX8 buffer. Necessarily a restatement of the
// mapping under test -- see this file's header. Its job is to be a fixed
// reference the mutation control can be shown to violate.
[[nodiscard]] std::vector<uint8_t> expected_rgb(const std::vector<uint8_t>& bgrx) {
    std::vector<uint8_t> out(bgrx.size() / 4 * 3);
    for (size_t i = 0, n = bgrx.size() / 4; i < n; ++i) {
        out[i * 3 + 0] = bgrx[i * 4 + 2];
        out[i * 3 + 1] = bgrx[i * 4 + 1];
        out[i * 3 + 2] = bgrx[i * 4 + 0];
    }
    return out;
}

}  // namespace

TEST(SandboxTargetSink, WhatTheSandboxEmitsIsTheFrameRenderProducedRulingSL10) {
    const WorldDesc world = plane_world(0.0f);
    const RenderScene scene = scene_or_fail(world);
    HeadlessTargetSink sink{std::string{}};
    std::vector<uint8_t> rendered;
    render_through(sink, scene, rendered);

    EXPECT_EQ(sink.accepted(), 1u) << "the sink must have been handed exactly one frame";
    EXPECT_FALSE(sink.failed());
    EXPECT_EQ(sink.width(), kW);
    EXPECT_EQ(sink.height(), kH);
    EXPECT_EQ(sink.emitted_rgb(), expected_rgb(rendered))
        << "the frame the sandbox emitted is not the frame spade::render produced";

    // Sanity floor: a frame of one flat colour would satisfy the comparison
    // above while proving nothing about a renderer. This fixture must actually
    // contain more than one distinct pixel value.
    const std::vector<uint8_t>& out = sink.emitted_rgb();
    ASSERT_GE(out.size(), 6u);
    bool varies = false;
    for (size_t i = 3; i < out.size(); i += 3) {
        if (out[i] != out[0] || out[i + 1] != out[1] || out[i + 2] != out[2]) {
            varies = true;
            break;
        }
    }
    EXPECT_TRUE(varies) << "sanity floor: this fixture rendered a single flat colour, so the comparison "
                            "above could not distinguish a correct sink from a constant one";
}

TEST(SandboxTargetSink, TheSamePathAssertionIsCapableOfFailingMutationControl) {
    // ⚠⚠ WITHOUT THIS CASE THE ONE ABOVE IS UNFALSIFIED. It asserts a property;
    // this asserts that the property's instrument can detect a violation. A
    // single flipped byte in the emitted frame MUST be caught.
    const WorldDesc world = plane_world(0.0f);
    const RenderScene scene = scene_or_fail(world);
    OneByteWrongSink wrong;
    std::vector<uint8_t> rendered;
    render_through(wrong, scene, rendered);

    const std::vector<uint8_t> reference = expected_rgb(rendered);
    ASSERT_EQ(wrong.emitted_rgb().size(), reference.size())
        << "the control must differ in CONTENT, not in size -- a size difference would be caught by a "
           "weaker assertion and would not exercise the comparison at all";
    EXPECT_NE(wrong.emitted_rgb(), reference)
        << "a sink that corrupted one byte was reported as agreeing with the rendered frame -- the "
           "same-path assertion above cannot fail and is measuring nothing";
}

TEST(SandboxTargetSink, TheSeamIsDeterministicAndDiscriminatesBetweenDifferentFrames) {
    // Two properties that are NOT restatements of the channel mapping, so this
    // file does not rest on that mapping alone.
    const WorldDesc world_a = plane_world(0.0f);
    const WorldDesc world_b = plane_world(-1.5f);
    const RenderScene scene_a = scene_or_fail(world_a);
    const RenderScene scene_b = scene_or_fail(world_b);

    HeadlessTargetSink first{std::string{}}, again{std::string{}}, other{std::string{}};
    std::vector<uint8_t> a1, a2, b1;
    render_through(first, scene_a, a1);
    render_through(again, scene_a, a2);
    render_through(other, scene_b, b1);

    EXPECT_EQ(first.emitted_rgb(), again.emitted_rgb())
        << "the same scene through the seam twice produced different bytes -- the seam is not deterministic";
    EXPECT_NE(first.emitted_rgb(), other.emitted_rgb())
        << "two DIFFERENT scenes produced identical bytes through the seam, so the equality above is "
           "satisfied by a sink that ignores its input";
}

TEST(SandboxTargetSink, TheWrittenPpmRoundTripsToTheEmittedFrame) {
    // The file is the artifact the sandbox actually hands a person, so it is
    // the thing SL10 is ultimately about. Exercises the header format and the
    // write path, neither of which the in-memory cases touch.
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "spade_target_sink_roundtrip.ppm";
    std::error_code ec;
    std::filesystem::remove(out, ec);

    const WorldDesc world = plane_world(0.0f);
    const RenderScene scene = scene_or_fail(world);
    HeadlessTargetSink sink{out.string()};
    std::vector<uint8_t> rendered;
    render_through(sink, scene, rendered);
    ASSERT_FALSE(sink.failed()) << "the sink could not write " << out.string();
    ASSERT_TRUE(std::filesystem::exists(out));

    // Streams, not stdio: this target is /W4 /WX and MSVC's C4996 makes
    // fopen/fscanf hard errors here -- see target_sink.hpp's own note.
    std::ifstream in(out, std::ios::binary);
    ASSERT_TRUE(in.good());
    std::string magic;
    uint32_t w = 0, h = 0, maxval = 0;
    in >> magic >> w >> h >> maxval;
    ASSERT_TRUE(in.good());
    in.get();  // the single whitespace byte after maxval, before the payload
    EXPECT_EQ(magic, "P6");
    EXPECT_EQ(w, kW);
    EXPECT_EQ(h, kH);
    EXPECT_EQ(maxval, 255u);
    std::vector<uint8_t> payload(static_cast<size_t>(w) * h * 3u, 0u);
    in.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    ASSERT_EQ(static_cast<size_t>(in.gcount()), payload.size());

    EXPECT_EQ(payload, sink.emitted_rgb()) << "the bytes on disk are not the frame the sink emitted";
    std::filesystem::remove(out, ec);
}
