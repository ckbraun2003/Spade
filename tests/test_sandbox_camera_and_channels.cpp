// Plan C task C2: the two pieces of the window that are NOT the window.
//
// ⭐⭐⭐ THIS FILE EXISTS BECAUSE OF WHAT CANNOT BE TESTED. A windowed sink can
// only be executed where there is a display, and the gate has none under any
// option -- plan-c-sandbox.md states it plainly: "proven cross-platform for a
// WINDOWED path can only mean COMPILED AND LINKED on both, never EXECUTED on
// both." So the answer is not to test the window: it is to leave as little
// inside the window as possible and assert THAT.
//
// Two subjects, both reachable with no display in existence:
//
//   A. bgrx_to_channels -- the ONE channel-order statement both sinks call.
//   B. OrbitCamera      -- every clamp, the spherical conversion and the WASD
//                          basis, which would otherwise be checkable only by a
//                          human looking at a screen.
//
// 🔴 AND (A) IS L301's DISCHARGE, WHICH IS WHY IT IS SHAPED THE WAY IT IS.
// That row records that the C1 suite's oracle is a byte-identical restatement
// of the conversion under test, so a swapped R/B passes all four cases. The
// fix it names is "a KNOWN-ANSWER case that renders a deliberately non-grey
// fixture and asserts the emitted triple DIRECTLY, turning an oracle that can
// only agree with itself into one that can be wrong."
//
// ⚠⚠ SO THE EXPECTATIONS BELOW ARE LITERALS. Not a loop, not a helper, not a
// second expression of the mapping -- bytes written out by hand. That is the
// entire point: an oracle derived from the code under test cannot contradict
// it, and this has already been paid for (an airframe's arithmetic once held
// to the last digit in two files FOR OPPOSITE REASONS).
//
// ⭐ NAMING WHICH ARM PROVES WHICH, per the standing bar minted after a
// production mutation arm FAILED GREEN elsewhere in this estate:
//   * KnownAnswer...        proves ABOUTNESS -- it fails for the channel order
//                           specifically, because it knows the answer already.
//   * ...FloorSourceChannels proves the fixture is CAPABLE of exposing a swap
//                           before any value is compared.
//   * ...RejectsASwappedOracle proves the assertion can FAIL at all (liveness).
//   * The camera's DiscriminatesBetweenPoses arm proves the camera tests are
//     not passing on a stuck value.
// LIVENESS IS NOT ABOUTNESS, and each arm above is labelled with which one it
// is rather than being left for a reader to assume.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/target.hpp"

#include "orbit_camera.hpp"
#include "target_sink.hpp"

using spade::render::PixelFormat;
using spade::render::RenderTarget;
using spade::sandbox::bgrx_to_channels;
using spade::sandbox::FrameInput;
using spade::sandbox::OrbitCamera;

namespace {

// TWO PIXELS, DELIBERATELY NOT GREY, and no two channels equal inside either.
//
// ⚠ The C1 fixture is near-grey (base_color {0.8, 0.75, 0.7}, ~13/255 apart
// before shading), which is why its anti-vacuity floor -- real, and correct on
// the PIXEL axis -- said nothing about the CHANNEL axis. AN ANTI-VACUITY FLOOR
// HAS AN AXIS, and a floor on one axis reads as coverage on all of them.
//
// The X bytes DIFFER between the two pixels (0x00 and 0xFF) on purpose: that
// is what makes "alpha is FORCED to 255" distinguishable from "alpha is COPIED
// from X". With equal X values the two behaviours are the same bytes.
constexpr uint8_t kBgrx[] = {
    /* pixel 0: B    G     R     X */ 0x10, 0x80, 0xF0, 0x00,
    /* pixel 1: B    G     R     X */ 0xC0, 0x20, 0x40, 0xFF,
};

// WRITTEN OUT BY HAND. If you are tempted to generate these, re-read L301.
constexpr uint8_t kExpectedRgb[] = {0xF0, 0x80, 0x10, 0x40, 0x20, 0xC0};
constexpr uint8_t kExpectedRgba[] = {0xF0, 0x80, 0x10, 0xFF, 0x40, 0x20, 0xC0, 0xFF};

// What a SWAPPED-channel sink would emit. Also by hand, and it is the reason
// the known-answer assertion is not vacuous: the two literals differ.
constexpr uint8_t kSwappedRgb[] = {0x10, 0x80, 0xF0, 0xC0, 0x20, 0x40};

[[nodiscard]] RenderTarget two_pixel_target(std::vector<uint8_t>& storage) {
    storage.assign(std::begin(kBgrx), std::end(kBgrx));
    return RenderTarget{
        .pixels = storage,
        .width = 2,
        .height = 1,
        .stride = 2 * 4u,
        .format = PixelFormat::bgrx8,
    };
}

}  // namespace

// --------------------------------------------------------------------------
// A. The channel order, asserted against an answer known in advance.
// --------------------------------------------------------------------------

TEST(SandboxChannels, KnownAnswerRgbDischargesL301Aboutness) {
    std::vector<uint8_t> storage;
    const RenderTarget target = two_pixel_target(storage);

    std::vector<uint8_t> rgb;
    bgrx_to_channels<3u>(target, rgb);

    const std::vector<uint8_t> expected(std::begin(kExpectedRgb), std::end(kExpectedRgb));
    EXPECT_EQ(rgb, expected)
        << "the BGRX->RGB conversion did not produce the triple this test knew in advance. "
           "Unlike the C1 oracle, this expectation is not derived from the conversion, so it can "
           "contradict it -- which is what L301 was opened for";
}

TEST(SandboxChannels, KnownAnswerRgbaForcesOpaqueAlphaRatherThanCopyingX) {
    std::vector<uint8_t> storage;
    const RenderTarget target = two_pixel_target(storage);

    std::vector<uint8_t> rgba;
    bgrx_to_channels<4u>(target, rgba);

    const std::vector<uint8_t> expected(std::begin(kExpectedRgba), std::end(kExpectedRgba));
    EXPECT_EQ(rgba, expected)
        << "the BGRX->RGBA conversion (the windowed sink's path) is wrong";

    // ABOUTNESS for the alpha rule specifically: pixel 0's X byte is 0x00, so
    // a conversion that COPIED X would put 0x00 here. X is padding, not alpha
    // (PA-1), and a texture that inherited it would blend against the clear
    // colour and read as a renderer bug.
    ASSERT_GE(rgba.size(), 4u);
    EXPECT_EQ(rgba[3], 0xFFu) << "alpha was copied from the undefined X byte instead of forced";
}

TEST(SandboxChannels, FloorTheSourceChannelsActuallyDifferSoASwapWouldShow) {
    // THE FLOOR L301 NAMES, AND IT IS ON THE CHANNEL AXIS. Asserted BEFORE any
    // comparison, because a fixture whose channels are nearly equal makes every
    // assertion above pass under a swap without anything noticing.
    for (size_t p = 0; p < 2; ++p) {
        const uint8_t b = kBgrx[p * 4 + 0];
        const uint8_t g = kBgrx[p * 4 + 1];
        const uint8_t r = kBgrx[p * 4 + 2];
        EXPECT_NE(r, b) << "pixel " << p << ": R and B are equal, so an R/B swap is undetectable";
        EXPECT_NE(r, g) << "pixel " << p << ": R and G are equal";
        EXPECT_NE(g, b) << "pixel " << p << ": G and B are equal";
        // And a real separation, not merely inequality: one LSB apart would
        // satisfy the assertions above and survive any rounding.
        EXPECT_GE(std::abs(static_cast<int>(r) - static_cast<int>(b)), 32)
            << "pixel " << p << ": R and B are too close for a swap to be visible";
    }
}

TEST(SandboxChannels, LivenessTheKnownAnswerAssertionRejectsASwappedOracle) {
    // LIVENESS, and it is doing a different job from the arms above. They
    // assert the conversion is RIGHT; this asserts the comparison can be WRONG.
    // Without it, "EXPECT_EQ(rgb, expected)" passing would be consistent with
    // both literals accidentally being the same bytes.
    const std::vector<uint8_t> expected(std::begin(kExpectedRgb), std::end(kExpectedRgb));
    const std::vector<uint8_t> swapped(std::begin(kSwappedRgb), std::end(kSwappedRgb));
    ASSERT_EQ(expected.size(), swapped.size())
        << "the two oracles must differ in CONTENT, not in size";
    EXPECT_NE(expected, swapped)
        << "the correct and R/B-swapped expectations are identical, so the known-answer test "
           "cannot tell them apart and proves nothing";
}

TEST(SandboxChannels, BothWidthsAgreeOnRgbWhichIsWhyThereIsOneStatement) {
    // The windowed and headless sinks must not drift apart. They cannot, because
    // there is one statement -- and this is what makes that structural claim
    // checkable rather than a comment.
    std::vector<uint8_t> storage;
    const RenderTarget target = two_pixel_target(storage);
    std::vector<uint8_t> rgb, rgba;
    bgrx_to_channels<3u>(target, rgb);
    bgrx_to_channels<4u>(target, rgba);

    ASSERT_EQ(rgb.size(), 6u);
    ASSERT_EQ(rgba.size(), 8u);
    for (size_t i = 0; i < 2; ++i) {
        EXPECT_EQ(rgb[i * 3 + 0], rgba[i * 4 + 0]) << "R disagrees at pixel " << i;
        EXPECT_EQ(rgb[i * 3 + 1], rgba[i * 4 + 1]) << "G disagrees at pixel " << i;
        EXPECT_EQ(rgb[i * 3 + 2], rgba[i * 4 + 2]) << "B disagrees at pixel " << i;
    }
}

// --------------------------------------------------------------------------
// B. The camera. Every one of these would otherwise need a display and a human.
// --------------------------------------------------------------------------

namespace {
[[nodiscard]] bool finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
[[nodiscard]] bool finite(const glm::quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}
}  // namespace

TEST(SandboxOrbitCamera, ThePitchClampIsACorrectnessBoundNotAFeelTweak) {
    // ⚠⚠ AT EXACTLY +/- PI/2 THE VIEW DIRECTION IS PARALLEL TO `up` AND
    // quatLookAt RETURNS A NaN QUATERNION -- every subsequent frame is garbage,
    // not merely oddly framed. This arm is why the clamp is a named constant.
    OrbitCamera cam;
    FrameInput in;
    in.orbit_dy = 100000.0f;  // far past the pole, in one frame
    cam.apply(in, 1.0f / 60.0f);

    EXPECT_LE(cam.pitch, OrbitCamera::kPitchLimit);
    EXPECT_GE(cam.pitch, -OrbitCamera::kPitchLimit);

    const spade::render::Camera rc = cam.to_render_camera();
    EXPECT_TRUE(finite(rc.position)) << "camera position is not finite at the pitch limit";
    EXPECT_TRUE(finite(rc.orientation)) << "camera orientation is NaN at the pitch limit -- the "
                                           "clamp did not protect quatLookAt";
    EXPECT_GT(glm::length(rc.orientation), 0.0f) << "degenerate (zero-length) orientation";

    in.orbit_dy = -100000.0f;
    cam.apply(in, 1.0f / 60.0f);
    EXPECT_GE(cam.pitch, -OrbitCamera::kPitchLimit);
    EXPECT_TRUE(finite(cam.to_render_camera().orientation)) << "NaN at the NEGATIVE pitch limit -- "
                                                               "both poles need the bound, and only "
                                                               "testing one is how half a clamp ships";
}

TEST(SandboxOrbitCamera, TheDollyIsBoundedAtBothEndsAndNeverReachesZero) {
    OrbitCamera cam;
    FrameInput in;
    in.dolly = 1000.0f;  // scroll toward the target, hard
    cam.apply(in, 1.0f / 60.0f);
    EXPECT_GE(cam.distance, OrbitCamera::kMinDistance)
        << "the rig passed through its own target, which inverts the view with no way back";
    EXPECT_GT(cam.distance, 0.0f);

    in.dolly = -1000.0f;
    cam.apply(in, 1.0f / 60.0f);
    EXPECT_LE(cam.distance, OrbitCamera::kMaxDistance)
        << "the rig flew far enough away that the scene is sub-pixel, which reads as a renderer "
           "drawing nothing";
}

TEST(SandboxOrbitCamera, TheCameraActuallyLooksAtItsTarget) {
    // ABOUTNESS for the whole rig: the orientation must carry local -Z onto the
    // direction from the eye to the target. This is the engine's stated
    // convention ("camera space is right-handed with forward = -Z",
    // raster_cpu.cpp) and it is the one thing a wrong quatLookAt would break
    // while leaving every other assertion here green.
    OrbitCamera cam;
    cam.yaw = 0.9f;
    cam.pitch = 0.4f;
    cam.distance = 6.0f;

    const spade::render::Camera rc = cam.to_render_camera();
    const glm::vec3 forward = rc.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
    const glm::vec3 to_target = glm::normalize(cam.target - rc.position);

    EXPECT_NEAR(forward.x, to_target.x, 1e-4f);
    EXPECT_NEAR(forward.y, to_target.y, 1e-4f);
    EXPECT_NEAR(forward.z, to_target.z, 1e-4f);
}

TEST(SandboxOrbitCamera, DiscriminatesBetweenPosesSoTheArmsAboveCannotPassOnAStuckRig) {
    // LIVENESS for the camera arms. If OrbitCamera ignored its input entirely,
    // every assertion above would still pass -- clamps hold, the camera looks
    // at its target, nothing is NaN. This is what makes them mean something.
    OrbitCamera a;
    OrbitCamera b;
    FrameInput in;
    in.orbit_dx = 120.0f;
    b.apply(in, 1.0f / 60.0f);

    EXPECT_NE(a.yaw, b.yaw) << "a drag changed nothing -- the rig ignores its input";
    const glm::vec3 pa = a.to_render_camera().position;
    const glm::vec3 pb = b.to_render_camera().position;
    EXPECT_GT(glm::length(pa - pb), 1e-3f) << "two different yaws produced the same eye position";

    // And determinism, which the seam's own SL10 arm asserts for frames: the
    // same rig state must produce the same camera twice.
    const glm::vec3 pa2 = a.to_render_camera().position;
    EXPECT_EQ(pa, pa2) << "to_render_camera is not a pure function of the rig";
}

TEST(SandboxOrbitCamera, PanMovesAlongTheGroundAndNeverTiltsTheTarget) {
    // W/S and A/D move the TARGET in the ground plane. A basis derived from the
    // 3D forward instead of from yaw alone drifts vertically as the pitch
    // steepens, which is the defect this arm exists to catch.
    OrbitCamera cam;
    cam.pitch = 1.2f;  // steep, where a bad basis shows up
    const float y_before = cam.target.y;

    FrameInput in;
    in.move_forward = 1.0f;
    cam.apply(in, 0.5f);
    EXPECT_FLOAT_EQ(cam.target.y, y_before)
        << "panning forward changed the target's HEIGHT -- the ground basis is not flat";

    in.move_forward = 0.0f;
    in.move_right = 1.0f;
    cam.apply(in, 0.5f);
    EXPECT_FLOAT_EQ(cam.target.y, y_before) << "panning sideways changed the target's height";

    // Q/E is the one that is SUPPOSED to move it, so this pair proves the arms
    // above are measuring the basis rather than a target that never moves.
    in.move_right = 0.0f;
    in.move_up = 1.0f;
    cam.apply(in, 0.5f);
    EXPECT_GT(cam.target.y, y_before) << "Q/E did not move the target vertically, so the height "
                                         "assertions above pass on a target that never moves at all";
}

// --------------------------------------------------------------------------
// The drone sim box's orbit (apply_drone_orbit): the target is the drone and
// never moves, and the keys that pan in the builder orbit instead.
// --------------------------------------------------------------------------

TEST(SandboxDroneOrbit, HoldingWOrSForTenSecondsStaysOnTheViewSphere) {
    OrbitCamera cam;
    cam.distance = 2.0f;
    FrameInput in;
    in.move_forward = 1.0f;  // W
    for (int i = 0; i < 600; ++i) {
        spade::sandbox::apply_drone_orbit(cam, in, 1.0f / 60.0f);
        ASSERT_GE(cam.distance, spade::sandbox::kDroneMinDistance) << "frame " << i;
    }
    EXPECT_FLOAT_EQ(cam.distance, spade::sandbox::kDroneMinDistance) << "W never reached the near limit";

    in.move_forward = -1.0f;  // S
    for (int i = 0; i < 600; ++i) {
        spade::sandbox::apply_drone_orbit(cam, in, 1.0f / 60.0f);
        ASSERT_LE(cam.distance, spade::sandbox::kDroneMaxDistance) << "frame " << i;
    }
    EXPECT_FLOAT_EQ(cam.distance, spade::sandbox::kDroneMaxDistance) << "S never reached the far limit";

    in.move_forward = 0.0f;
    in.dolly = 1000.0f;  // the scroll wheel is clamped to the same sphere
    spade::sandbox::apply_drone_orbit(cam, in, 1.0f / 60.0f);
    EXPECT_GE(cam.distance, spade::sandbox::kDroneMinDistance);
}

TEST(SandboxDroneOrbit, TheKeysOrbitAndTheCameraAlwaysFacesTheDrone) {
    OrbitCamera cam;
    cam.target = glm::vec3(0.0f);
    cam.distance = 2.0f;
    const OrbitCamera start = cam;

    FrameInput in;
    in.move_right = 1.0f;  // D: around
    in.move_up = 1.0f;     // E: over
    in.orbit_dx = 30.0f;   // and a mouse drag on top
    for (int i = 0; i < 120; ++i) {
        spade::sandbox::apply_drone_orbit(cam, in, 1.0f / 60.0f);
        EXPECT_EQ(cam.target, glm::vec3(0.0f)) << "the drone orbit moved its target";
        const spade::render::Camera rc = cam.to_render_camera();
        ASSERT_TRUE(finite(rc.orientation)) << "frame " << i;
        const glm::vec3 forward = rc.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
        const glm::vec3 to_drone = glm::normalize(cam.target - rc.position);
        ASSERT_GT(glm::dot(forward, to_drone), 0.9999f) << "frame " << i;
    }
    EXPECT_NE(cam.yaw, start.yaw) << "A/D did not orbit";
    EXPECT_GT(cam.pitch, start.pitch) << "E did not raise the camera";
    EXPECT_LE(cam.pitch, OrbitCamera::kPitchLimit);
}

TEST(SandboxDroneOrbit, DCarriesTheEyeToTheCamerasRight) {
    OrbitCamera cam;
    cam.target = glm::vec3(0.0f);
    cam.distance = 2.0f;
    const glm::vec3 right = cam.ground_right();
    const glm::vec3 before = cam.eye_position();
    FrameInput in;
    in.move_right = 1.0f;
    spade::sandbox::apply_drone_orbit(cam, in, 0.1f);
    EXPECT_GT(glm::dot(cam.eye_position() - before, right), 0.0f);
}
