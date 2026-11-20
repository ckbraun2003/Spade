// test_camera.cpp -- the camera sensor kind's row, stamp, tag, scheduling rule and validation (Phase 3.1 of rendering/camera-sensor-design).
//
// The layout cases are runtime EXPECTs over the facts camera.hpp's static_asserts already pin, so each has a name in `ctest -V` and this file's include is what
// makes the header compile at all (a header nothing includes is not compiled by a build of everything; tests/test_gnss.cpp's opening says the same).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "sensors/camera.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"

namespace sn = spade::sensors;

namespace {

// A valid camera: 640 x 480 pinhole, every substep a frame at divider 4, the carrying body at slot 2.
[[nodiscard]] sn::CameraSensorRow good_row() {
    sn::CameraSensorRow r{};
    r.body_slot = 2;
    r.kind = sn::sensor_kind::camera;
    r.rate_divider = 4;
    r.phase = 0;
    r.mount_pos = {0.1f, 0.0f, -0.05f};
    r.shutter_fraction = 0.5f;
    r.mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    r.lens_kind = sn::lens_kind::pinhole;
    r.width = 640;
    r.height = 480;
    r.mode = sn::camera_mode::colour;
    r.fx = 500.0f;
    r.fy = 500.0f;
    r.cx = 320.0f;
    r.cy = 240.0f;
    r.blur_samples = 1;
    r.aa_samples = 1;
    r.exposure_gain = 1.0f;
    r.cull_mask = 0xFBu;  // every group but the carrier's own bit (bit 2)
    r.channels = 0;
    return r;
}

}  // namespace

TEST(CameraRow, LayoutIsTheStd430ShapeRenderingAndTheSnapshotSee) {
    EXPECT_EQ(sizeof(sn::CameraSensorRow), 160u);
    EXPECT_EQ(alignof(sn::CameraSensorRow), 16u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, mount_pos), 16u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, mount_orient), 32u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, lens_kind), 48u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, fx), 64u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, k4), 92u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, cull_mask), 108u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, channels), 112u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, noise), 120u);
    EXPECT_EQ(offsetof(sn::CameraSensorRow, last_index), 136u);
    EXPECT_EQ(sizeof(sn::CameraStamp), 96u);
    EXPECT_EQ(offsetof(sn::CameraStamp, flags), 28u);
    EXPECT_EQ(offsetof(sn::CameraStamp, position), 32u);
    EXPECT_EQ(offsetof(sn::CameraStamp, orientation), 80u);
}

TEST(CameraKind, IsTheThirdKindAndItsTagIsItsOwn) {
    EXPECT_EQ(sn::sensor_kind::camera, 3u);
    EXPECT_EQ(sn::sensor_kind::count, 4u) << "the bound moves with the kind";
    EXPECT_NE(sn::sensor_kind::camera, sn::sensor_kind::imu);
    EXPECT_NE(sn::sensor_kind::camera, sn::sensor_kind::gnss);
    EXPECT_NE(sn::kCameraNoiseDomainTag, sn::kImuNoiseDomainTag);
    EXPECT_NE(sn::kCameraNoiseDomainTag, sn::kGnssNoiseDomainTag);
    EXPECT_EQ(sn::kCameraNoiseDomainTag, "sensor.camera") << "pinned: changing it re-seeds every camera";
    EXPECT_TRUE(sn::sensor_kind::poll_permits(sn::sensor_kind::none, sn::sensor_kind::camera));
    EXPECT_FALSE(sn::sensor_kind::poll_permits(sn::sensor_kind::imu, sn::sensor_kind::camera)) << "type confusion through a valid ref";
}

TEST(CameraKind, ItsNoiseStreamIsPerWorldLocalSlotAndDiffersFromTheOtherKinds) {
    const auto a = sn::camera_noise_stream(7, 0);
    const auto b = sn::camera_noise_stream(7, 1);
    EXPECT_NE(a.state, b.state) << "two cameras never share a stream";
    const auto g = sn::gnss_noise_stream(7, 0);
    EXPECT_NE(a.state, g.state) << "a camera and a GNSS at one slot never share one";
}

TEST(CameraSchedule, DividerAndPhaseDecideWhichSubstepsRenderAFrame) {
    sn::CameraSensorRow r = good_row();
    r.rate_divider = 4;
    std::string due;
    for (int i = 0; i < 12; ++i) due += sn::camera_advance(r) ? '1' : '0';
    EXPECT_EQ(due, "000100010001") << "every 4th substep, the phase wrapping to 0 on the due one";
    EXPECT_EQ(r.phase, 0u);

    sn::CameraSensorRow every = good_row();
    every.rate_divider = 1;
    for (int i = 0; i < 5; ++i) EXPECT_TRUE(sn::camera_advance(every)) << i;

    sn::CameraSensorRow zero = good_row();
    zero.rate_divider = 0;
    EXPECT_TRUE(sn::camera_advance(zero)) << "a divider of 0 reads as every substep, as the GNSS's";

    sn::CameraSensorRow phased = good_row();
    phased.rate_divider = 4;
    phased.phase = 2;  // two substeps into the period already
    due.clear();
    for (int i = 0; i < 6; ++i) due += sn::camera_advance(phased) ? '1' : '0';
    EXPECT_EQ(due, "010001") << "a phase offset moves the first frame earlier, not the rate";
}

TEST(CameraRowIssue, AValidRowAndAnInertZeroRowAreAccepted) {
    EXPECT_EQ(sn::camera_row_issue(good_row()), "");
    sn::CameraSensorRow zero{};
    EXPECT_EQ(sn::camera_row_issue(zero), "") << "kind none is inert: a freed or reserved slot";
}

TEST(CameraRowIssue, EachBrokenRuleIsRefusedNamingItsField) {
    const auto expect_refused = [](sn::CameraSensorRow r, const char* field) {
        const std::string issue = sn::camera_row_issue(r);
        EXPECT_NE(issue, "") << field << " should be refused";
        EXPECT_NE(issue.find(field), std::string::npos) << "the refusal should name " << field << ": " << issue;
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sn::CameraSensorRow r = good_row();

    r = good_row(); r.kind = sn::sensor_kind::imu; expect_refused(r, "kind");
    r = good_row(); r.rate_divider = 0; expect_refused(r, "rate_divider");
    r = good_row(); r.phase = 4; expect_refused(r, "phase");
    r = good_row(); r.lens_kind = sn::lens_kind::count; expect_refused(r, "lens_kind");
    r = good_row(); r.width = 0; expect_refused(r, "width");
    r = good_row(); r.height = 0; expect_refused(r, "height");
    r = good_row(); r.mode = sn::camera_mode::count; expect_refused(r, "mode");
    r = good_row(); r.fx = 0.0f; expect_refused(r, "fx");
    r = good_row(); r.fy = -1.0f; expect_refused(r, "fy");
    r = good_row(); r.cx = nan; expect_refused(r, "cx");
    r = good_row(); r.cy = std::numeric_limits<float>::infinity(); expect_refused(r, "cy");
    r = good_row(); r.lens_kind = sn::lens_kind::pinhole; r.k1 = 0.1f; expect_refused(r, "k1");
    r = good_row(); r.lens_kind = sn::lens_kind::kb4; r.k3 = nan; expect_refused(r, "k3");
    r = good_row(); r.shutter_fraction = 1.5f; expect_refused(r, "shutter_fraction");
    r = good_row(); r.shutter_fraction = -0.1f; expect_refused(r, "shutter_fraction");
    r = good_row(); r.blur_samples = 0; expect_refused(r, "blur_samples");
    r = good_row(); r.aa_samples = 0; expect_refused(r, "aa_samples");
    r = good_row(); r.exposure_gain = 0.0f; expect_refused(r, "exposure_gain");
    r = good_row(); r.cull_mask = 0x100u; expect_refused(r, "cull_mask");
    r = good_row(); r.channels = 1; expect_refused(r, "channels");
    r = good_row(); r.mount_pos.y = nan; expect_refused(r, "mount_pos");
    r = good_row(); r.mount_orient = glm::quat(2.0f, 0.0f, 0.0f, 0.0f); expect_refused(r, "mount_orient");
    r = good_row(); r._reserved0 = 1; expect_refused(r, "_reserved0");
}

TEST(CameraRowIssue, AKb4LensWithItsCoefficientsIsAccepted) {
    sn::CameraSensorRow r = good_row();
    r.lens_kind = sn::lens_kind::kb4;
    r.k1 = 0.02f;
    r.k2 = -0.003f;
    r.k3 = 0.0005f;
    r.k4 = -0.00007f;
    EXPECT_EQ(sn::camera_row_issue(r), "");
}
