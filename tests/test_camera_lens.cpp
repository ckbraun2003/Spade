// test_camera_lens.cpp -- the camera's lens maths: pinhole and Kannala-Brandt 4 (KB4), against fp64 (Phase 1 of rendering/camera-sensor-design, sections 5 and 10).
//
// WHAT IS PINNED. The ray table the trace kernel reads is computed once on the host in fp64 from the intrinsics, and stored as unit float vectors a pixel. Pixel
// (i, j) has its centre at (i + 0.5, j + 0.5), origin at the top-left corner; camera frame x right, y down, z forward. The forward projection (a point to a
// pixel) is the same maths run the other way and is what the keypoints use. The lens code under engine/ has no libm (SR-14: it is built from + - * / and sqrt and
// the engine's own atan2_arith and sincos_deg_arith); THIS FILE's reference uses libm on purpose, as an independent check.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/camera_lens.hpp"

namespace rd = spade::render;

namespace {

// KAT's AIGP camera: 640 x 360 pinhole, fx = fy = 320, the centre at (320, 180).
[[nodiscard]] rd::LensSpec aigp() {
    rd::LensSpec s{};
    s.kind = rd::kLensPinhole;
    s.width = 640;
    s.height = 360;
    s.fx = 320.0;
    s.fy = 320.0;
    s.cx = 320.0;
    s.cy = 180.0;
    return s;
}

// A T265-like fisheye: 848 x 800, about 163 degrees on the diagonal (half angle 81.5 degrees), KB4 with small distortion terms.
[[nodiscard]] rd::LensSpec fisheye() {
    rd::LensSpec s{};
    s.kind = rd::kLensKb4;
    s.width = 848;
    s.height = 800;
    s.fx = 405.0;
    s.fy = 405.0;
    s.cx = 424.0;
    s.cy = 400.0;
    s.k[0] = -0.0072;
    s.k[1] = 0.0053;
    s.k[2] = -0.0011;
    s.k[3] = 0.00012;
    return s;
}

// The independent reference, with libm: theta_d(theta) for KB4.
[[nodiscard]] double ref_theta_d(const rd::LensSpec& s, double t) {
    const double t2 = t * t;
    return t * (1.0 + t2 * (s.k[0] + t2 * (s.k[1] + t2 * (s.k[2] + t2 * s.k[3]))));
}

// The reference ray of a pixel position: pinhole closed form; KB4 by bisection on theta in [0, pi/2] (theta_d is increasing there for these lenses).
[[nodiscard]] glm::dvec3 ref_ray(const rd::LensSpec& s, double px, double py) {
    const double a = (px - s.cx) / s.fx;
    const double b = (py - s.cy) / s.fy;
    if (s.kind == rd::kLensPinhole) {
        const double n = std::sqrt(a * a + b * b + 1.0);
        return {a / n, b / n, 1.0 / n};
    }
    const double rd_ = std::sqrt(a * a + b * b);
    if (rd_ == 0.0) return {0.0, 0.0, 1.0};
    double lo = 0.0, hi = 1.5707963267948966;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (ref_theta_d(s, mid) < rd_ ? lo : hi) = mid;
    }
    const double theta = 0.5 * (lo + hi);
    return {std::sin(theta) * a / rd_, std::sin(theta) * b / rd_, std::cos(theta)};
}

// The reference forward projection of a camera-frame point with z > 0.
[[nodiscard]] glm::dvec2 ref_project(const rd::LensSpec& s, const glm::dvec3& p) {
    const double a = p.x / p.z;
    const double b = p.y / p.z;
    if (s.kind == rd::kLensPinhole) return {s.fx * a + s.cx, s.fy * b + s.cy};
    const double r = std::sqrt(a * a + b * b);
    if (r == 0.0) return {s.cx, s.cy};
    const double td = ref_theta_d(s, std::atan(r));
    return {s.fx * a * td / r + s.cx, s.fy * b * td / r + s.cy};
}

}  // namespace

TEST(CameraLens, ThePinholeTableIsTheClosedFormAndThePixelCentreConventionHolds) {
    const rd::LensSpec s = aigp();
    ASSERT_EQ(rd::lens_issue(s), "");
    // The exact image centre is at (320, 180): it looks straight down +z.
    const glm::dvec3 centre = rd::lens_ray(s, 320.0, 180.0);
    EXPECT_EQ(centre.x, 0.0);
    EXPECT_EQ(centre.y, 0.0);
    EXPECT_EQ(centre.z, 1.0);
    // The top-left corner of pixel (0, 0): normalize((0 - 320) / 320, (0 - 180) / 320, 1).
    const glm::dvec3 corner = rd::lens_ray(s, 0.0, 0.0);
    const glm::dvec3 want = ref_ray(s, 0.0, 0.0);
    EXPECT_NEAR(corner.x, want.x, 1e-15);
    EXPECT_NEAR(corner.y, want.y, 1e-15);
    EXPECT_NEAR(corner.z, want.z, 1e-15);
    EXPECT_LT(corner.x, 0.0) << "x right: the left edge looks left";
    EXPECT_LT(corner.y, 0.0) << "y down: the top edge looks up";

    // The table: one unit float vector a pixel, row-major, at the pixel centres (i + 0.5, j + 0.5).
    const std::vector<glm::vec3> table = rd::lens_table(s);
    ASSERT_EQ(table.size(), static_cast<std::size_t>(s.width) * s.height);
    for (const std::pair<uint32_t, uint32_t>& ij : {std::pair<uint32_t, uint32_t>{0, 0}, {639, 0}, {0, 359}, {639, 359}, {320, 180}, {17, 301}}) {
        const glm::dvec3 w = ref_ray(s, ij.first + 0.5, ij.second + 0.5);
        const glm::vec3 g = table[static_cast<std::size_t>(ij.second) * s.width + ij.first];
        EXPECT_NEAR(g.x, w.x, 1.2e-7) << ij.first << "," << ij.second;
        EXPECT_NEAR(g.y, w.y, 1.2e-7) << ij.first << "," << ij.second;
        EXPECT_NEAR(g.z, w.z, 1.2e-7) << ij.first << "," << ij.second;
    }
    for (const glm::vec3& r : table) {
        ASSERT_NEAR(glm::length(r), 1.0f, 2e-7f);
    }
    EXPECT_EQ(table, rd::lens_table(s)) << "the table is a pure function of the intrinsics: two calls give the same bytes";
}

TEST(CameraLens, TheKb4RayIsTheInverseOfItsProjectionAndMatchesAnIndependentSolve) {
    const rd::LensSpec s = fisheye();
    ASSERT_EQ(rd::lens_issue(s), "");
    // The principal point looks straight ahead; a ray at image radius r has angle theta with theta_d(theta) = r / f.
    const glm::dvec3 centre = rd::lens_ray(s, s.cx, s.cy);
    EXPECT_EQ(centre.x, 0.0);
    EXPECT_EQ(centre.y, 0.0);
    EXPECT_EQ(centre.z, 1.0);
    double worst_ray = 0.0, worst_round_trip = 0.0;
    for (double py = 0.5; py < 800.0; py += 37.0) {
        for (double px = 0.5; px < 848.0; px += 41.0) {
            const glm::dvec3 got = rd::lens_ray(s, px, py);
            const glm::dvec3 want = ref_ray(s, px, py);
            worst_ray = std::max(worst_ray, glm::length(got - want));
            EXPECT_NEAR(glm::length(got), 1.0, 1e-14);
            EXPECT_GT(got.z, 0.0) << "every ray of a lens inside 180 degrees is in front of the image plane";
            const std::optional<glm::dvec2> back = rd::lens_project(s, got);
            ASSERT_TRUE(back.has_value()) << px << "," << py;
            worst_round_trip = std::max(worst_round_trip, std::max(std::abs(back->x - px), std::abs(back->y - py)));
        }
    }
    EXPECT_LT(worst_ray, 1e-12) << "against the bisection reference with libm";
    EXPECT_LT(worst_round_trip, 1e-9) << "pixels: project(ray(p)) == p";
}

TEST(CameraLens, TheForwardProjectionAgreesWithTheFp64ReferenceForBothLenses) {
    for (const rd::LensSpec& s : {aigp(), fisheye()}) {
        double worst = 0.0;
        for (double az = 0.0; az < 6.28; az += 0.37) {
            for (double el : {0.0, 0.05, 0.3, 0.6, 0.9, 1.2, (s.kind == rd::kLensKb4 ? 1.35 : 0.9)}) {
                // A point at the angle `el` off the axis, around it by `az`, 3 m out: (sin el cos az, sin el sin az, cos el) * 3.
                const glm::dvec3 p = 3.0 * glm::dvec3(std::sin(el) * std::cos(az), std::sin(el) * std::sin(az), std::cos(el));
                const std::optional<glm::dvec2> got = rd::lens_project(s, p);
                ASSERT_TRUE(got.has_value());
                const glm::dvec2 want = ref_project(s, p);
                worst = std::max(worst, std::max(std::abs(got->x - want.x), std::abs(got->y - want.y)));
            }
        }
        EXPECT_LT(worst, 1e-8) << "pixels, kind " << s.kind;
    }
    // A point on the axis is the principal point; a point at or behind the image plane has no pixel.
    EXPECT_EQ(*rd::lens_project(fisheye(), glm::dvec3(0.0, 0.0, 2.0)), glm::dvec2(424.0, 400.0));
    EXPECT_FALSE(rd::lens_project(aigp(), glm::dvec3(0.1, 0.0, -1.0)).has_value());
    EXPECT_FALSE(rd::lens_project(aigp(), glm::dvec3(1.0, 0.0, 0.0)).has_value()) << "z == 0: edge on, not in front";
}

TEST(CameraLens, AKb4LensWithNoDistortionIsEquidistant) {
    rd::LensSpec s = fisheye();
    s.k[0] = s.k[1] = s.k[2] = s.k[3] = 0.0;
    ASSERT_EQ(rd::lens_issue(s), "");
    // r pixels from the centre along +x is theta = r / f from the axis, exactly the equidistant fisheye.
    for (double r : {10.0, 100.0, 300.0, 500.0}) {
        const glm::dvec3 d = rd::lens_ray(s, s.cx + r, s.cy);
        const double theta = r / s.fx;
        EXPECT_NEAR(d.x, std::sin(theta), 1e-13) << r;
        EXPECT_NEAR(d.y, 0.0, 1e-15) << r;
        EXPECT_NEAR(d.z, std::cos(theta), 1e-13) << r;
    }
}

TEST(CameraLens, ALensIsRefusedByNameWhenItsIntrinsicsCannotBeOne) {
    EXPECT_EQ(rd::lens_issue(aigp()), "");
    EXPECT_EQ(rd::lens_issue(fisheye()), "");
    auto issue_of = [](auto&& edit) {
        rd::LensSpec s = aigp();
        edit(s);
        return rd::lens_issue(s);
    };
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.kind = 2; }).find("kind"), std::string::npos) << "above KB4 is reserved";
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.width = 0; }).find("width"), std::string::npos);
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.height = 0; }).find("height"), std::string::npos);
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.fx = 0.0; }).find("fx"), std::string::npos);
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.fy = -1.0; }).find("fy"), std::string::npos);
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.cx = std::numeric_limits<double>::quiet_NaN(); }).find("cx"), std::string::npos);
    EXPECT_NE(issue_of([](rd::LensSpec& s) { s.k[1] = 0.1; }).find("k2"), std::string::npos) << "a pinhole's distortion terms must be zero";
    // A KB4 lens whose image corners look beyond 180 degrees (a half angle past 90) is refused by name: every ray must be in front.
    rd::LensSpec wide = fisheye();
    wide.fx = wide.fy = 250.0;
    const std::string why = rd::lens_issue(wide);
    EXPECT_NE(why.find("180"), std::string::npos) << why;
    rd::LensSpec fold = fisheye();
    fold.k[0] = -0.9;  // theta_d stops growing inside 90 degrees: the lens folds back
    EXPECT_NE(rd::lens_issue(fold).find("monotone"), std::string::npos) << rd::lens_issue(fold);
}
