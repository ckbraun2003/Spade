#pragma once

// ---------------------------------------------------------------------------
// camera_lens -- the camera sensor's lens maths (camera sensor design, sections 5 and 10; Phase 1, P1.1). The CPU reference's lens, the oracle the GPU's ray table is
// banded against.
//
// THE CONVENTIONS (L117, design section 5). A pixel (i, j) has its centre at (i + 0.5, j + 0.5), the origin at the image's top-left corner, x to the right and y
// down; the camera frame is x right, y down, z forward. Positions here are float pixels in that space. A lens is a pinhole (kind 0) or a Kannala-Brandt 4 fisheye
// (kind 1, OpenCV's `fisheye` model): theta_d(theta) = theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8), the normalised image radius against the
// angle off the axis. A lens beyond 180 degrees (a corner looking past 90 degrees off the axis) is refused, so every ray of a valid lens is in front of the image
// plane (z > 0).
//
// NO LIBM (SR-14). Everything is built from + - * /, sqrt and the engine's own kernels (core/detail/math_ops_f64.hpp: atan2_arith and sincos_deg_arith). The
// KB4 unprojection solves theta_d(theta) = r by Newton's method with a FIXED iteration count (no data-dependent exit), so the same intrinsics give the same bits on
// every platform that has the same arithmetic kernels. The ray TABLE is computed once on the host in fp64 and stored as unit float vectors a pixel.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace spade::render {

inline constexpr uint32_t kLensPinhole = 0;
inline constexpr uint32_t kLensKb4 = 1;

// The intrinsics of one camera: the fields of CameraSensorRow (engine/sensors/camera.hpp) that make the lens. k is k1..k4, all 0 for a pinhole.
struct LensSpec {
    uint32_t kind = kLensPinhole;
    uint32_t width = 0, height = 0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double k[4] = {0.0, 0.0, 0.0, 0.0};
};

// "" when the intrinsics can be a lens, else the first thing wrong, naming the field: kind (above 1 is reserved), width, height, fx, fy (finite and above 0),
// cx, cy, k1..k4 (finite), a pinhole's k (must be 0), and for a KB4: theta_d(theta) must be monotone over the field of view the corners ask for (naming the
// coefficient that turns it over; Newton has no unique answer where it is not), and the corners must be inside 180 degrees (a half angle under 90).
[[nodiscard]] std::string lens_issue(const LensSpec& spec);

// The unit ray, in the camera frame, of the image position (px, py), in pixels (a pixel's centre is at i + 0.5). Precondition: lens_issue(spec) is empty.
[[nodiscard]] glm::dvec3 lens_ray(const LensSpec& spec, double px, double py);

// The image position, in pixels, of a camera-frame point; nullopt for z <= 0 (at or behind the image plane) or a NaN. The keypoints' projection; the distorted
// pixel for a KB4. Precondition: lens_issue(spec) is empty.
[[nodiscard]] std::optional<glm::dvec2> lens_project(const LensSpec& spec, const glm::dvec3& point);

// The ray table: width x height unit float vectors, row-major, at the pixel centres. A pure function of the intrinsics.
[[nodiscard]] std::vector<glm::vec3> lens_table(const LensSpec& spec);

}  // namespace spade::render
