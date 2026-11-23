#pragma once

// camera_frames -- the engine camera's CPU reference frames of a scene, as PNGs (O150, video 4: "the camera"). The pinhole and the Kannala-Brandt 4 fisheye of ONE pose of ONE
// world, through render_camera_frame (engine/render/camera_frame.hpp), so a video can show them beside the editor's view of the same world. A tool outside engine/ (L7): it
// reads a world file, draws on the CPU (no GL, no GPU), writes files, and adds no engine call.
//
// THE POSE: a camera position, a target and an up (Spade's world: +y up, right-handed); the camera frame is the engine's (x right, y down, z forward), and look_at() builds its
// camera_to_world. THE LENSES: the pinhole is given by its horizontal field of view; the fisheye by its DIAGONAL field of view (a fisheye's corners are its widest angle, and
// the engine refuses a corner past 90 degrees off the axis, so the diagonal is the one number that cannot be asked for wrongly) and k1..k4 (all 0 = the equidistant fisheye).
// Both look through the same pixel size, so the two pictures are the same size side by side.

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/camera_frame.hpp"
#include "render/camera_lens.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"

namespace camera_frames {

struct Options {
    std::string world;                              // the world file to draw (required)
    glm::dvec3 position{0.0, 1.5, -4.0};
    glm::dvec3 target{0.0, 1.0, 0.0};
    glm::dvec3 up{0.0, 1.0, 0.0};
    uint32_t width = 640;
    uint32_t height = 480;
    double pinhole_fov_deg = 90.0;                  // horizontal
    double fisheye_fov_deg = 170.0;                 // diagonal
    std::array<double, 4> k{0.0, 0.0, 0.0, 0.0};    // the fisheye's k1..k4
    std::string out_dir = ".";
    std::string stem = "camera";
    bool instances = false;                         // also write the instance-id channel, in false colour
};

// The flags: --world FILE --camera X,Y,Z --look-at X,Y,Z --up X,Y,Z --size WxH --pinhole-fov DEG --fisheye-fov DEG --k K1,K2,K3,K4 --out DIR --name STEM --instances.
// A refusal names the flag.
[[nodiscard]] spade::Result<Options> parse_args(std::span<const std::string> args);

// The camera's camera_to_world (columns: x right, y down, z forward, in the world), looking from `position` at `target` with `up` the world's up.
[[nodiscard]] spade::Result<glm::dmat3> look_at(const glm::dvec3& position, const glm::dvec3& target, const glm::dvec3& up);

[[nodiscard]] spade::Result<spade::render::LensSpec> pinhole_lens(uint32_t width, uint32_t height, double horizontal_fov_deg);
[[nodiscard]] spade::Result<spade::render::LensSpec> fisheye_lens(uint32_t width, uint32_t height, double diagonal_fov_deg, const std::array<double, 4>& k);

struct Views {
    spade::render::CameraFrame pinhole;
    spade::render::CameraFrame fisheye;
};
// Both lenses at the options' pose, image and instance id. `label_ids` is the host's table for the scene's labels (one uint16 each; empty = the scene's own label index + 1).
[[nodiscard]] spade::Result<Views> render_views(const spade::render::RenderScene& scene, const spade::render::RayScene& rays, const Options& options,
                                                std::span<const uint16_t> label_ids);

// A planar C x H x W image as interleaved RGB (a grey image, C = 1, is spread to three).
[[nodiscard]] std::vector<uint8_t> interleave_rgb(std::span<const uint8_t> planar, uint32_t width, uint32_t height, uint32_t channels);
// A false colour for every instance id, 0 (sky, ground, unlabelled) black; one id one colour, always.
[[nodiscard]] std::vector<uint8_t> instance_rgb(std::span<const uint16_t> ids);
// An 8-bit RGB PNG (zlib stored blocks: no compression, so the encoder is short enough to check by eye).
[[nodiscard]] std::vector<uint8_t> encode_png_rgb(uint32_t width, uint32_t height, std::span<const uint8_t> rgb);

// Writes <out_dir>/<stem>_pinhole.png and <stem>_fisheye.png (and _pinhole_instances.png, _fisheye_instances.png when asked); the paths written.
[[nodiscard]] spade::Result<std::vector<std::string>> write_views(const Options& options, const Views& views);

}  // namespace camera_frames
