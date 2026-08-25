#pragma once

// ---------------------------------------------------------------------------
// The render target/camera contract (S7a Task 0). Every later task in the
// Spade Rendering & Scene Representation program depends on these exact
// names -- this header is the seam between whatever drives a frame (the
// viewer, a training harness, a future headless renderer) and the module
// that draws into a caller-owned pixel buffer.
//
// PA-1: RenderTarget never owns its pixel memory. `pixels` is a non-owning
// view over a buffer the CALLER allocates and keeps alive for the call --
// no allocation, no lifetime surprise, on the render module's side.
//
// PA-4: RenderOptions::overlays gates presentation-only annotations (ground
// grid, world bounds, spawn markers) that are never part of the physics
// digest and never feed anything downstream of a frame.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"

namespace spade::render {

enum class PixelFormat : uint32_t { bgrx8 = 1 };

struct RenderTarget {                 // caller owns the memory (PA-1)
    std::span<uint8_t> pixels;        // exactly stride * height bytes
    uint32_t width = 0, height = 0, stride = 0;   // stride == width * 4
    PixelFormat format = PixelFormat::bgrx8;
};

struct Camera {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};  // local->world, wxyz
    float fov_y_radians = 1.0471975511965976f;      // 60 deg
    float near_plane = 0.1f, far_plane = 1000.0f;
};

enum class DrawMode : uint32_t { shaded = 0, wireframe = 1, raymarch = 2 };

struct RenderOptions {
    DrawMode mode = DrawMode::shaded;
    bool shadows = true;
    bool overlays = true;             // ground grid, bounds, spawn markers (PA-4)
};

// Shared vocabulary lives HERE, not in scene.hpp: tessellate.hpp and csg_mesh.hpp
// both need Aabb, and scene.cpp includes both -- putting Aabb in scene.hpp would
// be an include cycle. Sentinels likewise precede every struct that defaults to them.
struct Aabb { glm::vec3 min{0.0f}, max{0.0f}; };
inline constexpr uint32_t kNoMaterial = 0xFFFFFFFFu;
inline constexpr uint32_t kNoMesh     = 0xFFFFFFFFu;

// Codes:
//   invalid_argument -- stride != width * 4, pixels.size() != stride * height,
//                       width == 0, height == 0, or an unrecognized format
[[nodiscard]] Result<void> validate_target(const RenderTarget&);  // stride/size/format sanity

}  // namespace spade::render
