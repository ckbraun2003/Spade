#pragma once

// ---------------------------------------------------------------------------
// raster_cpu -- the deterministic CPU rasterizer (S7a Task R3). Turns a
// RenderScene + Camera + RenderOptions into an opaque BGRX8 RenderTarget: the
// task where a Spade world stops being a wireframe diagram and becomes solid
// shaded geometry.
//
// PORT, NOT REDESIGN: the projection/clipping/depth core below
// (raster_cpu.cpp's worldToCameraSpace/projectCameraSpace/
// clipSegmentToHalfSpace/setPixelIfCloser/rasterizeLine/edgeFn/
// rasterizeTriangleFlat) is a direct, unchanged port of the wireframe
// rasterizer's own already-correct camera/projection/depth code -- see that
// file's header comment for the full "why port, not rework" rationale, which
// applies here identically. Nothing in that block changed: same pinhole
// perspective, same inverse-depth z-buffer (larger invDepth = closer, 0 =
// infinitely far, first writer at a tie wins), same near/far handling. What
// is genuinely NEW in this task: shading a triangle by its submesh's material
// colour instead of drawing an edge, drawing from MeshData index buffers
// instead of hand-built edge lists, and DrawMode::shaded back-face culling
// (SR-13, below).
//
// DrawMode::shaded (solid, depth-tested, Gouraud-shaded per-submesh material
// colour -- directional Lambert lighting + ambient + a vertical sky gradient
// and analytic ground landed at Task R6 (rulings SR-17/SR-18); a single-sun
// shadow map, sampled per pixel against a perspective-correct world
// position, landed at Task R7 (render/shadow.hpp, ruling SR-24)) and
// DrawMode::wireframe (edges only, both sides always
// drawn, flat UNLIT submesh colour never touched by lighting -- the prior
// wireframe rasterizer's own debug/comparison vocabulary, kept deliberately
// unlit) are both implemented here. DrawMode::raymarch (S7a Task R8) is
// implemented in render/raymarch.cpp instead -- an exact SDF sphere-tracer,
// not a rasterizer, so it has no business sharing this file's ported
// camera/projection/clip pipeline -- and render() below simply forwards to
// it; see raymarch.hpp's own header for that path's algorithm and constants.
//
// RenderOptions::overlays (PA-4) draws the ground grid, world-bounds box,
// spawn diamonds and body-pose markers -- real orientation aids, ported from
// the wireframe rasterizer's drawGroundGrid/drawWorldBounds/drawSpawnMarkers/
// drawBodyMarkers, layered over the shaded or wireframe scene rather than
// replacing it.
//
// SR-13 (controller ruling, binding): DrawMode::shaded culls back faces;
// DrawMode::wireframe does not (both sides draw, preserving the prior
// wireframe rasterizer's debug vocabulary). This is what makes Task R2's
// tessellation winding fix (four of seven primitives had backward triangle
// order) observable at the pixel level instead of merely at the vertex-order
// level -- see raster_cpu.cpp's cull-check comment for the actual sign
// derivation.
//
// DETERMINISM (constraint 4): render() reads only its four explicit
// parameters -- no RNG, no wall clock, no static mutable state, no unordered
// iteration, fixed operation order (background clear, statics in order,
// dynamics in order, then overlays in a fixed sub-order). Same scene +
// camera + options + target size => byte-identical pixels, unconditionally.
//
// PURITY: `scene` is `const RenderScene&` -- this module never mutates it
// and never touches anything outside its four explicit parameters (five,
// counting the optional `shadow_scratch` below -- see its own comment: it
// only ever affects an allocation's PROVENANCE, never a pixel render()
// produces, so the determinism claim above is unchanged by its presence).
// Render cadence (never called, called once, called every tick) changes
// nothing about scene/body/sim state, because this module never reaches
// into any of it.
//
// MN-14 (opaque BGRX8): every byte of `target.pixels` is written exactly
// once per render() call before any drawing happens (the background clear),
// and every pixel's 4th (X) byte is always 0xFF -- background and every draw
// call write it identically, so there is no code path that leaves it
// unset.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

// Errors: invalid_argument if `target` fails validate_target(), or if a
// field layer fails validate_field_layer(); both refuse before any pixel is
// written. Once those pass, `options.mode == DrawMode::raymarch` (S7a Task
// R8) forwards to render_raymarch() (render/raymarch.hpp) instead of this
// file's own rasterizer -- `shadow_scratch`, `options.shadows`/
// `options.overlays` and `scene.field_layers` are not consulted on that path
// (raymarch has no shadows or overlays by design; see raymarch.hpp's own
// header for why). The raster modes draw field layers after the meshes.
//
// `shadow_scratch` (S7a Task R7, fix round 1, review IMPORTANT I3): an
// OPTIONAL caller-owned buffer render() may use for its own per-frame
// shadow-map-plus-dynamics working copy, instead of allocating a fresh one
// every call. Defaulted to `nullptr` so every EXISTING call site keeps
// compiling and behaving exactly as before (a fresh, function-local
// allocation each call) -- this parameter is purely an opt-in optimisation,
// never a behavioural or output difference: the RENDERED PIXELS are
// identical whichever way this is passed, only where the 4 MiB-at-default-
// size copy's memory comes from differs. A caller that renders the same
// scene/shadow-map size across many frames (the common case) and passes the
// SAME `std::vector<float>*` every call gets that buffer's capacity reused
// via `std::vector::assign()` (a no-op resize once it has grown to fit),
// amortising the allocation to effectively zero after the first frame --
// see raster_cpu.cpp's own render() body for exactly when this path is
// even reached (only when `scene.dynamics` is non-empty; an empty-dynamics
// frame skips the copy entirely and never touches this parameter at all).
[[nodiscard]] Result<void> render(const RenderScene& scene, const Camera& camera, const RenderOptions& options,
                                   RenderTarget& target, std::vector<float>* shadow_scratch = nullptr);

// ---------------------------------------------------------------------------
// The overlays' geometry (RenderOptions::overlays, PA-4): one source for the
// CPU raster and GL, so the two paths cannot drift on what they draw.

// The inverse-depth bias every overlay vertex gets, so an overlay wins a
// depth tie with the surface it lies on. raster_cpu.cpp derives it and
// states its known limit; GL applies the same value.
inline constexpr double kOverlayDepthBias = 1e-4;

// A world-space line or triangle, flat and unlit, with its colour.
struct OverlayLine {
    glm::dvec3 a{0.0}, b{0.0};
    std::array<uint8_t, 3> rgb{};
};
struct OverlayTriangle {
    glm::dvec3 a{0.0}, b{0.0}, c{0.0};
    std::array<uint8_t, 3> rgb{};
};

// In draw order: lines first (the ground grid, then the world bounds), then
// triangles (spawn markers, then markers for meshless dynamic bodies). Each
// overlay is depth-tested and writes depth, so the order decides ties among
// overlays. Neither side culls the triangles.
struct OverlayGeometry {
    std::vector<OverlayLine> lines;
    std::vector<OverlayTriangle> triangles;
};

// Empty unless `options.overlays`. Spawn markers need `options.spawn_markers`
// as well. render() draws exactly this in its raster modes; the ray-march
// mode draws no overlays.
[[nodiscard]] OverlayGeometry overlay_geometry(const RenderScene& scene, const RenderOptions& options);

}  // namespace spade::render
