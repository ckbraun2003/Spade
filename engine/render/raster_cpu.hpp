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
// unlit) are both implemented here; DrawMode::raymarch is out of this task's
// scope entirely (Tasks R8/R9 own it -- RenderScene::sdf's own doc comment
// says so) and render() reports it as an unhandled argument rather than
// silently falling back to a mode the caller did not ask for.
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
// and never touches anything outside its four explicit parameters. Render
// cadence (never called, called once, called every tick) changes nothing
// about scene/body/sim state, because this module never reaches into any of
// it.
//
// MN-14 (opaque BGRX8): every byte of `target.pixels` is written exactly
// once per render() call before any drawing happens (the background clear),
// and every pixel's 4th (X) byte is always 0xFF -- background and every draw
// call write it identically, so there is no code path that leaves it
// unset.
// ---------------------------------------------------------------------------

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

// Errors: invalid_argument if `target` fails validate_target(), or if
// `options.mode == DrawMode::raymarch` (not implemented until Task R8/R9 --
// see this file's header comment).
[[nodiscard]] Result<void> render(const RenderScene& scene, const Camera& camera,
                                   const RenderOptions& options, RenderTarget& target);

}  // namespace spade::render
