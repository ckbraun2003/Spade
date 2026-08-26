#pragma once

// ---------------------------------------------------------------------------
// raymarch -- the exact SDF reference path (S7a Task R8). Renders the SAME
// world the tessellated rasterizer (raster_cpu.cpp) draws, but by sphere-
// tracing `scene.sdf` directly instead of drawing triangles -- this IS the
// ground truth Task R9 measures the fast tessellated path against, so it
// evaluates the physics side's own SdfProgram (world/sdf.hpp) directly,
// never a second/approximate representation of the same geometry.
//
// THE GROUND COMES FREE (controller amendment, task-R8-brief.md): the
// raster path needs an analytic background ground (render/scene.hpp's
// GroundPlane list, ruling SR-17) because a tessellated plane grid is
// necessarily bounded. An SDF `plane` primitive IS infinite, so
// sphere-tracing it produces an infinite ground with no special case at all
// -- this file never reads RenderScene::ground_planes or
// RenderScene::ground_y/has_ground; it marches `scene.sdf` and the horizon
// falls out on its own. Porting either of those raster-only devices here
// would be building a ground this path already has for free.
//
// SHARED, NOT REIMPLEMENTED:
//   * Shading is render/scene.hpp's shade_vertex_color() -- the SAME
//     function draw_mesh_triangle_shaded and the analytic ground pass call
//     (Task R6, ruling SR-18) -- reused here verbatim, never a second
//     Lambert model.
//   * The sky (a ray that never converges) is render/scene.hpp's
//     sky_gradient_color() -- the SAME per-row zenith/horizon blend
//     draw_sky_and_ground_background uses (Task R6, ruling SR-23), evaluated
//     at the SAME row fraction, so a raymarched miss and a rasterized
//     background pixel in the same screen row are bit-identical. The SDF has
//     nothing to say about the sky -- there is no primitive up there to
//     sphere-trace -- so a miss must fall back to exactly what the raster
//     path already draws there.
//   * The surface normal is world/sdf.hpp's own gradient()/sample(), which
//     already uses the PINNED central-difference stencil the physics
//     collision passes use (kSdfGradientStep, world/sdf.hpp) for every
//     primitive kind without an analytic gradient -- called directly here,
//     never re-derived, so there is no second stencil to disagree with the
//     first one by a different epsilon.
//
// SHADOWS ARE DELIBERATELY OUT OF SCOPE (controller amendment, not an
// oversight): Task R9 compares SILHOUETTES ONLY -- coverage against the sky
// reference colour -- precisely because shading, shadows included, is
// expected to diverge between the two paths; shadows never affect coverage.
// R7's shadow map is also orthographic, fitted to `scene.bounds`, and
// rasterised from TESSELLATED static geometry (render/shadow.hpp) --
// sampling it from this exact-SDF reference path would couple the reference
// to the very approximation it exists to check, defeating its purpose. This
// file never includes render/shadow.hpp and never samples one.
//
// MATERIAL: every hit shades with `scene.materials[0]` (the scene's default
// material, guaranteed present whenever `scene.materials` is non-empty --
// scene_from_world()'s own contract, render/scene.hpp). The SDF program
// combines every primitive into ONE implicit surface (world/sdf.hpp's
// postfix stack machine) with no per-hit "which leaf is nearest" signal
// exposed by its public eval()/gradient()/sample() API, so there is no
// cheap, correct way to recover a PER-PRIMITIVE material index at an
// arbitrary hit point the way the tessellated path does
// (SdfProgram::node_materials feeds MESHING -- one mesh per node,
// render/csg_mesh.hpp -- an entirely different mechanism this path does not
// have). This is exactly the divergence Task R9's silhouette-only comparison
// already expects and is unaffected by; per-primitive raymarch materials are
// future scope, not this task's.
//
// DETERMINISM (constraint 4, same contract as raster_cpu.cpp's render()):
// render_raymarch() reads only its three explicit parameters -- no RNG, no
// wall clock, no static mutable state, no unordered iteration, fixed
// operation order (row by row, column by column, sphere-trace steps in
// ascending order). Same scene + camera + target size => byte-identical
// pixels, unconditionally.
//
// NULL VS EMPTY (task brief, restated as a binding constraint):
// `RenderScene::sdf` is a non-owning pointer that MAY BE NULL (a hand-built
// scene that never pointed it at a program at all) -- a DIFFERENT case from
// a non-null pointer to a program with zero nodes (`SdfProgram::empty()`).
// Both must render sky only, never fault: render_raymarch() checks
// `scene.sdf != nullptr` before ever dereferencing it, and separately relies
// on world/sdf.hpp's own documented guarantee that eval()/gradient()/
// sample() on an empty program return a finite non-hit (kSdfEmptyDistance)
// rather than faulting -- belt AND suspenders, neither substituting for the
// other.
//
// NO LIBM TRANSCENDENTALS (ruling SR-14): the one trig call this file needs
// (the tangent of half the vertical FOV, to build a pinhole camera ray) is
// built from math::sin32/math::cos32, exactly like raster_cpu.cpp's own
// private tan32 helper -- duplicated locally in raymarch.cpp rather than
// shared, because that helper has internal linkage in a different
// translation unit and this is its only other call site. Nothing in this
// file calls a libm transcendental; sphere tracing needs distance evaluation
// and normalization (std::sqrt, IEEE-mandated and fine), not trig.
// ---------------------------------------------------------------------------

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

// ---------------------------------------------------------------------------
// Sphere-tracing constants (task brief Step 2: "a fixed step budget and
// epsilon (constant table, documented)"). Compile-time, never adaptive --
// the same reason kTessellationDefaults (render/tessellate.hpp) is not: an
// adaptive budget would make two runs of the SAME scene take a different
// number of steps depending on incidental floating-point history, a
// determinism hazard this program has already paid down once already (R7's
// own fixed-step-count shadow rasterization).
// ---------------------------------------------------------------------------

// The hard per-pixel iteration cap. There is deliberately no separate "max
// trace distance" constant: for an SDF that is an exact (or conservative --
// world/sdf.hpp's own heightfield note) distance bound, a ray moving away
// from all geometry has its returned distance grow at least as fast as the
// distance already travelled, so it escapes to "no hit" within a handful of
// steps regardless of the budget's size. The budget's real job is bounding a
// GRAZING ray -- one that stays close to a nearby surface without quite
// converging -- which is the standard, accepted sphere-tracing failure mode:
// such a ray reads as a miss (sky) once the budget is spent, never as an
// infinite loop or a fault.
//
// CHOSEN BY MEASUREMENT, not a round-number guess: 128 under-converged a
// real near-horizon band on RaymarchSmoke.HoverPadCoverageWithinTwoPercent-
// OfTessellated (test_render_raymarch.cpp) at 6.1% relative coverage
// difference; 256 improved it to 3.4%; 512 (this value) to 1.69%, safely
// inside the task's own 2% smoke bound. 768 produced the IDENTICAL pixel
// count -- confirming 512 already sits past the point where step count is
// the limiting factor for this fixture (see the KNOWN LIMITATION note
// below), so 768's extra cost bought nothing and was reverted.
inline constexpr uint32_t kRaymarchMaxSteps = 512;

// A step converges (hits) once |distance| <= this, in the SAME world-space
// units SdfProgram's own distances use (metres, by this program's
// convention). Deliberately a different constant from world/sdf.hpp's own
// kSdfGradientStep (1e-3, one order of magnitude coarser here): that one
// perturbs a point to estimate a DERIVATIVE and wants to be small relative
// to surface curvature; this one accepts a point as ON the surface and wants
// to be small relative to a rendered PIXEL's footprint at the scene's own
// scale -- a different quantity, conflating the two would be exactly the
// "same numbers, different provenance" mistake render/raster_cpu.cpp's own
// kOverlayDepthBias comment warns against for a different pair of constants.
// Measured, not merely reasoned about: 5e-2 (2.5x this value) produced the
// IDENTICAL covered-pixel count on the same fixture above -- epsilon is not
// the limiting factor at this scale, kRaymarchMaxSteps above and the KNOWN
// LIMITATION below are.
inline constexpr float kRaymarchSurfaceEpsilon = 2e-2f;

// KNOWN, ACCEPTED LIMITATION (measured, not fixed here -- mirrors this
// program's own precedent for a residual, understood gap: render/shadow.hpp's
// M6 note, render/raster_cpu.cpp's kOverlayDepthBias M5 note): a ray whose
// world-space direction is extremely close to horizontal (grazing an
// infinite ground plane almost edge-on -- the exact screen row nearest the
// true optical horizon) needs a step count that grows roughly as
// 1/|dir.y| to converge; no FIXED budget converges the row closest to
// dir.y == 0 for every possible camera pose, and past a few hundred steps
// more budget stops helping at all (measured above) because the residual
// gap at that scale is FLOAT-vs-DOUBLE precision, not iteration count: this
// file computes the camera ray in glm::vec3 (float, matching SdfProgram's
// own "fp32 everywhere" convention, world/sdf.hpp), while raster_cpu.cpp's
// own analytic background ray (its ported ViewContext/Vec3 block) is
// double-precision, so the two paths can round a hair's-breadth-from-
// horizontal direction to a different sign of "how far below horizontal"
// right at that one row. The result is a SMALL, ONE-SIDED bias (raymarch
// only ever UNDER-counts near the horizon, never over-counts -- a
// non-converging grazing ray reads as a miss, never a false hit) confined to
// a thin band around the exact horizon row, which is exactly what
// RaymarchSmoke.HoverPadCoverageWithinTwoPercentOfTessellated's loose 2%
// smoke bound exists to tolerate (task brief: "the real bound is measured in
// R9"). Not fixed here because the fix (computing the ray in double, or
// accepting an even larger fixed budget) trades real cost for a gap that is
// already inside this task's own stated tolerance; a tighter bound is R9's
// call to make against real frames, not this task's to guess at.

// Sphere-traces `scene.sdf` per pixel (S7a Task R8) into `target` -- the
// exact alternative to raster_cpu.cpp's tessellated render() for
// DrawMode::raymarch (see this file's own header comment above for the full
// algorithm and its constants).
//
// PRECONDITION: `target` has already passed validate_target(). raster_cpu.
// cpp's render() is the sanctioned public entry point (it validates once,
// before dispatching to either this function or its own rasterizer) and
// this function trusts that precondition rather than re-validating -- the
// same "validated by construction, not by every caller" posture
// world/sdf.hpp's eval()/gradient()/sample() document for a validated
// SdfProgram.
//
// Never fails: a null OR empty `scene.sdf`, or an empty `scene.materials`,
// all degrade to sky (this file's own header comment) -- never an error.
// Result<void> matches render()'s own signature so raster_cpu.cpp's
// dispatch needs no error-shape translation, and is this program's own
// stable-seam convention (scene_from_world()'s identical note, scene.hpp)
// rather than a real failure mode today.
[[nodiscard]] Result<void> render_raymarch(const RenderScene& scene, const Camera& camera, RenderTarget& target);

}  // namespace spade::render
