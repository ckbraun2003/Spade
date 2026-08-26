#pragma once

// ---------------------------------------------------------------------------
// raymarch -- the exact SDF reference path (S7a Task R8). Renders the SAME
// world the tessellated rasterizer (raster_cpu.cpp) draws, but by sphere-
// tracing `scene.sdf` directly instead of drawing triangles -- this IS the
// ground truth Task R9 measures the fast tessellated path against, so it
// evaluates the physics side's own SdfProgram (world/sdf.hpp) directly,
// never a second/approximate representation of the same geometry.
//
// FIX ROUND 1 (review): the algorithm (sphere tracing itself -- no
// overshoot, no false hit) was verified correct by construction; the
// CONSTANTS were not -- kRaymarchSurfaceEpsilon at the original 2e-2 (2 cm)
// was rendering a 2 cm OFFSET SURFACE, not the real one, and the fixture
// used to tune it (hover-pad at 96x72, ~0.096 m/pixel) could not see a 2 cm
// error at all. This round retunes both constants against a fixture that
// CAN see them (a bare sphere, whose exact analytic silhouette is known in
// closed form -- see test_render_raymarch.cpp's own SphereAnalytic section),
// adds a near/far-plane bound and a "started inside solid" cull (frustum
// parity with raster_cpu.cpp, below), and adds per-leaf material resolution
// (materials[0]-for-everything was a correctness bug, not merely a visual
// simplification -- see MATERIAL, below).
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
//     background pixel in the same screen row are bit-identical.
//   * The surface normal is world/sdf.hpp's own gradient()/sample(), which
//     already uses the PINNED central-difference stencil the physics
//     collision passes use (kSdfGradientStep, world/sdf.hpp) for every
//     primitive kind without an analytic gradient -- called directly here,
//     never re-derived, so there is no second stencil to disagree with the
//     first one by a different epsilon.
//   * The byte quantizer (to_byte()) and the camera-ray tangent (tan32())
//     are ALSO render/scene.hpp functions now (fix round 1, review Minor 3)
//     -- to_byte() because R9 compares the exact bytes it produces, and
//     tan32() because raster_cpu.cpp's own pre-fix-round private copy did
//     its division in DOUBLE while this file's own pre-fix-round copy did it
//     in float, which could disagree by a ulp for the identical
//     `fov_y_radians` -- a real geometry difference between the two paths'
//     camera rays that had nothing to do with tessellation.
//
// FRUSTUM PARITY WITH raster_cpu.cpp (fix round 1, review IMPORTANT):
// raster_cpu.cpp's ported clipTriangleNearFar rejects any triangle entirely
// closer than `camera.near_plane` or farther than `camera.far_plane`, and
// DrawMode::shaded's SR-13 back-face cull makes a camera fully enclosed by a
// convex solid see nothing (every triangle presents its back face from
// inside). Before this fix, render_raymarch() ignored both `near_plane` and
// `far_plane` entirely: a ray starting inside a solid satisfied
// `d <= 0 <= epsilon` at its very first sample and reported a hit at t=0
// (100% false coverage for a camera embedded in geometry), and nothing
// stopped marching at `far_plane`, so geometry beyond it was rendered when
// raster had already clipped it away. Both are fixed by bounding the march
// to `t IN [near_plane, far_plane]`:
//   * marching starts at t = near_plane, mirroring the near-plane clip
//     (geometry entirely closer than near_plane is invisible in raster and
//     is never sampled here either);
//   * if the FIRST sample (at t = near_plane) already has distance <= 0,
//     the ray origin's visible range starts already inside a solid -- this
//     ray is a miss, full stop, never a hit at t = near_plane. This is the
//     SDF-native analogue of SR-13's back-face cull for "camera inside a
//     convex solid": a sphere tracer can only detect a crossing FROM
//     outside INTO the solid, so a ray that starts already inside has no
//     such crossing to report, exactly mirroring back-face culling's own
//     "there is no front face to see from here" result;
//   * marching stops (miss) once t exceeds far_plane, mirroring the
//     far-plane clip.
// This is also the fix for the OTHER fix-round finding (a miss ray was
// burning the full kRaymarchMaxSteps budget with no escape -- see that
// constant's own comment): the far-plane bound IS the escape distance,
// derived from the same camera the caller already passed in rather than a
// second, invented constant.
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
// MATERIAL (fix round 1 -- SUPERSEDES this file's own original ruling, which
// was wrong): every hit now resolves its OWNING LEAF PRIMITIVE via
// world/sdf.hpp's nearest_leaf_node() -- the SAME branch every CSG operator
// already selects to produce the distance it returns, propagated up the
// stack rather than re-derived by a new nearest-primitive search -- and
// looks that node up in `scene.sdf->node_materials` (SdfProgram's own
// PA-2 array, built exactly for this). Falls back to material index 0 the
// SAME way draw_mesh_item's own submesh-material resolution does: an empty
// `node_materials` array, an out-of-range resolved index, or a
// nearest_leaf_node() miss (kNoSdfLeaf, only possible for an empty program,
// already excluded by `has_sdf` below) all resolve to `scene.materials[0]`.
// The original "materials[0] for every hit" choice was reasoned to be a
// harmless visual simplification covered by Task R9's silhouette-only
// comparison; it is not merely cosmetic. A world with a real material at
// node_materials[0] left arbitrary (any world whose FIRST authored material
// is not what every surface happens to be) can make an ENTIRE raymarched
// silhouette shade with the wrong colour -- including, in the worst case, a
// colour matching the sky reference exactly, which erases 100% of the
// silhouette from a colour-vs-sky classifier. This is exactly the failure
// mode test_render_raymarch.cpp's own
// MaterialMisresolutionAgainstSkyColourNoLongerErasesTheSilhouette test
// constructs and pins.
//
// SCENE-CONTENT SCOPE (fix round 1, review IMPORTANT -- document, don't fix):
// this function marches `scene.sdf` ONLY. `scene.statics`'s PROPS (resolved-
// mesh geometry attached to a body via a caller-supplied NamedMesh,
// render/scene.cpp's own comment on that split) and every entry of
// `scene.dynamics` (built from live BodyPose data, never from `scene.sdf`)
// are INVISIBLE to this reference path, even though both ARE visible in
// raster_cpu.cpp's tessellated render(). This is correct per this task's own
// brief -- "sphere-traces scene.sdf" -- but it is a HARD PRECONDITION for any
// comparison Task R9 makes: a world whose visible geometry includes props or
// dynamic bodies will show real coverage disagreement between the two paths
// that has nothing to do with tessellation accuracy, purely because one path
// can see content the other structurally cannot.
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
// RenderOptions IS NOT A PARAMETER HERE (fix round 1, review Minor 4):
// render_raymarch() takes no RenderOptions at all -- raster_cpu.cpp's
// render() calls it only for `options.mode == DrawMode::raymarch` and never
// forwards the struct, so `options.overlays`/`options.shadows` are SILENT
// NO-OPS for this draw mode: no ground grid, world-bounds box, spawn/body
// markers, or shadow ever appears in a raymarched frame, regardless of what
// a caller sets those fields to. Stated here, not only in raster_cpu.hpp's
// own render() doc comment, because a reader who opens THIS file first
// (e.g. Task R9, comparing frames) should not have to go find that comment
// to learn it.
//
// NO LIBM TRANSCENDENTALS (ruling SR-14): the one trig call this file needs
// (the tangent of half the vertical FOV) is scene.hpp's shared tan32(),
// built from math::sin32/math::cos32. Nothing in this file calls a libm
// transcendental; sphere tracing needs distance evaluation and
// normalization (std::sqrt, IEEE-mandated and fine), not trig.
// ---------------------------------------------------------------------------

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

// ---------------------------------------------------------------------------
// Sphere-tracing constants (task brief Step 2: "a fixed step budget and
// epsilon (constant table, documented)"). Compile-time, never adaptive --
// the same reason kTessellationDefaults (render/tessellate.hpp) is not.
//
// RETUNED IN FIX ROUND 1 against test_render_raymarch.cpp's SphereAnalytic
// fixture (a bare sphere with a closed-form exact silhouette, resolution
// 480x270) -- the ORIGINAL hover-pad-only tuning could not see either
// constant: at 96x72 a hover-pad pixel covers ~0.096 m, so the original 2 cm
// epsilon moved fewer than 0.2 px of any edge, and hover-pad's own coverage
// stat never separated "epsilon too big" from "steps too few". Measured on
// the sphere fixture (radius 0.5 m, camera 1.2 m out, exact analytic
// silhouette 36,085 px of 129,600):
//
//   config (eps / steps / escape)          sphere coverage   vs exact
//   shipped:  2e-2 / 512  / none            39,652 px         +9.88%
//   1e-3 / 1024 / far-plane escape (THIS)   see report's fix-round table
//
// (Exact figures for the chosen row are in this task's own report, fix
// round 1 section -- re-measured, not copied from the review that first
// found the defect.)
inline constexpr uint32_t kRaymarchMaxSteps = 1024;

// A step converges (hits) once distance <= this, in the SAME world-space
// units SdfProgram's own distances use (metres). Two orders of magnitude
// below the original 2e-2 -- close enough to world/sdf.hpp's own
// kSdfGradientStep (1e-3) that the two are now the SAME order of magnitude,
// though they remain deliberately DIFFERENT constants for DIFFERENT reasons
// (that one perturbs a point to estimate a derivative; this one accepts a
// point as ON the surface) -- conflating them would be the "same numbers,
// different provenance" mistake render/raster_cpu.cpp's own
// kOverlayDepthBias comment warns against for an unrelated pair.
inline constexpr float kRaymarchSurfaceEpsilon = 1e-3f;

// Sphere-traces `scene.sdf` per pixel (S7a Task R8) into `target` -- the
// exact alternative to raster_cpu.cpp's tessellated render() for
// DrawMode::raymarch (see this file's own header comment above for the full
// algorithm, its frustum bound, its material resolution, and its
// constants).
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
