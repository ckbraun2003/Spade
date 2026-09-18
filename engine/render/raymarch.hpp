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
// FIX ROUND 2 (review): fix round 1's frustum bound compared RADIAL march
// distance `t` to `far_plane`, but raster clips on CAMERA-SPACE DEPTH
// (`-pc.z`) -- for an off-axis pixel these differ by 1/cos(theta), so a ray
// toward a frame corner escaped up to ~1.55x too early (60 deg FOV, 16:9) and
// missed geometry raster still draws. Fixed by bounding camera-space depth
// directly (FRUSTUM PARITY, below). Also: the "started inside solid is a
// total miss" cull over-reached -- raster's SR-13 only hides the ENCLOSING
// solid itself, not whatever is BEHIND it, so a ray starting inside now
// marches THROUGH that solid (see FRUSTUM PARITY's own "march past" note)
// instead of abandoning the whole ray. Ruling SR-28 additionally moves
// kRaymarchSurfaceEpsilon to 1e-4 (from 1e-3): the reference's own error
// should be negligible against what it measures, not merely smaller than the
// original 2 cm defect -- see that constant's own comment.
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
// FRUSTUM PARITY WITH raster_cpu.cpp (fix round 1, review IMPORTANT; fixed
// AGAIN in fix round 2, review IMPORTANT -- the first attempt was itself
// wrong): raster_cpu.cpp's ported clipTriangleNearFar rejects any triangle
// entirely closer than `camera.near_plane` or farther than `camera.far_plane`
// in CAMERA-SPACE DEPTH (`-pc.z`, the forward distance along the view axis),
// and DrawMode::shaded's SR-13 back-face cull makes a camera fully enclosed
// by a convex solid see nothing (every triangle presents its back face from
// inside). Both are approximated by bounding the march:
//   * DEPTH, NOT RADIAL DISTANCE (fix round 2 -- fix round 1 bounded `t`
//     itself, which is the RADIAL distance travelled along the ray, against
//     `far_plane`; that is only equal to camera-space depth for the exact
//     centre pixel. For a ray at off-axis half-angle theta, depth =
//     t * cos(theta), so a fixed radial cap escapes an off-axis ray up to
//     1/cos(theta) TOO EARLY -- 1.55x at a 60 deg-FOV/16:9 frame corner --
//     missing geometry raster still draws. `dir_cam` (camera-space, unit
//     length) already carries cos(theta) as its own -Z component, so
//     `cos_theta = -dir_cam.z` converts between the two exactly: marching
//     starts at `t = near_plane / cos_theta` (the radial distance at which
//     DEPTH equals near_plane) and stops once `t * cos_theta > far_plane`
//     (equivalently `t > far_plane / cos_theta`, precomputed once per pixel
//     as `t_far` so the per-step check stays a single comparison).
//     RaymarchSmoke.GeometryWithinTheFrustumButBeyondTheOldRadialCapIsStill-
//     Visible (test_render_raymarch.cpp) pins this at a frame corner with
//     `far_plane` deliberately small enough to see the 1.55x error, per the
//     review's own admonition about fixtures that cannot see what they test.
//   * if the FIRST sample (at t = near_plane / cos_theta) already has
//     distance <= 0, the ray origin's visible range starts already inside a
//     solid. Fix round 1 treated this as an unconditional miss for the WHOLE
//     ray, which over-reached: SR-13's own back-face cull hides only the
//     ENCLOSING solid, not whatever sits behind it, so a raster frame with a
//     camera embedded in a thin slab still shows an unoccluded object further
//     along the ray. Fix round 2 instead MARCHES PAST the solid the ray
//     started inside: while a running `cleared_start_solid` flag is false,
//     each step advances by `max(|distance|, epsilon)` (the floor guards
//     against a degenerate exact-zero sample on an axis-aligned face
//     stalling progress) and registers no hit, however small `|distance|`
//     gets -- a sphere tracer can only detect a genuine crossing, and
//     re-crossing back OUT of the solid it started behind is not a new
//     surface to report. Once a sample's distance exceeds epsilon, the flag
//     flips permanently and ordinary exterior marching (hit when
//     distance <= epsilon) resumes from there, now free to find real
//     geometry further along the ray.
//     RaymarchSmoke.CameraInsideASolidWithAnotherObjectBehindItSeesThe-
//     SecondObject (test_render_raymarch.cpp) is the two-object fixture the
//     review asked for -- the original one-object fixture
//     (CameraFullyInsideAConvexSolidSeesNothingLikeRastersBackFaceCull)
//     cannot distinguish "the whole ray is abandoned" from "the enclosing
//     solid is correctly skipped and there is nothing else to find" (both
//     read as 100% sky), which is exactly why it did not catch the
//     over-reach the first time.
// Both bounds together are also what makes a miss ray terminate promptly
// instead of burning the full kRaymarchMaxSteps budget (fix round 1's other
// finding): the far-plane depth bound converts to a finite radial cap for
// every pixel, derived from the camera the caller already passed in rather
// than a second, invented distance constant.
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
// KNOWN, ACCEPTED LIMITATION -- A NEAR-HORIZONTAL RAY OVER AN INFINITE GROUND
// CAN LOSE THE HORIZON ROW (fix round 2, review MINOR -- diagnosed by
// measurement, replacing fix round 1's own INCORRECT "float-vs-double
// precision" guess for the identical symptom, which this file no longer
// makes since it cannot back it up): on
// RaymarchSmoke.HoverPadCoverageWithinTwoPercentOfTessellated's own 96x72
// camera, the entire raymarch/raster coverage deficit is EXACTLY ONE screen
// row -- row 13, the true optical horizon for that camera (y=3, pitch -20
// deg, 60 deg FOV) -- where raster covers 96/96 pixels and raymarch covers
// 0/96, with every OTHER row agreeing to the pixel. This is NOT the far-plane
// depth bound above: sweeping `far_plane` from 1000 to 1e7 leaves the deficit
// at exactly 96 px. The row's ray descends only ~0.15 deg below horizontal
// and meets the infinite ground plane at radial t ~= 1145 m -- past the
// default 1000 m far_plane, AND past what kRaymarchMaxSteps can reach at all
// (sphere tracing a near-parallel ray toward a plane converges geometrically
// with ratio (1 - cos(theta)) per step, and this row's shallow descent angle
// needs roughly 3000 steps to close the gap; EITHER cause alone is
// sufficient, independent of the other). Raster's own analytic ground
// (ruling SR-17) has no such bound -- a ray-plane intersection is a single
// division, not an iterated convergence -- so it paints the row raymarch
// cannot reach. The deficit is a NARROW, ONE-SIDED band (raymarch only ever
// UNDER-counts near the horizon, never over-counts) that WIDENS with screen
// resolution as the horizon spans more discrete rows: one row at 96x72 and
// 192x144; at 480x270, a row-by-row scan (fix round 3, correcting an earlier
// draft's stale "~2.2 rows" figure, which had been carried over from an
// initial estimate and mislabelled as measured) found rows 50-52 fully
// deficient (480/480 each) and row 53 partially recovered (460/480) -- 1,460
// px total, ~3.04 row-equivalents (1,460 / 480). Not fixed here -- a fix
// (a much larger step budget, or an analytic ground special-case that would
// reintroduce exactly the raster-only device this file's own header comment
// says it must not need) trades real cost or real duplication for a gap the
// task's own loose smoke bound already tolerates; Task R9 should expect a
// systematic horizon-band deficit on every world with an infinite ground
// primitive and account for it rather than chase it as a bug.
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
// SR-17a IS FORWARDED EXPLICITLY (2026-09-17), and it is the ONE exception to
// the paragraph below -- stated here rather than as a silent third parameter.
// `horizon_strength`/`horizon_onset` arrive as two floats, not as RenderOptions,
// precisely so the note below stays true of everything else. The distinction is
// that overlays and shadows are things this path CANNOT do; the atmospheric term
// is something it MUST do, because RS4 compares this path against raster within
// a pinned band and a term applied on one side only widens that band
// systematically on every world with geometry at range. Defaulted to 0.0f so a
// direct caller that has no opinion gets the pre-SR-17a frame EXACTLY.
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
// RETUNED IN FIX ROUND 1, then AGAIN in fix round 2 (ruling SR-28) against
// test_render_raymarch.cpp's SphereAnalytic fixture (a bare sphere with a
// closed-form exact silhouette, resolution 480x270) -- the ORIGINAL
// hover-pad-only tuning could not see either constant: at 96x72 a hover-pad
// pixel covers ~0.096 m, so the original 2 cm epsilon moved fewer than
// 0.2 px of any edge, and hover-pad's own coverage stat never separated
// "epsilon too big" from "steps too few" (confirmed directly: hover-pad
// reads the identical 1.69% at every epsilon this file has ever shipped).
// Measured on the sphere fixture (radius 0.5 m, camera 1.2 m out, exact
// analytic silhouette 36,080 px of 129,600):
//
//   config (eps / steps / escape)             sphere coverage   vs exact
//   shipped:        2e-2 / 512  / none          39,652 px        +9.88%
//   fix round 1:    1e-3 / 1024 / far-plane     36,224 px        +0.40%
//   fix round 2:    1e-4 / 1024 / far-plane     see report's fix-round-2 table
//
// (Exact figures for the chosen row are in this task's own report, fix
// round 2 section -- re-measured, not copied from the review that swept
// these values.)
inline constexpr uint32_t kRaymarchMaxSteps = 1024;

// A step converges (hits) once distance <= this, in the SAME world-space
// units SdfProgram's own distances use (metres).
//
// RULING SR-28 (fix round 2): moved from 1e-3 (fix round 1) to 1e-4. The
// reference path's OWN error against the true surface should be NEGLIGIBLE
// against what it is measuring, not merely smaller than the original 2 cm
// defect -- at 1e-3 the sphere fixture's own +0.40% over-coverage was
// running at ~26% of the SAME signal R9 pins Task R6's tessellation error
// against (measured -1.54% under-coverage there), with the OPPOSITE sign,
// which contaminates every band comparison rather than merely adding noise
// to it. At 1e-4 the sphere fixture reads +0.022% -- about 1.4% of that
// signal, and 20x inside this file's own committed 0.5% assert instead of
// 25x. Deliberately a DIFFERENT constant from world/sdf.hpp's own
// kSdfGradientStep (1e-3, one order of magnitude ABOVE this value) for a
// DIFFERENT reason: that one perturbs a point to estimate a DERIVATIVE and
// wants to be small relative to surface curvature; this one accepts a point
// as ON the surface and wants to be small relative to what a comparison
// against a DIFFERENT rendering technique's own error can resolve --
// conflating the two would be the "same numbers, different provenance"
// mistake render/raster_cpu.cpp's own kOverlayDepthBias comment warns
// against for an unrelated pair. Re-verified
// at this value that kRaymarchMaxSteps=1024 still converges the sphere
// fixture comfortably (a tighter epsilon needs more steps to reach, in
// general) -- see the table above.
inline constexpr float kRaymarchSurfaceEpsilon = 1e-4f;

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
[[nodiscard]] Result<void> render_raymarch(const RenderScene& scene, const Camera& camera, RenderTarget& target,
                                            float horizon_strength = 0.0f, float horizon_onset = 45.0f);

}  // namespace spade::render
