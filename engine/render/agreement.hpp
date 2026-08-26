#pragma once

// ---------------------------------------------------------------------------
// agreement -- the RS4 visual/physics agreement harness (S7a Task R9). This is
// the machinery, not the numbers: compare_silhouettes() is a pure function
// that turns two already-rendered frames of the SAME scene/camera (one drawn
// by the fast tessellated path, render/raster_cpu.cpp's DrawMode::shaded; one
// drawn by the exact SDF reference, render/raymarch.cpp's DrawMode::raymarch)
// into a single number: how much of the frame disagrees about whether
// there IS something there at all. The measured bands this task pins live in
// tests/golden/render/agreement_bands.json and tests/test_render_agreement.cpp
// -- this header is deliberately silent about any specific world's threshold.
//
// COVERAGE ONLY, NEVER COLOUR (RS4's own contract, restated from
// render/raymarch.hpp): shading -- and specifically shadows, which the
// raymarch path never casts by design (ruling SR-25) -- legitimately differs
// between the two paths. compare_silhouettes() never compares a hit pixel's
// colour against the OTHER frame's colour at that pixel; it only asks, of
// EACH frame independently, "is this pixel the sky, or not" and compares
// those two yes/no answers. Two frames that agree on every silhouette edge
// but disagree on every lit pixel's exact shade would report 0.0.
//
// WHY A SINGLE FLAT sky_reference_rgb, NOT A GRADIENT (the load-bearing
// design decision in this file -- read before calling compare_silhouettes()
// on a scene you built yourself): render/scene.hpp's sky_gradient_color()
// varies PER SCREEN ROW (zenith at the top, horizon at the bottom), and
// compare_silhouettes() is given no Lighting and no row index -- only one
// packed reference colour, applied to every pixel in the frame. That is only
// a correct "is this the sky" test if the two frames being compared were
// rendered from a scene whose sky is FLAT (sky_horizon == sky_zenith).
// Skipping that step breaks classification in BOTH directions at once, not
// just one:
//   * A pixel that is a genuine miss in ONE path but a genuine hit in the
//     OTHER (e.g. raymarch.hpp's own documented horizon-band deficit, or a
//     real tessellation gap this task exists to catch) has, on the miss
//     side, the row's own gradient colour -- which will almost never equal a
//     single fixed reference except at the one row that happens to match it.
//     That miss pixel then reads as "covered" too, and the real disagreement
//     this task is built to measure CANCELS ITSELF OUT before
//     compare_silhouettes() ever sees it. This is a silent, not a loud,
//     failure -- the reported disagreement_fraction is simply smaller than
//     the truth, in the direction that looks like success.
//   * Conversely, a reference that DOES coincide with a row's gradient
//     colour purely by chance would misclassify every genuinely-covered
//     pixel in that row as sky in BOTH frames at once -- reproducing, one
//     level up, the exact SR-27-era catastrophe this file's
//     assert_no_material_matches_sky() below exists to catch for materials.
// pack_sky_reference_bgrx() below does not flatten a Lighting for you --
// only pack the (single) colour you point it at. Flattening is the caller's
// job (mutate `RenderScene::lighting.sky_horizon = lighting.sky_zenith`, or
// build a Lighting with the two already equal, BEFORE rendering either
// frame) -- documented here because it is a precondition compare_silhouettes()
// cannot check after the fact: by the time it sees two pixel buffers, the
// gradient (or its absence) has already been baked into every background
// byte.
//
// DISAGREEMENT IS A SPATIAL (SYMMETRIC-DIFFERENCE) MEASURE, NOT A RAW
// COUNT DIFFERENCE: disagreement_fraction is the fraction of pixels where
// the two paths' covered/not-covered answers DIFFER (a per-pixel XOR),
// divided by the frame's total pixel count -- not
// |covered_a - covered_b| / covered_b, which this file deliberately does NOT
// compute as the headline number (covered_a/covered_b are reported alongside
// it precisely so a caller CAN compute that ratio too, for comparison against
// R8's own smoke bounds). A raw count difference is blind to WHERE the
// disagreement is: two silhouettes of equal total area that do not overlap
// at all would report a count difference of exactly zero, the same "fixture
// that cannot see the thing it checks" failure this program's own lessons
// (task-R9-brief.md's "failure mode this task has hit three times") warn
// against -- generalized from a fixture to the metric itself. A per-pixel
// XOR cannot be fooled that way: it is exactly the frame area where the two
// paths disagree about coverage, however the areas happen to be shaped or
// arranged.
//
// PRECONDITION: both `fast` and `raymarched` have already passed
// validate_target() (this task's own caller renders them via render(), the
// sanctioned public entry point, which validates once before dispatching --
// render_raymarch()'s own precondition note) and share the SAME width/height
// -- a caller comparing differently-sized frames is a caller error.
// compare_silhouettes() reports `AgreementResult{}` (its own default-
// constructed sentinel: disagreement_fraction = 1.0, covered_a = covered_b =
// 0) rather than faulting. Read that literally, not as "every pixel is
// covered" (an earlier revision of this comment said exactly that, which
// contradicts covered_a/covered_b both being 0 -- fixed at Task R9 fix round
// 1, review finding): it means "this input could not be meaningfully
// compared at all, so nothing was measured on either side, and the
// disagreement is reported at its maximum (1.0) so a caller who forgets to
// check for this case fails closed rather than silently passing."
//
// DETERMINISM: compare_silhouettes(), assert_no_material_matches_sky() and
// strip_to_ground_plane_only() read only their explicit parameters -- no
// RNG, no wall clock, no static mutable state, fixed row-by-row/column-by-
// column scan order (same contract as render()/render_raymarch()).
//
// SR-30 (task-R9-brief.md fix round 1): a band that a case would pass with
// its reference's ENTIRE geometry deleted is not measuring that case's
// geometry at all -- it is measuring the raymarch reference's OWN horizon-
// band deficit against itself, restated as a threshold. strip_to_ground_
// plane_only() below is the probe every matrix case must run at pin time
// (and re-check at test time, task-R9-report.md's fix-round-1 section): keep
// ONLY the standalone ground-plane leaf a world's SDF program authors,
// delete everything else, and compare the REAL fast render against THAT
// reference. A case whose disagreement does not move past its own pinned
// band under this mutation proves nothing about scene geometry -- see
// tests/golden/render/agreement_bands.json's `detects_total_deletion` field,
// which records the outcome test_render_agreement.cpp's own probe computes,
// not a claim asserted without checking.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>

#include "core/error.hpp"
#include "render/scene.hpp"    // Lighting, to_byte(), shade_vertex_color() (shared quantizer + shading model)
#include "render/target.hpp"   // RenderTarget
#include "world/builder.hpp"   // MaterialDesc, WorldDesc (Step 1c needs the AUTHORING name, not render::Material)

namespace spade::render {

struct AgreementResult {
    double disagreement_fraction = 1.0;
    uint32_t covered_a = 0, covered_b = 0;
};

// Coverage/silhouette comparison ONLY -- shading legitimately differs between
// the paths (RS4). See this file's header comment for the flat-sky
// precondition and why disagreement_fraction is a per-pixel symmetric
// difference, not a count ratio.
[[nodiscard]] AgreementResult compare_silhouettes(const RenderTarget& fast, const RenderTarget& raymarched,
                                                   uint32_t sky_reference_rgb);

// Packs a SINGLE reference colour (BGRX8, alpha/X forced to 0xFF -- the same
// byte layout render()/render_raymarch() write, MN-14) from `lighting`,
// quantized through render/scene.hpp's shared to_byte() -- the SAME
// quantizer either render path uses, so a scene whose sky really is flat
// packs to the EXACT bytes both paths will write for a miss. Reads ONLY
// sky_zenith -- see this file's header comment: the caller is responsible
// for having already flattened sky_horizon to match before rendering either
// frame, not this function, which has no way to tell a deliberately flat sky
// from one the caller forgot to flatten.
[[nodiscard]] uint32_t pack_sky_reference_bgrx(const Lighting& lighting);

// Step 1c (task-R9-brief.md controller amendments; SIGNATURE AND BEHAVIOUR
// CHANGED at Task R9 fix round 1, review IMPORTANT -- see below): fails
// LOUDLY, naming the offending material's authoring NAME and INDEX, if any
// of `materials` can produce a SHADED colour within `tolerance` (per BGR
// channel, out of 255; the default of 8 is comfortably above to_byte()'s own
// +-0.5/255 rounding noise and comfortably below the gap between any two
// materials that were meant to look different) of `sky_reference_rgb` for
// SOME achievable surface orientation. Takes the AUTHORING-side
// `spade::MaterialDesc` (world/builder.hpp), not render::Material -- the
// render-side struct that reaches compare_silhouettes()'s caller through
// RenderScene::materials has already dropped the name scene_from_world()
// never carries forward, and a failure that cannot say WHICH material is
// wrong is not the loud failure this guard exists to be.
//
// FIX ROUND 1 (review IMPORTANT): THE ORIGINAL VERSION OF THIS FUNCTION
// CHECKED THE WRONG QUANTITY. It compared `MaterialDesc::base_color`
// directly against `sky_reference_rgb` -- but compare_silhouettes()'s
// classifier never sees an authored base_color; it sees
// render/scene.hpp's shade_vertex_color()'s OUTPUT, `base * (sun_term +
// ambient_color)` for a lambert material (base_color verbatim only for
// unlit/emissive). A material can clear a base_color-only tolerance check by
// a wide margin -- opaque white against a blue sky, say -- and STILL shade to
// exactly the sky colour at whatever surface normal a real hit happens to
// present, because `sun_term` scales continuously with N.L from 0 (grazing/
// shadowed) to `sun_color * sun_intensity` (fully lit) as the normal varies
// across a world's actual geometry. This function now calls the SAME
// production `shade_vertex_color()` (render/scene.hpp) at a dense sweep of
// normals spanning the full ACHIEVABLE N.L range [0, 1] -- never re-deriving
// the Lambert formula -- and flags a material only if some SINGLE sampled
// normal lands within tolerance on ALL THREE channels AT ONCE. That "at
// once" is load-bearing: checking each channel's own achievable range
// independently (an earlier draft of this fix) is UNSOUND -- it flags any
// material whose base_color is grey-ish against a saturated (non-grey) sky,
// because a shared, single N.L drives every channel together, and R6/R7's
// own DEFAULT white material against the DEFAULT blue-ish sky is exactly
// such a false positive (white can only ever shade to a value with
// r == g == b, and (0.3, 0.5, 0.8) is not one, at ANY N.L -- verified by
// solving each channel's own required N.L independently and finding they
// disagree: 0.2 / 0.4 / 0.7, never simultaneously achievable). Degenerates
// to exactly the original base_color check for unlit/emissive materials
// (shade_vertex_color() ignores the normal entirely for those, so every
// sampled point collapses to the same value) -- this is a strict
// generalization, not a second, parallel check.
//
// WHY THIS MATTERS: `materials[0]` is the builder's own default-material
// slot (WorldDesc's own comment: "always exists once a WorldDesc has passed
// validate_world_desc()") -- a world that authors its real materials
// starting at index 1 and never touches index 0 leaves it at whatever the
// builder's own default happens to be, which is reachable by ordinary
// authoring, not by malice. Before ruling SR-27, EVERY raymarched hit
// resolved to materials[0] regardless of which primitive it actually hit;
// SR-27 fixed the raymarch side (per-leaf material resolution), but the
// fragility is inherent to compare_silhouettes()' colour-based
// classification and can still bite either path's own genuine
// materials[0]-authored geometry, or the raster path's per-submesh material
// 0 fallback (render/scene.hpp's SR-11 empty-submesh convention).
[[nodiscard]] Result<void> assert_no_material_matches_sky(std::span<const spade::MaterialDesc> materials,
                                                           const Lighting& lighting, uint32_t sky_reference_rgb,
                                                           uint8_t tolerance = 8);

// SR-30's probe (task-R9-brief.md fix round 1): a copy of `world` with every
// SDF node removed EXCEPT the one standalone ground-plane leaf (a node with
// op == SdfOp::none and kind == SdfPrim::plane) -- a single `prim` node is a
// trivially valid one-element postfix program (one push, stack depth 1,
// nothing left to pop). Everything else (materials, lighting, spawns,
// capacities, props, visual_refs, environment) is copied verbatim, and
// `transforms` is left FULLY INTACT (the surviving node's own `transform`
// index still resolves into it unchanged) -- only `nodes` (and, if present,
// the parallel `node_materials`) is truncated to the one surviving entry.
//
// This is a PRODUCTION utility, not test-only code, for the reason SR-2
// already states: Task C4 will dress these same worlds with prefab instances
// and needs to re-run this EXACT probe against the dressed worlds, not a
// second, independently-written "delete everything" helper that could drift
// from this one.
//
// Codes: not_found -- `world.sdf` has no standalone ground-plane leaf to
// keep (every one of today's ten shipped worlds has exactly one; a caller
// that has verified `raster_cpu.cpp`'s own analytic-ground extraction found
// one -- RenderScene::ground_planes non-empty -- will never hit this).
[[nodiscard]] Result<spade::WorldDesc> strip_to_ground_plane_only(const spade::WorldDesc& world);

}  // namespace spade::render
