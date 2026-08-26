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
// -- a caller comparing differently-sized frames is a caller error, and
// compare_silhouettes() reports total disagreement (1.0) rather than
// faulting, matching AgreementResult's own documented all-disagreement
// default.
//
// DETERMINISM: compare_silhouettes() and assert_no_material_matches_sky()
// read only their explicit parameters -- no RNG, no wall clock, no static
// mutable state, fixed row-by-row/column-by-column scan order (same contract
// as render()/render_raymarch()).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>

#include "core/error.hpp"
#include "render/scene.hpp"    // Lighting, to_byte() (shared quantizer)
#include "render/target.hpp"   // RenderTarget
#include "world/builder.hpp"   // MaterialDesc (Step 1c needs the AUTHORING name, not render::Material)

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

// Step 1c (task-R9-brief.md controller amendments): fails LOUDLY, naming the
// offending material's authoring NAME and INDEX, if any of `materials`'
// base_color -- quantized the same way (to_byte()) either render path
// quantizes it -- is within `tolerance` (per BGR channel, out of 255; the
// default of 8 is comfortably above to_byte()'s own +-0.5/255 rounding noise
// and comfortably below the gap between any two materials that were meant to
// look different) of `sky_reference_rgb`. Takes the AUTHORING-side
// `spade::MaterialDesc` (world/builder.hpp), not render::Material -- the
// render-side struct that reaches compare_silhouettes()'s caller through
// RenderScene::materials has already dropped the name scene_from_world()
// never carries forward, and a failure that cannot say WHICH material is
// wrong is not the loud failure this guard exists to be.
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
                                                           uint32_t sky_reference_rgb, uint8_t tolerance = 8);

}  // namespace spade::render
