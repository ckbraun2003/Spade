#pragma once

// ---------------------------------------------------------------------------
// shadow -- a single-sun, depth-only shadow map (S7a Task R7). Adds the one
// missing depth cue in the renderer: materials, directional lighting, a sky
// gradient and an infinite analytic ground (Tasks R1-R6) all landed with
// nothing casting a shadow, so an object and its receiver read as
// unrelated -- floating -- rather than sitting in the same lit space.
//
// FORWARD DECLARATIONS, NOT AN INCLUDE OF scene.hpp: this header is included
// FROM scene.hpp (RenderScene::static_shadow, this task's own field), so it
// must not include scene.hpp back -- that would be a cycle. RenderScene,
// MeshData and DrawItem are only ever taken here by const reference/span,
// neither of which needs a complete type at a DECLARATION -- shadow.cpp,
// which actually reads their members, includes scene.hpp for the complete
// definitions.
//
// ORTHOGRAPHIC, FITTED TO scene.bounds (ruling SR-17): the sun is a
// directional (parallel-ray) light, so its shadow camera is an orthographic
// projection, not a perspective one -- no near/far clip, no perspective
// divide, and the world-to-shadow-space map below is a single AFFINE
// transform. Fitted to `scene.bounds` means the analytic background ground
// (SR-17's own infinite plane, drawn past scene.bounds) is UNSHADOWED beyond
// that box -- accepted and expected, not a bug: shadows fall near the
// objects casting them, which are inside bounds, and extending the shadow
// frustum to cover an infinite plane is explicitly out of this task's scope.
//
// SR-14 (no libm transcendental on an engine/golden-test-feeding path): the
// light-space basis below is built EXCLUSIVELY from cross/dot/normalize
// (IEEE-mandated arithmetic plus a correctly-rounded sqrt) -- zero
// trigonometry, deliberately, in exactly the place a stray trig call is
// tempting (an orthonormal basis from a direction vector).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"

namespace spade::render {

struct RenderScene;
struct MeshData;
struct DrawItem;

// Sentinel: a texel no caster's silhouette has ever covered. Chosen so ANY
// real light-space z (however far from the sun) reads as "an occluder closer
// than this was never found" -- sample_shadow()'s own check below is the one
// place this value is ever compared against. Spelled as the IEEE-754 float32
// bit pattern for -FLT_MAX so this header does not need <cfloat>/<limits> for
// a single constant.
inline constexpr float kNoOccluder = -3.402823466e+38f;

// A depth-only, orthographic shadow map rasterised from the sun's own
// viewpoint. `depth` is `size*size` texels, row-major, each holding the
// LARGEST light-space distance-toward-the-sun (see `light_view_proj`'s own
// comment below) any rasterised caster triangle covered there -- i.e. the
// surface CLOSEST to the sun at that texel, which is exactly what a receiver
// farther from the sun at the SAME texel is occluded BY. A texel `depth`
// still holds kNoOccluder was never covered by any caster, and
// sample_shadow() reads that as unconditionally lit.
//
// `light_view_proj` maps a WORLD-space point p (as glm::vec4(p, 1.0f)) to
// (x_ndc, y_ndc, light_z, 1.0f): x_ndc/y_ndc in [-1, 1] index this map's
// texels (the SAME formula build_static_shadow_map/rasterize_shadow_casters
// use to RASTERISE into `depth` and sample_shadow() uses to QUERY it -- one
// shared matrix is what makes build and sample agree, the same "one function,
// two call sites, same inputs" discipline transform_normal() already
// established in scene.hpp for the R6 seam); light_z is the SAME light-space
// distance-toward-the-sun `depth` stores. Built once, from `scene.bounds` and
// `scene.lighting.sun_direction` alone -- STATIC in the sense that it never
// depends on scene.dynamics (Step 3: dynamics are folded into a per-frame
// COPY of this map, by a second rasterize_shadow_casters() call, never into
// this cached one).
struct ShadowMap {
    std::vector<float> depth;
    uint32_t size = 1024;
    glm::mat4 light_view_proj{1.0f};
};

// Builds a shadow map fitted to `scene.bounds`, rasterising `scene.statics`
// ONLY (never scene.dynamics -- Step 3's own split: a SEPARATE
// rasterize_shadow_casters() call folds dynamics into a per-frame copy of
// this result). Deterministic: iterates scene.statics/each mesh's triangles
// in their own fixed, existing order, no RNG/wall clock/unordered iteration.
//
// Errors: none of this function's own steps can fail today for a validated
// scene (light-basis construction is total for any non-zero sun_direction,
// which scene_from_world()'s own glm::normalize() call already guarantees
// upstream) -- Result<...> is this program's own stable-seam convention
// (scene.hpp's identical note on scene_from_world()), not a real failure
// mode yet.
[[nodiscard]] Result<ShadowMap> build_static_shadow_map(const RenderScene& scene, uint32_t size = 1024);

// Depth-only rasterises `items` (each entry's MeshData looked up via
// `item.mesh_index` into `meshes`, transformed by `item.local_to_world`, the
// SAME lookup/transform convention as raster_cpu.cpp's own draw_mesh_item)
// into `map.depth` IN PLACE, via `map.light_view_proj`. This is the exact
// mechanism build_static_shadow_map uses internally for scene.statics,
// exposed here so raster_cpu.cpp's render() can call it a SECOND time, once
// per frame, on a COPY of the cached static map, to fold in scene.dynamics
// (Step 3). Only ever RAISES a texel's stored occluder (the same "closer
// wins" comparison build_static_shadow_map's own triangle fill uses) --
// calling this on a map build_static_shadow_map already populated only ever
// adds occluders on top, never erases one the static pass found.
void rasterize_shadow_casters(std::span<const MeshData> meshes, std::span<const DrawItem> items, ShadowMap& map);

// SR-24: the per-pixel occlusion test. `world_pos` MUST already be the
// perspective-correct-interpolated world position at the point being shaded
// -- raster_cpu.cpp's own worldPos*invDepth-then-divide-by-invDepth, never a
// screen-space-affine barycentric lerp of a triangle's three vertex
// positions (see this task's own report for why that distinction is
// load-bearing at the scale of a single tessellated ground triangle).
//
// Returns 1.0f (unoccluded) when `world_pos` falls outside the map's own
// ortho footprint (ruling SR-17: unshadowed past scene.bounds, by design) or
// when no caster was ever rasterised at that texel; 0.0f when a closer-to-
// the-sun occluder was found there. Never a value in between -- a single
// texel sample, no filtering. The gap between "occluder" and "this point" is
// compared through a fixed bias (shadow.cpp's own kShadowDepthBias) tuned
// against shadow acne, not exposed here -- callers never see or tune it
// directly.
[[nodiscard]] float sample_shadow(const ShadowMap& map, const glm::vec3& world_pos);

}  // namespace spade::render
