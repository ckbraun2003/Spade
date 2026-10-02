#pragma once

// ---------------------------------------------------------------------------
// RenderScene -- the renderer's own view of a world (S7a Task R1). Built once
// per world load from a spade::WorldDesc plus a set of already-resolved
// visual meshes; refreshed once per frame from a tick-boundary copy of body
// poses. This is the seam between "what the SDF program/physics know" and
// "what a draw pass needs" -- everything downstream (the rasterizer, a
// headless renderer, a training harness's frame dump) reads only this
// struct, never the WorldDesc or the live simulation state directly.
//
// PURITY (binding global constraint): scene_from_world() and
// update_dynamics() are PURE functions of their explicit inputs -- no
// simulation state, no RNG, no wall clock, no static mutable state anywhere
// in this module. update_dynamics() takes `RenderScene&` because it writes
// into an existing container (no per-frame reallocation of the whole scene),
// not because its OUTPUT depends on the scene's prior contents: `dynamics`
// is rebuilt from scratch from `bodies` on every call, so the result never
// depends on anything the scene held before the call.
//
// SR-9 (controller ruling, task-R1-brief.md; REVISED at Task R5): one
// DrawItem per entry of render/csg_mesh.hpp's split_program() -- one per
// primitive leaf that sits under nothing but `union` (Task R2's
// tessellate_primitive() fills its mesh), and one per CSG root
// (subtract/intersect/smooth_union -- Task R5's mesh_csg_subtree() fills
// its mesh, the WHOLE subtree collapsed into a single draw item). The two
// output lists are merged back into the program's own left-to-right
// authoring order (ascending node index) before becoming `statics`, so a
// world's draw order is reproducible from the program alone and does not
// depend on which of the two populations a given piece of geometry happens
// to fall into.
//
// This SUPERSEDES Task R1's original one-non-op-node-one-EMPTY-item mapping:
// a CSG root's own node IS an operator node (op != SdfOp::none), yet it now
// gets exactly one draw item, while the primitive leaves consumed into its
// subtree get none of their own -- the node/item correspondence is
// split_program()'s, not "is this node a primitive leaf".
//
// MESH INDEX SPACE (this task's own design decision, driven by
// task-H1-brief.md's plan for how the eventual host wires a body to its
// visual mesh): `resolved_meshes` -- geometry the CALLER already produced by
// resolving each of a world's visual references to a file (see NamedMesh's
// own comment) -- is copied into `RenderScene::meshes` VERBATIM, IN SPAN
// ORDER, at the FRONT (indices [0, resolved_meshes.size())). The SDF-node-
// derived (currently empty) placeholder meshes follow immediately after, one
// per static DrawItem, in program order. This keeps a caller's own index
// into the `resolved_meshes` vector it built (e.g. "which loaded mesh does
// this body's model type reference") numerically identical to the matching
// index into `RenderScene::meshes` -- BodyPose::mesh_index is exactly that
// index, with no renumbering owed by either side of the seam.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "core/fp32_math.hpp"   // math::sin32/cos32 (S7a Task R8) -- tan32() below, SR-14
#include "render/shadow.hpp"   // ShadowMap (S7a Task R7) -- forward-declares RenderScene itself, no cycle
#include "render/target.hpp"   // Aabb, kNoMaterial, kNoMesh (Task 0) -- do not redeclare
#include "world/builder.hpp"   // WorldDesc
#include "world/sdf.hpp"       // SdfProgram

namespace spade::render {

// Transforms a LOCAL unit surface normal to WORLD space and re-normalizes
// (S7a Task R6). For a rigid + UNIFORM-scale local_to_world (SdfTransform's
// own contract, world/sdf.hpp), mat3(local_to_world) applied to a local
// direction yields that direction correctly ROTATED and then scaled by the
// uniform factor (the same relationship world/sdf.hpp's own gradient_world
// derivation states for the SDF gradient) -- normalizing removes that scale,
// leaving exactly the rotated normal.
//
// Used IDENTICALLY by two call sites that must agree bit-for-bit (SR-17's
// load-bearing seam): the tessellated-mesh per-vertex normal transform
// (raster_cpu.cpp's draw_mesh_item, shading a real triangle) and the
// analytic background ground plane's precomputed world normal
// (scene_from_world()'s ground-plane extraction, scene.cpp) -- for a
// standalone ground-plane node, both call this SAME function with the SAME
// local_to_world and the SAME local normal, so the two paths' shading
// agrees exactly, not merely approximately.
[[nodiscard]] inline glm::vec3 transform_normal(const glm::mat4& local_to_world, const glm::vec3& local_normal) {
    return glm::normalize(glm::mat3(local_to_world) * local_normal);
}

struct Material {
    glm::vec4 base_color{0.72f, 0.72f, 0.74f, 1.0f};
    uint32_t shading = 0;   // 0 = lambert, 1 = unlit, 2 = emissive
};

struct Lighting {
    glm::vec3 sun_direction{-0.35f, -0.86f, -0.37f};   // normalised, world space
    glm::vec3 sun_color{1.0f, 0.98f, 0.94f};
    float sun_intensity = 1.0f;
    glm::vec3 ambient_color{0.30f, 0.34f, 0.42f};
    glm::vec3 sky_zenith{0.28f, 0.42f, 0.62f}, sky_horizon{0.68f, 0.74f, 0.80f};
};

// ---------------------------------------------------------------------------
// Shading (S7a Task R6, ruling SR-18; relocated HERE at Task R8 -- see below)
// -- the pure function that turns a resolved Material, the scene's Lighting,
// and a WORLD-SPACE surface normal into a linear-light RGB colour. Originally
// a raster_cpu.cpp-private helper called from exactly two places (ONCE PER
// VERTEX by the Gouraud fill, SR-18, and ONCE PER BACKGROUND PIXEL by the
// analytic ground pass, SR-17 -- the SAME function both times, which is what
// makes that tessellated-grid/analytic-ground seam agree bit-for-bit whenever
// its three inputs do, transform_normal()'s own comment above). Task R8's
// exact SDF raymarcher is a THIRD call site with the identical requirement
// (its own brief: "shade with the same material/lighting model as R6 ...
// reuse this rather than reimplementing Lambert") -- moved to this shared
// header, unchanged, rather than duplicated a second time, so all three stay
// bit-for-bit identical by construction, not by two (or three) independent
// implementations happening to agree.
//
// unlit/emissive (MaterialShading 1/2, world/builder.hpp) both echo
// base_color verbatim -- "no lighting applied" and "treated as emitted
// radiance" amount to the same output today, since this renderer has no
// tonemap/bloom pass yet to tell emissive apart from unlit.
//
// lambert (0, the default, and the fallback for any other stored value):
// N.L clamped to >= 0 (Step 1's own "neither is pure black" ambient floor
// holds as long as ambient_color is nonzero -- a face pointing away from the
// sun still gets the ambient term, just never the sun term) times
// sun_color*sun_intensity, plus a flat ambient_color term. `n_world` need
// not be unit: a near/far-clip-interpolated normal (raster_cpu.cpp's
// ClipVertex) is a lerp of two unit vectors and is deliberately not
// re-normalized -- a slightly-non-unit vector here is a tiny cosine-law
// approximation right at a clipped triangle's edge, never something this
// function needs to correct. Task R8's raymarch caller passes an already-
// re-normalized SDF gradient instead (its own call site normalizes it, since
// gradient magnitude is only guaranteed 1 where the field is an exact metric
// and differentiable, world/sdf.hpp), so this function's "need not be unit"
// tolerance is exercised, never relied upon, from that side.
// Pins the magic 1u/2u literals below against world::MaterialShading's own
// values (world/builder.hpp) -- review MINOR 7 (Task R6): `unlit == 1` is
// already exercised end to end by
// RenderShading.UnlitMaterialIgnoresSunDirectionEntirely, but nothing
// previously pinned `emissive == 2` anywhere, so a future reordering of that
// enum would silently swap emissive's behaviour with lambert's and no test
// would catch it.
static_assert(static_cast<uint32_t>(spade::MaterialShading::unlit) == 1u,
              "shade_vertex_color's magic 1u must match MaterialShading::unlit");
static_assert(static_cast<uint32_t>(spade::MaterialShading::emissive) == 2u,
              "shade_vertex_color's magic 2u must match MaterialShading::emissive");

// Ruling SR-25 (S7a Task R7, fix round 1 -- corrects the original SR-24
// wording): a shadow attenuates the SUN term only. Ambient is never
// shadowed -- it models indirect/sky light, which a single-occluder sun
// shadow says nothing about. `combined` is the value this function has
// always returned (base * (sun_term + ambient_color)) -- every caller that
// never touches a shadow map (Task R8's raymarcher included -- shadows are
// deliberately out of that task's scope, see render/raymarch.hpp) uses
// `combined` alone. `sun` isolates the term a shadow is allowed to touch:
// `base * sun_term`, exactly zero for unlit/emissive (which have no sun term
// at all) -- so a shadow is a mathematical no-op on those materials by
// construction, not a branch anyone had to add for it.
struct ShadedColor {
    glm::vec3 combined;  // ambient + sun*N.L, base_color-scaled -- unshadowed value
    glm::vec3 sun;         // JUST the sun*N.L contribution -- the only thing a shadow may attenuate
};

[[nodiscard]] inline ShadedColor shade_vertex_color(const Material& material, const Lighting& lighting,
                                                     const glm::vec3& n_world) {
    const glm::vec3 base(material.base_color);
    if (material.shading == 1u || material.shading == 2u) {  // unlit, emissive: no sun term to shadow
        return ShadedColor{base, glm::vec3(0.0f)};
    }
    const float n_dot_l = std::max(glm::dot(n_world, lighting.sun_direction), 0.0f);
    const glm::vec3 sun_term = lighting.sun_color * (lighting.sun_intensity * n_dot_l);
    const glm::vec3 combined = base * (sun_term + lighting.ambient_color);
    return ShadedColor{combined, base * sun_term};
}

// The zenith-to-horizon sky gradient (S7a Task R6, ruling SR-23; relocated
// HERE at Task R8 for the identical reason shade_vertex_color() was above
// it; made ELEVATION-based at Task VQ-A, a fix round for the user's
// verdict). `ray_direction_world` is the WORLD-SPACE camera ray a
// background/sky-miss pixel casts -- need NOT be pre-normalized (only its
// direction matters; this function divides by its own length itself), so a
// caller that already has an un-normalized ray handy (raster_cpu.cpp's own
// background pass, which deliberately never normalizes its background ray --
// see draw_sky_and_ground_background()'s own comment, and S7a Task VQ-B's
// background_ray_basis() comment for how that ray is now built) does not
// need to spend an extra sqrt unit-normalizing it just to call this.
//
// `dir.y / |dir|` is exactly sin(elevation) -- the ray's angle above the
// horizontal (Y=0) plane -- which is monotone over the entire +/-90 degree
// range any camera ray can span, and needs exactly one std::sqrt (folded
// into the length below), IEEE-mandated (correctly rounded) and explicitly
// sanctioned by SR-14. No inverse-trig call (atan2/asin) is needed at all --
// an earlier version of this file's comment claimed one would be, which was
// established false and is the reason this ruling exists: the OLD gradient
// interpolated on a plain fraction of SCREEN ROW instead, which anchors the
// horizon COLOUR to the bottom screen row and drifts from the ray-cast
// horizon LINE the ground hit-test actually draws the moment the camera
// pitches (measured directly in the review book's own frames).
//
// `horizon_fraction` is 0 straight up (elevation == 1, pure zenith) and
// clamped to 1 AT and BELOW the true horizon (elevation <= 0) -- rather than
// letting the lerp overshoot past sky_horizon for a downward-pitched ray --
// so a pixel below the horizon with no ground plane to draw over it (or one
// the analytic/tessellated hit test does not cover) reads as flat horizon
// colour, never an out-of-gamut extrapolation. This is what keeps "hard
// horizon, no fog" (SR-17) true for the SKY half of the boundary; the GROUND
// half is still entirely the analytic-ground/tessellated-mesh hit test's own
// job, never this function's.
//
// Both render paths call this on the SAME per-pixel ray direction they
// already reconstruct for their own hit-testing (raster_cpu.cpp's
// draw_sky_and_ground_background, raymarch.cpp's per-pixel camera ray) --
// sharing this one function on that one shared input is what keeps the two
// paths' sky agreeing bit-for-bit rather than by two coincidentally-equal
// expressions (this task's own controller amendment: "share that code
// rather than writing a second gradient").
//
// LERP FORM MATTERS (found running Step 7's own re-measure): `zenith +
// (horizon - zenith) * horizon_fraction`, NOT `zenith*(1-t) + horizon*t`.
// Both are the same real-valued interpolation, but only the first is
// EXACT -- bit-for-bit, for every finite `horizon_fraction` -- when
// `sky_zenith == sky_horizon`. `horizon - zenith` is then exactly the zero
// vector (IEEE subtraction of two bit-identical values is exact), zero
// times any finite fraction is exactly zero, and `zenith + zero` is exactly
// `zenith` (adding zero never rounds). The `a*(1-t)+a*t` form has no such
// guarantee -- two separate roundings (each multiply) plus a third (the
// add) do not generally telescope back to bit-identical `a`, so two
// callers computing `t` along different floating-point paths (raster's
// background pass narrows a DOUBLE-precision ray to float;
// raymarch's is float throughout; a test oracle may use double
// throughout) could each round a per-pixel FLAT sky to a *different* byte
// whenever that byte sits exactly on a to_byte() rounding half-boundary --
// turning R9's "flatten the sky, then compare by exact colour" agreement
// strategy into a source of spurious per-pixel disagreement having nothing
// to do with geometry. The exact form removes the possibility entirely,
// independent of how `horizon_fraction` was computed or by whom.
[[nodiscard]] inline glm::vec3 sky_gradient_color(const Lighting& lighting, const glm::vec3& ray_direction_world) {
    const float len = std::sqrt(glm::dot(ray_direction_world, ray_direction_world));
    const float elevation = len > 0.0f ? ray_direction_world.y / len : 1.0f;
    const float horizon_fraction = std::clamp(1.0f - elevation, 0.0f, 1.0f);
    return lighting.sky_zenith + (lighting.sky_horizon - lighting.sky_zenith) * horizon_fraction;
}

// ---------------------------------------------------------------------------
// THE INFINITE GROUND GRID, and the horizon blend. Both are SHARED per-ray
// functions in exactly the sense sky_gradient_color is, and for the same
// reason: raster and raymarch must produce bit-identical background pixels,
// and the only way to guarantee that is ONE function called on the ONE ray
// each path already reconstructs -- never two expressions that happen to
// agree. (Task R9's controller amendment: "share that code rather than
// writing a second gradient.")
//
// ⚠ NO TRANSCENDENTALS. The rendered frame feeds a sha256 golden, so standing
// rule 1 applies here exactly as it does to physics: `+ - * / sqrt` only, plus
// the exactly-rounded floor/abs this file's callers already use. Every fade
// below is therefore RATIONAL, never exponential -- an exp() here would be a
// cross-libm coin flip on a byte a manifest pins across platforms.
//
// ⭐ AND EVERY BLEND USES THE `base + (other - base) * t` FORM, never
// `base*(1-t) + other*t`. sky_gradient_color's own comment explains why: only
// the first is EXACT when t is zero or when the two colours are equal --
// `other - base` is then exactly the zero vector, zero times a finite t is
// exactly zero, and adding zero never rounds. That makes "grid off" and
// "beyond the fade" bit-identical to the ungridded ground rather than
// approximately equal to it, which is what keeps the degenerate case a FREE,
// EXACT regression control instead of a tolerance.
// ---------------------------------------------------------------------------

// GroundGridParams lives in target.hpp, beside the RenderOptions that carries
// it -- target.hpp cannot include this header, and a parameter struct belongs
// with the options struct that holds one.

// An orthonormal basis for the plane, derived from its normal ALONE so it is
// deterministic and carries no Y-up assumption -- SR-17's own "no Y-up special
// case" clause applies to the grid exactly as it applies to the ground it sits
// on. The axis choice is by largest-component comparison, which is exact.
// NOT [[nodiscard]]: it returns void through out-params. MSVC accepts the
// attribute there silently; gcc-13 rejects it under -Werror=attributes, so
// this shipped as a WINDOWS-GREEN COMMIT THAT DOES NOT BUILD ON LINUX.
inline void ground_plane_basis(const glm::vec3& n, glm::vec3& u, glm::vec3& v) {
    const glm::vec3 a = (std::abs(n.x) <= std::abs(n.y) && std::abs(n.x) <= std::abs(n.z))
                            ? glm::vec3(1.0f, 0.0f, 0.0f)
                        : (std::abs(n.y) <= std::abs(n.z)) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                                           : glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 t = glm::cross(n, a);
    const float tl = std::sqrt(glm::dot(t, t));
    u = tl > 0.0f ? t / tl : glm::vec3(1.0f, 0.0f, 0.0f);
    v = glm::cross(n, u);
}

// Distance from `c` to the nearest multiple of `spacing`. floor-based rather
// than round-based: floor is exactly rounded and its behaviour at .5 does not
// depend on the current rounding mode.
[[nodiscard]] inline float distance_to_nearest_line(float c, float spacing) {
    const float k = std::floor(c / spacing + 0.5f);
    return std::abs(c - k * spacing);
}

// Returns the grid's coverage at a ground hit, in [0,1]. 0 means "no line
// here", and a 0 must produce a bit-identical pixel to the ungridded ground.
[[nodiscard]] inline float ground_grid_coverage(const glm::vec3& hit_world, const glm::vec3& plane_normal,
                                                 float view_distance, const GroundGridParams& p) {
    glm::vec3 u, v;
    ground_plane_basis(plane_normal, u, v);
    const float cu = glm::dot(hit_world, u);
    const float cv = glm::dot(hit_world, v);

    // Half-width grows linearly with distance so a line stays roughly one
    // pixel wide instead of collapsing into aliasing noise at range.
    const float hw = p.line_half_width * (1.0f + view_distance * p.width_growth);
    const float du = distance_to_nearest_line(cu, p.spacing);
    const float dv = distance_to_nearest_line(cv, p.spacing);
    const float d = du < dv ? du : dv;
    if (d >= hw) return 0.0f;  // EXACT zero: the common case is bit-identical ground

    // Linear ramp across the line's own half-width. No smoothstep: a cubic
    // buys nothing a manifest can see and costs two more roundings.
    const float edge = 1.0f - d / hw;

    // RATIONAL fade, never exponential. 1/(1+r^2) is 1 at the camera and
    // falls off smoothly; at view_distance == fade_distance it is exactly 1/2.
    const float r = view_distance / p.fade_distance;
    const float fade = 1.0f / (1.0f + r * r);
    return edge * fade;
}

// SR-17a's atmospheric term: how much the sky colour bleeds into a ground
// pixel at range. ⛔ NOT a screen-space filter -- it reads THIS ray and no
// neighbouring pixel, which is what keeps it out of RS15's post-processing
// exclusion and what lets both render paths compute it independently and
// agree. A neighbourhood filter could do neither, because the two paths do
// not share a framebuffer.
//
// ✅ SR-17a IS IN FORCE (03-world-and-render.md section 16, amended to ALL
// GEOMETRY AT RANGE by user ruling 2026-09-17). This comment previously read
// "IN FORCE ONLY WHEN SR-17a IS ... until then callers pass a zero strength",
// which was true when written and became false the moment section 16 landed --
// the R5 mode-1 shape, in the one place a reader checks before calling.
//
// ⚠ IT APPLIES TO EVERY SHADED SURFACE, not just the sky/ground boundary. The
// three call sites are the raster background pass (analytic ground + sky), the
// raster mesh fill (per PIXEL, never per vertex -- see raster_cpu.cpp's
// AtmosphereContext for why per-vertex breaks the seam), and raymarch's own
// hit. ONE function, three call sites, exactly like shade_vertex_color.
//
// ⚠ THE ENGINE DEFAULT IS STRENGTH 0 and at 0 this returns the surface colour
// EXACTLY, by the blend form above -- which is what lets every pre-SR-17a
// golden stand unchanged as a control rather than being regenerated.
[[nodiscard]] inline glm::vec3 horizon_blend(const glm::vec3& ground_color, const glm::vec3& sky_color,
                                              float view_distance, float onset_distance, float strength) {
    if (strength <= 0.0f || onset_distance <= 0.0f) return ground_color;
    const float r = view_distance / onset_distance;
    const float t = (r * r) / (1.0f + r * r);  // rational, 0 at the camera, ->1 at range
    const float k = t * strength;
    return ground_color + (sky_color - ground_color) * k;
}

// The linear-to-byte quantizer (S7a Task R6; relocated HERE at Task R8 fix
// round 1, review Minor 3): every BGRX8 byte either render path ever writes
// goes through this SAME clamp-then-round-to-nearest formula. This one is
// NOT a rendering decision either path could legitimately disagree on the
// way a shading model or a gradient stencil could (raster_cpu.cpp's own
// comment on shade_vertex_color) -- but Task R9 compares BYTES, so the
// quantizer that decides those bytes still has to be the SAME function, not
// two copies that happen to compute the same formula today and could drift
// tomorrow.
[[nodiscard]] inline uint8_t to_byte(float channel) {
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(clamped * 255.0f));
}

// tan(half the vertical FOV), for a pinhole camera ray (S7a Task R6;
// relocated HERE at Task R8 fix round 1, review Minor 3). SR-14: built from
// math::sin32/math::cos32 (core/fp32_math.hpp), never std::tan -- half a
// camera's vertical FOV is always small, always well inside sin32/cos32's
// accurate domain.
//
// DOUBLE, not float (review Minor 3's own finding): raster_cpu.cpp's
// original private copy of this function took/returned double -- sin32/
// cos32 themselves still compute in float, but the DIVISION that turns them
// into a tangent happens at double precision, and only THAT double result is
// narrowed to float, once, by the caller. Narrowing a double to float is not
// "bit-identical to the double" (it is the double's own correctly-rounded
// float value, a real rounding step) -- the property this buys is narrower
// but still load-bearing: every caller that shares THIS one function and
// narrows its result the same way (both render/raster_cpu.cpp's background
// pass and render/raymarch.cpp's camera ray, after this relocation) narrows
// the SAME double bit pattern, so their two `f` values agree with EACH OTHER
// bit-for-bit. A caller that instead divided in float directly (raymarch.cpp's
// own pre-fix-round private copy) could disagree with raster_cpu.cpp's `f` by
// a ulp for the exact same `fov_y_radians` -- a real, if tiny, geometry
// difference between the two paths' camera rays that R9 has no business
// measuring.
[[nodiscard]] inline double tan32(double half_fov_rad) {
    const float x = static_cast<float>(half_fov_rad);
    return static_cast<double>(spade::math::sin32(x)) / static_cast<double>(spade::math::cos32(x));
}

// One loaded or generated mesh. Positions/normals/indices stay empty for a
// slot this task allocated but did not fill (SR-9) -- R2/R5 fill the
// SDF-derived slots; a resolved visual reference (NamedMesh, below) arrives
// already filled by the caller.
//
// SUBMESH CONTRACT (controller ruling SR-11, settled at Task R2's review --
// binding on every producer AND consumer of this struct, not just R2's own
// tessellate_primitive()): an EMPTY submesh triple (all three arrays size 0,
// including a mesh whose positions/normals/indices are otherwise fully
// populated -- Task R2's tessellate_primitive() is exactly such a producer)
// means EXACTLY ONE IMPLICIT SUBMESH spanning the whole index buffer, with
// material index 0. Index 0 is always a valid default material: every
// RenderScene's `materials` carries a defaulted entry there
// (scene_from_world()'s own guarantee, scene.cpp). When the three arrays are
// NON-empty, they are PARALLEL (equal length) and their
// (submesh_first_index[i], submesh_index_count[i]) ranges PARTITION
// `indices` -- every index belongs to exactly one submesh, in submesh order.
// A consumer therefore never special-cases "no submeshes were declared" as a
// draw failure: it is the one-submesh-at-material-0 case, spelled with empty
// arrays instead of a redundant single-entry ones.
struct MeshData {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;              // per-vertex; flat meshes duplicate vertices
    std::vector<uint32_t>  indices;              // triangle list
    std::vector<uint32_t>  submesh_first_index;  // parallel; size = submesh count
    std::vector<uint32_t>  submesh_index_count;
    std::vector<uint32_t>  submesh_material;

    // The mesh FILE's own material palette, when the producer read one.
    //
    // ⚠ IT CHANGES WHAT submesh_material MEANS, so read both together:
    //   * EMPTY (tessellate_primitive, csg_mesh, every pre-2026-09-09
    //     producer) -- submesh_material indexes the SCENE's palette
    //     directly. The original contract, unchanged.
    //   * NON-EMPTY -- submesh_material is FILE-LOCAL and indexes THIS
    //     array. scene_from_world() appends these to RenderScene::materials
    //     and rewrites the indices by the offset; nothing downstream of that
    //     merge ever sees a file-local index.
    //
    // WHY A SECOND ARRAY RATHER THAN PRE-RESOLVED COLOURS: a MeshData is
    // loaded once and may be instanced into any scene, and the scene owns the
    // palette. Baking colours into the geometry would make the same file
    // un-shareable between two scenes with different palettes, which is the
    // coupling this indirection exists to avoid.
    //
    // The gap this closes: gltf.hpp declared reading a file's `materials[]`
    // out of scope and passed the raw index through, while raster_cpu applied
    // that index to the WORLD's palette. A ten-submesh aircraft therefore drew
    // in ten of the world's colours -- all in range, so not even a visible
    // fallback -- and a mesh's authored colours could not reach the frame at
    // all. gltf.hpp said resolving it was "a later task's job"; this is it.
    std::vector<Material> source_materials;
};

// A world visual reference already resolved to its geometry. `ref` names the
// same string one of WorldDesc::visual_refs carries; resolving ref -> file ->
// MeshData is entirely the CALLER's job (glTF loading, asset lookup -- no
// consumer- or file-format-specific vocabulary belongs in this codebase).
struct NamedMesh {
    std::string ref;
    MeshData mesh;
};

// Aabb, kNoMaterial and kNoMesh come from target.hpp (Task 0) -- do not redeclare.

// An infinite analytic ground candidate (S7a Task R6, ruling SR-17): one per
// STANDALONE plane primitive -- a node split_program() (render/csg_mesh.hpp)
// classifies as a union-primitive leaf, never one buried inside a
// subtract/intersect/smooth_union subtree (that plane is a cutting
// half-space, not a floor). `normal`/`offset` are already WORLD-SPACE
// (scene_from_world() bakes the node's own SdfTransform in once, at scene-
// build time, via transform_normal() above) so the background pass
// (raster_cpu.cpp) never re-derives them per pixel: solid is
// dot(p, normal) <= offset, the SAME convention world/sdf.hpp's plane node
// uses, so `normal` points away from the solid (the side a camera normally
// stands on) exactly like a plane's outward mesh normal does. `material` is
// always a valid index into RenderScene::materials (resolved the same way
// DrawItem::material_override resolves for that node -- node_materials, or
// submesh material 0 when node_materials is empty).
struct GroundPlane {
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    float offset = 0.0f;
    uint32_t material = 0;
};

struct DrawItem {
    uint32_t mesh_index = 0;   // index into RenderScene::meshes
    glm::mat4 local_to_world{1.0f};
    uint32_t material_override = kNoMaterial;

    // Metres per second, for DrawMode::velocity only (SL9c). 0 for a static,
    // which is exactly right: a static does not move. Every other draw mode
    // ignores it, so adding it moves no existing golden.
    float speed_mps = 0.0f;
};

// A tick-boundary copy of one body's pose (position/orientation only -- no
// scale; bodies are rigid). `mesh_index` is an index into RenderScene::meshes
// -- normally one of the `resolved_meshes` slots the caller resolved at
// world-load time (see the MESH INDEX SPACE note above), or kNoMesh for a
// body with nothing to draw.
struct BodyPose {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    uint32_t mesh_index = kNoMesh;

    // World-frame velocity at the same tick boundary as the pose. Carried as a
    // VECTOR rather than a scalar speed so a future direction-coded mode needs
    // no second migration of this struct; update_dynamics() reduces it to the
    // magnitude DrawItem stores.
    glm::vec3 velocity{0.0f};
};

struct RenderScene {
    std::vector<MeshData> meshes;
    std::vector<DrawItem> statics;    // built once at world load
    std::vector<DrawItem> dynamics;   // refreshed per frame from BodyPose
    std::vector<Material> materials;  // index 0 is always the default material
    Lighting               lighting;
    Aabb                   bounds;
    std::vector<glm::vec3> spawn_positions;   // overlay layer (PA-4)
    std::vector<glm::quat> spawn_orientations;
    const SdfProgram*      sdf = nullptr;     // non-owning; raymarch + agreement only
    float                  ground_y = 0.0f;
    bool                   has_ground = false;
    // Analytic background ground candidates (S7a Task R6, SR-17) -- empty for
    // a hand-built RenderScene (only scene_from_world() populates it) and
    // for a world with no standalone ground plane. Never consulted by
    // has_ground/ground_y's OLDER, unrelated purpose (the overlay grid's own
    // best-effort +Y-identity-plane heuristic, scene.cpp's ground_plane_y());
    // this is the raster background pass's own general, transform-aware list.
    std::vector<GroundPlane> ground_planes;
    // The sun's own shadow map (S7a Task R7), built ONCE from `statics` alone
    // at scene-build time and reused every frame -- raster_cpu.cpp's render()
    // never rebuilds this; it only re-rasterises `dynamics` into a per-frame
    // COPY of it (Step 3). std::nullopt for a hand-built RenderScene that
    // never calls build_static_shadow_map() (every existing fixture in this
    // corpus, until a test opts in) and, per scene_from_world()'s own
    // contract below, for a world with no static geometry at all -- either
    // way, RenderOptions::shadows has nothing to sample against and render()
    // treats every pixel as unshadowed, identically to shadows being off.
    std::optional<ShadowMap> static_shadow;
};

// Builds a RenderScene from a validated WorldDesc plus its already-resolved
// visual meshes.
//
// PRECONDITION: `world` has already passed validate_world_desc() -- true of
// anything WorldBuilder::build() or world_from_yaml() produced. This is a
// presentation-layer builder, not a second validator, and never fails on a
// well-formed WorldDesc; it returns Result<RenderScene> for the same reason
// validate_target() does (a stable seam for a future real failure mode),
// not because today's implementation has one.
//
// `sdf` in the result points at `world.sdf`: the returned RenderScene must
// not outlive `world`.
//
// `static_shadow` (S7a Task R7): populated via build_static_shadow_map()
// whenever `statics` ends up non-empty (props count too -- any real geometry
// the sun could cast a shadow FROM), left std::nullopt otherwise (an empty
// world has nothing to build a meaningful shadow map from).
[[nodiscard]] Result<RenderScene> scene_from_world(const WorldDesc& world,
                                                    std::span<const NamedMesh> resolved_meshes);

// Rebuilds `scene.dynamics` from `bodies`, one DrawItem per pose, in order.
// Never touches `scene.statics`, `scene.meshes`, or anything else in `scene`.
void update_dynamics(RenderScene& scene, std::span<const BodyPose> bodies);

}  // namespace spade::render
