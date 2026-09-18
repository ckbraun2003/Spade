#include "render/raymarch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "world/sdf.hpp"

namespace spade::render {

Result<void> render_raymarch(const RenderScene& scene, const Camera& camera, RenderTarget& target,
                              float horizon_strength, float horizon_onset) {
    const uint32_t width = target.width;
    const uint32_t height = target.height;

    // Loop invariants -- computed ONCE per frame, never per pixel (the same
    // discipline raster_cpu.cpp's own background pass applies to its
    // f/aspect precompute, measured there at 2.5-4.4x for the per-pixel
    // version, Task R6 review IMPORTANT 3). `cam_to_world` mirrors
    // transform_normal()'s own "mat3 * vec3 rotates a direction" convention
    // (render/scene.hpp) rather than raster_cpu.cpp's ported double-Vec3
    // quaternion machinery, which this file has no other reason to depend
    // on. `f` is computed via scene.hpp's shared tan32() (fix round 1,
    // review Minor 3) in DOUBLE, narrowed to float only here -- the same
    // point raster_cpu.cpp's own projectCameraSpace narrows it, so the two
    // paths' `f` agree bit-for-bit for the same fov_y_radians.
    const double f_double = 1.0 / tan32(static_cast<double>(camera.fov_y_radians) * 0.5);
    const float f = static_cast<float>(f_double);
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const glm::vec3 origin = camera.position;
    const glm::mat3 cam_to_world = glm::mat3_cast(camera.orientation);
    const float near_plane = camera.near_plane;
    const float far_plane = camera.far_plane;

    // NULL VS EMPTY (raymarch.hpp's own header comment): scene.sdf may be
    // null (never pointed at a program) or non-null-but-empty (a validly
    // constructed program with zero nodes) -- both must yield sky only.
    // Checked ONCE per frame, not per pixel: with nothing to march, marching
    // would only ever miss anyway (world/sdf.hpp's own eval()/gradient()
    // contract on an empty program returns kSdfEmptyDistance/zero, never a
    // fault), so this also skips wasted per-pixel work for the empty-scene
    // case (a real one -- an unarmed viewport with no world loaded).
    // `scene.materials.empty()` is folded into the SAME gate for the
    // identical reason draw_sky_and_ground_background's own
    // `ground_possible` gate checks it alongside geometry (raster_cpu.cpp):
    // a hit with nothing safe to shade with is exactly as undrawable as no
    // hit at all.
    const bool has_sdf = scene.sdf != nullptr && !scene.sdf->empty() && !scene.materials.empty();

    for (uint32_t y = 0; y < height; ++y) {
        const float y_ndc = 1.0f - 2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(height);

        for (uint32_t x = 0; x < width; ++x) {
            // Pinhole camera ray through pixel center (x+0.5, y+0.5) -- the
            // SAME NDC convention raster_cpu.cpp's own background pass
            // (draw_sky_and_ground_background) uses, but normalized here: sphere
            // tracing steps `t` by a real WORLD-SPACE distance every
            // iteration, which is only correct when `dir_world` has unit
            // length. Reconstructed for EVERY pixel now, not merely when
            // has_sdf -- the elevation-based sky gradient below (ruling
            // SR-23, Task VQ-A) needs this same per-pixel ray direction even
            // on a sky-only (no-SDF) frame, unlike the old per-ROW gradient
            // it replaces.
            const float x_ndc = 2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(width) - 1.0f;
            const glm::vec3 dir_cam = glm::normalize(glm::vec3(x_ndc * aspect / f, y_ndc / f, -1.0f));
            const glm::vec3 dir_world = cam_to_world * dir_cam;

            // Sky gradient (render/scene.hpp's sky_gradient_color(), shared
            // with raster_cpu.cpp's own background pass -- see raymarch.hpp's
            // header comment). Elevation-based since Task VQ-A/SR-23: keyed
            // on this SAME per-pixel world-space ray direction (already unit
            // length here, but the function does not require that), so the
            // two paths' sky agrees bit-for-bit rather than by two
            // independently-equal expressions.
            const glm::vec3 sky = sky_gradient_color(scene.lighting, dir_world);
            glm::vec3 color = sky;

            if (has_sdf) {
                // FRUSTUM PARITY, DEPTH NOT RADIAL DISTANCE (fix round 2,
                // review IMPORTANT -- see raymarch.hpp's own header comment
                // for the full derivation): raster clips on CAMERA-SPACE
                // DEPTH (`-pc.z`), not radial distance travelled along the
                // ray -- the two differ by cos(theta) for an off-axis pixel.
                // `dir_cam` is already unit length, so its own -Z component
                // IS cos(theta); dividing near_plane/far_plane by it converts
                // them from depth bounds into the equivalent RADIAL bounds
                // for THIS pixel's ray, computed once and reused every step
                // rather than converting back and forth per iteration.
                const float cos_theta = -dir_cam.z;  // > 0 always: the un-normalized z is exactly -1.0f before normalize()
                const float t_near = near_plane / cos_theta;
                const float t_far = far_plane / cos_theta;

                float t = t_near;
                glm::vec3 p = origin + dir_world * t;
                float d = spade::eval(*scene.sdf, p);
                bool hit = false;
                // Sticks at false while the ray is still inside (or, for the
                // ONGOING in-loop re-check below, within epsilon of) the
                // solid it started behind -- see the "march past" note in
                // raymarch.hpp's own header comment (fix round 2, review
                // MINOR) for why this may NOT be treated as an immediate
                // whole-ray miss the way fix round 1 did.
                //
                // INITIALIZED FROM `d > 0.0f`, NOT `d > epsilon` (fix round
                // 3, review MINOR -- a real behaviour narrowing round 2 had
                // introduced): the FIRST sample is a special case. A ray
                // whose very first sample already satisfies
                // `0 < d <= epsilon` is a genuine, legitimate hit converged
                // from OUTSIDE -- round 1's own `d > 0.0f` gate let it
                // register immediately, and round 2's `d > epsilon` gate
                // silently swept it into the march-past branch instead,
                // narrowing what counts as a hit for no reason tied to the
                // march-past fix. The ONGOING in-loop re-check just below
                // deliberately keeps the STRICTER `> epsilon` bound instead
                // of reusing this same `> 0.0f` test: promoting on the first
                // barely-positive sample seen WHILE MARCHING PAST a solid
                // would fire almost every time on axis-aligned geometry --
                // the march-past step lands very close to the solid's own
                // flat exit face by construction, so a naive `> 0.0f` there
                // reintroduced a MUCH worse regression than the one being
                // fixed (measured: the whole frame read as a hit on the
                // slab's own exit face in
                // CameraInsideASolidWithAnotherObjectBehindItSeesTheSecond
                // Object, and CameraFullyInsideAConvexSolidSeesNothingLike
                // RastersBackFaceCull regressed too). The two thresholds are
                // deliberately different for deliberately different
                // reasons, not an oversight.
                bool cleared_start_solid = d > 0.0f;

                // Sphere tracing (task brief Step 2): step by the exact
                // returned distance every iteration once outside (a lower
                // bound on how far the nearest surface can be,
                // world/sdf.hpp), so no step can ever cross through unseen
                // geometry. `t` strictly increases every iteration that
                // reaches the bottom of this loop -- either branch below
                // advances by a strictly positive amount -- so this loop
                // always terminates.
                //
                // FAR-PLANE ESCAPE (fix round 1, review IMPORTANT): the only
                // previous termination was hit-or-budget-exhausted, which
                // measured 81.5% of all SDF evaluations spent on rays that
                // hit nothing -- every miss burning the FULL kRaymarchMaxSteps
                // budget, because a ray nearly parallel to an infinite ground
                // plane has its distance stay roughly CONSTANT (neither
                // converging nor escaping) rather than growing the way a ray
                // receding from a bounded primitive does. Stopping at
                // `t > t_far` bounds this ray's cost by the SAME frustum
                // raster_cpu.cpp already clips to (now correctly, per the
                // depth-not-radial fix above), rather than a second, invented
                // distance constant.
                //
                // Uses eval() here, NOT sample() or gradient() -- Task R8's
                // own performance note: sample()/gradient() additionally
                // compute a per-node gradient (a central difference costs SIX
                // extra primitive_distance() calls per such node,
                // world/sdf.hpp), which is pure waste on every step that does
                // not converge. The gradient is fetched exactly ONCE below,
                // only at the final hit point.
                for (uint32_t step = 0; step < kRaymarchMaxSteps; ++step) {
                    if (!cleared_start_solid && d > kRaymarchSurfaceEpsilon) {
                        cleared_start_solid = true;
                    }
                    if (cleared_start_solid) {
                        if (d <= kRaymarchSurfaceEpsilon) {
                            hit = true;
                            break;
                        }
                        t += d;
                    } else {
                        // MARCH PAST THE STARTING SOLID (fix round 2, review
                        // MINOR): step toward the nearest surface without
                        // registering a hit, however small |d| gets -- a
                        // sphere tracer detects a genuine CROSSING, and
                        // re-crossing back OUT of the solid the ray started
                        // behind is not a new surface to report (raster's own
                        // SR-13 cull hides only that ENCLOSING solid, never
                        // whatever sits further along the ray). Floored at
                        // kRaymarchSurfaceEpsilon so a degenerate exact-zero
                        // sample (an axis-aligned face hit dead-on) cannot
                        // stall progress.
                        t += std::max(std::fabs(d), kRaymarchSurfaceEpsilon);
                    }
                    if (t > t_far) {
                        break;  // escaped the visible frustum -- miss, sky.
                    }
                    p = origin + dir_world * t;
                    d = spade::eval(*scene.sdf, p);
                }

                if (hit) {
                    // `p` already holds the converged hit point (fix round
                    // 1, review Minor 2 -- no redundant recomputation: the
                    // loop's own last-evaluated `p` IS the hit point, since
                    // the break above happens before `p` is ever advanced
                    // past it).
                    //
                    // world/sdf.hpp's gradient() -- the physics side's own
                    // pinned central-difference stencil (kSdfGradientStep)
                    // for every primitive kind without an analytic
                    // gradient, propagated exactly through every CSG op for
                    // the rest -- cited by calling it directly, never
                    // re-derived (raymarch.hpp's own header comment; task
                    // brief Step 2's binding instruction).
                    const glm::vec3 raw_normal = spade::gradient(*scene.sdf, p);
                    const float normal_len = glm::length(raw_normal);
                    // Gradient magnitude is only guaranteed 1 where the
                    // field is an exact metric and differentiable
                    // (world/sdf.hpp) -- re-normalized here rather than
                    // assumed unit, the same posture transform_normal()
                    // takes for a tessellated mesh's local normal
                    // (render/scene.hpp). A zero-length gradient is a
                    // pathological CSG cancellation, not an expected case
                    // for real geometry; the fallback below is a defensive
                    // floor, and affects only shading, never the hit/miss
                    // coverage Task R9 compares.
                    const glm::vec3 normal =
                        normal_len > 0.0f ? raw_normal / normal_len : glm::vec3(0.0f, 1.0f, 0.0f);

                    // PER-LEAF MATERIAL RESOLUTION (fix round 1, review
                    // IMPORTANT -- SUPERSEDES this file's original
                    // materials[0]-for-everything ruling; see raymarch.hpp's
                    // own header comment for the full "why it was a
                    // correctness bug, not a cosmetic simplification"
                    // reasoning). nearest_leaf_node() reports the postfix
                    // node index of the leaf primitive that produced this
                    // distance, using the SAME branch selection
                    // combine_distance()/combine_gradient() already apply
                    // internally -- never a second, independent
                    // nearest-primitive search. Resolution mirrors
                    // scene.cpp's own node_material_override() convention
                    // exactly: an empty node_materials array, an
                    // out-of-range leaf (kNoSdfLeaf, only reachable for an
                    // empty program -- excluded by `has_sdf` above), or an
                    // out-of-range resolved index all fall back to material
                    // 0, the SAME fallback draw_mesh_item's own submesh
                    // resolution uses.
                    const uint32_t leaf = spade::nearest_leaf_node(*scene.sdf, p);
                    const uint32_t node_material = (leaf != spade::kNoSdfLeaf && !scene.sdf->node_materials.empty())
                                                        ? scene.sdf->node_materials[leaf]
                                                        : kNoMaterial;
                    const uint32_t material_index = node_material < scene.materials.size() ? node_material : 0u;

                    // shade_vertex_color() (render/scene.hpp) -- the SAME
                    // Lambert/unlit/emissive model R6 built, reused
                    // verbatim (raymarch.hpp's own header comment).
                    const ShadedColor shaded =
                        shade_vertex_color(scene.materials[material_index], scene.lighting, normal);
                    color = shaded.combined;

                    // SR-17a (03-world-and-render.md section 16, ALL GEOMETRY
                    // AT RANGE): scene.hpp's horizon_blend(), the SAME function
                    // raster_cpu.cpp calls from its background pass and its
                    // mesh fill. Three values, all belonging to THIS ray: the
                    // colour just shaded, `t` -- which IS the eye-to-surface
                    // distance because `dir_world` is unit length, this file's
                    // own sphere-tracing precondition -- and the sky colour
                    // already computed above for this same direction.
                    //
                    // ⚠ A MISS NEEDS NO BLEND: `color` is still exactly `sky`
                    // there, and blending sky toward sky is the identity. The
                    // hard horizon therefore disappears because the GROUND side
                    // moves toward the sky, not because anything special
                    // happens at the boundary -- which is the whole point of
                    // the amendment: the boundary case is a CONSEQUENCE of the
                    // rule, never the rule.
                    color = horizon_blend(color, sky, t, horizon_onset, horizon_strength);
                }
            }

            // MN-14: every pixel's 4th (X) byte is 0xFF, written
            // unconditionally, exactly like raster_cpu.cpp's own background
            // and draw passes. to_byte() is render/scene.hpp's shared
            // quantizer (fix round 1, review Minor 3) -- the SAME function
            // raster_cpu.cpp calls, since R9 compares the exact bytes it
            // produces.
            uint8_t* px = &target.pixels[(static_cast<size_t>(y) * width + x) * 4];
            px[0] = to_byte(color.b);
            px[1] = to_byte(color.g);
            px[2] = to_byte(color.r);
            px[3] = 0xFFu;
        }
    }

    return {};
}

}  // namespace spade::render
