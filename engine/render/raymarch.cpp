#include "render/raymarch.hpp"

#include <cstdint>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "world/sdf.hpp"

namespace spade::render {

Result<void> render_raymarch(const RenderScene& scene, const Camera& camera, RenderTarget& target) {
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
        // Sky gradient (render/scene.hpp's sky_gradient_color(), shared with
        // raster_cpu.cpp's own background pass -- see raymarch.hpp's header
        // comment) -- computed once per ROW, exactly like the raster path's
        // own per-row hoist, since it depends only on screen row.
        const float row_fraction = height > 1 ? static_cast<float>(y) / static_cast<float>(height - 1) : 0.0f;
        const glm::vec3 sky = sky_gradient_color(scene.lighting, row_fraction);
        const float y_ndc = 1.0f - 2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(height);

        for (uint32_t x = 0; x < width; ++x) {
            glm::vec3 color = sky;

            if (has_sdf) {
                // Pinhole camera ray through pixel center (x+0.5, y+0.5) --
                // the SAME NDC convention raster_cpu.cpp's own
                // background_ray_camera_space uses, but normalized here:
                // sphere tracing steps `t` by a real WORLD-SPACE distance
                // every iteration, which is only correct when `dir_world`
                // has unit length.
                const float x_ndc = 2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(width) - 1.0f;
                const glm::vec3 dir_cam = glm::normalize(glm::vec3(x_ndc * aspect / f, y_ndc / f, -1.0f));
                const glm::vec3 dir_world = cam_to_world * dir_cam;

                // FRUSTUM PARITY (fix round 1, review IMPORTANT -- see
                // raymarch.hpp's own header comment for the full "why this
                // mirrors raster's near/far clip and SR-13's camera-inside-
                // a-solid cull" reasoning): march starts at t = near_plane,
                // not t = 0. The FIRST sample decides whether this ray is
                // even eligible to hit at all -- if the near-plane point is
                // already inside a solid (d <= 0), the whole ray is a miss,
                // full stop, mirroring a camera fully enclosed by a convex
                // mesh seeing nothing under SR-13's back-face cull.
                float t = near_plane;
                glm::vec3 p = origin + dir_world * t;
                float d = spade::eval(*scene.sdf, p);
                bool hit = false;

                if (d > 0.0f) {
                    // Sphere tracing (task brief Step 2): step by the exact
                    // returned distance every iteration (a lower bound on
                    // how far the nearest surface can be, world/sdf.hpp), so
                    // no step can ever cross through unseen geometry.
                    // Converges (`d <= kRaymarchSurfaceEpsilon`) once the ray
                    // is close enough to call it a hit. `t` strictly
                    // increases every iteration that reaches the bottom of
                    // this loop (the preceding `d` was > epsilon > 0 to get
                    // here), so this loop always terminates.
                    //
                    // FAR-PLANE ESCAPE (fix round 1, review IMPORTANT): the
                    // only previous termination was hit-or-budget-exhausted,
                    // which measured 81.5% of all SDF evaluations spent on
                    // rays that hit nothing -- every miss burning the FULL
                    // kRaymarchMaxSteps budget, because a ray nearly
                    // parallel to an infinite ground plane has its distance
                    // stay roughly CONSTANT (neither converging nor
                    // escaping) rather than growing the way a ray receding
                    // from a bounded primitive does. Stopping at
                    // `t > far_plane` bounds this ray's cost by the SAME
                    // frustum raster_cpu.cpp already clips to, rather than a
                    // second, invented distance constant.
                    //
                    // Uses eval() here, NOT sample() or gradient() -- Task
                    // R8's own performance note: sample()/gradient()
                    // additionally compute a per-node gradient (a central
                    // difference costs SIX extra primitive_distance() calls
                    // per such node, world/sdf.hpp), which is pure waste on
                    // every step that does not converge. The gradient is
                    // fetched exactly ONCE below, only at the final hit
                    // point.
                    for (uint32_t step = 0; step < kRaymarchMaxSteps; ++step) {
                        if (d <= kRaymarchSurfaceEpsilon) {
                            hit = true;
                            break;
                        }
                        t += d;
                        if (t > far_plane) {
                            break;  // escaped the visible frustum -- miss, sky.
                        }
                        p = origin + dir_world * t;
                        d = spade::eval(*scene.sdf, p);
                    }
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
