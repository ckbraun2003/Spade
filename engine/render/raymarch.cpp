#include "render/raymarch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "core/fp32_math.hpp"
#include "world/sdf.hpp"

namespace spade::render {

namespace {

using math::cos32;
using math::sin32;

// Mirrors raster_cpu.cpp's own private tan32 helper -- same formula, same
// domain reasoning (half a camera's vertical FOV is always small, always
// well inside sin32/cos32's accurate domain, core/fp32_math.hpp) --
// duplicated here because that helper has internal linkage in a different
// translation unit and this is its only other call site (raymarch.hpp's own
// header comment on "no libm transcendentals").
[[nodiscard]] float tan32(float half_fov_rad) { return sin32(half_fov_rad) / cos32(half_fov_rad); }

// Mirrors raster_cpu.cpp's own private to_byte helper -- same clamp-then-
// round-to-nearest formula. A generic byte-quantization utility, not a
// rendering decision either path could disagree on the way a shading model
// or a gradient stencil could, so duplicating it carries none of the
// "second implementation that can silently drift" risk this task's other
// shared functions (shade_vertex_color, sky_gradient_color) were written to
// avoid.
[[nodiscard]] uint8_t to_byte(float channel) {
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(clamped * 255.0f));
}

}  // namespace

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
    // on.
    const float half_fov = camera.fov_y_radians * 0.5f;
    const float f = 1.0f / tan32(half_fov);
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const glm::vec3 origin = camera.position;
    const glm::mat3 cam_to_world = glm::mat3_cast(camera.orientation);

    // NULL VS EMPTY (raymarch.hpp's own header comment): scene.sdf may be
    // null (never pointed at a program) or non-null-but-empty (a validly
    // constructed program with zero nodes) -- both must yield sky only.
    // Checked ONCE per frame, not per pixel: with nothing to march, marching
    // would only ever miss anyway (world/sdf.hpp's own eval()/gradient()
    // contract on an empty program returns kSdfEmptyDistance/zero, never a
    // fault), so this also skips kRaymarchMaxSteps wasted evaluations per
    // pixel for the empty-scene case (a real one -- an unarmed viewport with
    // no world loaded). `scene.materials.empty()` is folded into the SAME
    // gate for the identical reason draw_sky_and_ground_background's own
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
                // background_ray_camera_space uses (x_ndc in [-1,1] left to
                // right, y_ndc in [-1,1] bottom to top, camera looks down
                // -Z), but normalized here: sphere tracing steps `t` by a
                // real WORLD-SPACE distance every iteration, which is only
                // correct when `dir_world` has unit length (the raster
                // path's own analogous ray is deliberately left
                // un-normalized because it only ever compares one t against
                // another or tests a sign -- a scale-invariant use this
                // function's own t-stepping is not).
                const float x_ndc = 2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(width) - 1.0f;
                const glm::vec3 dir_cam = glm::normalize(glm::vec3(x_ndc * aspect / f, y_ndc / f, -1.0f));
                const glm::vec3 dir_world = cam_to_world * dir_cam;

                // Sphere tracing (task brief Step 2): step by the exact
                // returned distance every iteration (a lower bound on how
                // far the nearest surface can be, world/sdf.hpp), so no
                // step can ever cross through unseen geometry. Converges
                // (`d <= kRaymarchSurfaceEpsilon`) once the ray is close
                // enough to call it a hit; a program starting the ray
                // already inside a solid (d <= 0) converges immediately at
                // t=0 by the SAME test, rather than needing a separate
                // sign-based branch. `t` strictly increases every iteration
                // that does not converge (d is then > kRaymarchSurfaceEpsilon
                // > 0), so this loop always terminates within
                // kRaymarchMaxSteps iterations -- no distance-based escape
                // check is needed (raymarch.hpp's own header comment).
                //
                // Uses eval() here, NOT sample() or gradient() -- Task R8's
                // own performance note: sample()/gradient() additionally
                // compute a per-node gradient (a central difference costs
                // SIX extra primitive_distance() calls per such node,
                // world/sdf.hpp), which is pure waste on every step that
                // does not converge. The gradient is fetched exactly ONCE
                // below, only at the final hit point.
                float t = 0.0f;
                bool hit = false;
                for (uint32_t step = 0; step < kRaymarchMaxSteps; ++step) {
                    const glm::vec3 p = origin + dir_world * t;
                    const float d = spade::eval(*scene.sdf, p);
                    if (d <= kRaymarchSurfaceEpsilon) {
                        hit = true;
                        break;
                    }
                    t += d;
                }

                if (hit) {
                    const glm::vec3 p = origin + dir_world * t;
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
                    // for real geometry (every primitive's analytic or
                    // central-difference gradient is non-zero on its own
                    // surface); the fallback below is a defensive floor,
                    // and is safe precisely because Task R9 compares
                    // SILHOUETTES ONLY (raymarch.hpp's own header comment)
                    // -- a wrong shading normal never changes hit/miss
                    // coverage, only a shaded colour nothing downstream of
                    // R9 depends on.
                    const glm::vec3 normal =
                        normal_len > 0.0f ? raw_normal / normal_len : glm::vec3(0.0f, 1.0f, 0.0f);
                    // shade_vertex_color() (render/scene.hpp) -- the SAME
                    // Lambert/unlit/emissive model R6 built, reused
                    // verbatim (raymarch.hpp's own header comment). Shaded
                    // with scene.materials[0] (see that same header comment
                    // for why a per-primitive material is not available
                    // here) -- valid because `has_sdf` already established
                    // scene.materials is non-empty.
                    const ShadedColor shaded = shade_vertex_color(scene.materials[0], scene.lighting, normal);
                    color = shaded.combined;
                }
            }

            // MN-14: every pixel's 4th (X) byte is 0xFF, written
            // unconditionally, exactly like raster_cpu.cpp's own background
            // and draw passes.
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
