#include "render/agreement.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "world/sdf.hpp"  // SdfOp, SdfPrim -- strip_to_ground_plane_only()'s own leaf test

namespace spade::render {
namespace {

// Packs 4 bytes in the SAME order render()/render_raymarch() write a BGRX8
// pixel (px[0]=B, px[1]=G, px[2]=R, px[3]=X -- raster_cpu.cpp's
// setPixelIfCloser()/raymarch.cpp's own final write, both cited in this
// file's own header comment) into one uint32_t, B in the low byte. Never
// exposed: a caller only ever needs the PACKED value (pack_sky_reference_bgrx
// below), never this function's own byte order convention.
[[nodiscard]] uint32_t pack_bgrx(uint8_t b, uint8_t g, uint8_t r, uint8_t x) {
    return static_cast<uint32_t>(b) | (static_cast<uint32_t>(g) << 8) | (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(x) << 24);
}

[[nodiscard]] uint32_t pixel_at(const RenderTarget& target, uint32_t x, uint32_t y) {
    const std::size_t idx = static_cast<std::size_t>(y) * target.stride + static_cast<std::size_t>(x) * 4;
    return pack_bgrx(target.pixels[idx], target.pixels[idx + 1], target.pixels[idx + 2], target.pixels[idx + 3]);
}

}  // namespace

AgreementResult compare_silhouettes(const RenderTarget& fast, const RenderTarget& raymarched,
                                     uint32_t sky_reference_rgb) {
    AgreementResult result;
    if (fast.width != raymarched.width || fast.height != raymarched.height) {
        // A caller error (comparing two differently-sized frames is
        // meaningless), not a geometry disagreement to measure. Returns the
        // default-constructed AgreementResult{} UNCHANGED -- see this file's
        // header comment for what that sentinel means (total disagreement,
        // zero measured coverage on EITHER side because neither side was
        // actually measured, not a claim that every pixel is covered) --
        // never a crash and never a silent partial scan over whichever
        // dimensions are smaller.
        return result;
    }

    const uint32_t width = fast.width, height = fast.height;
    uint64_t disagreeing_pixels = 0;
    // Fixed row-by-row, column-by-column scan order (determinism contract,
    // this file's own header comment) -- not that summation order could
    // change an integer count, but the SAME discipline every other pass in
    // this program follows unconditionally, not only where it would matter.
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const bool a_covered = pixel_at(fast, x, y) != sky_reference_rgb;
            const bool b_covered = pixel_at(raymarched, x, y) != sky_reference_rgb;
            if (a_covered) {
                ++result.covered_a;
            }
            if (b_covered) {
                ++result.covered_b;
            }
            if (a_covered != b_covered) {
                ++disagreeing_pixels;
            }
        }
    }

    const uint64_t total_pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    result.disagreement_fraction =
        total_pixels > 0 ? static_cast<double>(disagreeing_pixels) / static_cast<double>(total_pixels) : 0.0;
    return result;
}

uint32_t pack_sky_reference_bgrx(const Lighting& lighting) {
    return pack_bgrx(to_byte(lighting.sky_zenith.b), to_byte(lighting.sky_zenith.g), to_byte(lighting.sky_zenith.r),
                      0xFFu);
}

namespace {

// A normal `n` (unit) with dot(n, sun_dir) == nl EXACTLY, for `sun_dir` unit
// and `perp` unit + perpendicular to it: n = nl*sun_dir + sqrt(1-nl^2)*perp
// is unit (|n|^2 = nl^2 + (1-nl^2)*1 + 2*nl*sqrt(1-nl^2)*dot(sun_dir,perp) =
// nl^2 + 1 - nl^2 + 0 = 1, since dot(sun_dir,perp) = 0 by construction) and
// dot(n, sun_dir) = nl*1 + sqrt(1-nl^2)*0 = nl. No trig: cross/dot/normalize/
// sqrt only (SR-14 -- this is engine/ source, unconditionally scanned for
// libm transcendentals, not merely golden-feeding-test-source scanned).
[[nodiscard]] glm::vec3 normal_at_nl(const glm::vec3& sun_dir, const glm::vec3& perp, float nl) {
    const float clamped_nl = std::clamp(nl, 0.0f, 1.0f);
    const float s = std::sqrt(std::max(0.0f, 1.0f - clamped_nl * clamped_nl));
    return clamped_nl * sun_dir + s * perp;
}

}  // namespace

Result<void> assert_no_material_matches_sky(std::span<const spade::MaterialDesc> materials, const Lighting& lighting,
                                             uint32_t sky_reference_rgb, uint8_t tolerance) {
    const int sky_b = static_cast<int>(sky_reference_rgb & 0xFFu);
    const int sky_g = static_cast<int>((sky_reference_rgb >> 8) & 0xFFu);
    const int sky_r = static_cast<int>((sky_reference_rgb >> 16) & 0xFFu);

    // sun_dir/perp are shared across every material and every sampled N.L --
    // computed once. A degenerate (near-zero) sun_direction (never produced
    // by a validated WorldDesc, per validate_world_desc()'s own "a zero
    // lighting.sun_direction" error code, but this function accepts a
    // hand-built Lighting too) falls back to checking ONLY N.L=0 -- the
    // ambient-only endpoint is still a real, achievable colour regardless of
    // what direction (if any) the sun points.
    const float sun_len = glm::length(lighting.sun_direction);
    const bool have_sun_dir = sun_len > 1e-6f;
    const glm::vec3 sun_dir = have_sun_dir ? lighting.sun_direction / sun_len : glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 helper = std::fabs(sun_dir.y) < 0.99f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 perp_raw = glm::cross(sun_dir, helper);
    const glm::vec3 perp = glm::length(perp_raw) > 1e-6f ? glm::normalize(perp_raw) : glm::vec3(1.0f, 0.0f, 0.0f);

    // 129 samples (step 1/128) over N.L in [0, 1] (fix round 2, review
    // finding: the ORIGINAL comment here overclaimed "no byte-level match is
    // ever skipped" -- for a channel whose full swing from N.L=0 to N.L=1
    // spans close to the full byte range, the per-sample step is close to
    // 255/128 =~ 2 BYTES, not fine enough on its own to guarantee catching
    // an exact 1-in-255 collision between two adjacent samples). The check
    // is sound anyway, for a different, correct reason: `tolerance`'s
    // default (8) is comfortably larger than half that worst-case step
    // (~1), so a true collision can miss the nearest sample by up to ~1
    // byte and still land inside the tolerance window that sample checks.
    // Cheap regardless -- this runs once per material at scene-build time,
    // never per pixel or per frame.
    constexpr int kNlSamples = 129;

    for (std::size_t i = 0; i < materials.size(); ++i) {
        const spade::MaterialDesc& md = materials[i];
        const Material shading_material{.base_color = md.base_color, .shading = static_cast<uint32_t>(md.shading)};

        const int nl_steps = have_sun_dir ? kNlSamples : 1;
        for (int s = 0; s < nl_steps; ++s) {
            const float nl = have_sun_dir ? static_cast<float>(s) / static_cast<float>(kNlSamples - 1) : 0.0f;
            const glm::vec3 normal = normal_at_nl(sun_dir, perp, nl);
            const ShadedColor shaded = shade_vertex_color(shading_material, lighting, normal);
            const int b = static_cast<int>(to_byte(shaded.combined.b));
            const int g = static_cast<int>(to_byte(shaded.combined.g));
            const int r = static_cast<int>(to_byte(shaded.combined.r));
            const int db = std::abs(b - sky_b);
            const int dg = std::abs(g - sky_g);
            const int dr = std::abs(r - sky_r);
            if (db <= tolerance && dg <= tolerance && dr <= tolerance) {
                return std::unexpected(Error{
                    Code::invalid_argument,
                    "material '" + md.name + "' (index " + std::to_string(i) + ") shades to BGR(" +
                        std::to_string(b) + "," + std::to_string(g) + "," + std::to_string(r) + ") at N.L=" +
                        std::to_string(nl) + ", within tolerance " + std::to_string(static_cast<int>(tolerance)) +
                        " of the sky reference colour BGR(" + std::to_string(sky_b) + "," + std::to_string(sky_g) +
                        "," + std::to_string(sky_r) +
                        ") -- compare_silhouettes()'s colour-based silhouette classifier cannot tell this "
                        "material's coverage apart from a miss at that surface orientation (S7a Task R9, Step 1c "
                        "guard, fix round 1: checks the SHADED range, not merely the authored base_color)"});
            }
        }
    }
    return {};
}

Result<spade::WorldDesc> strip_to_ground_plane_only(const spade::WorldDesc& world) {
    // UNIQUENESS, CHECKED (fix round 2, review finding): the original version
    // took the FIRST plane leaf and never asked whether it was the ONLY one.
    // Benign for every one of today's ten shipped worlds (the controller
    // amendments' own grep already confirms each has exactly one `prim:
    // plane` node), but this is production code SR-2 hands to Task C4 for
    // DRESSED worlds -- a second plane leaf there would silently make the
    // probe measure a different mutant than "everything but the ground" (an
    // arbitrary pick among two candidates, not a documented choice), which
    // is exactly the kind of quiet wrongness this whole task exists to
    // refuse. A world with more than one plane leaf is now a hard error
    // instead.
    std::size_t plane_index = static_cast<std::size_t>(-1);
    std::size_t plane_count = 0;
    for (std::size_t i = 0; i < world.sdf.nodes.size(); ++i) {
        const spade::SdfNode& node = world.sdf.nodes[i];
        if (node.op == static_cast<uint32_t>(spade::SdfOp::none) &&
            node.kind == static_cast<uint32_t>(spade::SdfPrim::plane)) {
            if (plane_count == 0) {
                plane_index = i;
            }
            ++plane_count;
        }
    }
    if (plane_count == 0) {
        return std::unexpected(
            Error{Code::not_found, "strip_to_ground_plane_only: world has no standalone ground-plane leaf node"});
    }
    if (plane_count > 1) {
        return std::unexpected(Error{
            Code::invalid_argument,
            "strip_to_ground_plane_only: world has " + std::to_string(plane_count) +
                " plane leaf nodes -- ambiguous which one is 'the ground'; this function refuses to pick arbitrarily"});
    }
    // NOT CHECKED HERE, BY SCOPE (documented, not silently assumed): whether
    // this node is actually the one raster_cpu.cpp's analytic background
    // pass treats as a ground (render/scene.hpp's SR-17 GroundPlane list,
    // built by render/csg_mesh.hpp's split_program(), which additionally
    // requires the leaf sit under nothing but `union` -- a plane nested
    // inside a subtract/intersect/smooth_union subtree as a CUTTING
    // half-space, not a floor, would still match the kind/op test above.
    // Pulling in split_program()'s own classification here, for a
    // validation-only path, was judged out of proportion for a fix round
    // scoped as "small" -- a caller with a world shaped that unusually
    // should independently confirm `RenderScene::ground_planes` is
    // non-empty for it before trusting this probe.
    spade::WorldDesc stripped = world;
    stripped.sdf.nodes = {world.sdf.nodes[plane_index]};
    if (!world.sdf.node_materials.empty()) {
        stripped.sdf.node_materials = {world.sdf.node_materials[plane_index]};
    }
    // `transforms` deliberately left FULLY INTACT -- the surviving node's own
    // `transform` index still resolves into it unchanged (this file's own
    // header comment).
    return stripped;
}

}  // namespace spade::render
