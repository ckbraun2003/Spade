#include "render/shadow.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <glm/geometric.hpp>
#include <glm/vec4.hpp>

#include "render/scene.hpp"  // complete RenderScene/MeshData/DrawItem/Aabb definitions

namespace spade::render {

namespace {

// ---------------------------------------------------------------------------
// Light basis (SR-14: cross/dot/normalize only, never a trig call).
//
// `forward` is `sun_direction` itself: world/builder.hpp's own LightingDesc
// doc comment says sun_direction points FROM the scene TOWARD the sun (the
// direction light arrives FROM), and scene_from_world() copies it into
// RenderScene::Lighting via a straight normalize -- no sign flip anywhere on
// that path. The shadow "camera" therefore sits out past the sun and looks
// back toward the scene along -forward, exactly how a real directional
// light's shadow camera would; `light_z = dot(p, forward)` (light_view_proj's
// own row 2, built below) increases toward the sun, so "closer to the sun"
// is "larger light_z" -- the same "larger wins" sense raster_cpu.cpp's own
// invDepth convention uses for camera-space depth, chosen here for the
// identical reason: one comparison direction, everywhere in this file.
// ---------------------------------------------------------------------------
struct LightBasis {
    glm::vec3 right;
    glm::vec3 up;
    glm::vec3 forward;
};

// A fixed reference "up" hint, swapped to world +Z when `forward` is too
// close to vertical for cross(worldUp, forward) to stay well-conditioned.
// Not a hypothetical corner case: LightingDesc's own default sun_direction
// is exactly (0,1,0) (world/builder.hpp), so a scene that never authors its
// own lighting hits this branch on every call.
constexpr float kNearVerticalDot = 0.999f;

// Guards glm::normalize() below against a zero (or exactly self-cancelling)
// `sun_direction` (M2, R7 fix round 1): normalize() of the zero vector is
// NaN, and that NaN survives sample_shadow()'s own `p.x < -1.0f || p.x >
// 1.0f` footprint test -- EVERY comparison against NaN is false, so a NaN
// `p.x` falls through as "inside the footprint" straight into an undefined
// float->int cast. scene_from_world()'s own caller path can never reach
// this (WorldBuilder/validate_world_desc() already rejects a zero
// sun_direction, world/builder.hpp's own doc comment), but every fixture in
// THIS program's own test corpus hand-builds a RenderScene directly and
// bypasses that validation entirely -- exactly how this would actually be
// reached. Falls back to LightingDesc's own default (straight up) rather
// than propagating a NaN through the rest of this file.
constexpr float kMinSunDirectionLengthSq = 1e-12f;

[[nodiscard]] LightBasis build_light_basis(const glm::vec3& sun_direction) {
    const glm::vec3 safe_sun_direction = glm::dot(sun_direction, sun_direction) > kMinSunDirectionLengthSq
                                              ? sun_direction
                                              : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 forward = glm::normalize(safe_sun_direction);
    const glm::vec3 up_hint = std::fabs(glm::dot(forward, glm::vec3(0.0f, 1.0f, 0.0f))) > kNearVerticalDot
                                   ? glm::vec3(0.0f, 0.0f, 1.0f)
                                   : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 right = glm::normalize(glm::cross(up_hint, forward));
    // cross() of two orthonormal unit vectors is already unit length (up to
    // float rounding) -- no second normalize() needed, matching
    // transform_normal()'s own "only normalize where scale could have crept
    // in" discipline (scene.hpp).
    const glm::vec3 up = glm::cross(forward, right);
    return LightBasis{right, up, forward};
}

// Guards the x/y ortho scale below against a divide-by-zero for a
// degenerate (zero-thickness along an axis) `scene.bounds` -- a hand-built
// RenderScene fixture can set bounds to anything; scene_from_world()'s own
// world_bounds_of() already clamps every axis to kMinBoundsSize=2.0
// (scene.cpp), so this is defensive for the former, never load-bearing for
// the latter.
constexpr float kMinLightExtent = 1e-3f;

// Builds the single AFFINE world-to-shadow-space matrix (ShadowMap::
// light_view_proj's own doc comment, shadow.hpp): rows 0/1 project onto
// `basis.right`/`basis.up` and rescale that AABB-corner-derived extent to
// [-1, 1] (this map's own texel space); row 2 is the raw light-space
// distance toward the sun, offset only for numerical hygiene (translation
// does not change any COMPARISON this file or sample_shadow() ever makes,
// only how large the numbers are).
[[nodiscard]] glm::mat4 build_light_view_proj(const LightBasis& basis, const Aabb& bounds) {
    const glm::vec3 corners[8] = {
        {bounds.min.x, bounds.min.y, bounds.min.z}, {bounds.max.x, bounds.min.y, bounds.min.z},
        {bounds.min.x, bounds.max.y, bounds.min.z}, {bounds.max.x, bounds.max.y, bounds.min.z},
        {bounds.min.x, bounds.min.y, bounds.max.z}, {bounds.max.x, bounds.min.y, bounds.max.z},
        {bounds.min.x, bounds.max.y, bounds.max.z}, {bounds.max.x, bounds.max.y, bounds.max.z},
    };

    float rmin = glm::dot(corners[0], basis.right), rmax = rmin;
    float umin = glm::dot(corners[0], basis.up), umax = umin;
    for (size_t i = 1; i < 8; ++i) {
        const float r = glm::dot(corners[i], basis.right);
        const float u = glm::dot(corners[i], basis.up);
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
        umin = std::min(umin, u);
        umax = std::max(umax, u);
    }

    const float sx = 2.0f / std::max(rmax - rmin, kMinLightExtent);
    const float sy = 2.0f / std::max(umax - umin, kMinLightExtent);
    // Solves ndc(rmin) == -1, ndc(rmax) == +1 for ndc(r) = sx*r + ox.
    const float ox = -1.0f - sx * rmin;
    const float oy = -1.0f - sy * umin;

    const glm::vec3 center = (bounds.min + bounds.max) * 0.5f;
    const float oz = -glm::dot(basis.forward, center);

    glm::mat4 m(1.0f);
    m[0][0] = sx * basis.right.x;
    m[1][0] = sx * basis.right.y;
    m[2][0] = sx * basis.right.z;
    m[3][0] = ox;
    m[0][1] = sy * basis.up.x;
    m[1][1] = sy * basis.up.y;
    m[2][1] = sy * basis.up.z;
    m[3][1] = oy;
    m[0][2] = basis.forward.x;
    m[1][2] = basis.forward.y;
    m[2][2] = basis.forward.z;
    m[3][2] = oz;
    m[0][3] = 0.0f;
    m[1][3] = 0.0f;
    m[2][3] = 0.0f;
    m[3][3] = 1.0f;
    return m;
}

// Depth-only fill of one world-space triangle into `map.depth`, via
// `light_view_proj`. AFFINE throughout (orthographic: no perspective divide,
// so the barycentric weights computed from the PROJECTED 2D points are
// already exact for interpolating `z` too -- unlike raster_cpu.cpp's camera-
// space rasterizer, there is no invDepth/perspective-correction concern on
// this path at all). Mirrors raster_cpu.cpp's rasterizeTriangleFlat
// structurally (bounding box, edge functions, both-signs-count-as-inside)
// but is not the SAME function: that one is `static` to raster_cpu.cpp's own
// anonymous namespace and works in double camera-space screen coordinates;
// this one works in float shadow-texel space and has no colour, only depth.
void rasterize_shadow_triangle(const glm::vec3& wa, const glm::vec3& wb, const glm::vec3& wc,
                                const glm::mat4& light_view_proj, ShadowMap& map) {
    const glm::vec4 pa = light_view_proj * glm::vec4(wa, 1.0f);
    const glm::vec4 pb = light_view_proj * glm::vec4(wb, 1.0f);
    const glm::vec4 pc = light_view_proj * glm::vec4(wc, 1.0f);

    const float size_f = static_cast<float>(map.size);
    const float xa = (pa.x * 0.5f + 0.5f) * size_f, ya = (pa.y * 0.5f + 0.5f) * size_f;
    const float xb = (pb.x * 0.5f + 0.5f) * size_f, yb = (pb.y * 0.5f + 0.5f) * size_f;
    const float xc = (pc.x * 0.5f + 0.5f) * size_f, yc = (pc.y * 0.5f + 0.5f) * size_f;

    const float area = (xb - xa) * (yc - ya) - (yb - ya) * (xc - xa);
    if (std::fabs(area) < 1e-9f) {
        return;  // degenerate (zero on-screen area)
    }

    // Clamped BEFORE the floor/ceil+cast, not merely after (M1, R7 fix round
    // 1): static_cast<int> of a float far outside int32's range is
    // undefined behaviour, and the std::max(0, ...)/std::min(size_i-1, ...)
    // calls below only clamp the ALREADY-cast int -- too late to matter for
    // the UB itself. kMaxTexelCoord is comfortably inside int32 range with
    // wide margin (the committed corpus's largest map reaches ~19,456
    // texel-units square) while still being far larger than any of this
    // file's own maps (`size` is a uint32_t but every constructed ShadowMap
    // in this program is 64-2048) could legitimately produce; a pathological
    // caller-supplied `light_view_proj` (a degenerate transform, an
    // unvalidated hand-built RenderScene) is exactly the case this guards.
    constexpr float kMaxTexelCoord = 1.0e8f;
    const auto safe_coord = [](float v) { return std::clamp(v, -kMaxTexelCoord, kMaxTexelCoord); };
    const int size_i = static_cast<int>(map.size);
    const int x0 = std::max(0, static_cast<int>(std::floor(safe_coord(std::min({xa, xb, xc})))));
    const int x1 = std::min(size_i - 1, static_cast<int>(std::ceil(safe_coord(std::max({xa, xb, xc})))));
    const int y0 = std::max(0, static_cast<int>(std::floor(safe_coord(std::min({ya, yb, yc})))));
    const int y1 = std::min(size_i - 1, static_cast<int>(std::ceil(safe_coord(std::max({ya, yb, yc})))));

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
            const float w0 = (xc - xb) * (py - yb) - (yc - yb) * (px - xb);
            const float w1 = (xa - xc) * (py - yc) - (ya - yc) * (px - xc);
            const float w2 = (xb - xa) * (py - ya) - (yb - ya) * (px - xa);
            const bool inside = (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) || (w0 <= 0.0f && w1 <= 0.0f && w2 <= 0.0f);
            if (!inside) {
                continue;
            }
            const float b0 = w0 / area, b1 = w1 / area, b2 = w2 / area;
            const float z = b0 * pa.z + b1 * pb.z + b2 * pc.z;
            const size_t idx = static_cast<size_t>(y) * map.size + static_cast<size_t>(x);
            if (z > map.depth[idx]) {
                map.depth[idx] = z;  // closer to the sun wins -- same "larger wins" sense as invDepth
            }
        }
    }
}

}  // namespace

void rasterize_shadow_casters(std::span<const MeshData> meshes, std::span<const DrawItem> items, ShadowMap& map) {
    // Deterministic: `items` and each mesh's own index buffer are iterated in
    // their existing, fixed order -- no RNG, no wall clock, no unordered
    // iteration, matching raster_cpu.cpp's own render()/draw_mesh_item
    // discipline.
    for (const DrawItem& item : items) {
        if (item.mesh_index >= meshes.size()) {
            continue;  // kNoMesh, or an out-of-range slot -- nothing to cast.
        }
        const MeshData& mesh = meshes[item.mesh_index];
        for (size_t t = 0; t + 3 <= mesh.indices.size(); t += 3) {
            const uint32_t ia = mesh.indices[t];
            const uint32_t ib = mesh.indices[t + 1];
            const uint32_t ic = mesh.indices[t + 2];
            if (ia >= mesh.positions.size() || ib >= mesh.positions.size() || ic >= mesh.positions.size()) {
                continue;  // an index points past the vertex buffer -- skip this triangle, not the mesh.
            }
            const glm::vec3 wa(item.local_to_world * glm::vec4(mesh.positions[ia], 1.0f));
            const glm::vec3 wb(item.local_to_world * glm::vec4(mesh.positions[ib], 1.0f));
            const glm::vec3 wc(item.local_to_world * glm::vec4(mesh.positions[ic], 1.0f));
            rasterize_shadow_triangle(wa, wb, wc, map.light_view_proj, map);
        }
    }
}

Result<ShadowMap> build_static_shadow_map(const RenderScene& scene, uint32_t size) {
    ShadowMap map;
    map.size = size;
    map.depth.assign(static_cast<size_t>(size) * static_cast<size_t>(size), kNoOccluder);

    const LightBasis basis = build_light_basis(scene.lighting.sun_direction);
    map.light_view_proj = build_light_view_proj(basis, scene.bounds);

    // Step 3's own split: STATIC geometry only, at build time. Dynamics are
    // folded into a per-frame COPY of this result by a separate
    // rasterize_shadow_casters() call (raster_cpu.cpp's render()), never
    // here.
    rasterize_shadow_casters(scene.meshes, scene.statics, map);
    return map;
}

float sample_shadow(const ShadowMap& map, const glm::vec3& world_pos) {
    if (map.size == 0 || map.depth.empty()) {
        return 1.0f;
    }
    // Depth bias (Step 2): trades shadow acne (too small -- a lit surface
    // incorrectly self-shadows against its own quantised shadow-map depth)
    // against peter-panning (too large -- a caster's shadow visibly
    // detaches/shrinks from its own base). TUNED against this task's own
    // tests/test_render_shadow.cpp fixtures (casters and receivers a few
    // world units apart, shadow-map texel footprints on that same scale) --
    // NOT the same constant as raster_cpu.cpp's kOverlayDepthBias, which
    // biases a different quantity (a screen-space invDepth, not a
    // world-space light-axis distance) to fix a different failure mode (an
    // overlay disappearing behind the surface it annotates, not shadow
    // acne). Conflating the two would be a defect even if the numbers
    // happened to coincide (this task's own controller amendment).
    constexpr float kShadowDepthBias = 0.05f;

    const glm::vec4 p = map.light_view_proj * glm::vec4(world_pos, 1.0f);
    if (p.x < -1.0f || p.x > 1.0f || p.y < -1.0f || p.y > 1.0f) {
        return 1.0f;  // SR-17: outside the shadow frustum's own footprint -- unshadowed, by design.
    }

    const float size_f = static_cast<float>(map.size);
    const int size_i = static_cast<int>(map.size);
    const int tx = std::clamp(static_cast<int>((p.x * 0.5f + 0.5f) * size_f), 0, size_i - 1);
    const int ty = std::clamp(static_cast<int>((p.y * 0.5f + 0.5f) * size_f), 0, size_i - 1);
    const size_t idx = static_cast<size_t>(ty) * map.size + static_cast<size_t>(tx);

    const float occluder = map.depth[idx];
    if (occluder == kNoOccluder) {
        return 1.0f;  // no caster ever covered this texel.
    }
    return (p.z < occluder - kShadowDepthBias) ? 0.0f : 1.0f;
}

}  // namespace spade::render
