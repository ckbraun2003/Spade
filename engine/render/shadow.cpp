#include "render/shadow.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

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
// A straight-up sun, (0,1,0), takes this branch, and so does the zero-length
// fallback below. LightingDesc's default is (0.4, 0.8, 0.6) since the VQ-A
// fix (world/builder.hpp), so an unauthored world no longer reaches it.
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
// reached. Falls back to straight up, (0,1,0), rather than propagating a
// NaN through the rest of this file.
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
    // kMaxTexelCoord is CAPTURED, not left to the empty-capture default:
    // std::clamp takes its bounds by const reference, so naming the variable
    // odr-uses it, and a constexpr local that is odr-used inside a lambda
    // must be captured. MSVC accepts the uncaptured form; g++-13 correctly
    // rejects it ("'kMaxTexelCoord' is not captured"), which is what broke
    // the spade-linux CI job. `-kMaxTexelCoord` alone would have been fine --
    // unary minus yields a prvalue -- but the positive bound binds a
    // reference straight to the variable.
    const auto safe_coord = [kMaxTexelCoord](float v) {
        return std::clamp(v, -kMaxTexelCoord, kMaxTexelCoord);
    };
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

// A receiver almost edge-on to the sun has a near-zero n . forward; its slope
// is held to 1 / kMinFacing (20) rather than divided by nothing. Such a
// surface takes almost no sun term anyway.
constexpr float kMinFacing = 0.05f;

// How much more bias (in texels' metres) a filter tap away from the point's own texel carries; see sample_shadow().
constexpr float kTapSlackTexels = 1.0f;

// sample_shadow()'s receiver plane: the light-space depth, at texel (tx, ty)'s
// centre, of the plane through the point whose light-space position is `p`,
// with unit normal `normal`. light_view_proj's rows are sx * right, sy * up
// and forward (build_light_view_proj), so a step of d in ndc is d / s metres
// along right (up), and along the plane the depth moves -(n . right) /
// (n . forward) per metre right and -(n . up) / (n . forward) per metre up.
// Either sign of `normal` gives the same plane.
// The scalars of that plane, which depend on the point's normal and the map but not on the texel: formed once per
// sample, so each tap of a filter costs two multiplies. The expression per tap is the one this file always had.
struct ReceiverPlane {
    float n_row0 = 0.0f, row0_row0 = 1.0f;  // n . row0, row0 . row0
    float n_row1 = 0.0f, row1_row1 = 1.0f;
    float facing = 1.0f;
    float size_f = 1.0f;

    [[nodiscard]] static ReceiverPlane of(const ShadowMap& map, const glm::vec3& normal) {
        const glm::mat4& m = map.light_view_proj;
        const glm::vec3 row0(m[0][0], m[1][0], m[2][0]);
        const glm::vec3 row1(m[0][1], m[1][1], m[2][1]);
        const glm::vec3 row2(m[0][2], m[1][2], m[2][2]);
        ReceiverPlane r;
        r.facing = glm::dot(normal, row2);
        if (std::fabs(r.facing) < kMinFacing) {
            r.facing = r.facing < 0.0f ? -kMinFacing : kMinFacing;
        }
        r.n_row0 = glm::dot(normal, row0);
        r.row0_row0 = glm::dot(row0, row0);
        r.n_row1 = glm::dot(normal, row1);
        r.row1_row1 = glm::dot(row1, row1);
        r.size_f = static_cast<float>(map.size);
        return r;
    }

    [[nodiscard]] float depth_at_texel_centre(const glm::vec4& p, int tx, int ty) const {
        const float dx = (static_cast<float>(tx) + 0.5f) / size_f * 2.0f - 1.0f - p.x;
        const float dy = (static_cast<float>(ty) + 0.5f) / size_f * 2.0f - 1.0f - p.y;
        const float along = dx * n_row0 / row0_row0 + dy * n_row1 / row1_row1;
        return p.z - along / facing;
    }
};

// The padding a focus map keeps clear around its bodies' footprint, in texels a side: what a filter of this width
// reads past the edge, and two more.
[[nodiscard]] float focus_pad_texels(float filter_texels) { return std::ceil(std::max(filter_texels, 0.0f)) + 2.0f; }

}  // namespace

void rasterize_shadow_casters(std::span<const MeshData> meshes, std::span<const DrawItem> items, ShadowMap& map) {
    // Deterministic: `items` and each mesh's own index buffer are iterated in
    // their existing, fixed order -- no RNG, no wall clock, no unordered
    // iteration, matching raster_cpu.cpp's own render()/draw_mesh_item
    // discipline.
    for (const DrawItem& item : items) {
        if (item.see_through || !item.casts_shadow) {
            continue;  // RND-11: a see-through item receives shadows and never casts one; the terrain the same.
        }
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

float shadow_texel_metres(const ShadowMap& map) {
    const glm::mat4& m = map.light_view_proj;
    const float per_metre = std::min(glm::length(glm::vec3(m[0][0], m[1][0], m[2][0])),
                                     glm::length(glm::vec3(m[0][1], m[1][1], m[2][1])));
    return map.size == 0 || per_metre <= 0.0f ? 0.0f : 2.0f / (per_metre * static_cast<float>(map.size));
}

namespace {

// The dynamic items' light-space bounds (the extremes of every vertex's right, up and forward in the light's basis).
struct LightBounds {
    float rmin = 0.0f, rmax = 0.0f, umin = 0.0f, umax = 0.0f, zmin = 0.0f, zmax = 0.0f;
    bool any = false;
};

// One item's vertices into `b`: the original loop's body, shared by the exhaustive fit and the skip fit so each vertex
// takes the same operations in the same order whichever way it is reached.
void accumulate_item(const DrawItem& item, const std::vector<glm::vec3>& positions, const LightBasis& basis,
                     LightBounds& b) {
    for (const glm::vec3& local : positions) {
        const glm::vec3 w(item.local_to_world * glm::vec4(local, 1.0f));
        const float r = glm::dot(w, basis.right), u = glm::dot(w, basis.up), z = glm::dot(w, basis.forward);
        if (!b.any) {
            b.rmin = b.rmax = r;
            b.umin = b.umax = u;
            b.zmin = b.zmax = z;
            b.any = true;
        } else {
            b.rmin = std::min(b.rmin, r);
            b.rmax = std::max(b.rmax, r);
            b.umin = std::min(b.umin, u);
            b.umax = std::max(b.umax, u);
            b.zmin = std::min(b.zmin, z);
            b.zmax = std::max(b.zmax, z);
        }
    }
}

// Whether the fit looks at this item at all: a see-through item casts nothing (RND-11), so it has no footprint to fit.
[[nodiscard]] bool fits_item(const RenderScene& scene, const DrawItem& item) {
    return !item.see_through && item.casts_shadow && item.mesh_index < scene.meshes.size();
}

// The original: every vertex of every item, in the items' fixed order.
LightBounds exhaustive_bounds(const RenderScene& scene, const LightBasis& basis, FocusFitStats* stats) {
    LightBounds b;
    uint64_t vertices = 0;
    for (const DrawItem& item : scene.dynamics) {
        if (!fits_item(scene, item)) continue;
        accumulate_item(item, scene.meshes[item.mesh_index].positions, basis, b);
        vertices += scene.meshes[item.mesh_index].positions.size();
    }
    if (stats != nullptr) stats->vertices_total = stats->vertices_transformed = vertices;
    return b;
}

// A mesh's local bounding box and ball (centre, half extents, radius), in double, for the item bounds.
struct MeshBall {
    double centre[3] = {0.0, 0.0, 0.0};
    double half[3] = {0.0, 0.0, 0.0};
    double radius = 0.0;
    bool ready = false;
};

// One item's light-space extent, per axis (right, up, forward), in double: lo and hi enclose every vertex's computed
// value, rounding included.
struct ItemExtent {
    double lo[3] = {0.0, 0.0, 0.0};
    double hi[3] = {0.0, 0.0, 0.0};
    uint32_t item = 0;
};

// The skip fit. Returns false (and the caller takes the exhaustive path) on anything not finite: a pose, a vertex or a
// bound.
[[nodiscard]] bool skip_bounds(const RenderScene& scene, const LightBasis& basis, LightBounds& out, FocusFitStats* stats) {
    std::vector<MeshBall> balls(scene.meshes.size());
    std::vector<ItemExtent> extents;
    extents.reserve(scene.dynamics.size());
    const glm::dvec3 dirs[3] = {glm::dvec3(basis.right), glm::dvec3(basis.up), glm::dvec3(basis.forward)};
    uint64_t total = 0;
    for (uint32_t index = 0; index < scene.dynamics.size(); ++index) {
        const DrawItem& item = scene.dynamics[index];
        if (!fits_item(scene, item)) continue;
        const std::vector<glm::vec3>& positions = scene.meshes[item.mesh_index].positions;
        if (positions.empty()) continue;
        total += positions.size();
        MeshBall& ball = balls[item.mesh_index];
        if (!ball.ready) {
            glm::dvec3 lo(positions.front()), hi(positions.front());
            for (const glm::vec3& p : positions) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
                lo = glm::min(lo, glm::dvec3(p));
                hi = glm::max(hi, glm::dvec3(p));
            }
            double radius_sq = 0.0;
            for (int k = 0; k < 3; ++k) {
                ball.centre[k] = 0.5 * (lo[k] + hi[k]);
                ball.half[k] = 0.5 * (hi[k] - lo[k]);
            }
            for (const glm::vec3& p : positions) {
                const double dx = p.x - ball.centre[0], dy = p.y - ball.centre[1], dz = p.z - ball.centre[2];
                radius_sq = std::max(radius_sq, dx * dx + dy * dy + dz * dz);
            }
            ball.radius = std::sqrt(radius_sq);
            ball.ready = true;
        }
        const glm::mat4& m = item.local_to_world;
        for (int k = 0; k < 4; ++k) {  // the fit uses columns 0..3 of rows 0..2 (the w row is dropped)
            if (!std::isfinite(m[k][0]) || !std::isfinite(m[k][1]) || !std::isfinite(m[k][2])) return false;
        }
        ItemExtent e;
        e.item = index;
        for (int axis = 0; axis < 3; ++axis) {
            const glm::dvec3& d = dirs[axis];
            double a[3], mag = 0.0, centre_value = 0.0, a_sq = 0.0, box = 0.0;
            for (int k = 0; k < 3; ++k) {
                a[k] = d.x * m[k][0] + d.y * m[k][1] + d.z * m[k][2];
                centre_value += a[k] * ball.centre[k];
                a_sq += a[k] * a[k];
                box += std::fabs(a[k]) * ball.half[k];
                mag += (std::fabs(d.x * m[k][0]) + std::fabs(d.y * m[k][1]) + std::fabs(d.z * m[k][2])) *
                       (std::fabs(ball.centre[k]) + ball.half[k]);
            }
            const double b = d.x * m[3][0] + d.y * m[3][1] + d.z * m[3][2];
            centre_value += b;
            mag += std::fabs(d.x * m[3][0]) + std::fabs(d.y * m[3][1]) + std::fabs(d.z * m[3][2]);
            const double reach = std::min(box, ball.radius * std::sqrt(a_sq));
            // The computed float is within a few units of 2^-24 of the exact sum of the terms' magnitudes; the margin
            // is 1e-5 of it, ten times what the transform and the dot can lose.
            const double margin = 1.0e-5 * mag + 1.0e-30;  // and a floor over any flushed subnormal
            e.lo[axis] = centre_value - reach - margin;
            e.hi[axis] = centre_value + reach + margin;
            if (!(std::fabs(e.lo[axis]) < 1.0e30) || !(std::fabs(e.hi[axis]) < 1.0e30)) return false;
        }
        extents.push_back(e);
    }
    if (stats != nullptr) stats->vertices_total = total;
    if (extents.empty()) return true;  // nothing to fit: out.any stays false, as the exhaustive loop's would
    std::vector<char> done(extents.size(), 0);
    uint64_t transformed = 0;
    const auto evaluate = [&](size_t at) {
        done[at] = 1;
        const DrawItem& item = scene.dynamics[extents[at].item];
        const std::vector<glm::vec3>& positions = scene.meshes[item.mesh_index].positions;
        accumulate_item(item, positions, basis, out);
        transformed += positions.size();
    };
    // The items whose bound reaches furthest each way set the extremes first, so the rest can be skipped against them.
    for (int axis = 0; axis < 3; ++axis) {
        size_t lowest = 0, highest = 0;
        for (size_t i = 1; i < extents.size(); ++i) {
            if (extents[i].lo[axis] < extents[lowest].lo[axis]) lowest = i;
            if (extents[i].hi[axis] > extents[highest].hi[axis]) highest = i;
        }
        if (done[lowest] == 0) evaluate(lowest);
        if (done[highest] == 0) evaluate(highest);
    }
    for (size_t i = 0; i < extents.size(); ++i) {
        if (done[i] != 0) continue;
        const ItemExtent& e = extents[i];
        // Strictly inside the extremes found: no vertex of this item can be the min or the max (an equal one could
        // flip a signed zero), so it is never transformed.
        const bool inside = e.lo[0] > static_cast<double>(out.rmin) && e.hi[0] < static_cast<double>(out.rmax) &&
                            e.lo[1] > static_cast<double>(out.umin) && e.hi[1] < static_cast<double>(out.umax) &&
                            e.lo[2] > static_cast<double>(out.zmin) && e.hi[2] < static_cast<double>(out.zmax);
        if (!inside) evaluate(i);
    }
    if (stats != nullptr) stats->vertices_transformed = transformed;
    return true;
}

std::optional<ShadowMap> fit_focus(const RenderScene& scene, uint32_t size, float filter_texels, const ShadowMap& statics,
                                   bool exhaustive, FocusFitStats* stats) {
    if (stats != nullptr) *stats = FocusFitStats{};
    if (size == 0 || scene.dynamics.empty()) {
        return std::nullopt;
    }
    const float pad = focus_pad_texels(filter_texels);
    if (static_cast<float>(size) <= 2.0f * pad) {
        return std::nullopt;
    }
    const LightBasis basis = build_light_basis(scene.lighting.sun_direction);
    // The dynamic items' light-space bounds, over every vertex of every mesh they draw.
    LightBounds bounds;
    if (exhaustive) {
        bounds = exhaustive_bounds(scene, basis, stats);
    } else if (!skip_bounds(scene, basis, bounds, stats)) {
        bounds = exhaustive_bounds(scene, basis, stats);
    }
    const float rmin = bounds.rmin, rmax = bounds.rmax, umin = bounds.umin, umax = bounds.umax, zmin = bounds.zmin,
                zmax = bounds.zmax;
    if (!bounds.any) {
        return std::nullopt;
    }
    // One texel size for both axes: the larger footprint side over the texels left inside the padding.
    const float size_f = static_cast<float>(size);
    const float side = std::max({rmax - rmin, umax - umin, kMinLightExtent});
    const float extent = side * size_f / (size_f - 2.0f * pad);
    ShadowMap map;
    map.size = size;
    map.depth_bias = kFocusShadowDepthBias;
    const float s = 2.0f / extent;
    const float rc = 0.5f * (rmin + rmax), uc = 0.5f * (umin + umax);
    glm::mat4 m(1.0f);
    m[0][0] = s * basis.right.x;
    m[1][0] = s * basis.right.y;
    m[2][0] = s * basis.right.z;
    m[3][0] = -s * rc;
    m[0][1] = s * basis.up.x;
    m[1][1] = s * basis.up.y;
    m[2][1] = s * basis.up.z;
    m[3][1] = -s * uc;
    m[0][2] = basis.forward.x;
    m[1][2] = basis.forward.y;
    m[2][2] = basis.forward.z;
    m[3][2] = -0.5f * (zmin + zmax);
    map.light_view_proj = m;
    // Never worse than folding: a map no finer than the world's gives nothing the folded copy does not.
    if (shadow_texel_metres(map) >= shadow_texel_metres(statics)) {
        return std::nullopt;
    }
    return map;
}

}  // namespace

std::optional<ShadowMap> fit_focus_shadow_map(const RenderScene& scene, uint32_t size, float filter_texels,
                                              const ShadowMap& statics, FocusFitStats* stats) {
    return fit_focus(scene, size, filter_texels, statics, false, stats);
}

std::optional<ShadowMap> fit_focus_shadow_map_exhaustive(const RenderScene& scene, uint32_t size, float filter_texels,
                                                         const ShadowMap& statics) {
    return fit_focus(scene, size, filter_texels, statics, true, nullptr);
}

std::optional<ShadowMap> build_focus_shadow_map(const RenderScene& scene, uint32_t size, float filter_texels,
                                                const ShadowMap& statics, std::vector<float>* scratch) {
    std::optional<ShadowMap> fitted = fit_focus_shadow_map(scene, size, filter_texels, statics);
    if (!fitted) {
        return std::nullopt;
    }
    ShadowMap& map = *fitted;
    if (scratch != nullptr) {
        map.depth = std::move(*scratch);
    }
    map.depth.assign(static_cast<size_t>(size) * size, kNoOccluder);
    rasterize_shadow_casters(scene.meshes, scene.dynamics, map);
    return fitted;
}

float shadow_tap_slack(const ShadowMap& map) { return kTapSlackTexels * shadow_texel_metres(map); }

float ShadowSet::sample(const glm::vec3& world_pos, const glm::vec3& normal) const {
    float lit = statics == nullptr ? 1.0f : sample_shadow(*statics, world_pos, normal, filter_texels);
    if (focus != nullptr && lit > 0.0f) {
        lit *= sample_shadow(*focus, world_pos, normal, filter_texels);
    }
    return lit;
}

// Whether `world` is certainly outside the map's [-1, 1] footprint in light-space x or y, from the matrix's first two
// rows alone (no z, no w: half the multiplies of the full product) and a margin that dwarfs the rounding of either
// way of forming the sum (each of the four terms and three additions is good to a few 1e-8 of the sum of the terms'
// magnitudes; the margin is 1e-5 of it). Only a point beyond the footprint by more than that margin is rejected, so
// every point the full product would put outside gets the full product's answer (lit), and every other goes on to it.
[[nodiscard]] static bool surely_outside_footprint(const glm::mat4& m, const glm::vec3& w) {
    for (int row = 0; row < 2; ++row) {
        const float a = m[0][row] * w.x, b = m[1][row] * w.y, c = m[2][row] * w.z, d = m[3][row];
        const float margin = 1.0e-5f * (std::fabs(a) + std::fabs(b) + std::fabs(c) + std::fabs(d));
        const float v = (a + b) + (c + d);
        if (v < -1.0f - margin || v > 1.0f + margin) {
            return true;
        }
    }
    return false;
}

// sample_shadow() with the early-outs switchable, so a test can hold them to the full computation.
static float sample_shadow_impl(const ShadowMap& map, const glm::vec3& world_pos, const glm::vec3& normal,
                                float filter_texels, bool allow_skip) {
    if (map.size == 0 || map.depth.empty()) {
        return 1.0f;
    }
    // kShadowDepthBias (shadow.hpp) says what the bias still covers; GL's
    // lookup uses the same value and the same receiver plane.
    if (allow_skip && surely_outside_footprint(map.light_view_proj, world_pos)) {
        return 1.0f;  // far outside the footprint: the answer the product below would give, without it.
    }
    const glm::vec4 p = map.light_view_proj * glm::vec4(world_pos, 1.0f);
    if (p.x < -1.0f || p.x > 1.0f || p.y < -1.0f || p.y > 1.0f) {
        return 1.0f;  // SR-17: outside the shadow frustum's own footprint -- unshadowed, by design.
    }

    const float size_f = static_cast<float>(map.size);
    const int size_i = static_cast<int>(map.size);
    const int tx = std::clamp(static_cast<int>((p.x * 0.5f + 0.5f) * size_f), 0, size_i - 1);
    const int ty = std::clamp(static_cast<int>((p.y * 0.5f + 0.5f) * size_f), 0, size_i - 1);
    const size_t idx = static_cast<size_t>(ty) * map.size + static_cast<size_t>(tx);

    if (!(filter_texels > 0.0f)) {
        const ReceiverPlane plane = ReceiverPlane::of(map, normal);
        const float occluder = map.depth[idx];
        if (occluder == kNoOccluder) {
            return 1.0f;  // no caster ever covered this texel.
        }
        // The receiver's plane may lift it toward the sun, never sink it (shadow.hpp).
        const float depth = std::max(p.z, plane.depth_at_texel_centre(p, tx, ty));
        return (depth < occluder - map.depth_bias) ? 0.0f : 1.0f;
    }

    // The area-weighted box `filter_texels` wide about the point, in texel coordinates; each texel it overlaps is
    // tested with its own receiver plane and weighted by its overlap (so a width of 1 is bilinear). Rows first, in
    // increasing order: the sum is the same on every run. A map's edge repeats its last texel.
    const float half = 0.5f * filter_texels;
    const float vx = (p.x * 0.5f + 0.5f) * size_f, vy = (p.y * 0.5f + 0.5f) * size_f;
    const int x0 = static_cast<int>(std::floor(vx - half)), x1 = static_cast<int>(std::floor(vx + half));
    const int y0 = static_cast<int>(std::floor(vy - half)), y1 = static_cast<int>(std::floor(vy + half));
    // Nothing under the box nearer than the point: lit, without the plane arithmetic (most pixels). A tap shadows only
    // when depth < occluder - bias, and depth is never below the point's own p.z, so a box whose nearest stored depth
    // (kNoOccluder, the lowest float, for an empty texel) is within the bias of p.z cannot shadow it: the same answer
    // the loop below gives, with fewer operations. The slack only ever adds bias, so it cannot change this either.
    if (allow_skip) {
        float nearest = kNoOccluder;
        for (int y = y0; y <= y1; ++y) {
            const size_t row = static_cast<size_t>(std::clamp(y, 0, size_i - 1)) * map.size;
            for (int x = x0; x <= x1; ++x) {
                nearest = std::max(nearest, map.depth[row + static_cast<size_t>(std::clamp(x, 0, size_i - 1))]);
            }
        }
        if (nearest - map.depth_bias <= p.z) {
            return 1.0f;
        }
    }
    const ReceiverPlane plane = ReceiverPlane::of(map, normal);
    // A tap away from the point's own texel reads a texel that may belong to a neighbouring face of the same body (a
    // convex edge): that face can lie nearer the sun than this receiver's plane by up to a texel's run times its own
    // slope, which the plane cannot know. Those taps carry that much more bias (a texel's metres, a neighbour's slope of 1), so the filter
    // widens a shadow's edge without shadowing a lit face beside an edge; the point's own texel keeps the bias alone.
    const float tap_slack = shadow_tap_slack(map);
    float lit_sum = 0.0f, weight_sum = 0.0f;
    for (int y = y0; y <= y1; ++y) {
        const float wy = std::min(static_cast<float>(y + 1), vy + half) - std::max(static_cast<float>(y), vy - half);
        if (!(wy > 0.0f)) continue;
        const int cy = std::clamp(y, 0, size_i - 1);
        for (int x = x0; x <= x1; ++x) {
            const float wx = std::min(static_cast<float>(x + 1), vx + half) - std::max(static_cast<float>(x), vx - half);
            if (!(wx > 0.0f)) continue;
            const int cx = std::clamp(x, 0, size_i - 1);
            const float w = wx * wy;
            const float occluder = map.depth[static_cast<size_t>(cy) * map.size + static_cast<size_t>(cx)];
            float lit = 1.0f;
            if (occluder != kNoOccluder) {
                const float depth = std::max(p.z, plane.depth_at_texel_centre(p, cx, cy));
                const float bias = map.depth_bias + ((cx == tx && cy == ty) ? 0.0f : tap_slack);
                lit = (depth < occluder - bias) ? 0.0f : 1.0f;
            }
            lit_sum += w * lit;
            weight_sum += w;
        }
    }
    return weight_sum > 0.0f ? lit_sum / weight_sum : 1.0f;
}

float sample_shadow(const ShadowMap& map, const glm::vec3& world_pos, const glm::vec3& normal, float filter_texels) {
    return sample_shadow_impl(map, world_pos, normal, filter_texels, true);
}

float sample_shadow_without_early_out(const ShadowMap& map, const glm::vec3& world_pos, const glm::vec3& normal,
                                      float filter_texels) {
    return sample_shadow_impl(map, world_pos, normal, filter_texels, false);
}

}  // namespace spade::render
