#include "render/raster_cpu.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/fp32_math.hpp"

namespace spade::render {

namespace {

using math::cos32;
using math::sin32;

// ---------------------------------------------------------------------------
// Camera-space math -- a DIRECT, UNCHANGED port of the wireframe rasterizer's
// own local Vec3/ScreenPoint/ViewContext/rasterization block (down to
// variable names). This block has zero dependency on RenderScene's concrete
// type -- it only ever touches a plain double Vec3/ScreenPoint -- so there is
// nothing scene-specific to adapt; see raster_cpu.hpp's own header for why
// porting it verbatim (rather than reworking it around glm::vec3/float) is
// the deliberate, lower-risk choice: it is the wireframe rasterizer's own
// thoroughly-commented, already-correct camera/projection/depth code, and
// this task is to port the rasterizer, not redesign it. Only the WORLD-SPACE
// GEOMETRY drawing functions further below (draw_mesh_item and the overlay
// passes) read RenderScene-typed inputs and are new/adapted.
// ---------------------------------------------------------------------------

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Rotates `v` by the unit quaternion q=(w,x,y,z) -- the standard optimized
// form: v' = v + 2*w*(u x v) + 2*(u x (u x v)), u = q.xyz.
Vec3 rotateByQuat(const double q[4], const Vec3& v) {
    const Vec3 u{q[1], q[2], q[3]};
    const Vec3 uCrossV = cross(u, v);
    return v + (uCrossV * (2.0 * q[0])) + (cross(u, uCrossV) * 2.0);
}

void quatConjugate(const double q[4], double out[4]) {
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

struct ViewContext {
    double camPos[3];
    double camOrientationConj[4];  // world-to-camera rotation = conj(pose orientation)
    double fovY;
    double nearP;
    double farP;
    uint32_t width;
    uint32_t height;
};

Vec3 worldToCameraSpace(const ViewContext& vc, const Vec3& p) {
    const Vec3 rel{p.x - vc.camPos[0], p.y - vc.camPos[1], p.z - vc.camPos[2]};
    return rotateByQuat(vc.camOrientationConj, rel);
}

struct ScreenPoint {
    double x = 0.0, y = 0.0, invDepth = 0.0;
};

// NEW (adaptation, not part of the port): the wireframe rasterizer's own
// projectCameraSpace calls std::tan() directly, which is legal there --
// dronesim/spade/ is kat-side, outside this engine's determinism contract.
// This file lives under spade/engine/render/, which IS scanned by
// tools/tests/test_m1b_bar.cpp's BitPortability.NoLibmTranscendentalInEngineSource
// (no libm transcendental may execute on a path that feeds a committed golden
// digest -- render/tessellate.cpp's own header comment states the identical
// rule for the exact same reason: two conforming libms can disagree by a ulp,
// which would move this file's own committed sha256 frame goldens silently
// between platforms/compilers). half_fov_rad is always small (a camera's half
// vertical FOV), always well inside sin32/cos32's accurate domain.
[[nodiscard]] double tan32(double half_fov_rad) {
    const float x = static_cast<float>(half_fov_rad);
    return static_cast<double>(sin32(x)) / static_cast<double>(cos32(x));
}

// Projects an ALREADY near/far-valid camera-space point (pc.z in
// [-farP, -nearP]) to screen space. invDepth = 1/-pc.z (larger = closer).
ScreenPoint projectCameraSpace(const ViewContext& vc, const Vec3& pc) {
    const double f = 1.0 / tan32(vc.fovY * 0.5);
    const double aspect = static_cast<double>(vc.width) / static_cast<double>(vc.height);
    const double invNegZ = 1.0 / (-pc.z);
    const double xNdc = (f / aspect) * pc.x * invNegZ;
    const double yNdc = f * pc.y * invNegZ;
    ScreenPoint sp;
    sp.x = (xNdc * 0.5 + 0.5) * vc.width;
    sp.y = (1.0 - (yNdc * 0.5 + 0.5)) * vc.height;  // NDC +Y (up) -> screen +Y (down)
    sp.invDepth = invNegZ;
    return sp;
}

// Clips camera-space segment [p0, p1] to the half-space `keepLessEq ? z <=
// planeZ : z >= planeZ`, in place. Returns false if the whole segment is
// outside (nothing to draw).
bool clipSegmentToHalfSpace(Vec3& p0, Vec3& p1, double planeZ, bool keepLessEq) {
    const auto inside = [&](const Vec3& p) { return keepLessEq ? (p.z <= planeZ) : (p.z >= planeZ); };
    const bool in0 = inside(p0);
    const bool in1 = inside(p1);
    if (!in0 && !in1) {
        return false;
    }
    if (in0 && in1) {
        return true;
    }
    const double t = (planeZ - p0.z) / (p1.z - p0.z);
    const Vec3 clipPoint = p0 + (p1 - p0) * t;
    if (!in0) {
        p0 = clipPoint;
    } else {
        p1 = clipPoint;
    }
    return true;
}

// A clip vertex -- camera-space position only for now. Deliberately a
// struct (not a bare Vec3) even though position is its only field today:
// Task R6 needs an interpolated normal at each clip vertex, computed at the
// SAME `t` the position below is, and must add it as a second field HERE
// rather than forking this clipper into a second, normal-aware copy
// (task-R5b-brief.md's own forward note).
struct ClipVertex {
    Vec3 pos;
};

// Sutherland-Hodgman: clips the convex polygon `poly` (>= 3 vertices, or
// empty) against the half-space `keepLessEq ? z <= planeZ : z >= planeZ`,
// returning the result (possibly empty; at most poly.size()+1 vertices).
// Reuses clipSegmentToHalfSpace's own interpolation formula --
// t = (planeZ - cur.z) / (next.z - cur.z), cur + (next - cur) * t -- rather
// than a second one (SR-15's "What to build").
std::vector<ClipVertex> clipPolygonToHalfSpace(const std::vector<ClipVertex>& poly, double planeZ, bool keepLessEq) {
    if (poly.empty()) {
        return {};
    }
    const auto inside = [&](const Vec3& p) { return keepLessEq ? (p.z <= planeZ) : (p.z >= planeZ); };
    std::vector<ClipVertex> out;
    out.reserve(poly.size() + 1);
    for (size_t i = 0; i < poly.size(); ++i) {
        const ClipVertex& cur = poly[i];
        const ClipVertex& next = poly[(i + 1) % poly.size()];
        const bool curIn = inside(cur.pos);
        const bool nextIn = inside(next.pos);
        if (curIn) {
            out.push_back(cur);
        }
        if (curIn != nextIn) {
            const double t = (planeZ - cur.pos.z) / (next.pos.z - cur.pos.z);
            out.push_back(ClipVertex{cur.pos + (next.pos - cur.pos) * t});
        }
    }
    return out;
}

// Exact near/far clip of a camera-space triangle (SR-15), replacing the
// whole-triangle near/far rejection draw_world_triangle and
// draw_mesh_triangle_shaded used before this task. A triangle clipped
// against one plane yields a convex polygon of at most 4 vertices; against
// both, at most 5. Returns an empty vector if the triangle is entirely
// outside either half-space (the trivial-reject case still applies, it is
// simply now a side effect of the general clip rather than a separate
// up-front check). Order matters: near first, then far, matching
// draw_world_segment's own two clipSegmentToHalfSpace calls.
std::vector<ClipVertex> clipTriangleNearFar(const ViewContext& vc, const Vec3& ac, const Vec3& bc, const Vec3& cc) {
    std::vector<ClipVertex> poly{ClipVertex{ac}, ClipVertex{bc}, ClipVertex{cc}};
    poly = clipPolygonToHalfSpace(poly, -vc.nearP, /*keepLessEq=*/true);
    poly = clipPolygonToHalfSpace(poly, -vc.farP, /*keepLessEq=*/false);
    return poly;
}

// MN-14: the fixed opaque-byte value every BGRX8 pixel's 4th byte carries,
// unconditionally -- background clear and every draw call below write it
// identically, so no code path can leave it unset.
constexpr uint8_t kBgrxOpaqueByte = 0xFFu;

struct FrameBuffers {
    std::span<uint8_t> pixels;       // non-owning -- target.pixels is caller-owned (PA-1)
    std::vector<double>& depth;      // inverse view-space depth; 0 = infinitely far
    uint32_t width;
    uint32_t height;
};

void setPixelIfCloser(FrameBuffers& fb, int x, int y, double invDepth, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || y < 0 || x >= static_cast<int>(fb.width) || y >= static_cast<int>(fb.height)) {
        return;
    }
    const size_t idx = static_cast<size_t>(y) * fb.width + static_cast<size_t>(x);
    if (invDepth <= fb.depth[idx]) {
        return;  // farther than (or tied with) what's already there -- first writer at a tie wins
    }
    fb.depth[idx] = invDepth;
    uint8_t* px = &fb.pixels[idx * 4];
    px[0] = b;
    px[1] = g;
    px[2] = r;
    px[3] = kBgrxOpaqueByte;
}

// Simple parametric line walk (screen-space DDA) with linear invDepth
// interpolation.
void rasterizeLine(FrameBuffers& fb, const ScreenPoint& a, const ScreenPoint& b, uint8_t r, uint8_t g,
                    uint8_t bC) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const int steps = static_cast<int>(std::max(std::fabs(dx), std::fabs(dy)));
    if (steps == 0) {
        setPixelIfCloser(fb, static_cast<int>(std::floor(a.x)), static_cast<int>(std::floor(a.y)), a.invDepth, r, g,
                          bC);
        return;
    }
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double x = a.x + dx * t;
        const double y = a.y + dy * t;
        const double invD = a.invDepth + (b.invDepth - a.invDepth) * t;
        setPixelIfCloser(fb, static_cast<int>(std::floor(x)), static_cast<int>(std::floor(y)), invD, r, g, bC);
    }
}

double edgeFn(const ScreenPoint& a, const ScreenPoint& b, const ScreenPoint& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Flat-filled, depth-tested triangle over its screen-space bounding box.
// Fills regardless of winding (both (w0,w1,w2) all >=0 and all <=0 count as
// "inside") -- back-face culling, where it applies, is a decision made by
// the CALLER before reaching this function (draw_mesh_triangle_shaded,
// below), never inside it. That split is deliberate: this function is a
// direct, unchanged port of the wireframe rasterizer's own rasterizeTriangleFlat,
// which never culled either (its two winding-agnostic callers -- spawn/body
// markers -- never had a guaranteed winding to cull by).
void rasterizeTriangleFlat(FrameBuffers& fb, const ScreenPoint& v0, const ScreenPoint& v1, const ScreenPoint& v2,
                            uint8_t r, uint8_t g, uint8_t b) {
    const double area = edgeFn(v0, v1, v2);
    if (std::fabs(area) < 1e-9) {
        return;  // degenerate
    }
    const int x0 = std::max(0, static_cast<int>(std::floor(std::min({v0.x, v1.x, v2.x}))));
    const int x1 =
        std::min(static_cast<int>(fb.width) - 1, static_cast<int>(std::ceil(std::max({v0.x, v1.x, v2.x}))));
    const int y0 = std::max(0, static_cast<int>(std::floor(std::min({v0.y, v1.y, v2.y}))));
    const int y1 =
        std::min(static_cast<int>(fb.height) - 1, static_cast<int>(std::ceil(std::max({v0.y, v1.y, v2.y}))));
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const ScreenPoint p{x + 0.5, y + 0.5, 0.0};
            const double w0 = edgeFn(v1, v2, p);
            const double w1 = edgeFn(v2, v0, p);
            const double w2 = edgeFn(v0, v1, p);
            const bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) {
                continue;
            }
            const double b0 = w0 / area, b1 = w1 / area, b2 = w2 / area;
            const double invD = b0 * v0.invDepth + b1 * v1.invDepth + b2 * v2.invDepth;
            setPixelIfCloser(fb, x, y, invD, r, g, b);
        }
    }
}

// ---------------------------------------------------------------------------
// Colors -- background/grid/bounds/spawn/drone unchanged from the wireframe
// rasterizer (drone body marker = the editor's own accent color #7C93FF,
// global-constraints.md). No "gate" color here: gate/primitive proxies are
// no longer overlay wireframes in this task -- they are real MeshData
// geometry drawn by draw_mesh_item, shaded by their own material.
// ---------------------------------------------------------------------------

constexpr uint8_t kBackgroundR = 20, kBackgroundG = 18, kBackgroundB = 16;
constexpr uint8_t kGridR = 90, kGridG = 90, kGridB = 90;
constexpr uint8_t kBoundsR = 90, kBoundsG = 140, kBoundsB = 200;
constexpr uint8_t kSpawnR = 190, kSpawnG = 90, kSpawnB = 170;
constexpr uint8_t kDroneR = 124, kDroneG = 147, kDroneB = 255;  // #7C93FF

// ---------------------------------------------------------------------------
// Adaptation boundary -- the only place double Vec3/raw double[4] quaternions
// meet RenderScene's glm::vec3/glm::quat (float). Converting at this single
// seam (rather than threading glm types through the ported block above) is
// what keeps the ported functions a byte-for-byte match to their source.
// ---------------------------------------------------------------------------

[[nodiscard]] Vec3 vec3d(const glm::vec3& v) {
    return Vec3{static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)};
}

[[nodiscard]] uint8_t to_byte(float channel) {
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(clamped * 255.0f));
}

[[nodiscard]] ViewContext build_view_context(const Camera& camera, uint32_t width, uint32_t height) {
    const double q[4] = {static_cast<double>(camera.orientation.w), static_cast<double>(camera.orientation.x),
                          static_cast<double>(camera.orientation.y), static_cast<double>(camera.orientation.z)};
    double qConj[4];
    quatConjugate(q, qConj);
    return ViewContext{
        {static_cast<double>(camera.position.x), static_cast<double>(camera.position.y),
         static_cast<double>(camera.position.z)},
        {qConj[0], qConj[1], qConj[2], qConj[3]},
        static_cast<double>(camera.fov_y_radians),
        static_cast<double>(camera.near_plane),
        static_cast<double>(camera.far_plane),
        width,
        height,
    };
}

// ---------------------------------------------------------------------------
// World-space draw passes. draw_world_segment is a DIRECT, UNCHANGED port
// (renamed to this file's snake_case convention) of the wireframe
// rasterizer's own drawWorldSegment -- same near/far clip on lines, same
// "no winding check" fill.
//
// draw_world_triangle originally matched it structurally (a direct,
// unchanged port of drawWorldTriangle's own whole-triangle near/far
// REJECTION), but Task R5b (SR-15) replaced that rejection with exact
// clipTriangleNearFar()-based clipping in both this function and
// draw_mesh_triangle_shaded below: real solid geometry (Task R5) makes a
// camera approaching a wall reject that wall's triangles one by one as they
// straddle the near plane, rather than rendering the part that is still in
// view. draw_mesh_triangle_shaded is otherwise genuinely NEW relative to
// draw_world_triangle: it adds the SR-13 back-face cull, kept as a separate
// function rather than a flag on draw_world_triangle because markers (drawn
// by draw_world_triangle) are never guaranteed a consistent winding and
// must never be culled, per SR-13's own text ("wireframe mode does not
// [cull] -- both sides draw") and the fact this project's own spawn/body
// marker triangles predate any winding convention at all. Clipping and
// culling are orthogonal: this task changed the former in both functions
// and left the latter exactly as it was in each.
// ---------------------------------------------------------------------------

void draw_world_segment(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, uint8_t r, uint8_t g,
                         uint8_t bC) {
    Vec3 ac = worldToCameraSpace(vc, a);
    Vec3 bc = worldToCameraSpace(vc, b);
    if (!clipSegmentToHalfSpace(ac, bc, -vc.nearP, /*keepLessEq=*/true)) {
        return;
    }
    if (!clipSegmentToHalfSpace(ac, bc, -vc.farP, /*keepLessEq=*/false)) {
        return;
    }
    rasterizeLine(fb, projectCameraSpace(vc, ac), projectCameraSpace(vc, bc), r, g, bC);
}

// Exact near/far clip (Task R5b, SR-15), fan-triangulated
// ((v0,v1,v2), (v0,v2,v3), ... over the clipped polygon) -- replaces the
// whole-triangle rejection this function used before this task. Never
// culls: used only by the overlay markers below, whose winding is not a
// guaranteed fact.
void draw_world_triangle(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, const Vec3& c,
                          uint8_t r, uint8_t g, uint8_t bC) {
    const Vec3 ac = worldToCameraSpace(vc, a);
    const Vec3 bc = worldToCameraSpace(vc, b);
    const Vec3 cc = worldToCameraSpace(vc, c);
    const std::vector<ClipVertex> poly = clipTriangleNearFar(vc, ac, bc, cc);
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        rasterizeTriangleFlat(fb, projectCameraSpace(vc, poly[0].pos), projectCameraSpace(vc, poly[i].pos),
                               projectCameraSpace(vc, poly[i + 1].pos), r, g, bC);
    }
}

// NEW (relative to draw_world_triangle): the SR-13 back-face cull for
// DrawMode::shaded mesh geometry, applied AFTER the same exact near/far clip
// draw_world_triangle now uses (Task R5b, SR-15) -- clipping and culling are
// independent steps, in that order: clip the camera-space triangle first,
// fan-triangulate, project each output triangle, THEN cull it, because
// edgeFn's front/back sign convention (below) is only defined post-
// projection. Fan triangulation preserves the source triangle's facing, so
// every output triangle inherits the same cull decision the original,
// unclipped triangle would have gotten.
//
// Sign derivation: camera space is right-handed with forward = -Z (a point
// in front of the camera has pc.z < 0; projectCameraSpace's invNegZ =
// 1/-pc.z is positive there). A front-facing triangle -- outward normal
// pointing back toward the camera, i.e. CCW as seen FROM the camera in a
// Y-up frame -- has positive signed area under the standard 2D cross-product
// formula edgeFn computes, evaluated in that Y-up frame. projectCameraSpace
// maps camera-space (x, y) to screen space with x UNFLIPPED but y FLIPPED
// (NDC +Y is up; screen +Y is down, "the wireframe rasterizer's own
// documented convention"). Flipping exactly one axis of a 2D signed-area
// computation negates its sign, so a front-facing triangle's screen-space
// edgeFn(v0, v1, v2) comes out NEGATIVE, and a backward-wound one -- the
// exact defect Task R2's review found in four of seven tessellated
// primitives -- comes out POSITIVE. Culling on `> 0.0` is therefore what
// makes that winding fix a pixel-level observable rather than only a
// vertex-order assertion (test_render_raster.cpp's
// RasterCpu.ShadedModeRendersOutwardFacingTriangleButCullsReversedOne is the
// test that pins this sign, independent of this comment's algebra).
void draw_mesh_triangle_shaded(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, const Vec3& c,
                                uint8_t r, uint8_t g, uint8_t bC) {
    const Vec3 ac = worldToCameraSpace(vc, a);
    const Vec3 bc = worldToCameraSpace(vc, b);
    const Vec3 cc = worldToCameraSpace(vc, c);
    const std::vector<ClipVertex> poly = clipTriangleNearFar(vc, ac, bc, cc);
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        const ScreenPoint sa = projectCameraSpace(vc, poly[0].pos);
        const ScreenPoint sb = projectCameraSpace(vc, poly[i].pos);
        const ScreenPoint sc = projectCameraSpace(vc, poly[i + 1].pos);
        if (edgeFn(sa, sb, sc) > 0.0) {
            continue;  // SR-13: back face, shaded mode culls it.
        }
        rasterizeTriangleFlat(fb, sa, sb, sc, r, g, bC);
    }
}

// ---------------------------------------------------------------------------
// draw_mesh_item -- NEW. Draws one DrawItem's MeshData, submesh by submesh
// (SR-11 contract: an empty submesh triple means exactly one implicit
// submesh spanning the whole index buffer at material index 0), honouring
// DrawItem::material_override when set (it replaces the submesh's own
// material for every submesh in the item, per task-R6-brief.md's own
// "material lookup per submesh with DrawItem::material_override" framing).
// DrawMode::shaded fills each triangle with rasterizeTriangleFlat via
// draw_mesh_triangle_shaded (culled); DrawMode::wireframe draws each
// triangle's three edges via draw_world_segment (not culled, per SR-13) --
// the "old look", now applied to real tessellated geometry instead of hand-
// built edge lists.
//
// MeshData::normals is deliberately unread here: this task's shading is flat
// per-submesh material colour only (no lighting) -- Task R6 is where normals
// start mattering.
// ---------------------------------------------------------------------------

void draw_mesh_item(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene, const DrawItem& item,
                     DrawMode mode) {
    if (item.mesh_index >= scene.meshes.size()) {
        return;  // kNoMesh, or an out-of-range slot -- nothing to draw.
    }
    const MeshData& mesh = scene.meshes[item.mesh_index];
    if (mesh.indices.empty()) {
        return;  // an allocated-but-not-yet-tessellated placeholder (SR-9), or a genuinely empty mesh.
    }
    if (scene.materials.empty()) {
        return;  // no material to shade with at all -- nothing safe to fall back to (review finding).
    }

    // Review finding: scene.hpp's own MESH INDEX SPACE note says resolving a
    // world visual reference to real geometry (a MeshData's positions/
    // indices/submesh_* arrays) is the CALLER's job -- e.g. Task R4's glTF
    // loader, reading a file this module never validated. SR-11 declares
    // submesh_first_index/submesh_index_count/submesh_material parallel and
    // partitioning, but nothing upstream of this function enforces it, so
    // this loop trusts none of it: `submesh_count` is the SHORTEST of the
    // three arrays (never first_index.size() alone, which could outrun the
    // other two), every submesh's [first, first+count) range is checked
    // against indices.size() before use, and every triangle's three indices
    // are checked against positions.size() before being dereferenced. A
    // malformed or truncated file degrades to "some triangles/submeshes
    // silently skipped" -- never an out-of-bounds read.
    const bool has_submeshes = !mesh.submesh_first_index.empty();
    const size_t submesh_count =
        has_submeshes ? std::min({mesh.submesh_first_index.size(), mesh.submesh_index_count.size(),
                                   mesh.submesh_material.size()})
                      : size_t{1};

    for (size_t s = 0; s < submesh_count; ++s) {
        const uint32_t first = has_submeshes ? mesh.submesh_first_index[s] : 0u;
        const uint32_t count = has_submeshes ? mesh.submesh_index_count[s] : static_cast<uint32_t>(mesh.indices.size());
        if (first > mesh.indices.size() || count > mesh.indices.size() - first) {
            continue;  // this submesh's range runs past the index buffer -- skip it, not the whole mesh.
        }
        const uint32_t submesh_material_index = has_submeshes ? mesh.submesh_material[s] : 0u;
        const uint32_t material_index =
            item.material_override != kNoMaterial ? item.material_override : submesh_material_index;
        // Fallback to the default material (index 0) only now that
        // scene.materials is known non-empty (checked above) -- an untrusted
        // index must never fall back onto an equally untrusted one (review
        // finding).
        const Material& material =
            material_index < scene.materials.size() ? scene.materials[material_index] : scene.materials[0];

        const uint8_t r = to_byte(material.base_color.r);
        const uint8_t g = to_byte(material.base_color.g);
        const uint8_t b = to_byte(material.base_color.b);

        for (uint32_t t = 0; t + 3 <= count; t += 3) {
            const uint32_t ia = mesh.indices[first + t];
            const uint32_t ib = mesh.indices[first + t + 1];
            const uint32_t ic = mesh.indices[first + t + 2];
            if (ia >= mesh.positions.size() || ib >= mesh.positions.size() || ic >= mesh.positions.size()) {
                continue;  // an index points past the vertex buffer -- skip this triangle, not the mesh.
            }

            const glm::vec3 wa(item.local_to_world * glm::vec4(mesh.positions[ia], 1.0f));
            const glm::vec3 wb(item.local_to_world * glm::vec4(mesh.positions[ib], 1.0f));
            const glm::vec3 wc(item.local_to_world * glm::vec4(mesh.positions[ic], 1.0f));

            if (mode == DrawMode::wireframe) {
                draw_world_segment(fb, vc, vec3d(wa), vec3d(wb), r, g, b);
                draw_world_segment(fb, vc, vec3d(wb), vec3d(wc), r, g, b);
                draw_world_segment(fb, vc, vec3d(wc), vec3d(wa), r, g, b);
            } else {
                draw_mesh_triangle_shaded(fb, vc, vec3d(wa), vec3d(wb), vec3d(wc), r, g, b);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Overlay passes (RenderOptions::overlays, PA-4) -- ported from the
// wireframe rasterizer's drawGroundGrid/drawWorldBounds/drawSpawnMarkers/
// drawBodyMarkers, adapted to read already-computed RenderScene fields
// (ground_y/has_ground, bounds, spawn_positions/spawn_orientations) instead
// of re-deriving them: scene_from_world() (Task R1) already ported
// groundPlaneY()/computeWorldBounds() into scene.cpp, so this file does not
// duplicate that heuristic a second time.
// ---------------------------------------------------------------------------

constexpr double kGridHalfExtent = 10.0;
constexpr double kGridStep = 1.0;

void draw_ground_grid(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene) {
    // value_or(0.0f) equivalent -- the wireframe rasterizer's own
    // drawGroundGrid always drew a grid, defaulting to y=0 with no ground
    // plane found; reproduced unchanged.
    const double y = scene.has_ground ? static_cast<double>(scene.ground_y) : 0.0;
    for (double x = -kGridHalfExtent; x <= kGridHalfExtent + 1e-9; x += kGridStep) {
        draw_world_segment(fb, vc, Vec3{x, y, -kGridHalfExtent}, Vec3{x, y, kGridHalfExtent}, kGridR, kGridG, kGridB);
    }
    for (double z = -kGridHalfExtent; z <= kGridHalfExtent + 1e-9; z += kGridStep) {
        draw_world_segment(fb, vc, Vec3{-kGridHalfExtent, y, z}, Vec3{kGridHalfExtent, y, z}, kGridR, kGridG, kGridB);
    }
}

void draw_world_bounds(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene) {
    const Aabb& box = scene.bounds;
    const Vec3 c[8] = {
        {static_cast<double>(box.min.x), static_cast<double>(box.min.y), static_cast<double>(box.min.z)},
        {static_cast<double>(box.max.x), static_cast<double>(box.min.y), static_cast<double>(box.min.z)},
        {static_cast<double>(box.max.x), static_cast<double>(box.max.y), static_cast<double>(box.min.z)},
        {static_cast<double>(box.min.x), static_cast<double>(box.max.y), static_cast<double>(box.min.z)},
        {static_cast<double>(box.min.x), static_cast<double>(box.min.y), static_cast<double>(box.max.z)},
        {static_cast<double>(box.max.x), static_cast<double>(box.min.y), static_cast<double>(box.max.z)},
        {static_cast<double>(box.max.x), static_cast<double>(box.max.y), static_cast<double>(box.max.z)},
        {static_cast<double>(box.min.x), static_cast<double>(box.max.y), static_cast<double>(box.max.z)},
    };
    static constexpr int kEdges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
    for (const auto& e : kEdges) {
        draw_world_segment(fb, vc, c[e[0]], c[e[1]], kBoundsR, kBoundsG, kBoundsB);
    }
}

constexpr double kSpawnMarkerRadius = 0.3;
constexpr double kSpawnMarkerYOffset = 0.02;  // lifted slightly off the grid plane

void draw_spawn_markers(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene) {
    for (size_t i = 0; i < scene.spawn_positions.size(); ++i) {
        const Vec3 base = vec3d(scene.spawn_positions[i]);
        const glm::quat& orient = scene.spawn_orientations[i];
        const double q[4] = {static_cast<double>(orient.w), static_cast<double>(orient.x),
                              static_cast<double>(orient.y), static_cast<double>(orient.z)};
        const Vec3 v0{kSpawnMarkerRadius, kSpawnMarkerYOffset, 0.0};
        const Vec3 v1{0.0, kSpawnMarkerYOffset, kSpawnMarkerRadius};
        const Vec3 v2{-kSpawnMarkerRadius, kSpawnMarkerYOffset, 0.0};
        const Vec3 v3{0.0, kSpawnMarkerYOffset, -kSpawnMarkerRadius};
        const auto place = [&](const Vec3& local) { return base + rotateByQuat(q, local); };
        draw_world_triangle(fb, vc, place(v0), place(v1), place(v2), kSpawnR, kSpawnG, kSpawnB);
        draw_world_triangle(fb, vc, place(v0), place(v2), place(v3), kSpawnR, kSpawnG, kSpawnB);
    }
}

constexpr double kDroneNose = 0.5;

// scene.dynamics carries no separate BodyPose list of its own (RenderScene's
// own design, scene.hpp) -- each dynamic DrawItem's local_to_world already
// IS a body's pose (update_dynamics() builds it as rotation-then-translation
// from BodyPose, with no scale: bodies are rigid), so position/orientation
// are recovered from it directly rather than threading a second BodyPose
// span through render()'s own signature.
void draw_body_markers(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene) {
    const Vec3 nose{0.0, 0.0, -kDroneNose};
    const Vec3 left{-0.2, -0.08, 0.2};
    const Vec3 right{0.2, -0.08, 0.2};
    const Vec3 top{0.0, 0.25, 0.2};
    for (const DrawItem& item : scene.dynamics) {
        const glm::vec3 position(item.local_to_world[3]);
        const glm::quat orientation = glm::quat_cast(glm::mat3(item.local_to_world));
        const Vec3 base = vec3d(position);
        const double q[4] = {static_cast<double>(orientation.w), static_cast<double>(orientation.x),
                              static_cast<double>(orientation.y), static_cast<double>(orientation.z)};
        const auto place = [&](const Vec3& local) { return base + rotateByQuat(q, local); };
        draw_world_triangle(fb, vc, place(nose), place(left), place(right), kDroneR, kDroneG, kDroneB);
        draw_world_triangle(fb, vc, place(nose), place(right), place(top), kDroneR, kDroneG, kDroneB);
        draw_world_triangle(fb, vc, place(nose), place(top), place(left), kDroneR, kDroneG, kDroneB);
        draw_world_triangle(fb, vc, place(left), place(right), place(top), kDroneR, kDroneG, kDroneB);
    }
}

}  // namespace

Result<void> render(const RenderScene& scene, const Camera& camera, const RenderOptions& options,
                     RenderTarget& target) {
    if (Result<void> valid = validate_target(target); !valid) {
        return valid;
    }
    // raymarch is out of this task's scope entirely (Tasks R8/R9 own it,
    // scene.hpp's own "raymarch + agreement only" note on RenderScene::sdf)
    // -- reported rather than silently rendered as shaded or left blank.
    if (options.mode == DrawMode::raymarch) {
        return std::unexpected(
            Error{Code::invalid_argument, "raster_cpu::render: raymarch draw mode is not implemented until Task R8/R9"});
    }

    const uint32_t width = target.width;
    const uint32_t height = target.height;
    const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

    // MN-14 / constraint 2: every byte written every frame, X always 0xFF.
    for (size_t i = 0; i < pixel_count; ++i) {
        uint8_t* px = &target.pixels[i * 4];
        px[0] = kBackgroundB;
        px[1] = kBackgroundG;
        px[2] = kBackgroundR;
        px[3] = kBgrxOpaqueByte;
    }

    std::vector<double> depth(pixel_count, 0.0);  // 0 = infinitely far (ported convention)
    FrameBuffers fb{target.pixels, depth, width, height};
    const ViewContext vc = build_view_context(camera, width, height);

    // Fixed operation order (constraint 4): statics, then dynamics, then
    // overlays -- never based on hashing, pointer identity, or anything else
    // unordered.
    for (const DrawItem& item : scene.statics) {
        draw_mesh_item(fb, vc, scene, item, options.mode);
    }
    for (const DrawItem& item : scene.dynamics) {
        draw_mesh_item(fb, vc, scene, item, options.mode);
    }

    if (options.overlays) {
        draw_ground_grid(fb, vc, scene);
        draw_world_bounds(fb, vc, scene);
        draw_spawn_markers(fb, vc, scene);
        draw_body_markers(fb, vc, scene);
    }

    return {};
}

}  // namespace spade::render
