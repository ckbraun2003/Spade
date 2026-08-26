#include "render/raster_cpu.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/geometric.hpp>
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

// A clip vertex: camera-space position, plus a WORLD-SPACE normal (S7a Task
// R6, SR-18) interpolated at the SAME `t` the position is -- extending this
// payload, rather than forking clipPolygonToHalfSpace/clipTriangleNearFar
// into a second normal-aware copy, is exactly what Task R5b's own forward
// note (below, and this struct's prior comment) asked for.
//
// `normal` defaults to the zero vector so the overlay paths (draw_world_
// triangle/draw_world_segment, further below) -- which build a ClipVertex
// from position alone and never read `.normal` back -- stay exactly as
// terse as before; only draw_mesh_triangle_shaded's Gouraud path (SR-18)
// ever populates or reads it. Not normalized after a lerp (see
// shade_vertex_color's own comment on why that is fine): a mesh whose
// normal is genuinely constant across a triangle (a plane, SR-17's own
// load-bearing case) lerps to the SAME value bit-for-bit regardless.
struct ClipVertex {
    Vec3 pos;
    glm::vec3 normal{0.0f};
};

// Fixed capacity: a triangle clipped against one plane yields at most 4
// vertices, against both planes at most 5 (SR-15's own bound) -- 8 is ample
// headroom, chosen once and not revisited, rather than sized exactly to
// that proven bound.
constexpr size_t kMaxClipVertices = 8;
using ClipPoly = std::array<ClipVertex, kMaxClipVertices>;

// Sutherland-Hodgman: clips the convex polygon `in[0..inCount)` against the
// half-space `keepLessEq ? z <= planeZ : z >= planeZ`, writing the result
// into `out` (from index 0) and returning its vertex count (0 if the whole
// polygon fell outside). `in` and `out` must be different buffers -- this
// is one step of a ping-pong, never in-place. Reuses
// clipSegmentToHalfSpace's own interpolation formula --
// t = (planeZ - cur.z) / (next.z - cur.z), cur + (next - cur) * t -- rather
// than a second one (SR-15's "What to build").
//
// Allocation-free (IMPORTANT 2, R5b review round 1): this used to return a
// freshly heap-allocated std::vector per call -- two mallocs per triangle,
// including on the overwhelmingly common wholly-inside path, at
// kTessellationDefaults' 5k-20k triangles/frame that is 15k-60k malloc/free
// pairs a frame for no reason (clipTriangleNearFar's own fast path below
// skips this function entirely on that path now, but this function stays
// allocation-free regardless, for the straddling case that does reach it).
//
// Capacity guard (MINOR, R5b review round 2): outCount = #inside +
// #sign-changes is provably <= 6 of kMaxClipVertices=8 for this function's
// only two call sites (clipTriangleNearFar clips a 3- then a <=4-vertex
// polygon, per-plane sign changes around a cycle are always even, and a
// triangle can have at most 3), so this can never fire today -- but it is
// cheap insurance against a silent out-of-bounds write if Task R6 or a
// future third clip stage (frustum side planes) ever raises inCount without
// this bound being re-derived first.
size_t clipPolygonToHalfSpace(const ClipPoly& in, size_t inCount, double planeZ, bool keepLessEq, ClipPoly& out) {
    if (inCount == 0) {
        return 0;
    }
    const auto inside = [&](const Vec3& p) { return keepLessEq ? (p.z <= planeZ) : (p.z >= planeZ); };
    size_t outCount = 0;
    for (size_t i = 0; i < inCount; ++i) {
        const ClipVertex& cur = in[i];
        const ClipVertex& next = in[(i + 1) % inCount];
        const bool curIn = inside(cur.pos);
        const bool nextIn = inside(next.pos);
        if (curIn) {
            assert(outCount < kMaxClipVertices && "clipPolygonToHalfSpace: output polygon exceeded kMaxClipVertices");
            out[outCount++] = cur;
        }
        if (curIn != nextIn) {
            const double t = (planeZ - cur.pos.z) / (next.pos.z - cur.pos.z);
            assert(outCount < kMaxClipVertices && "clipPolygonToHalfSpace: output polygon exceeded kMaxClipVertices");
            // Normal lerped at the SAME t as position (SR-18, ClipVertex's
            // own comment) -- cast to float only for the vec3 multiply;
            // when cur.normal == next.normal bit-for-bit (a constant-normal
            // mesh, e.g. a plane), next.normal - cur.normal is exactly the
            // zero vector and this reduces to cur.normal exactly, regardless
            // of t's value or precision.
            out[outCount++] =
                ClipVertex{cur.pos + (next.pos - cur.pos) * t, cur.normal + (next.normal - cur.normal) * static_cast<float>(t)};
        }
    }
    return outCount;
}

// Exact near/far clip of a camera-space triangle (SR-15), replacing the
// whole-triangle near/far rejection draw_world_triangle and
// draw_mesh_triangle_shaded used before this task. Writes the clipped
// polygon into `outPoly` and returns its vertex count (0 if the triangle is
// entirely outside either half-space -- the trivial-reject case still
// applies on that path, it is simply now a side effect of the general clip
// rather than a separate up-front check).
//
// Takes ClipVertex, not bare Vec3, at this boundary too (not just inside
// clipPolygonToHalfSpace) so Task R6's normal field is a pure addition when
// it lands here -- no signature change needed at either level.
//
// Clip order is FIXED (near, then far) for reproducibility and to read the
// same way as draw_world_segment's own two clipSegmentToHalfSpace calls --
// not because the two half-space intersections are order-dependent. They
// are not: intersecting two half-spaces is commutative, and far-then-near
// was verified to produce an identical covered pixel set on every geometry
// this file's tests exercise (order is observable only at the last ULP, on
// an edge that crosses both planes, as pure floating-point evaluation-order
// noise -- never as a different real-valued clip result).
size_t clipTriangleNearFar(const ViewContext& vc, const ClipVertex& a, const ClipVertex& b, const ClipVertex& c,
                           ClipPoly& outPoly) {
    // Fast path (IMPORTANT 2): all three vertices already inside both
    // half-spaces -- the overwhelmingly common case at kTessellationDefaults
    // triangle counts. Skips the Sutherland-Hodgman machinery (and its two
    // ClipPoly buffer copies) entirely and hands the original triangle
    // through unchanged -- byte-identical to running it through the general
    // path below (nothing would cross either plane, so that path would
    // produce these same three vertices in this same order too) and
    // byte-identical to this function's own pre-clipping behavior (the
    // same three points reaching the same three projectCameraSpace calls
    // downstream).
    const auto inRange = [&](const Vec3& p) { return p.z <= -vc.nearP && p.z >= -vc.farP; };
    if (inRange(a.pos) && inRange(b.pos) && inRange(c.pos)) {
        outPoly[0] = a;
        outPoly[1] = b;
        outPoly[2] = c;
        return 3;
    }
    ClipPoly triangle{a, b, c};
    ClipPoly afterNear;
    const size_t nearCount = clipPolygonToHalfSpace(triangle, 3, -vc.nearP, /*keepLessEq=*/true, afterNear);
    return clipPolygonToHalfSpace(afterNear, nearCount, -vc.farP, /*keepLessEq=*/false, outPoly);
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
// Colors -- grid/bounds/spawn/drone unchanged from the wireframe rasterizer
// (drone body marker = the editor's own accent color #7C93FF,
// global-constraints.md). No "gate" color here: gate/primitive proxies are
// no longer overlay wireframes in this task -- they are real MeshData
// geometry drawn by draw_mesh_item, shaded by their own material. The old
// flat kBackgroundR/G/B constant is GONE (S7a Task R6): the background is now
// draw_sky_and_ground_background()'s vertical sky gradient + analytic ground,
// read from the scene's own Lighting rather than a fixed literal.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Shading (S7a Task R6, ruling SR-18) -- material lookup happens per submesh
// in draw_mesh_item (below); this is the pure function that turns a resolved
// Material, the scene's Lighting, and a WORLD-SPACE surface normal into a
// linear-light RGB colour. Called ONCE PER VERTEX by draw_mesh_triangle_
// shaded's Gouraud fill (SR-18: shade at the vertices, interpolate the
// resulting colour -- never interpolate the normal and shade per pixel) and
// ONCE PER BACKGROUND PIXEL by the analytic ground pass (SR-17) -- the SAME
// function both times, which is what makes the tessellated-grid/analytic-
// ground seam agree bit-for-bit whenever its three inputs do (see
// transform_normal()'s own comment, scene.hpp, for how those inputs are kept
// identical at that seam).
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
// not be unit: a near/far-clip-interpolated normal (ClipVertex, above) is a
// lerp of two unit vectors and is deliberately not re-normalized (that
// struct's own comment) -- a slightly-non-unit vector here is a tiny
// cosine-law approximation right at a clipped triangle's edge, never
// something this function needs to correct, and it never actually occurs at
// all for a constant-normal mesh (SR-17's own seam), whose normal survives
// any lerp bit-for-bit.
// Pins the magic 1u/2u literals below against world::MaterialShading's own
// values (world/builder.hpp) -- review MINOR 7: `unlit == 1` is already
// exercised end to end by RenderShading.UnlitMaterialIgnoresSunDirectionEntirely,
// but nothing previously pinned `emissive == 2` anywhere, so a future
// reordering of that enum would silently swap emissive's behaviour with
// lambert's and no test would catch it.
static_assert(static_cast<uint32_t>(spade::MaterialShading::unlit) == 1u,
              "shade_vertex_color's magic 1u must match MaterialShading::unlit");
static_assert(static_cast<uint32_t>(spade::MaterialShading::emissive) == 2u,
              "shade_vertex_color's magic 2u must match MaterialShading::emissive");

[[nodiscard]] glm::vec3 shade_vertex_color(const Material& material, const Lighting& lighting,
                                            const glm::vec3& n_world) {
    const glm::vec3 base(material.base_color);
    if (material.shading == 1u || material.shading == 2u) {  // unlit, emissive
        return base;
    }
    const float n_dot_l = std::max(glm::dot(n_world, lighting.sun_direction), 0.0f);
    const glm::vec3 lit = lighting.sun_color * (lighting.sun_intensity * n_dot_l) + lighting.ambient_color;
    return base * lit;
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
    const ClipVertex ac{worldToCameraSpace(vc, a)};
    const ClipVertex bc{worldToCameraSpace(vc, b)};
    const ClipVertex cc{worldToCameraSpace(vc, c)};
    ClipPoly poly;
    const size_t count = clipTriangleNearFar(vc, ac, bc, cc, poly);
    if (count < 3) {
        return;
    }
    // poly[0] is shared by every fan triangle -- project it once rather than
    // once per iteration (review nit, R5b round 1).
    const ScreenPoint s0 = projectCameraSpace(vc, poly[0].pos);
    for (size_t i = 1; i + 1 < count; ++i) {
        rasterizeTriangleFlat(fb, s0, projectCameraSpace(vc, poly[i].pos), projectCameraSpace(vc, poly[i + 1].pos), r,
                               g, bC);
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
//
// Gouraud-filled, depth-tested triangle (S7a Task R6, SR-18): barycentrically
// interpolates a per-vertex COLOUR (already shaded once at each vertex by
// the caller -- shade_vertex_color(), never re-shaded per pixel) the same
// way rasterizeTriangleFlat interpolates invDepth, converting to a byte
// triple once per covered pixel. A deliberate near-duplicate of
// rasterizeTriangleFlat's own bounding-box/edge-function walk (rather than a
// shared helper parameterized over "what to do per pixel") -- this file's
// own convention throughout is porting proven blocks unchanged rather than
// generalizing them, and this keeps rasterizeTriangleFlat itself untouched.
//
// EXACT equal-colour fast path (SR-17's own load-bearing seam,
// task-R6-brief.md): when all three vertex colours are BIT-IDENTICAL --
// always true for a constant-normal mesh (a plane, whose normal is the same
// at every vertex before or after any near/far-clip lerp, ClipVertex's own
// comment) -- this falls straight through to rasterizeTriangleFlat's single
// evaluation rather than the barycentric weighted sum below: b0+b1+b2 is not
// always EXACTLY 1.0 in floating point, so a weighted sum of three EQUAL
// inputs is not guaranteed to reproduce that exact input bit-for-bit, which
// is precisely what the analytic background ground pass's own single
// per-pixel evaluation needs to match. Converting this into a tolerance
// instead would silently give away the entire reason the hard-horizon
// design is safe (task-R6-brief.md's own words) -- so this is an exact `==`,
// not a "close enough".
void rasterizeTriangleGouraud(FrameBuffers& fb, const ScreenPoint& v0, const ScreenPoint& v1, const ScreenPoint& v2,
                               const glm::vec3& c0, const glm::vec3& c1, const glm::vec3& c2) {
    if (c0 == c1 && c1 == c2) {
        rasterizeTriangleFlat(fb, v0, v1, v2, to_byte(c0.r), to_byte(c0.g), to_byte(c0.b));
        return;
    }
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
            const glm::vec3 color =
                c0 * static_cast<float>(b0) + c1 * static_cast<float>(b1) + c2 * static_cast<float>(b2);
            setPixelIfCloser(fb, x, y, invD, to_byte(color.r), to_byte(color.g), to_byte(color.b));
        }
    }
}

// `na`/`nb`/`nc` are the triangle's three WORLD-SPACE vertex normals
// (draw_mesh_item's own transform_normal() call, per vertex) -- shaded HERE,
// once per polygon vertex AFTER clipping (SR-18: "shade at the vertices" means
// the final clipped polygon's vertices, original or clip-synthesized alike,
// not only the original three), then handed to rasterizeTriangleGouraud to
// interpolate the resulting COLOUR across the fan triangle's pixels.
void draw_mesh_triangle_shaded(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, const Vec3& c,
                                const glm::vec3& na, const glm::vec3& nb, const glm::vec3& nc,
                                const Material& material, const Lighting& lighting) {
    const ClipVertex ac{worldToCameraSpace(vc, a), na};
    const ClipVertex bc{worldToCameraSpace(vc, b), nb};
    const ClipVertex cc{worldToCameraSpace(vc, c), nc};
    ClipPoly poly;
    const size_t count = clipTriangleNearFar(vc, ac, bc, cc, poly);
    if (count < 3) {
        return;
    }
    // poly[0] is shared by every fan triangle -- project and shade it once
    // rather than once per iteration (review nit, R5b round 1, extended here
    // to the shaded colour too).
    const ScreenPoint sa = projectCameraSpace(vc, poly[0].pos);
    const glm::vec3 ca = shade_vertex_color(material, lighting, poly[0].normal);
    for (size_t i = 1; i + 1 < count; ++i) {
        const ScreenPoint sb = projectCameraSpace(vc, poly[i].pos);
        const ScreenPoint sc = projectCameraSpace(vc, poly[i + 1].pos);
        if (edgeFn(sa, sb, sc) > 0.0) {
            continue;  // SR-13: back face, shaded mode culls it.
        }
        const glm::vec3 cb = shade_vertex_color(material, lighting, poly[i].normal);
        const glm::vec3 cc2 = shade_vertex_color(material, lighting, poly[i + 1].normal);
        rasterizeTriangleGouraud(fb, sa, sb, sc, ca, cb, cc2);
    }
}

// ---------------------------------------------------------------------------
// draw_mesh_item -- NEW. Draws one DrawItem's MeshData, submesh by submesh
// (SR-11 contract: an empty submesh triple means exactly one implicit
// submesh spanning the whole index buffer at material index 0), honouring
// DrawItem::material_override when set (it replaces the submesh's own
// material for every submesh in the item, per task-R6-brief.md's own
// "material lookup per submesh with DrawItem::material_override" framing).
// DrawMode::shaded Gouraud-fills each triangle (SR-18: per-vertex normals,
// interpolated colour) via draw_mesh_triangle_shaded (culled); DrawMode::
// wireframe draws each triangle's three edges via draw_world_segment (not
// culled, per SR-13) using the submesh's flat, UNLIT base colour -- the "old
// look", deliberately left untouched by lighting (this renderer's debug/
// comparison view, raster_cpu.hpp's own header comment), now applied to real
// tessellated geometry instead of hand-built edge lists.
//
// MeshData::normals is read HERE now (S7a Task R6 -- this comment used to say
// "deliberately unread ... Task R6 is where normals start mattering"; this is
// that task): each triangle's three per-vertex LOCAL normals are transformed
// to WORLD space (transform_normal(), scene.hpp) and handed to
// draw_mesh_triangle_shaded for Gouraud shading in DrawMode::shaded. Missing
// or truncated normals (a malformed/truncated file -- this function trusts
// none of MeshData, per the review-finding note above) skip just that
// triangle in shaded mode, the same "some triangles silently skipped, never
// a crash" posture already applied to indices/submesh ranges; wireframe mode
// never needs a normal at all and is unaffected.
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

        // Wireframe's flat colour -- the submesh's raw base_color, never
        // relit (this function's own header comment above).
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
                const bool has_normals =
                    ia < mesh.normals.size() && ib < mesh.normals.size() && ic < mesh.normals.size();
                if (!has_normals) {
                    continue;  // malformed/truncated normals -- skip this triangle rather than shade it wrong.
                }
                const glm::vec3 na = transform_normal(item.local_to_world, mesh.normals[ia]);
                const glm::vec3 nb = transform_normal(item.local_to_world, mesh.normals[ib]);
                const glm::vec3 nc = transform_normal(item.local_to_world, mesh.normals[ic]);
                draw_mesh_triangle_shaded(fb, vc, vec3d(wa), vec3d(wb), vec3d(wc), na, nb, nc, material,
                                           scene.lighting);
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

// ---------------------------------------------------------------------------
// Background: vertical sky gradient + analytic ground (S7a Task R6, SR-17;
// gating below is SR-22). Runs FIRST, before any geometry (constraint 6's
// fixed operation order), filling literally every background pixel -- it is
// what MN-14's "every pixel written" now means, replacing the old flat clear
// entirely. Writes colour only, never depth (`depth` stays 0 = infinitely
// far everywhere it touches), so any real, depth-tested geometry drawn
// afterward -- including the OLDER, unrelated ground-grid overlay below --
// always wins the z-test over it, and it never reaches R7's scene.bounds-
// fitted shadow frustum.
//
// The sky gradient applies to EVERY draw mode; the analytic ground is
// DrawMode::shaded-only (ruling SR-22) -- see draw_sky_and_ground_background's
// own comment for why an unconditional ground is a mode-contract violation
// (a filled surface with no edges, drawn under DrawMode::wireframe) and a
// real SR-13 inversion risk (wireframe never culls, so a from-below view
// would show the tessellated grid's edges over nothing, or over a solid
// analytic fill an unconditional ground would incorrectly still withhold
// there anyway -- the mismatch is the bug, not any one frame of it).
// ---------------------------------------------------------------------------

// Reconstructs the camera-space, UN-normalized ray direction through pixel
// center (px+0.5, py+0.5) -- the exact inverse of projectCameraSpace's own
// perspective divide, evaluated at pc.z = -1 so the direction's scale is
// whatever falls out of that choice. Never normalized: the analytic ground
// pass below only ever compares a t computed from this ray AGAINST ANOTHER
// t computed from the SAME ray (nearest hit among several standalone ground
// planes) or tests its SIGN (front/back) -- both scale-invariant, so
// normalizing here would only add a sqrt this pass does not need.
//
// `f`/`aspect` are precomputed by the caller (review IMPORTANT 3, fix round
// 1): both are per-VIEW constants (same for every one of a frame's pixels),
// so computing them here -- one sin32+cos32+divide plus a divide, PER PIXEL
// -- was pure, measured waste (2.5-4.4x slower at 1280x720; hoisting alone
// took a ground-bearing frame from 77.6ms to 31.3ms). The same species of
// defect R5b's own review caught in this file (allocation-per-triangle).
[[nodiscard]] Vec3 background_ray_camera_space(double f, double aspect, uint32_t width, uint32_t height, uint32_t px,
                                                uint32_t py) {
    const double xNdc = 2.0 * (static_cast<double>(px) + 0.5) / static_cast<double>(width) - 1.0;
    const double yNdc = 1.0 - 2.0 * (static_cast<double>(py) + 0.5) / static_cast<double>(height);
    return Vec3{xNdc * aspect / f, yNdc / f, -1.0};
}

// `draw_analytic_ground` gates the INFINITE analytic ground to
// DrawMode::shaded only (ruling SR-22, review IMPORTANT 4) -- the sky
// gradient still fills every draw mode's background. An analytic ground is
// a shaded-SURFACE device (a filled region with no edges), so it has no
// wireframe equivalent by construction; leaving it on unconditionally made
// DrawMode::wireframe show a solid, LIT ground fill behind its own
// unlit, edges-only geometry (a mode contract violation on its own), and
// -- worse -- from below the tessellated grid's wireframe edges still draw
// (SR-13: wireframe never culls) while the analytic ground correctly does
// not (front-facing-only), an inverted-seam shape exactly like the one
// SR-17 exists to prevent in shaded mode. Task R8's raymarch path needs no
// such device at all: a `plane` SDF primitive is already infinite, so
// sphere-tracing it directly gives an infinite ground for free, confirming
// this is a raster-only expedient, not a real scene feature R9 should ever
// see duplicated.
void draw_sky_and_ground_background(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene,
                                     bool draw_analytic_ground) {
    // Precomputed ONCE PER FRAME (review IMPORTANT 3): whether the analytic
    // ground pass has anything at all to do. Skips not just the per-pixel
    // ray/plane loop but the per-plane `front[]` precompute below too, for
    // the overwhelmingly common "no standalone ground plane in this world"
    // and "wireframe mode" cases (measured: 70.8ms -> 15.9ms at 1280x720 for
    // a groundless world, on top of the f/aspect hoist above).
    const bool ground_possible = draw_analytic_ground && !scene.ground_planes.empty() && !scene.materials.empty();

    Vec3 camPos{0.0, 0.0, 0.0};
    double camQ[4] = {1.0, 0.0, 0.0, 0.0};
    double f = 1.0, aspect = 1.0;
    // uint8_t, not vector<bool> (review MINOR 9): this file already removed
    // one hidden-cost STL specialization (R5b's own per-triangle heap
    // allocation finding); vector<bool>'s bit-packed proxy-reference
    // specialization is the same species of surprise, avoided here even
    // though this vector is at most `scene.ground_planes.size()` long.
    std::vector<uint8_t> front;

    if (ground_possible) {
        camPos = Vec3{vc.camPos[0], vc.camPos[1], vc.camPos[2]};
        // Undoes ViewContext's own stored conjugate (world-to-camera
        // rotation) to recover the camera-to-world rotation this background
        // ray needs -- quatConjugate is its own inverse, so this is exact,
        // not an approximation.
        quatConjugate(vc.camOrientationConj, camQ);
        f = 1.0 / tan32(vc.fovY * 0.5);
        aspect = static_cast<double>(vc.width) / static_cast<double>(vc.height);

        // SR-13 parity, precomputed ONCE PER PLANE (a per-frame fact about
        // the camera and that plane, not a per-pixel one): "shade only when
        // the ray meets the plane's FRONT side" means the camera itself
        // must be strictly on the side the normal points to --
        // dot(camPos, normal) > offset -- exactly the outward-normal
        // convention SR-13's mesh back-face cull already uses (world/sdf.hpp's
        // own "dot(p,n) <= offset is solid"). A plane the camera is at or
        // below never contributes a hit, the same way the tessellated
        // ground disappears when viewed from below.
        front.assign(scene.ground_planes.size(), 0);
        for (size_t i = 0; i < scene.ground_planes.size(); ++i) {
            const GroundPlane& gp = scene.ground_planes[i];
            const double n_dot_cam = static_cast<double>(gp.normal.x) * camPos.x +
                                      static_cast<double>(gp.normal.y) * camPos.y +
                                      static_cast<double>(gp.normal.z) * camPos.z;
            front[i] = (n_dot_cam > static_cast<double>(gp.offset)) ? 1u : 0u;
        }
    }

    for (uint32_t y = 0; y < fb.height; ++y) {
        // Vertical sky gradient (Step 1's own requirement, applies to EVERY
        // draw mode, SR-22): a plain fraction of SCREEN ROW, zenith at row 0
        // to horizon at the last row. This is simple and deterministic --
        // NOT, as an earlier version of this comment incorrectly claimed,
        // because an elevation-based gradient would need an inverse-trig
        // call this engine's determinism contract forbids. It would not:
        // dir.y / length(dir) is monotone in elevation and needs only one
        // std::sqrt, which is IEEE-mandated (correctly rounded) and
        // explicitly sanctioned by that same contract. The real
        // consequence of the row-based choice (ruling SR-23, deferred to
        // CK-2 for the user to judge against real frames rather than have
        // this task guess): it anchors the horizon COLOUR to the bottom
        // screen row, so under camera pitch that colour does not coincide
        // with the ray-cast horizon LINE the ground hit-test below actually
        // draws (the hard edge is still exactly where the ground begins;
        // only the gradient's own colour-vs-row mapping is camera-pose-
        // agnostic). "Hard horizon, no fog" (SR-17) is enforced by the
        // GROUND hit-test below, never by this gradient.
        const double sky_t = fb.height > 1 ? static_cast<double>(y) / static_cast<double>(fb.height - 1) : 0.0;
        const float sky_tf = static_cast<float>(sky_t);
        const glm::vec3 sky = scene.lighting.sky_zenith * (1.0f - sky_tf) + scene.lighting.sky_horizon * sky_tf;

        for (uint32_t x = 0; x < fb.width; ++x) {
            glm::vec3 color = sky;

            if (ground_possible) {
                const Vec3 dirCam = background_ray_camera_space(f, aspect, vc.width, vc.height, x, y);
                const Vec3 dirWorld = rotateByQuat(camQ, dirCam);

                double best_t = 0.0;
                int64_t best_plane = -1;
                for (size_t i = 0; i < scene.ground_planes.size(); ++i) {
                    if (!front[i]) {
                        continue;  // camera at or below this plane -- SR-13 parity, never a hit.
                    }
                    const GroundPlane& gp = scene.ground_planes[i];
                    const double nx = static_cast<double>(gp.normal.x), ny = static_cast<double>(gp.normal.y),
                                 nz = static_cast<double>(gp.normal.z);
                    const double denom = nx * dirWorld.x + ny * dirWorld.y + nz * dirWorld.z;
                    if (denom >= 0.0) {
                        continue;  // ray moving away from (or parallel to) the plane's front -- no hit.
                    }
                    const double n_dot_cam = nx * camPos.x + ny * camPos.y + nz * camPos.z;
                    const double t = (static_cast<double>(gp.offset) - n_dot_cam) / denom;
                    // front[i] (n_dot_cam > offset) and denom < 0 together
                    // guarantee t > 0 algebraically -- a negative-over-
                    // negative division -- so this is a defensive restatement,
                    // not a live branch for any front-facing plane.
                    if (t > 0.0 && (best_plane < 0 || t < best_t)) {
                        best_t = t;
                        best_plane = static_cast<int64_t>(i);
                    }
                }

                if (best_plane >= 0) {
                    const GroundPlane& gp = scene.ground_planes[static_cast<size_t>(best_plane)];
                    const uint32_t material_index = gp.material < scene.materials.size() ? gp.material : 0u;
                    color = shade_vertex_color(scene.materials[material_index], scene.lighting, gp.normal);
                }
            }

            uint8_t* px = &fb.pixels[(static_cast<size_t>(y) * fb.width + x) * 4];
            px[0] = to_byte(color.b);
            px[1] = to_byte(color.g);
            px[2] = to_byte(color.r);
            px[3] = kBgrxOpaqueByte;
        }
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

    std::vector<double> depth(pixel_count, 0.0);  // 0 = infinitely far (ported convention)
    FrameBuffers fb{target.pixels, depth, width, height};
    const ViewContext vc = build_view_context(camera, width, height);

    // MN-14 / constraint 2: every byte written every frame, X always 0xFF --
    // now the sky gradient + analytic ground background pass (S7a Task R6),
    // not a flat clear: it still writes every pixel unconditionally, just no
    // longer the same colour everywhere. Writes colour only, no depth, so it
    // never survives the z-test against any real geometry drawn afterward.
    // The analytic ground itself is gated to DrawMode::shaded (ruling SR-22,
    // review IMPORTANT 4) -- the sky gradient applies to every mode.
    draw_sky_and_ground_background(fb, vc, scene, options.mode == DrawMode::shaded);

    // Fixed operation order (constraint 4): background, then statics, then
    // dynamics, then overlays -- never based on hashing, pointer identity, or
    // anything else unordered.
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
