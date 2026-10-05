#include "render/raster_cpu.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "render/raymarch.hpp"
#include "render/shadow.hpp"

namespace spade::render {

namespace {

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

// tan32() (S7a Task R6; RELOCATED to render/scene.hpp at Task R8 fix round 1,
// review Minor 3) -- the interim wireframe rasterizer this replaced called
// std::tan() directly, which was legal there (it lived outside this
// engine's determinism contract); this file lives under
// engine/render/, which IS scanned by
// tools/tests/test_m1b_bar.cpp's BitPortability.NoLibmTranscendentalInEngineSource,
// so it uses scene.hpp's shared, sin32/cos32-built tan32() instead -- now
// render/raymarch.cpp's identical camera-ray need reads the SAME function
// rather than a second copy that could disagree with this one by a ulp.
//
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
//
// `world` (S7a Task R7, SR-24 -- a second field added to this struct, same
// posture as R6's own `normal` addition, and explicitly permitted by the
// R5b clipper bound comment below: adding a FIELD is safe, adding a third
// clip STAGE is not): the WORLD-space position, lerped at the SAME `t` as
// `pos`/`normal`. Feeds sample_shadow() -- but never directly: SR-24
// requires the shadow lookup to run PER PIXEL against a PERSPECTIVE-CORRECT
// world position, and this field only ever carries a per-VERTEX value.
// rasterizeTriangleGouraud (below) is what turns three of these into one
// perspective-correct per-pixel position, via invDepth-weighted
// interpolation -- never a plain barycentric lerp of the three `world`
// values directly, which would be screen-space-affine and visibly wrong
// across a large triangle. Defaults to the zero vector like `normal` --
// only draw_mesh_triangle_shaded's Gouraud path ever populates or reads it.
struct ClipVertex {
    Vec3 pos;
    glm::vec3 normal{0.0f};
    glm::vec3 world{0.0f};
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
            // Normal AND world position lerped at the SAME t as position
            // (SR-18/SR-24, ClipVertex's own comment) -- cast to float only
            // for the vec3 multiply; when cur.normal == next.normal
            // bit-for-bit (a constant-normal mesh, e.g. a plane),
            // next.normal - cur.normal is exactly the zero vector and this
            // reduces to cur.normal exactly, regardless of t's value or
            // precision. `world` has no such constant-value case in general
            // (two distinct clip vertices almost always have distinct world
            // positions), so no analogous exactness claim is made for it --
            // it is fed to sample_shadow() downstream, not compared for
            // equality anywhere.
            const float tf = static_cast<float>(t);
            out[outCount++] = ClipVertex{cur.pos + (next.pos - cur.pos) * t, cur.normal + (next.normal - cur.normal) * tf,
                                          cur.world + (next.world - cur.world) * tf};
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

// The inverse of vec3d() above -- draw_mesh_triangle_shaded (S7a Task R7)
// needs a WORLD-space `a`/`b`/`c` (its own Vec3 parameters, already widened
// to double via vec3d() at its one call site, draw_mesh_item) back as a
// glm::vec3 to populate ClipVertex::world. Exact, not merely approximate:
// float -> double widening never loses a bit, and every one of these Vec3
// values originated from a glm::vec3 (float) that fits exactly in a double,
// so narrowing back to float recovers the identical bit pattern -- no
// precision lost on the round trip.
[[nodiscard]] glm::vec3 vec3f(const Vec3& v) {
    return glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

// to_byte() (S7a Task R6; RELOCATED to render/scene.hpp at Task R8 fix
// round 1, review Minor 3): the byte quantizer every draw call below still
// calls exactly as before -- now the same function render/raymarch.cpp
// calls too, since R9 compares the bytes it produces.

// ---------------------------------------------------------------------------
// Shading (S7a Task R6, ruling SR-18) -- ShadedColor/shade_vertex_color()
// RELOCATED to render/scene.hpp at Task R8, alongside transform_normal(),
// because Task R8's raymarcher became a THIRD call site needing the exact
// same function (see scene.hpp's own comment there for the full "why shared,
// not duplicated" reasoning). Everything below still calls it exactly as
// before -- only the definition's address changed, not its behaviour.
//
// Applies a (binary, {0,1}) shadow factor per SR-25: ambient survives
// untouched, the sun term is scaled. When `lit == 1.0f` (unshadowed, or no
// shadow map at all) this reduces to `sc.combined - sc.sun * 0.0f ==
// sc.combined` EXACTLY (IEEE 754: multiplying by exactly 0.0 and
// subtracting exactly 0.0 are both exact) -- so every unshadowed pixel,
// mesh or analytic ground alike, is bit-identical to what shade_vertex_
// color's own `combined` field would have given directly, which is what
// keeps the R6 SR-17 bit-identity seam intact through this fix too.
[[nodiscard]] glm::vec3 apply_shadow(const ShadedColor& sc, float lit) { return sc.combined - sc.sun * (1.0f - lit); }

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

// `depth_bias` (S7a Task R7, ruling SR-21 -- the R3-opened, R6-declined
// carried ticket): added ONLY to bias overlay draws toward the camera; see
// kOverlayDepthBias's own comment further below for the full story. Defaults
// to 0.0, so draw_mesh_item's WIREFRAME-mode mesh-edge calls to this
// function (real geometry, never an overlay) are BYTE-IDENTICAL to before
// this task -- only the four true overlay call sites (draw_ground_grid,
// draw_world_bounds, further below) opt in explicitly.
void draw_world_segment(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, uint8_t r, uint8_t g,
                         uint8_t bC, double depth_bias = 0.0) {
    Vec3 ac = worldToCameraSpace(vc, a);
    Vec3 bc = worldToCameraSpace(vc, b);
    if (!clipSegmentToHalfSpace(ac, bc, -vc.nearP, /*keepLessEq=*/true)) {
        return;
    }
    if (!clipSegmentToHalfSpace(ac, bc, -vc.farP, /*keepLessEq=*/false)) {
        return;
    }
    ScreenPoint sa = projectCameraSpace(vc, ac);
    ScreenPoint sb = projectCameraSpace(vc, bc);
    sa.invDepth += depth_bias;
    sb.invDepth += depth_bias;
    rasterizeLine(fb, sa, sb, r, g, bC);
}

// Exact near/far clip (Task R5b, SR-15), fan-triangulated
// ((v0,v1,v2), (v0,v2,v3), ... over the clipped polygon) -- replaces the
// whole-triangle rejection this function used before this task. Never
// culls: used only by the overlay markers below, whose winding is not a
// guaranteed fact.
//
// `depth_bias` (S7a Task R7, ruling SR-21): same overlay-toward-camera bias
// as draw_world_segment's own identical parameter, added at every fan
// triangle's three projected points -- defaults to 0.0 (this function's own
// only callers, draw_spawn_markers/draw_body_markers further below, both
// opt in explicitly; there is no wireframe-mesh-edge caller of this
// function to stay byte-identical for, but the default keeps this
// function's own signature consistent with draw_world_segment's).
void draw_world_triangle(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, const Vec3& c,
                          uint8_t r, uint8_t g, uint8_t bC, double depth_bias = 0.0) {
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
    ScreenPoint s0 = projectCameraSpace(vc, poly[0].pos);
    s0.invDepth += depth_bias;
    for (size_t i = 1; i + 1 < count; ++i) {
        ScreenPoint si = projectCameraSpace(vc, poly[i].pos);
        ScreenPoint sj = projectCameraSpace(vc, poly[i + 1].pos);
        si.invDepth += depth_bias;
        sj.invDepth += depth_bias;
        rasterizeTriangleFlat(fb, s0, si, sj, r, g, bC);
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
// comment) -- this uses that single colour directly rather than the
// barycentric weighted sum below: b0+b1+b2 is not always EXACTLY 1.0 in
// floating point, so a weighted sum of three EQUAL inputs is not guaranteed
// to reproduce that exact input bit-for-bit, which is precisely what the
// analytic background ground pass's own single per-pixel evaluation needs
// to match. Converting this into a tolerance instead would silently give
// away the entire reason the hard-horizon design is safe (task-R6-brief.md's
// own words) -- so this is an exact `==`, not a "close enough".
//
// `shadow` (S7a Task R7, ruling SR-25 -- fix round 1): when non-null, this
// function ALWAYS runs its own per-pixel loop -- even in the equal-colour
// case above -- so the shadow term can be sampled PER PIXEL, never per
// vertex. The equal-colour optimisation survives in a WEAKENED form: the
// constant `combined0`/`sun0` are still used as-is (no barycentric
// recomputation, so the R6 seam above still holds for the COLOUR term), but
// each covered pixel still gets its own perspective-correct world position
// and its own sample_shadow() call, applied via apply_shadow() (SR-25:
// ambient untouched, only the sun term scaled) afterward. When a pixel's
// shadow factor is exactly 1.0f (unshadowed -- the common case, including
// every pixel outside the shadow map's own footprint, SR-17),
// `apply_shadow(sc, 1.0f) == sc.combined` bit-for-bit (IEEE 754: `x - y*0.0f
// == x` exactly), so the R6 seam's bit-identity with the analytic ground
// pass (which now ALSO samples the SAME shadow map at its own exact
// per-pixel hit point, draw_sky_and_ground_background below) survives
// through the shadow application too, not just through the colour term
// alone. `shadow == nullptr` (shadows off, or no static geometry to have
// built a map from) reproduces this function's own pre-Task-R7 behaviour
// exactly, byte for byte -- the reason none of this program's original
// three goldens moved from the shadow feature alone (see this task's own
// report).
// SR-17a's atmospheric term, distilled to EXACTLY what a fill needs -- the
// same house rule draw_sky_and_ground_background follows (a drawing function
// takes what it draws, never the whole RenderOptions). Four values that are
// meaningless apart travel as ONE thing so none can be passed without the
// others, and so a future third parameter does not become a fourth positional
// float next to two other floats of the same type.
//
// ⚠⚠ PER PIXEL, NEVER PER VERTEX, and this is a correctness requirement rather
// than a quality preference. The analytic ground applies this term per pixel
// (draw_sky_and_ground_background). A Gouraud fill shades per VERTEX (SR-18)
// and interpolates, so folding the term into shade_vertex_color would have the
// two sides of SR-17's own tessellated/analytic seam computing it at different
// RATES on the same ground -- a seam artifact that grows with triangle size and
// is invisible on the small triangles any fixture is likely to use.
struct AtmosphereContext {
    glm::vec3 eye{0.0f};                 // world-space camera position
    const Lighting* lighting = nullptr;  // for sky_gradient_color along THIS pixel's ray
    float strength = 0.0f;
    float onset = 0.0f;
    // strength 0 is the documented OFF state and horizon_blend() returns the
    // surface colour exactly there, so `active()` is an optimisation AND the
    // guard that keeps the equal-colour fast path below reachable.
    [[nodiscard]] bool active() const { return lighting != nullptr && strength > 0.0f && onset > 0.0f; }
};

void rasterizeTriangleGouraud(FrameBuffers& fb, const ScreenPoint& v0, const ScreenPoint& v1, const ScreenPoint& v2,
                               const glm::vec3& combined0, const glm::vec3& combined1, const glm::vec3& combined2,
                               const glm::vec3& sun0, const glm::vec3& sun1, const glm::vec3& sun2,
                               const glm::vec3& wpos0, const glm::vec3& wpos1, const glm::vec3& wpos2,
                               const ShadowMap* shadow, const AtmosphereContext& atmo) {
    const bool equal_combined = (combined0 == combined1 && combined1 == combined2);
    // ⚠⚠ `!atmo.active()` IS LOAD-BEARING, NOT DEFENSIVE. The flat fast path
    // paints ONE colour over the whole triangle, and it is only equivalent to
    // the Gouraud path because shading had no position dependence -- the exact
    // assumption SR-17a breaks. With the term on, a distant vertex and a near
    // one no longer share a colour, so taking this path would silently drop the
    // term across every constant-normal triangle (which is most of them: a
    // plane, a box face, a wall). It would look like the feature simply did not
    // work on flat surfaces, with nothing red.
    if (equal_combined && shadow == nullptr && !atmo.active()) {
        rasterizeTriangleFlat(fb, v0, v1, v2, to_byte(combined0.r), to_byte(combined0.g), to_byte(combined0.b));
        return;
    }
    // ambient_color is a scene-wide constant that never depends on the
    // per-vertex normal (shade_vertex_color's own formula), so for a SINGLE
    // material/triangle `combined_i - sun_i` (== `base * ambient_color`) is
    // vertex-invariant even when `sun_i` itself varies with N.L -- meaning
    // `equal_combined` and "sun is vertex-invariant" are the SAME fact.
    // Checked independently anyway (rather than relying on that derivation)
    // so this code stays correct even if a future change ever makes ambient
    // vary per vertex.
    const bool equal_sun = (sun0 == sun1 && sun1 == sun2);
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
            const glm::vec3 combined = equal_combined
                                            ? combined0
                                            : (combined0 * static_cast<float>(b0) + combined1 * static_cast<float>(b1) +
                                               combined2 * static_cast<float>(b2));
            glm::vec3 color = combined;
            // ONE perspective-correct world position, shared by the shadow
            // lookup (SR-24) and the atmospheric term (SR-17a). Computed once
            // when either needs it: two independent recoveries of the same
            // quantity is the shape SR-17 clause 4 exists to forbid, and it
            // would also double the per-pixel cost of the common case.
            glm::vec3 world(0.0f);
            if (shadow != nullptr || atmo.active()) {
                // SR-24: perspective-correct world position -- interpolate
                // world*invDepth AFFINELY (the SAME b0/b1/b2 screen-space
                // weights invD itself uses) and divide by the ALREADY
                // perspective-correct invD, rather than a plain barycentric
                // lerp of wpos0/wpos1/wpos2 (which would be screen-space
                // affine and visibly wrong across a large triangle -- this
                // task's own brief: "the ground grid is exactly that case").
                // invD > 0 always holds here: every one of v0/v1/v2.invDepth
                // is itself > 0 (both are post-near/far-clip camera-space
                // points, clipTriangleNearFar's own contract), and
                // b0+b1+b2 == 1 with each bi in [0, 1] inside the triangle,
                // so invD is a convex combination of positive numbers.
                const glm::vec3 world_over_depth = wpos0 * static_cast<float>(b0 * v0.invDepth) +
                                                    wpos1 * static_cast<float>(b1 * v1.invDepth) +
                                                    wpos2 * static_cast<float>(b2 * v2.invDepth);
                world = world_over_depth / static_cast<float>(invD);
            }
            if (shadow != nullptr) {
                const float lit = sample_shadow(*shadow, world);
                const glm::vec3 sun = equal_sun ? sun0
                                                 : (sun0 * static_cast<float>(b0) + sun1 * static_cast<float>(b1) +
                                                    sun2 * static_cast<float>(b2));
                // SR-25: ambient (already folded into `combined`) is never
                // shadowed -- only the sun term is attenuated.
                color = apply_shadow(ShadedColor{combined, sun}, lit);
            }
            if (atmo.active()) {
                // SR-17a: LAST, after shadowing. The atmosphere sits between
                // the eye and the surface, so it acts on whatever colour the
                // surface finally has -- shadowed or lit. Doing it before the
                // shadow would attenuate the haze by the shadow term, which is
                // backwards: a shadow darkens a surface, not the air in front
                // of it.
                //
                // The SAME scene.hpp functions the background pass and raymarch
                // call, on the ray THIS pixel already determined. Note the ray
                // is `world - eye` rather than a reconstructed NDC direction:
                // sky_gradient_color normalises internally, and clause 4's
                // bit-identity seam is the NORMAL transform (scene.hpp:85), not
                // position -- the tree already tolerance-compares the two ways
                // of recovering a ground pixel's world position
                // (test_render_shadow.cpp's own 1% fringe allowance).
                const glm::vec3 eye_to_surface = world - atmo.eye;
                const float view_distance = std::sqrt(glm::dot(eye_to_surface, eye_to_surface));
                const glm::vec3 sky = sky_gradient_color(*atmo.lighting, eye_to_surface);
                color = horizon_blend(color, sky, view_distance, atmo.onset, atmo.strength);
            }
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
//
// `shadow` (S7a Task R7): forwarded to rasterizeTriangleGouraud unchanged --
// this function's own job is only to populate ClipVertex::world (from `a`/
// `b`/`c`, already WORLD-space -- draw_mesh_item's own call site passes
// vec3d(wa)/vec3d(wb)/vec3d(wc)) so the per-PIXEL shadow sampling downstream
// has real per-vertex world positions to interpolate, per SR-24. Null when
// shadows are off or there is no static shadow map to sample.
void draw_mesh_triangle_shaded(FrameBuffers& fb, const ViewContext& vc, const Vec3& a, const Vec3& b, const Vec3& c,
                                const glm::vec3& na, const glm::vec3& nb, const glm::vec3& nc,
                                const Material& material, const Lighting& lighting, const ShadowMap* shadow,
                                const AtmosphereContext& atmo) {
    const ClipVertex ac{worldToCameraSpace(vc, a), na, vec3f(a)};
    const ClipVertex bc{worldToCameraSpace(vc, b), nb, vec3f(b)};
    const ClipVertex cc{worldToCameraSpace(vc, c), nc, vec3f(c)};
    ClipPoly poly;
    const size_t count = clipTriangleNearFar(vc, ac, bc, cc, poly);
    if (count < 3) {
        return;
    }
    // poly[0] is shared by every fan triangle -- project and shade it once
    // rather than once per iteration (review nit, R5b round 1, extended here
    // to the shaded colour too).
    const ScreenPoint sa = projectCameraSpace(vc, poly[0].pos);
    const ShadedColor sca = shade_vertex_color(material, lighting, poly[0].normal);
    for (size_t i = 1; i + 1 < count; ++i) {
        const ScreenPoint sb = projectCameraSpace(vc, poly[i].pos);
        const ScreenPoint sc = projectCameraSpace(vc, poly[i + 1].pos);
        if (edgeFn(sa, sb, sc) > 0.0) {
            continue;  // SR-13: back face, shaded mode culls it.
        }
        const ShadedColor scb = shade_vertex_color(material, lighting, poly[i].normal);
        const ShadedColor scc = shade_vertex_color(material, lighting, poly[i + 1].normal);
        rasterizeTriangleGouraud(fb, sa, sb, sc, sca.combined, scb.combined, scc.combined, sca.sun, scb.sun, scc.sun,
                                  poly[0].world, poly[i].world, poly[i + 1].world, shadow, atmo);
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

// DrawMode::velocity's ramp (SL9c): blue at rest, red at options.velocity_-
// scale_mps, CLAMPED above it rather than wrapped -- a fast outlier must read
// as "at least this fast", never loop back through the slow colours and look
// stationary. A display normalisation only; nothing here reaches physics.
[[nodiscard]] glm::vec3 velocity_ramp(float speed_mps, float scale_mps) noexcept {
    // A non-positive scale would divide by zero or invert the ramp; treat it as
    // "everything is at the top" rather than producing NaN colours.
    if (!(scale_mps > 0.0f)) return glm::vec3(1.0f, 0.0f, 0.0f);
    const float u = std::clamp(speed_mps / scale_mps, 0.0f, 1.0f);
    return glm::vec3(u, 0.0f, 1.0f - u);
}

void draw_mesh_item(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene, const DrawItem& item,
                     const RenderOptions& options, const ShadowMap* shadow) {
    const DrawMode mode = options.mode;
    if (item.mesh_index >= scene.meshes.size()) {
        return;  // kNoMesh, or an out-of-range slot -- nothing to draw.
    }
    // SR-17a, distilled ONCE per item rather than per triangle or per pixel.
    // SHADED-ONLY, the same gate ruling SR-22 puts on the analytic ground and
    // the infinite grid: hazing a wireframe's edges would be a mode-contract
    // violation, and there is no surface there for atmosphere to sit in front
    // of. `mode` is read here, so a future mode inherits the gate rather than
    // the term.
    const AtmosphereContext atmo{
        .eye = glm::vec3(static_cast<float>(vc.camPos[0]), static_cast<float>(vc.camPos[1]),
                         static_cast<float>(vc.camPos[2])),
        .lighting = &scene.lighting,
        .strength = (mode == DrawMode::shaded) ? options.horizon_blend_strength : 0.0f,
        .onset = options.horizon_blend_onset,
    };
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
        const Material& resolved =
            material_index < scene.materials.size() ? scene.materials[material_index] : scene.materials[0];

        // DrawMode::velocity keeps the geometry and the lighting and replaces
        // only the base colour, so the frame still reads as a lit 3D scene
        // whose hue encodes speed -- rather than a flat silhouette that loses
        // the shape the speed belongs to. Everything else about the pipeline is
        // untouched, which is what keeps this mode from perturbing the others.
        Material velocity_tinted;
        if (mode == DrawMode::velocity) {
            velocity_tinted = resolved;
            velocity_tinted.base_color =
                glm::vec4(velocity_ramp(item.speed_mps, options.velocity_scale_mps), 1.0f);
        }
        const Material& material = (mode == DrawMode::velocity) ? velocity_tinted : resolved;

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
                                           scene.lighting, shadow, atmo);
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

// Overlay depth bias (S7a Task R7, ruling SR-21 -- the ticket carried since
// Task R3, re-affirmed at Task R6, landed here because R7 is the remaining
// Phase-2 task that edits depth handling in this file). Pulls every OVERLAY
// draw (ground grid, world bounds, spawn/body markers, all four below) this
// many invDepth units toward the camera before it reaches setPixelIfCloser's
// depth test, so a genuine near-tie against the tessellated surface an
// overlay annotates is decided by INTENT -- the overlay should always read
// as drawn ON TOP of the surface it marks -- rather than by
// setPixelIfCloser's own "first writer wins" tie rule. That rule previously
// meant the MESH always won: overlays draw LAST in render()'s fixed order
// (statics, dynamics, THEN overlays), so at an exact or near-exact tie the
// mesh (drawn first) kept its pixel and the overlay silently vanished --
// e.g. the ground-grid overlay's y=0 line disappearing wherever a box's
// bottom face rests exactly at y=0, the SAME scenario this ticket's own
// text describes. Since overlays draw last, this bias can only ever affect
// an overlay-vs-already-drawn-geometry comparison, never the reverse.
//
// TUNED (not derived), and a DIFFERENT constant from shadow.cpp's
// kShadowDepthBias -- different units (an invDepth epsilon here, a
// world-space light-axis distance there), different failure mode (an
// overlay disappearing behind the surface it annotates, vs shadow acne),
// different provenance (this ticket, ruling SR-21, vs Step 2's shadow
// acne/peter-panning tuning). Conflating the two would be a defect even if
// the numbers happened to coincide (this task's own controller amendment).
// Chosen large enough to clear the floating-point evaluation-order noise
// between two DIFFERENT interpolation paths computing the SAME world point
// (a Gouraud-shaded mesh vertex vs. a grid line's own per-endpoint DDA,
// test_render_raster.cpp's own OverlayDepthBias.* tests measure that noise
// at several orders of magnitude below this value) while staying far too
// small to visibly displace an overlay line that is NOT at a real tie.
//
// KNOWN, DOCUMENTED (not fixed) LIMITATION (M5, R7 fix round 1): this is an
// ABSOLUTE bias on invDepth (~1/distance), so the EQUIVALENT world-space
// displacement it can win a depth tie by GROWS WITH THE SQUARE of distance,
// not linearly. Deriving it: biased invDepth = 1/d + eps has an equivalent
// distance d' = d / (1 + eps*d), so the displacement is
// Delta(d) = d - d' = eps*d^2 / (1 + eps*d) ~= eps*d^2 for eps*d << 1.
// Recomputed directly (not assumed) at eps = 1e-4: Delta(20m) ~= 4.0 cm,
// Delta(100m) ~= 99 cm (~1 m), Delta(200m) ~= 3.9 m. Harmless for this
// program's own OverlayDepthBias.* regression fixture (a ground grid within
// a few metres of the camera), but the world-bounds box and the spawn/
// body markers all follow scene.bounds and CAN be meaningfully far from the
// camera in a large world -- at ~100+ m this bias could plausibly let an
// overlay win a depth tie against real geometry that is genuinely closer by
// up to about a metre, a real (if rare) mis-ordering, not merely a
// theoretical one. NOT fixed here: a distance-proportional (relative, e.g.
// `invDepth *= 1 + k`) bias would bound this linearly instead of
// quadratically and is the natural next step, but retuning it needs its own
// empirical verification pass (this file's own "recompute, don't assume"
// discipline) rather than a hasty substitution alongside this round's other
// changes -- flagged for a follow-up rather than guessed at here.
//
// kOverlayDepthBias itself is declared in raster_cpu.hpp, so GL applies the
// same value (render_gl/gl_renderer.cpp's overlay pass).
//
// The four builders below produce overlay_geometry()'s lists in draw order.
// They compute every point in double exactly as the old draw_* functions
// did, so the CPU's overlay pixels are unchanged.

[[nodiscard]] glm::dvec3 dvec(const Vec3& v) { return glm::dvec3(v.x, v.y, v.z); }
[[nodiscard]] Vec3 from_dvec(const glm::dvec3& v) { return Vec3{v.x, v.y, v.z}; }

void append_ground_grid(OverlayGeometry& out, const RenderScene& scene) {
    // value_or(0.0f) equivalent -- the wireframe rasterizer's own
    // drawGroundGrid always drew a grid, defaulting to y=0 with no ground
    // plane found; reproduced unchanged.
    const double y = scene.has_ground ? static_cast<double>(scene.ground_y) : 0.0;
    const std::array<uint8_t, 3> rgb{kGridR, kGridG, kGridB};
    for (double x = -kGridHalfExtent; x <= kGridHalfExtent + 1e-9; x += kGridStep) {
        out.lines.push_back(OverlayLine{.a = {x, y, -kGridHalfExtent}, .b = {x, y, kGridHalfExtent}, .rgb = rgb});
    }
    for (double z = -kGridHalfExtent; z <= kGridHalfExtent + 1e-9; z += kGridStep) {
        out.lines.push_back(OverlayLine{.a = {-kGridHalfExtent, y, z}, .b = {kGridHalfExtent, y, z}, .rgb = rgb});
    }
}

void append_world_bounds(OverlayGeometry& out, const RenderScene& scene) {
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
        out.lines.push_back(OverlayLine{.a = dvec(c[e[0]]), .b = dvec(c[e[1]]), .rgb = {kBoundsR, kBoundsG, kBoundsB}});
    }
}

constexpr double kSpawnMarkerRadius = 0.3;
constexpr double kSpawnMarkerYOffset = 0.02;  // lifted slightly off the grid plane

void append_spawn_markers(OverlayGeometry& out, const RenderScene& scene) {
    const std::array<uint8_t, 3> rgb{kSpawnR, kSpawnG, kSpawnB};
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
        out.triangles.push_back(
            OverlayTriangle{.a = dvec(place(v0)), .b = dvec(place(v1)), .c = dvec(place(v2)), .rgb = rgb});
        out.triangles.push_back(
            OverlayTriangle{.a = dvec(place(v0)), .b = dvec(place(v2)), .c = dvec(place(v3)), .rgb = rgb});
    }
}

constexpr double kDroneNose = 0.5;

// scene.dynamics carries no separate BodyPose list of its own (RenderScene's
// own design, scene.hpp) -- each dynamic DrawItem's local_to_world already
// IS a body's pose (update_dynamics() builds it as rotation-then-translation
// from BodyPose, with no scale: bodies are rigid), so position/orientation
// are recovered from it directly rather than threading a second BodyPose
// span through render()'s own signature.
// F3: THE BODY MARKER IS A FALLBACK, NOT AN ANNOTATION. It is drawn only for
// a dynamic body that has NO MESH -- because for such a body nothing else
// marks the aircraft at all, which was exactly the state the user flew in:
// no shipped airframe declared a visual_ref, so every body resolved to
// kNoMesh, draw_mesh_item returned early, and this ~0.4 m placeholder
// triangle WAS the drone. ("A triangle on a big circle" -- this is the
// triangle; the circle was the spawn marker below.)
//
// Drawn over a real airframe it is clutter and it dominates: 0.4 m of
// overlay across a 0.27 m aircraft, in an accent colour, biased toward the
// camera. So it now yields the moment a body has geometry of its own.
//
// A FALLBACK rather than a RenderOptions flag, deliberately: no new option
// means no consumer has to change and no realm has to coordinate; it
// degrades correctly, since the case where it still fires is precisely the
// case where nothing else would draw; and it retires itself as airframes
// gain meshes, without anyone having to remember to turn it off. A flag for
// "marker ON TOP of a mesh" can be added when someone can say why they want
// one -- adding it now would be speculative.
void append_body_markers(OverlayGeometry& out, const RenderScene& scene) {
    const std::array<uint8_t, 3> rgb{kDroneR, kDroneG, kDroneB};
    const Vec3 nose{0.0, 0.0, -kDroneNose};
    const Vec3 left{-0.2, -0.08, 0.2};
    const Vec3 right{0.2, -0.08, 0.2};
    const Vec3 top{0.0, 0.25, 0.2};
    for (const DrawItem& item : scene.dynamics) {
        if (item.mesh_index != kNoMesh) {
            continue;  // this body draws itself -- see this function's own comment.
        }
        const glm::vec3 position(item.local_to_world[3]);
        const glm::quat orientation = glm::quat_cast(glm::mat3(item.local_to_world));
        const Vec3 base = vec3d(position);
        const double q[4] = {static_cast<double>(orientation.w), static_cast<double>(orientation.x),
                              static_cast<double>(orientation.y), static_cast<double>(orientation.z)};
        const auto place = [&](const Vec3& local) { return base + rotateByQuat(q, local); };
        const auto tri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
            out.triangles.push_back(
                OverlayTriangle{.a = dvec(place(a)), .b = dvec(place(b)), .c = dvec(place(c)), .rgb = rgb});
        };
        tri(nose, left, right);
        tri(nose, right, top);
        tri(nose, top, left);
        tri(left, right, top);
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

// The per-pixel camera-space background ray through pixel center (px+0.5,
// py+0.5) is `Vec3{xNdc*aspect/f, yNdc/f, -1.0}` -- the exact inverse of
// projectCameraSpace's own perspective divide, evaluated at pc.z = -1 so
// the direction's scale is whatever falls out of that choice, and NEVER
// normalized: the analytic ground pass below only ever compares a t
// computed from this ray AGAINST ANOTHER t computed from the SAME ray
// (nearest hit among several standalone ground planes) or tests its SIGN
// (front/back) -- both scale-invariant, so normalizing here would only add
// a sqrt this pass does not need.
//
// `f`/`aspect` are precomputed by the caller (review IMPORTANT 3, fix round
// 1): both are per-VIEW constants (same for every one of a frame's pixels),
// so computing them per pixel -- one sin32+cos32+divide plus a divide --
// was pure, measured waste (2.5-4.4x slower at 1280x720; hoisting alone
// took a ground-bearing frame from 77.6ms to 31.3ms). The same species of
// defect R5b's own review caught in this file (allocation-per-triangle).
//
// S7a Task VQ-B (ruling SR-34/SR-37 -- the user's verdict, "standard
// resolution, just low detail and feature"; this task fixes resolution
// only, and this hoist recovers what Task VQ-A's elevation-based sky
// gradient cost by killing this pass's OLD "skip ray reconstruction for a
// ground-less world" gate -- see draw_sky_and_ground_background()'s own
// comment on that gate, unchanged below). WORLD-space background rays are
// no longer reconstructed per pixel via rotateByQuat(camQ, dirCam) (one
// quaternion rotation -- two cross products -- per pixel); instead this
// exploits that rotateByQuat(q, ·) is LINEAR in its second argument, so for
// FIXED camQ:
//   rotateByQuat(camQ, {xNdc*aspect/f, yNdc/f, -1.0})
//     == (xNdc*aspect/f) * rotateByQuat(camQ, {1,0,0})     [right_world]
//      + (yNdc/f)        * rotateByQuat(camQ, {0,1,0})     [up_world]
//      +                   rotateByQuat(camQ, {0,0,-1})    [forward_world]
// `xNdc` depends ONLY on the pixel's COLUMN, `yNdc` ONLY on its ROW, and the
// third term is IDENTICAL for every pixel in the frame (dirCam.z is always
// exactly -1.0) -- so `right_world`/`up_world`/`forward_world` are three
// rotations per FRAME (not width*height), the two scaled terms are one
// Vec3 per COLUMN and one per ROW (not one per pixel), and what remains at
// each actual pixel is a plain three-term Vec3 add. draw_sky_and_ground_
// background() below builds `col_ray`/`row_ray` from this struct and reads
// them in its own per-pixel loop.
//
// BYTE-EXACT, VERIFIED EMPIRICALLY, NOT ASSUMED FROM THE ALGEBRA ABOVE:
// linearity is a fact about REAL-number arithmetic; IEEE 754 rounding does
// not automatically inherit it, and this hoist is a genuine change in
// floating-point OPERATION ORDER (three separate rotateByQuat calls summed
// per pixel, instead of one rotateByQuat call on the fully-assembled
// per-pixel vector). Checked against all four RasterGolden frame hashes and
// all 30 AgreementMatrix cases (real orbit-camera quaternions, not merely
// axis-aligned ones) before being kept -- zero pixels moved on either; see
// task-VQ-B-report.md for the run. Had either moved by even one bit, this
// hoist would have been reverted, not tolerated: this task's own rule is
// "optimization, not a behaviour change."
struct BackgroundRayBasis {
    Vec3 right_world;    // rotateByQuat(camQ, {1,0,0})
    Vec3 up_world;       // rotateByQuat(camQ, {0,1,0})
    Vec3 forward_world;  // rotateByQuat(camQ, {0,0,-1}) -- dirCam.z is always exactly -1.0
};

[[nodiscard]] BackgroundRayBasis background_ray_basis(const double camQ[4]) {
    return BackgroundRayBasis{
        rotateByQuat(camQ, Vec3{1.0, 0.0, 0.0}),
        rotateByQuat(camQ, Vec3{0.0, 1.0, 0.0}),
        rotateByQuat(camQ, Vec3{0.0, 0.0, -1.0}),
    };
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
// `shadow` (S7a Task R7, ruling SR-24): when the ray hits a ground plane,
// the EXACT world-space hit point (`camPos + dirWorld * best_t`, already
// computed below with no interpolation at all -- a ray/plane intersection,
// not a barycentric lerp) is sampled directly. This is the SAME
// sample_shadow() the mesh path calls (draw_mesh_triangle_shaded via
// rasterizeTriangleGouraud) with a PERSPECTIVE-CORRECT world position of its
// own -- since this pass's hit point has no interpolation error to begin
// with, the two paths agree on a shadow boundary that crosses from a
// tessellated mesh onto the analytic ground exactly the way SR-17's own
// bit-identity seam (rasterizeTriangleGouraud's own comment) already
// requires them to agree on colour.
// `grid` is nullptr when no analytic grid is to be drawn, and `horizon_strength`
// is 0 when SR-17a is not in force. Both are DISTILLED AT THE CALL SITE rather
// than passed as a whole RenderOptions, following this file's own rule that a
// drawing function takes exactly what it draws (see the overlay gating in
// render()'s body, and its comment on why the policy stays visible in one
// place).
void draw_sky_and_ground_background(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene,
                                     bool draw_analytic_ground, const ShadowMap* shadow,
                                     const GroundGridParams* grid, float horizon_strength,
                                     float horizon_onset) {
    // Whether the analytic ground pass has anything at all to do -- still
    // gates the per-plane `front[]` precompute and the per-pixel ray/plane
    // loop below, exactly as before.
    const bool ground_possible = draw_analytic_ground && !scene.ground_planes.empty() && !scene.materials.empty();

    // Per-frame camera constants (review IMPORTANT 3's own discipline: never
    // per pixel). UNCONDITIONALLY computed now, not merely when a ground
    // plane exists (Task VQ-A, ruling SR-23): the elevation-based sky
    // gradient below needs the real per-pixel WORLD-SPACE ray direction to
    // know how far above or below the TRUE horizon that pixel looks, so
    // there is no longer a "nothing to do" case that lets a frame skip ray
    // reconstruction the way a groundless world used to. This gives up this
    // function's own previously-measured "skip it entirely for a groundless
    // world" saving (70.8ms -> 15.9ms at 1280x720) -- that saving assumed
    // the OLD row-only sky gradient, for which a screen row was the whole
    // input; a row alone is no longer enough to colour a pixel correctly
    // under camera pitch, only a real ray is. S7a Task VQ-B recovers PART of
    // this cost a different way, below: not by skipping ray reconstruction,
    // but by no longer doing it PER PIXEL (see background_ray_basis()'s own
    // comment above for the measured recovery and its byte-exactness proof).
    const Vec3 camPos{vc.camPos[0], vc.camPos[1], vc.camPos[2]};
    double camQ[4];
    // Undoes ViewContext's own stored conjugate (world-to-camera rotation)
    // to recover the camera-to-world rotation a background ray needs --
    // quatConjugate is its own inverse, so this is exact, not an
    // approximation.
    quatConjugate(vc.camOrientationConj, camQ);
    const double f = 1.0 / tan32(vc.fovY * 0.5);
    const double aspect = static_cast<double>(vc.width) / static_cast<double>(vc.height);

    // S7a Task VQ-B: the three per-frame rotated basis vectors, and the
    // per-column/per-row vectors built from them -- see
    // background_ray_basis()'s own comment above for the linearity
    // argument this hoist relies on and its byte-exactness verification.
    const BackgroundRayBasis basis = background_ray_basis(camQ);
    std::vector<Vec3> col_ray(fb.width);
    for (uint32_t x = 0; x < fb.width; ++x) {
        const double x_ndc = 2.0 * (static_cast<double>(x) + 0.5) / static_cast<double>(vc.width) - 1.0;
        col_ray[x] = basis.right_world * (x_ndc * aspect / f);
    }
    std::vector<Vec3> row_ray(fb.height);
    for (uint32_t y = 0; y < fb.height; ++y) {
        const double y_ndc = 1.0 - 2.0 * (static_cast<double>(y) + 0.5) / static_cast<double>(vc.height);
        row_ray[y] = basis.up_world * (y_ndc / f);
    }

    // uint8_t, not vector<bool> (review MINOR 9): this file already removed
    // one hidden-cost STL specialization (R5b's own per-triangle heap
    // allocation finding); vector<bool>'s bit-packed proxy-reference
    // specialization is the same species of surprise, avoided here even
    // though this vector is at most `scene.ground_planes.size()` long.
    std::vector<uint8_t> front;

    if (ground_possible) {
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
        for (uint32_t x = 0; x < fb.width; ++x) {
            // The real per-pixel world-space background ray -- built for
            // EVERY pixel now (see the per-frame-constants comment above),
            // not merely when a ground plane exists, since the sky gradient
            // below needs it too. S7a Task VQ-B: a per-column vector plus a
            // per-row vector plus the per-frame forward term, added once
            // here -- replaces what used to be a fresh rotateByQuat(camQ,
            // dirCam) call at every pixel. See background_ray_basis()'s own
            // comment for why this is bit-exact, not merely fast.
            const Vec3 dirWorld = (col_ray[x] + row_ray[y]) + basis.forward_world;

            // Elevation-based sky gradient (ruling SR-23, Task VQ-A;
            // sky_gradient_color() lives in render/scene.hpp, shared
            // verbatim with render/raymarch.cpp's own sky-miss pixels on
            // the SAME per-pixel ray-direction convention, so the two
            // paths' sky agrees bit-for-bit rather than by two
            // independently-equal expressions). Replaces the old plain
            // fraction-of-SCREEN-ROW gradient, which anchored the horizon
            // COLOUR to the bottom screen row and so drifted from the
            // ray-cast horizon LINE the ground hit-test below actually
            // draws whenever the camera pitched. "Hard horizon, no fog"
            // (SR-17) is still enforced by the GROUND hit-test below, never
            // by this gradient.
            glm::vec3 color = sky_gradient_color(scene.lighting, vec3f(dirWorld));

            if (ground_possible) {
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
                    const ShadedColor sc = shade_vertex_color(scene.materials[material_index], scene.lighting, gp.normal);
                    color = sc.combined;
                    const Vec3 hit = camPos + dirWorld * best_t;
                    if (shadow != nullptr) {
                        // SR-25: ambient survives; only the sun term is
                        // attenuated -- see apply_shadow()'s own comment.
                        const float lit = sample_shadow(*shadow, vec3f(hit));
                        color = apply_shadow(sc, lit);
                    }

                    // The infinite analytic grid and SR-17a's atmospheric
                    // term, in that order: the grid is part of the GROUND, so
                    // the horizon blend must act on the gridded colour rather
                    // than on the bare one -- otherwise the lines stay crisp
                    // through a haze that dims everything around them, which
                    // is the artifact that makes a faked horizon look faked.
                    //
                    // BOTH are the SHARED scene.hpp functions, called on the
                    // ray THIS pass already reconstructed. raymarch.cpp calls
                    // the same two on its own ray. That is what keeps the
                    // bit-identity seam (SR-17 clause 4) intact: one function,
                    // one ray, never two expressions that agree by luck.
                    const glm::vec3 hitf = vec3f(hit);
                    const float view_distance =
                        std::sqrt(static_cast<float>(glm::dot(vec3f(dirWorld), vec3f(dirWorld)))) *
                        static_cast<float>(best_t);

                    if (grid != nullptr) {
                        const float cov = ground_grid_coverage(hitf, gp.normal, view_distance, *grid);
                        // base + (line - base) * cov -- EXACT when cov is 0.
                        color = color + (grid->color - color) * cov;
                    }
                    if (horizon_strength > 0.0f) {
                        const glm::vec3 sky = sky_gradient_color(scene.lighting, vec3f(dirWorld));
                        color = horizon_blend(color, sky, view_distance, horizon_onset, horizon_strength);
                    }
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

// Field layers (render/field_layer.hpp): each cell is two flat triangles in
// its bin's palette colour. A layer is data, so no lighting, shadow or
// atmospheric term touches it (SR-17a), and draw_world_triangle() culls
// nothing, so both sides draw. Corners come from the shared lattice, so
// neighbouring cells meet without cracks.
void draw_field_layers(FrameBuffers& fb, const ViewContext& vc, const RenderScene& scene) {
    for (const FieldLayer& layer : scene.field_layers) {
        const float top = resolved_range_max(layer);
        for (uint32_t j = 0; j < layer.cells_v; ++j) {
            for (uint32_t i = 0; i < layer.cells_u; ++i) {
                const float value = layer.values[static_cast<size_t>(j) * layer.cells_u + i];
                const glm::vec3 c = field_cell_colour(layer.colour_map, value, top);
                const uint8_t r = to_byte(c.r), g = to_byte(c.g), b = to_byte(c.b);
                const Vec3 p00 = vec3d(field_cell_corner(layer, i, j));
                const Vec3 p10 = vec3d(field_cell_corner(layer, i + 1u, j));
                const Vec3 p11 = vec3d(field_cell_corner(layer, i + 1u, j + 1u));
                const Vec3 p01 = vec3d(field_cell_corner(layer, i, j + 1u));
                draw_world_triangle(fb, vc, p00, p10, p11, r, g, b);
                draw_world_triangle(fb, vc, p00, p11, p01, r, g, b);
            }
        }
    }
}

}  // namespace

OverlayGeometry overlay_geometry(const RenderScene& scene, const RenderOptions& options) {
    OverlayGeometry out;
    if (!options.overlays) {
        return out;
    }
    append_ground_grid(out, scene);
    append_world_bounds(out, scene);
    // F3: authoring-only -- see RenderOptions::spawn_markers. Gated HERE
    // rather than inside append_spawn_markers so the builder keeps taking
    // exactly what it builds, and the policy stays visible in the one place
    // that reads the whole overlay order.
    if (options.spawn_markers) {
        append_spawn_markers(out, scene);
    }
    append_body_markers(out, scene);
    return out;
}

Result<void> render(const RenderScene& scene, const Camera& camera, const RenderOptions& options,
                     RenderTarget& target, std::vector<float>* shadow_scratch) {
    if (Result<void> valid = validate_target(target); !valid) {
        return valid;
    }
    // A malformed field layer is refused before any pixel is written (L6).
    for (const FieldLayer& layer : scene.field_layers) {
        if (Result<void> valid = validate_field_layer(layer); !valid) {
            return valid;
        }
    }
    // S7a Task R8: raymarch is an exact SDF sphere-tracer, not a rasterizer
    // -- it shares this file's target-validation entry point (above) but
    // none of the ported camera/projection/clip pipeline below, so it is
    // implemented in its own TU (render/raymarch.cpp) and simply forwarded
    // to here, target already validated.
    if (options.mode == DrawMode::raymarch) {
        // SR-17a's two values are forwarded EXPLICITLY rather than by handing
        // over the whole RenderOptions. raymarch.hpp documents that it takes no
        // RenderOptions so that `overlays`/`shadows` are known silent no-ops
        // there; passing the struct would quietly make those two look supported.
        // The term is different in kind from those: it is not something raymarch
        // cannot do, it is something raymarch MUST do, or RS4's agreement band
        // widens systematically on every world with geometry at range.
        return render_raymarch(scene, camera, target, options.horizon_blend_strength,
                                options.horizon_blend_onset);
    }

    const uint32_t width = target.width;
    const uint32_t height = target.height;
    const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

    std::vector<double> depth(pixel_count, 0.0);  // 0 = infinitely far (ported convention)
    FrameBuffers fb{target.pixels, depth, width, height};
    const ViewContext vc = build_view_context(camera, width, height);

    // S7a Task R7 (Step 3): the STATIC half of the shadow map is built once,
    // at scene-build time, and cached on `scene` -- render() never rebuilds
    // it. Only `scene.dynamics` is re-rasterised, once per render() call,
    // into a COPY of that cached map -- never `scene.statics` a second time.
    // Gated on DrawMode::shaded (like the analytic ground below, ruling
    // SR-22's own precedent): wireframe never shades (draw_mesh_item's
    // wireframe branch calls draw_world_segment, which never samples a
    // shadow map at all), so building a per-frame copy for it would be pure
    // waste. `shadow` stays null -- and every existing caller's output stays
    // byte-identical to before this task -- whenever shadows are off, there
    // is no cached static map (every hand-built RenderScene fixture in this
    // program's own test corpus, until one opts in), or the mode is not
    // shaded.
    //
    // I3 (review IMPORTANT, fix round 1, measured at 1443 microseconds/frame
    // for the unconditional-copy version at size=1024): the copy-plus-
    // re-rasterise below is only NEEDED when there is something to fold in.
    // `scene.dynamics.empty()` -- a loaded, disarmed editor session, the
    // common case the review named -- reads the cached map DIRECTLY, no
    // copy, no redundant re-rasterisation of `scene.statics` (which would
    // be a pure no-op on the depth VALUES anyway, since the same triangles
    // would recompute the same occluders, but is still wasted CPU to
    // recompute at all). Only the non-empty-dynamics path below allocates
    // (or, given `shadow_scratch`, reuses) a per-frame working copy.
    std::optional<ShadowMap> frame_shadow;
    const ShadowMap* shadow = nullptr;
    if (options.shadows && options.mode == DrawMode::shaded && scene.static_shadow.has_value()) {
        if (scene.dynamics.empty()) {
            shadow = &*scene.static_shadow;
        } else {
            ShadowMap& built = frame_shadow.emplace();
            built.size = scene.static_shadow->size;
            built.light_view_proj = scene.static_shadow->light_view_proj;
            // `shadow_scratch` (I3's own second fix, raster_cpu.hpp's doc
            // comment): when the caller supplies a persistent buffer,
            // reclaim whatever capacity it already grew on a previous
            // frame before overwriting it -- `assign()` below only
            // reallocates if that capacity is insufficient, so a caller
            // reusing the SAME buffer at a STABLE shadow-map size pays the
            // ~4 MiB allocation once, not every frame.
            if (shadow_scratch != nullptr) {
                built.depth = std::move(*shadow_scratch);
            }
            built.depth.assign(scene.static_shadow->depth.begin(), scene.static_shadow->depth.end());
            rasterize_shadow_casters(scene.meshes, scene.dynamics, built);
            shadow = &built;
        }
    }

    // MN-14 / constraint 2: every byte written every frame, X always 0xFF --
    // now the sky gradient + analytic ground background pass (S7a Task R6),
    // not a flat clear: it still writes every pixel unconditionally, just no
    // longer the same colour everywhere. Writes colour only, no depth, so it
    // never survives the z-test against any real geometry drawn afterward.
    // The analytic ground itself is gated to DrawMode::shaded (ruling SR-22,
    // review IMPORTANT 4) -- the sky gradient applies to every mode.
    // Grid and horizon term are SHADED-ONLY, distilled here beside the analytic
    // ground's own gate (ruling SR-22): a filled surface under wireframe is a
    // mode-contract violation, and a grid painted onto a surface that is not
    // drawn would be one too.
    const bool shaded = options.mode == DrawMode::shaded;
    const GroundGridParams* grid_params =
        (shaded && options.ground_grid) ? &options.ground_grid_params : nullptr;
    draw_sky_and_ground_background(fb, vc, scene, shaded, shadow, grid_params,
                                    shaded ? options.horizon_blend_strength : 0.0f,
                                    options.horizon_blend_onset);

    // Fixed operation order (constraint 4): background, then statics, then
    // dynamics, then overlays -- never based on hashing, pointer identity, or
    // anything else unordered.
    for (const DrawItem& item : scene.statics) {
        draw_mesh_item(fb, vc, scene, item, options, shadow);
    }
    for (const DrawItem& item : scene.dynamics) {
        draw_mesh_item(fb, vc, scene, item, options, shadow);
    }
    draw_field_layers(fb, vc, scene);

    // Overlays: overlay_geometry() decides what draws and in what order; GL
    // draws the same lists.
    if (options.overlays) {
        const OverlayGeometry overlays = overlay_geometry(scene, options);
        for (const OverlayLine& line : overlays.lines) {
            draw_world_segment(fb, vc, from_dvec(line.a), from_dvec(line.b), line.rgb[0], line.rgb[1], line.rgb[2],
                               kOverlayDepthBias);
        }
        for (const OverlayTriangle& tri : overlays.triangles) {
            draw_world_triangle(fb, vc, from_dvec(tri.a), from_dvec(tri.b), from_dvec(tri.c), tri.rgb[0], tri.rgb[1],
                                tri.rgb[2], kOverlayDepthBias);
        }
    }

    // Hands the (possibly newly-grown) depth buffer back to the caller for
    // reuse next frame (I3's own scratch-buffer mechanism) -- only reached
    // when the dynamics branch above actually built a per-frame copy AND
    // the caller opted in; every other combination leaves `shadow_scratch`
    // untouched. This is the LAST use of `frame_shadow`/`shadow` in this
    // function, so moving the buffer out here cannot affect anything drawn
    // above.
    if (shadow_scratch != nullptr && frame_shadow.has_value()) {
        *shadow_scratch = std::move(frame_shadow->depth);
    }

    return {};
}

}  // namespace spade::render
