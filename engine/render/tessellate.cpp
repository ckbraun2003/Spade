#include "render/tessellate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/vec3.hpp>

#include "core/fp32_math.hpp"

namespace spade::render {
namespace {

// sin32/cos32 (core/fp32_math.hpp), NOT std::sin/std::cos, everywhere in this
// file -- including the sphere/cylinder/capsule/torus circular positions,
// which world/sdf.hpp's own primitive_distance() never needed a transcendental
// for at all. The reason is the golden corpus, not the physics: this file
// produces a COMMITTED, cross-platform sha256 golden (tests/golden/render/
// tessellation/manifest.json), and the whole point of fp32_math.hpp (see its
// own file comment) is that the platform libm is exactly the kind of
// dependency that would make two conforming compilers disagree on it by a
// ulp and move the golden silently between Windows/MSVC and Linux/glibc.
// heightfield's y-coordinate and gradient additionally reuse the EXACT
// formula world/sdf.cpp's heightfield case evaluates, so the drawn surface
// agrees with the physics surface it is standing in for.
using math::cos32;
using math::sin32;

constexpr float kPi = glm::pi<float>();
constexpr float kTwoPi = glm::two_pi<float>();

// ---------------------------------------------------------------------------
// MeshBuilder -- a thin, ordered accumulator. Every generator below appends
// vertices and triangles in a FIXED, index-driven loop order (never based on
// hashing, pointer identity, or anything else unordered), which is what makes
// "call twice, get identical bytes" (constraint 4) true by construction
// rather than by luck.
// ---------------------------------------------------------------------------
struct MeshBuilder {
    MeshData mesh;

    uint32_t add_vertex(glm::vec3 position, glm::vec3 normal) {
        const uint32_t index = static_cast<uint32_t>(mesh.positions.size());
        mesh.positions.push_back(position);
        mesh.normals.push_back(normal);
        return index;
    }

    void add_triangle(uint32_t a, uint32_t b, uint32_t c) {
        mesh.indices.push_back(a);
        mesh.indices.push_back(b);
        mesh.indices.push_back(c);
    }
};

// Appends 2*quad_rows*quad_cols triangles over a grid of already-emitted
// vertices, laid out row-major starting at `base` with row stride `col_count`
// (vertex(r, c) == base + r*col_count + c).
//
// `row_wrap`/`col_wrap` select whether the LAST row/column connects back to
// row/column 0 (a genuinely periodic primitive -- torus, in both directions)
// or is a true open edge. Several of the generators below use `false` on a
// grid whose vertex data ALREADY duplicates the seam (sphere's (segments+1)th
// column repeats column 0's positions, heightfield/plane's grids simply end);
// that is a property of how the vertices were emitted, not of this function.
//
// `flip_winding` (S7a Task R2 fix wave 1, review CRITICAL finding): whether
// (r, c) increases in the SAME handedness as the stored outward normal, or
// the opposite one, is a fact about how each CALLER built its (row, col)
// parameterization in 3D -- not a fact about the grid topology row_wrap/
// col_wrap already describe. Two generators can (and do) share identical
// row_wrap/col_wrap values yet need opposite windings: sphere and plane both
// pass (false, false), but sphere's theta/phi sweep and plane's (u, v) basis
// disagree on handedness. Every call site below states which triangle order
// it needs and why, rather than leaving it to be re-derived by eye per
// generator (the review finding this fixes: four of seven primitives were
// wound backward before `flip_winding` existed, checked only by hand and not
// caught by any test in the process).
void append_grid(MeshBuilder& out, uint32_t base, uint32_t row_count, uint32_t col_count, bool row_wrap,
                  bool col_wrap, bool flip_winding) {
    const uint32_t quad_rows = row_wrap ? row_count : row_count - 1;
    const uint32_t quad_cols = col_wrap ? col_count : col_count - 1;
    for (uint32_t r = 0; r < quad_rows; ++r) {
        const uint32_t r0 = r;
        const uint32_t r1 = (row_wrap && r + 1 == row_count) ? 0 : r + 1;
        for (uint32_t c = 0; c < quad_cols; ++c) {
            const uint32_t c0 = c;
            const uint32_t c1 = (col_wrap && c + 1 == col_count) ? 0 : c + 1;
            const uint32_t v00 = base + r0 * col_count + c0;
            const uint32_t v01 = base + r0 * col_count + c1;
            const uint32_t v10 = base + r1 * col_count + c0;
            const uint32_t v11 = base + r1 * col_count + c1;
            if (flip_winding) {
                out.add_triangle(v00, v11, v10);
                out.add_triangle(v00, v01, v11);
            } else {
                out.add_triangle(v00, v10, v11);
                out.add_triangle(v00, v11, v01);
            }
        }
    }
}

// Appends a triangle fan connecting `center` to a periodic ring of
// `perim_count` already-emitted vertices starting at `perim_base`.
void append_fan(MeshBuilder& out, uint32_t center, uint32_t perim_base, uint32_t perim_count) {
    for (uint32_t i = 0; i < perim_count; ++i) {
        const uint32_t a = perim_base + i;
        const uint32_t b = perim_base + ((i + 1 == perim_count) ? 0 : i + 1);
        out.add_triangle(center, a, b);
    }
}

// ---------------------------------------------------------------------------
// plane -- params (n.x, n.y, n.z, offset). Bounded grid fitted to
// `world_bounds` (PA-5): the plane's own field has no inherent extent, so the
// bound comes from the same Aabb every other draw item's visible region comes
// from (render/scene.cpp's world_bounds_of()), not from the camera.
//
// Basis: an orthonormal (u, v) spanning the plane's tangent space, picked
// with a FIXED, data-independent rule (never a camera-relative "facing"
// choice) so the same params + bounds always produce the same grid. `origin`
// is the point on the plane closest to the bounds' center; the grid extends
// symmetrically about it far enough to cover every AABB corner's projection
// onto (u, v), so the drawn quad always covers the visible world region.
// ---------------------------------------------------------------------------
MeshData tessellate_plane(glm::vec4 params, const Aabb& world_bounds, const TessellationLimits& limits) {
    MeshBuilder out;
    const glm::vec3 n = glm::normalize(glm::vec3(params));  // re-normalized defensively
    const float offset = params.w;

    const glm::vec3 up = (std::fabs(n.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 u = glm::normalize(glm::cross(up, n));
    const glm::vec3 v = glm::cross(n, u);

    const glm::vec3 center = (world_bounds.min + world_bounds.max) * 0.5f;
    const glm::vec3 origin = center - (glm::dot(center, n) - offset) * n;

    float half_u = 0.0f, half_v = 0.0f;
    for (uint32_t corner = 0; corner < 8; ++corner) {
        const glm::vec3 c((corner & 1u) ? world_bounds.max.x : world_bounds.min.x,
                           (corner & 2u) ? world_bounds.max.y : world_bounds.min.y,
                           (corner & 4u) ? world_bounds.max.z : world_bounds.min.z);
        const glm::vec3 delta = c - origin;
        half_u = std::max(half_u, std::fabs(glm::dot(delta, u)));
        half_v = std::max(half_v, std::fabs(glm::dot(delta, v)));
    }

    const uint32_t cells = limits.plane_grid_cells;
    const uint32_t base = static_cast<uint32_t>(out.mesh.positions.size());
    for (uint32_t i = 0; i <= cells; ++i) {
        const float fu = -half_u + (2.0f * half_u) * (static_cast<float>(i) / static_cast<float>(cells));
        for (uint32_t j = 0; j <= cells; ++j) {
            const float fv = -half_v + (2.0f * half_v) * (static_cast<float>(j) / static_cast<float>(cells));
            out.add_vertex(origin + u * fu + v * fv, n);
        }
    }
    // flip_winding=false: correct as-is (review-verified, positive dot against
    // the shared normal `n`).
    append_grid(out, base, cells + 1, cells + 1, /*row_wrap=*/false, /*col_wrap=*/false,
                /*flip_winding=*/false);
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// sphere -- params (radius, 0, 0, 0). Standard UV sphere: (rings+1) latitude
// circles from pole to pole times (segments+1) longitude columns -- the
// (segments+1)th column duplicates the 0th column's positions/normals so the
// grid has a true open edge (no wraparound index math needed), which is the
// closed form the task brief itself names: (rings+1)*(segments+1) vertices.
// ---------------------------------------------------------------------------
MeshData tessellate_sphere(glm::vec4 params, const TessellationLimits& limits) {
    MeshBuilder out;
    const float radius = params.x;
    const uint32_t rings = limits.sphere_rings;
    const uint32_t segments = limits.sphere_segments;

    const uint32_t base = static_cast<uint32_t>(out.mesh.positions.size());
    for (uint32_t i = 0; i <= rings; ++i) {
        const float theta = kPi * static_cast<float>(i) / static_cast<float>(rings);
        const float y = radius * cos32(theta);
        const float r_xy = radius * sin32(theta);
        for (uint32_t j = 0; j <= segments; ++j) {
            const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(segments);
            const glm::vec3 position(r_xy * cos32(phi), y, r_xy * sin32(phi));
            const glm::vec3 normal = (radius > 0.0f) ? position / radius : glm::vec3(0.0f, 1.0f, 0.0f);
            out.add_vertex(position, normal);
        }
    }
    // flip_winding=true: review CRITICAL finding -- the default order was
    // backward here (theta/phi's handedness is opposite plane's (u, v) basis
    // despite both passing row_wrap=col_wrap=false).
    append_grid(out, base, rings + 1, segments + 1, /*row_wrap=*/false, /*col_wrap=*/false,
                /*flip_winding=*/true);
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// box -- params (hx, hy, hz, 0), half extents about the local origin. Six
// flat-shaded faces, four vertices each, NOT shared between faces (MeshData's
// own convention: "flat meshes duplicate vertices", scene.hpp) since adjacent
// faces disagree on the normal. 24 vertices, 36 indices -- fixed, no limits
// table entry, since a box's own shape is already exactly its geometry.
// ---------------------------------------------------------------------------
MeshData tessellate_box(glm::vec4 params) {
    MeshBuilder out;
    const float hx = params.x, hy = params.y, hz = params.z;

    struct Face {
        glm::vec3 normal;
        glm::vec3 corners[4];  // CCW as seen from outside the box
    };
    const Face faces[6] = {
        {{1.0f, 0.0f, 0.0f},
         {{hx, -hy, -hz}, {hx, hy, -hz}, {hx, hy, hz}, {hx, -hy, hz}}},  // +X
        {{-1.0f, 0.0f, 0.0f},
         {{-hx, -hy, hz}, {-hx, hy, hz}, {-hx, hy, -hz}, {-hx, -hy, -hz}}},  // -X
        {{0.0f, 1.0f, 0.0f},
         {{-hx, hy, hz}, {hx, hy, hz}, {hx, hy, -hz}, {-hx, hy, -hz}}},  // +Y
        {{0.0f, -1.0f, 0.0f},
         {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, -hy, hz}, {-hx, -hy, hz}}},  // -Y
        {{0.0f, 0.0f, 1.0f},
         {{hx, -hy, hz}, {hx, hy, hz}, {-hx, hy, hz}, {-hx, -hy, hz}}},  // +Z
        {{0.0f, 0.0f, -1.0f},
         {{-hx, -hy, -hz}, {-hx, hy, -hz}, {hx, hy, -hz}, {hx, -hy, -hz}}},  // -Z
    };
    for (const Face& f : faces) {
        const uint32_t base = out.add_vertex(f.corners[0], f.normal);
        out.add_vertex(f.corners[1], f.normal);
        out.add_vertex(f.corners[2], f.normal);
        out.add_vertex(f.corners[3], f.normal);
        out.add_triangle(base, base + 1, base + 2);
        out.add_triangle(base, base + 2, base + 3);
    }
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// cylinder -- params (radius, half_height, 0, 0), capped, local +Y. A side
// wall (two rings of `circle_segments` vertices, periodic around the
// circumference -- smooth radial normal, no seam to duplicate) plus two flat
// end caps (their own center + perimeter vertices, since a cap's normal is
// +-Y, not the side's radial normal).
// ---------------------------------------------------------------------------
MeshData tessellate_cylinder(glm::vec4 params, const TessellationLimits& limits) {
    MeshBuilder out;
    const float radius = params.x;
    const float half_height = params.y;
    const uint32_t segs = limits.circle_segments;

    // Side wall: two periodic rings, radial normal.
    const uint32_t side_base = static_cast<uint32_t>(out.mesh.positions.size());
    for (const float y : {-half_height, half_height}) {
        for (uint32_t j = 0; j < segs; ++j) {
            const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(segs);
            const glm::vec3 normal(cos32(phi), 0.0f, sin32(phi));
            out.add_vertex(glm::vec3(radius * normal.x, y, radius * normal.z), normal);
        }
    }
    // flip_winding=false: correct as-is (review-verified).
    append_grid(out, side_base, /*row_count=*/2, segs, /*row_wrap=*/false, /*col_wrap=*/true,
                /*flip_winding=*/false);

    // Two flat caps: center + `segs` perimeter vertices, own flat normal.
    for (const float sign : {-1.0f, 1.0f}) {
        const float y = sign * half_height;
        const glm::vec3 cap_normal(0.0f, sign, 0.0f);
        const uint32_t center = out.add_vertex(glm::vec3(0.0f, y, 0.0f), cap_normal);
        const uint32_t perim_base = static_cast<uint32_t>(out.mesh.positions.size());
        for (uint32_t j = 0; j < segs; ++j) {
            const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(segs);
            out.add_vertex(glm::vec3(radius * cos32(phi), y, radius * sin32(phi)), cap_normal);
        }
        // Winding: the top cap (sign > 0) is wound the opposite way from the
        // bottom cap so both face outward -- append_fan's (center, i, i+1)
        // order is reversed to (center, i+1, i) for one of the two.
        if (sign > 0.0f) {
            for (uint32_t j = 0; j < segs; ++j) {
                const uint32_t a = perim_base + j;
                const uint32_t b = perim_base + ((j + 1 == segs) ? 0 : j + 1);
                out.add_triangle(center, b, a);
            }
        } else {
            append_fan(out, center, perim_base, segs);
        }
    }
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// capsule -- params (radius, half_height, 0, 0), segment (0, +-half_height, 0)
// inflated by radius, local +Y. A REAL capsule (tessellate.hpp's file
// comment): a cylindrical body identical in construction to the cylinder
// side above, plus TWO HEMISPHERICAL caps -- never the flat-cap cylinder
// shortcut the wireframe rasterizer uses.
//
// The hemisphere's latitude subdivision reuses `circle_segments` alone
// (hemisphere_rings = circle_segments/4, a quarter turn's worth of the same
// angular resolution the circumference already uses) rather than adding a
// dedicated limits-table field: circle_segments already IS "the number of
// segments in one full turn" for this primitive (per the field's own
// doc comment, "cylinder/torus/capsule rings"), and a hemisphere's latitude
// sweep is exactly a quarter turn.
// ---------------------------------------------------------------------------
MeshData tessellate_capsule(glm::vec4 params, const TessellationLimits& limits) {
    MeshBuilder out;
    const float radius = params.x;
    const float half_height = params.y;
    const uint32_t segs = limits.circle_segments;
    const uint32_t hemisphere_rings = segs / 4;

    // Cylindrical body: identical shape to tessellate_cylinder's side wall.
    const uint32_t side_base = static_cast<uint32_t>(out.mesh.positions.size());
    for (const float y : {-half_height, half_height}) {
        for (uint32_t j = 0; j < segs; ++j) {
            const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(segs);
            const glm::vec3 normal(cos32(phi), 0.0f, sin32(phi));
            out.add_vertex(glm::vec3(radius * normal.x, y, radius * normal.z), normal);
        }
    }
    // flip_winding=false: correct as-is (review-verified, identical shape to
    // tessellate_cylinder's side wall above).
    append_grid(out, side_base, /*row_count=*/2, segs, /*row_wrap=*/false, /*col_wrap=*/true,
                /*flip_winding=*/false);

    // Two hemispherical caps. theta sweeps [0, pi/2]: 0 at the dome's tip,
    // pi/2 at the equator (which geometrically coincides with the side wall's
    // own end ring, though the two are not vertex-welded -- see this file's
    // append_grid doc comment on duplicated seams).
    //
    // Winding (review CRITICAL finding, fixed): the two domes are mirror
    // images of each other in Y, so the SAME (row, col) parameterization
    // walks in opposite handedness relative to the outward normal on the two
    // sides -- exactly one of the two domes needs flip_winding=true, and it
    // is the one where sign matches the +Y axis append_grid's default order
    // was derived against (review-verified: sign>0 needs the flip; sign<0 is
    // append_grid's default, unmodified). A previous version of this
    // function hand-rolled a second index-construction loop here instead of
    // passing flip_winding, and had BOTH signs backward -- swapping an
    // already-correct order into a wrong one for sign<0, while never
    // flipping the sign>0 case that actually needed it. One shared call for
    // both domes, as below, cannot independently drift out of sync with
    // itself the way two hand-written loops did.
    for (const float sign : {-1.0f, 1.0f}) {
        const glm::vec3 cap_center(0.0f, sign * half_height, 0.0f);
        const uint32_t dome_base = static_cast<uint32_t>(out.mesh.positions.size());
        for (uint32_t i = 0; i <= hemisphere_rings; ++i) {
            const float theta = (kPi * 0.5f) * static_cast<float>(i) / static_cast<float>(hemisphere_rings);
            const float y_dir = sign * cos32(theta);
            const float r_xy = sin32(theta);
            for (uint32_t j = 0; j < segs; ++j) {
                const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(segs);
                const glm::vec3 normal(r_xy * cos32(phi), y_dir, r_xy * sin32(phi));
                out.add_vertex(cap_center + radius * normal, normal);
            }
        }
        append_grid(out, dome_base, hemisphere_rings + 1, segs, /*row_wrap=*/false, /*col_wrap=*/true,
                    /*flip_winding=*/sign > 0.0f);
    }
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// torus -- params (major_radius, minor_radius, 0, 0), ring in local XZ, hole
// axis +Y. Fully periodic in BOTH the main-ring direction (`circle_segments`)
// and the tube's cross-section (`torus_ring_segments`) -- no seam to
// duplicate in either direction, so append_grid wraps both.
// ---------------------------------------------------------------------------
MeshData tessellate_torus(glm::vec4 params, const TessellationLimits& limits) {
    MeshBuilder out;
    const float major_r = params.x;
    const float minor_r = params.y;
    const uint32_t main_segs = limits.circle_segments;
    const uint32_t tube_segs = limits.torus_ring_segments;

    const uint32_t base = static_cast<uint32_t>(out.mesh.positions.size());
    for (uint32_t i = 0; i < main_segs; ++i) {
        const float theta = kTwoPi * static_cast<float>(i) / static_cast<float>(main_segs);
        const glm::vec3 radial(cos32(theta), 0.0f, sin32(theta));
        for (uint32_t j = 0; j < tube_segs; ++j) {
            const float phi = kTwoPi * static_cast<float>(j) / static_cast<float>(tube_segs);
            const glm::vec3 normal = cos32(phi) * radial + glm::vec3(0.0f, sin32(phi), 0.0f);
            const glm::vec3 position = major_r * radial + minor_r * normal;
            out.add_vertex(position, normal);
        }
    }
    // flip_winding=true: review CRITICAL finding -- the default order was
    // backward here.
    append_grid(out, base, main_segs, tube_segs, /*row_wrap=*/true, /*col_wrap=*/true,
                /*flip_winding=*/true);
    return std::move(out.mesh);
}

// ---------------------------------------------------------------------------
// heightfield -- params (amplitude, freq_x, freq_z, base_y). Just as
// analytically unbounded over (x, z) as the plane (world/sdf.hpp's own field
// is defined for every real x, z), so it is bounded to `world_bounds` the
// same way (PA-5's reasoning extended to the one other primitive with no
// inherent extent).
//
// h(x, z) and its gradient are the EXACT formula world/sdf.cpp's heightfield
// case evaluates (that file's own long comment carries the derivation) --
// same sin32/cos32 calls, same argument order -- so the drawn surface is the
// physics surface, not merely a picture that resembles it.
// ---------------------------------------------------------------------------
MeshData tessellate_heightfield(glm::vec4 params, const Aabb& world_bounds, const TessellationLimits& limits) {
    MeshBuilder out;
    const float amplitude = params.x;
    const float freq_x = params.y;
    const float freq_z = params.z;
    const float base_y = params.w;
    const uint32_t cells = limits.heightfield_cells;

    const float x0 = world_bounds.min.x, x1 = world_bounds.max.x;
    const float z0 = world_bounds.min.z, z1 = world_bounds.max.z;

    const uint32_t base = static_cast<uint32_t>(out.mesh.positions.size());
    for (uint32_t i = 0; i <= cells; ++i) {
        const float x = x0 + (x1 - x0) * (static_cast<float>(i) / static_cast<float>(cells));
        const float sin_fx_x = sin32(freq_x * x);
        const float cos_fx_x = cos32(freq_x * x);
        for (uint32_t j = 0; j <= cells; ++j) {
            const float z = z0 + (z1 - z0) * (static_cast<float>(j) / static_cast<float>(cells));
            const float sin_fz_z = sin32(freq_z * z);
            const float cos_fz_z = cos32(freq_z * z);

            const float y = base_y + amplitude * sin_fx_x * sin_fz_z;
            // Analytic gradient of h (world/sdf.cpp's own derivation):
            //   hx = a*fx*cos(fx*x)*sin(fz*z), hz = a*fz*sin(fx*x)*cos(fz*z)
            const float hx = amplitude * freq_x * cos_fx_x * sin_fz_z;
            const float hz = amplitude * freq_z * sin_fx_x * cos_fz_z;
            const glm::vec3 normal = glm::normalize(glm::vec3(-hx, 1.0f, -hz));

            out.add_vertex(glm::vec3(x, y, z), normal);
        }
    }
    // flip_winding=true: review CRITICAL finding -- the default order was
    // backward here (verified exactly: amplitude 0, the flat-plane special
    // case, yields cross(edge1, edge2) = (0, -100, 0) against the surface's
    // own up-normal without the flip).
    append_grid(out, base, cells + 1, cells + 1, /*row_wrap=*/false, /*col_wrap=*/false,
                /*flip_winding=*/true);
    return std::move(out.mesh);
}

}  // namespace

Result<MeshData> tessellate_primitive(SdfPrim kind, glm::vec4 params, const Aabb& world_bounds,
                                       const TessellationLimits& limits) {
    switch (kind) {
        case SdfPrim::plane:
            return tessellate_plane(params, world_bounds, limits);
        case SdfPrim::sphere:
            return tessellate_sphere(params, limits);
        case SdfPrim::box:
            return tessellate_box(params);
        case SdfPrim::cylinder:
            return tessellate_cylinder(params, limits);
        case SdfPrim::capsule:
            return tessellate_capsule(params, limits);
        case SdfPrim::torus:
            return tessellate_torus(params, limits);
        case SdfPrim::heightfield:
            return tessellate_heightfield(params, world_bounds, limits);
    }
    // Unreachable for a kind that came from a validated SdfNode -- matches
    // world/sdf.cpp's primitive_distance() switch, same reasoning.
    return std::unexpected(Error{Code::invalid_argument, "tessellate_primitive: unknown SdfPrim kind"});
}

}  // namespace spade::render
