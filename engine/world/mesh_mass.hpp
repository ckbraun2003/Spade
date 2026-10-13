#pragma once

// ---------------------------------------------------------------------------
// world/mesh_mass.hpp -- a triangle mesh's volume, centroid and inertia
// (CORE-6; the parts and meshes plan, Task B3). A host call, made once at
// import or compile time; no step reads it.
//
// PER UNIT DENSITY. Neither the mesh nor this result holds a density. A
// caller that gives one, a uniform rho, gets the mass rho * volume and the
// inertia rho * inertia, about the centroid, at the centroid. make_mesh()
// calls this once on every mesh it accepts, and the mesh keeps the result
// (TriangleMesh::mass_properties(), world/mesh.hpp): read it there rather
// than computing it again. The airframe compile does, for a mesh record,
// times the record's density (vehicles/airframe_compile.hpp, MASS FROM A
// MESH).
//
// THE CONVENTIONS are PartInertia::tensor's (vehicles/composite_inertia.hpp),
// so the compile hands the result, times rho, straight to composite_inertia():
//   * metres, in the mesh's own frame (the part frame), as
//     TriangleMesh::positions() gives them;
//   * the inertia is about the CENTROID, not the frame's origin;
//   * inertia[i][j] = integral of (|r|^2 delta_ij - r_i r_j) dV, r measured
//     from the centroid: the moments on the diagonal, the products of
//     inertia off it with their minus sign. It is symmetric bit for bit
//     (inertia[i][j] and inertia[j][i] are one value), so glm's column-major
//     layout does not matter.
//
// THE METHOD: Eberly's polyhedral mass properties ("Polyhedral Mass
// Properties (Revisited)", 2002, his simplification of Mirtich 1996 for
// triangles). The divergence theorem turns each volume integral of 1, x, y,
// z, x^2, y^2, z^2, xy, yz and zx into a sum over the triangles of a closed
// form in their vertices. It is exact for any closed, oriented mesh, so
// shells add, and an inward shell (a cavity) subtracts, with no special
// case. Every mesh make_mesh() returns is closed and oriented, with a
// positive total volume, so the centroid's division is safe.
//   * The sums run in DOUBLE, over the stored fp32 positions promoted
//     exactly, in TRIANGLE INDEX ORDER, each expression spelt left to right
//     in the .cpp, with only + - * / (no sqrt, no libm: TD-3).
//   * They are taken about the centre of the mesh's bounds, (lo + hi) / 2,
//     which limits cancellation (a mesh far from its frame's origin loses
//     no digits to the offset), and the second moments are then shifted to
//     the centroid by the parallel-axis rule. Moving a mesh therefore
//     changes its volume and inertia by rounding alone, and not at all when
//     the move keeps every vertex's offset from the centre exact (a short
//     dyadic shift, as tests/test_mesh_mass.cpp's): then only the centroid
//     moves.
//   * Nothing is rounded to float: a caller hands the doubles to
//     composite_inertia(), which rounds its own outputs once.
//
// DETERMINISM. One input gives one result, bit for bit, on every toolchain:
// the order is pinned, IEEE 754 rounds + - * / correctly, and the .cpp
// builds under spade_fp_strict (gcc: -ffp-contract=off; MSVC's /fp:precise
// never contracts), so no multiply-add is fused (tests/test_mesh_mass.cpp
// pins one mesh's bits).
// ---------------------------------------------------------------------------

#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "world/mesh.hpp"

namespace spade {

// MeshMassProperties (volume, centroid, inertia) is defined in world/mesh.hpp,
// beside the TriangleMesh that keeps one.

// Eberly's polyhedral integrals, double, triangle order, about the bounds'
// centre, then shifted to the centroid (above).
[[nodiscard]] MeshMassProperties mesh_mass_properties(const TriangleMesh& mesh) noexcept;

}  // namespace spade
