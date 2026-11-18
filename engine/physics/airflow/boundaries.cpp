#include "physics/airflow/boundaries.hpp"

namespace spade::physics::airflow {

FaceKind face_kind(const DomainBc& bc, Side side) noexcept {
    const FaceBc f = bc.face[side];
    if (f == FaceBc::periodic) return FaceKind::periodic;
    if (f == FaceBc::wall) return FaceKind::fixed_velocity;
    const auto axis = static_cast<glm::length_t>(side / 2u);
    const float inward = (side % 2u) == 0u ? bc.ambient[axis] : -bc.ambient[axis];
    return inward > 0.0f ? FaceKind::fixed_velocity : FaceKind::pressure_outlet;
}

std::array<FaceKind, 6> face_kinds(const DomainBc& bc) noexcept {
    std::array<FaceKind, 6> out{};
    for (uint32_t s = 0; s < 6; ++s) out[s] = face_kind(bc, static_cast<Side>(s));
    return out;
}

float fixed_normal_velocity(const DomainBc& bc, Side side) noexcept {
    const auto axis = static_cast<glm::length_t>(side / 2u);
    return bc.face[side] == FaceBc::wall ? bc.wall_velocity[side][axis] : bc.ambient[axis];
}

namespace {

// One axis' normal faces. `at(q, a, b)` addresses the component's array with q
// along the axis (0..n) and (a, b) across it. A pressure outlet's faces are not
// written (L90): the projection's p = 0 on the face sets what crosses it, and
// the advection reads the ambient past it (advect.cpp). Until L90 the faces took
// a copy of their neighbour and refused inflow, once a step and with no factor
// of h, which wiped the air a rotor draws in: a box held a smaller share of an
// impulse the smaller h was; copying without the refusal let backflow grow.
template <class At>
void apply_axis(const DomainBc& bc, uint32_t axis, uint32_t n, uint32_t m1, uint32_t m2, At at) noexcept {
    const auto lo = static_cast<Side>(2u * axis);
    const auto hi = static_cast<Side>(2u * axis + 1u);
    const FaceKind klo = face_kind(bc, lo);
    const FaceKind khi = face_kind(bc, hi);
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            if (klo == FaceKind::periodic) {
                at(n, a, b) = at(0, a, b);
                continue;
            }
            if (klo == FaceKind::fixed_velocity) {
                at(0, a, b) = fixed_normal_velocity(bc, lo);
            }  // else an outlet: its face is left as the projection set it (L90)
            if (khi == FaceKind::fixed_velocity) at(n, a, b) = fixed_normal_velocity(bc, hi);
        }
    }
}

}  // namespace

void apply_velocity_boundaries(MacGrid& g, const DomainBc& bc) noexcept {
    const GridShape& s = g.shape;
    apply_axis(bc, 0, s.nx, s.ny, s.nz,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(s, q, a, b)]; });
    apply_axis(bc, 1, s.ny, s.nx, s.nz,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(s, a, q, b)]; });
    apply_axis(bc, 2, s.nz, s.nx, s.ny,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(s, a, b, q)]; });
}

}  // namespace spade::physics::airflow
