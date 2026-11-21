#include "fc_link/frames.hpp"

namespace spade::fc_link::frames {

namespace {

// glm::dmat3 takes COLUMNS: column j is the image of the j-th basis vector.
// body (x, y, z) -> (x, z, -y): x -> (1,0,0), y -> (0,0,-1), z -> (0,1,0)
glm::dmat3 body_map() { return glm::dmat3(glm::dvec3(1, 0, 0), glm::dvec3(0, 0, -1), glm::dvec3(0, 1, 0)); }
// anchored world (x east, y up, z south) -> NED: x -> East (0,1,0), y -> Down (0,0,-1), z -> South = -North (-1,0,0)
glm::dmat3 anchored_world_map() { return glm::dmat3(glm::dvec3(0, 1, 0), glm::dvec3(0, 0, -1), glm::dvec3(-1, 0, 0)); }

glm::dquat positive_w(glm::dquat q) {
    q = glm::normalize(q);
    return q.w < 0.0 ? -q : q;
}

}  // namespace

FrameMap compass_x_north() { return FrameMap{body_map(), body_map()}; }
FrameMap anchored_world() { return FrameMap{anchored_world_map(), body_map()}; }

glm::dvec3 world_to_ned(const FrameMap& m, const glm::dvec3& v) { return m.world * v; }
glm::dvec3 ned_to_world(const FrameMap& m, const glm::dvec3& v) { return glm::transpose(m.world) * v; }  // a rotation: transpose = inverse
glm::dvec3 body_to_frd(const FrameMap& m, const glm::dvec3& v) { return m.body * v; }
glm::dvec3 frd_to_body(const FrameMap& m, const glm::dvec3& v) { return glm::transpose(m.body) * v; }

glm::dquat attitude_to_ned_frd(const FrameMap& m, const glm::dquat& q) {
    // R_ned,frd = W R_spade B^-1: the world's map on the left, the body's inverse on the right
    return positive_w(glm::quat_cast(m.world) * q * glm::inverse(glm::quat_cast(m.body)));
}

glm::dquat attitude_from_ned_frd(const FrameMap& m, const glm::dquat& q) {
    return positive_w(glm::inverse(glm::quat_cast(m.world)) * q * glm::quat_cast(m.body));
}

glm::dvec3 position_to_ned(const FrameMap& m, const glm::dvec3& p, const glm::dvec3& home) { return world_to_ned(m, p - home); }

geo::Geodetic ned_to_geodetic(const geo::LocalFrame& frame, const glm::dvec3& ned) {
    // NED (north, east, down) is the tangent plane's (east, north, up) = (E, N, -D)
    return geo::ecef_to_geodetic(geo::enu_to_ecef(frame, glm::dvec3(ned.y, ned.x, -ned.z)));
}

glm::dvec3 geodetic_to_ned(const geo::LocalFrame& frame, const geo::Geodetic& g) {
    const glm::dvec3 enu = geo::ecef_to_enu(frame, geo::geodetic_to_ecef(g));
    return glm::dvec3(enu.y, enu.x, -enu.z);
}

}  // namespace spade::fc_link::frames
