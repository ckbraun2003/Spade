#include "fc_link/frames.hpp"

// RED stub: every function returns zeros; tests/test_fc_link_frames.cpp fails against it.
namespace spade::fc_link::frames {

FrameMap compass_x_north() { return {}; }
FrameMap anchored_world() { return {}; }
glm::dvec3 world_to_ned(const FrameMap&, const glm::dvec3&) { return glm::dvec3(0.0); }
glm::dvec3 ned_to_world(const FrameMap&, const glm::dvec3&) { return glm::dvec3(0.0); }
glm::dvec3 body_to_frd(const FrameMap&, const glm::dvec3&) { return glm::dvec3(0.0); }
glm::dvec3 frd_to_body(const FrameMap&, const glm::dvec3&) { return glm::dvec3(0.0); }
glm::dquat attitude_to_ned_frd(const FrameMap&, const glm::dquat&) { return glm::dquat(0.0, 0.0, 0.0, 0.0); }
glm::dquat attitude_from_ned_frd(const FrameMap&, const glm::dquat&) { return glm::dquat(0.0, 0.0, 0.0, 0.0); }
glm::dvec3 position_to_ned(const FrameMap&, const glm::dvec3&, const glm::dvec3&) { return glm::dvec3(0.0); }
geo::Geodetic ned_to_geodetic(const geo::LocalFrame&, const glm::dvec3&) { return {}; }
glm::dvec3 geodetic_to_ned(const geo::LocalFrame&, const geo::Geodetic&) { return glm::dvec3(0.0); }

}  // namespace spade::fc_link::frames
