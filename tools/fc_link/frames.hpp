#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/geodesy.hpp"

// ---------------------------------------------------------------------------
// fc_link/frames.hpp -- Spade's frames against the autopilots' (docs/design/physics/plans/2026-11-21-fc-physics.md section 2, which
// owns the table of vectors this is tested against; docs/design/interface/plans/2026-11-21-fc-links.md section 2).
//
// Spade is right-handed with +Y up, and its design frame (the model files') has +X forward and +Z right. PX4 and ArduPilot use NED for
// the world (North, East, Down) and FRD for the body (Forward, Right, Down). The body map is the same for every drone:
//
//     (x, y, z) -> (x, z, -y)           determinant +1, a proper rotation
//
// so vectors, angular rates and a quaternion's vector part all go through one matrix. The WORLD map is a convention that depends on how a
// scene lays out its compass; two are named here:
//     compass_x_north   North is Spade's +X, East is +Z (the same matrix as the body's; Physics' table, the default for a world with no
//                       anchor);
//     anchored_world   a world with a geo anchor (core/geodesy.hpp): x east, y up, z south, so North = -z, East = +x.
// Orientation is body-to-world, so it takes BOTH maps: q' = Q(world) * q * Q(body)^-1. With the two maps equal that is the familiar
// R q R^-1, i.e. (w, x, y, z) -> (w, x, z, -y). The result is given the sign with w >= 0 (q and -q are one rotation); note that a consumer
// that DIFFERENCES successive quaternions sees a sign jump whenever w crosses 0 (harmless for HIL_STATE_QUATERNION and the JSON attitude).
//
// Pure functions: no engine state, no I/O, double precision.
// ---------------------------------------------------------------------------

namespace spade::fc_link::frames {

struct FrameMap {
    glm::dmat3 world{1.0};  // Spade world vector -> NED
    glm::dmat3 body{1.0};   // Spade body vector -> FRD
};

[[nodiscard]] FrameMap compass_x_north();
[[nodiscard]] FrameMap anchored_world();

// Vectors (position offsets, velocities, specific force, angular rates, fields), Spade -> autopilot and back.
[[nodiscard]] glm::dvec3 world_to_ned(const FrameMap& m, const glm::dvec3& v);
[[nodiscard]] glm::dvec3 ned_to_world(const FrameMap& m, const glm::dvec3& v);
[[nodiscard]] glm::dvec3 body_to_frd(const FrameMap& m, const glm::dvec3& v);
[[nodiscard]] glm::dvec3 frd_to_body(const FrameMap& m, const glm::dvec3& v);

// Body-to-world orientation, Spade -> body-FRD-to-NED and back (w >= 0).
[[nodiscard]] glm::dquat attitude_to_ned_frd(const FrameMap& m, const glm::dquat& q);
[[nodiscard]] glm::dquat attitude_from_ned_frd(const FrameMap& m, const glm::dquat& q);

// A position in the autopilot's NED frame about a home (the Spade world point that is North 0, East 0, Down 0).
[[nodiscard]] glm::dvec3 position_to_ned(const FrameMap& m, const glm::dvec3& p, const glm::dvec3& home);

// NED metres about the anchor's tangent plane to latitude, longitude and height (core/geodesy.hpp's exact conversion, the same bits
// on every platform), and back. The height is the anchor's ellipsoid height plus -Down: the geoid is ignored, so the "MSL" of
// HIL_GPS is the anchor's height (a documented choice: PX4 needs the GPS and barometer datum to agree, not the geoid).
[[nodiscard]] geo::Geodetic ned_to_geodetic(const geo::LocalFrame& frame, const glm::dvec3& ned);
[[nodiscard]] glm::dvec3 geodetic_to_ned(const geo::LocalFrame& frame, const geo::Geodetic& g);

}  // namespace spade::fc_link::frames
