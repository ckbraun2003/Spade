#pragma once

// ---------------------------------------------------------------------------
// world/mesh_shapes.hpp -- the library drones' shapes, generated (L7a of the
// realistic drones plan, docs/design/core/plans/
// 2026-10-26-l7-realistic-drones-draft.md). Our own geometry from published
// numbers, never downloaded CAD: a rounded body shell, a capped arm tube, a
// motor bell, and a lofted propeller in both hands.
//
// EACH IS A PLAIN FUNCTION from numbers to make_mesh() (world/mesh.hpp), so
// every mesh it returns has passed make_mesh()'s checks: closed, oriented
// outward, 2-manifold, with no self-intersection. A field out of its range,
// or a count past its limit (THE LIMITS below), is refused by name, before
// anything is built. What only the built mesh shows (a sliver of a leading
// edge at a very fine chordwise spacing, blades so many or so wide that
// their roots meet, a gap or a radius below fp32's resolution) comes back
// in make_mesh()'s own words. The positions are metres (MeshUnit::metre),
// built in double and rounded to fp32 once, by make_mesh().
//
// THE LIMITS bound every count before any allocation, so no product of them
// overflows a 32-bit index: a drone's shapes need far less.
//
// TD-3. Circles and pitch angles go through math::sin32 / math::cos32
// (core/fp32_math.hpp), always inside [0, 2 pi] or (-pi/2, pi/2), their
// 1-ulp domain. Everything else is + - * / and sqrt, in double, in a pinned
// order, so MSVC and gcc build the same bits (TD-12): tests/test_mesh_shapes.cpp
// pins each shape's content_hash(), and the Docker leg reproduces it.
//
// WELDED BY CONSTRUCTION. make_mesh() refuses two vertices at one position,
// so a seam reuses its first vertex and a pole is one vertex.
//
// THE FRAMES.
//   rounded_box:  centred on the origin, extents exactly `size`.
//   capped_tube:  along +Y, centred on the origin.
//   motor_bell:   the HUB frame: the top face is the rotor plane, y = 0, and
//                 the bell hangs below it to y = -height, about the +Y axis.
//   prop:         the HUB frame: the disc plane is y = 0, thrust is +Y, and
//                 blade 0 lies along +X at psi = 0. A rotor turns by
//                 R_y(spin_dir psi) (vehicles/part_pose.hpp), and a +psi turn
//                 carries +X toward -Z, so the ccw hand (spin_dir +1) has
//                 blade 0's leading edge on its -z side, above its trailing
//                 edge; the cw hand (spin_dir -1) is its mirror image.
//
// THE PROPELLER.
//   * Its blade is lofted through the part file's stations (BLADE,
//     scene/part_file.hpp): at each r/R, a section of chord c/R * R, pitched
//     by beta about its quarter chord, in the plane x = r (blade 0). The
//     quarter-chord line runs straight along the radius: no sweep, no rake.
//   * The section is NACA 4-digit-style: the camber line and the thickness
//     y_t(x) = 5 t (0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3
//     - 0.1015 x^4), with the thickness laid normal to the chord line, not
//     the camber line (no atan). Its trailing edge is the formula's own
//     blunt edge, 2 y_t(1) = 0.021 t of the chord, so no two surfaces meet
//     at a knife edge. (The plan's floor, max(1% c, 0.1 mm), is not used:
//     at a small propeller's tip it is thicker than the whole section --
//     mit_5x4's tip is 0.71 mm of chord and t c = 0.057 mm.) The chordwise
//     points sit at x = (i / n)^2, close at the leading edge, which is one
//     vertex. t runs linearly in r/R from thickness_root to thickness_tip.
//   * A cap closes each end of the blade in strips that pair the upper and
//     lower points at one x.
//   * The hub is a short cylinder, its own shell, of radius hub_cutout R -
//     hub_gap. The blades are other shells. make_mesh() refuses shells that
//     touch, so the hub cannot hold a blade root sunk into it: hub_gap keeps
//     them apart.
//   * The cw hand is the ccw mesh with every z negated and every triangle's
//     last two indices swapped, so it faces outward; its positions are the
//     ccw positions mirrored bit for bit.
//   * A propeller is never a solid (vehicles/part_pose.hpp, PartFilter): it
//     is drawn, so the airflow solids' 8 mm rule does not apply to it.
//
// MIN_FEATURE is each mesh's thinnest dimension, as a part's min_feature
// means it (vehicles/model_part.hpp): the smallest extent of a box, the
// diameter of a tube, the lesser of a bell's diameter and height, and a
// propeller's thinnest section, t c at its thinnest station.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "world/mesh.hpp"

namespace spade::shapes {

// THE LIMITS (above).
inline constexpr uint32_t kMaxBoxSegments = 64;     // rounded_box: steps across a rounded band
inline constexpr uint32_t kMaxRingSegments = 1024;  // chords around a tube, a bell or a hub
inline constexpr uint32_t kMaxChordPoints = 64;     // a blade section's points on each surface
inline constexpr uint32_t kMaxPropBlades = 16;
inline constexpr std::size_t kMaxPropStations = 256;

// A generated mesh and its thinnest dimension (MIN_FEATURE above), metres.
struct GeneratedMesh {
    std::shared_ptr<const TriangleMesh> mesh{};
    float min_feature = 0.0f;
};

// A box of extents `size`, its every edge and corner rounded at
// `edge_radius`: the inner box, size - 2 edge_radius, swept by a sphere.
// `segments` is the steps across each rounded band (at least 1), so a
// rounded edge's quarter circle takes 2 segments chords. The flat faces lie
// exactly at +-size / 2. Refused: a size not finite and above 0; an
// edge_radius not above 0 and below half the smallest extent; segments not
// in [1, kMaxBoxSegments].
[[nodiscard]] Result<GeneratedMesh> rounded_box(glm::dvec3 size, double edge_radius, uint32_t segments);

// A capsule along +Y: a cylinder of `radius` whose cap centres are `length`
// apart, closed by two hemispheres, `segments` chords around (at least 3)
// and max(2, segments / 4) rings in each cap. Refused: a radius or length
// not finite and above 0; segments not in [3, kMaxRingSegments].
[[nodiscard]] Result<GeneratedMesh> capped_tube(double radius, double length, uint32_t segments);

// A motor bell in the hub frame: a cylinder of `radius` from y = 0 down to
// y = -height, its top edge chamfered by `chamfer`, `segments` chords around
// (at least 3). Refused: a radius or height not finite and above 0; a
// chamfer not above 0 and below the lesser of the two; segments not in
// [3, kMaxRingSegments].
[[nodiscard]] Result<GeneratedMesh> motor_bell(double radius, double height, double chamfer, uint32_t segments);

// The blade section (THE PROPELLER above), as fractions of the chord.
struct PropSection {
    double camber = 0.0;          // the camber line's height, [0, 0.2)
    double camber_at = 0.0;       // where it peaks, in (0, 1)
    double thickness_root = 0.0;  // t at the first station, in (0, 0.5)
    double thickness_tip = 0.0;   // t at the last station, in (0, 0.5)
    uint32_t chord_points = 0;    // points on each surface after the leading edge, at least 2
};

// A propeller: the part file's blade (scene/part_file.hpp's BladeGeometry)
// and the propeller's own numbers, plus what the mesh assumes.
struct PropMeshSpec {
    double diameter = 0.0;  // m
    uint32_t blades = 0;
    std::vector<double> r_over_r{};  // strictly increasing in (0, 1], from hub_cutout
    std::vector<double> c_over_r{};  // each above 0
    std::vector<double> beta_deg{};  // each in (-90, 90)
    double hub_cutout = 0.0;         // r/R
    double hub_gap = 0.0;            // m, between the hub and each blade's root, above 0
    double hub_height = 0.0;         // m, the hub cylinder's height, above 0
    uint32_t hub_segments = 0;       // chords around the hub, at least 3
    PropSection section{};
};

// The two hands: ccw goes on a rotor of spin_dir +1, cw on spin_dir -1.
enum class PropHand : uint8_t { ccw, cw };

// The propeller's mesh, in the hub frame (THE FRAMES above). Refused, by the
// field:
//   - a diameter not finite and above 0;
//   - blades not in [1, kMaxPropBlades];
//   - stations not in [2, kMaxPropStations], or lists of different lengths;
//   - a hub_cutout outside [0, 1);
//   - an r/R not strictly increasing in (0, 1], or starting below hub_cutout;
//   - a c/R not above 0, or a beta outside (-90, 90) degrees;
//   - a hub_gap not above 0, or leaving no hub (hub_cutout R - hub_gap);
//   - a hub_height not above 0, or hub_segments not in [3, kMaxRingSegments];
//   - a section out of its ranges, or chord_points not in [2, kMaxChordPoints].
[[nodiscard]] Result<GeneratedMesh> prop(const PropMeshSpec& spec, PropHand hand);

}  // namespace spade::shapes
