#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// SOLIDS IN THE GRID (2026-10-13-airflow-solids-design.md, sections 2-4; spec
// section 2). Every part reaches the grid through a sampler the caller
// provides: vehicles::sample_parts() adapted, in the tests, the D1 demo and
// plan C's module. The airflow core includes nothing from vehicles/, which
// links it, and computes no bounds (TD-9): each part arrives with its posed box
// and its part-frame extent (Core's part_bounds()).
//
// Per velocity sample (u, v, w arrays, MacGrid's indexing):
//   theta  the face's open fraction. A wall part: clamp(0.5 + phi / dx, 0, 1)
//          (Batty, Bertails & Bridson 2007). A sheet: 0 where its mid-surface
//          crosses the segment between the face's two cell centres, half-open
//          (-dx/2, dx/2] along the face's axis, so a sheet on a cell-centre
//          plane closes exactly one face; otherwise 1.
//   wall   the owning part's rigid velocity, this component, at the sample.
//   phi    a wall part's signed distance, negative inside; a sheet's
//          (never negative) distance to its mid-surface; max off solids.
//   link   the viscous solve's wall crossings (Task 8), or kNoWallLinks.
// Only samples within the parts' posed boxes grown by 2 dx are sampled; every
// other face stays open.
//
// THE MEASURED SAFEGUARD (design section 4, as ruled 2026-10-13). A part the
// classifier calls a wall is measured on the grid it meets:
//   (A) a blob: it closes no face (no sample it wins has theta = 0);
//   (B) leaking runs: along each axis, a run joins two cell centres outside it
//       with k >= 0 centres inside between; for k = 0 the Lipschitz screen
//       phi1 + phi2 <= dx and a sphere-traced hit admit it. Traces from both
//       ends give the entry, the exit and their normals; the crossing is thin
//       when n_in . n_out <= -0.5 and tau = chord |n_in . e| < 2 dx, and a thin
//       crossing whose k + 1 faces are all open leaks, its area the smallest
//       theta times dx^2.
// A part that is not a blob, whose every leaking run lies within 4 cells
// (Chebyshev) of a cell with a closed face and whose leaks total at most 2%
// of its bounds' largest face, leaks at an edge: announced on the run line. A
// blob is sheet-like when it has a thin crossing and every thin crossing is
// WIDE (the lead, 2026-10-13: the criterion is local): along each tangent of
// its entry normal, a probe 2 dx from its mid-point, one way or the other,
// lands within dx/2 of the part. A sheet, flat or curved, is wide; a wire is
// narrow across itself and a bead every way. It is sheeted at its largest
// thin tau, announced, and sampled again. Any other leak -- a blob thin in
// more than one direction (a wire, a bead) included -- is refused by name:
// declare min_feature or refine dx.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kNoSolidPart = std::numeric_limits<uint32_t>::max();
inline constexpr uint32_t kNoWallLinks = std::numeric_limits<uint32_t>::max();

// What the geometry reports at a point of the grid's frame (metres).
struct SolidQuery {
    float distance = std::numeric_limits<float>::max();  // m, negative inside; max: no part
    glm::vec3 gradient{0.0f};                             // unit, out of the solid
    uint32_t part = kNoSolidPart;                         // index into the SolidPart list
    glm::vec3 velocity{0.0f};                             // the owner's rigid velocity at the point, m/s
};

// A non-owning view of the geometry. `nearest` is sample_parts()'s rule: the
// smallest distance wins, a tie to the lower part. `part` evaluates one part
// alone (the measured safeguard traces each part by itself).
struct SolidSampler {
    const void* ctx = nullptr;
    SolidQuery (*nearest)(const void* ctx, glm::vec3 x) noexcept = nullptr;
    SolidQuery (*part)(const void* ctx, uint32_t part, glm::vec3 x) noexcept = nullptr;
};

// One part as the grid sees it.
struct SolidPart {
    std::string name{};
    bool porous = false;
    float min_feature = 0.0f;  // m, declared (CORE-8 D5); 0: undeclared
    glm::vec3 extent{0.0f};    // m, its part-frame bounding box's size (vehicles::part_bounds())
    glm::vec3 box_min{0.0f};   // its posed bounding box in the grid's frame, m
    glm::vec3 box_max{0.0f};
};

enum class SolidRole : uint8_t { wall, sheet };

struct SolidClass {
    SolidRole role = SolidRole::wall;
    float thickness = 0.0f;  // a sheet's, m
};

// THE CLASSIFIER (design section 4; TD-9: Interface's resolve() calls this,
// not a rule of its own). A part is thin when min(declared min_feature, its
// smallest extent) < 2 dx; a thin part whose middle extent is also below 2 dx
// is refused (a wire or a fastener), any other thin part is a sheet of that
// thickness. Refused (invalid_argument, naming the part): a porous part (the
// porous task, design section 5); a non-finite or negative min_feature or
// extent; a wire, with the dx it needs.
[[nodiscard]] Result<SolidClass> classify_part(const SolidPart& part, float dx);

// The viscous solve's wall crossings at one sample (Task 8): per neighbour,
// -x, +x, -y, +y, -z, +z, 1 / f for a link the wall crosses at the fraction f
// of its length (else 0), and the wall's velocity there.
struct WallLinks {
    std::array<float, 6> inv_f{};
    std::array<float, 6> wall{};
};

// One component's sampled box, inclusive sample indices per axis.
struct SolidRange {
    std::array<uint32_t, 3> lo{};
    std::array<uint32_t, 3> hi{};
    bool empty = true;
};

struct SolidField {
    bool any = false;  // false: no part, every face open (plan A's paths)
    std::array<std::vector<float>, 3> theta;
    std::array<std::vector<float>, 3> wall;
    std::array<std::vector<float>, 3> phi;
    std::array<std::vector<uint32_t>, 3> link;
    std::vector<WallLinks> links;
    std::array<std::vector<uint32_t>, 3> fill1;  // the free-slip fill's first layer (Task 7)
    std::array<std::vector<uint32_t>, 3> fill2;  // and its second
    std::array<SolidRange, 3> range;             // per component, the sampled box
    std::vector<SolidClass> classes;             // per part
    std::string run_line;
};

[[nodiscard]] inline bool closed(float theta) noexcept { return theta == 0.0f; }

// Every array sized for `shape`, every face open. capacity_exceeded past
// kGridByteCap (16 bytes per face), invalid_argument for a shape make_grid()
// refuses.
[[nodiscard]] Result<SolidField> make_solid_field(const GridShape& shape);

// Rebuilds `field` from `parts` through `sample` (a host call; it allocates
// the link list and the fill layers). Each part is classified, every part's
// box grown by 2 dx is clipped to the grid, and every sample in the union of
// those boxes is sampled. On a periodic axis the last face plane copies the
// first. Refused (invalid_argument, naming what), with `field` left with every
// face open: classify_part()'s refusals; a part wholly outside the grid; a
// sampler that reports a non-finite distance, gradient or velocity, or
// a part past the list.
[[nodiscard]] Result<void> build_solids(SolidField& field, const GridShape& shape, std::array<bool, 3> periodic,
                                        std::span<const SolidPart> parts, const SolidSampler& sample);

}  // namespace spade::physics::airflow
