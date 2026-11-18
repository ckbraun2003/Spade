#pragma once

// ---------------------------------------------------------------------------
// THE ACTUATOR LINE ON THE GPU (P4c, L61; the plan .superpowers/sdd/2026-11-10-wake/p4c-plan.md): the host's half.
// The device twin of engine/physics/airflow/line.hpp's spread_line(), bit for bit: three kernels (airflow_line_sample,
// airflow_line_load, airflow_line_spread) over the four buffers bindings.slang declares (airflow_line_elems, _polar,
// _vel, _force). The binding carries DATA, so a change to the CPU model's smearing or sampling is a change here, not a
// new binding or pipeline: the host packs, for every element of every rotor at every sub-azimuth, what is geometry (the
// stencil's first face, tap count and weights per component and axis, from line_stencil(), the CPU's own exp32 and all)
// and the kernels do the sums the CPU does, in the CPU's order.
//
// THE ELEMENT RECORD, kLineStride floats, in the CPU's order (rotor, sub-azimuth, blade, element; a rotor's own call
// order):
//   [0-2] the rotor's axis   [3-5] the element's motion   [6] chord   [7] sin beta   [8] cos beta   [9] r
//   [10] dr * share (share = 1 / n_sub)   [11] omega   [12] h / (rho dx^3)   [13] rho   [14] its tip group + 1 (0: none)   [15] its place in the blade
//   then, for component c = 0, 1, 2 at kLineHeaderWords + c * kLineComponentWords:
//   [0-2] lo of axes x, y, z (a float, exact)   [3-5] n, the taps of each axis
//   [6] S, the product of the axes' weight sums   [7 + kLineTaps * a + t] the weight of tap t of axis a.
// THE TIP CORRECTION'S GROUPS (L72k's A1; airflow_line_tipcorr), in the same buffer after the element records, from word
// elements * kLineStride: kTipHeaderWords floats (word 0 the group count, a float, exact), then kTipStride floats a group (a blade at
// a sub-azimuth of a rotor whose tip_correction is on and that turns), in the CPU's order:
//   [0] first element (global)  [1] per (elements a blade)  [2] iterations  [3] segments  [4] reach  [5] spin  [6] dr  [7] r_in
//   [8] eps_m  [9] kern sum  [10] relax  [11] omega  [12] rho   [16-18] centre  [19-21] d, the blade's direction  [22-24] the motion
//   [25-27] the axis   [32 + j] th_j   [64 + j] cos th_j   [96 + j] sin th_j (j <= segments)   [128 + m] the span filter, m <= reach
// The tables are line_tip_tables()'s own floats (physics/airflow/line.hpp): the kernel computes none of them.
// THE POLAR (airflow_line_polar): [0] n (a float, exact), then the n diamond keys, the n C_l, the n C_d (the CPU's own
// arrays).
// THE FORCE (airflow_line_force), kLineForceStride floats an element: [0-2] the force on the air, already x dr * share;
// [3] the blade's axial load per metre; [4] its tangent load per metre; the rest spare. The host sums thrust and torque
// from [3] and [4] in double, in element order, as spread_line() does.
//
// Packed under the CPU's flush (core/detail/flush_subnormals.hpp), where spread_line() computes the same taps.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace spade::airflow_gpu {

inline constexpr uint32_t kLineTaps = static_cast<uint32_t>(physics::airflow::kLineTaps);
inline constexpr uint32_t kLineHeaderWords = 16;
inline constexpr uint32_t kLineComponentWords = 7 + 3 * kLineTaps;
inline constexpr uint32_t kLineStride = kLineHeaderWords + 3 * kLineComponentWords;
inline constexpr uint32_t kLineForceStride = 8;
inline constexpr uint32_t kTipHeaderWords = 16;
inline constexpr uint32_t kTipStride = 160;
inline constexpr uint32_t kTipMaxSegments = 31;  // the tables' 32-float rows
inline constexpr uint32_t kTipMaxReach = 31;
// The host's bound on a blade's elements; the kernel keeps no per-blade array (one thread an element).
inline constexpr uint32_t kTipMaxPer = 128;

struct LineRecords {
    std::vector<float> elems;          // kLineStride floats an element, every rotor's elements in order
    std::vector<float> tip;            // the tip correction's header and groups (kTipHeaderWords + groups * kTipStride)
    std::vector<uint32_t> per_rotor;   // each rotor's element count
};

// The records of `rotors` for a step of h seconds in `n_sub` sub-azimuths, each rotor checked as spread_line()
// checks it (refused with its Error, nothing packed).
[[nodiscard]] Result<LineRecords> line_records(std::span<const physics::airflow::LineRotor> rotors,
                                               const physics::airflow::GridShape& shape, float rho, float h,
                                               uint32_t n_sub);

// THE ROTOR BOXES (L86i; airflow_line_spread_binned), kLineRangeWords floats a rotor: [0] the rotor's first element (global), [1] its element
// count, then for component c = 0, 1, 2 at 2 + 6 c: the lo (x, y, z) of the faces its elements' stencils reach, then the hi (exclusive).
// All floats, exact integers. A rotor with no elements has count 0.
inline constexpr uint32_t kLineRangeWords = 20;
[[nodiscard]] std::vector<float> line_ranges(const LineRecords& records);

// airflow_line_polar's floats for a polar.
[[nodiscard]] std::vector<float> polar_records(const physics::airflow::PolarTable& polar);

// Each rotor's report from the force buffer's floats (kLineForceStride an element, in the records' order):
// spread_line()'s own sums.
[[nodiscard]] std::vector<physics::airflow::LineReport> line_reports(
    const LineRecords& records, std::span<const physics::airflow::LineRotor> rotors, std::span<const float> force);

}  // namespace spade::airflow_gpu
