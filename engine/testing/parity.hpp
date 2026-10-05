#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "state/arenas.hpp"
#include "state/registry.hpp"

// ---------------------------------------------------------------------------
// parity.hpp (S6 Task 6) -- THE CPU<->GPU COMPARISON, per quantity, per
// element, against a table of measured-then-pinned tolerance bands.
//
// TEST SUPPORT, NOT ENGINE API, exactly like replay.hpp and scenario_file.hpp
// beside it: header-only, compiled into spade_tests only, never installed,
// never exported, and allowed to be convenient (std::string, std::vector,
// printf) in ways the engine is not.
//
// ===========================================================================
// WHY BANDS AND NOT BIT EQUALITY -- the one question this file has to answer
// before any number in it means anything.
//
// The engine's CPU determinism contract IS bit equality: two runs of the same
// scenario on the same build produce byte-identical state, which is what the
// golden digests pin. CPU<->GPU is deliberately NOT that, and the reason is a
// standing global constraint rather than a concession:
//
//     "Vulkan OpFDiv and sqrt are <= 2.5-ulp (NOT correctly rounded): the
//      fp32_math Slang module must contain NO OpFDiv and NO sqrt ...
//      downstream kernels MAY use div/sqrt -- that is why CPU<->GPU is banded
//      rather than bit-identical."
//
// Every ported kernel's header lists its own div and sqrt sites against the
// CPU expression each one mirrors. The transcendentals are NOT a contributor:
// sin32/cos32/exp32/log32 come from the T4 Slang port and are proven
// bit-identical to the host on this device (tests/test_gpu_fp32_math.cpp). So
// the divergence this file measures is exactly the accumulated <= 2.5-ulp
// error of a bounded number of divides and square roots per substep, compounded
// over the run by the integrator -- which is why a band is a MEASUREMENT, and
// why it grows with step count and with how chaotic the scenario is.
//
// ===========================================================================
// MEASURE FIRST, THEN PIN. THE ORDER IS THE METHOD.
//
// The global constraint: "Tolerance bands are measured then pinned with margin
// (spec section 9): calibrated on the Iris, committed as constants with
// provenance comments (device, driver, measured max, margin factor). Bands are
// NEVER widened to make a failure pass without a root-caused coordinator
// ruling."
//
// compare_arrays() therefore ALWAYS reports the measured maxima, whether or not
// they are inside the band, and a caller prints them. The bands at the bottom
// of this file each carry the measurement they were derived from. Reading a
// band as "how much error is acceptable" is backwards: it is "four times what
// this device actually produced, so a real regression moves it and thermal
// noise does not".
//
// ===========================================================================
// THE PREDICATE, STATED PRECISELY.
//
// An element passes iff EITHER its absolute error is within `abs` OR its
// relative error is within `rel`. Not both, and the disjunction is deliberate:
//   * near zero, a relative error is meaningless (a position component passing
//     through 0 makes every rel enormous while the absolute error is an ulp),
//     so `abs` has to be able to carry it alone;
//   * far from zero, an absolute band would have to be scaled to the largest
//     value any scenario reaches, which would make it useless for the small
//     ones, so `rel` has to be able to carry it alone.
// Both maxima are reported regardless, so the report never hides which one is
// doing the work.
//
// THE RELATIVE HALF IS INAPPLICABLE, NOT SATISFIED, WHEN THE CPU VALUE IS
// EXACTLY ZERO (S6 Task 6 review round 1, finding C2 -- and the one place a
// disjunction like this is easy to get quietly wrong). There is nothing to
// divide by, so no relative error exists; treating the resulting placeholder 0
// as "within the relative band" made every cpu == 0 component pass
// unconditionally, band or no band. The absolute half carries that regime
// alone, which is exactly what a band of 0 abs is asking for: bit equality.
// See the predicate at the comparison site for the three checks that hole
// silently defeated.
//
// A NaN on either side is a FAILURE, never a pass: the comparison is spelled so
// that a NaN error fails both halves of the disjunction (`!(x <= band)` rather
// than `x > band`), the same NaN-safe spelling physics/contacts.cpp uses for
// its own guards.
//
// ===========================================================================
// QUATERNIONS get the component bands like everything else PLUS a unit-norm
// check on both sides. The reason is that four component errors can be within
// band individually and still describe a non-unit quaternion, which is a
// different KIND of defect -- a renormalization that did not happen, rather
// than a rounding difference -- and it would go on to scale every subsequent
// rotation. glm::normalize is the last thing integrate_orientation does on both
// paths (math_ops.cpp:47 and integrate.slang's glm_quat_normalize), so
// |q| - 1 is a direct check on that step surviving the port.
// ---------------------------------------------------------------------------

namespace spade::testing {

// ---------------------------------------------------------------------------
// One quantity's tolerance. Both halves are absolute numbers, not ulps: an ulp
// count would have to be evaluated against a magnitude that varies across the
// run, and the whole point of the pair below is that one half covers the
// near-zero regime and the other the large-magnitude one.
// ---------------------------------------------------------------------------
struct ToleranceBand {
    float abs = 0.0f;
    float rel = 0.0f;
};

enum class QuantityKind : uint32_t {
    // Plain float components, compared one by one.
    components = 0,
    // Four float components (x, y, z, w) that additionally must describe a
    // UNIT quaternion on both sides -- see the header note.
    quaternion = 1,
    // ------------------------------------------------------------------
    // RAW 32-BIT WORDS, COMPARED FOR EXACT EQUALITY (S6 Task 8).
    //
    // WHY A BANDED COMPARISON NEEDS AN UNBANDED KIND. Wave C ports the rng
    // (engine/shaders/rng.slang) and the sensor ring, and their state is
    // INTEGER: a splitmix64 stream position, a Box-Muller cache FLAG, a rate-
    // divider phase counter, a monotonic sample index, a tick stamp. None of
    // those is approximately right -- a stream one draw out of step produces a
    // completely different (and perfectly plausible) sequence from then on, and
    // no tolerance on a float could express "the two runs have drawn the same
    // number of values".
    //
    // They are also not COMPARABLE as floats. Reinterpreting a stream word as
    // a float lands on arbitrary magnitudes, infinities and NaNs, so
    // `components` would report a NaN failure on a perfectly healthy stream.
    //
    // So this kind loads uint32 lanes, demands bit equality, and reports the
    // count of differing words -- see compare_arrays() for how the report's
    // float-shaped fields carry that. A row of this kind must be given a band
    // of {0, 0}: there is nothing for a tolerance to mean.
    //
    // A 64-BIT FIELD IS TWO LANES, low word first, exactly as
    // shaders/shared/layouts.slang mirrors it (little-endian, engine-wide).
    // ------------------------------------------------------------------
    bits = 2,
};

// ---------------------------------------------------------------------------
// One row of the table: which array, which quantity inside its element, and the
// band it is held to.
//
// ADDRESSED BY BYTE OFFSET, not by a member pointer, because compare_arrays()
// walks the registry GENERICALLY -- it never names BodyState (or any other
// element type) and never includes its header, exactly as
// compute/vulkan/state_mirror.cpp does not. The offsets come from
// offsetof() at the call site, where the type IS in scope, so nothing here is a
// hand-typed number.
// ---------------------------------------------------------------------------
struct BandEntry {
    std::string_view array;     // registered array name, e.g. "bodies"
    std::string_view quantity;  // human name for the report, e.g. "pos"
    std::size_t offset = 0;     // byte offset of the first component within one element
    uint32_t components = 1;    // how many consecutive floats
    QuantityKind kind = QuantityKind::components;
    ToleranceBand band{};
};

using BandTable = std::span<const BandEntry>;

// What compare_arrays() measured for one table row.
struct QuantityReport {
    std::string array;
    std::string quantity;
    ToleranceBand band{};

    float max_abs = 0.0f;  // largest |cpu - gpu| over every compared component
    float max_rel = 0.0f;  // largest |cpu - gpu| / |cpu|, over components with cpu != 0
    // TD-14's measurement, split at kNearZeroCutoff: A, the largest
    // |cpu - gpu| where |cpu| < the cutoff, and R, the largest relative error
    // where |cpu| >= it. A band is pinned at {4A, 4R}, each rounded up to one
    // significant figure (pin_of()), the larger over every device of record.
    float near_zero_abs = 0.0f;  // A
    float far_rel = 0.0f;        // R

    // Where the largest ABSOLUTE error was, and what the two sides held there.
    std::size_t worst_element = 0;
    uint32_t worst_component = 0;
    float worst_cpu = 0.0f;
    float worst_gpu = 0.0f;

    // quaternion rows only: the largest ||q| - 1| seen on EITHER side. The
    // flag is carried on the report rather than re-read from the table so
    // print() below needs only the report.
    bool kind_is_quaternion = false;
    float max_unit_norm_error = 0.0f;

    // bit-exact rows only (QuantityKind::bits): carried so print() can LABEL
    // the row rather than showing a pinned band of (0, 0) that a reader would
    // mistake for "measured bit-exact, pinned there" -- a different and much
    // weaker claim than "this quantity is integer and admits no band at all".
    bool kind_is_bits = false;

    std::size_t elements_compared = 0;
    std::size_t elements_outside_band = 0;
    bool nan_seen = false;

    [[nodiscard]] bool within_band() const noexcept { return elements_outside_band == 0 && !nan_seen; }
};

// TD-14's near-zero cutoff s_q, in the quantity's SI unit: below it an
// element's error counts toward A (absolute), at or above it toward R
// (relative). One value for every quantity, as the policy states it.
inline constexpr float kNearZeroCutoff = 1.0e-3f;

// TD-14's pin of a measured maximum: 4x, rounded UP to one significant
// figure (4 * 2.3e-6 = 9.2e-6 -> 1e-5). 0 stays 0: a quantity measured
// exactly equal goes to the floor or to an argued exact band, not to 4 * 0.
[[nodiscard]] inline float pin_of(float measured) noexcept {
    if (!(measured > 0.0f)) return 0.0f;
    const double x = 4.0 * static_cast<double>(measured);
    const double decade = std::pow(10.0, std::floor(std::log10(x)));
    double digit = std::ceil(x / decade - 1e-9);
    return static_cast<float>(digit * decade);
}

struct ParityReport {
    std::vector<QuantityReport> quantities;

    [[nodiscard]] bool clean() const noexcept {
        for (const QuantityReport& q : quantities) {
            if (!q.within_band()) return false;
        }
        return true;
    }

    // The report as a fixed-width table, one row per quantity. PRINTED BY THE
    // TEST, ALWAYS -- passing or failing. The measured columns are the
    // checkpoint's actual deliverable ("the parity tables (measured vs pinned
    // bands) are the centrepiece"), and a table printed only on failure would
    // be a table nobody ever reads.
    //
    // %.*e with 9 significant digits, matching the house fp32 text rule --
    // these are floats and a truncated decimal would misreport a band by more
    // than the margin it was pinned with.
    void print(std::string_view label) const {
        std::printf("\n=== CPU<->GPU parity: %.*s ===\n", static_cast<int>(label.size()), label.data());
        std::printf("%-14s %-16s %6s  %-15s %-15s  %-15s %-15s  %s\n", "array", "quantity", "elems",
                    "measured |abs|", "measured rel", "pinned abs", "pinned rel", "verdict");
        for (const QuantityReport& q : quantities) {
            if (q.kind_is_bits) {
                // A bit-exact row has no band to print; saying BIT-EXACT (or
                // naming the count that broke it) is the whole verdict.
                std::printf("%-14s %-16s %6zu  %-15s %-15s  %-15s %-15s  %s\n", q.array.c_str(),
                            q.quantity.c_str(), q.elements_compared,
                            q.within_band() ? "0 (exact)" : "WORDS DIFFER", "n/a", "bit-exact", "n/a",
                            q.within_band() ? "within" : "OUTSIDE");
            } else {
                std::printf("%-14s %-16s %6zu  %-15.8e %-15.8e  %-15.8e %-15.8e  %s\n", q.array.c_str(),
                            q.quantity.c_str(), q.elements_compared, static_cast<double>(q.max_abs),
                            static_cast<double>(q.max_rel), static_cast<double>(q.band.abs),
                            static_cast<double>(q.band.rel), q.within_band() ? "within" : "OUTSIDE");
                // The TD-14 measurement line, grep-able as "td14 ".
                std::printf("    td14 A=%.3e R=%.3e  pin {%.0e, %.0e}\n", static_cast<double>(q.near_zero_abs),
                            static_cast<double>(q.far_rel), static_cast<double>(pin_of(q.near_zero_abs)),
                            static_cast<double>(pin_of(q.far_rel)));
            }
            if (!q.within_band()) {
                std::printf("    worst: element %zu component %u  cpu=%.9e gpu=%.9e  (%zu elements outside%s)\n",
                            q.worst_element, q.worst_component, static_cast<double>(q.worst_cpu),
                            static_cast<double>(q.worst_gpu), q.elements_outside_band,
                            q.nan_seen ? ", NaN seen" : "");
            }
            if (q.kind_is_quaternion) {
                std::printf("    unit-norm: max ||q| - 1| = %.9e (both sides)\n",
                            static_cast<double>(q.max_unit_norm_error));
            }
        }
        std::printf("\n");
    }
};

namespace parity_detail {

[[nodiscard]] inline const RegisteredArray* find_array(const ArenaSet& arenas, std::string_view name) {
    const RegisteredArray* found = nullptr;
    arenas.registry().for_each_array([&](const RegisteredArray& array) {
        if (found == nullptr && array.name == name) found = &array;
    });
    return found;
}

[[nodiscard]] inline float load_float(const std::byte* base, std::size_t offset) noexcept {
    float value = 0.0f;
    std::memcpy(&value, base + offset, sizeof(float));
    return value;
}

// The same bytes, as the 32-bit word they are, for QuantityKind::bits.
// memcpy rather than a reinterpret_cast for the identical strict-aliasing
// reason load_float() uses one.
// THE PER-ELEMENT PREDICATE (TD-14), on the error: `abs_err` = |gpu - cpu|
// passes when abs_err <= abs + rel * |cpu|. The two halves ADD: the absolute
// half carries the near-zero regime (alone at cpu == 0, where an exact-zero
// band is exactly the bit-equality demand), the relative half the large one,
// and an error between them is judged against their sum. It replaced the
// disjunction (abs_ok || rel_ok), which it never refuses where that passed
// and which is at most 2x stricter. A NaN error fails: NaN <= x is false.
[[nodiscard]] inline bool error_within(float abs_err, float cpu, ToleranceBand band) noexcept {
    return abs_err <= band.abs + band.rel * std::fabs(cpu);
}

// The same, on one float of one element. A NaN on either side fails.
[[nodiscard]] inline bool element_within(float cpu, float gpu, ToleranceBand band) noexcept {
    if (std::isnan(cpu) || std::isnan(gpu)) return false;
    return error_within(std::fabs(cpu - gpu), cpu, band);
}

[[nodiscard]] inline uint32_t load_word(const std::byte* base, std::size_t offset) noexcept {
    uint32_t value = 0;
    std::memcpy(&value, base + offset, sizeof(uint32_t));
    return value;
}

}  // namespace parity_detail

// ---------------------------------------------------------------------------
// compare_arrays -- the whole comparison.
//
// `cpu` and `gpu` are two ArenaSets holding the SAME shape (same arrays, same
// element sizes, same extents) at the same tick, one produced by the cpu
// schedule and one read back from the device. invalid_argument if a table row
// names an array neither side has, if the two disagree on shape, or if a row's
// (offset, components) does not fit inside one element -- all three would
// otherwise produce a comparison that silently read the wrong bytes.
//
// ARRAYS NOT NAMED IN THE TABLE ARE NOT COMPARED, and that is the mechanism by
// which a task with only SOME passes ported states its scope: wave A ports
// nothing that writes `dryden`, `imu_sensors`, `imu_ring` or `rotors`, so a
// caller's table lists `bodies` and those arrays go uncompared -- explicitly,
// in a table a reviewer can read, rather than by a silent skip.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Result<ParityReport> compare_arrays(const ArenaSet& cpu, const ArenaSet& gpu,
                                                          BandTable table) {
    ParityReport report;
    report.quantities.reserve(table.size());

    for (const BandEntry& entry : table) {
        const RegisteredArray* cpu_array = parity_detail::find_array(cpu, entry.array);
        const RegisteredArray* gpu_array = parity_detail::find_array(gpu, entry.array);
        if (cpu_array == nullptr || gpu_array == nullptr) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "compare_arrays: no registered array named '" +
                                             std::string(entry.array) + "' on " +
                                             (cpu_array == nullptr ? "the cpu side" : "the gpu side")});
        }
        if (cpu_array->elem_size != gpu_array->elem_size ||
            cpu_array->world_count != gpu_array->world_count ||
            cpu_array->capacity_per_world != gpu_array->capacity_per_world) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "compare_arrays: shape mismatch for '" + std::string(entry.array) +
                                             "' -- the two runs are not the same experiment"});
        }
        const std::size_t last_byte = entry.offset + std::size_t{entry.components} * sizeof(float);
        if (last_byte > cpu_array->elem_size) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "compare_arrays: quantity '" + std::string(entry.quantity) +
                                             "' runs past the end of a '" + std::string(entry.array) +
                                             "' element"});
        }
        if (entry.kind == QuantityKind::quaternion && entry.components != 4) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "compare_arrays: quantity '" + std::string(entry.quantity) +
                                             "' is declared a quaternion but has " +
                                             std::to_string(entry.components) + " components"});
        }
        // A bit-exact row with a non-zero band would read as "these integers
        // may differ by a little", which is not a thing (see QuantityKind::bits).
        // Rejected here rather than ignored, so the table cannot quietly claim
        // a tolerance nothing applies.
        if (entry.kind == QuantityKind::bits && (entry.band.abs != 0.0f || entry.band.rel != 0.0f)) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "compare_arrays: quantity '" + std::string(entry.quantity) +
                                             "' is bit-exact but carries a non-zero tolerance band"});
        }

        QuantityReport q;
        q.array = std::string(entry.array);
        q.quantity = std::string(entry.quantity);
        q.band = entry.band;
        q.kind_is_quaternion = entry.kind == QuantityKind::quaternion;
        q.kind_is_bits = entry.kind == QuantityKind::bits;

        const std::size_t element_count =
            static_cast<std::size_t>(cpu_array->world_count) * cpu_array->capacity_per_world;
        for (std::size_t e = 0; e < element_count; ++e) {
            const std::byte* cpu_elem = cpu_array->data + e * cpu_array->elem_size;
            const std::byte* gpu_elem = gpu_array->data + e * gpu_array->elem_size;

            bool element_outside = false;
            float cpu_norm_sq = 0.0f;
            float gpu_norm_sq = 0.0f;

            // ---------------------------------------------------------------
            // BIT-EXACT ROWS take a different loop entirely, because there is
            // no arithmetic to do: load the two words, demand equality.
            //
            // WHAT THE REPORT'S FLOAT-SHAPED FIELDS CARRY HERE, so the printed
            // table is readable rather than merely populated: `max_abs` is the
            // largest |cpu_word - gpu_word| seen, as a float (0 when every word
            // matches, which is the only passing value); `max_rel` stays 0
            // because no relative error is defined; and worst_cpu/worst_gpu
            // carry the two WORD VALUES converted to float, so a failure line
            // prints the actual integers rather than their bit patterns
            // reinterpreted as garbage floats. A difference above 2^24 loses
            // precision in that conversion, which is acceptable for a
            // diagnostic whose real content is "they differ, at element N".
            // ---------------------------------------------------------------
            if (entry.kind == QuantityKind::bits) {
                for (uint32_t c = 0; c < entry.components; ++c) {
                    const std::size_t at = entry.offset + std::size_t{c} * sizeof(uint32_t);
                    const uint32_t a = parity_detail::load_word(cpu_elem, at);
                    const uint32_t b = parity_detail::load_word(gpu_elem, at);
                    if (a == b) continue;

                    element_outside = true;
                    const float diff = static_cast<float>(a > b ? a - b : b - a);
                    if (diff > q.max_abs) {
                        q.max_abs = diff;
                        q.worst_element = e;
                        q.worst_component = c;
                        q.worst_cpu = static_cast<float>(a);
                        q.worst_gpu = static_cast<float>(b);
                    }
                }
                ++q.elements_compared;
                if (element_outside) ++q.elements_outside_band;
                continue;
            }

            for (uint32_t c = 0; c < entry.components; ++c) {
                const std::size_t at = entry.offset + std::size_t{c} * sizeof(float);
                const float a = parity_detail::load_float(cpu_elem, at);
                const float b = parity_detail::load_float(gpu_elem, at);
                cpu_norm_sq += a * a;
                gpu_norm_sq += b * b;

                const float abs_err = std::fabs(a - b);
                const float rel_err = a != 0.0f ? abs_err / std::fabs(a) : 0.0f;

                if (std::isnan(a) || std::isnan(b) || std::isnan(abs_err)) {
                    q.nan_seen = true;
                    element_outside = true;
                    continue;
                }
                if (abs_err > q.max_abs) {
                    q.max_abs = abs_err;
                    q.worst_element = e;
                    q.worst_component = c;
                    q.worst_cpu = a;
                    q.worst_gpu = b;
                }
                if (rel_err > q.max_rel) q.max_rel = rel_err;
                if (std::fabs(a) < kNearZeroCutoff) {
                    if (abs_err > q.near_zero_abs) q.near_zero_abs = abs_err;
                } else if (rel_err > q.far_rel) {
                    q.far_rel = rel_err;
                }

                // ---------------------------------------------------------
                // TD-14's predicate (parity_detail::error_within): the error
                // passes when it is within abs + rel * |cpu|. At cpu == 0 the
                // relative half contributes nothing, so the ABSOLUTE half
                // carries exact zeros alone -- which keeps the S6 Task 6
                // review's finding C2 closed: a zero band on a cpu == 0
                // component (force_acc, torque_acc, a free slot a kernel must
                // not scribble into) demands bit equality, and no relative
                // term can hide a GPU value there.
                // ---------------------------------------------------------
                if (!parity_detail::element_within(a, b, entry.band)) element_outside = true;
            }

            if (entry.kind == QuantityKind::quaternion) {
                // Both sides, so a drift on either is visible. Skipped for an
                // all-zero quaternion, which is what a FREE arena slot holds --
                // it is not a rotation and has no norm to be wrong about.
                if (cpu_norm_sq != 0.0f) {
                    const float err = std::fabs(std::sqrt(cpu_norm_sq) - 1.0f);
                    if (err > q.max_unit_norm_error) q.max_unit_norm_error = err;
                }
                if (gpu_norm_sq != 0.0f) {
                    const float err = std::fabs(std::sqrt(gpu_norm_sq) - 1.0f);
                    if (err > q.max_unit_norm_error) q.max_unit_norm_error = err;
                }
            }

            ++q.elements_compared;
            if (element_outside) ++q.elements_outside_band;
        }

        report.quantities.push_back(std::move(q));
    }

    return report;
}

// ===========================================================================
// THE PINNED BANDS.
//
// PROVENANCE, common to every constant below:
//   device   Intel Iris Plus Graphics (integrated), Vulkan 1.3.215
//   driver   31.0.101.2125
//   build    msvc-ninja-release. THE TASK VARIES BY BLOCK AND EACH BLOCK SAYS
//            WHICH -- this line read "S6 Task 6" through Task 7, which was
//            already false of the three blocks Task 7 measured, and is the
//            review minor (T7 M6) S6 Task 8 closed. Measured by Task 6 or 6b
//            and UNCHANGED since (re-measured by Task 8, same numbers):
//            gate_fleet, bounce, drag_componentwise, restore_resume,
//            heterogeneous_geometry_set. By Task 7, likewise unchanged:
//            contact_pair, shower (then `shower_calm`), shower_ladder. By
//            Task 8: `medium`, `two_world_isolation`, `quad_hover` (all new)
//            and `ballistic`, RE-MEASURED because the scenario itself changed
//            -- see that block's header.
//   method   measured max over the WHOLE run, every element of every world,
//            printed by tests/test_gpu_parity.cpp's own table; each constant
//            below records the measurement it came from and the margin factor
//            applied to it.
//   margin   ~4x the measured maximum, rounded UP to a round decimal. Four
//            rather than two because the measured value is one run on one
//            device and the band must survive a driver update that changes a
//            div/sqrt implementation within its <= 2.5-ulp allowance; not more
//            than four, because a band that cannot move is a band that catches
//            nothing.
//
// THE STOP RULE, which no constant here is allowed to quietly break: this
// task's brief pins a hard tripwire at 1e-3 RELATIVE. A quantity needing a
// wider relative band than that is reported to the coordinator as a
// measurement, not banded over. Every `rel` below is inside it.
//
// WHY EVERY SCENARIO NEEDS ITS OWN SET. The divergence is the accumulated
// <= 2.5-ulp div/sqrt error amplified by the dynamics, so it is a property of
// the RUN, not of the quantity: 200 steps of smooth ballistic flight, 900 steps
// of a body resting on a plane, 900 steps of a restitution ladder bouncing and
// 1800 substeps of a quadrotor closing a thrust/attitude loop are four
// different amplification regimes. One set covering all of them would be the
// loosest everywhere, which would stop the tight scenarios from catching
// anything.
//
// THE ONE EXCEPTION IS `medium` (S6 Task 8), and it is an exception because the
// Dryden filter is provably NOT in that amplification loop -- see that block's
// own header for the argument and for the eleven-scenario measurement that
// bears it out.
// ===========================================================================
namespace bands {

// ---------------------------------------------------------------------------
// ballistic -- THE CORPUS SCENARIO VERBATIM as of S6 Task 8. 200 steps x 5
// substeps (1 kHz), one body, quadratic drag, a scripted wrench every tick, no
// geometry (the world SDF is empty) -- and `turbulence: moderate`, which is the
// part that changed.
//
// RE-MEASURED, AND THE REASON IS A DIFFERENT EXPERIMENT RATHER THAN A DIFFERENT
// ANSWER. Through S6 Task 7 this block was `ballistic_calm`: the same corpus
// file with its authored turbulence overridden to none on the built world set,
// because MediumUpdate was a stub and stepping the real scenario on the vulkan
// backend was refused outright. Task 8 ports that pass, the override is gone,
// and the run this block describes now has a live gust reaching the body every
// substep through its drag element. Two rows moved as a result and are pinned
// from the new measurement; this is NOT a band widened to make a failure pass
// (the standing rule) but a band measured against the scenario it now names.
//
// IT IS THE CORPUS'S ONLY GUST-TO-BODY PATH. A gust reaches force_acc only
// through a drag element (physics/forces.cpp samples the medium per element).
// `shower` and `two_world_isolation` are turbulent and spawn none, so their
// gusts are drawn and never read; `quad_hover` has both a drag body and rotors
// but buries the gust inside a closed thrust/attitude loop. This scenario is
// the clean one: one body, one drag element, one gust, 1000 substeps.
//
// MEASURED (4 elements = the world's whole body partition; 1 live):
//     pos             max|abs| 0                max rel 0             <- BIT-EXACT
//     vel             max|abs| 1.19209290e-07   max rel 8.14619057e-08
//     orient          max|abs| 7.15255737e-07   max rel 3.12810835e-06
//     omega_body      max|abs| 1.69873238e-06   max rel 4.17549973e-06
//     specific_force  max|abs| 3.81469727e-06   max rel 3.75774880e-06
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 5.96046448e-08 (one ulp at 1.0) on both sides
//     (the `dryden` rows use the shared `medium` block above)
//
// WHAT MOVED FROM `ballistic_calm`, AND WHY EACH ONE IS EXPLAINED RATHER THAN
// ABSORBED:
//
//   * `vel` WAS BIT-EXACT AND IS NOW ONE ULP (1.19209290e-07 on a component of
//     magnitude 1.46). That is the gust arriving. In the calm run the drag law
//     read `v_rel = vel - params.wind` with a constant wind, and the whole
//     translational chain was exactly reproducible; now `wind` carries
//     `dryden_turbulence()`, whose own state differs by up to a few ulps (the
//     `medium` block above), and one ulp of wind becomes one ulp of drag force
//     becomes one ulp of velocity. The propagation is one-for-one, which is the
//     right order and is why this is a satisfying number rather than a
//     worrying one.
//   * `orient` ROSE FROM 4.17232513e-07 TO 7.15255737e-07 -- under 2x, and from
//     the same source arriving one step further along: the drag element is
//     mounted off-axis, so a perturbed drag force perturbs the body-frame
//     torque, which perturbs omega, which perturbs the exp-map.
//   * `omega_body` and `specific_force` moved within the same factor and stayed
//     inside the bands `ballistic_calm` already carried (8.0e-6 and 2.0e-5).
//     They are re-pinned at 4x the NEW measurement anyway, so this block's
//     numbers all describe one run.
//
// Bands are 4x measured, rounded up to a round decimal.
// ---------------------------------------------------------------------------
namespace ballistic {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{5.0e-7f, 5.0e-7f};
inline constexpr ToleranceBand kOrient{4.0e-6f, 2.0e-5f};
inline constexpr ToleranceBand kOmega{8.0e-6f, 2.0e-5f};
inline constexpr ToleranceBand kSpecificForce{2.0e-5f, 2.0e-5f};
}  // namespace ballistic

// ---------------------------------------------------------------------------
// gate_fleet -- 900 steps x 1 substep (1 kHz), four worlds from
// gate.world.yaml, one body each, dropped through the gate geometry onto the
// ground plane. The SDF is a four-primitive union (plane, torus, two boxes),
// so this is the scenario that actually exercises sdf_eval.slang: the torus
// takes the CENTRAL-DIFFERENCE gradient path (six primitive evaluations, each
// with its own sqrt), which is by far the longest div/sqrt chain wave A has.
// ---------------------------------------------------------------------------
// Each body additionally carries a DIFFERENT asymmetric inertia, so Integrate's
// step 4 (the reciprocal, the gyroscopic cross product, the componentwise
// I^-1 * tau) is genuinely live here rather than identically zero the way a
// spherical inertia would make it.
//
// MEASURED:
//     pos             max|abs| 4.76837158e-07   max rel 2.11566180e-06
//     vel             max|abs| 8.72015953e-05   max rel 6.41855923e-03   (!)
//     orient          max|abs| 7.59959221e-07   max rel 5.81913673e-06
//     omega_body      max|abs| 7.45058060e-09   max rel 2.41653538e-07
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 5.96046448e-08 (one ulp at 1.0) on both sides
//
// =========================================================================
// THE 6.4e-03 RELATIVE VELOCITY FIGURE, WHICH IS THE ONE NUMBER IN THIS FILE
// THAT NEEDS ITS OWN PARAGRAPH.
//
// This task's brief sets a hard tripwire: "if any quantity needs a band wider
// than 1e-3 relative, STOP and report the measurement (coordinator adjudicates
// vs a kernel bug)". The MEASURED relative error above is 6.4e-03, six times
// that. The band pinned below is NOT: `rel` stays at 1.0e-05 and the element
// that produced 6.4e-03 passes on the ABSOLUTE half of the disjunction. So no
// band here is wider than 1e-3 relative and the tripwire is not tripped -- but
// the measurement is reported anyway, prominently and in the task report,
// because a reviewer who saw only the pinned numbers would not know it existed.
//
// WHAT PRODUCES IT, and why it is a near-zero artefact rather than a defect.
// The worst element is world 1's body at tick 900, whose vel.y is
// 1.358585153e-02 (cpu) against 1.367305312e-02 (gpu) -- an ABSOLUTE difference
// of 8.7e-05 m/s on a body that has come to rest. A resting body in this
// contact model does not sit still, and contacts.cpp says so itself: the
// resting state is "an ACTIVELY BALANCED" fixed point, where every substep
// Integrate sinks the body by one substep of gravity (g*h = 9.8e-03 m/s) and
// the next static_contact.resolve cancels it with an impulse. Its velocity is
// therefore a LIMIT CYCLE whose entire amplitude is ~1e-02 m/s, and 8.7e-05 is
// under one percent of one substep's gravity increment -- the two runs are in
// the same cycle, offset by a fraction of a substep. Dividing a sub-percent
// difference by a quantity that is itself a rounding residue is what makes the
// RELATIVE number large; the absolute number, the one with physical meaning
// here, is 8.7e-05 m/s in a scenario whose bodies fall at up to 7 m/s.
//
// THE CORROBORATION THAT THIS IS NOT A CONTACT-KERNEL BUG: `bounce` below runs
// the SAME static_contact.resolve kernel for the same 900 steps over a four-rung
// restitution ladder WITH friction -- divisions and square roots included --
// and is BIT-IDENTICAL to the CPU in every quantity. A defect in the impulse,
// the friction cap or the Baumgarte correction would have to show up there
// first, and it does not. What gate_fleet adds over bounce is the SDF's own
// div/sqrt chain (the torus's six-evaluation central difference) and asymmetric
// inertia -- which is exactly where last-bit differences enter.
// =========================================================================
namespace gate_fleet {
inline constexpr ToleranceBand kPos{2.0e-6f, 1.0e-5f};
// abs 4.0e-04 is the OPERATIVE half here (4x the measured 8.72e-05 m/s); the
// relative half stays tight on purpose, so a genuinely large-magnitude velocity
// error could not hide behind it.
inline constexpr ToleranceBand kVel{4.0e-4f, 1.0e-5f};
inline constexpr ToleranceBand kOrient{4.0e-6f, 4.0e-5f};
inline constexpr ToleranceBand kOmega{4.0e-8f, 2.0e-6f};
// Zero, and structurally so rather than luckily: specific_force is
// `conjugate(orient) * (force_acc / mass)`, this scenario spawns no force
// elements at all, and Integrate clears force_acc at the end of every substep
// -- so accel_ext is exactly (0,0,0) in every substep and the rotation of a
// zero vector is a zero vector on both paths, for any orientation.
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace gate_fleet

// ---------------------------------------------------------------------------
// bounce -- 900 steps x 1 substep (1 kHz), the golden corpus scenario
// VERBATIM: four worlds, a four-rung restitution ladder (e = 0, 0.3, 0.6, 0.9),
// one ball each falling onto a plane with friction. The contact torture case,
// and the scenario the brief expects to be loosest: a bounce is a
// DISCONTINUOUS event, so a sub-ulp difference in the substep a contact is
// first detected on becomes a whole substep of divergence in the trajectory
// afterwards.
// ---------------------------------------------------------------------------
// MEASURED: ZERO. Every quantity, every component, every one of the 16
// elements -- the GPU run is BIT-IDENTICAL to the CPU run after 900 steps.
//
// THE BANDS BELOW ARE THEREFORE ALL ZERO, WHICH IS A CLAIM AND IS MEANT TO BE.
// Pinning a nominal band "for safety" would assert less than the measurement
// supports and would silently absorb a future regression. If a driver update
// moves this, the suite says so and the coordinator rules on it -- which is the
// global constraint's own posture ("bands are NEVER widened to make a failure
// pass without a root-caused coordinator ruling").
//
// IT IS NOT VACUOUS -- BUT IT PROVES LESS ABOUT DIVISION AND SQUARE ROOT THAN
// IT FIRST APPEARS, AND THE DISTINCTION MATTERS (S6 Task 6 review round 1,
// finding I3). Both statements are true and neither may be dropped:
//
//   * The division and square-root INSTRUCTIONS genuinely execute here, on
//     every substep a body is in contact. The friction step takes
//     `glm::length(v_t)` and then divides `v_t` by it -- the balls arrive with
//     vel.x = 0.5 m/s, so the tangential velocity is non-zero for hundreds of
//     substeps -- and the contact normal comes from `gradient / |gradient|`. So
//     the scenario did not reach bit-exactness by avoiding them.
//   * But their OPERANDS here are largely exactly-representable, and that is
//     WHY they agree. The world SDF is a single plane whose gradient is exactly
//     (0, 1, 0), so `|gradient|` is sqrt(1) == 1 and `gradient / 1` is exact
//     for any implementation; the bodies have mass exactly 1, so
//     `force_acc / mass` is exact too; and `v_t / |v_t|` is a vector divided by
//     its own norm, the best-conditioned division there is.
//
// SO WHAT THIS SCENARIO ESTABLISHES IS THE OP ORDER, NOT THE DEVICE'S ROUNDING.
// It says: with the ill-conditioned operands removed, the ported kernels
// reproduce the CPU twin EXACTLY -- so the sequence, the groupings and the
// parenthesisation are right, not merely close. It does NOT say this device's
// OpFDiv and GLSL.std.450 Sqrt are correctly rounded in general, and the other
// scenarios in this file show they are not always: gate_fleet's torus chain and
// restore_resume's stiction residue are both single-ulp division differences.
//
// WHY IT IS BIT-EXACT WHERE THE OTHER TWO ARE NOT, completing the picture
// ballistic_calm's note started. `bounce` spawns mass = 1 (so `force_acc/mass`
// is exact), inv_inertia_diag = (100,100,100) under a linear-only contact model
// with no torque source (so omega stays exactly 0, the exp-map takes its
// small-angle branch at theta = 0, and the orientation never moves at all), and
// its world SDF is a single PLANE whose gradient is the exactly-representable
// (0, 1, 0). None of the three divergence sources the other two scenarios carry
// -- the inertia reciprocal, the sinc division, the torus's central-difference
// chain -- is present here. Bit-exactness is therefore a measurement of what
// remains once those are removed, and it is the strongest single statement this
// task can make about the ported op order: with the ulp-level divergence
// sources taken away, the Slang kernels reproduce the CPU twin EXACTLY, which
// means the op ORDER itself is right rather than merely close.
namespace bounce {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{0.0f, 0.0f};
inline constexpr ToleranceBand kOrient{0.0f, 0.0f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace bounce

// ---------------------------------------------------------------------------
// drag_componentwise (S6 Task 6 review round 1, finding I1) -- 400 steps x 2
// substeps (1 kHz), one body in the VOID world (no geometry at all), carrying a
// componentwise drag element with three different per-axis coefficients and an
// off-axis mount, spawned tilted and spinning with an asymmetric inertia.
//
// WHAT IT COVERS THAT NOTHING ELSE DID: the componentwise branch of
// forces_drag.slang -- rotate v_rel into the body frame with the orientation's
// CONJUGATE, apply F_i = -c_i*|v_i|*v_i per axis, rotate the result back OUT.
// `ballistic` is quadratic-mode and the other two scenarios have no drag
// elements, so that round trip was compiled but never executed against the CPU.
//
// MEASURED (same device and driver as every other block in this file; the task
// report's round-1 fix section has the literal table):
//     pos             max|abs| 7.15255737e-07   max rel 3.46888470e-07
//     vel             max|abs| 9.53674316e-07   max rel 3.72444049e-07
//     orient          max|abs| 4.47034836e-07   max rel 2.46528452e-06
//     omega_body      max|abs| 1.90734863e-06   max rel 5.78806294e-07
//     specific_force  max|abs| 7.15255737e-06   max rel 4.26766064e-06
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 0 on both sides
//
// EVERY row moves here, unlike the other scenarios, and that is the coverage
// working rather than a worry: this is the only parity scenario whose force law
// depends on the ATTITUDE, so the rotational chain's ulp-level divergence feeds
// straight back into the translational one through F_body -> F_world. `pos` and
// `vel` are bit-exact in `ballistic_calm` precisely because a QUADRATIC drag
// element never consults the orientation; a componentwise one does, twice per
// element per substep. specific_force is the loosest row for the same reason it
// is in ballistic_calm -- it is the accumulated force divided by mass and then
// rotated, so it inherits both chains at once.
// ---------------------------------------------------------------------------
namespace drag_componentwise {
inline constexpr ToleranceBand kPos{4.0e-6f, 2.0e-6f};
inline constexpr ToleranceBand kVel{4.0e-6f, 2.0e-6f};
inline constexpr ToleranceBand kOrient{2.0e-6f, 1.0e-5f};
inline constexpr ToleranceBand kOmega{8.0e-6f, 4.0e-6f};
inline constexpr ToleranceBand kSpecificForce{4.0e-5f, 2.0e-5f};
}  // namespace drag_componentwise

// ---------------------------------------------------------------------------
// restore_resume (S6 Task 6 review round 1, finding C1) -- 2 worlds from
// gate.world.yaml, a vulkan Simulation stepped twice (so its device mirror is
// CLEAN), then restored from a CPU snapshot taken at tick 300 and resumed to
// tick 700, compared against an uninterrupted CPU run of the same sequence.
//
// This is a PARITY comparison in the same sense as the others -- the same
// kernels over the same 400 resumed steps -- so its bands are measured the same
// way. What makes it a C1 regression test rather than a fourth scenario is the
// pre-restore stepping: with `restore()` unmarked, the device would step the
// stale pre-restore state and the divergence would be GROSS (metres), not
// banded.
//
// MEASURED:
//     pos             max|abs| 0                max rel 0             <- BIT-EXACT
//     vel             max|abs| 1.49011612e-08   max rel 0             (see below)
//     orient          max|abs| 2.75671482e-07   max rel 3.05988851e-06
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 1.19209290e-07 (two ulp at 1.0) on both sides
//
// THE ONE NON-ZERO TRANSLATIONAL FIGURE, AND WHY IT IS ALSO THE PROOF THAT THE
// C2 PREDICATE FIX WAS NEEDED. `vel`'s worst element is world 1's resting body,
// component x: cpu = 0.000000000e+00 exactly, gpu = -1.490116119e-08. The CPU
// side is EXACTLY ZERO, so the measured RELATIVE error is 0 by definition --
// and under the pre-fix predicate that alone made the element pass, against a
// band of zero, hiding a genuine (if tiny) difference. It is caught now.
//
// What it is: the STICTION-FLOOR RESIDUE physics/contacts.cpp predicts in
// advance, at the friction step -- "when the cap binds, the subtracted vector is
// |v_t| * fl(v_t/|v_t|), which reproduces v_t to within about two roundings, so
// a residue of order eps*|v_t| ... can survive with either sign". `v_t/v_t_len`
// is a division, so the two paths land on different sides of that residue and
// one reaches exactly 0 while the other stops an ulp short. It cannot grow (the
// next substep's cap is at least as large), which is why 1.5e-08 is where it
// stays over 400 steps.
//
// THE RELATIVE HALF OF kVel IS THEREFORE PINNED AT ZERO, deliberately: every
// non-zero error in this row occurred where the CPU value was exactly 0, so
// there is no relative error for a relative band to be measured from, and the
// absolute half is the only live one. Pinning a nominal relative band would be
// inventing a measurement.
//
// pos, omega_body and specific_force are bit-exact for the structural reasons
// `bounce`'s note sets out: these bodies carry no force elements, so force_acc
// is exactly zero every substep, and the linear-only contact model never
// touches omega.
// ---------------------------------------------------------------------------
namespace restore_resume {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{1.0e-7f, 0.0f};
inline constexpr ToleranceBand kOrient{2.0e-6f, 2.0e-5f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace restore_resume

// ---------------------------------------------------------------------------
// heterogeneous_geometry_set (S6 Task 6b, checkpoint-1 ruling) -- 900 steps x
// 1 substep (1 kHz), TWO worlds that do not share a world file: world 0 is
// `ballistic` (empty SDF, no geometry at all, plain free-falling tumbling
// body, no force elements), world 1 is `gate` (the same plane+torus+two-box
// union gate_fleet above exercises, the SAME spawn and the SAME timing).
// Locks the property nothing else in this file checks: the per-world GPU
// geometry buffers (sdf_world_ranges/sdf_nodes/sdf_transforms, contact_params,
// and grid_params -- S6 Task 6b's own binding 20) correctly serve two worlds
// whose SDF PROGRAMS genuinely differ in shape (0 nodes vs 7), not merely
// whose material constants differ (bounce's restitution ladder shares one
// empty SDF across all four worlds).
// ---------------------------------------------------------------------------
// MEASURED (this device, this driver, S6 Task 6b's own run -- the task report
// has the literal table; 2 elements, one per world). Re-measured, unchanged,
// after fix round 1's I1 (world 0's spawn.pos.y moved 10.0 -> 2.0 to make the
// heterogeneous-geometry lock bidirectional -- see
// heterogeneous_ballistic_spawn()'s own comment): world 0's translational
// chain has no SDF collision and no drag, so it does not depend on starting
// height, and its rotational chain depends only on omega_body/
// inv_inertia_diag, which the fix did not touch -- so this scenario's worst
// elements were, and remain, sourced from the SAME components either way:
//     pos             max|abs| 4.15922841e-07   max rel 1.58518851e-02   (!)
//     vel             max|abs| 2.42014357e-06   max rel 1.61251780e-02   (!)
//     orient          max|abs| 4.17232513e-07   max rel 4.67018936e-06
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 0 on both sides
//
// THE TWO (!) FIGURES, same shape of artefact gate_fleet's own 6.4e-03 vel
// figure already documents in full, and the reason this file's predicate is a
// disjunction at all. compare_arrays() tracks max_abs and max_rel
// INDEPENDENTLY (parity.hpp's own compare_arrays, above) -- they need not
// come from the same component -- but the inequality still gives a real
// bound: for whichever component produced max_rel, |cpu - gpu| <= max_abs (the
// maximum over every compared component), so |cpu| = |cpu - gpu| / rel <=
// max_abs / max_rel. That bound is 4.15922841e-07 / 1.58518851e-02 =
// 2.62e-05 m for pos and 2.42014357e-06 / 1.61251780e-02 = 1.50e-04 m/s for
// vel -- both a few tens of microns (or micrometres/second) from zero. That
// is the near-zero-denominator class, definitively: dividing a sub-micron
// difference by a near-zero CPU value is what manufactures a 1.6% "relative"
// figure out of an absolute difference four orders of magnitude tighter than
// the pinned band below. Both elements pass on the ABSOLUTE half of the
// disjunction, not the relative one: `rel` stays tight (not widened to absorb
// either figure), exactly gate_fleet's own posture, so a genuinely
// large-magnitude error could not hide behind it. omega_body/specific_force/
// force_acc/torque_acc are BIT-EXACT for the same structural reasons
// gate_fleet's specific_force row is: neither world spawns a force element,
// so force_acc is exactly (0,0,0,0) every substep on both paths.
//
// orient's measured value (4.17232513e-07) is bit-identical to
// ballistic_calm's own measured orient figure above -- the same inertia-
// reciprocal ulp source (1/inv_inertia_diag[i]), plausibly, given world 0
// here carries the same order-of-magnitude asymmetric inertia ballistic_calm
// does. Pinned at the same 2.0e-6 abs this task gives ballistic_calm's orient
// row for that reason.
// ---------------------------------------------------------------------------
namespace heterogeneous_geometry_set {
inline constexpr ToleranceBand kPos{2.0e-6f, 1.0e-5f};
inline constexpr ToleranceBand kVel{1.0e-5f, 1.0e-5f};
inline constexpr ToleranceBand kOrient{2.0e-6f, 2.0e-5f};
// Measured bit-exact (0, 0); pinned at exactly what was measured, the same
// posture `bounce`'s header explains at length: a nominal nonzero band here
// would assert less than the measurement supports and would silently absorb
// a future regression rather than catching it.
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace heterogeneous_geometry_set

// ===========================================================================
// S6 TASK 7 -- the sorted-grid body-body pass (dynamic_contact.resolve). Three
// scenarios, and the reason there are three is the same reason there were five
// before: the divergence is a property of the RUN.
//
// A WORD ON WHY THESE ARE THE LOOSEST BANDS IN THIS FILE, stated up front so
// the numbers below are read correctly. Body-body contact compounds every
// divergence source wave A had AND adds two of its own:
//
//   * THE SWEEP IS GAUSS-SEIDEL. Each pair sees the velocities the earlier
//     pairs left (physics/grid.hpp's PARITY note), so a last-bit difference in
//     pair k's result is an INPUT to pair k+1 rather than an independent
//     error. In a 100-body pile with hundreds of simultaneous contacts, one
//     substep chains hundreds of these.
//   * A CONTACT IS A DISCONTINUOUS EVENT. `bounce`'s own note already says it:
//     "a sub-ulp difference in the substep a contact is first detected on
//     becomes a whole substep of divergence in the trajectory afterwards". A
//     pile forming from a dropped lattice is that event, several hundred times
//     over, with the outcome of each deciding the geometry the next ones see.
//
// So the honest expectation for `shower` is NOT bit-exactness and never was.
// What the bands assert is that the two runs stay in the SAME PILE -- the same
// configuration, differing by a rounding-scale perturbation -- rather than
// diverging into different ones.
//
// ===========================================================================
// THE HORIZON, AND WHY THESE BANDS STOP AT 400 STEPS. This is the one thing a
// reader must take from this block, because it is the reason no band below is
// wide.
//
// Both shower cases were ALSO measured at 800 steps, and there the CPU<->GPU
// figures blow up -- from 3.6e-07 m of position to 4.6e-06 m for the
// one-world case, and to 1.02e-02 m (with a max relative VELOCITY error of
// 33) for the two-world one. Taken alone, that last figure trips this
// program's stop rule (1e-3 relative at a non-near-zero absolute) and would
// have to be escalated as a possible kernel defect.
//
// IT IS NOT ONE, AND THE CONTROL THAT SETTLES IT NEEDS NO GPU:
// tests/test_gpu_parity.cpp's ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpu-
// Alone runs the same scenario TWICE ON THE CPU, differing only in ONE ULP of
// one body's initial x, and measures how far apart the two runs end up:
//
//     worst |pos| gap from ONE ULP, CPU vs CPU (this device, this build)
//                     400 steps            800 steps
//         e = 0.2     6.98491931e-10 m     2.98023224e-07 m
//         e = 0.8     1.19209290e-07 m     1.01754665e-02 m
//
// The bottom-right figure is 1.0175e-02 m. The CPU<->GPU figure at the same
// horizon and the same material is 1.0249e-02 m -- the SAME NUMBER TO WITHIN
// ONE PERCENT. Past roughly 400 steps this pile is not reproducible to
// rounding by ANY two implementations that differ in a last bit, two CPU runs
// included; the divergence is the scenario's Lyapunov amplification and not a
// property of the port.
//
// So the bands below are pinned where the comparison still MEASURES the port
// (400 steps, the corpus scenario's own decomposition), the 800-step numbers
// are REPORTED rather than banded over, and the standing rule is kept intact:
// "bands are NEVER widened to make a failure pass". A band pinned at 800 steps
// would not be a tolerance; it would be a Lyapunov exponent wearing one.
// ===========================================================================

// ---------------------------------------------------------------------------
// contact_pair -- 600 steps x 1 substep (1 kHz), ONE world, TWO bodies spawned
// 0.15 m apart against a 0.2 m contact distance (i.e. already overlapping),
// with UNEQUAL masses (0.6 / 1.4) so resolve_pair()'s two mass weights
// w_a = mb/(ma+mb) = 0.7 and w_b = ma/(ma+mb) = 0.3 are distinct and
// non-trivial. Falls onto gate.world.yaml's ground plane, so the static and
// dynamic passes interleave exactly as they do in the corpus.
//
// THE SMALLEST SCENARIO THAT EXERCISES THE WHOLE PAIR PATH, and the one whose
// failure is diagnosable: two bodies, one pair, one cell -- if this is outside
// its band the defect is in resolve_pair() itself and not in the sweep order.
// ---------------------------------------------------------------------------
// MEASURED (same device/driver/method as every other block in this file; the
// S6 Task 7 report has the literal table):
//     pos             max|abs| 0                max rel 0             <- BIT-EXACT
//     vel             max|abs| 0                max rel 0             <- BIT-EXACT
//     orient          max|abs| 2.01165676e-07   max rel 3.37340407e-06
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 5.96046448e-08 (one ulp at 1.0) on both sides
//
// pos AND vel ARE BIT-EXACT, WHICH IS THE STRONGEST SINGLE STATEMENT THIS TASK
// CAN MAKE ABOUT resolve_pair(). Those two are the only quantities the
// body-body contact model writes -- it is linear-only, so omega_body and
// torque_acc are never touched (physics/grid.hpp's scope note 1) -- and over
// 600 substeps of a resolved, overlapping, unequal-mass pair, every impulse,
// every friction cap, every Baumgarte correction and every mass weight
// reproduced the CPU EXACTLY. The op order, the groupings and the
// parenthesisation of grid.cpp:89-260 survived the port, not merely
// approximately.
//
// `orient` is the ONLY row that moves, and it moves for a reason that has
// nothing to do with this task: with omega_body bit-exact, the divergence can
// only enter downstream of it, in integrate_orientation()'s exp-map (the
// sin32(half_angle)/half_angle division) and its quaternion renormalization
// (1/|q|). Both are Integrate's, both are already documented in
// ballistic_calm's own note, and its measured orient figure
// (4.17232513e-07) is the same order. Bands are 4x measured, rounded up.
namespace contact_pair {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{0.0f, 0.0f};
inline constexpr ToleranceBand kOrient{1.0e-6f, 2.0e-5f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace contact_pair

// ---------------------------------------------------------------------------
// shower -- the golden corpus scenario VERBATIM as of S6 Task 8: 100 bodies in
// an SDF bowl, 400 steps x 2 substeps, `turbulence: light`.
//
// IT RAN AS `shower_calm` THROUGH TASK 7, with the authored turbulence
// overridden to none because MediumUpdate was a stub. The override is gone and
// THE BODY NUMBERS BELOW DID NOT MOVE -- re-measured, identical to the last
// digit -- which is the prediction that adaptation was made under, now
// confirmed rather than merely argued: shower spawns no drag element, a gust
// reaches a body only through one, so the gusts this scenario generates are
// drawn, advanced, compared (the `dryden` rows) and never read by any pass that
// touches `bodies`.
//
// ONE WORLD => WorldSetLayout::uniform_dynamic_params is true => the BATCHED
// dispatch shape (one power-of-two segment over the whole slot space).
// ---------------------------------------------------------------------------
// MEASURED (128 elements = the world's whole body partition; 100 live, 28 free
// slots that are zero on both sides):
//     pos             max|abs| 3.57627869e-07   max rel 4.31607259e-06
//     vel             max|abs| 4.39584255e-06   max rel 3.20072722e-05
//     orient          max|abs| 0                max rel 0             <- BIT-EXACT
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 0 on both sides
//
// THE ROTATIONAL ROWS ARE BIT-EXACT FOR A STRUCTURAL REASON, not by luck, and
// it is `bounce`'s reason exactly: shower spawns inv_inertia_diag =
// (500,500,500) -- SPHERICAL -- with omega_body = (0,0,0), under a LINEAR-ONLY
// contact model (physics/grid.hpp's scope note 1: impulses change `vel` and
// nothing else; no contact torque r x J is generated). So no pass in this
// scenario can ever make omega non-zero, the exp-map takes its small-angle
// branch at theta = 0 on every substep, and the orientation never moves at
// all. That is also what makes this scenario a CLEAN instrument for the
// contact solver: every non-zero number in the table above came from
// resolve_pair() and resolve_static_contacts(), with the integrator's
// rotational chain contributing exactly nothing.
//
// THE TWO NON-ZERO ROWS ARE GENUINE RELATIVE ERRORS, NOT NEAR-ZERO ARTEFACTS,
// and this block says so rather than borrowing gate_fleet's excuse. The worst
// absolute velocity element is body 11's z component at 1.836454421e-01 (cpu)
// against 1.836410463e-01 (gpu); and max_abs/max_rel bounds the max-rel
// element's own magnitude at 4.39584255e-06 / 3.20072722e-05 = 1.37e-01 m/s --
// a real speed, not a rounding residue. So 3.2e-05 is what a 100-body
// Gauss-Seidel pile actually costs: hundreds of chained pair resolutions per
// substep, 800 substeps, each pair's rounding feeding the next.
//
// IT IS WELL INSIDE THE STOP RULE. The brief's tripwire is 1e-3 relative with
// a non-near-zero absolute; the measurement is 3.2e-05, thirty times inside
// it, so there is nothing to escalate -- but it is the largest genuine
// relative figure in this file and is recorded as such.
//
// Bands are 4x measured, rounded up to a round decimal, both halves live.
namespace shower {
inline constexpr ToleranceBand kPos{2.0e-6f, 2.0e-5f};
inline constexpr ToleranceBand kVel{2.0e-5f, 2.0e-4f};
inline constexpr ToleranceBand kOrient{0.0f, 0.0f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace shower

// ---------------------------------------------------------------------------
// shower_ladder -- the same bowl and the same 100-body lattice, TWICE, in two
// worlds whose restitution differs (0.2 and 0.8). That single differing byte
// is what makes uniform_dynamic_params false and sends both the CPU pass and
// the device chain down the PER-WORLD dispatch shape (one independently-sorted
// segment per world) -- the other half of `batch_dynamic_collision`.
//
// ---------------------------------------------------------------------------
// MEASURED (256 elements = two 128-slot body partitions; 200 live):
//     pos             max|abs| 3.57627869e-07   max rel 6.37087533e-06
//     vel             max|abs| 4.39584255e-06   max rel 8.27308031e-05
//     orient          max|abs| 0                max rel 0             <- BIT-EXACT
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 0 on both sides
//
// THE TWO NON-ZERO ABSOLUTE FIGURES ARE BIT-IDENTICAL TO `shower_calm`'s, AT
// THE SAME WORST ELEMENTS (body 0 component z, body 11 component z -- both in
// WORLD 0), AND THAT IS THIS BLOCK'S REAL RESULT. `shower_calm` runs world 0
// alone under the BATCHED dispatch shape; this scenario runs the identical
// world, identical seed, identical material and identical spawns as world 0 of
// a two-world set under the PER-WORLD shape -- and the device reproduces the
// CPU with exactly the same worst-case error, down to the last bit of the
// reported maximum. The two dispatch shapes are not merely both "within band":
// on the world they share, they agree with each other.
//
// (The rel figures differ between the two blocks only because world 1's own
// elements enter the max-rel search; max_rel and max_abs are tracked
// independently, so a near-zero denominator in the added world moves the
// former without moving the latter.)
//
// Bands are 4x measured, rounded up.
// ---------------------------------------------------------------------------
namespace shower_ladder {
inline constexpr ToleranceBand kPos{2.0e-6f, 4.0e-5f};
inline constexpr ToleranceBand kVel{2.0e-5f, 4.0e-4f};
inline constexpr ToleranceBand kOrient{0.0f, 0.0f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace shower_ladder


// ===========================================================================
// S6 TASK 8 -- kernels wave C. THE MEDIUM, THE ROTORS AND THE SENSOR.
//
// Three of the four blocks below are new; the fourth (`medium`) is the one set
// of bands EVERY scenario in the file uses, for a reason stated at its own
// header.
// ===========================================================================

// ---------------------------------------------------------------------------
// medium -- the Dryden gust filter's own state, SHARED BY EVERY SCENARIO IN
// THIS FILE. Four rows: two integer (bit-exact, no band possible) and two
// float.
//
// WHY ONE SET RATHER THAN ONE PER SCENARIO, which is a departure from this
// file's own rule and needs its reason on the page. Every other block here is
// per-scenario because "the divergence is a property of the RUN, not of the
// quantity" -- the ulp-level error is amplified by the dynamics, so 200 steps of
// smooth flight and 900 steps of a bouncing ladder are different regimes.
//
// THE FILTER IS NOT IN THAT LOOP. dryden_advance() is a fixed linear recurrence
// driven by exactly two things: its own rng stream and its own world's constant
// DrydenParams. No body state, no contact, no collision -- nothing that varies
// with the trajectory -- feeds back into it. So it cannot amplify, and the
// measurement below bears that out: across ELEVEN scenarios spanning 200 to 900
// steps and 1 to 5 substeps, every `filter_state` figure lands between 5.96e-08
// and 4.77e-07, i.e. between ONE AND EIGHT ULPS at these magnitudes. That is a
// flat rounding floor, not a growth curve, and a per-scenario band would be
// eleven copies of one fact.
//
// MEASURED, EVERY SCENARIO, EVERY WORLD (`stream.state` and `stream.flag` are
// integer rows and were BIT-EXACT everywhere, which no band expresses -- see
// QuantityKind::bits):
//
//     scenario                     stream.cached          filter_state
//                                  abs / rel              abs / rel
//     ballistic                    0        / 0           1.19209290e-07 / 1.37007273e-05
//     gate_fleet                   0        / 0           1.19209290e-07 / 2.38535932e-07
//     bounce                       1.19e-07 / 1.05e-07    2.38418579e-07 / 3.41500879e-07
//     two_world_isolation          0        / 0           4.76837158e-07 / 1.63916241e-07
//     quad_hover                   0        / 0           1.78813934e-07 / 6.26855638e-07
//     contact_pair                 0        / 0           5.96046448e-08 / 8.89908023e-08
//     drag_componentwise           0        / 0           5.96046448e-08 / 7.05371122e-08
//     restore_resume               0        / 0           1.19209290e-07 / 1.63440987e-07
//     heterogeneous_geometry_set   0        / 0           2.38418579e-07 / 2.73745087e-07
//     shower                       0        / 0           5.96046448e-08 / 1.30886846e-07
//     shower_ladder                0        / 0           1.19209290e-07 / 3.43488580e-07
//
//     worst abs  4.76837158e-07 (two_world_isolation)  -> band 2.0e-6  (4.2x)
//     worst rel  1.37007273e-05 (ballistic)            -> band 6.0e-5  (4.4x)
//     stream.cached worst 1.19209290e-07 / 1.05459421e-07 (bounce) -> 5.0e-7 (4.2x)
//
// WHERE THE DIVERGENCE COMES FROM, exactly, because the list is short enough to
// be exhaustive. The filter's transcendentals are exp32 (the coefficients) and
// log32/sin32/cos32 (Box-Muller), all T4 ports proven bit-identical to the host
// on this device; the integer half of splitmix64 is exact by construction
// (engine/shaders/u64.slang). What is left is the arithmetic Vulkan does NOT
// specify to be correctly rounded: Box-Muller's `sqrt(-2 * log32(u1))`, the
// Cholesky's three square roots and its one division, and the step ratio's
// division. That is the whole list, and one-to-eight ulps after up to 4500
// substeps is what it costs.
//
// BALLISTIC'S 1.37e-05 RELATIVE IS THE LARGEST FIGURE HERE AND IS A NEAR-ZERO
// ARTEFACT, not an outlier in the arithmetic: max_abs/max_rel bounds the
// producing component's own magnitude at 1.19209290e-07 / 1.37007273e-05 =
// 8.7e-03, a filter state that happens to be passing close to zero at the final
// tick. The absolute figure there is one ulp.
// ---------------------------------------------------------------------------
namespace medium {
inline constexpr ToleranceBand kFilterState{2.0e-6f, 6.0e-5f};
inline constexpr ToleranceBand kCachedGauss{5.0e-7f, 5.0e-7f};
}  // namespace medium

// ---------------------------------------------------------------------------
// two_world_isolation -- the corpus scenario VERBATIM, 300 steps x 2 substeps
// (1 kHz), TWO worlds at overlapping coordinates, eight bodies in world 0 and
// two in world 1, `turbulence: moderate` on both with DIFFERENT seeds. In the
// parity set for the first time at S6 Task 8: it was excluded through Task 7
// because MediumUpdate was a stub.
//
// WHAT IT ADDS: the only scenario in the file with two differently-seeded
// TURBULENT worlds, so a kernel that served both worlds one world's `dryden` or
// `dryden_params` row fails here and nowhere else. It is also the broad phase's
// cross-world leak case (world 1's bodies sit inside world 0's cluster).
//
// MEASURED (32 elements = two 16-slot body partitions; 10 live):
//     pos             max|abs| 0                max rel 0             <- BIT-EXACT
//     vel             max|abs| 5.56146938e-29   max rel 3.41933774e-05  (!)
//     orient          max|abs| 0                max rel 0             <- BIT-EXACT
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 0                max rel 0             <- BIT-EXACT
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     quaternion unit-norm error 0 on both sides
//     (the `dryden` rows use the shared `medium` block above)
//
// THE ONE NON-ZERO ROW IS 5.6e-29 METRES PER SECOND, and the exponent is not a
// typo. The worst element is a settled body's vel.x at cpu = 1.626475682e-24
// against gpu = 1.626531297e-24: both are the residue of a Baumgarte correction
// and a friction cap cancelling each other to within a few ulps of a quantity
// that is already twenty-four orders below anything physical. The RELATIVE
// figure (3.4e-05) is what dividing one such residue by another produces, and
// it is meaningless in the same way. The band is 4x the measurement on both
// halves rather than a round "close enough to zero" number, because a band that
// large would stop catching a real velocity error on these bodies -- which
// otherwise reproduce EXACTLY.
//
// EVERYTHING ELSE IS BIT-EXACT, structurally so rather than luckily, for
// `bounce`'s reasons one for one: the bodies spawn mass 0.5 and
// inv_inertia_diag (200,200,200) -- SPHERICAL -- with omega_body zero under a
// linear-only contact model that generates no torque, so the exp-map takes its
// small-angle branch at theta = 0 every substep and the orientation never
// moves; and the world SDF is empty.
//
// THE GUST REACHES NO BODY HERE, which is the point rather than a gap: a gust
// enters force_acc only through a drag element (physics/forces.cpp samples the
// medium per element) and this scenario spawns none. So what this scenario
// proves is that the filter runs PER WORLD, correctly, on two different seeds,
// WITHOUT perturbing anything -- and the ParityCheck attached to its test
// additionally requires the two worlds' states to differ from each other, which
// is the half no CPU<->GPU band can see.
// ---------------------------------------------------------------------------
namespace two_world_isolation {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
// 9.0x and 5.9x the measured 5.56146938e-29 / 3.41933774e-05 -- NOT the ~4x
// this file's margin rule nominally asks for, and the overshoot is the "rounded
// UP to a round decimal" half of that rule landing badly rather than a decision
// to loosen. 4x the two measurements is 2.22e-28 and 1.37e-04; both sit just
// above a decade boundary, so the next round decimal above each is 5.0e-28 and
// 2.0e-04. Tightening to 2.5e-28 / 1.5e-04 would honour the factor at the cost
// of a band that is no longer a round number -- the factor is the
// approximation and the round decimal is the convention, so the convention wins
// and the REAL margins are recorded here rather than implied (S6 Task 8 review,
// minor M7).
//
// It costs nothing that matters: the band is still twenty-four orders of
// magnitude below anything physical in this scenario, and every other row here
// is pinned at exactly zero. See the block note above for why this row is
// pinned at an absurd-looking exponent rather than at zero as well.
inline constexpr ToleranceBand kVel{5.0e-28f, 2.0e-4f};
inline constexpr ToleranceBand kOrient{0.0f, 0.0f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace two_world_isolation

// ---------------------------------------------------------------------------
// quad_hover -- the corpus scenario VERBATIM, and THE ONE WAVE C EXISTS FOR:
// two worlds, one quadrotor each (one body + four rotors + a drag body + an IMU
// mount, from one registered ModelType), spawned in exact hover trim, flying a
// three-step collective script over 900 steps x 2 substeps with
// `turbulence: moderate`. Excluded through Task 7 on THREE counts at once
// (rotors, sensors, turbulence); all three are ported here.
//
// IT IS A CLOSED LOOP THROUGH EVERY PORTED PASS, which is why its body rows are
// the loosest in the file while its component rows are the tightest: the gust
// perturbs the rotors' axial inflow AND the drag body, the inflow correction
// scales thrust, thrust changes attitude, attitude changes the drag rotation and
// the next substep's inflow -- 1800 times. Every division and square root in
// rotors.slang (v_h, both momentum-theory branches, the ground factor, the lag
// ratio) sits inside that loop.
//
// MEASURED (2 body elements, one per world; 10 rotor rows, 8 live; 2 sensors;
// 128 ring samples, the resident tail of 1800 written):
//     pos             max|abs| 3.81469727e-06   max rel 7.90625336e-07
//     vel             max|abs| 2.83122063e-07   max rel 2.15674072e-06
//     orient          max|abs| 0                max rel 0             <- BIT-EXACT
//     omega_body      max|abs| 0                max rel 0             <- BIT-EXACT
//     specific_force  max|abs| 4.76837158e-07   max rel 5.24325969e-07
//     force_acc       max|abs| 0                max rel 0             <- BIT-EXACT
//     torque_acc      max|abs| 0                max rel 0             <- BIT-EXACT
//     rotors.omega    max|abs| 0                max rel 0             <- BIT-EXACT
//     imu bias_a      max|abs| 0                max rel 0             <- BIT-EXACT
//     imu bias_g      max|abs| 0                max rel 0             <- BIT-EXACT
//     imu noise.cached max|abs| 0               max rel 0             <- BIT-EXACT
//     imu_ring accel  max|abs| 1.43051147e-06   max rel 6.60766204e-07
//     imu_ring gyro   max|abs| 0                max rel 0             <- BIT-EXACT
//     phase, both rng streams and their flags, last_index, sample index and
//     sample tick: ALL BIT-EXACT (integer rows)
//     quaternion unit-norm error 0 on both sides
//
// ===========================================================================
// `rotors.omega` IS BIT-EXACT AFTER 1800 CHAINED APPLICATIONS OF THE RPM LAG,
// AND THAT IS THIS BLOCK'S STRONGEST RESULT.
//
// The recurrence is `omega += (omega_cmd - omega) * (1 - exp(-h/tau))`, applied
// every substep, with the collective stepped twice mid-flight. Reproducing it
// exactly for 1800 iterations means exp32's port agreed with the host on every
// evaluation AND the series branch was taken on the same side of
// kRotorLagSeriesThreshold every time (h/tau = 0.001/0.02 = 0.05 here, so the
// Taylor branch, on both paths). A one-ulp difference anywhere in that chain
// would have compounded visibly.
//
// `orient` AND `omega_body` ARE BIT-EXACT while `pos` and `vel` are not, which
// looks backwards until the wrench is followed: the four rotors are mounted
// symmetrically and commanded identically, so their torques cancel to exactly
// zero on both paths and the airframe never rotates -- the exp-map takes its
// small-angle branch at theta = 0 every substep, exactly as in `bounce`. The
// TRANSLATIONAL chain has no such cancellation: thrust*axis_world accumulates,
// and every ulp of inflow correction lands in it.
//
// `imu_ring gyro` IS BIT-EXACT FOR THE SAME REASON -- it is `conjugate(mount) *
// omega_body`, and omega_body is exactly zero throughout -- while `accel` is
// not, because it carries `specific_force`, which is the accumulated wrench
// divided by mass and rotated. The gyro row is therefore NOT vacuous: it proves
// the mount rotation reproduces a zero vector exactly, and the tick/index rows
// beside it prove the samples were emitted on the same substeps at all.
//
// THE BIAS ROWS ARE BIT-EXACT STRUCTURALLY: this model authors sigma_ba =
// sigma_bg = 0, so the walk adds `0 * n` to a bias that starts at exactly zero.
// The DRAWS still happen -- twelve per emitted sample, unconditionally
// (sensors/imu.hpp section 4) -- so `noise.state` being bit-exact is a real
// claim about 21600 gaussian draws per sensor, and it is the bias VALUE that
// has nothing to diverge.
// ===========================================================================
// WELL INSIDE THE STOP RULE. The brief's tripwire is 1e-3 relative at a
// non-near-zero absolute; the largest relative figure anywhere in this block is
// 2.16e-06, nearly three orders inside it.
//
// Bands are 4x measured, rounded up to a round decimal; a row measured exactly
// zero is pinned at exactly zero, the posture `bounce`'s header explains.
// ---------------------------------------------------------------------------
namespace quad_hover {
inline constexpr ToleranceBand kPos{2.0e-5f, 4.0e-6f};
inline constexpr ToleranceBand kVel{2.0e-6f, 1.0e-5f};
inline constexpr ToleranceBand kOrient{0.0f, 0.0f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{2.0e-6f, 4.0e-6f};
// The RPM lag's own state -- the one field the rotor pass writes. Measured
// bit-exact over 1800 substeps; pinned there.
inline constexpr ToleranceBand kRotorOmega{0.0f, 0.0f};
// Both bias random walks; bit-exact because this model's walk sigmas are zero.
inline constexpr ToleranceBand kBias{0.0f, 0.0f};
// The Box-Muller partner value carried in each sensor's stream.
inline constexpr ToleranceBand kCachedGauss{0.0f, 0.0f};
// The sample values themselves, mount frame. `accel` is the only ring row that
// moves; `gyro` is bit-exact because this airframe never rotates.
inline constexpr ToleranceBand kAccel{8.0e-6f, 4.0e-6f};
inline constexpr ToleranceBand kGyro{0.0f, 0.0f};
}  // namespace quad_hover

// ---------------------------------------------------------------------------
// gnss_receiver -- a HAND-BUILT scenario (test_gpu_parity.cpp), not a corpus
// one, and the first thing in this tree ever to run sensor_gnss.slang over a
// LIVE row. Measured 2026-09-23 on MSVC + Vulkan, 200 substeps, one receiver at
// rate_divider 1, over 64 ring slots.
//
// Same posture as every namespace above: 4x measured, rounded up to one
// significant figure; a row measured exactly zero is pinned at exactly zero.
//
// ⭐ NINE OF TWELVE QUANTITIES CAME BACK BIT-EXACT UNDER A ZERO BAND -- phase,
// the whole rng stream INCLUDING the cached Box-Muller partner (a float),
// last_index, the fix's sigma_h/sigma_v, and index/tick. So the two backends
// agree exactly on WHEN a fix is emitted, HOW MANY are emitted, WHICH random
// draws feed it and WHICH ROW it came from; only the arithmetic diverges.
//
// ⚠ AND THIS SCENARIO DELIBERATELY RUNS NON-ZERO SIGMAS, WHICH IS THE OPPOSITE
// OF quad_hover's CHOICE AND FOR A REASON WORTH STATING. That IMU sensor
// authors every sigma zero, which its comment notes makes the sample bands "a
// clean measurement of the mount rotation rather than of accumulated noise".
// Here sigma_bias MUST be non-zero or bias_drive is zero, the Gauss-Markov
// state never leaves zero, and the `bias` row would be bit-exact FOR THE WRONG
// REASON -- a band measuring a quantity that cannot move. The cost is that
// position/velocity carry accumulated noise; the benefit is that the bias path
// is compared at all.
// ---------------------------------------------------------------------------
namespace gnss_receiver {
// ⛔ THE THREE NUMBERS BELOW WERE MEASURED. THE CAUSES ORIGINALLY WRITTEN
// BESIDE THEM WERE GUESSED, AND TWO OF THE THREE WERE WRONG -- corrected here
// by tracing each one to a file that decides it.
//
// The first version of this block blamed `bias` on FMA contraction and
// `position` on Vulkan's <= 2.5 ulp divide/sqrt. Neither can happen on this
// path, and the tree says so in four places. That mistake is worth a comment
// rather than a silent fix, because of the SHAPE it had: THE DELTA AND ITS
// EXPLANATION WERE WRITTEN IN THE SAME SENTENCE, so the guess inherited the
// measurement's credibility. A number measured under a pre-registered zero
// band is authority for THAT the band is 4e-6; it is authority for NOTHING
// about WHY -- and the zero-band discipline that produced these numbers
// governs the quantity only, never the stated cause.
// ---------------------------------------------------------------------------

// The Gauss-Markov bias state. Measured abs 3.73e-09, rel 3.15e-07 -- roughly
// 4 ulp at this magnitude.
//
// ⛔ CAUSE UNRESOLVED, AND THIS BAND IS PROVISIONAL. Every input to
// `bias = bias * bias_retention + bias_drive * bias_walk` is identical on the
// two backends: the expression is character-for-character the same at
// sensors/gnss.cpp:137 and in sensor_gnss.slang (same grouping); the two
// coefficients are CPU-precomputed row config, and run_parity's state_digest
// precondition proves the rows START byte-identical; the draws come from a
// stream whose `noise.state` and `noise.cached` both compare BIT-EXACT.
//
// It is NOT contraction, which was the original guess. Contraction is off on
// both sides AND enforced: engine/CMakeLists.txt:22 (MSVC defaults /fp:precise,
// "never fuses a multiply-add") and :77 (-ffp-contract=off for non-MSVC);
// cmake/SpadeSlang.cmake compiles every kernel -fp-mode precise; and
// tests/test_slang_layouts.cpp's SlangSpirv.FloatControlsPinned scans
// sensor_gnss's SPIR-V -- every workgroup variant -- asserting NoContraction on
// every contractable op. A cause an existing test forbids is not a cause.
//
// The one candidate left on this path is rng.slang's Box-Muller sqrt, the only
// operation here carrying a documented <= 2.5 ulp licence. fp32_math.slang's
// SQRT AUDIT already names it and defers it ("Box-Muller's, OUTSIDE these
// kernels, and is a later task's problem"). ⚠ `noise.cached` comparing
// bit-exact is evidence AGAINST that candidate, but only partial: nine draws
// is odd, so the cache holds draw 9's twin and the earlier pairs' magnitudes
// were consumed. It is evidence at the sampled points, not a proof over every
// draw.
//
// ⛔⛔ AND THIS BAND HAS A SILENT EXPIRY DATE, WHICH IS WHY IT IS THE ONE TO
// SETTLE FIRST. sensor_gnss.slang's header pre-registered this quantity as a
// BIT-IDENTITY claim, not a banded one, and said why: "A RECURSIVE BIAS FILTER
// DOES NOT TOLERATE ERROR, IT ACCUMULATES IT ... 'close enough' is a different
// answer next hour." THIS NUMBER WAS MEASURED OVER 200 FIXES. Its adequacy is
// therefore a function of run length, and no test on this path states a run
// length -- so a longer scenario can walk out of this band with nothing having
// changed. The one band here that compounds is the one that was reasoned about
// least. Do not widen it to make a longer run pass; that converts a compounding
// divergence into a permanently invisible one.
inline constexpr ToleranceBand kGnssBias{2.0e-8f, 2.0e-6f};

// The reported antenna position. Measured abs 9.54e-07 on values near 4.09 --
// 2 ulp.
//
// ⛔⛔ SECOND ERRATUM, 2026-09-23, AND IT IS IN THE TEXT THAT FIXED THE FIRST.
// This block previously read: "IT READS BANDED INPUTS ... body_bands() bands
// `bodies.pos` and `bodies.orient` ... only force_acc and torque_acc are pinned
// at zero there. A sensor that reports a banded body's state CANNOT be
// bit-exact." THE MECHANISM IS WRONG AND IT IS WRONG THE SAME WAY THE FMA GUESS
// IT REPLACED WAS WRONG.
//
// MEASURED: body_bands() (tests/test_gpu_parity.cpp) BANDS NOTHING. It is a
// table constructor taking pos/vel/orient/omega/specific_force AS PARAMETERS;
// only force_acc and torque_acc are hardcoded at zero. Whether `pos` is banded
// is a property of THE CALLING SCENARIO, and across the eleven corpus
// scenarios it goes both ways:
//     kPos    BIT-EXACT in ballistic, bounce, restore_resume, contact_pair,
//             two_world_isolation                                    (5 of 11)
//     kOrient BIT-EXACT in bounce, shower, shower_ladder,
//             two_world_isolation, quad_hover                        (5 of 11)
// ballistic -- gravity, one body, no contacts, the closest analogue to this
// scenario -- has pos BIT-EXACT and orient BANDED. So the sentence asserted as
// a structural property of the harness something that is a per-run outcome, and
// asserted it in the direction the conclusion needed.
//
// ⛔ AND THE RUN THIS BAND COMES FROM NEVER COMPARED A BODY. gnss_receiver
// passes gnss_bands() ALONE -- every other scenario in this file passes
// join(body_bands(...), ...) and this one does not. THE BODY STATE IS
// UNMEASURED HERE. The cited cause was inferred from other scenarios' pins and
// written as though observed in this one.
//
// > A CITED GUESS IS HARDER TO CATCH THAN AN UNCITED ONE. The FMA guess named
// > no source and was found in a day. This one named a function and a file, so
// > it READ as measured -- and it survived the commit that existed to correct
// > guessed causes, a coordinator's review, and my own sweep for exactly this
// > defect. Naming a source is what makes a guess look checked.
//
// ✅ SETTLED BY MEASUREMENT 2026-09-24, and the hypothesis this block carried
// for a few hours was RIGHT while the cited guess before it was WRONG.
// GnssReceiverBodyStateIsMeasuredNotAssumed runs this same scenario with
// body_bands() at a zero band, and the answer is:
//     pos BIT-EXACT . vel BIT-EXACT . omega_body BIT-EXACT .
//     specific_force / force_acc / torque_acc BIT-EXACT .
//     orient  abs 5.96046448e-08  rel 6.52729909e-07  <- THE ONLY DIVERGENCE
//
// ⭐ SO THE CAUSE IS ONE QUANTITY, NOT A CLASS OF THEM: fix.position is
// `bodies.pos + lever_world + bias + noise`, `bodies.pos` is BIT-EXACT here,
// and the entire band is the orientation quaternion reaching the report through
// lever_world. The original comment named pos first and treated it as the
// primary contributor; it contributes nothing.
//
// ⚠ AND THE CONCLUSION IS UNCHANGED WHILE EVERY STATED REASON FOR IT HAS NOW
// BEEN WRONG TWICE. The band is right, `position` still cannot be bit-exact,
// and no edit inside sensor_gnss.slang can change that -- but the first reason
// was FMA contraction, the second was "body_bands() bands pos and orient", and
// only the third is measured. A CONCLUSION THAT SURVIVES EVERY REFUTATION OF
// ITS OWN REASONING IS NOT THEREBY WELL-SUPPORTED; it was just easy to reach
// from several directions.
//
// ⛔ NOT divide or sqrt, which was the ORIGINAL guess: sensor_gnss.slang
// contains neither, and its own header says so outright ("NO exp(). NO
// DIVISION ... DIVISION AND SQRT SITES: none in this file"). The Gauss-Markov
// coefficients are precomputed on the CPU precisely so that this kernel only
// multiplies. That exclusion is measured and stands.
inline constexpr ToleranceBand kGnssPosition{4.0e-6f, 4.0e-6f};

// The reported antenna velocity, which carries the omega x lever-arm cross
// product. Measured abs 2.38e-07 on values near 2.17 -- exactly 1 ulp.
//
// ⛔ SAME ERRATUM AS position, and it was WORSE HERE because this block said
// the cause was "worth stating rather than leaving to inference from the
// neighbour" -- it presented an inference as an independent observation while
// announcing that it was not merely inheriting one.
//
// It read: "body_bands() bands bodies.vel, bodies.omega_body AND bodies.orient
// -- all three of this row's physical inputs." body_bands() bands NONE of
// them; they are parameters, several scenarios pin them bit-exact, and
// gnss_receiver never compares a body at all. See the position block above for
// the measurement.
//
// ✅ SETTLED BY THE SAME MEASUREMENT, and this row's original claim was wrong
// on TWO of the three inputs it named. fix.velocity is `bodies.vel +
// cross(omega_world, lever_world) + noise`; the block said body_bands() "bands
// bodies.vel, bodies.omega_body AND bodies.orient -- all three of this row's
// physical inputs". MEASURED: bodies.vel is BIT-EXACT and bodies.omega_body is
// BIT-EXACT. ONLY `orient` diverges, and it reaches this row exactly as it
// reaches position's -- through lever_world.
//
// The shape observation survives and is now better supported: EXACTLY 1 ULP is
// what a single slightly-divergent input reproduced through an exact arithmetic
// path looks like. With two of three inputs proven bit-exact, that is no longer
// an inference about "banded inputs" in general -- it is one input, named.
inline constexpr ToleranceBand kGnssVelocity{1.0e-6f, 5.0e-7f};
}  // namespace gnss_receiver

// ---------------------------------------------------------------------------
// gnss_receiver, THE BODY ROWS -- the inputs kGnssPosition and kGnssVelocity
// actually read, measured rather than inferred from other scenarios' pins.
//
// ⛔ PASS 1 IS PRE-REGISTERED TO FAIL, exactly as the gnss rows above were
// pinned: every band here is ZERO, so the harness prints the delta for anything
// genuinely band-legal and THE FAILURE REPORT IS THE MEASUREMENT. A quantity
// that comes back exact under a zero band is a bit-exact claim on its merits
// and keeps the zero -- and in this scenario that is a live possibility rather
// than a formality, because `ballistic` (gravity, one body, no contacts, the
// closest analogue) pins pos BIT-EXACT and orient BANDED.
//
// Guessing these would reproduce the exact defect this block exists to correct.
// ---------------------------------------------------------------------------
namespace gnss_receiver_body {
// MEASURED 2026-09-24, pass 1, this box, msvc-ninja-release, Vulkan on Intel
// Iris Plus. 2 body slots compared, 1 live. THE RESULT REFUTED THE CAUSE THAT
// HAD BEEN WRITTEN FOR kGnssPosition/kGnssVelocity ON TWO OF THREE COUNTS:
//     pos             abs 0               rel 0              <- BIT-EXACT
//     vel             abs 0               rel 0              <- BIT-EXACT
//     orient          abs 5.96046448e-08  rel 6.52729909e-07 <- THE ONLY ONE
//     omega_body      abs 0               rel 0              <- BIT-EXACT
//     specific_force  abs 0               rel 0              <- BIT-EXACT
//     force_acc       abs 0               rel 0              <- BIT-EXACT
//     torque_acc      abs 0               rel 0              <- BIT-EXACT
// worst: element 0 component 3 (the quaternion's w), cpu 9.963023663e-01 vs
// gpu 9.963023067e-01.
//
// ⭐ SO THE ENTIRE GNSS POSITION/VELOCITY DIVERGENCE ENTERS THROUGH EXACTLY ONE
// QUANTITY: THE BODY'S ORIENTATION QUATERNION, via lever_world. Not pos, not
// vel, not omega_body -- all three were named as banded inputs and all three
// are bit-exact here. The six zeros are kept AT ZERO rather than given nominal
// bands, the posture `bounce`'s header sets out: a nominal band would assert
// less than the measurement supports and would silently absorb a real change.
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{0.0f, 0.0f};
// ~4x the measured maximum, rounded up to a round decimal, per this file's
// standing margin rule. Far tighter than ballistic's {4.0e-6, 2.0e-5} because
// this scenario has no contacts and no force elements.
inline constexpr ToleranceBand kOrient{4.0e-7f, 4.0e-6f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
}  // namespace gnss_receiver_body

// ---------------------------------------------------------------------------
// gnss_tumble -- the corpus GNSS scenario (PHY-6), measured, not borrowed.
//
// 2 worlds x 1600 substeps, one receiver per world at rate_divider 8 (200 fixes
// each), the body tumbling torque-free.
//
// MEASURED 2026-10-03 under a pre-registered zero band, so the failure report
// was the measurement. Device: Intel Iris Plus Graphics, Vulkan, this box;
// msvc-ninja-release; tree master 4b3f9f4 plus this change, bands at zero.
//     bodies pos, vel, omega_body, specific_force   0 (bit-exact)
//     bodies orient       abs 9.54e-07  rel 2.43e-05  (worst: world 0, w)
//     gnss_sensors bias   abs 5.96e-08  rel 2.01e-07  (1 ulp at |bias| 0.63 m)
//     gnss_sensors noise.cached                      0 (bit-exact)
//     gnss_ring position  abs 7.63e-06  rel 5.24e-06  (2 ulp at 36 m)
//     gnss_ring velocity  abs 2.38e-06  rel 2.16e-06  (5 ulp at 4 m/s)
//
// Two causes reach the fix, and this run does not separate them. The body's
// orientation enters through the lever arm; it is the only body quantity that
// diverges, as gnss_receiver_body found. The gaussian draws enter through the
// noise and the bias; CORE-3 owns that band, so it is cited here, not
// re-derived. The bias stayed at 1 ulp over 200 fixes; gnss_receiver's note on
// a compounding bias still applies to a longer run.
//
// Bands are 4x measured, rounded up to one significant figure. A quantity
// measured exactly zero stays at zero.
// ---------------------------------------------------------------------------
namespace gnss_tumble {
inline constexpr ToleranceBand kPos{0.0f, 0.0f};
inline constexpr ToleranceBand kVel{0.0f, 0.0f};
inline constexpr ToleranceBand kOrient{4.0e-6f, 1.0e-4f};
inline constexpr ToleranceBand kOmega{0.0f, 0.0f};
inline constexpr ToleranceBand kSpecificForce{0.0f, 0.0f};
inline constexpr ToleranceBand kGnssBias{3.0e-7f, 9.0e-7f};
inline constexpr ToleranceBand kCachedGauss{0.0f, 0.0f};
inline constexpr ToleranceBand kGnssPosition{4.0e-5f, 3.0e-5f};
inline constexpr ToleranceBand kGnssVelocity{1.0e-5f, 9.0e-6f};
}  // namespace gnss_tumble

}  // namespace bands

}  // namespace spade::testing
