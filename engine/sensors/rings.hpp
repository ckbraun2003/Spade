#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

// ---------------------------------------------------------------------------
// SENSOR OUTPUT RINGS -- the tick-stamped, monotonically indexed sample buffer
// every sensor writes into (the SampleIndex/poll convention comes from the
// EDITOR TECH SPEC's TA5 SENSOR-POLL CONVENTION; this header is that
// convention made executable on the engine side).
//
// TA5, restated as the three properties this file implements:
//
//   1. EVERY SAMPLE CARRIES A MONOTONICALLY INCREASING SampleIndex, per sensor.
//      Indices start at 1 and never repeat, never skip and never rewind (except
//      by a snapshot restore, which rewinds the whole engine). Index 0 is
//      reserved for "no sample", matching core/ids.hpp's "generation 0 means
//      never issued" convention -- which is what makes `poll(sensor, 0)` mean
//      "give me everything you have" rather than "give me everything after the
//      first sample".
//   2. `poll(sensor, since_index)` RETURNS THE SAMPLES WITH INDEX > since_index
//      THAT ARE STILL RESIDENT in the fixed-depth ring. A ring holds the last
//      kRingDepth samples and nothing else, so a caller that polls too slowly
//      loses the oldest ones. That loss is REPORTED (PollResult::dropped)
//      rather than silent: an estimator that quietly misses IMU samples is a
//      bug that looks like bad tuning.
//   3. EVERY SAMPLE IS TICK-STAMPED with the step it was produced in.
//
// ---------------------------------------------------------------------------
// THE STORAGE IS NOT HERE, AND THAT IS THE POINT
//
// This header holds NO storage and NO object with a lifetime -- only the index
// arithmetic and the two operations, over a `std::span` window the caller
// supplies. Ring storage lives in a REGISTERED ArenaSet array (see
// sim/simulation.cpp's "imu_ring"), because a ring that determinism-after-
// restore depends on has to be in the state registry's walk like every other
// authoritative byte (state/registry.hpp's "no unregistered state" invariant).
// A ring class that owned a std::vector would be exactly the state a snapshot
// misses.
//
// The per-sensor WRITE CURSOR is likewise not here: it is a field of the
// sensor's own registered row (ImuSensorRow::last_index), so it rides every
// snapshot alongside the samples it indexes. Every function below therefore
// takes `last_index` as an argument rather than remembering it.
//
// ---------------------------------------------------------------------------
// WHY poll() COPIES INTO A CALLER BUFFER instead of returning a span into the
// ring itself: a resident range that has WRAPPED is two disjoint pieces of the
// window, which is not one span. The alternatives were returning two spans (an
// API every caller has to remember to loop over twice, and the one that will
// be got wrong) or copying. Copying kRingDepth * sizeof(Sample) bytes at a
// poll rate of 25-200 Hz is nothing, and it hands the caller ONE span in index
// order with the wrap already resolved.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// A sensor sample's monotonically increasing identity (editor tech spec TA5).
// A plain uint64 --
// it is a field of a std430 POD row that S6's Slang mirror must agree with
// byte for byte, so a typed wrapper would have to be unwrapped at exactly the
// boundary where the type safety mattered.
//
// 0 means "no sample": the value ImuSensorRow::last_index holds before the
// first sample, and the `since_index` a first-time poller passes.
using SampleIndex = uint64_t;

// ---------------------------------------------------------------------------
// The ring depth, in samples. FIXED AT COMPILE TIME, for the same reason every
// other capacity in this engine is fixed at create(): nothing reallocates
// mid-run, and the S6 device buffer's stride is a constant rather than a
// per-world lookup.
//
// 64 is sized against the rate contracts the foundation spec states (25 Hz
// vision, 200 Hz supervisor) and the substep rates this engine runs at: a
// 1 kHz IMU polled by a 200 Hz consumer produces 5 samples per poll, so 64
// leaves an order of magnitude of headroom for a consumer that skips a beat.
// A consumer that wants more history buffers it itself; a ring is a hand-off
// buffer, not a log.
// ---------------------------------------------------------------------------
inline constexpr uint32_t kRingDepth = 64;

// Where sample `index` lives in a depth-kRingDepth window.
//
// Indices start at 1, so the -1 puts the FIRST sample in slot 0 -- which makes
// a freshly-filled ring read in index order with no rotation, and makes a
// hand-inspected ring in a debugger obvious rather than off by one.
[[nodiscard]] constexpr uint32_t ring_slot_of(SampleIndex index) noexcept {
    return static_cast<uint32_t>((index - 1u) % kRingDepth);
}

// The oldest index still resident in a ring whose newest is `last_index`.
// Returns 1 (the first index there could ever be) until the ring has wrapped.
[[nodiscard]] constexpr SampleIndex ring_oldest_resident(SampleIndex last_index) noexcept {
    return last_index > kRingDepth ? last_index - kRingDepth + 1u : SampleIndex{1};
}

// ---------------------------------------------------------------------------
// Writes `sample` at its own index. `window` is ONE sensor's depth-kRingDepth
// slice of the ring array.
//
// WHOLE-OBJECT ASSIGNMENT IS DELIBERATE AND SAFE HERE, which is worth stating
// because sim/simulation.cpp writes every other arena row FIELD-WISE. That rule
// exists because WorldParams has real, addressable TAIL PADDING that a stack
// temporary would carry indeterminate bytes into. Sample types used with this
// ring must static_assert that every byte belongs to a named field (ImuSample
// does), so a fully-assigned temporary has no indeterminate byte to copy.
// ---------------------------------------------------------------------------
template <class Sample>
void ring_write(std::span<Sample> window, SampleIndex index, const Sample& sample) noexcept {
    window[ring_slot_of(index)] = sample;
}

// ---------------------------------------------------------------------------
// What a poll returns.
//
// `samples` spans the caller's own output buffer, OLDEST FIRST, and is empty
// when nothing new is resident.
//
// `next_since` is what to pass as `since_index` next time. It is the index of
// the last sample returned, or the caller's own `since_index` when nothing was
// returned -- never a value that would skip a sample the caller has not seen.
// Threading it back is what makes a poll loop correct without the caller doing
// index arithmetic.
//
// `dropped` counts samples that fell out of the ring between `since_index` and
// the oldest sample returned -- i.e. how many the caller LOST by polling too
// slowly. Zero in the healthy case.
//
// TRUNCATION: when `out` is smaller than the resident range, the OLDEST
// samples that fit are returned and `next_since` stops there, so the caller
// gets the rest on the next poll. Polling never silently jumps forward.
// ---------------------------------------------------------------------------
template <class Sample>
struct PollResult {
    std::span<const Sample> samples;
    SampleIndex next_since = 0;
    uint64_t dropped = 0;
};

// ---------------------------------------------------------------------------
// Copies the samples with index > `since_index` that are still resident in
// `window` (whose newest sample is `last_index`) into `out`.
//
// Preconditions, documented rather than checked -- the same posture as the
// physics passes, since this sits under the poll path: `window.size()` is
// kRingDepth, and `window` was only ever written through ring_write().
// ---------------------------------------------------------------------------
template <class Sample>
[[nodiscard]] PollResult<Sample> ring_poll(std::span<const Sample> window, SampleIndex last_index,
                                           SampleIndex since_index, std::span<Sample> out) noexcept {
    PollResult<Sample> result;
    result.next_since = since_index;

    // Nothing produced yet, or the caller is already up to date. Checked BEFORE
    // any `since_index + 1`, so a caller passing the maximum uint64 cannot wrap
    // the arithmetic below.
    if (last_index == 0 || since_index >= last_index || out.empty()) {
        return result;
    }

    const SampleIndex oldest = ring_oldest_resident(last_index);
    const SampleIndex wanted = since_index + 1u;
    const SampleIndex first = wanted > oldest ? wanted : oldest;

    result.dropped = first - wanted;

    uint64_t count = last_index - first + 1u;
    if (count > out.size()) {
        count = out.size();  // truncated; next_since stops here, caller re-polls
    }

    for (uint64_t i = 0; i < count; ++i) {
        out[static_cast<std::size_t>(i)] = window[ring_slot_of(first + i)];
    }

    result.samples = std::span<const Sample>(out.data(), static_cast<std::size_t>(count));
    result.next_since = first + count - 1u;
    return result;
}

}  // namespace spade::sensors
