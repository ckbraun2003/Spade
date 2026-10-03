#pragma once

// ---------------------------------------------------------------------------
// timestamps.hpp (S6 Task 10) -- per-pass GPU timing via Vulkan timestamp
// queries, so the bench sweep's "how fast is each pass" numbers are measured
// rather than guessed (task brief, quoting spec section 9).
//
// RECORD-ONCE COMPLIANCE. compute/vulkan/step_recorder.hpp's StepRecorder
// records its whole per-substep dispatch chain EXACTLY ONCE (at create()) and
// resubmits the same VkCommandBuffer verbatim on every step -- this module's
// query writes are baked into that SAME recording, not re-recorded per
// submit, for the identical reason every dispatch in that chain is: a value
// that must vary per submit cannot live in something fixed at record time.
// What DOES vary per submit is the query POOL's CONTENTS -- each execution of
// the recorded buffer resets every query slot it owns (record_reset(), called
// once, at the very top of the recording) and then rewrites every slot fresh
// (record_mark(), one call per pass boundary) -- so "results read per submit"
// (this task's brief) is literally true: read_durations_ns() after any one
// submit's fence wait sees THAT submit's numbers, because the recorded reset
// wiped the previous submit's before this one wrote its own.
//
// SKIP-GRACEFULLY, RECORDED POSTURE. Not every device/queue can time compute
// work: VkPhysicalDeviceLimits::timestampComputeAndGraphics may be VK_FALSE,
// or the compute queue family's VkQueueFamilyProperties::timestampValidBits
// may be 0 (a family that supports compute but not timing it -- legal per the
// spec). create() below never fails for either reason -- it returns a valid
// object with supported() == false, and every recording method on it becomes
// a documented no-op (nothing written to the command buffer at all, not even
// a zero-cost placeholder), which is what "zero-cost when timestamps are
// off" (this task's brief) means for a caller that unconditionally calls
// record_reset()/record_mark() regardless of device capability -- exactly
// what StepRecorder now does (its own header/impl carry the wiring).
//
// ONE BRACKET PER GPU PASS, NAMED (module-API plan, stage 2). The passes are
// the compiled schedule's, in schedule order, each named "<module>.<pass>" --
// the same list StepRecorder records. A pass whose recipe dispatches nothing
// (the two behavior passes) still gets its bracket, so it reports a MEASURED
// ~0 ns rather than an absent one.
//
// ONE MARK BLOCK PER SUBSTEP. A shape's recorded chain repeats
// `shape.substeps` times (StepRecorder::record()'s own substep loop), and
// this object needs one block of (passes + 1) marks -- 1 "start of this
// substep" mark + one "this pass just finished" mark per pass -- per
// repetition, because the pass boundaries themselves repeat.
// read_durations_ns() sums each pass's duration ACROSS every substep in the
// recording -- "how long did dynamic_contact.resolve run, in total, for one
// full step()" is the number a bench counter wants, not a single substep's
// slice of it.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <volk.h>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/error.hpp"

namespace spade::compute {

class PassTimestamps {
public:
    // ctx: non-owning, exactly like every other vulkan/ helper (VulkanContext
    // outlives whatever it hands a reference to) -- this object stores only
    // the device handle and the timestampPeriod scale it needs, both read out
    // of `ctx` once here.
    //
    // substeps: StepShape::substeps -- how many mark blocks the query pool
    // needs (see the class comment above). pass_names: the GPU passes in
    // recorded order; block index 0 is "before this substep's first pass",
    // index N is "pass N-1 has just finished". Must be >= 1; a 0 is treated the
    // same as "device cannot time this" (an empty query pool is not a legal
    // VkQueryPoolCreateInfo::queryCount) rather than asserted, since a
    // degenerate shape is StepRecorder's problem to reject, not this
    // constructor's.
    //
    // ALWAYS SUCCEEDS FOR THE CAPABILITY CHECK ITSELF: a device/queue that
    // cannot time compute work is not an error (see the class comment's
    // skip-gracefully posture) -- the returned object is simply
    // !supported(). Only a genuine Vulkan failure creating the (small:
    // substeps * (passes + 1) queries) pool on a device that DOES claim
    // support propagates as Code::internal, matching every other resource
    // this tree creates.
    [[nodiscard]] static Result<std::unique_ptr<PassTimestamps>> create(VulkanContext& ctx, uint32_t substeps,
                                                                       std::vector<std::string> pass_names);

    // Held only by unique_ptr, so neither copyable nor movable: no hand-written
    // move to fall behind a new member (the defect-5 lesson).
    ~PassTimestamps();
    PassTimestamps(const PassTimestamps&) = delete;
    PassTimestamps& operator=(const PassTimestamps&) = delete;

    // False iff this device/queue could not time compute work (see the class
    // comment) -- every recording method below is then a no-op and
    // read_durations_ns() reports an all-zero, `supported == false` result.
    [[nodiscard]] bool supported() const noexcept { return pool_ != VK_NULL_HANDLE; }

    // Recorded ONCE, at the very top of StepRecorder::record() -- right after
    // the one descriptor-set bind, before the first dispatch of the first
    // substep. Resets every query slot this object owns so the SAME recorded
    // buffer can be resubmitted arbitrarily many times without ever writing
    // an already-populated query (a Vulkan validation error). A no-op when
    // !supported().
    void record_reset(VkCommandBuffer cmd) const;

    // Records one timestamp write at the CURRENT point in `cmd`'s command
    // stream. `substep` is which of the shape's repeated blocks this mark
    // belongs to (0 .. substeps-1); `boundary` is 0 for "start of this
    // substep" or 1..passes for "pass boundary-1 just finished". A
    // no-op when !supported() -- nothing is written to `cmd` at all, which is
    // the zero-cost half of the class comment's contract.
    void record_mark(VkCommandBuffer cmd, uint32_t substep, uint32_t boundary) const;

    // Reads the query pool back SYNCHRONOUSLY (VK_QUERY_RESULT_WAIT_BIT) and
    // returns the per-pass durations, summed across every substep of the
    // MOST RECENTLY COMPLETED submit -- safe to call any time after that
    // submit's own fence wait has returned (StepRecorder::submit()'s existing
    // contract), because the query pool's contents are exactly what that
    // submit's own execution just wrote.
    //
    // One PassDuration per pass name, in recorded order. An empty list with
    // `supported == false` when this object was created unsupported -- never
    // a list of zeros that could be read as measured, and never
    // Code::unavailable, so a caller does not have to
    // branch before asking (the same "always answer, let the flag carry the
    // fact" posture VulkanContext::supports_int64() documents for itself).
    [[nodiscard]] Result<PassDurationsNs> read_durations_ns() const;

private:
    PassTimestamps() = default;
    void destroy() noexcept;

    VkDevice device_ = VK_NULL_HANDLE;
    VkQueryPool pool_ = VK_NULL_HANDLE;
    uint32_t substeps_ = 0;
    std::vector<std::string> pass_names_;
    uint32_t marks_per_substep_ = 1;  // pass_names_.size() + 1
    // VkPhysicalDeviceLimits::timestampPeriod -- nanoseconds per tick this
    // device's timestamps count in (NOT guaranteed to be 1.0; Intel Iris Plus
    // reports its own value, read once at create() time).
    double timestamp_period_ns_ = 1.0;

    // VkQueueFamilyProperties::timestampValidBits for the COMPUTE family.
    //
    // ⚠ THIS FIELD WAS READ AND DISCARDED FOR THE WHOLE OF S6. create() below
    // has always fetched it, but only to test it against 0 as a support
    // check -- it was never stored and read_durations_ns() subtracted RAW
    // 64-bit values. The spec says the upper (64 - timestampValidBits) bits
    // of a timestamp are UNDEFINED, and this box's Intel Iris Plus reports
    // 36, so 28 undefined bits rode into every duration this engine has
    // ever reported. That is the leading explanation for a measured
    // collision_dynamic of 119.8 ms against a 69.3 ms wall-clock STEP -- a
    // pass cannot outlast the step containing it.
    //
    // 64 is the correct default for the unsupported path: it makes the mask
    // all-ones, so a device that never gets here is never silently truncated.
    uint32_t timestamp_valid_bits_ = 64u;
};

}  // namespace spade::compute
