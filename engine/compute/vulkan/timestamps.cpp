#include "compute/vulkan/timestamps.hpp"

#include <string>
#include <utility>
#include <vector>

namespace spade::compute {

namespace {

// L307 (2): the threshold above which a single pass duration is reported as
// IMPLAUSIBLE rather than believed. One second, chosen as a value no real
// pass on any device can approach while still being orders of magnitude
// below what a garbage high-bit sample produces -- at 52.0833 ns/tick, a
// single stray bit 36 sets a delta of ~57 minutes.
//
// It is a DIAGNOSTIC BOUND, not a tolerance: nothing is clamped, rejected or
// rescaled by it. It only decides whether PassDurationsNs::implausible_
// samples is incremented, so a caller can tell "the instrument is broken"
// from "the kernel is slow" -- the exact distinction that was unavailable
// when collision_dynamic reported 119.8 ms against a 69.3 ms step.
constexpr double kImplausibleSampleNs = 1e9;

// Same taxonomy as step_recorder.cpp's identical helper -- duplicated rather
// than shared through a third header for the same reason that file gives:
// four lines, and every caller already has <core/error.hpp> and <volk.h> in
// scope.
[[nodiscard]] Error map_vk_error(VkResult result, std::string_view op) {
    switch (result) {
        case VK_ERROR_DEVICE_LOST:
            return Error{Code::internal, "Vulkan device lost during " + std::string(op)};
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        case VK_ERROR_OUT_OF_HOST_MEMORY:
            return Error{Code::capacity_exceeded,
                         "Vulkan allocation failed (VkResult " + std::to_string(static_cast<int>(result)) +
                             ") during " + std::string(op)};
        default:
            return Error{Code::internal, "Vulkan call failed (VkResult " +
                                              std::to_string(static_cast<int>(result)) + ") during " +
                                              std::string(op)};
    }
}

}  // namespace

Result<std::unique_ptr<PassTimestamps>> PassTimestamps::create(VulkanContext& ctx, uint32_t substeps,
                                                               std::vector<std::string> pass_names) {
    auto self = std::unique_ptr<PassTimestamps>(new PassTimestamps());
    self->device_ = ctx.device();
    self->substeps_ = substeps;
    self->marks_per_substep_ = static_cast<uint32_t>(pass_names.size()) + 1u;
    self->pass_names_ = std::move(pass_names);

    // DEGENERATE SHAPE: a 0-substep recording needs no query pool at all, and
    // VkQueryPoolCreateInfo::queryCount == 0 is not legal input -- treated
    // identically to "device cannot time this" rather than asserted, per this
    // class's own "StepRecorder's problem to reject, not this constructor's"
    // note.
    if (substeps == 0) {
        return self;
    }

    // CAPABILITY CHECK ONE: can this device time compute work at all?
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(ctx.physical_device(), &properties);
    if (properties.limits.timestampComputeAndGraphics == VK_FALSE) {
        return self;  // unsupported -- pool_ stays VK_NULL_HANDLE
    }

    // CAPABILITY CHECK TWO: can the COMPUTE queue family specifically report
    // timestamps? (A device can satisfy check one yet still expose a compute
    // family with timestampValidBits == 0 -- legal per the spec, and the
    // family-specific bit is the one that actually governs vkCmdWriteTimestamp
    // validity on ctx's compute queue.)
    uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(ctx.physical_device(), &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(ctx.physical_device(), &family_count, families.data());
    const uint32_t compute_family = ctx.compute_queue_family();
    if (compute_family >= families.size() || families[compute_family].timestampValidBits == 0) {
        return self;  // unsupported
    }

    self->timestamp_period_ns_ = static_cast<double>(properties.limits.timestampPeriod);

    // STORE the valid-bit count, do not merely test it. The check above uses
    // this same field as a support predicate (== 0 means "this family cannot
    // time"); every value ABOVE zero and below 64 is equally a statement that
    // the high bits are undefined, and read_durations_ns() has to mask by it.
    self->timestamp_valid_bits_ = families[compute_family].timestampValidBits;

    VkQueryPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    pool_info.queryType = VK_QUERY_TYPE_TIMESTAMP;
    pool_info.queryCount = substeps * self->marks_per_substep_;
    if (VkResult r = vkCreateQueryPool(self->device_, &pool_info, nullptr, &self->pool_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateQueryPool (PassTimestamps)"));
    }

    return self;
}

void PassTimestamps::record_reset(VkCommandBuffer cmd) const {
    if (pool_ == VK_NULL_HANDLE) return;
    vkCmdResetQueryPool(cmd, pool_, 0, substeps_ * marks_per_substep_);
}

void PassTimestamps::record_mark(VkCommandBuffer cmd, uint32_t substep, uint32_t boundary) const {
    if (pool_ == VK_NULL_HANDLE) return;
    // VK_PIPELINE_STAGE_ALL_COMMANDS_BIT: the timestamp is written only once
    // every command submitted before it in program order -- including the
    // barrier(s) StepRecorder::record() emits between passes -- has finished
    // every one of its own stages. That is a stronger guarantee than this
    // module strictly needs (COMPUTE_SHADER would do, since every recorded
    // command here is a compute dispatch, a barrier, or another query op) but
    // it is the unambiguous choice: consecutive marks are then guaranteed
    // monotonically non-decreasing, which is what read_durations_ns() below
    // subtracts on trust.
    vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, pool_, substep * marks_per_substep_ + boundary);
}

Result<PassDurationsNs> PassTimestamps::read_durations_ns() const {
    PassDurationsNs out{};
    if (pool_ == VK_NULL_HANDLE) {
        return out;  // supported == false, no passes -- the documented posture
    }
    out.supported = true;

    const uint32_t count = substeps_ * marks_per_substep_;
    std::vector<uint64_t> ticks(count, 0);
    if (VkResult r = vkGetQueryPoolResults(device_, pool_, 0, count, ticks.size() * sizeof(uint64_t),
                                            ticks.data(), sizeof(uint64_t),
                                            VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkGetQueryPoolResults (PassTimestamps)"));
    }

    // Sum, per pass, across every substep's own mark block. Marks are recorded
    // in program order (block[0] == "start of this substep", block[N] ==
    // "pass N-1 just finished" for N in 1..passes), so pass i's duration is
    // the gap between CONSECUTIVE marks, block[i+1] - block[i] -- NOT
    // block[i+1] - block[0], which would accumulate every earlier pass's
    // time into each later one. DEFENSIVELY clamped at 0 rather than allowed
    // to underflow into an enormous unsigned wraparound: record_mark()'s
    // ALL_COMMANDS_BIT choice above should make end >= start impossible, but
    // a wrong sign reported as a nonsense multi-second "duration" is a far
    // more confusing failure than a reported zero.
    // THE MASK (L307 (2)). Bits at or above timestampValidBits are UNDEFINED
    // per the spec, not zero, so they must be cleared before any arithmetic.
    // `mask` is all-ones when valid_bits >= 64, which keeps a fully-reporting
    // device bit-identical to the pre-mask behaviour.
    //
    // The shift is guarded because `1ull << 64` is UNDEFINED BEHAVIOUR in C++,
    // not a conveniently-zero result -- the one line where getting this
    // "obviously right" would reintroduce undefined behaviour to remove it.
    const uint64_t mask =
        (timestamp_valid_bits_ >= 64u) ? ~0ull : ((1ull << timestamp_valid_bits_) - 1ull);

    const uint32_t pass_count = marks_per_substep_ - 1u;
    std::vector<double> totals(pass_count, 0.0);
    uint32_t implausible = 0;
    for (uint32_t s = 0; s < substeps_; ++s) {
        const uint64_t* block = &ticks[static_cast<std::size_t>(s) * marks_per_substep_];
        for (uint32_t slot = 0; slot < pass_count; ++slot) {
            const uint64_t start = block[slot] & mask;
            const uint64_t end = block[slot + 1] & mask;

            // Wrapped difference in one expression. Correct for BOTH the
            // ordinary case and a counter wrap, because the subtraction is
            // modular and the true delta is far below the counter's range:
            // at 36 bits and 52.0833 ns/tick this counter wraps every ~59.6
            // MINUTES, and a pass is microseconds. Replaces the old
            // `if (end > start)` guard, which the mask makes both unnecessary
            // (garbage high bits were the reason end < start showed up at
            // all) and wrong (it clamped a genuine wrap to zero).
            const uint64_t delta = (end - start) & mask;
            const double ns = static_cast<double>(delta) * timestamp_period_ns_;

            // ⚠ AND THAT GUARD'S REMOVAL IS EXACTLY WHY THIS COUNTER EXISTS.
            // The old code silently discarded any sample it could not explain;
            // the new code cannot, because after masking every delta is a
            // plausible-looking non-negative number. A single pass taking
            // longer than a second is not a measurement, it is a broken
            // instrument -- and an instrument that hides its own failures is
            // how "collision_dynamic 119.8 ms" survived beside a 69.3 ms step
            // for a whole session. Reported, never clamped away.
            if (ns > kImplausibleSampleNs) {
                ++implausible;
            }
            totals[slot] += ns;
        }
    }
    out.implausible_samples = implausible;

    out.passes.reserve(pass_count);
    for (uint32_t i = 0; i < pass_count; ++i) out.passes.push_back(PassDuration{pass_names_[i], totals[i]});
    return out;
}

void PassTimestamps::destroy() noexcept {
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyQueryPool(device_, pool_, nullptr);
        pool_ = VK_NULL_HANDLE;
    }
    device_ = VK_NULL_HANDLE;
}

PassTimestamps::~PassTimestamps() { destroy(); }

}  // namespace spade::compute
