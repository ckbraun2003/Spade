#include "compute/vulkan/timestamps.hpp"

#include <string>
#include <utility>
#include <vector>

namespace spade::compute {

namespace {

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

// 8 real slots (kSchedule's own count) + 1 leading "start of substep" mark.
constexpr uint32_t kSlotsPerSubstep = 8;

}  // namespace

Result<std::unique_ptr<PassTimestamps>> PassTimestamps::create(VulkanContext& ctx, uint32_t substeps) {
    auto self = std::unique_ptr<PassTimestamps>(new PassTimestamps());
    self->device_ = ctx.device();
    self->substeps_ = substeps;

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

    VkQueryPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    pool_info.queryType = VK_QUERY_TYPE_TIMESTAMP;
    pool_info.queryCount = substeps * kPassTimestampMarksPerSubstep;
    if (VkResult r = vkCreateQueryPool(self->device_, &pool_info, nullptr, &self->pool_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateQueryPool (PassTimestamps)"));
    }

    return self;
}

void PassTimestamps::record_reset(VkCommandBuffer cmd) const {
    if (pool_ == VK_NULL_HANDLE) return;
    vkCmdResetQueryPool(cmd, pool_, 0, substeps_ * kPassTimestampMarksPerSubstep);
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
    vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, pool_,
                        substep * kPassTimestampMarksPerSubstep + boundary);
}

Result<PassDurationsNs> PassTimestamps::read_durations_ns() const {
    PassDurationsNs out{};
    if (pool_ == VK_NULL_HANDLE) {
        return out;  // supported == false, every field 0.0 -- the documented posture
    }
    out.supported = true;

    const uint32_t count = substeps_ * kPassTimestampMarksPerSubstep;
    std::vector<uint64_t> ticks(count, 0);
    if (VkResult r = vkGetQueryPoolResults(device_, pool_, 0, count, ticks.size() * sizeof(uint64_t),
                                            ticks.data(), sizeof(uint64_t),
                                            VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkGetQueryPoolResults (PassTimestamps)"));
    }

    // Sum, per schedule slot, across every substep's own 9-mark block. Marks
    // are recorded in program order (block[0] == "start of this substep",
    // block[N] == "slot N-1 just finished" for N in 1..8), so slot i's
    // duration is the gap between CONSECUTIVE marks, block[i+1] - block[i] --
    // NOT block[i+1] - block[0], which would accumulate every earlier slot's
    // time into each later one. DEFENSIVELY clamped at 0 rather than allowed
    // to underflow into an enormous unsigned wraparound: record_mark()'s
    // ALL_COMMANDS_BIT choice above should make end >= start impossible, but
    // a wrong sign reported as a nonsense multi-second "duration" is a far
    // more confusing failure than a reported zero.
    double totals[kSlotsPerSubstep] = {};
    for (uint32_t s = 0; s < substeps_; ++s) {
        const uint64_t* block = &ticks[static_cast<std::size_t>(s) * kPassTimestampMarksPerSubstep];
        for (uint32_t slot = 0; slot < kSlotsPerSubstep; ++slot) {
            const uint64_t start = block[slot];
            const uint64_t end = block[slot + 1];
            if (end > start) {
                totals[slot] += static_cast<double>(end - start) * timestamp_period_ns_;
            }
        }
    }

    out.medium_update_ns = totals[0];
    out.force_elements_ns = totals[1];
    out.gravity_ns = totals[2];
    out.collision_static_ns = totals[3];
    out.collision_dynamic_ns = totals[4];
    out.integrate_ns = totals[5];
    out.sensor_synthesis_ns = totals[6];
    out.publish_ns = totals[7];
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

PassTimestamps::PassTimestamps(PassTimestamps&& other) noexcept
    : device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      pool_(std::exchange(other.pool_, VK_NULL_HANDLE)),
      substeps_(other.substeps_),
      timestamp_period_ns_(other.timestamp_period_ns_) {}

PassTimestamps& PassTimestamps::operator=(PassTimestamps&& other) noexcept {
    if (this != &other) {
        destroy();
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        pool_ = std::exchange(other.pool_, VK_NULL_HANDLE);
        substeps_ = other.substeps_;
        timestamp_period_ns_ = other.timestamp_period_ns_;
    }
    return *this;
}

}  // namespace spade::compute
