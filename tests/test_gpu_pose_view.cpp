// test_gpu_pose_view.cpp -- the read-only device-side view of the body poses (camera design section 12, item 1;
// compute/vulkan/pose_view.hpp states the contract). The tests read the device buffers back with their OWN Vulkan copy (an
// independent path from the mirror's readback) and compare them with the host rows the Simulation reads back after each
// step. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <volk.h>

#include "compute/backend.hpp"
#include "compute/vulkan/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/pose_view.hpp"
#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/simulation.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::BodyState;
using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::DevicePoseView;
using spade::compute::VulkanContext;
using spade::compute::pose_view;
using spade::test::bare_body;
using spade::test::floor_world_set;
using spade::test::row_of;

// The bytes of a device buffer, read back by a copy of this test's own (a command pool and a host-visible buffer), not by
// the mirror's readback.
[[nodiscard]] std::vector<std::byte> read_device_buffer(VulkanContext& ctx, VkBuffer src, VkDeviceSize bytes) {
    std::vector<std::byte> out;
    const VkDevice device = ctx.device();

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = bytes;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkBuffer host = VK_NULL_HANDLE;
    if (vkCreateBuffer(device, &buffer_info, nullptr, &host) != VK_SUCCESS) {
        ADD_FAILURE() << "vkCreateBuffer";
        return out;
    }
    VkMemoryRequirements need{};
    vkGetBufferMemoryRequirements(device, host, &need);
    VkPhysicalDeviceMemoryProperties memory{};
    vkGetPhysicalDeviceMemoryProperties(ctx.physical_device(), &memory);
    uint32_t type = UINT32_MAX;
    for (uint32_t i = 0; i < memory.memoryTypeCount; ++i) {
        const VkMemoryPropertyFlags flags = memory.memoryTypes[i].propertyFlags;
        if ((need.memoryTypeBits & (1u << i)) != 0 && (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0 &&
            (flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0) {
            type = i;
            break;
        }
    }
    VkDeviceMemory host_memory = VK_NULL_HANDLE;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    do {
        if (type == UINT32_MAX) {
            ADD_FAILURE() << "no host-visible coherent memory type";
            break;
        }
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = need.size;
        alloc.memoryTypeIndex = type;
        if (vkAllocateMemory(device, &alloc, nullptr, &host_memory) != VK_SUCCESS ||
            vkBindBufferMemory(device, host, host_memory, 0) != VK_SUCCESS) {
            ADD_FAILURE() << "vkAllocateMemory/vkBindBufferMemory";
            break;
        }
        VkCommandPoolCreateInfo pool_info{};
        pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool_info.queueFamilyIndex = ctx.compute_queue_family();
        VkCommandBufferAllocateInfo cmd_info{};
        cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cmd_info.commandBufferCount = 1;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkFenceCreateInfo fence_info{};
        fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        if (vkCreateCommandPool(device, &pool_info, nullptr, &pool) != VK_SUCCESS) {
            ADD_FAILURE() << "vkCreateCommandPool";
            break;
        }
        cmd_info.commandPool = pool;
        if (vkAllocateCommandBuffers(device, &cmd_info, &cmd) != VK_SUCCESS ||
            vkCreateFence(device, &fence_info, nullptr, &fence) != VK_SUCCESS) {
            ADD_FAILURE() << "vkAllocateCommandBuffers/vkCreateFence";
            break;
        }
        VkCommandBufferBeginInfo begin{};
        begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd, &begin);
        VkBufferMemoryBarrier visible{};
        visible.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        visible.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
        visible.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        visible.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        visible.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        visible.buffer = src;
        visible.offset = 0;
        visible.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1,
                             &visible, 0, nullptr);
        const VkBufferCopy region{0, 0, bytes};
        vkCmdCopyBuffer(cmd, src, host, 1, &region);
        vkEndCommandBuffer(cmd);
        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        if (vkQueueSubmit(ctx.compute_queue(), 1, &submit, fence) != VK_SUCCESS ||
            vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) {
            ADD_FAILURE() << "vkQueueSubmit/vkWaitForFences";
            break;
        }
        void* mapped = nullptr;
        if (vkMapMemory(device, host_memory, 0, bytes, 0, &mapped) != VK_SUCCESS) {
            ADD_FAILURE() << "vkMapMemory";
            break;
        }
        out.resize(bytes);
        std::memcpy(out.data(), mapped, bytes);
        vkUnmapMemory(device, host_memory);
    } while (false);
    if (fence != VK_NULL_HANDLE) vkDestroyFence(device, fence, nullptr);
    if (pool != VK_NULL_HANDLE) vkDestroyCommandPool(device, pool, nullptr);
    if (host_memory != VK_NULL_HANDLE) vkFreeMemory(device, host_memory, nullptr);
    vkDestroyBuffer(device, host, nullptr);
    return out;
}

// One body's row out of a device copy of the bodies array: its first 32 bytes (position + proxy radius, orientation) are
// what the camera reads.
struct Pose {
    glm::vec3 pos{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
};

[[nodiscard]] Pose pose_at(const std::vector<std::byte>& bytes, const DevicePoseView& view, uint32_t slot) {
    Pose p;
    const std::byte* row = bytes.data() + static_cast<std::size_t>(slot) * view.stride;
    std::memcpy(&p.pos, row + view.pos_offset, sizeof(glm::vec3));
    std::memcpy(&p.orient, row + view.orient_offset, sizeof(glm::quat));
    return p;
}

[[nodiscard]] Pose pose_of(const BodyState& b) { return Pose{b.pos, b.orient}; }

[[nodiscard]] bool same(const Pose& a, const Pose& b) {
    return std::memcmp(&a.pos, &b.pos, sizeof(a.pos)) == 0 && std::memcmp(&a.orient, &b.orient, sizeof(a.orient)) == 0;
}

// A flying, turning body in a one-world floor set (room for two bodies), on Vulkan with the view asked for or not.
struct Rig {
    Simulation sim;
    BodyRef body;
};

[[nodiscard]] Rig make_rig(bool view) {
    const spade::Result<spade::WorldSetDesc> set = floor_world_set(2);
    EXPECT_TRUE(set.has_value()) << set.error().context;
    BackendDesc desc{.kind = BackendKind::vulkan};
    desc.device_pose_view = view;
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 2, desc);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    BodySpawn flying = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    flying.orient = glm::quat(0.95f, 0.1f, -0.2f, 0.2f);
    flying.vel = glm::vec3(0.4f, 0.0f, -0.2f);
    flying.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const spade::Result<BodyRef> body = sim->spawn(0, flying);
    EXPECT_TRUE(body.has_value()) << body.error().context;
    EXPECT_TRUE(sim->flush_structural().has_value());
    return Rig{std::move(*sim), *body};
}

}  // namespace

// OPT-IN: without BackendDesc::device_pose_view the view is refused, and the default world set records and allocates what
// it did before.
TEST(GpuPoseView, TheViewIsRefusedUnlessAskedFor) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Rig rig = make_rig(false);
    ASSERT_NE(rig.sim.vulkan_backend(), nullptr);
    const auto view = pose_view(*rig.sim.vulkan_backend());
    ASSERT_FALSE(view.has_value());
    EXPECT_EQ(view.error().code, spade::Code::unavailable) << view.error().context;
    EXPECT_NE(view.error().context.find("device_pose_view"), std::string::npos) << view.error().context;
    EXPECT_EQ(Simulation::create(*floor_world_set(2), 1'000'000, 2, BackendDesc{}).value().vulkan_backend(), nullptr)
        << "a cpu Simulation has no Vulkan backend";
}

TEST(GpuPoseView, TheViewDescribesTheBodiesArrayAndHoldsTheHostsRowsAtTheEndOfEachTick) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Rig rig = make_rig(true);
    auto view = pose_view(*rig.sim.vulkan_backend());
    ASSERT_TRUE(view.has_value()) << view.error().context;
    EXPECT_NE(view->context, nullptr);
    EXPECT_EQ(view->stride, sizeof(BodyState));
    EXPECT_EQ(view->stride, 128u);
    EXPECT_EQ(view->pos_offset, offsetof(BodyState, pos));
    EXPECT_EQ(view->orient_offset, offsetof(BodyState, orient));
    EXPECT_EQ(view->world_count, 1u);
    EXPECT_EQ(view->capacity_per_world, 2u);
    EXPECT_EQ(view->bodies_bytes, VkDeviceSize{2} * 128u);
    EXPECT_EQ(view->slot_to_world_bytes, VkDeviceSize{2} * sizeof(uint32_t));
    EXPECT_FALSE(view->stepped) << "before the first step the view holds the upload";

    for (uint64_t tick = 0; tick < 5; ++tick) {
        ASSERT_TRUE(rig.sim.step(1).has_value());
        view = pose_view(*rig.sim.vulkan_backend());
        ASSERT_TRUE(view.has_value()) << view.error().context;
        EXPECT_TRUE(view->stepped);
        EXPECT_EQ(view->tick, tick);
        const std::vector<std::byte> device = read_device_buffer(*view->context, view->bodies, view->bodies_bytes);
        ASSERT_EQ(device.size(), view->bodies_bytes);
        EXPECT_TRUE(same(pose_at(device, *view, rig.body.slot), pose_of(row_of(rig.sim, rig.body))))
            << "the device pose at the end of tick " << tick << " is the host's row";
    }

    // The slot-to-world map: the body's slot names world 0, the other slot is dead.
    const std::vector<std::byte> map = read_device_buffer(*view->context, view->slot_to_world, view->slot_to_world_bytes);
    ASSERT_EQ(map.size(), view->slot_to_world_bytes);
    std::vector<uint32_t> worlds(2);
    std::memcpy(worlds.data(), map.data(), map.size());
    EXPECT_EQ(worlds[rig.body.slot], 0u);
    EXPECT_EQ(worlds[1u - rig.body.slot], spade::kInvalidWorld);
}

TEST(GpuPoseView, PreviousIsTheEndOfThePreviousTickAndOneTickDeepWhateverTheStepCount) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // The host's row at the end of each tick, from one run stepped a tick at a time (the same run is deterministic).
    Rig one = make_rig(true);
    std::vector<Pose> after;  // after[t] = the pose at the end of tick t
    const Pose initial = pose_of(row_of(one.sim, one.body));
    for (int t = 0; t < 4; ++t) {
        ASSERT_TRUE(one.sim.step(1).has_value());
        after.push_back(pose_of(row_of(one.sim, one.body)));
    }
    ASSERT_NE(std::memcmp(&after[0].pos, &after[1].pos, sizeof(glm::vec3)), 0) << "the body must move for this to prove anything";

    // After tick 3 of that run: previous is the end of tick 2.
    {
        const auto view = pose_view(*one.sim.vulkan_backend());
        ASSERT_TRUE(view.has_value()) << view.error().context;
        ASSERT_NE(view->previous, VkBuffer{VK_NULL_HANDLE});
        const auto prev = read_device_buffer(*view->context, view->previous, view->bodies_bytes);
        EXPECT_TRUE(same(pose_at(prev, *view, one.body.slot), after[2])) << "previous = the end of tick 2";
    }

    // A second run stepped in ONE call of 3: bodies = the end of tick 2, previous = the end of tick 1 (one tick deep).
    Rig batch = make_rig(true);
    ASSERT_TRUE(batch.sim.step(3).has_value());
    {
        const auto view = pose_view(*batch.sim.vulkan_backend());
        ASSERT_TRUE(view.has_value()) << view.error().context;
        EXPECT_EQ(view->tick, 2u);
        const auto cur = read_device_buffer(*view->context, view->bodies, view->bodies_bytes);
        const auto prev = read_device_buffer(*view->context, view->previous, view->bodies_bytes);
        EXPECT_TRUE(same(pose_at(cur, *view, batch.body.slot), after[2]));
        EXPECT_TRUE(same(pose_at(prev, *view, batch.body.slot), after[1]));
    }

    // The first tick of a run: previous is the state the host uploaded.
    Rig first = make_rig(true);
    ASSERT_TRUE(first.sim.step(1).has_value());
    {
        const auto view = pose_view(*first.sim.vulkan_backend());
        ASSERT_TRUE(view.has_value()) << view.error().context;
        const auto prev = read_device_buffer(*view->context, view->previous, view->bodies_bytes);
        EXPECT_TRUE(same(pose_at(prev, *view, first.body.slot), initial)) << "previous = the upload";
    }
}

// The opt-in changes the recording by a copy and a barrier OUTSIDE the counted chain, and allocates one buffer the size of
// the bodies array: the chain's dispatch and barrier counts are today's, and the memory report names the extra buffer.
TEST(GpuPoseView, TheOptInAddsOneBodiesSizedBufferAndLeavesTheCountedChainAlone) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Rig off = make_rig(false);
    Rig on = make_rig(true);
    const auto chain_off = off.sim.vulkan_recorded_chain();
    const auto chain_on = on.sim.vulkan_recorded_chain();
    ASSERT_TRUE(chain_off.has_value() && chain_on.has_value());
    EXPECT_EQ(chain_on->dispatches, chain_off->dispatches);
    EXPECT_EQ(chain_on->barriers, chain_off->barriers);

    const auto find = [](const spade::compute::MemoryReport& report, const char* name) -> const spade::compute::MemoryRecord* {
        for (const auto& record : report.buffers) {
            if (record.name == name && !record.staging) return &record;
        }
        return nullptr;
    };
    const spade::compute::MemoryReport report_off = off.sim.memory_report();
    const spade::compute::MemoryReport report_on = on.sim.memory_report();
    EXPECT_EQ(find(report_off, "bodies.previous"), nullptr);
    const auto* extra = find(report_on, "bodies.previous");
    ASSERT_NE(extra, nullptr);
    EXPECT_EQ(extra->bytes, VkDeviceSize{2} * 128u);
    EXPECT_TRUE(extra->device_local);
    EXPECT_EQ(report_on.device_bytes, report_off.device_bytes + extra->bytes);
}
