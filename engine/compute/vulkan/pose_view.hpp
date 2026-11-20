#pragma once

// ---------------------------------------------------------------------------
// pose_view.hpp (camera design section 12, item 1) -- THE READ-ONLY DEVICE-SIDE VIEW OF THE BODY POSES, for a pass that
// draws from them (the camera's TLAS-instance kernel) without a host readback.
//
// WHAT IT IS. The Vulkan world set keeps the registered "bodies" array on the device (a mirror of the host arena, one
// BodyState per slot; BodyState is float32, 128 bytes, eight 16-byte rows). This hands the consumer the handles and the
// layout it needs to bind that buffer as a read-only storage buffer, the slot-to-world map beside it, and, when asked for,
// the poses of the PREVIOUS tick, for camera-motion blur (one tick of history).
//
// OPT-IN. BackendDesc::device_pose_view = true makes the backend (1) allocate the previous-tick buffer, (2) record a
// device-side copy of "bodies" into it at the START of every tick's command buffer, before the first dispatch writes, and
// (3) end every tick's command buffer with a memory barrier making the tick's shader writes visible to later compute and
// transfer reads on the same queue. Without it nothing is recorded or allocated beyond today's, and the view is refused
// (`unavailable`): the default recording is byte for byte today's.
//
// THE TICK BOUNDARY (the contract):
//   * After Simulation::step(n) returns, `bodies` holds the state at the END of tick `tick` (the last tick the call ran,
//     0-based, the value StepParams carried), and `previous` holds the state at the END of tick `tick - 1`, i.e. the state
//     tick `tick` started from (for the first tick of a run: the state the host uploaded). "Previous" is per TICK, so a
//     step(n) with n > 1 keeps one tick of history, not n.
//   * The view is valid until the next call that uploads, steps or restores the Simulation. A consumer submitting on the
//     backend's own compute queue after step() returns is ordered after the tick's writes (the step's last barrier plus
//     the host's fence wait); a consumer on another queue or API waits on the semaphore of camera design item 2, which does
//     not exist yet.
//   * The buffers are DEVICE_LOCAL storage buffers with transfer-src and transfer-dst usage: bind them read-only. Writing
//     to them breaks the world set.
//   * LAYOUT. `bodies` is the walk's array exactly as the kernels see it: world w's slots are the contiguous range
//     [w * capacity_per_world, (w + 1) * capacity_per_world) (state/arenas.hpp's WorldRange), and a slot is live iff
//     slot_to_world[slot] != kInvalidWorld (0xFFFFFFFF); a live slot holds its world's index. A dead slot's row is stale
//     bytes, not a pose.
//
// A VulkanContext& is part of the view because the consumer must create its pipelines and descriptor sets on the SAME
// device the buffers live on (a VkBuffer is not usable on another VkDevice). The context outlives nothing: it belongs to
// the Simulation, and the view is invalid once the Simulation is destroyed.
// ---------------------------------------------------------------------------

#include <cstdint>

#include <volk.h>

#include "compute/vulkan/backend.hpp"
#include "core/error.hpp"

namespace spade::compute {

class VulkanContext;

struct DevicePoseView {
    VulkanContext* context = nullptr;           // the device the buffers are on
    VkBuffer bodies = VK_NULL_HANDLE;           // the "bodies" array: world_count * capacity_per_world BodyStates
    VkDeviceSize bodies_bytes = 0;
    VkBuffer previous = VK_NULL_HANDLE;         // the same layout at the end of the previous tick
    VkBuffer slot_to_world = VK_NULL_HANDLE;    // the "bodies.slot_to_world" array: one uint32 per slot
    VkDeviceSize slot_to_world_bytes = 0;
    uint32_t stride = 0;                        // bytes a body: sizeof(BodyState) = 128
    uint32_t pos_offset = 0;                    // within a body: vec3 position (+ proxy_radius in the fourth float)
    uint32_t orient_offset = 0;                 // within a body: the unit quaternion body->world (glm::quat order)
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;            // bodies per world
    uint64_t tick = 0;                          // the last tick whose end state `bodies` holds (0-based)
    bool stepped = false;                       // false until the first step: `bodies` then holds the upload only
};

// The view of `backend`'s body poses (see the contract above). `unavailable`, naming BackendDesc::device_pose_view, unless the
// backend was created with it. A free function, not a member, because VulkanBackend's header must not name Vulkan handles.
[[nodiscard]] Result<DevicePoseView> pose_view(const VulkanBackend& backend);

}  // namespace spade::compute
