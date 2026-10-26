#pragma once

// ---------------------------------------------------------------------------
// airflow_exact_device.hpp -- where the exact airflow kernels are held to the
// CPU's bits. TD-14 as amended 2026-10-22 (the user's "Full bit-exact"): an
// airflow kernel whose every division and square root is an exact fp32_math
// operation, with its CPU twin flushing as the device does, equals that twin
// bit for bit on each device and driver measured; another device or driver
// keeps the kernel's band. The one measured so far is rtx3060ti-572.83
// (engine/testing/parity.hpp's records): the RTX 3060 Ti on driver 572.83,
// which the device reports as 0x8F14C000.
// ---------------------------------------------------------------------------

#include "compute/vulkan/context.hpp"

namespace spade::testing {

[[nodiscard]] inline bool airflow_bit_equal_device(const spade::compute::VulkanContext& ctx) {
    const spade::compute::DeviceReport& r = ctx.device_report();
    return r.device_name == "NVIDIA GeForce RTX 3060 Ti" && r.driver_version == 0x8F14C000u;
}

}  // namespace spade::testing
