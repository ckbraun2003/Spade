// ---------------------------------------------------------------------------
// editor_air_gpu.hpp -- Run air on the GPU (Rendering's design note,
// docs/design/rendering/plans/2026-10-27-run-air-on-gpu.md, slice 3; the
// lead, 2026-10-27): the starter the editor's window gives Run air.
//
// gpu_air_starter() makes one Vulkan context, at the first Run air, and
// keeps it for every run after. Each run starts through airflow_gpu::
// start_live_run(): on the device when it takes the case, else on the CPU,
// the device's refusal verbatim in the run's backend ("CPU: " and why). A
// context that can't be made is the why ("no Vulkan device: " and its
// error). Each run holds the context, so it outlives the run.
//
// Vulkan's headers come with it, so only the window (editor_session.cpp,
// when the build links spade_airflow_runner: SPADE_EDITOR_AIR_GPU) and the
// device's tests include this; editor_airflow.hpp stays free of them.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "editor_airflow.hpp"
#include "spike/airflow/live_fluid.hpp"

namespace spade::sandbox::editor {

// `workgroup_size` is GpuFluid's: a test asks for one with no kernel variant
// to see a device's refusal through.
[[nodiscard]] inline AirStarter gpu_air_starter(uint32_t workgroup_size = 64) {
    struct Device {
        bool tried = false;
        std::shared_ptr<compute::VulkanContext> ctx;
        std::string why;  // why there is none, once tried
    };
    auto device = std::make_shared<Device>();
    return [device, workgroup_size](const analysis::ResolvedCase& resolved,
                                    const vehicles::ModelType& model) -> Result<AirStart> {
        if (!device->tried) {
            device->tried = true;
            Result<std::unique_ptr<compute::VulkanContext>> made =
                compute::VulkanContext::create(compute::BackendDesc{.kind = compute::BackendKind::vulkan});
            if (made) {
                device->ctx = std::move(*made);
            } else {
                device->why = "no Vulkan device: " + made.error().context;
            }
        }
        Result<airflow_gpu::LiveRunOn> on =
            airflow_gpu::start_live_run(resolved, model, device->ctx.get(), workgroup_size);
        if (!on) return std::unexpected(on.error());
        std::string backend = on->cpu_because.empty() ? std::string("GPU")
                              : device->ctx            ? "CPU: " + on->cpu_because
                                                       : "CPU: " + device->why;
        return AirStart{std::move(on->run), std::move(backend), device->ctx};
    };
}

}  // namespace spade::sandbox::editor
