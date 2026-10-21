# B0: The GPU Airflow Solver's Prerequisites — Core Plan (DRAFT for the lead)

**Goal:** what Physics' runner (B1 onward) needs from Core, built where stage 7 will inherit it:
- the airflow binding block in `bindings.slang`;
- `VulkanContext::memory_budget()`;
- the device-limit checks of stage 7 §7.1, as functions the runner calls and never copies.

**Spec:** Physics' GPU plan, B0 (`docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md`, merged at `c7a1be2`). The chain-runner design §2, with A1-A3 (`2026-10-12-airflow-chain-runner-design.md`). Stage 7 §7.1, §7.4 and §9 (`2026-10-12-module-api-stage7-design.md`, signed as `CORE-8`). `CORE-5` as amended 2026-10-12: a device is admitted if it meets the limits the module set declares, and is otherwise refused by name.

**Branch:** `core/airflow-b0`, cut from master after B7b merges. Gates: release build; release CPU suite; the GPU suite (B0 touches Vulkan code); `gcc-check.sh`; a leg, since the task adds engine `.cpp` files and generated headers.

## Decisions this plan makes (each one open to the lead)

1. **The airflow block is its own descriptor set: set 1, named "airflow".** It is not more bindings in set 0.
   - Today's mirror binds every set-0 registry binding, and `DescriptorSetBindsEveryRegistryBinding` pins that count (`state_mirror.cpp:625-660`). Airflow bindings in set 0 would change every pipeline layout and the census, and would need empty buffers on every run without airflow.
   - A set 1 leaves set 0, the mirror, `StepShape` and every identity byte for byte, which is the runner's G1.
   - `gen_layout_check.py` already emits a `kSet_<name>` and a `kBindingCount_<name>` per named set (`:364`, `:382`).
   - The runner builds a pipeline layout of an empty set 0 and its own set 1. Stage 7's module binds the same set 1.
   - The cost: stage 7 §9 said "with stage 6's empty-buffer pattern", which assumed set 0. This plan departs from that.
2. **The block's names come from Physics' kernels, appended at the end, never renumbered.** B0 reserves the arrays of the spec's §4.3 table: params, u, v and w, pressure, dye, the MacCormack scratch, the force field, vorticity, the right-hand side, the residual, the multigrid levels, the apertures θ, the wall velocity, the region labels, the PCG scratch, the disc points, the tile partials, the health flag, the two publish halves, and the wall load. Physics confirms or edits the list before B0's RED.
   - B1's own tests use copy and add-one kernels, so B1 is not blocked on the final list. It needs set 1 to exist.
3. **The limit checks are pure functions over a captured `DeviceLimits`,** so they test on every machine without Vulkan. The limits join `DeviceReport` (`CORE-5`: every run records them).
   - Wiring them into `Simulation`'s own `create()` for set 0 stays stage 7's work. This plan adds the functions and the report fields only, so no device admitted today is refused by B0.
4. **`memory_budget()` is a query, not an enable** (A1).
   - It reads `VkPhysicalDeviceMemoryBudgetPropertiesEXT` through `vkGetPhysicalDeviceMemoryProperties2` when the device advertises `VK_EXT_memory_budget`. Otherwise it reports half of the largest device-local heap.
   - `source` names which, and device creation stays byte for byte.
   - The A2 arithmetic is a pure function: refuse when `requested > min(4 GiB, budget − usage − reserve)`. With `half_heap`, usage is what the caller says it has allocated itself.

## Task B0.1: Device limits, captured and checked

**Files:**
- Create `engine/compute/device_limits.hpp` and `.cpp`:
  - `struct DeviceLimits` with `max_storage_buffer_range`, `max_memory_allocation_size`, `max_per_stage_storage_buffers`, `max_set_storage_buffers`, `max_push_constants_size`, `max_group_count[3]`, `max_group_size[3]` and `max_group_invocations`, plus the device's name;
  - `check_buffer(name, bytes, limits)`, `check_storage_buffers(count, limits)`, `check_push(bytes, limits)`, `check_groups(pass, groups[3], limits)` and `check_workgroup(kernel, size[3], limits)`, each returning `Result<void>`.
- Modify `backend.hpp`: `DeviceReport` gains `DeviceLimits limits`.
- Modify `context.cpp`: fill the limits from `VkPhysicalDeviceLimits`, plus `maxMemoryAllocationSize` from `VkPhysicalDeviceMaintenance3Properties`, which is 1.1 core.
- Tests: `tests/test_device_limits.cpp` and `tests/test_gpu_device_limits.cpp`.

- [ ] RED (CPU):
  - `EachLimitPassesAtItsValueAndIsRefusedOnePast`. The message names what it is checking, the bytes or count, the limit and the device; the code is `capacity_exceeded`.
  - `TheGroupCheckNamesTheAxis`.
  - `TheWorkgroupCheckNamesTheKernel`.
- [ ] RED (GPU): `GpuDeviceLimits.TheReportCarriesTheDevicesLimits`. Every field is non-zero and at least Vulkan's guaranteed minimum.
- [ ] GREEN, then the gate.

## Task B0.2: `VulkanContext::memory_budget()` and the VRAM check

**Files:**
- Modify `context.hpp` and `.cpp`:
  - `struct MemoryBudget { uint64_t budget; uint64_t usage; uint32_t heap; enum class Source : uint32_t { ext, half_heap } source; }`;
  - `[[nodiscard]] MemoryBudget memory_budget() const`;
  - whether the extension is advertised, read once at `create()` from the enumerated device extensions.
- Add `check_vram(requested, budget, reserve, own_allocated) -> Result<void>` to `device_limits.hpp` (pure), with `kAirflowVramCap = 4 GiB` (the spec's cap, equal to `kGridByteCap`).
- Tests: in `test_device_limits.cpp` and `test_gpu_device_limits.cpp`.

- [ ] RED (CPU):
  - `TheVramCapIsTheSmallerOfFourGibAndTheFreeBudget`.
  - `AGridOverTheLiveBudgetIsRefusedNamingTheBytesAndTheCap`. This is the GPU plan's first review focus.
  - `HalfHeapSubtractsWhatTheCallerAllocated`.
- [ ] RED (GPU):
  - `GpuMemoryBudget.ReportsItsSourceAndABudgetWithinTheHeap`.
  - `GpuMemoryBudget.DeviceCreationIsUnchanged`. The enabled-extension list is the same as before.
- [ ] GREEN, then the gate.

## Task B0.3: The airflow set in `bindings.slang`

**Files:**
- Modify `engine/shaders/shared/bindings.slang`: set 1 (`airflow`), with the confirmed list, each `[[vk::binding(i, 1)]]`, and the rules in its header (append only; no kernel in set 0 reads it).
- Modify `gen_layout_check.py`'s set names (`1: "airflow"`). The airflow structs, if any, go in `layouts.slang`.
- Test: `tests/test_bindings_airflow.cpp`.

- [ ] RED:
  - `TheAirflowSetIsSetOne`.
  - `TheStateSetIsUnchanged`: `kBindingCount_state` stays 28, and `DescriptorSetBindsEveryRegistryBinding` still passes.
  - `AirflowBindingsAreContiguousFromZero`.
  - The standard identity and the schema hash are already pinned (`test_module_schedule.cpp`), and stay put.
- [ ] GREEN, then the gate and a leg (generated headers).

## Review focus

1. A grid under the 4 GiB cap but over the live budget, while another program holds VRAM: refused by name, never failing on allocation (B0.2).
2. `maxMemoryAllocationSize` on a 1.1 device whose `Maintenance3` properties are not chained: it falls back to Vulkan's guaranteed 2^30 and says so (B0.1).
3. An integrated GPU (the Iris Plus), whose "device-local" heap is system memory: the half-heap fallback must not claim the whole of system RAM (B0.2).
4. Set 1 must not leak into set 0's pipelines: every existing pipeline layout is created byte for byte as before (B0.3).
