# The airflow chain runner — design note for Core

**Owner:** Physics; Core reviews and approves it before any code (the lead, 2026-10-05). **Status:** DRAFT for Core.
**What it is for:** plan B, the GPU twin of plan A's solver and the 128³ spike; and the GPU half of the demonstration's first step (`PHY-10`: "the sandbox drives the airflow solver directly … then GPU via the chain runner").
**Guards, adopted by the lead 2026-10-05:**
- **G1.** A spike and bench harness in its own target. It is not linked into `Simulation`, makes no `StepShape` or mirror changes, and installs no header. Engine-path residency is stage 7's design, which Cameron signs first.
- **G2.** Built on `VulkanContext` (devices, queues, `CORE-5` admission). It extends `step_recorder`'s chain and barrier code, not a second recorder, so stage 7 inherits one pattern.
- It refuses by name (`L6`) on device limits and VRAM.

## 1. One chain pattern, shared (Core's code, Core's review)

New: `compute/vulkan/dispatch_chain.hpp/.cpp`. It moves out of `StepRecorder`, unchanged in behaviour:
- **`ChainDispatch`** `{ uint32_t pipeline; std::array<std::byte, 128> push; uint32_t push_size; uint32_t groups[3]; uint32_t mark; }`. `mark` names a timestamp range, or is none.
- **`record_dispatch_chain(cmd, layout, set, pipelines, dispatches, timestamps)`** binds the set once. For each dispatch, it binds the pipeline only when it changes, pushes the constants, dispatches, and writes a timestamp at each mark. **`StepRecorder`'s barrier rule, verbatim:** one whole-memory `VkMemoryBarrier` (SHADER_WRITE → SHADER_READ|WRITE, COMPUTE → COMPUTE) between each adjacent pair, none after the last.
- **`make_compute_pipeline(device, layout, variants, workgroup_size)`**: today's `StepRecorder::make_pipeline`, with its refusal of a missing workgroup variant.

`StepRecorder::record()` then builds its dispatch list from the recipes, as now, and calls `record_dispatch_chain`. The existing `recorded_chain()` count test and the GPU parity suites prove the move is neutral. Stage 7 records the airflow module's chain through the same function.

## 2. The runner (Physics' target, outside the engine)

- **Target:** `spade_airflow_runner`, a static library in `spike/airflow/`. It is not part of `spade::sim`, not installed and not exported. Its consumers are plan B's parity tests, its spike benchmark and the sandbox's demonstration harness (D1/D2, `PHY-10`).
- **API:**

```cpp
struct RunnerBuffer { std::string name; uint64_t bytes; };       // binding i = buffers[i]
struct RunnerDesc {
    std::span<const RunnerBuffer> buffers;
    std::span<const SpirvVariantSet* const> kernels;              // pipeline i = kernels[i]
    uint32_t push_bytes;                                          // one push range, every kernel
    uint32_t workgroup_size;
};
class ChainRunner {
public:
    static Result<std::unique_ptr<ChainRunner>> create(VulkanContext&, const RunnerDesc&);
    Result<void> upload(uint32_t buffer, std::span<const std::byte> bytes);  // staging copy, fenced
    Result<void> read(uint32_t buffer, std::span<std::byte> out) const;      // on demand, fenced
    Result<uint32_t> record(std::span<const ChainDispatch> chain);           // recorded once; returns its id
    Result<void> submit(uint32_t chain, uint64_t n);                         // n submits of the same buffer
    Result<std::vector<uint64_t>> durations_ns(uint32_t chain) const;        // per mark, or empty
};
```

- **Buffers.**
  - Every buffer is device-local and lives as long as the runner. One host-visible staging buffer, the largest of any upload or read, is reused.
  - One descriptor set layout of N storage buffers, binding i = buffer i. It is its own layout, not `bindings.slang`'s state set, so no generated binding and no mirror is touched (G1).
  - A fluid step is one recorded chain, resubmitted. Publish is a second chain.
- **Refusals, by name (`L6`).** `create()` and `record()` refuse when:
  - a buffer's bytes exceed `maxStorageBufferRange`;
  - the buffer count exceeds `maxPerStageDescriptorStorageBuffers` or `maxDescriptorSetStorageBuffers`;
  - the push size exceeds `maxPushConstantsSize`;
  - any dispatch's groups exceed `maxComputeWorkGroupCount` on an axis;
  - a pipeline index or binding is out of range;
  - the total bytes exceed the VRAM budget.
- **The VRAM budget (a Core hunk in `VulkanContext`).**
  - Enable `VK_EXT_memory_budget` when it is present, and expose the device-local heap's budget.
  - The runner refuses past that budget less a stated reserve (the spec's 4 GiB airflow cap is the default ceiling).
  - Without the extension, it refuses past half the device-local heap and says that it is using the fallback.
- **Timestamps:** `timestamps.hpp` as `StepRecorder` uses it, reset once per recording. On a device that cannot time compute work, `durations_ns` is empty, not an error.

## 3. Determinism, and its one exception

- The chain runs in a fixed order, with the barrier rule above. Every kernel mirrors plan A's expressions term for term, in the same order: faces −x, +x, −y, +y, −z, +z; the lerp form; red then black. GPU results are graded against the CPU within `TD-14` bands.
- **The exception, named (the lead, 2026-10-12):** plan A's `remove_mean`, for closed domains, is a sequential double sum. The GPU computes it as a **fixed-shape tree reduction**, the same tree for every run on a device. That is deterministic per device, and its difference from the CPU's sum is banded under `TD-14`. A literal mirror would be one thread, or fp64 at 1/64 rate.

## 4. Tests (plan B's first task, before any solver kernel)

- `DispatchChain` extraction: the existing `recorded_chain()` counts and the GPU parity suites, unchanged.
- The runner, using two tiny test kernels (copy, then add-one):
  - an upload followed by a read is exact;
  - a two-dispatch chain sees the first kernel's write (the barrier);
  - recording once and submitting n times gives n increments;
  - each refusal fires by name. The VRAM and range cases ask for one buffer of `maxStorageBufferRange + 4` bytes and are refused before any allocation;
  - timestamps are present, or empty on an unsupported device.

All of these are GPU-labelled, so they run only when no game is up (Cameron's throttle rule).

## 5. What stage 7 takes, and what it drops

- **Takes:** `dispatch_chain` (already shared), the limit and budget refusals, and the record-once pattern.
- **Drops:** `ChainRunner`. The `airflow` module's buffers become stage 7's resident arrays with their hashed configuration (`PHY-9`, Q12), and plan C retires the runner, except as a bench.
