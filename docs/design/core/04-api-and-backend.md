# Core — the public API, numerics and the backend seam

**Owner:** Core. **Normative.** Where the code is today is in `07-status.md`.

## The public API

- **One entry point.** `Simulation` (with `WorldSetDesc` and the world description) is the engine's public surface. Interface and every consumer use only the installed headers. A need the API does not meet is a finding for Core.
- **Errors are values.** Fallible calls return `Result<T>` (`std::expected<T, spade::Error>`), and no exception crosses a module boundary. `Error` is a closed code plus a context string (`engine D12`). `Code::unavailable` means a resource is architecturally absent, such as no Vulkan device or a behavior the backend cannot run. It is how `L6`'s refusals are spelled.
- **Handles are typed.** A handle is an index plus a generation, and its generation is checked on use. There are no bare integer aliases.
- **Isolation.** There are no globals and no process-wide teardown. N `Simulation`s in one process is a supported configuration. A `Simulation` is externally synchronized (one caller thread).
- **Diagnostics stay off the step path.** Calls like `vulkan_step_witness()` and `vulkan_pass_durations_ns()` read; they never change what a step computes.

## Numerics

- **fp32 by default** for state and step math (`engine D2`). A module may declare fp64 as part of its grade (charter numerics note).
- **Pass op order is part of the result.** Non-MSVC builds compile engine math with `-ffp-contract=off` so the optimizer cannot fuse a multiply-add the pinned order spells as two roundings.
- **No libm transcendental feeds state.** `core/fp32_math` provides `log32`, `exp32`, `sin32` and `cos32`, bit-reproducible across MSVC and glibc. On the CPU, `+ - * /` and `sqrt` are IEEE-exact and stay. Kernels use the Slang port of the same functions, which adds `div32` because the device's divide is not exact. They never use vendor intrinsics.
- **Authored rotations use exact spellings** (sqrt forms or correctly-rounded constants), never `glm::angleAxis`.

## The backend seam

`BackendDesc` picks the backend at `create()`. The CPU is the default and the reference. Vulkan is the GPU path. The choice is runtime, not compile time, so a build without Vulkan still links and refuses a Vulkan request with `Code::unavailable`.

**The configuration surface** has three tiers (`engine A7`):

1. **Simulation content.** Affects results, is versioned and is hashed: `dt`, substeps, world content, contact and grid parameters, turbulence.
2. **The backend block.** Device, workgroup size, batching selection. It is **result-neutral by contract**, and each knob owes an invariance test (digest equality across its settings). CPU-versus-GPU is never a tunable.
3. **Not configurable.** Schedule shape, op order, publish semantics.

**The Vulkan path** (`compute/vulkan/`, Core owns the seam, context, mirror and recorder):

- **Headless by construction.** No surface is needed to step or to sense.
- **The device must preserve fp32 denormals.** `create()` refuses one that does not.
- **The state mirror is derived storage.** It uploads and reads back every registered array, and calls nothing that registers state.
- **Record once per shape.** The whole substep chain is recorded once and resubmitted per step. The only per-step input is the tick, written through a persistently mapped buffer.
- **A barrier between every adjacent pair of dispatches, none after the last.** The last dispatch is found from what was emitted, never from a tally (`CORE-2`).
- **The chain is derived from the schedule** (target, `01-modules-and-scheduler.md`). Today it is a separate table.

Kernels and their parity bands belong to the module that owns them (Physics for physics passes, Rendering for raster). The SPIR-V rules and the parity harness belong to Test/Docs.

## Grades (`L3`, `L4`)

- **Each module declares a grade for each backend:** reference (CPU implementation plus golden), banded (a measured band against the reference, with its provenance), or best-effort.
- **A world's grade is its weakest module's grade on the chosen backend.**
- **At `create()`**, a consumer may state a minimum grade, and a world below it is refused, never quietly downgraded.
- **Today** there is no grade declaration. The CPU path is reference by construction, and the Vulkan physics path is banded by the parity suite (`07-status.md`).
