# Module API Stage 7 Implementation Plan — the airflow module's GPU groundwork

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **This plan is written batch by batch:** batches A, B and C are written to step level; D, E and F are written to task level (files, named tests, blockers) and are expanded at their cut, when their blocking input exists (see "How this plan is staged").

**Goal:** At the end of stage 7 the airflow module (Physics' plan C) can be written against stable engine API, with no further Core change, for the disc, line and blade-resolved rotor models and both surface modes (`CORE-8`, design §1 done-when 1–9). A set without such a module computes today's bits on both backends.

**Architecture:**
- **The airflow block is descriptor set 1, "airflow"** (design "For the lead" 4, amended 2026-10-21; B0 built it: 28 bindings, append-only). Set 0, the mirror, `StepShape` and every identity stay byte for byte, and set 0 stays the consumer contract: 30 bindings (stage 6's consumers line).
- **Only the kernels that read the module's resources take set 1,** and only the bindings they name. A set layout is a subset of the one declaration; the binding numbers are the declaration's. Standard kernels that read module data optionally (T8's point readers, T12's loads) use stage 6's dummy-when-absent and switch-bit rule at set 1, never new set 0 bindings.
- **T1 counts the limits per pipeline layout, across its sets,** and refuses by name with the layout, the count and the limit.
- **Everything the design signed stands** (`CORE-8`, D1–D7). This plan re-anchors it to master after stage 5, stage 6, `CORE-7` parts and Physics' standalone GPU solver, and shrinks the tasks those already covered (next section).

**Tech Stack:** C++23, Slang kernels, GoogleTest through ctest, Vulkan 1.1 (no optional features), MSVC + Ninja presets, the gcc-13 Docker leg.

**Spec:** `2026-10-12-module-api-stage7-design.md` (signed, `CORE-8`); `../2026-10-05-airflow-spec-core-answers.md`; `2026-10-21-airflow-gpu-b0-plan.md` (set 1, the limits and the budget); `2026-11-05-module-api-stage6-plan.md` (the pattern). Physics' side: `../../physics/plans/2026-10-22-airflow-gpu-solver-plan.md`.

**Checked against:** master `6e66edab` (stage 6 merged, `801efd16`). Every `file:line` below must be re-checked at each batch's cut.

## What exists now (read before the batches)

| Piece | State on master | Consequence for the plan |
|---|---|---|
| Device limits | `compute/device_limits.hpp`: `DeviceLimits`, `check_buffer`, `check_storage_buffers`, `check_push`, `check_groups`, `check_workgroup`, `over_limit`; `DeviceReport::limits`; tests `test_device_limits.cpp`. **Called only by the spike's `ChainRunner`.** No engine path calls them | T1 is **wiring**, not new checks |
| Memory budget | `MemoryBudget`, `vram_cap()`, `check_vram()`, `VulkanContext::memory_budget()`; `kAirflowVramCap` = 4 GiB | T14 is `memory_report()`, the cap in the result-neutral block, and the create-time check |
| Set 1 | `bindings.slang` section F: 28 bindings `airflow_params` … `airflow_wall_load` (0–27), `kSet_airflow`, `kBindingCount_airflow` = 28, `kAirflowSetBuffers` in `spike/airflow/chain_runner.hpp` tied to it by `static_assert`; `tests/test_bindings_airflow.cpp` | New set-1 entries are **appended at 28 and up**, never renumbered; Physics confirms the list |
| The runner | `spike/airflow/chain_runner.*`: a pipeline layout of an **empty set 0 plus set 1**, three buffer kinds (device-only, resident, staged), recorded chains with barriers, timestamps | The recorder extraction (G2) is T3's; the runner stays the per-kernel parity tool |
| Grid kernels | `engine/shaders/kernels/airflow_*.slang` (Physics): `tile_sum`, `tile_dot`, `pack_f16`, `copy`, `publish_fields`, the solids kernels. **Solids are packed on the host** (`solids_chain`) and uploaded | T10 **audits first and builds only what is missing**: the posed-SDF raster, the integer-shift copy, point interpolation if absent |
| Publish | CPU: `publish_render_volume` (threaded, merged `30520d3e`). GPU: `airflow_publish_*` kernels and `publish_chain` in the spike (B6) | T13 moves Physics' publish kernels behind `request_publish`; it does not write a second GPU writer |
| Parts | `ModelType::parts`, `parts_identity` (`CORE-7`), part contact (`3ed1a95a`) are built; the part table is not (a configuration table, T11) | T11's external dependency is met |
| The pattern | `compute/optional_bindings.hpp`, `PassParams::switches` (48 B), `StepShape::switches`, the mirror's dummy-when-absent | T8, T11, T12 add rows, not a mechanism |

## Global Constraints

- **No existing golden moves and none is regenerated.** The standard identity `0x3303cfc86821f502`, the one-body schema hash `0x1f60a6fef22e254a`, snapshot format v3, the 22-entry standard walk, every corpus golden and `kHeldQuadDigest` hold at every task's head. Every GPU parity band holds unchanged (`TD-2`).
- **Set 0 does not move after stage 6:** 30 bindings, `StepShape` unchanged but for fields a task names, the standard kernels' pipeline layouts byte for byte (set 0 only) except the two optional readers (T8, T12), whose layouts gain a set-1 subset and say so.
- **Wire contracts that do move, and only these** (each generated or pinned): `PassParams` grows by a few generic lanes (T7); set 1 gains appended bindings (T8, T11, T12); `DeviceReport` and `BackendDesc` gain fields (T1, T14); `ArrayDecl`, `PassDecl`, `ModuleDesc` gain defaulted fields (T5, T6, T7, T4). Every SPIR-V variant is rebuilt when `PassParams` moves; no fp32 expression in a standard kernel changes.
- **Refuse, never skip (`L6`):** every refusal names the module, the buffer, the layout or the pass, and comes before any allocation or write.
- **Determinism:** no atomics; every scatter a gather in a fixed order; reductions that feed stepping are fixed-tile, fixed-block (`kReduceBlockTiles` = 256) and workgroup-independent; the recording is a function of the tick; dispatch counts fixed at `create()`; halves and bytes packed in integer arithmetic; no libm and no wall-clock under `engine/` (`BitPortability*:M1B*`); set order then declaration order.
- **Layering:** `compute/device_limits.hpp` and `compute/optional_bindings.hpp` stay Vulkan-free; `physics/` includes no `sim/`.
- **Grades:** every module with a resident array declares CPU `reference` and Vulkan `banded` (design §2.1, §8).
- `-Wall -Wextra -Wpedantic` clean under gcc, warning-free under MSVC.
- **Where the code lives:** `../spade-wt/core-2` (or the slot's worktree), **one branch per batch** `core/module-api-stage7-<a..f>`, cut from master when the lead approves that batch's section, merged serially, each through its own batch gate as stage 6 was. Commit only named paths; never push, never stash, never amend master.
- **The branch gate:** red first; per task the focused sets; at a batch's end the full CPU `ctest -L spade -LE gpu` and `-L gpu`, each one `jobrun` ticket under the lead's slot (`--queue-status` first, targeted builds, `-j 4`, default floor). Debug, the `-NoSeed` leg and smoke run at the batch gate. `gcc-check.sh --compile` on every changed `.cpp`, plain on headers, after a build has regenerated the headers.
- **Red first.** Every task commits its failing tests before the change; the red commit compiles; a test green at red is a pin and is named in its comment.

## How this plan is staged

| Batch | Tasks | Written to | Blocked on | Owner of the blocker |
|---|---|---|---|---|
| A | T1, T2, T14 | step level | — (slot after Interface's drone box) | — |
| B | T3, T7, T10 | step level | A's T1 | Core |
| C | T4, T5, T6 | step level | B's T3 | Core |
| D | T8, T9 | task level | C's T5 (resident rows); Physics confirms the set-1 appendices | Physics |
| E | T11, T12 | task level | **Physics' ψ pass and load formulas** (design §5.3, §5.5); D's switch rows | Physics |
| F | T13, T15 | task level | **Physics' publish kernels for the engine path** (the CPU writer `publish_render_volume` exists; the GPU kernels are in the spike) and the `FlowVolume` band record | Physics |

D, E and F are expanded to step level at their cut, with the blocking input in hand: their formulas and bands are Physics', and writing steps against inputs that do not exist yet would only be rewritten. The lead approves each batch's section before its branch is cut. **Batch C moves an identity-adjacent value** (decision 5); its section is the one the lead takes to the owner if it has to go that far, and Kat is warned when it is approved.

## Decisions this plan makes

1. **One set-1 declaration, per-kernel subset layouts.** `bindings.slang` section F stays the single list. A pipeline's layout carries set 0 in full (any kernel that reads state needs the full state set) and, if it reads module data, a **set-1 layout holding only the bindings the kernel names**, at the declaration's numbers. `maxPerStageDescriptorStorageBuffers` and `maxDescriptorSetStorageBuffers` count every set of a layout, so a standard kernel with a set-1 subset of k bindings costs 30 + k and a module kernel costs 30 + its k; all other standard kernels cost 30, as today. The subset is declared per kernel group in one table (`compute/layout_classes.hpp`, Vulkan-free), read by the recorder, the mirror and T1's check. *Alternative:* the full 28-binding set 1 in every layout that touches it, 58 per stage; rejected, it pushes T8's readers past what the guaranteed minimums could ever admit and buys nothing.
2. **Names are the contract (as `lock_rows` is).** A walk array or scratch whose name is in `kSetOneBindings` (a table beside `kOptionalBindings`: name, set-1 binding, dummy bytes, switch bit or none) binds at that set-1 binding; the mirror builds a set-1 descriptor set only when the schedule has a set-1 user, and writes the zeroed dummy for an absent optional one. The generated `kBinding_airflow_*` constants are the numbers; a `static_assert` ties the table to them as `kAirflowSetBuffers` is tied.
3. **Appended set-1 bindings (proposed; Physics confirms before the batch that needs them):** `airflow_points` (T8's per-point rows), `airflow_parts` (T11's part table), `airflow_posed` (the posed-part table), `airflow_vehicle` (the vehicle rows), `airflow_rotor_model` (T12's per-rotor model row), `airflow_config` (T4's configuration image). Numbered 28 upward in that order, so `airflow_points` = 28, `airflow_parts` = 29, `airflow_posed` = 30, `airflow_vehicle` = 31, `airflow_rotor_model` = 32 and `airflow_config` = 33; the list never renumbers. 34-39 are spare. **40-47 are reserved for Physics' LBM twin** (`lbm_f` = 40, `lbm_u` = 41, declared by Physics), so `tests/test_bindings_airflow.cpp` counts the set by owner (airflow 0-27 = 28, stage 7 28-39 = 0 until a batch declares its row, LBM 40-47 = 2; the sum is `kBindingCount_airflow`), and whichever branch lands second rebases it.
4. **The limit checks run in one pure function** (`compute::admit(shape, layouts, limits, device)`) called by `create()` before any allocation and tested with synthetic limits on every machine. It does not change what a working device does: no device of record is near a minimum (the devices' values are recorded by the first test run and printed in the batch's READY).
5. **`parts_identity` folds into the snapshot's registry identity only for a set that declares a part table.** `registry_identity()` already folds every non-empty configuration table (L10/L11) as a tail; stage 7 adds a second tail after it: when the schedule declares a part table, for each registered model with non-empty parts, in registration order, `parts_identity(model)`. A set without that table, which is every set today, gets the value it gets now. `model_identity()`, `parts_identity()` and `kNoPartsIdentity` never change. The persisted-things list is in Task C1, and the lead is asked to approve this gate before C's code (Question 1).
6. **The tests do not need Physics' module.** Stage 7 is defined by capability (design §8): a test-local module declaring a resident grid, a set-1 user and a parts reader, plus the domain-neutral grid kernels, exercises everything. Kernel-level parity uses the existing `ChainRunner`, which stays the per-kernel tool (design §9).
7. **D2 is signed at `j` = 2** by default and re-set by Physics' spike; stage 7 builds the mechanism, not the default (the default is the module's configuration, T4).

## Review Focus (the failure modes the tasks' tests are built around)

1. **A minimum-spec device** (4 storage buffers per stage) meets today's 30-binding set 0: `admit` must refuse it by name, naming the layout, 30 and 4, not crash and not refuse a working device.
2. **A standard kernel with the switch off** reads a set-1 dummy and must compute today's bits (T8, T12), and a set that never builds set 1 must create no set-1 descriptor set at all.
3. **A snapshot taken between fluid steps** (`j` = 2, tick odd) restores onto the same recording, with resident arrays, held loads and ψ carried.
4. **A stale resident array read through any host path** (`const snapshot()`, `module_rows`, `arenas()`, `state_digest`) syncs or refuses by name; never returns stale bytes silently.
5. **A world past the configured part capacity, or a part the grid cannot resolve** is refused at spawn or `create()`, naming the part and the limit (D5), never downgraded.

---

# BATCH A — limits, device-only buffers, budget (branch `core/module-api-stage7-a`)

Independent of every other batch. Gate: release CPU suite, the GPU tier (Vulkan code), `gcc-check`, a leg (new engine `.cpp` and a generated header).

## Task A1 (T1): `admit` — the device-limit checks wired into `create()`

**Files:**
- Create `engine/compute/admit.hpp` (Vulkan-free) and `admit.cpp`: `compute::admit(const AdmitShape&, const DeviceLimits&, std::string_view device) -> Result<void>`; `AdmitShape` lists the layouts (name, bindings per set), every buffer (name, bytes), every dispatch (pass, groups) and every kernel (name, local size) of a created `StepShape`.
- Create `engine/compute/layout_classes.hpp` (Vulkan-free): the table of decision 1, starting with one class, `state` (set 0, 30 bindings), so A1 changes nothing for the standard kernels.
- Modify `engine/compute/vulkan/state_mirror.cpp`, `step_recorder.cpp`, `backend.hpp`'s create path: build the `AdmitShape` from the shape the backend is created for, call `admit` before the first `vkCreate*`; replace `step_recorder.cpp`'s axis-0-only group check with `check_groups`.
- Tests: create `tests/test_admit.cpp` (CPU, no Vulkan; registered in `tests/CMakeLists.txt`); extend `tests/test_gpu_state_mirror.cpp` for the device half.

- [ ] **Step 1: Write the failing tests** (red commit; `admit` is a declared stub returning `ok`):
  - `Admit.AMinimumSpecDeviceIsRefusedByLayoutCountAndLimit`: synthetic `DeviceLimits` at Vulkan's guaranteed minimums (`kGuaranteedLimits`) against the standard shape: refused, `Code::capacity_exceeded`, text names layout `state`, `30`, `maxPerStageDescriptorStorageBuffers`, `4`, and the device.
  - `Admit.ADeviceOfRecordLimitsAdmitTheStandardShape`: the limits captured for the devices of record (copied into the test as literals with their source named) admit it.
  - `Admit.ABufferOverMaxStorageBufferRangeIsRefusedByName`: a 16-world 128³ face array past 128 MiB, naming the buffer and the limit.
  - `Admit.ADispatchPastTheGroupCountOnAnyAxisIsRefusedByPassAndAxis`, `Admit.AKernelOverTheLocalSizeIsRefusedByKernel`, `Admit.APushBlockPastTheLimitIsRefused` (`sizeof(PassParams)` against 128).
  - `Admit.TheCountIsPerLayoutAcrossSets`: a synthetic layout of set 0 (30) plus a set-1 subset of 3 is counted 33, and refused at a limit of 32 naming both sets' contributions.
  - `GpuStateMirror.CreateRecordsTheLimitsAndAdmitsTheStandardShape` (device): `device_report().limits` is non-zero and the standard create succeeds.
- [ ] **Step 2: Run, confirm each fails for its stated reason** (`ctest -R Admit|GpuStateMirror`). Keep the output.
- [ ] **Step 3: Implement** `admit` over B0's functions; no new limit logic. `create()` refuses before allocating anything (a test with an injected tiny `max_storage_buffer_range` asserts no `vkAllocateMemory` happened, by the absence of any mirror entry in `memory_report()` once A3 lands, and by a counting hook until then).
- [ ] **Step 4: Focused green:** `Admit|DeviceLimits|GpuStateMirror|GpuInvariance|GpuParity` (nothing admitted today is refused; the parity bands are untouched). `gcc-check` the changed `.cpp`.
- [ ] **Step 5: Commit** red, then green, separately.

## Task A2 (T2): scratch and derived buffers lose their staging twins

**Files:** `engine/compute/vulkan/state_mirror.cpp` (the entry builder at `:514-535`, the derived builder at `:574-600`), `state_mirror.hpp`; tests in `tests/test_gpu_state_mirror.cpp`.

- [ ] **Step 1: Write the failing tests:** `StateMirrorScratch.AScratchBufferHasNoStagingTwin` and `…ADerivedBufferHasNoStagingTwin` (the entry's `staging_buffer` is null; its `byte_size` is unchanged); `GpuStateMirror.ScratchIsZeroAtCreateWithoutAHostWrite` (read-back through the witness path shows zeros, so the zero-fill moved to `vkCmdFillBuffer`); `GpuStateMirror.StagingIsNotDeviceLocalOnADiscreteDevice` (skips on an integrated device, names why); the existing `DescriptorSetBindsEveryRegistryBinding` stays green. A host-size assertion: the mirror's host-visible bytes drop by exactly the scratch plus derived sizes.
- [ ] **Step 2: Run, confirm fail** (twins still exist).
- [ ] **Step 3: Implement.** Staging stays for: walk entries, `step_params`, the configuration tables' uploads, and the read-on-demand diagnostics. The dummy for an absent optional binding (stage 6) is device-only, zero-filled by `vkCmdFillBuffer` in the create-time command buffer. `find_memory_type` records the type and heap chosen per buffer kind (read by A3).
- [ ] **Step 4: Focused green, then the whole `-L gpu` tier.** The standard set's device state must be unchanged: print one same-device `state_digest` of the standard set after N Vulkan steps before and after this task (the one-off overlay of stage 6's Task 4; not committed) — **READY prints both numbers**.
- [ ] **Step 5: Commit.**

## Task A3 (T14): the memory budget, the cap and `memory_report()`

**Files:** `engine/compute/backend.hpp` (`BackendDesc::vram_cap_bytes`, default 0 = the budget's own; `MemoryReport`, `memory_report()`), `engine/compute/vulkan/state_mirror.*` and `step_recorder.*` (feed every allocation to a ledger), `engine/sim/simulation.*` (`memory_report()` forwards; a CPU run reports the host arena only). Tests: `tests/test_memory_report.cpp`.

- [ ] **Step 1: Write the failing tests:** `MemoryReport.ListsEveryBufferWithKindBytesMemoryTypeAndHeap`; `MemoryReport.TheHostArenaIsReported`; `MemoryReport.ACapBelowTheTotalRefusesCreateNamingTheLargestBuffer` (uses `check_vram`; code `capacity_exceeded`; names what asked, bytes, the cap and how it was reached); `MemoryReport.ACapChangesNoResult` (the result-neutral block: two creates, cap 0 and a generous cap, same `state_digest` after N steps; `engine A7`'s invariance form); `MemoryReport.TheCpuBackendReportsNoDeviceBuffers`.
- [ ] **Step 2: Run, confirm fail. Step 3: Implement,** the reserve a named constant beside `kAirflowVramCap`. **Step 4:** focused sets, `-L gpu`, `gcc-check`. **Step 5: Commit.**

**Batch A READY** prints: counts, the device limits recorded, A2's before/after digests, and any pin that moved (named as expected moves).

---

# BATCH B — 3D dispatch, recipes as functions, cadence, grid kernels (branch `core/module-api-stage7-b`)

Needs A1 (`admit`, `layout_classes`). Gate as A, plus the recorded-order and `CORE-2` barrier tests on every recording.

## Task B1 (T3): 3D dispatch; recipes as recording functions; layout classes; pipelines on demand

**State at the cut (2026-11-07).** The G2 extraction is already on master: `compute/vulkan/dispatch_chain.{hpp,cpp}` records the chain and its barriers for the recorder and the `ChainRunner`, and `ChainCommand` carries `groups[3]`. What is left is four things. (1) **Recipes as recording functions** (lead's ruling, 2026-11-07): `compute::RecipeDesc {name, kernels, per-kernel set-1 subset, record fn(Emitter&, RecipeContext&)}`; `GpuPass` and `PassDecl` carry a pointer to one, defaulting to none; the twelve built-in recipes become `RecipeDesc`s of the same form, so there is one path and no global registry; the standard chain comes out identical. (2) **The 3D knob**: `compute/dispatch_shape.hpp` (`local_shape`, `grid_groups` with the worlds folded into z, `world_of_group_z`) and per-size `SPADE_WG_X/Y/Z` variant defines in `cmake/SpadeSlang.cmake`, with test kernels in `tests/kernels/*.slang`; the `Dispatch3D` id test runs through the `ChainRunner`. Every existing 1D kernel's SPIR-V is byte-identical before and after the cmake change (a test compares the variants' bytes). (3) **Layout classes**: `compute/layout_classes.hpp` (`kernel_layout`: set 0's thirty bindings plus the set-1 subset a kernel names, counted as a sum by `admit()`). (4) **Pipelines on demand**: only the recipes the schedule names are built. The REDs are in three commits: the host arithmetic and layouts first, then the recorder's.

**Files:** `engine/compute/vulkan/step_recorder.{hpp,cpp}` (the recipe switch at `:590-666` becomes a registry of recording functions; one emitter; a recorded command carries three group counts), `engine/compute/vulkan/dispatch_chain.hpp` (the chain-and-barrier code **extracted once**, used by the recorder and by `spike/airflow/chain_runner` — G2), `engine/compute/layout_classes.hpp` (adds the `state+subset` class), `engine/compute/backend.hpp` (`ChainCommand` and the recorded command: `groups[3]`). Tests: `tests/test_gpu_dispatch_3d.cpp`; the existing `RecordedChain` and `CORE-2` tests.

- [ ] **Step 1: Write the failing tests:**
  - `Dispatch3D.AKernelSeesEveryGlobalIdOnce` (a test kernel in `tests/kernels/` writes its linear id; every cell once; no cell twice): at 16³, 33×17×9 (not a multiple of the local shape), two worlds.
  - `Dispatch3D.WorldsFoldIntoTheZGroupsEachWorldContiguous`.
  - `Dispatch3D.TheWorkgroupSizeKnobMapsTo3DShapes` (32 → 4×4×2, 64 → 4×4×4, 128 → 8×4×4) `…AndTheThreeSizesAreBitIdentical` (the A7 invariance sweep over a grid kernel).
  - `RecordedChain.TheStandardChainIsUnchanged` (**pins**, green at red: the standard chain's dispatch count, order and barriers; named as a pin in its comment).
  - `RecordedChain.OnlyTheRecipesTheScheduleNamesAreBuilt` (a set without the test module builds the same pipelines as today; one with it builds one more).
  - `LayoutClasses.AModuleKernelCountsSetZeroPlusItsSetOneSubset` (host-only; count 30 + k) and `…AStandardKernelWithoutASetOneUserIsStillThirty`.
  - `GpuStateMirror.ASetWithNoSetOneUserCreatesNoSetOneDescriptorSet` (**Review Focus 2**).
- [ ] **Step 2: Run, confirm fail. Step 3: Implement** (extraction first with the chain tests green, then the registry, then 3D). **Step 4:** focused sets; `GpuInvariance*` and `GpuParity*` unchanged; the whole `-L gpu`. **Step 5: Commit** in the order extraction, registry, 3D.

## Task B2 (T7): the cadence — `every_steps`, `due()`, two recordings, the substep index

**Files:** `engine/sim/module.hpp` (`PassDecl::every_steps`, default 1; `SubstepContext::substep`), `engine/sim/cadence.hpp` (new; `due(tick, substep, every_steps)`, the one rule), `engine/physics/schedule.cpp` (`compile_schedule` refuses two different `every_steps` among the set's passes, naming both), `engine/sim/simulation.cpp` (the CPU scheduler skips by `due`), `engine/compute/vulkan/step_recorder.*` (second command buffer recorded once at `create()`; submit picks by `(first_tick + i) mod j`; a query pool per recording), `engine/compute/backend.hpp` (`vulkan_pass_durations_ns()` also reports which recording ran and the tick), `PassParams` (+ generic lanes: level, colour, sweep, sub-step), `engine/shaders/shared/layouts.slang`. Tests: `tests/test_cadence.cpp`, `tests/test_gpu_cadence.cpp`.

- [ ] **Step 1: Write the failing tests:** `Cadence.DueIsTheOneRule` (table over tick, substep, j); `Cadence.AScheduleWithTwoDifferentEveryStepsIsRefusedNamingBoth`; `Cadence.TheCpuSkipsAPassOffItsTick` and `…RunsItInSubstepZeroOnly`; `Cadence.ASnapshotBetweenFluidStepsRestoresOntoTheSameCadence` (tick odd, j = 2, CPU; **Review Focus 3**); `GpuCadence.TheCore2BarrierTestPassesOnBothRecordings`; `GpuCadence.TheRecordedOrderHoldsOnBothRecordings`; `GpuCadence.TimingNamesTheRecordingThatRan`; `GpuCadence.ASkippedStepHasNoFluidTiming`; `Cadence.TheStandardSetHasOneRecordingAndTheSameDispatchCount` (**pin**, green at red); the generated layout check for `PassParams`.
- [ ] **Step 2–5:** run red, implement, focused + `-L gpu` (every SPIR-V variant rebuilds; `GpuParity*` bands unchanged), commit. READY names `PassParams`' new size.

## Task B3 (T10): the domain-neutral grid kernels Core owns

**Audit settled (lead, 2026-11-07).** The integer-shift copy (`grid_shift`) and the trilinear point interpolation (`grid_points`) are new: `airflow_copy` is a plain copy, and the solids are packed on the host. Their CPU twins come first (`engine/world/grid_kernels.hpp`, header-only), then the Slang kernels; Physics reviews both. **`grid_raster` moves to batch E**, where its inputs, the part table and the posed parts, exist; it stays **Core's** kernel (Core owns the domain-neutral grid kernels; Physics reviews), and does not move to Physics.

**Audit first (Step 0, no code):** list which of {posed-SDF raster, fixed-order tile sums, integer-shift copy, trilinear point interpolation, fill/copy, fp16 pack} already exist as `airflow_*.slang` (Physics: `tile_sum`, `tile_dot`, `copy`, `pack_f16`, `publish_fields`), confirm with Physics that they are the ones the module will use, and write the missing list into the commit message. Core builds **only the missing ones**; Physics reviews them (design "For the lead" 3).

**Files (expected, after the audit):** `engine/shaders/kernels/grid_raster.slang` (the face thread loops parts in (world, body slot, part index) order, keeps the smallest `φ`, ties to the lower part index; writes `φ`, owner, wall velocity), `grid_shift.slang` (integer-shift copy), `grid_points.slang` (trilinear points, only if absent), CPU twins in `engine/world/grid_kernels.{hpp,cpp}` using `world/sdf.hpp`'s `eval()`; `tests/test_grid_kernels.cpp`, `tests/test_gpu_grid_kernels.cpp` (through `ChainRunner`).

- [ ] **Step 1: Write the failing tests:** `GridRaster.AMovingSphereMatchesTheCpuTwinWithinTheBand` and `…ASpinningBarMatchesTheCpuTwin` at 16³; `GridRaster.ATieGoesToTheLowerPartIndex`; `GridRaster.OwnerDisagreementsAreCountedNotBanded` (the fraction pinned as a statistic with its record, design §5.6); `GridRaster.TheThreeWorkgroupSizesAreBitIdentical`; `GridShift.IsBitExactAndWrapsNothing`; `GridPoints.MatchTheCpuTwinWithinTheBand`; `GridKernels.TheCpuTwinsFollowTheDeviceOrderOfEvaluation` (an fp32 expression-order test against hand-evaluated values). Bands under `TD-14`; a band wider than 1e-4 relative is a grade change (`TD-2`), never widened.
- [ ] **Step 2–5:** red, implement, focused + `-L gpu`, `gcc-check`, commit. Physics reviews the raster before the batch merges.

---

# BATCH C — hashed configuration, resident arrays, the rebuild hook (branch `core/module-api-stage7-c`)

The risky batch: it touches the identity spelling, the digest and the snapshot paths. **Its section is approved by the lead before the branch is cut, and Kat is warned then.**

## Task C1 (T4): hashed module configuration; the gated `parts_identity` fold

**Files:** `engine/sim/module.hpp` (`ModuleDesc::config`, a span of the canonical image; the identity spelling at `:605-615`), `engine/physics/schedule.cpp`, `engine/sim/simulation.{hpp,cpp}` (`configuration_report()`, `registry_identity()`'s second tail), `engine/vehicles/model_identity.hpp` (comment only), `engine/shaders/shared/bindings.slang` (appends `airflow_config` at 33). Tests: `tests/test_module_config.cpp`, `tests/test_parts_fold.cpp`.

**The persisted things this task refuses or moves (the lead's requirement), and how each is caught:**

| Persisted thing | Moves? | Why | Caught by |
|---|---|---|---|
| Standard identity `0x3303cfc86821f502`, schema hash `0x1f60a6fef22e254a`, the 22-entry walk | **No** | a module with no config appends nothing | `StandardModules.StateDeclarationsLeaveTheOrderAndTheIdentityAlone` (existing) |
| Snapshot format v3 | **No** | a resident array is one more section; a blob from a set without airflow is byte-identical | `Snapshot.AStandardBlobIsByteIdenticalAfterStage7` (new, green at red: a blob made in-process and compared with a fixture byte stream saved from master `6e66edab` — committed as a hex literal) |
| A stored blob from a set without a part-table module (all of Kat's today: session ring, replay, saved cases) | **No** | `restore()` checks the schedule identity first; the registry identity of a set without a part table is the value it is now | `ModelSnapshot.ABlobRestoresAfterTheModelsGainParts` (existing, must stay green and unchanged), `test_model_parts.cpp:315,572` (existing pins that parts do not move `model_registry_identity`), new `PartsFold.ASetWithoutAPartTableKeepsItsRegistryIdentity` |
| `model_identity`, `parts_identity`, `kNoPartsIdentity` | **No** | never change | `test_airframe_compile.cpp:889` pin and `test_model_parts.cpp:625` (existing) |
| Interface's CFD case pin `(model_identity, parts_identity)`, live-run files | **No** | `parts_identity` is unchanged; `live_run.cpp:1008` already stores it | existing `AnalysisCaseFile*` and live-run tests |
| A blob from a set **with** a part-table module and models with parts | **New value** | that set does not exist yet, so no blob exists; the first ones carry the fold | `PartsFold.APartsModelChangesTheRegistryIdentityOfAPartTableSetOnly`, `…ARestoreUnderOtherPartsIsRefusedNamingBothIdentities` |
| Every corpus golden, `kHeldQuadDigest`, every GPU band | **No** | digests fold arrays, not the registry identity (`replay.hpp`) | the full suites |
| `docs/` records | **No** | none pins a registry identity (searched) | `AnalysisValidation*` |
| A world's `config_hash` | **No** in this stage | regions' placement joins it only when a world declares one (T9) | `ReplayConfig*`, `ReplayConfigHash*` (existing) |

**Nothing frozen or signed moves,** so nothing goes to Cameron — **provided Question 1 is answered "yes"** (the gate on a declared part table). If the lead reads `CORE-8`'s "once a step reads it" as requiring the fold for every set holding a model with parts, a frozen value moves for every such set and the lead takes it to Cameron before any code.

- [ ] **Step 1: Write the failing tests:** `ModuleConfig.AModuleWithNoConfigKeepsItsIdentity`; `…AConfigImageIsSpeltAfterTheVersionWith0x02AndItsLength`; `…TwoConfigsGiveTwoIdentities`; `…TheSameBytesAreWhatTheKernelsRead` (the uploaded buffer equals the hashed image); `ModuleConfig.ARestoreUnderAnotherGridIsRefusedByIdentity`; `ModuleConfig.ConfigurationReportListsEachModuleNameVersionAndHash`; the `PartsFold.*` and `Snapshot.*` rows above. A canonical image must be trivially copyable with no implicit padding, asserted field by field as `ReplayConfig` is.
- [ ] **Step 2–5:** red; implement; focused (`ModuleConfig|PartsFold|ModelSnapshot|ModelParts|ModelIdentity|Snapshot|ModuleSchedule|StandardModules|Determinism|GoldenCorpus`); `AnalysisValidation*`; commit.

## Task C2 (T5): device-resident arrays

**Files:** `engine/sim/module.hpp` (`enum class Residency`; `ArrayDecl::residency`), `engine/physics/schedule.cpp` (`compile_schedule` refuses `device` on anything but `per_world`, and on a module without CPU `reference`, naming module and array; refuses `elem_size` past 4 GiB), `engine/compute/vulkan/state_mirror.*` (each entry carries its residency; the per-step pair skips resident entries; `upload_resident()`, `readback_resident()`; one shared staging buffer sized to the largest resident row; the set-1 descriptor set and `kSetOneBindings`, decision 2), `engine/sim/simulation.*` (the ledger `in_sync / host_newer / device_newer` beside `vulkan_dirty_`; `sync_resident()`, `resident_current()`; the non-const `snapshot()` and `module_rows<T>()` sync, the const ones refuse `unavailable` naming the array; a non-const `arenas()` syncs), `engine/testing/replay.hpp` (`state_digest(Simulation&)`: **count the const call sites first** and put the count in the commit message; if many, add a refusing const overload instead, as the lead ruled). Tests: `tests/test_resident.cpp`, `tests/test_gpu_resident.cpp` with a test-local module `resident_grid_module` in `tests/`.

- [ ] **Step 1: Write the failing tests:** on CPU, `Resident.ADeviceArrayIsAnOrdinaryHostArrayOnTheCpu` (snapshot, digest, goldens as today); `Resident.OnlyPerWorldMayBeDevice`, `…RequiresACpuReference`, `…ARowPast4GiBIsRefused`; on the device, `GpuResident.AStepUploadsAndReadsBackNothingResident`, `…ASetRotorCommandsLoopDoesNotReuploadTheGrid` (counts the bytes moved per step), `…AHostWriteSyncsFirstThenMarksForUpload`, `…AConstSnapshotOfAStaleArrayRefusesNamingTheArrayAndSyncResident`, `…ModuleRowsAndArenasAndDigestSyncOrRefuse` (**Review Focus 4**), `…StagingIsOneBufferSizedToTheLargestRow`, `…TheDigestFoldsTheResidentArrayAfterASync` (D1 = A), `…AVulkanSnapshotRestoreResumeIsBitIdenticalToTheUninterruptedRun` (the banded-parity T7 test the design says is built nowhere yet), `…ACpuBlobRestoresIntoVulkanWithinTheBand` and the reverse; `GpuResident.AModuleWithNoSetOneUserCreatesNoSetOneSet` (pin).
- [ ] **Step 2–5:** red; implement; focused; the full CPU and `-L gpu`; `gcc-check`; commit. READY prints the const-call-site count and the host bytes before and after on a shape with a 16³ grid.
- **As built (lead, 2026-11-09):** C2 drops the planned 'row past 4 GiB' check (`…ARowPast4GiBIsRefused`): `elem_size` is u32, so it is vacuous, and `admit()` already bounds each buffer by `maxStorageBufferRange`.

## Task C3 (T6): the rebuild hook and on-demand recordings

**Files:** `engine/sim/module.hpp` (`ModuleDesc::rebuild_cpu`, `rebuild_gpu`), `engine/sim/simulation.*` (`create()` and `restore()` run the rebuild before the next step), `engine/compute/vulkan/step_recorder.*` (an on-demand recording, submitted once; the mechanism T13's publish reuses). Tests: `tests/test_rebuild.cpp`, `tests/test_gpu_rebuild.cpp` (test-local module with a derived buffer computed from its configuration and a region row).

- [ ] **Step 1: Write the failing tests:** `Rebuild.RunsAtCreateAndAtRestoreBeforeTheNextStep`; `Rebuild.ARestoreReplacesTheRegionRowAndTheDerivedBufferFollows`; `GpuRebuild.TheOnDemandRecordingIsSubmittedOnceNotPerStep`; `GpuRebuild.AVulkanRestoreResumeIsBitIdenticalWithADerivedBuffer`; `Rebuild.AModuleWithNoHookIsUnchanged` (pin).
- [ ] **Step 2–5:** red; implement; focused; `-L gpu`; commit.

---

# BATCH D — point samples and regions (branch `core/module-api-stage7-d`; step level, 2026-11-09)

Needs C2. Physics confirmed the names and layouts (below); the leads' rulings L40 (amends decision 2) and 1-5 of the survey are folded in. Plan lines for E and F are unchanged.

**Set-1 rows D declares (bindings.slang section F, `kSetOneBindings`, `is_declared_set1`):** 28 `airflow_points` (`RWStructuredBuffer<float>`; 8 words per row: pos xyz, reserved 0.0, wind xyz m/s, density; order per body in slot order, then element × k, then sensor, then host requests; counts and offsets ride in the config image as uint lanes) and 34 `airflow_centred` (declared only; owner/writer Physics, fp32 `[ux,uy,uz,p]` per cell, index `4*(x+nx*(y+ny*z))`, device-only scratch, registered only with a position-dependent provider; Physics' `airflow_centre` pass is its own slot after D). 29-32, 35-39 and 42-47 stay undeclared: `kernel_layout` refuses them by name (done in D's first commits, `79f4cbc3` RED, GREEN on top).

**L40 — amends decision 2 (switch bit 1).** There is no `airflow_points` switch bit and no dummy buffer. `drag.forces` gets a SECOND SPIR-V variant `forces_drag_points` (a second `RecipeDesc`), compiled from the one Slang source with a define, chosen only when the schedule has a point provider; the standard chain keeps the standard variant, so the standard SPIR-V rows and the device digest do not move (the new variant adds its own pin rows). Proof (Review Focus 2): the points variant fed the world sample reproduces the standard bits, CPU and device.

## Task D1 (T8): point samples

**Files:** `engine/sim/module.hpp` (`ModuleDesc::points` = {`k`, host-request capacity, `position_dependent`}; `FieldDecl` untouched in D1), `engine/sim/module_schedule.cpp` (`compile_schedule` injects the positions pass after `behaviors.kinematic` and registers the internal `points` array AFTER the 22-entry walk, only when a position-dependent provider is present; sized per world from the declared counts, 8-word rows), `engine/compute/optional_bindings.hpp` and `engine/compute/vulkan/builtin_recipes.cpp` (the second recipe, chosen by the schedule), `shaders/**` (`forces_drag` source gains a points define), `engine/compute/set_one_bindings.hpp` (collect_set_one picks `airflow_points` for a schedule with a point provider). Tests: `tests/test_point_samples.cpp`, `tests/test_gpu_point_samples.cpp` (new, named in READY), the config-image float helper test in `tests/test_module_config.cpp`.

- Readers: `drag.forces` only. Sensor points are enumerated and filled; no sensor kernel reads them in D1 (sensors read no wind today).
- `grid_points` is unchanged (reads points from `airflow_v` binding 2, writes `airflow_w`); D's test module fills its own rows, the gather from `airflow_centred` is Physics'.
- The config image of a float-bearing module is all-uint32 (floats as bits via a `float_bits(float)` helper) with `static_assert(std::has_unique_object_representations_v<...>)`; identity hashes the bits, the image is built once.

- [ ] **Step 1: Write the failing tests:** `PointSamples.AStandardSetAllocatesNothingNew` (pin: standard identity 0x3303cfc86821f502, the 22-entry walk and the readback unchanged, no `points` array); `PointSamples.TheStandardChainIsUnchanged` (pin: no standard pass added or renamed); `PointSamples.APositionDependentProviderSeesEachPointsPosition` (CPU: body, element × k and sensor rows hold the position each provider sees, in the declared order); `PointSamples.ThePointsVariantFedTheWorldSampleReproducesTheStandardBits` (**Review Focus 2**, CPU and device); `GpuPointSamples.TheReaderKernelReadsThePerPointRowBehindTheVariant`; `PointSamples.ThePointsDeclarationIsCheckedByName` (k 0, a capacity with no position-dependent provider, a point declaration on a set with no reader are refused naming the module); `ModuleConfig.AFloatConfigHashesItsBitsAndTheSameInputsGiveTheSameBytes`; the identity of a set with a point provider differs from the standard one and from a set with the declaration but no provider.
- [ ] **Step 2: Run them red** (RED head built; quote each failure line).
- [ ] **Step 3: Implement** the declaration, the positions pass injection, the array registration, the second variant and recipe, and the `collect_set_one` row.
- [ ] **Step 4: Run focused, then the full CPU and `-L gpu`;** SpirvPin must not move for the standard rows (the new variant's rows are added); standard device digest `68eff17213201d8f`.
- [ ] **Step 5: `gcc-check --compile` on each changed .cpp, plain on headers; mutations** (positions pass omitted; variant chosen for a standard set; the array registered before the walk) each shown red and reverted; commit.

## Task D2 (T9): regions

**Files:** `engine/sim/module.hpp` (`FieldDecl` gains a host sample function; region declarations with priority), `engine/sim/module_schedule.cpp` (the one-provider rule at the `FieldOwners` map is rewritten: several providers of one field are allowed only when each is bound to a region of distinct priority, exactly one bound to the whole world; equal-priority overlap is refused naming both), `engine/sim/world_set.hpp/.cpp` (`WorldInstanceDesc` gains `regions`, empty = none; `config_hash` folds regions only when non-empty, so a world without regions hashes as today; `WorldConfig` carries a copy), `engine/sim/simulation.cpp` (`sample_medium` routed: outside every region today's Dryden answer; inside one, the region provider's host function on the CPU and `unavailable` naming the region on Vulkan, which has no device region row in D2). Tests: `tests/test_regions.cpp`; `tests/test_module_schedule.cpp` (`ModuleFields.TwoProvidersOfOneFieldAreRefused` changes).

- [ ] **Step 1: Write the failing tests:** `Regions.AWorldWithoutRegionsHashesAsToday` (pin); `Regions.EqualPriorityOverlapIsRefusedNamingBoth`; `Regions.PriorityResolvesTheHighestRegionPerPoint`; `Regions.SampleMediumIsRoutedAndRefusesOnVulkanInsideARegion`; `Regions.ACpuOnlyPositionDependentProviderNeedsNoCoreChange` (the SPH-shaped second consumer, design §6.2); **the refusal that survives the rewrite:** `ModuleFields.TwoProvidersOfOneFieldWithoutDistinctRegionsAreRefused` (renamed from `…TwoProvidersOfOneFieldAreRefused`; two providers of one field with no region, or both bound to the whole world, or two whole-world providers, are refused naming both modules), named in READY.
- [ ] **Step 2–5:** red; implement; focused; full CPU and `-L gpu`; `gcc-check`; mutations (priority ignored; overlap accepted; regions folded into an empty world's hash); commit.

**Carried into D (from C2):** a digest or replay of a set with a device-resident array syncs or refuses, never folds stale bytes: `Resident.AReplayOfASetWithAResidentArraySyncsOrRefuses`; audit every const caller of `state_digest`, `world_digest` and `arenas()`; the result is named in READY.

**As built so far (Core, 2026-11-09; gcc-check only, nothing built):** the internal module is `points` (pass `points.positions`, array `airflow_points`, `Extent::per_point`): the array carries the set-1 name because names are the contract, so the mirror binds it at 28. The L43 guard (the positions pass writes body rows only; element x k and sensor rows are the declaring module's) refuses at compile for k > 0 and at `create()` for sensor slots when no other pass declares a write of `points.airflow_points`. The float config-image test is dropped from D: D has no float config, so it waits for the first module that has one. The replay carry from C2 is `GpuResident.AReplayOfASetWithAResidentArraySyncsOrRefuses` plus `Resident.TheWorldDigestOnTheCpuIsOneNumberThroughEveryForm` (the plan's `Resident.` name takes the Gpu prefix so the gpu label attaches): `testing::world_digest(Simulation&)` syncs and `testing::checked_world_digest(const Simulation&)` refuses by name, as `state_digest` does. THE AUDIT of const callers of `state_digest`, `world_digest` and `arenas()`: in `engine/` the two const folds in `testing/replay.hpp` (they return a bare uint64_t and cannot refuse, so with a stale resident array they now STOP LOUDLY: one stderr line naming the array and pointing at `sync_resident()`, then `abort()`; Review Focus 4 allows no silent stale bytes; `GpuResidentDeathTest.AConstStateDigestOfAStaleArrayStopsLoudlyNamingIt` and `…AConstWorldDigestOf…`) and the viewer's `write_checkpoint`; no other `.arenas()` caller exists. `engine/tools/viewer/trajectory.cpp`, which the gcc leg does not build: the one change is `write_checkpoint(std::string&, const spade::Simulation&, …)` becoming `(std::string&, spade::Simulation&, …)` so its digests sync, and its one caller (`write_trajectory`'s loop, which holds a `spade::Simulation& sim`) is unchanged. The test helpers that take `const Simulation&` and read `arenas()` (`array_bytes`, `walk_bytes`, `find_registered`, `image_of`, `expect_same_rows` and the like) run on sets with no resident array, where nothing is ever stale. D2: `FieldDecl::region` + `sample`, `ModuleDesc::regions`, `WorldInstanceDesc::regions` (boxes); equal-priority overlap and unknown names are refused at `create()`; the compile rule is one whole-world provider per field plus region providers (a region provider has a sample and no pass). The still-forbidden case is named `Regions.TwoWholeWorldProvidersOfOneFieldAreStillRefusedNamingBoth` (the older `ModuleFields.TwoProvidersOfOneFieldAreRefused` still holds and is unchanged); the Vulkan refusal is `GpuRegions.SampleMediumRefusesOnVulkanInsideARegionNamingIt` (the plan's `Regions.SampleMediumIsRouted…` splits into the CPU routing test and this one, so the gpu label attaches).

**READY for D prints:** every D1 and D2 test name with their RED failure lines, the surviving refusal test, the SPIR-V rows added, the device digest before and after, and the consumers line.

# BATCH E — posed parts and loads (branch `core/module-api-stage7-e`; task level)

**Blocker: Physics' ψ pass and load formulas (design §5.3, §5.5), and its confirmation of the posed-part and vehicle-row layouts.** The part table's device mirror is buildable now (set 1, `airflow_parts`). Expanded at the cut.

- **Task E1 (T11): posed parts.** The part table (a configuration table rebuilt by `register_model()`, flattened like the world SDF) and its set-1 mirror; the vehicle rows (`per_body`, the model's part range and rotor slots, written by the module's vehicle hook at spawn; a model with more parts than the configured capacity is refused at spawn, naming the model and the limit — **Review Focus 5**); the azimuth row (`per_row`, owned by `rotors`, an integer phase — a `uint32` fraction of a turn); pose and raster kernels using B3's raster. Tests: `PostedParts.AMovingSphereAndASpinningBarMatchTheCpuTwin`; `PostedParts.APartPastTheCapacityIsRefusedAtSpawn`; `PostedParts.TheAzimuthWrapsExactlyOnBothBackends`; `PostedParts.ASetWithoutThePartTableIsUnchanged` (pin).
- **Task E2 (T12): loads.** Per-part fixed-order reductions (the two-level `kReduceBlockTiles` rule), per-body and per-rotor held-load rows (`per_body`, `per_row`; registered, in the readback), `apply_loads` with its apply switch off in `one_way`, the `BodyState::flags` bit for two-way coupling (allocated on the module's request; `drag.forces` skips a flagged body), and the per-rotor model row read optionally by `rotor.forces` behind switch bit 2 at set 1. Tests: `Loads.AnalyticPressureGivesTheAnalyticLoad` at 16³; `Loads.TheSumIsIndependentOfTheWorkgroupSize`; `Loads.OneWayAppliesNothing`; `Loads.AFlaggedBodyIsNotAlsoDragged`; `Loads.RotorForcesWithTheSwitchOffComputesTodaysBits` (pin; **Review Focus 2**); `Loads.AccumulationOrderIsPinnedByEdges`.

# BATCH F — publish and prose (branch `core/module-api-stage7-f`; task level)

**Blocker: Physics' publish kernels (`airflow_publish_*`, now in the spike) confirmed as the engine path's, and the `FlowVolume` band record (`TD-14`).** The CPU writer `publish_render_volume` exists.

- **Task F1 (T13): publish.** `request_publish(world, request)`, `publish()`, `flow_volume(world)`, `publish_points(world)`; its own command buffer (the on-demand mechanism of C3), submitted between steps and never inside the step recording (`L5` by construction); output buffers sized at `create()`; the fp16 pack in integer arithmetic; the CPU run calls `publish_render_volume`. Tests: `Publish.AStepRecordingIsUnchangedByAPublishRequest`; `GpuPublish.TheVolumeMatchesTheCpuWriterCellByCellWithinTheBand`; `Publish.HeaderMinMaxAndNoDataCountAreExact`; `Publish.SteppingCannotReadPublishOutput`.
- **Task F2 (T15): prose.** Comments, Core's docs (`01-modules-and-scheduler.md`, `02-state-and-snapshot.md`, `07-status.md`, `backlog.md`), `CHANGELOG.md`, and the `consumers.md` line (snapshot overloads, `sync_resident`, `memory_report`, `configuration_report`, new `BackendDesc` fields, `state_digest`'s signature, set 1 and its subset rule). `AnalysisValidation*`.

---

## Goldens and identities

| | Moves? |
|---|---|
| Every corpus golden; `kHeldQuadDigest` | No |
| Standard identity `0x3303cfc86821f502`, schema hash `0x1f60a6fef22e254a`, snapshot v3, the 22-entry walk | No |
| Every GPU parity band; the standard chain's dispatch count and order | No |
| `model_identity`, `parts_identity`, `kNoPartsIdentity` | No |
| Registry identity | Only for a set that declares a part table (decision 5; C1's table) |
| `PassParams` (48 B → +generic lanes), set 1 (28 → appended), `DeviceReport`, `BackendDesc`, `ArrayDecl`/`PassDecl`/`ModuleDesc` fields | Yes: wire contracts, generated or pinned checks |
| Set 0, the mirror's set-0 descriptor set, the standard kernels' pipeline layouts | No (two optional readers gain a set-1 subset, D1 and E2) |

Nothing here changes a law or a frozen identity, so nothing goes to Cameron, **unless** Question 1 is answered "fold for every set".

## gcc-check list

Per batch: every changed `.cpp` with `--compile`, headers plain, after a build has regenerated the headers (the release tree must be on the batch's head). Batch A: `engine/compute/admit.cpp`, `vulkan/state_mirror.cpp`, `vulkan/step_recorder.cpp`, `sim/simulation.cpp`, `tests/test_admit.cpp`, `tests/test_gpu_state_mirror.cpp`, `tests/test_memory_report.cpp`; headers `admit.hpp`, `layout_classes.hpp`, `backend.hpp`, `device_limits.hpp`.

## Questions for the lead

1. **The `parts_identity` fold's gate (decision 5).** Fold only for a set that declares a part table (recommended: nothing existing moves, and it matches `CORE-8`'s "once a step reads it"), or for every set holding a model with parts (a frozen registry identity would move for every such set; goes to Cameron)?
2. **Subset layouts (decision 1).** One declaration, per-kernel subset set-1 layouts (recommended), or the full set 1 wherever it appears?
3. **Appended set-1 names (decision 3).** Physics confirms the six proposed names and their order before the batch that first needs each.
4. **Staging of this plan.** A, B and C written to step level now; D, E, F to task level and expanded at their cut with their blockers in hand (recommended), or all six to step level now?

## Notes

- **Estimate:** about 23 task units *(estimate)* — A 4, B 5, C 5, D 3.5, E 3.5, F 2 — against the design's 20; the growth is the layout classes (B1), the audit and the per-batch gates. B0 removed about 1.5 units from A. Calendar time is set by the slots and the Physics blockers, not by this estimate.
- **The longer pole is still Physics' moving-wall numerics,** not Core's stage.
