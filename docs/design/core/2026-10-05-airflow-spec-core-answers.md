# Airflow spec: Core's answers

**Owner:** Core. **Status:** DRAFT for the lead. Read-only review; no repo file changed, nothing built.
**Reviews:** `docs/design/physics/plans/2026-10-05-airflow-design.md` at master `9308fe0`: §1.7, §4, §7, §10 (Core's six items), and §8 Q2, Q8, Q10.
**Code read:** master `9308fe0` (marked **m**); stage 4 on `core/module-state` at `c5c0c53` (marked **ms**), plus Task 7's uncommitted work in `spade-wt/core`; `PHY-7` on `physics/imu-contact` at `43587ee`. The spec's anchors are from `e3b0560`. Every anchor below was re-checked at the commit named.

## Verdicts

| Item | Verdict | Stage that carries it |
|---|---|---|
| 1. The six capabilities | **Amended.** Stage 4 already covers more of the grid extent than the spec says. The rest is a new Core stage after stages 5 and 6. The spec's list also misses six things Core must build | 4 (owes the scratch kind), 6 (fixed bindings), new stage 7 |
| 2. Per-element outputs of `rotor.forces` and `drag.forces` to scratch | **Confirmed, with four conditions** | The scratch kind: 4 (owed) or 7. The stores: Physics |
| 3. `sample_medium` on Vulkan inside a region | **Confirmed:** refused with `unavailable`, naming the region. **Amended:** today's function is hard-wired to Dryden, so region routing is new work | 7 |
| 4. Buffer packing and the `maxStorageBufferRange` refusal | **Confirmed. Amended:** three more device limits, 3D dispatch, and one LBM buffer | 7 (the limit checks may land earlier) |
| 5. The staging memory type | **Amended.** System memory on a discrete card, by inference from the code and the Vulkan spec, not measured. Scratch and derived buffers must lose their staging twins | 7 |
| 6. Snapshot growth; a v3 blob without airflow is unchanged | **Confirmed** for the unchanged blob (pinned in stage 4). **Amended** on growth: per-cell maps, every world pays, and a sync at snapshot | 4 (pins), 7 (sync) |
| Q2. The fluid step's cadence | **Confirmed C. Amended:** the second recording is Core's recorder change, in stage 7. Its cost is small. `j` and `n_V` must be set-wide | 7 |
| Q8. Order against SPH | **Amended.** Core is neutral and builds the shared work once, defined by capability. The order is Cameron's (`PHY-4` is signed) | 7 |
| Q10. Airflow on the CPU | **Confirmed A.** The spec keeps a CPU reference, so airflow can be graded. "Announced" must be a query, not a log line | 5 (grades), 7 |

## 1. The six capabilities

### What exists, what stages 5 and 6 add, what is new

| Capability | Stage 4 as built | Stage 5 or 6 must add | New, beyond the module-API plan |
|---|---|---|---|
| **Grid extent** | **Partly there.** `ArrayDecl` takes a plain `uint32_t elem_size` (`module.hpp:177-189` ms), and a module's descriptor may be built at run time (`module.hpp:395-396` ms: compiled names are owned). So a whole grid can be one `per_world` row, with `elem_size` = cells × 4 B. That row is registered, walked, snapshotted, digested and mirrored, with one `slot_to_world` entry per world. Limits: capacity is one value for every world (`array_capacity`, `simulation.cpp:251-273` ms); a row is at most 4 GiB; nothing hashes the configuration that sized it. The identity spells only names, versions and pass order (`module.hpp:514-521` ms). | Stage 6: a fixed binding for an optional built-in's array, with an empty buffer when the module is absent. Nothing in stage 5 | A hashed module configuration. Set-wide parameters (cells, `dx`, `j`, `n_V`, `ε` on or off, dye on or off, `n_trace`) ride the configuration identity. Per-world placement (box origin, mode, tracked body) rides `config_hash`, folded only when present, so other worlds hash as today. A named `grid` extent sized from it, with **no per-cell map**. Mixed set-wide parameters refused at `create()` |
| **Device-resident state** | **None.** The Vulkan step uploads the whole walk when dirty and reads all of it back after every `step(n)` (`simulation.cpp:996-1013` ms; `:881-896` m). One dirty flag covers everything, and any host write sets it: `set_rotor_commands` (`:2058` ms, `:2182` m), so a host flight loop re-uploads the whole walk every step. The arena is "the one authoritative CPU-side store" (`state_mirror.hpp:39-44` ms) | None | A residency flag per array, outside the per-step upload and readback. A sync before `snapshot()` (const today, `simulation.hpp:1035` m) and before any host read of a resident array: `arenas()` (`simulation.hpp:1244` m), `module_rows<T>`, and `testing::state_digest`, which reads `arenas()` directly (`testing/replay.hpp:143-154` m). Each one syncs or refuses (`L6`). An upload at `create()`, `restore()` and an explicit host write. Its own dirty flag. This changes the mirror's founding rule, so it needs a spec amendment |
| **Point samples** | **None.** One field row per world (`module.hpp:95-124` ms). On the GPU, one `FieldSampleRow` per world at binding 26 (`state_mirror.hpp:296-300` ms). Position-independent providers only (`core/07-status.md:19`) | None | Per-point rows. Module-API design §6 specifies points per body, per element and per world, but no stage plan schedules them. New beyond §6: k probes per element, points per sensor, and host requests for Publish points. The rotor, drag and Integrate kernels read per-point rows (Physics). Goldens hold, because Dryden's per-point values are copies |
| **Regions** | **None.** `compile_schedule` refuses a second provider of a field: "a field has one provider" (`module_schedule.cpp:396-401` ms) | None | Module-API design §11 specifies regions; no stage schedules them. Several providers of one field, each bound to a region. Priority resolved per point once per substep in Fields. The region row as state. The per-body "outside" flag. `sample_medium` routed by region |
| **Two recordings per step** | **One recording.** One command buffer holds every substep of a step. It is recorded once and resubmitted, and only the tick varies (`step_recorder.cpp:492-711`, `:719-760` m). A recipe can already differ by substep index at record time (`params.substep = s`, `:581-588`). `SubstepContext` has the tick (`schedule.hpp:226` ms) but no substep index | None | A second recording, chosen per submit by `tick mod j`. Barrier and timing checks for each recording. The substep index in `SubstepContext` for the CPU twin (trivial; Physics reviews `schedule.hpp`) |
| **VRAM budget check** | **Refusals by count only.** `buffer_bytes()` refuses more than 2^32−1 elements by name (`state_mirror.cpp:174-182` ms, `a346cda`). An allocation failure becomes `capacity_exceeded` (`:44-58`). Every entry is sized before any is created (`:255-267`), which is where a budget check fits. No device limit or budget is read anywhere in `engine/` | None | The check at `create()` (§4 below). The limit refusals (item 4) |

### What the spec's list misses

1. **The declared scratch kind.** The stage-4 note (`stage4-plan.md:704`) promised it, and §9 says "Stage 4 (with the scratch kind)". As built there is none: no extent or kind for it in `module.hpp` (ms), and Task 7's work adds none. `PHY-7` hand-registers `contact_dv` instead: a `Simulation` member, a mirror derived entry and binding 27 (`physics/imu-contact`, `simulation.hpp`, `state_mirror.cpp`, `bindings.slang`). Airflow's scratch also has a different invariant: "rebuilt before it is read", where `contact_dv` is "zero at every substep boundary".
2. **Derived buffers rebuilt after `create()` and `restore()`.** Apertures and the SDF distance depend on the region row, which a restore replaces. No hook rebuilds a derived buffer today. On the GPU a derived buffer is never uploaded or read back, so a kernel must run before the first step after a restore. The CPU needs the same hook.
3. **The fixed-binding pattern (stage 6).** Airflow is an optional built-in with kernels. Its bindings, and `rotor.forces`' optional probe read on the GPU, need stage 6's empty buffer and a recipe switch. §9 runs M3 before M5, which is before stage 6.
4. **A hashed module configuration** (row 1). Without it, two simulations with the same cell count but another `dx` or `j` share an identity and a schema hash, and a restore across them would be accepted (`L2`).
5. **Device limits and 3D dispatch** (item 4).
6. **Device-only buffers.** Today every walked and every derived buffer gets an equal staging buffer (item 5).

### Which stage

- **Stage 4** (in flight): as planned. Plus the scratch kind, if the lead agrees (decision 2 below).
- **Stage 5:** grades. Airflow declares CPU reference and Vulkan banded.
- **Stage 6:** the lock, and with it the fixed-binding pattern airflow reuses.
- **Stage 7, new: fields at points, regions and device residency.** Grid extent with a hashed configuration, device residency, derived rebuild, point samples, regions, two recordings, device-only buffers, 3D dispatch, the limit refusals and the budget check. It needs its own design section, approved by Cameron, because residency and the configuration hash change rules the approved module-API spec did not cover. The limit refusals and the budget check stand alone and may land earlier.

## 2. Per-element outputs to scratch, with no golden moving

**Confirmed, with four conditions.**

**What the digest folds today.** `testing::state_digest` folds the tick, then for every registered array, maps included: its name, element size, world count, capacity and raw bytes (`testing/replay.hpp:143-154` m). Snapshots serialize the same walk (`snapshot.cpp:493-507` m). Neither folds `field_rows_`, `scratch_`, `contact_dv_` or any mirror-derived buffer.

The conditions:
1. **The outputs are unregistered scratch.** A field added to `RotorRow` or `DragBodyRow` would change the element size, the schema hash and every golden.
2. **The CPU values and their fp32 order do not change.** Each pass stores a value it already computes. No expression is reassociated, and `drag.forces` still runs `after: rotor.forces`. On the GPU each kernel gains one store and one binding; `NoContraction` holds, so no band should move. Re-run `ctest -L gpu` to show it.
3. **No module version bump.** A bump changes the standard identity, and every stored blob is then refused (as stage 3's change was: `consumers.md:18`). The replayed state is identical, so no bump is due.
4. **The scratch kind exists first.** Without it, each output is another hand-registered buffer: a `Simulation` member, a mirror entry and a binding, as `contact_dv` is.

Two notes. On Vulkan the HUD cannot read scratch after `step(n)`: it is not read back, so the aero breakdown needs a Publish copy. The standard set's descriptor set grows by two bindings, so `bindings.slang`, `binding_for()` and `test_slang_layouts.cpp` move together (`state_mirror.hpp:32-37` ms).

## 3. `sample_medium` on Vulkan inside a region

**Confirmed:** on Vulkan, inside a region, `sample_medium` returns `unavailable`, naming the region and pointing to Publish points (`L6`). A hidden 40 MiB readback inside a point query would be a performance trap.

**Amended:**
- Today the function is hard-wired to Dryden: it builds a `DrydenMedium` from the world's row (`simulation.cpp:2717-2726` ms, `:2815-2824` m). It does not look up a provider. Routing by region is stage 7's.
- Outside every region it keeps today's answer on both backends, because the `dryden` row stays in the per-step readback.
- On the CPU backend it answers inside a region from host state, which stays authoritative there.

## 4. Buffer packing and the `maxStorageBufferRange` refusal

**Confirmed.** 128 MiB (2^27) is the Vulkan minimum. A storage binding wider than the device's limit is a valid-usage violation, not a reported error, so it must be checked in code. The house pattern exists twice: `buffer_bytes()` (`state_mirror.cpp:174-182` ms) and the recorder's group-count refusal (`step_recorder.cpp:306-320` m). The descriptor range is the buffer's full size today (`state_mirror.cpp:581` ms). The test template is `GpuStateMirrorTest.AShapeTheMirrorCannotSizeIsRefusedByName`.

**Amended:**
- **Three more limits, checked by name at `create()`.** `maxMemoryAllocationSize` (Vulkan 1.1 core): each buffer is its own allocation. The storage-buffer descriptor counts: Vulkan guarantees 4 per stage and 24 per set, and the set already declares 26 bindings (27 with `PHY-7`), unchecked. Batching matters here too: 16 worlds of one 128³ face component already exceed 128 MiB in one buffer.
- **3D dispatch.** Every dispatch is 1D today (`vkCmdDispatch(cmd_, c.groups, 1, 1)`, `step_recorder.cpp:694` m). Vulkan guarantees only 65,535 groups on X. At workgroup size 32, a 128³ cell grid needs 65,536 groups, and a face grid 66,048. Batched 64³ worlds pass the limit at any size. The recorder's check would refuse those shapes on a device at the minimum. Airflow's kernels need 3D dispatch, which is a recorder change.
- **Packing.** Yes. Velocity components, multigrid levels and scratch sets share buffers with offsets. The offsets are shape data, so they go in a small derived per-world buffer uploaded once at `create()`, like `grid_params`, not in the push constants.
- **LBM distributions (Core's call).** One 216 MiB buffer, refused by name on a device whose limit is lower. Splitting per device would make the layout, and so the kernels' indexing, depend on the device. 27 buffers would use 27 bindings. `maxStorageBufferRange` joins the device report. This is M6 work.
- **16-bit and 8-bit storage are optional features.** That covers fp16 apertures, the RGBA16F volume and u8 cell flags. Under `CORE-5` any Vulkan 1.1 device is admitted. So pack them into `uint32` words, or refuse by name. Making the features required would narrow `CORE-5`. The pack's rounding is a banded difference unless it is done in integer arithmetic.

## 5. The staging memory type

**Amended. Answered by the code and the Vulkan spec, not measured.**
- `find_memory_type()` returns the **first** type with `HOST_VISIBLE | HOST_COHERENT` (`state_mirror.cpp:60-70` ms). Vulkan orders a type whose flags are a strict subset of another's first. So on a discrete card, a plain host-visible type comes before a `DEVICE_LOCAL | HOST_VISIBLE` (BAR) type, and staging lands in system memory. On a unified-memory device both are one heap. This is unmeasured on driver 572.83.
- **Every walked array and every derived buffer gets an equal staging buffer** (`state_mirror.cpp:333-345` and `:368-388` ms). At 128³ that adds about 194 MiB of host-visible memory unless scratch and derived buffers become device-only. Only resident state (for snapshot syncs) and the Publish buffers need staging.
- **Core will:** record the chosen memory type and heap per buffer kind as a mirror diagnostic. Add a test that staging is not `DEVICE_LOCAL` on a discrete device. Add a device-only buffer kind in stage 7.

## 6. Snapshot growth, and a v3 blob without airflow

**Confirmed: a world set without airflow keeps its blob byte for byte.** Airflow is not in `standard_modules()`. Module arrays append after the standard walk in set order. The standard set keeps 22 entries and master's schema hash (`ModuleState.TheStandardWalkIsTodaysTwentyTwoEntries`, `ADevelopersArrayIsAppendedAfterTheStandardWalk`) and master's identity (`StandardModules.StateDeclarationsLeaveTheOrderAndTheIdentityAlone`), all on `core/module-state`.

**Amended on growth:**
- **Maps.** Every registered array gets a `slot_to_world` map of one `uint32_t` per element, in the walk, the snapshot and the mirror (`state_array_shapes`, `simulation.cpp:281-310` ms). Per-cell arrays would double the bytes: about 80 MiB per world at 128³, not 40. The grid row form (item 1) keeps it at about 40 MiB.
- **Every world pays.** The module set belongs to the simulation, and capacity is uniform. In a set with airflow, every world carries the largest region's arrays, with or without a region of its own.
- **Resident state needs a sync.** On Vulkan, `snapshot()` must first read the resident arrays back: about 40 MiB per world over PCIe, plus a copy out of staging.
- **Restorability depends on the standard identity.** The format holds. But if stage 7 adds or renames a standard pass, the standard identity changes, and stored blobs are refused once, as after stage 3.

Which of the spec's state is which:
- **Walked and snapshotted.** Velocity, pressure, dye, the region row and the accumulators. Velocity, pressure and dye are the device-resident part. The region row and the accumulators are small and stay in the per-step readback.
- **Scratch.** Advection buffers, multigrid levels, the force field and vorticity. Not walked, so a restored run restarts them. That is sound only if every fluid step writes each one fully before reading it, coarse-level corrections included. The Vulkan restore-resume test is what proves it.
- **Derived.** Apertures and the SDF distance. Not walked; rebuilt after `create()` and `restore()` from the configuration and the restored region row (item 1, miss 2).

## Q2. The fluid step's cadence

**Confirmed C** (every `j` steps, fixed at `create()`). D (adaptive CFL) would break the fixed recording.

**Amended:**
- **A and B need no second recording.** The recorder already knows the substep index at record time (`step_recorder.cpp:581-588` m). A recipe can record its dispatches at substep 0 only.
- **C with `j` ≥ 2 needs a second recording.** Recording A has the fluid step in substep 0's Fields; recording B has none. The submit loop picks one per step by `tick mod j` (`step_recorder.cpp:719-760` m). The choice is a pure function of the tick, so a restore lands on the same cadence.
- **Its cost is small.** A second command buffer, recorded once at `create()`. No per-step cost. The `CORE-2` barrier test and `vulkan_recorded_chain()` cover both recordings. `vulkan_pass_durations_ns()` reports only the most recent step (`backend.hpp:282` ms), so it must say which recording ran.
- **The alternative costs more.** A data-driven skip records the fluid step once per step, in substep 0. It costs about 115 empty dispatches and barriers on every skipped step, **not "per substep"** as §1.7 says. At the spec's 4 µs each that is about 0.46 ms, about a quarter of a 2 ms step at `j` = 2.
- **Set-wide parameters.** One recording serves every world, so anything that changes the recorded dispatch list must be one value for the whole set: `j`, `n_V`, the level count, confinement on or off, dye on or off, and any per-segment backtrace dispatch. `create()` refuses mixed values, as `dt` and the substep count are fixed for the whole set (`L1`). Per-world `j` would need a per-world data-driven skip.
- **CPU twin.** It needs the substep index in `SubstepContext`. The CPU pass runs every substep and acts only when `substep == 0` and `tick mod j == 0`.
- **Stage:** 7. It is a recorder change and Core's.

## Q8. Order against SPH

**Amended. Core is neutral on the order; the order is Cameron's.** `PHY-4` is signed, and SPH closes the last v1 transfer row, so putting airflow first delays the v1 quarantine (`SL14a`).

- **Shared:** point samples, regions, and the scratch kind. Core builds them once either way.
- **Not shared.** Airflow alone needs device residency, the grid extent, two recordings, 3D dispatch and the budget check. SPH alone needs per-body rows (stage 4 has them), contact bits (module-API design §11) and stage 5's `absent` grade: under `PHY-4` it is "the first field provider that is not a built-in", so it has no GPU path.
- **The amendment.** Stage 7 is defined by capability, not by its first consumer. It is tested with a CPU-only, position-dependent test provider, so whichever consumer comes second needs no Core change.

## Q10. Airflow on the CPU

**Confirmed A.** The spec keeps a CPU implementation of both tiers at 16³–32³, with a golden at 32³ (§1.1, §4.6). So airflow has a golden and a band, and it can ship graded. It is the same code at any size, so a larger CPU grid is still a reference run, only slower. Core would refuse a GPU-only airflow unless it declared `absent` on the CPU, a lower grade that needs stage 5. The spec proposes none.

**Amended:**
- **"Announced" means a query,** such as a cost estimate beside `vulkan_device_report()`. It is not a library log line (banded-parity plan, risk 3). Strictly, `L6` does not require it: slow is not unavailable. Refusing large CPU grids (option C) would be an arbitrary limit.
- **What verifies the GPU at 128³** is Q9's record: per device, statistics against a CPU run at 128³. The CPU run is the source of comparison, never the GPU (`L4`).

## Factual errors in §1.7, §4 and §7

**§1.7**
1. **Two stage claims contradict §9 and §10.** It says the bold rows are "a Core stage after stage 6". §9 puts that work in M2, before M5 (stages 5 and 6). Its bold rows are scratch, grid extent, residency, point samples, regions and two recordings. §10's six swap scratch for the budget check.
2. **"SPH needs the first two as well."** The first two bold rows are scratch and the grid extent. SPH needs point samples and regions, as Q8 says.
3. **Module-declared arrays and the `per_row` extent are not "Planned"; they are built.** Task 1 merged (`eb57d4d`). Tasks 2 to 4 are on `core/module-state` (`d2ff26d`, `a346cda`, `bd0f640`). A `per_row` owner may belong to another module (`module.hpp:181` ms), and the despawn cascade clears the children.
4. **The optional read is built on the CPU only** (`b40d0ad`), and it binds arrays, plus tables in Task 7, not field samples. Built-in passes do not read `ctx.state` (`stage4-plan.md:700`). The GPU half is stage 6's pattern.
5. **The scratch kind is agreed, not built** (item 1, miss 1).
6. **The grid extent is not "not planned" in mechanism.** A `per_world` row sized from configuration works today. What is missing is the hashed configuration.
7. **The data-driven skip costs about 115 dispatches per skipped step, not per substep** (Q2).
8. **The readback happens once per `step(n)` call, not per step,** and every host write re-uploads the whole walk. With per-cell maps the grid would move about 80 MiB, not 40.

**§4**

9. **§4.1: no check exists yet.** "Checked against the device and refused by name" is to be built. The spec omits `maxMemoryAllocationSize`, the descriptor counts and the 1D dispatch limit (item 4).
10. **§4.2: the timing API is coarser than the spec assumes.** `vulkan_pass_durations_ns()` gives one number per pass, summed over the substeps of the most recent step. With `j` > 1 the fluid passes read about 0 after a non-fluid step. Per-V-cycle numbers come from the spike, not this API.
11. **§4.3: the memory table counts device-local bytes only.** It leaves out the staging twins (about 194 MiB of host-visible memory), per-cell maps if used, and the host arena's own copy of the state.
12. **§4.3: fp16, u8 and RGBA16F storage need optional device features** (item 4).
13. **§4.3: "each world owns its region" is not what the arenas do.** Capacity is uniform, so every world pays the largest region, and a world with no region still pays when airflow is in the set.
14. **§4.4: `sample_medium` does not call "the provider's CPU function".** It is hard-wired to Dryden (item 3).
15. **§4.4: `state_digest()` is not a `Simulation` call.** It is `testing::state_digest`, which reads `arenas()` directly. The sync must sit in `Simulation`, on every host read path. `snapshot()` is const.
16. **§4.4 contradicts §5 on Publish.** It puts "the Publish buffers" in the per-step readback; §5 runs `airflow.publish` "only on frames that request it". A recorded chain cannot see a frame request. Core proposes the volume as its own command buffer, submitted on request between steps. Then the step's recording does not change, and `L5` holds by construction.
17. **§4.5: only the denormal flush falls under the absolute term.** NVIDIA's division and square root are 1 ulp off on **normal** operands (report finding 4). That falls under the relative term, everywhere, and chaos amplifies it. A cheap mitigation: compute reciprocals such as `1/dx` and `h_f/ρ` on the host at `create()`, and multiply on both backends.
18. **§4.5: T7's Vulkan restore-resume test is not built.** No branch has `VulkanSnapshotRestoresIntoVulkanBitIdentically`. Airflow's version must hold the resident arrays and rebuild the derived buffers.

**§7**

19. **"A world without it registers nothing" is about the wrong unit.** The module set belongs to the simulation and is shared by every world (module-API design §3, `L8`). A simulation whose set lacks airflow registers nothing.
20. **"Configuration identity unchanged" holds for stage 4, not for stage 7.** If stage 7 adds or renames a standard pass, the identity changes, and Kat's stored blobs are refused once. Goldens hold. Core will try to avoid it, but cannot promise it yet.
21. **"Under world file v3" names something not built.** The world file is schema v2 (`core/07-status.md:17`). The region's hashed home is new Core work (item 1).
22. **"Announced at `create()`" must be a query** (Q10).
23. **"Snapshots grow by about 40 MiB per world"** applies to every world in the set. It doubles with per-cell maps. On Vulkan it adds a readback at each snapshot.
24. **"Airflow is a GPU feature" is misleading.** Real time needs the GPU, but the CPU path is the reference (`L4`).
25. **"Rotors and drag are unchanged without airflow, bit for bit" holds on the CPU only.** On the GPU it needs stage 6's empty binding and a recipe switch.

## Outside Core's sections (for the owners)

- §9 M1 runs the spike "in the measurement tree". T5 retires that build (`test-docs/retire-measurement-build`, `86f1cb6`). Since T4 the default build admits the card, so the spike can run there.
- §0's Core anchors (`simulation.cpp:881-896`, `state_mirror.hpp:1-10`, `step_recorder.cpp:492-497`, `simulation.hpp:696`) still hold on master `9308fe0`. On `core/module-state` the step path is at `simulation.cpp:981-1015`.

## For the lead and Cameron

1. **Where stage 7 sits.** Core recommends 4 → 5 → 6 → 7: airflow on Vulkan needs stage 6's fixed bindings and stage 5's grades. §9 puts the Core work (M2) before stages 5 and 6 (M5). Keeping §9's order means stage 7 builds the fixed-binding pattern itself, and stage 6 reuses it. This decides how soon "GPU asap" arrives.
2. **The scratch kind in stage 4.** It is not built, though the stage-4 note and §9 assume it. Core's lean: add it to stage 4 before merge, per body and per world, so `PHY-7` and item 2 declare their scratch instead of hand-registering it. The alternative: merge as is, and add it in stage 7.
3. **A spec amendment for Cameron.** Device residency ends the arena's role as the one authoritative store. A hashed module configuration changes what the identity and `config_hash` cover. Neither is in the approved module-API spec. Q8 also needs Cameron's word, because it changes `PHY-4`'s timing.
