# Module API stage 7: the airflow module's GPU groundwork — design

**Owner:** Core. **Status:** SIGNED by the user 2026-10-12 (via lead; `CORE-8`), with D1-D7 at the recommendations. See "Rulings received" at the end.
**Builds:** module-API spec §15 (`plans/2026-10-02-module-api-design.md:186-192`), approved as principles 2026-10-05. **Rests on:** Cameron's airflow rulings (`PHY-9`, 2026-10-05) and his demo rulings of 2026-10-12 (`backlog.md:66`: two-step path; disc, actuator line and blade-resolved rotors; the drone a solid wall, porous optional).
**Inputs:** the airflow spec (`../physics/plans/2026-10-05-airflow-design.md`); Core's answers (`2026-10-05-airflow-spec-core-answers.md`); Core's demo inputs (`2026-10-12-airflow-demo-core-inputs.md`); the stage 4 and 5 plans; the per-part model geometry note (drafted in parallel; its working shape is assumed below, not redesigned).
**Anchors:** **m** = master `80d564b`; **mg** = `core/module-grades` `0133ef3` (stage 5 Task 1); **pa** = `physics/airflow-core` `a80420f` (plan A Task 6). Estimates are marked *(estimate)*; facts not checked in the code are marked *(unconfirmed)*.

## Summary

- Stage 7 makes a module that owns a 3D grid a first-class citizen of `Simulation`, on both backends: its grid lives on the device, its configuration is hashed, it steps at its own cadence, it answers fields at points by region, it sees the vehicles' parts as moving solid walls, it pushes loads back into the bodies, and it publishes a render volume on request.
- Core builds the plumbing, the data flow, residency and determinism. Physics owns every numeric choice: the solver, the immersed-boundary formulas, the load integrals, the rotor models. The airflow module itself is Physics' plan C, after this stage.
- Stage 7 is defined by capability, not by its first consumer. It is tested with test modules and a small set of domain-neutral grid kernels (posed-SDF rasterization, fixed-order sums, integer-shift copy, point interpolation, fp16 packing).
- The standard set's identity, schema hash, walk, goldens and bands do not move.
- About 15 tasks: about 2.5 to 4 working days of Core effort *(estimate)*, after stages 5 and 6. Four tasks stand alone and may start earlier.
- Six decisions for Cameron (§11). The largest: the digest policy for device-resident state, and whether blade-resolved rotors' plumbing belongs in this stage.

## 1. Goal and done-when

**Goal.** At the end of stage 7, the airflow module (plan C) can be written against stable engine API, with no further Core change, for all three rotor models and both surface modes Cameron named.

**Done when:**
1. A configured module declares device-resident arrays. They register, snapshot and mirror like any array. On Vulkan they stay out of the per-step upload and readback.
2. Every host read path of a resident array syncs from the device or refuses by name (`L6`, §15). A Vulkan snapshot, restore and resume of a resident array is bit-identical to the uninterrupted Vulkan run on the same device.
3. A module's configuration is spelt into the configuration identity. A restore under another configuration is refused (`L2`). The standard identity stays `0x3303cfc86821f502`.
4. A pass can run every `j`-th step. Vulkan picks one of two recordings per step by `tick mod j`; the CPU scheduler skips the pass by the same rule. The `CORE-2` barrier test passes on both recordings.
5. Dispatches are 3D. The device limits are read and checked at `create()`, by name: storage-buffer range, allocation size, descriptor counts, group counts on each axis, and a memory budget. Scratch and derived buffers have no staging twin.
6. Field samples are taken at points (per body, per element with k probes, per sensor, and host requests). Regions with priorities bind providers. `sample_medium` is routed by region.
7. Vehicle parts are posed every fluid step and rasterized into a solid field (distance, owner part, wall velocity), on both backends, from body, rotor and azimuth state. Loads are reduced per part in a fixed order and returned to bodies and rotors.
8. A publish recording, submitted on request between steps, fills `FlowVolume` on the GPU, banded against the CPU writer.
9. Every golden, the standard identity, the one-body schema hash (`0x1f60a6fef22e254a`), the 22-entry walk and every parity band are unchanged at the stage's cut.

**What stays out:**

| Out of stage 7 | Whose |
|---|---|
| The solver: advection, projection, multigrid, viscosity, WALE, the wall model, LBM | Physics (plans A, B, later) |
| How a distance becomes an aperture, how a moving wall enters the projection, the load integrals, the porous resistance law | Physics |
| The actuator disc, actuator line and blade-element formulas; which rotor quantity drives the shaft | Physics |
| The airflow module itself (`airflow` passes, config struct, plan C) | Physics, on this stage's API |
| Per-part model geometry: `ModelType` parts, their YAML, CAD input | Core, the parallel note |
| World file v3 (regions and module configuration in the file) | Core, its own plan; this stage reserves the shape |
| The volume views, surface colouring | Rendering |
| The editor's airflow panel, the drone builder's CFD front end | Interface |
| Refinement (R1 nests, R2 adaptive grids), nested LBM regions | Later (spec §11.4, M6, M7) |
| A GPU module ABI for developer kernels | Later (module-API spec §5) |
| Adaptive CFL | Refused: it breaks the fixed recording (spec Q2) |

## 2. Device-resident module state

### 2.1 Declaration

**A residency flag on `ArrayDecl`, not a new declaration kind.** A resident array is otherwise an ordinary module array: named, walked, snapshotted, digested and mirrored (`module.hpp:178-190` m).

```cpp
// sim/module.hpp
enum class Residency : uint8_t { host = 0, device = 1 };
// ArrayDecl gains:  Residency residency = Residency::host;
```

- **Extent.** `device` is legal on a `per_world` array only. That array's one row is the whole grid, sized from the module's configuration (§3), with one `slot_to_world` entry per world, not one per cell (answers doc, row 1). Small state (the region row, the impulse accumulators, held loads, azimuths) stays host-resident and rides the per-step readback.
- **Size.** `elem_size` is `uint32_t`, so a resident row holds at most 4 GiB per world. Larger is refused at `compile_schedule`. Device limits refuse earlier (§7).
- **Grade.** A module with a resident array must declare CPU `reference` (§15). `compile_schedule` refuses it otherwise, naming the module and the array. This joins stage 5's rule that Vulkan `banded` needs a CPU `reference`.
- **No new extent.** The answers doc floated a named `grid` extent. A `per_world` row sized from the hashed configuration does the same with less: one invariant, one site (`TD-9`).

### 2.2 Lifetime

| Event | CPU backend | Vulkan backend |
|---|---|---|
| `create()` | The arena row, zero-filled; the module's row `init` may write initial values | The same host row, then uploaded once before the first step |
| `step(n)` | Passes read and write the arena | The device copy is authoritative. Not uploaded, not read back. The host copy goes stale |
| A host write to a resident array | Writes the arena | Syncs first if stale, writes the host copy, marks it for upload before the next step |
| `snapshot()` | As today | Syncs every stale resident array, then saves (§2.4) |
| `restore()` | As today | Writes the host copy, marks every resident array for upload, queues the derived rebuild (§2.6) |

**A small ledger per resident array:** `in_sync`, `host_newer` (upload before the next step), `device_newer` (sync before a host read). It sits beside today's single `vulkan_dirty_` (`simulation.hpp:1912` m), which then covers only host-resident arrays. So a host flight loop that calls `set_rotor_commands` every step no longer re-uploads the grid (today it re-uploads the whole walk: `simulation.cpp:1125-1135` m).

**The mirror's founding rule changes**, as §15 approved. Today the arena is "the one authoritative CPU-side store" (`state_mirror.hpp:39-45` m), and `upload()`/`readback()` move every walk entry or fail (`state_mirror.cpp:790-880` m). In stage 7 each mirror entry carries its residency. The per-step pair skips resident entries; two new calls move only resident entries, on demand.

**Staging.** Resident arrays share **one** staging buffer, sized at `create()` to the largest resident row, and are synced one after another. So host-visible memory grows by one grid row, not by the sum of them (today every entry has an equal staging twin, `state_mirror.cpp:464-476` m).

### 2.3 Host read paths: sync or refuse

| Path | Today | Stage 7 on Vulkan with a stale resident array |
|---|---|---|
| `snapshot()` (`simulation.hpp:1175` m, `const`) | Saves the arena walk | A non-const overload syncs, then saves. The `const` one refuses (`unavailable`), naming the array and `sync_resident()` |
| `module_rows<T>()` (`simulation.hpp:1002` m, `const`) | Arena rows | The same split: the non-const overload syncs; the `const` one refuses, naming the array |
| `testing::state_digest` (`testing/replay.hpp:143-154` m) | Folds `arenas()` directly | Takes `Simulation&` and syncs. A call through a `const` reference stops compiling, so the build finds every site (the plan counts them first) |
| `sample_medium` (`simulation.cpp:2953-2961` m) | Hard-wired to Dryden | §6: outside every region, today's answer; inside one, `unavailable` naming the region (agreed in the answers doc) |
| `arenas()` (`simulation.hpp:1382` m) | A raw `const ArenaSet&` | **Cannot refuse**: it returns a reference. It stays the raw view, documented: a resident array's bytes are those of the last sync. `resident_current()` says whether they are current. Every engine reader that folds bytes (save, digest) goes through a synced path |
| New: `sync_resident()` | — | Syncs every stale resident array. A no-op on the CPU and when in sync |

**On the CPU backend nothing changes.** A resident-declared array is an ordinary host array there: authoritative, digested and snapshotted as today, with no sync and no refusal. The CPU stays the reference (`L3`, `L4`), and its goldens include the grid.

### 2.4 Digest and snapshot

- **Digest (decision D1).** Recommended: a resident array is folded into `state_digest` after a sync, like any array. One digest; the CPU golden covers the grid; two Vulkan runs compared for determinism cover it too. The cost is one readback per digest call on Vulkan, in tests only: about 40 MiB per world at 128³ *(estimate)*.
- **Snapshot.** Format v3 is unchanged (`snapshot.hpp:114-155` m). A resident array is one more section. A blob from a set without airflow is byte-identical, as stage 4 pinned (answers doc §6).
- **Growth.** About 40 MiB per world at 128³ (velocity, pressure, dye), and every world in the set pays, because capacity is uniform (answers doc §6). On Vulkan each snapshot adds a readback of that much.
- **Cross-backend.** A CPU blob restores into Vulkan and resumes within the band; a Vulkan blob restores into the CPU likewise. Neither is bit-identical (stage 5's meaning of `banded`).

### 2.5 Scratch

Stage 4 Task 7b's scratch (`module.hpp:366-423` m) already fits the grid's scratch (advection sets, multigrid levels, the force field, the solid field of §5): not walked, zero-filled at `create()`, a fixed binding each. Two changes:
- **A second invariant, named.** `contact_dv`'s is "zero at every substep boundary". The grid's is "every reader's rows are written earlier in the same fluid step". Both make `restore()` need nothing. The `ScratchDecl` comment names both, and the module states which it keeps.
- **No staging twin** (§7.3). Scratch is never uploaded or read back.

### 2.6 Derived buffers and the rebuild hook

Static apertures and the static SDF distance are a pure function of the configuration and the region row, which `restore()` replaces (answers doc, miss 2). No hook rebuilds a derived buffer today.

```cpp
// ModuleDesc gains:  PassFn rebuild_cpu = nullptr;  compute::GpuRecipe rebuild_gpu = compute::GpuRecipe::none;
```

`create()` and `restore()` run the rebuild before the next step: on the CPU directly, on Vulkan as an **on-demand recording**, submitted once. The publish recording (§6.4) uses the same on-demand mechanism. In following mode the module also rewrites shifted-in cells in-step, at a recentre (spec §1.5 step 1); that is its own kernel.

## 3. Hashed module configuration

### 3.1 What is hashed, and where

| Configuration | Scope | Where it is folded | Folded when |
|---|---|---|---|
| Grid extent `nx, ny, nz`, cell size `dx`, tier (NS or LBM), lattice and `m` for LBM | Set-wide | Configuration identity (snapshot header) | The module has a configuration |
| Cadence `j`, fluid sub-steps `m_f`, `n_V`, multigrid levels, `n_trace`, `u_design`, `ω_design` | Set-wide | Identity | Same |
| Confinement `ε`, dye, viscous mode, subgrid and wall models, domain face kinds | Set-wide | Identity | Same |
| Rotor model kind (disc, line, blade-resolved), its point counts, surface mode (wall or porous) and porous coefficients, coupling mode (one-way, two-way), part capacity | Set-wide | Identity | Same |
| Box origin cell, fixed or following, the tracked body, region priority | Per world | `config_hash` (in `replay_config`, `simulation.hpp:470-475` m) | Only when the world declares a region, so every other world hashes as today |

- **The bytes.** `ModuleDesc` gains `std::span<const std::byte> config`: the module's configuration struct as a canonical image (trivially copyable, no implicit padding, asserted field by field as `ReplayConfig` is). The same bytes are uploaded as the module's configuration buffer, so what is hashed is exactly what the kernels read (`TD-9`). Values derived from it, such as `1/dx` and `h_f/ρ` (answers doc, item 17), are computed on the host at `create()` and uploaded beside it. They are not hashed: they are a function of hashed bytes and the module version.
- **The spelling.** Today the identity spells each module's name, `0x00` and its version, then `0x01` and the passes (`module.hpp:605-615` m). A module with a non-empty configuration appends, after its version, `0x02`, the image's length (4 bytes, little-endian) and the image. A module with none appends nothing. **So the standard set's identity does not move**, and the existing pin (`StateDeclarationsLeaveTheOrderAndTheIdentityAlone`) proves it.
- **Mixed set-wide values.** The module set is the simulation's (`L8`), so a set-wide value has one home, the module's configuration. When world file v3 lets a region spell `dx` or `j`, `create()` refuses a world that disagrees with the module, naming both values.
- **Geometry.** Once a step reads vehicle parts (§5), every part field a pass reads must fold into `model_identity`, only when non-empty, so every existing identity and stored blob holds (demo inputs §1b). The model-registry identity in the header then covers geometry (`snapshot.hpp:149-154` m). This belongs to the per-part note; stage 7 depends on it.

### 3.2 The refusal on restore

`restore()` already refuses a header whose configuration identity differs (`L2`). A blob from another grid is refused there. The schema hash also refuses a different cell count, since `elem_size` follows it; the identity is what catches the same cell count with another `dx`, `j` or `n_V` (answers doc, miss 4).

The message can name only the two identities: the 48-byte header holds no per-module detail. Stage 7 adds `configuration_report()`, listing each module's name, version and configuration hash, so a caller holding both can say which module differs. No format change.

## 4. The fluid cadence

### 4.1 Two cadences

| Cadence | When the fluid runs | For | Recorder |
|---|---|---|---|
| Every `j`-th step (`j ≥ 1`) | Fields of substep 0 when `tick mod j == 0` | The real-time tier, actuator disc (spec §1.4) | Two recordings when `j ≥ 2` |
| `m_f` fluid sub-steps per substep (`m_f ≥ 1`) | Every substep, `m_f` times in a row | Actuator line and blade-resolved, where the blade must move a small angle per fluid step; LBM's `m` | One recording; the module's recipe unrolls `m_f` |

Only one of `j` and `m_f` may exceed 1. **Why the second cadence:** at `h_f` = 4 ms and ω ≈ 700 rad/s *(unconfirmed for the sandbox quad)*, a blade turns about 2.8 rad per fluid step. A blade-resolved or line model needs a few degrees at most. `m_f` is fixed at `create()` from a declared `ω_design`, as `n_trace` is from `u_design` (spec §1.4); a run above `ω_design` is announced by a monitor, not refused.

### 4.2 Recorder and scheduler changes

- **`PassDecl` gains `every_steps` (default 1).** A pass with `every_steps > 1` runs in substep 0 only. Every such pass in the set must share one `j`; `compile_schedule` refuses two values (a second `j` would need more recordings).
- **One rule, two sites that read it.** `due(tick, substep)` lives in one header. The CPU scheduler calls it and skips the pass, so the module's CPU function never tests the tick. The recorder calls it at record time to decide which recording holds the pass.
- **`SubstepContext` gains the substep index**, for modules that need it (answers doc, row 5; Physics reviews `physics/schedule.hpp`).
- **Recordings.** `StepRecorder` records once today and resubmits (`step_recorder.cpp:492-711`, `:719-758` m). It gains a second command buffer, recorded once at `create()`. The submit loop picks one per step by `(first_tick + i) mod j`. The choice is a function of the tick only, so a restore lands on the same cadence.
- **Recipes become recording functions.** Today a recipe is an enumerator in one switch (`backend.hpp:273-287`, `step_recorder.cpp:590-666` m). A built-in module's recipe becomes a function that appends dispatches to an emitter, from the shape and the module's configuration (the second half of spec §5: "asks each pass with a GPU recording to append its dispatches"). The fluid step's `n_V` cycles, levels and `m_f` sub-steps are unrolled there, fixed at `create()`.
- **Push constants.** `PassParams` gains a few generic lanes (level, colour, sweep, sub-step). Everything else comes from the configuration buffer. Push constants stay within the 128-byte guaranteed minimum.
- **Pipelines on demand.** Today all twelve pipelines are always built (`step_recorder.cpp:350-377` m). With airflow's 20 or so kernels, only recipes the schedule names are built.
- **Timing.** `vulkan_pass_durations_ns()` reports the latest step (`backend.hpp:329-380` m). It gains which recording ran and the tick, so a fluid pass reading about 0 after a non-fluid step is not mistaken for a measurement (answers doc, item 10). Each recording has its own query pool.
- **Checks.** `RecordedChain` is reported per recording. The `CORE-2` barrier test and the recorded-order test run on both.

### 4.3 Cost

- A second command buffer and query pool, recorded once at `create()`. No per-step cost.
- The rejected alternative, a data-driven skip, costs about 115 empty dispatches and barriers per skipped step, about 0.46 ms *(estimate)*, a quarter of a 2 ms step at `j` = 2 (answers doc, Q2).
- Each step still submits and waits on one fence (`step_recorder.cpp:735-754` m). Batching several steps per submit is a separate measurement, not this stage's.

## 5. Moving immersed solids

### 5.1 The working shape assumed

From the per-part note: each `ModelType` holds parts `{name, role, attachment, mount, geometry, surface}`. Attachment is rigid to the body, or spinning with rotor k. The mount is a transform in the BODY frame (the hub frame for a spinning part); authors work in the design frame through `AirframeSpec` (approved note, `CORE-7`). Geometry is an exact `SdfProgram` in the part frame; triangle meshes follow (`CORE-6`). Surface is a wall, or porous: `inv_permeability` (1/K, 1/m²) and `inertial_coeff` (C₂, 1/m), both finite and ≥ 0 (Physics, 2026-10-12). Blade-resolved props need a per-rotor azimuth `ψ`, which `RotorRow` lacks (`rotor.hpp:535-549` m: 64 bytes, no azimuth).

### 5.2 How the grid sees each part, each fluid step

Three steps, in Fields, inside the fluid step's recipe (so they repeat `m_f` times when sub-stepping):

| Step | What it computes | Reads | Writes (scratch) |
|---|---|---|---|
| **Pose** | Each live part's world pose, the velocity of its frame, and its bounding box in grid cells | `bodies` (pose, `vel`, `omega_body`), `rotors` (`local_pos`, `local_orient`, `omega`, `spin_dir`), the azimuth rows, the part table, the vehicle rows | The posed-part table |
| **Raster** | Per face and cell in the union of the boxes: the signed distance `φ`, the owner part, the wall velocity, the porous part if any | Posed parts, the part SDF programs | The solid field |
| **Fluid** | Physics' step, reading the solid field and the static (world SDF) derived field | Velocity, pressure, solid field, sources | Velocity, pressure |

- **The pose.** A rigid part is posed by `X_body ∘ M_p`: mounts are in the body frame, so the design-to-principal transform was applied once, when the model type was built (`CORE-7`, `DBP-45`). A spinning part adds the hub and `R(axis, ψ_k)` before its own mount.
- **The wall velocity** at a point `x` is `v + ω × (x − c)` for a rigid part. A spinning part adds `spin_dir · ω_k · axis × (x − hub)`. Each face stores the component along its own axis.
- **The raster** evaluates `eval(program_p, X_p⁻¹ x)`, the same evaluator `sdf_eval.slang` already mirrors for the world SDF (bindings 14–16). Each face thread loops over parts in a fixed order (world, body slot, part index) and keeps the smallest `φ`. **A tie goes to the lower part index.** It is a gather: no atomics (as the actuator gather, spec §3.1).
- **What Core writes, and what Physics writes.** Core's raster writes `φ`, the owner and the wall velocity. Physics turns them into apertures (`θ = clamp(0.5 + φ/dx, 0, 1)` today, spec §2) and imposes the wall in the projection. Static world solids stay derived (§2.6); the fluid combines the two.
- **Memory** at 128³ *(estimate)*: `φ`, the owner and the wall velocity at faces, 4 bytes each, about 72 MiB of scratch; about 48 MiB if `φ` and the owner pack into one word in integer arithmetic. The spec counted only +24 MiB for moving solids (§4.3); the plan re-states the table with Physics.

### 5.3 Rigid parts and spinning props

| | Rigid part (body, arm, motor, landing gear) | Spinning part (a blade) |
|---|---|---|
| Pose from | The body | The body, the rotor's hub and `ψ_k` |
| Wall velocity | `v + ω × r` | `v + ω × r` plus the blade's spin |
| State read | `bodies` | `bodies`, `rotors`, the azimuth row |
| Changes each fluid step | Only as the body moves | Always, at `ω_k` |

**The azimuth `ψ`.** It cannot join `RotorRow`: a new field changes the element size, the schema hash and every golden (answers doc §2, condition 1). It is a `per_row` array owned by `rotors`, depth 1, declared by the module that needs it, so a set without that module registers nothing. It is registered state, small and host-resident, because a snapshot can fall between fluid steps (`L2`). One pass writes it (`TD-9`), after `rotor.forces` updates `ω`. The integration and the wrap are Physics'; Core's note: an integer phase (a `uint32` fraction of a turn) wraps exactly on both backends, where an fp32 angle wraps with rounding.

**The vehicle rows.** At spawn, the module's vehicle hook (stage 4 Task 7, `module.hpp:324-364` m) writes a `per_body` row with the model's part range in the part table and the vehicle's rotor slots. The posed table is indexed by body slot and part index, up to a configured maximum per vehicle. A model with more parts, or a world past the configured part capacity, is refused at spawn, naming the model and the limit (`L6`).

**The part table on the device** is a configuration table (stage 4 Task 7), rebuilt by `register_model()`, flattened like the world SDF (nodes, transforms, ranges; `sdf_program.hpp`). Its device mirror waits for stage 6's fixed-binding pattern (`module.hpp:310-313` m).

### 5.4 Walls and porous parts

| | Wall (Cameron's default) | Porous (the option) |
|---|---|---|
| The grid sees | Closed and partly closed faces, at the wall's velocity | Open faces, plus a momentum sink in the cells inside, against the part's velocity |
| Raster output used | `φ`, owner, wall velocity | The porous part id per cell, and its coefficients from the part table |
| Loads | Pressure and viscous traction over the cut faces | The sink's reaction, summed over its cells |
| The formulas | Physics | Physics (a Darcy–Forchheimer form is the usual one) |

**No double count.** A body that the grid resolves as a two-way wall must not also be dragged by its drag element (spec §2). A blade-resolved or line rotor must not also add momentum theory's thrust. Core's mechanism, not a formula:
- a named bit in `BodyState::flags`, allocated on the module's request (module-API spec §11; `layout.hpp:127` m, none assigned yet), set by the airflow module's vehicle row `init` ONLY when it couples two-way; `drag.forces` skips elements on a flagged body. The COUPLING MODE decides this, never the part's surface kind (Physics, 2026-10-12). One-way, the flight drag model stays and the grid's wall force is for display; two-way, a wall part's drag is the grid's wall force, and a porous part's drag is its sink's reaction;
- a per-rotor model row (`per_row`, owned by `rotors`), which `rotor.forces` reads optionally (stage 4 Task 6) and, on the GPU, through stage 6's recipe switch.

With no flag and no row, both passes compute today's bits, so no golden moves.

### 5.5 Data flow, both ways

| Phase | Pass | Reads | Writes |
|---|---|---|---|
| Fields | `airflow.step` (pose, raster, fluid; × `m_f`) | bodies, rotors, azimuths, parts, the module's state | Velocity, pressure (resident); scratch |
| Fields | `airflow.loads` (two-way, wall parts) | Pressure, velocity, solid field | Per-part loads (scratch), then per-body and per-rotor loads (state) |
| Fields | `airflow.sample` | Velocity, the point table | Point samples (§6) |
| Forces | `rotor.forces` | Optionally, per-rotor loads (line, blade) | `force_acc`, `ω` |
| Forces | `drag.forces` | Point samples | `force_acc`, except flagged bodies |
| Forces | `airflow.apply_loads`, `after: drag.forces` | Per-body loads | Accumulates `force_acc`, `torque_acc` |
| Forces | `airflow.accumulate`, `after: drag.forces` | Rotor and drag outputs (spec §1.8) | The impulse accumulators (disc sources) |
| Forces or Integrate | The azimuth pass, after `rotor.forces` | `ω`, `spin_dir` | `ψ` |

- **Held loads are state.** With `j ≥ 2`, the loads from one fluid step act on the bodies until the next. A snapshot can fall in between, so they are registered rows (`per_body`, and `per_row` under `rotors`), small and in the per-step readback, like the impulse accumulators (spec §1.4). The HUD's aero breakdown can then read the exact applied loads, with no Publish copy.
- **Accumulation order** into `force_acc` is pinned by edges, because fp32 addition is not associative (module-API spec §4).
- **One-way solids** (the grid sees the drone, the loads are published, nothing pushes back) is the coupling mode `one_way`: `airflow.apply_loads` is recorded with its apply switched off. The mode is hashed.

**Fixed-order reductions.** A load is a sum over thousands of faces; the order of that sum decides the bits. Core provides one reduction, used by every reduction that feeds stepping:
1. The faces of a part's box are cut into **fixed tiles** (for example 4×4×4 cells), independent of the workgroup size. One thread sums a tile in (k, j, i) order.
2. One thread per part sums its tiles in tile order. Per body, the parts in part order; per rotor, its blades in blade order.
3. The CPU twin sums in exactly that order. So the only CPU/GPU difference is each operation's rounding (banded), and the GPU result does not depend on the workgroup size (`A7` invariance).

No fp64: `shaderFloat64` is optional, and `CORE-5` admits any Vulkan 1.1 device. No atomics.

### 5.6 The CPU reference

The same three steps and the same reductions run on the host, in the same fp32 expressions and order: pose with the same quaternion operations; raster with `world/sdf.hpp`'s `eval()`, which `sdf_eval.slang` mirrors; tile sums in tile order. A moving-solid golden is a CPU digest at 16³ to 32³ (`L4`). Physics' airflow golden adds its own.

Two integer outputs come from banded floats, so they can differ at near-ties: the **owner** part (two parts' `φ` within ulps) and a point's **region** (a point within ulps of a box face). Parity counts such faces and points and pins their fraction as a statistic, with its record; it does not band an integer.

### 5.7 Resolution limits

| Feature (sandbox quad, *unconfirmed* sizes) | Size | Cells at 2.5 cm (R0) | at 6 mm (R1 nest) | at 1 mm |
|---|---|---|---|---|
| Body | about 10 cm | 4 | 16 | 100 |
| Arm thickness | about 1.5 cm | 0.6 | 2.5 | 15 |
| Rotor disc diameter | 24 cm | 9.6 | 40 | 240 |
| Blade chord | about 1.5–2 cm | under 1 | about 3 | 15–20 |
| Blade thickness | about 2 mm | under 0.1 | 0.3 | 2 |

- **The rule (decision D5).** A wall part whose declared smallest feature is under a minimum number of cells (Physics sets it; 2 is a placeholder) is refused at `create()` or at spawn, naming the part, its size, `dx`, and the `dx` it needs. A blade-resolved rotor whose chord is under its minimum is refused likewise, naming the rotor. The smallest feature is part metadata, from the per-part note.
- **Time resolution.** A blade-resolved or line rotor needs `m_f ≥ ceil(ω_design · h / Δψ_max)` and a tip CFL under its limit (§4.1). At 1 mm cells a sandbox blade tip needs fluid steps of about 12 µs, about 80 per 1 ms substep *(estimate)*. Blade-resolved is a slow-motion tier, as Cameron expects ("some stuff will take processing time").

### 5.8 Which rotor model each fidelity uses

| Rotor model | The fluid sees | Grid need | Cadence | Loads on the vehicle | Core plumbing in stage 7 |
|---|---|---|---|---|---|
| **Actuator disc** | Disc points (48) spread as a momentum source, with swirl (spec §3.1) | 4 or more cells across the disc | Every `j` steps | Momentum theory (`rotor.forces`, unchanged); the air gets `−J` | Accumulators, the source gather, point samples |
| **Actuator line** | `B` lines of `k` points, turning with `ψ`, spread as forces from blade-element lift and drag | Several cells across the radius *(Physics)* | `m_f` sub-steps, tip moving about a cell or less per fluid step | The line forces summed per rotor, replacing `k_T ω²` | `ψ`, rotating point sets (§6), the gather, the per-rotor model row |
| **Blade-resolved** | Each blade as a spinning wall part | 8 or more cells across the chord *(Physics)*: an R1 nest or LBM | `m_f` sub-steps | The surface loads per blade, summed per rotor | `ψ`, posed spinning parts, the raster, fixed-order loads |

The body, arms and motors are wall parts (or porous) in every case. Validation stays Physics': C8 for the disc; Caradonna–Tung after C8 for blade-resolved (spec §11.2).

## 6. Point samples, regions and Publish

### 6.1 Point samples

The module-API spec §6 specified points per body, element and world; no stage scheduled them (answers doc, row 3). Stage 7 builds them:

| Point set | Count per world | Position | Use |
|---|---|---|---|
| Per body | Body capacity | Centre of mass | Drag, the "outside the region" flag |
| Per element × k | Element capacity × k | Hub plus k offsets, rotating with `ψ` when declared | Rotor probes (spec option B), actuator line points |
| Per sensor | Sensor capacity | The mount | The airspeed sensor |
| Host requests | A capacity set at `create()` | Set by the host between steps | Publish points |

- **When.** A built-in pass writes every point's world position in Fields, after the kinematic behaviors, before any provider samples: today's sampling instant (module-API spec §6).
- **Rows.** Per-point rows exist only when a position-dependent provider is in the set. Otherwise the registry answers every point from the per-world row, as today, so the standard set allocates nothing new.
- **Readers.** On the GPU, a reader's kernel reads the per-point row behind a recipe switch (stage 6's pattern). Switched off, it reads the per-world row and computes today's bits. No standard pass is added or renamed, so the identity holds.

### 6.2 Regions

From module-API spec §11:
- A region is the whole world, a box, or an SDF shape. It has an integer priority and binds one provider per field.
- The whole-world region always exists, at the lowest priority, so every field has a value everywhere.
- Two regions of equal priority that overlap are refused at `create()`, naming both.
- Each point's region is resolved once per substep in Fields: the highest priority that holds it. A per-point region index is written; each provider then writes the points it owns.
- `compile_schedule` today refuses a second provider of a field (answers doc, row 4). Stage 7 allows several providers of one field when each is bound to a region of distinct priority, with exactly one bound to the whole world.
- **The region row** (origin cell, recentre count) is registered, small, host-resident state. **The per-body "outside" flag** is published.
- **Declared where.** A world's regions are per-world placement (§3.1), in `WorldInstanceDesc` now and in world file v3 later; empty means none, so the world's `config_hash` is unchanged.
- **Tested with a CPU-only, position-dependent test provider**, so SPH, the second consumer, needs no Core change (answers doc, Q8).

### 6.3 `sample_medium`, routed

| Where the point is | CPU backend | Vulkan backend |
|---|---|---|
| Outside every region | Today's Dryden answer, through the whole-world provider's host function | The same: `dryden` stays in the per-step readback |
| Inside a region | The region provider's host sample function, on host state | `unavailable`, naming the region, pointing to Publish points (agreed: a hidden 40 MiB readback in a point query is a trap) |

So a provider declares a **host sample function**. Dryden's is today's `DrydenMedium` expression, moved, so its values are bitwise unchanged.

### 6.4 Publish, the render volume and the GPU writer

- **Its own command buffer**, submitted on request between steps, never inside the step's recording. So the step recording does not change, and stepping cannot read Publish (`L5` by construction). This settles the spec's §4.4 against its §5 (answers doc, item 16).
- **API sketch:** `request_publish(world, request)` (the volume, its decimation, host points), `publish()`, then `flow_volume(world)` and `publish_points(world)`. The output buffers are sized at `create()` (`flow_volume.hpp:10-13` m): 32 MiB at 128³, 4 MiB decimated by 2, plus one staging copy each.
- **The GPU writer.** `flow_volume.hpp` names plan C as the writer (`:24-27` m). The interpolation off the MAC faces, `|ω|` and `q̂` are Physics'. The f32 → f16 pack is done in integer arithmetic, as `core/fp16.hpp` does, so equal inputs give equal halves on both backends, with no dependence on `shaderRoundingModeRTEFloat16`. The header's channel minimum and maximum are exact (order-free); the no-data count is an exact integer sum.
- **Banded** against the CPU writer (Physics' `publish_render_volume`), cell by cell, under `TD-14`.
- **Surface pressure.** Rendering's surface colouring reads the volume's pressure at each surface pixel (visualization plan §5, line 124). With wall parts, the cell next to the wall carries the wall pressure to first order. If a finer view is wanted, a part's surface sample points are one more host point set, posed with the part.
- **On the CPU** the same call runs the CPU writer on host state.

## 7. Device limits, memory and 3D dispatch

### 7.1 Refusals at `create()`, by name

No device limit is read anywhere in `engine/` today (searched at **m**). The only check is the group count on axis 0 (`step_recorder.cpp:306-323` m). The checks run where every buffer is sized before any is created (answers doc, row 6), before any allocation.

| Limit | Vulkan's guaranteed minimum | Checked against | The refusal names |
|---|---|---|---|
| `maxStorageBufferRange` | 2^27 (128 MiB) | Every bound buffer; the descriptor range is the whole buffer (`state_mirror.cpp:654` m) | The buffer, its bytes, the limit, the device |
| `maxMemoryAllocationSize` | 2^30 (Vulkan 1.1) | Every buffer (one allocation each) | The same |
| `maxPerStageDescriptorStorageBuffers` | 4 | Every binding in the one shared layout | The count, the limit |
| `maxDescriptorSetStorageBuffers` | 24 | The same | The same |
| `maxComputeWorkGroupCount[0..2]` | 65,535 each | Every dispatch, per axis | The pass, the axis |
| `maxComputeWorkGroupSize`, `maxComputeWorkGroupInvocations` | 128, 128, 64; 128 | The 3D local shapes | The kernel |
| The memory budget | — | §7.4 | The module, the region, the total |

- **The limits join `DeviceReport`** (`backend.hpp:146-159` m), so every run records them (`CORE-5`).
- **Batching matters.** 16 worlds of one 128³ face array already pass 128 MiB in one buffer (spec §4.1).
- **A known edge.** The set already declares 28 storage bindings (`bindings.slang`, 0–27), over the guaranteed 24 per set and 4 per stage. A device below those counts cannot legally create today's pipeline layout: exceeding `maxPerStageDescriptorStorageBuffers` is invalid API use, so such a device is not running correctly today, it is undefined behaviour. The check turns that into a named refusal (`L6`); it refuses no device that works today. But `CORE-5` is signed ("any Vulkan 1.1 or later device is admitted"), so its wording changes: decision D7. No device of record is near those minimums *(unconfirmed: the 3060 Ti's and the Iris Plus's values are read by the first task)*. Packing (spec §4.1) keeps airflow's additions small.

### 7.2 Optional features not required

16-bit and 8-bit storage and `shaderFloat64` stay unrequired (`CORE-5`). Halves and bytes are packed into `uint32` words in integer arithmetic (answers doc, item 4). Reductions are fp32 in a fixed order (§5.5).

### 7.3 Device-only buffers and memory types

- Scratch and derived buffers lose their staging twins (today every buffer has one: `state_mirror.cpp:464-476`, `:512-530` m). At 128³ that removes about 194 MiB of host-visible memory *(estimate, answers doc §5)*.
- Staging stays only for: resident syncs (one shared buffer, §2.2), Publish outputs, `step_params`, and the read-on-demand diagnostics (`step_witness`, `grid_entries`, `field_samples`).
- `find_memory_type()` takes the first matching type (`state_mirror.cpp:63-73` m). A mirror diagnostic records the type and heap chosen per buffer kind, and a test checks that staging is not `DEVICE_LOCAL` on a discrete device (answers doc §5).

### 7.4 The memory budget and the cost query

- **The check (decision D6).** Recommended: the budget comes from `VK_EXT_memory_budget` when the device has it, otherwise from the heap size, less a reserve. A `BackendDesc` cap may lower it. A cap changes no result, only whether `create()` refuses, so it belongs in the result-neutral block (`engine A7`), with its invariance test.
- **The cost query** (spec Q10): `memory_report()` lists every buffer with its kind, bytes, memory type and heap, and the host arena's bytes. "Announced" means this query, not a log line.

### 7.5 3D dispatch

- Today every dispatch is 1D (`vkCmdDispatch(cmd_, c.groups, 1, 1)`, `step_recorder.cpp:694` m), and a 128³ cell grid at workgroup 32 needs 65,536 groups on X, one past the guaranteed limit (answers doc, item 4).
- A recorded command carries three group counts. Worlds fold into the z groups, each world contiguous.
- **The `workgroup_size` knob maps to 3D shapes:** 32 → 4×4×2, 64 → 4×4×4, 128 → 8×4×4. So the existing invariance sweep covers the grid kernels too (spec §4.5). A kernel that needs one fixed shape (the 4³ coarse solve) is single-variant.
- `CORE-2` is unchanged: a barrier between every adjacent pair of emitted dispatches.

## 8. Determinism, banding and grades

**Grades** (stage 5; `GradeDecl` is stage 5 Task 3, planned, not yet built at **mg**):
- Airflow and every test module here declare CPU `reference` and Vulkan `banded`. Stage 5's rule holds them to the CPU reference; §2.1 adds that residency requires it.
- One grade per module and backend, not per configuration (stage 5 decision; spec Q9 A). The blade-resolved slow-motion tier and the real-time disc carry the same grade claim. Physical accuracy is the separate validation axis (spec §11.1).

**Deterministic per device and driver (`CORE-5`):**

| Rule | Where it holds |
|---|---|
| No atomics; every scatter is a gather in a fixed order | Sources, raster, loads, point providers |
| Reductions that feed stepping are fixed-tile and fixed-order, independent of workgroup size | §5.5 |
| The recording is a function of the tick | §4.2 |
| Dispatch counts are fixed at `create()` (`n_V`, levels, `m_f`, `n_trace`) | §4.2 |
| Halves and bytes are packed in integer arithmetic | §6.4, §7.2 |
| Reciprocals on the host, multiplied on both backends | §3.1 |
| Exact maxima only for monitors stepping never reads (CFL, `ω_design`) | Physics |

**In-process pins:** two Vulkan runs bit-identical; the three workgroup sizes bit-identical; a Vulkan snapshot, restore and resume bit-identical to the uninterrupted run, with resident arrays synced and uploaded, scratch rewritten before it is read, derived buffers rebuilt, and the cadence and `ψ` carried. That last test is banded-parity T7's, not built anywhere yet (answers doc, item 18); stage 7 builds it.

**`TD-14` bands for the fluid** (spec §4.6; Test/Docs owns the harness and gate):
- **At 32³ in the gate,** element bands up to the horizon `T_p`, where the one-ulp CPU control stays under 1e-5 of the scene's scale: velocity at the probes, the wind at each body, body pose and rates, every cell of the published volume, and, with moving solids, per-body and per-rotor loads.
- **Past `T_p`,** invariants exactly (no NaN or infinity, divergence under its bound, the momentum budget, dye non-negative, every face's owner a live part or none) and statistics at 4× the largest of eight one-ulp-perturbed CPU runs (spec's list, plus the time-averaged body load and rotor load).
- **Integers from banded floats** (owner, region) are counted, not banded (§5.6).
- **Moving walls shorten `T_p`** *(expected, unmeasured)*: blade passage amplifies an ulp. Statistics will carry most of the moving-solid evidence.
- **128³:** one opt-in, report-only record per device of record (spec Q9 A).
- **Targets:** 1e-4 relative; past 1e-3 is a grade change (`TD-2`), never a wider band.
- **Gate time:** at most about 60 s added to the debug gate (spec §4.6). Stage 7's own tests stay at 16³.

**What stage 7 tests with.** Since no developer module may carry kernels yet (module-API spec §5), stage 7's GPU tests use a small set of **domain-neutral grid kernels** that Core owns and plan C reuses: posed-SDF rasterization, fixed-order tile sums, integer-shift copy (the following box's recentre), trilinear point interpolation, fill and copy, and the fp16 pack. With them, a test module keeps a resident grid, rasterizes a moving sphere and a spinning bar at 16³, reduces a load from an analytic pressure field, and is banded against its CPU twin. Each is useful to any grid module, not only airflow (`L7`).

## 9. The chain runner

**What it is.** Physics' standalone GPU harness for plan B: persistent device buffers, a recorded multi-pipeline chain with barriers, repeated submission, timestamps and readback on demand. It is a spike and per-kernel parity tool, not engine API, not installed, not linked into `Simulation` (`tasks/todo-physics.md:65-77`). Its design note is still owed to Core (demo inputs §3), with two guards: G1, its own target and no `StepShape` or mirror change; G2, built on `VulkanContext`, extending the step recorder's chain and barrier code rather than a second recorder. `probe_runner` is its pattern (`probe_runner.hpp:18-23` m: "NOT A DISPATCHER").

**How it folds in.**

| Now (runner) | After stage 7 |
|---|---|
| Its own buffer list | Each entry tagged device-only, resident or staged, the same three kinds the mirror gets (§2, §7.3), so the list maps one-to-one onto `ScratchShape` and resident arrays |
| Its own recorded chain | The recorder's emitter (§4.2): G2 means the chain-and-barrier code is extracted once and both use it |
| Its kernels' bindings | **Bound at fixed indices reserved now** for airflow in `bindings.slang`, so plan C's module binds the same kernels unchanged |
| Its readback for the demo's volume | The publish kernels of §6.4, written once and used by both |
| Its limit checks | §7.1's checks, which the runner calls rather than copies |

**After plan C**, the sandbox demo moves to the module (two-way, the final demo). The runner stays as the per-kernel parity and timing harness, because per-kernel bands are finer evidence than whole-step bands. If the module's tests come to cover every kernel, it can be deleted then.

**Three asks of the runner's note,** so nothing is reworked: the airflow binding block reserved; the three buffer kinds; the recorder's code extended, not copied.

## 10. Order of work

An outline, not a plan. Each task is test-first, both presets, the gcc check, the leg (`TD-7`, `TD-12`). Pace: about 1 to 1.5 hours elapsed per task, from stage 4's measured day (answers doc, "Time to a first GPU airflow"). Larger tasks count double. Calendar time has run past every estimate (stage 5 Task 1 began 2026-10-06 and is not merged on 2026-10-12), so read these as effort, not dates.

| # | Task | Size | Needs |
|---|---|---|---|
| T1 | Device limits read into `DeviceReport`; refusals by name; the memory-type diagnostic | 1 | — (may land early) |
| T2 | Device-only buffers: scratch and derived lose staging | 1 | T1 |
| T3 | 3D dispatch; recipes as recording functions; per-axis checks; pipelines on demand | 1.5 | T1 |
| T4 | Hashed module configuration; the identity spelling; `configuration_report()`; regions' placement in `config_hash` when present | 1 | Stage 5 (the identity pins) |
| T5 | Resident arrays: the flag, the ledger, the mirror split, shared staging, sync or refuse, `sync_resident()`, the snapshot overloads, `state_digest`, the Vulkan restore-resume test | 2 | T2, T4, stage 5 (grade rule) |
| T6 | The rebuild hook and on-demand recordings | 1 | T3, T5 |
| T7 | The cadence: `every_steps`, `due()`, two recordings, timing per recording, the substep index | 1 | T3 |
| T8 | Point samples: point sets, the positions pass, per-point rows, readers' recipe switch | 1.5 | Stage 6 (recipe switch) |
| T9 | Regions: declarations, priority resolution, several providers, host sample functions, `sample_medium` routing and refusal | 2 | T8 |
| T10 | The grid kernels: fill, copy, integer shift, fixed-order tile sums, trilinear points, fp16 pack; CPU twins; bands | 1.5 | T3 |
| T11 | Posed parts: the part table and its device mirror, the vehicle rows, the azimuth row mechanism, pose and raster; a moving sphere and a spinning bar, banded | 2 | T10, stage 6 (table mirror), the per-part note's code |
| T12 | Loads: per-part fixed-order reductions, per-body and per-rotor rows, `apply_loads`, the flag bit and the per-rotor model row | 1.5 | T11, stage 6 |
| T13 | Publish: requests, the publish recording, the `FlowVolume` GPU writer, banded against the CPU writer | 1.5 | T6, T10, Physics' CPU writer |
| T14 | The memory budget, the cap, `memory_report()` | 1 | T1 (may land early) |
| T15 | Prose: comments, Core's docs, `CHANGELOG.md`, `consumers.md` lines (snapshot overloads, new calls, `state_digest`'s signature) | 0.5 | All |

- **Size:** about 20 task units. **Effort:** about 2.5 to 4 working days of Core time *(estimate)*. Core's 2026-10-05 estimate was about 8 tasks and 1 to 1.5 days; moving solids (T11, T12), the grid kernels (T10), the GPU Publish writer (T13) and the cost query (T14) are the growth.
- **Before it:** the rest of stage 5 (Tasks 2–8, about a day *(estimate)*) and stage 6 (its plan and code, about 1 to 1.5 days *(estimate)*).
- **What can start early:** T1, T2, T3 and T14 need neither stage 5 nor stage 6 (answers doc: "the limit refusals and the budget check stand alone"). Pulling them in beside stage 6 shortens the path by about a day.
- **Outside Core, on the path:** the per-part note's code (`ModelType` parts, their `model_identity` fold) before T11; Physics' `ψ` pass and load formulas before T12 ends; Physics' CPU volume writer before T13; the runner's note (§9) before plan B's code.
- **The longer pole is still the solver** and now the moving-wall numerics (Physics), not Core's stage.

## 11. Decisions for Cameron

**D1. The digest of device-resident state.**
- (A) Folded after a sync, like any array: one digest; the CPU golden covers the grid; Vulkan determinism tests cover it.
- (B) Left out of `state_digest`, with its own digest and its own golden: two things to keep in step.
- (C) Left out of every digest and banded only: no golden for the grid, which weakens its reference grade (`L4`).
- **Recommend A.** The cost is one readback per digest call on Vulkan, in tests only.

**D2. The default fluid cadence.**
- (A) `j` = 1: the fluid every step (2 ms in the sandbox). Likely not real time at 128³.
- (B) `j` = 2 (4 ms), the spec's recommended start, re-set from the spike's measurement (M1).
- (C) No default: every configuration must name `j`.
- **Recommend B,** with `m_f` = 1 by default. Blade-resolved and line rotors derive `m_f` from their declared `ω_design` (§4.1).

**D3. Blade-resolved rotors: in stage 7, or later.**
- (A) Stage 7 builds the plumbing for all three rotor models: spinning parts posed by `ψ`, fluid sub-steps, fixed-order blade loads. Physics' blade numerics and validation follow after C8, with nests.
- (B) Disc and line in stage 7; spinning wall parts and sub-steps in a later stage, with R1 nests.
- (C) The disc only in stage 7.
- **Recommend A.** Rigid wall parts need the pose, raster and load machinery anyway; spinning parts add only `ψ` and the sub-step unroll, about half a task. B and C would reopen the recorder and the raster later.

**D4. What `snapshot()` does on Vulkan with a resident grid.**
- (A) A non-const `snapshot()` syncs, then saves (about 40 MiB of readback per world at 128³); called through a `const` reference it refuses, naming the array.
- (B) Always explicit: every read path refuses until the caller calls `sync_resident()`.
- (C) Never sync: blobs leave the grid out, and a restore restarts the air at ambient. That breaks replay (`L2`).
- **Recommend A.** It satisfies §15 ("sync or refuse") with no signature change for callers holding a non-const `Simulation`.

**D5. A part the grid cannot resolve.**
- (A) Refuse at `create()` or spawn, naming the part, its size and the cell size it needs; the rotor model is always chosen explicitly.
- (B) Downgrade automatically (a blade to an actuator line or disc, a thin wall to porous), announced through a query.
- (C) Run anyway and flag it in the diagnostics.
- **Recommend A** for the engine (`L6`). The drone builder can offer the downgrade as an explicit choice in its UI (Interface).

**D6. The memory budget.**
- (A) From `VK_EXT_memory_budget` when present, otherwise the heap size, less a reserve; a caller may lower it with a `BackendDesc` cap.
- (B) A fixed 4 GiB cap for airflow (spec §4.3).
- (C) No budget check: an allocation failure becomes `capacity_exceeded`, as today.
- **Recommend A.** It follows the device the run is on; B's 4 GiB is right for an 8 GiB card and wrong for others.

**D7. `CORE-5`'s wording, for the device-limit check (§7.1).**
- (A) Amend `CORE-5`: any Vulkan 1.1 or later device is admitted *if it meets the limits the module set declares*; otherwise `create()` refuses by name, giving the limit, the need and the device. A device below those limits misbehaves silently today (invalid API use), so no working device is lost.
- (B) Keep `CORE-5` as written, and leave the limits unchecked: today's silent undefined behaviour stays.
- **Recommend A.** It is `L6`, and it is how the airflow buffers' size limits are checked anyway.

## For the lead (not Cameron's)

1. **`arenas()` cannot refuse** (§2.3). §15 lists "CPU state views" among the paths that sync or refuse. Core proposes `arenas()` as the one documented raw view, with `resident_current()`, while every engine path that folds bytes syncs.
2. **`state_digest` takes `Simulation&`** (§2.3). The plan counts the `const` call sites first; if there are many, a `const` overload that refuses at run time instead.
3. **Core owns the domain-neutral grid kernels** of §8 (raster, sums, shift, points, fp16 pack); Physics reviews them. Physics owns every kernel with a fluid formula in it.
4. **Reserve airflow's binding block in `bindings.slang` now,** with stage 6's empty-buffer pattern, so plan B's kernels bind there from the start.

## Conflicts found

| With | The conflict | Proposal |
|---|---|---|
| Airflow spec §2 ("Moving bodies") and Q4 | Vehicles are porous in the real-time tier; solid moving bodies belong to LBM. Cameron superseded this on 2026-10-12: a solid wall by default, porous optional (`backlog.md:66`) | Physics revises §2, Q4 and §4.3's memory row (moving solids are about 48–72 MiB at 128³, not 24) |
| Airflow spec §4.5 | "No order-dependent reductions in the step." Two-way loads are reductions that feed stepping. So is plan A's closed-domain mean, a sequential **double** sum (`projection.cpp:32-37`, `pa`), and fp64 is optional on the GPU | Every reduction that feeds stepping uses §5.5's fixed-order fp32 tiles on both backends. Physics decides whether the CPU reference's mean moves to it |
| Airflow spec §1.4 | Only `h_f = j · dt`. A blade at 700 rad/s *(unconfirmed)* turns about 2.8 rad per 4 ms fluid step | A second cadence, `m_f` sub-steps per substep (§4.1) |
| `RotorRow` (`rotor.hpp:535-549`, m) | Blade-resolved and line models need `ψ`; a field in `RotorRow` moves every golden | `ψ` as a `per_row` array owned by `rotors` (§5.3) |
| Module-API spec §15 | "CPU state views" must sync or refuse; `arenas()` returns a reference and can do neither | For the lead, item 1 |
| `bindings.slang` and `CORE-5` | 28 bindings already exceed Vulkan's guaranteed 24 per set and 4 per stage, unchecked. The new check could refuse a minimum-spec device admitted today | Check and refuse by name (§7.1); T1 records the devices of record's limits |
| `StateMirror`'s founding rule (`state_mirror.hpp:39-45`, m) | "The arena remains the one authoritative CPU-side store" | Changes as §15 approved (§2.2) |
| Demo inputs §1b (`ModelType::collider`) | One collider per model | Superseded by the per-part note's parts. The identity rule carries over: fold when non-empty, once a step reads it |
| Stage 5 plan, "Notes for later stages" | "Stage 7 grades nothing new" | It adds one compile rule: a resident array needs a CPU `reference` (§2.1) |

## Rulings received

- **The user, 2026-10-12 (via lead):** the design is signed. D1 [A], D2 [B], D3 [A], D4 [A], D5 [A], D6 as drafted, and D7 [A]: `CORE-5` reads "any Vulkan 1.1+ device is admitted if it meets the limits the module set declares; otherwise it is refused by name" (`CORE-5`, `CORE-8`).
- **The lead, 2026-10-12, on "For the lead":**
  1. `arenas()`: no silent stale read. The non-const `arenas()` syncs resident state before it returns. A const path that cannot sync refuses at run time when resident state is newer than the host copy. A raw "may be stale" view would break §15's sync-or-refuse. If the const call sites make this painful, the count goes to the lead.
  2. `state_digest`: as proposed. Count the const call sites; add a refusing const overload if there are many.
  3. Core owns the domain-neutral grid kernels; Physics reviews them.
  4. Reserve airflow's binding block in `bindings.slang` now.
- **Conflicts, resolved:**
  - spec §4.5: two-way loads and the closed-domain mean both use §5.5's fixed-tile fp32 reduction on both backends. That is Physics' numerics call; Physics moves `remove_mean` to it before plan A Task 8 pins its golden (`af51484`).
  - spec §2/Q4: amended to the solid wall (`c52b487`).
  - §4.3: +48 to +72 MiB.
  - §1.4: `m_f` accepted.
  - D5's first thresholds (Physics, `af51484`; revised by validation): a wall feature ≥ 2 cells; actuator line R/dx ≥ 20 with ε = 2dx; blade-resolved ≥ 32 cells per chord and ≥ 2 per thickness. All refused by name, never silently made porous.
