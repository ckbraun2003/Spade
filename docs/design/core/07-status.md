# Core — status: specified vs built

**Owner:** Core. The only Core document that describes the present. Checked against the tree at **master `0dd8bdc`** (2026-10-01) by reading the code and CMake, and re-checked at **`332a186`** (2026-10-02): nothing in Core's code changed after `8009587` except the merged install fix, so every row holds there. The module rows, the snapshot row and the debt were updated at **`062e1fa`** (2026-10-03), the merge of module-API stage 1. The GPU-chain and per-pass refusal rows were updated at **`dca7cfb`** (2026-10-03), the merge of stage 2, and the field row at **`42b352a`** (2026-10-03), the merge of stage 3. The module, state, snapshot and reseed rows, Next and the debt were updated for stage 4 (2026-10-05) on `core/module-state` at **`3a6dbea`** (Task 7b, rebased onto PHY-7): Task 1 merged as `eb57d4d`, Tasks 2 to 7 as `a698ea4`. The only measured results are the branch runs named in a row, each with its tree.

## Specified vs built

| Spec (this library) | Built today | Where |
|---|---|---|
| Modules declare state, fields, components, passes, grades | **State, fields and passes** (module-API stages 1 to 4). A `ModuleDesc` declares its passes (phase, placement, access, `after` and `before` edges, optional reads), the fields it provides, the arrays it owns with their extents and row `init` and `validate`, its seeded streams, its configuration tables, a vehicle-spawn hook, and its scratch. `Simulation::create()` takes a module set, which defaults to the standard set of ten built-ins, and registers every module array from the declarations. A pass sees the arrays it declares through `SubstepContext::state`; the built-in passes keep their typed views. Components and grades are not declared yet. Measured on the branch at `b7a120b` (Task 8's comments over `3a6dbea`; msvc-ninja release and debug): 1386 total, 0 failed, 2 skipped by design, all 108 `gpu` passed, no golden moved | `sim/module.hpp`, `sim/module_schedule.cpp`, `sim/standard_modules.cpp`, `sim/builtin_state.*` |
| Scheduler phases, placement by reads/writes | **Built for the CPU** (plan stage 1). `compile_schedule` orders a module set's passes into the six phases by declared access, placement and edges. It refuses a conflict, a dangling edge or a cycle. The standard set compiles to the old ten-pass order, and the CPU step runs the compiled list. Measured on the branch tree (`spade-wt/core`, msvc-ninja-release) at `25f3c8a`: full suite 959 total, 0 failed, 2 skipped, all 70 `Gpu*` passed, no golden moved | `sim/module_schedule.cpp` |
| GPU chain derived from the schedule | **Yes** (module-API stage 2, merged `dca7cfb`). Each pass names a GPU recipe, a built-in kernel, and may name only its own CPU function's recipe (`compile_schedule` refuses anything else). `Simulation::create()` hands `gpu_passes(schedule)` to the backend, and the recorder walks it every substep. The standard set records 11 + S dispatches per substep since stage 3, which added the two field-sample kernels; until then it recorded the same dispatches and barriers as before stage 2, 9 + S. Timings are per pass, by name. Measured on the branch: full suite 992 total, 0 failed, 2 skipped, all 82 `Gpu*` passed (msvc-ninja-release); the gcc-13 Docker leg passed on `ec37857` | `compute/vulkan/step_recorder.cpp`, `sim/standard_modules.cpp` |
| Registered, world-partitioned state | **Yes.** 11 arrays in the standard set, 22 walk entries with their `slot_to_world` siblings. The core registers four; the rest come from the module declarations, the four legacy-marked modules' before `replay_config` and every other module's after it, so the standard walk and its schema hash are unchanged. A set that omits or reshapes one of the seven built-in arrays is refused. The GPU mirror sizes its buffers from the same declarations, counting in 64 bits. Lowest-free-slot allocation. Attached rows enter through `attach_row` (the typed `add_*` calls front it) and leave through the despawn cascade, by declaration and in walk order | `state/arenas.*`, `state/registry.*`, `sim/simulation.cpp`, `compute/vulkan/state_mirror.*` |
| Snapshot, restore refusing a different config | **Yes.** Format version 3. The header carries the module set's configuration identity and the model registry's identity, and `Simulation::restore` refuses another of either. The config hash lives in the `replay_config` row, which is pinned at walk position 16. State declarations are not in the configuration identity; the schema hash covers the arrays | `state/snapshot.*`, `sim/simulation.*` |
| Reseed (`engine A3`), cursor split (`engine A4`) | **Yes.** `reseed()` walks the declared streams (`ModuleDesc::streams`): per world, each `per_world` stream and every live row of a slot-allocated one. A tag is unique in the set and never `world`. A set that drops or retags a built-in stream is refused | `Simulation::reseed`, `sim/standard_modules.cpp`, sensor rows |
| Configuration hash with the size guard | **Yes.** `WorldDesc` is 320 bytes (384 under MSVC iterator debugging) | `sim/world_set.*` |
| Many worlds, batching invariance | **Yes**, tested | `sim/`, `physics/grid.*` |
| World description, file schema v2 | **Yes**, with v1 upgrade on load | `world/builder.*`, `world/world_file.*` |
| Regions | **No.** Nothing in the code or the KAT-era docs. The world is the only partition | — |
| Field registry and sampling | **Built for gravity, density and wind** (module-API stage 3, merged `42b352a`). A module declares the fields it provides: scalar, vec3, or a band array of 1 to 32 floats. `compile_schedule` refuses a read with no provider, two providers, a built-in of another kind or unit, and a write outside the provider or outside Fields. Two providers, `environment.sample` and `dryden.sample`, write each world's scratch sample row once per substep in Fields, on both backends (11 + S dispatches). Rotors, drag and Integrate read it, bitwise the old inline values. Position-dependent providers and per-point sample points wait for SPH. `Simulation::sample_medium(world, pos)` still reads the provider's CPU function on current state. Measured on the branch: full suite 1013 total, 0 failed, 2 skipped, 83 `Gpu*` passed; the gcc-13 leg passed on `b425a2a` | `world/medium.*`, `sim/simulation.*` |
| Object graph, frozen component ids, JSON | **Yes**, but not wired into stepping: `Simulation` holds no `ObjectGraph` | `objects/` |
| Attach/detach queue to step boundaries (`SL4`) | **No.** They mutate at once. Harmless until objects take part in stepping | `objects/graph.*` |
| Behaviors | **CPU only**, as the two placed passes of the `behaviors` module: kinematic first in Fields, force last in Forces. Read/write masks are declared but not consumed. `record_gpu` is stored and never called | `objects/behavior.*` |
| Refuse a pass the GPU cannot run (`L6`) | **For behaviors, yes** (`CORE-1`, merged `ceb4aef`): a Vulkan `step()` with a registry attached returns `unavailable`. Since stage 2, a pass with no GPU recipe is refused on Vulkan at `create()`, naming `<module>.<pass>`. Measured on the branch tree (`spade-wt/core`, msvc-ninja-release): `GpuBehaviorRefusal` was red before the fix and green after; full suite 900 total, 0 failed, 33 skipped | `sim/simulation.cpp` |
| Grades, grade check at `create()` | **No** | — |
| Backend seam: CPU default, Vulkan record-once, any Vulkan 1.1 device admitted and announced (`CORE-5`) | **Yes** | `compute/backend.hpp`, `compute/vulkan/` |
| A barrier between every adjacent dispatch pair | **Yes** (`CORE-2`, merged `c7a36a4`). Measured on the branch tree: before the fix the recorder made 13/27/69 barriers where 14/29/74 were needed (1/2/5 substeps); after, `RecordedChainHasABarrierBetweenEveryAdjacentDispatchPair` passes and all 64 `Gpu*` tests pass on the Iris Plus | `compute/vulkan/step_recorder.cpp` |

## Needs a user decision

None open. The sensor dedup (`CORE-4`) is done: stage 4 collapsed the IMU/GNSS pairs (`bd0f640`).

## Next

- **The drone builder, Core's part** (`plans/2026-10-03-drone-builder-core-plan.md`): done. A, `transform_of` public, merged `65c2295`. B, the design frame, `7532259`. C, snapshot format v3 with the model registry's identity, `7d46b6d`. D, the scene-file schema and its first golden, `43eac56`.
- **Module API, stage 4: modules own their state** (`CORE-4`, `plans/2026-10-05-module-api-stage4-plan.md`): done, merged 2026-10-06.
  - The spec was approved 2026-10-02 (`plans/2026-10-02-module-api-design.md`). Stages 1, 2 and 3 are merged (`062e1fa`, `dca7cfb`, `42b352a`).
  - Task 1 is merged (`eb57d4d`), Tasks 2 to 7 (`a698ea4`), and Task 7b, the scratch kind, with Task 8, this prose (head `5663293`).
  - The configuration tables' GPU mirror waits for stage 6's fixed bindings.
- **Module API, stage 5: grades, roles, component availability** (`plans/2026-10-05-module-api-stage5-plan.md`, approved 2026-10-05). Task 1 first: tables shared by every world, `table_rows<T>`, `all_rows<T>`, and a sticky fault for a misused accessor.
- **Then stage 6** (the translation lock), **then stage 7** (airflow's GPU groundwork, design to the user before code; spec §15).

## Debt

**Defects (2026-10-02):** none open. StateMirror's buffers come from one list (defect 5, `835677d`), and `reseed()` reaches the GNSS noise streams (defect 6, `09eb364`). Both were red, then green.

**Defects (2026-10-01):** none open. All three are on master with red-then-green evidence: the behavior refusal (`CORE-1`, `ceb4aef`), the barrier shortfall (`CORE-2`, `c7a36a4`), and the `SPADE_VULKAN=OFF` install, which now ships the three `compute/` headers `sim/simulation.hpp` includes (`3a48c4d`). The install fix was verified by hand: with Vulkan off, `tests/consumer` failed to compile against master's install (`C1083`, `compute/backend.hpp`) and built and ran against the fix. No gate runs that check yet (`../backlog.md`, consumer smoke).

**Code that disagrees with itself:** none known in Core. Stage 4 deleted the dead `free_sensors_of` and `clear_sensor_ring` templates, which nothing called, with the comment that called the live field-wise ring clear a memset (`bd0f640`); the cascade now clears a ring by `memset`. Core's stale comments were corrected in `8009587` and stage 4's in Task 8, and `CONTRIBUTING.md`/`README.md` no longer carry the "frozen at 18" wording. Three Physics comments still say their arrays are registered with `register_array<T>` (`physics/forces.hpp`'s `DragBodyRow`, `vehicles/rotor.hpp`'s `RotorRow`, `world/medium.hpp`'s `DrydenState`); since stage 4 the module declarations register them.

**Deferred on purpose:**
- `fnv1a64` belongs in a `core/hash.hpp`. It was left in `state/snapshot.hpp` because four golden digests depend on its output.
- The model registry is configuration and not in the blob, so a restoring caller re-registers models.
- The Vulkan path flushes the structural queue once per `step(n)`. That is sound while nothing queues from inside a step.
- Any new method that writes arenas owes a `mark_vulkan_dirty` call.
- `replay_config` does not yet carry the configuration identity, which lives in the snapshot header (module-API plan, Ruling 1). It can move into `replay_config` with the next deliberate golden regeneration.

**Restructure R2, step 4:** done. Core's code cites no KAT series (merged `8009587`).
