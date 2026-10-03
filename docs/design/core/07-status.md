# Core — status: specified vs built

**Owner:** Core. The only Core document that describes the present. Checked against the tree at **master `0dd8bdc`** (2026-10-01) by reading the code and CMake, and re-checked at **`332a186`** (2026-10-02): nothing in Core's code changed after `8009587` except the merged install fix, so every row holds there. The module rows, the snapshot row and the debt were updated at **`062e1fa`** (2026-10-03), the merge of module-API stage 1. The GPU-chain and per-pass refusal rows were updated at **`dca7cfb`** (2026-10-03), the merge of stage 2. The only measured results are the branch runs named in a row, each with its tree.

## Specified vs built

| Spec (this library) | Built today | Where |
|---|---|---|
| Modules declare state, fields, components, passes, grades | **Passes only** (module-API stage 1, merged `062e1fa`). A `ModuleDesc` declares passes with phase, placement, access and edges. `Simulation::create()` takes a module set, which defaults to the standard set of nine built-ins, and still registers every array itself | `sim/module.hpp`, `sim/standard_modules.cpp` |
| Scheduler phases, placement by reads/writes | **Built for the CPU** (plan stage 1). `compile_schedule` orders a module set's passes into the six phases by declared access, placement and edges. It refuses a conflict, a dangling edge or a cycle. The standard set compiles to the old ten-pass order, and the CPU step runs the compiled list. Measured on the branch tree (`spade-wt/core`, msvc-ninja-release) at `25f3c8a`: full suite 959 total, 0 failed, 2 skipped, all 70 `Gpu*` passed, no golden moved | `sim/module_schedule.cpp` |
| GPU chain derived from the schedule | **Yes** (module-API stage 2, merged `dca7cfb`). Each pass names a GPU recipe, a built-in kernel, and may name only its own CPU function's recipe (`compile_schedule` refuses anything else). `Simulation::create()` hands `gpu_passes(schedule)` to the backend, and the recorder walks it every substep. The standard set records the same dispatches and barriers as before, 9 + S per substep. Timings are per pass, by name. Measured on the branch: full suite 992 total, 0 failed, 2 skipped, all 82 `Gpu*` passed (msvc-ninja-release); the gcc-13 Docker leg passed on `ec37857` | `compute/vulkan/step_recorder.cpp`, `sim/standard_modules.cpp` |
| Registered, world-partitioned state | **Yes.** 11 arrays, 22 walk entries with their `slot_to_world` siblings. Lowest-free-slot allocation | `state/arenas.*`, `state/registry.*` |
| Snapshot, restore refusing a different config | **Yes.** Format version 2. The header carries the module set's configuration identity, and `Simulation::restore` refuses another one. The config hash lives in the `replay_config` row, which is pinned at walk position 16 | `state/snapshot.*`, `sim/simulation.*` |
| Reseed (`engine A3`), cursor split (`engine A4`) | **Yes** | `Simulation::reseed`, sensor rows |
| Configuration hash with the size guard | **Yes.** `WorldDesc` is 320 bytes (384 under MSVC iterator debugging) | `sim/world_set.*` |
| Many worlds, batching invariance | **Yes**, tested | `sim/`, `physics/grid.*` |
| World description, file schema v2 | **Yes**, with v1 upgrade on load | `world/builder.*`, `world/world_file.*` |
| Regions | **No.** Nothing in the code or the KAT-era docs. The world is the only partition | — |
| Field registry and sampling | **No registry.** One field family (medium), one provider per world (Dryden over a constant), position-independent. `Simulation::sample_medium(world, pos)` reads it from the host (merged `ceb4aef`) | `world/medium.*`, `sim/simulation.*` |
| Object graph, frozen component ids, JSON | **Yes**, but not wired into stepping: `Simulation` holds no `ObjectGraph` | `objects/` |
| Attach/detach queue to step boundaries (`SL4`) | **No.** They mutate at once. Harmless until objects take part in stepping | `objects/graph.*` |
| Behaviors | **CPU only**, as the two placed passes of the `behaviors` module: kinematic first in Fields, force last in Forces. Read/write masks are declared but not consumed. `record_gpu` is stored and never called | `objects/behavior.*` |
| Refuse a pass the GPU cannot run (`L6`) | **For behaviors, yes** (`CORE-1`, merged `ceb4aef`): a Vulkan `step()` with a registry attached returns `unavailable`. Since stage 2, a pass with no GPU recipe is refused on Vulkan at `create()`, naming `<module>.<pass>`. Measured on the branch tree (`spade-wt/core`, msvc-ninja-release): `GpuBehaviorRefusal` was red before the fix and green after; full suite 900 total, 0 failed, 33 skipped | `sim/simulation.cpp` |
| Grades, grade check at `create()` | **No** | — |
| Backend seam: CPU default, Vulkan record-once, denormal-preserving device | **Yes** | `compute/backend.hpp`, `compute/vulkan/` |
| A barrier between every adjacent dispatch pair | **Yes** (`CORE-2`, merged `c7a36a4`). Measured on the branch tree: before the fix the recorder made 13/27/69 barriers where 14/29/74 were needed (1/2/5 substeps); after, `RecordedChainHasABarrierBetweenEveryAdjacentDispatchPair` passes and all 64 `Gpu*` tests pass on the Iris Plus | `compute/vulkan/step_recorder.cpp` |

## Needs a user decision

None open. The sensor dedup was ruled on 2026-10-02: the module API absorbs it (`CORE-4`).

## Next

- **Module API, stage 3: fields and sample buffers** (`plans/2026-10-03-module-api-stage3-plan.md`, approved). It starts once Physics' `integrator.hpp` comment sweep lands.
  - The spec was approved 2026-10-02 (`plans/2026-10-02-module-api-design.md`).
  - Stage 1 is merged (`062e1fa`), and so is stage 2 (`dca7cfb`).
  - Stages 4 to 6 follow: modules owning state (where the IMU/GNSS pairs collapse, `CORE-4`), grades and roles, and the translation lock.

## Debt

**Defects (2026-10-02):** none open. StateMirror's buffers come from one list (defect 5, `835677d`), and `reseed()` reaches the GNSS noise streams (defect 6, `09eb364`). Both were red, then green.

**Defects (2026-10-01):** none open. All three are on master with red-then-green evidence: the behavior refusal (`CORE-1`, `ceb4aef`), the barrier shortfall (`CORE-2`, `c7a36a4`), and the `SPADE_VULKAN=OFF` install, which now ships the three `compute/` headers `sim/simulation.hpp` includes (`3a48c4d`). The install fix was verified by hand: with Vulkan off, `tests/consumer` failed to compile against master's install (`C1083`, `compute/backend.hpp`) and built and ran against the fix. No gate runs that check yet (`../backlog.md`, consumer smoke).

**Code that disagrees with itself:** none known. Core's stale comments were corrected in `8009587`, and `CONTRIBUTING.md`/`README.md` no longer carry the "frozen at 18" wording.

**Deferred on purpose:**
- `fnv1a64` belongs in a `core/hash.hpp`. It was left in `state/snapshot.hpp` because four golden digests depend on its output.
- The model registry is configuration and not in the blob, so a restoring caller re-registers models.
- The Vulkan path flushes the structural queue once per `step(n)`. That is sound while nothing queues from inside a step.
- Any new method that writes arenas owes a `mark_vulkan_dirty` call.
- `replay_config` does not yet carry the configuration identity, which lives in the snapshot header (module-API plan, Ruling 1). It can move into `replay_config` with the next deliberate golden regeneration.

**Restructure R2, step 4:** done. Core's code cites no KAT series (merged `8009587`).
