# Core — status: specified vs built

**Owner:** Core. The only Core document that describes the present. Checked against the tree at **master `0dd8bdc`** (2026-10-01) by reading the code and CMake, and re-checked at **`8009587`** (2026-10-02) for the rows Core's merges changed. The only measured results are the branch runs named in a row, each with its tree.

## Specified vs built

| Spec (this library) | Built today | Where |
|---|---|---|
| Modules declare state, fields, components, passes, grades | **No.** `Simulation::create()` registers every array itself; there is no module type | `sim/simulation.cpp` |
| Scheduler phases, placement by reads/writes | **No.** A fixed `constexpr` array of ten passes; no API to add one | `physics/schedule.cpp` (`kSchedule`) |
| GPU chain derived from the schedule | **No.** The recorder keeps its own eight-slot table and never reads `kSchedule`. A CPU test pins `kSchedule`'s non-behavior passes to the same eight names (`Schedule.RemovingTheBehaviorSlotsLeavesSpecSectionThreeExactly`) | `compute/vulkan/step_recorder.cpp` |
| Registered, world-partitioned state | **Yes.** 11 arrays, 22 walk entries with their `slot_to_world` siblings. Lowest-free-slot allocation | `state/arenas.*`, `state/registry.*` |
| Snapshot, restore refusing a different config | **Yes.** Format version 1; the config hash lives in the `replay_config` row, which is pinned at walk position 16 | `state/snapshot.*`, `sim/simulation.*` |
| Reseed (`engine A3`), cursor split (`engine A4`) | **Yes** | `Simulation::reseed`, sensor rows |
| Configuration hash with the size guard | **Yes.** `WorldDesc` is 320 bytes (384 under MSVC iterator debugging) | `sim/world_set.*` |
| Many worlds, batching invariance | **Yes**, tested | `sim/`, `physics/grid.*` |
| World description, file schema v2 | **Yes**, with v1 upgrade on load | `world/builder.*`, `world/world_file.*` |
| Regions | **No.** Nothing in the code or the KAT-era docs. The world is the only partition | — |
| Field registry and sampling | **No registry.** One field family (medium), one provider per world (Dryden over a constant), position-independent. `Simulation::sample_medium(world, pos)` reads it from the host (merged `ceb4aef`) | `world/medium.*`, `sim/simulation.*` |
| Object graph, frozen component ids, JSON | **Yes**, but not wired into stepping: `Simulation` holds no `ObjectGraph` | `objects/` |
| Attach/detach queue to step boundaries (`SL4`) | **No.** They mutate at once. Harmless until objects take part in stepping | `objects/graph.*` |
| Behaviors | **CPU only**, in two fixed slots. Read/write masks are declared but not consumed. `record_gpu` is stored and never called | `objects/behavior.*` |
| Refuse a pass the GPU cannot run (`L6`) | **For behaviors, yes** (`CORE-1`, merged `ceb4aef`): a Vulkan `step()` with a registry attached returns `unavailable`. There is no general per-pass refusal yet, because there are no modules. Measured on the branch tree (`spade-wt/core`, msvc-ninja-release): `GpuBehaviorRefusal` was red before the fix and green after; full suite 900 total, 0 failed, 33 skipped | `sim/simulation.cpp` |
| Grades, grade check at `create()` | **No** | — |
| Backend seam: CPU default, Vulkan record-once, denormal-preserving device | **Yes** | `compute/backend.hpp`, `compute/vulkan/` |
| A barrier between every adjacent dispatch pair | **Yes** (`CORE-2`, merged `c7a36a4`). Measured on the branch tree: before the fix the recorder made 13/27/69 barriers where 14/29/74 were needed (1/2/5 substeps); after, `RecordedChainHasABarrierBetweenEveryAdjacentDispatchPair` passes and all 64 `Gpu*` tests pass on the Iris Plus | `compute/vulkan/step_recorder.cpp` |

## Needs a user decision

- **Sensor dispatch duplication.** `Simulation` carries six IMU/GNSS function pairs that differ only by type. `superseded/2026-09-consolidation/sensor-arena-dedup.md` proposes two things: retire amendment `a0f81cde`, which would put every sensor in one kind-tagged arena, and remove the duplication with one compile-time template instead, which moves no bytes and no digest. That proposal was never ruled on, and nothing has started. The module API will also reshape this, so the ruling may simply be "fold it into the module-API spec".
- **The next Core spec** (module API, scheduler, regions; `../backlog.md`, first row) will bring its own questions. One is already visible: whether kinematic behaviors run first in Forces or last in Fields (`01-modules-and-scheduler.md`).

## Debt

**Defects (2026-10-01):**
- A `SPADE_VULKAN=OFF` install ships `sim/simulation.hpp` without the `compute/` headers it includes. Fixed on branch `core/vulkan-off-install`, awaiting its own OFF-tree build. (The behavior refusal and the barrier shortfall merged as `CORE-1` and `CORE-2`.)

**Code that disagrees with itself:** Core's own stale comments were corrected in `8009587`. What remains is outside Core's code: `CONTRIBUTING.md` still says the state arrays are "frozen at 18" and that adding one moves `kSnapshotVersion`, and `README.md` still says "18-entry walk" (Test/Docs, restructure R4).

**Deferred on purpose:**
- `fnv1a64` belongs in a `core/hash.hpp`. It was left in `state/snapshot.hpp` because four golden digests depend on its output.
- The model registry is configuration and not in the blob, so a restoring caller re-registers models.
- The Vulkan path flushes the structural queue once per `step(n)`. That is sound while nothing queues from inside a step.
- Any new method that writes arenas owes a `mark_vulkan_dirty` call.

**Restructure R2, step 4:** done. Core's code cites no KAT series (merged `8009587`).
