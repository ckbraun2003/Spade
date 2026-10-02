# Core — status: specified vs built

**Owner:** Core. The only Core document that describes the present. Checked against the tree at **master `0dd8bdc`** (2026-10-01) by reading the code and CMake. No build tree existed in this checkout, so nothing below is a measured test result. Three Core branches are written and awaiting a build slot; they are listed under "Debt" until they merge.

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
| Field registry and sampling | **No registry.** One field family (medium), one provider per world (Dryden over a constant), position-independent. Host sampling (`sample_medium`) is on branch `core/drone-medium` | `world/medium.*` |
| Object graph, frozen component ids, JSON | **Yes**, but not wired into stepping: `Simulation` holds no `ObjectGraph` | `objects/` |
| Attach/detach queue to step boundaries (`SL4`) | **No.** They mutate at once. Harmless until objects take part in stepping | `objects/graph.*` |
| Behaviors | **CPU only**, in two fixed slots. Read/write masks are declared but not consumed. `record_gpu` is stored and never called | `objects/behavior.*` |
| Refuse a pass the GPU cannot run (`L6`) | **No** on master: the Vulkan step skips an attached registry. Fixed on `core/drone-medium` | `sim/simulation.cpp` |
| Grades, grade check at `create()` | **No** | — |
| Backend seam: CPU default, Vulkan record-once, denormal-preserving device | **Yes** | `compute/backend.hpp`, `compute/vulkan/` |
| A barrier between every adjacent dispatch pair | **No** on master: the last `substeps` barriers are dropped. Fixed on `core/barrier-count` | `compute/vulkan/step_recorder.cpp` |

## Needs a user decision

- **Sensor dispatch duplication.** `Simulation` carries six IMU/GNSS function pairs that differ only by type. `superseded/2026-09-consolidation/sensor-arena-dedup.md` proposes two things: retire amendment `a0f81cde`, which would put every sensor in one kind-tagged arena, and remove the duplication with one compile-time template instead, which moves no bytes and no digest. That proposal was never ruled on, and nothing has started. The module API will also reshape this, so the ruling may simply be "fold it into the module-API spec".
- **The next Core spec** (module API, scheduler, regions; `../backlog.md`, first row) will bring its own questions. One is already visible: whether kinematic behaviors run first in Forces or last in Fields (`01-modules-and-scheduler.md`).

## Debt

**Defects (2026-10-01), fixed on branches awaiting a build slot:**
- The Vulkan step skips an attached behavior registry instead of refusing it. Branch `core/drone-medium`, `CORE-1`.
- The step recorder drops the last `substeps` barriers (its hand tally says 8 + S dispatches per substep; it emits 9 + S). Branch `core/barrier-count`, `CORE-2`.
- A `SPADE_VULKAN=OFF` install ships `sim/simulation.hpp` without the `compute/` headers it includes. Branch `core/vulkan-off-install`.

**Code that disagrees with itself:**
- The registered state is described as "frozen at 18" / an "18-entry walk" in `CONTRIBUTING.md`, `README.md`, `engine/CMakeLists.txt` and `compute/vulkan/{backend.hpp, state_mirror.hpp, state_mirror.cpp}`. It is 22, and growing is now allowed. `CONTRIBUTING.md` also says adding an array moves `kSnapshotVersion`; it moves the schema hash. The README and CONTRIBUTING are Test/Docs (restructure R4); the comments are Core's sweep.
- "Nine schedule kernels" (`compute/backend.hpp`, `compute/spirv_variants.hpp`): twelve are compiled with variants. "Nine pipelines" (`step_recorder.hpp`): ten. The latter is fixed on `core/barrier-count`.
- `compute/vulkan/backend.hpp` still describes `StepParams` as unconsumed.
- `core/rng.hpp` says a stream is "not (yet)" GPU-mirrored, but streams live inside mirrored rows.
- Stale "later task" and "inert until the registry lands" notes: `core/math_ops.hpp`, `state/layout.hpp`, `physics/schedule.hpp`.
- `objects/graph.hpp` says parents serialize by name; they serialize by index. `world/world_ref.hpp` says schema v1.
- `state/layout.hpp` says it is generated by slangc. It is hand-written and checked against slangc's reflection.

**Deferred on purpose:**
- `fnv1a64` belongs in a `core/hash.hpp`. It was left in `state/snapshot.hpp` because four golden digests depend on its output.
- The model registry is configuration and not in the blob, so a restoring caller re-registers models.
- The Vulkan path flushes the structural queue once per `step(n)`. That is sound while nothing queues from inside a step.
- Any new method that writes arenas owes a `mark_vulkan_dirty` call.

**Restructure R2, step 4:** sweep the KAT-citing comments in Core's code (comment-only, worktree branch). Not started.
