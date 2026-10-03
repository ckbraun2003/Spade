# Spade engine changelog

Changes to the v2 engine (`engine/`) and its tooling. v1 (`src/ include/ examples/ assets/`) is frozen. Spade was developed inside the KAT monorepo until 2026-09-28 and is now its own repository; KAT is a consumer (`docs/design/consumers.md`).

## Unreleased

### Module API, stage 2: the GPU chain is the schedule's (2026-10-03)

- **The Vulkan step records the compiled module schedule.** Each pass names a GPU recipe (`compute::GpuRecipe`), a built-in kernel, and may name only the recipe of its own CPU function. Any module set whose passes all have one runs on Vulkan in the schedule's order. A pass with none is refused at `Simulation::create()`, by name. `Simulation::vulkan_recorded_chain()` reports what was recorded. `compute::VulkanBackend::create()` now takes the GPU chain to record (`modules::gpu_passes()` of a compiled schedule) as a third argument.
- **`Simulation::vulkan_pass_durations_ns()` reports one named duration per GPU pass**, in schedule order (`PassDurationsNs::passes`, looked up with `find()`, which has no default), replacing the eight fixed fields. The bench keeps its six counter names, adds `gpu_pass.<module>.<pass>_ns`, and fails on a missing pass. Rotors and drag, and IMU and GNSS, now have a bracket each, so the two summed counters include one more timestamp mark than before.

### Module API, stage 1: modules and the scheduler (2026-10-02 to 2026-10-03)

- **Modules, compiled into one schedule.** A module (`sim/module.hpp`) is a name, a version and its passes. Each pass declares its phase, its placement, what it reads, writes or accumulates, and any `after` edges. `modules::compile_schedule()` orders a module set into the engine model's six phases, and refuses a conflict between two writers, an unknown quantity, a dangling edge or a cycle. `modules::standard_modules()` is today's engine as nine modules and compiles to the old pass order. The CPU step runs the compiled list.
- **`Simulation::create()` takes a module set.** A new last parameter, `module_set`, follows `backend` and defaults to `modules::standard_modules()`, so existing calls are unchanged. An invalid set is refused at `create()`. `Simulation::schedule()` returns the compiled schedule. Until stage 2, a set other than the standard one is refused on Vulkan.
- **Snapshot format v2, with the configuration identity.** The snapshot header now carries the configuration identity: an FNV-1a fold over the module set's names and versions and the compiled pass order. `Simulation::restore()` refuses a blob taken under another module set. A version-1 blob is refused with the version message, so snapshots written before this change cannot be restored (`docs/design/consumers.md`).
- **`physics/schedule.hpp` is pass functions only.** `kSchedule`, `run_substep` and `substep_schedule` are gone. The rotor/drag and IMU/GNSS pairs are now separate passes. Gravity and Publish, which were empty, are removed.
- **Merged as `062e1fa`.** Measured on the branch at `25f3c8a`: 959 tests, 0 failed, 2 skipped by design, all 70 GPU tests passed. No golden moved.

### The restructure (2026-10-01 to 2026-10-02)

- **A design library of Spade's own.** `docs/design/` now opens on a charter (laws `L1`–`L8`), an engine model (modules, fields, regions, responders, grades) and a realm map, with one library per realm: `core/`, `physics/`, `rendering/`, `interface/`, `test-docs/`. Every legacy ruling ID has exactly one home row in a realm's `00-decisions.md`. The KAT-era documents moved, unedited, to `docs/design/superseded/`.
- **`AGENTS.md` and `CLAUDE.md`**: short, broad guidance for anyone changing the code.
- **Test labels are `spade` and `gpu`.** The KAT-era `T0` tier label is gone; nothing here selected it (`TD-10`).
- **Scripts, presets, `.gitignore`, the bench baselines' notes and the pre-push guard** describe Spade in its own terms. `*.sh` checks out LF, so the guard can be installed by copying it.
- **Spade's suite no longer reads a KAT checkout.** The 30 render-agreement cases over KAT's worlds and the scene-drift guard moved out; 3 `AgreementProbe` cases keep the agreement machinery tested here. The bands are to be re-measured on Spade's own content (`docs/design/backlog.md`).
- **Two Vulkan-path defects fixed.** The Vulkan step now refuses a world with behaviors attached instead of silently skipping them. The step recorder now derives its barriers from the dispatches it actually emits: since GNSS it had been counting 8+S dispatches per substep against 9+S emitted, dropping the last `substeps` barriers. A test pins a barrier between every adjacent dispatch pair.
- **One sun convention.** The GL path lit scenes with `-sun_direction` while the CPU used `+sun_direction`, so world-loaded scenes were lit from below. It also hid inward-wound builder meshes, which were fixed in the same change (spheres and cylinders now wind outward, and a test checks every triangle).
- **`Simulation::sample_medium`**: a host-side read of a world's medium (density and wind, including the turbulence gust) at a point.
- **`vehicles::rotor_wake_velocity`**: an actuator-disc rotor wake for visualising the air a rotor moves. CPU-only, best-effort grade.
- **The drone sim box is the sandbox's default scene.** A quadrotor is held in place while the air moves around it, driven through the engine's rotor model. It has attitude keys, an orbit camera, an air-velocity heatmap view and a physics panel, and Vulkan is refused with a reason.
- **Measured at `df33f09`:** 921 tests on both presets, 919 passed, 2 skipped by design, 0 failed. All 65 GPU tests ran (`docs/design/test-docs/07-status.md`).

### Stands alone (2026-09-28)

- **Extracted from the KAT monorepo with its full history.** About 70 path citations lost their `spade/` prefix. KAT now builds against an installed Spade through `find_package(spade CONFIG)`.
- **A reach outside the repository, found and guarded.** The render agreement tests read KAT's content through a relative path that escaped this repository once it stood alone; they now skip loudly when that tree is absent, instead of failing.
- **A pre-push guard** (`scripts/pre-push-guard.sh`): nothing is pushed without `SPADE_PUSH_AUTHORIZED` set for that push.
- **A front-facing `README.md` and a `CONTRIBUTING.md`.**

### GPU rasterizer, stages 1–2 (2026-09-25 to 2026-09-27)

- **A Vulkan raster kernel and its host side**, written beside the CPU rasterizer. They are **in no build graph** and paused for the restructure (`docs/design/backlog.md`).
- Writing the host side found, and fixed, three colour defects in the stage-1 kernel (R and B swapped, the grid line shaded, unlit materials lit). The silhouette comparison could not see them, because it compares coverage and never colour. **Colour on this path is still unguarded.** An audit then found four more (a shading test that was the complement of the reference's, an unkept forward promise, a struct-stride footgun, a stale count), all fixed.

### GNSS (2026-09-21 to 2026-09-23)

- **A second sensor kind.** `sensors/kinds.hpp` holds the kind vocabulary, so a kind no longer lives inside the IMU's header.
- **A GNSS receiver** can be spawned, polled and despawned on its own clock (`add_gnss_sensor`, `poll_gnss`, …). Its state joins the registered walk.
- **Synthesis on both backends**, as a matched CPU/GPU pair. Nine of twelve compared quantities are bit-exact between backends. The rest sit inside bands pinned from measurement.

### The object model (Plan A, 2026-09-07)

**`kSnapshotVersion` did not change, and neither did any golden file.** That is
the fact most worth confirming up front: a structural addition this size cost
the determinism estate nothing, because the object graph is composition, not
registered state (SL3). Every scenario digest, every render golden and every
CPU↔GPU parity band is byte-identical to before.

#### Added

- **`spade::objects`** — the composition half of the ECS the engine design spec
  ratified and never built. `ObjectGraph` is a recycling slot pool with
  generational handles whose parity convention matches `BodyRef`'s (odd live,
  even dead, 0 never issued). See `engine/objects/README.md`.
- **Ten compile-time-registered component types.** The id *is* the serialization
  key, so ids are hand-assigned, monotonic and dense; a type with no
  `ComponentTraits` specialization fails to compile. `FluidComponent` is
  declared now, ahead of Plan B, so that adding SPH later cannot force a
  renumber — which would silently reinterpret every saved scene.
- **Object graph JSON serialization.** Components travel by registered name;
  parent links travel by array index, because object names are user data and
  duplicates are legal. Saving a graph whose child names a destroyed parent is
  refused rather than silently re-rooted.
- **The behavior registry (SL6).** Registration order is execution order; a
  behavior with no `record_gpu` makes the registry ineligible for the
  GPU-authoritative path — a refusal, never a silent fallback.
- **`kinematic_mover`**, the first declared behavior. Its pose is a closed form
  of `(params, tick)`, so a restored snapshot resumes exactly where the original
  run was. This is the mechanism the demo-mission program and the
  `tracking-moving-target` training structure were blocked on.
- **`DrawMode::velocity`** and `RenderOptions::velocity_scale_mps` (SL9c),
  closing the v1 `RenderVelocity`/`Velocity.frag` row. `BodyPose` now carries a
  world-frame velocity; `DrawItem` carries the derived speed.
- **`spawn_in_sphere` / `spawn_in_cube`** (SL9e), seeded from domain-separated
  `splitmix64` streams rather than v1's `std::random_device`-seeded mt19937.
  All-or-nothing: capacity is checked before any body is placed.
- **`Simulation::set_behaviors()`** and **`Simulation::body_capacity()`**.
- **`docs/v1-transfer-register.md`** — the SL7 register, machine-checked
  by `test_transfer_register.cpp`. One row remains open (SPH fluid); until Plan
  B closes it, v1 must not be quarantined.

#### Changed

- **The substep schedule is ten passes, not eight.** `BehaviorsKinematic` sits
  after `MediumUpdate` and before `ForceElements`; `BehaviorsForce` sits after
  `ForceElements`. Both are inert with no registry attached — which is every
  golden-corpus scenario, so the corpus keeps proving that inertness on every
  run. This is a change to a parity contract and it landed alone, with "every
  existing golden byte-identical" as its entire deliverable.
- The Vulkan step recorder deliberately continues to model **spec section 3's
  eight passes only**. SL6 refuses a behavior with no `record_gpu` half from a
  GPU-authoritative world, so the GPU-side shape is the registry's decision, not
  two no-dispatch slots added ahead of it. The divergence is pinned by
  `Schedule.RemovingTheBehaviorSlotsLeavesSpecSectionThreeExactly`, because
  nothing previously related the recorder's slot table to `kSchedule` — the
  schedule grew from eight to ten with every GPU test green.

#### Notes for anyone extending this

- A field belongs on `Object` only if it could be rebuilt from a saved
  description. Velocities, contact sets and accumulated forces are state; they
  live in a `spade_state` arena and a component references the slot.
- One invariant, one site. A recycled slot is reset in `create()` and nowhere
  else, so the reset can actually be shown to fail.
