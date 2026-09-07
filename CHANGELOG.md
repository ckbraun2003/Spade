# Spade engine changelog

Spade has its own upstream repository and is developed inside the kat monorepo
(24th spec SL2a). This file records changes to the **v2 engine** (`engine/`);
v1 is frozen.

## Unreleased — the object model (24th spec, Plan A of 3)

**`kSnapshotVersion` did not change, and neither did any golden file.** That is
the fact most worth confirming up front: a structural addition this size cost
the determinism estate nothing, because the object graph is composition, not
registered state (SL3). Every scenario digest, every render golden and every
CPU↔GPU parity band is byte-identical to before.

### Added

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
- **`spade/docs/v1-transfer-register.md`** — the SL7 register, machine-checked
  by `test_transfer_register.cpp`. One row remains open (SPH fluid); until Plan
  B closes it, v1 must not be quarantined.

### Changed

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

### Notes for anyone extending this

- A field belongs on `Object` only if it could be rebuilt from a saved
  description. Velocities, contact sets and accumulated forces are state; they
  live in a `spade_state` arena and a component references the slot.
- One invariant, one site. A recycled slot is reset in `create()` and nowhere
  else, so the reset can actually be shown to fail.
