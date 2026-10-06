# Core — modules and the scheduler

**Owner:** Core. **Normative.** The target design, in the engine model's terms (`../01-engine-model.md`). Where the code is today is in `07-status.md`.

## Modules

A **module** is the unit of extension. Spade's built-ins (rigid body, contacts, drag, rotor, Dryden, IMU, GNSS) are modules written against the same API a developer uses. A module declares:

- **State.** The arrays it owns. Each has a name, a row size and an extent: one row per world, per body slot, per force element or per sensor, or a fixed number of rows per row of another array (a sensor's ring). The core registers them, so registered state is snapshotted, digested and mirrored to the GPU with no further work by the module (`02-state-and-snapshot.md`).
- **Attached rows.** A row that belongs to a body, such as a drag element, a sensor or a rotor, enters through one door, `attach_row`. The module's `validate` checks the spawn record, the row is reserved, and the module's `init` writes it at the next step boundary. A module may also initialize its rows when a vehicle spawns. Freeing is the core's: despawning a body frees its attached rows by declaration and zeroes their per-row children. A module has no `free` function until one needs more than a zero-fill.
- **Seeded streams.** Each random stream its rows hold, with its domain tag and the array that holds it. `create()` and `reseed()` walk these declarations, so no list of streams is kept by hand.
- **Configuration tables.** Float tables built from the model registry and rebuilt only when a model is registered. They are configuration, not state: not in the walk, the digest or the snapshot. A pass may only read one.
- **Scratch.** Rows a module's passes share within a substep. Scratch is not state: it is not walked, digested or snapshotted, and it is zero at every substep boundary by the module's own invariant.
- **Fields.** The fields it provides or reads (`03-world-objects-regions.md`).
- **Components.** The components it defines and what each one refers to.
- **Passes.** Each pass names its phase, what it reads, writes or accumulates, any `after` or `before` edge, and carries a CPU implementation. A GPU kernel is optional. A pass sees the arrays, tables and scratch it declares, whichever module owns them, and nothing else. A read of another module's array may be optional: when that module is not in the set, the read binds nothing and orders nothing.
- **Grade.** One per backend: reference, banded or best-effort (`L3`, `L4`).

Arrays, tables and scratch share one namespace in a set, and a stateful module's quantities name its own arrays, tables or scratch (`<module>.<name>`).

A `Simulation` has one **module set**, shared by every world in it (`L8`). Worlds differ in content, parameters and seeds, never in which modules run. The module set is configuration, so it is part of the configuration hash (`02-state-and-snapshot.md`).

## The step

- **`dt` is fixed** at `create()` and never changes (`L1`). A step is `substeps` substeps of `dt / substeps`.
- **The tick counts steps, not substeps.** `Simulation::step()` advances it once, after the last substep (`engine A9`).
- **Structural changes queue to step boundaries.** Spawn, despawn, attach and detach take effect between steps, never inside one. Nothing inside a step allocates.
- **Nothing in the step path reads a clock.** Randomness comes only from per-world, per-system streams derived from the world seed under a domain tag, each declared by the module that owns it (`02-state-and-snapshot.md`).

## The scheduler

Every substep runs six phases, in this order:

**Fields → Forces → Constraints/Contacts → Integrate → Sensors → Publish**

- **Placement.** A pass goes in the phase it names. Inside a phase, order comes from declared reads and writes and from `after` and `before` edges, then from module-set order. Two writers of one quantity, or a writer and an accumulator, need an edge between them. The resulting order is deterministic and is part of the configuration hash.
- **Order is a numeric contract.** fp32 addition is not associative, so two passes that accumulate into the same quantity must keep a fixed order. Moving one is a behaviour change, even when the algebra says otherwise.
- **One schedule, two backends.** The GPU dispatch chain is derived from the same schedule. There is no second, hand-kept table, and no hand-kept count of dispatches. Barriers are placed between adjacent dispatches as they are emitted (`CORE-2`).
- **Refuse, never skip.** If a pass has no GPU kernel and no declared fallback, a world that needs it cannot run on the GPU. `create()` or `step()` refuses with `Code::unavailable` (`L6`). It never runs the GPU chain without that pass.

## Today's passes as modules

When the scheduler lands, today's ten-pass array becomes built-in modules placed in phases. Every golden digest must stay where it is (`../backlog.md`, first row). That requirement fixes the order inside each phase:

| Today's pass | Phase | Note |
|---|---|---|
| MediumUpdate | Fields | Dryden advances once per substep |
| BehaviorsKinematic | first in Forces | A pose a behavior writes must be in place before anything reads it (`SL6`). Open: `07-status.md` |
| ForceElements | Forces | Rotors then drag, in that order |
| BehaviorsForce | Forces, after built-in elements | Behavior wrenches accumulate last (`SL6`) |
| Gravity | none | Gravity is a field that Integrate reads. The empty slot goes away (`engine A9`) |
| CollisionStatic, CollisionDynamic | Constraints/Contacts | Static, then dynamic |
| Integrate | Integrate | Symplectic Euler (`engine D1`, Physics) |
| SensorSynthesis | Sensors | IMU then GNSS. Their order is not a numeric contract, because they share no output |
| Publish | Publish | Becomes the frame-state copy point for rendering (`L5`) |

## Behaviors

A behavior is a small module with a single pass and no registered state. It declares a stable name, its phase position (kinematic or force), what it reads and writes, and a CPU implementation (`SL6`).

- **Today**, behaviors exist only on the CPU. On Vulkan, `step()` refuses an attached registry (`CORE-1`).
- **Declared read and write sets are mandatory** once the scheduler places passes by them. A set that is too coarse costs only parallelism. Undeclared access would cause silent divergence.
