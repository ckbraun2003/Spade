# Drone builder: the Spade engine side

**Owner:** lead, with Physics, Core and Interface. **Status:** approved by the user 2026-10-03, with the joint spec (Kat `c9683af3`, `design-specs/integration/drone-builder.md`).
**Joint spec:** KAT `design-specs/integration/drone-builder.md`. Kat owns the parts, the catalog, the builder and the compiler. Spade owns the physics and the world model.
**Physics sections:** `../physics/plans/2026-10-03-drone-builder-physics.md`.
**Rulings this rests on (user, 2026-10-03):** the joint spec, and the world/scene split (`../backlog.md`, "Ruled by the user, 2026-10-03").

## 1. Purpose

Kat builds an airframe from generic parts. Spade simulates it from physical part data, not from fitted curves. This page states what Spade provides at the seam and when. The physics models themselves are on the Physics page.

## 2. What Kat sends and what Spade runs

```
Kat part files ──Kat compiler (a template)──> Spade scene
                                               ├─ world reference (physics, environment, regions, static terrain)
                                               └─ placed objects (assets; vehicles with start poses and parts)
Spade scene ──compose (Spade utility)──> runnable world ──> Simulation
```

- Kat compiles an assembly of parts into Spade's public descriptions. In engine-model terms, the compiler is a template (`../01-engine-model.md`).
- Each part arrives with a mount pose in the design frame. Spade never sees slots, presets or brands.
- Kat sees only the design frame, which is the flight controller's frame. Spade keeps the design-to-principal rotation internal: the compile expresses mounts in the body frame, and host reads of a vehicle return the design frame.
- Tuning (rates, gains, idle) stays in Kat's flight package. Spade's motor model takes a command input.

## 3. World and scene

The user ruled that Spade mirrors Kat's split.

- A **world** holds physics and environment only: the module set, regions and their fields, gravity and medium, and static terrain.
- A **scene** holds a world reference plus placed objects: assets (tracks, posts, props), vehicles and their start poses.
- One world serves many scenes. Batching many worlds (`L8`) varies scenes over one module set.
- The editor saves scenes, which amends `INT-3`. The engine model's terms change with the same signature.

**Before objects take part in stepping**, a Spade utility composes a scene into one runnable world:
- an asset's static collider joins the world's SDF program, in scene order;
- an asset's visual joins the world's props;
- a vehicle becomes a spawn with its model.

The scene's order is part of the configuration hash, because SDF evaluation order and spawn order feed digests. After objects take part in stepping, the scene's objects map onto object-graph components, and composition stops being a merge.

## 4. Fields for sound and radio

Sound and radio analysis is designed now and built later.

- Each is a field: `acoustic` (pressure level per frequency band) and `rf` (power density per band).
- Emitter parts are components that add source terms: motors and props for sound; video transmitters and antennas for radio, with band, power and pattern.
- One provider per field sums the sources. The first tier is free-field propagation. Occlusion by the static SDF is a later tier.
- Sensor parts (microphone, radio receiver) read the field through sample points. Cameras draw it as a field channel (an overlay).
- These fields are best-effort, and stepping never reads them (`PHY-3`). A flight controller that reacts to radio is a later promotion, with its own grade.

The field registry therefore needs values that are small fixed arrays (bands), not only scalars and vectors.

## 5. Static evaluation

The builder shows live figures (all-up weight, thrust-to-weight, hover throttle, flight time, peak current). These are a view, never a gate.

- Each physics model exposes the pure functions the step calls. One function serves both uses, so the builder and the simulation cannot disagree (`TD-9`).
- Spade adds a steady-state solver for one motor, propeller and battery chain.
- Kat composes the figures from these. The composite-inertia utility gives mass, centre of mass and inertia.

## 6. Timeline

| Spade work | Needs | Order |
|---|---|---|
| Composite-inertia utility; the models' pure functions; steady-state solver | nothing new | can start after the joint spec is approved |
| Scene file and composition utility | the world/scene terms | with the joint spec; before the editor spec |
| Propeller tier (CT, CQ by advance ratio) | per-rotor data in the rotor module (module API stage 4); not a role | after stage 4 |
| Motor/ESC and battery modules | modules own state (stage 4) | after stage 4 |
| Acoustic and radio fields | field registry (stage 3), field channels | contract now; build later |
| Parts as objects with components | objects in stepping | with the editor step |

Module API stage 1 of 6 is in progress (`../core/plans/2026-10-02-module-api-plan.md`).

## 7. Requirements

- **DBE-001** A part description MUST give every value in SI units as fp32.
- **DBE-002** A part description MUST give its mount pose in the design frame.
- **DBE-003** The order of parts in a vehicle MUST be fixed, because it sets the fp32 accumulation order.
- **DBE-004** A table in a part (a motor curve, CT or CQ by advance ratio) MUST be a fixed-size sample set with a stated interpolation.
- **DBE-005** A part model MUST carry a version, and the version MUST enter the configuration hash.
- **DBE-006** A part description MUST NOT carry random seeds. Spade derives every noise stream from the world seed.
- **DBE-007** Each physics model MUST have a CPU implementation. A GPU implementation MUST pin its operation order, or the model MUST declare its GPU grade `absent`.
- **DBE-008** No model may break or fail. A model at a limit MUST clamp and MUST publish an over-limit flag.
- **DBE-009** The builder's figures MUST come from the same pure functions the step calls.
- **DBE-010** Composing a scene into a world MUST be deterministic, and the scene order MUST enter the configuration hash.
- **DBE-011** The research airframes MUST be rebuilt as part assemblies, and their measurements MUST become validation data with stated tolerances. Kat holds system data only, so the rebuild is an inverse fit within the parts' published ranges, validated on held-out flights.
- **DBE-012** A recorded run MUST carry the compiled airframe it used, through Spade's replay configuration.
- **DBE-013** A vehicle's pose and rates MUST reach Kat in the design frame. The principal-axis rotation and the centre-of-mass offset MUST stay inside Spade: the model type carries both, spawn converts design-frame starts, and a vehicle-state read converts back (DBP-45, DBP-46).
- **DBE-014** Kat MUST compile its scene into Spade's scene file. Spade's scene holds engine content only: a world reference, assets with poses, and vehicles with start poses and compiled models.
- **DBE-015** Part files use Kat's convention (+Z thrust, X layout). The compiler's one rotation into Spade's convention MUST be an axis permutation with signs, so it adds no rounding.
- **DBE-016** Kat runs the research-airframe fit on the Kat machine against Spade's published pure functions. The flight logs MUST stay on the Kat machine.

## 8. Open points

| Point | Owner | Note |
|---|---|---|
| Scene file schema | Core (schema) with Interface (editor) | Drafted and approved by Core as the base: `../interface/plans/2026-10-03-scene-file-draft.md` at `4294f9b` (SCN-001..009). SCN-006 is a model-registry identity in the snapshot header, which makes snapshot format v3 when built; it moves into ReplayConfig at the next deliberate golden regeneration |
| Engine-model and `INT-3` text | lead, Interface | Land with the joint spec's approval |
| Band layout for `acoustic` and `rf` | Physics, Core | Fixed per field, part of the field's type |
