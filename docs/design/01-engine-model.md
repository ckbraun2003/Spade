# Spade — engine model

**Owner:** lead (Core builds it). **Status:** signed by the user 2026-10-02 (approved 2026-10-01; `plans/2026-10-01-spade-restructure-design.md` §2).  World/scene split signed 2026-10-03 with the joint drone-builder spec. This is the target structure; each realm's `07-status.md` says how far the code is from it.

```
Simulation ─ fixed dt, scheduler, module set, backend
 └─ World × N (batched; one module set shared, content/params/seeds differ)
     ├─ Regions ── volumes that bind FIELDS
     ├─ Objects ── transform + components, some of which are RESPONDERS
     └─ Cameras ── objects with a technique and channels

Scene ─ a world file reference + placed objects (assets, vehicles, start states)
```

**World and scene** (user ruling, 2026-10-03, mirroring Kat). A **world file** holds physics and environment only: the module set, regions and their fields, gravity and medium, and static terrain. A **scene** holds a world reference plus the objects placed in it: assets such as tracks and props, and vehicles with their start states. One world file serves many scenes. A running world is a scene composed onto its world file (`plans/2026-10-03-drone-builder-engine-design.md` §3).

## Concepts

- **Module.** The unit of extension; Spade's built-in catalog uses the same API as a developer's module. A module declares:
  - the state it registers (snapshotted automatically);
  - the fields it provides or reads;
  - the components it defines;
  - the passes it contributes, each with a phase and its reads and writes;
  - a CPU implementation, optionally a GPU kernel, and a grade per backend.
- **Field.** A named, typed quantity over space and time: gravity, medium density, flow velocity or pressure, temperature. A provider supplies it at a fidelity:
  - constant;
  - analytic or procedural (Dryden turbulence is one);
  - solved on a grid (SPH lands here).
- **Region.** A volume (the whole world, a box, any SDF shape) with a priority. It binds field providers; an explicit rule resolves overlaps.
- **Object.** A transform plus components. A component refers to module-owned state and holds none itself.
- **Responder.** A component that samples fields where its object is and produces forces: rigid body, contact shape, aero at a tier, buoyancy, propulsor, sensors (IMU, GNSS, camera).
- **Fidelity tiers.** Interchangeable modules behind one responder interface. For aero:
  - T0 point drag;
  - T1 coefficient model;
  - T2 blade-element or panel;
  - T3 two-way coupled to a solved flow field.
- **Scheduler.** Each substep runs fixed phases in order: Fields → Forces → Constraints/Contacts → Integrate → Sensors → Publish.
  - Modules are placed by what they read and write.
  - Order within a phase is deterministic and part of the configuration hash.
  - The GPU dispatch chain is derived from the same schedule; there is no hand-maintained parallel table.
- **Publish.** A consistent frame state (poses, plus any fields that cameras request). Rendering reads only this.
- **Camera.** A technique (raster, ray-traced, ray-marched) plus channels (colour, depth, IDs, or any registered field — a pressure map is a field sampled by a camera). It acts as a viewport or as a graded, tick-stamped sensor.
- **Template.** A ready-made assembly of objects, components and parameters (quadrotor, car), built only on the public API and never linked into the core.
- **Grade check.** At `create()`, the world's grade is computed for the chosen backend and refused if it is below what the consumer requires.

## Obligations carried in

- Object-graph changes queue to step boundaries once objects take part in stepping.
- A module with no GPU kernel and no declared fallback cannot be placed in a GPU chain: refused, never skipped.
