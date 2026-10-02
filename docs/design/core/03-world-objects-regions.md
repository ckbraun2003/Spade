# Core — worlds, regions, fields and objects

**Owner:** Core. **Normative.** Where the code is today is in `07-status.md`.

## The world description

- **One construction path, two front doors** (`engine D7`). `WorldBuilder` (a fluent API) and the YAML world file both produce a `WorldDesc`. Both run the same validation, so the file cannot describe a world the builder cannot build. Every entry path runs `validate_world_desc`: builder, file, path reference and in-memory reference.
- **The world file is Spade's format** and is versioned by `world_version`. Schema v2 (`RS5`) added materials, lighting and props. A v1 file still loads, upgraded with defaults. `save_world_file` writes v2.
- **The file's rules.**
  - Every key is required.
  - An unknown key is an error at every level.
  - `world_version` is the only upgrade path.
  - Floats round-trip through `std::to_chars` / `std::from_chars` with 9 significant digits and never through locale-sensitive IO.
  - A save followed by a load is byte-exact.
- **Physics and appearance are separate.** The SDF program is a physics artifact. Materials, lighting, props and per-node materials are render-only. They do not enter the configuration hash and do not reach the step.
- **Target.** A world file will also declare regions, field-provider bindings and the module set. Each addition gets a schema version bump and a v(n−1) upgrade path.

## Regions and fields

A **field** is a named, typed quantity over space and time: gravity, air density, wind, flow velocity or pressure, temperature. A **region** is a volume with a priority that binds field providers. Core owns:

- **The field registry.** Field names, value types and units, and who provides and who reads each field. A module declares this; the scheduler uses it to put providers in the Fields phase ahead of readers.
- **Regions.** A region is the whole world, a box, or any SDF shape. Each region has a priority, and an explicit overlap rule resolves where regions meet. The whole-world region always exists, so every field has a value everywhere.
- **The sampling interface.** `sample(field, world, position)` returns exactly what the step's readers saw on the last substep. A host-side read never advances a provider.

Physics owns the providers: constant, analytic or procedural (Dryden), and solved (SPH). Rendering owns how a camera turns a field into a channel.

**Today** there is one field family, the medium (density plus wind). It has one provider per world, and its region is the whole world. `Simulation::sample_medium(world, pos)` is the first host-side sampling call (`07-status.md`).

## Objects and components

- **The object graph is composition and identity, never state** (`SL3`). It never influences buffer layout, pass order or any computed value. Components hold references to module-owned state slots, never the state itself. That is why adding the graph moved no golden digest.
- **Object.** A generational handle with a stable name, a transform, and zero or more components (`SL4`). Nesting affects transform composition only.
- **Component types have frozen ids** (`SL5`). The id is the serialization key, so ids are hand-assigned, dense and monotonic, and never renumbered. A new type is appended. Ten are declared today; `fluid` is reserved for the SPH provider.
- **Serialization.** JSON, keyed by the component type's registered name (stronger than the id `SL4` named). Slot references resolve at load.
- **Attach and detach queue to step boundaries** (`SL4`). They are structural changes, the same as spawn and despawn. The rule becomes enforceable when objects take part in stepping, and it must be in place by then.
- **Responders** are the components that sample fields and produce forces or measurements: rigid body, contact shape, aero tier, propulsor, buoyancy, IMU, GNSS, camera. The module that defines a responder owns its state and its pass (`01-modules-and-scheduler.md`).

## Templates

A template is a ready-made assembly of objects, components and parameters, such as a quadrotor or a car. It is built only on the public API and is never linked into the core (`L7`). Today's `ModelType`/`Quadrotor` layer (`engine D4`) is the first template in all but name. Physics owns its parts, and Interface owns the template catalogue.
