# The Spade scene file: draft schema

**Owner:** Core owns the schema. The lead drafts it, with Interface for the editor's needs. **Status:** approved as the base by the lead (2026-10-03) and by Core (2026-10-03), with Core's changes applied. It aligns with Kat's scene shape when that arrives. **Joint spec:** `../../plans/2026-10-03-drone-builder-engine-design.md` §3 and `DBE-014`.

## What a scene is

The user ruled the split on 2026-10-03 (`../../backlog.md`).
- A **world** holds physics and environment. The world file is that half.
- A **scene** holds a world reference plus placed objects: assets, vehicles and their start states.

Kat's compiler writes scenes (`DBE-014`), and the editor saves them (`INT-3`, as amended). A Spade utility composes a scene into one runnable world (§3).

## The shape

YAML, with the world file's conventions (`world/world_file.hpp`):
- canonical text;
- floats written to round-trip bit for bit;
- one validation path;
- an error on any unknown, missing or duplicate key;
- a version gate.

```yaml
scene_version: 1
name: gate_run
world:
  file: ../worlds/flight_field.world.yaml   # relative to this file, forward slashes
  hash: "0x9c41d0e2a7b3f518"                # FNV-1a 64 of write(load(file)); see below
materials:                                  # appended after the world's palette
  - {name: gate_orange, base_color: [0.95, 0.45, 0.10, 1.0], shading: lambert}
models:                                     # compiled vehicle models, registered in this order
  - name: kat_5in
    version: 3                              # >= 1 (DBE-005)
    param_schema_id: 1
    visual_ref: kat/5in_frame
    design_to_principal: [1, 0, 0, 0]       # unit quaternion (w, x, y, z); always written
    body: {mass: 0.62, inertia_diag: [0.0021, 0.0038, 0.0023]}
    proxy_radius: 0.12
    rotors:                                 # fixed order (DBE-003)
      - {local_pos: [0.08, 0.0, 0.08], local_orient: [1, 0, 0, 0], spin_dir: 1,
         tau: 0.03, radius: 0.0635, thrust_coeff: 4.1e-6, torque_coeff: 6.2e-8}
    drag_bodies: [...]
    imu_mounts: [...]
assets:                                     # static scenery, in scene order
  - name: gate_1
    pose: {position: [0.0, 1.8, 0.0], orientation: [1, 0, 0, 0], scale: 1.0}
    collider: {sdf: ...}                    # the world file's SDF encoding, in the asset's frame
    visual: {mesh_ref: gates/ring_1500, material: gate_orange}
vehicles:                                   # in scene order
  - name: quad_1
    model: kat_5in
    start: {position: [0.0, 0.3, -8.0], orientation: [1, 0, 0, 0],
            velocity: [0, 0, 0], omega_body: [0, 0, 0], rotor_omega: 0.0}
spare: {bodies: 1, force_elements: 0, sensors: 0, contacts: 0}  # optional on read; always written
```

- **A model** is `vehicles::ModelType` plus two fields that ModelType gains (Core's ruling, 2026-10-03):
  - `version` (`uint32_t`, at least 1). It lands with the model-registry identity (SCN-006). Until then, the scene carries it and the loader validates it.
  - `design_to_principal`: the rotation from the design frame to the principal-axis body frame, identity when they agree. Kat's compiler knows it, because it writes mounts in the body frame. `body.inertia_diag` is the principal moments, as in `BodyTemplate`. The design-frame vehicle read (module-API stage 4) undoes the rotation (`DBE-013`).
- **A start** is `VehicleSpawn`, spelt as the world file spells poses.
- **An asset's pose** is `SdfPose` (`position`, `orientation`, `scale`), spelt as the world file's props and spawn points are. Its scale is uniform, as SDF evaluation needs.
- **The world hash** is FNV-1a 64 over the canonical text the world-file writer produces from the loaded world, `write(load(file))`, not over the file's raw bytes.
  - So a v1 world that upgrades on load still matches, and so does a hand-edited file with other comments or spacing but the same content.
  - It covers the world's own content only, not the meshes, textures or other files the world references by name. Those are render data and never reach the step (`L5`).
  - It is written as the scenario files write digests: a quoted `"0x"` plus 16 lowercase hex digits.
- **`spare`** reserves slots for objects that appear at runtime, such as a payload a vehicle drops. Its four counts match the world file's `capacities`. It is optional on read (each count defaults to 0) and always written, zeros included, so the bytes stay a pure function of the content.
- **Materials** are referenced by name, never by palette index. That holds for visuals and for an asset collider's SDF nodes.

## Composition

The scene composes into one runnable world, in this order:
1. **Load the world.** `write(load(file))` must hash to the scene's `world.hash`.
2. **Append the scene's materials** after the world's palette. A scene material whose name a world material already has is refused, so names stay unique across the combined palette.
3. **Add the assets, in scene order.**
   - Each collider is posed by its asset and joined to the world's SDF program by union. Its nodes' material names resolve against the combined palette.
   - Each visual becomes a `PropDesc`.
4. **Register the models in the `models:` section's order.**
   - Reordering vehicles therefore never renumbers a `ModelTypeId`.
   - An unused model is allowed: it is registered and never spawned, as when the editor stages one.
5. **Spawn the vehicles in scene order**, each with its start state.
6. **Size the capacities.** For each of the four fields (bodies, force elements, sensors, contacts), the composed value is the world's count, plus what the scene's contents need, plus `spare`.
   - Force elements are the rotors and drag bodies; sensors are the IMU mounts.
   - With no spares every new slot is live. With spares, a reader counts live bodies (`Simulation::live_body_count()`), not arena slots.
   - World file v3 revisits whether a world keeps capacities at all.

## Decisions

**The lead answered decisions 1–6 on 2026-10-03. Core ruled on the mechanism and the details the same day.**

1. **The world reference:** a path plus a content hash, refused on mismatch (`L6`). One world serves many scenes.
2. **Models:** inline, so a scene is self-contained and a recorded run carries its airframe (`DBE-012`). Kat's prefabs stay Kat's; its compiler writes them inline.
3. **Seeds:** none in the scene. A world instance's seed is a run parameter, so a batch can vary seeds over one scene (`L8`). The world's `environment.seed` stays the default.
4. **The configuration identity (Core's mechanism, SCN-006).**
   - A vehicle's parameters and its spawn order are already registered state from the first tick: rotor, drag, IMU and body rows. Every snapshot blob carries them and every state digest folds them in. A different vehicle list, order or parameter already changes the digest, and restore overwrites them.
   - What a blob does not carry is the model registry, which is configuration. Core lists it under "Deferred on purpose".
   - **Now:** the snapshot header gains a second identity, the model registry's: each model's full definition and version, in registration order. Restore refuses another, naming "model registry", apart from the module-set refusal. This is snapshot format v3, which reaches consumers. It lands with the scene composer or module-API stage 4, whichever comes first.
   - **Later:** both identities fold into `ReplayConfig` at the next deliberate golden regeneration, because `ReplayConfig` feeds every golden digest.
5. **The world's physics records.** Turbulence, contact and grid records belong to the world and land in Core's world file v3, with the module set and regions. The scene never carries them. Scenario files keep their per-instance values, which v3 turns into overrides of a world default.
6. **Frames.**
   - A start orientation is the design frame in Spade's axes (Y up). The compiler has already turned Kat's +Z-thrust axes with a signed permutation (`DBE-015`).
   - Each model's `design_to_principal` carries the rotation into the principal-axis body frame, and Spade applies it internally (`DBE-013`).

## What the editor needs from it (Interface)

- A bit-exact load, edit and save round trip, as the world file already has. The editor saves scenes, not worlds (`INT-3` as amended).
- Names unique across assets and vehicles together, since both appear in the editor's hierarchy and both become object-graph components later. A rename is then an edit, not a new object.
- Order that is visible and editable. Asset and vehicle order enter the configuration (the SDF program, the registered state), so reordering is a configuration change: the editor rebuilds, debounced (`INT-2`). Model order is fixed by the `models:` section, so reordering vehicles does not renumber models.
- Later, when objects take part in stepping, the scene's objects map onto object-graph components, and composition stops being a merge (§3). The schema should not need a version change for that, only a new composer.

## Requirements

- **SCN-001** A scene MUST name its world by a path relative to the scene file, with forward slashes. It MUST pin the world by FNV-1a 64 over the canonical text of the loaded world, `write(load(file))`. The hash MUST NOT cover files the world references. A mismatch MUST be refused with the cause.
- **SCN-002** A scene file MUST be canonical text: its bytes MUST be a pure function of its content, and every float MUST round-trip bit for bit. `spare` MUST always be written, zeros included.
- **SCN-003** Loading MUST reject unknown, missing and duplicate keys. It MUST validate through one function that the scene builder also calls.
- **SCN-004** Names MUST be unique within materials, within models, and across assets and vehicles together. A scene material MUST NOT reuse a world material's name.
- **SCN-005** Composition MUST add asset colliders and spawn vehicles in scene order, MUST register models in the `models:` section's order, and MUST be deterministic (`DBE-010`).
- **SCN-006** The model registry (each model's definition and version, in registration order) MUST enter the snapshot's configuration identity, and restore MUST refuse another (`L2`). The vehicle list and spawn order are registered state and enter every digest. Core owns the mechanism (`core/02`).
- **SCN-007** For each capacity field (bodies, force elements, sensors, contacts), composition MUST set the world's count plus the scene contents' need plus the scene's `spare` count. Each `spare` count MUST default to 0 on read.
- **SCN-008** A scene MUST hold engine content only: no slots, presets, brands or tuning (`DBE-014`).
- **SCN-009** Each model MUST carry `version` (at least 1) and `design_to_principal` (a unit quaternion). Materials MUST be referenced by name, never by palette index.
