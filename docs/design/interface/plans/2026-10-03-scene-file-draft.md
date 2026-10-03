# The Spade scene file: draft schema

**Owner:** Core owns the schema. The lead drafts it, with Interface for the editor's needs. **Status:** draft for the lead, then Core's review. It aligns with Kat's scene shape when that arrives. **Joint spec:** `../../plans/2026-10-03-drone-builder-engine-design.md` §3 and `DBE-014`.

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
  file: ../worlds/flight_field.world.yaml   # relative to this file
  hash: 9c41d0e2a7b3f518                    # FNV-1a 64 of the world's canonical text
materials:                                  # appended after the world's palette
  - {name: gate_orange, base_color: [0.95, 0.45, 0.10, 1.0], shading: lambert}
models:                                     # compiled vehicle models, by name
  - name: kat_5in
    version: 3                              # DBE-005
    param_schema_id: 1
    visual_ref: kat/5in_frame
    body: {mass: 0.62, inertia_diag: [0.0021, 0.0038, 0.0023]}
    proxy_radius: 0.12
    rotors:                                 # fixed order (DBE-003)
      - {local_pos: [0.08, 0.0, 0.08], local_orient: [1, 0, 0, 0], spin_dir: 1,
         tau: 0.03, radius: 0.0635, thrust_coeff: 4.1e-6, torque_coeff: 6.2e-8}
    drag_bodies: [...]
    imu_mounts: [...]
assets:                                     # static scenery, in scene order
  - name: gate_1
    pose: {position: [0.0, 1.8, 0.0], rotation: [1, 0, 0, 0], scale: 1.0}
    collider: {sdf: ...}                    # the world file's SDF encoding, in the asset's frame
    visual: {mesh_ref: gates/ring_1500, material: gate_orange}
vehicles:                                   # in scene order
  - name: quad_1
    model: kat_5in
    start: {position: [0.0, 0.3, -8.0], orientation: [1, 0, 0, 0],
            velocity: [0, 0, 0], omega_body: [0, 0, 0], rotor_omega: 0.0}
```

- A model's fields are `vehicles::ModelType`.
- A start's fields are `VehicleSpawn`.
- An asset's pose is `SdfPose`, so its scale is uniform, as SDF evaluation needs.

## Composition

The scene composes into one runnable world, in this order:
1. **Load the world.** Its hash must match the scene's `world.hash`.
2. **Add the assets, in scene order.**
   - Each collider is posed by its asset and joined to the world's SDF program by union.
   - Each visual becomes a `PropDesc`.
   - The scene's materials are appended after the world's palette.
3. **Add the vehicles.**
   - Each model is registered once, in order of first use.
   - Each vehicle is spawned in scene order with its start state.
4. **Size the capacities exactly from the scene:** bodies, force elements (rotors and drag bodies) and sensors. With no spare slots, every arena slot is live, which the viewer's invariant already needs.

## Decisions for the lead and Core

1. **The world reference.**
   - Recommended: a path plus a content hash, refused on mismatch (`L6`). One world serves many scenes.
   - Alternative: embedding the world, which makes one file per scene but copies the world into each.
2. **Models: inline or separate files.**
   - Recommended: inline, so a scene is self-contained and a recorded run carries its airframe (`DBE-012`).
   - Alternative: model files pinned by path and hash, which Kat may prefer for reuse across scenes.
3. **Seeds.**
   - Recommended: none in the scene. A world instance's seed is a run parameter, so a batch can vary seeds over one scene (`L8`). The world's `environment.seed` stays the default.
4. **The configuration hash.** `config_hash(WorldSetDesc)` covers the composed SDF program, so asset order is already in it. Vehicles are registered and spawned at runtime, so neither their models' versions nor their order is hashed today. `DBE-005` and `DBE-010` need both.
   - Proposal: fold the scene's vehicle list (model parameters and version, start states, order) into `ReplayConfig`.
   - This is Core's call; it sits on the module API's configuration work.
5. **The world's physics records.** Turbulence, contact and grid parameters live in `WorldInstanceDesc` today, set in code, and are not in the world file. Under the split they belong to the world, so they wait on Core's world file v3 (module set, regions), not on the scene.
6. **Frames.**
   - A start orientation is the design frame in Spade's axes (Y up). The compiler has already turned Kat's +Z-thrust axes with a signed permutation (`DBE-015`).
   - Composition applies the design-to-principal rotation inside Spade (`DBE-013`).

## What the editor needs from it (Interface)

- A bit-exact load, edit and save round trip, as the world file already has. The editor saves scenes, not worlds (`INT-3` as amended).
- Names that are unique per section, so the hierarchy can show and select them, and a rename is an edit, not a new object.
- Order that is visible and editable. Order enters the configuration hash, so reordering is a configuration change: the editor rebuilds, debounced (`INT-2`).
- Later, when objects take part in stepping, the scene's objects map onto object-graph components, and composition stops being a merge (§3). The schema should not need a version change for that, only a new composer.

## Requirements

- **SCN-001** A scene MUST name its world by a path relative to the scene file, and MUST pin the world by its content hash. A mismatch MUST be refused with the cause.
- **SCN-002** A scene file MUST be canonical text: its bytes MUST be a pure function of its content, and every float MUST round-trip bit for bit.
- **SCN-003** Loading MUST reject unknown, missing and duplicate keys. It MUST validate through one function that the scene builder also calls.
- **SCN-004** Names MUST be unique within each section.
- **SCN-005** Composition MUST add asset colliders and spawn vehicles in scene order, and MUST be deterministic (`DBE-010`).
- **SCN-006** The scene's vehicle list, with model versions and order, MUST enter the configuration hash (`DBE-005`, `DBE-010`).
- **SCN-007** Composition MUST size the world's capacities exactly from the scene.
- **SCN-008** A scene MUST hold engine content only: no slots, presets, brands or tuning (`DBE-014`).
