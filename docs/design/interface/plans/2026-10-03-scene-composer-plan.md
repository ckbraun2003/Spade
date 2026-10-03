# The scene composer: plan

**Owner:** Interface (the composer), with Core reviewing, because it touches the world model. **Status:** plan only. Nothing is built until the user approves the drone-builder joint spec (`../../plans/2026-10-03-drone-builder-engine-design.md`). **Schema:** `2026-10-03-scene-file-draft.md` (Core's; approved as the base). This plan covers `SCN-005` and `SCN-007`.

## What it does

The composer turns a scene and its world into one runnable world, then into a `Simulation`:

```
scene file ──load──> SceneDesc ─┐
world file ──load──> WorldDesc ─┴─compose──> ComposedScene ──instantiate──> Simulation + vehicle refs
```

`compose` is a pure function of its two inputs. `instantiate` registers and spawns in a fixed order. Both are deterministic (`DBE-010`).

## Where it lives (decision for the lead and Core)

Kat composes scenes too (joint spec §2: "Spade scene ──compose (Spade utility)──> runnable world"). So the composer cannot be sandbox code: `SL2b` keeps the sandbox off the installed surface, and Kat links only installed targets.

- **Recommended:** a new installed module, `spade::scene` (`engine/scene/`), exported under `SL2`.
  - `scene_file.{hpp,cpp}`: the schema's reader and canonical writer. The schema is Core's, so Core reviews this file.
  - `compose.{hpp,cpp}`: the composer. Interface owns it.
- **Alternative:** put both into `spade::world`. That module is Core's, and it would grow a dependency on `vehicles`.

## The interface

```cpp
namespace spade::scene {

struct SceneDesc;  // the parsed scene file (Core's schema)
[[nodiscard]] Result<SceneDesc> load_scene_file(const std::filesystem::path& path);
[[nodiscard]] Result<std::string> scene_to_yaml(const SceneDesc& scene);  // canonical (SCN-002)

struct VehiclePlacement {
    uint32_t model = 0;   // index into ComposedScene::models
    VehicleSpawn start;
};

struct ComposedScene {
    WorldDesc world;                            // assets, props, materials and capacities added
    std::vector<vehicles::ModelType> models;    // the models: section's order
    std::vector<VehiclePlacement> vehicles;     // scene order
};

// Pure. Refuses with the cause, never composes in part.
[[nodiscard]] Result<ComposedScene> compose(const SceneDesc& scene, const WorldDesc& world);

// load_scene_file, load the world it names, check the hash (SCN-001), compose.
[[nodiscard]] Result<ComposedScene> compose_file(const std::filesystem::path& scene_file);

struct SceneRun {
    Simulation sim;
    std::vector<VehicleRef> vehicles;           // scene order
};

// One world instance: the run's seed and the world's physics records (turbulence,
// contacts, grid) come from `instance`; its `world` field is replaced by the
// composed world. Registers the models in order, spawns the vehicles in order,
// flushes once.
[[nodiscard]] Result<SceneRun> instantiate(const ComposedScene& scene, WorldInstanceDesc instance,
                                           uint64_t dt_ns, uint32_t substeps,
                                           const compute::BackendDesc& backend = {});

}  // namespace spade::scene
```

Batching (`L8`) composes several scenes over one world, and builds a `WorldSetDesc` from them; `instantiate` is the one-world case.

## Composition, step by step

1. **The hash** (`compose_file` only): FNV-1a 64 of `world_to_yaml(load_world_file(path))` must equal `world.hash`.
2. **Materials:** append the scene's materials after the world's palette. A name the world already uses is refused (`SCN-004`).
3. **Assets, in scene order** (`SCN-005`). For each asset:
   - **Transforms:** each collider transform is composed with the asset's pose.
     - The new `world_to_local` is the collider's `world_to_local` times the inverse of the asset's pose matrix (translation, rotation, uniform scale).
     - The new `scale` is the collider's scale times the asset's.
     - The transforms are appended to the world's table.
   - **Nodes:** the collider's postfix nodes are appended with their transform indices offset. Then one `union_` node joins them to everything before them. The first contributor to an empty world program needs no union.
   - **Node materials:** each name is resolved against the combined palette. An unknown name is refused.
   - **Visual:** it becomes a `PropDesc`: the mesh reference, the asset's pose, and the material index resolved by name.
4. **Models**, copied in the `models:` section's order. Registration gives `ModelTypeId` *i* + 1 to `models[i]`.
5. **Vehicles**, in scene order. Each model name becomes an index; an unknown name is refused.
6. **Capacities** (`SCN-007`). For each field, the composed count is the world's count, plus the contents' need, plus `spare`.
   - Bodies: one per vehicle.
   - Force elements: each vehicle's model's rotors plus drag bodies.
   - Sensors: each vehicle's model's IMU mounts.
   - Contacts: no per-vehicle need. Core to confirm.
7. **One validation:** `validate_world_desc(composed.world)`, the function `WorldBuilder::build()` calls. It enforces the SDF depth limit (`kMaxSdfDepth`) and every other world rule, so the composer adds no second notion of a valid world.

## Determinism and identity

- **Order is fixed by the file.** The composer reads every list in file order and appends in that order. It never sorts and never uses a hash map's iteration order.
- **The transform composition is fp32 with a fixed operation order.** Its target compiles under `spade_fp_strict`, so gcc does not contract it into FMA.
- **Identity.** The composed SDF program is part of the `WorldDesc`, so asset order enters `config_hash`. The vehicles' parameters and spawn order are registered state, so they enter every state digest. The model registry enters the snapshot identity (`SCN-006`, Core's mechanism).

## Tests, when it is built

- **Round trip:** `scene_to_yaml(load_scene_file(f))` reproduces `f` byte for byte.
- **Union semantics:** at sample points, the composed SDF equals min(world SDF, asset SDF in the asset's frame × the asset's scale).
- **Identity pose:** an asset at the identity pose leaves its transforms bitwise unchanged.
- **Order:**
  - swapping two assets changes `config_hash`;
  - swapping two vehicles changes the first state digest;
  - reordering vehicles does not renumber models.
- **Capacities:** the formula per field, and `spare` defaulting to 0.
- **Refusals**, each naming its cause: a hash mismatch, a duplicate material name, an unknown model or material name, the SDF depth limit.
- **Determinism:** composing twice gives byte-identical `world_to_yaml(composed.world)`.
- **A successor check worth taking:** the eight viewer scenes, written as a world plus a scene, must reproduce their trajectory goldens (`tests/golden/viewer/`). That is `SL14b`'s functional axis, met by the composer's first real use.

## Dependencies

- **The user's approval of the joint spec.** Nothing is built before it.
- **Core:**
  - `ModelType` gains `version` and `design_to_principal`;
  - the snapshot's model-registry identity (`SCN-006`, snapshot format v3);
  - world file v3 for the physics records, later. Until then they come from `WorldInstanceDesc`, as today.
- **The editor:** it loads and saves through `scene_file`, and `INT-3`'s text is amended with the joint spec.

## Tasks, after approval

1. The `spade::scene` module and its install rules (`SL2`). The consumer smoke links it.
2. `scene_file`: reader, canonical writer and round-trip tests. Core reviews the schema code.
3. `compose`, with the tests above.
4. `instantiate`, porting the viewer's setup order (`engine/tools/viewer/setup.cpp`).
5. The viewer scenes as scene files, asserting their goldens: the successor check.
