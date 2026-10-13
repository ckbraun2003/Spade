# Spade engine changelog

Changes to the v2 engine (`engine/`) and its tooling. v1 (`src/ include/ examples/ assets/`) is frozen. Spade was developed inside the KAT monorepo until 2026-09-28 and is now its own repository; KAT is a consumer (`docs/design/consumers.md`).

## Unreleased

### Per-part model geometry and triangle meshes (2026-10-12)

The parts and meshes plan (`docs/design/core/plans/2026-10-12-parts-meshes-plan.md`; `CORE-6`, `CORE-7`): Part A, and Part B's Tasks B1 to B5.

- **A model's solid geometry, part by part** (`CORE-7`). `ModelType::parts` is a list of `PartDesc` (`vehicles/model_part.hpp`), empty by default. A part has a name, a free `role` label, an attachment (`PartAttach::body`, or `rotor` with the rotor's index), a mount pose in the body frame (a rotor part's in its hub frame), its geometry, a `PartSurface` (a wall, or porous with `inv_permeability` and `inertial_coeff`), and `min_feature`, the smallest feature a grid must resolve. A part carries no mass. Nothing in `step()` reads a part; `register_model()` validates and copies them once.
- **A part's geometry is exactly one of two forms:** an exact `SdfProgram` (`shape`), or a shared, immutable triangle mesh (`mesh`). `vehicles::part_sample()` reads either, in the part frame.
- **The posing rule** (`vehicles/part_pose.hpp`), the one CPU rule that stage 7's GPU raster will mirror:
  - `posed_parts()` poses each part by the body, and a rotor part by its hub and the blade azimuth ψ (radians, an input; Physics owns ψ). `PartFilter::rigid_only` leaves the rotor parts out;
  - `posed_program()` gives the wall parts as one world-frame `SdfProgram`, the form a solver takes for the static world SDF;
  - `sample_parts()` evaluates every posed part, wall or porous, mesh or SDF, in its own frame. The smallest distance wins, and a tie goes to the lower part index (`CORE-8` §5.2). It reports the owner, its surface kind and its wall velocity;
  - `part_wall_velocity()` is `v + ω × (x − com)`, plus the rotor's spin about its hub.

  `compose_transform` moved down to `world/builder.hpp`, byte for byte, so `vehicles/` never includes `scene/`. `spade::scene::compose_transform` still compiles.
- **The airframe compile emits parts.** `PartInertia` gains `geometry` (`PartGeometrySpec`: `source`, `name`, `role`), which `composite_inertia()` never reads. Each sphere, box, cylinder or tube record, and each shaped motor, ESC or battery block, becomes a body-frame wall part: one primitive in the record's frame, with `min_feature` the solid's smallest dimension. A propeller never does, and `GeometrySource::none` opts a record out. The reference quad compiles to ten parts.
- **`vehicles::parts_identity(model)`** (`vehicles/model_identity.hpp`) is the parts' own identity, the one rule for it (`TD-9`): a field-by-field fold, a mesh part by its content hash. `kNoPartsIdentity` (`0x1bffc7475bebbf17`) is a model with none. It is the same on MSVC and gcc. A consumer that must notice a geometry change pins the pair (`model_identity`, `parts_identity`). `model_identity` never folds parts (`CORE-7`, amended 2026-10-12). Stage 7 will fold `parts_identity` into the snapshot's registry identity, for a model with parts.
- **Triangle meshes** (`CORE-6`, `world/mesh.hpp`).
  - `make_mesh()` builds a `TriangleMesh` from a `MeshInput` with a declared unit. It stores fp32 metres, rounded once, and an FNV-1a content hash.
  - `mesh_sample()` and `mesh_eval()` give the exact distance through a two-level BVH, bit-equal to brute force, signed by angle-weighted pseudonormals. Corner angles use Spade's own `atan2`, in double, in the internal `core/detail/math_ops_f64.hpp` (`TD-3`: no libm).
  - `mesh_mass_properties()` (`world/mesh_mass.hpp`) gives the volume, centroid and inertia per unit density, in double, in a fixed order, bit-identical across toolchains. Nothing in the engine calls it yet.
- **Mesh sidecars and scene file v3.** A mesh lives in a `.spmesh` file (`world/mesh_file.hpp`: `save_mesh_file()`, `load_mesh_file()`), named in the scene by a relative path and its content hash, as a scene names its world. `PartDesc::mesh_file` holds that path. `scene_from_yaml()` gains a defaulted `base_dir`, and `load_scene_file()` passes the file's directory.
- **Scene versions follow the content.** A scene is written as version 3 only when a part holds a mesh, version 2 only when a model has parts, and version 1 otherwise. The reader reads versions 1 to 3. `tests/golden/scenes/parts.scene.yaml` is the first version-2 text, hand-written and reproduced on gcc.
- **What is refused now:**
  - a bad part, by its index, name and field: an empty or repeated name; an unknown attachment, a rotor out of range, or a rotor index on a body part; a non-finite mount or a zero orientation; both or neither of `shape` and `mesh`; a shape that does not validate, holds `node_materials` or is unbounded; an unknown surface kind, a negative or non-finite coefficient, or any coefficient on a wall; a `min_feature` that is not finite and positive. `ModelIssue` gains `code` and `Element::part`, and `validate()` returns the first issue's own code, so a shape deeper than `kMaxSdfDepth` keeps `capacity_exceeded`;
  - `posed_program()` with a posed wall part that is a mesh (`unavailable`, naming it: read it through `sample_parts()`), or a wall part with no geometry;
  - a mesh with no unit, or one that is not a closed, oriented, 2-manifold, non-self-intersecting solid with its shells correctly nested, by name and with counts. `make_mesh()` repairs nothing;
  - a sidecar whose content is not the hash the scene names (both hashes given), a corrupt or truncated sidecar, a mesh part read with no base directory, and a mesh part written with no `mesh_file`;
  - `parts` in a version-1 scene and `mesh` in a version-2 one (unknown keys, blamed on the file's own version), and a scene version past 3 (`schema_mismatch`);
  - in `check_airframe()`, an unknown geometry source, and a part problem, listed at its record as `parts[k].geometry.<field>`.
- **Import only** (`CORE-6`, amended 2026-10-12 by the user). There is no native part authoring in any UI: Kat hosts the builder, and Spade provides the analysis. Geometry comes from CAD meshes, and STEP through an offline converter. The readers come next: glTF (B6), binary STL and OBJ (B7), and 3MF (B8), then the STEP converter.
- **Nothing moved.** The standard identity (`0x3303cfc86821f502`), the one-body schema hash (`0x1f60a6fef22e254a`), the standard walk, every golden digest and every `model_identity` are unchanged, the reference quad's `0x9be262bdf825ba41` included, with its parts. Snapshot format v3 holds, and every stored v3 blob restores after its models gain parts. Every version-1 scene text is unchanged byte for byte, `gate_run.scene.yaml` included, and version 3 left every version-2 text as A3 writes it. One golden was added and none regenerated.
- **Merged** as `86a9c80` (B1), `1949df8` (B2), `6302fcb` (A1), `2e5e41f` (B3), `0503350` (A2), `4a39929` (A3), `b52d7e2` (B4), `0870cf0` (A4), `40b49b7` (B5) and `57772c8` (`parts_identity`).

### Module API, stage 4: modules own their state (2026-10-05)

- **Modules declare their arrays** (`ModuleDesc::state`, `ArrayDecl`). Each has a name, a row size and an extent: per world, per body slot, per force element, per sensor, or a fixed number of rows per row of another array (`per_row`, such as a sensor's ring). `compile_schedule` tables them in walk order and refuses, by name, a bad name, size, extent or owner, or a name declared twice. A stateful module's `<module>.<name>` quantities must name its own arrays, so the standard set's tokens changed:
  - `dryden.state` is `dryden.dryden`, and `rotor.state` is `rotor.rotors`;
  - `imu.state` is `imu.imu_sensors` and `imu.imu_ring`;
  - `gnss.state` is `gnss.gnss_sensors` and `gnss.gnss_ring`.

  A developer's module that names an old token is refused as an unknown quantity.
- **`Simulation::create()` registers module arrays from the declarations.** The core registers its own four. The four modules that carry the legacy marker (drag, dryden, imu, rotor) register before `replay_config`, and every other module after it, each in set order. `ArenaSet` gains `register_bytes()` (an untyped `register_array`), `typed<T>()` and `bytes()`. `Simulation::module_array()` finds a module array by name, and `module_rows<T>()` reads one world's rows of it.
- **The GPU mirror sizes its buffers from the declarations.** `compute::StepShape` gains `arrays`, one `StateArrayShape` per walk entry, which `create()` fills from `state_array_shapes()`. So `compute/` keeps no list of arrays, and a developer's array with no binding is allocated, uploaded and read back unbound. Sizes are counted in 64 bits: a buffer past 2^32-1 elements is refused with `capacity_exceeded`, naming it. A `StepShape` with no arrays is refused.
- **Attached rows go through one door, `Simulation::attach_row()`.** It checks the module's spawn record with its `validate`, reserves the row, writes its `body_slot` at once, and queues the module's `init` for the next step boundary. `add_drag_element`, `add_imu_sensor` and `add_gnss_sensor` are fronts over it, with unchanged signatures and messages. Rotors still arrive only with a vehicle. Despawning a body frees its attached rows by declaration, in walk order, and zeroes their per-row children. The IMU and GNSS function pairs collapse into this path (`CORE-4`).
- **Seeded streams are declared** (`ModuleDesc::streams`): a tag, the array whose rows hold it, and a reseed function. `create()` and `reseed()` walk the declarations, so a developer's stream is reseeded too. A tag is unique in the set and is never `world`.
- **A pass sees the state it declares** through `SubstepContext::state`: one `StateView` per declared access (`state/state_view.hpp`, read with `world_rows<T>()`), another module's array included. `QuantityAccess::optional` reads an array of a module the set may not hold; when it is absent, the read binds nothing and orders nothing. `PassDecl::before` mirrors `after`. The built-in passes keep their typed views.
- **Configuration tables and the vehicle-spawn hook**, for Physics' optional modules.
  - A configuration table (`ConfigTableDecl`) is a float table built from the model registry and rebuilt only by `register_model()`. It is not in the walk, the digest or the snapshot, and a pass may only read it. `Simulation::config_table()` returns one. It has no GPU copy until stage 6.
  - `ModuleDesc::vehicle_rows` lets a module initialize its own rows when `spawn(world, model, where)` places a vehicle. A bad request refuses the spawn and releases every slot it took.
- **Scratch** (`ModuleDesc::scratch`, `ScratchDecl`): per-body or per-world rows a module's passes share within a substep. It is not walked, digested or snapshotted, and it is zero at every substep boundary by the module's own invariant. On Vulkan a scratch names a fixed binding (`StepShape::scratch`). One with none is CPU-only, and a Vulkan set whose pass declares it is refused.
- **What is refused now:**
  - a set that omits one of the seven built-in arrays, or declares one with another row size, extent, owner, depth or spawn size;
  - a set that drops or retags the stream on `dryden`, `imu_sensors` or `gnss_sensors`;
  - a legacy array declared by a module without the legacy marker, and the marker on a module with any other array.
- **Nothing moved.** Every golden digest, the standard walk (22 entries, `replay_config` at 16), the schema hash (pinned for one body at `0x1f60a6fef22e254a`), the standard configuration identity (`0x3303cfc86821f502`), snapshot format v3 and every GPU band are unchanged. `Simulation::restore()`'s identity refusal now prints both identities in hex.
- **Tasks 1 to 7 merged as `eb57d4d` and `a698ea4`**; Tasks 7b and 8 are on `core/module-state`, rebased onto PHY-7, whose `contact_dv` is the mirror's twelfth hand-listed derived buffer (binding 27). Measured on the branch at `b7a120b`: 1386 tests on both presets, 0 failed, 2 skipped by design, all 108 GPU tests passed.

### Banded CPU-GPU parity: any Vulkan 1.1 device is admitted (2026-10-05)

- **The RTX 3060 Ti, and any device that cannot preserve fp32 denormals, now runs the Vulkan path** (`CORE-5`, the user's ruling of 2026-10-05). No kernel requests a denormal mode (`SPIR-V rule P3`), so each device runs them legally with its own default. Vulkan results are banded against the CPU (`TD-14`), not bit-identical; the CPU stays the reference for goldens and replays.
- **`Simulation::vulkan_device_report()`** returns the device, driver and float controls a Vulkan run uses (`compute::DeviceReport`); `compute::describe()` gives one line. `unavailable` on the CPU backend.
- **The one capability clause left is Vulkan 1.1**, because the kernels are SPIR-V 1.3. `vulkan_available()` and `VulkanContext::create()` refuse only that, by name.
- **`fp32_math` no longer depends on a device's denormal mode**: its three subnormal paths are integer on both twins, so no CPU result moved.

### Module API, stage 3: fields and sample buffers (2026-10-03)

- **Fields.** A module declares the fields it provides (`ModuleDesc::fields`): scalar, vec3, or a band array of 1 to 32 floats for the acoustic and RF fields to come. A pass reads or writes `field.<name>`. `compile_schedule` refuses:
  - a read with no provider;
  - two providers of one field;
  - a built-in field declared with another kind or unit;
  - a write outside the provider module or outside the Fields phase.
- **Gravity, density and wind are fields.**
  - The new stateless `environment` module (`environment.sample`) and the Dryden module's `dryden.sample` write each world's sample row once per substep, on both backends.
  - Rotors, drag and Integrate read it: the same values, bit for bit.
  - `Simulation::field_samples(world)` is a diagnostic read of a row.
  - `physics::integrate_bodies` gains an overload taking gravity by value; the `WorldParams` overload forwards to it.
- **The Vulkan chain is 11 + S dispatches per substep:** two field-sample kernels, `field_environment` and `field_dryden`. The bench reports them as `gpu_pass.*` counters.
- **The standard set's configuration identity changed** (a new module and pass), so a snapshot taken before this change is refused by `Simulation::restore` (`docs/design/consumers.md`).
- **Merged as `42b352a`.** Measured on the branch at `b425a2a`: 1013 tests, 0 failed, 2 skipped by design, all 83 GPU tests passed; the gcc-13 leg passed. No golden moved.

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
