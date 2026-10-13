# Per-part model geometry, and CAD input options

**Owner:** Core. Physics reviews the ModelType and airframe-compile changes, because `vehicles/` is Physics' code (`02-realms.md:8`).
**Status:** design-note draft for the lead. Docs only. Nothing edited, built or committed.
**Base:** master `80d564b`.
**The ask:** Cameron, 2026-10-12 (`backlog.md:66`): "The drone builder is going to be a mini drone cfd analysis system on top of the builder with cad design inputs and proper geometry for each part." The drone is a solid wall, with porous as an option. The engine is headed for F1-grade accuracy. The lead asked for a public per-part geometry on `ModelType` and an outline of the CAD input options.

## Summary

- **Shape.** `ModelType` gains `parts`. Each part has a name, a role label, an attachment (rigid to the body, or turning with rotor k), a mount pose, its own `SdfProgram`, and a surface (wall, or porous with its coefficients). Parts carry no mass. `BodyTemplate` stays the only mass the step reads.
- **Exact truth, derived samples.** A part's geometry is exact: an `SdfProgram` now, and a triangle mesh once the CAD choice is made. Sampled distance grids are never authored. The solver builds them at its own resolution, as derived buffers.
- **Blade azimuth ψ.** Today it is not stored and cannot be derived. For the demo, the host integrates it. In the engine it is a per-rotor array that Physics owns, declared by the module that reads it, so the goldens hold.
- **Identity** (amended 2026-10-12; accepted by the lead). `model_identity` NEVER folds parts. Parts have their own identity, `vehicles::parts_identity` (one rule, TD-9), which a CFD case pins beside `model_identity`. Stage 7 folds `parts_identity` into the snapshot's model-registry identity, only for a model with parts. Every existing identity and stored blob holds.
- **CAD.** Recommended: native CSG now (C); triangle meshes as the engine's one imported format (B); STEP through an offline tool (D). OpenCASCADE does not go into the installed library (A).
- **First step.** About three Core tasks: `ModelType::parts` with its refusals, a CPU posing rule, and the YAML. The demo harness then gets a solid drone (body, arms, motors) from the model. The props stay actuator discs.

## 1. The shape

### 1.1 Recommendation

```cpp
// vehicles/model_type.hpp (sketch)
enum class PartAttach : uint32_t { body = 0, rotor = 1 };  // later: wheel, hinge (F1)

struct PartSurface {
    enum class Kind : uint32_t { wall = 0, porous = 1 } kind = Kind::wall;
    float viscous_resistance = 0.0f;   // porous: 1/K, 1/m^2 (Darcy)
    float inertial_resistance = 0.0f;  // porous: C2, 1/m (Forchheimer)
};

struct PartDesc {
    std::string name;                                // non-empty, unique in the model
    std::string role;                                // a label: "body", "arm", "motor", "prop", ...
    PartAttach attach = PartAttach::body;
    uint32_t rotor = 0;                              // attach == rotor: whose azimuth turns it
    glm::vec3 mount_pos{0.0f};                       // part -> BODY frame, or -> rotor hub frame
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};
    SdfProgram shape;                                // the solid, in the part's own frame
    PartSurface surface{};
    // after the CAD choice: PartMesh mesh;  exactly one of shape / mesh non-empty
};

struct ModelType { /* ...today's fields... */ std::vector<PartDesc> parts; };  // empty by default
```

What the engine reads, and what it does not:

| Field | Read by | Note |
|---|---|---|
| `name`, `role` | people, the builder, the YAML | The engine never branches on `role`. A string keeps F1 roles (wing, floor, wheel) out of an engine enum (`L7`) |
| `attach`, `rotor` | the posing rule (§1.4) | `rotor` tells a solver which parts a rotor model replaces. The disc and actuator-line tiers drop them from the wall. The blade-resolved tier keeps them |
| mount | the posing rule | Rigid parts mount in the principal body frame, like every other mount (`model_type.hpp:215-217`). Rotor parts mount in their rotor's hub frame (+Y is the thrust axis), with ψ = 0 at the authored pose |
| `shape` | the solver, the renderer | In the part frame. The compile never rewrites it. Only the mount is converted |
| `surface` | the solver | §1.5 |

**Frames.** In Core's note of 2026-10-12, `ModelType::collider` was in the design frame (`2026-10-12-airflow-demo-core-inputs.md:66`). This note changes that. The stage-7 solver poses from `BodyState` on the device (centre of mass, body axes). `DBP-45` keeps `q_bd` and `c` in the model type, spawn and the read only. So `ModelType` stores body-frame mounts, and authors still work in the design frame. `AirframeSpec` takes design-frame mounts, and the compile converts each mount once, by its existing rule (`airframe_compile.hpp:38-45`). Moving a battery then changes mount poses, never part geometry. `parts` also replaces the single `collider`: the union of the wall parts is the collider.

### 1.2 Geometry representation

What extreme-accuracy CFD needs from the geometry:
- **Wall distance φ** at any point the solver asks for: apertures θ = clamp(0.5 + φ/dx, 0, 1) (airflow spec §2, `:200`); the wall model's sample 2–3 cells off the wall; and the link fraction for LBM interpolated bounce-back (§11.3, `:653`).
- **Normals** ∇φ/|∇φ|: to split slip from no-slip, and to integrate the pressure force ∮ p n dA.
- **A surface** (points, normals, areas): for force integration and the pressure-on-surface view.
- **Curvature:** for refinement by geometry (R2, `:665`).
- **A pose and a wall velocity** for every fluid step.
- **Determinism:** arithmetic and `sqrt` only, with pinned stencils (`TD-3`; `sdf.hpp:21-33`).
- **No resolution of its own.** The fluid grid and its nests set the resolution (R0–R2, `:657-668`). The geometry must not cap it.

**Blades are thin.** On a 5-inch prop (R = 63.5 mm), the chord is about 10–15 mm, the thickness about 1–1.5 mm, and the trailing edge about 0.2–0.4 mm (typical figures, not measured here). A sampled field at spacing h loses any sheet thinner than about 2h: both sides sample positive, so the blade vanishes. Edges round to about h. A dense 0.1 mm grid over one prop's box is about 2.4·10⁸ samples, roughly 1 GB in fp32. Only a narrow band is affordable, and even then the accuracy stops at h.

| Representation | Distance | Thin features | GPU today | Verdict |
|---|---|---|---|---|
| **`SdfProgram` per part** | Exact for every primitive except the heightfield (`sdf.hpp:31-32`). Through intersect, subtract and smooth_union it is a bound | Exact at any resolution, for what the primitives can make | Yes, `sdf_eval.slang` | **Now:** body, arms, motors, plates, holes |
| **Sampled SDF**, as a new `SdfPrim` (a narrow-band grid) | Trilinear, O(h²) on smooth surfaces | Lost below about 2h; edges rounded | Needs a third program-owned buffer (`sdf.hpp:99-101`), a new node layout in `compute/sdf_program.hpp` and `layouts.slang`, and a world-file change | **Not as authored truth.** Its accuracy is fixed before the solver's grid is known |
| **Exact triangle mesh per part** | Exact distance to the mesh (a BVH). The sign comes from pseudonormals stored with the mesh, so a query is arithmetic and `sqrt` only. A winding number needs `atan2` (`TD-3`), so it is an import-time check only | Down to the CAD export's tolerance | Not yet | **The CAD path** (§2) |
| **Both: exact truth, derived samples** | As the truth | As the truth | The derived grid is a plain buffer | **Recommended** |

The recommendation: each part holds exact geometry. A narrow-band grid, if the solver wants one for meshes on the GPU, is a derived buffer. It is a pure function of the part and the solver's spacing, rebuilt after `create()` and `restore()`. This is the derived class the airflow spec already defines (`:160`, `:171`). It is never authored, never stored in `ModelType`, and never identity on its own.

### 1.3 Attachment and blade azimuth

**One posing rule, on the CPU, in `vehicles/`.** Stage 7's GPU twin mirrors it (`TD-9`). B = (`pos`, `orient`) is read from `body()`.
- A rigid part: X = B · M.
- A part on rotor k: X = B · H_k · R_y(s_k ψ_k) · M, with H_k = (`local_pos`, `local_orient`) and s_k = `spin_dir`.
- Wall velocity at world point x: v + ω × (x − pos). On rotor k, add s_k Ω_k a_k × (x − h_k), with Ω_k = `RotorRow::omega`, a_k the world thrust axis and h_k the world hub.

Props, and outrunner motor bells, are rotor parts. A bell's shape does not change as it turns, but its wall velocity does.

**ψ is needed by the actuator line** (blade lines at ψ_k + 2πi/N) **and by the blade-resolved tier.** The disc does not need it.
- **Not derivable.** ψ = ψ₀ + ∫Ω dt needs Ω's history. `RotorRow` keeps only `omega` and `omega_cmd` (`rotor.hpp:546-547`). Its reserved row is held for the gyroscopic and BEMT terms and must stay 0 (`rotor.hpp:526-533, 551`).
- **One-way demo:** the harness integrates ψ_k += Ω_k Δt each frame, on the host, at display grade. No engine change.
- **Engine (stage 7, replay):** ψ must be walked state. Otherwise a restore resumes with the blades somewhere else (`L2`). It is Physics' to add, as a module-owned per-rotor array: stage 4's `ArrayDecl`, `per_row`, owned by `rotors`. The module that reads it declares it, so the standard walk, the schema hash and every golden hold. A field in `RotorRow` would move every golden (`2026-10-05-airflow-spec-core-answers.md:57`).
- **Update and seed:** ψ += Ω h, wrapped by subtracting 2π. Posing takes `sin32` and `cos32` (`TD-3`). The `vehicle_rows` hook seeds ψ at spawn.

### 1.4 Surface: wall or porous

- **Wall** (the default): a solid boundary moving at the wall velocity above.
- **Porous:** a Darcy–Forchheimer sink inside the part's volume, S = −(μ/K)·u_rel − ½ρC₂|u_rel|u_rel. With both coefficients at zero the part is transparent. That is the airflow spec's old real-time mode, momentum sources only (`:218`). One parameter therefore spans from transparent to nearly solid.
- **Per run:** whether a solver honours walls, or treats every part as transparent, is a run setting. That is Physics', not the model's.
- **Double counting:** a wall part inside a two-way region must not also feel its drag element (`:218`). Physics' rule, at stage 7. The one-way demo pushes nothing back, so nothing is counted twice.

### 1.5 Mass properties (TD-9)

Parts are the one source, in the compile's input:
- **One record per part.** In `AirframeSpec`, each part record holds its mass (today's `PartInertia`, `composite_inertia.hpp:54-61`) and its geometry.
- **Two outputs.** The compile derives the inertia (composite_inertia's rule, unchanged) and the `ModelType` part from that one record.
- **`ModelType::parts` carries no mass**, so the model never has two mass sources.

| Given | Geometry | Inertia |
|---|---|---|
| Inertia shape sphere, box, cylinder or tube | Derived exactly: SdfPrim sphere, box, cylinder (the tube's outer wall). Both use the +Y axis (`composite_inertia.hpp:43-50`, `sdf.hpp:27-30`) | As today |
| Inertia shape point or tensor | None | As today |
| Geometry only: one primitive | As given | Its closed form, uniform density |
| Geometry only: closed mesh (later) | As given | Exact by the divergence theorem, in double, in a fixed order |
| Geometry only: general CSG | As given | **Refused by name:** "give an inertia shape or tensor" (`L6`). No silent voxel estimate |

Mass is always given. A real motor is not of uniform density, and the tensor override stays. The prop's mass stays in the rigid body as a thin disc (`airframe_compile.cpp:178-184`). That disc is mass only. It is not a geometry part.

### 1.6 Identity and blobs

- **Fold only when non-empty.** With no parts, no count is written either, so every existing identity and stored blob holds.
- **Superseded 2026-10-12:** parts never enter `model_identity`. `parts_identity` is their identity now, and stage 7 folds it into the snapshot's registry identity (stage 7 design §3, "Geometry"). The bullets below describe the fold that `parts_identity` now implements; read "the fold" as `parts_identity`'s.
- **Fold only once a step reads the parts.** That is stage 7's in-Simulation solver, or a table build (Task 7's rule, `module.hpp:298-308`). Until then, the fold note says parts are not folded, beside `visual_ref` (`model_identity.hpp:27`). A test pins that adding parts leaves the identity alone.
- **The fold, when it comes:** after the IMU mounts, in declaration order. It folds each part's name, role, attach, rotor, mount, shape and surface. The shape's nodes and transforms are folded as the world file encodes them. `node_materials` is render data and is not folded.
- **The one-time change.** When the fold begins, part-bearing models change identity once. That includes the compiled reference quad, whose identity is pinned. Bundle it with stage 7's own identity change (`airflow-spec-core-answers.md:163`): one `consumers.md` line.
- **Mesh parts:** fold the mesh's content hash, computed once at `register_model`. Cache each model's identity there too. Today `snapshot()` and `restore()` refold the whole registry each time (`simulation.cpp:2355, 2533`). That is free for a few nodes and costly for megabytes of mesh.

### 1.7 Public API, YAML and validation

- **API.** The installed `vehicles/model_type.hpp` gains `PartDesc`, `PartAttach`, `PartSurface` and `ModelType::parts`. `ModelIssue::Element` gains `part`. The installed `airframe_compile.hpp` part records gain a name, a role, a shape and a surface. Kat's compiler fills them. One `consumers.md` line.
- **YAML.** Scene models are full `ModelType`s (`scene_file.hpp:106-108`), and the editor's library uses `model_to_yaml` and `model_from_yaml` (editor plan, Task 13). Both write `parts` only when the list is non-empty, so the `gate_run` golden holds byte for byte. A shape uses the world file's SDF encoding, as `SceneAsset::collider` does, behind `check_sdf_representable` (`yaml_text.hpp:629`). The writer emits `scene_version` 2 only when a model has parts. An older reader then says `schema_mismatch` ("upgrade") instead of "unknown key". Version-1 files read and write unchanged.
- **Refusals.** `invalid_argument`, naming the part's index and field:

| Field | Refused |
|---|---|
| `name` | empty; a duplicate of another part's name |
| `attach` | an unknown value |
| `rotor` | out of range for `rotor` attachment; non-zero for `body` attachment |
| `mount_pos`, `mount_orient` | non-finite; a rotation that is not orientable (today's rule, `model_type.cpp:105`) |
| `shape` | empty; anything `SdfProgram::validate()` refuses, with its message; non-empty `node_materials`. Depth over `kMaxSdfDepth` keeps `capacity_exceeded`, as `validate_scene()` does (`scene_file.hpp:129`). For that, `ModelIssue` gains a code, as `AirframeIssue` already has |
| `surface` | an unknown kind; a coefficient that is negative or non-finite; any coefficient on a wall |

### 1.8 Performance

- **No per-step cost.** Nothing in the step reads `parts`. `register_model` copies them once.
- **The demo.** The host poses the parts once per fluid step, at a cost of O(parts × transforms).
- **Stage 7** owns the rest: uploading the parts once per register, posing them on the device from body rows and ψ, evaluating them over each body's box every fluid step, and the derived grid. Its design is being drafted now.

## 2. CAD input options (for Cameron)

| | A. STEP/IGES through a B-rep kernel in the engine | B. Triangle meshes in the engine, exact distance | C. Native CSG in the builder, plus B | D. An offline tool: CAD to the engine's mesh |
|---|---|---|---|---|
| **Gives** | Exact NURBS. STEP assemblies with part names, placements and units. Exact mass properties | Every CAD tool and scanner exports meshes. Exact distance to the mesh. Exact mass properties. The surface for pressure integration and the pressure view. glTF and 3MF hold several named objects, so a file can be a part list | Parts built in the builder from primitives and CSG. Editable as parameters, GPU-evaluable today, nothing to import | STEP, IGES and native CAD through existing tools, outside Spade. Each part tessellated at a stated tolerance, with names, placements and units kept. The B-rep's own mass properties written alongside, as a cross-check |
| **Dependencies, licence** | OpenCASCADE: LGPL-2.1 with the OCCT exception. It would be the first copyleft dependency in the installed library, and a binary distribution must allow relinking. A large build on the MSVC and gcc legs and in the consumer smoke test | STL and OBJ: Spade's own code. glTF: nlohmann/json is already linked, but today's loader is render-only (`render/gltf.hpp`: `meshes[0]` only, no node transforms, no `.glb`) and lives in `spade_render`, which the step must not depend on (`L5`), so a reader goes lower. 3MF: zip and XML (miniz and pugixml, both MIT, both new) | None new | The kernel stays outside the installed library. A copyleft tool running as its own process leaves Spade's Apache-2.0 untouched |
| **Accuracy** | Exact at import. But no solver can query NURBS directly: exact point-to-surface distance is an iterative CPU projection, slow and not parity-safe. So it is tessellated anyway, at a tolerance, which gives D's output. OCCT's mesher must also be pinned and run single-threaded to repeat | The export's chordal tolerance, which the user sets; 1–10 µm is routine. Problems: STL and OBJ carry no units, so an import must name one or be refused. glTF is metres and +Y up by its spec, as Spade is. 3MF declares its units. STL is a triangle soup: weld it, then refuse open edges, bad orientation and degenerate triangles, by name and with counts | Exact for what primitives make. A twisted airfoil blade, a canopy or a moulded frame needs B | As B. The tolerance is recorded with the mesh, and the stored tessellation makes the tool's own repeatability irrelevant |
| **Effort** | Large: the build, the Docker leg, install and export, then a STEP-to-parts mapping. Several days, plus Test/Docs' build work (estimate) | Medium: a mesh type and its checks, BVH distance, mass properties, then readers one at a time (glTF, then binary STL). About 4–6 Core tasks (estimate) | Small for Core (the first step, §3). The authoring UI is Interface's | Small once B exists: a script and a page of docs |

**Recommendation: C now, B as the engine's one imported geometry, D for STEP. Not A in the engine.**
- **Why it is enough for extreme accuracy.** Every planned solver path uses φ, ∇φ and surface points. A mesh at a stated tolerance gives these to any accuracy asked for. The fluid grid and its nests set the resolution, sampling exact geometry, so the geometry never caps it.
- **Later, for F1.** STEP assemblies of hundreds of parts arrive through D as named parts. Wheels and DRS flaps become later attachment kinds. The validation bodies (Ahmed, NACA 0012) are meshes or primitives.
- **When to revisit A:** only if a solver needs exact NURBS queries at run time. None planned does.
- **Settled in the CAD spec after the choice:** how a mesh is stored. Core leans to a binary sidecar named in the YAML by path and content hash, as a scene names its world (`SCN-001`).

## 3. The smallest first step

**Core builds (about three tasks; Core's estimate, effort not dates):**
1. **`ModelType::parts`**, holding `SdfProgram` only, with the §1.7 refusals and `issues()` entries. Not folded. Physics reviews.
2. **The posing rule** in `vehicles/`:
   - `posed_parts(model, body, psi, filter)` returns one world-frame `SdfProgram`, the union of the chosen parts;
   - `part_wall_velocity(...)`;
   - the filter is `rigid_only` for the disc and actuator-line tiers, or `all`.

   It needs `compose_transform` moved from `scene/compose.hpp:53` down to `world/`, beside `transform_of` (`builder.hpp:64`). The move is byte-identical. It is required because `vehicles/` may not depend on `scene/`.
3. **`parts` in the scene YAML**, and in the model YAML when that task lands (§1.7).

**Physics' compile (DBP-44)** gains part geometry and surface, and maps inertia shapes to parts (§1.5). The reference quad's pinned identity holds, because parts are not folded yet.

**What the demo gets.**
- **The parts.** The harness appends the sandbox drone's parts to `make_quadrotor`'s model: a box body, four box arms and four motor cylinders, at `drone_view.hpp:125-128`'s sizes.
- **The wall.** Each fluid step, the harness poses the parts with step 2. It hands the solver a world-frame `SdfProgram`, the same form as the static world SDF. So the drone rides whichever solid path Physics' runner gains (plan A defers static solids to V2). Wall velocity comes from step 2.
- **The props** stay actuator discs, and the filter leaves them out of the wall.
- **One source.** Later, `drone_view` draws from the same parts (Interface), so the picture and the wall agree.

**What waits for the CAD choice:**
- mesh parts: the type, readers, checks, BVH distance, pseudonormal sign and mass from the mesh;
- the STEP tool;
- real blade geometry, for the blade-resolved tier;
- derived narrow-band grids (stage 7);
- the identity fold (stage 7);
- ψ (Physics, with the actuator line).

## 4. Open questions

| # | For | Question | Recommendation |
|---|---|---|---|
| 1 | Lead | Parts in the body frame in `ModelType`, with authoring in the design frame (`AirframeSpec`)? This changes Core's design-frame `collider` proposal of 2026-10-12 | Yes. The GPU poses from `BodyState`, and `DBP-45` stays as it is |
| 2 | Lead | Do parts replace the single `ModelType::collider`? | Yes. One record per part. The union of the wall parts is the collider |
| 3 | Lead, Physics | `role` as a free label, or an engine enum? | A label. The engine branches only on `attach` and `surface` (`L7`; F1 roles) |
| 4 | Lead | When to fold parts into the identity? | At stage 7, when the solver first reads them, bundled with stage 7's identity change in one `consumers.md` line. Folding now would move the reference quad's pin as soon as the compile emits parts, for no gain |
| 5 | Physics | ψ: host-integrated for the demo, and a module-owned per-rotor array in the engine, declared by the actuator-line or blade module? | Yes. Goldens and the standard walk hold |
| 6 | Physics | Porous as Darcy–Forchheimer per part, with zero meaning transparent; a per-run override; no drag element on a wall part inside a two-way region? | Yes. Physics decides the solver side |
| 7 | Lead | Should the compile emit geometry automatically from inertia shapes? | Yes, for sphere, box, cylinder and tube. One record, one rule (`TD-9`) |
| 8 | Cameron, via the lead | The CAD input formats (§2) | C now, B in the engine, D for STEP. Not A in the engine |
