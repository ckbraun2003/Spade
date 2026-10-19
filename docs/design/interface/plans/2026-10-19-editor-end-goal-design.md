# The editor's end goal: drones, objects, airflow and views, live

**Owner:** Interface. **Status:** draft, for the lead's review.
**Asked by:** Cameron (relayed 2026-10-19): "The end goal of this session is a live smoke of the Spade editor / sandbox with the ability to add drones, add other objects, add airflow, add different rendering schemas, etc. ... like unity but for high fidelity sims rather than games." Also Cameron, the same day: "there will be a default library of objects/content in spade ... There is no part authoring ... spade is not a CAD dupe."
**Inputs:** Core (the library, object and scene formats), Physics (the airflow region's semantics, quoted in §4), Rendering (the render-mode selector and its measured costs, §5), the lead's distinctions (view modes versus scene cameras; mesh obstacles are Physics' call), and a read-only inventory of the editor on master.
**Builds on:** the editor design (2026-10-05, EDT-001..021), the I1 harness design (LiveRun), and the live tour storyboard (INT-9).

## 1. Done when

A user, live in the editor, can:
- **(a) add a drone** from the default library or a model file, place it, arm it and fly it;
- **(b) add objects** from the library (props, obstacles, gates, floors) or by importing a mesh, and place them on the ground or a picked surface;
- **(c) add airflow:** an airflow region placed in the scene, running one-way around the scene's drones;
- **(d) switch each view's render mode:** shaded, wireframe, velocity, the exact SDF, the flow views and C_p, with one or two views side by side.

**The proof is one smoke chapter** (§6): a scripted editor session that does all four, each step a declared check. It is the tour's chapter 3 grown, not a separate end-to-end run, so each capability is tested on its own headlessly and once together on screen.

Never in scope: authoring geometry. Shapes come from the library or from imports (glTF now, STL/OBJ with B7, STEP through the converter later).

## 2. What exists today, and what is missing

### (a) Add a drone

| Exists | Missing |
|---|---|
| `AddVehicle` copies a model in for its vehicle alone (EDT-019). One built-in library drone, "quadrotor" (`make_quadrotor(drone_stand_params())`, hand-tuned). Placing marches the composed SDF and rests the drone on it (`place_armed`). | **Library files:** no model file reader (Core's `model_version 1` is in progress on `core/model-file`), no `assets/library/`, no Load from file (editor plan Task 13, EDT-020). |
| The compiled reference quad: `compile_airframe(reference_quad())` plus `airframe_propulsion_chain()`. Today it lives only in a test fixture. | **The reference quad as a library model file:** Core proposes a committed file, written by a generator and pinned by a golden test (§3). |
| Run mode (`EditorRun`): Run/Stop, Pause, Step. LiveRun's rotors turn through the propulsion chain (merged, 74b1552). | **Pilot controls:** a placed drone flies open-loop at its start's `rotor_omega`. The attitude controller, mixer and keys exist only in the drone box (`drone_sim.hpp`), for `QuadrotorParams`. The editor needs a flight controller for any model's rotor layout, driven by throttle, attitude and arm, with commands through the chain. |
| `append_vehicle_items` draws a hull, arms and discs from the rotor layout. | **The drone's real parts:** `render_parts` (`prepare_parts` / `bind_parts`) is not linked into the sandbox, and `visual_ref` is ignored. |
| | **Propulsion in a scene:** a scene's models carry no chain, so a placed library drone loses it. Core proposes a per-model `propulsion:` block (§4). |

### (b) Add other objects

| Exists | Missing |
|---|---|
| Four library primitives (box, sphere, cylinder, capsule), all collider-only SDF assets. `AddAsset`, `RemoveObject`, `DuplicateObject`, `MoveObject`, `SetAssetPose`; `SetColliderMaterial` (new, in the tour branch) recolours them. | **Library objects** (props, obstacles, gates, floors) as files: Core's proposed object file (`object_version 1`, exactly the scene's per-asset block), listed in a `library.yaml` index. |
| World edits: `AddProp`, `AddSpawn`, `AddTerrainPrimitive` and their sets and removes, through the edit text and `--edit`. | **UI for props and terrain:** the window reaches only "Add spawn point". The props list is read-only, terrain has no panel, and `new_world()` is never called. |
| glTF reading (`read_gltf`, in `spade::mesh_import`), `.spmesh` files (`save_mesh_file` / `load_mesh_file`), and mesh parts in scene v3. | **Import:** `spade::mesh_import` is not linked into the sandbox, and nothing in the editor reads or writes a mesh. |
| | **Drawing visuals:** the editor calls `scene_from_world(world, {})` with no meshes, so any prop or asset visual silently draws nothing. A mesh resolver is needed. |
| | **A mesh obstacle's contact** is OPEN (Physics' call): the world SDF has no mesh node. In AIRFLOW it is supported (P2's solids: a part with zero rigid velocity). |

### (c) Add airflow

| Exists | Missing |
|---|---|
| LiveRun, the drone box: one drone HELD at the centre of a box that follows it; one-way; rotors through the chain and P7a discs; save and restore (both merged, 74b1552). | **Anything in the editor:** `spade::analysis` is not linked into the sandbox; there is no airflow component, no region in the scene format, and no flow view. |
| Physics' P2 design: `set_solids()` takes any posed parts from any number of bodies, every step. | **P2 and P2b themselves** (Tasks 3-10, then P2b): the solids, and the rule from a flight to a region's faces. |
| | **A driven LiveRun:** LiveRun holds its drone. In the editor the drone flies, so the region needs the drone's state from the editor's Simulation each step, instead of the hold. |

### (d) Render modes per view

| Exists | Missing |
|---|---|
| Rendering's selector, merged: `ViewSpec{base, layers}`, `supports(spec, backend, inputs)` with a fixed `why_not`, `apply_view(..., ViewCache&)` returning one legend per layer, `view_base_name` / `flow_view_name`. GL's volume cache (one upload per generation) and live streamlines (`StreamlineOptions::live()`, traced once per generation). | **Any mode UI:** nothing in the sandbox sets a draw mode. The editor has one viewport, one `RenderOptions` and one camera. |
| CPU raster: every mode. GL: every mode except the exact SDF (refused, `unhonoured()` says "mode"). | **Per-view state:** a camera, a `ViewSpec` and a legend for each view, and a one- or two-view layout. |
| | **Sensor views** (depth, normals, segmentation, per-camera field channels): not built anywhere (Rendering's backlog). |

## 3. The default library

Cameron's ruling: a default library AND imports; never geometry creation. Its shape (Core's proposal, which the lead accepted):
- **Where:** `assets/library/` in the repo, installed with Spade, so Kat's builder shows the same defaults.
- **What:** a drone is a model file (`model_version 1`, the scene's per-model block, with its optional `propulsion:` block). An object is an object file (`object_version 1`, exactly the scene's per-asset block). One emit and parse each, Core's (TD-9).
- **The index:** a hand-written `library.yaml`, one entry per item: name, category, file, thumbnail. It is not a folder walk, so the order is explicit and a test can load every entry and pin its identities. Core owns the formats and the index's reader; Interface owns the categories and what the editor shows.
- **Placing copies the entry in**, so a scene stays self-contained and never refers to the library (EDT-012).
- **First entries:**
  - drones: the reference quad, from a committed model file. Core's generator writes it from `compile_airframe(reference_quad())` plus `airframe_propulsion_chain()`, and a test regenerates it and compares the bytes (TD-1), pinning its model, parts and propulsion identities;
  - objects: the four primitives, plus a gate, a floor tile, a pole and a wall, all SDF assets (§2 b);
  - the sandbox's hand-tuned "quadrotor" stays until the reference quad replaces it, then goes.
- **The editor's Library panel** lists the index by category, with thumbnails, plus an Import button (§2 b).

## 4. The airflow region

Physics' semantics, quoted:
- **What a region is:** "a world-aligned box, never rotated. Its minimum corner is grid_origin_world. It holds nx × ny × nz cubic cells of side dx." Each face is open (a pressure outlet, the default), a wall (no-slip, with its own velocity; a floor is the −y face as a wall), periodic (in pairs), or fixed inflow (the wind). The fluid is ρ and ν.
- **A following region** "is the drone box: the drone sits fixed in the box, and its velocity goes into the faces (Galilean)." It holds exactly one drone, the one it follows.
- **A fixed region** takes any number of drones, "re-pose every step", with one caveat: "moving immersed walls shed some grid-scale pressure noise where cells uncover. Fine for one-way views."
- **Rules, each refused by name:**
  - the grid's byte cap;
  - each disc's support inside the box;
  - a wire thinner than its cells;
  - leaking parts;
  - unbalanced fluxes;
  - a drone or part leaving its region (P2b);
  - the floor clearance (P2b);
  - overlapping regions: "nothing couples two grids".
  Announced, not refused: "R/dx ≥ 4 for a readable rotor flow, and U·h/dx ≤ 0.5".
- **Two-way (D4)** is "a coupling flag on the SAME region component", refused until D4 lands.
- **Faces from the flight:** "the region's wind and floor go through the SAME one rule", P2b's `resolve(region, wind, drone state)`, "so the editor never maps faces itself".

**The component** (scene data, Core's scene version bump; §7) holds:
- `kind: follow | fixed`;
- `follows:`, a vehicle name, for `follow`;
- the box: size, and for `fixed` its minimum corner;
- the resolution, `dx`, or cells across;
- the faces;
- the wind;
- the fluid;
- `coupling: one_way | two_way`.

The editor shows the region as a wireframe box, with its grid and R/dx from the same resolve LiveRun uses, and each rule's refusal verbatim in the inspector.

**Running it, in milestones:**
- **M2:** one `follow` region around one placed drone, through LiveRun's core in a DRIVEN mode: the editor's Simulation flies the drone, and each step LiveRun takes that body's state and rotor rows instead of its hold. The faces carry the drone's velocity once P2b's rule exists. Until then, a moving drone's region is refused by name ("a following region needs P2b's moving faces"), and the editor's first airflow runs with the drone held: the drone box, live in the editor.
- **M3:** `fixed` regions with several drones, after P2's `set_solids()`.
- **Later:** two-way (D4).

Static mesh obstacles join the airflow as zero-velocity parts. Their contact is refused by name ("contact with a mesh obstacle"), never passed through silently, until Physics' design lands.

## 5. Views

- **A view's render mode is editor state, not scene data** (Core, the lead). It lives in the editor's workspace: each view's camera, `ViewSpec` and legend. The scene stays what is simulated.
- **A camera PLACED IN THE SCENE** (a sim camera or sensor rendering pressure or velocity maps, the restructure design's "cameras in many modalities") is scene data, and a LATER step, after Rendering's sensor views.
- **The menu** lists every view kind with `supports()`'s answer for this view's backend and inputs (`has_volume`, `has_walls`, `has_sdf`, `gl_flow_ready`). A kind that can't be drawn shows its `why_not` verbatim instead of drawing blank. Each view draws its own spec onto the one world scene through `apply_view(..., cache)`, with ONE `ViewCache` shared across views and frames.
- **Layout:** one view, or two side by side, switchable. Rendering measured both, along with every other layout tried. A whole frame, all views to `glFinish`, at the live 32³, measured 2026-10-19 on an RTX 3060 Ti with the box loaded:

| mix | 1 × 1280x960 | 2 × 640x960 | 1 × 1920x1080 | 2 × 960x1080 |
|---|---|---|---|---|
| vortices (iso by speed, plus lines) | 13.1 ms | 15.0 ms | 17.8 ms | 18.8 ms |
| absorbing \|ω\| volume | 4.8 ms | 8.4 ms | 6.3 ms | 11.0 ms |
| slice + C_p | 2.4 ms | 4.4 ms | 2.4 ms | 4.7 ms |
| lines | 8.5 ms | 9.5 ms | 8.4 ms | 9.7 ms |

  Every mix fits a 60 Hz frame in every layout, so cost doesn't decide between one view and two. At 64³ the ranges are vortices 23-32 ms, the volume 8-17, slice + C_p 3-10 and lines 14-26.
- **Defaults:** one shaded view. "Split" adds a second view, defaulting to vortices when a region runs and wireframe otherwise. The legends (quantity, unit, colour map, the range drawn) come from `apply_view`.

## 6. The end-goal smoke

The tour's chapter 3, the editor, grows these steps. Each is a declared step with its checks (EDT-011); the existing 11 editor steps stay.

| Step | On screen | Checks |
|---|---|---|
| `editor.library` | The Library panel by category | lists-index (every `library.yaml` entry shown), thumbnails |
| `editor.add_drone` | The reference quad placed from the library | placed-at (where asked, resting on the picked surface), drawn (its parts, one drawable per part), propulsion-kept (its chain's identity survives the copy) |
| `editor.add_objects` | A gate and a wall placed; a glTF prop imported | object-added (Test/Docs' predicate: placed where asked, on the ground or the picked surface, and drawn, one drawable per object), import-named (the mesh's name and source shown) |
| `editor.fly` | Run: arm, throttle up, pitch | armed, climbs (height rises at throttle above hover), follows-attitude |
| `editor.airflow` | A region added around the quad; the run line in the band | region-resolved (grid, dx, R/dx shown, as LiveRun resolves them), publishes, run-line-shown |
| `editor.views` | Shaded, then wireframe, then vortices with streamlines, then slice + C_p; then Split | each-mode-drawn (`draw()` succeeded and nothing `unhonoured`, or the `why_not` shown for a refused kind), legend-shown, two-views |
| `editor.save_reopen` | Save the scene, reopen it | round-trip (the reopened scene's text equals the saved one; the region and the drone's propulsion survive) |

The capabilities are each tested headlessly first: the library index, placing, the flight controller, the region's resolve and refusals, the view specs, and the save round trip. The chapter proves they work together on screen.

**Known-open entries,** expiring as in the tour:
- "cp-needs-p2": C_p on the solid drone waits for P2;
- "follow-needs-p2b": a following region on a moving drone waits for P2b;
- "mesh-contact-open": a mesh obstacle's contact waits for Physics' design.

## 7. Needs, by realm

| # | Need | Owner | Depends on | Status |
|---|---|---|---|---|
| N1 | The model file, `model_version 1`, with `propulsion:` | Core | | in progress (`core/model-file`) |
| N2 | The object file, `object_version 1` | Core | N1's emitter style | proposed, accepted by the lead |
| N3 | `library.yaml` and its reader; the test that loads every entry and pins its identities | Core (format, reader); Interface (categories) | N1, N2 | proposed |
| N4 | The reference quad's committed model file, generator and golden | Core | N1 | proposed |
| N5 | Scene version bump: the `airflow:` block (§4) and a per-model `propulsion:` block | Core | N1; Physics' semantics (in) | proposed |
| N6 | `build_identity()` | Core | | in progress (`core/build-identity`) |
| N7 | P2's solids (`set_solids()`) | Physics | | Tasks 3-10 |
| N8 | P2b: `resolve(region, wind, drone state)` to the faces; the leaving and floor-clearance refusals | Physics | N7 | after P2 |
| N9 | Mesh obstacles' contact (a mesh SDF in the world) | Physics, with Core's world/ | | OPEN: a design after P2 |
| N10 | Two-way coupling (D4) | Physics | N7 | later |
| N11 | A mesh resolver for `scene_from_world`, so visuals and props draw | Rendering | | to assign |
| N12 | `render_parts` in the editor: drones drawn from their parts | Interface (link and bind); Rendering (owns render_parts) | | exists; to wire |
| N13 | Sensor views (depth, normals, segmentation, camera field channels); then scene cameras | Rendering; Core for the scene's camera block | | backlog; later |
| N14 | The Library panel (categories, thumbnails, place), Load from file, Save to library (Task 13) | Interface | N1-N3 | to build |
| N15 | Import (glTF now): read, write the `.spmesh` sidecar, add as an object or a drone's part | Interface | N2, N11 | to build |
| N16 | The props and terrain UI, and New world | Interface | | to build |
| N17 | A flight controller for any model's rotor layout, with throttle, attitude and arm, commanding through the chain; pilot keys in Run mode | Interface | N1 (for the chain) | to build |
| N18 | The airflow component's UI, and LiveRun's driven mode (the host supplies the body and rows) | Interface | N5; N8 for a moving drone | to build |
| N19 | Per-view state and the layout; the mode menu through `supports()`; legends | Interface | | to build (the selector is merged) |
| N20 | The end-goal chapter's checks and predicates | Test/Docs (predicates); Interface (steps) | the above | Test/Docs offered the object-added predicate |
| N21 | The library installed for Kat's builder | Core (install); Spade Host (Kat's side) | N3 | later |

## 8. The scene format (Core)

- **One scene version bump** adds, both optional:
  - `airflow:`, the regions of §4, so a scene without one is unchanged;
  - a per-model `propulsion:` block, in the model file's own encoding (one emit and parse).
- **Not scene data:** a view's mode, camera, legend and layout (editor workspace state).
- **Later scene data:** placed cameras and sensors.
- **Imports** need nothing new for drones: mesh parts with sidecars exist in scene v3. A mesh object (an obstacle) needs the object file (N2), and its contact waits for N9.

## 9. Order

- **M1, library and flying:**
  - N1-N4, N14;
  - the reference quad placed and flown (N17);
  - one view with the mode menu, raster modes only (N19).
  - Smoke: `editor.library`, `editor.add_drone`, `editor.fly`, `editor.views` (raster).
- **M2, airflow in the editor:**
  - N5, N18 with the drone held (the drone box live in the editor), then moving once N8 lands;
  - the flow views and Split.
  - Smoke: `editor.airflow`, `editor.views` (flow), `editor.save_reopen`.
- **M3, objects and imports:** N11, N15, N16; fixed regions with several drones (N7).
  - Smoke: `editor.add_objects`.
- **Later:** N9, N10, N13.

Each milestone is red, then green, with its headless tests before its smoke steps.

## 10. Decisions for the lead

1. **In M2, the editor's airflow runs with the drone HELD until P2b** (the drone box, live in the editor), then flying. The alternative, waiting for P2b before any airflow in the editor, leaves (c) dark for longer. I recommend held first.
2. **The flight controller (N17) is Interface's**, generalising the drone box's controller and mixer from `QuadrotorParams` to any model's rotor layout. It stays in the sandbox until a second consumer needs it, then moves to an engine module. I recommend this over a new Core module now.
3. **N11, the mesh resolver**, is assigned to Rendering, since it is `scene_from_world`'s missing input.
4. **The default layout is one view**, with Split for two. Cost doesn't decide it (§5); one view is simpler to read.
