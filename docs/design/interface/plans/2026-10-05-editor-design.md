# The editor, first cut — design

**Owner:** Interface. **Status:** draft for the user's approval (`INT-3`); the decisions in §9 are his, and nothing is built from this until he approves it. **Builds:** `../01-editor.md` ("The target") and `../../backlog.md` ("Sandbox → editor"). **Rulings this rests on:** `INT-2` (rebuild debounced, carry state), `INT-3` as amended 2026-10-03 (the editor saves scenes; worlds stay world files), `SL2b` (only the installed API), `SL15b` (headless first), the world/scene split (`../../01-engine-model.md`). **Inputs:** the scene file (`2026-10-03-scene-file-draft.md`, `SCN-001`–`SCN-009`, built), the composer (`2026-10-03-scene-composer-plan.md`, built), the module-API design (`../../core/plans/2026-10-02-module-api-design.md`), the drone-builder joint spec (`../../plans/2026-10-03-drone-builder-engine-design.md`, `DBE-013`, `DBE-014`).

## 1. Goal and done-when

The sandbox becomes Spade's editor: the place a developer opens a scene, changes what is in it, runs it, and saves it. A scene is the engine's own format (a world reference plus assets and vehicles), so the scene the editor saves is the scene Kat sends and the scene a batch run composes.

**Done when:**
1. The editor opens any valid scene file, and saving it unchanged writes its canonical text, so a file Spade or Kat wrote comes back byte for byte (`SCN-002`).
2. A user can place, select, move, re-pose, duplicate and delete assets and vehicles, and every edit can be undone.
3. Run, pause and single-step work on the edited scene, through `compose()` and `instantiate()`; an edit while running rebuilds the run, debounced, and carries vehicle state across (`INT-2`).
4. Save writes a valid scene file that `compose_file()` composes; an invalid scene is never written.
5. Every operation in 1–4 runs headless from the command line and is tested without a display (`SL15b`).
6. The live smoke tours the editor as a main function, so its recording shows each of the above.

## 2. Scope

| In the first cut | Specified here, built later |
|---|---|
| A document: one scene, its selection and its undo history | Components and modules on scene objects (with objects in stepping, module-API §11) |
| Scene sources (`SL13`): built-in scenes, test scenarios and saved scenes, all as scene files (§9 D3) | Editing a world (its environment, regions, terrain) — see §9 D4 |
| Assets: primitive colliders (box, sphere, cylinder, capsule) with a visual | Asset browser over meshes and materials |
| Vehicles: placed from the scene's models, or from the quadrotor template | Editing a model's parameters (Kat's builder owns airframes) |
| Run, pause, single-step; debounced rebuild with carry | Scrub with snapshot restore |
| Save, Save As, Open, undo and redo | CPU/Vulkan lockstep view, physics debug draw, SDF slices |
| The hierarchy, the inspector, the scene settings | A component menu (needs Core's availability table, module-API stage 5). It lists every type from that table and shows `fluid` as reserved with its reason, never as an option (`../01-editor.md`) |

## 3. The document

The editor edits a **document**, never the running simulation.

- **The document is a `scene::SceneDesc`** plus the path it came from, the selection, and the undo history. It holds no engine state of its own, so what the user edits is exactly what is saved.
- **The world is referenced, not edited.** The document loads the world its scene names (relative to the scene file, by hash, as `compose_file()` does) and shows it, but its edits change only the scene.
- **Every edit is a function** `Result<void> apply(Document&, const Edit&)` over the `SceneDesc`, in a display-free header (`sandbox/editor_document.hpp`). It validates the result with `validate_scene()` before committing it: an edit the scene file could not carry is refused with the validator's message and leaves the document unchanged.
- **Undo and redo** keep whole `SceneDesc` copies, one per committed edit. A scene is small (a world reference plus tens of objects), so copies are cheap and an undo can never disagree with the edit it reverses. The history is bounded (a fixed count, stated in code).
- **Names are the identity.** Assets and vehicles are addressed by their unique names (`SCN-004`), so a selection, an undo and a carried vehicle state survive a reorder.

## 4. Editing

Each operation is an `Edit` the window's widgets and the command line both produce.

- **Assets:** add (a primitive at a pose, with a collider of that primitive and a visual of the matching mesh and a material), select, move (the builder's ground drag), set pose (position, orientation, uniform scale), rename, recolour (a scene material), duplicate, delete, reorder.
- **Vehicles:** add (a model and a design-frame start pose), select, move, set start (pose, velocity, rates, rotor speed), rename, duplicate, delete, reorder. A vehicle's start stays in the design frame, as the scene holds it (`DBE-013`).
- **Models:** a vehicle may use any model the scene declares. The editor can add the quadrotor template's model to a scene that has none (§9 D2). It does not edit a model's parameters.
- **Materials:** add or rename a scene material; a name the world's palette already has is refused at compose time, and the inspector says so when it happens.
- **Order is visible.** The hierarchy lists assets and vehicles in scene order, because order enters the configuration hash (`SCN-005`, `DBE-010`); reordering is an edit like any other.
- **The builder's interactions carry over** (pick, drag, place on the ground, delete, duplicate), now acting on scene assets rather than render-only objects.

## 5. Running

- **Run** composes the document (`compose()` on the scene and its loaded world), instantiates it (`instantiate()`), and steps it at the fixed step the drone box uses. Pause stops stepping; single-step advances one step.
- **An edit while running rebuilds** (`INT-2`): when the user lets go of a control, the editor composes and instantiates the edited scene, and carries each vehicle's state by name (pose, velocity, rates, rotor speeds) through `vehicle_state()` and the new spawn. A vehicle the edit removed is dropped; one it added starts at its start pose.
- **A refused rebuild keeps the running simulation** and shows the reason (a `compose()` or `create()` error, verbatim) and leaves the document edited, so the user can fix it. Running never changes the document.
- **Structural edits on a running world** (`../01-editor.md`'s open item) are settled for this cut by the rule above: the editor never mutates a running `Simulation`; it rebuilds from the document. Queued structural edits wait for objects in stepping (module-API §11).

## 6. Saving and opening

- **Save** validates and writes the document with Core's canonical writer (`scene_to_yaml()`), atomically (a temporary file renamed over the target), so a failed save never leaves a half-written scene. The scene's world reference keeps its path and hash.
- **Save As** to another folder rewrites the world path relative to the new location, so the saved scene still finds its world (`SCN-001`).
- **Open** reads with `load_scene_file()`, loads the named world, and refuses with the cause when the world is missing or its hash differs; the user can re-point the scene at another world, which re-pins the hash (§9 D4).
- **Round trip.** Saving an unedited scene writes its canonical text (`SCN-002`), so a file Spade or Kat wrote comes back byte for byte, and a hand-edited one comes back canonical with the same content. A Kat scene opened and saved stays a Kat scene: the editor adds nothing that is not engine content (`SCN-008`).
- **Scene sources** (`SL13`, as amended by `INT-3`): one picker over the built-in scenes (the drone box and the builder's ground), the test scenarios, and saved scenes, all as scene files. A test scenario joins the picker when it is a scene file; composer task 5 converts the viewer scenes first (§9 D3).

## 7. What it needs from the installed API

| Need | Today | Owner |
|---|---|---|
| Load, validate and write a scene; world hash | `scene::load_scene_file`, `validate_scene`, `scene_to_yaml`, `world_hash` | built |
| Compose and run | `scene::compose`, `compose_file`, `instantiate` | built |
| **Save a scene file atomically** | none; `save_world_file()` exists for worlds | **finding for Core:** `save_scene_file(const SceneDesc&, path)`, the counterpart of `save_world_file()` |
| Carry vehicle state across a rebuild | `Simulation::vehicle_state()` (design frame) | built |
| **Spawn a vehicle with a full state**, not only a start pose | `VehicleSpawn` carries pose, velocity, rates and one rotor speed | **finding for Core:** per-rotor speeds on spawn, or a documented single-speed carry |
| Pose an asset | `transform_of()`, `SdfPose` | built |
| Pick an asset under the cursor | `eval()` and `gradient()` on an SDF program | built; the editor marches the composed world's SDF |
| Draw the scene | `render::scene_from_world`, `render_gl` | built |
| Meshes for visuals | `render::load_gltf` | built; the first cut draws primitives only |
| List component types and their availability | none | module-API stage 5 (Core); not needed until components |

Each finding is reported to Core, and nothing widens an include path (`SL2b`).

## 8. The window

The window stays a dumb shell (`SL15b`): it turns input into `Edit`s and draws panels over the document.

- **The hierarchy:** the scene's assets and vehicles in scene order, with the selection.
- **The inspector:** the selected object's pose, collider or model, material and start state. A field commits on release (`INT-2`).
- **The scene panel:** the world it references (path, hash, whether it matched), the scene's materials and models, and `spare`.
- **The run bar:** run, pause, step, the tick, and the last refusal if there was one.
- **Menus:** Open, Save, Save As, Undo, Redo, and the scene sources.

## 9. Decisions for the user

Each is one question with options; the first is recommended.

- **D1. First-cut scope.** (A, recommended) The scene editor of §§3–6: assets, vehicles, run, save. (B) A plus choosing a module's grade on the running world — waits on module-API stage 5. (C) A viewer that opens and runs scenes, with no editing.
- **D2. Where a vehicle's model comes from.** (A, recommended) Models the scene declares, plus the quadrotor template the editor can add. (B) Only the scene's models; vehicles need Kat. (C) A plus editing model parameters in the editor.
- **D3. Scene sources in the first cut.** (A, recommended) `SL13`'s three: built-in scenes, test scenarios and saved scenes, each as a scene file; each test scenario joins as it is converted. (B) Built-in and saved scenes only, with test scenarios later — this narrows `SL13` for the first cut. (C) Saved scenes only.
- **D4. Worlds.** (A, recommended) The editor references worlds and never writes one; re-pointing a scene at another world is allowed. (B) A plus "new world" (environment and ground) saved as a world file. (C) A plus editing the referenced world.

## 10. Requirements

- **EDT-001** The editor MUST edit a scene document and MUST NOT mutate a running `Simulation`; a run is rebuilt from the document.
- **EDT-002** Every edit MUST be a display-free operation, exercised headless and by the command line (`SL15b`).
- **EDT-003** An edit MUST pass `validate_scene()` before it commits; a refused edit MUST leave the document unchanged and show the cause.
- **EDT-004** Every committed edit MUST be undoable and redoable, to the history's stated bound.
- **EDT-005** Saving an unedited scene MUST write its canonical text, so a canonical file MUST come back byte for byte (`SCN-002`).
- **EDT-006** Save MUST write only a valid scene, atomically, through Core's canonical writer.
- **EDT-007** An edit while running MUST rebuild debounced and carry each surviving vehicle's state by name; a refused rebuild MUST keep the running simulation and show the reason (`INT-2`).
- **EDT-008** Objects MUST be addressed by name; order MUST be shown and edited as configuration (`SCN-005`).
- **EDT-009** Vehicle starts MUST stay in the design frame end to end (`DBE-013`).
- **EDT-010** The editor MUST use only installed headers and `spade::` targets (`SL2b`); a missing capability is a finding for Core.
- **EDT-011** The live smoke MUST tour the editor's operations, with each one a declared step.

## 11. Testing

- **Document:** each edit's effect on the `SceneDesc`; a refused edit leaves it unchanged; undo and redo restore exact bytes (`scene_to_yaml` equal).
- **Round trip:** every committed scene file and Kat's sample (all canonical) opens and saves byte for byte; a hand-formatted scene saves as its canonical text with equal content.
- **Save:** an invalid document is not written; Save As rewrites the world path so `compose_file()` composes the copy.
- **Run:** composing and instantiating the document equals `compose_file()` on its saved file (the first state digest); a rebuild carries a vehicle's pose and rates within the step's tolerance; a refusal keeps the old run.
- **Headless:** the command line opens, edits, saves and runs a scene with no window, in `spade_tests` and a CLI test.
- **Live smoke:** an `editor` main function with a step per operation.

## 12. Order of work (for the plan)

1. The document, its edits, validation and undo, headless, with tests.
2. Open, save (with Core's `save_scene_file`), round trip.
3. Run, pause, step; rebuild with carry.
4. The window: hierarchy, inspector, scene panel, run bar, menus, on the existing sessions.
5. The live smoke's `editor` main function.
6. The scene picker: the built-in scenes as scene files, saved scenes, and the test scenarios as composer task 5 converts them.
