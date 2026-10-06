# The editor, first cut — design

**Owner:** Interface. **Status:** approved 2026-10-05 (user, via lead), as revised for his decisions D1–D4 and Q1–Q2 (§9). The implementation plan is `2026-10-05-editor-plan.md`. **Builds:** `../01-editor.md` ("The target") and `../../backlog.md` ("Sandbox → editor"). **Rulings this rests on:** `INT-2` (rebuild debounced, carry state), `INT-3` as amended 2026-10-03 (the editor saves scenes), `SL2b` (only the installed API), `SL13` (three scene sources), `SL15b` (headless first), the world/scene split (`../../01-engine-model.md`). **Inputs:** the scene file (`2026-10-03-scene-file-draft.md`, `SCN-001`–`SCN-009`, built), the composer (`2026-10-03-scene-composer-plan.md`, built), the world file (`world/world_file.hpp`, built), the module-API design (`../../core/plans/2026-10-02-module-api-design.md`), the drone-builder joint spec (`../../plans/2026-10-03-drone-builder-engine-design.md`, `DBE-013`, `DBE-014`).

## 1. Goal and done-when

The sandbox becomes Spade's editor, on the Unity pattern: a library of models and assets, placed into scenes, every parameter configurable in an inspector, and worlds created, edited and saved. A scene is the engine's own format (a world reference plus models, assets and vehicles), so the scene the editor saves is the scene Kat sends and the scene a batch run composes. The editor configures models; it never builds a new one from parts, which is Kat's builder's job.

**Done when:**
1. The editor opens any valid scene file, and saving it unchanged writes its canonical text, so a file Spade or Kat wrote comes back byte for byte (`SCN-002`).
2. A user places models and assets from the library, and can select, move, re-pose, duplicate and delete them; every edit can be undone.
3. The inspector shows and edits every parameter of a placed model and asset, and shows each validation problem at its field.
4. A user creates a new world, opens an existing one, edits and configures it, and saves it as a world file; the open scene follows the world's new hash.
5. Run, pause and single-step work on the edited scene and world, through `compose()` and `instantiate()`; an edit while running rebuilds the run, debounced, and carries vehicle state across (`INT-2`).
6. Every operation in 1–5 runs headless from the command line and is tested without a display (`SL15b`).
7. The live smoke tours the editor as a main function, so its recording shows each of the above.

## 2. Scope

| In the first cut | Specified here, built later |
|---|---|
| Two documents, a scene and its world, each with its selection, undo history and dirty state | Components and modules on scene objects (with objects in stepping, module-API §11) |
| The library: built-in items (the quadrotor template, the primitive assets) and a folder of library files | An asset browser over meshes and textures |
| Vehicles from library models; the kind is general, and drones are the only kind today | Further vehicle kinds: each arrives with its own model type (Core) and inspector table |
| The inspector: every parameter of a model, an asset, a vehicle's start, a world | Adding or removing a model's parts (rotors, drag bodies, IMU mounts): never in the editor, always Kat's builder |
| Worlds: new from a template, open, edit, configure, save | World physics records that world file v2 cannot hold (turbulence, contacts, grid): saved with the world once world file v3 does (§6) |
| Scene sources (`SL13`): built-in scenes, test scenarios and saved scenes, all as scene files | Regions and objects in worlds (world file v3, module-API §11) |
| Run, pause, single-step; debounced rebuild with carry | Scrub with snapshot restore; CPU/Vulkan lockstep view; physics debug draw; SDF slices |
| Save, Save As, Open, undo and redo, for both documents | A component menu (needs Core's availability table, module-API stage 5). It lists every type from that table and shows `fluid` as reserved with its reason, never as an option (`../01-editor.md`) |

## 3. The documents

The editor edits **documents**, never a running simulation.

- **The scene document is a `scene::SceneDesc`** plus its path, selection and undo history. **The world document is a `WorldDesc`** plus the same. Neither holds engine state of its own, so what the user edits is exactly what is saved.
- **Every edit is a function** `Result<void> apply(Document&, const Edit&)` in a display-free header (`sandbox/editor_document.hpp`). It validates the result before committing it: a scene with `validate_scene()`, a world with `validate_world_desc()`, a model with `ModelType::issues()`. An edit the file could not carry is refused with the validator's message and leaves the document unchanged.
- **Undo and redo** keep whole copies, one per committed edit, per document. Scenes and worlds are small (tens of objects, an SDF program of tens of nodes), so copies are cheap and an undo can never disagree with the edit it reverses. Each history is bounded (a fixed count, stated in code).
- **Names are the identity.** Models, assets and vehicles are addressed by their unique names (`SCN-004`), so a selection, an undo and a carried vehicle state survive a reorder.
- **The two documents meet only through the hash.** The scene names its world by path and `world_hash()` (`SCN-001`). §6 says how a world edit reaches the scene.

## 4. The library and the inspector

**The library** (Cameron, D2: "an asset/model library, with all parameters configurable … Think unity").

- **Items:** models and assets. Built-in items ship with the editor: the quadrotor template, and the primitive assets (box, sphere, cylinder, capsule). Further items are library files in a library folder: one model, or one asset, in the scene file's own syntax (§7, a need for Core).
- **The panel** lists items by kind: vehicle models grouped by vehicle kind (drones today), then assets. The kind comes from the model, so a new kind needs no change to the panel.
- **Placing copies.** Placing a model adds a copy of it to the scene's `models`, one per placed vehicle under its own name, and the vehicle that uses it; placing an asset adds a copy to the scene's `assets`. A scene never refers to a library file, so it stays self-contained, as `SCN-008` and Kat's scenes require.
- **Placement** is the builder's interaction, carried over: pick a point on the ground or a surface, drop the item there, drag it in the ground plane.

**The inspector** shows the selection's every parameter:

- **A vehicle:** its name, its model, and its start in the design frame (pose, velocity, rates, rotor speed), as the scene holds it (`DBE-013`).
- **Its model:** every `ModelType` field: the body (mass, inertia), the design frame (`design_to_principal`, `com_offset`), the proxy radius, each rotor's pose, spin, lag, radius and coefficients, each drag body's, each IMU mount's pose and noise, and the visual reference.
- **An asset:** its name, pose, collider (each primitive's parameters and transform) and its materials, and its visual.
- **The world** (§6) when nothing in the scene is selected.
- **Problems at their fields.** The inspector validates a model with `ModelType::issues()`, which names every problem with its element, index and field, so each one shows at the field it is about. An invalid value does not commit.
- **Parameters, never parts.** The inspector edits the values of a model's existing parts and never adds or removes a part. A model with a different structure is a new model, and new models come from Kat's builder (Cameron, D2: "You cannot build new models in spade editor").
- **One model per vehicle** (Cameron, Q1: "Always per vehicle"). An inspector edit to a vehicle's model reaches that vehicle alone. A scene opened from a file may share one model between vehicles (Kat's may); it is kept as it is until an edit, and the first parameter edit on such a vehicle copies the model for it under a new name before the edit commits, so the other vehicles keep theirs.
- **Back to the library** (Cameron, Q2: "Save as new item"). **Save to library** writes the configured model or asset as a new library item, under a new name, and never overwrites an existing one. Its structure is the one Kat's builder gave it.
- **Commit on release** (`INT-2`): a field commits when the user lets go of it.

## 5. Running

- **Run** composes the documents (`compose()` on the scene and the world document), instantiates the result (`instantiate()`), and steps it at the drone box's fixed step. Pause stops stepping; single-step advances one step.
- **An edit while running rebuilds** (`INT-2`): when the user lets go of a control, the editor composes and instantiates the edited documents and carries each vehicle's state by name (pose, velocity, rates, rotor speeds) through `vehicle_state()` and the new spawn. A vehicle the edit removed is dropped; one it added starts at its start pose.
- **A refused rebuild keeps the running simulation** and shows the reason (a `compose()` or `create()` error, verbatim), and leaves the documents edited, so the user can fix them. Running never changes a document.
- **Structural edits on a running world** (`../01-editor.md`'s open item) are settled for this cut by the rule above: the editor never mutates a running `Simulation`; it rebuilds from the documents. Queued structural edits wait for objects in stepping (module-API §11).

## 6. Worlds

Cameron, D4: "You can create new worlds, edit/configure/save existing worlds."

- **New world** starts from a template: an empty world with a ground plane, or a copy of any world file. It is unsaved until Save As names it.
- **Open** reads a world file with `load_world_file()`. Opening a scene opens its world as the world document.
- **Configure:** the name, the environment (gravity, wind, air density, temperature, seed), the capacities, the material palette, the lighting, the spawn points and the props, each in the inspector.
- **Edit the static geometry.** A world's terrain is a compiled SDF program, not a recipe (`world_file.hpp`), so the editor edits that program directly:
  - edit any node's parameters (a primitive's size, a transform);
  - add a primitive, joined to the terrain by a union;
  - remove or re-pose a piece that a union joins to the rest.
  A piece inside an intersection or a subtraction is edited only in its parameters. The helper that does this works on the public `SdfProgram`; Interface writes it, and Core reviews it, as Core reviewed the composer (§7).
- **Physics records that world file v2 cannot hold** (turbulence, contact parameters, the broad-phase grid) live in `WorldInstanceDesc` today. The editor shows them as run settings, says that they are not saved with the world, and saves them with it once world file v3 carries them (module-API §11, Core).
- **Save** validates the world and writes `world_to_yaml()` text atomically (a temporary file renamed over the target; `save_world_file()` writes in place). A saved world reloads bit-exactly, as the world file guarantees.
- **The scene follows the world.** Saving a world changes its `world_hash()`:
  - the open scene's `world.hash` is re-pinned as an undoable scene edit, once `compose()` succeeds on the scene and the new world;
  - every other scene file in the project folder that names this world and pins the old hash is listed, and each is re-pinned only on the user's word and only after `compose()` succeeds on it;
  - a world edit re-runs `compose()` on the open scene, so a conflict it creates (a material name the scene also uses, `SCN-004`; capacities, `SCN-007`) shows at once, not at the next run.

## 7. Saving and opening scenes

- **Save** validates the scene document and writes `scene_to_yaml()` text atomically (a temporary file renamed over the target), so a failed save never leaves a half-written scene. A world document with unsaved edits is saved first, so the scene never pins a hash that is not on disk.
- **Save As** to another folder rewrites the world path relative to the new location, so the saved scene still finds its world (`SCN-001`).
- **Open** reads with `load_scene_file()`, opens the named world, and refuses with the cause when the world is missing or its hash differs. The user can re-point the scene at another world, which re-pins the hash.
- **Round trip.** Saving an unedited scene writes its canonical text (`SCN-002`), so a file Spade or Kat wrote comes back byte for byte, and a hand-edited one comes back canonical with the same content. A Kat scene opened and saved stays a Kat scene: the editor adds nothing that is not engine content (`SCN-008`).
- **Scene sources** (`SL13`, as amended by `INT-3`): one picker over the built-in scenes (the drone box and the builder's ground), the test scenarios, and saved scenes, all as scene files. A test scenario joins the picker when it is a scene file; composer task 5 converts the viewer scenes first.

## 8. What it needs from the installed API

| Need | Today | Owner |
|---|---|---|
| Load, validate and write a scene; world hash | `scene::load_scene_file`, `validate_scene`, `scene_to_yaml`, `world_hash` | built |
| Compose and run | `scene::compose`, `compose_file`, `instantiate` | built |
| Load, validate and write a world | `load_world_file`, `validate_world_desc`, `world_from_yaml`, `world_to_yaml`, `save_world_file` | built |
| Atomic saves | the editor writes `*_to_yaml()` text to a temporary file and renames it | Interface; Core's `save_scene_file()` (routed) is a convenience, not a blocker |
| **A library file: one model or one asset**, read and written canonically | none; the scene file serializes models and assets only inside a scene | **need for Core:** `model_to_yaml` / `model_from_yaml` and `asset_to_yaml` / `asset_from_yaml`, in the scene file's syntax and validators |
| Every problem in a model, by field | `ModelType::issues()` | built |
| **Edit a compiled world's SDF** | the public `SdfProgram`; `WorldBuilder` builds programs, it does not edit them | Interface writes the helper; **Core reviews** it (world model) |
| **Save the world's physics records** | `WorldInstanceDesc` only; not in world file v2 | **Core:** world file v3 (module-API §11) |
| Carry vehicle state across a rebuild | `Simulation::vehicle_state()` (design frame) | built |
| **Spawn a vehicle with a full state** | `VehicleSpawn` carries pose, velocity, rates and one rotor speed | **Core** (routed): per-rotor speeds on spawn, or a documented single-speed carry |
| Pose an asset | `transform_of()`, `SdfPose` | built |
| Pick under the cursor | `eval()` and `gradient()` on an SDF program | built; the editor marches the composed world's SDF |
| Draw the scene | `render::scene_from_world`, `render_gl` | built |
| Meshes for visuals | `render::load_gltf` | built; the first cut draws primitives |
| List component types and their availability | none | module-API stage 5 (Core); not needed until components |

Each need is reported to Core through the lead, and nothing widens an include path (`SL2b`).

## 9. Decisions and open questions

**Decided by Cameron, 2026-10-05** (asked by the lead):
- **D1, first-cut scope:** the scene editor.
- **D2, models:** "There will be an asset/model library, with all parameters configurable. For now we only have drones as vehicles, but there will be other ones. You cannot build new models in spade editor. Think unity." This revision answers it in §4.
- **D3, scene sources:** all three of `SL13`'s.
- **D4, worlds:** "You can create new worlds, edit/configure/save existing worlds." This revision answers it in §6.

**Decided by Cameron, 2026-10-05** (the lead's second round):
- **Q1, a model edit on a placed vehicle:** "Always per vehicle". Each placed vehicle has its own copy of its model, so an edit never reaches another vehicle. There is no shared-model listing and no Make unique (§4).
- **Q2, configured models and the library:** "Save as new item". Save to library writes a new item and never overwrites one; the structure stays Kat's (§4).

## 10. Requirements

- **EDT-001** The editor MUST edit scene and world documents and MUST NOT mutate a running `Simulation`; a run is rebuilt from the documents.
- **EDT-002** Every edit MUST be a display-free operation, exercised headless and by the command line (`SL15b`).
- **EDT-003** An edit MUST pass its document's validator before it commits; a refused edit MUST leave the document unchanged and show the cause.
- **EDT-004** Every committed edit MUST be undoable and redoable, per document, to the history's stated bound.
- **EDT-005** Saving an unedited scene MUST write its canonical text, so a canonical file MUST come back byte for byte (`SCN-002`).
- **EDT-006** Save MUST write only a valid scene or world, atomically, through Core's canonical writers.
- **EDT-007** An edit while running MUST rebuild debounced and carry each surviving vehicle's state by name; a refused rebuild MUST keep the running simulation and show the reason (`INT-2`).
- **EDT-008** Objects MUST be addressed by name; order MUST be shown and edited as configuration (`SCN-005`).
- **EDT-009** Vehicle starts MUST stay in the design frame end to end (`DBE-013`).
- **EDT-010** The editor MUST use only installed headers and `spade::` targets (`SL2b`); a missing capability is a need for Core.
- **EDT-011** The live smoke MUST tour the editor's operations, with each one a declared step.
- **EDT-012** Placing a library item MUST copy it into the scene, a model once per placed vehicle; a scene MUST NOT refer to a library file.
- **EDT-013** The inspector MUST expose every parameter of a placed model and asset, and MUST NOT add or remove a model's parts.
- **EDT-014** A model edit MUST be checked with `ModelType::issues()`, and each problem MUST show at its field.
- **EDT-015** The editor MUST create, open, edit and save world files through Core's world file API; a saved world MUST reload bit-exactly.
- **EDT-016** Saving a world MUST re-pin the open scene's world hash only after `compose()` succeeds, and MUST list every scene in the project folder that pins the old hash; another scene is re-pinned only on the user's word, after `compose()` succeeds on it.
- **EDT-017** A world edit MUST re-run `compose()` on the open scene and show any conflict it creates.
- **EDT-018** A setting the world file cannot hold MUST be shown as not saved with the world.
- **EDT-019** A model parameter edit MUST reach only the selected vehicle; a model an opened scene shares MUST be copied for that vehicle before the edit commits.
- **EDT-020** Save to library MUST write a new library item and MUST NOT overwrite an existing one.
- **EDT-021** The editor's and the sandbox's UI MUST follow §13: one theme, consolidated dockable panels, collapsible sections and aligned property rows, and no boxes or separators for structure. A source scan checks the conventions.

## 11. Testing

- **Documents:** each edit's effect on its document; a refused edit leaves it unchanged; undo and redo restore exact bytes (`scene_to_yaml` and `world_to_yaml` equal).
- **Library:** placing copies the item (the scene holds no library path), a model once per vehicle; a library file round-trips; a placed model's copy equals the library item; Save to library writes a new item and leaves an existing item's bytes unchanged.
- **Inspector:** every `ModelType` field has an edit; an invalid value is refused and its `issues()` entry names the field; no edit changes a part count; an edit to a model an opened scene shares copies it first, and the other vehicles' model bytes are unchanged.
- **Worlds:** a new world from each template validates and saves; an edited world reloads bit-exactly; the SDF helper's add, remove and re-pose leave a program `SdfProgram::validate()` accepts and that evaluates as the edit says at sample points.
- **Hash follow:** saving a world re-pins the open scene after `compose()` succeeds; a scene that no longer composes is not re-pinned and says why; other scenes pinning the old hash are listed.
- **Round trip:** every committed scene file and Kat's sample (all canonical) opens and saves byte for byte; a hand-formatted scene saves as its canonical text with equal content.
- **Run:** composing and instantiating the documents equals `compose_file()` on their saved files (the first state digest); a rebuild carries a vehicle's pose and rates within the step's tolerance; a refusal keeps the old run.
- **Headless:** the command line opens, edits, saves and runs a scene and a world with no window, in `spade_tests` and a CLI test.
- **Live smoke:** an `editor` main function with a step per operation.
- **Look and feel:** the source scan over `sandbox/` (§13), plus the live smoke's still of each panel and of the default layout, for the user's review.

## 12. Order of work (for the plan)

1. The scene and world documents, their edits, validation and undo, headless, with tests.
2. Open and save for both, atomically; the scene's round trip; the hash follow.
3. The library: built-in items, library files (with Core's model and asset files), placement by copy.
4. The inspector's tables: vehicle, model (every `ModelType` field, with `issues()`), asset, world.
5. The SDF edit helper, with Core's review.
6. Run, pause, step; rebuild with carry.
7. The window, to §13: toolbar, hierarchy, inspector, library and viewport, docked, on the existing sessions.
8. The live smoke's `editor` main function.
9. The scene picker: the built-in scenes as scene files, saved scenes, and the test scenarios as composer task 5 converts them.

## 13. Look and feel

The user's direction (2026-10-06 00:42 UTC, relayed by the lead; `backlog.md`): "I want a sleek feel where things look consolidated and properly in place ... I really am going for a simplistic yet powerful editor, unity and unreal do a great job with this." It applies to the editor and to the sandbox's existing panels and HUD.

**Principles**

- **Consolidated panels**, as in Unity and Unreal:
  - a toolbar: run, pause, step and the tick; undo and redo; save;
  - the scene hierarchy;
  - the inspector;
  - the library;
  - the viewport.
  Scene and world settings show in the inspector when the scene or the world is selected in the hierarchy; they get no panels of their own. The sandbox's scenes split the same way.
- **No boxes inside panels.** A panel's content sits on the panel. There are no bordered child regions, frames, group borders, or separators drawn for structure. A list scrolls with its panel.
- **Collapsible sections instead of boxed groups:** flat, full-width, one level deep where possible.
- **Properties as aligned rows:** the label in a fixed left column and the value filling the right, one property per row. Units go in the value's format, not the label.
- **One spacing scale and one type scale:** every gap is a multiple of one base unit. Text uses one font at one size, and headings are the sections' own headers.
- **Quiet colours:** neutral greys, plus one accent used only for selection, focus, hover and active state. A warning colour is used for status, such as a refusal, which `INT-2` requires to be shown.
- **Progressive disclosure:** common settings are open and advanced ones are folded by default. Examples: a model's IMU noise and parts, a world's sky colours, the frame-time breakdown.
- **Dockable:** every panel docks, tabs and undocks. The default layout puts the hierarchy on the left, the inspector on the right, the library at the bottom, the toolbar at the top and the viewport in the centre. "Reset layout" restores it.

The HUD overlay (frame timing, the controls legend, and later the airflow readouts) is the one floating window. It is translucent and undocked by design, with the same theme and rows and no separators.

**ImGui conventions (what the check reads)**

- **Theme:** one function, `apply_spade_theme()` in `sandbox/ui_theme.hpp`, runs once after `ImGui::CreateContext()`. Nothing else sets style colours or style variables. Its values:
  - base unit 4 px;
  - `WindowPadding` (8, 8), `FramePadding` (6, 4), `ItemSpacing` (8, 6), `ItemInnerSpacing` (6, 4), `IndentSpacing` 12, `ScrollbarSize` 10;
  - `FrameRounding`, `GrabRounding` and `TabRounding` 3, `WindowRounding` 0;
  - `WindowBorderSize`, `ChildBorderSize`, `FrameBorderSize` and `TabBorderSize` 0, `PopupBorderSize` 1.
- **Font:** ImGui's default font at one size, until a proportional UI font is vendored. That is a follow-up, with its licence checked.
- **Panels:** `ImGui::Begin` windows docked into `DockSpaceOverViewport` with a pass-through central node, so the viewport shows through. The default layout is built once with the DockBuilder API.
- **Sections:** `ImGui::CollapsingHeader`, with `ImGuiTreeNodeFlags_DefaultOpen` only on the common ones.
- **Rows:** one helper, `property_row(label)`, over a two-column `BeginTable` with `ImGuiTableFlags_SizingStretchProp` and no border flags, so every row aligns.
- **Lists:** `Selectable` rows directly in the panel.
- **Status text:** one helper, `status_text()`, in the theme's warning colour.
- **Not used in `sandbox/`:**
  - `ImGui::Separator`, `SeparatorText`, `BeginGroup` used as a box;
  - `BeginChild` with `ImGuiChildFlags_Borders`, or `BeginChild` for a list;
  - `TextColored`, `PushStyleColor`, `PushStyleVar` and `StyleColors*` outside `ui_theme.hpp`.
- **Checked by** a source scan in `spade_tests` over `sandbox/`, in the manner of `TD-3`'s canary: none of the calls above appear, and the style calls appear only in `ui_theme.hpp`. The live smoke also takes a still of each panel and of the default layout, for the user to review.
