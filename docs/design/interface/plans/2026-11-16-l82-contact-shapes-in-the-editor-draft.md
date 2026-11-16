# L82: contact shapes in the editor (L68 reached by a user), DRAFT design note, for review

Interface, 2026-11-16, for the lead. Asked after the smoke audit: L68's contact tumbling is in the engine and only the tour's `BodiesSession` uses it. Nothing is built or queued.

## What stands between a user and a tumbling body (found by reading, each checked in the source)

1. **The scene file cannot carry a shape.** `ModelType::shape` (`std::optional<physics::ShapeDesc>`) exists, but `scene_file`/`model_file` neither read nor write it (newest scene version 5; a model's keys stop at `propulsion`).
2. **A Run cannot turn the module on.** `scene::instantiate()` takes no module set, so it builds with `standard_modules()`; `Simulation::spawn` refuses a shaped model there (`unavailable`, naming `rigid_contact`).
3. **A scene has no body that is not a vehicle.** Assets are static scenery; `spare.bodies` are runtime slots. A box that falls needs a model (parts, mass, inertia) used as a vehicle.

(1) and (2) are Core's files: they go to Core through the lead; the editor side is mine. (3) is question Q1.

## The design, by the lead's headings

**Inspector.** A "Collision" section under Model for a selected vehicle (a model's shape is every instance's: engine rule, `ModelType::shape`): kind none | sphere | capsule | box | hull; per kind the radius (sphere, capsule), the two ends (capsule), half extents (box), skin (box, hull), the points (hull, up to 16: more is refused by name); friction and restitution, blank = the world's (the field's -1 sentinel); iterations (0 = the engine default). Rest speed sits under Advanced. A hull has a "From model" button (Q6). Each edit is one undo step through the existing command path; a description `compile_shape()` refuses is shown verbatim in the status line and the document is not changed (the editor's rule for every edit, EDT-009). The values are shown in the design frame the rest of the Model section uses; the engine keeps the shape in the principal frame, so the inspector converts with the model's `design_to_principal` and `com_offset` (Q7).

**Scenes.** A model gets an optional `collision:` map, scene version 6, written only when a model has one (the file's own rule: oldest version that carries the content). A scene without shapes is byte-identical to today's and an older build still reads it; a version-6 file in an older build is refused with `schema_mismatch` naming the version (the existing gate). Keys follow `ShapeDesc`: `kind`, `radius` (sphere, capsule) or `skin` (box, hull), `a`/`b`, `half_extents`, `points`, `friction`, `restitution` (absent = the world's), `rest_speed`, `iterations`. The reader runs `compile_shape()` and blames line and column, as for every other value. **Save version question:** recommend 6, not a bump of the unrelated keys; the editor saves what it was given (v5 scenes stay v5 until a shape is added).

**Run.** `instantiate()` (Core) adds `rigid_contact_module()` when any model has a shape and only then, so a scene without shapes builds the standard set and keeps its digest bit for bit (the standard pins already prove this for the module set; I add one editor test: the same scene, shape removed, equals the stored digest). The editor's `EditorRun` needs no change beyond rebuilding on a shape edit, as it already does for any scene edit.

**Display.** A wireframe of the selected vehicle's shape through `set_view_lines` (the gizmo's and the airflow box's path): sphere three circles, capsule two end circles and the side lines, box twelve edges, hull its crosses and edges from a small deterministic hull of at most 16 points. It follows the body in Run and sits at the start pose in Edit. The pose is the vehicle's design pose composed with `design_to_principal`, the same conversion as the inspector. Display-free geometry in a header, tested; the drawing is a few lines.

**Library parts.** No silent default: a shape on the library's drones would change how existing scenes behave (a shaped drone on the ground answers to the new pass as well as the L9 contact set, Q5) and breaks "a set without shapes stays bit-identical" for every scene that uses them. The first cut leaves the library files alone; the user adds a shape with "From model" and saves it into their scene. A "Crate" model (a box part, 1 kg, box shape) is added to the library as the first shaped item, so a box can be placed and dropped from the Library panel (Q1).

**Tests.** (a) Scene round trip: a model with each of the four kinds written and read equal, version 6 only then, a v5 scene byte-identical, each refusal by name with its line (Core's file, a test in `tests/` next to `test_scene_file`). (b) Editor commands: set, change, clear, undo, redo; a refused value leaves the document unchanged (display-free workspace test). (c) Run: the Crate dropped from a tilt on a corner; after 3 s of Run its up axis is within 1 degree of a world axis and |v| and |omega| are under the thresholds `test_rigid_contact` uses, and two runs are bit-equal. (d) The scene without a shape has the standard digest. (e) The wireframe's lines for each kind (counts and extents). (f) The live tour's Bodies chapter can later place the same Crate through the editor instead of `BodiesSession`; not in this cut.

## Open questions

- **Q1. A body that is not a vehicle.** Recommend (a): the shape is on the model and the Crate is a model used as a vehicle; no new scene entity. (b) a `bodies:` section (dynamic props with a shape, mass, inertia, start) is a larger schema for Core and a better long-run answer: say if you want it now.
- **Q2. Order.** Core first (the `collision` key, version 6, `instantiate` choosing the module), then the editor. Both are small on the Core side; I can write the editor half against the header while it lands.
- **Q3. Mass and inertia** are the model's, not the shape's. I propose an "Inertia from box" button (the engine's `box_inv_inertia` helper, inverted) and nothing automatic.
- **Q4. Per model or per vehicle.** The engine says per model. Two vehicles of one model cannot differ; the inspector says "applies to every vehicle of this model". Say if a per-vehicle override is wanted.
- **Q5. A shaped drone with an L9 contact set** gets both responses on the ground. Physics' ruling needed: shape replaces the set, or both act.
- **Q6. "From model".** A box of the parts' bounds in the principal frame, or the hull of up to 16 of their corners and rotor tips (the tour's drone hull is 8 + 4). I propose the second, refused by name if the parts need more.
- **Q7. Frames.** A model whose `design_to_principal` is a rotation permutes a box's half extents. First cut: refuse "box" with a message when the rotation is not axis-aligned.
- **Q8. Backend.** The editor's Run steps the CPU backend; the shape pass on the device is banded against the CPU (L68). Nothing to do unless Run moves to the device.

Size, after Core's half: inspector and commands 1.5 days, wireframe 1, Crate and library 0.5, tests 1.5, gates and review 1: about 5 working days. Held until the lead's word on Q1, Q2 and Q5.
