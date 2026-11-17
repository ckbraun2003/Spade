# Spec B revision proposal (Spade's side): a drone is a parts list of imported parts

Core, 2026-11-17. DRAFT for the lead; not sent to Kat. Responds to L88 (an engine, not a builder: library defaults are imported, publicly available CAD parts, and a drone is a parts list with install points), L88a (material properties for CFD and pressure or tension come later) and L88b (local only for now: third-party CAD stays in a local folder outside git; nothing third-party is committed). Read: Spec B (`KAT/design-specs/integration/drone-builder.md`, sections 2 to 8), the x500 airframe asset's keys, the catalog and data-pack headings; Spade's part file, model part and import headers. No download was made.

What L88 reverses, and what holds. `world/mesh_shapes.hpp` (:4-8) and the default-library plan (:228) say library meshes are generated, "never from downloaded CAD"; L88 reverses that for local imported defaults, and this proposal amends both texts when approved. `assets/library/SOURCES.md` (:5), "stores no source's own files: no CAD, no mesh", still holds, because under L88b the CAD stays in the local store. The split in the drone-builder engine design (:4) holds: Kat owns the parts, the catalogue, the builder and the compiler; Spade gives the format, the import and the physics. DBC-051 holds: this proposal adds no third-party dependency (the import route is the existing `mesh_import` and the existing `tools/step_convert`).

What does not change: the owner's 2026-10-19 answer already retires parametric frames and meshes generated from part dimensions in Spec B; Spade adds no generator (Spade's few shape generators stay as the CI placeholder, below). L6 holds: `make_mesh()` refuses a bad mesh, and nothing in Spade repairs one.

## 1. The part format (Spec B section 4)

Today (Spade): a part file (`part_version: 1`, `part_file.hpp` :17-30) is a motor, prop, ESC or battery only, every field cited, with no mount (the drone owns mounts, :57-59). A model's parts carry no mass of their own (`model_part.hpp` :9-12: `BodyTemplate` is the one mass), and a mesh part's mass is density x volume with the mesh's own centre of mass and inertia (:74-78). There is no way to say "this part weighs 38 g, here is its tensor", and `fpv_5inch.sources.yaml` (:12-16) lumps the FC, VTX and camera into one assumed "stack" record. An install point is the part's own interface point, not a mount: the drone still owns where a part goes (the placement is the drone's slot pose composed with the part's install point).

Proposed, as `part_version: 2` for any part that carries the new blocks (the writer writes the lowest version that holds the part, so every existing part file stays version 1 byte for byte, and an older reader refuses version 2 as `schema_mismatch`; the CORE-10 rule):

- `geometry`: where the solid comes from. `source: {format: step|3mf|gltf|stl|obj|spmesh, sha256, object (optional name inside the file), unit (stl and obj)}`, `to_part: {axes, scale}` (an exact axis permutation with signs, then a scale, applied once, so CAD frames and Kat's +Z-thrust frame meet without rounding), and `fallback: generated:<shape>` (below).
- `mass_properties`: spec-sheet `mass` (kg), optional `com` (m, part frame), optional `inertia` (tensor about the centre of mass, kg m^2). Three levels: mass only (the mesh's shape, scaled to that mass, as `wear_look` does today), mass + com, or the full tensor. These override density x volume; the engine refuses an inconsistent set by field (a tensor that breaks the triangle inequality, a centre of mass outside the mesh's box, a non-positive mass).
- `install_points`: a list of `{name, kind, pos, orient}` in the part frame. A frame lists its slots (motor k, battery tray, FC stack, companion plate, camera mount, GNSS mast, radio); any other part lists its own base point (and optionally a second, for a part with two interfaces). A placement is `slot pose x install point`, composed once in double.
- `source` record: the existing library pattern (`assets/library/sources.yaml`: id, citation, url, retrieved, license, license_evidence) plus the source file's `sha256`. The licence is recorded, not enforced: local-only (L88b) means the licence text never reaches the repo, only its id.
- `material`: a name, reserved for L88a; validated as a non-empty string, stored, ignored by every pass, not hashed. When L88a uses it, it joins the identity under a version bump.

The part-kind list grows (section 2). Everything already in section 4.1 (motor, prop, ESC and battery physics blocks) is unchanged.

## 2. Slots (Spec B section 6)

| Slot | Exists in Spade | New |
|---|---|---|
| frame | a body record and arm records (generated meshes) | a `frame` part: imported geometry, mass properties, install points for every slot below |
| motors | `motor` part, a rotor per motor, bell mesh | the motor's geometry from `geometry`; its install point is the hub |
| props | `prop` part, blade stations, generated prop mesh, massless on the rotor | geometry may be imported; the generated prop stays (it carries the blade physics) |
| battery | `battery` part | geometry, mass properties, install point |
| ESC | `esc` part | geometry, mass properties, install point |
| flight controller (Pixhawk-class) | `imu_mounts` in the model; the sensor models in `sensors/` | a `fc` part: geometry, mass, install point, and its IMU's mount pose (the IMU noise stays Kat's block, section 4.1) |
| companion computer (Raspberry Pi or Jetson) | nothing | a `companion` part: geometry, mass, install point only (no physics) |
| camera | nothing (Kat's visual sensors) | a `camera` part: geometry, mass, install point and its optical axis; the field of view stays Kat's |
| IMU (extra) | `imu_mounts` | a mount pose from an install point |
| radio (receiver or telemetry) | nothing | a `radio` part: geometry, mass, install point (RF stays reserved, section 12) |
| GNSS | the GNSS sensor (L85) and its antenna offset | a `gnss` part: geometry, mass, install point; the antenna position feeds the sensor |

Rule: a part kind with no Spade physics is geometry, mass and install points only; its behaviour stays in Kat's blocks. `role` stays the model file's free label; the roles in use are body, plate, arm, motor, prop, battery, strap, camera, gimbal, lens, gear, esc, gps, antenna (Rendering's material table keys on them), plus fc, companion, radio.

## 3. The compiler (Spec B section 7)

- Step 1 (expand the frame into primitives) goes: the frame is one imported part.
- Step 3: the compiler passes each part's `mass_properties` to Spade's composite (`airframe_composite`, the one implementation); a part with none keeps density x volume. The model file keeps carrying the compiled mass, centre of mass and inertia, as today.
- Step 8 (the visual): each part's sidecar, named in the model file by path and content hash, as today.
- What Spade's model file takes from it: a part list (name, role, mount, sidecar ref and hash, `min_feature`), unchanged in shape. What the library takes: part files (version 2 where used). No new model-file keys are needed. A library drone stays a model file; the demo's is written the way the library ones are today (by a test, or by hand from the records). The step from a parts list to a model file is Kat's compiler (Spec B section 7), not a Spade tool.
- Byte-identical rule for existing drones: a part without the new blocks compiles to the same bits as today; a test pins it.

## 4. The format

Unchanged and complete: glTF/glb, binary STL, OBJ and 3MF through `spade::mesh_import`; STEP offline through `tools/step_convert` (OpenCASCADE via pinned wheels, then 3MF, then `spade_mesh_pack`) to a `.spmesh` sidecar. Every route ends in `make_mesh()`'s gate (closed, oriented, 2-manifold, no self-intersection), visual refs included (`render/mesh_resolver.hpp` loads them through `load_mesh_file`); the ungated `render/gltf.hpp` loader is used only by tests. A part that fails is refused with the gate's message and fixed upstream in the CAD. No repair step in Spade (L6). A STEP route is the likelier clean source for real parts; mesh-soup downloads often fail.

## 5. L88b mechanics

- A local part store, a folder outside git, named by an environment variable (`SPADE_PART_STORE`) with a per-user default. Layout: `<store>/<sha256>/<original file name>`, and the converted sidecar beside it. The store is content-addressed, so a record cannot silently point at a different file.
- The converted sidecar is cached in the store, keyed by the source's sha256, the converter's version and the tessellation tolerance; it is rebuilt only on a miss (the conversion itself is Interface's `spade_part_import`).
- The repo holds only records: part files naming `geometry.source.sha256` and the spec-sheet numbers (facts), and the source record. No third-party geometry or sidecar is committed.
- Resolution: the record's sha256 finds the file; a changed or missing file is refused by name (`unavailable`, naming the hash and the store path), never substituted.
- Tests that need real parts skip, with a named reason, when the store or the file is absent. The CI default is the generated meshes: a part with `geometry.fallback: generated:<shape>` builds from Spade's existing generators (a rounded box, a capped tube, a bell, a prop), so a clean checkout still builds, flies and renders a plain drone. The result with the store is the same physics (spec-sheet mass properties) and a better look.
- Open: whether a published demo (the website) may ship a mesh. That needs a licence that allows redistribution (CC0, CC-BY with attribution, or a permissive project licence) and the owner's word; out of this proposal.

## 6. Engine work, owners and days (working days, before queue waits)

Core (me), in the lead's order (demo first), re-sized after KAT's review: 1. part file version 2 and the block functions shared by Spade's reader and the validator: 3. 2. `validate_part_blocks` and `read_part_blocks` (issues with code, part id, field path, message; the kind table; per-field citations; the proper-rotation rule; the `PartFrameConvention` pin): 2. 3. Declared mass properties through `PartGeometrySpec` and the compile, with the byte-identical pin: 2.5. 4. The store resolver, the sidecar cache, `resolve_part_geometry` with the A9 rules and the `stand_in` flag, the generated fallback, skip-without-store support: 3. 5. New part kinds (Spade side: geometry, mass and install-point fields; `provides` is KAT's and ignored): 2. 6. Install points and `compose_placement`: 1.5. 7. `mesh_file_info`: 0.5. 8. The demo model and library records (record-only; the model written by a test or by hand), consumers.md and decisions lines, and the Spec B wording for KAT (A1): 2. Total about 16.5 working days (13 before KAT's review: the validator and the resolver rules add 3.5), calendar about four weeks with gate waits. No code starts until Spade says go.
Interface (`mesh_import`, asked directly): a `spade_part_import` command: any format to the store (sha256, unit and axes flags, the STEP venv documented, the gate's refusals passed through), 3 days; a gate check of each demo source (does it make a clean solid), 1 day.
Rendering: role to material (their display-only table, already started), 1 to 2 days; `flow_movie --drone` loading a sidecar from the store, about 1 day (Rendering sizes it). The editor's viewport and its API are KAT Spade Host's (B2).
Physics: nothing now. L88a (materials for CFD and pressure or tension) is theirs later and joins `material`.
Kat: the slot and airframe asset, the compiler's calls and the builder UI; not sized here.

## 7. Demo build for the website (names only; no download, no licence read)

An X500-class research quad: a Pixhawk-class flight controller plus a Raspberry Pi or a Jetson, a camera and a radio. Candidate public CAD sources, from memory and each to be checked before any fetch (Spade's word, then a licence read recorded in `sources.yaml`): PX4's `PX4-gazebo-models` x500 (Kat already draws its meshes, pinned at commit e5997f45; likely visual meshes that may not pass the solid gate); the Holybro X500 frame and the Pixhawk boards' published STEP or drawings from the vendor or the Pixhawk open-hardware pages; Raspberry Pi's published mechanical files (STEP) for a Pi board and its camera module; NVIDIA's Jetson carrier-board mechanical files; an ExpressLRS receiver's open hardware files; a community CC0 or CC-BY model from a public CAD site for anything missing. None of these has been opened, and none is assumed to be redistributable.

## Revision 2: KAT's review (Editor and Components, 2026-11-17)

Where this section and the text above disagree, this section wins. By KAT's item number; "agreed" means taken as KAT wrote it.

**A1. One part file, and what it means for Spade's library.** Agreed, with this shape. There is ONE catalogue part file, KAT's, written by the editor; it holds KAT's keys (`provides`, `limits`, `emits`, `senses`, provenance) and the five version-2 blocks (`geometry`, `mass_properties`, `install_points`, `source`, `material`). Spade ships, in the installed library, ONE pure function for the blocks' rules:
`std::vector<PartIssue> validate_part_blocks(std::string_view text)` (JSON or YAML; JSON is read as YAML), each issue `{Code code, std::string part_id, std::string field, std::string message}`, every issue in the document, not only the first. It looks only at `id` and the five blocks and ignores every other key, so the engine never depends on KAT's keys. The editor's Inspector (while typing) and the compiler's Problems row both call it and show the same text. A second function, `read_part_blocks(text)`, returns the validated blocks as a struct for KAT's compiler to put into Spade's public descriptions (REQ-DB-006 holds: Spade reads nothing but the blocks).
Spade's own library defaults stay Spade's files: `assets/library/parts/*.yaml` keep `part_file` format, strict about unknown keys, version 1 for every file that exists today and version 2 where a default carries the blocks; the reader shares the block functions with the validator, so there is one implementation of the rules. Spade's library does not become KAT's files, and the resolver reads the blocks struct, not a file format, so it serves both.
D-3 CHANGES, named: it said "part files use Kat's convention; Spade never reads part files". New D-3: the catalogue part is KAT's one file; its version-2 blocks use the part frame below (A5); Spade reads those blocks, and only those, through `validate_part_blocks` and `read_part_blocks`, and never the rest of the file. Text KAT amends in Spec B (Spade supplies the wording): section 2 rulings Meshes (generated meshes retired, `geometry` imported) and Frames (retired: a frame is one imported part with install points); section 4 (`mesh` becomes `geometry`, `mount` becomes `install_points`, `mass_properties`, `source` and `material` added); section 5 (retired); section 7 steps 1 (no frame expansion) and 8 (the visual is each part's sidecar); REQ-DB-001 (declare mass, an inertia or a declared tensor, and install points) and REQ-DB-004 (retired with the parametric frames).

**A2. `provides`.** Agreed. It is KAT's key in the one file, outside the five blocks, so the engine ignores it. Each new kind's row gets it: fc provides `cap.sensor.imu`, camera `cap.sensor.camera`, gnss `cap.sensor.gnss`, radio the receiver capability.

**A3. Provenance per field.** Agreed. The blocks cite per field path in Spade's existing form (`{source, grade, note}`, grades measured, published, retailer, derived, fitted, assumed): `mass_properties.mass`, `.com`, `.inertia`, and each install point's `pos` and `orient`. The validator refuses a declared value with no citation, by field path (`invalid_argument`). Mapping KAT's SOURCED to a cited grade and ESTIMATED to `assumed` is a KAT-side choice. The `source` record keeps covering the file.

**A4. A closed install-point `kind` list.** Agreed: motor, prop_hub, battery, esc, fc, companion, camera, gnss, radio, imu, vtx, antenna. Placement: the list is a table inside the validator, not in the compile, and the compile never branches on `kind`; it takes poses only, so the engine stays generic (a new kind is a one-line library change). `spin` (+1 or -1) is allowed on a `motor` point only (refused by field elsewhere); KAT's compiler turns it into the rotor's spin. Per-slot adjustments are KAT's asset data. Spade provides one helper, `compose_placement(slot_pose, adjustment, install_point)`, which composes in double in that fixed order, once.

**A5. The frame.** `pos` and `orient` are in the PART's own frame, right-handed. `to_part.axes` must be a proper rotation (determinant +1): a mirror is refused by field, because it reverses winding and hands an IMU the wrong way. The design frame the placement lands in is KAT's (Spec B D-3). I do not decide its axes here: KAT's x500 asset states +X forward, +Y left, +Z thrust, and Spec B's header notes a conflict with `body_axes.h` (+X nose, +Y up, +Z right); KAT states the frame in D-3. Spade pins whatever it is with a test (`PartFrameConvention`, modelled on `test_gate_axis_convention`), using a part whose install points sit on known axes.

**A6. One source per value.** Agreed. The gnss part's install point is the antenna's one position; Spade adds no second offset. The camera part gives the pose and the optical axis only (the install point's forward axis; tilt is KAT's adjustment); the field of view, near and far are KAT's `vision.camera` params.

**A7. Byte-identical, reworded.** Version 1 part and model records compile unchanged, and a test pins it. This work never regenerates `content/airframes/*.airframe.json`; a parts-list drone writes a NEW file. The rule covers Spade's own library, not KAT's content.

**A8. KAT's parts home.** `content/parts/` is KAT's, with its own generator and checker; its tests run on the generated fallback and skip store-only checks. Spade's `assets/library/parts` stays Spade's.

**A9. A missing file.** Adopted verbatim: file absent and a fallback set: build from the fallback and return a warning issue naming the hash, the store path and the source record's URL; file absent, no fallback: refused by name at compile; file present but not matching its hash: refused; SAVE always works (REQ-DB-009). The function is `resolve_part_geometry(blocks, store)`, returning the geometry, `used_fallback` and the issues.

**A10. A stand-in changes the physics.** Agreed, and stated: a stand-in gives the same physics only at mass level 3 (mass, COM, tensor); at levels 1 and 2 the mesh gives the COM or the inertia, so the flown numbers, the model's content hash and a recorded run differ. `resolve_part_geometry` returns `stand_in = true` for the part; the compile output list and the banner "stand-in geometry: <part>" are KAT's. No full tensor is required, and no model-file key is added.

**B1. The import tool's contract.** Agreed: `spade_part_import --inspect <file>` (JSON: format, objects, bbox, guessed unit, triangle count, volume, gate result; writes nothing); `--import` with an explicit `--store <dir>` and `--unit`, `--axes`, `--scale`, `--object` (the environment variable is only a default); JSON-lines progress and result (sha256, sidecar path, converter version); stable exit codes, no prompts, cancellable; `--probe` (the STEP venv present, the converter version); gate refusals as stable codes with text the user can act on ("export as a closed solid"). Interface owns it; I ask them whether it changes their 3 days.

**B2. The rendering row.** Re-owned. Loading a sidecar in the editor is KAT Spade Host's viewport. Spade Rendering keeps only what Spade's own tools need (`flow_movie --drone` from the store). One Core item supports the viewport's "bbox and volume without loading the whole mesh": `mesh_file_info(path)` (triangle count, bbox, volume, content hash), 0.5 day.

**B3. IMU mounts.** The step from an install point to `imu_mounts` is KAT Spade Host work, pinned by "no mount = today's bits". Spade's model already carries `imu_mounts`; nothing is needed from the engine.

**B4. Avionics power (`load_w`).** The battery chain has no constant-load term today (I found none in the battery or propulsion code), so a constant bus load would be a Physics module change (a `bus_load_w` on the battery model, drawn as P / V); the per-part `load_w` and the sum are KAT's. Not mine to decide; I put it to Physics through the lead.

**C1. Vendored meshes already in git.** Not Spade's repo and not mine: Spade's `assets/library` holds no third-party file, and L88b is Spade's rule. KAT asks Cameron for its own `content/meshes`.

## Lead's answers (2026-11-17)

(a) `part_version: 2` for the new blocks: Kat's authored catalogue parts will carry them, which is CORE-10's own trigger.
(b) The converted sidecar is cached in the store, keyed by the source sha256 plus the converter version and the tolerance; rebuilt only on a miss.
(c) The licence is recorded only. Nothing ships under L88b. Anything published, such as the website, gets its licence read before it goes out, and Cameron decides.
CUT (YAGNI; the builder is Kat's): the `*.drone.yaml` parts-list builder. A library drone stays a model file; the parts-list-to-model step is Kat's compiler.
ORDER, demo first: part file v2, declared mass with the byte-identical pin, the store resolver with the generated fallback and skip-without-store, new kinds, install points and slot placement.
No code until Kat's Editor and Components have answered.
