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
- What Spade's model file takes from it: a part list (name, role, mount, sidecar ref and hash, `min_feature`), unchanged in shape. What the library takes: part files (version 2 where used) and drone records that list their parts and slots by id. No new model-file keys are needed; the drone's parts list is a Core-side input to the compile (an `AirframeSpec`), and a small tool can build a model file from a `*.drone.yaml` parts list (section 6, item 5).
- Byte-identical rule for existing drones: a part without the new blocks compiles to the same bits as today; a test pins it.

## 4. The format

Unchanged and complete: glTF/glb, binary STL, OBJ and 3MF through `spade::mesh_import`; STEP offline through `tools/step_convert` (OpenCASCADE via pinned wheels, then 3MF, then `spade_mesh_pack`) to a `.spmesh` sidecar. Every route ends in `make_mesh()`'s gate (closed, oriented, 2-manifold, no self-intersection), visual refs included (`render/mesh_resolver.hpp` loads them through `load_mesh_file`); the ungated `render/gltf.hpp` loader is used only by tests. A part that fails is refused with the gate's message and fixed upstream in the CAD. No repair step in Spade (L6). A STEP route is the likelier clean source for real parts; mesh-soup downloads often fail.

## 5. L88b mechanics

- A local part store, a folder outside git, named by an environment variable (`SPADE_PART_STORE`) with a per-user default. Layout: `<store>/<sha256>/<original file name>`, and the converted sidecar beside it. The store is content-addressed, so a record cannot silently point at a different file.
- The repo holds only records: part files naming `geometry.source.sha256` and the spec-sheet numbers (facts), and the source record. No third-party geometry or sidecar is committed.
- Resolution: the record's sha256 finds the file; a changed or missing file is refused by name (`unavailable`, naming the hash and the store path), never substituted.
- Tests that need real parts skip, with a named reason, when the store or the file is absent. The CI default is the generated meshes: a part with `geometry.fallback: generated:<shape>` builds from Spade's existing generators (a rounded box, a capped tube, a bell, a prop), so a clean checkout still builds, flies and renders a plain drone. The result with the store is the same physics (spec-sheet mass properties) and a better look.
- Open: whether a published demo (the website) may ship a mesh. That needs a licence that allows redistribution (CC0, CC-BY with attribution, or a permissive project licence) and the owner's word; out of this proposal.

## 6. Engine work, owners and days (working days, before queue waits)

Core (me): 1. part file version 2: geometry, mass properties, install points, source record, material name, with tests and the version rule: 3. 2. Declared mass properties through `PartGeometrySpec` and the compile (override, validity, the byte-identical pin for existing records): 2.5. 3. Install points and slot placement in the compile: 1.5. 4. New part kinds (frame, fc, companion, camera, radio, gnss), geometry, mass and install points only: 2. 5. The part store resolver, the skip-without-store tests, the generated fallback, and a `*.drone.yaml` parts-list builder: 3. 6. The demo model and library records (record-only, no third-party file), consumers.md and decisions lines: 1.5. Total about 13 days, calendar about three weeks with 30 to 60 minute gate waits.
Interface (`mesh_import`, asked directly): a `spade_part_import` command: any format to the store (sha256, unit and axes flags, the STEP venv documented, the gate's refusals passed through), 3 days; a gate check of each demo source (does it make a clean solid), 1 day.
Rendering: role to material (their display-only table, already started), 1 to 2 days; loading a sidecar from the store in `--drone` and the editor, 1.5 days; named materials after L88a.
Physics: nothing now. L88a (materials for CFD and pressure or tension) is theirs later and joins `material`.
Kat: the slot and airframe asset, the compiler's calls and the builder UI; not sized here.

## 7. Demo build for the website (names only; no download, no licence read)

An X500-class research quad: a Pixhawk-class flight controller plus a Raspberry Pi or a Jetson, a camera and a radio. Candidate public CAD sources, from memory and each to be checked before any fetch (Spade's word, then a licence read recorded in `sources.yaml`): PX4's `PX4-gazebo-models` x500 (Kat already draws its meshes, pinned at commit e5997f45; likely visual meshes that may not pass the solid gate); the Holybro X500 frame and the Pixhawk boards' published STEP or drawings from the vendor or the Pixhawk open-hardware pages; Raspberry Pi's published mechanical files (STEP) for a Pi board and its camera module; NVIDIA's Jetson carrier-board mechanical files; an ExpressLRS receiver's open hardware files; a community CC0 or CC-BY model from a public CAD site for anything missing. None of these has been opened, and none is assumed to be redistributable.

## Questions for the lead

1. `part_version: 2` for the new blocks (this proposal), or optional keys inside version 1 as for L72k (CORE-11)? I recommend version 2: third-party records are authored outside the build, and CORE-11 says to bump then.
2. May a sidecar converted from a store file be cached in the store only (this proposal), or must it be rebuilt in memory each run? Caching is faster and stays outside git.
3. Is the licence recorded but not enforced (this proposal), or should Spade refuse a record whose licence id is not on a list?
