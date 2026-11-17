# The `spade_library` JSON contract (step 0 of the first slice)

Core, 2026-11-17. DRAFT for the lead; docs only. It fixes the shapes KAT codes against (its editor, a test-double `spade_library`, its spec builder), answers KAT's G1, G2 and G3, and defines the v3 index and entry layout the slice's code (S1 to S4) will read. Built on Spec B revision 4 (`2026-11-17-spec-b-rev4-library-design.md`). Nothing here is code. Where a shape is "shape only", the first slice does not build it; the shape is frozen so KAT can code against it.

Marks: **[slice]** built in the first slice; **[slice if cheap]** built if it costs under a day; **[shape]** shape only, built after the review.

## 0. Conventions

- **Text.** Requests and responses are UTF-8 JSON. Paths are UTF-8 absolute paths (KAT's `path_utf8.h` form); a relative path in a request is refused (`relative_path`). Backslashes in JSON are escaped as usual.
- **Numbers.** Finite JSON numbers only (no NaN, no infinity). A number is written as the shortest decimal that reads back to the same double (`std::to_chars`, the same on MSVC and gcc); a float32 field is written as its exact widening to double. Lengths are metres, masses kilograms, angles degrees where a key ends in `_deg`, radians otherwise only where the key says `rad`.
- **Keys.** Every key in a shape below is always present. An absent value is `null`; an absent list is `[]`. A client ignores keys it does not know (a minor version adds keys); Spade refuses unknown keys in a REQUEST (`invalid_request`, naming the key).
- **Hashes** are `"sha256:<64 lower-case hex>"`.
- **Ids.** A full id is `<root>:<local id>` (`kat:drone.x500`). A local id is lower-case ASCII, dotted, `[a-z0-9_]` segments, and its first segment is the entry's kind (below). A bare id (no `<root>:`) is refused (`bare_id`), never guessed.
- **Frame.** Every pose in this contract is in the drone's design frame: metres, +X forward, +Y left, +Z up (thrust along +Z), right-handed, a rotation as a unit quaternion `[w, x, y, z]` with `w >= 0`. Spade converts to its internal frame inside `compile` (the part-frame convention, settled in Spec B A5). A pose is `{"pos": [x, y, z], "quat": [w, x, y, z]}`.

## 1. The format string and the C ABI

Every response carries `"format": "spade.library/1"` and `"minor": <int>`. The major (`1`) changes only on a break; a client refuses a response whose format it does not know, by name. A request MAY carry `"format"`; if present it must name a major Spade supports (`unsupported_format`).

The C ABI (`spade_library.dll`, `.so`; C linkage; DynLib-loadable; no struct crosses):

```c
const char* spade_lib_version(void);   // static string, "spade.library/1.0"
int  spade_lib_open (const char* request_json, spade_lib** out, char** response_json);
int  spade_lib_call (spade_lib* h, const char* op, const char* request_json, char** response_json);
void spade_lib_close(spade_lib* h);
void spade_lib_free (char* response_json);   // frees any response_json, from open or call
```

- Return value: `0` the call was answered with `ok: true`; `1` it was answered with `ok: false` (a refusal object is in the response); `2` ABI misuse (a null argument, a null or closed handle): no response is produced and `*response_json` is null. Every response is a NUL-terminated string the caller frees with `spade_lib_free`.
- **Threads.** On one handle the read ops (`list`, `find`, `resolve`, `validate`, `mesh_file_info`, `compile`, `evaluate`, `compose_placement`) may run concurrently from any threads. `add_entry`, `update_entry`, `remove_entry` and `reload` take the handle exclusively; a read in flight sees the index wholly before or wholly after a write, never in the middle. `spade_lib_close` must not race any other call on the handle.
- **Locating the library.** The host passes the full path to `spade_library` to DynLib; Spade looks nowhere else. There is no environment variable inside the library. The CLI `spade_library` takes the same `op` and request JSON (`spade_library <op> --request <file>|-`, roots in the request) and prints the same response; the SDK uses it.

The response envelope:

```json
{ "format": "spade.library/1", "minor": 0, "op": "find", "ok": true,
  "result": { },
  "warnings": [ <refusal-shaped objects, severity "warning"> ] }
```
and on refusal `{ "format": ..., "minor": 0, "op": "find", "ok": false, "refusal": <refusal object> }`.

## 2. The refusal object (and warnings)

```json
{ "code": "root_not_loaded", "severity": "error",
  "message": "root 'kat' is not loaded",
  "root": "kat", "path": null, "id": "kat:drone.x500", "field": null,
  "issues": [] }
```
`code` is stable; `message` is for people and may change. `root`, `path`, `id`, `field` say what the problem is about (null when not applicable). `issues` is a list of issue objects when the refusal has many (`invalid_entry`, `compile_refused`, `open_failed`):

```json
{ "code": "invalid_argument", "severity": "error", "kind": "motor", "index": 0,
  "id": null, "path": "motor.kv", "line": 12, "column": 5, "message": "kv must be > 0" }
```
(`kind` and `index` follow `AirframeIssue`; `path` is the field path; `line` and `column` are present only for text that was parsed, else null.)

Codes (all [slice] unless marked): `invalid_request` (unknown key, wrong type, missing key), `unknown_op`, `unsupported_format`, `relative_path`, `duplicate_root`, `root_unavailable` (folder or index absent or unreadable; `root`, `path`), `root_name_mismatch` (a v3 index's `name` differs from the name the host gave), `dependency_missing` (a `depends` name not among the opened roots), `dependency_cycle`, `foreign_root_reference` (an entry names a root it does not depend on; in `default`, any other root), `bad_index` (the index does not parse or breaks a rule; `issues` lists them), `root_not_loaded`, `bare_id`, `entry_not_found`, `wrong_kind`, `alias_conflict`, `hash_mismatch` (a WARNING in `find` and `list`; a refusal in `update_entry` and `remove_entry`), `peek_failed` (warning), `read_only_root`, `invalid_entry` (an entry or its file fails validation; `issues`), `still_required` (`remove_entry` when other entries `require` it; `issues` names them), `io_error`, `adjust_out_of_range` [shape], `compile_refused` [shape], `not_implemented` (an op the loaded library does not build yet; so a double and the real library agree on which calls exist).

`open` refuses as `open_failed` with `issues` holding one refusal-shaped object per root problem; it never opens part of a set and never skips a root.

## 3. The v3 index and entry layout (what S1 reads) [slice]

**A root** is a folder with `library.yaml`.
- **Version 1 and 2** (today): `library_version`, `entries:` of `{name, category, file, thumbnail}`. Still valid and read unchanged. A v1/v2 root has no name of its own: it takes the name the host gives in `open`. Its entries get DERIVED ids, `<kind>.<stem>` with `-` and `.` in the stem turned to `_` and the kind from the file extension (`*.model.yaml` is `model`, `*.object.yaml` `object`, the four part extensions `part`): `default:model.reference_quad`, `default:part.motor.<stem>` (a part's id is `part.<part_kind>.<stem>`). A derived id that collides is refused (`bad_index`). Writing an id into the index is how a root takes control of its ids; that is a v3 index.
- **Version 3**:

```yaml
library_version: 3
name: kat                       # must equal the name the host gave
depends: [default]              # names of roots this one may reference; default depends on nothing
aliases:                        # optional; opaque old string -> full id (this root's own or a dependency's)
  "mesh:drone/x500-body": kat:mesh.drone.x500_body
indexes:                        # per-kind index files, relative, in display order
  - parts.yaml
  - drones.yaml
  - worlds.yaml
```

**A per-kind index file**: `index_version: 1`, `entries:` in display order:

```yaml
index_version: 1
entries:
  - id: drone.x500                    # required; local id; unique in the root
    name: x500                        # required; display; unique per kind, ignoring case
    category: Drones                  # required (may be "")
    file: drones/x500.model.yaml      # relative, forward slashes, no '..'; may be omitted for a drone with a parts map only
    type: null                        # OPAQUE kinds: a string KAT chose; then the id prefix is free except a Spade kind name
    tags: [quad, 5inch]
    thumbnail: thumbnails/x500.png    # optional
    requires: [default:part.motor.x]  # ids; each must be this root's or a dependency's
    files: [worlds/x.gates.json]      # companions (travel with the entry; part of its hash)
    generated_from: {recipe: gate-corridor, hash: "sha256:..."}   # optional
    parts_map: [ ... ]                # drones only, section 5
```
Unknown, missing and duplicate keys are errors, as in a scene file (`bad_index`).

**Kinds.** An entry's kind is the first segment of its id, one of `model`, `object`, `part`, `drone`, `world`, `mesh`, or an entry has `type:` and is OPAQUE (kind `opaque`; Spade lists, finds and hashes it and never reads its file or interprets the type). The file extension must agree where it decides a kind (`kind_mismatch` is a `bad_index` issue): `model` and `drone` are `*.model.yaml` (a `drone` may instead have only a `parts_map`), `object` `*.object.yaml`, `part` one of the four part extensions (`part_kind` = motor, prop, esc, battery; others arrive with the new part kinds), `world` `*.world.yaml`, `mesh` `*.gltf`, `*.glb` or `*.spmesh` (listed, found and hashed; not validated in the slice).

**The hash** of an entry is SHA-256 over text lines `"<rel path>\t<sha256 hex of that file's bytes>\n"`, for its `file` and each of `files`, sorted by relative path bytewise, plus, for a drone with a `parts_map`, the line `"parts_map\t<sha256 hex of its canonical JSON>\n"` (keys sorted, no whitespace). It is computed by Spade, never written in the index, and is not transitive (a drone's hash does not change when a part's file does; KAT's saved map stores each part's own `{id, hash}`).

**Aliases.** The tables of all opened roots are merged. A key that is also an id, or that two roots both define, is `alias_conflict` at open. An alias must name an entry that exists (`bad_index` otherwise).

## 4. The calls

Every request is a JSON object; the keys below are all of them (others are refused). Responses show `result` only.

### open [slice]
`spade_lib_open` request:
```json
{ "roots": [ { "name": "default", "path": "C:\\spade\\share\\spade\\assets\\library", "writable": false },
             { "name": "kat",     "path": "C:\\kat\\library", "writable": false },
             { "name": "user",    "path": "C:\\Users\\me\\kat-library", "writable": true } ] }
```
Order matters for `list`. `name` is `[a-z][a-z0-9_]*`; Spade knows only the name `default` (it is the root Spade ships; nothing else is special). `writable` is the host's setting; a write to a root not marked is `read_only_root`. `result`:
```json
{ "roots": [ { "name": "kat", "path": "C:\\kat\\library", "library_version": 3, "depends": ["default"],
               "entries": 94, "writable": false } ], "entry_count": 190 }
```

### reload [slice if cheap]
`{ "root": "kat" }` (`null`: every root). Re-reads the index (and drops cached hashes) for that root; same `result` as `open`, for the roots reloaded. A failure leaves the handle as it was and refuses.

### list [slice]
```json
{ "roots": ["default", "kat"],     // null: every opened root, in open order
  "kind": "part",                  // null | model|object|part|drone|world|mesh|opaque
  "part_kind": "motor",            // null | motor|prop|esc|battery
  "type": null,                    // null | an opaque entry's type string, exact match
  "text": "2306",                  // null | ASCII case-folded substring of name, category or a tag
  "tags": [],                      // every tag listed must be on the entry
  "offset": 0, "limit": null }
```
`result`: `{ "entries": [ <entry> ... ], "total": 12, "offset": 0 }` in root order then index order (the hand order). Each row is a full `<entry>`:
```json
{ "id": "default:part.motor.fpv_2306_1700kv", "root": "default", "local_id": "part.motor.fpv_2306_1700kv",
  "kind": "part", "part_kind": "motor", "type": null,
  "name": "FPV 2306 1700 kV", "category": "Motors", "tags": ["fpv"],
  "file": "C:\\...\\motors\\fpv_2306.motor.yaml", "rel_file": "motors/fpv_2306.motor.yaml",
  "thumbnail": null, "requires": [], "files": [],
  "hash": "sha256:...", "generated_from": null,
  "peek": { "mass_kg": 0.032, "provides": [], "limits": {}, "rotor_count": null, "world_version": null } }
```
`peek` has these four-plus keys for every kind (null where they do not apply): `mass_kg` (a part's mass or a model's total, from its header; null if the file declares none), `provides` (list) and `limits` (map) for a part (empty in the slice: those keys arrive with part_version 2), `rotor_count` for a model or drone, `world_version` for a world. A header that cannot be read gives `peek: null` and a `peek_failed` warning; the entry still lists. `hash` is cached per handle until `reload` or a write; a `hash_mismatch` warning is not raised by list.

### find [slice]
```json
{ "id": "kat:drone.x500", "expect_hash": "sha256:...",   // null: no check
  "with": ["record", "parts_map"] }                       // any of; default []
```
`result`:
```json
{ "entry": <entry>, "hash_matches": true,       // null when expect_hash is null
  "drone": { "frame": "design/1", "parts_map": [ <slot> ... ], "measured": null },   // null unless kind drone and requested
  "record": { ... } }                           // null unless requested: a part file's own mapping as JSON, key for key
```
A hash that does not match gives `hash_matches: false` and a `hash_mismatch` warning; the entry still returns (KAT warns or refuses). A missing id: `entry_not_found` (`id`, `root`); a root that is not open: `root_not_loaded` (`root`, `id`) so the Problems row can name it. `record` is built for the four existing part kinds in the slice; other kinds give `record: null`. `measured` is reserved for the measured rotor block (null until that piece).

### resolve [slice if cheap]
`{ "alias": "mesh:drone/x500-body" }` -> `{ "id": "kat:mesh.drone.x500_body", "root": "kat" }`; `entry_not_found` if no table has it.

### validate [slice]
```json
{ "as": "part.motor",     // model | object | world | part.motor | part.prop | part.esc | part.battery | index | drone
  "text": "...",          // the document, UTF-8 (unsaved edits)
  "name": "fpv_2306" }    // optional, for messages
```
`result`: `{ "valid": false, "issues": [ <issue> ... ] }`. A document that fails is NOT a refusal (`ok: true`); a refusal is only a malformed request or an `as` Spade does not know. In the slice the issues come from the existing loaders (one issue per call, `path` and `line` null where the loader's message carries none; the v2 block validator will add many with paths later without changing the shape). `index` validates a library index text; `drone` is [shape].

### mesh_file_info [slice if cheap]
`{ "path": "C:\\...\\x.spmesh" }` or `{ "id": "kat:mesh.x" }` ->
```json
{ "container": "spmesh", "triangles": 2112, "vertices": 1056,
  "bbox_min": [-0.1, -0.1, -0.03], "bbox_max": [0.1, 0.1, 0.03],
  "volume_m3": 0.000052, "unit": "m", "sha256": "sha256:...", "solid": true }
```
(`container` is spmesh, 3mf, gltf, stl or obj; `unit` null where the file declares none; `volume_m3` and `solid` null for a presentation glTF that is not a closed solid.)

### add_entry, update_entry, remove_entry [shape]
```json
// add_entry
{ "root": "user",
  "entry": { "id": "drone.my_quad", "name": "My quad", "category": "Drones", "tags": [], "requires": [],
             "file": "drones/my_quad.model.yaml", "type": null, "thumbnail": null, "files": [],
             "generated_from": null, "parts_map": [ ... ] },
  "files": [ { "rel": "drones/my_quad.model.yaml", "text": "..." }, { "rel": "thumbnails/x.png", "base64": "..." } ],
  "replace": false }
// update_entry : { "id": ..., "expect_hash": "sha256:...", "entry": { only the keys to change }, "files": [ ... ] }
// remove_entry : { "id": ..., "expect_hash": "sha256:...", "delete_files": true }
```
Each validates the whole result first (`invalid_entry` with `issues`, nothing written), refuses `read_only_root`, `hash_mismatch` (update and remove, so two writers cannot overwrite each other), `still_required` (remove), and returns `{ "id": "user:drone.my_quad", "hash": "sha256:...", "entry": <entry> }`.
**Atomic (G3a).** One write covers the entry's file, its companions and its index line: the files are written to a temp folder inside the root, then renamed into place, and the index is replaced last (temp file and rename). A crash leaves either the old state or the new one: a leftover temp folder or a journal line is rolled forward or back by the next `open` or `reload`, and a reader on a live handle sees before or after only. This call is the only writer of an index.

### compose_placement [shape]
```json
{ "items": [ { "key": "motor_fl",
               "parent": { "pos": [0,0,0], "quat": [1,0,0,0] },          // null: identity (the frame part in the drone)
               "install_point": { "pos": [0.09,0.09,0], "quat": [1,0,0,0] },   // in the parent part's frame
               "adjust": { "pos": [0,0,0.002], "yaw_deg": 0 },           // null: none
               "adjust_range": { "pos_min": [..], "pos_max": [..], "yaw_deg": [-5, 5] },   // null: unchecked
               "part_to_mount": null } ] }                               // the placed part's own offset, null: identity
```
`result`: `{ "items": [ { "key": "motor_fl", "pose": {pos, quat}, "refusal": null } ... ] }` in request order (G3d: an array). pose = parent · install_point · adjust · part_to_mount (quaternion product in that order, normalised, `w >= 0`). A bad item has `pose: null` and its own refusal (`adjust_out_of_range`, `invalid_request`); the call is still `ok: true`, so one bad slot does not hide the others. The call is pure and needs no open library, but takes a handle for symmetry.

### compile [shape]
```json
{ "spec": <AirframeSpec>, "options": { "report": true } }
```
`<AirframeSpec>` is key for key the C++ `AirframeSpec` in `engine/vehicles/airframe_compile.hpp` (its field names are the keys; blocks `motor`, `prop`, `esc`, `battery`, `rotors`, `drag`, `imus`, `parts`, `rotor_parts`, `proxy_radius`, `air_density`, `gravity`, `state_of_charge`; a `MountPose` is a pose; the one motor, prop, ESC and battery type of today). A `parts[]`, `motor`, `prop`, `esc` or `battery` entry may instead carry `{"from_part": "<id>", "hash": "sha256:..."}` to take its record from a library part (a hash mismatch warns). The full key table is generated from the header when the compile piece is built.
`result`:
```json
{ "model_yaml": "...",                       // the model file Spade would write, as text
  "inertia": { "mass_kg": 0.526, "com": [..], "tensor": [[..],[..],[..]] },
  "fit": { "hover_duty": .., "hover_omega": .., "hover_thrust": .., "thrust_coeff": .., "torque_coeff": .., "tau": ..,
           "rotor_inertia": .., "provenance": { "mass_and_inertia": "from_parts", "rotor": "fitted", "drag": "given", ... } },
  "physical": { "mass_kg": .., "inertia_diag": [..], "com": [..], "drag": { "mode": "componentwise", "coeffs": [..] },
                "collision_radius": .., "rotors": [ { "pos": [..], "spin_dir": 1, "kappa": .., "thrust_per_motor": .., "tau": .., "omega_max": .. } ] },
  "report": { "stand_ins": [ { "part": "default:part.motor.x", "reason": "no geometry file" } ], "issues": [ <issue> ] } }
```
`physical` is the compile's result in Spade's own names, in the numbers the host hands the simulator (float32 widened); KAT's package build maps it to its flat `airframe.json` keys and the pin test compares it. A refusal is `compile_refused` with every issue.

### evaluate [shape] (G2)
```json
{ "spec": <AirframeSpec>, "options": { "state_of_charge": 1.0, "air_density": null, "avionics_load_w": 0.0 } }
```
`evaluate` always answers `ok: true` for a parseable spec and returns what it can; each quantity is a value object or `{ "missing": [ { "field": "battery.cell_capacity", "message": "..." } ] }`:
```json
{ "status": "complete",                   // "complete" | "partial"
  "compilable": true, "compile_issues": [ <issue> ],
  "all_up_weight": { "mass_kg": 0.526, "weight_n": 5.158, "provenance": "from_parts" },
  "com": { "pos": [x,y,z], "offset_from_hub_centroid": [dx,dy,dz] },
  "thrust_to_weight": { "max_thrust_n": 19.3, "ratio": 3.74, "at_state_of_charge": 1.0 },
  "hover": { "throttle": 0.52, "omega_rad_s": 801, "thrust_per_rotor_n": 1.29, "electrical_w": 118 },
  "flight_time": { "hover_s": 612, "excludes": ["avionics_load"], "usable_capacity_ah": 1.3, "cutoff_soc": 0.1 },
  "peak_current": { "battery_a": 61.0, "battery_limit_a": 65.0, "esc_a": 15.2, "esc_continuous_limit_a": 30.0, "esc_burst_limit_a": 40.0 },
  "limit_warnings": [ { "code": "battery_current_near_limit", "severity": "warning", "message": "...", "value": 61.0, "limit": 65.0 } ] }
```
`avionics_load_w` (default 0) is subtracted from the flight time's energy when given; the `excludes` list says what the number leaves out when it is not. A spec that is not yet compilable (no battery, no motor) returns `status: "partial"`, with the quantities it can still compute (weight from the parts present) and `missing` on the rest; it is never a refusal.

## 5. A drone's parts map and the slot adjustments (G3b) [slice: stored and returned]

A drone entry's `parts_map` is a list, in slot order, in the index (`library_version` 3), returned by `find` with `with: ["parts_map"]`:
```json
{ "slot": "motor_fl", "part": "default:part.motor.fpv_2306_1700kv", "hash": "sha256:...",
  "install_point": "motor_fl",
  "placement": { "pos": [0.09, 0.09, 0.0], "quat": [0.7071, 0, 0, 0.7071] },
  "adjust": { "pos": [0, 0, 0], "yaw_deg": 0 }, "count": 1 }
```
- `install_point` names a point on the PART that carries the slot's frame (a frame part), or null for a part placed by `placement` alone. `placement` is the part's pose in the drone's design frame as authored; `adjust` is the per-slot adjustment value (a small offset and a yaw, null for none).
- The allowed RANGE of an adjustment is a property of the frame part's install point (`adjust: {pos_min, pos_max, yaw_deg: [min, max]}` in its part record, a [shape] until the install-point piece), not of the drone; `compose_placement` takes it as `adjust_range`. The slice stores and returns `adjust` untouched.
- `hash` pins the part as KAT's saved map does; `find` of the drone does not check it (the parts' own `find` does).
- `frame` is `"design/1"`: the frame of section 0.

## 6. What the slice builds, and examples

[slice]: v3 index and entry layout (S1); N roots, `open`, the refusals, aliases at open (S2); `list` and `find` with the peek fields, the hash and a drone's parts map (S3); `validate` text for model, object, world, the four part kinds and `index` (S3); the C ABI over those (S4). [slice if cheap]: `reload`, `resolve`, `mesh_file_info`. [shape]: `add_entry`, `update_entry`, `remove_entry`, `compose_placement`, `compile`, `evaluate`, `validate as drone`. In the slice an op that is [shape] answers `not_implemented`.

Golden exchanges: S4 commits `tests/data/library_contract/*.json`, a request and its exact response per call (every refusal code the slice can raise), as data. Spade's tests replay them against the real library, and KAT's test-double replays the same files, so the two cannot drift.

## 7. Points for the lead

1. Frame (section 0): I state the contract's frame as KAT's descriptor frame, per Spec B A5; `AirframeSpec`'s own comments say "design frame" and I have not re-read whether its hub thrust axis is +Y or +Z there. S4's compile piece converts and pins it with a test; KAT should treat section 0 as the contract.
2. A drone's `parts_map` lives in the index entry (so one atomic `add_entry` writes it); a drone may have no model file until `compile` is built. Alternative: a `*.drone.yaml` file beside the index (Spec B round 2 cut that builder, so I did not).
3. `compile.physical` is in Spade's names, not KAT's `airframe.json` keys: KAT's spec builder and package build own that mapping, so no KAT key appears in Spade.
4. `validate` returns one issue in the slice (the existing loaders stop at the first error). The shape already carries many.
5. Files in `add_entry` are inline text or base64; a large mesh would be added by path through a later `from_path` key (not now: YAGNI).
