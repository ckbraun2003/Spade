# The `spade_library` JSON contract (step 0 of the first slice)

Core, 2026-11-17. DRAFT for the lead; docs only. It fixes the shapes KAT codes against (its editor, a test-double `spade_library`, its spec builder), answers KAT's G1, G2 and G3, and defines the v3 index and entry layout the slice's code (S1 to S4) will read. Built on Spec B revision 4 (`2026-11-17-spec-b-rev4-library-design.md`). Nothing here is code. Where a shape is "shape only", the first slice does not build it; the shape is frozen so KAT can code against it.

Marks: **[slice]** built in the first slice; **[slice if cheap]** built if it costs under a day; **[shape]** shape only, built after the review.

## 0. Conventions

- **Text.** Requests and responses are UTF-8 JSON. Paths are UTF-8 absolute paths (KAT's `path_utf8.h` form); a relative path in a request is refused (`relative_path`). Backslashes in JSON are escaped as usual. A path Spade writes in a response uses forward slashes on every platform (`C:/kat/library/index.yaml`), so a golden exchange is the same file on Windows and Linux; Windows reads either form.
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

- Return value: `0` the call was answered with `ok: true`; `1` it was answered with `ok: false` (a refusal object is in the response); `2` ABI misuse (a null argument, a null handle): when `response_json` is non-null it is set to an `invalid_request` refusal ("no library is open", `ok: false`), so a call before `open` answers `invalid_request` for `list`, `find` and every other op; with a null `response_json` nothing is produced. A handle after `spade_lib_close` is dangling and any use is undefined: the ABI cannot detect it (a host or test double that tracks its own closed state answers `invalid_request`, the same refusal). Every response is a NUL-terminated string the caller frees with `spade_lib_free`.
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

Codes (all [slice] unless marked): `open_failed` (the refusal of `open`; its `issues` hold one object per problem), `invalid_request` (unknown key, wrong type, missing key), `unknown_op`, `unsupported_format`, `relative_path`, `duplicate_root`, `root_unavailable` (the root's folder or index is absent or unreadable; `root`, `path`), `store_file_missing` (a file in the local store that an entry's record cites by sha256 is absent; `id`, `root`, `path` (where it looked, `<store>/<sha256>/<file>`: the hash is the folder name), `field` is `store`; a refusal in `find`, a warning in `list`), `root_not_empty` (`create_root` into a folder that already has files; [shape]), `root_name_mismatch` (a v3 index's `name` differs from the name the host gave), `dependency_missing` (a `depends` name not among the opened roots), `dependency_cycle`, `foreign_root_reference` (an entry names a root it does not depend on; in `default`, any other root), `bad_index` (the index does not parse or breaks a rule; `issues` lists them), `root_not_loaded`, `bare_id`, `entry_not_found`, `wrong_kind`, `alias_conflict`, `hash_mismatch` (a WARNING in `find` and `list`; a refusal in `update_entry` and `remove_entry`), `peek_failed` (warning), `read_only_root`, `invalid_entry` (an entry or its file fails validation; `issues`), `still_required` (`remove_entry` when other entries `require` it; `issues` names them), `io_error`, `internal_error` (an exception inside the library, answered rather than thrown across the ABI; a bug to report), `adjust_out_of_range` [shape], `compile_refused` [shape], `not_implemented` (an op the loaded library does not build yet; so a double and the real library agree on which calls exist).

`open` refuses in two ways, and never opens part of a set or skips a root. A REQUEST problem (checked first, before any file is read): a root name that is not `[a-z][a-z0-9_]*` (`invalid_request`), a root given twice (`duplicate_root`), a root path or the store that is empty or not absolute (`relative_path`): the refusal's top-level `code`, `root` and `path` are the FIRST problem's, and `issues` lists every request problem. A problem in what is READ (`root_unavailable`, `root_name_mismatch`, `bad_index`, `dependency_missing`, `dependency_cycle`, `foreign_root_reference`, `bare_id`, `alias_conflict`): top-level `open_failed`, with `issues` holding one refusal-shaped object per problem.

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
    store: null                       # INSTEAD of `file`: {sha256: "<64 hex>", file: x500.glb}, a file in the local store (below)
    type: null                        # OPAQUE kinds: a string KAT chose; then the id prefix is free except a Spade kind name
    tags: [quad, 5inch]
    thumbnail: thumbnails/x500.png    # optional
    requires: [default:part.motor.x]  # ids; each must be this root's or a dependency's
    files: [worlds/x.gates.json]      # companions (travel with the entry; part of its hash)
    generated_from: {recipe: gate-corridor, hash: "sha256:..."}   # optional
    parts_map: [ ... ]                # drones only, section 5
```
Unknown, missing and duplicate keys are errors, as in a scene file (`bad_index`).

**Kinds.** An entry's kind is the first segment of its id, one of `model`, `object`, `part`, `drone`, `world`, `mesh`, or an entry has `type:` and is OPAQUE (kind `opaque`; Spade lists, finds and hashes it and never reads its file or interprets the type). The file extension must agree where it decides a kind (`kind_mismatch` is a `bad_index` issue): `model` and `drone` are `*.model.yaml` (a `drone` may instead have only a `parts_map`), `object` `*.object.yaml`, `part` one of the six part extensions (`part_kind` = motor, prop, esc, battery, frame, payload; a frame is a vehicle's structure and a payload is avionics, a camera, a computer, a radio or a mast, and a file of either kind carries the version-2 part blocks of `read_part_blocks`: install points, mass, meshes, shapes), `world` `*.world.yaml`, `mesh` `*.gltf`, `*.glb` or `*.spmesh` (listed, found and hashed; not validated in the slice).

**Store files (L88b).** A third-party file (the PX4 x500, the Bitcraze Crazyflie, a mesh made from them) is not committed to any root; it lives in the box's local STORE, `<store>/<sha256>/<file>`: one store per machine, shared by every root, content-addressed. An entry names such a file with `store: {sha256: "<64 hex>", file: "x500_frame.glb"}` INSTEAD of `file` (exactly one of the two, except a drone with a `parts_map`, which needs neither). `sha256` is 64 lower-case hex of the file's exact bytes and `file` is a plain file name (no '/', no '\', no '..', not empty); the extension of `file` decides the kind exactly as for `file`. The RECORD (the entry, its citations and licence) stays in the root; only the bytes are in the store. Companions (`files`) stay in the root. The host gives the store's absolute path at `open` (`store`, below); Spade never searches for it and reads no environment variable. An absent store, or a store without the file, does NOT stop `open` (a CI machine has no store): the entry lists with `store_file_missing` as a WARNING in `list` and as the refusal of `find`, its `file` null and its peek empty. Its hash is NOT read from disk: its line in the hash is `"store/<sha256>/<file>\t<sha256>\n"` with the declared sha256 (the content address is the proof), so the hash is the same on a machine with the store and on one without; `validate` with `verify_store: true` re-hashes the bytes and refuses a mismatch (`hash_mismatch`). A store folder `<store>/<sha256>/` may hold more than the named file (the import tool's cache directories, `import.json`, `.tmp-*` folders): the reader looks a file up by `<sha256>/<file>` only, never lists a store folder, and ignores everything it did not ask for. Store entries are built right after S3a (S3d); until then an index with `store:` is refused as an unknown key.

**The hash** of an entry is SHA-256 over text lines `"<rel path>\t<sha256 hex of that file's content>\n"`, for its `file` and each of `files`, sorted by relative path bytewise, plus, for a drone with a `parts_map`, the line `"parts_map\t<sha256 hex of its canonical JSON>\n"` (keys sorted, no whitespace). A file's CONTENT is its bytes, except that for a TEXT file every CR LF pair is read as LF: text files are those whose extension, ASCII case-folded, is `.yaml`, `.yml`, `.json`, `.gltf`, `.md`, `.txt` or `.csv` (a fixed list); `.glb`, `.bin`, `.png`, `.spmesh` and every other extension are hashed exactly as they are. Only CR LF pairs are folded (a lone CR stays), and a file is never refused for its line endings, so a Windows checkout with autocrlf hashes the same as an LF one. It is computed by Spade, never written in the index, and is not transitive (a drone's hash does not change when a part's file does; KAT's saved map stores each part's own `{id, hash}`).

**Aliases.** The tables of all opened roots are merged. A key that is also an id, or that two roots both define, is `alias_conflict` at open. An alias must name an entry that exists (`bad_index` otherwise).

## 4. The calls

Every request is a JSON object; the keys below are all of them (others are refused). Responses show `result` only.

### open [slice]
`spade_lib_open` request:
```json
{ "roots": [ { "name": "default", "path": "C:\\spade\\share\\spade\\assets\\library", "writable": false },
             { "name": "kat",     "path": "C:\\kat\\library", "writable": false },
             { "name": "user",    "path": "C:\\Users\\me\\kat-library", "writable": true } ],
  "store": "C:\\parts\\part-store" }
```
`store` is optional (null: no store) and must be an absolute path (`relative_path` otherwise); it is where `store:` entries' files are looked up. Order matters for `list`. `name` is `[a-z][a-z0-9_]*`; Spade knows only the name `default` (it is the root Spade ships; nothing else is special). `writable` is the host's setting; a write to a root not marked is `read_only_root`. `result`:
```json
{ "roots": [ { "name": "kat", "path": "C:\\kat\\library", "library_version": 3, "depends": ["default"],
               "entries": 94, "writable": false } ], "entry_count": 190, "store": "C:\\parts\\part-store" }
```

### reload [slice if cheap]
`{ "root": "kat" }` (`null`: every root). Re-reads the index (and drops cached hashes) for that root; same `result` as `open`, for the roots reloaded. A failure leaves the handle as it was and refuses.

### list [slice]
```json
{ "roots": ["default", "kat"],     // null: every opened root, in open order
  "kind": "part",                  // null | model|object|part|drone|world|mesh|opaque
  "part_kind": "motor",            // null | motor|prop|esc|battery|frame|payload
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
  "thumbnail": null, "requires": [], "files": [], "store_file": null,
  "hash": "sha256:...", "generated_from": null,
  "peek": { "mass_kg": 0.032, "provides": [], "limits": {}, "rotor_count": null, "world_version": null, "install_kinds": ["prop_hub"] } }
```
`store_file` is null for an entry whose file is in the root, and for a store entry `{ "sha256": "...", "file": "x500_frame.glb", "path": "C:\\parts\\part-store\\<sha256>\\x500_frame.glb", "present": true }` (`path` is null when no store was given; `present` is false when the file is missing); for a store entry `rel_file` is null and `file` is that resolved path, or null when it is absent. `peek` has these keys for every kind (null where they do not apply): `mass_kg` (a part's mass or a model's total, from its header; null if the file declares none), `provides` (list) and `limits` (map) for a part (empty in the slice: those keys arrive with part_version 2), `rotor_count` for a model or drone, `world_version` for a world, and `install_kinds` for a part: the sorted, de-duplicated list of the kinds of install point the part itself carries (`["motor", "battery"]` for a frame; `["prop_hub"]` for a motor), empty in the slice (install points arrive with part_version 2). A header that cannot be read gives `peek: null` and a `peek_failed` warning; the entry still lists. `hash` is cached per handle until `reload` or a write; a `hash_mismatch` warning is not raised by list.

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
`expect_hash` is the caller's pin (the `{id, hash}` KAT stores in a package manifest, not in `airframe.json`): `hash_matches` is the comparison of that pin with the entry's current hash, and `entry.hash` is the current one. A hash that does not match gives `hash_matches: false` and a `hash_mismatch` warning; the entry still returns (KAT warns or refuses). A missing id: `entry_not_found` (`id`, `root`); a root that is not open: `root_not_loaded` (`root`, `id`) so the Problems row can name it. `record` is built for the four existing part kinds in the slice; other kinds give `record: null`. `measured` is reserved for the measured rotor block (null until that piece).

### resolve [slice if cheap]
`{ "alias": "mesh:drone/x500-body", "root": null }` -> `{ "id": "kat:mesh.drone.x500_body", "root": "kat" }`; `entry_not_found` (field `alias`) if no table has it; `root_not_loaded` (field `root`) if `root` is not open; `invalid_request` for an empty alias; an id is not an alias. The result's `root` is the resolved id's own root, which need not be the table's root. Alias tables are PER ROOT, in each root's `library.yaml`, and an alias key is any non-empty string: it need not contain ':' (a world file's refs may not), so `gate-corridor` and `mesh:drone/x500-body` both work. `root` (a root name, or null) limits the lookup to that root's table. Keys are unique across the opened set (`alias_conflict` at open), so a lookup with `root: null` is never ambiguous.

### hash_text [slice if cheap]
`{ "text": "..." }` -> `{ "hash": "sha256:..." }`: the SHA-256 of the UTF-8 bytes of `text` exactly as given (no newline translation), for a host that pins a document it holds in memory.

### create_root [shape]
`{ "name": "user", "path": "C:\\Users\\me\\kat-library", "depends": ["default", "kat"], "writable": true }`. Creates the folder if it is absent, refuses a folder that has any file in it (`root_not_empty`), and writes a version 3 `library.yaml` (`name`, `depends`, `indexes: [index.yaml]`) and an empty `index.yaml` (`index_version: 1`, `entries: []`), atomically as `add_entry` does. The new root is added to the handle (last in order) and `result` is the same root object as `open`'s. A root named `default` is refused (`invalid_request`: that name is the one Spade ships); `depends` must name roots already open.

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
               "adjust": { "pos": [0,0,0.002], "yaw_deg": 0, "pitch_deg": -10, "roll_deg": 0 },   // null: none
               "adjust_range": { "pos_min": [..], "pos_max": [..], "yaw_deg": [-5, 5], "pitch_deg": [-10, 60], "roll_deg": [0, 0] },   // null: unchecked
               "part_to_mount": null } ] }                               // the placed part's own offset, null: identity
```
`result`: `{ "items": [ { "key": "motor_fl", "pose": {pos, quat}, "refusal": null } ... ] }` in request order (G3d: an array). pose = parent · install_point · adjust · part_to_mount (a product of poses in that order, normalised, `w >= 0`): the ADJUSTMENT acts in the install point's own frame, after the install point's pose (so a camera's tilt turns about the mount's own axes). An adjustment is a position offset `pos` (metres) and three rotations in degrees, right-handed about the install point's axes (+X forward, +Y left, +Z up), composed as `Rz(yaw_deg) · Ry(pitch_deg) · Rx(roll_deg)`: yaw about +Z, then pitch about +Y, then roll about +X, each about the axes of the frame produced so far (intrinsic). A positive angle is a right-handed turn about its axis, so a positive pitch tips +X toward -Z; KAT converts its editor's names and signs. All four keys are always present in a result; an input may omit angles (0) and `pos` (zero). A bad item has `pose: null` and its own refusal (`adjust_out_of_range`, `invalid_request`); the call is still `ok: true`, so one bad slot does not hide the others. An absent `adjust` is a zero adjustment and is checked against an `adjust_range` that is present (a range that excludes 0 refuses it); the bounds are inclusive; `adjust_out_of_range` names the first axis outside its range (`adjust.pos[0..2]`, then `adjust.yaw_deg`, `adjust.pitch_deg`, `adjust.roll_deg`), and `invalid_request` names the field (`parent.quat`, `install_point.pos`, `adjust_range.yaw_deg`, ...). A `w` within 1e-15 of 0 is 0, and then the first axis of x, y, z beyond 1e-15 is positive, so a rotation has one spelling. The call is pure and needs no open library, but takes a handle for symmetry.

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
`physical.imus` (left out of the sketch for length) is one `{ "pos": [x, y, z] }` per IMU, in the design frame measured from the centre of mass: an install point of kind `imu` reaches KAT's sensor descriptor as `mount_pos`, never as the model's `imu_mounts`.
`physical` is the compile's result in Spade's own names, in the numbers the host hands the simulator (float32 widened); KAT's package build maps it to its flat `airframe.json` keys and the pin test compares it. A refusal is `compile_refused` with every issue. REQUIREMENTS for KAT's five measured drones (Spade Host, H4): the compile is fp32-exact against today's compiled values, sets no collision shape of its own and gives a zero centre-of-mass offset. The Spade host reads identity.name, mass, inertia, collision_radius, motor n, arm_x, arm_y, kappa, thrust_per_motor, tau and drag mode and coeff, and NOT omega_max (it assumes 3000 rad/s); `physical.rotors[].omega_max` still carries it for KAT's flat `airframe.json`. KAT pins the result at JSON level and per field.

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
  "parent": "frame", "install_point": "motor_fl", "placement": null,
  "adjust": { "pos": [0, 0, 0], "yaw_deg": 0, "pitch_deg": 0, "roll_deg": 0 }, "count": 1 }
{ "slot": "prop_fl", "part": "default:part.prop.5x4", "hash": "sha256:...",
  "parent": "motor_fl", "install_point": "prop_hub", "placement": null, "adjust": null, "count": 1 }
{ "slot": "frame", "part": "kat:part.frame.x500", "hash": "sha256:...",
  "parent": null, "install_point": null,
  "placement": { "pos": [0, 0, 0], "quat": [1, 0, 0, 0] }, "adjust": null, "count": 1 }
```
- A slot is placed in ONE of two ways, never both (`invalid_entry` if both or neither): by `parent` + `install_point` (a SLOTTED part: `parent` is the `slot` key of the part that carries the install point, `install_point` its name on that part), or by `placement` (a FREE part: a FULL pose in the drone's design frame, with `parent` null; the frame part itself is free). A slotted part's pose is derived by composing up the parent chain (`compose_placement`) and is never stored, so it cannot disagree. `adjust` is the per-slot adjustment value (`pos` and `yaw_deg`, `pitch_deg`, `roll_deg`, as in `compose_placement`; null for none), applied at the install point.
- PROPS attach to the MOTOR part's `prop_hub` install point (KAT's ruling; Spade's part model agrees): a prop slot has `parent` = the motor's slot and `install_point` = `prop_hub`. The motor part carries the `prop_hub` point and the spin sign (`spin` on a motor only, as Spec B fixes); the prop's own record gives its hub origin.
- The allowed RANGE of an adjustment is a property of the frame part's install point (`adjust: {pos_min, pos_max, yaw_deg: [min, max], pitch_deg: [min, max], roll_deg: [min, max]}` in its part record, a [shape] until the install-point piece), not of the drone; `compose_placement` takes it as `adjust_range`. The slice reads, validates the form of, stores and returns `adjust` (a drone's `adjust` is `{pos, yaw_deg, pitch_deg, roll_deg}`, finite numbers; unknown keys are refused); the range check is `compose_placement`'s.
- `hash` pins the part as KAT's saved map does; `find` of the drone does not check it (the parts' own `find` does).
- `frame` is `"design/1"`: the frame of section 0.

## 6. What the slice builds, and examples

[slice]: v3 index and entry layout (S1); N roots, `open`, the refusals, aliases at open (S2); `list` and `find` with the peek fields, the hash and a drone's parts map (S3); `validate` text for model, object, world, the four part kinds and `index` (S3); the C ABI over those (S4). [slice if cheap]: `reload`, `resolve`, `mesh_file_info`. [shape]: `add_entry`, `update_entry`, `remove_entry`, `compose_placement`, `compile`, `evaluate`, `validate as drone`. In the slice an op that is [shape] answers `not_implemented`.

Golden exchanges: S4 commits `tests/data/library_contract/*.json`, a request and its exact response per call (every refusal code the slice can raise), as data. Spade's tests replay them against the real library, and KAT's test-double replays the same files, so the two cannot drift.

## 7. Revision 2 of this document (KAT's builder notes)

Added, from KAT's builder spec and Spade Host: `create_root` (1), `install_kinds` in the list peek (2), the pin semantics of `find`'s `expect_hash` (3), the parts-map placement rule (4, 8: a slot is slotted by `parent` + `install_point` or free by a full-pose `placement`; props attach to the motor's `prop_hub`), `root_unavailable` and `store_file_missing` as two codes (5), `hash_text` (7), per-root alias tables with plain keys (H1), and the compile requirements for the measured drones and IMU mounts (H4, H6). `mesh_file_info` stays marked [slice if cheap] and is now in the slice (H5). `evaluate`'s partials (6) were already in section 4.

## 8. Points for the lead

1. Frame (section 0): I state the contract's frame as KAT's descriptor frame, per Spec B A5; `AirframeSpec`'s own comments say "design frame" and I have not re-read whether its hub thrust axis is +Y or +Z there. S4's compile piece converts and pins it with a test; KAT should treat section 0 as the contract.
2. A drone's `parts_map` lives in the index entry (so one atomic `add_entry` writes it); a drone may have no model file until `compile` is built. Alternative: a `*.drone.yaml` file beside the index (Spec B round 2 cut that builder, so I did not).
3. `compile.physical` is in Spade's names, not KAT's `airframe.json` keys: KAT's spec builder and package build own that mapping, so no KAT key appears in Spade.
4. `validate` returns one issue in the slice (the existing loaders stop at the first error). The shape already carries many.
5. Files in `add_entry` are inline text or base64; a large mesh would be added by path through a later `from_path` key (not now: YAGNI).
