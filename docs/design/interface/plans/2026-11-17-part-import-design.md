# spade_part_import: the part import tool (L88, L88b; Spec B revision 2, B1)

Interface, 2026-11-17. The lead's GO, with KAT's rulings: `--inspect` writes nothing and gives one gate result per object; the STEP inspect is fast by default and deep on request with an explicit deflection; `step_convert.py` ships beside the exe in the editor component; OpenCASCADE is not bundled. Contract: `docs/design/core/plans/2026-11-17-spec-b-revision-proposal.md` (B1, section 5). No download is made by the tool or by this work.

## 1. What it is

`spade_part_import`, a C++ exe in `tools/part_import/` (a library the tests drive in-process, as `spade_geo_fetch` is), installed beside the editor. It reads a CAD file through `spade::mesh_import` (glTF/glb, binary STL, OBJ, 3MF) or, for STEP, through `step_convert.py` run as its own process, and either **inspects** it (writes nothing) or **imports** it into the local part store. Three modes, one per run:

| Mode | Does | Writes |
|---|---|---|
| `--probe` | is the STEP converter usable: the python found, OCP present, the OpenCASCADE and wheel versions, the converter script's version; how to install it when not | nothing |
| `--inspect <file>` | the format, the file's sha256, its objects (name, triangles, bbox, volume, gate result each), the unit (declared, or a guess for STL and OBJ) | nothing |
| `--import <file> --store <dir>` | everything `--inspect` knows, then the file into the store and one `.spmesh` sidecar per object | the store |

Output is **JSON lines on standard output**, nothing else there: `{"event":"stage",...}`, `{"event":"progress",...}` and, last, one `{"event":"result",...}` (its `ok`, `exit`, `code`, `message`, `advice`, and the mode's payload). Standard error carries only a usage text. No prompts, ever; the environment variable `SPADE_PART_STORE` is only the default for `--store`.

## 2. Options

`--store DIR` (import; required unless `SPADE_PART_STORE`; refused inside a git work tree). `--unit U` (`m mm cm um in ft`; required for STL and OBJ on import; refused beside glTF, 3MF and STEP, which declare theirs, as `spade_mesh_pack` does). `--axes SPEC` and `--scale S` (below). `--object NAME` (one object of a file with several; an unknown name is a usage error listing the names). STEP only: `--deflection LEN` (a length with a unit, no default: the chord error is a choice), `--angle RAD` (default 0.5), `--deep` (inspect: tessellate and gate). `--step-python PATH`, `--step-convert PATH` (otherwise `SPADE_STEP_PYTHON`, then `.venv-step/` under the working directory or beside the exe; the script beside the exe, or `SPADE_STEP_CONVERT`).

**`--axes`** is `to_part.axes` of the part file: an exact permutation of the source axes with signs, three comma-separated items, each the source axis (`x`, `y` or `z`, optionally `-`) that becomes part x, y and z: `x,y,z` (the default, no change), `-y,x,z`. It must be a **proper rotation** (determinant +1): a mirror reverses the winding and is refused as `axes_mirror`. **`--scale S`** (finite, above zero) multiplies every position once, after the unit, in double. Axes then scale, applied once to the metre positions (a permutation and a sign are exact in fp32; the scale rounds once), and the mesh goes through `make_mesh()` again, so what is stored passed the gate in its final frame.

## 3. The store and the cache

Layout, from Spec B section 5: `<store>/<sha256>/<original file name>` is the source, copied in (content-addressed: present with the same bytes means kept); the converted sidecars are beside it, under `<store>/<sha256>/<key>/`: one `<object>.spmesh` each and `import.json` (the options, the converter version, the object to file mapping, each sidecar's content hash), **written last** (the temp-dir-then-rename pattern of `spade_geo_fetch`'s cache; an entry without `import.json` is incomplete and rebuilt). `<key>` is the first 16 hex of the sha256 of the canonical text of everything that changes a sidecar: the converter version (a number for the mesh readers, the OpenCASCADE and wheel versions for STEP), the tessellation tolerance and angle (STEP), the unit, the axes, the scale and the object selection. Rebuilt only on a miss; a hit says `"cached":true` and does no work. A `.gltf` with external buffers is refused as `external_buffers` ("export as .glb"): the stored file must be the whole source, so it is read from memory with no base directory.

## 4. Codes

Stable strings, never renamed; the exit code is one of seven. The advice is text a person can act on, kept in the tool by code, never by matching a message.

| Exit | Meaning | Codes |
|---|---|---|
| 0 | done | `ok` |
| 1 | an io or internal failure | `store_io`, `file_unreadable`, `internal` |
| 2 | usage: nothing was read | `usage`, `unit_required`, `unit_not_allowed`, `axes_invalid`, `axes_mirror`, `scale_invalid`, `object_not_found`, `deflection_required`, `store_required`, `store_in_git_tree` |
| 3 | unavailable: a format feature this build does not read, or the STEP converter | `format_unknown`, `format_unavailable`, `external_buffers`, `step_unavailable`, `converter_failed` |
| 4 | refused: the file is not a good solid | `file_invalid`, and the gate's kinds (`not_closed`, `not_manifold`, `inconsistent_winding`, `degenerate_triangle`, `self_intersecting`, `shells_not_nested`, `inside_out`, ... from `make_mesh_diagnosed`, Core; until it lands every gate refusal is `mesh_refused`) |
| 5 | cancelled | `cancelled` |
| 6 | written, but the STEP cross-check flagged a solid (the mesh volume is more than 1% from the B-rep's) | `cross_check_flagged` |

## 5. Cancel

The tool stops on Ctrl+C or Ctrl+Break (the Editor ends the process; both are handled): it kills the converter, removes its temporary directory and exits 5 with the `cancelled` result. The store is never half written, since an entry appears only by one rename, and a killed run leaves at most a temporary directory outside any entry, removed by the next run on that store.

## 6. Inspect depth, per-object results, the unit guess

**Per-object gate results.** The readers stop at the first refusal; they stay exactly so. `spade::mesh_import` gains collecting variants (`read_gltf_objects`, `read_stl_objects`, `read_obj_objects`, `read_3mf_objects`) returning one outcome per object: the mesh, or its `make_mesh()` refusal. A file-level problem (a malformed container, an accessor past its buffer) stays one refusal for the file. Until the readers land `--inspect` lists the objects of a good file and gives the first refusal of a bad one.

**STEP.** Fast (the default): `step_convert.py --inspect` reads the STEP and reports the solids' names, the declared unit, each solid's bbox and B-rep volume from OpenCASCADE, with the gate `not_checked`. Deep (`--deep --deflection LEN`): the converter writes the 3MF into a temporary directory and the tool reads it as any 3MF, so every solid has its own gate result and volume; the deflection used is reported. The import path is the same: `step_convert.py --emit-3mf` writes the 3MF, the tool reads, transforms and writes the sidecars itself (provenance: unit, tolerance, source), so `spade_mesh_pack` is not in this chain.

**The unit guess** (STL and OBJ, which carry none): the longest side of the bbox, read as millimetres when it is 5 to 5000, as metres when it is 0.005 to 5, and no guess outside both; reported with `"basis"` and never applied: an import still needs `--unit`.

## 7. Tests

In `spade_tests`, in-process (`run_part_import(args, env, out, err)` on a temporary store): the axes and scale arithmetic (exactness, the determinant rule), the cache key (each input changes it, order does not), the unit guess, each mode on generated STL, OBJ, glTF and 3MF fixtures, every code and exit, the hit and the incomplete entry, the store inside git refused, the sidecar read back through `load_mesh_file` against `import.json`'s hash. The STEP route runs on a **fake converter**: a small python script (`tests/fake_step_convert.py`) that answers the same command line with a cube, a probe, a failure, and a slow mode for the cancel test; those cases skip, naming why, when no Python is found. The real converter's tests stay `-L step` and skip without OCP, so the converter's own new modes (`--inspect`, `--emit-3mf`, `--json`) are verified by the fake's contract and, on a box with the venv, by new `StepConvert` cases.

## 8. Order of work

1. This note. 2. Axes, scale, key, unit guess, JSON writer (pure functions). 3. `--inspect` and `--import` for the mesh formats, the store, the codes, `--probe`. 4. The process layer, cancel and the STEP route; `step_convert.py`'s `--inspect`, `--emit-3mf`, `--json`. 5. The collecting readers, and the gate kinds when Core's `make_mesh_diagnosed` lands. 6. Install beside the editor with `step_convert.py`, `package-test`, `consumers.md`. Each is RED then GREEN on `interface/part-import`.

## 9. Status

Added at the lead's go (2026-11-17, after Core's S3d contract): each sidecar is also a store file of its own, `<store>/<sha256 of the sidecar>/<object>.spmesh` (atomic temp + rename, kept when already there with the right bytes), named by `store_ref: {sha256, file}` per object in the result line and in `import.json`; `find_entry` checks that file's bytes against its address.

Built on `interface/part-import`: the pure functions (codes, JSON, axes, scale, unit guess, cache key), the store, the in-process tool (`run_part_import`) for STL, OBJ, glb and 3MF, the child-process layer, the STEP route against `tests/fake_step_convert.py`, `step_convert.py`'s `--probe`, `--inspect` and `--emit-3mf`, the program (`main.cpp`, Ctrl+C as the cancel flag) and the editor install. Not built yet: the collecting readers (one gate result per object in `--inspect`) and Core's `make_mesh_diagnosed` kinds (both change the same four `make_mesh` call sites in `engine/world/mesh_import/`, so they go in together when Core's lands); a flagged STEP import is not remembered by the store. The real converter's new modes have not been run (no OpenCASCADE on the build machine).
