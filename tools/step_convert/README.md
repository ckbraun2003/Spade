# step_convert -- a STEP file to Spade mesh sidecars

`step_convert.py` takes a STEP file (ISO 10303-21, AP203/AP214/AP242 solids) once, offline, to one `.spmesh`
sidecar per solid: a closed, validated triangle mesh in metres, named after the part, placed by the assembly, with the
tolerance it was tessellated at recorded in the file. A scene then names the sidecar like any other mesh part.

It reads the STEP with OpenCASCADE (OCCT 7.9.3, from the pinned `cadquery-ocp` wheels), writes a 3MF, and hands that to
`spade_mesh_pack`, so every mesh passes Spade's own reader and `make_mesh()` before a sidecar exists. It is a build-tree
tool: it is not installed and the engine never loads it.

## Install

OCP needs a Python 3.10 to 3.14 venv. The requirements are fully pinned and hash-checked:

```
python -m venv .venv-step
.venv-step/Scripts/python -m pip install --require-hashes -r tools/step_convert/requirements.txt   # bin/python on Linux, macOS
```

`spade_mesh_pack` is built with the rest of the project (`scripts/build.ps1 -Target spade_mesh_pack`); it ends up under
`build-ninja/<preset>/bin/`.

## Run

```
.venv-step/Scripts/python tools/step_convert/step_convert.py bracket.step --out parts --deflection 0.1mm \
    --pack build-ninja/release/bin/spade_mesh_pack
```

| Option | Meaning |
|---|---|
| `--out DIR` | where the sidecars go, one `<name>.spmesh` per solid (names come from the STEP, made safe for a file name and unique) |
| `--deflection LEN` | **required, no default.** The largest distance between a curved face and its triangles, with a unit (`m mm cm um in ft`); a bare number is refused |
| `--angle RAD` | angular deflection, default 0.5 |
| `--solid NAME` | convert only the named solid (repeatable) |
| `--pack PATH` | the `spade_mesh_pack` executable (or set `SPADE_MESH_PACK`) |

**Choosing the deflection.** It is a geometric choice, not a quality knob: a mesh of a curved part is always a little
inside it. Take `d` at most a tenth of the smallest feature that matters to the flow, and no finer than the airflow grid
can see. A mesh inscribed in a cylinder is short of its volume by about `(2/3) d` times the curved area. The cross-check
below tells you what you got.

**The cross-check.** OpenCASCADE's own B-rep volume of each solid is printed beside the mesh's, with the relative
difference. Above 1% the solid is marked `FLAGGED` and the exit code is 5 (the sidecars are written all the same): use a
smaller `--deflection` or `--angle`.

## What is recorded

Each sidecar carries, outside its content hash: the **unit** the STEP declared, the **tolerance** (`--deflection`, in
metres), and a **source** such as
`bracket.step [STEP; OCCT 7.9.3 via cadquery-ocp 7.9.3.1.1; deflection 0.1mm, angle 0.5 rad]: arm_left`.
The mesh itself is in metres. Nothing about the tessellation is golden: a different OCCT may place triangles differently.

## What it refuses (nothing is repaired)

| Exit | Meaning |
|---|---|
| 0 | written |
| 1 | an input, output or the packer failed otherwise |
| 2 | a usage error (nothing read) |
| 3 | OCP is not installed, the STEP cannot be read, or it holds what is not read: no solid (shells, surfaces, wires), no length unit or more than one, a unit other than `m mm cm um in ft` |
| 4 | `make_mesh()` refused a solid (open, non-manifold, self-intersecting, ...); the packer names it and says why. Change the deflection or fix the CAD |
| 5 | written, but the cross-check flagged a solid |

Solid instances are meshed one by one in their own frame and placed by the assembly's transforms. **Names:** an
instance is named after the assembly's occurrence, else the part (a part of several solids adds `_1`, `_2`, ...). When
names are already unique nothing changes. When several instances carry the same name (a motor placed four times, the
exporter leaving the occurrences unnamed) each gets `_1`, `_2`, ... in the order the assembly lists them, so they come
out as `Motor_1` .. `Motor_4`, and `--solid Motor_3` picks one (the bare `Motor` is then a usage error: it names
four things). If a suffixed name lands on another instance's real name (`Motor_2` already exists), the tool
refuses (exit 3) naming both and writes nothing; it never guesses. Triangles that collapse where nodes of neighbouring faces meet are dropped, and
those nodes are one vertex when they are within a millionth of the deflection of each other (compared in double: 1e-7 mm
at 0.1 mm). OpenCASCADE gives bit-equal nodes along a shared edge, so the weld is for rounding; a seam it ever missed would
leave an open edge, which `make_mesh()` refuses (exit 4) rather than writing a mesh with a hole.

## Tests

```
SPADE_STEP_PYTHON=.venv-step/Scripts/python ctest --test-dir build-ninja/release -L step
```

The tests carry the label `step`, never `spade`, so neither gate runs them. They run on the development box on any branch
touching `tools/step_convert/` or `tools/mesh_pack/`. OCP is found by `import OCP` in the python running the test or by
`SPADE_STEP_PYTHON`; without it the cases that need it **skip** (exit 77) -- a skip is not a pass, and the run is green only
with 0 skipped. The fixtures in `tests/golden/step/` (a 10 mm cube, a cylinder with a through hole, a two-solid assembly
with a quarter turn) are plain-text Part 21 written by `gen_fixtures.py` with no CAD kernel, so the tests check OpenCASCADE's
reading against the specification and against analytic volumes, not against itself.

## Licence

OpenCASCADE is LGPL-2.1 with an exception. This script runs it as its own process; Spade never links, loads or
ships it, and the wheels are installed by the user from `requirements.txt`, not from this repository.
