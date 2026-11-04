# The STEP converter (CORE-6 D): plan

**Status:** proposed to the lead, 2026-11-03. No code is written until the lead has checked the dependency against Cameron's `CORE-6` sign-off ("Meshes + STEP converter"; no OpenCASCADE or any B-rep kernel in the engine).

**Goal.** Take a STEP file once, offline, to the engine's own mesh sidecars (`.spmesh`, `world/mesh_file.hpp`): one closed, validated mesh per solid, named, placed, in metres, with the tolerance it was tessellated at recorded in its `MeshProvenance`. This is the design's option D (`2026-10-12-model-parts-design.md` §2): "a script and a page of docs".

## 1. The STEP reader and its licence

A STEP solid is a B-rep: trimmed NURBS and analytic faces. Reading one into triangles needs a geometry kernel to evaluate and trim those faces, not just a Part 21 parser.

**The non-copyleft readers.** I surveyed these from what I know; I have not run a 2026 spike on any of them.

| Reader | Licence | Why not |
|---|---|---|
| STEPcode (ex-SCL) | BSD-3 | It parses Part 21 into EXPRESS entities, and has no B-rep evaluation and no tessellation. Spade would have to write a trimmed-NURBS tessellator, a large job and an `L6` risk |
| truck (`truck-stepio`, `truck-meshalgo`; Rust) | Apache-2.0 | It reads a subset of AP203/AP214 and was incomplete on real CAD exports when I last knew it. It also brings a Rust toolchain. Worth a one-hour spike if the lead wants a non-copyleft path |
| Foxtrot (Rust) | MIT/Apache-2.0 | An experimental viewer, unmaintained |
| Assimp | BSD-3 | It reads IFC's STEP, not mechanical-CAD B-reps |
| HOOPS Exchange, CAD Exchanger | commercial | Out of scope |

**None is good enough.**

**Recommendation: OpenCASCADE (OCCT 7.9.3, LGPL-2.1 with the OCCT exception), as a separate offline process.** It comes through the `cadquery-ocp` Python wheels, pinned exactly (`cadquery-ocp==7.9.3.*`, the exact build fixed in S2).
- **Nothing in Spade links OCCT.** It has no CMake target, is not in any installed library, is not in the default build, and is not in the consumer smoke test. The converter is a Python script that the user runs when asked.
- **Spade's Apache-2.0 is untouched.** OCCT runs in its own process (the design's own argument for D), and Spade distributes neither OCCT nor the wheel: the user installs it with pip.
- **The same OCCT on Windows and Linux** comes from one pin, with no 40-minute OCCT build on either platform. Ubuntu noble's packaged OCCT is 7.6.3, which would leave the two platforms on different kernels.
- **The wheel bundles OCCT and more** (VTK among them, BSD-3). S2 lists the wheel's licence files on the docs page. The wheel is a user-side tool, so it gets no `THIRD_PARTY_NOTICES.md` row, and the page says why.
- **Alternative,** if the lead wants one language: a C++ tool under `tools/step_convert/` that `find_package`s a prebuilt OCCT, OFF by default. It is heavier, because each platform needs its own OCCT install and version skew must be managed. It is otherwise the same plan.

## 2. Shape: two pieces, the copyleft one outside

1. **`spade_mesh_pack`** (C++, Apache-2.0, `tools/mesh_pack/`, no new dependency).
   - It reads any file `spade_mesh_import` reads (glTF, binary STL, OBJ, 3MF).
   - It writes one `.spmesh` per object through `save_mesh_file()`, with `MeshProvenance{source, unit, tolerance}`.
   - It needs only `make_mesh()` and the content hash, which already exist, and it is useful on its own: Kat can pack a glTF or 3MF into sidecars.
2. **`tools/step_convert/step_convert.py`** (Python with OCP).
   - It reads the STEP through `STEPCAFControl_Reader`, keeping part names (XCAF labels), placements and the file's units.
   - It tessellates each solid (§3) and writes a core-specification 3MF: one object per solid, named, its placement as the build item's transform, in the file's unit.
   - It then runs `spade_mesh_pack` on that 3MF, so every mesh passes Spade's own reader and `make_mesh()` before a sidecar exists.
   - 3MF is the hand-off because names, placements and units already survive it, and `read_3mf()` is tested.

## 3. Tessellation, and the provenance block

- **The mesher:** `BRepMesh_IncrementalMesh(solid, d, /*isRelative*/ false, a, /*isInParallel*/ false)`.
  - The linear deflection `d` is absolute. It is given in metres and converted to the file's unit.
  - The angular deflection `a` is in radians.
  - The mesher runs single-threaded, so a run repeats (design §2, A's accuracy note).
- **`d` is required on the command line, with no default.** The chord error is a geometric choice (`L6`), and the right value depends on the part and on the airflow grid it will meet. The docs page gives the rule of thumb: `d` at most a tenth of the part's declared `min_feature`.
- **`a` defaults to 0.5 rad** (OCCT's own) and is printed in the source text.
- **One topology across faces.** BRepMesh triangulates face by face, and a shared edge's nodes are computed per face, so they need not be bit-equal. Spade's weld is exact (no tolerance), so the script stitches by topology instead. Each edge's discretisation comes from its `PolygonOnTriangulation`, and each node is numbered once per edge, before positions are written. Without this, a perfectly good solid would come back as "open edges".
- **Validation, never repair.**
  - `make_mesh()` checks every mesh in the packer: closed, oriented, manifold, no self-intersection, correctly nested.
  - A refusal is reported with the solid's name and `make_mesh()`'s message. The user changes `d` or fixes the CAD.
  - Nothing is healed: OCCT's `ShapeFix` is not run unless asked (`--fix-shape`, off by default, and recorded when used).
- **The provenance block** (`MeshProvenance`, outside the content hash):
  - **`tolerance`:** `d` in metres.
  - **`unit`:** the unit the STEP file declared.
  - **`source`:** for example `"bracket.step: arm_left (STEP AP214; OCCT 7.9.3 via cadquery-ocp 7.9.3.x; deflection 1e-4 m, angle 0.5 rad)"`.
- **The B-rep cross-check,** as the design asks: OCCT's `GProp` volume and centroid of each solid beside the mesh's own mass properties.
  - It is printed as a table: name, B-rep volume, mesh volume, relative difference.
  - It is a cross-check, not a gate. A relative volume difference above 1% is flagged in the output and the exit code says so (§4).

## 4. The CLI

```
python tools/step_convert/step_convert.py IN.step --out DIR --deflection 0.1mm [--angle 0.5]
                                          [--solid NAME]... [--fix-shape] [--pack PATH/spade_mesh_pack]
spade_mesh_pack IN.{gltf,glb,stl,obj,3mf} --out DIR [--unit U] [--tolerance M] [--source TEXT]
```

- **Output:** one `DIR/<name>.spmesh` per solid. Names come from the STEP labels, are made safe for a file name, and are made unique as `name`, `name-2`, and so on; the mapping is printed.
- **Lengths** take a unit suffix (`mm`, `m`, `in`). A bare number is refused, so a unit is never guessed.
- **Exit codes:**
  - 0: written;
  - 2: usage;
  - 3: OCP is not installed, or the STEP uses what this tool does not read, named;
  - 4: a solid was refused by `make_mesh()`, named;
  - 5: written, but the cross-check flagged a solid.

## 5. Tests, and how the gates treat the optional dependency

- **`spade_mesh_pack`: ordinary ctest tests**, label `spade`, built in every configuration and run by both legs.
  - The test writes a 3MF with miniz (as `test_mesh_import.cpp` does) and packs it.
  - Each `.spmesh` loads back through `load_mesh_file()`, with the provenance as given and the content hash equal to `read_3mf()`'s mesh.
  - Also tested: a unit refusal for STL and OBJ, and an object `make_mesh()` refuses, named.
- **`step_convert.py`: ctest tests labelled `step`**, registered only when CMake finds a Python.
  - **Fixtures:** three small STEP AP214 files under `tests/golden/step/`, written once with OCP and committed with a provenance note:
    - a 10 mm cube;
    - a cylinder with a through-hole (curved faces, genus 1);
    - a two-solid assembly with a rotated placement.
  - **Each test checks:**
    - the sidecars load, which proves closed and manifold;
    - the names and placements;
    - each volume within the bound `d` implies of the analytic volume.
  - **No bit goldens.** The tessellation is OCCT's, and a golden would tie Spade's tests to OCCT's version.
- **The skip rule, read as `TD-13` reads GPU tests.**
  - Without OCP, a `step` test skips and names why, and a skip does not count toward a green gate.
  - Both gates exclude the label explicitly (`-LE step`, beside `-LE gpu`).
  - The Docker leg's image does not install OCP.
  - The `step` tests run on the development box at the batch gate whenever `tools/step_convert/` changes.
  - This needs a Test/Docs row beside `TD-13`. That is Test/Docs' call, and S2 waits on it.

## 6. The page of docs

`docs/tools/step-converter.md` (or wherever Test/Docs keeps user-facing tool pages) covers:
- installing OCP (`pip install -r tools/step_convert/requirements.txt`, the exact pin);
- the two commands;
- choosing `d` (the `min_feature` rule, and what a coarser `d` costs in volume and in the airflow wall);
- what is refused and why (no repair, `L6`);
- reading the provenance and the cross-check;
- the licence position: OCCT and the wheel are LGPL and other licences, run as a separate process, and never shipped with Spade.

## 7. Tasks

| Task | What | Gate |
|---|---|---|
| S1 | `spade_mesh_pack`, RED then GREEN, with its ctest tests | the usual per-branch tiers, gcc |
| S2 | `step_convert.py`, its `requirements.txt` pin, the three fixtures, the `step`-labelled tests | the dev box with OCP; the legs `-LE step`; Test/Docs' row first |
| S3 | The docs page, 07-status, the CHANGELOG, and the consumers line drafted for the lead | docs only |

S1 needs nothing from the lead's dependency decision and can start on its own.

## 8. Questions for the lead

1. OCCT through the pinned `cadquery-ocp` wheels, out of process (recommended), or a C++ tool that `find_package`s OCCT, OFF by default?
2. A one-hour spike on truck first, for a non-copyleft path? I expect it to fall short on real exports.
3. The `step` label, excluded from both gates like `gpu` (Test/Docs' row)?
4. `d` required on the command line with no default (recommended), or a default recorded in the provenance?
5. `spade_mesh_pack`: installed beside `spade_analysis_runner` (Kat could pack glTF and 3MF itself), or build-tree only?
6. Where the user-facing page lives.
