# The D1 hover preview's checks and 04 rows

**Owner:** Test/Docs. **Status:** DRAFT, 2026-10-21. The claims (§2) are for Physics to agree, and the predicates (§1) for Interface. The lead assigned it on 2026-10-21.
**The case:** the library's reference quad held level at hover, solid, in Physics' 0.896 × 0.672 × 0.64 m box at `dx` 4 mm (224 × 168 × 160 cells, 6.02 M), run through `LiveRun` once Interface's `interface/liverun-solids` lands.
**Fixed a priori:** each band is fixed when Physics and Interface agree, before the first D1 run. After that, a band changes only by a recorded lead decision (`TD-2`, `01-verification.md`).

The rules of the end-goal checks hold here (`2026-10-19-end-goal-checks.md` §0):
- Each check is a predicate on state the run produced.
- There is one display-free implementation in `sandbox/end_goal_checks.hpp`, called by the tour, the film and a headless test.
- Each check is red first.
- `absent:` and `mis-staged:` verdicts never pass.

## 1. What the film and the tour assert

### D1-1. `solids-in-flow`: the run line names the body

- **Input:** `LiveRun::solids()`'s `any` and `run_line` (`interface/liverun-solids`), and the model's rigid part count (`PartFilter::rigid_only`).
- **Predicate:** `any` is true, and the run line starts `solids: P parts, W walls, S sheets` with `P = W =` the rigid part count and `S = 0`.
  - For the reference quad that is `solids: 10 parts, 10 walls, 0 sheets`. Its parts are the body, four arms, four motors and the battery.
  - The smallest `min_feature` is the arms' 12 mm, which is at least 2 `dx` = 8 mm, so every part is a wall (`tests/test_vehicles_airflow_solids.cpp`, `TheReferenceQuadAtHoverIsTenCleanWallsAt4Millimetres`).
- **Band:** exact; these are counts.
- **Catches:**
  - The parts cleared before the run. `sandbox/airflow_live.hpp` and `tools/flow_movie/live_case.cpp` do this today with `run_model.parts.clear()`. The verdict is `absent: no solids in the run`.
  - Rotor parts let through the filter (`P` ≠ the rigid count): `mis-staged:`.
  - A part classed as a sheet at 4 mm (`S > 0`): a fail that names the part.

### D1-2. `no-leak`: no leak announced at 4 mm

- **Input:** the same run line.
- **Predicate:** none of the safeguard's advisories follow the counts:
  - no `leaks at an edge`;
  - no sheet note `no closed face` (`engine/physics/airflow/solids.cpp`, the measured safeguard).
- A part that leaks outright fails `start()` by name. The run then never starts, and every check is `absent:`.
- **Band:** exact.
- **Catches:** a leaking classification. At 6 mm the same quad announces `parts[3]: leaks at an edge (...)`, pinned in the same test file. That line is this check's red.

### D1-3. `downwash-present`, with the body in the flow

- **Predicate:** the existing `kDownwashPresent`: a floor of 0.25 of momentum theory's `1.894 v_h` at 2R below each hub, at `R/dx` ≥ 2 (`2026-10-19-end-goal-checks.md` §5).
  - The premises are unchanged: level within 1°; ΣT at least 0.5 m g; the window `[max(0.30 s, 4L/v_min), +0.20 s]` after arming; at least 6 frames.
  - One premise is added: D1-1 passes in the same run. Otherwise the verdict is `mis-staged: the body is not in the flow`, so a run with its parts cleared can't pass for D1.
- **Geometry:** each probe disc is 0.5R = 38 mm across, 2R = 152 mm below a hub at ±110 mm. That is about 126 mm below the body's centre. No part reaches it, the battery included. A probe point that falls in a solid reads no-data, which is `absent:` and means the probe is misplaced.
- **The body in the wake:** each motor (28 mm across, 0.18R) sits under its hub inside the probe disc. An arm and its motor shade about 7.5% of each disc's area (the arm's 12 mm width from 34 to 96 mm out, 4.1%; the motor, 3.4%), well inside a 0.25 floor.
- **Band:** the floor, unchanged. A magnitude claim is the 04 row's (§2).

### D1-4. `cp-note-shown`: the C_p legend note

- **Predicate, every view:** `legend_shown` also compares `ViewLegend::note`. Today it compares the layer, quantity, unit, colours and range, but not the note.
- **Predicate, D1:** the `surface_cp` legend's note is the solids variant, which begins `the fluid's pressure one cell off the wall` (`engine/render/view_mode.cpp`, when `extrapolated_cells > 0`). The other variant, `the walls are not in this flow`, is `mis-staged:` for D1.
- **The film:** its caption carries the note verbatim (`consumers.md`: "Show it (Interface: the per-view legend; the film's caption)").
- **Band:** exact strings.
- **Catches:**
  - The note dropped. The editor's legend UI doesn't draw it today (`sandbox/editor_panels_ui.cpp`).
  - A film of C_p drawn with no body in the flow.

### D1-5. `publishes`, with solids

- **Predicate:** `publishes` passes only with `no_data == 0` today. With solids, `publish()` writes the first solid layer from its fluid side (`extrapolated_cells`) and marks the deeper solid cells no-data (`engine/physics/airflow/publish.hpp`). So with solids, `no_data` = the grid's solid cells − `extrapolated_cells`, exactly. Without solids it stays 0.
- **Band:** exact.
- **Catches:** no-data leaking into the fluid, and a solid mask that never reached `publish()`.
- **Needs:** the grid's solid cell count from `LiveRun` (§4).

### On the film's card and caption

These come from `Videos\CATEGORIES.md`, not new checks:
- The card and the caption name the stand-ins in plain words: the drone held at hover, not flying, and the rotors as actuator discs.
- No body force is stated below P3's floor (`physics/04-verification.md`'s grades; `d2d03ed`).

## 2. 04 rows, proposed for Physics

**D1** (hover preview, CPU reference, MSVC release, `LiveRun`, 4 mm; `R/dx` = 76.2/4 = 19):

| Measure | Tolerance | Built from |
|---|---|---|
| The solids line | `10 parts, 10 walls, 0 sheets`, no advisory | exact (D1-1, D1-2) |
| Steps whose pressure solve was announced unconverged (`> kResidualAnnounce`, 1e-4) | 0, or report only (Physics) | P2's S2 reached 1.7e-6 at `n_v` 4; D1 is 6 M cells at `kSolidsCgIterations` |
| Downwash at 2R below each hub over `1.894 v_h` | `kDownwash`'s 0.75 to 1.25 at `R/dx` ≥ 4, or report only (Physics) | The headless band (`2026-10-19-end-goal-checks.md` §5). The body shades about 7.5% of each disc |
| Surface C_p | report only | first order in `dx`; where stagnation and suction are, not validated magnitudes (the note) |
| Body force | not stated below P3's floor | the lead's ruling under `TD-2`, 2026-10-21 |

## 3. Red first

The work goes on branch `test-docs/d1-checks`. Each test fails on a stub of its fault, then passes:
- `solids_in_flow`:
  - `absent:` on `solids: none` and on `any` false;
  - `mis-staged:` on 14 parts;
  - a fail on `9 walls, 1 sheets`;
  - a pass on the 4 mm line.
- `no_leak`: a fail on the 6 mm line (`parts[3]: leaks at an edge`) and on a `no closed face` note; a pass on the 4 mm line.
- `downwash` with the body: `mis-staged:` when D1-1 fails. Otherwise its verdict is the existing check's.
- `legend_shown`:
  - a fail when the shown note differs or is empty;
  - D1's `mis-staged:` on `the walls are not in this flow`.
- `publishes` with solids:
  - a pass at `no_data` = solid − extrapolated;
  - a fail at one more;
  - a fail on any no-data with no solids.

The predicates take plain inputs (`any`, the run line, counts), so they don't wait on `interface/liverun-solids`.

## 4. Asked of others

**Physics:**
1. The box isn't in the repo yet; the nearest 4 mm rig is `tests/solid_drone_rig.hpp`'s 96 × 32 × 96. For the 0.896 × 0.672 × 0.64 m box: which axis is up, where the hubs sit, the boundaries (open, walls, a floor), and `ν`.
2. The run's length. The downwash premise needs at least `max(0.30 s, 4L/v_min) + 0.20 s` simulated, with at least 6 frames. Does the preview reach that at 4 mm on the CPU? If not, D1-3 is `absent:` by its premise, and the preview claims no downwash. Please say which.
3. The downwash row: `kDownwash`'s band with the body, or report only?
4. The pressure row: 0 unconverged steps, or report the count?
5. Any other claim D1 makes, such as C_p's sign on the arms' tops under the discs.

**Interface:**
1. `LiveSolids` gains the grid's solid cell count for D1-5. Or say how `publishes` should get it.
2. The tour and the film keep the model's parts. `sandbox/airflow_live.hpp:131` clears them, and so does `tools/flow_movie/live_case.cpp` (Rendering's).
3. The editor's legend draws `note`, and the film's caption carries it.
4. The predicates' inputs: plain values, so they don't depend on the unmerged branch:
   - `SolidsLine{bool any; std::string run_line; std::size_t expected_parts;}`
   - `Published` gains `expected_no_data`.
   - `LegendCheck` gains nothing; the note is compared.
