# The D1 hover preview's checks and 04 rows

**Owner:** Test/Docs. **Status:** approved by the lead as written (`0076fd1`), 2026-10-21. That covers the bands fixed a priori, D1-1 as a premise of D1-3, and the 0.10 floor with its reason, the wind. Physics agreed the claims (§2) and Interface the predicates (§1), both on 2026-10-21. The checks are on branch `test-docs/d1-checks`.
**The lead's rulings, 2026-10-21:**
- **Physics writes D1's record**, since the claims and bands are Physics' and the records live with Physics. Rendering's run supplies the `published_pressure_at` samples at every published frame in D1-3's window (§2's arm-top row), and the record carries the film's run label. Test/Docs checks the record against §2.
- **The run's length:** Rendering first measures the step time on the full 224 × 160 × 168 box. Then it simulates at least 0.55 s (0.50 s plus a margin) in resumable chunks. If a step costs more than 30 s, the lead hears about it before the run starts.
**The case:** the library's reference quad held level at hover, solid, in a 3 m/s wind, run through `LiveRun` once Interface's `interface/liverun-solids` lands. The box is Physics' proposal to Rendering, not yet built in the repo:
- 224 × 160 × 168 cells at `dx` 4 mm (6.02 M), so 0.896 m along x, 0.64 m along y and 0.672 m along z. Up is +y, the rotor axes are along +y, and the wind blows along +x.
- The centre of mass is 0.416 m from the inlet face, 0.354 m above the bottom, and centred in z. The hubs sit ±0.110 m from it along x and z, and 0.0264 m above it: the rotor plane is at y ≈ 0.380 m, and the tips reach ±0.186 m.
- `ν` = 1.5e-5 m²/s (air), so the viscous path runs, with the wall links and the rigid interior. The cell Reynolds number is about 2700.
- The boundaries are whatever `LiveRun` maps the case to (Interface), until P2b builds the drone box's own. Physics assumed inflow on −x, outflow on +x and at the bottom, and the sides and top open. A floor, if the case sets one, makes the bottom a wall, and the box is then redone.
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
  - The parts cleared before the run. The verdict is `absent: no solids in the run`.
    - D1 runs through `tools/flow_movie/live_case.cpp` (Rendering's), which clears them today with `run_model.parts.clear()`. It must keep them. `LiveRun` takes the model's rigid parts whenever the model has any (Interface).
    - The tour's airflow chapter (`sandbox/airflow_live.hpp`, 32³ at 25 mm) clears them on purpose: the quad's parts can't be held at 25 mm. Its card says the parts are drawn, not in the air. So it is honestly `absent:`, and not a D1 path.
  - Rotor parts let through the filter (`P` ≠ the rigid count): `mis-staged:`.
  - A part classed as a sheet at 4 mm (`S > 0`): a fail that names the part.
  - Interface's review: `AirflowSolids` also passes porous rigid parts to `set_solids()`, so they count in `P`. A model with one would fail as "not every part is a wall". That is right for D1, since the library quad has none. If a porous part ever appears, the verdict should name it as porous (a follow-up).

### D1-2. `no-leak`: no leak announced at 4 mm

- **Input:** the same run line.
- **Predicate:** none of the safeguard's advisories follow the counts:
  - no `leaks at an edge`;
  - no sheet note `no closed face` (`engine/physics/airflow/solids.cpp`, the measured safeguard).
- A part that leaks, or that the grid can't hold, fails `start()` and names the part. For example: `the model's parts as solids: airflow solids: part 'body' is below 2 dx = 40.0 mm in two extents (…) (refused, never dropped)` (Interface, measured). The run then never starts, and every check is `absent:`.
- **Band:** exact.
- **Catches:** a leaking classification. At 6 mm the same quad announces `parts[3]: leaks at an edge (...)`, pinned in the same test file. That line is this check's red.

### D1-3. `downwash-present`, with the body in the flow

- **Predicate:** the existing downwash check (`2026-10-19-end-goal-checks.md` §5): the mean axial velocity on a probe disc 2R below each hub, over momentum theory's `1.894 v_h`, judged against a floor at `R/dx` ≥ 2. D1's floor is below.
  - The premises are unchanged: level within 1°; ΣT at least 0.5 m g; the window `[max(0.30 s, 4L/v_min), +0.20 s]` after arming; at least 6 frames.
  - One premise is added: D1-1 passes in the same run. Otherwise the verdict is `mis-staged: the body is not in the flow`, so a run with its parts cleared can't pass for D1.
- **Geometry:** each probe disc has radius 0.5R = 38 mm, and sits 2R = 152 mm below a hub at ±110 mm. That is about 126 mm below the body's centre and 228 mm above the box's bottom. No part reaches it, the battery included. A probe point that falls in a solid reads no-data, which is `absent:` and means the probe is misplaced.
- **The run's length (Physics):** the window is `[max(0.30 s, 4L/v_min), +0.20 s]`. With `L` = 0.38 m from the rotor plane to the bottom and `v_min` = 5.0 m/s, 4L/v_min is 0.30 s, so the run needs 0.50 s simulated. At a peak speed of about 10.5 m/s, `h` is about 0.38 ms at CFL 1, so about 1300 steps. The step time on 6 M cells with solids isn't measured. Physics estimates 10 to 20 s a step on the CPU, so 4 to 7 hours. Rendering's rig gives the real figure. **If the preview stops short of 0.50 s, D1-3 is `absent:` by its premise, and the preview claims no downwash.**
- **The floor for D1: 0.10, not the tour's 0.25.** It is fixed here, before any D1 run, because this case has a wind and the tour has none:
  - The probe sits on the rotor axis. The 3 m/s wind leans the wake: across a mean descent of about 7.8 m/s (from `v_h` ≈ 5.4 at the disc to 1.894 `v_h` ≈ 10.2 at 2R), the lean is 3/7.8 = 0.385, so at 2R the wake's centre is 0.77R downwind of the probe's.
  - A contracted wake of radius about 0.75R then covers about 40% of the probe disc.
  - Inside the wake the ratio is about (1 − 0.17)(1 − 0.075) ≈ 0.77: the body shades about 17% of each disc (below), and the wind lowers the induced velocity by about 7.5% (Glauert).
  - So the estimate is 0.40 × 0.77 ≈ 0.31, a top-hat wake's lower bound.
  - The tour's 0.25 would leave only 1.2 times that estimate. A floor of 0.10 leaves 3.1 times, and is still about 1 m/s downward, well clear of the zero an uncoupled or idle rotor gives. A reversed or absent downwash still fails.
- **The body in the wake** (corrected 2026-10-21, before any run: the first figure, 7.5%, left out the body box; the floor stands): each disc's area is shaded by about 17% in all, and three parts sit below it:
  - The body box's corner, half-width 60 mm, reaches under each disc from 34 to 60 mm out: the segment past 0.66R from the hub, 11.5%. It lies outside the 0.5R probe disc's own radius.
  - The arm's 12 mm width shades from 60 to 96 mm out: 2.4%.
  - The motor, 28 mm across (0.18R), sits under its hub inside the probe disc: 3.4%.
- **Band:** a floor of 0.10 of `1.894 v_h` at `R/dx` ≥ 2 (`kDownwashPresentInWind`). The magnitude is report-only in the 04 rows (§2).
- **D1's run, 2026-10-22:** not measured: frames deleted before sampling (2026-10-22); to be re-run on the GPU solver. Never a pass or a fail (§2, "D1's run").

### D1-4. `cp-note-shown`: the C_p legend note

- **Predicate, every view:** `legend_shown` also compares `ViewLegend::note`. Today it compares the layer, quantity, unit, colours and range, but not the note.
- **Predicate, D1:** the `surface_cp` legend's note is the solids variant, which begins `the fluid's pressure one cell off the wall` (`engine/render/view_mode.cpp`, when `extrapolated_cells > 0`). The other variant, `the walls are not in this flow`, is `mis-staged:` for D1.
- **The editor:** its per-view legend draws the note (Interface, on `interface/liverun-solids`).
- **The film:** its caption carries the note verbatim (`consumers.md`: "Show it (Interface: the per-view legend; the film's caption)"). The caption is Rendering's.
- **Band:** exact strings.
- **Catches:**
  - The note dropped. The editor's legend UI doesn't draw it today (`sandbox/editor_panels_ui.cpp`).
  - A film of C_p drawn with no body in the flow.

### D1-5. `publishes`, with solids

- **Predicate:** `publishes` passes only with `no_data == 0` today. With solids, `publish()` writes the first solid layer from its fluid side (`extrapolated_cells`) and marks the deeper solid cells no-data (`engine/physics/airflow/publish.hpp`). `no_data` counts samples, one per channel (`publish.cpp` writes `kNoDataBits` to each of a deeper cell's `kFlowChannelCount` = 8 channels; Interface). So with solids, `no_data` = 8 × (the grid's solid cells − `extrapolated_cells`), exactly. Without solids it stays 0. An fp16 overflow would add to it, and a sane run has none.
- **Band:** exact.
- **Catches:** no-data leaking into the fluid, and a solid mask that never reached `publish()`.
- **Input:** `LiveSolids::solid_cells` (Interface, on `interface/liverun-solids`). It is counted from the same mask the run publishes with (`scratch.regions.solid`, the CG mask), so it and `no_data` agree by construction. The interior is `solid_cells − extrapolated_cells`, constant over the run because the solids are posed once.

### On the film's card and caption

These come from `Videos\CATEGORIES.md`, not new checks:
- The card and the caption name the stand-ins in plain words: the drone held at hover, not flying, and the rotors as actuator discs.
- No body force is stated below P3's floor (`physics/04-verification.md`'s grades; `d2d03ed`).

## 2. 04 rows (Physics agreed, 2026-10-21)

**D1** (hover preview, CPU reference, MSVC release, `LiveRun`, 4 mm; `R/dx` = 76.2/4 = 19):

| Measure | Tolerance | Built from |
|---|---|---|
| The solids line | `10 parts, 10 walls, 0 sheets`, no advisory | exact (D1-1, D1-2). "No advisory" means the safeguard's notes only, `; <part>: leaks at an edge (...)` and a sheet's `no closed face`, on `LiveRun::solids().run_line`. The unconverged note is on `LiveRun::run_line()`, and only the next row reads it |
| Steps whose pressure solve was announced unconverged (`> kResidualAnnounce`, 1e-4) | report only: two counts, before D1-3's window and within it, each with its largest residual. The rule below holds within the window | `kSolidsCgIterations` (8) was measured from a cold start on S3 and P2's cases, not on 6 M cells around the quad. The steps run warm, but Physics won't claim zero. The run announces each such step (L6) |
| Downwash at 2R below each hub over `1.894 v_h` | report only | No headless run has solids. Three effects stack: the body's shade (about 17%), the wind's lower induced velocity (about 7.5%, Glauert), and the arms' own wakes (unmeasured). D1-3's floor is the check |
| The sign of C_p on each arm's top, under its disc: per arm, the mean C_p over samples one cell above the arm's own top, from where the arm leaves the body (or the disc's inner edge, if that is farther out) to the motor's edge, against the inflow's pressure. On this quad that is 0.06 to 0.096 m, about 9 samples per arm at 4 mm. The body box (half-width 0.06 m) is a different surface at a different height, so it is left out (Physics and Rendering, 2026-10-21, before any run). The samples follow RND-8's rule, trilinear in the published cell-centre pressure with P6's first layer, as S1's P3 test reads them (`published_pressure_at`, `tests/test_airflow_solids_step.cpp`). **In time** (the lead, 2026-10-22, before any run): the mean over D1-3's window (`[max(0.30 s, 4L/v_min), +0.20 s]` after arming, every published frame in it, at least 6), never one frame. The wake is chaotic past its horizon, so one frame is one trajectory's instant (`2026-10-22-c8-spread.md`; 01-verification, "A case judged past its horizon") | > 0 on every arm; the magnitude report only | The disc raises the static pressure below it above the free stream, and the downwash stagnating on the arm adds to it (Physics: the sign is robust). Physics' record measures it |
| Surface C_p elsewhere | report only | The readings sit 0.5 to 1 `dx` off the wall and are first order in `dx`: where stagnation and suction are, not validated magnitudes (the note) |
| An arm's wake | shape only | It is 1 to 2 cells wide |
| Body force | none stated below P3's floor (`c₀` ≈ 0.02 to 0.03) | the lead's ruling under `TD-2`, 2026-10-21 |

**Unconverged pressure steps.** The lead approved this, and Physics signed its threshold, on 2026-10-22. It was fixed before any frame was judged, after D1's run (`2ddb2b9`) announced step 1 from rest at 1.26e-4 at the 32-iteration cap.
- **No check reads them.** D1-1 and D1-2 read `LiveRun::solids().run_line`, which never carries the note. D1-3's premises are level, ΣT, the window and the frame count. D1-4 looks only for its own note, and D1-5 counts no-data. So an announced step is neither mis-staged nor a fail.
- **(a) The record's row stays report only.** It gives two counts, before the window and within it, each with its largest residual. They are read from Rendering's `convergence.csv`, one row per exported frame: frame, t, steps, the running unconverged count since the start or the last restore, and the last residual. Rendering's film takes the build that writes it before frame 9 (t = 0.30 s). The count before the window is the run line's announcement up to that switch, plus any of the file's rows before 0.30 s. The window's count is the growth of the file's running count across the window's frames. Iterations per step are not recorded in this run: LiveRun doesn't expose them.
- **(b) Before the window** (t < 0.30 s), any count is allowed. Each step projects afresh, so a start-up step's leftover divergence doesn't carry into the window.
- **(c) Within the window,** an announced step is allowed while every one ends under 1e-3. That is P2's cold-solve bar (the solids design §3: `kSolidsCgIterations` was chosen so cold solves reach 1e-3 and warm ones 1e-4). The step's leftover divergence is then under a thousandth of its own. That is far inside D1-3's margin of 3.1× over its floor, and it can't move the arm-top C_p sign, which is a mean over at least 6 frames (Physics).
- **(d) Over 1e-3 within the window,** D1-3 and the arm-top C_p row are mis-staged, not failed. The run is redone with a higher CG cap rather than judged.
- **If the convergence log misses the window,** the record states "convergence not logged in this run", and (c) and (d) can't be judged. D1-3 and the arm-top C_p sign are still reported with their own verdicts, each with that limitation named beside it. Neither is passed silently. The same applies within the window when the count grows by more than one between two frames: the residuals between them are not logged, and the record names it.

**D1's run, 2026-10-22** (exe `1f5fb1a`; 17 frames, 1492 steps, t = 0.567 s). The render ran before the arm-pressure pass without `--keep-cache`, and it deleted the frame cache. So neither the probe discs nor the arm-top samples in D1-3's window were taken. Cameron decided not to re-run until the GPU solver makes it quick (backlog, 2026-10-22). The video, the run line and the convergence log survive. Physics writes the record. This doc states what the checks above can say:
- **D1-1 and D1-2** read the run line, which survives: `solids: 10 parts, 10 walls, 0 sheets`. They are judged from it, in Physics' record.
- **D1-3 and the arm-top C_p row:** not measured: frames deleted before sampling (2026-10-22); to be re-run on the GPU solver. Never a pass or a fail. The downwash row's report-only magnitude has the same probe samples, so it has no value either.
- **D1-4, the film's caption:** judged from the posted video's caption once Rendering posts it. It passes if the C_p note is there as D1-4 requires, and fails if not. **The editor's legend:** not this run's: judged in the editor's own check (the lead, 2026-10-22).
  - **Judged 2026-10-22, on `Videos/Research/Spade/2026-10-22_1202_d1-hover-parts-in-flow`: passes.** The C_p view's burned-in caption carries the solids note verbatim, as `view_mode.cpp` writes it: "the fluid's pressure one cell off the wall, on the seen side: first order in the cell size; where stagnation and suction are, not validated C_p magnitudes" (checked on the frame 22 s in, t = 0.334 s).
  - The posted `.md` caption paraphrases the note, dropping "on the seen side: first order in the cell size;" and "C_p". That is for Rendering to quote verbatim; it doesn't change the film's verdict.
- **D1-5:** not measured: frames deleted before sampling (2026-10-22); to be re-run on the GPU solver. Never a pass or a fail. No surviving file holds `no_data`, `solid_cells` or `extrapolated_cells` for any frame. Rendering searched the chunk logs, the job log, the render log and the clips' burned-in captions. `run.txt` held only each frame's step and run label, and it went with the cache (Rendering, 2026-10-22).
- **Unconverged pressure steps:** the rule has its evidence. `convergence.csv.recovered`, recovered from the run's log, shows no announced step within the window, and each frame's last residual is 2.2e-7 to 5.0e-7. So (c) holds and (d) doesn't arise.
  - Its limit: the file holds each frame's last residual, not the largest residual among the steps between frames.
  - The count covers those steps instead: any step over 1e-4 is announced and counted, and the count didn't grow in the window.

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
  - a pass at `no_data` = 8 × (solid − extrapolated);
  - a fail at one more;
  - a fail on any no-data with no solids.

The predicates take plain inputs (`any`, the run line, counts), so they don't wait on `interface/liverun-solids`.

## 4. Asked of others

**Physics** (answered, 2026-10-21): the box, the run's length, the downwash and pressure rows as report-only, and the arms' C_p sign as a check (the case at the top, D1-3, §2). Still open: the step time on 6 M cells with solids, from Rendering's rig. It decides whether D1-3 can be reached at all.

**Interface** (all agreed, 2026-10-21):
1. `LiveSolids` gains `uint32_t solid_cells` for D1-5.
2. `LiveRun` takes the model's rigid parts whenever the model has any. The tour's 32³ chapter stays cleared, and is not a D1 path.
3. The editor's per-view legend draws `note`.
4. The predicates take plain values, so they don't depend on the unmerged branch:
   - `SolidsLine{bool any; std::string run_line; std::size_t expected_parts;}`
   - `publishes(frames, interior_solid_cells)`
   - `legend_shown` compares the note.

**Rendering** (through the lead):
1. `tools/flow_movie/live_case.cpp` keeps the model's parts for D1.
2. The film's caption carries the C_p note verbatim.
