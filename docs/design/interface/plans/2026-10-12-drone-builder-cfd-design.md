# Drone CFD analysis: Spade's component for Kat's builder

**Owner:** Interface (Spade's side). Kat's builder hosts it; Kat's lead owns the host side. **Status:** revised after Cameron's answers (`backlog.md`, cd298d6), for the lead's review. The host side waits for Kat's lead's reply. Nothing here is built.

**Rulings this rests on:**
- Cameron's answers to this spec's eight questions (below).
- The airflow demo's path, rotor models and surfaces. Cameron's words: "a mini drone cfd analysis system on top of the builder with cad design inputs and proper geometry for each part", "multiple modeling options", "real time analysis being cool and all, but some stuff will take processing time".
- "Meshes + STEP converter" (CORE-6): triangle meshes are the one imported geometry; STEP and IGES come in through an offline converter.
- PHY-10, the airflow demonstration plan.
- CORE-7 (model parts) and CORE-8 D5 (refuse, never downgrade).
- The editor spec's §13 look, which Cameron's look ruling applies to Kat's editor too.

**Inputs:**
- `core/2026-10-12-model-parts-design.md`
- `core/plans/2026-10-12-parts-meshes-plan.md`
- `physics/plans/2026-10-03-drone-builder-physics.md` §9 (DBP-*)
- `physics/plans/2026-10-05-airflow-design.md`
- `physics/plans/2026-10-12-airflow-drone-demonstration-plan.md`
- `interface/2026-10-12-airflow-demo-interface-inputs.md`
- Physics' blade definition proposal for P4 (2026-10-12, §7 here)
- Test/Docs' validation record format
- Rendering's flow views

## Decided by Cameron, 2026-10-12 (asked by the lead)

- **Q1, Kat hosts the builder.** Spade provides the analysis: cases, runs, results views, and the API Kat's builder calls.
- **Q2, import only.** Spade authors no parts. Geometry comes from CAD as meshes (STEP via the converter), through Kat's builder.
- **Q3, one blade definition per rotor:** chord, twist and sections, imported polars, a flat-plate default labelled "estimated", and the blade mesh lofted from it.
- **Q4, the drone-box premise.** Cameron's words: "The current drone conditions will be like the drone sandbox, with a drone in the center plane, and the same manual movement, just with actual arm/disarm, adding a floor, etc, but the sandbox drone view premise is optimal especially since demonstrating the drone in different orientations and thrust states mimics a real drone visual much better and rhe aerodynamics can look much cleaner." So:
  - the drone is held at the domain's centre;
  - the manual attitude and throttle controls of the drone sim box;
  - real arm and disarm;
  - an optional floor (ground effect);
  - wind through the box for forward-flight conditions;
  - no following box and no free flight, for now.
- **Q5.** Live runs, plus queued headless runs, one heavy run at a time.
- **Q6.** Both coefficient conventions, rotorcraft and propeller, each with its definition, defaulting by case.
- **Q7.** Linked side-by-side views on one colour scale, plus field differences on matching grids.
- **Q8.** A Spade case format (`.case.yaml`) and run folders, which Kat's builder reads and writes through Spade's API. They move into stage 7's hashed configuration later.

**Three honesty rules run through every section:**
1. Every number says where it came from: measured in this run, from the airframe compile, given, or estimated.
2. What the grid can't resolve is refused by name, with the spacing it needs. It is never quietly made coarser, porous or another rotor model.
3. A result is "target, not validated" until a validation record covers its configuration.

## 1. Goal and done-when

Kat's builder can put a drone it built through the air with Spade. It is done when:

1. **A case.** Kat's builder can make, edit, save and load a case through Spade's API, for a model it built. The case holds:
   - the drone held at the centre of the box, its attitude and throttle, and armed or not;
   - an optional floor, and wind;
   - the fluid, the grid and its nests;
   - a rotor model per rotor, and a blade definition where the model needs one;
   - what to record.

   What the grid can't resolve is refused by name, with the spacing it needs.
2. **Live runs.** A case runs live, flown by hand like the drone sim box, with real arm and disarm. The live inputs are recorded, so the same flight can be queued again at a higher fidelity.
3. **Queued runs.** A case queues as a headless run with progress and an honest cost estimate, one heavy run at a time. Kat's builder can close and reopen to watch it.
4. **Results** come through Spade's API and views:
   - C_p on the surfaces, slices, streamlines and vortex surfaces;
   - forces and moments with both coefficient conventions;
   - a validation report.
5. **Comparing.** Runs compare: settings and scalars, linked views on one colour scale, and field differences on matching grids.
6. **Spade's own uses.** The sandbox exercises all of this itself: the airflow demo scene (I1, I2) is the reference host and Spade's test bed.

## 2. Who does what

| | Kat's builder (the host) | Spade's analysis component (Interface) | Spade's engine |
|---|---|---|---|
| Geometry | Imports CAD meshes (Core's readers), places parts, sets each part's surface and `min_feature` | Reads the model; refuses what its grid can't resolve | Core: parts, meshes, exact distance, mass from geometry |
| The case | Its panels edit a case | The case type, its file (`.case.yaml`), resolving, refusals, estimates | Physics: the resolution rules and the solver's limits |
| Runs | Its buttons start, queue and cancel; it shows progress | The live run, the headless runner, run folders, calibration | Physics: the solver, the coupler (P7), the actuator models (P1, P4, P5) |
| Results | Its windows show the views | Reading runs, coefficients, the validation report, comparisons | Rendering: the flow views; Test/Docs: the record format |
| Reference host | | The sandbox's airflow scene and panels (I1, I2) | |

Spade's part is display-free C++ behind installed headers (SL2b holds for Kat as for the sandbox). Whether Kat's host also reuses Spade's reference ImGui panels or draws its own over the API is Kat's choice (A2 in §11).

## 3. The analysis API

The API is display-free and installed. Where it is installed is A1 in §11. These are the names, not yet their signatures.

**The case**
- `AnalysisCase`: the file's contents (§4).
- `load_case(path)` and `save_case(case, path)`: the `.case.yaml` form, written only through these.
- `resolve(case, model)` gives a `ResolvedCase`, or every refusal by name. Resolving fills every default and computes every derived value: cells, memory, the time step, the step count, the blade-derived values.

**The estimate**
- `estimate(resolved, calibration)`: a range with its source (§5).
- `calibrate(path)`: measures this machine's cost per million cells per step on a solver path.

**Live runs**
- `LiveRun::start(resolved)`.
- `step(dt, controls)`: attitude, throttle, armed.
- `state()`, `loads()`, `publish(volume)`, and `real_time_ratio()`, measured outside stepped code.
- `recorded_controls()`: the flight so far, as a control program.

**Queued runs**
- `queue(resolved, controls, runs_dir)` starts the headless runner, `spade_analysis_runner`, as its own process and returns a `RunHandle`.
- `progress(handle)`, `cancel(handle)`, and `list_runs(runs_dir)`.

**Results**
- `open_run(folder)` gives a `RunResults`: settings, histories, snapshots and the log.
- `coefficients(results, convention)`.
- `validation_report(results, records_dir)`.
- `compare(a, b)` gives a `Comparison`: settings that differ, scalars, histories, and the field difference where the grids match.

**Views**
- Rendering's installed views draw a `RunResults` or `LiveRun` snapshot: `FlowLayer`, `IsoSurface`, `trace_streamlines`, `slice_flow_volume`, `FlowSurfaceColouring`.
- C_p is computed by this API (§6), not by the views.

## 4. The case: the drone-box premise

A case is a scene of one drone in a box of air, as the drone sim box is.

**The model**
- Kat's model file, named and pinned by its `model_identity` hash, as a scene pins its world.
- A changed model shows as a conflict until the case is re-pinned.

**The drone**
- Held at the domain's centre. Its attitude (pitch, roll, yaw) and throttle are set by hand in a live run, or by a control program in a queued one.
- Armed or disarmed: disarmed rotors are stopped, and arming spins them up with their τ.
- The rotor speeds come from the throttle and attitude through the drone's own mixer, as in the sandbox. Each speed is shown measured.

**The control program**
- A queued run's inputs: either constant (one attitude, one throttle, armed), or keyframes, or a live run's recording.
- How much of this the first cut covers is A4.

**The floor and the wind**
- The floor is optional: the box's lower face as a wall at a height below the drone (ground effect).
- The wind blows through the box for forward-flight conditions. It needs an inflow face from Physics (§9).
- Under this premise, the demo's `--flight forward:<v>` maps to the wind and `--height` to the floor (Physics' plan).

**The fluid**
- Density and kinematic viscosity, from the ISA at an altitude or given, shown with their source.

**The domain**
- A fixed box centred on the drone. The default size comes from the drone's reach and is shown as "default".
- Faces: open, wall (the floor), or inflow (the wind).

**The grid**
- A base spacing, plus static 2:1 nests up to 3 levels (R1), placed around what must be resolved.
- Cells and memory (`grid_bytes`, `fluid_bytes`) update live. Anything above the 4 GiB cap is refused.

**The rotors**
- A model per rotor: disc, line or blade-resolved, each with its resolution rule beside it:
  - the disc: about 10 cells across R;
  - the line: R/dx ≥ 20;
  - blade-resolved: ≥ 32 cells across the chord and ≥ 2 across the thickness.
- The line and blade-resolved models need the rotor's blade definition (§7).

**Surfaces**
- Each part's, as Kat's builder set it on the part (wall, or porous with Darcy's 1/K and Forchheimer's C₂).
- The case can't change a surface. A wall feature under 2 cells is refused by name (P2), never made porous.

**Time**
- The step rule (h_f = j·dt, sub-steps for the line and blade models), the duration, the start-up to discard, and the averaging window.

**Outputs**
- How often to record histories, and field snapshots at times or every T.

**Validation**
- A case that matches a validation case (C8, the hover rotor) is compared with its reference.

**The summary line.** Always shown, for example: "disc · 128³ + 2 nests (5.3 M cells, 1.2 GiB) · dt 1 ms · 2.0 s · estimate 4-9 min CPU (calibrated 2026-10-12)".

## 5. Running

**Live**
- The drone is flown by hand: attitude and throttle, arm and disarm, as in the drone sim box.
- The run line shows the model, the grid and the measured real-time ratio.
- Changing a rotor model, the grid or the floor restarts the flow.
- The inputs are recorded. "Queue at higher fidelity" sends the same flight to the queue with another rotor model or grid. The live session becomes the offline run's script.

**Queued**
- A separate headless process writes its run folder, so the host can close and reopen.
- One heavy run runs at a time; the queue is serial.
- A run doesn't start below its memory need plus the box's margin; it waits and says why.
- Each run's state is one of: queued, calibrating, running, done, refused, faulted, cancelled. The host also sees simulated time against target, steps, elapsed time, time left, ms per step and memory.
- A cancelled run keeps what it wrote, labelled "partial".

**The estimate**
- **Before a run:** a range from the cells × the steps × this machine's measured cost per million cells per step on that path. A short benchmark measures the cost the first time a path is used; it is kept per machine with its date and commit.
- **Once running:** the measured rate replaces the range.
- **Without a calibration:** "no estimate yet: calibrating".

**The run folder:** `runs/<case>/<date>_<case hash>_<commit>/`. It holds:
- `run.yaml`: the resolved case, the commit, the machine, the measured cost;
- the control program;
- the progress file;
- histories (CSV, units in the header);
- snapshots;
- the log;
- the validation record block, for a validation case.

A finished run is read-only. On the CPU path a run re-runs to the same bits from its folder. The GPU path is banded (TD-14). A fault stops the run, names the fault, and keeps the last good snapshot.

## 6. Results

**Surface C_p**
- C_p = (p − p_ref)/(½ ρ V_ref²). V_ref is the tip speed in hover and the wind in forward flight, following the default convention (Q6).
- The cool_warm map is centred on 0, and the scale names V_ref and ρ.

**Views**
- Slices: speed, u, v, w, pressure, C_p, vorticity, Q.
- Streamlines from rakes and rings (RK4). The legend counts how lines ended.
- Tracers.
- Iso-surfaces of |ω| and Q, up to 4. λ₂ when Rendering adds it.

**Forces and coefficients**
- For the whole drone, each part and each rotor: force and moment about the centre of mass, thrust, torque and power.
- Both conventions (Q6), each with its definition, defaulting by case:
  - rotorcraft in hover: C_T = T/(ρA(ΩR)²), C_Q, C_P, and the figure of merit;
  - propeller under wind: C_T = T/(ρn²D⁴), C_P = P/(ρn³D⁵), and J.
- Averages state their window and show the standard deviation over it.
- Histories and scalars export to CSV.

**Time.** A scrubber over the snapshots.

**The validation report**
- The case, the resolved settings and the commit.
- Every number with its unit and provenance. A rotor on flat-plate polars says "estimated polars".
- The validation records covering the configuration (case, verdict, commit), or "target, not validated".
- A validation-case run appends its record in Test/Docs' format, through the same writer as `--airflow-record` (I2).

## 7. Blade definitions (Q3; Physics' P4 and P5 read the same data)

One per rotor, in the case. A blade design can then vary between cases for the same model.

- **BladeDefinition** (Physics' proposal):
  - `blade_count`;
  - `radius` R (m), which must equal the model rotor's radius or be refused by name;
  - `hub_radius` (m);
  - `collective` (rad);
  - `stations`, ordered by r/R from hub to tip, each {`r_over_radius`, `chord` (m), `twist` (rad about the pitch axis, nose-up positive), `airfoil`}.
- **Airfoil:** {`id`, `source` (a file and citation, or "estimated"), `polars`}. Each polar is {`reynolds`, `alpha` (rad, increasing), `cl`, `cd`}. P4 interpolates linearly in α, then in Re.
- **The file form** keeps angles in degrees with a `_deg` suffix (`twist_deg`, `collective_deg`, `alpha_deg`), as XFOIL and wind-tunnel tables give them. The API converts once, to radians.
- **No polar for an airfoil** means the flat-plate default: C_l = 2 sin α cos α, C_d = 0.02 + 2 sin² α. It is labelled "estimated" on the run line, in the HUD and in the records.
- **Refused by name:**
  - stations not increasing in r/R;
  - a chord ≤ 0;
  - a polar whose α isn't increasing;
  - R/dx < 20 for the line (D5).

  How a polar that doesn't reach ±90° is treated is Physics' call (A3).
- **Import:** XFOIL polar files and CSV tables.
- **Lofting:** the blade mesh for blade-resolved runs is lofted from the same stations (P5). The analysis attaches it to the rotor for that run only, and Kat's model is unchanged.
- **The disc** keeps the compile's fitted `k_T` and `k_Q`, or takes blade-element momentum values from the definition, labelled as such.

## 8. Comparing runs

- **Settings first.** The settings that differ are shown before the results that differ.
- **Scalars and histories:** differences and percentages; histories overlaid.
- **Side by side:** linked cameras and one colour scale, fixed to the union of both ranges.
- **Field differences** (A − B) only on exactly matching grids. Otherwise the comparison says why not.

## 9. What it needs

| Need | From | Status, 2026-10-12 |
|---|---|---|
| Parts with meshes: `PartDesc::mesh` (B4), mass (B3), sidecar (B5), readers (B6-B8), the STEP converter | Core | B1 and B2 merged; the rest unbuilt |
| Core's model YAML, which Kat's model file uses | Core | Unbuilt |
| Walls and porous parts (P2); wall pressure and loads (P3) | Physics | Unbuilt |
| The coupler (P7); swirl (P1); the line (P4) and blade-resolved (P5) | Physics | Unbuilt. The blade definition's fields are agreed in §7 |
| An inflow face carrying the wind; the floor as a wall face at a height | Physics | Needed for Q4's wind and floor |
| The drone held at the centre with its attitude driven, as in the drone sim box | Physics and Interface | The sandbox's drone box does this today for the analytic air |
| Flow views; λ₂ | Rendering | Views merged; λ₂ a follow-up |
| The record format and reader | Test/Docs | On `test-docs/validation-tolerances` |
| The host: panels, the import UI, its own look | Kat | Waits on Kat's lead's reply |

## 10. Spade's own uses

- **The airflow demo scene (I1, I2)** takes the drone-box premise: the drone centred, manual attitude and throttle, arm and disarm, the floor, and the wind as forward flight. It is the API's first client and reference host. Its panel edits a case, which is the durable form of its session options.
- **The scene editor is untouched (D2).** The transform gizmo (in progress) serves the scene editor; it is not part authoring.

## 11. Open questions (for the lead, Kat's lead and Physics)

**Ruled by the lead, 2026-10-12 (INT-6):**
- **A1:** A, the new installed target `spade::analysis`, owned by Interface. Core reviews its CMake, install rules and layering, and it gets a `consumers.md` line when it lands.
- **A2:** A, the display-free API only, with the sandbox panels as the reference host.
- **A3:** Physics' call.
- **A4:** A, constant inputs plus a live run's recording.

**A1. Where is the analysis API installed?**
- **A (recommended).** A new installed target, `spade::analysis` (`engine/analysis/`), owned by Interface, on Physics' and Rendering's installed API.
- **B.** Inside Physics' airflow headers.
- **C.** Sandbox-only headers that Kat copies. This breaks SL2b's one installed surface.

**A2. What does Kat's host take from Spade?**
- **A (recommended).** The display-free API only, with Spade's sandbox panels as the reference.
- **B.** The API plus Spade's ImGui panels, as an installed UI library.
- **C.** The API plus a separate analysis window Spade owns, launched by Kat.

**A3 (Physics). A polar that doesn't reach ±90°.** Real data (XFOIL, wind tunnels) usually stops near ±20°.
- **A (recommended).** Extend it to ±90° by Viterna-Corrigan, labelled "extrapolated" on the run line and in the records.
- **B.** Refuse it by name; the user supplies full-range tables.
- **C.** Fall back to flat plate beyond the data, labelled "estimated".

**A4. The control program's first cut.**
- **A (recommended).** Constant inputs plus a live run's recording.
- **B.** Constant inputs only.
- **C.** A, plus hand-edited keyframes.

## 12. Requirements

Prefix `DBC-`.

**API and host**
- **DBC-001** Spade's analysis MUST be display-free and reachable through installed headers only (SL2b).
- **DBC-002** Spade MUST NOT author parts (Q2). The analysis MUST read the host's model as it is.
- **DBC-003** A refusal MUST name the field, the part or rotor, and the value it needs. The host shows it verbatim (INT-2).
- **DBC-004** The case file MUST be written and read only through `save_case` and `load_case`.

**Cases**
- **DBC-010** A case MUST pin its model's identity. A changed model MUST show as a conflict until the case is re-pinned.
- **DBC-011** The drone MUST be held at the domain's centre (Q4). Attitude, throttle and arming MUST come from the controls or the control program.
- **DBC-012** Disarmed rotors MUST be stopped. Arming MUST spin them up with their τ.
- **DBC-013** What the grid can't resolve MUST be refused by name with the spacing it needs:
  - a wall feature ≥ 2 cells;
  - the line, R/dx ≥ 20;
  - blade-resolved, ≥ 32 cells across the chord and ≥ 2 across the thickness.
- **DBC-014** The analysis MUST NOT make a wall porous, change a rotor model, or coarsen a requirement to fit a grid (P2; CORE-8 D5).
- **DBC-015** A grid above the solver's cap MUST be refused before running.
- **DBC-016** The resolved case MUST be exactly what the solver receives, and its hash MUST name the run.
- **DBC-017** A blade definition's radius MUST equal its rotor's. A rotor on flat-plate polars MUST be labelled "estimated polars" everywhere its numbers appear.

**Running**
- **DBC-020** A live run MUST show its model, its grid and its measured real-time ratio, measured outside stepped code.
- **DBC-021** A live run's inputs MUST be recorded as a control program that a queued run can replay.
- **DBC-022** A queued run MUST run in its own process, keep running when the host closes, and be readable by a reopened host.
- **DBC-023** One heavy run MUST run at a time. A run MUST NOT start below its memory need plus the box's margin, and MUST say why it waits.
- **DBC-024** The estimate MUST be a range with its source, from this machine's calibration. The measured rate MUST replace it once there is one.
- **DBC-025** A run MUST record its commit, resolved case, control program, machine and measured cost. On the CPU path it MUST re-run to the same bits.
- **DBC-026** A fault MUST stop the run, name the fault, and keep the last good snapshot. A cancelled run MUST say "partial".

**Results**
- **DBC-030** C_p MUST show its reference velocity and density.
- **DBC-031** Both coefficient conventions MUST be available, each with its definition shown, defaulting by case (Q6).
- **DBC-032** Every number MUST carry its unit and provenance. An average MUST state its window and show its standard deviation.
- **DBC-033** The validation report MUST list the records covering the configuration, or say "target, not validated".
- **DBC-034** A validation-case run MUST append its record in Test/Docs' format, through the `--airflow-record` writer.
- **DBC-035** Histories and scalars MUST export to CSV with units in the header.

**Comparing**
- **DBC-040** A comparison MUST show the settings that differ before the results.
- **DBC-041** Field differences MUST be offered only on exactly matching grids. Otherwise the comparison MUST say why.
- **DBC-042** Side-by-side views MUST share one colour scale.

**Build**
- **DBC-050** Spade's reference panels MUST follow §13 and pass EDT-021's scan.
- **DBC-051** A new third-party dependency MUST NOT come in without the lead's approval.

## 13. Testing

- **Unit tests (spade_tests):**
  - the case file round-trips byte for byte;
  - resolving and its hash;
  - every refusal, named;
  - the estimate's arithmetic;
  - the blade definition's checks and degree conversion;
  - coefficients in both conventions against hand-computed values;
  - reading a run folder;
  - the comparison and its settings diff.
- **No new physics goldens.** The analysis owns no physics numbers. Its tests check that settings reach the solver unchanged, refusals show verbatim, and provenance and units stay on every number.
- **A consumer smoke** builds a tiny host against the installed headers. It loads a case, resolves it, runs a few live steps, queues a short run and reads its folder. This is Kat's path, proven on Spade's side.
- **The live smoke tour's airflow section** flies the drone-box case (arm, throttle, attitude, floor) and shows each view. Red by injection, then green.
- **A C8 run** at the record's configuration appends a record that Test/Docs' reader accepts.

## 14. Order of work

The stages are W0-W6, so they don't collide with Core's B1-B9.

1. **W0.** The case: its type, its file, resolving and the refusals. Display-free; it can start now.
2. **W1.** The live drone-box run: I1's harness on the API, with the airflow scene as the host. Needs P2, P3, P7, and the inflow and floor faces.
3. **W2.** The queue, the headless runner, run folders, calibration and the estimate.
4. **W3.** Results and the validation report (I2).
5. **W4.** Comparing.
6. **W5.** Blade definitions and the line (P4).
7. **W6.** Blade-resolved, offline (P5).

Kat's host integrates from W1 on, as Kat's lead plans it.
