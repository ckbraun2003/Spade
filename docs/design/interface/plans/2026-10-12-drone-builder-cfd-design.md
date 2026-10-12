# The drone builder and its CFD analysis

**Owner:** Interface. **Status:** draft for Cameron's approval (via the lead), 2026-10-12. Nothing here is built. **Builds:** a workspace of Spade's sandbox, on the installed API only (SL2b).

**Rulings this rests on:**
- `backlog.md`, the airflow demo's path, rotor models and surfaces. Cameron's words: "The drone builder is going to be a mini drone cfd analysis system on top of the builder with cad design inputs and proper geometry for each part. The analysis is expected to have multiple modeling options, with real time analysis being cool and all, but some stuff will take processing time and that is expected for larger sims." The same ruling makes the drone a solid wall, with a porous option.
- `backlog.md`, "Meshes + STEP converter": parts are built natively first; triangle meshes (glTF, STL, OBJ, 3MF) are the one imported geometry; STEP and IGES come in through an offline converter.
- The editor's D2, Cameron's words: "You cannot build new models in spade editor. Think unity".
- PHY-10, the airflow demonstration plan, approved as written.
- CORE-7 (model parts), CORE-6 (meshes) and CORE-8 D5 (a part the grid can't resolve is refused, never downgraded).
- The editor spec's §13 (the look).

**Inputs:**
- `core/2026-10-12-model-parts-design.md`
- `core/plans/2026-10-12-parts-meshes-plan.md`
- `physics/plans/2026-10-03-drone-builder-physics.md` §9 (the airframe compile, DBP-*)
- `physics/plans/2026-10-05-airflow-design.md` (the solver, its grid, nests and validation cases C0-C8)
- `physics/plans/2026-10-12-airflow-drone-demonstration-plan.md` (D1-D4, P1-P11)
- `interface/2026-10-12-airflow-demo-interface-inputs.md` (I1, I2)
- Test/Docs' validation record format (`test-docs/01-verification.md`, "Validation records"; on branch `test-docs/validation-tolerances` today)
- Rendering's flow views (`render/flow_render.hpp`, `flow_lines.hpp`, `flow_field.hpp`, `transfer_function.hpp`)

**One conflict to settle first (Q1 in §11).**
- The joint drone-builder spec (`plans/2026-10-03-drone-builder-engine-design.md`, line 4) says Kat owns "the parts, the catalog, the builder and the compiler".
- Cameron's later ruling gives Interface "the builder as a drone CFD analysis system", and Core's parts design leaves the native authoring UI to Interface.
- This draft assumes Q1's recommended answer, Spade's own builder workspace, and says in §11 what the other answers would change.

**Three honesty rules run through every section:**
1. Every number says where it came from: measured in this run, from the airframe compile, given by the user, or estimated.
2. What the grid can't resolve is refused by name, with the spacing it needs. It is never quietly made coarser, porous or a different rotor model.
3. A result is "target, not validated" until a validation record covers its configuration.

## 1. Goal and done-when

The builder is where a drone is made from real geometry and put through the air. It is done when:

1. **Parts.** A drone can be authored from parts, each mounted on the body or on a rotor:
   - native solids built from the engine's primitives;
   - meshes imported from CAD (glTF, STL, OBJ, 3MF; STEP through the converter).
2. **Surfaces.** Each part has a surface: a solid wall, or porous with its two coefficients.
3. **Case.** An analysis case can be set up:
   - the flight condition and the fluid;
   - the domain, the grid and its nests;
   - the rotor model for each rotor (disc, line or blade-resolved);
   - what to record.

   What the grid can't resolve is refused by name, with the spacing it needs.
4. **Running.** A case runs live when it can (the disc on the GPU). Otherwise it queues as a long run, with progress and an honest cost estimate that a measurement replaces as soon as one exists.
5. **Results** show:
   - C_p on the surfaces, slices, streamlines and vortex surfaces;
   - forces and moments with their coefficients over time;
   - a validation report saying which numbers the engine's validation records cover and which are targets, not validated.
6. **Comparing.** Runs compare: their settings and numbers side by side, their views with one colour scale, and their fields when their grids match.

## 2. Scope

| First cut (stages W0-W6, §14) | Specified, built later |
|---|---|
| Native parts: primitives, booleans, smooth blends, mirror and circular repeat | Sketch, extrude and revolve, if Q2 asks for them (new Core primitives) |
| Surfaces: wall or porous, per part | Surface presets backed by measured data |
| The airframe compile's readout, every value with its provenance | Mass from imported geometry (Core B3/B4) |
| Cases: hover, vertical climb, and forward flight (Q4), with the disc model | Actuator line (needs Physics' line source); blade-resolved, offline (D3) |
| Live disc runs; queued CPU runs with run folders, progress, estimate and cancel | GPU queued runs (plan B); checkpoints (Q5 C); other machines (Q5 D) |
| Results: C_p, slices, streamlines, \|ω\| and Q iso-surfaces, forces and coefficients, the validation report | λ₂ (Rendering's follow-up); probes along points and lines |
| Comparing: settings, scalars, histories, linked side-by-side views; field differences on matching grids | Interpolated differences; grid convergence studies (Q7 D) |
| Import UI, as soon as Core's readers land (B6-B8) | The STEP converter's UI (it is an offline tool) |
| | Two-way free flight (D4, after stage 7) |

## 3. Where it lives, and what it saves

**The workspace.** `spade_sandbox --builder [model]` is a sibling of `--editor`. It uses the same layout rules and the same theme, and it is not a mode of the scene editor: D2 and EDT-013 still hold there. The editor places what the builder saves, through its library (editor Task 13).

**Layout (§13's panels):**
- The toolbar: Run, Queue, Stop, Undo, Redo, Save, File.
- The hierarchy has three groups:
  - the model: body, rotors, parts;
  - the cases;
  - each case's runs.
- The inspector, for whatever is selected.
- Below, two tabs:
  - the library: primitives, imported meshes, blade templates;
  - Runs: the queue.
- The 3D view between the panels.

**What it saves.** The format and its owner are Q8; this is the shape.
- **The model** is a `ModelType` with its parts, in Core's model YAML (the format editor Task 13 waits on). The builder never keeps a model format of its own.
- **A case** (`<name>.case.yaml`) names its model by file and pins it by `model_identity` hash, the way a scene pins its world. If the model changes, the case shows the conflict until it is re-pinned on save.
- **A run** is a folder, `runs/<case>/<date>_<case hash>_<commit>/`. It holds:
  - `run.yaml`: the resolved settings, the commit, the machine, the measured cost;
  - the progress file;
  - the force and moment histories (CSV, units in the header);
  - field snapshots (flow volumes at the requested times);
  - the log;
  - the validation record block, for a validation case.

  A finished run is read-only. Comparisons and reports read runs, never live state.

## 4. Authoring parts

### 4.1 Native parts

A part is Core's `PartDesc`:
- name, and role (a free label the engine never reads);
- attach: `body`, or `rotor` with a rotor index;
- the mount pose. For a rotor part it is relative to the hub, with +Y as thrust and ψ = 0 at the authored pose;
- the shape (`SdfProgram`), the surface, and `min_feature`.

How the builder authors them:
- **Shapes** are the world's own SDF primitives (sphere, box, cylinder, capsule, torus and the rest) with union, subtract, intersect and smooth blends. That is the same `SdfProgram` the world composes, so the engine gains nothing new. Q2 asks how far native authoring goes.
- **Editing.** The inspector edits a primitive's parameters. The transform gizmo (Interface's next task, shared with the editor) moves and rotates it, one undo per drag.
- **Mirror and circular repeat** (a quadrotor's arms) write ordinary parts with derived unique names. They are an authoring convenience: the model has no instancing.
- **`min_feature`** defaults to the shape's thinnest dimension and is shown as "from the shape" until edited. It is the smallest feature the grid must resolve (CORE-8 D5).
- **Refusals** from `register_model()`'s issues show verbatim next to the field they name, by part index and name (INT-2). Examples: an empty or duplicate name, an unbounded shape, a coefficient on a wall, `min_feature` ≤ 0.

### 4.2 Imported parts

- **Formats.** File → Import geometry reads glTF/.glb, binary STL, OBJ (triangles) and 3MF. STEP and IGES go through Core's offline converter, which writes one of these.
- **Units are never guessed.** glTF is in metres and 3MF declares its unit. STL and OBJ ask for the unit before importing (`MeshUnit`).
- **The builder repairs nothing:** no welding and no hole filling. Fixing a mesh is the CAD tool's job. Every `make_mesh` refusal shows with its count and up to 8 offending elements (open, non-manifold or misoriented edges, self-intersections, wrong nesting, coincident vertices and so on). Where it can, the view highlights the offending triangles, edges or vertices.
- **Size.** The triangle count and the time the exact-distance structure took to build are shown.
- **Dependencies:** imported parts need Core's `PartDesc::mesh` (B4), the `.spmesh` sidecar (B5) and the readers (B6-B8).

### 4.3 Rotors and blades

A rotor already has a hub, an orientation, a spin, a radius, `k_T`, `k_Q` and τ. The line and blade-resolved models also need its blades: a blade definition with the blade count, radial stations (chord, twist and airfoil at each), and each airfoil's polars (C_l and C_d against α, at a Reynolds number).

One blade definition feeds all three models:
- **the disc:** `k_T` and `k_Q` stay the compile's fitted values, or come from blade-element momentum on the definition, labelled as such;
- **the line:** its sections;
- **blade-resolved:** a blade mesh lofted from the same definition, attached as a rotor part.

An imported blade mesh can stand in for the lofted one. Q3 asks where the polars come from.

### 4.4 Mass properties

- The airframe compile (`compile_airframe`) gives mass, inertia, centre of mass and principal axes.
- Each value carries the compile's own `Provenance`: from parts, fitted, estimated or given. The builder shows that tag beside the value.
- Until Core's mass from geometry lands (B3/B4), a part's mass is given by the user.

## 5. Surfaces

- **Each part** is a wall (the default) or porous. Porous takes Darcy's 1/K (1/m²) and Forchheimer's C₂ (1/m).
- **Transparent warning.** A porous part with both coefficients 0 is transparent, and the inspector says so.
- **The Surfaces overlay** colours parts by kind: walls solid, porous parts translucent.
- **No presets until they have sources.** A preset such as "foam" or "mesh guard" without measured data would be a fabricated number.
- **A wall never quietly becomes porous.** If the grid can't resolve a wall feature (fewer than 2 cells), the case is refused by name with the spacing it needs (P2). Choosing porous instead is the user's explicit act.

## 6. The analysis case

The case inspector has these sections, in order.

**Flight**
- Hover; vertical climb or descent at a speed; or forward flight at a speed, angle of attack and sideslip (Q4).
- Forward flight uses the airflow spec's following box: the drone moves through still air.
- Rotor speeds: fixed ω per rotor, or "trim to weight". Trim starts from the compile's steady-state solver (DBP-50-52) and is labelled as trimmed.
- An optional ground plane at a height, for ground effect.

**Fluid**
- Density and kinematic viscosity, from the ISA at an altitude or given.
- Shown with their source.

**Domain**
- Box size and placement, fixed or following.
- Each face open, wall or periodic.
- The default size comes from the drone's reach and is shown as "default" until edited.

**Grid**
- A base spacing, plus static 2:1 nests up to 3 levels (R1).
- Nests are placed automatically around what must be resolved, and can be edited.
- The cell count and memory (the solver's own `grid_bytes` and `fluid_bytes`) update live.
- Anything over the solver's 4 GiB cap is refused before running.

**Rotors**
- A model per rotor: disc, line or blade-resolved.
- Each shows its resolution rule next to it:
  - disc: about 10 cells across R;
  - line: R/dx ≥ 20;
  - blade-resolved: ≥ 32 cells across the chord and ≥ 2 across the thickness.
- What fails is refused by name with the spacing it needs.
- Changing a model shows the new cost at once.

**Time**
- The step rule: h_f = j·dt, and sub-steps for the line and blade models.
- The simulated duration.
- The start-up to discard, and the averaging window.

**Outputs**
- How often to record force and moment histories.
- Field snapshots, at given times or every T, with the quantities to keep.

**Validation**
- When the case matches a validation case (C8, the hover rotor against Caradonna-Tung), the report compares against its reference.

**The summary line.** One line is always visible, for example: "disc · 128³ + 2 nests (5.3 M cells, 1.2 GiB) · dt 1 ms · 2.0 s simulated · estimate 4-9 min CPU (calibrated 2026-10-12)".

**What the solver receives.** A case is resolved before it runs: every default is filled and every derived value computed. The resolved case is exactly what the solver receives. Its hash names the run.

## 7. Running

**Live runs**
- When real time is possible (the disc on the GPU, D2), Run runs in the window, as the editor's run does.
- It shows its model, its grid, and its real-time ratio, measured outside stepped code.
- Changing the model, a surface or the grid restarts the flow (the airflow inputs, I1).

**Queued runs**
- Queue adds the case to the run queue, and a queued run runs headless (Q5).
- The Runs tab shows each run's:
  - state: queued, calibrating, running, done, refused, faulted or cancelled;
  - simulated time against target, and steps;
  - elapsed time, time left and measured ms per step;
  - memory.
- Cancel keeps what was written, and the results say "partial".

**Memory.** Heavy runs go one at a time. A run doesn't start while free memory (or VRAM, for the GPU path) is below what its estimate needs plus the box's margin; it waits and says why.

**The estimate.** Decided in this spec; say if you disagree.
- **Before a run:** a range from the cell count × the step count × this machine's measured cost per million cells per step on that solver path.
- **Calibration:** the first time a path is used, a short benchmark measures that cost. It is kept per machine, with its date and commit. It never comes from published numbers or another machine.
- **Once running:** the first steps' measured rate replaces the range, and the time left updates as it goes.
- **Wording:**
  - before: "estimate 4-9 min (calibrated 2026-10-12, this machine)";
  - running: "about 6 min left (measured)";
  - no calibration yet: "no estimate yet: calibrating".

**Reproducible.** A run records its commit, its resolved case and its machine. On the CPU reference path it re-runs to the same bits; the GPU path is banded (TD-14).

**Faults.** A fault (a non-finite value, an ill-posed domain) stops the run with the fault named and keeps the last good snapshot.

## 8. Results

The views are Rendering's; the builder arranges and labels them.

**Surface C_p**
- On the parts, from the wall pressure (the publish step extrapolates the first solid layer so C_p reads the wall).
- C_p = (p − p_ref)/(½ ρ V_ref²), with V_ref per Q6.
- The cool_warm map is centred on 0, and the scale names V_ref and ρ.

**Slices**
- Axis-aligned planes, placed anywhere.
- Speed, u, v, w, pressure, C_p, vorticity or Q.

**Streamlines**
- Rakes and rings, traced with RK4.
- The legend counts how lines ended: left the box, stagnation, no data, or step budget.
- Tracers for unsteady flow.

**Vortices**
- Up to 4 iso-surfaces of |ω| and Q.
- λ₂ when Rendering adds it.

**Forces and coefficients**
- For the whole drone, each part and each rotor:
  - force, and moment about the centre of mass;
  - thrust, torque and power;
  - C_T, C_Q and C_P, and the figure of merit in hover. Conventions per Q6, with each definition shown beside the number.
- Time histories show the averaging window marked, with the mean and the standard deviation over it.
- Histories and the scalar table export to CSV, with units in the header.
- Plots use ImGui's own plotting first. ImPlot only comes in with the lead's approval of the dependency.

**Time.** A scrubber over the run's snapshots.

**The validation report.** One page per run:
- the case, its resolved settings and the commit;
- every number with its unit and provenance;
- a Validation section listing the engine's validation records that cover this configuration (case, verdict, commit), or "target, not validated" where none does.

A run that is itself a validation case appends a record in Test/Docs' format, through the same writer as `--airflow-record` (I2). Records are append-only.

## 9. Comparing runs

- **Settings first.** Pick two runs (or more, for the table). The builder shows what differs in their settings (grid, model, surface, ω and so on) before what differs in their results.
- **The scalar table:** thrust, torque, power, figure of merit, the coefficients and mean forces, with differences and percentages. Time histories are overlaid.
- **Side by side:** two views with linked cameras and one colour scale, fixed to the union of both ranges.
- **Field differences** (A − B) only when the grids match exactly: the same domain, spacing and nests. Otherwise the builder says why not (Q7).

## 10. What it needs from other realms

| Need | Realm | Status, 2026-10-12 |
|---|---|---|
| `ModelType::parts` (CORE-7) | Core | A1 merged. The posing rule (A2), scene v2 (A3) and the compile emitting parts (A4) are unbuilt |
| Core's model YAML (save and load a model) | Core | Unbuilt. Editor Task 13 waits on it too |
| Mesh parts: mass (B3), `PartDesc::mesh` (B4), sidecar (B5), readers (B6-B8), the STEP converter | Core | B1 and B2 (exact distance) merged; the rest unbuilt |
| Immersed walls and porous parts (P2); wall pressure and loads (P3) | Physics | Unbuilt |
| The coupler (P7): a rotor list with a model per rotor | Physics | Unbuilt. I1 waits on it |
| Swirl (P1); the actuator line source and its section format; blade-resolved rotors | Physics | Unbuilt. The builder needs the section format agreed before §4.3's blade definition is fixed |
| Nests (R1) and the following box | Physics | Specified in the airflow spec |
| The GPU twin (plan B) | Physics | In progress |
| Flow views; λ₂; the Full/Half/Auto march (RND-7) | Rendering | Views and the live march merged; λ₂ a follow-up |
| The validation record format and its reader | Test/Docs | On `test-docs/validation-tolerances` |
| Per-machine cost calibration | Interface | This spec, using Physics' step functions |

## 11. Decisions for Cameron

Each question has 3-4 options. The recommended option comes first.

**Q1. Where does the builder live?**
- **A (recommended).** Spade's own builder workspace (`--builder`), beside the editor and with its look. The editor places what it saves. Kat's builder stays Kat's product and uses the same engine pieces: parts, the compile, the analysis harness.
- **B.** Kat's builder is the only builder. Spade ships the analysis (cases, runs, results) as pieces Kat hosts, and this spec shrinks to §6-§9.
- **C.** A builder mode inside Spade's editor. This reverses D2's "You cannot build new models in spade editor".
- **D.** An analysis workbench only. Spade opens models made elsewhere (Kat, CAD imports), sets surfaces and runs analyses, but authors no parts; §4.1 drops out.

**Q2. How far does native part authoring go?**
- **A (recommended).** Primitives with booleans and smooth blends, plus mirror and circular repeat. Real CAD comes in by import.
- **B.** A, plus sketch, extrude and revolve (a 2D profile tool). This needs new Core SDF primitives and makes a larger first cut.
- **C.** Import only: no native shapes; all geometry comes from CAD.
- **D.** A parametric CAD history with features and constraints: a CAD kernel of Spade's own. Not recommended.

**Q3. Where do blade sections and airfoil data come from (line and blade-resolved)?**
- **A (recommended).** One blade definition per rotor (count; chord, twist and airfoil per station). Polars are imported as tables (XFOIL or measured), with a flat-plate default labelled "estimated". The builder lofts the blade mesh from the same definition.
- **B.** Imported polars only. A rotor without them can't use the line model.
- **C.** The blade mesh is imported for blade-resolved, and the line's sections are entered separately, with no lofting.
- **D.** Polars computed by running 2D sections in Spade itself. Later, once 2D cases are validated.

**Q4. Which flight conditions does the first cut cover?**
- **A.** Hover and vertical climb or descent only.
- **B (recommended).** A, plus steady forward flight (speed, angle of attack, sideslip) via the following box, plus an optional ground plane for ground effect.
- **C.** B, plus prescribed manoeuvres: a pose and ω history, or a recorded flight replayed.
- **D.** Free flight, two-way coupled (D4). Needs stage 7.

**Q5. How do long runs run?**
- **A.** Inside the builder's window only. Closing it stops them.
- **B (recommended).** Live runs in the window. Queued runs run as a separate headless process writing a run folder, so the builder can close and reopen to watch. One heavy run at a time.
- **C.** B, plus checkpoints, so a stopped run resumes where it left off.
- **D.** B, plus sending runs to another machine (a server or a cloud GPU).

**Q6. Which coefficient conventions?**
- **A.** Rotorcraft: C_T = T/(ρA(ΩR)²), with C_Q, C_P and the figure of merit the same way. C_p by tip speed in hover and freestream in flight. Matches Caradonna-Tung.
- **B.** Propeller: C_T = T/(ρn²D⁴), C_P = P/(ρn³D⁵), and the advance ratio J. Matches the UIUC data.
- **C (recommended).** Both, each with its definition shown. The default follows the case: rotorcraft in hover, propeller in forward flight.
- **D.** Reference values set per case by the user.

**Q7. How do runs compare?**
- **A.** Settings, scalars and overlaid histories only.
- **B (recommended).** A, plus linked side-by-side views with one colour scale, and field differences when the grids match exactly.
- **C.** B, plus differences across different grids, interpolated and labelled as such.
- **D.** B, plus grid convergence studies: three runs at halving spacing, with Richardson extrapolation and the GCI.

**Q8. Where are cases and runs kept?**
- **A (recommended).** Sandbox files now: `.case.yaml` beside the model, and run folders. When stage 7's world v3 lands, a case moves into the hashed module configuration, with a converter.
- **B.** An engine format from the start, hashed into the identity. The analysis waits for stage 7.
- **C.** One project file bundling the model, its cases and an index of runs.
- **D.** Cases inside the scene file, as an analysis section per scene.

**Decided in this spec; say if you disagree:**
- the cost estimate's method (§7);
- the builder repairs no meshes (§4.2);
- mirror and repeat write ordinary parts (§4.1);
- no surface presets without sources (§5);
- one heavy run at a time (§7);
- no new dependency without the lead's approval (DBC-073).

## 12. Requirements

The prefix is `DBC-` (drone builder CFD), beside DBP and DBE.

**Workspace and documents**
- **DBC-001** The builder MUST be its own workspace, not a mode of the scene editor (D2; Q1).
- **DBC-002** The builder MUST author models only through the engine's `ModelType` and `register_model()`. It MUST NOT keep a model format of its own.
- **DBC-003** A refusal from the engine MUST show verbatim beside the field it names, and the edit MUST NOT apply (INT-2).
- **DBC-004** Every edit MUST be one undo step. One gizmo drag MUST be one step.

**Parts**
- **DBC-010** A native part's shape MUST be an `SdfProgram` of the engine's own primitives and operators.
- **DBC-011** Mirror and repeat MUST write ordinary parts with derived unique names.
- **DBC-012** `min_feature` MUST default from the shape and show its provenance.
- **DBC-013** Import MUST NOT guess a unit: STL and OBJ MUST ask; glTF and 3MF use their own.
- **DBC-014** Import MUST NOT repair a mesh. A refused mesh MUST show the engine's counts and elements, highlighted in the view where possible.
- **DBC-015** Mass properties MUST show their provenance: from parts, fitted, estimated or given.

**Surfaces**
- **DBC-020** A part MUST be a wall unless the user chooses porous. A porous part with both coefficients 0 MUST be flagged transparent.
- **DBC-021** The builder MUST NOT make a wall porous, change a rotor model, or coarsen a requirement to make a case fit its grid (P2; CORE-8 D5).

**Cases**
- **DBC-030** A case MUST pin its model's identity. A changed model MUST show as a conflict until the case is re-pinned.
- **DBC-031** The case MUST show its cells, memory, time step, steps and estimate live while it is edited.
- **DBC-032** What the grid can't resolve MUST be refused by name with the spacing it needs:
  - a wall feature ≥ 2 cells;
  - for the line, R/dx ≥ 20;
  - for blade-resolved, ≥ 32 cells across the chord and ≥ 2 across the thickness.
- **DBC-033** A grid above the solver's cap MUST be refused before running.
- **DBC-034** The resolved case MUST be exactly what the solver receives, and its hash MUST name the run.

**Running**
- **DBC-040** A live run MUST show its model, its grid and its measured real-time ratio, measured outside stepped code.
- **DBC-041** A queued run MUST keep running when the builder closes, and a reopened builder MUST read its progress (Q5).
- **DBC-042** The estimate MUST be a range with its source, from this machine's calibration, and MUST be replaced by the measured rate once there is one. Without a calibration it MUST say so.
- **DBC-043** A run MUST NOT start below its memory need plus the box's margin. It MUST wait and say why.
- **DBC-044** A run MUST record its commit, resolved case, machine and measured cost. On the CPU path it MUST re-run to the same bits.
- **DBC-045** A fault MUST stop the run, name the fault, and keep the last good snapshot. A cancelled run's results MUST say "partial".

**Results**
- **DBC-050** C_p MUST show its reference velocity and density with its scale.
- **DBC-051** Every coefficient MUST show its definition.
- **DBC-052** Every number MUST carry its unit and provenance.
- **DBC-053** An average MUST state its window and show the standard deviation over it.
- **DBC-054** The validation report MUST list the records covering the configuration (case, verdict, commit), or say "target, not validated".
- **DBC-055** A validation-case run MUST append its record in Test/Docs' format, through the same writer as `--airflow-record`.
- **DBC-056** Histories and the scalar table MUST export to CSV, with units in the header.

**Comparing**
- **DBC-060** A comparison MUST show the settings that differ, not only the results.
- **DBC-061** Field differences MUST be offered only between exactly matching grids. Otherwise the builder MUST say why.
- **DBC-062** Side-by-side views MUST share one colour scale.

**Build and look**
- **DBC-070** The builder MUST follow §13 and pass EDT-021's source scan.
- **DBC-071** The builder's decisions MUST live in display-free headers, as in the editor's workspace. The window only reads and queues (INT-2).
- **DBC-072** The builder MUST include only installed headers and link only `spade::` targets (SL2b).
- **DBC-073** A new third-party dependency (ImPlot, a STEP library) MUST NOT come in without the lead's approval.

## 13. Testing

- **Unit tests (spade_tests).** Everything the builder decides is display-free, so it is tested there:
  - authoring and its undo;
  - mapping refusals to fields;
  - resolving a case and its hash;
  - the resolution rules' refusals;
  - the estimate's arithmetic;
  - reading a run folder;
  - the comparison table and the settings diff.
- **No new physics goldens.** The builder owns no physics numbers. Its tests check that:
  - settings reach the solver unchanged (the resolved case's hash);
  - refusals show verbatim;
  - every number keeps its provenance and unit.
- **The live smoke tour gets a `builder` section:** author a part, make it porous, set up a case, run the disc live for a few seconds, queue a short CPU run, open its results, and compare two runs. Red by injection, then green, as Task 12 was.
- **The validation report:** a C8 run at the record's configuration appends a record that Test/Docs' reader accepts.

## 14. Order of work

Each stage waits on the needs in §10. The stages are W0-W8 so they don't collide with Core's B1-B9.

1. **W0.** The workspace, native parts in memory, the view, and the shared gizmo. Saving waits on Core's model YAML.
2. **W1.** Surfaces, and the compile's readout with provenance.
3. **W2.** Cases and the live disc. This is I1's harness; it needs P2, P3 and P7.
4. **W3.** The queue, run folders, the estimate and the Runs tab.
5. **W4.** Results and the validation report (I2).
6. **W5.** Comparing.
7. **W6.** Import (Core B4-B8).
8. **W7.** The actuator line and the blade definition (Physics' line source).
9. **W8.** Blade-resolved, offline (D3).

The airflow demo's panel (I1, I2) stays in the drone sim box for D1-D3. It and the builder share one harness, `AirflowRun`. The builder's case is the durable form of the demo's session options.

## 15. Look and feel

The editor spec's §13 applies unchanged. In particular:
- no boxes, borders or separators;
- sections fold one level deep;
- aligned label and value rows, with the unit in the value or, for a vector, in the label;
- one accent colour, and the warning colour only for status;
- advanced settings folded.

The builder adds only:
- the Runs tab, a list with one row per run;
- plots, drawn with the theme's colours and no frames.
