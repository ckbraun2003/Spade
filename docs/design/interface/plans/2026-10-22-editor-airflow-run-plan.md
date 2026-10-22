# The editor's airflow run: M2, held first (N18) — plan

**Owner:** Interface. **Status:** PLAN for the lead's review, 2026-10-22. Nothing is built.
**Asked by:** the lead, 2026-10-22: "an airflow run in the editor. Run the selected drone with LiveRun's airflow (parts as solids, the status line from LiveSolids, the flow view and its render modes in the viewport). This is the step that puts the CFD into the Unity-like editor."
**Rests on:**
- the editor's end-goal design (`2026-10-19-editor-end-goal-design.md`, approved): §4's regions, M2 ("one `follow` region around one placed drone ... the editor's first airflow runs with the drone held: the drone box, live in the editor"), and §6's `editor.airflow` row with its physical predicate;
- LiveRun, the held drone box;
- LiveSolids (approved at af54a71, merging after batch 15): `pressure_line()`, `airflow::status_line()`, the HUD's `set_status_line()`;
- scene v5's `SceneAirflowRegion` (N5b, merged b3657c6);
- P2's solids and N15's drawn-only parts.

**Not in it:** a flying drone in its region. That needs P2b's moving faces, which follow the GPU solver by Cameron's order of 2026-10-22. Until then the region's drone is held, as M2 says.

## 1. What the user does

1. **Add a region.** Select a placed drone. The inspector's new **Airflow** section has "Add airflow region": a `follow` region, one-way, named `<drone>_air`, with the defaults of §4.
   - The inspector shows the region's resolve as LiveRun resolves it ("32³, dx 25 mm, R/dx 3"), or the refusal verbatim.
   - The viewport draws the region as a wireframe box around the drone (`set_view_lines`).
   - Remove and undo work as for any scene edit, and Save writes it (scene v5).
2. **Run with air.** The command bar gets a second run command, "Run air", beside Run (D1).
   - The region's drone is held at its placed pose at the box's centre, its rotors through the chain, flown by the Pilot section's throttle and attitude.
   - The rest of the scene stays as edited.
   - LiveRun publishes the volume each run frame.
3. **See it.** The view-mode menu gains the flow views once a volume exists: vortex cores, streamlines, slice, surface C_p. These are Rendering's modes through `editor_views::apply_to()`, which already takes a volume (the editor passes `nullptr` today). §5's default split shows vortices when a region runs.
4. **Read it.** The HUD's status line carries the run's pressure line, in the warning tone when any step went unconverged (LiveSolids). The run label (grid, dx, R/dx, the solids line, parts drawn only) is in the band, with the stated playback speed (§4).
5. **Stop.** Stop returns to editing. The held drone never moved, so the scene is as it was.

## 2. Pieces, all in Interface's files

| Piece | Where | What |
|---|---|---|
| T1 region component | `editor_scene_edits.hpp`, `editor_panels_ui.cpp`, `editor_workspace.hpp` | Add, remove and undo a `follow` region on the selected drone; the inspector's resolve line or refusal; the wireframe box. |
| T2 `AirflowRun` | new `sandbox/editor_airflow.hpp`, display-free | It builds the AnalysisCase from the region and the drone's model (§4) and resolves it with `analysis::resolve()`. It starts LiveRun, steps it to a wall budget (§4), and publishes the volume with `header.origin` shifted to the drone's world position (LiveRun's box is centred on its own origin). It also gives the drawn body's pose in the scene, `pressure_line()`, and the downwash frames for the tour. |
| T3 workspace and session | `editor_workspace.hpp`, `editor_session.cpp` | `Command::run_air` and stop; the held drone drawn at its scene pose plus the run's body; the volume into `apply_to()`; the status line; the flow modes in the menu. |
| T4 tour and docs | `live_tour.cpp`, the storyboard | The `editor.airflow` chapter (§5), its still, and the storyboard entry. |
| T5, after P2b | | The driven mode: the editor's Simulation flies the drone and LiveRun takes its state and rotor rows. Run and Run air become one. |

Each task goes red first, then green, with gcc, the CPU suite and the smoke as for any sandbox change.

## 3. Parts as solids, honestly

At a live grid, few library parts can be solids.
- **The rule:** a part is thin below 2 dx (`solids.hpp`, min of its declared `min_feature` and its smallest extent).
- **The 6-inch quads:** at R/dx 3 (dx 25 mm) a part must be 50 mm thick; arms, props and most bodies are not.
- **The Crazyflie:** at R/dx 3 (dx 7 mm), its 6 mm board is not 14 mm thick either.
- **Today's choice:** the tour's airflow chapter clears the model's parts for exactly this reason, so the discs carry the physics and the parts are drawn.
- **Where solids get real:** the GPU solver's finer grids (plan B) are what make solids at real thickness practical.

So the run puts every part that passes 2 dx into the air as a solid, and draws the rest. The run label names them ("3 parts drawn only: thinner than 2 dx at this grid"). This needs D2, because today a thin part that is not imported refuses the run instead.

## 4. Defaults and budget

| Default | Value | Why |
|---|---|---|
| Cells | 32³, cubic | The live grid the tour already flies. `AnalysisCase`'s box is one length, so M2's region is cubic. |
| dx | R/3 rounded down to the millimetre, so R/dx ≥ 3 | The tour's live R/dx (3.05). Per drone: the 6-inch quads and the reference quad 25 mm (box 0.8 m); fpv_5inch 21 mm (0.67 m); phantom3_class 39 mm (1.25 m); iris_plus_class 40 mm (1.28 m); crazyflie_class 7 mm (0.22 m). Each drone fits its box with room for the wake below. |
| Step h | 1 ms, unless Physics' rule (P-2) asks for less | It is airflow_live's step. At the Crazyflie's 7 mm cells, its far wake (about 8 m/s, twice the hover's induced speed) moves 1.1 cells a step, which is why P-2 asks for the rule. |
| Power | idle duty 0.05, state of charge 1.0 | airflow_live's, until the chain pin (INT-11). |
| Fluid | the region's density and viscosity (1.225 kg/m³, inviscid) | scene v5's defaults. |

**Budget: measure first (M0).** Nothing has measured LiveRun's cost per step at 32³ on this box.
- **Estimates only:** the storyboard scaled 64³ to about 33 ms per step, and the airflow design guessed 10 to 40 ms.
- **What the tour's chapter needs:** its quarter-speed playback needs about 250 steps per wall second, under 4 ms a step.
- **So before T2:** a headless, opt-in bench measures ms per step at 32³ with and without solids, in release, through the job queue. Physics checks the numbers.

**The pacing, whatever M0 finds.** Each displayed frame, the session runs whole LiveRun steps until a wall budget is spent: 25 ms for the physics, which leaves room for the flow view's measured 2 to 18 ms of render. The status line states the speed achieved ("air at 1/12 speed").
- **Timing:** the wall clock is the sandbox's (`steady_clock`, as AirflowSession already times `physics_ms`), never the engine's (M1B).
- **Determinism:** the steps themselves are deterministic; only how many run per displayed frame varies. The tour judges by simulated time, never by frame counts.

## 5. How the tour judges it: `editor.airflow`

On `hover.scene.yaml`'s placed reference quad (R/dx 3.05 at 25 mm, the airflow chapter's own setting, so its bands carry over):

| Check | Judges |
|---|---|
| `region-resolved` | The added region resolves as LiveRun resolves it: grid, dx and R/dx in the inspector equal the run label's. |
| `publishes` | Each run frame's volume is new: its generation advances, and its origin is the box placed around the drone. |
| `status-line` | The HUD's status line is the run's `pressure_line()` (empty for a converged run), in the warning tone exactly when it says anything. |
| `views-drawn` | Each flow mode the menu offers draws without a refusal, with its legends. |
| `downwash-present` | The design's PHYSICAL predicate: the airflow chapter's `end_goal::downwash()` with `kDownwashPresent`, over the hover's frames. Publishing alone would pass still air. |
| `air-speed` | Report-only: the playback speed achieved and the physics ms per displayed frame. |
| `round-trip` | `editor.save_reopen`: the region survives Save and reopen. |

Plus the `editor_airflow` still, taken the frame after the vortices first draw.

## 6. What I'd need

**From Physics:**
- **P-1 (D2):** a thin part of a library drone at a live grid is left out of the air and named, as N15's drawn-only parts are, instead of refusing the run. The classification is Physics' (`classify_part()`); the call would be in LiveRun, Interface's file. Or Physics prefers the refusal, and the editor clears the parts as the tour's chapter does.
- **P-2:** the step's rule for small cells. Is h = 1 ms sound at 7 mm cells with an 8 m/s wake, or should h scale with dx (h ≤ c·dx/v)? A refusal by name would do.
- **P-3:** checking M0's numbers.
- **Later:** P2b, for T5.

**From Rendering:**
- **R-1:** confirm the flow modes need nothing beyond `apply_to()` with the volume in the editor's GL present path (`attach_editor`, the GPU frame, the shared `ViewCache`).
- **R-2:** confirm nothing assumes a volume at the world origin, since the editor shifts `header.origin` to the drone's position.
- **R-3:** the editor viewport's flow costs, at the sizes the editor-layout measurements used, to size the wall budget.

**From Core:** nothing. Scene v5's regions are merged, and the region's edit and save are Interface's.

## 7. Decisions for the lead

- **D1, where it runs.** Recommended: a separate "Run air" command for M2. It is the drone box, live: the region's drone held, the rest of the scene static.
  - The alternative is one Run, in which the region's drone is held while every other body flies. That mixes a held drone into a moving scene, and its contacts would be wrong.
  - Run and Run air merge at T5, when P2b lets the drone fly.
- **D2, thin parts.** Recommended: drawn-only and named (P-1). The alternative is refusing, which leaves the editor clearing all parts, so the run never shows a solid.
- **D3, the budget.** Recommended: M0 first, then the wall-budget pacing with the stated speed. The alternative is a fixed playback like the tour's quarter speed, which stalls the window whenever a step costs more than about 4 ms.
- **D4, the region's life.** Recommended: an explicit scene component (§1), saved with the scene. The alternative is an implicit box made each time Run air starts, which is simpler but is not saved and has no resolve to show before running.

## 8. Order

M0, the bench; then T1 the region; T2 `AirflowRun`; T3 the workspace and session; T4 the tour. One branch per task, each READY on its own. T5 waits for P2b.
