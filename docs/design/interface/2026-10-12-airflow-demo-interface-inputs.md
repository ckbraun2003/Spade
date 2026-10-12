# Airflow demonstration: Interface's inputs

**Owner:** Interface. **Status:** inputs for Physics' demonstration plan (`../physics/plans/2026-10-12-airflow-drone-demonstration-plan.md`, I1 and I2), for the lead.
**Base:** master `cfa92be`.
**The goal** (Cameron, 2026-10-12): a full 3D view of a drone and the air around it, with propeller flow, vortices, surface pressure, and the CFD analyses behind it. Interface's part is the airflow scene and its panel, the view toggles, the airspeed, α/β and forces readouts, and the live-smoke steps that record it.

## Summary

- **One stepping core, two front ends.** A display-free `AirflowRun` steps the drone and the fluid. A headless recording command (D1) and the window (D2) both drive it.
- **The panel offers Cameron's choices:** the rotor model (disc, actuator line, blade-resolved) and the surface (solid, or porous). It also holds the grid, the flight state, and the views (volume, streamlines, iso, surface pressure, slice, tracers).
- **The run line is always on screen:** model, grid and real-time ratio, as the plan requires.
- **The live smoke proves each view drew,** and records the demo.
- **Interface needs nothing new from Core for D1.** It needs P7's signature from Physics, per-view drawn reports from Rendering, and a record reader from Test/Docs.

## I1. The harness (D1)

**Stepping.** The new file is `sandbox/airflow_run.hpp`, display-free (SL15b). It holds the `DroneSim`, the solver's grid, and the coupler (P7).
- `advance()` moves sim time by one frame, 1/30 s.
- Inside a frame, each fluid substep does three things:
  - it reads the rotors and the pose, as Core's inputs §1 lists and `drone_view.hpp`'s `air_field_from` already does;
  - it calls P7;
  - it steps the solver.
- The flow volume is published at the end of each frame (P6, plan A Task 8b).
- One-way: the drone never feels the air.
- The fluid substep comes from the flow CFL and, for the actuator line, the tip's one-cell rule. I'd like that rule as a Physics function, so the harness doesn't copy it.
- **No wall-clock time in stepped code.** The real-time ratio is measured around `advance()` by the display layer and only shown. Recorded frames are uniform in sim time, so a recording plays in sim time, with its slow-motion factor in the caption.

**The headless recording** (D1, and D3 for blade-resolved):
```
spade_sandbox --airflow-record --rotor disc|line|resolved --surface solid|porous
              --grid 64|96|128 --flight hover|forward:<m/s> --height <m>
              --seconds <s> --out <dir>
```
- It renders frames on the CPU (R1) and writes PNGs, a journal, the validation numbers and an mp4, using the live smoke's encoder.
- A tiny-grid run is cheap enough to be a CPU test.

**The window** (D2) drives the same `AirflowRun`. Before R3 lands it shows R1's CPU frames, as the CPU fallback path does today.

**The scene** is a scene file, `assets/scenes/drone_airflow.scene.yaml` (the quad and a ground plane). So it joins the scene picker (SL13) and opens in the editor.
- The airflow settings stay session options (the command line and the panel) until a scene-file field is specced. Lead: flag it if the airflow settings should live in the scene file. That would be Core's format change.

**The run line**, on every frame and in the window's caption: `actuator line · 128³, dx 5 mm, R/dx 24 · 0.004× real time · t = 0.250 s`.

## The panel

An "Airflow" window, docked beside "Drone". It follows the editor spec's §13: folded sections, aligned property rows, no boxes.
- **Model:**
  - Rotors: Disc / Actuator line / Blade-resolved, for all rotors. A folded "Per rotor" section overrides one rotor (P4: selectable per rotor).
  - Blade-resolved is greyed in the window, with its reason: "offline: record with --rotor resolved".
  - Surface: Solid / Porous, per body. The porosity shows only when Porous is chosen.
- **Grid:** the cell count and the box size. dx and R/dx are shown read-only. The nest settings arrive with P4 and P5.
- **Flight:** hover or a forward speed, and the height above the ground (ground effect).
- **Views,** combinable, each with its colour bar, units and auto-range, as today's Air speed view has:
  - Volume (speed or vorticity);
  - Streamlines (seeded on a ring at each hub);
  - Iso (Q-criterion, with a threshold, coloured by speed);
  - Surface pressure (`C_p`);
  - Slice (pressure, on a chosen plane);
  - Tracers.
- **Changing the model, the surface or the grid restarts the flow,** since the fluid state isn't carried across. Changing a view does not.

## I2. Readouts and live mode (D2)

- **Readouts** (the "Drone" window's Readouts section):
  - airspeed (Physics' definition: the mean rotor-inflow air minus v_body);
  - α and β in degrees, "—" below the low-airspeed threshold (the lead's ruling);
  - per-rotor thrust, the sim's `k_T ω²` beside the actuator line's integral (P4: one-way, they differ, so both show);
  - the wall-pressure force and moment in body axes (P3);
  - `C_T`.
- **Validation:** a folded section lists each record from `tests/data/airflow/records/` (T1): case, quantity, measured, reference, tolerance, verdict and source. It is read-only. The recorded report draws the same numbers.
- The HUD stays UI-1's single stats line; the run line is in the caption.

## The live smoke

A tour main `airflow` in `live_tour.cpp`. Its steps:
- hover with the disc, turning on the volume, streamlines, iso and surface pressure in turn;
- the actuator line, in slow motion;
- porous, then solid;
- forward flight.

Its checks:
- each view drew, reported by the sink per view as `help_drawn()` does;
- the flow volume was published (cells, and a bounded no-data count);
- `C_p`'s range is finite;
- the run line was shown;
- the real-time ratio is above 0;
- the mp4 is under the 30 MB sendable limit.

The tour opens a GL window, so it doesn't run while a game is running.

## What Interface needs

- **Physics:**
  - P7's signature, with the rotors as a plain list the harness fills from Core's §1 reads;
  - the fluid substep rule (CFL, and the tip rule) as a function;
  - the rotor model and the surface as data (per rotor, per body), so the panel binds to them;
  - the flow volume's publish, callable per frame.
- **Rendering:**
  - R1's `render()` to an image the sandbox writes as PNG;
  - per-view option structs the panel binds to;
  - a per-view drawn-or-refused report for the tour's checks, in the `render_gap.hpp` pattern.
- **Test/Docs:** T1's record format, with a reader in installed headers (SL2b).
- **Core:** nothing new for D1. The physics/airflow headers are already installed with `physics/` (`engine/CMakeLists.txt:654`), so the sandbox can use them under SL2b.
- **Later:** `ModelType::parts` and the builder's part-authoring UI get their own spec, once Core's parts land.

## Order (Interface)

1. Editor Task 9, in hand.
2. `AirflowRun` and the recording command against plan A's grid as it lands, with the disc first.
3. The panel and the command-line options. D1's recording follows once P2–P4, P7, R1 and R2 land.
4. The live mode, the readouts and the tour (I2) once R3 lands.
