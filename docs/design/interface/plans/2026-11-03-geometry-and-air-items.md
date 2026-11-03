# Interface items left for the drone geometry and the air

Interface, 2026-11-03, for the lead. This is Cameron's current focus: the quad-rotor geometry and the fluid sim, optimized and working on both CPU and GPU. Below is what Interface still owes it, in the order I recommend. Each item says what exists now, what is left, and what it needs from another realm. Nothing here is built. The chase camera (C) is held and isn't listed.

## 1. Run air with the drone's body as a solid, on the GPU (B8's LiveRun hook)

- **Now:** LiveRun's device seam (`gpu_live_fluid()`, Rendering's slice 2) passes the run's solids to GpuFluid, which takes them since G3 (`68761817`). Run air starts on the device when the device takes the case, and on the CPU otherwise (slice 3, `editor_air_gpu.hpp`). But `air_model()` (`editor_airflow.hpp`) clears the drone's parts, so no editor or tour run puts a solid in the air. That is the lead's standing ruling: discs only until solids come in on the GPU's finer grid.
- **Left:**
  - **Which parts enter the air.** Today's comment defers this to Physics' P-1: thin parts are drawn only, thick ones are solid.
  - **A cell that resolves them.** Run air's cell is R/3 of its region's drone, which is 39 mm at 32³ for the Phantom.
  - **The run's label naming the solids.**
  - **A rule for the CPU fallback.** Discs only, or refuse.
- **Needs:**
  - Physics: P-1's rule.
  - Rendering: the device's step time and memory at the chosen grid.
  - The lead: a ruling on the CPU fallback.
- **Tests:** a CPU twin at a small grid. Then `GpuEditorAirflow` to the bit on the device of record.

## 2. The placed Phantom's air on the GPU (Test/Docs' gap 4, with Rendering)

- **Now:** `GpuEditorAirflow` runs the reference quad. The placed Phantom's air runs only on the CPU (`EditorAirflow.ThePlacedPhantomsAirStartsAndPublishes`), as discs.
- **Left:** `GpuEditorAirflow` with the placed Phantom, a few frames to the bit, opt-in if slow. Discs only, it can start now, and it grows with item 1 once the parts are solid. The dense disc and the prop-hub geometry are on the device today; mesh solids join after item 1.

## 3. The airflow chapter on the device, then with the body (poster 02)

- **Now:** the chapter flies the library Phantom as discs (A, this batch) on the CPU. `AirflowSession::create` starts LiveRun with no device maker.
- **Left:** start it through `start_live_run()` as Run air does, so the band names the backend. After item 1, keep the parts the rule passes, so the poster shows the wake meeting the body.
- **Needs:** item 1's rule and grid. The chapter's frame-time budget at that grid on the device.

## 4. B: the INT-1 drone box (poster 01) draws a meshed quad with turning props

- **Now:** the box simulates `QuadrotorParams` and draws layout discs.
- **Left:** draw a library-style meshed quad whose geometry matches the params, with props posed by their azimuth (`advance_blade_azimuths()`).
- **Needs from Core, the builder mapping:**
  - the model the params map to, for example `basic_quad` with a larger look;
  - which of the params set its arm length, rotor radius, mass and inertia;
  - whether the box keeps `QuadrotorParams` (the azimuth from its rotor speeds) or flies the model's rotor rows.
