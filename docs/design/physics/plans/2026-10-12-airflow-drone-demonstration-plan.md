# The drone airflow demonstration — plan

**Owner:** Physics, with Rendering, Interface, Core and Test/Docs. **Status:** **approved by Cameron as written, 2026-10-12** (via the lead; `PHY-10`). Later folds: Core's `CORE-7` parts and the porous model, Interface's inputs, and Cameron's drone-box premise and blade definition (backlog `cd298d6`).
**Goal** (Cameron, 2026-10-12, word for word): "I want to ensure that regarding Spade, the aerodynamics grid solver, pressure surface visualizations, and overall computational fluid dynamics analyses are present with the conclusion and validation being a full 3D visualization of a drone and the airflow around that drone, including propeller flow, vortices, etc."
**Rests on:** `PHY-9` and the airflow spec (`2026-10-05-airflow-design.md`), plus Cameron's three answers of 2026-10-12, recorded as `PHY-10`:
- **Path:** a sandbox demo first, then the engine module.
- **Rotors:** the disc, the actuator line and blade-resolved, all three available.
- **Surfaces:** the drone is a solid wall, with porous as an option.

Inputs: Core (`../../core/2026-10-12-airflow-demo-core-inputs.md`, `a411a50`), Interface (`../../interface/2026-10-12-airflow-demo-interface-inputs.md`, `7292cbd`) and Rendering (2026-10-12).

## What the demonstration shows

**The scene is the drone box** (Cameron, 2026-10-12, backlog `cd298d6`, his words: "The current drone conditions will be like the drone sandbox, with a drone in the center plane, and the same manual movement, just with actual arm/disarm, adding a floor, etc, but the sandbox drone view premise is optimal especially since demonstrating the drone in different orientations and thrust states mimics a real drone visual much better and rhe aerodynamics can look much cleaner."):
- **The drone is fixed at the domain's centre.** It turns in attitude under the sandbox's manual controls, but never translates, and the box does not follow it.
- **Thrust follows the throttle,** with real arm and disarm, so the rotors spin up and down through the motor model. Spin-up sheds the start-up vortex ring.
- **The floor is optional.** With a floor, the box's lower face is a no-slip wall at the chosen height below the drone (ground effect, the outwash). Without one, that face is an outlet.
- **Wind gives forward-flight conditions:** an inflow face at the wind speed and an outlet opposite. In still air every face is an outlet, as in V1's C8.
- **Two flight conditions, and they differ only at the floor:**
  - **Forward flight at `v`:** in the drone's frame the air and the ground both pass at `−v`. Plan A's `DomainBc` gives this directly: the open faces carry `ambient = −v`, and the floor is a moving wall (`wall_velocity = −v`, a rolling road, as in an F1 tunnel). Without that, the floor would grow a boundary layer that real flight doesn't have.
  - **Hover in a wind:** the air passes and the ground is still, so `wall_velocity = 0`.
  - Interface's flags name the two conditions separately.

The scene shows:
- **The drone as a solid wall.** Its surfaces are coloured by the pressure coefficient `C_p` from the solved wall pressure.
- **Propeller flow.** The slipstream, its contraction and rotation (swirl), the ground outwash, and the start-up vortex ring.
- **Vortices.** Q-criterion iso-surfaces coloured by speed: the helical tip vortices from the actuator line (D1 onward) and, in D3, from the resolved blades.
- **The flow itself.** Streamlines seeded on a ring at each hub, tracer smoke through the props, and a pressure slice.
- **The analyses behind it.** The validation records, beside the picture in a report and in the HUD.

## What each rotor model can and cannot show

Every number in this table is an **estimate, to be replaced by measurement**: plan B's spike for the GPU, and each model's first run for the CPU.

Assumptions (**corrected 2026-10-13**, the lead): the **compiled reference quad**, `compile_airframe(reference_quad())` of `tests/test_airframe_compile.cpp`. It is 0.526 kg, plus layout, with hubs 0.11 m from the centre of mass and rotor radius `R` = 0.0762 m, hovering. Its hover fit gives 1.290 N and 0.0131 N m per rotor at 801 rad/s (7,650 rpm), `v_h` ≈ 5.4 m/s, and a tip speed of about 61 m/s. Adjacent hubs are 0.156 m apart against 2`R` = 0.152 m, so the props clear by 3 mm.
- **What changed:** this line first assumed `QuadrotorParams`' defaults (1 kg, 0.15 m arms, `R` = 0.12 m). Those defaults overlap: in the plus layout, adjacent hubs are 0.212 m apart against 2`R` = 0.24 m. The backlog has a row for fixing them.
- **The table below:** its grid and step figures were estimated for `R` = 0.12 m. With `R` = 0.0762 m, each "`R` ≈ n cells" needs a cell about 0.64 times as large.

| Rotor model | It shows | It cannot show | Grid near the rotors | Time step | Run time per simulated second |
|---|---|---|---|---|---|
| **Actuator disc** (exists; swirl is new work) | thrust and swirl; slipstream, contraction, rotation, outwash, the start-up ring | blade tip vortices: a disc makes a shear layer, not helices | about 1.25 cm (`R` ≈ 10 cells), one 128³ box of 1.6 m | the flow CFL, about 1 ms | GPU: real time is the target, which the spike settles. CPU: minutes |
| **Actuator line** (new) | everything above, plus the blades' helical tip vortices, blade passage and per-blade loads | blade surface pressure and boundary layers | about 5 mm (`R` ≈ 24 cells; Rendering's clean vortex core is about 2.6 cells), a nest around the rotors | the tip moves at most 1 cell per step: about 85 µs | GPU: minutes. CPU: hours (slow motion; recorded) |
| **Blade-resolved** (new) | everything above, plus blade surface pressure, the tip vortex as it forms, and blade–wake interaction | turbulence below the grid (WALE; §11.3); a wall model, not a resolved boundary layer, at this Reynolds number (about 8×10⁴ at the tip) | about 0.5 mm on the blades: about 70 M cells in a nest around one rotor | about 8 µs | offline: hours to days per simulated second, so a few revolutions, recorded |

Cameron expects this: "real time analysis being cool and all, but some stuff will take processing time." **The demonstration will say so on screen:** each run shows its model, its grid, and its real-time ratio.

## Milestones

| | Contents | Depends on |
|---|---|---|
| **D1, one-way CPU demo, recorded** | The drone-box scene: the solid drone fixed at the centre, turned in attitude, throttle with arm and disarm, an optional floor, wind for forward flight. Actuator disc with swirl, then actuator line. The sandbox drives the standalone solver (one-way, slow motion). Rendering renders frames offline on the CPU. `C_p` on the surfaces, tip vortices from the actuator line | plan A to Task 9 and 8b; P1–P4, P6, P7 below; R1–R2; I1 |
| **D2, GPU real time, live** | The same scene live: the disc in real time, the actuator line in slow motion (shown as such). Plan B's chain runner and kernels, a GPU pack of the flow volume, Rendering's GL views | D1; plan B; P8; R3; I2 |
| **D3, blade-resolved, offline** | One rotor resolved and validated against Caradonna–Tung. Then the drone's rotors from the builder's part geometry, a few revolutions recorded | D2; P5, P9; Core C1 |
| **D4, the engine module, two-way** | Plan C after stage 7: probe-and-subtract, drag pushback, and the solid body's pressure force replacing its drag element | stage 7 (Cameron signs its design); plan C |
| **Validation, shown at every milestone** | V1 (cavity, Taylor–Green, disc wake: plan A Task 11) in D1. Actuator-line validation in D2, blade-resolved validation in D3. Records in `tests/data/airflow/records/`, shown in a report and in the HUD | plan A Tasks 10–11; P9; T1–T2 |

## New work beyond plans A, B and C

**Physics**
- **P1. Swirl in the actuator disc** (spec §3.1). Plan A Task 9 carries thrust only. Add torque and spin direction: a tangential force with `Σ r_i |f_i| = Q`. A small addendum to plan A, after Task 9.
- **P2. Immersed solids in the real-time tier: static first, then rotating about the fixed centre.** Apertures `θ` per face from a posed body SDF each fluid step, evaluated only in the body's support box. The projection is weighted by `θ` (spec §2, Batty et al.). In the drone box the body never translates, so its solid faces take `ω × r` about the domain's centre. The general `v + ω × r` waits for a translating body (D4's module, or a later following box). Velocity is extrapolated two layers into the solid. Static solids also serve V2's cylinder. **Porous stays an option per part** (Cameron).
  - **The porous model is Darcy–Forchheimer** (confirming Core's `CORE-7` proposal). Each porous part carries a viscous resistance `1/K` (1/m²) and an inertial coefficient `C₂` (1/m). The grid applies the sink `−(ν/K) u − ½ C₂ |u| u` over the cells the part occupies, weighted by its SDF fraction. It is point-implicit, `u ← u / (1 + h (ν/K + ½ C₂ |u|))`, because large coefficients are stiff, and that form is stable at any `h`.
  - **The minimum resolution** (Core's stage 7 rule D5, numbers set by Physics):
    - a wall part's smallest declared feature must span **at least 2 cells**; the actuator line needs **R/dx ≥ 20**, with smoothing width 2 dx; a blade-resolved rotor needs **at least 32 cells across the chord and 2 across the thickness**;
    - anything smaller is refused by name, with the `dx` it needs. It never quietly becomes porous;
    - these are first values, to be revised by validation;
    - at R0's 2.5 cm the sandbox quad's arms (about 1.5 cm) are refused, so D1 runs a nest of about 6 mm around the drone.
  - **The fluid sub-step:** the actuator line and blade-resolved models need `m_f` fluid sub-steps per substep (stage 7 §4.1): the tip moves at most about 1 cell per fluid step. Spec §1.4's `h_f = j · dt` gains that second cadence.
  - **Drag elements:** a wall part has none once the coupling is two-way (D4), because the grid's integrated wall force replaces it. In one-way (D1–D3) the sim keeps its drag model for flight, and the grid's wall force is display only. So the drag element's presence follows the coupling mode, not the part's surface kind. A porous part's drag, two-way, is its Darcy–Forchheimer sink's reaction.
- **P3. Wall pressure.** The surface pressure at body surface points, and the integrated force and moment, for display (one-way). In D4 that force replaces the body's drag element, so nothing is counted twice.
- **P4. The actuator line.** Per blade: elements along a rotating line, with lift and drag from tabulated airfoil polars, chord and twist.
  - *Amended 2026-10-26 (with P11, merged at 97ddfdd):*
    - **The convention.** Blade `b` of `B` lies at azimuth `s_k ψ + 2π b / B` about the hub's +Y. `ψ` is P11's and grows with `ω ≥ 0` whatever the hand, and `s_k` is the rotor's `spin_dir`, as `posed_parts()` turns a drawn blade. So a drawn blade and P4's line coincide.
    - **Its floating-point flags** (Core's note). P11's rule is header-only, so `ψ + ω h` compiles under each includer's FP flags, while `spade_fp_strict` is PRIVATE to the engine libraries (`engine/CMakeLists.txt:94-106`). That is fine while `ψ` is visual only. When P4 feeds `ψ` into the step, the rule moves into an engine `.cpp` built once under `spade_fp_strict`, and its callers link it, so every caller's bits are the engine's.
  - **Its input is one blade definition per rotor** (Cameron, backlog `cd298d6`, Q3):
    - the blade count and radius, the hub radius, and stations along the span, each with chord, twist and an airfoil;
    - the airfoil's polars, `C_l` and `C_d` against α, imported with their source;
    - absent a polar, a flat-plate default, labelled "estimated" wherever it shows;
    - a polar that stops short of ±90° (XFOIL and tunnel tables stop near ±15–20°) is extended there by Viterna–Corrigan, with the blade's aspect ratio, labelled "extrapolated" (decided with Interface, 2026-10-12);
    - P5 lofts the blade mesh from the same definition.
  - The format is shared with Interface (the case format, Q8). The flat plate's trigonometry goes through `core/fp32_math` (`TD-3`). Each element's force is spread with a Gaussian kernel of width about 2 `dx`. The time step is limited by the tip (1 cell per step). It is selectable per rotor beside the disc. Its integrated thrust is reported against the sim's `k_T ω²`, since one-way they can differ.
- **P5. Blade-resolved.** Rotating blade geometry as a moving immersed solid in a nested fine grid (spec §11.4's nests), with a wall model. **One decision to bring back with measurements:** this belongs in the real-time NS tier with nests, or in the LBM tier (spec §1.6, where bounce-back handles moving walls directly).
- **P6. Publish's rule in solid cells** (Rendering's ask; it goes into plan A Task 8b). The first layer of solid cells (any solid cell with a fluid face-neighbour) takes a fluid-side extrapolation, so `C_p` reads the wall pressure. Deeper cells stay no-data. The header counts the extrapolated cells: a new `FlowVolumeHeader` field, which is Core's call too.
- **P7. The coupler.** A pure function in `physics/airflow/`, used by the sandbox harness in D1–D3 and by plan C's module in D4:
  - In: each rotor's hub, axis, ω, spin, radius and torque; thrust recomputed on the host (Core: not stored); the body's pose and velocity; the drone's SDF.
  - Out: the sources and the moving boundary for one fluid step.
  - What Interface's harness binds to, all in `physics/airflow/`:
    - **(i)** the coupler takes the rotors as a plain list the harness fills, one row per rotor: hub, axis, ω, spin, radius, torque, thrust, and the rotor's model;
    - **(ii)** the fluid-substep rule is one function: the flow CFL, plus the actuator line's tip rule (at most 1 cell per step). The harness calls it and never copies it;
    - **(iii)** the rotor model per rotor (disc, line, resolved) and the surface per body or part (wall, or porous with its two coefficients) are plain data, so the panel binds to them;
    - **(iv)** the flow-volume publish (8b) is callable per frame.
  - **No libm transcendental anywhere in P7** (`TD-3`): the blade azimuth, swirl and any trigonometry go through `core/fp32_math`, and `sqrt` is fine. Test/Docs' T3 golden of the coupler is pinned on MSVC and gcc and final on a fresh `-NoSeed` leg, so P7 is built this way from the start.
- **P8. The flow volume on the GPU.** Pack f16 on the device, or read back fp32 and pack on the CPU, whichever the spike measures cheaper. It is part of plan B (Core: no plan held it).
- **P9. Rotor validation data**, with provenance, reviewed by Test/Docs like Ghia. The candidates, each to be read from its source:
  - Caradonna & Tung (1981, NASA TM 81232): blade pressure and tip-vortex positions in hover, at the low-Mach condition. Incompressible at tip Mach 0.44, about a 10% compressibility caveat, stated.
  - Small-propeller thrust and power data (the UIUC propeller database) for the actuator line.
- **P10. The dye scalar** (Rendering). Tracers stand in for D1.
- **P11. The blade azimuth `ψ`**, a per-rotor array Physics owns, declared by the airflow module. It is advanced by `ω h` each step and read by the actuator line and blade-resolved models. It sits outside `RotorRow`, so every existing golden holds.
  - *Amended 2026-10-26 (P11 merged at 97ddfdd; Core's L7 plan; the lead's order):* P11's home is `engine/physics/blade_azimuth.hpp`, header-only and not the airflow module, because the editor's Run is a plain `Simulation` without that module.
    - `advance_blade_azimuth(float& psi, float omega, float h)` computes `ψ ← ψ + ω h`, then subtracts `kAzimuthTwoPi` (6.28318548f, `0x40C90FDB`) while `ψ ≥` it, giving [0, 2π) with `+ - *` only and no `fmod`.
    - `ω` is the rotor row's value after the rotors' step, the one its forces used.
    - A disabled rotor holds its `ψ`.
    - `ψ` outside [0, 2π), a negative or non-finite `ω` or `h`, or `ω h` above 64 rad is refused: `ψ` is held and the call returns false.
    - `advance_blade_azimuths(psi, rows, h)` advances every rotor.
    - It is held on the host, outside registered state, with nothing read back. The editor's Run and LiveRun call it, and `posed_parts()` applies `s_k` (`R_y(s_k ψ)`).
    - For now it is visual only (Core's L7c2).

**Core**
- **C1. Per-part model geometry, `CORE-7`** (`../../core/2026-10-12-model-parts-design.md`, approved). `ModelType::parts` gives each part a role (rigid, or spinning with rotor k), a body-frame mount, an exact `SdfProgram` shape and a wall or porous surface. A posing rule gives one world-frame SDF of the posed parts and the wall velocity `v + ω × r`. Parts are in YAML. Core builds it right after stage 5 Task 1. D1 uses it if it has landed, otherwise a harness SDF from `QuadrotorParams`. CAD inputs follow `CORE-6`: native CSG parts now, imported triangle meshes with exact BVH distance, and STEP through an offline tool later.
- **C2. The chain-runner review** (plan B; Physics' design note is owed now).
- **C3. Stage 7's design to Cameron, now, in parallel** (Core's recommendation). D4 waits on it.

**Rendering:** R1, offline CPU `render()` to frames (D1). R2, the `C_p` surface view (D1). R3, step 7, GL live: volume, iso, lines, points and surfaces (D2). Seed rings per hub come from P7's rotor list.

**Interface** (`7292cbd`):
- **I1, the harness:** `sandbox/airflow_run.hpp`, display-free, with one stepping core.
  - `advance()` moves one frame (1/30 s). Each fluid substep reads the rotors and the pose, calls P7, and steps the solver; the volume is published at frame end.
  - It is one-way, with no wall clock in stepped code.
  - Two front ends drive it. The headless one, `spade_sandbox --airflow-record --rotor disc|line|resolved --surface solid|porous --grid N --flight hover|forward:<v> --height h --seconds s --out dir`, serves D1 and D3. The window serves D2.
  - Under the drone-box premise, `--flight forward:<v>` becomes the wind through the box and `--height` the optional floor. The attitude and throttle come from the manual controls, or from a recorded input script when headless. Interface sets the exact flags.
- **The panel:** an "Airflow" window with the rotor model for all rotors plus a per-rotor override (blade-resolved is greyed live: offline only), the surface per body, the grid (dx and R/dx shown), the flight state, the ground height, and the views. Changing the model, surface or grid restarts the flow.
- **The run line** is in every frame and caption, e.g. "actuator line · 128³, dx 5 mm, R/dx 24 · 0.004× real time · t = 0.250 s".
- **I2, the readouts:** airspeed, α/β, and per-rotor thrust as `k_T ω²` beside the actuator-line integral. Also the P3 wall force and moment, `C_T`, and a Validation section read from `tests/data/airflow/records/`.
- **Live smoke:** an `airflow` tour main. It checks that each view drew, the volume was published, `C_p` is finite and the run line was shown, and that the mp4 is under 30 MB.

**Test/Docs:** T1, the record format (on `test-docs/validation-tolerances`). T2, the review of P9's reference data. T3, a small-grid golden of the coupler (P7) on the CPU.

## Order of Physics' work

1. Plan A Tasks 7–12. Task 8 adds the closed-domain refusal and the health check; 8b adds P6. V1's records come out of this step.
2. The chain-runner design note to Core, now, in parallel.
3. P1 (swirl).
4. P2 and P3 (solids and wall pressure): static, then rotating about the fixed centre (the drone box).
5. **P2b, the drone box's boundaries** (Interface's spec §9). The wind and the floor are already in plan A's `DomainBc`:
   - the wind is the open faces with `ambient`;
   - the floor is the lower face as a wall, with `wall_velocity` the air's in forward flight (the rolling road) and 0 in hover in a wind.
   P2b adds the box's placement, so the lower face sits `h` below the hub plane with the drone centred horizontally, and one function from the flight condition to the `DomainBc`, which the coupler (P7) calls. Its tests:
   - a rolling-road floor under a uniform `ambient` leaves the flow uniform;
   - a still floor under the same wind grows a boundary layer;
   - a floor height that leaves the drone's parts outside the box, or within 2 cells of the floor, is refused by name.
6. P7 (the coupler).
7. P4 (the actuator line). D1 needs P1, P2, P2b, P3, P4 and P7.
8. Plan B with P8. D2 needs it.
9. **P2c, the solids' coarse face weights** (the lead's ruling 1c, 2026-10-21). Series/harmonic coarse face weights, so a coarse face sees every fine plane it spans instead of every second one. This also fixes P2's finding 2, the sheet blindness. Since 2026-10-21 the solids' CG runs by residual: a floor of `kSolidsCgIterations` (8), then on to 1e-4, with a cap of 32. A cell-centred sphere needs 24 iterations because the coarse levels see a different body; P2c brings the count back toward the floor. It follows plan B's GPU work unless the cap starts firing. Even-grid solids bits shift at about 1e-6, inside the bands, so the solids goldens are re-cut under TD-1 with the lead's word.
9. P5 and P9. D3 needs them.
10. **The quad's four wakes against measured data** (the lead, 2026-10-22, from C8q's finding; after the GPU solver, once L5 builds the drones). A quad-level case against NASA Ames' measured hover and wind-tunnel data for the Phantom 3 and Iris+ classes (thrust and inflow per rotor; the library plan names the sources). Four interacting wakes have no closed-form reference:
    - C8q measured each rotor of the reference quad (tips 0.9R apart) at 0.81 of single-rotor momentum theory, and at 0.85 without swirl; rotor 0 alone in the same case read 1.06, C8's disc.
    - Linear mutual induction (each neighbour's inflow, as a sink at its disc) accounts for only about 3% at C8q's probe, 0.5R wide and 2R below. The rest, the wakes' mixing or the open box's response to four times the flux, is not yet separated.
    - C8q keeps the banded single-rotor check (rotor 0 alone, against momentum theory in C8's band) and reports the four-rotor ratios only.
11. **The swirl's root relief** (later). P1's constant circulation reaches the disc's inner ring at 0.41R, and it costs C8q's quad about 4% of the downwash at its 0.5R probe (0.81 against 0.85 without torque). The change: a circulation rising from 0 at the hub, as a real rotor's root unloads, designed and measured against C8q.
    - *Withdrawn 2026-10-22, after C8q's banded run:* a lone rotor's swirl costs nothing at the probe (1.066 of momentum theory with its torque, the four runs' mean, against 1.0617 without it). So the quad's ~4% from torque comes only with the neighbours, through the four wakes' interaction, not from a core vortex in each wake. Root relief is not indicated. The swirl's part in the quad goes to item 10's measured-data case.

## Spec changes (Cameron's, 2026-10-12)

§2 ("Moving bodies"), §3 (the rotor models) and §8 Q4 are amended to match. Q4's realm-settled C ("porous in the real-time tier") is superseded: a vehicle is a solid wall in both tiers, with porous as an option per body, and the three rotor models are selectable.
