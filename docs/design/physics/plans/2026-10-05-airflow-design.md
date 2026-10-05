# 3D airflow: grid solvers as field providers — joint spec

**Owner:** Physics (lead for this spec), with Core, Rendering and Interface. Test/Docs owns the gates. **Status:** DRAFT for Cameron's approval. Reviewed by Physics 2026-10-05; every realm's answers folded in (Core, Rendering, Interface, Test/Docs: §10); to the lead for the user, with the questions in §8. **Base:** master `e3b0560`, refreshed at `8ffe9fe` for what merged since (M0 done: T2–T4 and `CORE-5`; `PHY-7` approved; today's `TD-14` rules in §4.6). File:line anchors are still `e3b0560`'s; each realm re-checks its own at the cut.
**Ruling this answers:** the user's, 2026-10-05 (`backlog.md:63`, open item `backlog.md:73`): "We really need this to be a full 3D simulation of airflow and dynamics, grid solver and everything. Also I want the gpu set up asap, since that allows for the complex physics and visualization I am looking for." His answers: both solvers as fidelity tiers per region (incompressible Navier-Stokes real time, lattice Boltzmann high fidelity), two-way coupled; real time first at about 128³ on the RTX 3060 Ti; volume rendering, streamlines, smoke and an airspeed and forces HUD, not slices; GPU solver as the product with a small-grid CPU reference, banded under `TD-14` (`L4`).
**Marking:** a claim marked *(unconfirmed)* could not be checked in the code or docs. Every timing in this spec is an estimate from bandwidth arithmetic until the spike (§9, M1) measures it.

## Summary

Air becomes a solved field. A new optional built-in module, `airflow`, owns a 3D grid region and answers the existing `wind` (and `density`) field at every sample point inside it, so drag, rotors and sensors get local air through the field registry they already read. The real-time tier is an incompressible solver on a staggered (MAC) grid: semi-Lagrangian advection with a MacCormack correction, a pressure projection by geometric multigrid with a fixed cycle count, immersed boundaries from the world SDF as face apertures, and optional vorticity confinement. It steps once every `j` physics steps (a fixed multiple, about 4 ms), and its sources are the impulses rotors and drag elements accumulate in between, so momentum given to the vehicle is taken from the air exactly. At 128³ it needs about 194 MiB and an estimated 1.6–3.3 ms of GPU per fluid step, which puts real time at 4 ms fluid steps within reach but unproven. The lattice Boltzmann tier (D3Q27, one distribution copy, about 274 MiB at 128³) is not real time at useful resolution on this card: its time step is tied to the lattice speed of sound, so 128³ runs an estimated 12–16× slower than real time. It is the high-fidelity tier, for small nested regions or slow-motion runs, and it comes after the real-time tier ships. Both tiers fit 8 GiB with room to spare at 128³; at 256³ only one fits at a time under a 4 GiB airflow cap. The CPU runs the same discretisation at 16³–32³ as the reference, with a golden, and the GPU is banded against it to a measured chaos horizon, then held to invariants and statistics. What Spade lacks today is mostly Core's, gathered into a new module-API stage 7: a hashed grid configuration, device-resident state (the Vulkan step reads back the whole registered walk after every `step(n)`), point samples, regions, a step recording that can skip the fluid on most steps, 3D dispatch, and device-limit and budget checks. Where stage 7 sits (Q11) and the spec amendment it needs (Q12) are Cameron's. "GPU asap" began with banded-parity T2–T4, now merged: the 3060 Ti is admitted.

## 0. What exists today

| Fact | Where |
|---|---|
| A field is one value per world per substep, written in Fields and read later. The built-in prefix is gravity, density, wind | `engine/sim/module.hpp:71-100`; `engine/physics/field_row.hpp:26-44` |
| Every provider is position-independent. Per-point samples "wait for SPH" | `engine/physics/sampled_medium.hpp:9-12`; `core/07-status.md:19` |
| Regions: not built. The whole world is the only partition | `core/07-status.md:18`; spec `core/plans/2026-10-02-module-api-design.md` §11 |
| `wind` comes from Dryden: mean wind plus one gust per world | `engine/sim/standard_modules.cpp:10-20`; `physics/01-fields-and-media.md` |
| Two providers of one field are refused at `create()` | `engine/sim/module.hpp:147-156` |
| Rotors sample the medium at the hub; drag at the body's position | `engine/vehicles/rotor.cpp:318,357`; `engine/physics/forces.cpp:39` |
| Rotor thrust: `k_T ω² · f_inflow(v_axial) · f_ground(φ)`; `v_axial` reads the wind | `engine/vehicles/rotor.hpp:80-83`; `physics/02-responders.md` |
| Aero T3 ("two-way coupled to a solved flow field") is named and not built | `physics/02-responders.md`, aero tiers table |
| The only flow field is an analytic actuator-disc wake, CPU only, best-effort, never read by stepping (`PHY-3`), drawn as a 2D slice | `engine/vehicles/rotor_wake.hpp:8-13,77-89`; `sandbox/drone_view.hpp:1-9,205-216` |
| A field layer is a slice; "a volume waits for a consumer" | `engine/render/field_layer.hpp:73-83`; `rendering/plans/2026-10-03-field-channel-plan.md:29` |
| The Vulkan step uploads the whole walk when dirty and reads it all back after every `step(n)` | `engine/sim/simulation.cpp:881-896`; `engine/compute/vulkan/state_mirror.hpp:1-10` |
| One step's command buffer is recorded once and resubmitted; only the tick varies per submit | `engine/compute/vulkan/step_recorder.cpp:492-497,714-740` |
| One descriptor set holds every storage buffer; bindings come from `bindings.slang` | `engine/shaders/shared/bindings.slang` |
| The 3060 Ti is admitted (`CORE-5`, banded-parity T4, merged `e146575`) and every `gpu` test runs on it. It flushes fp32 denormals (sign-preserving FTZ+DAZ), and its div and sqrt are 1 ulp off correct rounding; GPU results are banded against the CPU (`TD-14`) | `core/plans/2026-10-05-banded-parity-plan.md` §1, T2–T4; `core/2026-10-04-nvidia-denorm-report.md` (finding 4) |
| The drone box steps at `dt` = 2 ms with 2 substeps (`h` = 1 ms) and refuses Vulkan, because its stand is CPU behaviors | `sandbox/drone_sim.hpp:210-211,217-221` |
| The sandbox quad: 1 kg, rotor radius 0.12 m, hover induced velocity 4.70 m/s; the heatmap shows 0–9.40 m/s | `sandbox/drone_sim.hpp:100-104`; `engine/vehicles/quadrotor.hpp:146`; `interface/07-status.md:18` |
| The GL renderer runs a 4.3 core context (compute shaders and 3D textures available) | `engine/render_gl/gl_renderer.cpp:159,774`; `sandbox/gl_target_sink.cpp:270-271` |

## 1. The solvers as field providers

**Owner:** Physics (solvers, providers). Core (module API, regions, state kinds, sampling).

### 1.1 Two tiers

| | Real-time tier | High-fidelity tier |
|---|---|---|
| Equations | Incompressible Euler with numerical dissipation (an implicit LES). Physical viscosity is negligible at these cells: `ν h_f / dx²` ≈ 1e-4 | Weakly compressible lattice Boltzmann, with viscosity through the relaxation time and a Smagorinsky subgrid term |
| Grid | MAC: velocity components on faces; pressure and dye at cell centres | D3Q27 lattice (recommended; §8 Q5), cell-centred |
| Time step | `h_f = j · dt`, fixed at `create()` (§1.4) | `dt_lbm = h / m`, fixed at `create()`; `m` ≈ 12 at 2.5 cm |
| Real time at 128³ on the 3060 Ti | Plausible at `h_f` ≈ 4 ms (estimate, §4.2) | No: about 12–16× slower (estimate, §1.6) |
| Fits | The whole air around the vehicles | A small nested region, or a slow-motion run |
| Grade | CPU reference (golden at 32³); Vulkan banded (`TD-14`) | The same, at 16³–32³ |

### 1.2 The domain

A region is an axis-aligned box of `nx × ny × nz` cells of side `dx`, on a lattice anchored to the world origin, so a cell's world position is `origin + (i, j, k) · dx` with an integer origin cell. This is the module-API spec's region (§11): a box with a priority that binds one provider per field.

| Mode | What it does | Use |
|---|---|---|
| **Fixed** | The box never moves | Test scenes, hover over a pad, the CPU reference |
| **Following** | The box tracks one body. When the body is more than `s` cells (default 8) from the box centre on an axis, the box moves by a whole number of cells. Grid values are copied by that integer shift, with no interpolation, so recentring adds no diffusion and the same world point keeps the same lattice. New cells take the ambient state | Flight |

Recentring is decided on the device from the tracked body's position, in Fields, at a fluid step only. It is a pure function of state, so it is deterministic per backend. Near the threshold a CPU and a GPU run can recentre on different fluid steps; parity scenarios therefore use fixed mode or keep the body far from the threshold.

Cell size at 128³:

| Box side | `dx` | Sandbox rotor disc (0.24 m) in cells |
|---|---|---|
| 2.0 m | 1.56 cm | 15.4 |
| **3.2 m** (recommended for the sandbox quad) | **2.50 cm** | **9.6** |
| 4.0 m | 3.13 cm | 7.7 |
| 6.4 m | 5.00 cm | 4.8 |

A 5-inch racing prop (R ≈ 6.4 cm, *unconfirmed* as a Kat part) is about 5 cells across at 2.5 cm. Below about 4 cells across, the disc is a smeared momentum source, and the coupling (§3.2) does not rely on the grid resolving the inflow.

### 1.3 Per-region fidelity

Regions resolve by priority (module-API spec §11): at a point, the highest-priority region that contains it answers.

| Priority | Region | Provider of `wind` |
|---|---|---|
| 0 | The whole world (always exists) | Dryden (today's) |
| 1 | The airflow box | The real-time tier |
| 2 | An optional nested high-fidelity box, inside the priority-1 box | The LBM tier |

**Nesting is one-way.** The LBM box takes its boundary values from the real-time grid. Both solvers receive every source inside them, so each is a complete model of the air. A sample point is answered by exactly one tier, the highest-priority region that holds it, so no reader sums two tiers. Two-way exchange (LBM feeding back into the outer grid) is later work. The first deliverable has one real-time box per world and no LBM.

A vehicle outside every airflow box reads Dryden, and its sources are dropped (§3). The pass publishes an "outside the airflow region" flag per body, and the HUD shows it (`L6`: announced, not silent).

**The box edge is a discontinuity, and it is stated.** A body crossing it switches from Dryden's wind to the grid's. The inflow faces carry the same ambient (mean wind plus the current gust), so the jump is the local wake only, which is the point of the box. If it shows in a measurement, a blend band a few cells deep is the later fix; the first deliverable does not blend.

### 1.4 Time stepping against the physics dt

`L1` fixes `dt` and the substep count at `create()`. The fluid step is a fixed multiple:

- **Real-time tier:** `h_f = j · dt`, with `j ≥ 1` in the configuration (hashed). The fluid step runs in Fields of substep 0 of every `j`-th step. Bodies sample the latest fluid field at their current position every substep, so positions move at substep rate and the air at fluid rate.
- **LBM tier:** `m` LBM steps per physics substep, `m = ceil(u_design · h / (u_lat,max · dx))`, fixed at `create()`.

**Why a fixed multiple, not adaptive CFL.** A count chosen from the state's maximum speed is deterministic per device (a max is exact), but a CPU and a GPU run would pick different counts near a boundary, and the recorded command buffer cannot change its dispatch count per submit. Instead:
- the backtrace is subdivided at `create()` so each segment moves at most one cell at the declared design speed `u_design` (default 30 m/s; `n_trace = ceil(u_design · h_f / dx)`);
- semi-Lagrangian advection is unconditionally stable, so a flow faster than `u_design` loses accuracy, not stability;
- the pass publishes a CFL monitor, `max|u| · h_f / dx` (an exact max, a diagnostic that stepping never reads), and the HUD announces a run above the design speed.

CFL `= u · h_f / dx` at `dx` = 2.5 cm:

| `h_f` | 10 m/s | 20 m/s | 30 m/s |
|---|---|---|---|
| 1 ms | 0.4 | 0.8 | 1.2 |
| 2 ms (`j` = 1 in the sandbox) | 0.8 | 1.6 | 2.4 |
| **4 ms** (`j` = 2, recommended start) | 1.6 | 3.2 | 4.8 |
| 8 ms (`j` = 4) | 3.2 | 6.4 | 9.6 |

The sandbox quad's far wake is 9.4 m/s, so 4 ms gives CFL 1.5 there. A racing quad at full throttle can exceed 30 m/s in its wake *(estimate from momentum theory)*; that run is announced, not refused.

**Sources across steps.** Between fluid steps, each rotor and drag element accumulates its reaction impulse `J = Σ F·h` (and, for rotors, `Σ Q·h` and an impulse-weighted hub position). The accumulators are registered state (`per_row` rows owned by `rotors` and `drag_bodies`, stage 4's extent), because a snapshot can fall between fluid steps (`L2`). The fluid step applies them and zeroes them.

### 1.5 Real-time tier: the discretisation

**Grid.** MAC. It gives an exact discrete divergence after projection, no odd-even pressure modes, and natural face apertures for immersed boundaries. A collocated grid would need a Rhie-Chow-type correction and still checkerboards; it is rejected.

**One fluid step**, in order:
1. **Recentre** (following mode) and refresh the solid data of shifted-in cells (§2).
2. **Sources.** Gather the accumulated impulses onto faces (§3.1, §3.3): `Δu_face = J · w_face / (ρ dx³)`, with weights that sum to 1, so the momentum added is `J` to fp32 rounding. Zero the accumulators.
3. **Boundaries.** Domain faces, solid faces, and two fixed layers of velocity extrapolation into solid cells (§2).
4. **Advect** velocity and dye: semi-Lagrangian with an RK2 backtrace subdivided into `n_trace` segments, then the MacCormack correction with a min/max clamp to the stencil of the departure cell. Interpolation uses the lerp form `a + t·(b − a)`, so a uniform field is reproduced bit for bit (a tested invariant, §4.6).
5. **Vorticity confinement** (optional, `ε` in the configuration, default 0): `f = ε · dx · (N × ω)`, `N = ∇|ω| / |∇|ω||`. It is non-physical energy injection that offsets numerical dissipation, and stepping reads the result, so it is a hashed parameter, not a display setting.
6. **Project.** Solve `∇·(θ ∇p) = (ρ / h_f) ∇·(θ u*)` with face apertures `θ` (§2), then `u = u* − (h_f / ρ) ∇p` on faces with `θ > 0`. Faces with `θ = 0` take the solid's velocity.

**Pressure solver.** Geometric multigrid V(2,2) cycles: red-black Gauss-Seidel smoothing, full-weighting restriction, trilinear prolongation, levels 128 → 64 → 32 → 16 → 8 → 4, apertures coarsened by averaging, and the 4³ level solved by 16 red-black sweeps inside one workgroup. The cycle count `n_V` is fixed in the configuration (default 2); nothing terminates on a residual. The previous step's `p` is the initial guess, so `p` is registered state.

Why this solver:
- Red-black Gauss-Seidel is order-free within a colour: each cell's update reads only the other colour. The CPU's loop order cannot change a bit, so the CPU twin computes the same per-cell expressions as the GPU.
- Jacobi needs O(N²) iterations for the low modes: about 10⁴ at 128³ for a 10⁻² reduction from a cold start. Real-time engines run 20–80 Jacobi iterations and accept a field that is visibly not divergence-free. One Jacobi iteration costs about as much as one red-black sweep (§4.2).
- Multigrid-preconditioned CG converges faster per cycle, but needs dot products. Their reduction order differs between the CPU and GPU, and their scalars feed back into every cell, which amplifies differences. It is rejected for the first cut.
- A V(2,2) cycle reduces the residual about 10× on a plain Poisson problem; apertures degrade that to perhaps 3–5× *(literature figure, unmeasured here)*. Two cycles with a warm start are the real-time default. The spike (§9, M1) measures the residual each cycle count leaves.

**Ambient and density.** `ρ` is the world's `density` field (constant, `environment`). The ambient velocity, used at inflow faces and for shifted-in cells, is the mean wind plus the current Dryden gust, read from the `dryden` state at the fluid step. Inside the box, the gust therefore arrives through the inflow faces and is convected, where today it changes everywhere at once. That is a modelling change readers inside the box will see.

### 1.6 High-fidelity tier: lattice Boltzmann

| Choice | Recommendation | Why |
|---|---|---|
| Lattice | D3Q27 (§8 Q5) | D3Q19 lacks some isotropy and is known to misbehave in high-Reynolds jets, which a rotor wake is |
| Collision | Recursive-regularized BGK plus Smagorinsky (`C_s` ≈ 0.1) | At 2.5 cm and `dt_lbm` = 83 µs, `ν_lat = ν dt/dx²` = 2.0e-6, so `τ = 3ν_lat + 0.5` = 0.500006. Plain BGK is unstable that close to 0.5; the subgrid term raises the effective `τ` |
| Storage | fp32, one distribution copy, esoteric-pull streaming (in place) | Half the memory of two copies; each cell reads and writes only its own links, so it stays per-cell independent |
| Forcing | Guo's scheme for actuator disks and drag pushback (§3) | Second-order body forces |
| Walls | Halfway bounce-back from the SDF first; interpolated bounce-back (wall distance from `φ`) later; moving walls with the wall-momentum term | §2 |
| Stability | `u_lat ≤ 0.1` (Mach ≤ 0.17) at `u_design`; `τ_eff ≥ 0.5 + δ` | Compressibility error grows with Mach² |

**Why it is not real time at 128³.** Its explicit step is bound to the lattice speed of sound: `dt_lbm = u_lat,max · dx / u_design`. At 2.5 cm and 30 m/s, that is 83 µs, 12 LBM steps per 1 ms substep, 12,000 per simulated second.

| Region | Traffic per LBM step | GPU per step (est.) | GPU s per simulated s |
|---|---|---|---|
| 128³, D3Q27, 2.5 cm | ≈ 232 B/cell, 487 MB | ≈ 1.36 ms | ≈ 16 |
| 128³, D3Q19, 2.5 cm | ≈ 168 B/cell, 352 MB | ≈ 0.98 ms | ≈ 12 |
| 64³, D3Q27, 2.5 cm (1.6 m box) | 61 MB | ≈ 0.17 ms | ≈ 2.0 |
| 64³, D3Q27, 1.25 cm (0.8 m box, around one rotor pair) | 61 MB | ≈ 0.17 ms | ≈ 4.1 |

(Bandwidth at 80% of 448 GB/s, *unconfirmed*.) Raising `u_lat` to 0.2 halves the cost at about 4× the compressibility error. Hence §8 Q6: the LBM tier is a slow-motion or small-region tier, and it follows the real-time tier.

### 1.7 State ownership

The real-time tier at 128³ holds three kinds of storage:
- **state** (registered, snapshotted, digested): velocity, pressure, dye, the region row (origin cell, recentre count), the impulse accumulators;
- **scratch** (not walked, not snapshotted, rebuilt before it is read): advection buffers, multigrid levels, the force field, vorticity;
- **derived** (a pure function of configuration and the region's position): apertures and the static SDF distance.

What each module-API stage provides, and what is missing (Core's answers, `core/2026-10-05-airflow-spec-core-answers.md`, at master `9308fe0` and `core/module-state`):

| Need | Today | Status, and the stage that carries it |
|---|---|---|
| Module-declared arrays; a `per_row` extent for the accumulators | Built: stage 4 Task 1 merged (`eb57d4d`), Tasks 2–4 on `core/module-state`. A `per_row` owner may belong to another module, and the despawn cascade clears the children | **Built** |
| An optional read: `rotor.forces` reads the airflow's probe samples and binds nothing when airflow is absent | Built on the CPU only (`b40d0ad`), and it binds arrays (and tables, Task 7), not field samples. The GPU half is stage 6's fixed-binding pattern | CPU built; GPU **stage 6** |
| A **scratch** kind: not walked, zero at `create()` | Agreed, **not built**: stage 4 as built has none, and `PHY-7` hand-registers `contact_dv` (a `Simulation` member, a mirror derived entry, binding 27). Airflow's scratch has a different invariant from `contact_dv`'s: "rebuilt before it is read", not "zero at every substep boundary" | **Stage 4 before merge, or stage 7** (the lead decides) |
| A **grid extent**: a grid per world, sized from the module's configuration | **Partly there**: `ArrayDecl` takes a plain `elem_size`, and a descriptor may be built at run time, so a whole grid fits one `per_world` row sized at run time, with one `slot_to_world` entry per world (not one per cell). Missing: a **hashed module configuration**. The identity spells only names, versions and pass order, so two simulations with the same cell count and another `dx` or `j` would share an identity, and a restore across them would be accepted (`L2`) | **Stage 7**, with a spec amendment (§8) |
| **Device-resident state**: walked and snapshotted, but outside the per-step upload and readback | **None.** The Vulkan step uploads the whole walk when dirty and reads all of it back after every `step(n)` call; one dirty flag covers everything, and any host write sets it (`set_rotor_commands` included, so a host flight loop re-uploads the whole walk every step). The arena is "the one authoritative CPU-side store" | **Stage 7**, with a spec amendment (§8): it ends the arena's role as the one authoritative store |
| **Derived buffers rebuilt after `create()` and `restore()`** (apertures and the SDF distance depend on the region row, which a restore replaces) | **None**: no hook rebuilds a derived buffer, and on the GPU a derived buffer is never uploaded, so a kernel must run before the first step after a restore. The CPU needs the same hook | **Stage 7** |
| Grades per backend and the grade check (CPU reference, Vulkan banded) | — | **Stage 5** |
| Roles | — | **Not needed.** Coupling is the airflow module's presence, not a swapped tier (§3.4) |
| Fixed GPU bindings that hold an empty buffer when a module is absent; a recipe switch | — | **Stage 6** (the lock's pattern). Airflow is an optional built-in with kernels, and `rotor.forces`' optional probe read on the GPU needs it too |
| The lock, so the drone box runs on Vulkan | — | **Stage 6.** The airflow scene does not wait for it (§6) |
| **Per-point sample rows**: per body, per element × k probes, per sensor, and host Publish point requests | **None**: one field row per world (binding 26 on the GPU). Module-API design §6 specifies points per body, element and world; no stage schedules them | **Stage 7** |
| **Regions**: several providers of one field, each bound to a region, priority resolved per point once per substep in Fields; the region row as state; the per-body "outside" flag; `sample_medium` routed by region | **None**: `compile_schedule` refuses a second provider of a field. Module-API design §11 specifies regions; no stage schedules them | **Stage 7** |
| **Two recordings per step**, with and without the fluid step, chosen per submit by `tick mod j`; the substep index in `SubstepContext` for the CPU twin | **One recording**, resubmitted with only the tick varying. A recipe can already differ by substep index at record time. A data-driven skip would cost about 115 empty dispatches and barriers **per skipped step**, about 0.46 ms, a quarter of a 2 ms step at `j` = 2 | **Stage 7** (a small recorder change: a second command buffer recorded once at `create()`, no per-step cost) |
| **Device limits and 3D dispatch** (§4.1) | Every dispatch is 1D; no device limit is read anywhere in `engine/` | **Stage 7** (the limit refusals may land earlier) |
| **Device-only buffers** (§4.3) | Every walked and every derived buffer has an equal staging buffer | **Stage 7** |
| A VRAM budget check at `create()` (`VK_EXT_memory_budget`), refusing a region set that does not fit (`L6`) | Refusals by element count only; every entry is sized before any is created, which is where the check fits | **Stage 7** (may land earlier) |

**Where stage 7 sits is Cameron's decision** (§8, Q11). Core recommends 4 → 5 → 6 → 7: airflow on Vulkan needs stage 6's fixed bindings and stage 5's grades. Stage 7 is defined by capability, not by its first consumer, and is tested with a CPU-only, position-dependent test provider, so whichever of airflow and SPH comes second needs no Core change. SPH needs point samples and regions from it (§8 Q8); airflow alone needs residency, the grid extent, two recordings, 3D dispatch and the budget check.

### 1.8 Placement in the schedule

| Phase | Passes (airflow's in bold) |
|---|---|
| Fields | kinematic behaviors; `dryden.advance`; `environment.sample`, `dryden.sample` (whole-world region); **`airflow.step`** (on fluid steps: recentre, sources, boundaries, advect, confine, project); **`airflow.sample`** (wind and density at every point its region holds) |
| Forces | `rotor.forces` (optional read of the probe samples; writes `T`, `Q` per rotor to its own scratch); `drag.forces` (writes `F` per element to its own scratch); **`airflow.accumulate`** (`after: drag.forces`; reads both scratches, adds `−F·h` to the accumulators) |
| Constraints, Integrate, Sensors | unchanged; the airspeed sensor joins Sensors (§3.5) |
| Publish | **`airflow.publish`**: requested points, the per-body aero breakdown, the render volume (§5) |

`rotor.forces` and `drag.forces` write their per-element outputs whether or not airflow is present. That is a store to scratch, so no golden moves, on Core's four conditions: the outputs are **unregistered** scratch (a field added to `RotorRow` or `DragBodyRow` would change the element size, the schema hash and every golden; `state_digest` folds every registered array, maps included); the CPU values and their fp32 order do not change (each pass stores a value it already computes, and `drag.forces` still runs after `rotor.forces`; on the GPU each kernel gains one store and one binding, and `ctest -L gpu` shows no band moved); no module version bump; and the scratch kind exists first. On Vulkan scratch is not read back, so the HUD's aero breakdown is a Publish copy. The HUD reads the same outputs (§6). The airflow module may not write another module's array: stage 4 refuses an optional write (`:123`), so it reads and accumulates into its own rows.

## 2. Boundaries

**Owner:** Physics.

**Static solids from the world SDF.** The world SDF (`world/sdf.hpp`, evaluated on the GPU by `shaders/sdf_eval.slang`) gives `φ` at each cell centre and face centre. A face's open fraction is `θ = clamp(0.5 + φ_face / dx, 0, 1)`: a one-cell linear ramp. The projection uses `θ` as face weights, the variational form of Batty, Bertails and Bridson (2007), which enforces `u·n = u_solid·n` in a weighted sense without cut-cell geometry. Ground planes, walls, heightfields and CSG come through the same SDF program, so the ground is not a special case. `φ` is evaluated once for a fixed box and again only on a recentre (about 8.4 M SDF evaluations at 128³, estimated under 1 ms for worlds of tens of nodes *(unconfirmed)*).

**Slip.** The real-time tier constrains only the normal velocity (free slip). A 2.5 cm cell cannot resolve a boundary layer. The LBM tier's bounce-back gives no-slip.

**Advection near solids.** Velocity is extrapolated two layers into solid cells by a fixed number of sweeps before advection, so a backtrace that lands in a solid reads a plausible value. Dye is clamped to zero inside solids.

**Domain faces.**

| Face | Velocity | Pressure |
|---|---|---|
| Inflow (`ambient · n < 0`) | Dirichlet: the ambient velocity | Neumann |
| Outflow (otherwise) | Zero normal gradient, with backflow clamped to the ambient | Dirichlet `p = 0` |
| A face inside a solid | As any solid face | — |

In still air every face is open with `p = 0`, so the wake leaves the box. The face classification follows the gust per fluid step. The LBM tier uses equilibrium inflow at the ambient (or, nested, at the outer grid's velocity) and zero-gradient outflow.

**Moving bodies.** In the real-time tier, vehicles are **porous**: they reach the air only through momentum sources (§3), never as solids. A quad's body blocks little of its rotors' wakes. A solid moving body (a body-frame SDF in its model, faces inside it taking `v + ω × r`, and the pressure force integrated over its surface in place of its drag element) belongs to the LBM tier and to large bodies later (§8 Q4). Treating a body as both solid and dragged would count its drag twice.

## 3. Coupling

**Owner:** Physics.

### 3.1 Rotors as actuator disks

Each rotor's thrust and shaft torque stay exactly what `rotor.forces` computes (`rotor.hpp` §§1–6). The air receives the opposite:
- **Thrust:** an impulse `−J`, along the thrust axis, spread over the disc.
- **Swirl:** an angular impulse equal to `Σ Q·h` about the axis, in the rotor's spin direction. The airframe already receives the reaction `−spin·Q` (`rotor.hpp` §6).

**Distribution.** The disc is a fixed set of M Lagrangian points (default 48 on concentric rings, area-weighted, weights summing to 1). Each point is spread with Peskin's 4-point kernel, which is a partition of unity per axis. The momentum injected is therefore `J` to fp32 rounding, without any reduction, and a tilted disc needs no special case. The swirl force at a point is tangential, scaled so that `Σ r_i |f_i| = Q`. The disc sits at the impulse-weighted hub position over the interval, with the latest axis.

**Deterministic gather.** One thread per face in the union box of the sources' supports. It loops over sources in slot order and their points in index order, and adds in that order. There are no atomics. With 4 vehicles × 4 rotors × 48 points the loop is about 800 support checks per face over roughly 50 k faces, under 0.1 ms *(estimate)*.

### 3.2 Inflow without double counting

Momentum theory's `f_inflow` already contains the rotor's own induced velocity. If `v_axial` read the grid at the hub, it would contain that induced velocity a second time. Three options:

| Option | How | Fits |
|---|---|---|
| **A. One-way** | The rotor keeps reading the whole-world wind (Dryden). The grid receives the sources | Milestone 1. Nothing can double-count |
| **B. Probe and subtract** | `v_axial` reads the grid averaged over 8 probes on a ring at `s = −2R` on the inflow side, minus this rotor's own analytic induced velocity there. On the axis that is `v_i (1 + s/√(s² + R²))` = 0.106 `v_i` at `s = −2R` (`rotor_wake.hpp` §1). The correction is small, and the grid's own resolved self-induction there is small too, so the error is second-order. Other rotors' wakes, the ground's outwash and gusts come through. `f_ground` stays: a disc 4–10 cells across does not resolve the image effect | The real-time tier's two-way coupling |
| **C. Grid-resolved inflow** | `f_inflow = f_ground = 1`. The disc-averaged grid velocity sets the inflow, through the model's own closure inverted (`U_d = V_c + v_h λ(V_c/v_h)` solved for `V_c`) | The LBM tier, when a disc spans 10 cells or more |

Recommendation: A, then B, with C for LBM (§8 Q3). B makes the on-axis wake profile read by stepping. Under `PHY-3` that formula must then carry a grade: a CPU implementation with a golden (the airflow golden covers it) and a GPU twin. It is a few lines, not the whole `rotor_wake_velocity`.

### 3.3 Drag elements

`drag.forces` keeps its law (`forces.hpp`: quadratic or componentwise). Inside a region, its `field.wind` sample is the grid velocity at the body's position, interpolated with the same Peskin kernel used to spread the reaction. Because interpolation and spreading are adjoint, the coupling neither creates nor destroys energy. The reaction `−F` (world frame) is accumulated as an impulse and spread at the body's position. Each element's torque stays on the body; the air receives only the force.

**Known bias.** A point force induces a velocity at its own position, which reduces `v_rel` and so the drag. For the sandbox quad at 10 m/s (drag about 1 N, spread over about (4 dx)³), the self-induced velocity is of order 1 m/s *(estimate)*. Physics measures it in M4 and, if it matters, samples a shell around the body instead of its centre. Recorded here, not hidden.

### 3.4 Beside today's tiers

- **T0 drag** is unchanged. Outside a region it reads Dryden as today. Inside one it reads local air (one-way), and with `airflow.accumulate` it pushes back, which is T3 in the engine model's terms. T3 here is the airflow module's presence, not a role: no pass of drag changes (`module-api-design.md` §7 keeps roles for tiers that change passes, such as T1 and T2).
- **The rotor element** is unchanged without airflow, bit for bit, because its read of the probes is optional (stage 4 Task 6). That holds on the CPU; on the GPU it needs stage 6's empty binding and a recipe switch. With airflow it reads the probes (option B).
- **The analytic wake** (`rotor_wake.hpp`) stays best-effort and visual-only for worlds without airflow, including the CPU drone box's slice. Worlds with airflow draw the solved field. Its on-axis profile becomes graded if option B lands (§3.2).
- **Disc drag** stays `DragBody`'s job (`rotor.hpp`, "THE DISC'S BLUFF-BODY DRAG IS NOT ITS JOB").

### 3.5 Sensors

**IMU.** No change. Coupled aero forces reach `force_acc`, so the specific force Integrate captures includes them; turbulence shows up as vibration. `PHY-7` (contact specific force) is independent of this spec.

**A new airspeed sensor.** It has the shape `03-sensors.md` names for the next sensors (a row, a kind tag, a seeded stream, a ring), and is added under `sensors/kinds.hpp`'s rule: row, sample, stream and poll path first, then the tag.
- **Mount:** a body slot, a body-frame position `r` and an orientation. The probe axis is the mount's +X.
- **Reads:** `wind` and `density` at its mount point (a per-sensor sample point). Without airflow it reads Dryden, so it is useful before the solver lands. Inside a region it reads the local air, the vehicle's own wake included, as a real probe does: a probe meant to read the free stream is mounted forward on a boom, clear of the discs. The HUD's airspeed does not depend on it (§6).
- **Model:** the relative wind `w = u_air(p) − (v + ω × r)`. True airspeed along the probe is `V = max(0, −w·a)`; the differential pressure is `q = ½ ρ V²`; indicated airspeed is `sqrt(2q / ρ₀)` with ρ₀ = 1.225 kg/m³. Optionally the three-axis `w` in the mount frame, for angle of attack and sideslip.
- **Noise:** white noise on `q` (Pa), plus a bias random walk, from a per-sensor seeded stream. Noise on pressure makes low-speed readings noisy, as a real pitot's are.
- **Placement:** the Sensors phase. It reads the Fields-phase sample, taken at the substep's starting positions. That instant is documented, not corrected.
- **Grade:** CPU reference with a golden; Vulkan banded through `CORE-3`.
- **API:** `add_airspeed_sensor` and `poll_airspeed`, beside the IMU and GNSS calls.

## 4. GPU

**Owner:** Core (backend, memory, residency, determinism). Physics (kernels). Test/Docs (gates and bands, §4.6).

### 4.1 Kernels

| Kernel | Grid | Per fluid step |
|---|---|---|
| `airflow_recentre` (copy by integer shift; early exit at shift 0) | per face | 1–2 |
| `airflow_solid` (`φ`, apertures; on recentre only) | per face | 0–1 |
| `airflow_sources` (gather, §3.1) | union of the sources' supports | 1 |
| `airflow_boundary`, `airflow_extrapolate` | per face | 3 |
| `airflow_advect` (forward), `airflow_advect_back`, `airflow_maccormack` × (velocity, dye) | per face / cell | 6 |
| `airflow_vorticity`, `airflow_confine` (if `ε > 0`) | per cell / face | 0–2 |
| `airflow_divergence`, `airflow_project` | per cell / face | 2 |
| `mg_smooth_red`, `mg_smooth_black`, `mg_restrict` (with residual), `mg_prolong`, `mg_coarse` | per level | ≈ 51 per V-cycle |
| `airflow_sample` | per point | every substep |
| `airflow_accumulate` | per element | every substep |
| `airflow_publish` (render volume, points) | per cell / point | at frame request |
| LBM: `lbm_stream_collide` (fused, esoteric pull), `lbm_boundary`, `lbm_macro`, `lbm_nest` | per cell | per LBM step |

About 115 dispatches per fluid step with `n_V` = 2, each followed by the barrier `CORE-2` requires. Velocity components, multigrid levels and scratch sets are packed into a few buffers with offsets, to keep the binding count small; the offsets are shape data, so they live in a small derived per-world buffer uploaded once at `create()`, like `grid_params`, not in push constants.

**Device limits, none checked today** (Core; stage 7, though the refusals may land earlier). Each is checked at `create()` and refused by name (`L6`), with `GpuStateMirrorTest.AShapeTheMirrorCannotSizeIsRefusedByName` as the test template:
- `maxStorageBufferRange` (128 MiB guaranteed). A binding wider than it is a valid-usage violation, not a reported error. Batching matters: 16 worlds of one 128³ face component already exceed 128 MiB in one buffer.
- `maxMemoryAllocationSize`: each buffer is its own allocation.
- The storage-buffer descriptor counts: Vulkan guarantees 4 per stage and 24 per set, and the set already declares 26 bindings (27 with `PHY-7`), unchecked.
- **3D dispatch.** Every dispatch is 1D today, and Vulkan guarantees only 65,535 groups on X: at workgroup 32 a 128³ cell grid needs 65,536 groups and a face grid 66,048. Airflow's kernels need 3D dispatch, a recorder change.
- **The LBM distributions** are one 216 MiB buffer (Core's call), refused by name on a device whose limit is lower; splitting per device would make the kernels' indexing depend on the device. `maxStorageBufferRange` joins the device report. M6 work.
- **16-bit and 8-bit storage are optional features** (fp16 apertures, the render volume's half floats, u8 cell flags). `CORE-5` admits any Vulkan 1.1 device, so they are packed into `uint32` words, with the rounding done in integer arithmetic so it is exact on both backends, or refused by name; making the features required would narrow `CORE-5`.

### 4.2 Cost per fluid step at 128³ (estimates)

Assumptions *(unconfirmed)*: 80% of the 3060 Ti's 448 GB/s (about 358 GB/s), and 4 µs per dispatch plus barrier.

| Work | Traffic | Estimate |
|---|---|---|
| Advect velocity (MacCormack, `n_trace` ≤ 3) | ≈ 180 MiB | 0.5–0.8 ms |
| Advect dye (optional) | ≈ 120 MiB | 0.3 ms |
| Sources, boundaries, extrapolation | small | 0.1–0.2 ms |
| Vorticity confinement (optional) | ≈ 72 MiB | 0.2 ms |
| Divergence and projection | ≈ 100 MiB | 0.3 ms |
| One red-black sweep at 128³ | ≈ 36 MiB | 0.1 ms |
| One V(2,2) cycle, all levels | ≈ 217 MiB, 51 dispatches | ≈ 0.85 ms |
| Pressure, `n_V` = 2 | | ≈ 1.7 ms |
| **Fluid step, full** | | **≈ 2.6–3.3 ms** |
| **Fluid step, lean** (`n_V` = 1, no dye, no confinement) | | **≈ 1.6–2.0 ms** |
| For comparison: 40 Jacobi iterations | ≈ 1.4 GiB | ≈ 4 ms, and the low modes barely move |

Real time needs the fluid at no more than about half of `h_f`, leaving the rest for bodies and drawing. At `h_f` = 4 ms that is 2 ms: the lean step fits, and the full step does not. At `h_f` = 8 ms both fit, at CFL 3.2–9.6 (§1.4). The spike measures this before any configuration is chosen (§8 Q2). `Simulation::vulkan_pass_durations_ns()` (`simulation.hpp:696`) gives one number per pass, summed over the substeps of the most recent step: with `j` > 1 the fluid passes read about 0 after a non-fluid step, so it must say which recording ran. Per-V-cycle numbers come from the spike, not this API.

### 4.3 Memory at 128³

128³ = 2,097,152 cells. One fp32 scalar is 8.00 MiB; one face array (129 × 128 × 128) is 8.06 MiB.

**Real-time tier**

| Array | Floats/cell | Kind | MiB |
|---|---|---|---|
| Velocity u, v, w (faces) | 3 | state | 24.2 |
| Pressure (warm start) | 1 | state | 8.0 |
| Dye (optional) | 1 | state | 8.0 |
| Accumulators, region row | — | state | < 0.1 |
| Velocity scratch: forward and backward MacCormack sets | 6 | scratch | 48.4 |
| Dye scratch | 2 | scratch | 16.0 |
| Force field (faces) | 3 | scratch | 24.2 |
| Vorticity (cells) | 3 | scratch | 24.0 |
| Divergence / right-hand side | 1 | scratch | 8.0 |
| Residual | 1 | scratch | 8.0 |
| Multigrid 64³ … 4³: p, rhs, residual | 3 × 1/7 | scratch | 3.4 |
| Face apertures (fp16), finest and coarse | 1.5 + | derived | 13.8 |
| Static SDF distance at cells | 1 | derived | 8.0 |
| **Total** | ≈ 97 B/cell | | **≈ 194** |
| Later, moving solids: solid face velocity | 3 | scratch | +24.2 |

Aliasing vorticity with the backward MacCormack set would save 24 MiB; this table does not count it. **It counts device-local bytes only.** Today every walked and derived buffer also has an equal staging buffer, about 194 MiB more of host-visible memory at 128³, unless scratch and derived buffers become device-only (stage 7); the host arena holds its own copy of the state; and per-cell `slot_to_world` maps would double the state's bytes, which the grid-row form avoids (Core).

**High-fidelity tier**

| Array | D3Q19 | D3Q27 | Kind |
|---|---|---|---|
| Distributions, one copy (esoteric pull) | 152.0 | 216.0 | state |
| Density and velocity (published, sampled) | 32.0 | 32.0 | scratch |
| Force field (Guo forcing) | 24.0 | 24.0 | scratch |
| Cell flags (u8) | 2.0 | 2.0 | derived |
| **Total, one copy** | **210** | **274** | |
| Two copies (A/B streaming) instead | 362 | 490 | |
| Later: wall distance per link (u8), for interpolated bounce-back | +36 | +52 | derived |

**Rendering**

| Buffer | MiB | Where |
|---|---|---|
| Render volume V, RGBA16F (`u, v, w`, dye), and S, RG16F (pressure gauge, `|ω|`): 12 B/cell (§5) | 24 | VRAM (Vulkan) |
| Its staging copy | 24 | host memory *(unconfirmed: depends on the memory type the mirror picks)* |
| GL 3D textures, V and S | 24 | VRAM (GL) |
| Streamlines: 4,096 seeds × 128 points × 16 B | 8 | VRAM |
| Smoke particles, 262,144 × 32 B (if used) | 8 | VRAM |
| **VRAM total** | **≈ 64** | |

**Against 8 GiB.** Proposed cap for airflow: 4 GiB. That leaves about 4 GiB for Windows and the desktop, the driver, the GL editor and the engine's own buffers *(the reserve is unmeasured; Core measures it with `VK_EXT_memory_budget`)*.

| Configuration | Real-time | LBM (D3Q27, one copy) | Render | Total |
|---|---|---|---|---|
| **128³ both tiers** | 194 | 274 | 64 | **≈ 532 MiB** |
| 192³ both | 655 | 925 | ≈ 180 | ≈ 1.7 GiB |
| 256³ real-time + 192³ LBM | 1,552 | 925 | ≈ 400 | ≈ 2.8 GiB |
| 256³ both, the volume decimated by 2 | 1,552 | 2,192 | ≈ 64 | ≈ 3.7 GiB (at full resolution, ≈ 400 and ≈ 4.0 GiB: over the cap) |

Batching (`L8`): the module set belongs to the simulation and array capacity is uniform, so **every world carries the largest region's arrays**, with or without a region of its own. 32 worlds at 64³ (real-time tier, 24 MiB each) use 776 MiB. `create()` refuses a set whose declared footprint exceeds the budget, naming the region (`L6`).

**Staging memory** (Core, from the code and the Vulkan spec; **not measured**): `find_memory_type()` takes the first `HOST_VISIBLE | HOST_COHERENT` type, and Vulkan orders a type whose flags are a strict subset of another's first, so on a discrete card staging lands in system memory, not the BAR heap. Core records the chosen type and heap per buffer kind as a mirror diagnostic, tests that staging is not `DEVICE_LOCAL` on a discrete device, and adds a device-only buffer kind in stage 7. Only resident state (for snapshot syncs) and the Publish buffers keep staging.

### 4.4 Device residency

The grid lives on the device (stage 7; a module-API spec amendment, §8 Q12). On Vulkan:
- each resident array carries a **residency flag** and its own dirty flag, outside the per-step upload and readback (whose one dirty flag any host write sets today);
- it is uploaded at `create()`, at `restore()` and after an explicit host write;
- it is synced, or the call refuses (`L6`), on every host read path: `snapshot()` (const today), `arenas()`, `module_rows<T>`, and `testing::state_digest`, which reads `arenas()` directly, so the sync sits in `Simulation`, not in the digest;
- the per-step readback (once per `step(n)` call, not per step) stays the small walk: bodies, rows, rings, the region row and the accumulators;
- **Publish runs as its own command buffer**, submitted on request between steps (Core's proposal). The step's recording does not change, a recorded chain need not see a frame request, and `L5` holds by construction. At frame rate the host reads only the Publish buffers (§5, §6).

Which state is which: velocity, pressure and dye are walked, snapshotted and resident; the region row and the accumulators are walked and small, and stay in the per-step readback. Scratch (advection buffers, multigrid levels, the force field, vorticity) is not walked, so a restored run restarts it: sound only if every fluid step writes each one fully before reading it, coarse-level corrections included, which the Vulkan restore-resume test proves. Derived buffers (apertures, the SDF distance) are rebuilt after `create()` and `restore()` from the configuration and the restored region row.

**Snapshots** grow by about 40 MiB per world at 128³ (velocity, pressure, dye) in the grid-row form, and 0.6 MiB at 32³; per-cell maps would double it. **Every world in the set pays**, since capacity is uniform. On Vulkan, `snapshot()` first reads the resident arrays back: about 40 MiB per world over PCIe, plus a copy out of staging. A simulation without airflow keeps its v3 blob byte for byte (pinned on `core/module-state` by `ModuleState.TheStandardWalkIsTodaysTwentyTwoEntries` and its neighbours). If stage 7 adds or renames a standard pass, the standard identity changes and stored blobs are refused once, as after stage 3; Core will try to avoid it.

**`sample_medium`** is hard-wired to Dryden today (it builds a `DrydenMedium` from the world's row); routing by region is stage 7's work. Outside every region it keeps today's answer on both backends. On Vulkan, inside a region, it returns `unavailable`, naming the region and pointing to Publish points (`L6`): a hidden 40 MiB readback inside a point query would be a performance trap. On the CPU backend it answers from host state, which stays authoritative there.

### 4.5 Determinism per device

- **No atomics.** Every scatter is written as a gather in a fixed order (sources by slot, points by index).
- **No order-dependent reductions in the step.** The pressure solve runs a fixed number of cycles and never tests a residual. The only reductions are exact maxima (the CFL monitor) and diagnostics that stepping never reads, computed with a fixed tree.
- **Per-cell kernels.** Advection, red-black smoothing, sources and projection compute one cell's value from its neighbours only, so they are invariant to workgroup size (the existing `{32, 64, 128}` tests extend to airflow). The 4³ coarse solve runs in one workgroup with barriers between sweeps.
- **The recording choice is a function of the tick** (`tick mod j`), never of state.
- **Operation order is the contract** (`engine D2`). The CPU twin writes every stencil in the GPU's order, uses the lerp form, uses `fp32_math` for any transcendental (`sqrt` is allowed, `SR-14`), and allows no contraction.
- **Expected CPU↔GPU differences:** NVIDIA's flushed denormals (decaying flow far from sources produces subnormals) fall under `TD-14`'s absolute term near zero. Its division and square root are 1 ulp off on **normal** operands too (the denorm report's finding 4): that is the relative term, everywhere, and chaos amplifies it (Core). The cheap mitigation is adopted: reciprocals such as `1/dx` and `h_f/ρ` are computed on the host at `create()` and multiplied on both backends. Determinism holds per device and driver (`CORE-5`), pinned in-process: two Vulkan runs bit-identical, the workgroup sizes, and a Vulkan snapshot-restore-resume. That test (banded-parity T7) is **not built yet**; airflow's version must hold the resident arrays and rebuild the derived buffers.

### 4.6 The CPU reference, goldens and bands

**Owner:** Physics (content), Test/Docs (harness and gate).

| Size | Use | CPU per fluid step (est., single-threaded *(unconfirmed that CPU passes stay single-threaded)*) |
|---|---|---|
| 16³ | Unit tests: projection, free stream, multigrid convergence factor, momentum budget | 1–5 ms |
| 32³ | Parity against the GPU, and the corpus golden | 10–40 ms |
| 64³ | Report-only measurements | 0.1–0.3 s |
| 128³ | Once per device of record, report-only (§8 Q9): an opt-in test (an environment variable, with a named skip, like `Fp32Exp`'s full sweep), never a gate test. Its record follows `test-docs/01-verification.md`'s band records: the device block, the scenario line with the control and `T_p`, and per statistic the 8-run spread, the GPU gap and the band | 1–2 s |

**Closed-form and invariant tests** (in double where the test derives a value, `physics/04-verification.md`):
- a uniform flow with no sources and no solids is reproduced bit for bit, on both backends;
- after projection, the discrete divergence is below a pinned bound for the declared `n_V`;
- a gradient body force produces no velocity;
- injected momentum equals `−Σ J` to fp32 rounding (in a closed box with no outflow);
- an actuator disc in still air: far-wake speed against momentum theory's `2 v_i`, within a band measured at 128³;
- LBM: Poiseuille flow (second order), Taylor-Green decay against its closed form in `ν`, and the drag on a sphere at Re 20–100 against published correlations.

**The golden:** `airflow_hover` at 32³ on the CPU. A quad hovers in a fixed 1.6 m box with a ground plane, for 250 fluid steps. The digest is final only when a fresh `-NoSeed` Docker gcc leg reproduces it, with the provenance block in the scenario file (`TD-1`, `TD-12`). It must not depend on the thread count (assumption 6).

**Gate time** (Test/Docs, 2026-10-05). The gate runs debug as well as release, about 11× slower (at `86f1cb6` the non-gpu suite took about 41 s in release and 468 s in debug), and this box's load moves timings another 3–4×. So:
- each new CPU test's time is measured in both presets before review and stated in the merge note (`TD-8`);
- the airflow CPU tests together add at most about 60 s to the debug gate: one CPU run is shared between the golden and the parity test where possible, and the unit tests stay at 16³;
- if the live 8-run spread breaks that budget, the statistics' spread is pinned as a measured band with its `L4` record, re-measured when the scenario or the solver changes (`TD-2`), instead of being recomputed every gate;
- a test over 60 s in debug gets a per-test timeout override with its reason beside it (`test-docs/01-verification.md`, "Timeouts"), as the last resort, not the plan.

**Bands (`TD-14`).** At 32³, element bands cover velocity at the probes, the wind sample at each body, body pose and rates, and every cell of the published volume. They hold up to the scenario's horizon `T_p`: the longest tested horizon at which the one-ulp CPU control (one face's velocity nudged by one ulp) stays under 1e-5 of the scene's scale (the scale is `v_h`, so 5e-5 m/s for the sandbox quad). Past `T_p`:
- **invariants,** exact: no NaN or infinity; divergence under its bound; the momentum budget; the dye non-negative;
- **statistics,** banded at 4× the largest deviation across eight one-ulp-perturbed CPU runs: the mean downwash on each rotor's axis at `s = 2R`, the time-averaged thrust, the RMS speed in the box, the kinetic energy, and the dye's centroid.

The target is 1e-4 relative. Above 1e-3 is a grade change (`TD-2`), never a wider band. As for SPH (`SL8`): if no band exists, the GPU path's grade drops; the band is not widened. Today's `TD-14` rules apply as written (`test-docs/01-verification.md`): no band tighter than the floor's halves (`compare_arrays` refuses one), records printing A and R to three figures, and a row's own near-zero cutoff only where it is derived from the quantity's absolute resolution and its record carries the measurement that fails at the default (as `PHY-7`'s specific force does). Airflow's quantities start at the default 1e-3; a declared cutoff, if the spike's measurements call for one, is argued then.

## 5. Rendering

**Owner:** Rendering.

The views read only Publish (`L5`, the engine model). `airflow.publish` writes a render volume at the region's resolution, or decimated by 2, cell-centred (it interpolates off the MAC faces), as two textures: **V**, RGBA16F = (`u, v, w`, dye), and **S**, RG16F = (pressure gauge, vorticity magnitude). The velocity is a vector because the streamlines and tracers need it; speed is `|uvw|` in the shader. That is 12 B/cell: 24 MiB at 128³, 3 MiB decimated. Each frame carries a small header: the world box (origin, cell size, dims), since a box can recentre (§8 Q1), and a finite min and max per channel for the transfer functions. The f32 → f16 conversion rounds to nearest even and keeps subnormals; a non-finite sample is flagged no-data and drawn transparent (`L6`), never as a value, as the field layer does. Publish runs only on frames that request it, never at substep rate. (Rendering, 2026-10-05.)

**Volume rendering (GL, the editor's path, `RND-4`).**
- The volume reaches GL by readback: 24 MiB per frame, about 1 ms over PCIe 4.0 *(estimate)*, then uploaded to two 3D textures.
- A fragment ray-march uses front-to-back compositing with early exit, at 1–2 samples per cell. A transfer function per channel: speed, pressure (diverging) and vorticity. Dye is absorption plus one constant in-scatter colour, so smoke reads as smoke, not ink; no self-shadowing at first. The march reads the scene's depth, so solid geometry hides the air behind it: `GlRenderer` draws into the caller's framebuffer, whose depth may be a renderbuffer, so Rendering redraws depth only into an internal texture. Estimated 2–4 ms at 1080p *(unmeasured; Rendering measures it in M3, against 2–4 ms measured for small GL CSG marches on the same card)*.
- Vulkan-GL interop (`VK_KHR_external_memory_win32` with `GL_EXT_memory_object_win32` and semaphores) removes the copy. It comes only if the readback measures too slow (§8 Q7).

**Streamlines.** Seeds come as a rake, a grid or rings around each rotor. They are integrated by RK2 through the published velocity and drawn as lines coloured by speed. Display only. The CPU integrator comes first: it is the reference and the small-seed path. A GL 4.3 compute shader follows, banded against it.

**Smoke.**
- **Dye** is a passive scalar the solver advects (state, optional, §1.5). It is emitted at configurable sources: rotor tips, a wand, a ground patch. It is rendered as absorption in the same ray-march.
- **Tracer particles** (optional, later) are advected through the published field on the device, render-only, and drawn as points.

**Grades (`RND-3`).** A CPU volume ray-march at 32³ is the reference, with a frame golden. The golden renders a synthetic analytic volume (a vortex ring with a dye blob) through the Publish format, so it pins rendering alone and does not move with the solver; the gcc cross-check is `TD-12`. CPU trilinear sampling is cell-centred and edge-clamped. GL is best-effort, with a colour band measured against the CPU on the same volume, as the field layer has (`GpuGlRenderer.FieldLayerMatchesTheCpuWithinItsMeasuredBand`); NVIDIA's hardware trilinear (8-bit weights) sits inside it. There is no end-to-end frame golden from `airflow_hover`: the solver's golden pins the solver and this one pins rendering, so neither moves for the other's reason (Physics, 2026-10-05).

**The Vulkan raster path needs nothing for this spec.** It is paused at background only (`rendering/07-status.md`). When it resumes (`RND-4`), a volume pass is a compute ray-march over the published buffer on the same device, with no copy. Airflow is a reason to bring that forward, and that decision is the user's (§8 Q7).

**The 2D slice** stays as the CPU backend's view and as the drone box's view without airflow. The airflow views use the volume, as the user asked.

## 6. Interface

**Owner:** Interface.

**The HUD, per vehicle:**

| Item | Source |
|---|---|
| Airspeed, α and β | The **free-stream** relative wind the vehicle's aero actually used, so the HUD shows what the force laws saw: the mean over the vehicle's rotors of the air their inflow read, minus `v_body`. That is Dryden in M3 (option A) and option B's probe-and-subtract value from M4 (§3.2). `aero_breakdown` carries each element's air-relative velocity as used. **Not** a Publish point at the body centre: on `hover.scene`'s quad the discs' inner edges are 5 cm from the centre, inside the Peskin kernel's support, so that point reads the vehicle's own downwash, about `v_h` ≈ 4.3 m/s at hover *(estimate)* (Interface, 2026-10-05) |
| Angle of attack and sideslip: **the reference convention** | Spade's convention is the reference, and Kat adopts it from `consumers.md` (Kat had none recorded; the lead, 2026-10-05). Exactly:<br>• `v` is the air-relative velocity of the body in body axes (nose +X, up +Y, right +Z): `v = Rᵀ(v_body − u_air)` (axes: `sandbox/drone_sim.hpp:116-117`, pitch about Z, roll about X; right = forward × up);<br>• `α = atan2(−v_y, v_x)`, positive nose-up against the relative wind;<br>• `β = asin(v_z / |v|)`, positive with the relative wind from the right;<br>• radians in the API (the HUD shows degrees).<br>**Low airspeed:** both angles are undefined at `|v|` = 0, so `α = β = 0` when `|v|` < 0.1 m/s, and the HUD shows "—" there. Why 0.1 m/s: below it the angles describe the free-stream estimate's own error, not the flight (that estimate is good to centimetres per second at best, so at 0.1 m/s an angle can already be off by a tenth of a radian); it is under any speed at which an angle means something to control or aero (a hovering quad's drift); and both formulas are still well conditioned there in fp32, so nothing numerical sets the threshold. In steady level forward flight in still air, `α` equals the pitch |
| Local air at the body | A Publish point at the body centre, labelled as such: it includes the vehicle's own wake |
| The airspeed sensor, if one is mounted | Its reading (§3.5), labelled as the sensor's |
| Ground speed, and the wind at the body | Body state; the Publish point |
| Aero force and moment, total and split (rotors, drag), body and world frame; thrust per rotor | The per-element outputs of `rotor.forces` and `drag.forces` (§1.8), copied per body in Publish. A new public read, for example `aero_breakdown(VehicleRef)` (Core and Physics name it), carrying each element's air-relative velocity as used. It and the region diagnostics reach the installed headers (`SL2b`) |
| "Outside the airflow region" | The per-body flag (§1.3) |
| Region diagnostics: CFL monitor, divergence residual, GPU ms per fluid step, real-time ratio (simulated over wall seconds) | Publish diagnostics; `vulkan_pass_durations_ns()` |
| Device | `vulkan_device_report()` (`CORE-5`) |

**Sandbox.** A new scene, `airflow`, on Vulkan:
- **The vehicle:** a free-flying quad, held by a host-side position loop wrapped around the existing attitude controller (`sandbox/drone_sim.hpp`). It needs no behavior: drone_sim's CPU behaviors are only the test stand (`drone_sim.hpp:5-13`), and the controller and mixer are host-side code that writes `set_rotor_commands` (`simulation.hpp:998`), which has no Vulkan restriction. So it runs on Vulkan before stage 6's lock. Once the lock lands, the drone box itself can move to Vulkan and gain the airflow views.
- **The loop's rate:** DroneSim runs the controller once per engine step (`drone_sim.hpp:307-317`), which on Vulkan is one state readback and one submission per step. M1 and M3 measure that round trip; if it eats the real-time budget, the attitude loop runs every `k` steps with retuned gains, `k` chosen from the measurement, and Interface tests that the loop holds position at that `k`.
- **The world:** a ground plane, so ground effect and outwash show.
- **Views:** volume (speed, pressure, vorticity), streamlines, smoke, and the HUD.
- **Panel:** box size (2–6 m), fixed or following, `j`, `n_V`, `ε`, dye on or off, emitters. These are the region's fields in world file v3 (§7), so the panel edits the world document and rebuilds through the editor's single rebuild path (the editor plan's `WorldEdit` and `EditorRun::rebuild` with carry), not a second path. The vehicles' poses and rates carry; the fluid field carries only when the grid is unchanged, and a box-size or `dx` change restarts it at the ambient wind, which the panel says (`INT-2`).
- **Files:** `assets/scenes/airflow.scene.yaml` and `assets/worlds/airflow.world.yaml`, once world file v3 carries the region. Until then the region's configuration is code in the airflow session, as `sandbox/builtin_records.hpp` is, deleted with v3.

**Live smoke.** A third main function, `airflow`, joins `kMains` (`sandbox/live_tour.cpp:91`), with its steps in the ledger (`:72`). Every check can fail (Interface, 2026-10-05):

| Step | Check |
|---|---|
| `airflow.open` | The device report is journaled and the backend is Vulkan. No device is named, so the smoke means the same on another machine (`L4` records are per device) |
| `airflow.volume_speed`, `volume_pressure`, `volume_vorticity` | No cell is non-finite; with the channel on, the frame differs from the volume-off frame over at least 5% of the region's screen footprint |
| `airflow.downwash` | At hover, the Publish point 2R below each hub reads downward at more than 1.5 `v_h`; `v_h` is computed from the scene's model and density, and both are journaled with the reading |
| `airflow.streamlines` | Seeds 0.5R below each hub move down by more than half of `v_h` × the integration time |
| `airflow.smoke` | The dye outside the emitter's cells rises from zero and keeps rising over 1 s |
| `airflow.hud` | From the free-stream source above: in still air at hover, airspeed reads under 0.5 m/s; with 5 m/s of wind, within 0.5 m/s of 5 |
| `airflow.wind` | With 5 m/s of wind, on a plane 2R below the hub, the strongest downwash lies downwind of the hub, and further downwind than in still air |
| `airflow.ground` | At 0.3 m altitude, a point 2R out from each hub and 5 cm above the ground reads an outward horizontal velocity above 0.3 `v_h`; the same point at 3 m altitude reads under half of that. (`k` = 0.3 is Physics' provisional value, re-set from M4's measurement) |
| `airflow.forward` | In steady level forward flight above 2 m/s in still air, `α` equals the pitch within 2°, `|β|` < 2°, and the drag force's along-track component opposes the velocity |
| `airflow.timing` | GPU ms per fluid step and the real-time ratio are journaled and checked finite and positive (the ledger makes a step with no checks an anomaly); targets stay ungated until the spike sets them |

The tour's shared video must stay under `kSendableBytes` (30 MB). Volume frames compress worse; if the airflow main pushes it over, it writes its own file under the same checks, decided by measurement.

The 2D-block complaint is closed by the volume steps, "no airfield" by `downwash` and `wind`, and "no airspeed" by `hud`.

## 7. Kat and other consumers

**Owner:** Core (API), the lead (`consumers.md`), Physics (models).

Kat runs the CPU backend today *(per the lead's brief; not stated in this repository)*.
- **Opt-in.** `airflow` is an optional built-in, like the lock, and not in `standard_modules()`. The module set belongs to the simulation and is shared by every world (`L8`), so **a simulation** whose set lacks airflow registers nothing: its 22-entry walk, snapshot format v3, goldens and Kat's airframe hashes are unchanged (stage 4: "Optional modules register only when in the set"). Its configuration identity is unchanged through stage 4; if stage 7 adds or renames a standard pass, the identity changes and Kat's stored blobs are refused once (goldens hold). Core will try to avoid it but cannot promise it yet.
- **Rotors and drag are unchanged without airflow,** bit for bit on the CPU: the probe read is optional, and the per-element outputs are scratch. On the GPU this needs stage 6's empty binding and a recipe switch.
- **CPU cost:** 16³ costs about 1–5 ms per fluid step, and 32³ about 10–40 ms. At `h_f` = 4 ms, 32³ runs about 2.5–10× slower than real time *(estimates)*, and 128³ on a CPU is about 2 s per fluid step. Real time needs the GPU, but the CPU path is the reference (`L4`), the same code at any size. Kat leaves airflow off on the CPU, or uses 16³ for coarse wake effects in offline runs. Nothing refuses a large CPU grid (slow is not unavailable); the cost is a **query**, a cost estimate beside `vulkan_device_report()`, not a log line (§8 Q10).
- **The airspeed sensor** works without airflow (it reads Dryden), so Kat can adopt it at once.
- **API Kat would see:** the world file declares the airflow region (box, `dx`, cells, mode and tracked body, `j`, `n_V`, `u_design`, `ε`, dye, tier) under a world file schema after today's v2 (`core/07-status.md:17`), with the region's configuration hashed into the identity (stage 7, not built). New calls: `add_airspeed_sensor`, `poll_airspeed`, the aero breakdown read, and Publish point requests. `sample_medium` becomes position-dependent inside a region, and on Vulkan it is refused there (§4.4).
- **Snapshots** grow by about 40 MiB per world at 128³, for **every** world in a set with airflow, which matters for Kat's session ring. On Vulkan each snapshot adds a readback of the resident arrays (§4.4).
- **Determinism:** the CPU path is reference grade with a golden, so Kat's CPU replays stay bit-exact.
- **The lead adds a `consumers.md` entry** when the module merges. The α/β convention (§6: `v` in body axes, `α = atan2(−v_y, v_x)`, `β = asin(v_z/|v|)`, radians, both 0 below 0.1 m/s) goes into `consumers.md` when the HUD and the aero breakdown land, and Kat adopts it from there.

## 8. Open questions for Cameron

**Q1. The domain.** (A) A fixed box per world. (B) A box that follows one vehicle, recentring by whole cells. (C) Both, as a mode. (D) One box per vehicle. **Recommend C.** It is the same code with recentring switched off, and the CPU reference and parity tests need the fixed mode anyway. D comes later, for spread-out formations.

**Q2. The fluid step against the physics step.** (A) Every substep (1 ms): the tightest coupling, and likely not real time at 128³. (B) Once per step (2 ms in the sandbox). (C) Every `j` steps, fixed at `create()`, with `j` set from the spike's measurement (about 4 ms). (D) Adaptive CFL. **Recommend C.** D breaks the fixed recording and lets the CPU and GPU take different step counts. Core confirms C: the second recording is its recorder change in stage 7, at no per-step cost, and anything that changes the recorded dispatch list (`j`, `n_V`, the level count, confinement and dye on or off) is one value for the whole set, with mixed values refused at `create()`. A and B would need no second recording.

**Q3. Rotor two-way coupling.** (A) Staged: one-way first (rotors push air, read Dryden), then probe-and-subtract. (B) Probe-and-subtract from the start. (C) Grid-resolved inflow. **Recommend A,** with C kept for the LBM tier. The first milestone then cannot double-count, and B lands with its own measurement.

**Q4. Vehicles in the grid.** (A) Porous: momentum sources only. (B) Solid moving bodies from a body SDF. (C) A per tier: porous in the real-time tier, solid as an option in the LBM tier. **Recommend C.**

**Q5. The LBM lattice.** (A) D3Q19 with TRT and Smagorinsky: 210 MiB at 128³, the cheapest. (B) D3Q27, regularized, with Smagorinsky: 274 MiB, better isotropy in jets. (C) D3Q27 cumulant: the most stable at high Reynolds number, and the most code for both twins. **Recommend B.**

**Q6. The LBM tier's speed.** It cannot be real time at 128³ on this card (about 12–16× slower). (A) Accept slow motion: the sandbox shows the real-time ratio. (B) Only small nested regions, 64³, about 2–4× slower. (C) Both. (D) Defer LBM until the real-time tier ships. **Recommend C, sequenced as D.**

**Q7. How the volume reaches the screen.** (A) Readback into GL each frame. (B) Vulkan-GL interop. (C) Bring the Vulkan raster forward and ray-march on the device. **Recommend A,** with B only if the readback measures too slow. C changes `RND-4`'s order and is a separate decision. Rendering agrees (2026-10-05).

**Q8. Order against SPH.** `PHY-4` (signed) puts SPH right after the module API, and SPH closes the last v1 transfer row, which unblocks the v1 quarantine (`SL14a`). (A) Airflow first; SPH then reuses point sampling and regions. (B) SPH first. (C) In parallel, sharing the Core stage. **Recommend A.** It follows the 2026-10-05 ruling ("GPU asap"), and the shared Core work is built once. It amends `PHY-4`'s timing, so it needs the user's word. Core is neutral: it builds the shared work (point samples, regions, the scratch kind) once either way, in stage 7, tested with a CPU-only provider. Airflow first delays the v1 quarantine (`SL14a`), which SPH unblocks.

**Q9. The grade at 128³.** The gate can afford a CPU comparison at 32³, not at 128³. (A) Banded at both sizes: 32³ in the gate, plus a recorded 128³ statistics measurement per device of record (`L4`'s record, like T3's). (B) Banded at 32³, best-effort at 128³. (C) Gate at 64³ too. **Recommend A.** Test/Docs agrees (2026-10-05), with the 128³ measurement as an opt-in, report-only test (§4.6).

**Q10. Airflow on the CPU.** (A) Opt-in only, with the cost announced at `create()`. (B) A supported coarse CPU tier for Kat (16³–32³), with its own scenarios. (C) Refuse CPU grids above 32³. **Recommend A.** B can come when Kat asks for it. Core confirms A: the CPU reference makes airflow gradeable, and "announced" means a cost query, not a log line.

**Q11. Where module-API stage 7 sits** (Core's question). Stage 7 carries the grid's hashed configuration, residency, point samples, regions, two recordings, 3D dispatch, device-only buffers and the limit and budget checks (§1.7). (A) After stages 5 and 6, 4 → 5 → 6 → 7: Core's path to a first GPU airflow is about 2.5–3.5 working days after stage 4, and airflow ships graded through the shared binding pattern, with nothing reworked. (B) First, the §9 order this spec began with: about 1.5–2 days, roughly 1–1.5 days sooner, but stage 6 then reworks stage 7's binding pattern (about half a day), and airflow runs on Vulkan ungraded until stage 5. (C) A, with the device-limit refusals and the budget check pulled forward into stage 5 (they stand alone). **Recommend A** (Core's and Physics'). The estimates are Core's, from one day's measured pace (about 1–1.5 hours per test-first task), not promises. Physics' side, the spike and the real-time solver with its CPU reference and bands, is likely the longer pole (about 3–5 working days at the same pace, an estimate), so A probably costs nothing in time to a first airflow.

**Q12. The module-API spec amendment** (Core's question). Two rules the approved module-API spec does not cover: **device residency** (a resident array's authoritative copy is on the device, synced on every host read, which ends the arena's role as the one authoritative store) and **a hashed module configuration in the identity** (set-wide grid parameters in the configuration identity, per-world placement in `config_hash`, so a restore across grids is refused, `L2`). (A) Approve both now as principles; stage 7's design section returns for sign-off before code. (B) Approve the hashed configuration now, and decide residency after the spike measures the per-step readback. (C) Neither: airflow stays host-authoritative and reads its grid back every step (about 40 MiB per step at 128³, not real time). **Recommend A.** Without residency 128³ cannot run in real time, and without the hash a restore across grids is silently accepted.

## 9. Sequencing: the GPU as soon as possible

| Milestone | Realms | Content | Depends on |
|---|---|---|---|
| **M0** | Core, Test/Docs, Physics | Banded-parity T2 → T3 → T4: the 3060 Ti admitted and announced. **Done** (T4 merged `e146575`; T6, the probe as a standing device record, in review) | Already ruled ahead of `PHY-7` (`backlog.md:63`) |
| **M1, the spike** | Physics, with Core | Standalone NS kernels at 64³ and 128³ in a worktree, run in the default build (T5 retires the measurement build; since T4 the default build admits the card). It measures ms per pass, per V-cycle and per fluid step, the residual each `n_V` leaves, the readback cost, and the host control loop's round trip (one state readback and one submission per step, §6). A report like the NVIDIA denorm report. It fixes `j`, `n_V` and MacCormack before any engine change | None now (M0 is done) |
| **M2** | Core | Stage 4 (the scratch kind before its merge, or in stage 7: the lead's call), then stage 7 (§1.7): the hashed grid configuration, residency, derived rebuild, point samples, regions, two recordings, 3D dispatch, device-only buffers, the limit and budget checks | Stage 4; this spec approved; Q11 (with A, stages 5 and 6 come first, so M5's Core half moves ahead of M3) and Q12 |
| **M3** | Physics, Rendering, Interface | Real-time tier on the CPU (16³/32³ tests, the golden) and on the GPU. One-way rotor sources; drag reads local air. The `airflow` scene on Vulkan with a free-flying quad; volume rendering by readback. This is the backlog's done-when ("the real-time tier running on the GPU in the sandbox and the live smoke") | M0, M2 |
| **M4** | Physics, Interface, Rendering | Two-way: drag pushback, probe-and-subtract, the airspeed sensor, the HUD's forces and moments, streamlines, smoke, the live smoke's `airflow` main | M3 |
| **M5** | Core, Interface | Stage 5 (grades declared and checked), stage 6 (the lock): the drone box on Vulkan with airflow | M3 |
| **M6** | Physics | LBM tier: CPU reference at 16³–32³, GPU kernels, the nested region, slow-motion mode | M4; Q5, Q6 |
| **M7** | All | 192³–256³; interop if measured necessary; the Vulkan raster's volume pass (`RND-4`) | M4 |

`PHY-7` followed M0 as planned: approved 2026-10-05, merging after stage 4, whose first declared scratch is its `contact_dv`. `PHY-8` stays after stage 4; it moves positions and velocities within a substep, not where fields are sampled (Fields samples at the substep's starting positions under either order), so it does not interact with this spec (confirmed by Physics).

## 10. What each realm must confirm

**Physics** (confirmed in review, 2026-10-05; the numbers that rest on the spike, `j` and `n_V`, are confirmed by M1's measurement, not here)
- The tier choices: MAC grid, MacCormack, V(2,2) red-black multigrid with a fixed `n_V`, apertures as face weights, free slip.
- The coupling: Peskin 4-point kernels, M = 48 disc points, coupling options A → B, impulse accumulators as `per_row` state.
- The airspeed sensor model, and its noise on `q`.
- The test list in §4.6, the golden `airflow_hover`, and the statistics past the horizon.
- That option B's on-axis profile takes a grade under `PHY-3`.
- That `PHY-8` does not interact (§9).

**Core** (answered 2026-10-05 in `core/2026-10-05-airflow-spec-core-answers.md`; folded into §1.7, §1.8, §3.4, §4.1–§4.5, §7, §8 and §9, including its 25 factual corrections)
- The missing capabilities: **amended.** The grid extent is partly there (a run-time-sized `per_world` row); a hashed configuration is missing. Six more were missing from the list: the scratch kind (not built), derived rebuild, stage 6's fixed bindings, the hashed configuration, device limits and 3D dispatch, device-only buffers. Proposed home: a new stage 7 (Q11).
- `rotor.forces` and `drag.forces` writing per-element outputs to scratch with no golden moving: **confirmed,** on four conditions (§1.8).
- `sample_medium` on Vulkan inside a region: **confirmed** as an `unavailable` refusal; **amended**: it is hard-wired to Dryden today, so region routing is new work.
- The buffer packing and the `maxStorageBufferRange` refusal: **confirmed; amended** with `maxMemoryAllocationSize`, the descriptor counts, 3D dispatch, one LBM buffer, and packed 16- and 8-bit storage.
- The staging memory type: **amended**: system memory by inference, not measured; scratch and derived buffers lose their staging twins.
- The snapshot growth, and the unchanged v3 blob without airflow: **confirmed** for the blob; **amended** on growth: every world pays, and a sync at snapshot.
- Q2 C and Q10 A confirmed; Q8 is Cameron's (Core neutral). Its decisions for Cameron are Q11 and Q12; whether the scratch kind joins stage 4 before merge is the lead's.

**Rendering** (confirmed 2026-10-05, with its amendments folded into §4.3 and §5)
- The Publish volume format: **amended** to two cell-centred textures, V (RGBA16F: `u, v, w`, dye) and S (RG16F: pressure, `|ω|`), with a per-frame header and no-data for non-finite samples.
- GL ray-march by readback; streamlines in a GL compute shader; dye as absorption: **amended**: a depth-only redraw into an internal texture, dye with one in-scatter colour, and streamlines on the CPU first.
- The CPU volume reference and its frame golden: **amended**: the golden renders a synthetic analytic volume, so it pins rendering alone.
- That the Vulkan raster needs nothing now: **confirmed.**

**Interface** (confirmed 2026-10-05, with its amendments folded into §3.5, §6 and §9)
- The HUD items and the α and β conventions: **amended**: airspeed, α and β from the free-stream air the aero used, not a body-centre point; the signs stated (aerospace); Spade's convention is the reference Kat adopts (the lead, 2026-10-05; §6).
- The `airflow` scene with a host position loop on Vulkan: **confirmed,** with the loop's rate measured (M1, M3) and the scene's files waiting on world file v3.
- The live smoke's `airflow` main and its checks: **amended**: every check made falsifiable (`wind`, `ground`, `forward`, the volume channels), and no device named.
- `INT-2` for the airflow panel: **confirmed,** through the editor's single rebuild path, with the fluid field carried only when the grid is unchanged.

**Test/Docs** (confirmed 2026-10-05, with its amendments folded into §4.6 and assumption 6)
- `TD-14` applied to airflow at 32³: the one-ulp control and `T_p`, the statistics past it, and the 128³ per-device record (Q9): **confirmed,** the 128³ record as an opt-in test.
- The golden's cross-check on the gcc leg (`TD-12`): **confirmed,** final on a fresh `-NoSeed` leg, and the digest independent of the thread count.
- The gate time the new CPU tests add: **amended** to a budget: at most about 60 s added to the debug gate, each test timed in both presets (§4.6).
- The SPIR-V scan of the new kernels: **confirmed.** Every new kernel, LBM and `mg_*` included, is registered in `kSpirvModules` under the parity profile, so `P1`–`P5` apply: no contraction, no `OpDot` or matrix products (stencils written componentwise), no Int64, no GLSL.std.450 transcendental, and no denormal mode (`P3`); `sqrt` is allowed. Every workgroup variant is scanned; a fixed-size kernel (`mg_coarse`) registers as single-variant. Anything that seems to need an exemption goes to Test/Docs first.

## Assumptions not confirmed in the code

1. The 3060 Ti sustains about 80% of 448 GB/s on these kernels, at about 4 µs per dispatch plus barrier. Every timing in §1.6 and §4.2 rests on this.
2. Windows, the driver, the GL editor and the engine together take under 4 GiB of VRAM.
3. Staging buffers land in host memory, not the device's host-visible BAR heap.
4. `VK_EXT_memory_budget` is available on driver 572.83, and `maxStorageBufferRange` there exceeds 216 MiB.
5. A V(2,2) cycle with apertures reduces the residual 3–5×.
6. The CPU passes stay single-threaded (no threading was found under `engine/`). If that ever changes, the CPU twin keeps a fixed partition and gather order, so the golden's digest does not depend on the thread count: the leg's container and the Windows box need not see the same core count (Test/Docs, 2026-10-05). `std::sqrt` is correctly rounded on both toolchains; any other libm call stays forbidden (`TD-3`).
7. Kat runs the CPU backend, and its `dt` and substep count are unknown here.
8. A 5-inch prop (R ≈ 6.4 cm) is representative of Kat's vehicles.
9. The GL loader can add the compute and 3D-texture entry points. **Confirmed by Rendering** (2026-10-05): the private table is now 58 entries at `gl_renderer.cpp:67-125`; `glDispatchCompute` (4.3), `glMemoryBarrier`, `glBindImageTexture` and `glTexStorage3D` (4.2) are past the vendored glad 4.1, so they get hand-declared types, as `glCopyImageSubData` does (`:41`); `glTexImage3D`/`glTexSubImage3D` (1.2) are already in glad; `create()` already refuses below 4.3.
10. The SDF evaluation cost per cell is small enough to re-evaluate the box on a recentre.
