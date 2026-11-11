# L52: the subgrid eddy viscosity (Smagorinsky), CPU step

Decided by Cameron, 2026-11-10, after the L48 wake probes. Code: `engine/physics/airflow/sgs.hpp/.cpp`,
`FluidConfig::sgs_cs`. Tests: `tests/test_airflow_sgs.cpp`. The GPU kernel is L54 (the last section).

## Why

The air step has no viscosity worth the name at the grid (nu = 1.5e-5 m^2/s acts over 1.4 um; a cell is 4 to 10 mm), and
the advection clamp only damps what it must. A wake's energy therefore piles up at the grid scale and grows with time.
The L48 probes (physics/wake-probe, never merged; `C:/Users/Cameron/wake-out/*/analysis.json`) measured it:

| run (quad hover, 0.6 s unless noted) | ke_hi | filtered/raw vorticity | clamp fire |
|---|---|---|---|
| 1 cm CPU, Cs 0 | 0.073 | 0.51 | 7.35% |
| 1 cm CPU, Cs 0 at 1.2 s | 0.076 | 0.40 | 11.9% |
| 1 cm CPU, Cs 0.10 | 0.025 | 0.72 | 3.8% |
| 1 cm CPU, Cs 0.17 | 0.010 | 0.78 | 2.1% |
| 1 cm, disc tapered and root-cut | 0.085 | 0.50 | 7.5% |
| 1 cm, box padded 20 cells | 0.075 | 0.53 | |
| GPU 5 mm / 4 mm, Cs 0, 0.15 s to 0.6 s | 0.014 to 0.038 / 0.012 to 0.034 | 0.70 to 0.40 / 0.71 to 0.37 | |

ke_hi is the share of kinetic energy above k = pi / (2 dx) on the wake sub-box. The disc rim and the box are not the
cause; Cs 0.17 is the lowest value that clears the speckle without weakening the jet (the axis speed under the discs and
the momentum flux are unchanged by Cs). The centreline collapse at 0.2 to 0.3 m under the discs is a separate, open item.

## The setting

`FluidConfig::sgs_cs`, a float, 0 (the default) is off: the step is today's, bit for bit, with no scratch and no extra
pass. Valid: 0, or finite and at most `kSgsCsMax` = 0.3. A bad Cs is refused by name: `make_fluid_scratch` names
`sgs_cs`, and a step whose Cs was changed to a bad one faults `FluidFault::bad_sgs_cs` (kept, as the other faults are).
`GpuFluid::create` takes it too since L54 and refuses a bad one by name.

The step adds du_i/dt = d_j (nu_t d_j u_i), nu_t = (Cs dx)^2 |S|, |S| = sqrt(2 S_ij S_ij), explicitly, after the sources
and before the molecular diffusion (conditions and solids re-applied after it).

The step uses the Laplacian form d_j(nu_t d_j u_i). The transpose term d_j(nu_t d_i u_j) of 2 nu_t S_ij is omitted: it
vanishes for a uniform nu_t by continuity, and it is an approximation where nu_t varies (the jet's shear layer).

## Order and threads

Two phases from the field the step holds: every cell's nu_t, then every face's change into a buffer, then one add. No
face reads another's result, so the loops split by plane on the existing pool with no cross-plane reduction (the
diagnostics are a max and an integer sum, exact in any order): the same bits at any thread count.

## The wall rule (solids are in scope)

The molecular diffusion's own face handling (viscous.cpp), term for term, with the link viscosity nu_t:

- a sample the solids hold (theta = 0 or phi < 0) is not changed;
- a link a wall crosses at the fraction f takes nu_t (1 / f) (u_wall - x) in place of nu_t (x_nb - x);
- a domain face's ghost: a wall 2 (U - x), an inflow 2 (ambient - x), an outlet the zero gradient, a periodic face its wrap;
- nu_t is 0 in a solid cell (`regions.solid`, no open face), and the nu_t of an edge (three of a face's links) is the mean
  over the fluid cells round it, none: 0.

## Stability

Explicit diffusion in 3-D needs nu_t h / dx^2 <= 1/6. nu_t is held at dx^2 / (8 h) and the cells held are counted. A face
whose weighted links (a wall link weighs up to 1 / f = 100, `kMinCrossing`) would break the bound is divided by it, so it
takes a weighted mean of its neighbours and the wall and cannot overshoot; those faces are counted too. Neither refuses a
step: they limit the model; they do not make the state ill-posed. `FluidHealth` carries `sgs_capped_cells`,
`sgs_limited_faces` and `sgs_nu_t_max`; flow_movie prints them with its progress line.

At 4 mm with Run air Detailed's CFL of 2 (h about 0.73 ms) the cap is about 2.7e-3 m^2/s against about 1.3e-3 wanted in a
10 m/s shear: a 2x margin. Measure it there before choosing an implicit solve for the GPU kernel.

## Where it is on

The 10-13 quad film (`tools/flow_movie/quad_case.cpp`, discs only) sets Cs 0.17. No Run air or LiveRun config sets it in
this ticket; every test and pin that builds a `FluidConfig` keeps Cs 0.

## Tests (AirflowSgs, opt-in AirflowSgsFilm)

Off touches nothing; on changes the field; a constant shear's nu_t, the cap, a quadratic shear's d_y (nu_t d_y u) exactly;
uniform flow and rigid rotation left alone; the wall rule on a sphere and a maximum principle with the cap and the wall
limit engaged; the same bits at 1, 2, 3 and 8 threads and on a reversed pool, with and without solids; the refusals by
name; a sphere in a wind at Cs 0.17 (default) and the reference quad's D1 airframe at 4 mm (opt-in, SPADE_SGS_D1=1).
The film test (SPADE_SGS_FILM=1) holds ke_hi at Cs 0.17 below half of Cs 0 and the flux through the plane 0.30 m under the
hubs within Test/Docs' band, 0.90 <= R <= 1.10 (docs/design/test-docs/2026-11-10-sgs-film-bands.md).

## L54: the GPU kernel

`engine/shaders/kernels/airflow_sgs.slang`, an exact twin of `sgs.cpp` (TD-14 as amended): the same expressions in the same
order, every division `div32_ftz`, the one square root `sqrt32_ftz`, nothing contracted. Five modes in one kernel: nu_t per
cell (the raw value is stored; a reader holds it at the cap, which is the CPU's stored value bit for bit), each component's
change (the link's nu_t the fluid-cell mean, the wall limit's flag beside it), each component's add, and a two-stage
reduction of the report (a fixed stride over cells and faces into 1024 partial slots, then one invocation summing them:
integer counts and a max do not depend on order, so no atomics, which the exact profile forbids). The chain puts the
passes where `fluid_step()` puts the hook (after the sources and boundary passes, before the molecular diffusion, the
boundaries and solids re-applied after) and only when Cs > 0: at Cs 0 nothing is dispatched and nothing is sized, so the
step is as it was.

No new binding. The scratch is `airflow_advect` (dead between the add-increments pass and the next step's advection),
sized for it when Cs is on; k = h / dx^2, c2 = (Cs dx)^2 and the cap dx^2 / (8 h) are airflow_params words 29-31, the
CPU's own expressions computed on the host under its flush; the report is airflow_scalars' words 28-30 (the counts as
uint bits, the peak), read by the host with the CG's health. `GpuFluid::create` refuses a bad Cs by name as before and
no longer refuses a good one; LiveRun carries no Cs, so its GPU twin (`gpu_live_fluid`) needed no change: the maker
passes the config through and the device's refusal it relayed is gone (`GpuLiveFluid.TheMakerTakesASubgridViscosityAndRefusesABadOne`).

Evidence (the device of record): every `GpuAirflowStep.Subgrid*` case is bit for bit with `fluid_step()`, u, v, w, p and
the eight health values after every step (open box, odd box at the cap, periodic channel, spinning sphere at the cap,
turned part, periodic channel with a sphere, a block over the inflow; the cap and the wall limit are reached in them, the
test asserts it); the film's quad case 32 steps from rest; D1 with its solids (b7's box, the dense discs) 32 steps, 28.66
million values a step and the wall load, 0 differing.

Bound, D1 with solids at 4 mm, 40 steps from rest (start-up, not a settled jet), GPU, report only:

| h | cap (m^2/s) | nu_t max | capped cells (max, mean) | wall-limited faces (max, mean) of 21.5 M |
|---|---|---|---|---|
| 0.20 ms (the film's) | 0.0100 | 0.00245 | 0, 0 | 97, 80 |
| 0.73 ms (Run air Detailed's CFL 2) | 0.00274 | 0.00305 | 8, 0.5 | 1135, 849 |

Cost, the same box, mean of 38 steps after 2: Cs 0 134.68 ms, Cs 0.17 149.13 ms (+14.5 ms, +10.7%) at h 0.2 ms; at 0.73
ms 131.72 and 145.78 ms (+14.1 ms). Against the record's 138.4 ms the Cs 0 reading of this box today is 134.7.
At CFL 2 the cap binds on a handful of cells and the wall limit on under 0.01% of the faces; an implicit solve stays a
later question, and the numbers that would raise it are a settled jet's, not these.
