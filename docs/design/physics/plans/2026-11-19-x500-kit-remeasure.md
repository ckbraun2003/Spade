# The x500 flow model on the kit table, and its re-measure (L98, L99a, L103, L104; L105 open)

Physics, 2026-11-19. Branch `physics/x500-kit-model`. Files: `tests/x500_model.hpp` (the builder), `tests/test_x500_case.cpp`, `tests/test_x500_kit_inertia.cpp`, the regenerated `tools/flow_movie/data/x500/x500.model.yaml`, `x500.notes.yaml` and `parts/x500_10in_apc10x5.prop.yaml`.

## The model

Core's kit table (`tools/flow_movie/data/x500_build/parts.yaml`, commit b1a5a7f0 on `core/x500-hubs-measured`), 16 entries, 1.6326 kg, in the kit file's frame (Y up, +X forward, +Z right, the origin the bottom plate's lower face). That frame is the design frame here.

| What | Value | Basis |
|---|---|---|
| mass | 1.6326 kg | the table's entries' sum (was 1.6389 kg on the 13-part table) |
| centre of mass, table frame | (-0.00446, -0.02543, 0.00088) m | the part meshes (below); Core's own (-0.00448, -0.02581, 0.00056) is 0.02, 0.38 and 0.32 mm away: the meshes confirm it |
| motor axes | (+-0.17689, +-0.17689) m | Interface's circle fit, L98 |
| actuator disc | 0.0607 m above the table's origin | the blade mid-plane, Core's measure on the rotor glTF, L99a; the drawing's rotor origin (0.070 m) is not the disc |
| rotor local_pos | R (hub - com_offset), R the file's `design_to_principal` | the compile's own frame rule |

**Inertia, basis by part kind** (`X500KitInertia.TheKitsMassCentreAndInertiaFromItsPartsMeshes`, opt-in, reads the local part store; the numbers it prints are the constants in `x500_model.hpp`):

- Parts with store meshes (the frame's 61 objects, the 4 motors, the controller, the GPS, the Pi, the camera): `TriangleMesh::mass_properties()`, the engine's polyhedral volume, centroid and tensor, exact for the shape, at **uniform density scaled to the part's mass**. The frame's 0.610 kg is split over its 61 objects by volume: **equal density across the carbon, aluminium and steel objects is assumed** (the same assumption as Core's centre of mass for the frame).
- The pack (0.4365 kg), the radio and the misc carrier: uniform boxes (shapes only, no mesh).
- The propellers: thin discs of R 0.127 m, 0.013 kg each.

Composite about the centre of mass (design axes, kg m^2): xx 0.0237929, yy 0.0289195, zz 0.0233956; products xy +4.85e-4, yz -5.54e-5, xz -1.7e-6.

**The products turn the principal axes 5.4 degrees about z** (the 2% xy product, with Ixx and Izz 2% apart). The lead chose to keep `compile_airframe`'s own `design_to_principal` in the file, with nothing cancelled and nothing re-expressed after the compile (L104a): the model file carries that rotation, `body.inertia` is the principal moments (0.0237474, 0.0289656, 0.0233951), and every body-frame field (rotors, mounts) is the design-frame one turned by it and shifted by -`com_offset`. Compared with the earlier model (0.0146482, 0.0225855, 0.0157698 kg m^2 at 1.6389 kg, a box-and-tube sum that put the frame's mass in the stack) the principal moments are **+62%, +28% and +48%**.

The proxy solids for the flow (the plate stack 0.15 x 0.0805 x 0.15 m, four arm tubes at y 0.016, the pack box under the pad at y -0.0653, the ESC block) compile as they are; one tensor record, `kit_remainder` (no solid), completes the compile's composite to the kit's mass, centre of mass and whole tensor.

**Who reads `design_to_principal`** (so the turn is applied, not assumed identity): appliers in `engine/analysis/live_run.cpp`, `engine/sim/simulation.cpp`, `sandbox/editor_shapes.hpp`, `sandbox/live_tour.cpp`, `sandbox/vehicle_draw.hpp`, `sandbox/flight_controller.hpp`; the editor's `check_shape` refuses a turned frame openly for box collision shapes. `tools/flow_movie/build_parts.hpp:778-779` assumes the identity and the origin at the centre of mass, which is wrong for this kit: Rendering's, RED first.

The X500 grid is 270 x 237 x 270 at 2.5 mm (was 266 wide): the measured hubs are 2.7 mm further out, and 4 more cells keep the 0.0165 m past the kernel's reach; 17.3 M cells, 3.775 GiB at the device runner's 234.6 B a cell.

## The re-measure (the fixed solver, L90)

Kit model, 270 x 237 x 270 at 2.5 mm, rotor speed 567.1 rad/s (the model's hover speed), n_sub 8, no frames kept, 25 min GPU on one device, no new disk. The lines' thrust in all four rotors, N (the weight is 16.011 N):

| t | h 0.2 ms | h 0.1 ms | h 0.05 ms |
|---|---|---|---|
| 0.01 s | 15.606 | 15.396 | 14.348 |
| 0.02 s | 16.943 | 16.068 | 14.636 |
| 0.03 s | 16.818 | 16.423 | 15.135 |
| 0.05 s | 17.212 | 16.712 | 15.765 |
| 0.075 s | 17.045 | 16.410 | 15.267 |
| 0.10 s | 17.382 | 16.595 | 15.551 |

- The box's momentum over the impulse put in is the same at every h (1.0155, 1.0156, 1.0160 at 0.01 s; 0.9667, 0.9749, 0.9833 at 0.05 s) with a healthy pressure solve (8 iterations, residual under 1e-4, none capped). L90 conserves the momentum at every h.
- **The thrust at a fixed rotor speed still falls with h** (-2.9% and -5.7% per halving at 0.05 s; before L90, on the previous model: -3.5% and -5.6%). L90 did not remove it. There is **no settle time**: the thrust scatters between 15.4 and 17.4 N after 0.03 s at h 0.2 ms, and none of these values is a converged balance against the weight.

## L105: where the thrust-against-h dependence comes from (CPU, so far)

One kit rotor (the 1045 at 569.2 rad/s) in an open box at 5 mm, `AirflowX500.TheLinesThrustAgainstTheStepInOneRotorOnTheCpu`, opt-in (`SPADE_L105=1`), thrust at 0.05 s, N:

| h ms | as now | no tip correction | one azimuth a step (n_sub 1) | frozen uniform inflow, no fluid step |
|---|---|---|---|---|
| 0.4 | 4.150 | 4.341 | 4.549 | 4.088 |
| 0.2 | 3.967 | 4.181 | 4.049 | 4.088 |
| 0.1 | 3.811 | 3.969 | 3.824 | 4.088 |
| 0.05 | 3.707 | 3.860 | 3.712 | 4.088 |

- The line alone is h-independent (frozen inflow, constant to 4 digits, step 1 identical at every h). It is the feedback through the fluid step. Not the sub-azimuth sampling (one azimuth a step gives the same numbers at small h), not the tip correction, not the pressure solve (n_v 8 against 2: within 0.006 N).
- The disc's mean downflow follows momentum theory at every h (the ratio to sqrt(T / 2 rho A) of the reported thrust is 0.959, 0.958, 0.962 at 5 mm): the fluid does not gain momentum it should not.
- The advection is already RK2 with a clamped MacCormack correction: no higher-order mode to switch to.
- The same cells a blade moves a step at dx 10 mm (h doubled) gives -0.9%, -1.3%, -0.9% a halving at 0.05 s, against -4.4%, -3.9%, -2.7% at 5 mm: the dependence is not a function of cells a step alone, it grows with resolution. Cause open.

### L105 on the device: the thrust against the tip's cells a step (`GpuX500OneRotor`, opt-in `SPADE_L105_GPU=1`)

The same rotor and box on the GPU (n_sub 8, n_v 2, **the lean mode: nu 0, no subgrid model, the config's defaults**; h = cells dx / omega R, the tip at 72.3 m/s), the lines' thrust **as the mean over the last revolution (11.04 ms)** at 0.05 s, N, and the change per halving:

| tip cells a step | 1.45 | 0.72 | 0.36 | 0.18 |
|---|---|---|---|---|
| dx 5 mm | 3.728 | 3.639 (-2.4%) | 3.596 (-1.2%) | 3.572 (-0.7%) |
| dx 2.5 mm | 3.795 | 3.600 (-5.1%) | 3.485 (-3.2%) | 3.716 (broken, below) |

- **Those rows are the lean mode (the config's defaults: nu 0, no subgrid model), which breaks down.** The disc's mean downflow, 0.97 of momentum theory in every sound row, falls to about 0 while the lines' thrust stays 3.6 to 4.1 N: dx 5 mm 0.18 cells at 0.1 s (step 8032: 0.02 m/s), 0.36 cells at 0.1 s (step 4016: 0.89 of theory, going), dx 2.5 mm 0.36 cells at 0.1 s (step 8032: 0.48 m/s) and 0.18 cells by 0.05 s (step 8032: 0.48 m/s). n_v 8 gives the same thrust to four digits and the same collapse at the same step: not the projection.
- **The breakdown is a grid-scale instability on the rotor's axis, undamped in the lean mode; the film's configuration (Smagorinsky Cs 0.17, nu 1.4607e-5, which the stills and the film use) does not have it.** Evidence (`SPADE_L105_EVERY`, dx 5 mm, 0.18 cells, lean mode, every 500 steps): the box's downward momentum is the impulse put in to 1% up to step 4000, the projection is healthy throughout (2 iterations, residual 1e-4 to 1e-3, none capped), the host azimuth keeps to 0.007 rad of step x h x omega with no drift (no counter, no wrap: nothing in the step chain is sized at 4096 or 8192, read in `spike/airflow` and the line, advection and add-increments kernels); then the extreme v grows from 17 m/s (step 4500) to +-47 (5000), +-141 (5500) and +-200 m/s (7000 on), of both signs, in the same cells (the hub's plane, and the axis 0.1 m under the disc), and the downflow and the box's momentum fall away from the impulse (ratio 0.86 at step 6000, 0.31 at 8000). With Cs 0.17 and nu 1.4607e-5 on the same case the extreme v stays at 15.6 m/s to step 8032, the downflow stays 5.3 m/s (0.98 of theory) and the thrust 3.659 N at 0.1 s. The lean mode is a mode of the solver, not a bug in a counter; what the user (and Spade's own default config) is told about a long inviscid run with lines is the open point.
- **With the film's configuration the thrust converges, at first order, at both cell sizes** (n_sub 8, n_v 2, thrust at 0.05 s, the last revolution's mean, N; the changes a halving in brackets; every row sound, the disc's downflow 0.95 to 0.98 of momentum theory):

| tip cells a step | 1.45 | 0.72 | 0.36 | 0.18 |
|---|---|---|---|---|
| dx 5 mm | 3.725 | 3.637 (-2.4%) | 3.595 (-1.2%) | 3.573 (-0.6%) |
| dx 2.5 mm | 3.797 | 3.606 (-5.0%) | 3.534 (-2.0%) | 3.493 (-1.2%) |

  The limit in h is about 3.55 N at 5 mm and about 3.45 N at 2.5 mm (a first-order extrapolation of the last two changes): 3% apart, the cell's own dependence. **At 2.5 mm the film's 0.05 ms is 1.45 cells and gives 3.80 N, 10% over the converged thrust; the blade's step must be about 0.18 cells (0.0062 ms) for the thrust to be within about 1% a halving.** And the converged thrust at the model's hover speed is 3.45 to 3.55 N against W/4 of 4.003 N: the lines at this speed carry about 86 to 89% of the weight, at 0.05 s of an impulsive start (the 5 mm run is still rising at 0.1 s: 3.573 at 0.05, 3.598 at 0.075, 3.659 at 0.1).
- The wake front crosses the 0.35 m under the rotor in the first box (0.55 m high) at about 0.065 s: its 0.075 and 0.1 s rows are box-limited, so the table above is from 0.05 s and the deep box (1.0 m high, `SPADE_L105_DEPTH`) was used for the late rows; the 5 mm 0.05 s values are the same in both boxes to 3 digits.
- **Cost:** about 52 min of GPU in all, no frames kept, no new disk: the lean-mode sweeps 31 min (the 2.5 mm deep sweep 22 min at 41 ms a step, 10 M cells; the 5 mm sweeps 9 ms a step), the diagnostic and Cs run 4 min, the film-configuration sweeps 14 min (2.5 mm: 13 min at 51 ms a step, 5.6 M cells). Peak VRAM was not measured.
