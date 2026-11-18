# L90 — an open face's outflow is left to the projection, and the advection reads the ambient past it

Physics, 2026-11-18. Branch `physics/open-outlet-fix`. The design table is amended in `2026-10-05-airflow-design.md` (the "Outflow" row and "Advection past an outflow face"); this is the account behind it.

## The finding

A box held a smaller share of an injected impulse the smaller the step h was, and the actuator lines' thrust at a fixed rotor speed fell with h (x500, 0.05 s: 17.269 N at 0.2 ms, 16.669 at 0.1, 15.735 at 0.05, 15.105 at 0.025). The loss was about 7e-4 of the momentum per step at every h.

Cause: `apply_velocity_boundaries` copied the neighbouring face into each pressure-outlet face and refused inflow (`if (copy > 0) copy = amb`), once per step with no factor of h, four times per step. The refusal wiped the air a rotor draws in at an open face. Split on the 64³ CPU twin: refusal alone loses the momentum; copy alone (inflow allowed) conserves it.

Deleting the refusal alone is not enough: the advection's `locate()` clamps a back-traced point outside the box to the edge sample, so inflow re-injects the jet's own momentum as a source of kinetic energy and the backflow grows (the angled jet reaches 60 m/s).

## The fix (both parts)

1. Outlet faces are left as the projection sets them. `boundaries.cpp` and `airflow_velocity_bc.slang` write only periodic and fixed-velocity faces.
2. The advection reads an ambient ghost past an outlet. A back-traced point below the first face (or above the last) of an outlet axis takes the ambient value, linearly blended with the edge sample over one cell. `advect.cpp`, `airflow_advection.slang` and the velocity sampler of `airflow_dye_advect.slang`.
3. The dye's own scalar sampling keeps the edge clamp (`sample_centred`, `centred_axis`): the dye has no ambient value to read, and the clamp is what lets a plume leave.

## Evidence

- Box momentum over impulse at h 0.4 / 0.2 / 0.1 ms (24³ pin box): old rule 0.748 / 0.618 / 0.450; at the final test shape on the old engine 0.6965 / 0.544 / 0.366 (the pin fails at 1.5%); untouched + ambient ghost 1.027 / 1.034 / 1.038.
- CPU twin, 64³: old 0.700 / 0.561 / 0.399; outlets untouched 1.029-1.032.
- GPU discs (0.03 s): 0.883 / 0.786 / 0.640 old.
- The angled jet over 3000 steps at h 0.2 ms, peak every 250 steps: old rule 34 m/s flat (the forcing disc's own faces carry ~34); copy + inflow 42-59 m/s noisy; untouched + ambient ghost 31-35 m/s flat.
- A disc drawing its air in through a side face stays bounded; the column air is +4.7% (middle disc) and +1.4% (side disc), the angular share within 3%.
- Tests: `AirflowMomentumAccount.*` (three pinned: the share at h and h/2, the angled jet, the side-face disc; four opt-in with `SPADE_MOMENTUM_ACCOUNT=1`).

## Validation, old rule against new

The old column is the current master's engine; the new is the fix. Both pass every test in the filter (49/49: sphere, couette, sheets, actuator, dense disc, disc peak, line).

| measure | old | new | change |
|---|---|---|---|
| sphere max abs(Cp - Cp,pot), one cell out, step 3 | 0.0581 | 0.0582 | +0.0001 |
| sphere drag, wall_load(), step 3, U h/dx 0.1 | 0.0674 | 0.0676 | +0.0002 |
| the same, U h/dx 0.05 | 0.0466 | 0.0471 | +0.0005 |
| sphere side forces | 5.77e-6 | 2.25e-6 | smaller |
| phantom 10 mm, 125³, step 2, CG 8 residual | 9.36e-6 | 9.48e-6 | +1% |
| the same, step 3 | 6.74e-6 | 6.97e-6 | +3% |

C8 actuator-disc far wake (momentum theory 7.5777 m/s; band 10%, half-width 0.758 m/s; 8 runs, CPU):

| measure | old | new | change |
|---|---|---|---|
| the eight runs' mean downwash, 80 v faces within 0.5R, 2R below, steps 385-768 | 7.8893 | 7.8707 | -0.019 m/s (-0.24%) |
| the worst of the eight runs | 7.9796 (run 6) | 7.9050 (run 0) | -0.075 |
| the eight runs' largest deviation from their mean | 0.0902 | 0.0343 | -0.056 |
| the axis 2R below, steps 257-384, run 0 | 7.9698 | 8.0452 | +0.075 (+0.9%) |
| the axis, eight runs' range | 7.7495 to 8.0064 | 7.5184 to 8.0836 | wider |

All inside the band. **Mind the drift:** the record `tests/data/airflow/records/c8-actuator-disc.md` reads 8.0333 m/s for the first measure; the current master (before this fix) reads 7.8893. That is the change of commits since the record was taken, not this fix; this fix moves it by -0.019. The C8 wake sits 2R below a disc and barely uses an outlet, so it does not show the fix. C8q (two ensembles of four at 128³, hours of CPU) was not re-run: costs first; the x500 re-measure on the fixed GPU solver is where a rotor wake reaches the outlets, and if it moves the induced velocity by more than about 3%, C8q is the next question.

Caradonna: the repo's Caradonna tests check the table's own data and run no solver; there is no old/new number to give.

## Gates on the merged head (master e7e1d076 merged)

- `-L gpu`: 417 pass, 0 fail, 40 skipped (env-gated timing and live tests). Device against CPU is bit for bit: `GpuAirflowAdvection` (four tests), `GpuAirflowStep` open box, odd-sized open box and periodic channel.
- CPU subset (664 tests): `AnalysisValidation*`, `SpirvPin`, `AirflowConfigs`, `AirflowMomentumAccount` pass.

## Re-pins, each with its reason

- `AirflowConfigs.EachBoundaryConfigurationsDigestIsPinned`: the five open-face configurations (open 9x7x5, open 16x12x16 disc, wind 12x10x8, floor under wind, one-cell x) move: 0x56707cc190cd8e1d, 0x747a9a1e77bf9740, 0x6ce704702b0a9ba7, 0x03de1fb4e13e3e72, 0xf9055fa7b35de410. Periodic, walls and cavity digests are unchanged (their faces are bit-identical).
- `SpirvPin`: nine hashes (airflow_advection, airflow_dye_advect, airflow_velocity_bc at local sizes 32, 64, 128); the kernels changed on purpose.
- `AnalysisLiveRun.ALooseSolveIsCountedBesideTheUnconverged` at k=3: 16 unconverged / 1 loose becomes 6 / 11, the loosest at 9.5e-5. The solve is held to 3 iterations; 10 of the steps that did not reach the mark now reach the loose mark, with the same cap. Cases k=4 and k=8 are as before.
- `AirflowAdvect.ANonFiniteOrHugeCoordinateSamplesInsideTheField`: the clamped box's sample range may reach the ambient (0), not only the field's values.
- `AirflowGrid` outlet tests: an outlet's faces stay as the projection set them (the old refusal test is replaced, with this reason).
- `GpuAirflowAdvection` parity test: its params block now carries the face kinds, words 18-23 (0 fixed velocity, 1 pressure outlet, 2 periodic), and the ambient velocity in the face block (word 27 gives its offset; ambient at +19, +20, +21), as `airflow_params_layout.slang` names them; the kernel reads them for the ghost.

## Records whose numbers change

- The x500 notes and step-size finding: the thrust balance, the settle time and the h table are re-measured on the fixed GPU solver (approved: about 42 min GPU, 3.5 GiB, no new disk).
- Phantom and near-field figures that were taken at an open face; the re-measure names the ones that move.
- Kept frames: a re-render waits for Core's kit table and for costs.
- `c8-actuator-disc.md`: not edited here; it keeps its older commit's number, now named in this record.
