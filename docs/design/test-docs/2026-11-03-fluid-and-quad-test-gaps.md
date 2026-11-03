# Test gaps: the quad-rotor geometry and the fluid sim, CPU and GPU

Test/Docs, 2026-11-03, for the lead. Master at `4aedb311`. What isn't pinned that should be, in the order I recommend. Each item names what exists, the gap, and a proposed test or record. Nothing here is built.

What is already well pinned, so it isn't listed: GpuFluid's step against `fluid_step()` bit for bit on ten boxes, with and without solids, at every workgroup size (`GpuAirflowStep`); the CG's decisions (B5.4); NaN faults on both sides (`GpuLiveFluid`, P6); B7 on the device of record (`GpuB7Acceptance`, short and opt-in); the library drones' specs, sources, hover points, props by hand and solids at 4 mm (`LibraryDrones`, `LibraryDroneSolids*`); the dense disc (`AirflowDenseDisc`, `AirflowSpreadSkip`, `AirflowDiscPeak`).

## 1. B8's records: the GPU results are not stored anywhere a test reads

- **Now:** no record in `tests/data/airflow/records/` holds a GpuFluid run. Only `c8-actuator-disc.md` has GPU-path blocks (B4's gather inside the CPU step). B7's D1 block (0 of 917,091,424 values over 32 steps with solids, step median 295.3 ms, film 391.53 s) is printed by the opt-in test and lives only in logs and the backlog (`6d24ef38`).
- **Plan:** B8 asks for each band's record. B5.6 says S1's and S3's records "become bit-equal records … checked by `AnalysisValidation*`"; `s1-sphere.md` and `s3-sheet.md` hold no GPU block.
- **Proposed:** a GPU block per case, pasted from the opt-in runs the way C8's were: D1 with solids (B7), S1, S3, and C8q (the quad-rotor validation case). Each carries the device line, "0 of N values differ, steps 1 to N", and B7's step time as a report-only measure, so the profiling fix shows as a record diff. Owners: Physics (B8), Test/Docs (format, `AnalysisValidation*`).

## 2. The GPU grade is unplaced and the status says "CPU only"

- **Now:** `physics/04-verification.md`'s "Airflow solver core" row says "not placed: the GPU twin is plan B". The solids and `wall_load()` rows say CPU only, and so do `physics/07-status.md`'s airflow rows. Since G3 (`68761817`) the whole step, solids included, is bit-equal on the device of record.
- **Proposed:** place the grade (B8): bit-equal to the CPU reference on the device of record (`TD-14` as amended), not placed elsewhere (item 6), with the tests named. A docs change. Physics writes it and Test/Docs reviews it.

## 3. Disc points inside solids: no test says what happens

- **Now:** `actuator.cpp` spreads every disc point without reading θ. Nothing pins what a point in a solid cell, or on a closed face, does: masked by the projection, or forcing a wall. The Phantom's and the Iris+'s discs sit over their motor bells (L7b1's motor meshes), and the dense disc's innermost ring is at 0.22 R (`ring_radius()`, 10 rings).
- **Proposed:** a CPU test with a disc whose inner rings overlap a solid. After the step, no closed face carries an increment, or each point is named. One GpuAirflowStep case of the same box, bit-equal. If the projection already makes it harmless, the test pins that. Owner: Physics.

## 4. A placed library drone has never run on GpuFluid

- **Now:** `GpuEditorAirflow` runs the reference quad, and `GpuLiveFluid` runs a ball quad on 32³. The Phantom's mesh-derived solids run only on the CPU (`EditorAirflow.ThePlacedPhantomsAirStartsAndPublishes`).
- **Proposed:** `GpuEditorAirflow` with the placed Phantom: a few frames to the bit on the device of record, opt-in if slow. It covers mesh solids, the prop-hub geometry and the dense disc together on the GPU. Owners: Interface and Rendering.

## 5. Debug coverage of the drone solids rests on two tests near the limit

- **Now:** the Phantom, the Iris+ and Physics' `ThePhantomsBodyTopOnACellCentrePlaneBuilds` (`e39fdca6`) skip in debug. The basic quads that cover them take 40 to 57 s in debug against ctest's 60 s. The `e39fdca6` skip has no guard-table row, which is moot because Core's cull removes every skip.
- **Proposed:** when the cull lands, re-time every `LibraryDroneSolids*` case in debug under ctest, `-j 1` and under load. Keep each under about 30 s (Core's target). Then I drop the guard-table rows.

## 6. Other GPUs: nothing fails

- **Now:** off the device of record, B7 skips, and `GpuLiveFluid.FliesTheCpusRunToTheBit` and `FliesTheBallQuadToTheBit` print a differing count without asserting (`if (exact)`). A wrong kernel on another driver passes every test.
- **Proposed:** a floor that runs on any device. The run stays finite and under the physical bound, 4 × (2 v_h + the wind), and its mean speed stays within a loose band of the CPU's, until a second device can measure a real band. This places a grade other than "skipped" elsewhere, so it needs your call, and the owner's if it touches `L4`.

## 7. The step time is printed, never kept

- **Now:** B7a was accepted with its MISS (295.3 ms against 190), and profiling is scheduled. The opt-in prints the number, and nothing keeps it.
- **Proposed:** item 1's record keeps it, report-only. After the profiling fix, compare the record with B7's 95 ms estimate. It's a reading, not a gate (`TD-8`'s spread applies).
