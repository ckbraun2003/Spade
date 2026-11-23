# The OpenFrame-5F flow model and its checks (O138)

Physics, 2026-11-21. Branch `physics/openframe5f-model`. Files: `tests/openframe5f_model.hpp` (the builder), `tests/test_openframe5f_case.cpp`, `tests/test_openframe5f_inertia.cpp`, the generated `tools/flow_movie/data/openframe5f/` (model, notes, stand-in prop part). The table is Interface's `tools/flow_movie/data/openframe5f_build/parts.yaml` (commit 3157ff93, 14 entries, 0.5665 kg); every input's grade is in the notes file.

## The model

| What | Value | Basis |
|---|---|---|
| mass | 0.5665 kg | the table's sum; nobody's take-off weight (no camera, video transmitter, antennas or wiring) |
| centre of mass, design frame | (0.00191, 0.03004, -0.0000023) m | the part meshes, `test_openframe5f_inertia.cpp` on the part store |
| tensor about it | xx 1.879e-3, yy 2.970e-3, zz 1.594e-3, xy 5.51e-5 kg m^2 | the same meshes; the principal axes turn 2.88 degrees about z |
| hubs | (+-0.069534, +-0.089000) m, disc plane 0.0436 m above the design origin (0.0136 m above the centre of mass) | measured on the exported arm (Interface); the 4 mm of hub above the motor is assumed |
| rotors | 0 FR +1, 1 FL -1, 2 RL +1, 3 RR -1; R 0.0635 m | the library's fpv_5inch order and spins |
| propeller | the library's uiuc_mit_5x4, a stand-in for the HQProp 5X4.3X3 V2S (no published data) | the polar was fitted at 6,057 rpm with a 12 mm kernel; here it runs at about 9,000 rpm and a 1.25 mm cell |
| hover | **9,298 rpm (973.704 rad/s)**, the chain trim | the model's static C_T table (C_T(0) = 0.18147) at a quarter of the weight, 1.389 N, which is what the film's case and any controller call hover. The grid-free blade-element balance gives 8,986 rpm (941.0 rad/s): it is 3.4% lower in speed because the line's polar was fitted through the line in a grid to UIUC's 6,057 rpm point, not through the grid-free balance, and the balance's static C_T (about 0.194) is 7% above the table's. **941.0 was this record's first hover figure and is retired** (correction of 2026-11-21) |

The proxy solids (three plates, four arms, the ESC, the pack, four motors) sit at their meshes' centroids with the meshes' second moments; the rest of the table's mass is one tensor record with no solid. A first version with bounding-box proxies left a remainder that was not a rigid body (`compile_airframe`: the tensor breaks the triangle inequality), and a motor stand-in 8 mm too long was part of the cause: the motors are now the table's own cylinders as part records.

## The checks (film box 252 x 237 x 284 at 1.25 mm, 17.0 M cells; held by `d1_fpv_hold(0.0436)`, y 0.00875 m: the held point is the design origin)

| Check | Result |
|---|---|
| frame in the air (CPU, 12 s) | 13 parts, 13 walls, 0 sheets, no part drawn only, 101,728 solid cells |
| 600 steps with the four lines and the solids (GPU, 25 us a step, 261 s; **run at the chain trim, 973.7 rad/s, not 941.0**: the near-field test takes its controls from the case) | no fault; 386 ms a step; device 3,569 MiB; pressure unconverged 0 of 601 steps, loose 267 (largest max\|r\|/max\|b\| 9.7e-5); subgrid viscosity: nothing bound |
| four rotors at 941.0 rad/s (**not the film's speed**), open air, no frame, 0.05 s (GPU, 293 s, 146 ms a step, 1,888 MiB in use) | last revolution, of W/4 = 1.389 N: FR 0.843, FL 0.841, RL 0.839, RR 0.839 (mean 1.167 N, 0.840); rear - front -0.0046 N (-0.4%); disc downflow 1.035 to 1.057 of momentum theory. The 84% was this speed's figure and is not the film's |
| **four rotors at 973.704 rad/s (9,298 rpm, the film's speed), open air, no frame, to 0.20 s** (GPU 1,228 s = 20.5 min, 153.4 ms a step, process VRAM 1,886 to 1,888 MiB, card-wide peak not sampled; `of-gate-973.ps1`, sha256 3ab2b0f2...1c93; **the exe was built at `ed6946ad`**, the worktree was at `987ce0e1`; gtest PASSED 1 test, `=== END four-rotor gate 973.704 exit 0`) | mean last-revolution thrust, of W/4: 0.778 at 0.01 s, 0.871 at 0.02, 0.897 at 0.03, 0.905 at 0.04, 0.900 at 0.05, 0.894 at 0.10, 0.904 at 0.15, **0.904 at 0.20** (1.256 N); from 0.05 s on it stays between 0.891 and 0.906 with no trend. At 0.20 s FR 0.903, FL 0.903, RL 0.910, RR 0.901 (spread 1.0%), rear - front +0.0033 N (+0.3%); rear - front ranged -2.8% (0.035 s) to +1.0% while the wake formed and stays within +1.0% after 0.05 s. Disc downflow at 0.05 s 1.032 to 1.058 of momentum theory (not printed later). Tip speed 61.83 m/s, one revolution 6.45 ms (258 steps), projection residual 6.90e-3, capped, unconverged and stalled 0 |

## What the card should say

- **At the film's speed, 9,298 rpm (973.704 rad/s), the lines carry about 90% of a quarter of the weight each** (0.904 at 0.20 s; 0.89 to 0.91 from 0.05 s on): the thrust had settled by about 0.05 s. The earlier "84% in a 0.05 s check" was the 941.0 rad/s gate and is retired. The generic fpv_5inch at 8,735 rpm gave 0.90 and the X500 0.86 to 0.89. Every speed-derived figure on the card comes from 973.704 rad/s: 9,298 rpm, one rotor turn 6.45 ms, tip speed 61.83 m/s; "above the 6,057 rpm it was measured at" holds (9,298 is 1.53 times it).
- **The film's window:** the solve runs from t = 0 with the air at rest and the rotors at speed from the first step; it is shown from step 8,000 (0.200 s) to step 9,400 (0.235 s) at 25 us a step (step n is at n x 25 us; Rendering's run record logged step 600 at t = 0.015 s). Why the wake is formed by then: the discs sit 0.2005 m above the floor of the box and the air under them moves at 6 to 14 m/s, so a pass takes about 15 to 35 ms and 0.20 s is six or more; and the thrust of all four lines had settled by 0.05 s (0.900 at 0.05 s, 0.904 at 0.20 s). This is the thrust, in open air, not a measurement of the wake or of the frame's effect on it. Card wording: "air at rest, rotors at their hover speed from the first step; shown from about 0.20 s to 0.235 s of simulated time".
- The four rotors do not interfere beyond a 0.4% front/rear difference; the front and rear kernels overlap by about 3 mm in their tails (12.1 mm between the discs).
- The propeller is a stand-in. The check has no frame in it (the frame was checked for the air taking it, not for what it does to the thrust).

## Dated correction, 2026-11-23: the polar, its kernel, and the 2-cell figures above

The propeller row's "the polar was fitted at 6,057 rpm with a 12 mm kernel" is the library's fit and no longer what the film runs. The film's polar is its own fit (polar C: alpha_0 -4.3344, cd_scale 2.0, declared kernel 2.97 cells = 3.7 mm at R/dx 51), recorded in `docs/design/physics/records/2026-11-23-film-polar-fit.md`; a part's declared kernel is held by `docs/design/physics/records/2026-11-23-fit-kernel-declaration.md`. The thrust figures above (the lines carry about 0.9 of a quarter of the weight at hover; the X500's 0.86 to 0.89; the 0.778 to 0.904 four-rotor gate figures) were measured with the line's kernel at **2 cells and the library's polar**. They are history for that setting and say nothing about the film's setting (2.97 cells, polar C); the figure at the film's setting is not yet measured, and until it is the card must not quote these.
