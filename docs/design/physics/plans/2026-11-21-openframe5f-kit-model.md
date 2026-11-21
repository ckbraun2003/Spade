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
| hover | 8,986 rpm (941.0 rad/s) | the grid-free balance for a quarter of the weight, 1.389 N |

The proxy solids (three plates, four arms, the ESC, the pack, four motors) sit at their meshes' centroids with the meshes' second moments; the rest of the table's mass is one tensor record with no solid. A first version with bounding-box proxies left a remainder that was not a rigid body (`compile_airframe`: the tensor breaks the triangle inequality), and a motor stand-in 8 mm too long was part of the cause: the motors are now the table's own cylinders as part records.

## The checks (film box 252 x 237 x 284 at 1.25 mm, 17.0 M cells; held by `d1_fpv_hold(0.0436)`, y 0.00875 m: the held point is the design origin)

| Check | Result |
|---|---|
| frame in the air (CPU, 12 s) | 13 parts, 13 walls, 0 sheets, no part drawn only, 101,728 solid cells |
| 600 steps with the four lines and the solids (GPU, 25 us a step, 261 s) | no fault; 386 ms a step; device 3,569 MiB; pressure unconverged 0 of 601 steps, loose 267 (largest max\|r\|/max\|b\| 9.7e-5); subgrid viscosity: nothing bound |
| four rotors at 941.0 rad/s, open air, no frame, 0.05 s (GPU, 293 s, 146 ms a step, 1,888 MiB in use) | last revolution, of W/4 = 1.389 N: FR 0.843, FL 0.841, RL 0.839, RR 0.839 (mean 1.167 N, 0.840); rear - front -0.0046 N (-0.4%); disc downflow 1.035 to 1.057 of momentum theory |

## What the card should say

- The lines carry 0.84 of a quarter of the weight at hover after 0.05 s; the thrust was still settling (the mean was 0.843 at 0.04 s and 0.811 at 0.02 s), so this is not a balance. The generic fpv_5inch at 8,735 rpm gave 0.90 and the X500 0.86 to 0.89.
- The four rotors do not interfere beyond a 0.4% front/rear difference; the front and rear kernels overlap by about 3 mm in their tails (12.1 mm between the discs).
- The propeller is a stand-in. The check has no frame in it (the frame was checked for the air taking it, not for what it does to the thrust).
