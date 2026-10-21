# Physics — verification and grades

**Owner:** Physics for what is checked and the bands' content. Test/Docs owns the harness, golden-corpus governance, the SPIR-V scanner and the gates (`../test-docs/`). **Normative.**

## How a physics change is checked

1. **Against closed forms in double.** Each model's reference values are recomputed in the test from the header's equations, never read back from a run:
   - momentum-theory roots for the rotor;
   - Dryden coefficients at the branch thresholds;
   - ballistic and energy behaviour for the integrator;
   - SDF distances.
   `test_rotor.cpp` is the model: it checks the defining equations, then independent closed forms, then the whole pass.
2. **Physics validation:**
   - a ballistic drop;
   - symplectic energy behaviour;
   - hover trim (thrust = weight gives station-keeping);
   - climb and descent against momentum theory;
   - Dryden spectral sanity.
3. **The golden corpus** (`tests/golden/scenarios/`): CPU-golden, bit-identical per platform. A moved digest is a deliberate physics change or a bug, and the two look identical in a diff. **Understand it before regenerating**, and record why in that scenario's provenance block (the governance rules are Test/Docs's).
4. **CPU↔GPU parity** (`engine/testing/parity.hpp`, `TD-14`): per-scenario bands on positions, velocities, orientations, angular rates, specific force and the sensors. An element passes when `|gpu − cpu| ≤ abs + rel·|cpu|`. Bands are **measured on each device of record** (the RTX 3060 Ti since 2026-10-05; the Iris Plus numbers stay as history), pinned at 4× rounded up to one significant figure with an `L4` record beside each, and **never widened** without a named cause (`TD-2`). A float row keeps a zero band only when it is argued structural; one measured exact gets the floor `{5e-10, 5e-7}`. Errors split at a near-zero cutoff of 1e-3 in the quantity's unit, except specific force and the IMU accel that reads it, which take 1 m/s²: the contact term Δv/h is quantized at about 1e-4 m/s², so relative error below 1 m/s² measures that, not the port (the lead, 2026-10-05). Their floor is `{5e-7, 5e-7}`. Element bands hold up to each scenario's horizon, where one ulp amplified by the CPU alone stays under 1e-5 of the scene's scale (1 m, 1 m/s, and g for specific force); past it, invariants and chaos-banded statistics, the RMS specific force among them (`gate_fleet`, `heterogeneous_geometry_set`, `contact_pair`, and `shower_ladder`'s specific force).
5. **Invariance:** solo vs in-set batching and workgroup sizes `{32, 64, 128}` are bit-identical on one device; a snapshot restores across CPU and Vulkan exactly, and the continuation is banded (`test_gpu_invariance.cpp`).

**GPU parity runs only where the gate has a device** (`TD-13`). On the development box all `gpu` tests run inside `scripts\test.ps1`, and a skipped one is reported by name and does not count toward a green gate. The Docker leg excludes them (`-LE gpu`), so it says nothing about agreement. Anyone adding or touching a GPU physics path runs `ctest -L gpu` on real hardware and says so in the commit.

## Grades (`PHY-2`)

`L3`/`L4`: reference needs a CPU implementation and a golden; banded needs a measured band against the reference, with its record. The GPU is never a golden source.

| Module | CPU | Vulkan | Evidence |
|---|---|---|---|
| Integrator | reference | banded | `ballistic` golden; `parity::ballistic` |
| Static contact | reference | banded | `bounce` golden; `parity::bounce` |
| Dynamic contact, Gauss-Seidel | reference | banded | `shower` golden; `parity::shower`, `contact_pair` |
| Drag (T0) | reference | banded | `quad_hover` golden; `parity::drag_componentwise` |
| Rotor element | reference | banded | `quad_hover` golden; `parity::quad_hover` |
| Dryden gust | reference | banded via `CORE-3` | `quad_hover` (moderate); `parity::medium` |
| IMU | reference | banded via `CORE-3` | `quad_hover`'s IMU; unit tests |
| GNSS | reference (`PHY-6`); the golden is final, reproduced by the Docker gcc leg (`TD-12`, `fe4934a`) | banded via `CORE-3` | `gnss_tumble` golden; `parity::gnss_tumble`, `gnss_receiver`, `gnss_receiver_body` |
| Dynamic contact, Jacobi | not placed | not placed | built and unit-tested, not in the schedule |
| Rotor wake | best-effort, CPU only, not read by stepping (`PHY-3`) | — | `test_rotor_wake.cpp` |
| Drone builder: motor, battery, propeller | not placed: pure functions, unit-tested; reference with a golden once the rows exist (`DBP-60`) | not placed; banded once the kernels exist (`DBP-61`) | `test_motor.cpp`, `test_battery.cpp`, `test_propeller.cpp` |
| Drone builder: composite inertia, steady-state solver | reference by closed forms; host utilities, never in the step | — | `test_composite_inertia.cpp`, `test_propulsion_steady.cpp` |
| Airflow solver core, standalone (`PHY-9`, plan A) | reference: two goldens, `kSolverCoreDigest` (the walled cavity; the gcc leg reproduced it, `742f56e`) and nine boundary configurations (`test_airflow_configs.cpp`; the gcc leg reproduced it, `4622f84`) | not placed: the GPU twin is plan B | `test_airflow_step.cpp`, `test_airflow_configs.cpp`; V1 below |
| Drone builder: the airframe compile (`DBP-44`) | mass, inertia and mounts: reference by closed forms. The fitted momentum rotor and the drag estimate: best-effort | — | `test_airframe_compile.cpp`; the τ fit is checked against a stepped response |

A world's grade is its weakest module's (`L3`). Until the grade check exists (Core), this table is the declaration.

## Validation (`PHY-9`, the airflow spec's §11)

Against published data, beside the goldens. Each case's tolerance was fixed in plan A's Task 11 before its first run (spec §11.2). Each record is in `tests/data/airflow/records/`, in Test/Docs' format. The reference data, Ghia, Ghia & Shin (1982) at Re 100 and 400, have their provenance in `tests/data/airflow/`, reviewed by Test/Docs. The opt-in cases run with `SPADE_AIRFLOW_VALIDATION=1`.

**V1** (plan A, 2026-10-12, CPU reference, MSVC release):

| Case | Measure | Result | Tolerance | Verdict | Record |
|---|---|---|---|---|---|
| C1 lid-driven cavity, Re 100, 32² (in the gate) | max \|u − u_Ghia\| on x = 0.5; max \|v − v_Ghia\| on y = 0.5 | 0.0064; 0.0041 | ≤ 0.06 | within | `c1-cavity.md` |
| C1, Re 100, 128² (opt-in) | the same | 0.0037; 0.0087 | ≤ 0.02 | within | `c1-cavity.md` |
| C1, Re 400, 128² (opt-in) | the same; Table II's point 117 left out of v, as decided before any run | 0.0076; 0.0065 | ≤ 0.02 | within | `c1-cavity.md` |
| C0 Taylor–Green vortex, 64³, inviscid (opt-in) | kinetic energy per unit mass, t = 0 to 10 | 0.125 to 0.0701 | none | report only (DNS data enter at V3) | `c0-taylor-green.md` |
| C8 actuator disc's far wake (opt-in) | axial velocity on the axis 2R below | 7.4124 m/s against momentum theory's 7.5777 | within 10% | within | `c8-actuator-disc.md` |

**P2** (solids, 2026-10-20, CPU reference, MSVC 14.43 release; tolerances in `docs/design/test-docs/2026-10-13-solids-tolerances.md`, fixed before the first run, except where marked `TD-2`):

| Case | Measure | Result | Tolerance | Verdict | Record |
|---|---|---|---|---|---|
| S1 sphere in a uniform stream, R/dx 6, step 1 from cold (in the gate) | closed faces off their wall velocity; the solve's relative residual; the aperture divergence left over the right side's | 0; 3.509e-6; 3.274e-6 | exact; ≤ 1e-4; ≤ 2e-4 (P2 plan, Task 7) | within | `s1-sphere.md` |
| S2 circular Couette, R1/dx 16 (release gate and every leg; debug skips it under `TD-5`) | max \|u_θ − u_θ,exact\| / (Ω R1); the last step's residual | 0.0052; 1.715e-6 | ≤ 0.11; ≤ 1e-4 | within | `s2-couette.md` |
| S2, R1/dx 32 (opt-in) | the same; e(32) / e(16) | 0.0022; 0.429 | ≤ 0.054; report only, as e(16) is under a tenth of its band | within; report only | `s2-couette.md` |
| Planar Couette, plates a quarter cell off the faces (in the gate) | max \|u − u_exact\| / U with the wall links; closed faces only, the negative control | 5.624e-8; 6.459e-2 | ≤ 1e-5 (`TD-2`); the control must exceed it | within; the control fails, as it must | `s2-couette.md` |
| S3 sheet across a channel, an even plane and an odd one (in the gate) | max \|u\| / U; the pressure step Δp h / (ρ U (L − dx)) | 7.987e-6 and 4.783e-6; 1.000002 and 1.000001 | ≤ 1e-3; 1 ± 1e-3 | within | `s3-sheet.md` |
| Design test 7: the posed drone's body from A4 and from B4 (in the gate) | max \|θ_A − θ_B\|; the worst sample's \|φ_A − φ_B\| in ulps of max(\|φ_A\|, \|φ_B\|, dx); wall velocities; faces closed in one form and open past θ's band in the other; closed faces, A4 and B4 | 2.980e-8; 0.05 ulps; 0; 0; 4336 and 4336 | ≤ 1.2e-7 (`TD-2`); ≤ 4 ulps per sample (`TD-2`); exact; exact; report only | within; report only | the test's print (a5a1b4c; a box 1 µm wider misses at θ 1.664e-4 and φ 2144 ulps) |
| The chosen count, the sheet from cold on both planes (in the gate) | CG iterations to 1e-3; the residual after `kSolidsCgIterations` (8) | 5 and 5; 2.551e-5 and 2.113e-5 | ≤ 8; ≤ 1e-4 | within | the test's print |

gcc-13.3 reproduced the planar Couette (5.624e-8, and the control's 6.459e-2) and design test 7's θ (2.980e-8) and closed faces (4336 and 4336) digit for digit, in P2's leg at `4e53383` (2097 passed, 0 failed). Design test 7's φ passed its 4-ulp band there, though the leg's JUnit file cut its record short of the ulp count.

S2 checks the solution, not the wall treatment. It passes at 16 with the viscous wall links (0.0052) and without them (0.0602). The planar Couette check is the one that tells the treatments apart.

## Writing a physics test

- Derive the expected value from the model's equations, in double. A transcription of the implementation into the test agrees with the implementation's bugs.
- Pin the degenerate inputs (zero, negative, NaN, overflow) to the documented answer. Totality is a tested claim, not a comment.
- A GPU case is `Gpu*`-named, starts with `if (!compute::vulkan_available()) GTEST_SKIP()`, and is run on hardware before it is committed.
