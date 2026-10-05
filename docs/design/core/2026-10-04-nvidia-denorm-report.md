# NVIDIA fp32 denormals on the RTX 3060 Ti — report

**Owner:** Core, with Physics (M1) and Test/Docs (the flag). **Status:** DRAFT for the lead, 2026-10-05. Not reviewed, not committed. Its home is `docs/design/core/2026-10-04-nvidia-denorm-report.md`. **Plan:** `plans/2026-10-04-nvidia-denorm-measurement-plan.md`. **Ruling:** the user's "Measure NVIDIA first" (2026-10-04). This report changes no rule.

## Summary

The broadphase cell flip comes from the division `pos / cell_size`, whose subnormal quotient flushes to -0, so floor gives -0 and the cell is 0, not -1, for |pos| < 2^-126 × cell_size. floor itself preserves. That is M1's finding 3, and it shows the shape of the whole device: with no denormal mode requested, loads and stores keep subnormals, and every arithmetic op flushes, as a sign-preserving FTZ+DAZ. On the real kernels (M2), 72 of 84 `Gpu*` tests pass and 12 fail. Four fail on denormals, all at `fp32_math.slang`'s three documented denormal dependencies: exp32's subnormal results flush to +0, and log32's pre-scale and sin32's reduction read a subnormal argument as zero. The other eight are not denormals. Six parity tests, and the two invariance tests that repeat one of them, fail on NVIDIA's native division and square root on normal operands (M1's finding 4), measured against bands pinned on the Intel Iris. A port of the CPU path with M1's flush rule changes no state bit in any of those scenarios. The same port with NVIDIA-style division reproduces two_world_isolation's worst elements bit for bit. No failure is unexplained, and no M2 test reaches the cell flip. The evidence rules out bit-identity, so this report proposes no admission rule. Core recommends keeping the refusal, making fp32_math's three denormal dependencies exact in integer arithmetic, and taking a banded NVIDIA admission to Cameron with module-API stage 5.

## Setup

- **Device:** NVIDIA GeForce RTX 3060 Ti. Driver 572.83 (`driverVersion` 0x8F14C000). Vulkan 1.4.303. Windows 10 Home 10.0.19045.
- **Float controls:** `denormPreserveF32=0`, `denormFlushToZeroF32=0`, `roundingModeRTEF32=1`, `signedZeroInfNanPreserveF32=1`, `denormBehaviorIndependence=1`. `shaderInt64=1`.
- **Build:** `spade-wt/core/build-ninja/release-unpinned`. Release, Ninja, MSVC 14.43.34808, `SPADE_MEASURE_UNPINNED_DENORMS=ON`.
  - Kernels by the pinned slangc (v2026.14.1), with `-fp-mode precise` and without `-denorm-mode-fp32 preserve`.
  - The SPIR-V gate's rule P3 is inverted: no module carries `DenormPreserve 32`.
  - The context skips only the denormal clause, and says so on every create: "admitted with the denormal mode not requested".
- **Commits:**
  - Measurement merge: core `8258e4c` + test-docs `d9b4cb8`, on master as `cc63c97`.
  - M1 probe: `physics/denorm-probe` `650310b` (fix `ef12d32`) on `test-docs/denorm-flag` `d9b4cb8`, run on a local throwaway merge with Core's admission.
  - The kernels, `engine/physics`, `world/sdf.cpp`, `engine/core`, `parity.hpp` and the three gpu test files are unchanged from `cc63c97` to master `39ae212`.
- **Raw output:** M1 `data/2026-10-05-nvidia-denorm-m1.txt`. M2 `data/2026-10-04-nvidia-denorm-m2.log` (`--gtest_filter=Gpu*`, 84 tests, 999 lines, 78 `MISMATCH` lines). Two of the failing parity tests were re-run on 2026-10-05 and printed identical tables.

## M1: the op classes (Physics)

**Table 1. M1 on RTX 3060 Ti, driver 572.83 (0x8F14C000).** No fp32 denormal mode, `-fp-mode precise`. Each case is compared bit for bit with the CPU twin (x86 SSE, FTZ and DAZ off).

| class (572.83) | cases | control | kept | in-ftz | out-ftz | either | both | contr | acc | other | inputs | outputs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pass-through | 30 | 18 | 30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | preserved | - |
| add | 900 | 316 | 646 | 166 | 8 | 80 | 0 | 0 | 0 | 0 | FLUSHED | FLUSHED |
| sub | 900 | 316 | 646 | 166 | 8 | 80 | 0 | 0 | 0 | 0 | FLUSHED | FLUSHED |
| mul | 900 | 312 | 728 | 64 | 12 | 96 | 0 | 0 | 0 | 0 | FLUSHED | FLUSHED |
| abs | 30 | 18 | 20 | 10 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| neg | 30 | 18 | 20 | 10 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| min | 900 | 324 | 600 | 300 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| max | 900 | 324 | 600 | 300 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| mul-then-add | 27000 | 5660 | 18172 | 3282 | 286 | 5230 | 30 | 0 | 0 | 0 | FLUSHED | FLUSHED |
| floor | 30 | 18 | 30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | preserved | - |
| floor->int | 30 | 18 | 30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | preserved | - |
| compares | 900 | 324 | 590 | 310 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| clamp | 13980 | 3078 | 7510 | 6470 | 0 | 0 | 0 | 0 | 0 | 0 | FLUSHED | - |
| sqrt | 30 | 18 | 19 | 10 | 0 | 0 | 0 | 0 | 1 | 0 | FLUSHED | - |
| div | 900 | 296 | 444 | 360 | 28 | 60 | 0 | 0 | 8 | 0 | FLUSHED | FLUSHED |

"-" means the class cannot tell an output flush apart. The run passes all four of the probe's rules: no "other", no differing control case outside div and sqrt, no empty class, no contraction.

Physics' findings, in short:
1. Loads and stores keep subnormals. Every arithmetic op flushes, as a sign-preserving FTZ+DAZ: add, sub, mul and div on inputs and outputs; abs, neg, min, max, clamp, compares and sqrt on inputs. `-denorm < 0` is false, `abs(-denorm) = +0`, `max(denorm, 0) = 0`, `sqrt(denorm) = 0`.
2. NoContraction is honoured: 0 of 27,000 mul-then-add cases match `std::fma`.
3. floor and float->int keep subnormals. The cell flip comes one step earlier, from the division (see the summary).
4. Not denormals: division and square root are not correctly rounded on NORMAL operands. 8 division cases and 1 square-root case are 1 ulp off. `(1+2^-12)/3` gives 0x3EAAB556 against the host's 0x3EAAB555; `sqrt(1.5 × 2^-126)` gives 0x201CC470 against 0x201CC471.
5. M2's `log32(denorm_min)` fits finding 1: log32's pre-scale multiplies a subnormal, and the multiply flushes its input.

## M2: the gpu suite

**Table 2. M2 on RTX 3060 Ti, driver 572.83 (0x8F14C000), Vulkan 1.4.303: 84 run, 72 passed, 12 failed.** For GpuFp32Math the last column is the first mismatch the test printed. For the parity tests it is the worst element at the final tick: the harness does not report when two runs diverged.

| # | Test | What failed (572.83) | First mismatch, or worst element at the last tick |
|---|---|---|---|
| 1 | `GpuFp32Math.Exp32MatchesTheHostBitForBitOverTheLargestQuotientsAndTheSubnormalTail` | 2,180,453 of 10,485,762 | [6117537] arg 0xC2AEAC50 (-87.3365479): gpu 0x00000000, host 0x007FFFE6 |
| 2 | `GpuFp32Math.Exp32MatchesTheHostBitForBitAcrossTheWholeFiniteDomainAndBothSaturationEdges` | strided 8,517 of 8,757,250; underflow window 4,096 of 8,193; overflow window 0 of 8,193 | [8740187] arg 0xC2AEAD00 (-87.3378906): gpu 0, host 0x007FD3EE. Window [0] arg 0xC2CFE1B5 (-103.940834): gpu 0, host 0x00000001 |
| 3 | `GpuFp32Math.Exp32MatchesTheHostBitForBitOverTheCorpusLiveArgumentsAndADeterministicLattice` | corpus-live 0 of 6; lattice 79,978 of 1,000,001 | [135] arg 0xC2CFF19F (-103.971916): gpu 0, host 0x00000001 |
| 4 | `GpuFp32Math.EdgeCasesAreBitIdenticalToTheHost` | log32 5 of 19; exp32 7 of 29; sin32 2 of 29; cos32 0 of 29 | log32 [8] arg 0x00000001: gpu 0xC2D15498 (-104.665222), host 0xC2CE8ED0 (-103.278931). exp32 [13] arg 0xC2CFF1B4: gpu 0, host 0x00000001. sin32 [27] arg 0x00000001: gpu 0, host 0x00000001; [28] arg 0x80000001: gpu 0x00000000, host 0x80000001 |
| 5 | `GpuParityTest.GnssBiasUnderAnUnderflowedRetentionIsAPureFunctionOfTheDraws` | `gnss_sensors/bias`, band (0, 0): 1 of 1 | element 0 component 0: cpu 8.730303645e-01, gpu 8.730303049e-01 (1 ulp) |
| 6 | `GpuParityTest.GateFleetMatchesTheCpuWithinBands` | `pos`, band (2e-6, 1e-5): 1 of 20 | element 0 component 0: cpu 2.479006797e-01, gpu 2.478974760e-01 (abs 3.20e-06) |
| 7 | `GpuParityTest.TwoWorldIsolationMatchesTheCpuWithinBands` | `pos`, band (0, 0): 8 of 32. `vel`, band (5e-28, 2e-4): 8 of 32 | pos element 6 component 1: cpu 3.584776819e-01, gpu 3.584775925e-01. vel element 4 component 1: cpu -8.405701257e-03, gpu -8.405704051e-03 |
| 8 | `GpuParityTest.TwoActiveBodiesInAWorldStepAndMatchTheCpu` (contact_pair) | `vel`, band (0, 0): 2 of 5. `pos` bit-exact | element 0 component 1: cpu 1.564665079e+00, gpu 1.564664125e+00 (8 ulp) |
| 9 | `GpuParityTest.RestoredRunResumesAndMatchesTheCpuWithinBands` (restore_resume) | `pos`, band (0, 0): 2 of 10. `vel`, band (1e-7, 0): 2 of 10 | element 5 component 1: pos 3.267495632e+00 against 3.267495155e+00; vel 2.049585432e-01 against 2.049568743e-01 |
| 10 | `GpuParityTest.HeterogeneousGeometrySetMatchesTheCpuWithinBands` | `pos`, band (2e-6, 1e-5): 1 of 2. `vel`, band (1e-5, 1e-5): 1 of 2 | element 1 component 0: pos 7.023340464e-01 against 7.023205161e-01; vel 1.771813512e+00 against 1.771769166e+00 |
| 11 | `GpuInvarianceTest.CpuSnapshotRestoresIntoVulkanAndContinuesWithinBands` | restore_resume's bands: `pos` 2 of 10, `vel` 2 of 10 | identical to row 9, to the last digit |
| 12 | `GpuInvarianceTest.VulkanSnapshotRestoresIntoCpuAndContinuesWithinBands` | restore_resume's bands: `pos` 2 of 10, `vel` 2 of 10 | element 5 component 1: the vulkan reference 3.267495155e+00 against vulkan-then-cpu 3.267495632e+00. vel max rel is 1: a component the vulkan run holds non-zero where the CPU holds 0 |

Everything else passed. That includes run-to-run determinism (QuadHover, Shower), all seven workgroup-size invariance tests, the batching-invariance test, the six grid-sort tests, and the eleven GL renderer tests (Rendering's `raster_background` divisions stay inside their bands). Away from the subnormal tail, GpuFp32Math matched the host bit for bit: sin32 and cos32 over all 2^24 Box-Muller angles each, log32 over more than 17 million arguments, and 9,437,184 correctly rounded divisions by `log32_div`.

## M3: each failure mapped to a class

**Table 3. M3 map, driver 572.83 (0x8F14C000).**

| # | Test | Class | Kernel line | Confidence |
|---|---|---|---|---|
| 1-3 | the three exp32 sweeps | denormal: mul, output flushed | `fp32_math.slang:490` (k = -126: `m * pow2(k)` with m < 1) and `:502` (k <= -127: `(m * pow2(k + 64)) * 0x1.0p-64`) | traced |
| 4 | `EdgeCasesAreBitIdenticalToTheHost` | denormal, three ways. log32: mul, input flushed. exp32: as rows 1-3. sin32: sub, input flushed | log32 `fp32_math.slang:430` (`x * 0x1.0p24`); exp32 `:490`, `:502`; sin32 `:386` (`x - n * kPio2A`), then `:348` | traced |
| 5 | GnssBias | accuracy (finding 4): native sqrt | `rng.slang:214` (Box-Muller `sqrt(-2 * log32(u1))`) | inferred, by elimination |
| 6 | GateFleet | accuracy (finding 4); denormal excluded | not pinned. Candidates: `sdf_eval.slang:196-197` (torus, two sqrt per evaluation, seven evaluations per sample), `collision_static.slang:153`, `:157`, `:179`, `:182` | inferred; denormal excluded by the port |
| 7 | TwoWorldIsolation | accuracy (finding 4): native division | `collision_dynamic.slang:177` (`n = d / dist`); the rest likely the sqrt at `:173` | traced for the worst elements; rest inferred |
| 8 | contact_pair | accuracy (finding 4); denormal excluded | not pinned. Likely late, at the torus crown: free fall brings the bodies to it near tick 545, they are rising at 1.56 m/s at tick 600, and `pos` is still bit-exact while `vel` is 8 ulp off. Candidates as row 6, plus `collision_dynamic.slang:173`, `:177` | inferred; denormal excluded by the port |
| 9 | restore_resume | accuracy (finding 4); denormal excluded | not pinned. World 1's body bounces on a post top from about tick 534. Candidates: `sdf_eval.slang:255`, `:267` (box gradient), `collision_static.slang:153-182` | inferred; denormal excluded by the port |
| 10 | Heterogeneous | accuracy (finding 4); denormal excluded | not pinned. Same body and geometry as gate_fleet's world 0. Candidates as row 6 | inferred; denormal excluded by the port |
| 11-12 | the two cross-backend restores | echo of row 9 | as row 9 | traced: identical numbers |

Notes on the map:
- **Rows 1-3, how they were traced.** Each test's mismatch count equals the number of its arguments whose host result is a non-zero subnormal, exactly. The tail runs from 0xC2AEAC50 (-87.3365479, host 0x007FFFE6) to 0xC2CFF1B4 (-103.972076, host 0x00000001). That gives 0x42CFF1B4 - 0x42AEAC50 + 1 = 2,180,453; 8,517 of the stride-256 arguments; 4,096 in the underflow window; and lattice points 135 to 80,112, which is 79,978. Every printed mismatch is gpu +0 against a subnormal host.
- **Row 4, log32.** The flushed product is +0, so its exponent field reads 0: e = -24 - 127 = -151 and m = 1. The device answers -151 × ln 2 = -104.665222 for every subnormal argument, which is what it printed.
- **Row 4, sin32.** n is 0 on both sides, so `x - n * kPio2A` should return x. DAZ makes it ±0. `sin_kernel` then computes `r + r * (r2 * poly)`, which is +0 for both signs (-0 + +0 = +0). The host returns exactly x.
- **Row 5, by elimination.** The retention is exactly 0 by exp32's `x <= -104` guard (`fp32_math.slang:474`), with no subnormal on the way. So `bias = 0.8 × walk` (`sensor_gnss.slang:158`). log32, sin32 and cos32 match the host on this device over every argument the draw can form (rows passed in M2). The multiplies are on normal operands. That leaves the sqrt. This is `CORE-3`'s declared divergence. The test compares one draw, the last; on the Iris that draw happened to agree. `GnssDrawsDivergeAcrossBackends_KNOWN_OPEN` passed on both devices: 11 of 64 fixes differ on the Iris, 19 of 64 here.
- **Rows 11-12, the echo.** Same scenario, seed and window as row 9. The final value depends only on which backend ran ticks 300-700: CPU throughout and vulkan-then-CPU end at 3.267495632; vulkan throughout and CPU-then-vulkan end at 3.267495155. So the restore itself is exact in both directions.
- **Unexplained failures:** none.

### How the parity rows were traced

The parity harness prints only the final tick's worst element, so the kernel log cannot show where a run first diverged. Core wrote a throwaway Python fp32 port of the pos/vel path: the SDF (plane, torus, box, union, transforms), static contacts, the grid and the Gauss-Seidel pair sweep, and Integrate. It follows the kernels statement for statement. It reproduces 11 logged CPU values bit for bit (listed in table 4). Each scenario then ran in three ways:
- IEEE with subnormals: the CPU twin. Every op with a subnormal operand or result is counted, by kernel line.
- M1's rule on every arithmetic op: a subnormal operand or result becomes a zero of its sign. Loads, stores, floor and float->int are left alone.
- NVIDIA-style division, `a/b` computed as `a × fl(1/b)`. This model reproduces all 284 of M1's division controls with a finite quotient, including the 8 that differ, with the device's exact bits. It is a model, not the driver's documented algorithm.

**Table 4. What the port shows. Device facts from M1 and M2 on driver 572.83 (0x8F14C000).**

| Scenario | CPU values reproduced bit for bit | Subnormal operands or results on the CPU path | M1's flush rule changes a state bit | `a × fl(1/b)` division reproduces the device |
|---|---|---|---|---|
| two_world_isolation | pos[6].y, vel[4].y (log); vel[4].x = 1.626475682e-24 (`parity.hpp`'s Iris note) | 92 squares in the friction length: `collision_static.slang:179` ×32, `collision_dynamic.slang:218` ×60. The first is at tick 274, world 0 slot 2: v_t.x = -3.24e-21, squared 1.05e-41 | no. All 92 flushes are absorbed | **yes, from `collision_dynamic.slang:177` alone**: the worst pos and vel elements and both max |abs| (8.94069672e-08, 2.79396772e-09). Not every element: 2 pos elements differ in the port, 8 on the device |
| gate_fleet | pos[0].x (log); vel[5].y = 1.358585153e-02 (`parity.hpp`) | none | no | no |
| contact_pair | vel[0].y (log) | none | no | no: no state bit changes |
| restore_resume, ticks 300-700 | pos[5].y, vel[5].y (log); vel[5].x = 0 (`parity.hpp`) | none | no | no. It reproduces the Iris's own residue instead: vel[5].x = -1.490116119e-08, as `parity.hpp` records for the Iris |
| heterogeneous_geometry_set | pos[1].x, vel[1].x (log) | none | no | no: one 1-ulp change, not the device's |

## Findings

1. **The device is a uniform sign-preserving FTZ+DAZ on arithmetic.** Loads, stores, floor and float->int keep subnormals. NoContraction is honoured (M1).
2. **Every denormal failure in M2 is in `fp32_math.slang`, at its three denormal dependencies:** log32's pre-scale (`:430`), exp32's subnormal tail (`:490`, `:502`) and sin32's reduction (`:386`). The first two are the ones the module's own comments name.
3. **No parity failure is a denormal.** In four of the five failing physics scenarios no subnormal occurs anywhere on the pos/vel path, and GnssBias has none on its path either. In two_world_isolation, 92 squares in the friction length go subnormal from tick 274 and flush, and no state bit moves.
4. **Finding 4 is independent of denormals, and it explains all eight remaining failures.** two_world_isolation is traced to one division, `collision_dynamic.slang:177`. For any NVIDIA path, this means:
   - it cannot be bit-identical to the CPU twin while the kernels use the native `/` and `sqrt`, whatever is done about denormals;
   - today's bands are Iris measurements, so they encode the Iris's own division and square-root rounding. The `a × fl(1/b)` model reproduces the Iris's restore_resume residue, so the Iris rounds like that in at least one place. NVIDIA's rounding lands outside nine band rows in six tests;
   - a software route exists. Spade's own correctly rounded division, `log32_div`, matched the host 9,437,184 times on this device.
5. **GnssBias is `CORE-3`'s sqrt band biting a zero-band test that samples one draw.** It is not new in kind.
6. **The cell flip (finding 3) is not reached by any M2 test.** It needs |pos| < 2^-126 × cell_size, about 2.8e-39 m at a 0.24 m cell. The division's accuracy can also flip a cell, for a position within an ulp of a cell boundary. In the port, `a × fl(1/b)` changed 12,440 quotients at `grid_build.slang:149` in two_world_isolation, but no cell.
7. **NVIDIA is deterministic against itself.** Run-to-run and workgroup-size invariance hold. The device differs from the CPU, not from itself.

## Operations that differ, and what each would take

The plan asks for this list when M2 is not green.

**Table 5. Driver 572.83 (0x8F14C000).**

| Operation (M1 class) | Where M2 meets it | What it would take |
|---|---|---|
| mul, output flushed | exp32's subnormal tail, `fp32_math.slang:490`, `:502` | Build the subnormal result in integer arithmetic: shift m's significand with round-to-nearest-even, as `log32_div` does. The host's results do not change, so no golden moves |
| mul, input flushed | log32's pre-scale, `fp32_math.slang:430` | Normalise a subnormal by its leading-zero count instead of multiplying by 2^24. The host multiply is exact, so the integer form gives the same bits |
| sub, input flushed | sin32's reduction, `fp32_math.slang:386` | Return x for \|x\| < 2^-126 before reducing. The host already returns exactly x there |
| mul output into add (friction length) | `collision_static.slang:179`, `collision_dynamic.slang:218`, two_world_isolation | Nothing for a banded grade: it is absorbed in every M2 run. For bit-identity, a threshold on \|v_t\|² on both twins, which may move goldens |
| div and sqrt rounding on normal operands | `collision_dynamic.slang:177` (traced), the SDF and contact chains, `rng.slang:214` | NVIDIA-measured bands (option 4), or software correctly rounded division and square root (option 5) |
| div into floor (the cell flip) | `grid_build.slang:149-151`; not reached | None today |

## Evidence limits

- **One card, one driver, one OS, one build.** Nothing here speaks for another NVIDIA card, another driver version, Linux, or another vendor.
- **M1 covers classes, not every op sequence.** It uses a fixed table of 15 magnitudes with both signs. Its accuracy rows are 8 of 296 division controls and 1 of 18 square-root controls. That shows the rounding differs. It is too little to model the square root.
- **M2 covers what the tests exercise.** The 72 passes include banded quantities that differ inside their bands. No test reaches the cell flip. The parity harness reports where runs ended, not where they diverged.
- **M2 is the stronger evidence.** It runs the shipped kernels as the driver compiles them in context, and the driver may pick different instructions there than in a probe. A select can become a flushing min; a division can become a reciprocal and a multiply. M1 explains M2's failures. Where they could disagree, M2 wins.
- **M3's port is a model.** It reproduces 11 CPU values bit for bit, and it applies M1's rule to every arithmetic op. It does not model the driver's compiler. Its division model fits M1's division controls but is not NVIDIA's documented algorithm, and no square-root model fit. The orientation and omega chains were not ported; their rows passed. The port was a throwaway and is not committed.

## Options

1. **Keep the refusal (status quo).**
   - Cost: on this box the GPU path's tests run only their CPU side. 72 of 84 `Gpu*` tests are refused in default builds; GPU changes are labelled, and re-verified once a device can run them (`TD-13`'s interim reading).
   - No engine change. The measurement build stays available for spot checks under its test-only flag.
2. **A CPU twin with FTZ and DAZ for NVIDIA.** x86 MXCSR does not reproduce M1's pattern in full:
   - matches: add, sub, mul and div flush inputs and outputs; sqrt flushes its input; compares and `minss`/`maxss` read subnormal inputs as zero;
   - does not match: abs and neg are bit operations on x86 (`andps`/`xorps`), which DAZ does not touch, so the CPU keeps `abs(-denorm) = +denorm` where NVIDIA gives +0. A min or max compiled as compare-and-branch returns the unflushed operand. If floor compiles to `roundss`, DAZ flushes its input, where NVIDIA's floor preserves (harmless in today's kernels, because floor's input is a quotient the division already flushed). x86 detects tininess after rounding; NVIDIA's boundary is unmeasured;
   - costs: MXCSR is per thread, so every thread that steps the twin sets and restores it, and it changes library code on that thread too. An ARM host's flush mode behaves differently again. The twin is a second reference, with its own goldens and fp32 unit-test expectations (`L4`). Devices that preserve, such as the Iris, still need today's twin, so the twin is chosen per device;
   - it fixes only the four fp32_math failures. All eight parity and invariance failures are finding 4 and stay.
3. **Keep subnormals out of the GPU path at their sources, on both twins.**
   - fp32_math: the three integer rewrites in table 5. No CPU result changes, so no golden moves. They remove all four denormal failures and make the module independent of the device's denormal mode. Cost: a small Core branch with red/green in the measurement build, re-verified on the Iris.
   - Physics: the friction length (`collision_static.slang:179`, `collision_dynamic.slang:218`). A threshold changes CPU results and may move goldens. Cost: Physics' branch and review. Not needed for a banded grade.
   - It does not touch finding 4, so the parity failures remain.
4. **Admit NVIDIA at a lower grade, with bands (module-API stage 5's grades).**
   - Six parity tests (nine band rows) already fail against the Iris bands, plus the two invariance tests that reuse restore_resume's. Four of those rows are zero bands: GnssBias `bias`, two_world_isolation `pos`, contact_pair `vel`, restore_resume `pos`.
   - Costs: stage 5 is not built (grades and the `create()` check are "No" in `07-status.md`). Kernels need a second build without `DenormPreserve 32`, selected at context creation, and rule P3 needs two profiles. NVIDIA needs its own bands, measured on NVIDIA with a record of how (`L4`). The exp32 tail tests need option 3 or a declared exception.
   - Driver change: the admission names the (device, `driverVersion`) pairs whose bands were measured. On any other driver, a banded request is refused (`L6`: no silent fallback), or the world runs best-effort if the caller allows it, and says so. A new driver is admitted by re-running the gpu suite and re-pinning the NVIDIA bands. Re-probing M1's classes at every `create()` would cost one pipeline and one small dispatch (tens of milliseconds, unmeasured). It would confirm the flush pattern but not the division and square-root rounding the bands depend on, so it is a check, not the rule.
   - This never claims bit-identity.
5. **Remove finding 4 at its source:** software correctly rounded division and square root on the parity paths, like `log32_div`.
   - Costs: each division becomes about 25 dependent integer steps (GPU cost unmeasured), at many sites in physics and the SDF. It reopens `CORE-3`, which refused tightening because "the platform cannot keep it"; an integer routine does not depend on the platform, as `log32_div` shows on this device.
   - With option 3 it is the only route by which NVIDIA could become bit-identical. The port predicts that today's parity scenarios would then have no divergence source left except the absorbed friction flushes. That is a prediction, not evidence.

## Core's recommendation, for the lead to take to Cameron

1. **Keep the refusal** (option 1) and propose no admission rule now. The evidence rules out bit-identity, and a banded admission needs grades that do not exist yet.
2. **Approve option 3's fp32_math part** as a small Core branch. It changes no CPU result, removes all four denormal failures, and makes fp32_math device-independent.
3. **Do not build an FTZ/DAZ twin** (option 2). It would fix only what step 2 fixes, at the price of a second reference that still does not match NVIDIA in full.
4. **Put option 4 on module-API stage 5's agenda:** NVIDIA at banded grade, with bands measured on NVIDIA and pinned to `driverVersion`, refused on an unmeasured driver.
5. **Option 5 only if Cameron wants a bit-identical NVIDIA path.** It reopens `CORE-3`; Core would measure its GPU cost first.

## Corrections to the inputs

- The plan and the backlog say 83 `Gpu*` tests ("72 of 83 skip"). This build ran 84; the refusal test added in the plan's step 2 is the 84th.
- The plan expects M2 to give "the quantity, element and bits" of each failure. For the parity tests the harness gives only the final tick's worst element, so M3 needed the port.
- `parity.hpp`'s two_world_isolation note says "the world SDF is empty". The world file has a ground plane, and the bodies rest on it, at the heights the logged CPU values give (about 0.12 m and 0.36 m). That note is Test/Docs' and Physics' to correct.
- `CORE-3`'s reason for refusing tightening holds for the native `sqrt` only. Option 5 is the case it did not consider.
