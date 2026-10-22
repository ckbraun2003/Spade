# The CPU airflow step flushes subnormals as the 3060 Ti does -- implementation plan

**Owner:** Physics. **Status:** DRAFT 2026-10-22, for the lead's review before any build. **Order:** the lead's, signed by Cameron, 2026-10-22 ~14:00 UTC (`tasks/todo-physics.md:66-70`; backlog `docs/design/backlog.md:102`, `4a758db`). Rendering's cost check has cleared: about 2.9% of a D1 step against the 15% line (backlog `:103`, `81dedfb`). **Facts:** `facts.md` beside this file, cited below as F1-F10.

**Goal:** the CPU twin of every airflow stage the GPU runs flushes subnormals as the RTX 3060 Ti does: a sign-preserving FTZ+DAZ on arithmetic, with copies kept.
- It flushes only inside those stages, restores the caller's MXCSR on every exit, and gives the same bits on MSVC and gcc-13.
- `BitPortability*` and `M1B*` still pass.
- Then each kernel's band can retire to bit-equal as it goes exact (B5 on).

## Global constraints

- **TD-1:** a pin that moves is named, with its cause, for the lead's word before it is re-pinned. **TD-12:** the `-NoSeed` gcc leg reproduces every new pin. **TD-9:** one home for the MXCSR scope (the lead's rule in Core's plan, `docs/design/core/plans/2026-10-22-exact-div-sqrt-plan.md:8`). **TD-5:** a skip names its environmental absence.
- **TD-3 and M1B:** no libm transcendental and no wall-clock word in `engine/`. The wall-clock scan reads comments as plain substrings, so the new header must not say "chrono", and "synchronous" contains it (F6).
- **Rule 3:** every file outside `engine/physics/airflow` and `tests/test_airflow_*.cpp` is listed below and granted before it is built.
- **Gates (tiered):**
  - all targets in release, and the release CPU suite;
  - gcc `--compile` on every changed `.cpp`, and the plain check on the header;
  - `BitPortability*:M1B*` before the leg;
  - this branch's own `-NoSeed` leg, because it touches the solver and moves pins;
  - the GPU tier for Task 4;
  - `AnalysisValidation*` after the last record commit.

## The mechanism

**One scope, moved into the engine.**
- Core's MXCSR scope (`ScopedFlushSubnormals`, F10) moves to `engine/core/detail/flush_subnormals.hpp`. It does not get copied.
- `core/detail/` is internal: no installed header includes it (`engine/CMakeLists.txt:183-186`). So `<xmmintrin.h>` never reaches a consumer, and a header-only file needs no CMake edit.
- Core's tests and the airflow code include the one header.

**Its exact semantics (decisions D3, D4 and D5 below).**
- **On x86-64** (`__x86_64__` or `_M_X64`) only.
  - The constructor saves the whole MXCSR and writes **`0x9FC0`**:
    - FTZ (bit 15) and DAZ (bit 6) set;
    - round to nearest (bits 13-14 clear);
    - all six exceptions masked (bits 7-12);
    - sticky flags clear.
  - The destructor writes the saved word back whole. Both are `noexcept`. Copy and move are deleted. There is no static state, so each thread's MXCSR is its own.
  - **The change from Core's helper:** today it writes `saved | 0x8040`, keeping the caller's rounding mode and masks (F10). That lets a caller change the step's bits (a rounding mode other than nearest), or turn the flush off: FTZ acts only while underflow is masked, and Rendering's locator unmasks it.
  - Core's tests run from the default word 0x1F80, so they see the same 0x9FC0 either way.
- **Elsewhere:** the header stops the build with `#error` (D5).

**The wrapper rule (both compilers).**
- Neither MSVC (`/fp:precise`) nor gcc (no `-frounding-math`) treats MXCSR as an input to float arithmetic. Either may move a register-only float op across `_mm_setcsr`.
- So each guarded entry point is a thin wrapper: the scope, then one call to a body marked `SPADE_NOINLINE` (`__declspec(noinline)` / `__attribute__((noinline))`, defined in the same header), passing the arguments through. The wrapper itself does no float work.
- Every float op of the step then runs inside an opaque call, strictly between the two MXCSR writes. RAII covers early returns and unwinding (`publish_render_volume` may throw `bad_alloc`).

**What it does, op by op** (F3, F4, F5):

| Class | 3060 Ti, no denormal mode | x86 under the scope | Same? |
|---|---|---|---|
| add, sub, mul, div, sqrt, compares, minss/maxss | flushes inputs and outputs | DAZ inputs, FTZ outputs | yes |
| loads, stores, copies, selects | keep | keep | yes |
| floor, float→int | keep | `roundss` under DAZ flushes; an integer-bit `floorf` keeps | the input is always a flushed quotient (`advect.cpp:34`), so they never meet one |
| abs, neg | flush the input | `andps`/`xorps`: keep | differ only on a subnormal that arrived from outside the step |
| min/max compiled as compare-and-blend | flush | keep | as above: `advect.cpp:103-104` → `:192` is the one bit-op→store path |
| a product just under 2^-126 | **unmeasured** | after rounding: rounds up to 2^-126 if within 2^-151 of it (measured, F4) | Task 4 measures it |

- **No step-computed value is subnormal on either twin.** A subnormal enters only from the grid as the caller left it, the configuration, `set_solids()`' fields, or hand-written increments (F3).
- **The edge needs a multiply:** division and sqrt cannot round across 2^-126, and an exact sum below it is exact (F4).

**Threads (F1).**
- Nothing in the engine starts a thread, and there is no pool.
- Every thread that steps the fluid (C8's eight, C8q's four LiveRuns, the actuator test's three) calls an entry point. The scope inside covers it.
- A future pool inside the step must open the scope in each worker task. The header says so, and Task 2's thread test is the pattern.

## Where each guard goes

| Entry point | Wrapper at | Noinline body | GPU twin | Guard |
|---|---|---|---|---|
| `fluid_step` | `fluid_step.cpp:226`. The scope comes before the early returns at `:227` and `:230-236` | today's body, as `step_flushed` | the whole chain | yes |
| `spread_actuator` | `actuator.cpp:103` | `spread_flushed` (its `disc_spread` runs inside) | B4 `actuator_gather` | yes (D1) |
| `wall_load` | `wall_load.cpp:9` | `wall_load_flushed` | B5 `wall_load` | yes (D1) |
| `publish_render_volume` | `publish.cpp:111` | `publish_flushed` (Core's `lambda2_hat` runs inside) | B6 `publish_fields` | yes (D1) |
| `set_solids`, `build_solids`, `label_regions`, `set_weights` | — | — | none: uploaded once (GPU plan `:58`) | no (D2) |
| stage functions (`project`, `advect_velocity`, `diffuse`, …) | — | — | kernel by kernel | no: a GPU parity test going bit-equal wraps its CPU-twin call in the scope |
| `disc_from_rotor`, `grid_io`, `check_*`, `make_*` | — | — | host side, or no arithmetic | no |

## Cross-realm files

| File | Owner | Change |
|---|---|---|
| `tests/flush_subnormals.hpp` (Core's C0, not yet on any branch, F10) → `engine/core/detail/flush_subnormals.hpp` | **Core** (both ends) | **moved** (`git mv`), then D3's word, D5's `#error` and `SPADE_NOINLINE`. The lead's grant first; Core reviews. If C0 has not landed: promoted from `tests/test_fp32_math.cpp:632-644`, and C0 targets the engine path (D7) |
| `tests/test_fp32_math.cpp:16-18`, `:632-644`, `:663` | Core | include the engine header; the class leaves the file |
| Core's planned `tests/test_fp32_exact.cpp`, `tests/fp32_exact_battery.hpp`, `tests/test_gpu_fp32_math.cpp` (plan `:89-137`) | Core | include the engine header |
| `tests/test_gpu_denorm_probe.cpp` | Physics (M1's probe), outside the airflow set | one new TEST (Task 4); no shader change |
| `tests/CMakeLists.txt` | shared | one line for `tests/test_airflow_ftz.cpp` (standing grant; named in READY) |
| `engine/analysis/live_run.cpp:492`, `:497`, `:526` | Interface | no edit. Its fluid, sources and publish now flush. Its tests are self-comparisons (F8) |
| `tools/flow_movie/main.cpp:243-252` | Rendering | no edit; same behaviour change |
| `rendering/subnormal-scan` `991b5e6`, `rendering/d1-hover-film` `6a8b36f`/`0dc71dd` | Rendering | no edit; unmerged. Their DE/UE reads around `fluid_step` see nothing from inside it after this. Told in READY |
| `tests/test_gpu_airflow_{projection,advection,actuator,publish}.cpp` | Rendering (B2, B3, B4, B6) | no edit now. When a band retires to bit-equal, its CPU-twin call runs under the scope. B4's record builder (`test_gpu_airflow_actuator.cpp:162-179`) calls `disc_spread` under the scope |
| `engine/core/lambda2.cpp` | Core | no edit; runs under publish's scope |
| `docs/design/core/00-decisions.md:39` (`CORE-3`, R3's home) | Core | a dated note that R3 is lifted for the airflow solver's sqrt. Core or the lead writes it |
| `docs/design/test-docs/00-decisions.md:49` (`TD-14`), `01-verification.md`'s GPU rule | Test/Docs | bit-equal for exact airflow kernels (backlog `:102` names Test/Docs) |
| `tests/bench` baselines | Test/Docs | timings may move: from rest, the DE/UE microcode assists go |

Physics' own: `fluid_step.cpp`, `actuator.cpp`, `wall_load.cpp`, `publish.cpp`, `tests/test_airflow_ftz.cpp`, the re-pinned `tests/test_airflow_*.cpp`, the records, the spec's §4.5, and the GPU plan's `:13-20`.

## Decisions for the lead

1. **D1. What "inside the fluid step" covers.** I recommend `fluid_step` plus the three stages with GPU twins in the chain: `spread_actuator` (B4), `wall_load` (B5) and `publish_render_volume` (B6). Not LiveRun's body and IMU step.
   - Without them, B4 and B5 cannot be bit-equal where a subnormal arises.
   - Publish's f16 output cannot show the flush (F2), so publish's scope matters only for B6's fp32 comparison.
   - Alternative: `fluid_step` only.
2. **D2. `set_solids` stays unguarded.** Both twins read its uploaded bits. Task 0 checks that no θ, φ, wall velocity, `inv_f` or coarse weight is subnormal on every solids test and on D1's parts.
   - If one is: guard it, or that θ reads open in `label_regions` and closed in the step (F3).
3. **D3. The word inside is a fixed 0x9FC0**, not Core's `saved | 0x8040`. A caller's rounding mode or unmasked underflow then cannot change the step or disable the flush.
4. **D4. On exit, the saved word is restored whole**, as Core's helper does: the literal "restored", with the step's flags discarded.
   - Alternative: restore the control bits and add the step's flags (C's `feupdateenv`), so a caller that watches DE/UE still sees them.
5. **D5. Off x86-64, the build stops with `#error`** (Physics, 2026-10-22). No such build exists (F9). A target without the flush cannot meet the bit-equal rule, so it must not compile silently; whoever adds one decides its flush then. The alternative was a no-op reporting `kFlushesSubnormals = false`, with the tests skipping by name.
6. **D6. Subnormals at entry** (a test's field, a pre-change save, the configuration) are accepted and documented.
   - After one guarded step the state holds none (Task 2's test).
   - The twins could then differ only through `advect.cpp:103-104` → `:192`, and only if the compiler emits a blend.
   - Alternative: flush the entry state by bits on both twins each step. That is an extra pass, about 0.4% of a step's bytes, plus a kernel.
7. **D7. Order with Core's C0.** I recommend that C0 create the scope at `engine/core/detail/flush_subnormals.hpp` directly, which satisfies ruling (c) in one step. Physics' branch then only changes its semantics (D3, D5, the macro). Otherwise Physics moves `tests/flush_subnormals.hpp` after C0 merges.
8. **D8. If Task 4 finds the device detects tininess before rounding,** MXCSR cannot reproduce it. Only a product within 2^-24 relative under 2^-126 can differ (F4). I recommend naming that in §4.5 as the one divergence left, with no fallback design now.

## Tasks

Each test is shown red under its named mutation by mutate-and-undo. New tests go in `tests/test_airflow_ftz.cpp`, prefix `AirflowFtz`. Flush tests read their operands through `volatile` or from memory, so nothing folds to IEEE at compile time (F3).

### Task 0: measure first (scratch harness, not committed)

- **The tiny-lid cavity.** The case is a 16³ walled cavity from rest: lid velocity `L`, ν 0.01, h = 0.5/16, `visc_sweeps` 8, `n_v` 2 (`test_airflow_step.cpp`'s `cavity_bc()`).
  - Start at `L = 1e-30f` and scale it down by 2^8 until master's `fluid_step` leaves at least one subnormal in u, v, w, p or the scratch.
  - Record `L`, the count, and the first array to hold one.
- **The tiny disc.** Choose a disc thrust for which `disc_spread` plus `peskin4` products times `scale`, computed unguarded in the test, give at least one subnormal increment.
- **Task 3's wall-load case:** p = 2^-130 (`0x00080000`) in one cut cell beside a sphere, with `dx` set so that `−p·A` is a non-zero subnormal.
- **D2's check:** a scan by bits of `set_solids()`' outputs on every solids test case and on D1's parts.

### Task 1: the scope in its one home (Core's file, granted)

- Move it as above, and point Core's tests at it (D7). Add D5's `#error`, `SPADE_NOINLINE` and the 0x9FC0 word.
- `AirflowFtz.TheScopeWritesOneWordAndRestoresTheCallers`.
  - The caller sets RC = down (`0x3F80`) plus a sticky flag. Inside, `_mm_getcsr() == 0x9FC0`. After, the caller's word is back exactly.
  - Mutations: no restore (red after); Core's `saved | 0x8040` (red inside).
- `AirflowFtz.ProductsAtTheSmallestNormalFlushAsThisCpuDoes`. These are F4's rows, pinned inside the scope:
  - upper `0x00800000`, lower `0x00000000`, exact `0x00800000`, neg-upper `0x80800000`;
  - DAZ `0x00000000`, and 2^-70 × 2^-70 → `0x00000000`;
  - outside the scope, upper and lower are both `0x00800000`, which shows the rows discriminate.
  - Mutations: FTZ only (`0x9F80`), so the DAZ row goes red; DAZ only (`0x1FC0`), so the lower row goes red.
- `AirflowFtz.EachThreadKeepsItsOwnWord`.
  - The scope runs on the main thread, then on a `std::thread`. The worker reads 0x9FC0 inside and its own prior word after. The main thread's word is unchanged after the join.
  - Mutation: a process-wide "set once" flag in the constructor, so the worker reads its default inside.

### Task 2: `fluid_step` under the scope

- The wrapper at `fluid_step.cpp:226`, with the body as `step_flushed`.
- `AirflowFtz.AStepFromRestLeavesNoSubnormal`.
  - Precondition: `diffuse()` (unguarded) on the pre-diffusion state makes at least one subnormal (printed).
  - After one `fluid_step`, there are 0 subnormals by bits in u, v, w, p, `du/dv/dw`, `rhs`, `next`, the advection scratch, `div`, and every level's p, b and r.
  - Red on master. Mutation: the scope removed.
- `AirflowFtz.TheStepRestoresTheCallersWordOnEveryReturn`. The caller's word is RC = down. Three returns:
  - healthy;
  - a poisoned scratch (`:227`);
  - an ill-posed closed box with a net inflow (`:234-236`).
  - After each, the word is back exactly. The healthy step's digest from an RC-down caller equals the default caller's.
  - Mutations: a hand-written restore before the last return only, so the early rows go red; the OR form, so the digest row goes red.
- `AirflowFtz.AWorkerThreadStepsFlushedAndIsRestored`.
  - The tiny-lid step runs on a `std::thread`. The digest equals the main thread's, there are 0 subnormals, the worker's word after equals its word before, and the main thread's word is unchanged.
  - Mutation: the "set once" flag.
- `AirflowFtz.TheTinyLidDigestIsPinned`.
  - The tiny lid runs 16 steps, FNV over u, v, w, p.
  - The pin is generated on MSVC release with its provenance comment. The `-NoSeed` gcc leg reproduces it: **MSVC and gcc agree.**
  - Mutation: the scope compiled under `#ifdef _MSC_VER` only, so the leg goes red.

### Task 3: the sources, the wall load and publish (D1)

- The wrappers at `actuator.cpp:103`, `wall_load.cpp:9` and `publish.cpp:111`.
- `AirflowFtz.ATinyDiscsSpreadLeavesNoSubnormal`: Task 0's disc gives 0 subnormals in `du/dv/dw`, and the caller's word comes back. Mutation: the scope removed.
- `AirflowFtz.AWallLoadReadsASubnormalPressureAsZero`: Task 0's cell gives a force and moment of exactly 0. Mutation: the scope removed (non-zero).
- `AirflowFtz.PublishRestoresTheCallersWordOnEveryReturn`: an ok publish, and a refused one (`publish.cpp:116-123`). Mutation: a hand restore on the ok path only.
  - No flush test: its f16 output cannot show one (F2).

### Task 4: the device's rule at 2^-126 (Physics; GPU tier via jobrun)

- `GpuDenormProbe.AProductJustBelowTheSmallestNormalFlushesAsTheCpuScopeDoes` in `tests/test_gpu_denorm_probe.cpp`, as its own TEST so the standing table's rules are untouched.
  - F4's rows and their sign flips go through the existing `Op::mul` mode. There is no shader change.
  - The expected bits come from the host under the moved scope.
  - It prints each row's device bits and the rule they fit: before rounding, after rounding, or flush-of-gradual. It asserts device = host.
- Mutation: expected bits computed outside the scope (IEEE gradual), so the lower row goes red on a flushing device.
- On a miss, D8: the rows and the rule go to the lead.

### Task 5: TD-1, run, then name

- At the green head, run the release CPU suite, then the opt-in record runs through jobrun, queued together:
  - `SPADE_AIRFLOW_VALIDATION=1` for C1 at 128 and the S records;
  - C8's eight-run ensemble, about 11 minutes.
- Diff each record's printed lines against its last block.
- For each pin that moves, name the test, the old and new value, the first array that differs, its max |d|, and the cause. The cause comes from MXCSR's DE/UE read around the call, as `991b5e6` did, in the scratch harness.
- All of it goes to the lead in one message.
- On the lead's TD-1 word:
  - re-pin with provenance;
  - append the record blocks;
  - run `AnalysisValidation*` after the last record commit;
  - the leg reproduces every new pin.
- Predictions (F8), not evidence:
  - `kSolverCoreDigest` likely moves, and C1's bits likely move;
  - S2 and planar Couette are possible;
  - `AirflowConfigs`, C0, S1, S3 and S5 are unlikely;
  - C8 was measured unchanged;
  - D1 is not re-run (Cameron's "Don't re-run"). Its next run carries subnormal-size differences from rest.

### Task 6: the docs

- The §4.5 amendment below, on master after the lead's review (Physics' spec, docs only).
  - The same change goes into the GPU plan's constraints (`2026-10-22-airflow-gpu-solver-plan.md:13-20`).
  - It also goes into §4.6's and the Summary's "banded" wording (`2026-10-05-airflow-design.md:9`, `:448-450`).
- Physics' `04-verification` and `07-status` rows change as kernels go bit-equal (B5 on), not in this branch.

## Corrections from Tasks 0 and 4 (2026-10-22; the lead's approval)

- **D8 does not arise.** The 3060 Ti (driver 572.83) detects tininess after rounding, as the CPU scope does. All 20 rows of the edge test match (`GpuDenormProbe`, `tests/test_gpu_denorm_probe.cpp`), so §4.5 names no divergence.
- **D2 is settled.** 154 bit scans of `set_solids`' outputs found no subnormal: every CPU suite that calls it, S1 and S2 at their record resolutions, and D1's parts on D1's grid.
- **Task 1's FTZ-only mutation** (0x9F80) needs a "DAZ to normal" row, 0x00400000 × 0x40800000 → 0x01000000 without DAZ. The "daz" row alone stays green under it, since FTZ zeroes a subnormal product too.
- **The hand-restore mutation** goes inside the noinline body. The wrapper has only one return.
- **The wall load's moment** is tested about a box corner. About the sphere's centre it is 0 even unguarded, since the arm lies along the area vector.

## Results (2026-10-22)

Tasks 0-5 are done at `39d9a0a`, on Core's C0 (`d0899ee`, merged as `824c492`).

- **The TD-1 list is empty.** No golden, pin or record moved.
  - The release CPU suite passes 2400 of 2400, with 13 opt-in or environment skips.
  - The 14 opt-in records print the same lines as their last blocks. Two differ, and neither from this branch: C1 32×32 names its tolerance differently, and C0 prints four more lines (plan A's residual, added 2026-10-21).
  - `BitPortability*:M1B*` pass, 13 of 13.
- **The new pin.** `AirflowFtz.TheTinyLidDigestIsPinned` holds `0xe2b5d178b90aba65`: the tiny lid (L = 1e-30) for 16 steps, FNV over u, v, w and p. It was made on MSVC release, and Task 0's emulated scope gave the same value.
  - The `-NoSeed` gcc leg on `39d9a0a` passes: "docker-leg: exit 0, PASS", 0 failed of 2397. It prints the same digest. MSVC and gcc agree.
- **The cause reads.** A scratch replica of the no-solids step read MXCSR's DE and UE around every step, with no scope.
  - `kSolverCoreDigest`'s cavity (64 steps), C1 at Re 100 on 32² (1280 steps) and on 128² (10240 steps) raise neither flag in any step. No step ends with a subnormal.
  - The guarded and unguarded runs are bit-identical at every step. A lid of U = 1 makes no subnormal: cells the sweeps have not reached stay exactly 0. So the flush cannot move these pins.
  - C8 was measured unchanged before (`991b5e6`). D1 is not re-run; its next run carries the shift.
- **The mutations.** Each test went red under its named mutation. Each mutation was then undone, and Core's header is unchanged.
  - Scope removed, no restore: the step leaves 6361 subnormals; the disc's du, dv and dw hold 340, 336 and 340; the wall force is `0xBCC`; the digest is `0x010b593c983c26ef`.
  - Core's OR word (`saved | 0x8040`): the word inside is `0xBFC1`, and an RC-down caller changes the step's digest.
  - FTZ only (`0x9F80`), with a hand restore before the body's last return: "DAZ to normal" reads `0x01000000`; the poisoned and ill-posed returns leave `0x9FC0`; so does publish's refused return.
  - DAZ only (`0x1FC0`): the lower row reads `0x00800000`.
  - A process-wide "set once" flag: the worker reads `0x1F80` inside, and its digest differs from the main thread's.
  - Task 4's expected bits taken outside the scope (IEEE gradual): 12 rows red on the 3060 Ti.
  - The scope under `_MSC_VER` only, on a gcc leg (a scratch branch, deleted): "docker-leg: exit 1, FAIL". All ten `AirflowFtz` tests are red and nothing else is. The tiny-lid digest is `0x010b593c983c26ef`; one step leaves 6361 subnormals; the disc's du, dv and dw hold 340, 336 and 340; the wall force is `0xBCC`; the word inside is the caller's.
- **The GPU tier** at `39d9a0a`. `ctest -L gpu` fails nothing: 170 of 181 pass, and 11 skip by name (opt-in measurements and the poster).
  - B2's projection (6 tests), B3's advection (4; no value differs), B4's gather (2) and B6's publish (3) pass. B5 has no GPU test yet.
  - C8's GPU gather in the CPU step (opt-in, run from the binary) passes all five judgements. Its downwash is 8.0068 m/s, and it leaves the CPU's run at step 169, not before 141.
  - `GpuDenormProbe` passes 2 of 2, the edge test included.
- **Task 6.** The §4.5 amendment is in the spec, with the after-rounding sentence: no divergence is left at the edge. The Summary, §4.6 and the GPU plan's constraints say the same.

## TD-1 candidates

F8's table is the list. Every row marked "likely", "possible" or "unlikely" is run in Task 5. The rows marked "not exposed" (frame goldens, scenario goldens) and "cannot fail" (self-comparisons) are not. The live smoke is a video and pins nothing.

## The §4.5 amendment (draft)

**The reductions bullet:**
- Replace "Only each operation's rounding can differ between backends, and that difference is banded." with "Only each operation's rounding can differ between backends: banded, until the kernel is exact (below)."

**Insert after "Operation order is the contract":**

> - **Bit-equal for exact kernels; both twins flush** (**amended by Cameron, 2026-10-22**: AskUserQuestion via the lead, "Full bit-exact", backlog `4a758db`, with SPIR-V `E1`/`E2` restated, "Restate E1/E2, ~3%", backlog `81dedfb`. This reopens his 2026-10-05 banded ruling for the airflow solver only).
>   - **Bit-equal replaces banded, kernel by kernel.** An airflow kernel must equal its CPU twin bit for bit, on each device and driver measured, when two things hold. First, every division and square root in it is an exact `fp32_math` operation (`div32_ftz`, `sqrt32_ftz`: the device's estimate finished by an integer correction proven for any estimate within Vulkan's bound, `E1`/`E2`). Second, its other float operations are the twin's adds, subtracts, multiplies, compares, selects, loads and stores, in the twin's order, uncontracted. Its band retires when it goes exact. Until then it stays banded under `TD-14`.
>   - **Subnormals: both twins flush.** No kernel requests a denormal mode (`P3` stands; the 3060 Ti supports neither). With none requested, the 3060 Ti's arithmetic is a sign-preserving FTZ+DAZ, and its loads, stores, floor and float→int keep subnormals (the denorm report's finding 1). The CPU twin matches it. The fluid step, the sources' spread, the wall load and publish each run with x86 MXCSR's FTZ and DAZ set, round to nearest and every exception masked. Each restores the caller's MXCSR on every exit, on whichever thread calls it (`engine/core/detail/flush_subnormals.hpp`, one home). So neither twin's arithmetic writes a subnormal, and a copy keeps one on both.
>   - **The edge.** Division and square root cannot round across 2^-126, and a sum below it is exact, so only a multiply meets the tininess rule. x86 detects tininess after rounding. `GpuDenormProbe.AProductJustBelowTheSmallestNormalFlushesAsTheCpuScopeDoes` pins the device to the same rule (or: names a product within 2^-24 relative under 2^-126 as the one divergence left; Task 4 decides which sentence stands).
>   - **`R3` is lifted for the airflow solver's square roots** (Cameron's word, the same answer; its home is `CORE-3`). R3 refused tightening because "Vulkan does not guarantee a correctly-rounded sqrt". That holds for the native `sqrt`, not for an integer-corrected one. `peskin4`'s weights and publish's `|ω|`, `q̂` and `λ2` use `sqrt32_ftz`. The gaussian draw (`rng`, `CORE-3`'s scope) is unchanged.
>   - **What stays.** Bit-equality is per device and driver (`CORE-5`). Another device, another driver, or a kernel not yet exact keeps its `TD-14` band. The CPU stays the reference and the only golden source (`TD-1`).

**"Expected CPU↔GPU differences" becomes:**

> - **Expected CPU↔GPU differences** (as amended 2026-10-22): flushed denormals are no longer one, since the CPU twin flushes as the device does (above). The device's native division and square root, 1 ulp off on **normal** operands (finding 4), remain a difference only in kernels not yet exact: the relative term, and chaos amplifies it (Core). The host-computed reciprocals (`1/dx`, `h_f/ρ`) stay. Determinism holds per device and driver (`CORE-5`) … *(the rest unchanged)*.

## Risks

- **Code motion across the MXCSR write.** The wrapper rule (noinline body, no float work in the wrapper) removes it on both compilers. Task 2's pinned digest and the leg are the check.
- **The device's tininess rule** (D8). It is unmeasured until Task 4. It can matter only for a multiply landing within 2^-24 relative under 2^-126.
- **Entry subnormals and x86's bit-ops** (D6). abs and neg are `andps`/`xorps`, and min/max may be emitted as a blend. They are harmless unless a subnormal arrives from outside the step and reaches `advect.cpp:192`.
- **A driver change.** The device's flush is implementation-defined with no mode (P3). `GpuDenormProbe`'s standing table shows a change in every gate log, and Task 4's test fails on one.
- **Library code under the scope.**
  - `floorf` (MSVC's CRT, glibc's ifunc) differs between implementations only on a subnormal input, and it never gets one here (F3).
  - Core's `lambda2_hat` runs under publish's scope. Its f16 output cannot move (F2).
- **Tools that read MXCSR around the step** (Rendering's unmerged scans and locator) see nothing from inside it, under D3 and D4. That is intended. They are told.
- **D1's measured shift (8.8e-39) flushed the whole `LiveRun::step()`**, body and IMU included (F5). The fluid-only scope's own shift is Task 5's to name, for the next D1 run.
- **Cross-realm timing.** Core's C0 and its exact ops are moving at the same time (D7). One home must hold at every merge: no second copy, even for a batch.
- **Performance.** Two MXCSR writes per call is nothing against a step. From rest the step may get faster, so a bench baseline can move.

## Review focus

- That every float op of each guarded entry point is inside the noinline body, and the wrapper does none.
- That the early returns in `fluid_step` sit inside the scope.
- That no comment in the new header trips the wall-clock scan.
- Each moved pin's named cause, before its re-pin.
