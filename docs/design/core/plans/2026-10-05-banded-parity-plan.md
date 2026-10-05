# Banded CPU↔GPU parity everywhere — plan

**Owner:** Core, with Test/Docs (SPIR-V rules, Slang build, `TD-*` rules, scanners) and Physics (parity bands). **Ruling:** the user's, 2026-10-05: "Just remove the bit exact gpu-cpu parity. As long as values are basically the same, extremely low error, note in documentation that gpu and cpu based calculations are going to differ slightly" (`../../backlog.md`, `7a2b8c8`). **Status:** DRAFT for the lead. Not reviewed, not committed. Its home is `docs/design/core/plans/2026-10-05-banded-parity-plan.md`. **Base:** master `b4a64a7`; re-check every file:line anchor at each branch cut. **Evidence:** `../2026-10-04-nvidia-denorm-report.md` and its data (`../data/2026-10-04-nvidia-denorm-m2.log`, `../data/2026-10-05-nvidia-denorm-m1.txt`).

## Why

This machine's only GPU, the RTX 3060 Ti (driver 572.83), cannot preserve fp32 denormals. Every kernel requests `DenormPreserve 32`, and requesting it there is undefined behaviour, so the default build refuses the device and 72 `gpu` tests skip. The report showed bit-identity with the CPU is out of reach on NVIDIA anyway: its division and square root are not correctly rounded on normal operands (finding 4). The user then removed bit-exact parity. This plan makes CPU↔GPU banded everywhere, admits the device, and says so in the docs.

## The ruling against the laws

The laws already allow it. Quoted from `../../00-charter.md`:

- `L1`: "Fixed step and seeded. No clock is read in the step path; `dt` is immutable after a simulation is created."
- `L2`: "Snapshot, restore and replay are first-class. Restoring under a different configuration is refused."
- `L3`: "Every module declares a grade: **reference**, **banded** or **best-effort**. A world's grade is that of its weakest module. A consumer may require a minimum grade; a world that cannot meet it is refused, never silently downgraded."
- `L4`: "Reference grade needs a CPU implementation and a golden result. Banded grade needs a measured band against the reference, with a record of how it was measured. The GPU is never a golden source."
- `L6`: "No silent fallback. An unavailable backend, module or grade is refused or announced."

And from `../../test-docs/00-decisions.md`:

- `TD-1`: "Never loosen a golden, never keep per-platform goldens, never take one from the GPU."
- `TD-2`: "A tolerance band is measured, then pinned with margin, with the device and toolchain it was measured on. It is never widened to make a failure pass without a root cause; a band that cannot be met is a grade change (`L3`), not a wider band".

**Gone:** bit-identity between CPU and GPU as a requirement. That covers SPIR-V rule `P3` as written, the context's refusal of devices without `shaderDenormPreserveFloat32`, `CORE-3`'s bit-identical clauses, and the zero bands that claim arithmetic agreement.

**Stays:**
- CPU determinism. One golden set, reproduced by MSVC and by the gcc leg (`TD-1`, `TD-12`), which is stronger than "per platform".
- The GPU deterministic against itself, per device and driver. It is pinned by in-process comparisons only, never by a committed GPU digest (`L4`, `TD-1`).
- Exact rows that no rounding touches: integers, copies, and structural zeros (section 2).
- `L6`: every device's float behaviour is recorded and reported (section 1).

## 1. Admitting devices without fp32 denormal preservation

### What the code does today

- **Kernels.** `cmake/SpadeSlang.cmake` compiles every kernel with `-fp-mode precise` and `-denorm-mode-fp32 preserve` (`_spade_slang_denorm_args`, `:143-151`; used at `:342-351` and `:426-446`). That is 15 kernels: `fp32_math_probe` once and 14 schedule kernels at three local sizes, so 43 SPIR-V modules. `embed_spirv.py` turns each into a `uint32_t[]` in a build-tree `.spv.gen.hpp`. Nothing loads a file at runtime.
- **Pipelines.** `StepRecorder` passes the embedded words straight to `vkCreateShaderModule` (`step_recorder.cpp:438-446`), as does `probe_runner.cpp:235`. The arrays are `const`.
- **Admission.** `vulkan_available()` (`context.cpp:216-257`) and `create()` (`:404-453`) refuse a device whose `shaderDenormPreserveFloat32` is false. Requesting the mode there is a valid-usage violation, not a reported error (`SpadeSlang.cmake:104-111`).
- **The measurement build** (`SPADE_MEASURE_UNPINNED_DENORMS`) already compiles every kernel with no denormal mode, inverts `P3`, and admits the device. M2 ran the real kernels that way on the 3060 Ti: 72 of 84 `Gpu*` tests passed. Four failures were `fp32_math`'s three subnormal paths (item 3). Eight were NVIDIA's division and square-root rounding against bands pinned on the Iris (item 2).
- **Intel's default**, recorded in the code (`SpadeSlang.cmake:80-87`, `spirv_scan.hpp:42-50`): the Iris Plus (driver `0x0019484D`, i.e. 31.0.101.2125) "FLUSHES DENORMALS TO ZERO by default while advertising support for both behaviours".
- **AMD's default** is not in the code and nobody here has measured it. A public llvm-dev note says the AMDGPU graphics default assumes fp32 denormals flushed (https://groups.google.com/g/llvm-dev/c/TDGKHFU4hzE/m/k-LEa3NvBQAJ). Treat it as unknown until measured.

### The three options

| | (a) Two kernel sets, chosen at context creation | (b) One set, no denormal mode, on every device | (c) One set with the mode, stripped at pipeline creation where unsupported |
|---|---|---|---|
| **Build** | 86 SPIR-V modules instead of 43. The slangc step doubles, and so does the embedded SPIR-V | 43, as today. The flag is dropped | 43, as today |
| **Engine code** | A second axis on `SpirvVariants`. Recorder and probe runner pick a set from a context flag | Less code: the refusal, the `#if` blocks, the marker and the measurement option go | A SPIR-V rewriter in product code: copy the words, drop `OpCapability DenormPreserve`, `OpExecutionMode … DenormPreserve 32` and the extension. The module that runs is not the module the gate scanned |
| **`P3`** | Two profiles with opposite expectations | One rule: no denormal mode | The scanner must also scan the stripper's output |
| **Behaviours to band** | Two: preserving devices and the rest | One: the device default, which is flush on both devices on record | Two, as (a) |
| **Tested here** | The preserving set cannot run on the 3060 Ti, so it goes untested on the only gate machine (`TD-6`, `TD-13`) | Everything. M2 is already a run of it | The unstripped path goes untested here, as (a) |
| **Risk** | Low per line, but the set this box cannot run rots | A future kernel that needs subnormals would differ silently. `P3`, the bands and the probe catch it | A malformed module from a future slangc. No `spirv-val` in tree; the validation layer runs only in Debug, if installed |
| **What it buys** | Subnormal agreement with the CPU, on preserving devices only | One artifact, one profile, one behaviour | As (a), at more risk |

### Testing Core's lean

- **The cost of (b) is subnormal magnitudes only, once item 3 lands.** Flushing changes values below 1.18e-38. It reaches normal magnitudes in four ways:
  - `fp32_math`'s three paths. Item 3 removes them.
  - Compares with a subnormal operand. The friction step's `if (v_t_len > 0.0f)` (`collision_static.slang:179`, `collision_dynamic.slang:218`) skips a stiction step of order 1e-21 m/s. M3 found 92 such flushes in `two_world_isolation`, all absorbed.
  - A broadphase cell flip, for |pos| < 2.8e-39 m at a 0.24 m cell. No test reaches it.
  - Division by a flushed zero. The guards above prevent it in today's kernels.
- **What (a) and (c) buy** is agreement below 1.2e-38 on devices that preserve. Every band in section 2 is many orders above that.
- **Both known devices flush by default.** So under (b) the Iris and the 3060 Ti behave alike, where (a) would make them differ.
- **Coverage decides it.** Under (a) or (c) the preserving path never runs on the gate machine. Under (b) every path runs on it.

**Recommendation: (b).** Core's lean holds. One kernel set, compiled with no denormal mode, on every device.

### `P3` under (b)

`P3` becomes: **no fp32 denormal mode is requested.** No module declares a `DenormPreserve` or `DenormFlushToZero` execution mode (SPIR-V 4459, 4460) for any width, or the matching capability (4464, 4465). Test/Docs confirms the four values against the SPIR-V grammar, as `P4`'s note did for `Int64`. The rule is a tripwire: if the flag, or a future slangc default, puts a preserve request back, the gate fails before a device runs it. On NVIDIA that request would be undefined behaviour again.

### How the context announces a device (`L6`)

- **Record at `create()`:** device name, `vendorID`, `deviceID`, `apiVersion`, `driverVersion`, and, on Vulkan 1.2 or later, `VkPhysicalDeviceDriverProperties` (`driverName`, `driverInfo`) and `VkPhysicalDeviceFloatControlsProperties`. Below 1.2 the float controls are reported as "not queryable". (`gpu_skip.hpp:69` already uses the 1.2 bound; `context.cpp:97` uses 1.1, which is a latent mismatch this fixes.)
- **Expose it:**
  - `compute::DeviceReport`, a plain Vulkan-free struct in `compute/backend.hpp`;
  - `VulkanContext::device_report()`;
  - `Simulation::vulkan_device_report()`, a diagnostic like `vulkan_pass_durations_ns()`, `Code::unavailable` on the CPU backend;
  - `compute::describe(const DeviceReport&)`, one line, for example: "NVIDIA GeForce RTX 3060 Ti, driver 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no)".
- **Show it.** The library writes nothing to stderr, as today. The gpu test fixture prints the line once per process. Every parity table prints it in its header, so every measurement carries its device and driver (`L4`, `TD-8`). The viewer prints it when run with `vulkan`, and the sandbox shows it beside its backend picker (Interface, a follow-up through the lead).
- **The one capability clause left** is Vulkan 1.1, because the kernels are SPIR-V 1.3. A device below it is refused by name.

## 2. "Extremely low error" as a band policy

### The machinery today

- `parity.hpp` compares per quantity, per element, at the **final tick** of the run. Its provenance block says "measured max over the WHOLE run" (`:498`); the report found the harness reports only the final tick. Physics corrects the line.
- The predicate is a disjunction: an element passes if `|g − c| ≤ abs` **or**, for `c ≠ 0`, `|g − c|/|c| ≤ rel` (`:66-91`, `:452-454`). A NaN fails.
- Every band was measured on the Iris Plus (driver 31.0.101.2125) at about 4× and rounded up. The stop rule is 1e-3 relative (`:509`).
- Integer lanes (`QuantityKind::bits`) compare exactly.
- Horizons run from 0.2 s to 1.8 s of simulated time: `ballistic` 200 × 5, `gate_fleet` 900 × 1, `bounce` 900 × 1, `drag_componentwise` 400 × 2, `restore_resume` ticks 300 to 700, `heterogeneous_geometry_set` 900 × 1, `contact_pair` 600 × 1, `shower` and `shower_ladder` 400 × 2, `two_world_isolation` 300 × 2, `quad_hover` 900 × 2, `gnss_receiver` 200 substeps, `gnss_tumble` 400 × 4.

### The policy (for the user's signature as `TD-14`)

1. **The test.** Per quantity and element: `|gpu − cpu| ≤ abs_q + rel_q × |cpu|`. A NaN on either side fails. At `cpu = 0`, `abs_q` carries it alone, as today. This replaces the disjunction. It is at most 2× more lenient, and every band is re-measured anyway.
2. **The measurement.** For each device of record, run the scenario to its horizon. The harness reports two maxima per quantity, with a near-zero cutoff `s_q` of 1e-3 in the quantity's SI unit:
   - `A` = the largest `|g − c|` over elements with `|c| < s_q`;
   - `R` = the largest `|g − c| / |c|` over elements with `|c| ≥ s_q`.
3. **The pin.** `abs_q = 4A` and `rel_q = 4R`, each rounded up to one significant figure, and the larger over all devices of record. The Iris numbers stay on record as history; the 3060 Ti is the measuring device from now on.
4. **The floor.** A float row measured exactly equal on every device of record gets `rel_q = 5e-7` (four ulps) and `abs_q` = four ulps at `s_q`. "Measured zero on one device" is not a claim this policy makes any more.
5. **Exact rows that stay exact.** A row keeps a zero band only if no rounding operation can touch it: integer lanes; copies (`StoredFieldSamplesMatchTheCpuCopiesBitwise`'s gravity and density); and structural zeros, such as `force_acc` and `torque_acc`, or `specific_force` in a scenario with no force element. Each carries its argument beside it, as `body_bands()` does for the accumulators (`test_gpu_parity.cpp:147-152`).
6. **The ceiling.** The target is `rel_q ≤ 1e-4`. The stop rule stays: a quantity that needs more than 1e-3 relative is a grade change (`TD-2`), never a band.
7. **The record (`L4`).** Beside each band: device, vendor, driver and `driverVersion`, Vulkan version, float controls, build and commit, scenario, steps × substeps and horizon in seconds, the one-ulp CPU control at that horizon, method ("final tick, every element of every world, `A`/`R` split at `s_q`"), measured `A` and `R`, margin (4×, rounded up), and the older devices' numbers.
8. **Root cause.** A band that widens cites its cause. For the rows below it is the report's finding 4: NVIDIA's division and square root on normal operands. That satisfies `TD-2`.

### Starting values from M2 (3060 Ti, driver 572.83)

These are conservative first drafts: `abs` is 4× M2's largest absolute error and `rel` is 4× the worst element's relative error, rounded up. Physics' split re-measure replaces them.

**The nine failing rows and the gaussian source:**

| Scenario, quantity | M2 worst element (`|c|`, `|g − c|`) | Band today (Iris) | Starting band `{abs, rel}` |
|---|---|---|---|
| `gate_fleet` pos | 0.248, 3.20e-6 | {2e-6, 1e-5} | {2e-5, 6e-5} |
| `two_world_isolation` pos | 0.358, 8.94e-8 | {0, 0} | {4e-7, 1e-6} |
| `two_world_isolation` vel | 8.4e-3, 2.79e-9 (resting residues near 1e-24 elsewhere) | {5e-28, 2e-4} | {2e-8, 2e-6} |
| `contact_pair` vel | 1.565, 9.54e-7 | {0, 0} | {4e-6, 3e-6} |
| `restore_resume` pos | 3.267, 4.77e-7 | {0, 0} | {2e-6, 6e-7} |
| `restore_resume` vel | 0.205, 1.67e-6 | {1e-7, 0} | {7e-6, 4e-5} |
| `heterogeneous_geometry_set` pos | 0.702, 1.35e-5 | {2e-6, 1e-5} | {6e-5, 8e-5} |
| `heterogeneous_geometry_set` vel | 1.772, 4.43e-5 | {1e-5, 1e-5} | {2e-4, 2e-4}, above the 1e-4 target (risk 5) |
| `gnss_bias_discriminator` bias | 0.873, 5.96e-8 | {0, 0} | {3e-7, 5e-6} (cites `CORE-3`) |
| `CORE-3`, the gaussian draw (`gnss_ring` position, zero lever arm) | 3.42, 4.77e-7; max rel 1.07e-6; 19 of 64 differ | asserted outside {0, 0} | {2e-6, 5e-6} |

The two cross-backend restore tests (`test_gpu_invariance.cpp:853`, `:921`) reuse `restore_resume`'s bands and follow them.

**Seven rows that pass on NVIDIA with less than 4× margin,** so `TD-2`'s margin raises them: `gate_fleet` vel {4e-4 → 7e-4}; `gnss_receiver` bias {2e-8, 2e-6 → 4e-8, 4e-6}; `gnss_receiver_body` orient abs {4e-7 → 5e-7}; `quad_hover` specific_force abs {2e-6 → 4e-6}; `drag_componentwise` vel abs {4e-6 → 1e-5} and omega abs {8e-6 → 2e-5}; `shower_ladder` vel abs {2e-5 → 4e-5}. The relative halves cannot be judged from M2, because its maximum relative error may come from a near-zero element that passed on `abs`. Two may also be thin: `drag_componentwise` omega (1.8×) and `shower_ladder` pos (3.6×). The split decides.

### The 41 zero rows in `parity.hpp` (from Physics' survey, checked against the file)

- **Structural, stay at {0, 0} with the argument written beside them (25):**
  - `specific_force` where no force element exists: `gate_fleet`, `bounce`, `restore_resume`, `heterogeneous_geometry_set`, `contact_pair`, `shower`, `shower_ladder`, `two_world_isolation`, `gnss_receiver_body`, `gnss_tumble` (10). `force_acc` is cleared every substep, so it is `0 / m`, rotated.
  - `omega_body` where no torque source exists and ω starts at zero: `bounce`, `restore_resume`, `contact_pair`, `shower`, `shower_ladder`, `two_world_isolation`, `gnss_receiver_body` (7); and `quad_hover`, whose four symmetric, identically commanded rotors cancel exactly (1).
  - `orient` where ω is identically zero: `bounce`, `shower`, `shower_ladder`, `two_world_isolation`, `quad_hover` (5). Physics checks that the small-angle branch leaves the quaternion untouched, renormalisation included; a row that fails the check moves to the floor.
  - `quad_hover` gyro (ω is zero) and bias (its walk sigmas are zero) (2).
- **Measured, move off zero to the floor or to a re-pinned value (14):** `ballistic` pos; `bounce` pos and vel; `restore_resume` pos (re-pinned above); `contact_pair` pos, and vel (above); `two_world_isolation` pos (above); `gnss_receiver_body` pos and vel; `gnss_tumble` pos, vel and omega (a torque-free tumble); `heterogeneous_geometry_set` omega (likewise); `quad_hover` rotor omega.
- **Gaussian, move to `CORE-3`'s band (2):** `quad_hover` and `gnss_tumble` cached gauss. So do the call-site zeros in `test_gpu_parity.cpp`: `gnss_receiver`'s cached (`:614`), and the discriminator's `noise.cached` and `bias`.

A guard is worth its few lines: a `QuantityKind::structural` row must carry {0, 0}, and a `components` row may not. Then a zero band cannot creep back as an unstated bit-identity claim. Physics decides.

### Horizons, and chaotic scenes

- **A band means something only while it measures the port, not the scene's amplification.** `ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone` shows the line: one ulp of one body's x, CPU against CPU, gives 1.2e-7 m at 400 steps and 1.0e-2 m at 800 steps (e = 0.8). The CPU↔GPU gap at 800 steps was 1.02e-2 m, the same number.
- **The rule.** A scenario's horizon `T_p` is the longest tested horizon at which the one-ulp CPU control stays under 1e-5 of the scene's scale (1e-5 m for a metre-scale scene; likewise for velocity). Element bands are pinned only at horizons up to `T_p`. Physics adds the one-ulp control, host-only, for each scenario with contacts. `heterogeneous_geometry_set` comes first: NVIDIA's 1.35e-5 m at 900 steps is 30× the Iris figure, and may be amplification.
- **Past `T_p`, element bands give way to:**
  - **invariants**, exact: body count and liveness, no NaN or infinity, quaternion norm within two ulps on both sides, every body inside its container (no tunnelling);
  - **statistics**, banded against chaos: the centre of mass, mean and RMS speed, total energy and the number of bodies in contact. The band is 4× the largest deviation of the same statistic across eight one-ulp-perturbed CPU runs at the same horizon. The GPU then has to be as close to the CPU as the CPU is to itself, nudged by one ulp.
- **Recursive filters accumulate rather than amplify.** `gnss_receiver`'s bias note ("this band has a silent expiry date", `parity.hpp:1429`) stays true: the band holds to its stated horizon. Physics adds one report-only run at 10× a non-chaotic horizon (`quad_hover` or `gnss_tumble`), so the docs can say how fast differences grow.

This is what "basically the same" means in numbers. Over the tested horizons the largest genuine relative difference measured is 3.2e-5 (the Iris, `shower` vel) and 2.5e-5 on NVIDIA (`heterogeneous_geometry_set` vel). The largest position difference is 1.35e-5 m, and the largest near-zero one is 1.6e-4 m/s on a resting body (`gate_fleet`). Starting bands reach 2e-4 relative in two places (`heterogeneous_geometry_set` and the shower velocities); most sit at 1e-6 to 1e-5.

## 3. The `fp32_math` integer fix: still worth doing

**Yes, and under (b) it is required, not optional.** With no denormal mode requested anywhere, both devices on record flush, so without the fix `fp32_math` gives device-dependent answers on every GPU:

- `log32` of any subnormal returns −104.665222 where the host returns, for example, −103.278931. That is an error of 1.39, about 1.3%. It is a wrong answer from a library function, not a rounding difference.
- `exp32` returns +0 for its 2,180,453 subnormal-result arguments, below −87.3365479.
- `sin32` returns 0 for a subnormal `x`; the host returns `x`.

**The fix**, from the report's table 5, in both twins:
- `exp32`'s tail (`fp32_math.slang:490`, `:497-502`; `fp32_math.cpp:426-431`): build the subnormal result from `m`'s significand by an integer shift with round-to-nearest-even, as `log32_div` does.
- `log32`'s pre-scale (`fp32_math.slang:416-430`; `fp32_math.cpp:293-299`): normalise a subnormal by its leading-zero count instead of multiplying by 2^24.
- `sin32` (and `cos32` through the shared reduction): for |x| < 2^-126, decided on the bits, return `x` before reducing.

**Cost.**
- No CPU result changes, so no golden moves. The host's float forms are exact, and the integer forms give the same bits by construction; the new test below proves it over every affected argument.
- CPU time: the `exp32` and `log32` paths run only on subnormal inputs or results. `sin32` gains one integer compare. Nothing measurable.
- Work: three functions in two twins, one new host test, and comment updates. A small Core branch.
- **Risk:** an off-by-one in the rounding. The exhaustive host test catches it on both toolchains.

**What it buys.** `fp32_math` becomes independent of any device's denormal mode. The 13 `GpuFp32Math` batteries go green on the 3060 Ti (M2 had them exact everywhere else: all 2^24 Box-Muller angles, 17 million `log32` arguments, 9,437,184 divisions). The library stays a bit-exact library, which `TD-3`'s whole design rests on.

## 4. Inventory: everything that states or depends on CPU↔GPU bit-identity

Searched: `engine/ tests/ cmake/ scripts/ docs/ README.md CHANGELOG.md CONTRIBUTING.md AGENTS.md CLAUDE.md` and the top-level `CMakeLists.txt`, for "bit-identical", "bit-exact", "bitwise", "bit for bit", "DenormPreserve", "shaderDenormPreserveFloat32", "zero band", `P3`, `CORE-3`, "identical to the CPU" and "== cpu". `docs/design/superseded/` is kept unedited by rule and is not listed. Hits that speak of the CPU alone (goldens, replays, CPU-only scenes) or of the GPU against itself are kept and not listed, except where a reader could mistake them. "Keep" rows are listed where the reason matters. `AGENTS.md` and `CLAUDE.md` have no hit; `AGENTS.md`'s "the same seed and inputs should give the same results" holds per backend. Physics' read-only survey of master supplied the zero-row list, the GNSS items, `ImuRingPoll`, the cross-backend restore tests and `StoredFieldSamplesMatchTheCpuCopiesBitwise`; each was checked against the code. Changes ride each realm's branch, comment-only edits included.

### Core (34 rows)

| File:line | States | Change | Why |
|---|---|---|---|
| `engine/compute/vulkan/context.cpp:70-110` | `device_preserves_fp32_denormals()`; "the GPU must match it bit for bit" | Replace with the float-controls read for the report | The clause goes; the read stays |
| `context.cpp:216-257` | The predicate's denormal clause, and its `#if` | Delete; keep a Vulkan 1.1 clause | Admission |
| `context.cpp:264-269` | The measurement marker | Delete | The option retires |
| `context.cpp:286-291` | "or the default device cannot preserve fp32 denormals" | Reword: "or the default device reports Vulkan below 1.1" | Admission |
| `context.cpp:404-453` | The refusal: "GPU results must be bit-identical to it" | Delete; record the report | The ruling |
| `engine/compute/vulkan/context.hpp:43-60`, `:165-199` | `create()` and `vulkan_available()` docs: the denormal refusal | Reword; document `device_report()` | Admission |
| `engine/compute/vulkan/step_recorder.cpp:110-112` | Cites context's denormal check as its model | Reword; keep the local-size argument | The cited check is gone |
| `engine/compute/backend.hpp:82-85` | "same device, knob varied, GPU results must stay bit-identical" | Keep | GPU against itself |
| `engine/CMakeLists.txt:1020-1029` | The measurement definition on `spade_compute` | Delete | The option retires |
| `engine/CMakeLists.txt:1165-1169` | "P3 (fp32 denormals preserved)" | Reword to the new `P3` | `P3` changes |
| `engine/shaders/fp32_math.slang:416-430` | `log32`'s pre-scale "depends on denormal behaviour" | Integer form (item 3); reword | Item 3 |
| `fp32_math.slang:490`, `:497-502` | `exp32`'s tail, "second denormal-behaviour dependency" | Integer form; reword | Item 3 |
| `fp32_math.slang` `reduce_quadrant` (`:363-386`) | `sin32` of a subnormal | Early return; comment | Item 3 |
| `fp32_math.slang:100-101`, `:128` | OpFDiv "would stop being bit-identical"; edge cases "bit for bit" | Keep | A library property, which item 3 preserves |
| `engine/core/fp32_math.cpp:293-299`, `:426-431`, `:434-449` | The CPU twins of the three paths | Integer form, same bits | Twins stay statement for statement |
| `engine/core/rng.hpp:226-229` | "the gaussian sequence is now an EXACT parity claim" | Reword: exact on the CPU across toolchains; banded on the GPU (`CORE-3`) | It was never true on the GPU |
| `engine/shaders/rng.slang:24-75` | `CORE-3`'s record; "Tightening is REFUSED" | Add the NVIDIA measurement; drop the refusal | `CORE-3` changes |
| `engine/shaders/u64.slang:20` | "the same shape of hazard the denormal mode was" | Keep; one-word tense check | Still true as history |
| `engine/shaders/kernels/fp32_math_probe.slang:46` | `u64` must "match the host bit for bit" | Keep | Integer arithmetic; exact by construction |
| `engine/sim/simulation.cpp:1228`, `engine/sim/simulation.hpp:867` | A wrong-clock receiver: "both backends still agree bit for bit" | Reword: "agree within their bands" | The ruling |
| `simulation.hpp:1073-1080` | Same-backend resume bit-identical; cross-backend banded | Keep | Already the policy |
| `tests/test_compute_context.cpp:50-117` | `ADeviceThatCannotPreserveFp32DenormalsIsRefused` | Invert: admitted and announced | Admission |
| `test_compute_context.cpp:303-345` | The forced-path discriminator is the token `shaderDenormPreserveFloat32` | New token from the Vulkan 1.1 clause | The token leaves the message |
| `tests/gpu_skip.hpp:1-7`, `:36-109` | A skip names the missing denormal capability | Drop that reason; keep the read for the new test | Admission |
| `tests/test_gpu_fp32_math.cpp:1-14` | "the only measurement that answers it is equality of bits … 1-ulp … is a divergence" | Reword: a library property, not a parity requirement | The ruling |
| `test_gpu_fp32_math.cpp:320-333` | "two lines of the port would answer differently on a device that flushed" | Reword after item 3 | No longer true |
| `test_gpu_fp32_math.cpp` batteries (`:396-858`) | Bit-for-bit against the host | Keep exact | Item 3 makes them device-independent (section 6) |
| `tests/test_gpu_module_schedule.cpp:93-127` | `StoredFieldSamplesMatchTheCpuCopiesBitwise` | Keep | Copies |
| `tests/test_gpu_invariance.cpp:1-30`, `:296-626` | Workgroup-size and batching invariance, bit-identical | Keep | GPU against itself |
| `docs/design/core/00-decisions.md:39` | `CORE-3` | **Cameron**, below | A signed row |
| `docs/design/core/04-api-and-backend.md:34` | "The device must preserve fp32 denormals. `create()` refuses one that does not." | Replace with the admission and report (`CORE-5`) | The ruling |
| `04-api-and-backend.md:18` | `CORE-3` summary | Add: every Vulkan result is banded, and differs slightly from the CPU | The ruling |
| `docs/design/core/07-status.md:25` | "denormal-preserving device" | Reword after T4 | Status |
| `docs/design/core/plans/` (`2026-10-02-module-api-design.md:127`, `:171`; `-plan.md:1253`; `2026-10-03-module-api-stage3-plan.md:20`, `:379`; `2026-10-05-module-api-stage4-plan.md` global constraints) | Held-body pos and vel bit-exact on Vulkan; stored samples bitwise; "every GPU parity band holds unchanged" | Keep, except stage 4's band constraint, which reads "as at its cut" | Copies are exact; plans are records |

The report and the measurement plan stay as records. `CHANGELOG.md:66` ("Nine of twelve … bit-exact between backends") is history and stays; Core adds an entry with T4.

### Test/Docs (31 rows)

| File:line | States | Change | Why |
|---|---|---|---|
| `cmake/SpadeSlang.cmake:80-139` | "WHY -denorm-mode-fp32 preserve IS ON EVERY KERNEL COMPILE" | Rewrite: why no denormal mode, with Intel's recorded default and the UB reason | `P3` changes |
| `SpadeSlang.cmake:142-151` | `_spade_slang_denorm_args` | Delete; no flag in any build | (b) |
| `SpadeSlang.cmake:342-358`, `:426-454` | The flag and "denorm preserve" in each COMMENT | Drop | (b) |
| `engine/testing/spirv_scan.hpp:42-61` | `P3`: "FP32 DENORMALS ARE PRESERVED" | Rewrite as the new `P3` | `P3` changes |
| `spirv_scan.hpp:144-146` | `P3`'s history | Add the 2026-10-05 change | Record |
| `spirv_scan.hpp:291-298` | `kExecutionModeDenormPreserve` only | Add `DenormFlushToZero` and both capabilities | The new rule checks all four |
| `spirv_scan.hpp:315`, `:320-333` | `"P3-unpinned"`, `DenormPolicy` | Delete | One rule in every build |
| `spirv_scan.hpp:385`, `:418`, `:461-468` | `denorm_preserve_fp32` detection | Detect any denormal mode or capability | The new rule |
| `spirv_scan.hpp:620-648` | The `P3` and `P3-unpinned` findings | One finding: "a denormal mode is requested" | The new rule |
| `spirv_scan.hpp:45`, `:64-65`, `:621` | "this program's correctness device", the Iris | Reword device-neutrally | The Iris is gone |
| `spirv_scan.hpp:116-120`, `:557-559` | Ports "proven bit-identical by tests/test_gpu_fp32_math.cpp" | Keep | Library property |
| `tests/test_slang_layouts.cpp:34` | The gate's description | Reword | `P3` changes |
| `test_slang_layouts.cpp:70-71` | The probe's include under the flag | Per T6 | The option retires |
| `test_slang_layouts.cpp:399-427`, `:446-449`, `:467-468`, `:501` | Profile notes on "P3's denormal pin" | Reword | `P3` changes |
| `test_slang_layouts.cpp:544-560` | `kDenormPolicy` | Delete | One rule |
| `test_slang_layouts.cpp:564-637` | `FloatControlsPinned`'s `P3` assertions | Assert no denormal mode | The new rule |
| `test_slang_layouts.cpp:761-850` | `RuleP3InvertsUnderTheUnpinnedPolicy` and helpers | Replace with `RuleP3RefusesAnyDenormalMode` | The new rule |
| `test_slang_layouts.cpp:959-962` | `kSingleVariantKernels` under the flag | Per T6 | The option retires |
| `CMakeLists.txt:93-126` | The option, the configure guard, the install guard | Delete | The option retires |
| `tests/CMakeLists.txt:404-411` | The test definition and the probe source under the flag | Delete; probe per T6 | The option retires |
| `scripts/denorm-marker-scan.sh` | The install marker scan | Delete | Nothing to scan for |
| `scripts/docker-leg.sh:232-243`, `:257`, `:311-312` | The leg's `denorm` step and summary line | Delete | As above |
| `docs/design/test-docs/00-decisions.md:20` | `SPIR-V rule P3` | **Cameron**, below | A signed row |
| `00-decisions.md:48-53` | `TD-13` and its interim reading | **Cameron**, below | The reading ends |
| `00-decisions.md` (new) | `TD-14`, the band policy | **Cameron**, below | It defines "extremely low error" |
| `docs/design/test-docs/02-build-and-gate.md:18`, `:24-25`, `:54` | The marker scan, the option, the `denorm` summary | Delete | The option retires |
| `docs/design/test-docs/07-status.md:35-38`, `:51`, `:127`, `:150` | 72 refused; "GPU unverified" | Update after T4 | Status |
| `docs/design/test-docs/01-verification.md:39` | Parity kernels obey `P1`–`P5` | Keep; add the new `P3` in a clause | Accuracy |
| `docs/design/test-docs/08-lessons.md:29` | The old iGPU flushed unless told | Keep | A lesson |
| `CONTRIBUTING.md:31` | CPU↔GPU compared "inside a measured band" | Keep; link the user note | Already the policy |
| `README.md:3` | "snapshot and replay them bit-for-bit" | Keep; add the note (section 5) | CPU replay is exact |

### Physics (27 rows)

| File:line | States | Change | Why |
|---|---|---|---|
| `engine/testing/parity.hpp:27-47` | Why bands: transcendentals "proven bit-identical to the host" | Rewrite: banded everywhere; `fp32_math` exact as a library | The ruling |
| `parity.hpp:66-91`, `:420-454` | The disjunctive predicate | Sum form; `A`/`R` split | Section 2 |
| `parity.hpp:481-526` | Iris-only provenance; "WHOLE run"; the stop rule | Per-device record template; "final tick"; stop rule kept | `L4`, correction |
| `parity.hpp` (41 rows) | Zero bands | Classify: 25 structural, 14 floor, 2 `CORE-3` | Section 2 |
| `parity.hpp:672-734` | `bounce`: "ALL ZERO, WHICH IS A CLAIM AND IS MEANT TO BE" | Rewrite: structural rows exact; pos and vel at the floor | The ruling |
| `parity.hpp:1242-1264` | `two_world_isolation` vel {5e-28, 2e-4} | Re-pin | Section 2 |
| `parity.hpp:1174` | `medium`: "all T4 ports proven bit-identical to the host" | "measured exact" | Wording |
| `parity.hpp:1385-1440` | `gnss_receiver` bias "pre-registered … BIT-IDENTITY claim"; "silent expiry date" | Reword; state the horizon | The ruling |
| `parity.hpp:932-965` | The 400-step horizon | Keep; point to the horizon rule | Section 2 |
| `tests/test_gpu_parity.cpp:147-152` | `force_acc`/`torque_acc` zero "DELIBERATELY" | Keep | Structural |
| `test_gpu_parity.cpp:614` | `gnss_receiver` cached at {0, 0} | `CORE-3` band | Gaussian |
| `test_gpu_parity.cpp:631-740` (discriminator) | Bias and cached at {0, 0}; its outcome table | `CORE-3` band; rewrite the table | Gaussian |
| `test_gpu_parity.cpp:820-917` | `GnssDrawsDivergeAcrossBackends_KNOWN_OPEN`: an inverted assertion | Replace with `GnssDrawsMatchTheCpuWithinTheCore3Band` | On a device with a correctly rounded `sqrt` it would fail for no defect |
| `test_gpu_parity.cpp:3062-3146` | `ImuRingPoll…`: `worst_gyro <= kGyro.abs`, i.e. `<= 0` | Use the band predicate per component | The ruling |
| `test_gpu_parity.cpp:2411` | `ParityChaos` | Keep; generalise per scenario | Horizons |
| `test_gpu_parity.cpp:3022`, `:3030` | Run-to-run, bit-identical | Keep | GPU against itself |
| `tests/test_gpu_invariance.cpp:853`, `:921` | Cross-backend restore, `restore_resume`'s bands | No edit; follows the re-pin | Shared bands |
| `engine/shaders/kernels/sensor_gnss.slang:24-35`, `:46` | "a BIT-IDENTITY claim rather than a banded one"; cached "bit-identical across backends" | Reword: the multiply is exact; the draws are `CORE-3`-banded | The ruling |
| `engine/sensors/gnss.cpp:53-56`, `:75-80` | The same two claims | Reword | As above |
| `engine/world/medium.cpp:29-36` | "PARITY HERE IS AN EXACT CLAIM" | Reword: exact across CPU toolchains; banded on the GPU (it uses `sqrt` and division) | It was never true on the GPU |
| `engine/vehicles/rotor.hpp:406-413` | "THAT PARITY IS AN EXACT CLAIM … must agree bit for bit" | Reword, as above | As above |
| `dryden.slang:66`, `collision_static.slang:64`, `collision_dynamic.slang:108`, `rotors.slang:106`, `sdf_eval.slang:48` | "banded rather than bit-identical" | Keep | Already the policy |
| `engine/physics/grid.hpp:148`, `collision_dynamic.slang:17` | "making S6 bit-identical means reproducing this sweep" | Keep | Op order still sets the band |
| `docs/design/physics/04-verification.md:20-21` | "pinned at zero because … bit-identical across backends"; "CPU↔Vulkan snapshot restore are bit-identical" | Reword (the restore is exact; the continuation is banded) | The ruling; the second was already wrong |
| `04-verification.md:32-33` | "banded (bit-identical rows)", "(bit-identical contact pairs)" | "banded" | The ruling |
| `docs/design/physics/03-sensors.md:33` | `KNOWN_OPEN` "asserts on purpose" | Reword to the banded test | Follows the test |
| `engine/shaders/kernels/denorm_probe.slang`, `tests/test_gpu_denorm_probe.cpp` | Built only in the measurement tree | Standing report test, or delete (T6) | The option retires |

### Rendering (1 row)

| File:line | States | Change | Why |
|---|---|---|---|
| `engine/shaders/kernels/raster_background.slang:20` | fp32 on both sides "lets tests/test_gpu_parity.cpp pin most quantities BIT-EXACT" | Reword: "tightly" | The ruling |

### Lead (4 rows)

| File:line | States | Change | Why |
|---|---|---|---|
| `docs/design/backlog.md:59` | `P3` becomes "requested where supported, not required" | "never requested" | That wording is option (a) or (c), not (b) |
| `backlog.md:50`, `:57`, `:71` | The refused device, `TD-13`'s reading, "measure NVIDIA" | Close after T4 | Done |
| `backlog.md:70` | The open item | Close when its "done when" is met | Done |
| `README.md` | No statement that CPU and GPU differ | Add the note (section 5) | The ruling |

Interface has nothing to change: `interface/plans/2026-10-02-v1-baselines.md:15` and `engine/tools/viewer/main.cpp:25`, `:105-109` already say Vulkan is banded. Its follow-up is showing the device report.

### Rows that need Cameron's own confirmation

The charter's laws need no change. Five register rows do.

**1. `SPIR-V rule P3`** (`test-docs/00-decisions.md:20`)

BEFORE:
> | `SPIR-V rule P3` | fp32 denormals preserved, declared in the module | signed 2026-10-02 (user, via lead); in force since S6 Task 4 | live | as `P1` |

AFTER:
> | `SPIR-V rule P3` | No fp32 denormal mode is requested: no module declares a `DenormPreserve` or `DenormFlushToZero` execution mode or capability. Every device then runs the kernels legally, with its own default | signed 2026-10-02 (user, via lead); amended by the user's ruling of 2026-10-05 | live, as amended. Was: "fp32 denormals preserved, declared in the module" | as `P1`; `../core/plans/2026-10-05-banded-parity-plan.md` |

**2. `TD-13`** (`test-docs/00-decisions.md:48-53`)

BEFORE: the row's status reads "live, under an interim reading on this machine (below)", followed by the paragraph "**`TD-13`'s interim reading on this machine.** The user ruled on 2026-10-05 … 'Label, don't block.' … It lasts until the NVIDIA denorm measurement settles device admission … and then `TD-13` is read as written again, or amended by a new ruling." and its three bullets.

AFTER: the ruling text unchanged; the status reads:
> live, read as written. The interim reading of 2026-10-05 ("Label, don't block") ended when the RTX 3060 Ti was admitted under the ruling of 2026-10-05 (`../core/plans/2026-10-05-banded-parity-plan.md`)

and the paragraph is replaced by one line:
> **`TD-13`'s interim reading has ended.** It covered the days the RTX 3060 Ti was refused. Every `gpu` test runs on it now (`07-status.md`).

**3. `CORE-3`** (`core/00-decisions.md:39`)

BEFORE:
> | `CORE-3` | The gaussian draw is CPU↔GPU banded at its source, the Box-Muller `sqrt` in `rng` / `rng.slang`. Vulkan allows ≤ 2.5 ulp where IEEE 754 mandates correct rounding; `log32`/`sin32`/`cos32` and the integer state are bit-identical. Measured: max abs `4.76837158e-07`, max rel `5.06893741e-07`, 11 of 64 ring elements outside a zero band, on an Intel Iris Plus Graphics (Vulkan 1.3.215, driver 31.0.101.2125, msvc-ninja-release), isolated through a GNSS receiver (200 substeps, `bias_tau_s = 1e-6`, `mount_pos = 0`). Scope is by call: `dryden.slang`, `sensor_gnss.slang`, `sensor_imu.slang`. Downstream bands (Physics) cite this one rather than re-deriving a cause. Tightening is refused, because the platform cannot keep it. `GnssDrawsDivergeAcrossBackends_KNOWN_OPEN` asserts the divergence exists, so it goes red if anyone tightens the `sqrt`. The CPU-path goldens are unaffected | signed 2026-10-02 (user, via lead) | live (declared 2026-09-24; the measurement is also recorded in `rng.slang`'s header) | `superseded/2026-09-consolidation/01-charter.md` §4.1 |

AFTER:
> | `CORE-3` | The gaussian draw is CPU↔GPU banded at its source, the Box-Muller `sqrt` in `rng` / `rng.slang`. Vulkan allows ≤ 2.5 ulp where IEEE 754 mandates correct rounding. The integer stream state is exact by construction, and `log32`/`sin32`/`cos32` match the host on every device measured; neither is a parity requirement. Measured, isolated through a GNSS receiver (200 substeps, `bias_tau_s = 1e-6`, `mount_pos = 0`), against a zero band: on an Intel Iris Plus Graphics (Vulkan 1.3.215, driver 31.0.101.2125, msvc-ninja-release), max abs `4.76837158e-07`, max rel `5.06893741e-07`, 11 of 64 ring elements differ; on an NVIDIA GeForce RTX 3060 Ti (Vulkan 1.4.303, driver 572.83, msvc-ninja-release), max abs `4.76837158e-07`, max rel `1.06963546e-06`, 19 of 64 differ. Scope is by call: `dryden.slang`, `sensor_gnss.slang`, `sensor_imu.slang`. Downstream bands (Physics) cite this one rather than re-deriving a cause. A tighter implementation is allowed but not required. The CPU-path goldens are unaffected | signed 2026-10-02 (user, via lead); amended by the user's ruling of 2026-10-05 | live, as amended (the measurements are also recorded in `rng.slang`'s header) | `superseded/2026-09-consolidation/01-charter.md` §4.1; `plans/2026-10-05-banded-parity-plan.md` |

**4. `CORE-5`, new** (`core/00-decisions.md`)

> | `CORE-5` | **Any Vulkan 1.1 or later device is admitted, whatever its fp32 denormal support.** No kernel requests a denormal mode (`SPIR-V rule P3`), so each device's own default applies. The context records the device's name, driver and float controls, and `Simulation::vulkan_device_report()` returns them, so every Vulkan run can say which device and driver produced it (`L6`). Vulkan results are banded against the CPU (`L3`, `L4`; `TD-14`) and deterministic per device and driver | the user's ruling of 2026-10-05; wording confirmed by the user | live once T4 merges | `plans/2026-10-05-banded-parity-plan.md` §1 |

**5. `TD-14`, new** (`test-docs/00-decisions.md`)

> | `TD-14` | **Banded CPU↔GPU parity.** An element passes when `|gpu − cpu| ≤ abs + rel × |cpu|`; a NaN fails. A band is measured on every device of record and pinned at 4× the largest measurement, rounded up to one significant figure, with `L4`'s record: device, driver, commit, scenario, horizon, method and margin. A float quantity measured exactly equal everywhere gets a floor of four ulps. Only integers, copies and quantities no rounding operation touches keep an exact band, each with its argument. A band is pinned only at a horizon where one ulp of input, amplified by the CPU alone, stays under 1e-5 of the scene's scale; past it, invariants and statistics replace element bands. The target is 1e-4 relative; a quantity that needs more than 1e-3 is a grade change (`TD-2`) | the user's ruling of 2026-10-05 ("basically the same, extremely low error"); wording confirmed by the user | live once T3 merges | `../core/plans/2026-10-05-banded-parity-plan.md` §2 |

## 5. The user-facing note

**Where.** `README.md`, "What it promises", as a new bullet after "Every module declares a grade". README is the only user-facing document today. Interface owns user guides (`02-realms.md`) and carries the same paragraph into the first one. `CONTRIBUTING.md:31` links to it. The numbers trace to `docs/design/physics/04-verification.md`.

**The paragraph:**

> - **CPU and GPU results differ slightly.** The CPU and the GPU (Vulkan) compute the same steps by different hardware routes, so their results are not identical. In every scene we test, over the first second or two of simulated time, they agree to within one part in ten thousand (0.01%) of each value, and usually to a few parts in a million. Positions agree to within a few hundredths of a millimetre. Values near zero, such as the speed of a body resting on the ground, can instead differ by a tiny absolute amount, up to about 0.2 mm/s. Over long runs, and in chaotic scenes such as a pile of colliding bodies, the two drift apart the way any two slightly different calculations do: individual bodies end up in different places, while the scene as a whole behaves the same. Each path is deterministic on its own. The CPU gives the same bits for the same seed and inputs, and its goldens are checked on Windows with MSVC and on Linux with gcc. The GPU gives the same bits on the same device and driver. The CPU is the reference: goldens, replays and any run that must reproduce exactly use the CPU backend. The GPU is never a golden source. On Vulkan, `Simulation::vulkan_device_report()` names the device and driver a run used.

The numbers come from M2 (largest genuine relative difference 2.5e-5; largest position difference 1.35e-5 m; `gate_fleet`'s resting body at 1.6e-4 m/s). The lead re-checks them against Physics' final pins before the note merges.

## Tasks, in order

**Every task's gate,** run in the foreground at the task's head before "ready for review":
1. `scripts\build.ps1 -Preset debug -ParallelLevel 8` and `-Preset release -ParallelLevel 8`;
2. `scripts\test.ps1 -Preset debug` and `-Preset release`: 0 failed, skips named, counts with tree and commit (`TD-7`, `TD-8`);
3. `scripts\docker-leg.ps1 -Memory 8g -Jobs 8`: PASS. One fresh leg at a time on this machine (the 10 GB VM cap).

Until T4 merges, the 3060 Ti is refused in the default build, so a task that touches the GPU path also runs `ctest --test-dir <its measurement tree> -L gpu` on the 3060 Ti and reports it. That tree is the realm worktree's `build-ninja/release-unpinned`, configured with `SPADE_MEASURE_UNPINNED_DENORMS=ON`.

**Merge order is a safety rule:** T4's admission never lands before T2's kernels. Admitting the 3060 Ti while kernels still request `DenormPreserve 32` is undefined behaviour.

**T1. Core: `fp32_math` independent of the denormal mode** (item 3).
- Files: `engine/core/fp32_math.cpp`, `engine/shaders/fp32_math.slang`, `tests/test_fp32_math.cpp`, comments in `tests/test_gpu_fp32_math.cpp`.
- Red: a new host test, `Fp32Math.SubnormalPathsDoNotDependOnTheFlushMode`. On x86 it sets MXCSR FTZ and DAZ, evaluates `log32` over every positive subnormal, `exp32` over its 2,180,453-argument tail, and `sin32`/`cos32` over every ±subnormal, and compares the bits with the same calls under the default MXCSR. It restores MXCSR on every exit, and skips by name off x86 (`TD-5`). It fails today in the same three places NVIDIA does, on both toolchains. In the measurement tree, `GpuFp32Math`'s four failing batteries are red too.
- Green: the integer forms. The new test passes, every existing `fp32_math` test is unchanged, no golden moves (`Determinism.DigestsMatchTheCommittedGoldenCorpus`), and the measurement tree runs all 13 `GpuFp32Math` batteries green on the 3060 Ti.

**T2. Test/Docs: no denormal mode in any kernel; the new `P3`.** Parallel with T1.
- Files: `engine/testing/spirv_scan.hpp`, `cmake/SpadeSlang.cmake`, `tests/test_slang_layouts.cpp`; the `P3` comment at `engine/CMakeLists.txt:1165-1169`, agreed with Core.
- Red: `SlangSpirv.RuleP3RefusesAnyDenormalMode` on synthetic modules (`DenormPreserve 32`, `DenormFlushToZero 32`, a capability alone: each one finding; none: clean), and `FloatControlsPinned` asserting no mode. It fails on all 43 modules, which still carry `DenormPreserve 32`.
- Green: drop `-denorm-mode-fp32 preserve`. The merge note carries a sha256 table showing the default build's `.spv` files are byte-identical to the measurement tree's. That is what lets T3 measure there.
- The default build still refuses the 3060 Ti. That is legal now, just unneeded; T4 removes it.

**T3. Physics: the bands under `TD-14`.** After T1 and T2, and after `physics/airframe-compile` merges (it touches `physics/04-verification.md`).
- Files: `engine/testing/parity.hpp`, `tests/test_gpu_parity.cpp`, `docs/design/physics/04-verification.md` and `03-sensors.md`, and the comments in `sensor_gnss.slang`, `gnss.cpp`, `medium.cpp` and `rotor.hpp`.
- Red, host-only: `ParityPredicate.SumFormAtTheBoundary`. An element at `abs + rel × |c|` minus one ulp passes and one ulp past it fails; the disjunction fails the first. On the device: the eight failing tests and the two cross-backend restore tests, in the measurement tree, pre-registered at today's bands so the failure report is the measurement.
- Work: the `A`/`R` split; re-measure every parity scenario on the 3060 Ti; classify the 41 zero rows; re-pin with `L4` records, keeping the Iris numbers as history; the GNSS rework (`KNOWN_OPEN` replaced, the discriminator and cached rows on `CORE-3`, `ImuRingPoll` on the predicate); the one-ulp CPU control for the gate-geometry scenarios; the `parity.hpp` header.
- Green: every `Gpu*` test passes in the measurement tree on the 3060 Ti.

**T4. Core: admit and announce.** After T3.
- Files: `engine/compute/vulkan/context.cpp` and `.hpp`, `engine/compute/backend.hpp`, `engine/sim/simulation.hpp` and `.cpp`, `step_recorder.cpp:110-112`, `engine/CMakeLists.txt:1020-1029`, `tests/test_compute_context.cpp`, `tests/gpu_skip.hpp`, `engine/core/rng.hpp:226-229`, `engine/shaders/rng.slang:24-75`, `docs/design/core/04-api-and-backend.md`, `07-status.md`, `CHANGELOG.md`.
- Red:
  - `GpuContext.ADeviceWithoutFp32DenormalPreservationIsAdmittedAndAnnounced` replaces the refusal test. Where the driver says preserve is false (this box) it expects admission and a report naming the device. Elsewhere it asserts admission and skips the rest by name.
  - `GpuContext.ReportsTheDeviceDriverAndFloatControls`: the report's fields equal a direct driver read (`gpu_skip.hpp`'s, an independent source, `TD-4`).
  - `ComputeBackendAvailability.ForcedUnavailableReturnsUnavailable` with its new discriminator.
- Green: implement. The default build then runs every `gpu` test on the 3060 Ti.
- Gate: as above, with every `gpu` test running and none refused. This is the first gate on this machine that reads `TD-13` as written.

**T5. Test/Docs: retire the measurement build.** After T4.
- Files: `CMakeLists.txt:93-126`, `tests/CMakeLists.txt:404-411`, `scripts/denorm-marker-scan.sh`, `scripts/docker-leg.sh`, `tests/test_slang_layouts.cpp` (`:70-71`, `:959-962`, per T6), `docs/design/test-docs/02-build-and-gate.md`, `07-status.md`, and `00-decisions.md` once Cameron confirms.
- Red/green: no behaviour changes, so there is no new test. The evidence is the gate: `EveryCompiledVariantIsScanned`'s count still holds, and the leg's summary has no `denorm` line.

**T6. Physics: the probe as a standing device record** (Physics decides; the alternative is deletion in T5).
- Files: `engine/CMakeLists.txt:1143-1149`, `tests/CMakeLists.txt:411`, `tests/test_gpu_denorm_probe.cpp`, a `kSpirvModules` row.
- `denorm_probe` compiles in every build. `GpuDenormProbe.EveryFp32OpClassIsExplainedByAFlushVariant` runs on every device, prints its class table (the device's default, on record in every gate log), and asserts only its harness rules: no "other", no contraction.
- Red/green: the test exists and passed its four rules on the 3060 Ti in M1; it moves into the default build. The red is its absence from the default build's test list; the green is its run there.

**T7. Core: GPU determinism per device, pinned wider.** After T4.
- Files: `tests/test_gpu_invariance.cpp`, `tests/test_gpu_parity.cpp`.
- New: `GpuInvarianceTest.VulkanSnapshotRestoresIntoVulkanBitIdentically`. Step on Vulkan to tick 300, snapshot, restore into a fresh Vulkan `Simulation`, resume to 700, and require the uninterrupted Vulkan run's `state_digest`. Run-to-run determinism extends to every corpus scenario.
- Red/green (`SL18`): remove `restore()`'s dirty mark (the C1 fix `parity.hpp`'s `restore_resume` note describes), and the new test fails; put it back, and it passes.

**T8. Lead: docs on master.** The README note, after T4. The backlog rows. The five register rows as Cameron confirms them: ask before T2 merges for `P3`, before T3 for `TD-14`, before T4 for `CORE-3` and `CORE-5`, and before T5 for `TD-13`. Interface's device-report display, through the lead.

## What stays bit-exact, and what pins it

| Property | Pinned by |
|---|---|
| CPU goldens, one set across MSVC and gcc | `Determinism.DigestsMatchTheCommittedGoldenCorpus`; `BackendSeamReproof.CpuPathCorpusDigestsUnchangedAtTip`; `BackendKnobInvariance.CpuBackendReproducesTodaysDigestsAcrossTheFullCorpus`; the gcc leg (`TD-12`) |
| CPU snapshot, restore and replay | the snapshot and replay suites (`L2`) |
| `fp32_math` across CPU toolchains, and on every device measured | `BitPortability` and `test_fp32_math.cpp`; the 13 `GpuFp32Math` batteries; T1's flush-mode test |
| GPU against itself, per device and driver | `QuadHoverIsBitIdenticalAcrossTwoGpuRuns`, `ShowerIsBitIdenticalAcrossTwoGpuRuns` and T7's extension; the seven `…BitIdenticalAcrossWorkgroupSizes` tests; `TwoWorldIsolationWorldsMatchSoloRunsOnGpu`; T7's Vulkan-to-Vulkan restore; `RoundTripUploadReadbackWithoutSteppingIsByteIdentical` |
| Integer state, CPU against GPU | the `QuantityKind::bits` rows; `GpuGridSort` and `GpuGridSortDeepTest`; `GpuU64.ShiftMatchesTheHostAtEveryDistance` |
| Copies and structural zeros, CPU against GPU | `StoredFieldSamplesMatchTheCpuCopiesBitwise` (gravity, density); `force_acc`, `torque_acc` and the 25 structural rows |

No GPU result is ever committed as a digest (`L4`, `TD-1`); GPU determinism is always checked within one process.

## Risks and open questions for the lead

1. **Structural zero rows.** The reading says the zero bands go. This plan keeps 25 rows, plus `force_acc`, `torque_acc` and the integer lanes, because no rounding touches them; they claim "no arithmetic happened", not "the two arithmetics agree". *Recommend:* keep them, each with its argument, and confirm this reading.
2. **`GpuFp32Math`: exact or ulp bounds?** Physics' survey suggests ulp bounds. `fp32_math` contains no `OpFDiv` and no `Sqrt` (rules `E1`, `E2`), so M1's division and square-root finding cannot reach it. M2 found it exact everywhere except the three paths T1 removes. *Recommend:* keep the batteries exact as a library property, not a parity rule. If a future device fails one, that is a finding (an ignored `NoContraction`, or a non-IEEE add or multiply), handled by a recorded band for that device, never a refusal.
3. **The announcement.** *Recommend:* a query and first-party front ends that print it, no library stderr. If the lead wants a log line at `create()`, it goes through a sink the caller installs, like `set_error_sink`.
4. **Unmeasured devices (AMD, other NVIDIA drivers).** Under the ruling there is no per-driver admission (the report's option 4). *Recommend:* admit, and report the device and driver. Bands come from the devices of record; Physics measures a new device when one is available, and its numbers join the record.
5. **`heterogeneous_geometry_set` may be past its horizon.** NVIDIA's 1.35e-5 m is 30× the Iris figure. *Recommend:* T3 runs the one-ulp control first. If the scene is amplifying at 900 steps, shorten its horizon rather than widen its band.
6. **The predicate change.** Sum form is at most 2× looser than today's disjunction. *Recommend:* adopt it now, while every band is re-measured anyway.
7. **Stage 4's constraint "every GPU parity band holds unchanged".** If stage 4 is cut before T3 merges, the two collide. *Recommend:* stage 4 reads it as "at its cut", and whichever merges second re-runs `ctest -L gpu`.
8. **Gate time.** Every gate now runs 85 or more `gpu` tests on this box (`GpuFp32Math` alone took about 17 s in M2). *Recommend:* measure at T4 and record it in `test-docs/07-status.md`.
9. **The Iris is gone.** Its bands become history, and nothing here can re-run them. Under (b) the Iris would flush by default, as recorded, and M3's port predicts no state bit moves. That is a prediction, not evidence.
10. **The user note's numbers.** They are M2's. *Recommend:* the lead checks them against T3's pins before the note merges.

## Where the ruling's reading meets the laws and the code

- **The backlog row disagrees with (b).** `backlog.md:59` says `P3` becomes "requested where supported, not required". That is option (a) or (c). Under (b), `P3` means "never requested", which is also the reading's own wording for (b). The row needs that change.
- **`L6` does not strictly apply.** Nothing is unavailable or downgraded under (b): the GPU is banded on every device. The announcement is good practice and this plan builds it, but it is not `L6`'s requirement. And since no device gets a mode, "a device running without the denormal mode" is every device; the report says what each device would support.
- **`L3`'s grade check does not exist yet** (module-API stage 5). A consumer cannot yet ask for reference grade and be refused on Vulkan. That gap predates the ruling. Until stage 5, the docs carry it: Vulkan is banded.
- **"CPU determinism per platform" understates what the code holds.** `TD-1` forbids per-platform goldens and `TD-12` makes the gcc leg reproduce them, so the CPU is bit-identical across MSVC and gcc. The user note says so.
- **"GPU deterministic against itself" can only be pinned in-process.** `L4` and `TD-1` forbid a GPU golden, so no committed GPU digest exists to compare against. T7 widens the in-process checks.
- **`CORE-3`'s "log32/sin32/cos32 … are bit-identical"** is not false after T1; it is measured. The amended row keeps it as a measured fact, not a requirement, and drops "Tightening is refused", which would make a correctly rounded `sqrt` a failure.
- **`TD-2` is met, not bent.** The widened rows cite their cause (report finding 4), so no band is widened "without a root cause".
- **User guides are Interface's** (`02-realms.md`); the reading names Core, Test/Docs and Physics only. The README is the lead's; Interface carries the note into its guides.

## Corrections to the inputs

- `parity.hpp:498` says bands were measured over "the WHOLE run". The harness compares the final tick only (the report's "Corrections").
- Physics' survey suggests `GpuFp32Math` "become ulp bounds" because of M1's division and square-root finding. That finding does not reach `fp32_math` (risk 2).
- M2 ran 84 `Gpu*` tests. Master now also builds `GpuDenormProbe` in the measurement tree (`6349c72`), so that tree runs 85.
- `context.cpp:97` queries float controls on Vulkan 1.1, where the structure is core only from 1.2; `gpu_skip.hpp:69` uses 1.2. T4's report uses 1.2.
