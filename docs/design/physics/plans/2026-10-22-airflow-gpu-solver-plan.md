# The GPU airflow solver (plan B) -- implementation plan

**Owner:** Physics, with Rendering on the publish chain. **Status:** DRAFT 2026-10-22, for the lead's review before any build (Cameron's order, 2026-10-22: the GPU solver comes after the D1 hover preview and before the actuator line, P4).

**Goal:** the airflow step, with solids and rotor discs, on the GPU, banded against the CPU reference, so a D1-sized film (6M cells, about 1300 steps) runs in minutes instead of hours.

**Design:** `2026-10-12-airflow-chain-runner-design.md` (plan B, Core-approved with A1-A5) and the spec, `2026-10-05-airflow-design.md` §4 (kernels, determinism, sizes). Solids: `2026-10-13-airflow-solids-design.md` (P2, P3). This plan adds no new design. It orders the work, names the tests and settles who builds what.

## Constraints (copied from the design and the laws)

- **G1.** Its own target, `spade_airflow_runner` in `spike/airflow/`. It is not linked into Simulation, makes no StepShape or mirror change, and installs no header.
- **G2.** Built on `VulkanContext` and Core's `dispatch_chain`. A fluid step is one recorded chain, resubmitted. Publish is a second chain.
  - *Amended 2026-10-22 (B5, the lead's ruling 2):* except the solids' CG, whose count the device decides. The host loops over five recorded chains (B5) and reads a 16-byte device decision word each CG iteration past the floor. B5.0 measures that readback's latency.
- **Determinism (spec §4.5, plan B §3).**
  - Each kernel mirrors plan A term for term: face order −x, +x, −y, +y, −z, +z; the lerp form; red then black.
  - No atomics: gathers only. Reciprocals are computed on the host, transcendentals use `fp32_math`, and there is no contraction.
  - Every reduction, dot and `remove_mean` uses the fixed fp32 4×4×4 tile order of `reduce.hpp`, with the same bits at every workgroup size.
  - Both twins flush subnormals (spec §4.5, amended 2026-10-22). No kernel requests a denormal mode, so the 3060 Ti's arithmetic is a sign-preserving FTZ+DAZ. The CPU's `fluid_step`, `spread_actuator`, `wall_load` and `publish_render_volume` run under MXCSR's FTZ and DAZ (`engine/core/detail/flush_subnormals.hpp`). Both detect tininess after rounding.
- **Grades (L3, L4, TD-14; spec §4.5, amended 2026-10-22).**
  - The CPU stays the reference and the golden; the GPU is never a golden source.
  - Each kernel is banded against its CPU twin until it is exact. A band is 4× the largest measurement, with a floor of 4 ulps, and holds to the horizon T_p. The target is 1e-4 relative; anything above 1e-3 is a grade change, never a wider band.
  - A kernel is exact when every division and square root in it is `div32_ftz` or `sqrt32_ftz`, and its other float operations are the twin's, in the twin's order. Then it equals its CPU twin bit for bit on each device and driver measured, and its band retires (B5 on).
  - The airflow module's GPU grade is **banded**, placed in `04-verification.md`. Its rows change as kernels go bit-equal.
- **Refusals by name (L6).**
  - Device limits: maxStorageBufferRange, the descriptor counts, the push size, the per-axis workgroup count, maxMemoryAllocationSize, and the workgroup size and invocations.
  - The VRAM budget.
  - Everything `check_grid()` already refuses.

## The memory cap on the box's 8 GiB GPU (RTX 3060 Ti)

- **The device holds what `check_grid()` already counts** (`fluid_bytes` + `solid_bytes`), plus three more items:
  - the render volume, FlowVolume's two RGBA16 halves at 16 B per cell;
  - one staging buffer, the size of the largest single upload or readback;
  - the timestamp pool.
- *Amended 2026-10-22 (B5; `fluid_bytes()` and `solid_bytes()` as coded, with P2c's weights and P2d's best x):*
- **Per cell:**
  - fluid about 104 B;
  - solids about 100 B: θ, the wall velocity, φ, the link and the layer 51 B; the regions 9 B; the CG's seven vectors, best x among them, 28 B; every level's face weights 12 to 14 B;
  - render volume 16 B;
  - total about 220 B.
- **Plus, outside `check_grid()`:** the tile partials (about 0.4 B a cell), the staging buffer (the largest upload, every level's weights: about 100 MB at the D1 box), the scalars, the region table and the timestamp pool.
- **The D1 box at 4 mm** (240 × 177 × 168, 7.14M cells) needs about 1.46 GiB: fluid 0.69, solids 0.67, render 0.11.
- **The cap** is `min(4 GiB, budget − usage − reserve)`.
  - 4 GiB is the spec's cap and `kGridByteCap`, the CPU ceiling.
  - budget and usage come from `VulkanContext::memory_budget()`, using `VK_EXT_memory_budget` or half the heap (A1/A2).
  - reserve is for the GL renderer's own textures. It is unmeasured, so it starts at 1 GiB, and B7 measures the renderer's VRAM during the D1 film.
- **At 4 GiB the grid tops out near 19.5M cells,** about 269³, or the D1 box at about 2.9 mm. A larger request is refused before any allocation, naming the bytes asked for and the cap.

## The kernels and their CPU twins

| Chain stage | Kernels (Slang, `engine/shaders/kernels/airflow_*.slang`) | CPU twin |
|---|---|---|
| reductions | `tile_sum` and `tile_dot` (4³ tiles, then tiles in order) | `reduce.hpp` |
| boundaries | `velocity_bc` (per face kind), `extrapolate` | `boundaries.cpp`, `fluid_step.cpp` |
| advection | `trace_rk2`, `advect`, `advect_back`, `maccormack_clamp` (per component) | `advect.cpp` |
| viscosity | `viscous_red`, `viscous_black` (no-slip ghosts) | `viscous.cpp` |
| sources | `actuator_gather` (each cell gathers its Peskin-4 weights over the disc points; no scatter) | `actuator.cpp`, `rotor_discs.cpp` |
| projection | `divergence` (with θ), `mg_smooth_red`/`black`, `residual`, `restrict`, `restrict_full`, `prolong`, `coarse`, `gradient_subtract` (with θ and the wall velocity) | `pressure.cpp`, `multigrid.cpp`, `projection.cpp` |
| solids | `pcg_update` (x, r, p axpy), CG dots by `tile_dot`, `region_mean` (per region, by tile), `solid_extrapolate` | `pcg.cpp`, `regions.cpp`, `fluid_step.cpp` |
| loads | `wall_load` (per cell −p·A_c, then `tile_sum`) | `wall_load.cpp` |
| publish (second chain) | `publish_fields` (velocity, pressure, \|ω\|, q̂, λ2 with P3's one-sided rule), `pack_f16` (integer rounding, `core/fp16.hpp`'s rule) | `publish.cpp` |

- **Static solids** (D1): `set_solids()` stays on the CPU. It builds θ, the wall velocity and the region labels, and they are uploaded once. Moving solids are a later plan.
- **The health check** (non-finite state, residual) is a `tile_sum` reduction plus a flag. It is read back once per step, 64 bytes.
  - *Amended 2026-10-22 (B5, the lead's ruling 2):* plus the CG's 16-byte decision word, read each iteration past the floor (twice a step at D1's 9 iterations).

## Tasks

Each task is red first. GPU tests are `Gpu`-prefixed, so they carry the ctest `gpu` label and skip by name without Vulkan (`gpu_skip.hpp`). Each kernel's test runs at 16³ against its CPU twin, and whole-step tests run at 32³.

- **B0, prerequisites** (cross-realm; see decision 1):
  - the airflow block in `bindings.slang`;
  - `VulkanContext::memory_budget()`;
  - the device-limit checks, as stage 7 §7.1 will call them.
- **B1, the runner** (plan B §2, §4 first):
  - `spike/airflow/`, with `ChainRunner` create, upload, read, record, `submit(n)` and `durations_ns`;
  - device-local buffers with one staging buffer.
  - Tests use copy and add-one kernels:
    - upload and read are exact;
    - writes are visible across a barrier;
    - n submits give n increments;
    - each refusal is named, including a `maxStorageBufferRange + 4` buffer refused before allocation;
    - timestamps are reported or empty.
  - A4: `tile_sum` and `tile_dot` give the same bits at workgroup sizes 32, 64 and 128, and the same bits as `reduce.hpp`.
  - *Amended 2026-10-21 (Physics' ruling on B1):* `tile_dot` moves to B5, its only consumer, the CG's dots. B1 delivers `tile_sum` on set 1, with push {field, pass, cell_count, slot}.
- **B2, the projection without solids:**
  - kernels: divergence, red-black smoothing, residual, restrict, prolong, coarse, gradient subtract;
  - per-kernel bands at 16³;
  - one V(2,2) projection at 32³ against `project()`;
  - timings at 64³ and 128³ (M1).
  - *Amended 2026-10-21 (the odd-axis multigrid, physics/mg-odd-axes):* the twin mirrors the CPU's one hierarchy rule, `coarser()` in multigrid.cpp:
    - halve every axis while all of them halve;
    - past that, only while the coarsest level holds more than `kCoarsestCells` cells, coarsen axis by axis: an odd closed axis goes to (n + 1) / 2 with a closed partial cell; a periodic axis halves only from a multiple of 4 and is otherwise held.
    - Restrict, prolong and plan A's coarse weights (`multigrid_plan_a_weight_faces()`) carry the partial cells and held axes the same way.
    - Plan A reports its residual on the health line every step and announces above `kPlanAResidualAnnounce`.
- **B3, the fluid step without solids:**
  - kernels: boundaries, advection, viscosity;
  - `fluid_step` parity at 32³ to T_p on the walled cavity and the nine boundary configurations;
  - past T_p, invariants (divergence, kinetic energy within its band);
  - the CPU goldens are not re-cut.
- **B4, rotor discs:**
  - `actuator_gather`, banded against `actuator.cpp` on the reference quad's four discs;
  - C8's actuator-disc case: the GPU's downwash ratio against the CPU's.
  - *Amended 2026-10-21 (C8's re-cut):* C8 on the GPU: banded against the CPU to T_p, then judged against momentum theory with the CPU's spread band (01-verification, 6b77cbc; the C8 spread study docs/design/test-docs/2026-10-22-c8-spread.md).
- **B5, solids (P2 on the GPU):**
  - θ in divergence and in the gradient subtract;
  - PCG with tile dots, with `kSolidsCgIterations`;
  - *Amended 2026-10-21 (the lead's ruling 1b):* the CG runs by residual: at least `kSolidsCgIterations`, then on to `kResidualAnnounce`, capped at `kSolidsCgCap` (32) and announced at the cap (`solve_pcg_to()`). The CPU records each step's count on the health line (`FluidHealth::pressure_iterations`). B5's parity runs the GPU at the CPU's recorded count, so a one-iteration difference in where the mark is met never flips parity.
  - region means and the extrapolated interior.
  - Bands measured on S1 and S3 (the spec's solids amendment), and S2 as a whole-step check.
  - `wall_load` banded against P3.
  - *Amended 2026-10-22 (bit-exact GPU airflow: Cameron's "Full bit-exact" and "Restate E1/E2, ~3%", backlog; spec §4.5 as amended, 1130608; P2c; P2d; the lead's rulings 1-6 on the B5 draft, 2026-10-22):* B5 is built bit-equal, kernel by kernel.
    - A kernel is exact when every division in it is `div32_ftz` and its other float ops are its twin's, in the twin's order, uncontracted. An exact kernel must equal its CPU twin bit for bit on the 3060 Ti (driver 572.83).
    - This replaces the 2026-10-21 amendment's parity at the CPU's recorded count. That survives only in the replay harness for other devices (ruling 6). The bands above apply only to a kernel not yet exact.
    - B5 changes no CPU source, so its TD-1 list is empty. The proof at READY: the release CPU suite unchanged, `BitPortability*:M1B*` 13 of 13, and the records' `AnalysisValidation*` unchanged. B5's tests compare the GPU with the CPU only.
  - **What is bit-equal, and why:**
    - **θ in the divergence** (`projection.cpp:102-118`) and **in the gradient subtract** (`:49-92`): adds, subtracts and multiplies, with the host's `b_coef`, `g_coef` and `inv_dx`. They are new kernels beside B2's, which stay plan A's.
    - **`apply_operator`** (`pressure.cpp:208-218`): θ is level 0's weights, and B2's one stencil (`mg_levels.slang:95-125`) computes it over a CG vector, through a source selector (ruling 4).
    - **The symmetric V(2,2)** (`multigrid.cpp:206-258`):
      - P2c's coarse weights stay on the host. `set_solids()` computes them through `set_weights()` (`multigrid.cpp:419-424`), whose divisions are in `series()` (:309-324). B2's `mg_layout()` already packs every weighted level, level 0 included (`projection_chain.cpp:41-51`). They are uploaded once, and no device op touches them.
      - `restrict_full_weighting` (`multigrid.cpp:225-252`) is a new kernel: `(wzy * w) * r` in its (a, b, d) order. It zeroes the coarse p, as B2's restriction does.
      - Residual and prolongation are B2's kernels, already bit-equal.
      - The smoothing divides (`pressure.cpp:157`). Rendering makes `smooth_divide()` (`airflow_mg_smooth.slang:41`) call `div32_ftz`, landing before B5.2 (ruling 3).
      - Post-smoothing and the coarsest level sweep black first (`multigrid.cpp:209-210, 220`), through a black-first sweep in `projection_chain.{hpp,cpp}` (ruling 4).
    - **The dots** (`pcg.cpp:36-38`, `reduce.cpp:12-16`): `tile_dot` is B1's `tile_sum` with a product in pass 0 and the same pass 1. It keeps the CPU's serial order (`reduce.hpp:27-48`), where B1 measured 0 mismatches (ruling 1).
      - B5.0 measures the final single-thread pass first. If the passes cost more than 5% of a step, pass 1 is staged through workgroup memory, which keeps the order. That needs an exception to `AirflowRunnerSpirv`'s no-groupshared rule (`test_gpu_airflow_runner.cpp:841-844`).
      - A tile tree on both twins moves every fixed-tile golden: the solids goldens and records, plan A's closed-domain mean (`kSolverCoreDigest`'s cavity, C0's and C1's records, `kTinyLidDigest`), `wall_load` and the region means. It goes back to the lead with the shifts measured before anyone chooses it.
    - **α, β** (`pcg.cpp:118, 126`) and **the measured residual** (`:57`): `div32_ftz`.
    - **The max-norm** (`pcg.cpp:47-58`) is order-free. Each fold is the CPU's `std::max`, the select `(a < x) ? x : a` from 0, never the device's max, so a NaN never wins on either twin. A per-tile max and a parallel fold give the same bits in any order. It needs no serial pass.
    - **The CG's updates, mask and copies** (`pcg.cpp:119-122, 127, 27-32`): bit-equal, uncontracted.
    - **The region means** (`regions.cpp:151-192`): per closed region, its cells' tile partials in tile order, times the host's reciprocal (:185). Two dispatches: one thread per (tile, region), then one thread per region over its tiles. `airflow_partials` is sized for max(6, closed regions) floats a tile.
    - **The interior** (`solids.cpp:758-799`): rigid is a copy. The extrapolated layers divide by their count of sources (:796) with `div32_ftz`. Layer 2 is dispatched after layer 1.
    - **`wall_load`** (`wall_load.cpp:14-36`, its body under the scope): six fixed-tile sums, one per component, and `glm::cross`'s `a*b - c*d`. It has no division.
    - No B5 kernel takes a square root or a transcendental.
  - **What stays banded, and why:**
    - Nothing in B5, once the smoother is exact.
    - Until then the preconditioner is banded (B2's sweep band, 7e-7), and with it the CG and everything after it. B5.2 waits for the exact smoother rather than measure a band that would retire.
    - Another device or driver keeps its `TD-14` bands (spec §4.5). There a test-only harness replays `solve_pcg_to()`'s path (the count, the restart, `residual_at`) from the test's own CPU run. `ProjectionReport` and `FluidHealth` gain no fields (ruling 6).
  - **The stall, the restart and best x:**
    - **The device decides.** One single-thread kernel, `airflow_pcg_scalars`, keeps `solve_pcg_to()`'s scalars in `airflow_scalars`: r·z, d·q, α, β, the iteration, the residual, `least`, `since`, `restarted`, `best`, `best_at`, `measured`, `exact`, a stop word and `stalled`.
      - It runs the branches of `pcg.cpp:81-137` in their order.
      - Its compares and its one multiply (`kStallGain * least`, :86) are exact. Its three divisions are `div32_ftz`.
      - So it makes the CPU's decisions from the CPU's bits: the count, the restart, `stalled`, `residual_at` and the x returned.
    - **One home for the constants.** The host writes them from `pcg.hpp` into `airflow_params`: word 27 the closed-region count, 28 `kResidualAnnounce`, 29 `kStallGain`, 30 `kStallIterations`, 31 the floor and the cap (16 bits each). These are the layout's last free words (`airflow_params_layout.slang:20-25`).
      - *Amended 2026-10-26 (Physics; Rendering's B3 took word 27, the lead's grant, Physics' review):* word 27 is B3's `off_face_block` (alpha, the face velocities, the ambient). B5's constants move to a block of their own: word 28 is `off_pcg_block`, in words into `airflow_params`, after the face block, holding {the closed-region count, `kResidualAnnounce`, `kStallGain`, `kStallIterations`, the floor, the cap}. Words 29-31 stay free. The device still decides, as above.
    - *Amended 2026-10-26 (the lead, from Rendering's B3 plan):* B5 owns the SolidField host packer, `solids_chain`'s `pack_solid_field()`, landed first in B5. It writes bindings 13-17 by the rule Physics confirmed to Rendering on 2026-10-22: θ, the wall velocity and φ as floats per face, u then v then w from `aperture_base(c)`; `face_tags` as `link[c][idx]` or `kNoWallLinks`, then the free-slip layer's bytes, little-endian, from word F_u + F_v + F_w; `wall_links` as 12 words a link, `inv_f[0..5]` then `wall[0..5]`; `has_solids()` iff `SolidField::any`. Rendering's B3 test-local packer is deleted when it lands. `airflow_solid_velocities` (above) is the GPU `apply_solid_velocities()`, both `SolidInterior` modes.
    - *Naming (2026-10-26):* the merged "B5.1" (34b718c) is Cameron's two-level reduction and `tile_sum`'s three passes. B5.1's tests below (the tile dot, the exact quotient, the order-free max) remain, as **B5.1b**, built with B5.2.
    - **The host follows the device and never decides** (ruling 2; G2 and the health check amended). Five chains, each recorded once:
      - setup: x ← p, masked; b with θ, masked, and its region means; `start()` (`pcg.cpp:61-69`);
      - iteration: `:111-127`. Every kernel returns at once if the stop word is set, so an exact zero inside the floor (`:113-116`) stops the rest;
      - measure: `gauge(x)`, A x, the max, the decision (`:83-109`), and best ← x when the decision keeps it (`:100-104`);
      - restart: `start()` again;
      - finish: the measure if none was taken (`:130`), the exact-zero stall (`:132`), x ← best on a stop above the mark (`:133-137`), p ← x, the gradient with θ, and the health word.
      - The host submits setup, then iteration `floor` times (`submit(chain, n)`), then measure, and reads a 16-byte decision word {stop, restart}. Until the word says stop, it submits restart when the word asks, then iteration and measure, and reads the word again. Then it submits finish. D1 at 9 iterations reads the word twice.
      - *Amended 2026-10-27 (the lead's ruling on B5.4, 3f4b9e2): what returns on the stop word, and what state a stop leaves dead.*
        - **Return at once on stop:** `airflow_pcg_update`'s ops 0-7, and `airflow_pcg_scalars`' step (op 4) and next (op 5). After a stop, decide (op 7) only takes the finish's measure. Gate (op 6), on a solve already stopped and measured, sets the closed regions' means to +0.0, so the measure's `gauge(x)` leaves x alone.
        - **Run on after a stop:** `airflow_pcg_apply` (A v), the dots (`airflow_tile_dot`, `airflow_tile_sum`), `airflow_region_mean` and the V-cycle's kernels. They write only state a stop leaves DEAD: q, r, z and d; the d·q and rz_next slots; level 0's b and p; the coarse levels.
        - **Live after a stop:** x, best and b, and the scalars other than those dead slots: the residual and slots 9-23, including the decision word. Anything that runs after a stop (the finish, B5.6's p ← x, the gradient with θ, the health word) reads only live state.
        - **The evidence:** bit-equality with `solve_pcg_to()` on all seven of B5.4's cases, the exact-zero ones included.
        - **The decision word** is slots 9-12: {stop, restart, iterations, stalled}.
    - **Best x** is one more grid vector. `airflow_pcg` packs x, b, r, z, d, q and best, seven floats a cell, as `pcg_bytes()` already counts (`pcg.hpp:38`).
    - **Level 0** (`airflow_pressure`, `airflow_rhs`, `airflow_residual`) is the preconditioner's scratch during the solve, as `levels[0]` is on the CPU (`pcg.cpp:39-45`). The grid's p goes into x first and comes back last (`projection.cpp:122, 126`).
    - **Scalars:** B5 takes slots 1-31 of `airflow_scalars`. Slots 1-7 were reserved for it (`projection_chain.hpp:69`). `airflow_params_layout.slang`'s header lists them.
  - **Cross-realm files** (rulings 3 to 5; the standing test grant):
    - `airflow_mg_smooth.slang` (Rendering): Rendering makes `smooth_divide()` exact before B5.2. B2's sweep and V(2,2) bands retire (`test_gpu_airflow_projection.cpp:484, 574`), and its `AirflowRunnerSpirv` row moves to `exact`.
    - `mg_levels.slang` (Rendering) and `spike/airflow/projection_chain.{hpp,cpp}` (Rendering's, in Physics' target): granted to Physics with Rendering's review, for a source selector in `stencil()` (the same faces in the same order, TD-9) and a black-first sweep. One realm edits a file at a time, in an order agreed with Rendering.
    - Core's `fp32_math.slang` and `flush_subnormals.hpp`, and Test/Docs' `spirv_scan.hpp`, are used as they are.
    - `tests/CMakeLists.txt` takes `test_gpu_airflow_solids.cpp` under the standing grant, named in READY. `AirflowRunnerSpirv` (`test_gpu_airflow_runner.cpp`) takes B5's rows under `SpirvProfile::exact`.
    - B3's viscous solve with solids (`viscous.cpp:22-26, 65, 88-89`) goes to Rendering with B3's remaining kernels (ruling 5).
    - Physics' own: the kernels `airflow_solid_divergence`, `airflow_solid_gradient`, `airflow_pcg_apply`, `airflow_mg_restrict_full`, `airflow_tile_dot`, `airflow_pcg_update`, `airflow_pcg_measure`, `airflow_pcg_scalars`, `airflow_region_mean`, `airflow_solid_velocities` and `airflow_wall_load`; `airflow_params_layout.slang` (words 27-31); `spike/airflow/solids_chain.{hpp,cpp}`; the test file and the docs.
    - Spec §4.5 (`2026-10-05-airflow-design.md:415`) is restated with B5's docs: "Plan A's solve runs a fixed number of cycles. The solids CG tests an exact max, so both twins make the same decisions."
  - **The tests:** `tests/test_gpu_airflow_solids.cpp`, prefix `GpuAirflowSolids`, ctest label `gpu`.
    - **Red first.** Each test is written against a stub kernel and fails before its kernel exists.
    - **The scope.** B5's tests open the FTZ scope themselves: each CPU twin a test calls runs inside `spade::math::ScopedFlushSubnormals`, as `fluid_step()` does.
    - **Mutations.** Each named mutation is run once and must go red. It is undone by edit and recorded in READY.
    - **Every kernel** also gets a `SameBitsAtEveryWorkgroupSize` row (32, 64, 128), and a row in `AirflowRunnerSpirv` under `SpirvProfile::exact`: no finding, and `estimate_ops` above 0 only where `div32_ftz` is called.
    - **Heavy runs** go through `jobrun.py --heavy-class gpu`.
    - **B5.0, measure first.** A test-only timing harness, opt-in and skipped by default (`SPADE_AIRFLOW_TIMINGS`, as B2's timings). The numbers go to the lead before B5.1.
      - The dot's final single-thread pass (`tile_sum`'s pass 1) and the whole dot, at the D1 box (113,400 tiles) and at 128³ (32,768), from `durations_ns()` over 20 submits. Against B7's 95 ms: 19 dots a step at D1's 9 iterations (ruling 1's 5%).
      - The host's round trip for one device word after a submit and its fence: one a CG iteration past the floor (ruling 2).
      - Whether S1 at a 1e-30 inflow writes subnormals in the CG without the scope, read from MXCSR's DE and UE flags as the FTZ plan's Task 0 read them. This is B5.4's subnormal case, or a substitute is found.
    - **B5.1, the dot, the max and the scalars:**
      - `TileDotSameBitsAtEveryWorkgroupSizeAndAsTheCpu`: 5×7×9 and 32³, as B1's A4 test. Mutations: the products summed in array order; a tile's products in (i, j, k) order.
      - `TheResidualsQuotientIsExact`: `r_max / b_max`, α and β from pairs that the 3060 Ti's `OpFDiv` misses. One such pair is `0x3F800800 / 3.0`: the host gives `0x3EAAB555` and the device `0x3EAAB556`. Mutation: a bare `/`, which goes red on the device and is an E1 finding in the scan.
      - `TheMaxNormIsOrderFree`: the max at every workgroup size, with a NaN planted in r. Mutation: the device's `max()` in place of the select.
    - **B5.2, the operator and the preconditioner** (after the smoother is exact). Each test runs at 16³ on S1's sphere, a sheet on an odd plane of S3's periodic channel, and an odd grid with a held periodic ring.
      - `DivergenceAndGradientWithAperturesMatchTheCpuBitForBit`. Mutations: `(1 - t)` dropped from the flux; a closed face subtracting the gradient.
      - `TheOperatorMatchesApplyOperatorBitForBit`. Mutation: level 0 unweighted (plan A's stencil).
      - `FullWeightingMatchesTheCpuBitForBit`, per level. Mutations: `wzy * (w * r)`; plan A's restriction (the children's sum), which is the solids design's test-10 mutation.
      - `ThePreconditionerMatchesTheCpuBitForBit`: one symmetric V(2,2). Mutations: `clear_weights()`' on-plane weights uploaded in place of `set_weights()`'; post-smoothing red first; the bare `OpFDiv` smoother.
    - **B5.3, the region means.** `RegionMeansMatchTheCpuBitForBit`, on two closed regions and a pinned one (`AirflowRegions`' geometries) and on S3. Mutations: a region summed in array order; the pinned region gauged.
    - **B5.4, the CG on its own.** `TheSolveMakesTheCpusDecisions`: from the same b and x, the GPU's count, `stalled`, `residual_at`, residual bits and returned x equal `solve_pcg_to()`'s. The cases:
      - S1's step 1, which meets the mark at its floor;
      - S3's sheet on plane 17 with the mark at 1e-9: stalled after 17 iterations, x from the 9th (`AirflowSolidsSolve.AMarkBelowFp32sFloorStallsInsideTheCap`);
      - the 32-cell blocked channel's step 2, which restarts and then meets the mark (`AStalledSolveRestartsOnceAndMeetsTheMark`);
      - the 128-cell channel's step 1: stalled after 20, with the least (1.276e-4) returned (`ASolveAtFp32sFloorAboveTheMarkStalls`);
      - b = 0: an exact zero at the start, and no iteration;
      - floor = cap = 8: capped, not stalled;
      - B5.0's subnormal case, against the scoped CPU.
      - Mutations: the stall rule off (it runs to the cap); no restart; the last x returned in place of best; `<=` for `<` against `kStallGain * least`; `least` reset on every measurement; the exact-zero test after α; the restart's d left as it was; the CPU twin unscoped on the subnormal case.
    - **B5.5, the interior and the load:**
      - `TheInteriorMatchesTheCpuBitForBit`, rigid and extrapolated, on S1's sphere. One sample has three sources that sum to `0x3F800800`. Mutations: a bare `/`; both layers in one dispatch.
      - `TheWallLoadMatchesTheCpuBitForBit`: the force and moment bits on `AirflowWallLoad`'s sphere, sheet and uniform cases, and on the field of `AirflowFtz.AWallLoadReadsASubnormalPressureAsZero` (p = 2^-130). Mutations: the force's tiles in array order; x_c without its half cell; the CPU twin's body unscoped (force `0xBCC`, as the FTZ plan measured).
    - **B5.6, the projection and the step:**
      - `TheSolidsProjectionMatchesTheCpuBitForBit`: S1 for 4 steps, S3 on planes 16 and 17, and the held ring. At each step, 0 values of p, u, v and w differ, and the health is equal (the count, the residual's bits, `stalled`, `capped`). Mutation: one ulp nudged into the GPU's b.
      - `S2WithTheGpuProjectionInTheCpuStep` (opt-in, `SPADE_AIRFLOW_VALIDATION`): the GPU projection inside the CPU's step, as B4's `C8GpuGatherInTheCpuStep` did, bit-equal at every step (ruling 5).
      - `TimingsAtTheD1Box` (opt-in, `SPADE_AIRFLOW_TIMINGS`): each chain, the serial passes, one CG iteration, the decision word's readback and the whole solve, for B7.
    - With every kernel bit-equal there is no band to measure. The records the plan asked for on S1 and S3 become bit-equal records: "0 of N values differ, steps 1 to N", in Test/Docs' format, checked by `AnalysisValidation*`.
  - **Memory:** `best` adds 4 B a cell. It is already in `pcg_bytes()`, and so in `check_grid()`: 27.2 MiB at the D1 box. The runner's VRAM check also counts what `check_grid()` does not: `airflow_partials` (max(6, closed regions) floats a tile, about 0.4 B a cell at 6), the scalars and the region table (bytes), and the staging buffer.
- **B6, publish on the GPU (P8):**
  - `publish_fields` and `pack_f16`, banded against `publish_render_volume`; the f16 bits should match exactly where the fp32 inputs match.
  - Measure readback of fp32 plus the CPU pack against packing on the device plus reading back halves, and keep the cheaper (P8's rule).
- **B7, the D1 box:**
  - the quad at hover, 224 × 168 × 160, on the GPU;
  - step time, and the time for the D1 film's 0.50 s;
  - the GL renderer's VRAM, measured to set the reserve;
  - the comparison against a CPU run of the same box for as many steps as the CPU affords, inside the band to T_p.
  - *Amended 2026-10-21 (the box, and the step time expected before B7 runs):* the D1 box is 240 × 177 × 168 at 4 mm (7.14M cells). The CPU reference takes 19.7 s per step at the CG floor of 8 (residual 4.97e-5 / 4.05e-5; measured 2026-10-21 on physics/mg-odd-axes). **The expected GPU step is about 95 ms on the 3060 Ti**, from bandwidth:
    - bytes moved per cell per step, fp32:
      - the solids' projection, about 3.3 KB: nine symmetric V(2,2) preconditions (the first, then one per CG iteration) at about 265 B each (four fine sweeps, each colour reading p, b and three face weights; residual, restrict, prolong; coarser levels about 1/7 more), plus CG's operator, two dots, three updates, gauge and mask at about 110 B per iteration;
      - viscosity, about 0.7 KB: 8 red-black sweeps × 3 components × about 28 B;
      - advection, about 0.16 KB: two MacCormack passes, each reading u, v and w and writing or reading hat, lo and hi;
      - the rest (divergence, gradient subtract, sources, boundaries, extrapolation, health), about 0.15 KB;
      - total about 4.3 KB;
    - 4.3 KB × 7.14M cells ≈ 31 GB per step; at 80% of 448 GB/s (358 GB/s), ≈ 86 ms;
    - plus about 1,000 dispatches (the coarse levels are latency-bound) at about 8 µs each, ≈ 8 ms;
    - so ≈ 95 ms per step, about 200× the CPU, and about 2 minutes for the D1 film's 0.50 s (about 1,300 steps).
    - **If B7 misses it by more than 2× (above 190 ms per step):** profile per kernel with the runner's pass timings (`durations_ns`), report to the lead before optimising, and bring P2c forward if the CG count drives the time.
- **B8, records and grade:**
  - each band's record (Test/Docs' format);
  - the module's GPU grade placed in `04-verification.md`, its row in `07-status.md`;
  - the hook LiveRun uses, an Interface task: the runner behind the same step and publish calls LiveRun makes now.

## Decisions for the lead

1. **Who builds B0.** Plan B gives the binding block, `memory_budget()` and the limit checks to Core, which stage 7 inherits. I recommend Core builds them, since stage 7 takes them over. If Core has no room, Physics builds them under Core's review.
2. **Rendering's share.** I recommend Rendering takes B6 (the publish chain and P8's measurement), since it owns FlowVolume's consumers and the f16 rule. Physics takes B1 to B5, B7 and B8. If Rendering has more room, the advection kernels in B3 are the next cleanest piece to split off: one kernel set with its own CPU twin and tests.
3. **The integration point.** I recommend the runner reaches LiveRun through one backend switch (Interface's code, in B8) rather than a separate film tool, so D2's film uses the same LiveRun path D1 does.
4. **The reserve** starts at 1 GiB until B7 measures it.

## Review focus

- A grid under the 4 GiB cap but over the live budget, with another program holding VRAM: it must be refused by name, not fail on allocation.
- Workgroup-size invariance for every reduction, not only `tile_sum`: CG's dots and the region means.
- Parity past T_p: a band check there is invalid, and the test must switch to invariants.
- A periodic axis in every kernel that indexes a neighbour: the wrap matches the CPU's.
- The f16 pack's rounding at ties and at subnormals: the same integer rule as `core/fp16.hpp`.
- *Added 2026-10-22 (B5, bit-exact):*
  - The scalar kernel against `pcg.cpp:81-137`, line by line: the order of the mark, cap and stall checks, and best kept after them.
  - Every float `/` in B5's kernels is `div32_ftz`, and `estimate_ops` says so.
  - The max's select, never the device's max.
  - The stop word, read by every kernel in the iteration chain.
  - The tests' CPU twins, inside the scope.
