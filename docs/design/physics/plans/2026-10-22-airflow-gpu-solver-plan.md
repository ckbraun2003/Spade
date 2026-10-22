# The GPU airflow solver (plan B) -- implementation plan

**Owner:** Physics, with Rendering on the publish chain. **Status:** DRAFT 2026-10-22, for the lead's review before any build (Cameron's order, 2026-10-22: the GPU solver comes after the D1 hover preview and before the actuator line, P4).

**Goal:** the airflow step, with solids and rotor discs, on the GPU, banded against the CPU reference, so a D1-sized film (6M cells, about 1300 steps) runs in minutes instead of hours.

**Design:** `2026-10-12-airflow-chain-runner-design.md` (plan B, Core-approved with A1-A5) and the spec, `2026-10-05-airflow-design.md` §4 (kernels, determinism, sizes). Solids: `2026-10-13-airflow-solids-design.md` (P2, P3). This plan adds no new design. It orders the work, names the tests and settles who builds what.

## Constraints (copied from the design and the laws)

- **G1.** Its own target, `spade_airflow_runner` in `spike/airflow/`. It is not linked into Simulation, makes no StepShape or mirror change, and installs no header.
- **G2.** Built on `VulkanContext` and Core's `dispatch_chain`. A fluid step is one recorded chain, resubmitted. Publish is a second chain.
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
- **Per cell:**
  - fluid about 102 B;
  - solids about 95 B;
  - render volume 16 B;
  - total about 213 B.
- **The D1 box at 4 mm** (224 × 168 × 160, 6.02M cells) needs about 1.3 GB.
- **The cap** is `min(4 GiB, budget − usage − reserve)`.
  - 4 GiB is the spec's cap and `kGridByteCap`, the CPU ceiling.
  - budget and usage come from `VulkanContext::memory_budget()`, using `VK_EXT_memory_budget` or half the heap (A1/A2).
  - reserve is for the GL renderer's own textures. It is unmeasured, so it starts at 1 GiB, and B7 measures the renderer's VRAM during the D1 film.
- **At 4 GiB the grid tops out near 20M cells,** about 270³ or the D1 box at 2.6 mm. A larger request is refused before any allocation, naming the bytes asked for and the cap.

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
