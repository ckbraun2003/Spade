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
- **Grades (L3, L4, TD-14).**
  - The CPU stays the reference and the golden; the GPU is never a golden source.
  - Each kernel is banded against its CPU twin. A band is 4× the largest measurement, with a floor of 4 ulps, and holds to the horizon T_p. The target is 1e-4 relative; anything above 1e-3 is a grade change, never a wider band.
  - The airflow module's GPU grade is **banded**, placed in `04-verification.md`.
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
- **B2, the projection without solids:**
  - kernels: divergence, red-black smoothing, residual, restrict, prolong, coarse, gradient subtract;
  - per-kernel bands at 16³;
  - one V(2,2) projection at 32³ against `project()`;
  - timings at 64³ and 128³ (M1).
- **B3, the fluid step without solids:**
  - kernels: boundaries, advection, viscosity;
  - `fluid_step` parity at 32³ to T_p on the walled cavity and the nine boundary configurations;
  - past T_p, invariants (divergence, kinetic energy within its band);
  - the CPU goldens are not re-cut.
- **B4, rotor discs:**
  - `actuator_gather`, banded against `actuator.cpp` on the reference quad's four discs;
  - C8's actuator-disc case: the GPU's downwash ratio against the CPU's.
- **B5, solids (P2 on the GPU):**
  - θ in divergence and in the gradient subtract;
  - PCG with tile dots, with `kSolidsCgIterations`;
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
