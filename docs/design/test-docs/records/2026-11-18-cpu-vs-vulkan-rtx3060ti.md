# CPU vs Vulkan stepping on the RTX 3060 Ti (2026-11-18)

**Status:** measured once, on a box that was not quiet; read the caveats before quoting a ratio.
**Asked by:** the lead, for the owner ruling KAT D180 ("use the GPU for physics and rendering wherever possible").
**Replaces, as a measurement:** the Iris Plus table in `engine/compute/backend.hpp` (the comment above `BackendDesc`, lines 100-131 on master `e7e1d076`). That comment still quotes the Iris figures and should point here. Its owner changes it; this record does not.

## What was measured

The same two scenes as the 2026-09-18 table, on this machine, Release build, wall clock:

- **1 world x N bodies**, N = 10, 100, 1000. Plain bodies in a void world, 2 m apart, no vehicles.
- **N worlds x the quad scene**, N = 1, 4, 16, 64, 256. One quadrotor per world, hovering in trim, over a ground plane.

Each scene ran on the CPU backend and the Vulkan backend. Each ran as `step(1)` and `step(16)` per call. One `step(n)` call uploads once if the host changed state, submits `n` steps with one fence wait each, and reads back once. So `step(16)` shows what batching hides, and `step(1)` is what every production call site does (all four pass `n = 1`).

Times below are per step: the call's wall time divided by `n`. All values are medians of 5 repetitions.

**Machine:** Ryzen 7 3700X, 16 logical CPUs, 32 GB, RTX 3060 Ti (driver 572.83), Windows 10. **Build:** MSVC Release (`/O2`), master `e7e1d076` plus the probe patch below. **Run:** one gpu-class job, 147 s.

## Wall-clock per step (ms)

1 world x N bodies:

| N | CPU, step(1) | Vulkan, step(1) | Vulkan / CPU | CPU, step(16) | Vulkan, step(16) | Vulkan / CPU |
|---|---|---|---|---|---|---|
| 10 | 0.0066 | 0.439 | 67x | 0.0057 | 0.306 | 54x |
| 100 | 0.072 | 2.26 | 31x | 0.073 | 2.08 | 28x |
| 1000 | 0.985 | 24.5 | 25x | 1.03 | 23.7 | 23x |

N worlds x quad scene:

| N | CPU, step(1) | Vulkan, step(1) | Vulkan / CPU | CPU, step(16) | Vulkan, step(16) | Vulkan / CPU |
|---|---|---|---|---|---|---|
| 1 | 0.0020 | 0.322 | 162x | 0.0022 | 0.171 | 76x |
| 4 | 0.0060 | 0.407 | 67x | 0.0065 | 0.195 | 30x |
| 16 | 0.026 | 0.90 | 35x | 0.023 | 0.235 | 10.4x |
| 64 | 0.123 | 2.55 | 21x | 0.120 | 0.377 | 3.2x |
| 256 | 0.506 | 11.2 | 22x | 0.498 | 0.836 | 1.7x |

**Against the Iris table.** Vulkan is faster here in every row: quad 1 world, 15.1 ms to 0.17 ms (`step(16)`); 1000 bodies, 285 ms to 23.7 ms; quad 64 worlds, 15.5 ms to 0.38 ms. The CPU is faster too, by 2x to 4x. Vulkan is still slower than the CPU in every row, by 1.7x to 162x, where the Iris table showed 25x to 2139x. The fixed floor of about 14 ms per step is gone: the floor is now about 0.3 ms per `step(1)` call and about 0.14 ms per step inside a `step(16)` call. The Iris `step` figures were batched (50 steps per call), so `step(16)` is the closer comparison.

## Where the Vulkan time goes

The breakdown is per call, in ms. "Submit" and "fence wait" are summed over the `n` steps of the call. "Readback" is the whole `readback()` in `Simulation::step`, with its mirror copy (the device-to-staging copy batch, submit and fence wait) in brackets. "GPU kernels" is per step, from the pass timestamps.

`step(1)`:

| scene | call | submit | fence wait | readback (mirror copy) | GPU kernels / step |
|---|---|---|---|---|---|
| plain 10 | 0.44 | 0.033 | 0.251 | 0.153 (0.103) | 0.166 |
| plain 100 | 2.26 | 0.047 | 1.93 | 0.280 (0.171) | 1.74 |
| plain 1000 | 24.5 | 0.059 | 23.6 | 0.83 (0.22) | 23.3 |
| quad 1 | 0.32 | 0.029 | 0.156 | 0.136 (0.094) | 0.074 |
| quad 4 | 0.41 | 0.025 | 0.158 | 0.222 (0.085) | 0.083 |
| quad 16 | 0.90 | 0.026 | 0.190 | 0.683 (0.092) | 0.103 |
| quad 64 | 2.55 | 0.040 | 0.242 | 2.27 (0.134) | 0.132 |
| quad 256 | 11.2 | 0.082 | 0.363 | 10.8 (0.252) | 0.214 |

`step(16)`:

| scene | call | submit | fence wait | readback (mirror copy) | GPU kernels / step |
|---|---|---|---|---|---|
| plain 10 | 4.90 | 0.527 | 4.09 | 0.187 (0.130) | 0.166 |
| plain 100 | 33.2 | 1.23 | 31.5 | 0.310 (0.188) | 1.89 |
| plain 1000 | 379 | 1.15 | 377 | 0.870 (0.177) | 23.3 |
| quad 1 | 2.74 | 0.371 | 2.20 | 0.132 (0.091) | 0.068 |
| quad 4 | 3.13 | 0.429 | 2.42 | 0.266 (0.104) | 0.076 |
| quad 16 | 3.77 | 0.392 | 2.65 | 0.640 (0.105) | 0.092 |
| quad 64 | 6.03 | 0.450 | 3.14 | 2.40 (0.132) | 0.120 |
| quad 256 | 13.4 | 0.529 | 4.11 | 8.85 (0.222) | 0.167 |

The host timers account for all but about 0.001 ms of every call. Flush and host upload are under 0.001 ms everywhere: the bench never changes host state, so the upload batch never runs. A caller that edits state between calls pays an upload; that is not measured here.

**One world, many bodies: the GPU kernel is the whole cost.** `dynamic_contact.resolve` takes 23.3 ms of the 23.3 ms of kernels at 1000 bodies, 1.85 ms at 100 and 0.13 ms at 10. Every other pass is under 0.025 ms. That is the one-thread-per-world collision kernel on a single lane, as the `backend.hpp` comment says. The time grows faster than linearly with bodies.

**Many worlds, few bodies: two costs, and neither is the kernels.**

- **A fixed cost per step.** At one world a step costs 0.14 to 0.16 ms of fence wait. The kernels are 0.07 ms of that. About 0.07 ms is the latency between the 12 passes (barriers, queue wake), and `vkQueueSubmit` adds about 0.025 ms. The kernels stay under 0.22 ms per step up to 256 worlds.
- **The readback.** Every `step(n)` call reads back every arena. Its host part is the readback minus the mirror copy: the memcpy from staging into the arenas and the bookkeeping. It costs 33 to 42 us per world per call, linear in worlds (42, 34, 37, 33 and 41 us at 1, 4, 16, 64 and 256 worlds). At 256 worlds it is 10.5 ms of the 11.2 ms of a `step(1)` call, while the mirror copy is 0.25 ms. The cost that grows with worlds is host memcpy, not GPU work.

## Where Vulkan would overtake the CPU

The question is which worlds-count, if any, makes Vulkan win. The `step(16)` row at 256 worlds says 1.7x behind, and the tail of the readback is why: it grows with worlds as fast as the CPU's step does.

Take the host time of `backend->step(n)` alone (submit and fence waits, no upload or readback) and divide by `n`. For the quad scene at `step(16)`:

| worlds | 1 | 4 | 16 | 64 | 256 |
|---|---|---|---|---|---|
| Vulkan step only (ms/step) | 0.163 | 0.180 | 0.191 | 0.226 | 0.292 |
| CPU (ms/step) | 0.0022 | 0.0065 | 0.0227 | 0.120 | 0.498 |
| Vulkan / CPU | 73x | 28x | 8.4x | 1.9x | 0.59x |

Between 64 and 256 worlds the Vulkan step-only time grows by about 0.00034 ms per world. The CPU's grows by about 0.00197 ms per world. A straight line through each pair of medians meets at about **130 worlds**. That figure is an interpolation of two medians, not a measurement; no 128-world case was run.

So the device steps faster than the CPU from roughly 130 worlds, and 1.7x faster at 256. What `step(n)` delivers today includes the readback of every arena every call, and that keeps the whole call behind the CPU at 256 worlds (1.7x at `step(16)`, 22x at `step(1)`). Two things would move the crossover down: a larger `n` (one readback over more steps), or reading back less than every arena. This record measures neither change.

## Method: the probe

The probe is a scratch patch on a git archive of master `e7e1d076`. It was built and run outside the repository and is not committed. The patch does four things:

1. It adds `engine/scratch_probe.hpp`. This holds accumulators (nanosecond sums and counts per slot) and an RAII scope timer on `std::chrono::steady_clock`.
2. It times, in `Simulation::step` (the Vulkan branch): `flush_structural()`, `upload_to_device()`, `vulkan_backend_->step(n)` and `vulkan_backend_->readback(arenas_)`.
3. It times, in `StepRecorder::submit`: each `vkQueueSubmit` and each `vkWaitForFences` of the recorded step, summed over the `n` steps. In `StateMirror::run_copy_batch` it times the whole copy batch and its submit and its fence wait, separately for each direction.
4. It appends a benchmark family `Probe/<plain|quad>/<cpu|vulkan>/step<1|16>/<N>` to `tests/bench/bench_sim.cpp`. It reuses that file's scene builders (`build_plain_bodies_sim`, `build_quad_worlds_sim`), with `dt` 1 ms and one substep.

Each case builds its simulation, does 3 warm-up calls and resets the accumulators. It then runs google-benchmark's iterations with `UseRealTime()`, and divides each accumulator by the number of calls. The GPU kernel time is the sum of the passes in `vulkan_pass_durations_ns()` after the run, which describes the last submit (one step). The counters quantise at about 1 us.

The engine source forbids wall-clock symbols (`M1B.FixedStepNoWallClockSymbolsInEngineSource`); that is why the probe lives in a scratch copy and not in a branch. The build used MSVC and Ninja with the dependency sources copied from an existing build, so nothing was fetched. 32 cases, 5 repetitions each.

## Caveats

- **One run, not a quiet window.** Other realms' builds and tests were on the CPUs. The GPU slot was this run's own. The spread across the 5 repetitions (max minus min, over the median) is 0.4% to 31% for the Vulkan rows, mostly under 20%; the widest is quad 256 at `step(1)` (9.0 to 11.8 ms per step, the host memcpy). The CPU rows spread 5% to 24%. Read the CPU rows under 0.01 ms to one digit.
- **The timers cost about 0.1 us per scope** (an estimate, not measured): under 1% of the smallest Vulkan figure.
- **GPU kernel time is the last submit's.** It is one step's pass timestamps, not an average over the run.
- **Not measured:** an upload of changed host state; `n` above 16; more than 256 worlds; scenes with many bodies in many worlds; the render path; any GPU other than this one.
- **The raw run is not kept.** The tables hold the medians. A later run should reuse the method above and compare to these rows only on this machine.
