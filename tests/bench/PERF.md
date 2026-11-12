# Performance measurement protocol

**Owner:** Test/Docs. **Status:** approved by the lead 2026-11-11 (L74, step 1). The harness is built and dry-run. **No gated measurement has been taken yet:** the ledger is empty and no floor is measured. Nothing here optimises anything.
**Why:** Cameron wants training, physics and rendering to run much faster. A speed-up is only real if two numbers taken a week apart can be compared. This page fixes how a number is taken, so it can.
**Not a gate.** A slow number is a finding for the lead, not a red build (`01-verification.md`, the bench paragraph; TD-8).

**Scope.** This page measures Spade. **KAT's training throughput is measured by KAT, not here:** its rollout and its PPO update run on KAT's own fake host and do not step Spade. Two Spade numbers sit in KAT's path and are in the table: the live training view's frame time with 128 and 1024 drone puppets (Rendering), and the real-time step for 1 to 16 vehicles (Core).

## What counts as a metric

A metric is **something a person waits for**, on a command they can type. Each one names:

1. **who waits** and for what (a tester opening the editor, a trainer waiting for a batch);
2. the **workload** (a fixed scene, size and step count);
3. the **command**, exactly;
4. the **unit**: seconds, milliseconds, frames per second, or megabytes (never "iterations");
5. the **owner**, the realm that can change it.

A number that no person waits for (a counter, a sub-step, a ratio) is a **diagnostic**. It may sit in `baselines.json`, but it is not a ledger metric. The metrics table says which each one is.

## The measuring window

The numbers are only comparable if nothing else is using the box. So:

1. **Start only when `jobrun.py --queue-status` shows no other heavy, gpu or quick job** running, and nothing waiting that would start inside the window.
2. **The measurement is a heavy job:** `jobrun.py --heavy-class test --ticket perf-<sha7>-<n> --expect-s <N> --limit-s <2N> --log <abs log> -- python tests/bench/measure.py ...`, with `--gpu` for a metric that needs the card.
3. **The harness watches the queue.** It reads `--queue-status` before the first run, after each repeat and at the end. If another heavy, gpu or quick job appears in any reading, that **run is marked noisy and repeated**; a noisy run is never in the median.
4. **The box is idle at the start:** total CPU use under 5% over 10 s before the first run. The harness waits up to **10 minutes** for that. If the box never gets under 5%, it stops and reports the last reading; it does not wait forever.
5. **The lead coordinates the quiet windows** with KAT, who holds its heavy tickets for a window the lead names. A window is 10 to 20 minutes, so a suite is split into runs that each fit one window. The harness cannot hold the other slots; it can only tell that they were not held.

## The run

For each metric, in this order:

1. **Warm-up:** one full run, thrown away. It fills the disk cache, the shader and driver caches, and the CPU's clocks.
2. **Five repeats**, one after another, the same command, on a fresh process each time.
3. **Record the median, the minimum, the maximum and n.** The median is the number. The range is the spread. Never the mean.
4. **The suite twice for the noise floor** (next section), at the first run and at every re-seed.

The micro-benchmarks (`spade_bench`, Google Benchmark) use `--benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json`; the harness takes the five per-repetition rows, not the library's aggregates, so the minimum and maximum are real. The five repetitions of one benchmark share a process; the warm-up run is a whole separate process before them. The basis is **`cpu_time` for CPU metrics and `real_time` for anything that waits on the GPU or the disk** (TD-8), and the ledger says which in `notes`.

A command that is not a micro-benchmark prints one line per number, `PERF <metric> <value> <unit>`; without that, the harness uses the process's wall time and its **peak working set**.

## Recorded each time

One row in `tests/bench/environment.csv` per measurement session (the `run` column joins it to the ledger's `notes`):

| Field | How |
|---|---|
| run | `<date>_<sha7>` |
| sha | the **gated** SHA, the one the lead's gate passed, of the measured tree; a number is never taken on an ungated tree |
| harness_sha, tree | the harness checkout's `HEAD`, and the measured tree's path |
| build type | `Release`, MSVC, the preset's build tree; debug is not measured |
| gpu | the card's name |
| gpu driver | the driver version (`nvidia-smi --query-gpu=driver_version` or the video controller's `DriverVersion`) |
| power plan | `powercfg /getactivescheme`, recorded and **not changed** |
| free memory | free physical and free commit in GB at the start (the same reading jobrun uses) |
| cpu | name and logical count |
| idle cpu | the pre-run reading |
| queue | the three `--queue-status` readings, or `clean` |
| noisy | how many runs were marked noisy and repeated |

## The noise floor

The floor is how much a number moves when **nothing changed**. It is measured, not assumed:

1. Run the whole suite **twice**, in two separate windows, on the same SHA.
2. Per metric, the floor is `|median_A − median_B| / median_A`, in percent, and never less than the larger of the two runs' own (max − min) / median.
3. The floors go in the metrics table. They are measured again at every re-seed, and when the box, the driver or the compiler changes.

**When a change is real:** the new median differs from the reference median by **more than the metric's floor**, and the two runs' [min, max] ranges **do not overlap**. Anything else is noise. A change smaller than the floor is not reported as a gain or a loss.

## The cadence

Measure at **every gated batch** while the whole suite fits in about **15 minutes**. If it grows past that, run the metrics of the realms whose code changed in the batch every time, and the **full suite every third batch**; this page says so when it happens. The first window comes from the lead once a gated SHA exists. The dry run, not a guess, sets `--expect-s`.

## The harness

`tests/bench/measure.py`, Python standard library only. It runs the metrics in `metrics.json`, and:

- measures a **tree**, not itself: `--tree <dir>` is the checkout and build being measured. Its `HEAD` must equal `--sha`, it must have no uncommitted change, and the binaries come from its build folder (`--build-dir`, inside it). The harness can run from any checkout, including one newer than the measured SHA; the environment row records both SHAs;
- reads the queue, checks the box is idle, records the environment row, runs the warm-up and the repeats, and appends the ledger rows;
- `--dry-run` runs and prints but writes nothing (`--ignore-queue` with it tests the mechanics on a busy box);
- `--out-json` keeps the raw values; `--floors A.json B.json --out-floors tests/bench/floors.json` computes the noise floors from two runs of one SHA;
- `--trend` prints each metric's last two ledger rows and says "REAL" or "noise" by the rule above.

A real run: `python jobrun.py --heavy-class test --ticket perf-<sha7>-1 --expect-s <N> --limit-s <2N> --log <abs log> --cwd <worktree> -- python -I tests/bench/measure.py --sha <gated sha> --tree <measured checkout> --build-dir build-ninja/release --out-json build-bench/<run>.json`. Run it from the Test/Docs worktree and commit the new ledger rows on a branch; the measured tree is never written to.

## The ledger

`tests/bench/ledger.csv`, one row per metric per measurement:

`date,sha,metric,unit,median,min,max,n,notes`

- **Appended, never edited.** A wrong row gets a new row with `notes` saying "supersedes <date>".
- **One row per metric per gated batch.** A slowdown shows as a step in a column.
- `metric` is `<owner>.<name>` (for example `core.step_quad_worlds/16`). `n` is 5. `notes` holds the run id, the basis (`cpu_time` or `real_time`), and `noisy=<count>`.
- Dates come from the machine clock.

## The metrics

The table is generated from `tests/bench/metrics.json` (`python tests/bench/measure.py --table`); that file is the source. A `spade_bench` family gives one ledger row per size, named `<metric>/<size>`. The first rows are the existing `spade_bench` families: the step and render families are metrics (Core and Rendering), `IntegrateOrientation` and `CsgMeshShell` are diagnostics, in `baselines.json` only. The realms' own metrics (Physics, Core, Rendering, Interface) are added as they send them; KAT's training numbers are KAT's.

| Metric | Who waits, for what | Workload | Command | Unit | Owner | Kind | Floor |
|---|---|---|---|---|---|---|---|
| `core.step_plain_bodies` | a developer or user stepping the simulation (one world of plain bodies) | BM_StepPlainBodies, the sizes in bench_sim.cpp | `{bench} --benchmark_filter=^BM_StepPlainBodies --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | not measured |
| `core.step_dense_bodies` | a developer or user stepping a world crowded with bodies | BM_StepDenseBodies, the sizes in bench_sim.cpp | `{bench} --benchmark_filter=^BM_StepDenseBodies --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | not measured |
| `core.step_quad_worlds` | a user flying drones in real time (the real-time step for 1 to 16 vehicles, Core's proposal pending) | BM_StepQuadWorlds, the sizes in bench_sim.cpp | `{bench} --benchmark_filter=^BM_StepQuadWorlds --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | not measured |
| `rendering.render_background_fill` | a user waiting for a frame (the empty-scene floor of the CPU renderer) | BM_RenderBackgroundFill, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderBackgroundFill --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | not measured |
| `rendering.render_ground_plane` | a user waiting for a frame (the ground plane on the CPU renderer) | BM_RenderGroundPlane, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderGroundPlane --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | not measured |
| `rendering.render_slots` | a user waiting for a frame with many objects on the CPU renderer | BM_RenderSlots, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderSlots --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | not measured |
| `core.integrate_orientation` | no one: one sub-step of the integrator | BM_IntegrateOrientation | `{bench} --benchmark_filter=^BM_IntegrateOrientation --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ns | core | diagnostic | not measured |
| `rendering.csg_mesh_shell` | no one: one stage of mesh building | BM_CsgMeshShell | `{bench} --benchmark_filter=^BM_CsgMeshShell --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | diagnostic | not measured |

## Re-seeding `baselines.json`

The October seed predates most of the code. At the first gated SHA, under this protocol, the existing `spade_bench` families are measured again and `baselines.json` is rewritten with the box, driver, compiler and SHA in its `_meta`. The old seed stays in git history. After that, the ledger carries the trend and `baselines.json` carries only the reference run.

## Not covered

- **Other machines.** A number is for this box, this driver and this power plan; the environment row says which.
- **A noisy neighbour outside jobrun** (a browser, the owner's game, a virus scan) is caught only by the idle check and the noisy mark, not prevented.
- **Debug builds and sanitizer builds** are not measured.
