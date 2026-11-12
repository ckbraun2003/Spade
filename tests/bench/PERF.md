# Performance measurement protocol

**Owner:** Test/Docs. **Status:** DRAFT, written 2026-11-11 for L74 (the lead's step 1), before any measurement. Nothing here optimises anything.
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
4. **The box is idle at the start:** total CPU use under 5% over 10 s before the first run. The harness records it and refuses to start otherwise.
5. **The lead coordinates the quiet windows** with KAT, who holds its heavy tickets for a window the lead names. A window is 10 to 20 minutes, so a suite is split into runs that each fit one window. The harness cannot hold the other slots; it can only tell that they were not held.

## The run

For each metric, in this order:

1. **Warm-up:** one full run, thrown away. It fills the disk cache, the shader and driver caches, and the CPU's clocks.
2. **Five repeats**, one after another, the same command, on a fresh process each time.
3. **Record the median, the minimum, the maximum and n.** The median is the number. The range is the spread. Never the mean.
4. **The suite twice for the noise floor** (next section), at the first run and at every re-seed.

The micro-benchmarks (`spade_bench`, Google Benchmark) use `--benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json`; the harness takes the five per-repetition rows, not the library's aggregates, so the minimum and maximum are real. The basis is **`cpu_time` for CPU metrics and `real_time` for anything that waits on the GPU or the disk** (TD-8), and the ledger says which in `notes`.

A command that is not a micro-benchmark prints one line per number, `PERF <metric> <value> <unit>`; without that, the harness uses the process's wall time and its **peak working set**.

## Recorded each time

One row in `tests/bench/environment.csv` per measurement session (the `run` column joins it to the ledger's `notes`):

| Field | How |
|---|---|
| run | `<date>_<sha7>` |
| sha | the **gated** SHA, the one the lead's gate passed; a number is never taken on an ungated tree |
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

## The ledger

`tests/bench/ledger.csv`, one row per metric per measurement:

`date,sha,metric,unit,median,min,max,n,notes`

- **Appended, never edited.** A wrong row gets a new row with `notes` saying "supersedes <date>".
- **One row per metric per gated batch.** A slowdown shows as a step in a column.
- `metric` is `<owner>.<name>` (for example `physics.step_quad_worlds`). `n` is 5. `notes` holds the run id, the basis (`cpu_time` or `real_time`), and `noisy=<count>`.
- Dates come from the machine clock.

## The metrics

*To be filled from the realms' proposals (Physics, Core, Rendering, Interface; KAT's training batch shape from Kat). Each row: name, who waits, workload, command, unit, owner, kind (metric or diagnostic), floor.*

| Metric | Who waits, for what | Workload | Command | Unit | Owner | Kind | Floor |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

## Re-seeding `baselines.json`

The October seed predates most of the code. At the first gated SHA, under this protocol, the existing `spade_bench` families are measured again and `baselines.json` is rewritten with the box, driver, compiler and SHA in its `_meta`. The old seed stays in git history. After that, the ledger carries the trend and `baselines.json` carries only the reference run.

## Not covered

- **Other machines.** A number is for this box, this driver and this power plan; the environment row says which.
- **A noisy neighbour outside jobrun** (a browser, the owner's game, a virus scan) is caught only by the idle check and the noisy mark, not prevented.
- **Debug builds and sanitizer builds** are not measured.
