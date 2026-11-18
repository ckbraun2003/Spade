# Performance measurement protocol

**Owner:** Test/Docs. **Status:** approved by the lead 2026-11-11 (L74, step 1). The harness is built, and the first baseline is taken (below). Nothing here optimises anything.
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

1. **Start only when `vantage.py job status` shows no other heavy, gpu or quick job** running, and nothing waiting that would start inside the window.
2. **The measurement is a heavy job:** `python <vantage.py> job run --class test --ticket perf-<sha7>-<n> --expect-s <N> --limit-s <2N> --log <abs log> --cwd <worktree> -- python tests/bench/measure.py ...`, with `--class gpu` for a metric that needs the card. `<vantage.py>` is `%USERPROFILE%\Desktop\Vantage\vantage.py`; the harness reads `VANTAGE_PY` instead when it is set. The repo is public, so the harness writes the home folder as `%USERPROFILE%` in the ledger and environment rows (`public()` in `measure.py`), and a metric's command writes it `{home}`. A native job **outlives its caller**: if the session that started a run dies, the run goes on under the jobs service and its log holds the result. Stop one with `vantage.py job cancel <ticket>`; a stopped or killed run writes no ledger rows (they are appended only at the end), so a half run leaves nothing to clean.
3. **The harness watches the queue.** It reads `job status` before the first run, after each repeat and at the end. If another heavy, gpu or quick job appears in any reading, that **run is marked noisy and repeated**; a noisy run is never in the median.
4. **The box is idle at the start:** total CPU use under 5% over 10 s before the first run. The harness waits up to **10 minutes** for that. If the box never gets under 5%, it stops and reports the last reading; it does not wait forever.
5. **The lead coordinates the quiet windows** with KAT, who holds its heavy tickets for a window the lead names. A window is 10 to 20 minutes, so a suite is split into runs that each fit one window. The harness cannot hold the other slots; it can only tell that they were not held.

## The run

For each metric, in this order:

1. **Warm-up:** one full run, thrown away. It fills the disk cache, the shader and driver caches, and the CPU's clocks.
2. **Five repeats**, one after another, the same command, on a fresh process each time.
3. **Record the median, the minimum, the maximum and n.** The median is the number. The range is the spread. Never the mean.
4. **The suite twice for the noise floor** (next section), at the first run and at every re-seed.

The micro-benchmarks (`spade_bench`, Google Benchmark) use `--benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json`; the harness takes the five per-repetition rows, not the library's aggregates, so the minimum and maximum are real. The five repetitions of one benchmark share a process; the warm-up run is a whole separate process before them. The basis is **`real_time` for every metric** (the lead, 2026-11-16), and the ledger says which in `notes`. It was `cpu_time` for CPU metrics (TD-8) until the first baseline: Windows counts process time in 15.6 ms steps, so on this box a `cpu_time` of a few hundred microseconds per iteration is a whole number of those steps and the rows looked quantised. The CPU metrics run one thread on an idle box, where real time tracks CPU time. The benchmark JSON carries both, and the harness keeps each run's whole JSON next to its output (`--raw-dir`, default next to `--out-json`), so a basis can be re-read without a rerun.

A command that is not a micro-benchmark prints one line per number, `PERF <metric> <value> <unit>`; without that, the harness uses the process's wall time and its **peak working set**. A name printed several times in one run (one line per timed step) is that run's **median**, so a metric has one value per run and n is 5. A name that holds `.diag_` is a diagnostic: it is measured and kept in the run's JSON, never in the ledger. A program that opens a window (the editor) is run by the `editor-idle` kind: it launches it, leaves it hands off for `hold_s` seconds (30), closes it with `WM_CLOSE`, and reads the exit summary the sandbox prints (the mean frame, the split, the working set) plus the process's CPU seconds. The `live-smoke` kind runs the scripted tour into a fresh folder (ffmpeg on PATH), takes the process's wall time as the metric (3 repeats, its own window), and reads the journal for the video length, the frame count and the Run air chapters' wall seconds, as diagnostics. Both window kinds also read `PERF` lines from the program, like `perf-lines` (the editor prints them with `SPADE_EDITOR_PERF=1`). The mean is over every frame since launch, so start-up frames are in it; the notes say `mean_is_lifetime`. `PERF-NOTE <key> <text>` adds `key=text` to the ledger notes (a fixed input's path and size, for example). A command that prints no `PERF` line fails the run.

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
| free memory | free physical and free commit in GB at the start (the same reading the job service uses) |
| cpu | name and logical count |
| idle cpu | the pre-run reading |
| queue | the three `job status` readings, or `clean` |
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
- gives a metric its **env** (a registry field; a null value removes a variable), its **class** (`test` or `gpu`, the job class: run each class as its own job, `--class test` and `--class gpu`) and the placeholders `{bench}`, `{tests}`, `{flow_movie}`, `{build}`, `{repo}` and `{ticket}`;
- reads the queue, checks the box is idle, records the environment row, runs the warm-up and the repeats, and appends the ledger rows;
- `--dry-run` runs and prints but writes nothing (`--ignore-queue` with it tests the mechanics on a busy box);
- `--raw-dir` keeps every `spade_bench` run's whole JSON (both bases);
- `--out-json` keeps the raw values; `--floors A.json B.json --out-floors tests/bench/floors.json` computes the noise floors from two runs of one SHA (the test-class and gpu-class runs are separate files; `--out-floors` merges into the file that exists);
- `--trend` prints each metric's last two ledger rows and says "REAL" or "noise" by the rule above.

A real run: `python <vantage.py> job run --class test --ticket perf-<sha7>-1 --expect-s <N> --limit-s <2N> --log <abs log> --cwd <worktree> -- python -I tests/bench/measure.py --sha <gated sha> --tree <measured checkout> --build-dir build-ninja/release --out-json build-bench/<run>.json`. Run it from the Test/Docs worktree and commit the new ledger rows on a branch; the measured tree is never written to.

## The ledger

`tests/bench/ledger.csv`, one row per metric per measurement:

`date,sha,metric,unit,median,min,max,n,notes`

- **Appended, never edited.** A wrong row gets a new row with `notes` saying "supersedes <date>".
- **One row per metric per gated batch.** A slowdown shows as a step in a column.
- `metric` is `<owner>.<name>` (for example `core.step_quad_worlds/16`). `n` is 5. `notes` holds the run id, the basis (`cpu_time` or `real_time`), and `noisy=<count>`.
- Dates come from the machine clock.

## The first baseline (2026-11-16, da7a37bd)

Gate 30's SHA, measured read-only from its own checkout, in quiet windows agreed with KAT: pass A 01:30 to 02:05 local, pass B 07:30 to 08:05, each as three jobs (T: the test class; G1: Physics, the puppet frame and the web frame; G2: the editor, the live smoke and the GPU quad worlds). Every run had `noisy=0`.

- **The ledger** holds the gpu-class rows from pass A (n=5; the live smoke n=3) and the test-class rows from pass B. The test class was first run on `cpu_time`; the lead then ruled `real_time` for every metric, and pass A's raw JSON had not been kept, so pass A's test rows are not used. The test-class rows come from window B, so their run id is `2026-11-16_da7a37b_floor` and they carry `floor=pending_second_window`.
- **The floors** (`floors.json`) are measured for the gpu class from A and B. **The test-class floors are pending a second window** on `real_time` (one more T-only window, the evening of 2026-11-16, on the same SHA). The test class matters only for changes to CPU code (the Lab's T003 and T004 are the cases): a change claimed on a test-class row is not called real until its floor is in. None of the L74b changes is judged on a test-class row: the shadow fit on the puppet frame, `box_down` on the 4K web frame, the gather early-out on the Physics GPU rows, and render on demand on the idle editor.
- **`interface.editor_idle_frame` and `_cpu` have the widest floors** (92% and 64%): the idle editor runs under `--no-vsync`, uncapped, so its frame rate and the CPU seconds it burns in the 30 s hold are not steady quantities. The floor is kept as measured. A render-on-demand change aims at near zero idle CPU, far outside any floor.
- **The harness SHA** in the environment rows reads the measured tree's `HEAD`, because the harness for these windows ran from a copy of this branch's `measure.py` (the measured tree must stay at the gated SHA, so the file was patched outside it: `--raw-dir` and the table's floor column).
- **Not in this baseline** (their first rows are their own baselines, taken in a short extra window after each merges): Interface's `SPADE_EDITOR_PERF` rows (`interface.editor_first_frame`, `interface.run_air_frame_*`), Physics' editor 64^3 grid and per-phase readout, `BM_RenderShadows`, and the live smoke's reworked tour (`interface/smoke-refurb`).

## The metrics

The table is generated from `tests/bench/metrics.json` (`python tests/bench/measure.py --table`); that file is the source. A `spade_bench` family gives one ledger row per size, named `<metric>/<size>`. The first rows are the existing `spade_bench` families: the step and render families are metrics (Core and Rendering), `IntegrateOrientation` and `CsgMeshShell` are diagnostics, in `baselines.json` only. The realms' own metrics (Physics, Core, Rendering, Interface) are added as they send them; KAT's training numbers are KAT's.

| Metric | Who waits, for what | Workload | Command | Unit | Owner | Kind | Class | Floor |
|---|---|---|---|---|---|---|---|---|
| `core.step_plain_bodies` | a developer or user stepping the simulation (one world of plain bodies) | BM_StepPlainBodies, the sizes in bench_sim.cpp; one iteration is step(50), reported per step (ms) | `{bench} --benchmark_filter=^BM_StepPlainBodies/ --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | test | pending the second window |
| `core.step_dense_bodies` | a developer or user stepping a world crowded with bodies | BM_StepDenseBodies, the sizes in bench_sim.cpp; one iteration is step(50), reported per step (ms) | `{bench} --benchmark_filter=^BM_StepDenseBodies/ --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | test | pending the second window |
| `core.step_quad_worlds` | a user flying drones in real time on the CPU backend (quad worlds, 1 to 64 vehicles; the 1 kHz step budget is 1 ms) | BM_StepQuadWorlds, the sizes in bench_sim.cpp; one iteration is step(50), reported per step (ms) | `{bench} --benchmark_filter=^BM_StepQuadWorlds/ --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | test | pending the second window |
| `rendering.render_background_fill` | a user waiting for a frame (the empty-scene floor of the CPU renderer) | BM_RenderBackgroundFill, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderBackgroundFill --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | test | pending the second window |
| `rendering.render_ground_plane` | a user waiting for a frame (the ground plane on the CPU renderer) | BM_RenderGroundPlane, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderGroundPlane --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | test | pending the second window |
| `rendering.render_slots` | a user waiting for a frame with many objects on the CPU renderer | BM_RenderSlots, the sizes in bench_render.cpp | `{bench} --benchmark_filter=^BM_RenderSlots --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | metric | test | pending the second window |
| `core.step_quad_worlds_gpu` | a user or trainer flying drones in real time on the GPU backend (quad worlds, 1 to 64 vehicles; the 1 kHz step budget is 1 ms) | BM_StepQuadWorldsGpu, the sizes in bench_sim.cpp; one iteration is a batched step(50), reported per step (ms) | `{bench} --benchmark_filter=^BM_StepQuadWorldsGpu/ --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | core | metric | gpu | up to 15.9% |
| `physics.gpu_airflow` | a user of Run air waiting for the GPU airflow: one step of the D1 box with its solids, one step of the deep box (and the device memory that run holds, in MiB), and one live frame on the editor's 32^3 grid | the three tests in the command (Physics' physics/perf-lines 679bf4e8): physics.gpu_step_d1_solids (ms, a line per timed step), physics.gpu_step_deep (ms, a line per timed step after 2 warm-ups), physics.gpu_step_deep_peak_mem (MiB, this process's device memory), physics.gpu_frame_editor_32 (ms, the mean of 30 LiveRun frames, 32^3 at 25 mm, 4 discs). About 60 s a run. The editor's 64^3 grid is not here yet | `SPADE_AIRFLOW_TIMINGS=1 SPADE_SGS_MEASURE=1 SPADE_SGS_D1_DEEP=1 {tests} --gtest_filter=GpuAirflowStep.StepTimeInTheD1BoxWithItsSolids:GpuAirflowStep.SubgridBoundAndCostInTheD1BoxWithItsSolids:GpuLiveFluid.FrameTimeOnTheEditorsGrid` | ms | physics | metric | gpu | up to 1.9% |
| `physics.gpu_airflow_diag` | no one: the open-box step, the projection, gather and diffuse kernels alone, and the CPU references | the four tests in the command (physics.diag_*; report only); run with --include-diagnostics. About 16 s a run | `SPADE_AIRFLOW_TIMINGS=1 {tests} --gtest_filter=GpuAirflowStep.StepTimeInTheD1Box:GpuAirflowProjection.TimingsAt64And128Cubed:GpuAirflowActuator.GatherTimeInTheD1Box:GpuAirflowViscous.DiffuseTimeInTheD1Box` | ms | physics | diagnostic | gpu | not measured |
| `rendering.puppet_frame` | a trainer watching the live training view (the frame with N drone puppets on GL, 1280x720, shadows on and off; N = 1, 128, 1024) | GpuGlRenderer.PuppetFrameCostAt128And1024Drones on the in-repo UV sphere (14,000 triangles); median of 20 frames after 3 warm-ups in each run. Ledger rows rendering.puppet_frame_<N>_shadows_<on|off>; phases rendering.diag_{pose,submit,wait,gpu,read}_<N>_shadows_<on|off> are diagnostics | `SPADE_GL_PUPPET_COST=1 {tests} --gtest_filter=GpuGlRenderer.PuppetFrameCostAt128And1024Drones` | ms | rendering | metric | gpu | up to 22.2% |
| `rendering.web_frame_4k` | a person making a website clip (one 4K web frame, drawn from a solved cache; the solve is in the discarded warm-up, never in a value) | flow_movie --web S1, the 30 frames drawn from the fixed 180-frame cache (S1 draws from frame 150); the median per-frame loop body without the encode tail. Phases rendering.diag_web_<phase> are diagnostics. The cache path and byte count go in the ledger notes. The cache (about 20.5 GB) is made in the warm-up run (a solve of about 219 s), kept for the repeats and deleted after the metric (scratch); the metric needs 24 GB free (free_gb), else the harness exits 5 before measuring | `{flow_movie} --web S1 --web-frames 180 --keep-cache --out {web_out} --ticket {ticket}` | s | rendering | metric | gpu | 2.7% |
| `interface.editor_idle` | a tester with the editor open and hands off (frame time and CPU seconds of an idle editor on the hover scene; the baseline for render on demand) | the editor on hover, no vsync, 30 s hands off, then the window closed. Rows interface.editor_idle_frame (ms, the exit summary's lifetime mean, so start-up frames are in it) and interface.editor_idle_cpu (CPU s over the run); the split and the frame count are diagnostics interface.editor_idle.diag_*. With SPADE_EDITOR_PERF=1 the editor also prints PERF interface.editor_first_frame (ms, process creation to the first presented frame), a ledger row once interface/l74-metrics (ef82d560) is merged | `SPADE_EDITOR_PERF=1 {sandbox} --no-vsync` | ms | interface | metric | gpu | up to 92.0% |
| `interface.startup_version` | a tester starting the program (the floor: loader, launcher and exit; not the time to the first frame, which is not recorded) | spade_sandbox --version, process wall time and peak working set | `{sandbox} --version` | s | interface | metric | test | pending the second window |
| `interface.live_smoke` | a developer or the gate waiting for the live smoke (the whole scripted tour with its recording; wall time of the process, not the video's length) | spade_sandbox --live-smoke, the tour of the drone box, the airflow, the editor and the batch; ffmpeg on PATH. Row interface.live_smoke_wall (s); diagnostics: the video length, the frame count, the Run air chapters' wall seconds and the peak working set. The run must pass (a FAILED tour fails the run). n=3, its own window. With SPADE_EDITOR_PERF=1 each Run air that stops prints interface.run_air_frame (ms, one air frame, a ledger row) and the diagnostics interface.diag_run_air_step, _publish and _assemble; the tour has two (the editor.airflow chapter's, then batch.turbulence_air with subgrid on), suffixed _editor_airflow and _batch_turbulence. These rows exist once interface/l74-metrics (ef82d560) is merged | `SPADE_EDITOR_PERF=1 {sandbox} --live-smoke --out {smoke_out} --label perf {ticket}` | s | interface | metric | gpu | 1.7% |
| `core.integrate_orientation` | no one: one sub-step of the integrator | BM_IntegrateOrientation | `{bench} --benchmark_filter=^BM_IntegrateOrientation --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ns | core | diagnostic | test | not measured |
| `rendering.csg_mesh_shell` | no one: one stage of mesh building | BM_CsgMeshShell | `{bench} --benchmark_filter=^BM_CsgMeshShell --benchmark_repetitions=5 --benchmark_min_warmup_time=1 --benchmark_format=json` | ms | rendering | diagnostic | test | not measured |

## The web cache (`rendering.web_frame_4k`)

The metric draws 30 frames from a solved 180-frame cache of about **20.5 GB**. The harness no longer keeps that cache between runs (C: had 28 GB free):

- The cache is made under `build-bench/perf-web-s1-30` in the **warm-up run**, which is thrown away, so the solve (about 219 s) is in no value. The repeats reuse it. The folder (the metric's `scratch`) is emptied before the first run and **deleted after the metric**, whatever happens; the harness refuses to delete anything outside `build-bench`.
- The metric needs **24 GB free** (`free_gb`). If the drive has less, the harness exits **5** ("disk short") before measuring anything. The job queue's `--floor-gb` is memory, not disk, so the caller starts this metric when C: has the room.
- Plan the job at about 4 minutes more than the repeats (`--expect-s`), for the solve.
- **The cache's byte count** is in the ledger note (`web_cache=<path>_<bytes>_bytes`). It was 20,553,588,651 at da7a37bd. The same solver gives the same count, which is a free determinism check. A change to the solver changes the count (the L90 fix will): record the new count as the new baseline in the note, with the reason. A different count then is no failure.

## Re-seeding `baselines.json`

The October seed predates most of the code. At the first gated SHA, under this protocol, the existing `spade_bench` families are measured again and `baselines.json` is rewritten with the box, driver, compiler and SHA in its `_meta`. The old seed stays in git history. After that, the ledger carries the trend and `baselines.json` carries only the reference run. **Done 2026-11-16, at da7a37bd:** the nine `spade_bench` families, the median of five repetitions each, from window B's raw JSON; the October seed and its two older blocks are in git history.

## Not covered

- **Other machines.** A number is for this box, this driver and this power plan; the environment row says which.
- **A noisy neighbour outside the job service** (a browser, the owner's game, a virus scan) is caught only by the idle check and the noisy mark, not prevented.
- **Debug builds and sanitizer builds** are not measured.
