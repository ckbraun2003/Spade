# Test/Docs — build and gate

**Owner:** Test/Docs. **Normative** for the gate; the rest describes the tooling as it is.

## Scripts

Run them in the foreground. The `.ps1` scripts keep to ASCII (non-ASCII breaks `.ps1` on Windows PowerShell 5.1); the `.sh` scripts run inside the Docker leg's container and check out LF (`.gitattributes`).

| Script | What it does | Parameters |
|---|---|---|
| `scripts\build.ps1` | Imports the MSVC environment, configures the preset if needed, builds | `-Preset debug\|release` (default release), `-Target <one>`, `-BuildDir <configured tree>`, `-ParallelLevel 1..64` (default 8), `-Clean` |
| `scripts\test.ps1` | `ctest -L spade --output-on-failure --no-tests=error` against a built preset | `-Preset`, `-Filter <regex>` |
| `scripts\demo.ps1` | Builds if needed and launches one viewer scene | `-Scene <name>`, `-Preset`, `-Backend cpu\|vulkan` |
| `scripts\docker-leg.ps1` | The Docker leg (below): builds and tests one commit on Linux/gcc in a container, then the consumer smoke | `-Commit <rev>` (default `HEAD`), `-Run <id>` (default the commit's short sha), `-Step all\|image\|sync\|configure\|build\|test\|consumer`, `-Memory` (default `8g`), `-Jobs` (default 8), `-NoSeed`, `-Follow`, `-Stop`, `-Clean`, `-Prune [-Keep N]` (default 4) |
| `scripts/docker-leg.sh` | The leg's steps inside the container. The driver runs the commit's copy for the step, and its own copy for `seed` and `sync` | `<step> [--jobs N]`, `seed [--from VOLUME]` |
| `scripts/gcc-check.sh` | The gcc check before review (below), from Git Bash | `[--compile] <file>...`; `GCC_CHECK_VOLUME=<volume>` picks the volume; `GCC_CHECK_BUILD=<dir>` names the worktree build whose generated headers go first (default `build-ninja/release`) |
| `scripts/consumer-smoke.sh` | **Interface's.** Installs Spade into a fresh prefix, then builds and runs `tests/consumer` against it | `--vulkan ON\|OFF --work DIR [--from-build DIR] [--deps DIR] [--jobs N]` |

## Presets and options

- `CMakePresets.json`: `msvc-ninja-debug` and `msvc-ninja-release`, building into `build-ninja/debug` and `build-ninja/release`. CMake 3.28 or newer.
- Options: `SPADE_VULKAN`, `SPADE_RENDER_GL`, `SPADE_BUILD_SANDBOX` and `SPADE_BUILD_V1`, all default `ON`. A `-DSPADE_VULKAN=OFF` configure has no preset of its own.
- A worktree builds in its own directory; two sessions never share a build tree.

## The gate

`TD-7`: green means `scripts\test.ps1` green on **both** presets, with the counts reported per `TD-8`. There is no hosted CI (`TD-11`). The gate therefore runs on one machine, one toolchain (MSVC) and one GPU, and it says so when it reports.

**GPU coverage** (`TD-13`). On the development box the `gpu`-labelled tests run as part of the gate. A `gpu` test that skips is reported by name and does not count toward green, even though `scripts\test.ps1` exits 0 with skips. A run on a machine without a device therefore cannot make the gate green. The Docker leg excludes the label explicitly (`-LE gpu`) rather than letting those tests skip.

**The second toolchain and the consumer smoke** are one self-run Docker leg (`TD-11`, below). A regenerated golden is final only once that leg reproduces it (`TD-12`).

`--no-tests=error` fires only when **zero** tests are registered. A battery that always skips stays invisible to it, which is why skips are reported by name.

### Verification is tiered

Cameron decided this on 2026-10-21, by AskUserQuestion to the lead (backlog, `3473490`), to spend fewer tokens per merge. `TD-7`'s "both presets" is now met at the lead's batch gate, not on every branch. Heavy runs go through the job queue ("This machine", below).

**Per branch, before "ready":**
- **A full build** of all targets.
- **The release CPU suite** (`-LE gpu`), in jobrun chunks.
- **The GPU tests** (`-L gpu`, the queue's `gpu` class), only when the branch changes GPU code: `engine/render`, `engine/render_gl`, `engine/shaders`, `engine/compute`, the flow volume's GPU path, or anything else that runs on the device.
- **The live smoke,** only when the branch changes `sandbox/` or editor code.
- **The gcc check** ("The gcc check before review", below): `--compile` on every changed `.cpp`, the plain check on changed headers. A new `.cpp` that gcc-check can't place uses a sibling's command line or a `TEMPLATES` entry. Failing both, the branch needs a leg.
- **Its own `-NoSeed` leg,** as well, when the branch touches the solver (`engine/sim`, the airflow code) or moves a pinned format or golden (`TD-1`, `TD-12`).
- **The records' reader** (`AnalysisValidation*`, which holds `TheV1RecordsRead`), whenever the branch changes anything under `tests/data/airflow/records/`, whatever else it skips. That includes a pasted block that nothing is built from. It runs after the branch's last record commit. It needs no rebuild, because the test reads the records from the source tree when it runs. The lead set this on 2026-10-22, after B4's block on master (`65cfccd`) broke `TheV1RecordsRead`: the block was committed after the branch's gated run.

Per-branch debug chunks are dropped.

**At the lead's batch gate:** the debug suite in chunks, the `-NoSeed` leg, and the live smoke. The lead posts the smoke as `C:\Users\Cameron\Desktop\Projects\Videos\Smoke\Spade\<UTC time>_batch-<n>-live-smoke.mp4` with its caption, gets `check_video.py` PASS, then sends a push notification. `Videos\CATEGORIES.md` holds the rules for every posted video (Cameron, 2026-10-21; both leads agreed); `Videos\check_video.py` checks and encodes them. A branch's smoke is not posted; its MP4 stays in `spade-smoke\`.

A ready message names the tiers it ran, and which of the conditional ones it skipped and why ("no GPU code changed").

### Messages and subagents

These are Cameron's token decisions of 2026-10-21, relayed by Kat's lead (backlog `64ff6c4`, `f17c503`).

- **Send only actionable messages:** a decision needed, a ready SHA with its gate lines, or a blocker. Leave out acknowledgements and FYI copies. Batch them; the lead answers once per batch.
- **Subagents:** use Sonnet at high effort for mechanical work (running tests, reading logs, pin searches, merges, surveys). Opus does design, code and review.
- **End subagents; never leave one stuck** (Cameron, 2026-10-21 07:28 UTC; backlog `47e8767`): end each subagent when its task is done (its last step is `SubagentHandback`), and stop one the token monitor marks stuck (45 minutes mid-turn, nothing written) by its task id, then restart its work from the resume note. Check that nothing is left running before ending a turn, and give every build, gate and wait a timeout.
- **Where to commit** (Cameron's token monitor, 2026-10-21, with the lead's correction): commit code in your own worktree (`spade-wt/<realm>`). Docs-only commits stay on master in the main checkout, under your realm's scope (`docs(test-docs): ...`). Don't split or pad commits to move the numbers.
- **Read less.** Everything read is paid for again on every turn, and 55–70% of sampled reads were of files already read three or more times.
  - Find a symbol with `grep -n`, then read only that range (an offset and a limit).
  - For gate scripts and jobrun logs, read the pass/fail summary lines, never the whole log.
  - Read an image only to judge that frame, and downscale it first.

## The Docker leg

`scripts\docker-leg.ps1` builds and tests **one commit** on Linux, the way a Linux checkout sees it, and runs the consumer smoke against an installed prefix. It is the second toolchain (`TD-12`) and the only place `tests/consumer` runs (`TD-11`). It complements the gate and does not replace it.

| | |
|---|---|
| **Covers** | gcc-13 and CMake 3.28.3, the project's floor; Release with Vulkan, the sandbox and GL compiled; `ctest -L spade -LE gpu`, with the viewer-trajectory guard at full length (`SPADE_FULL_VIEWER_TRAJECTORIES=1`; the gate checks a prefix); `tests/consumer` against a fresh install with `SPADE_VULKAN` ON (from the leg's own tree) and OFF (a library-only tree), plus the `SL2b` sandbox stage when the commit's `consumer-smoke.sh --help` offers `--sandbox`; the summary says whether it ran |
| **Leaves out** | v1 (`SPADE_BUILD_V1=OFF`: frozen, and never on a Linux leg); the `gpu` label, excluded rather than skipped (`TD-13`); Debug; Windows, which the gate covers; a window (no X11, so GLFW builds without a backend) |
| **Tests a commit** | `git -c core.autocrlf=false archive <commit>`, because a plain archive on this box emits CRLF. Uncommitted changes are not in the leg, and the driver says so when there are any. The image is built from that commit's own `scripts/docker-leg.Dockerfile` and tagged by the file's content hash |
| **State** | One Docker volume per run, `spade-docker-leg-<run>`; the run is the commit's short sha unless `-Run` names it. The volume holds `/leg/src` (the commit, synced so unchanged files keep their mtimes and ninja stays incremental), `/leg/build` (dependencies in `_deps`), `/leg/consumer`, and `/leg/seed` and `/leg/previous`, which record where the volume came from. `-Clean` drops the run's volume; `-Prune -Keep N` drops idle leg volumes beyond the newest N and never one a container uses. A leg volume carries the label `spade.leg`, and the scripts consider no other: mounting a volume name that does not exist makes Docker create it empty and unlabelled |
| **Seeds** | A new volume starts as a copy of the newest leg volume that holds a configured build and that no running container uses: build, source, consumer tree and commit, every mtime kept. It then rebuilds only what the commit changed and fetches nothing. The sync after it stamps every file it writes past the newest output if the clock is not ahead of them, so a seed built under a later clock (or a WSL clock that stepped back) cannot leave a stale object. `-NoSeed` starts empty |
| **Runs** | Detached, in a container named like its volume, with `--memory 8g` and `-j8` by default and no swap. Legs on different commits run side by side. Closing the terminal does not stop a leg; running the driver again for the same run re-attaches rather than starting a second leg, and `-Stop` ends it. `-Follow` and `-Stop` act on the only leg, or on the one `-Run` (or `-Commit`) names. The driver's own `docker-leg.sh` seeds and syncs, since an older commit's copy predates those steps; the commit's copy runs the step. Stay with a leg in the foreground |
| **`TD-12`** | A leg that makes a regenerated golden final runs on a fresh volume (`-NoSeed`), so no object in it predates the commit |
| **Order** | configure, build (`ninja -k 0`, so one run lists every failing file), test, agreement, consumer ON, consumer OFF. `-Step consumer` also configures and builds first, so consumer ON installs the commit under test and never a tree some earlier commit built. A failed part blocks only the parts that need it. The exit code is 0 only if every part passed |
| **Output** | `build-docker\<short-sha>\`, or `build-docker\<short-sha>-<run>\` for a named run: `summary.txt` (each part's result and seconds, the commit, the toolchain, peak memory, free disk, and the volume: how it began and what it held before the run), `build-errors.txt` (repo-relative `file:line`), `ctest.log`, `ctest-junit.xml`, `tests.txt` (registered names), `leg.log` |
| **Verdict** | The driver exits 0 only when every docker call answered, the leg's container exited 0, and this run's `summary.txt` opens `docker leg: PASS`. Anything else exits non-zero, and the last line names why (`docker-leg: exit N, FAIL: ...`): a daemon that died (`docker wait`, `logs` or `ps` failed), a missing summary (the leg did not finish), or a summary that says FAIL. Before it starts a run, the driver deletes the folder's `summary.txt`, so one left by an earlier run of the same commit cannot stand. Why: on 2026-10-13 Docker's daemon died mid-build in Physics' leg, `docker wait` printed nothing, and the driver read that as exit 0 |
| **Refuses** | Under 4 GB free on the host drive (`-MinFreeGB`), since a full disk truncates files mid-build |
| **Prune** | Each run leaves its volume behind, about 1.1 GB. `-Prune [-Keep N]` keeps the newest `N` (default 4, by creation date) and any volume a container uses, running or exited, and removes the rest. **The lead prunes with `-Keep 4` after each merge batch** (adopted 2026-10-12, when 61 volumes held 68 GB). Before that, a realm with a pending `TD-12` leg, or a branch volume it still needs, names it to the lead, and the prune waits until that is done. Older volumes serve nothing by default: seeding and `gcc-check.sh` take the newest idle volume, and a leg's record is its `build-docker` summary, which no prune touches. Pruning frees space inside Docker's disk file (`docker_data.vhdx`); the file itself on C: does not shrink by itself |

A red leg is reported per realm with the failing `file:line`, and each realm fixes its own code. Measured times and peak memory are in `07-status.md`.

## The gcc check before review

**Before "ready for review", every C++ file a branch adds or changes is compiled with gcc-13 in the leg's image, and the message says so.** This is the lead's standing practice from 2026-10-03, after master was gcc-red from `da2fcf5` to `7ed7572` over a warning MSVC never raised.

- **By default it is a syntax check** (`-fsyntax-only` with the engine's `-Wall -Wextra -Wpedantic -Werror`). It catches front-end warnings such as `dangling-else` and `missing-field-initializers`. It does not run the optimiser, so it misses warnings only optimisation finds, such as `-Wmaybe-uninitialized` and `-Warray-bounds`.
- **`--compile` compiles each `.cpp` in full,** with the leg volume's own Release command line (`-O3`, from `ninja -t compdb`) plus `-Wall -Wextra -Wpedantic -Werror`, so those optimiser warnings show up in seconds rather than a leg's minutes. Headers still get the syntax check. A `.cpp` the volume's build never compiled (a new file) fails with that reason, and needs a leg, except a new `tests/test_*.cpp`: every `spade_tests` file compiles with the target's one command line, so a new one borrows the line of the first `spade_tests` file the build does compile, and the output names it (the lead, 2026-10-19). A new `spike/airflow/*.cpp` likewise borrows `chain_runner.cpp`'s line, its target's one command line (`TEMPLATES`). Use it on every changed `.cpp` before review (Rendering's practice from `flow-iso`, made a script mode on 2026-10-06).
- **`spike/` is checked like `engine/`** (the lead, 2026-10-22, after plan B's B1 and B2 were checked with patched copies). It is copied in, and the repository root on the compile line (`-I/leg/src`, which `spike/airflow/...` includes are spelled from) points at the branch's copy. Before, a changed `spike/` header was read from the volume's leg commit, so a defect in it passed.
- **Generated headers come from the worktree's own build first** (same date). The volume's are its leg commit's, so a branch's new or changed kernel was checked against old headers or none. Each generated-header directory the volume holds (`engine/`'s and `spike/airflow/`'s `generated/spade_slang`) is read from the worktree's build first: `build-ninja/release` when it exists, `GCC_CHECK_BUILD=<dir under the worktree>` for another, empty for the volume's alone. Build first: a `.slang` newer than the build's last generator output is named in a `gcc note:`.
- **Until batch 14's gate leg makes a post-B4 volume, the airflow files need a pinned volume** (the lead, 2026-10-22). For `spike/airflow/` and `tests/test_gpu_airflow_*` files, set `GCC_CHECK_VOLUME` to a volume at or after B1, for example `spade-docker-leg-physics-b1-4576086`. The newest volume, Core's `spade-docker-leg-ac57cb8c5416`, predates B2: its command lines have no directory for the airflow kernels' generated headers (`airflow_actuator_gather.spv.gen.hpp`), so those files fail there when nothing is wrong with them.
- **Some files it cannot check honestly.** A file that depends on a generated-header directory the volume lacks, or on a target's own compile definitions, is named in the message instead, and the lead schedules a leg run before merge.
- **Window-backend code borrows or adds a line** (the lead, 2026-10-13). The leg's container has no X11, so GLFW builds without a window backend and some code never meets gcc in a leg; `--compile` covers it anyway, and says how in each line:
  - `tools/flow_movie/*` is never built by a leg (the root `CMakeLists.txt` adds it only with a window backend). Its files compile with the command line of `tests/test_gpu_gl_renderer.cpp`, which the leg does build against the same `render_gl`, glad and GLFW, with the same defines (`TEMPLATES` in `scripts/gcc_check_compile.py`);
  - `sandbox/editor_panels_ui.cpp` is built with its backend branch off (`SPADE_SANDBOX_HAS_GL=0`), so it compiles a second time with it on, adding imgui's and GLFW's includes from the volume's `_deps` (`BACKEND_ON`);
  - `sandbox/gl_target_sink.cpp`'s backend branch includes `<GL/gl.h>`, which the leg's image lacks, so only MSVC compiles it, and the check prints a `gcc note:` saying so (`UNCHECKED`).

  A new window-backend target adds its row to `TEMPLATES` in the same commit. Never built by a leg for other reasons, and not covered: v1 (`src/`, `examples/sandbox`, the viewer; frozen, `SPADE_BUILD_V1=OFF`), and `tests/consumer/`, which the leg's consumer step builds in its own trees.

From the worktree root, in Git Bash:

```bash
scripts/gcc-check.sh tests/test_example.cpp engine/render/example.hpp
scripts/gcc-check.sh --compile tests/test_example.cpp engine/render/example.cpp
```

- **What it does:**
  - picks the newest leg volume (label `spade.leg`) that holds a configured build and that no running container uses, so it never reads a volume a leg is building, and prints that volume and its commit; `GCC_CHECK_VOLUME=<volume>` picks one instead;
  - mounts the worktree and that volume, both read-only; the volume supplies the dependency headers, and the generated headers after the worktree build's;
  - copies the sources inside (`engine`, `sandbox`, `tests`, `tools`, `include`, `src`, `examples`, `spike`), because reading headers through the Windows mount is slow;
  - prints `gcc ok:` or `gcc FAIL:` per file, and exits non-zero if any file fails;
  - checks each header through a one-line translation unit that includes it.
- **Its compile line was proved red, then green,** on 2026-10-03, when it was still a command pasted from this page:
  - at `da2fcf5` it failed `tests/test_render_agreement_matrix.cpp` at :95, :97 and :100 (`missing-field-initializers`);
  - at `ec6e2da` the fixed file passed, and so did two headers.
- **The `spike/` and generated-header fix was proved red, then green,** on 2026-10-22, on trees exported from B1 (`4576086`, against its own leg volume) and B2 (`12594a0`, with Rendering's build's generated headers):
  - the old check failed every `spike/` file (not copied), failed B2's new `projection_chain.cpp` (no line) and its new test (no `airflow_divergence.spv.gen.hpp`), and passed `tests/test_gpu_airflow_runner.cpp` with a dangling `else` seeded in `chain_runner.hpp` (exit 0);
  - the fixed check passed the clean files, and failed the seeded header in all three files that read it.
- **`--compile` was proved the same way** on 2026-10-06, with Rendering's `flow-iso` commits and the leg volumes built from them:
  - at `81a6438` it failed `engine/render/flow_render.cpp` with both `-O3` errors the leg had found: `array-bounds` in `std::sort`'s insertion pass, and `maybe-uninitialized` copying an optional vector. The syntax check passed the same file;
  - at `02ac4bd` the fixed file passed;
  - a `.cpp` the volume never built failed with its reason, and a header fell back to the syntax check;
  - cost: 49 s for two files.
- **Cost:** about a minute per heavy test file on the old box (126 s for three files); on this machine, 11 s for one heavy test file and one header.
- **When to run it:** whenever it's needed, with as many files as you like, beside any number of legs. It refuses (exit 2) only when no leg volume is idle.

## This machine

**Since 2026-10-04 Spade builds on a new machine:** 32 GB, 16 threads and an RTX 3060 Ti. Kat's tree shares it.
- **No slots and no budget:** build and test as the work needs (`docs/design/consumers.md`, "Shared build machine"). The user lifted every memory-based limit on 2026-10-04.
- **Docker's Linux VM is capped at 10 GB** (Docker reports `MemTotal` 9.71 GB). The cap is `.wslconfig` (`memory=10GB`), which is the user's machine configuration, set on 2026-10-05 on the user's word; only the user changes it. Before the cap, `vmmem` held 8.6 GB; just after it, 2.2 GB.
- **The VM hands back its cache:** `.wslconfig` also sets `autoMemoryReclaim=gradual` under `[experimental]`. Cameron approved it on 2026-10-12, via AskUserQuestion to the lead; like the cap, it is the user's configuration, and only the user changes it. Why: the VM kept the page cache of builds and legs long after they finished, so Windows' free memory stayed low; Rendering's debug run was killed at 3.9 GB free with no game running. With `gradual`, the VM returns cached memory to Windows a little at a time once it has been idle for a few minutes. It was applied by restarting WSL (version 3.0.1) and Docker on 2026-10-12. `vmmem` fell from 7.4 to 2.2 GB, but that drop came from the restart itself; the setting's own effect shows after a leg, as `vmmem` falling back while the VM idles.
- **Docker legs under the cap:** run at most one fresh leg at a time. If two legs must overlap, run both with `-Memory 4g -Jobs 4`.
  - Why: a fresh (`-NoSeed`) leg at the defaults peaked at 4.29–4.50 GB, and a seeded leg with nothing to rebuild at about 0.9 GB.
  - Two fresh legs plus the VM's own overhead come near the cap, and the VM's out-of-memory killer acts before either leg's own `--memory` limit would.
- **Settings:** the scripts' defaults suit this machine (`scripts\build.ps1 -ParallelLevel 8`; the Docker leg `-Memory 8g -Jobs 8`, within the rule above). Run builds in the foreground, in chunks of at most 10 minutes, resuming each.
- **Every heavy command goes through the box-wide job queue** (live 2026-10-20; Kat installed it on Cameron's word, and the lead made it Spade's box rule). It replaces the hand-run memory checks below.
  - **The hook:** while memory is under the floor or Cameron's game runs, it blocks heavy commands run outside the queue. In the shared checkouts it also blocks `git add -A`, `git add -u`, `git add .` and `git stash`.
  - **The call:** run python directly, not inside PowerShell:

    ```text
    python C:/Users/Cameron/Desktop/Projects/kat-wt/_handoff/tools/jobrun.py --heavy --heavy-class <build|test|gpu> [--floor-gb 8] --limit-s <N> [--expect-s <E>] [--wait-max-s <S>] --ticket <tree>-<realm>-<what>-<sha> --log <FILE> -- <command>
    ```

  - **What v2 adds** (live 2026-10-21; Cameron ran the install):
    - **`--expect-s`:** the run time you expect, which defaults to `--limit-s`. A job expected to take over 900 s is LONG, and only one long job runs at a time.
    - **The quick lane:** a job with `--expect-s` of 90 or less and `--limit-s` of 180 or less.
    - **`--ticket <tree>-<realm>-<what>-<sha>`:** says whose job it is and what it does, for example `spade-testdocs-leg-1a2b3c4`. Never start a ticket with the class (`gpu-`, `cc-`) or with only the realm (`testdocs-`). A retry uses the same ticket, so it keeps its place instead of adding a second entry.
    - **`--queue-status`** shows each job's size and its working directory, as well as who holds the queue and who waits.

  - **No duplicate jobs** (Cameron, 2026-10-21, via the lead). The monitor marks a job as DUP when its command and folder match another job's.
    - Before you queue a job, run `python C:/Users/Cameron/Desktop/Projects/kat-wt/_handoff/tools/jobrun.py --queue-status`.
    - Don't queue a command that is already running or waiting from the same folder. Wait for that run instead.
    - If another realm has the same test queued at the same commit, use that run's result. Ask the lead if you can't get it.
    - If a job you no longer need is still waiting, stop it before you queue a new version.
  - **Don't pay to wait, and clear a cross-realm change before you make it** (Cameron, 2026-10-22, via the lead; for every realm in every project). In the owner's words:
    > "Don't pay to wait, and clear a cross-realm change before you make it.
    > 1. Don't wait with a big context. When a subagent's next step is a queued job (build, test or gate) and its context is over 200k, it writes its state to its resume note (the ticket, the log path, and what to check when the job ends) and hands back. A fresh, small subagent picks up when the job's notice arrives and reads the log. No polling: one look when the notice comes.
    > 2. Queue checks together. Queue independent legs in one go, not one after another, and re-run only the legs your change touches.
    > 3. Ask before you cross a realm. Before you change something another realm's tests or data rely on (a sign or unit convention, a shared schema, pinned outputs), list the files and tests it reaches and get your lead's grant for them first, so the commit isn't held up after the gate fails.
    > 4. Keep wip parking commits rare. At most one per change, with the reason in the resume note.
    > Why: last week KAT Components spent about 317 responses per code commit (Editor spent 74). Most of it was subagents waiting in the queue with 500k to 700k contexts, plus a fix that broke Runtime's tests and then waited for a grant."
    - **Standing grant under rule 3** (the lead, 2026-10-22; backlog `9449f79`). A realm may add its own new test files to the shared `tests/CMakeLists.txt` lists without asking first, as long as nothing else there changes (no other target, option or link). It names those files in its READY message. Every other edit to a shared CMake file (the root `CMakeLists.txt`, `engine/CMakeLists.txt`, another realm's target, links, options) is still listed for the lead's grant before it is made.

  - **The classes:**

    | Job | Class and options | Command after `--` |
    |---|---|---|
    | An MSVC build | `--heavy-class build` | `powershell -NoProfile -ExecutionPolicy Bypass -File scripts\build.ps1 ...` (it takes the queue's `JOBRUN_PARALLEL` as its job cap, and says so on its first line) |
    | A CPU ctest chunk | `--heavy-class test` | `ctest ...` |
    | `ctest -L gpu` | `--heavy-class gpu` (one at a time) | `ctest -L gpu ...` |
    | A Docker leg | `--heavy-class test --floor-gb 8` | `powershell -NoProfile -ExecutionPolicy Bypass -File scripts\docker-leg.ps1 -Commit <sha> ...` |

  - **A path pitfall:** from Git Bash, write `C:/Users/...`, never `/c/Users/...`. Python takes an MSYS path literally.
    - History: before v2, a relative executable after `--` resolved against jobrun's own working directory, not `--cwd`, and failed with WinError 2 only after the whole queue wait (Physics, 2026-10-20). v2 resolves `argv[0]` against `--cwd`, or refuses at once.
  - **Its limits:** `--limit-s` counts from the job's start. Exit 125 means the job gave up waiting for its turn (`--wait-max-s`, by default 3600 s).
  - **From a session's tool call:** run a short job in the foreground with `--wait-max-s 300`. Run a leg or a long suite in the background, with a `--log`. If the reaper kills a background call, run it again; never detach it.
  - **Exempt:** `scripts/gcc-check.sh --compile`, reads, `git merge` and `git commit`, and `git add` with explicit paths.
  - **A wrong block** goes to the lead, with the exact command.
- **The owner's machine: throttle while a game runs** (the user, 2026-10-06: "Throttle while I play"). Before every heavy run, check for one: `Get-Process r5apex_dx12 -ErrorAction SilentlyContinue` in PowerShell (Apex Legends), or any other full-screen game. While one runs:
  - no Docker legs, no GPU tests and no GPU bench. A full `scripts\test.ps1` includes the GPU tests, so run `ctest -L spade -LE gpu` instead;
  - builds run at `-ParallelLevel 4`;
  - CPU tests are fine, and so is `scripts/gcc-check.sh` in either mode (one translation unit, capped at 3 GB): only full `docker-leg.ps1` runs wait.

  Full speed resumes when the game closes. A bench window needs no game running, as well as both trees quiet.
- **Stop only processes you started, by PID or task id, never by name or pattern** (Kat's rule, adopted for Spade by the lead on 2026-10-12). The box is shared, so `Stop-Process -Name`, `pkill`, `taskkill /IM` or `docker stop $(docker ps -q)` can kill another session's build, suite or leg. Record the id when you start a background job or a leg, and stop that id: `Stop-Process -Id`, `taskkill /PID`, or `docker stop <container>` for a container you started. If you can't prove a process is yours, leave it and tell the lead.
- **Replaced by the queue on 2026-10-20 (above): the hand-run memory check.** It's kept here for its reasons. Before each heavy run (a build, a full or chunked suite, a leg), free memory and free commit both had to be at least 6 GB (the lead: free memory 2026-10-12, free commit 2026-10-13), checked by `Get-CimInstance Win32_OperatingSystem | ForEach-Object { 'free {0:F1} GB, free commit {1:F1} GB' -f ($_.FreePhysicalMemory/1MB), ($_.FreeVirtualMemory/1MB) }`, which still reads the box. Suites still run serially (no `ctest -j`), in chunks of under 10 minutes each.
  - Why free memory: Rendering's debug run was killed at 3.9 GB free with no game running, while Kat's builds and several Spade realms' `-j8` builds and suites ran at once.
  - Why free commit: Windows refuses an allocation once the commit charge reaches the commit limit, however much memory is free ("A test that dies without failing", below). On 2026-10-13 the box had 14.2 GB free but only 6.2 GB of commit left, at rest.
- **The page file is fixed at 32 GB** (Cameron's decision, 2026-10-13). He applies it himself, since it needs admin rights, and it takes effect at his next reboot. Until then Windows manages the page file (6.4 GB on 2026-10-13), so the commit limit is about 38 GB, only about 6 GB above the charge at rest, and the free-commit check will often say wait.
- **Only Kat writes `build-host/` and `install-host/`** (Kat's `spade-prefix.ps1`, run by Kat's Spade Host realm). Never configure, build or install into either one.
- **The old box** (about 7.6 GB, `-j1`, one build at a time) is history. Its rows below are marked as such.

| Symptom | Cause | Response |
|---|---|---|
| A build or test run dies with no error | The old box killed backgrounded jobs | Run in the foreground. Split long runs into legs that provably cover the whole suite (check with `ctest -N`) |
| `git` is not recognized in PowerShell | The session started before 2026-10-04 ~04:25 UTC. At the user's choice, `C:\Program Files\Git\cmd` has been on the user PATH (HKCU) since then, but a running process keeps the environment it started with | Restart the session, or prepend it for that process only: `$env:Path = "C:\Program Files\Git\cmd;" + $env:Path`. `docker-leg.ps1` needs git |
| `powershell.exe` launched from Git Bash refuses to run a `.ps1` | The execution policy | Pass `-ExecutionPolicy Bypass` on that call only; it changes no setting |
| `docker-leg.ps1` stops at the image build's first line | `2>&1` on the driver: Windows PowerShell turns docker's progress on stderr into a terminating error | Don't redirect the driver's stderr |
| Old box: Docker answers HTTP 500, or `docker desktop status` stays "starting" | Docker's WSL VM stopped under memory pressure. A cold start under pressure took about 11 min on 2026-10-03 | `docker desktop restart`, then wait at least 15 min without intervening. `docker desktop start` is a no-op while the app runs. Stopping, `wsl --shutdown` or force-quitting mid-start made it worse (`07-status.md`); escalate one step at a time, through the lead |
| A leg's watcher dies (a tool timeout, or the old box's memory pressure) | The watcher only streams the log | The detached container carries on; collect it with `docker-leg.ps1 -Follow` |
| A revert "does not fix" the thing it should | `Copy-Item` keeps the old timestamp, so ninja sees no work | `touch` the file after restoring it |
| Nondeterminism appears from nowhere | A full disk truncated a file | Check free space before debugging determinism |
| PowerShell logs look garbled | `*>` writes UTF-16 | Redirect through `Out-File -Encoding utf8`, or run from bash |
| An exit-code predicate is always false | PowerShell `if (cmd)` tests output, not the exit code | Use bash for exit-code predicates |

The first build of a fresh checkout fetches dependencies. On the old box, release took 1006 s at `-ParallelLevel 1`. On this machine, the Docker leg's first configure took 77 s with the fetch, and its first full build took 229 s at `-j4` (`07-status.md`).

## A test that dies without failing

A test that fails an assertion prints `[  FAILED  ]` and its message. If its output stops after `[ RUN      ]`, the process ended instead: a crash, an abort or a kill. On this box it has happened under load (the lead, 2026-10-13: `shower_inside` at 2.4 s and `VulkanSnapshot` at 0.55 s).

- **ctest does not say which.** ctest 4.2.0 here prints `***Failed` and `(Failed)` for every non-zero exit, a crash's included, and `LastTest.log` keeps no exit code (checked on 2026-10-13 with scripted exit codes). Windows' event log has no record either: GoogleTest turns off Windows Error Reporting for its process (`SetErrorMode(SEM_NOGPFAULTERRORBOX)`) and clears `abort()`'s report flag (`gtest.cc`, `UnitTest::Run`). The exit code shows only when the test runs again, directly.
- **`spade_tests` says why it died.** Before the process ends, `tests/test_fatal_report.cpp` writes a line starting `[spade_tests] fatal:` to stderr, which `LastTest.log` keeps. The line gives an escaped exception's type and `what()`, or, on Windows, an unhandled exception's code, address, thread and module, or an `abort()` from any thread. Look for that line first. The process still dies as it would have, with the same exit code.
- **Rerun it alone, directly, once,** in PowerShell from the build tree: `& .\bin\spade_tests.exe --gtest_filter=Suite.Name; '{0:X8}' -f $LASTEXITCODE`. Don't use Git Bash, which turns the code into a signal number: `0xC0000005` reads as 139 ("Segmentation fault"), and `0xC0000409` and `0xC000012D` as 127, which also means "command not found".
- **What the exit code says:**

| Exit code | What ended the test | Points to |
|---|---|---|
| `00000000` on the rerun | Nothing: it passed alone | Load. Read the box's state at the time (below) |
| `00000003` | `abort()` or `std::terminate`, silent under GoogleTest. An exception that escapes a `noexcept` function or a thread ends here, `std::bad_alloc` from a failed allocation included | Memory if the box was short of commit (below), otherwise a bug |
| `C0000005` | An access violation GoogleTest did not catch. It catches one in the test's own body and prints `SEH exception with code 0xc0000005`, so a silent one came from another thread, such as a driver's or a worker's | A bug, or a driver failing under memory pressure |
| `C0000017`, `C000012D` | An allocation refused (`STATUS_NO_MEMORY`), or a commit refused at the limit (`STATUS_COMMITMENT_LIMIT`) | Memory: the box's commit limit |
| `C00000FD`, `C0000374`, `C0000409` | A stack overflow, heap corruption, or a fail-fast (`/GS`, the CRT's invalid-parameter check) | A bug, not load |
| `FFFFFFFF`, or `00000001` with no `[  FAILED  ]` | A kill from outside: `Stop-Process` leaves −1 and `taskkill /F` leaves 1 (both checked 2026-10-13) | Someone stopped processes by name, against the box rule above |
| ctest's `***Timeout` | ctest's own kill at the test's time limit (60 s in debug) | Slow, not dead |

- **Short of memory means short of commit, not of free memory.** Windows refuses an allocation when the commit charge reaches the commit limit (RAM plus the page file), however much memory is free. At 03:55 UTC on 2026-10-13 the box had 14.2 GB free, but its commit charge was 32.2 GB of a 38.4 GB limit. Windows manages the page file and grows it on demand, and each time it does it logs "Your system is low on virtual memory" (System log, event 26), as it did at 17:26 UTC on 2026-10-12. An allocation that arrives as the limit is reached fails before the file grows.
- **Record at once, since the box changes within minutes:**
  - the test, its binary and preset, the commit, the time in UTC, how long it ran before it died (`LastTest.log`'s `Test time`), and its last line of output;
  - memory: `Get-CimInstance Win32_OperatingSystem | ForEach-Object { 'free {0:F1} GB; commit {1:F1} of {2:F1} GB' -f ($_.FreePhysicalMemory/1MB), (($_.TotalVirtualMemorySize - $_.FreeVirtualMemory)/1MB), ($_.TotalVirtualMemorySize/1MB) }`;
  - whether the page file grew around then: `Get-WinEvent -FilterHashtable @{LogName='System'; Id=26; StartTime=(Get-Date).AddHours(-1)} -ErrorAction SilentlyContinue | Where-Object Message -match 'virtual memory'`;
  - what else was running: builds (`cl`, `link`, `ninja`), suites (`ctest`, `spade_tests`), legs (`docker ps`), GPU users (`nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader`), and any game;
  - the rerun's exit code and result.

  Send it to the lead. Test/Docs keeps the list in `07-status.md`.
- **The rerun rule.** Rerun the test alone, once, and record both results: the run's and the rerun's. A pass alone makes it a flake, and the run's report names it with both results. A death or a failure on the rerun is a failure like any other. A test that dies silently a second time, on any run, goes to the lead as a finding instead of another rerun.

## Pushing

Nothing is pushed without the user's word. `.git/hooks/pre-push` refuses every push unless `SPADE_PUSH_AUTHORIZED` is set for that push. The tracked copy is `scripts/pre-push-guard.sh`; the hook is per-checkout, so install it in each clone (LF line endings).

## Committing in a shared tree

Several sessions share the main tree. Commit with explicit paths (`git commit -- <paths>`), and check `git diff --cached --stat HEAD` first. A stale index can record a peer's files as deleted under your commit message, and every local check still passes. To prove a commit's tree builds, build it in a fresh worktree at that commit.
