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
| `scripts/gcc-check.sh` | The gcc check before review (below), from Git Bash | `<file>...`; `GCC_CHECK_VOLUME=<volume>` picks the volume |
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
| **Refuses** | Under 4 GB free on the host drive (`-MinFreeGB`), since a full disk truncates files mid-build |

A red leg is reported per realm with the failing `file:line`, and each realm fixes its own code. Measured times and peak memory are in `07-status.md`.

## The gcc check before review

**Before "ready for review", every C++ file a branch adds or changes is compiled with gcc-13 in the leg's image, and the message says so.** This is the lead's standing practice from 2026-10-03, after master was gcc-red from `da2fcf5` to `7ed7572` over a warning MSVC never raised.

- **It is a syntax check** (`-fsyntax-only` with the engine's `-Wall -Wextra -Wpedantic -Werror`). It catches front-end warnings such as `dangling-else` and `missing-field-initializers`. It does not catch warnings that need the optimiser, such as `-Wmaybe-uninitialized`; only a leg run finds those.
- **Some files it cannot check honestly.** A file that depends on generated headers newer than the leg volume's (any change under `engine/shaders/`), or on a target's own compile definitions, is named in the message instead, and the lead schedules a leg run before merge.

From the worktree root, in Git Bash:

```bash
scripts/gcc-check.sh tests/test_example.cpp engine/render/example.hpp
```

- **What it does:**
  - picks the newest leg volume (label `spade.leg`) that holds a configured build and that no running container uses, so it never reads a volume a leg is building, and prints that volume and its commit; `GCC_CHECK_VOLUME=<volume>` picks one instead;
  - mounts the worktree and that volume, both read-only; the volume supplies the dependency headers and the generated headers from its last leg run;
  - copies the sources inside, because reading headers through the Windows mount is slow;
  - prints `gcc ok:` or `gcc FAIL:` per file, and exits non-zero if any file fails;
  - checks each header through a one-line translation unit that includes it.
- **Its compile line was proved red, then green,** on 2026-10-03, when it was still a command pasted from this page:
  - at `da2fcf5` it failed `tests/test_render_agreement_matrix.cpp` at :95, :97 and :100 (`missing-field-initializers`);
  - at `ec6e2da` the fixed file passed, and so did two headers.
- **Cost:** about a minute per heavy test file on the old box (126 s for three files); on this machine, 11 s for one heavy test file and one header.
- **When to run it:** whenever it's needed, with as many files as you like, beside any number of legs. It refuses (exit 2) only when no leg volume is idle.

## This machine

**Since 2026-10-04 Spade builds on a new machine:** 32 GB, 16 threads and an RTX 3060 Ti. Kat's tree shares it.
- **No slots and no budget:** build and test as the work needs (`docs/design/consumers.md`, "Shared build machine"). The user lifted every memory-based limit on 2026-10-04.
- **Docker's Linux VM is capped at 10 GB** (Docker reports `MemTotal` 9.71 GB). The cap is `.wslconfig` (`memory=10GB`), which is the user's machine configuration, set on 2026-10-05 on the user's word; only the user changes it. Before the cap, `vmmem` held 8.6 GB; just after it, 2.2 GB.
- **Docker legs under the cap:** run at most one fresh leg at a time. If two legs must overlap, run both with `-Memory 4g -Jobs 4`.
  - Why: a fresh (`-NoSeed`) leg at the defaults peaked at 4.29–4.50 GB, and a seeded leg with nothing to rebuild at about 0.9 GB.
  - Two fresh legs plus the VM's own overhead come near the cap, and the VM's out-of-memory killer acts before either leg's own `--memory` limit would.
- **Settings:** the scripts' defaults suit this machine (`scripts\build.ps1 -ParallelLevel 8`; the Docker leg `-Memory 8g -Jobs 8`, within the rule above). Run builds in the foreground, in chunks of at most 10 minutes, resuming each.
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

## Pushing

Nothing is pushed without the user's word. `.git/hooks/pre-push` refuses every push unless `SPADE_PUSH_AUTHORIZED` is set for that push. The tracked copy is `scripts/pre-push-guard.sh`; the hook is per-checkout, so install it in each clone (LF line endings).

## Committing in a shared tree

Several sessions share the main tree. Commit with explicit paths (`git commit -- <paths>`), and check `git diff --cached --stat HEAD` first. A stale index can record a peer's files as deleted under your commit message, and every local check still passes. To prove a commit's tree builds, build it in a fresh worktree at that commit.
