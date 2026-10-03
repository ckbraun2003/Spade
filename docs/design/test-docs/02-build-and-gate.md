# Test/Docs — build and gate

**Owner:** Test/Docs. **Normative** for the gate; the rest describes the tooling as it is.

## Scripts

Run them in the foreground. The `.ps1` scripts keep to ASCII (non-ASCII breaks `.ps1` on Windows PowerShell 5.1); the `.sh` scripts run inside the Docker leg's container and check out LF (`.gitattributes`).

| Script | What it does | Parameters |
|---|---|---|
| `scripts\build.ps1` | Imports the MSVC environment, configures the preset if needed, builds | `-Preset debug\|release` (default release), `-Target <one>`, `-BuildDir <configured tree>`, `-ParallelLevel 1..64` (default 1), `-Clean` |
| `scripts\test.ps1` | `ctest -L spade --output-on-failure --no-tests=error` against a built preset | `-Preset`, `-Filter <regex>` |
| `scripts\demo.ps1` | Builds if needed and launches one viewer scene | `-Scene <name>`, `-Preset`, `-Backend cpu\|vulkan` |
| `scripts\docker-leg.ps1` | The Docker leg (below): builds and tests one commit on Linux/gcc in a container, then the consumer smoke | `-Commit <rev>` (default `HEAD`), `-Step all\|image\|sync\|configure\|build\|test\|consumer`, `-Memory` (default `3g`), `-Jobs` (default 1), `-Follow`, `-Stop`, `-Clean` |
| `scripts/docker-leg.sh` | The leg's steps inside the container; the driver runs the copy from the commit under test | `<step> [--jobs N]` |
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
| **Covers** | gcc-13 and CMake 3.28.3, the project's floor; Release with Vulkan, the sandbox and GL compiled; `ctest -L spade -LE gpu`; `tests/consumer` against a fresh install with `SPADE_VULKAN` ON (from the leg's own tree) and OFF (a library-only tree) |
| **Leaves out** | v1 (`SPADE_BUILD_V1=OFF`: frozen, and never on a Linux leg); the `gpu` label, excluded rather than skipped (`TD-13`); Debug; Windows, which the gate covers; a window (no X11, so GLFW builds without a backend) |
| **Tests a commit** | `git -c core.autocrlf=false archive <commit>`, because a plain archive on this box emits CRLF. Uncommitted changes are not in the leg, and the driver says so when there are any. The image is built from that commit's own `scripts/docker-leg.Dockerfile` and tagged by the file's content hash |
| **State** | One Docker volume, `spade-docker-leg`: `/leg/src` (the commit, synced so unchanged files keep their mtimes and ninja stays incremental), `/leg/build` (dependencies in `_deps`, fetched once), `/leg/consumer`. `-Clean` drops it |
| **Runs** | Detached, in a container named `spade-docker-leg`, with `--memory 3g` and no swap, at `-j1`. Closing the terminal does not stop it. Running the driver again re-attaches rather than starting a second leg, and `-Stop` ends it. It holds an exclusive build slot for the whole run; stay with it in the foreground |
| **Order** | configure, build (`ninja -k 0`, so one run lists every failing file), test, consumer ON, consumer OFF. A failed part blocks only the parts that need it. The exit code is 0 only if every part passed |
| **Output** | `build-docker\<short-sha>\`: `summary.txt` (each part's result and seconds, the commit, the toolchain, peak memory, free disk), `build-errors.txt` (repo-relative `file:line`), `ctest.log`, `ctest-junit.xml`, `tests.txt` (registered names), `leg.log` |
| **Refuses** | Under 4 GB free on the host drive (`-MinFreeGB`), since a full disk truncates files mid-build |

A red leg is reported per realm with the failing `file:line`, and each realm fixes its own code. Measured times and peak memory are in `07-status.md`.

## This machine

| Symptom | Cause | Response |
|---|---|---|
| A build or test run dies with no error | Backgrounded jobs get killed here | Run in the foreground. Split long runs into legs that provably cover the whole suite (check with `ctest -N`) |
| A build is killed or crawls | Memory-bound: about 7.6 GB, shared by several sessions | `-ParallelLevel 1` (the default). Builds are scheduled one at a time by the lead |
| A revert "does not fix" the thing it should | `Copy-Item` keeps the old timestamp, so ninja sees no work | `touch` the file after restoring it |
| Nondeterminism appears from nowhere | A full disk truncated a file | Check free space before debugging determinism |
| PowerShell logs look garbled | `*>` writes UTF-16 | Redirect through `Out-File -Encoding utf8`, or run from bash |
| An exit-code predicate is always false | PowerShell `if (cmd)` tests output, not the exit code | Use bash for exit-code predicates |

The first build of a fresh checkout fetches dependencies and took 1006 s for release at `-ParallelLevel 1` (`07-status.md`).

## Pushing

Nothing is pushed without the user's word. `.git/hooks/pre-push` refuses every push unless `SPADE_PUSH_AUTHORIZED` is set for that push. The tracked copy is `scripts/pre-push-guard.sh`; the hook is per-checkout, so install it in each clone (LF line endings).

## Committing in a shared tree

Several sessions share the main tree. Commit with explicit paths (`git commit -- <paths>`), and check `git diff --cached --stat HEAD` first. A stale index can record a peer's files as deleted under your commit message, and every local check still passes. To prove a commit's tree builds, build it in a fresh worktree at that commit.
