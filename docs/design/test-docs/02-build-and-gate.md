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
| **Covers** | gcc-13 and CMake 3.28.3, the project's floor; Release with Vulkan, the sandbox and GL compiled; `ctest -L spade -LE gpu`, with the viewer-trajectory guard at full length (`SPADE_FULL_VIEWER_TRAJECTORIES=1`; the gate checks a prefix); `tests/consumer` against a fresh install with `SPADE_VULKAN` ON (from the leg's own tree) and OFF (a library-only tree), plus the `SL2b` sandbox stage when the commit's `consumer-smoke.sh --help` offers `--sandbox`; the summary says whether it ran |
| **Leaves out** | v1 (`SPADE_BUILD_V1=OFF`: frozen, and never on a Linux leg); the `gpu` label, excluded rather than skipped (`TD-13`); Debug; Windows, which the gate covers; a window (no X11, so GLFW builds without a backend) |
| **Tests a commit** | `git -c core.autocrlf=false archive <commit>`, because a plain archive on this box emits CRLF. Uncommitted changes are not in the leg, and the driver says so when there are any. The image is built from that commit's own `scripts/docker-leg.Dockerfile` and tagged by the file's content hash |
| **State** | One Docker volume, `spade-docker-leg`: `/leg/src` (the commit, synced so unchanged files keep their mtimes and ninja stays incremental), `/leg/build` (dependencies in `_deps`, fetched once), `/leg/consumer`. `-Clean` drops it |
| **Runs** | Detached, in a container named `spade-docker-leg`, with no swap. The defaults are `--memory 3g` and `-j1`, the old box's; on this machine run it with `-Memory 6g -Jobs 4` ("This machine", below). Closing the terminal does not stop it. Running the driver again re-attaches rather than starting a second leg, and `-Stop` ends it. It holds the lead's Docker slot for the whole run; stay with it in the foreground |
| **Order** | configure, build (`ninja -k 0`, so one run lists every failing file), test, agreement, consumer ON, consumer OFF. `-Step consumer` also configures and builds first, so consumer ON installs the commit under test and never a tree some earlier commit built. A failed part blocks only the parts that need it. The exit code is 0 only if every part passed |
| **Output** | `build-docker\<short-sha>\`: `summary.txt` (each part's result and seconds, the commit, the toolchain, peak memory, free disk), `build-errors.txt` (repo-relative `file:line`), `ctest.log`, `ctest-junit.xml`, `tests.txt` (registered names), `leg.log` |
| **Refuses** | Under 4 GB free on the host drive (`-MinFreeGB`), since a full disk truncates files mid-build |

A red leg is reported per realm with the failing `file:line`, and each realm fixes its own code. Measured times and peak memory are in `07-status.md`.

## The gcc check before review

**Before "ready for review", every C++ file a branch adds or changes is compiled with gcc-13 in the leg's image, and the message says so.** This is the lead's standing practice from 2026-10-03, after master was gcc-red from `da2fcf5` to `7ed7572` over a warning MSVC never raised.

- **It is a syntax check** (`-fsyntax-only` with the engine's `-Wall -Wextra -Wpedantic -Werror`). It catches front-end warnings such as `dangling-else` and `missing-field-initializers`. It does not catch warnings that need the optimiser, such as `-Wmaybe-uninitialized`; only a leg run finds those.
- **Some files it cannot check honestly.** A file that depends on generated headers newer than the leg volume's (any change under `engine/shaders/`), or on a target's own compile definitions, is named in the message instead, and the lead schedules a leg run before merge.

From the worktree root, in Git Bash, with the files to check after the `_`:

```bash
MSYS_NO_PATHCONV=1 docker run --rm --memory 3g \
  -v spade-docker-leg:/leg:ro -v "$(pwd -W):/src:ro" \
  "$(docker images -q spade-docker-leg | head -n 1)" bash -c '
  mkdir -p /tmp/s && cp -r /src/engine /src/sandbox /src/tests /tmp/s && cd /tmp/s
  D=/leg/build/_deps; rc=0
  for f in "$@"; do
    tu=$f; case $f in *.hpp) printf "#include \"/tmp/s/%s\"\n" "$f" > /tmp/hdr.cpp; tu=/tmp/hdr.cpp ;; esac
    g++-13 -std=gnu++23 -fsyntax-only -Wall -Wextra -Wpedantic -Werror -ffp-contract=off \
      -DGLM_ENABLE_EXPERIMENTAL -DGLFW_INCLUDE_NONE -DYAML_CPP_STATIC_DEFINE \
      -DSPADE_ENGINE_DIR=\"/src/engine\" -DSPADE_GOLDEN_DIR=\"/src/tests/golden\" \
      -DSPADE_TESTS_DIR=\"/src/tests\" -DSPADE_TEST_OUTPUT_DIR=\"/tmp/test-output\" \
      -Iengine -Isandbox -Iengine/tools/viewer -I/leg/build/engine/generated/spade_slang \
      -I$D/glm-src -I$D/glad-src/include -I$D/glfw-src/include -I$D/imgui-src -I$D/imgui-src/backends \
      -I$D/yaml-cpp-src/include -I$D/nlohmann_json-src/include -I$D/volk-src -I$D/vulkan-headers-src/include \
      -isystem $D/googletest-src/googletest/include -isystem $D/benchmark-src/include \
      "$tu" && echo "gcc ok:   $f" || { echo "gcc FAIL: $f"; rc=1; }
  done; exit $rc' _ tests/test_example.cpp engine/render/example.hpp
```

- **What it does:**
  - mounts the worktree and the leg's volume, both read-only; the volume supplies the dependency headers and the generated headers from the last leg run;
  - copies the sources inside, because reading headers through the Windows mount is slow;
  - prints `gcc ok:` or `gcc FAIL:` per file, and exits non-zero if any file fails;
  - checks each header through a one-line translation unit that includes it.
- **Proved red, then green,** on 2026-10-03:
  - at `da2fcf5` it failed `tests/test_render_agreement_matrix.cpp` at :95, :97 and :100 (`missing-field-initializers`);
  - at `ec6e2da` the fixed file passed, and so did two headers.
- **Cost:** about a minute per heavy test file on the old box (126 s for three files).
- **When to run it:** it needs no slot. Run one check container at a time per tree, with up to four files in it (`docs/design/consumers.md`, "Shared build machine"). Never run it during a leg's build step: that step rewrites the volume's generated headers, which the check reads.

## This machine

**Since 2026-10-04 Spade builds on a new machine:** 32 GB, 16 threads, an RTX 3060 Ti, and Docker Desktop with 16 GB. Kat's tree shares it.
- **The build budget** is in `docs/design/consumers.md`, "Shared build machine": it says how many heavy jobs run at once, and how much memory must be free first. The lead grants every slot.
- **Spade's settings for a heavy job:** an MSVC build is `scripts\build.ps1 -ParallelLevel 4`, and the Docker leg is `docker-leg.ps1 -Memory 6g -Jobs 4`. Run builds in the foreground, in chunks of at most 10 minutes, resuming each.
- **Only Kat writes `build-host/` and `install-host/`** (Kat's `spade-prefix.ps1`, run by Kat's Spade Host realm). Never configure, build or install into either one.
- **The old box** (about 7.6 GB, `-j1`, one build at a time) is history. Its rows below are marked as such.

| Symptom | Cause | Response |
|---|---|---|
| A build or test run dies with no error | The old box killed backgrounded jobs | Run in the foreground. Split long runs into legs that provably cover the whole suite (check with `ctest -N`) |
| `git` is not recognized in PowerShell | Git is not on the PowerShell tool's PATH on this machine; Git Bash has it | Prepend `C:\Program Files\Git\cmd` to `$env:Path` in that command, for that process only: `$env:Path = "C:\Program Files\Git\cmd;" + $env:Path`. `docker-leg.ps1` needs git. The machine's PATH is left alone; changing it is the user's call |
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
