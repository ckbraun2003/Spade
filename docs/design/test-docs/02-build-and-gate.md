# Test/Docs — build and gate

**Owner:** Test/Docs. **Normative** for the gate; the rest describes the tooling as it is.

## Scripts

All three are PowerShell, run in the foreground, and keep to ASCII (non-ASCII breaks `.ps1` on Windows PowerShell 5.1).

| Script | What it does | Parameters |
|---|---|---|
| `scripts\build.ps1` | Imports the MSVC environment, configures the preset if needed, builds | `-Preset debug\|release` (default release), `-Target <one>`, `-BuildDir <configured tree>`, `-ParallelLevel 1..64` (default 1), `-Clean` |
| `scripts\test.ps1` | `ctest -L spade --output-on-failure --no-tests=error` against a built preset | `-Preset`, `-Filter <regex>` |
| `scripts\demo.ps1` | Builds if needed and launches one viewer scene | `-Scene <name>`, `-Preset`, `-Backend cpu\|vulkan` |

## Presets and options

- `CMakePresets.json`: `msvc-ninja-debug` and `msvc-ninja-release`, building into `build-ninja/debug` and `build-ninja/release`. CMake 3.28 or newer.
- Options: `SPADE_VULKAN`, `SPADE_RENDER_GL`, `SPADE_BUILD_SANDBOX` and `SPADE_BUILD_V1`, all default `ON`. A `-DSPADE_VULKAN=OFF` configure has no preset of its own.
- A worktree builds in its own directory; two sessions never share a build tree.

## The gate

`TD-7`: green means `scripts\test.ps1` green on **both** presets, with the counts reported per `TD-8`. There is no hosted CI. The gate therefore runs on one machine, one toolchain (MSVC) and one GPU, and it says so when it reports. The second toolchain and the consumer smoke are owed (`07-status.md`).

`--no-tests=error` fires only when **zero** tests are registered. A battery that always skips stays invisible to it, which is why skips are reported by name.

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
