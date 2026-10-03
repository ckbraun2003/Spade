# The Docker leg (`TD-11`, `TD-12`): plan

**Owner:** Test/Docs, with Interface for the consumer half. **Status:** draft, for the lead's review. This is item 1 of "What's next" (`../07-status.md`) and first in the backlog's order. Branch: `test-docs/docker-leg`, in `../spade-wt/test-docs`.

## Goal

One command on the development box builds a Linux/gcc image, builds a commit of Spade in it, runs `ctest -L spade -LE gpu` on release, and builds and runs `tests/consumer` against an installed prefix with `SPADE_VULKAN` ON and OFF. It reports every part's result with the commit, toolchain and timings (`TD-8`). It is the second toolchain for goldens (`TD-12`) and the consumer smoke's first home.

## What the leg is

| | |
|---|---|
| **Image** | `ubuntu:24.04` pinned by digest; `gcc-13` (13.3.0) as CC/CXX; `cmake` 3.28.3 from apt, which is exactly the project's floor; `ninja-build`, `git`, `python3` (`cmake/SpadeSlang.cmake:139`), `ca-certificates`, `rsync`. Tagged by the Dockerfile's content hash, so a changed Dockerfile builds a new image |
| **Configure** | Ninja, `Release`, every option at its default except `SPADE_BUILD_V1=OFF`: Vulkan, the sandbox and GL are compiled. No X11, so GLFW builds without a window backend, as `vendor/CMakeLists.txt` intends. This matches the KAT-era Spade leg (its 2026-09-18 cache: gcc 13.3.0, cmake 3.28.3, `SPADE_VULKAN=ON`, `SPADE_BUILD_SANDBOX=ON`, `SPADE_BUILD_V1=OFF`). v1 is frozen and was never on the Linux leg |
| **Build** | Every target, `-j1`, with `ninja -k 0`, so one run lists **every** translation unit gcc rejects, not just the first |
| **Test** | `ctest -L spade -LE gpu --output-on-failure --no-tests=error`. It reports the total, passed, skipped (by name), failed, and how many `gpu` tests were excluded (`TD-13`). It writes the registered test names, so a report can say which tests exist on one platform and not the other at the same commit. The suite's golden tests are `TD-12`'s cross-check |
| **Consumer** | Interface's `scripts/consumer-smoke.sh` (branch `interface/consumer-smoke`, `e63478a`), called twice: `--vulkan ON --from-build /leg/build` and `--vulkan OFF`, both with `--work /leg/consumer --deps /leg/build/_deps --jobs 1`. Exit 0 pass, 1 a stage failed, 2 usage. A missing script is a failure, not a skip (`TD-5`) |
| **Order** | configure, build, test, consumer ON, consumer OFF. A red part does not stop the parts that don't depend on it, so consumer OFF still runs after a failed build. The leg exits 0 only if every part passed |

## How it runs

- **`scripts\docker-leg.ps1`** (host, PowerShell, ASCII): `-Commit <rev>` (default `HEAD`), `-Step all|image|sync|configure|build|test|consumer`, `-Memory 3g`, `-Jobs 1`, `-Follow`, `-Stop`, `-Clean`.
- **It tests a commit, not a working tree.** `git -c core.autocrlf=false archive <commit>` gives the repository's LF bytes, which is what a Linux checkout sees. Measured: on this box a plain `git archive` emits CRLF (139 of 139 lines of `CMakeLists.txt`). The driver says so when the tree has uncommitted changes, since they are not in the leg. The image is built from that commit's own Dockerfile.
- **State lives in one named volume, `spade-docker-leg`,** at `/leg`: `src/`, `build/` (deps in `build/_deps`), and `consumer/`. Sync extracts the archive to a staging directory and runs `rsync -r --checksum --delete` into `src/` **without preserving times**, so unchanged files keep their mtimes and ninja rebuilds only what changed. `-Clean` drops the volume.
- **One leg at a time.** The container has a fixed name, `spade-docker-leg`, `--memory 3g --memory-swap 3g` (no swap, so an OOM kill fails loudly), and runs **detached**. The driver streams its log and waits for its exit code. A closed terminal or a tool timeout doesn't kill the leg; re-running the driver, or `-Follow`, re-attaches to it rather than starting a second one. `-Stop` ends it.
- **Output** goes to `build-docker/<short-sha>/` (ignored by `build-*/`): `leg.log`, `ctest.log`, `ctest-junit.xml`, `summary.txt`.
- **`scripts/docker-leg.sh`** (in the container, from the commit under test) runs the steps and writes the summary: each part's exit code and seconds, gcc and cmake versions, the commit, and the container's memory limit.
- **Memory and disk.** `.wslconfig` is untouched: WSL's default cap of 50% of RAM gives the VM 3.9 GB, and the container takes 3 GB of it. C: has 14 GB free (95% full). The image is about 0.6 GB and the volume an estimated 4–6 GB. The driver prints free space on C: and refuses to start below 4 GB, since a full disk truncates files (`08-lessons.md`). The stale `spade-linux-build` volume (1.8 GB, the KAT-era Spade leg's tree from 2026-09-18) could go, but only on the user's word. The `kat-ci-linux` images are KAT's, so they stay.

## Review focus

1. **An OOM kill or a full disk mid-build** must show as a named failure (`Killed signal terminated program cc1plus`, `No space left on device`) in the summary, not as a hang or a silent truncation. Task 3 checks how the summary handles a non-zero ninja exit; Task 2 checks the disk refusal.
2. **An interrupted run** (Ctrl+C, a closed terminal, a 10-minute tool timeout) must leave one leg running that can be re-attached, never two on one volume. Task 2.
3. **A dirty tree** must not be mistaken for the commit: the summary names the commit, and the driver prints the uncommitted count. Task 2.
4. **CRLF**: no file in `/leg/src` may contain CR unless the repository's blob does. Task 2 checks a sample against the blobs.
5. **A commit without `scripts/consumer-smoke.sh`** (every commit before Interface's merges) must fail the consumer part, not skip it. Task 3, run on my branch alone.

## Tasks

- [ ] **1. Image.** `scripts/docker-leg.Dockerfile` and the driver's `image` step. Proof: the image builds; `gcc-13 --version`, `cmake --version` and `python3 --version` print in a throwaway container; the image size is recorded. *Short slot, about 10 min.*
- [ ] **2. Driver.** Sync, volume, fixed-name detached container, follow/stop, output directory, dirty-tree notice and disk refusal. Proof:
  - sync the same commit twice, and a file's mtime is unchanged;
  - sync a commit that changes one file, and only that file's mtime moves;
  - zero files with CR in `/leg/src` where the blob has none;
  - a second driver started during a run follows the first;
  - the disk threshold refuses when set above the free space (a control).
  *Same short slot as Task 1.*
- [ ] **3. In-container steps** (`scripts/docker-leg.sh`): configure, build (`-k 0`), test, consumer, summary and exit code. Proof: `bash -n`; on my branch alone, `-Step consumer` fails with "consumer-smoke.sh is not in <sha>" (the `TD-5` control).
- [ ] **4. First full run** on a local integration commit (my branch plus `interface/consumer-smoke`), never pushed. Report the gcc, `-Werror` and portability failures per realm with `file:line`, and don't fix them. Send Interface the consumer log. A red suite is a result, not a blocker. *Exclusive slot: an estimated 60–75 min the first time (image 5, fetch and configure 5, build 35–45 at `-j1`, tests 5, consumer 10–15), then 10–20 min for an incremental run.* Those figures are guesses until this run measures them.
- [ ] **5. Docs.**
  - `02-build-and-gate.md`: the script table and a "Docker leg" section saying what it covers and what it leaves out: v1, `gpu`, Debug, Windows.
  - `07-status.md`: the `TD-11` row and the first run's results with provenance.
  - `01-verification.md` and the `07-status.md` debt line: "No gate runs it yet" becomes the leg (the lead's nit).
  - The backlog row is the lead's to close.
  - Then "ready for review: test-docs/docker-leg".

Merge goes ahead once the run goes end to end, even if the suite is red, so every realm can reproduce its own failures with `scripts\docker-leg.ps1`.
