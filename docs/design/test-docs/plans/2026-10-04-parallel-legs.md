# Parallel Docker legs: plan and record

**Owner:** Test/Docs. **Status:** built and proved; ready for review. Assigned by the lead on 2026-10-04, after the user lifted every build and test limit. Branch: `test-docs/parallel-legs`, in `../spade-wt/test-docs`.

## Goal

Several Docker legs, and any number of gcc checks, run at once. Before this, the driver had one fixed container name and one volume, `spade-docker-leg`. A second leg re-attached to the first instead of starting. A gcc check could not run during a leg's build step, because the leg rewrote the volume the check reads.

## Design

- **One container and one volume per run,** `spade-docker-leg-<run>`. The run is the commit's short sha, so the steps of one commit share a volume. `-Run <id>` names a run instead.
- **Seeding.** A new volume starts as a copy of the newest leg volume no running container uses. The copy is the build tree, source, consumer tree and commit, with every mtime kept, so the volume fetches nothing and rebuilds only what the commit changed. `-NoSeed` starts it empty.
  - `/leg/seed` records how the volume began, and `/leg/previous` what it held before the last sync.
  - The driver appends both to `summary.txt` as the volume line.
- **The clock guard.** After the sync, if the newest output in the build tree is not older than the clock, every file the sync wrote is stamped one second past that output. A seed built under a later clock, or a WSL clock that stepped back, would otherwise leave an object newer than its changed source. Ninja would then keep the stale object.
- **The driver's own `docker-leg.sh` seeds and syncs.** Those steps arrange the volume, and an older commit's copy predates them. The commit's copy still runs the step under test.
- **`scripts/gcc-check.sh`.** The gcc check's command, as a script. It mounts the newest idle leg volume read-only and prints which volume and commit it used.
- **`-Prune [-Keep N]`** removes idle leg volumes beyond the newest N (default 4), never one any container uses. `-Clean` drops the run's own volume.
- **`TD-12`:** a leg that makes a regenerated golden final runs with `-NoSeed`.
- **Defaults for this machine:** the leg runs at `-Memory 8g -Jobs 8`, and `scripts\build.ps1` at `-ParallelLevel 8`.

## Evidence (2026-10-04, image `spade-docker-leg:ed5552f460d7`)

| What | Red | Green |
|---|---|---|
| **Two legs at once** | Master's driver, started on `c73a8d5` while another realm's leg ran at `97efb4d`: "a leg is already running; following it instead of starting another (-Commit and -Step are ignored)" | Leg X on `626e878` (`-NoSeed`) and leg Y on `d88c188` (seeded) were both running at 04:42:10. Both passed: 989 run, 987 passed, 2 skipped, 0 failed. X took 345 s with a 157 s fresh build. Y took 145 s; it was seeded from `9a98693`'s volume, not X's busy one |
| **gcc check during a leg's build** | The old rule forbade it | During X's build step (its `build.log` grew from 102 to 188 lines meanwhile), `gcc-check.sh` chose `spade-docker-leg-9a986939dc97`, skipped both busy volumes, and passed in 14 s |
| **Staleness under clock skew** | With the guard removed, a seed built "an hour ahead" (outputs and `.ninja_log` shifted together) left a stale object: the rebuilt binary returned 1, not 2 | With the guard, the sync stamped the changed source past the newest output. Ninja rebuilt it alone, and the binary returned 2 |
| **Seeding is incremental** | | A seeded CMake tree rebuilt only the changed TU and the link. On Spade, `9929c4c` seeded from `97efb4d`'s volume rebuilt in 82 s; `c73a8d5` seeded from `9929c4c`'s built in 0 s |
| **Older commits** | | `9929c4c` and `c73a8d5` carry the old `docker-leg.sh`. The driver's copy seeded and synced them, and both passed |
| **Fresh volume (`TD-12`)** | | `9a98693` with `-NoSeed`: configure 72 s (fetch included), build 109 s, all parts PASS. The digests, render goldens and 8 viewer trajectories reproduce. All five agreement cases are bit-identical to the three pins, as in every run here |
| **Prune, clean, follow, stop** | | `-Prune -Keep 2` kept the newest two idle volumes and removed the third. It kept three volumes that a created or running container held. `-Clean` removed only its run's volume. `-Follow` with two legs asked for `-Run` and exited 2; `-Stop` acted on the named leg, then on the only one |

The step logic also has local harnesses, kept out of the repository (the lead's 2026-10-03 decision):
- the Git Bash stand-in harness: 59 checks, all green, up from 58;
- an in-image seed and sync harness: 29 checks, all green. It goes red on both clock-guard checks when the guard is removed.
