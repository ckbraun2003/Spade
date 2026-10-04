# Resuming Spade on a new machine

**Owner:** lead. **Written:** 2026-10-03, when the user moved every Spade session to a new machine. This page holds what a fresh clone needs and git does not show by itself. Private working notes (memory, `tasks/`) travel separately, in the migration archive described in §5.

## 1. State at migration

- `origin/master`: the last batch gate passed at `65c2295`, release and debug: 1072 total, 1070 passed, 2 skipped by design, 0 failed, 83 gpu. Later master commits are docs only.
- Module API: stages 1–3 of 6 merged (`062e1fa`, `dca7cfb`, `42b352a`). Stage 4 has no plan yet.
- The joint drone-builder spec with Kat is approved (2026-10-03). Physics' model functions (`1d3112d`) and Core's `transform_of` (`65c2295`) are merged.
- The Docker gcc leg (`TD-11`, `TD-12`) is built and green (`scripts/docker-leg.ps1`).

## 2. Open branches, in merge order

All are on origin. None is merged.

| Branch | Owner | State | Next |
|---|---|---|---|
| `core/design-frame` (B) | Core | Written, reviewed (Physics approved), not built | Slot: red then green |
| `core/snapshot-v3` (C, on B) | Core | Written, reviewed, not built | Same slot; `consumers.md` line at merge |
| `core/scene-schema` (D, on C) | Core | Written, reviewed (Interface, Test/Docs), not built | Same slot: generate the scene golden, add its provenance, then one full suite and one gcc leg at D's head |
| `physics/airframe-compile` | Physics | DBP-44 and the DBP-26 fix; red and green written; carries copies of B and C | gcc check on its 4 test files, then a slot; merges after B and C |
| `interface/scene-compose` | Interface | compose, compose_file, instantiate; written, reviewed, not compiled; carries copies of B–D | gcc check, then an MSVC slot; merges after D |
| `rendering/csg-cell-size` | Rendering | World-space CSG cell size and the ≥2-cell wall warning; red and green written | Slot: bench at 169/80/50 mm, send the lead the numbers and the cap, then red, green and full suite; the moved CSG hash stays pending until the gcc leg reproduces it |

Every branch that adds or changes a C++ file needs the gcc check before review (`../test-docs/02-build-and-gate.md`, "The gcc check before review").

## 3. Waiting on the user

- **RND-6** (proposed): raster heightfields are drawn only within the world bounds, and `RS4` holds there.
- **A1**: an analytic heightfield background, which amends `SR-17`.
- **B2**: ray-marched CSG in the raster, which replaces `RS3`.
- Every push needs the user's word.

## 4. Setting up the new machine

1. Clone `https://github.com/ckbraun2003/Spade` and install the pre-push guard (`scripts/pre-push-guard.sh` as `.git/hooks/pre-push`). It releases only on `SPADE_PUSH_AUTHORIZED=1`, set for a single command.
2. Build the main tree with `scripts\build.ps1` and test it with `scripts\test.ps1` on both presets. The first build fetches dependencies.
3. Make one worktree per realm: `git worktree add ../spade-wt/<realm> <branch>`. To configure a worktree without fetching dependencies again, pass `-DFETCHCONTENT_SOURCE_DIR_<NAME>=<main tree>/build-ninja/release/_deps/<name>-src` for each dependency. Never point `FETCHCONTENT_BASE_DIR` at the main tree.
4. Docker: install Docker Desktop. `scripts\docker-leg.ps1` builds the image from `scripts/docker-leg.Dockerfile` and creates its own volume on the first run. Nothing from the old machine's volume is needed.
5. On a machine with little memory, build at `-j1`, run one build at a time, and run gcc check containers one at a time. Keep each build call under 10 minutes and resume it, because the session harness stops background builds when memory runs low.

## 5. Local-only state

`tasks/` (lessons and per-realm todo files), the realms' `.superpowers` ledgers and the session memory notes are not in git by the user's choice. They travel in the migration archive the lead prepared at migration. Restore them like this:
- `tasks/` goes to the root of the main tree. The Interface worktree's own `tasks/` goes to that worktree.
- Core's `.superpowers/` ledger goes to the root of `spade-wt/core`.
- The memory notes go to the new machine's Claude project memory directory for the main tree.

## 6. How the sessions work

The lead and five realm sessions (Core, Physics, Rendering, Interface, Test/Docs) are described in `../02-realms.md`. Realms message the lead, the lead hands out build slots one at a time, and code merges only after the lead's review. Each realm's `07-status.md` and its todo file's resume block say where that realm stopped.
