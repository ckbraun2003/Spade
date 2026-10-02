# Spade restructure — implementation plan

> **For agentic workers:** realm sessions do their own tasks; the lead reviews and merges. Steps use checkbox (`- [ ]`) syntax.

**Goal:** give Spade its own design library — a top-level charter, engine model and realm map, plus one library per realm — and finish separating it from KAT.

**Architecture:** the lead writes the shared top-level documents and moves the old ones into `superseded/`. Then five realms build their libraries in parallel, docs only. Then cleanup and a measured baseline. Two Core defects are fixed alongside.

**Spec:** `docs/design/plans/2026-10-01-spade-restructure-design.md`

## Global constraints

- **Keep it short.** A realm library is a working tool, not an essay. One fact lives in one place, decision status lives only in `00-decisions.md`, and current state lives only in `07-status.md`. Corrections are a one-line dated note; the story belongs in git history.
- **Cite by document and section, never by line number.**
- **Never edit the contents of `superseded/`.** Moving files into it is fine.
- **Work tree and branches.** Docs-only work happens in the main tree, on `master`, with path-scoped commits. Code work happens in a per-realm worktree (`git worktree add ../spade-wt/<realm> -b <realm>/<topic>`, with its own `-BuildDir`). The lead reviews and merges. Nothing is pushed.
- **Builds** run in the foreground, one at a time, in slots the lead hands out.

## Review focus

1. **A legacy ID with no home, or with two homes.** The lead and Test/Docs cross-check every `D`/`A`/`P`/`SA`/`RS`/`SL`/`SR` ID in `superseded/2026-09-consolidation/` against the five `00-decisions.md` (Task R3).
2. **A link into a moved file.** Moving the old docs breaks relative links that point at them. Fix the live ones; leave `superseded/` alone.
3. **`test_transfer_register.cpp` reads `docs/v1-transfer-register.md` by path.** That file does **not** move in this plan.
4. **Status prose duplicated across a README and `07-status`.** The README points to `07-status`; it never restates it.
5. **KAT rule series outside `consumers.md`.** Checked in review, not by a test.

---

### Task R1: Top-level documents and the move (Lead)

**Files:**
- Create in `docs/design/`: `README.md` (replacing the old one, which moves), `INDEX.md`, `00-charter.md`, `01-engine-model.md`, `02-realms.md`, `consumers.md`, `backlog.md`.
- Move (`git mv`): `docs/design/superseded/*` → `docs/design/superseded/kat-originals/`; `01-charter.md 02-engine.md 03-world-and-render.md 04-objects.md 06-sandbox-and-v1.md 07-status.md 08-lessons.md README.md RECONCILIATION.md pivot-audit.md plan-c-sandbox.md interleaving-shuffle.md sensor-arena-dedup.md` → `docs/design/superseded/2026-09-consolidation/`.
- Create: `docs/design/{core,physics,rendering,interface,test-docs}/` (each realm creates its own files in R2).

- [ ] **Step 1:** Do the moves in one commit: `docs(design): the KAT-era consolidation moves to superseded/, unedited`.
- [ ] **Step 2:** Write `00-charter.md`, `01-engine-model.md` and `02-realms.md` from spec §1–§3, verbatim in substance.
- [ ] **Step 3:** Write `consumers.md`:
  - what KAT consumes: the installed `spade::` package, the world-file format and the snapshot format;
  - the C5 adapter lives in KAT (`dronesim/spade/`);
  - the list of KAT series that legacy text cites (C5, TS, HS, F, B, G, CP, TR, DM, CN, R, …) with one line each saying that KAT owns it;
  - the 30 agreement cases on KAT worlds, which move to KAT.
- [ ] **Step 4:** Write `backlog.md`, one row per item: what isn't built, why, what proves it done. Seed it from spec §5: the separation items, the paused work (SPH, wiring the GPU rasterizer, sandbox C3–C8 as the editor), the Core redesign (module API, scheduler, regions), the engine-side translation lock (from the drone design), Spade's own content for the agreement bands, and the owed consumer-smoke leg.
- [ ] **Step 5:** Write `README.md` (register) and `INDEX.md`.
  - **README:** the realm table (each realm, its directory, the session that owns it), the series table (one line each: `L1–L8` charter; `D1–D12`, `A1–A11` engine; `P1–P8`, `SA1–SA3` pillars; `RS1–RS15`, `SR-*` render; `SL1–SL18` library/sandbox; `CORE-`, `PHY-`, `RND-`, `INT-`, `TD-` going forward; the home of each filled in after R3), how to cite (`engine D7`, `RND-3`), and how the user signs (a row in a realm's `00-decisions.md` gets `signed <date>`).
  - **INDEX:** about ten lines of "to do X, read Y".
- [ ] **Step 6:** Commit: `docs(design): Spade's own top level — charter, engine model, realms, register`.

### Task R2: Each realm builds its library (all five realms, in parallel, docs only)

**Files:** `docs/design/<realm>/README.md`, `00-decisions.md`, `01-…-<subject>.md` (as many as the realm needs, usually 2–5), `07-status.md`, `plans/` (may be empty), and optionally `08-lessons.md`.

**Sources to harvest** (all in `superseded/2026-09-consolidation/` unless noted):

| Realm | Harvest from |
|---|---|
| Core | `02-engine.md` (step model, the schedule, state, many-worlds, snapshot, compute backend seam), `04-objects.md`, the world-file part of `03-world-and-render.md`, `sensor-arena-dedup.md`, `interleaving-shuffle.md` |
| Physics | `02-engine.md` (physics decisions `D1`–`D6`, parity, the effects/aero roadmap), the physics rows of `07-status.md` and `08-lessons.md` |
| Rendering | `03-world-and-render.md` (`RS*` and `SR-*`), the render rows of `07-status.md` and `08-lessons.md` |
| Interface | `06-sandbox-and-v1.md` (`SL7`–`SL18`), `plan-c-sandbox.md` (stays in `superseded/`; `interface/plans/README.md` points to it as superseded by the editor direction — 2026-10-02), the charter's `SL2`/`SL2a`/`SL2b` |
| Test/Docs | `07-status.md` (traceability, counts, the standing rules of §4), `08-lessons.md` (process and box lessons), `CONTRIBUTING.md`'s rules, `docs/v1-transfer-register.md` (stays where it is; referenced) |

- [ ] **Step 1: `00-decisions.md`.** A table: `ID | ruling (one line) | signed | status | source`. One row for every legacy ID your realm is the home of. Status is `live`, `superseded by X`, or `repealed <date>`. A ruling the new charter changes, such as "state arrays frozen" or "fixed ten-pass schedule", is marked `superseded by L3` or `superseded by engine model`, not deleted. Then any new `<PREFIX>-n` rulings.
- [ ] **Step 2: Specs.** Rewrite the harvested content by subject, in the engine model's terms (modules, fields, responders, techniques, channels, grades). Write the target design and say plainly where the code is today. Keep each spec short.
- [ ] **Step 3: `07-status.md`.** Specified vs built, checked against the tree at a named commit. Open items go in two lists: "needs a user decision" and "debt". Include every defect and drift item the realm reported on 2026-10-01.
- [ ] **Step 4: Sweep KAT-citing comments in your own code**, comment-only, in your worktree branch: reword them to Spade terms or delete them. The lead reviews this with your code work.
- [ ] **Step 5: `README.md`.** Read order, "if you're here to do X", series owned vs quoted, and how to cite. Point to `07-status`; don't restate it.
- [ ] **Step 6:** Commit your realm directory only: `docs(<realm>): the <realm> library`. Then tell the lead.

### Task R3: Cross-check (Test/Docs with the lead)

- [ ] List every legacy ID token in `superseded/2026-09-consolidation/` (`grep -ohE "\b(D|A|P|SA|RS|SL)[0-9]+[a-z]?\b|\bSR-[0-9]+[a-z]?\b"`, then de-duplicate) and check that each has exactly one home row across the five `00-decisions.md`. A split ruling such as `SL6` is listed in both realms, each covering its own half. Report gaps to the owning realm.
- [ ] Fill in the "home" column of `docs/design/README.md`'s series table.
- [ ] Commit: `docs(design): every legacy ruling has a home`.

### Task R4: KAT separation cleanup (Test/Docs; Rendering for item 1; Lead for item 2)

- [ ] **1. (Rendering, worktree)** Remove the `../KAT` reach from Spade's tests:
  - drop the `SPADE_KAT_CONTENT_DIR`/`SPADE_KAT_SCENES_DIR` cache variables (`tests/CMakeLists.txt`) and the KAT-world `AgreementMatrix` cases and scene-drift guard (`tests/test_render_agreement.cpp`);
  - keep the agreement machinery and every non-KAT case;
  - add a backlog row: rebuild the bands on Spade's own content.

  Build, run the full suite, and confirm that only the removed cases are gone.
- [ ] **2. (Lead)** `git remote remove katsrc` (this also drops `remotes/katsrc/spade-extract`).
- [ ] **3. (Test/Docs, worktree)** Remove or reword KAT references in `scripts/build.ps1`, `scripts/test.ps1`, `CMakePresets.json` (preset descriptions), `.gitignore` (the KAT-era entries), `tests/bench/baselines.json` (`_meta` path and citation strings only; never a number) and `scripts/pre-push-guard.sh` comments. Also re-install the hook from the tracked copy with LF line endings. Decide whether the `T0` label stays; if it does, define it locally rather than by reference to KAT.
- [ ] **4. (Test/Docs)** Rewrite `README.md` (a general-purpose quickstart; correct test counts only after R5; drop the drone-first framing), `CHANGELOG.md` (fix the "inside the kat monorepo" line; add entries for the split, GNSS, the GPU rasterizer stages and this restructure) and `CONTRIBUTING.md` (point to `AGENTS.md` and the library; drop the stale "frozen at 18" and its `kSnapshotVersion` claim).
- [ ] Commit each item separately.

### Task R5: Measured baseline (Test/Docs) — the first build slot

- [ ] Configure and build both presets in the main tree (foreground; this is the first build on this checkout, so it fetches dependencies).
- [ ] `scripts\test.ps1 -Preset release` and `-Preset debug`. Record in `test-docs/07-status.md`: commit, build tree, totals, passed, skipped (GPU), failed, with any failures named. These are the numbers the README then quotes.

### Task R6: Defects 2 and 3 (Core, worktree)

- [ ] **Defect 2.** In `engine/compute/vulkan/step_recorder.cpp`, derive the dispatch total from what is actually emitted, not from a hand tally. Either count in a first pass, or emit every barrier and drop the trailing one. Update the tally table in `step_recorder.hpp`. Add a test that pins the recorded dispatch count against the emitted count for 1-, 2- and 5-substep shapes (`Gpu*`-prefixed if it needs a device; host-only if the count can be computed without one). Run the GPU parity suite on the dev GPU.
- [ ] **Defect 3.** Install the `compute/` headers that `sim/simulation.hpp` includes (`backend.hpp`, `grid_entry.hpp`, `step_params.hpp`) whatever `SPADE_VULKAN` is set to, or stop including them from the public header. Verify with a `-DSPADE_VULKAN=OFF` configure plus install plus the `tests/consumer/` project built against that prefix.
- [ ] Commit each separately; the lead reviews and merges.

## Order

1. **R5's release build first.** It creates the build tree everyone measures against.
2. **R1 (lead)** runs alongside R5.
3. **R2 starts once R1 has landed.**
4. **Drone plan Tasks 1–4** and **R6** use the build slots that follow, in the order they report ready.
5. **R3 and R4** run after R2.
